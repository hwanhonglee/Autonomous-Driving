#!/usr/bin/env python3
"""Production-equivalent, all-snapshot preflight for the Town06 60 km/h v5.

The frozen v4 bag contains both the selected raw VAD trajectory and the final
trajectory published by ``vad_route_manager``.  This tool reconstructs the
route-manager geometry pipeline from every active raw snapshot, first at the
recorded strength (10) and then at the proposed strength (10000).  The
strength-10 result is checked against the recorded final trajectory before any
candidate conclusion is allowed.

This is an offline CARLA evidence screen.  It never starts or publishes to a
ROS graph and it never establishes real-vehicle readiness.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
import copy
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace
from typing import Any, Iterable, Mapping, Sequence

# Match the route-manager runtime contract and avoid allowing BLAS to turn this
# offline timing screen into a many-core benchmark.
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["OMP_NUM_THREADS"] = "1"
os.environ["MKL_NUM_THREADS"] = "1"
os.environ["NUMEXPR_NUM_THREADS"] = "1"

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import rosbag2_py  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from rosidl_runtime_py.utilities import get_message  # noqa: E402
import yaml  # noqa: E402


REPO_ROOT = Path(__file__).resolve().parents[2]
VAD_SCRIPTS = REPO_ROOT / "autoware_e2e_vad_launch/scripts"
if str(VAD_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(VAD_SCRIPTS))

from vad_route_logic import (  # noqa: E402
    RoutePlan,
    constrain_trajectory_points_to_route,
    limit_trajectory_speed_for_curvature,
    limit_trajectory_speed_for_route_curvature,
    limit_trajectory_speed_recovery,
    resample_trajectory_points,
    smooth_trajectory_geometry,
    trajectory_arc_lengths,
    trajectory_planar_curvatures,
    trajectory_sample_distances,
    zero_velocity_distance_for_goal,
)
from vad_route_manager import VadRouteManager  # noqa: E402


ANALYSIS_ID = "town06_60kph_smoothing_production_preflight_v1"
BASELINE_STRENGTH = 10.0
CANDIDATE_STRENGTH = 10000.0
TARGET_SPEED_MPS = 60.0 / 3.6
CURVATURE_LIMIT_PER_M = 1.0 / (15.0**2)
# The recorder starts after the route manager already owns its first progress
# state.  Replaying that state from the recorded Float32 remaining-distance
# sample is not a bitwise replay of the hidden pre-capture state.  A companion
# pairing probe bounded that uncertainty at 0.281 mm / 20.8 urad; the seeded
# implementation below is more accurate, but keeps the conservative explicit
# recorder-boundary allowance.  Point count remains exact for all snapshots.
ORACLE_XY_TOLERANCE_M = 1.0e-3
ORACLE_YAW_TOLERANCE_RAD = 5.0e-5
ORACLE_SCALAR_TOLERANCE = 1.0e-6
FIXED_POINT_TOLERANCE_M = 1.0e-8
CORRIDOR_TOLERANCE_M = 1.0e-6
PAIRING_TOLERANCE_NS = 120_000_000
SELECTION_DEVIATION_LIMIT_M = 0.05
ENDPOINT_YAW_DELTA_LIMIT_RAD = 0.02
SOLVE_P95_LIMIT_MS = 20.0
SOLVE_MAX_LIMIT_MS = 50.0

RAW_TRAJECTORY_TOPIC = "/planning/vad_route/selected_raw_trajectory"
FINAL_TRAJECTORY_TOPIC = "/planning/trajectory"
ODOMETRY_TOPIC = "/localization/kinematic_state"
STATUS_TOPIC = "/planning/vad_route/status"
COMMAND_TOPIC = "/planning/vad_route/command"
REMAINING_TOPIC = "/planning/vad_route/remaining_distance"
GOAL_TOPIC = "/planning/vad_route/goal_reached"
REQUIRED_TOPICS = (
    RAW_TRAJECTORY_TOPIC,
    FINAL_TRAJECTORY_TOPIC,
    ODOMETRY_TOPIC,
    STATUS_TOPIC,
    COMMAND_TOPIC,
    REMAINING_TOPIC,
    GOAL_TOPIC,
)

CANONICAL_TRIAL = REPO_ROOT / (
    "artifacts/validation/2026-09-02/"
    "autoware_vad_runtime_control_campaign_v1/30_60kph/"
    "town06_straight_60kph_geometry_corridor_0p2_v4/trial/attempt_001"
)
CANONICAL_OUTPUT = REPO_ROOT / (
    "artifacts/validation/2026-09-02/"
    "autoware_vad_runtime_control_campaign_v1/50_reports/"
    "town06_60kph_smoothing_production_preflight_v1"
)

PINNED_SHA256 = {
    "source_route.json": "ae019ba6f839935919e7b11fa3a3131255849bfbc7ae191b8a517a3182233018",
    "aligned_route.json": "3d2e0a9784df24f0023cd0519760c2315777017f9659ae3bdf23013b631c620b",
    "bag/metadata.yaml": "04525c3a77091d6533b49241aea4c38619018a0deff2878b3f681ce5feda9236",
    "bag/bag_0.db3": "28a77cb3cb68cd9f0d3b7c31c0d4950114460ad2bf1c83afac6f2271e8071c6a",
    "vad_route_manager.params.yaml": "1c9ad06a2a9dd40fbeb8e3e4990f882e5f22c965a10fbd1ee24ec27b831fdc07",
    "trajectory_code_provenance/vad_route_logic.py": (
        "bce2972346b95a76de968d44e286d7921003d6b802387e620ad5c3ef9cff0db1"
    ),
    "trajectory_code_provenance/vad_route_manager.py": (
        "92f677c151e0b7f3f2f91db0ef1c7d4a89f26388fd974a5d3d9459314ac92fcd"
    ),
}

CURRENT_CODE_BINDINGS = {
    "autoware_e2e_vad_launch/scripts/vad_route_logic.py": (
        "trajectory_code_provenance/vad_route_logic.py"
    ),
    "autoware_e2e_vad_launch/scripts/vad_route_manager.py": (
        "trajectory_code_provenance/vad_route_manager.py"
    ),
}

PARAMETER_CONTRACT: dict[str, Any] = {
    "comfortable_deceleration_mps2": 2.0,
    "controller_stop_offset_m": 0.6,
    "curvature_speed_preview_m": 6.0,
    "goal_tolerance_m": 1.0,
    "left_turn_outward_corridor_half_width_m": 0.0,
    "left_turn_trajectory_lateral_filter_gain": 0.0,
    "longitudinal_velocity_source": "explicit_simulation_nominal",
    "maneuver_exit_lookahead_m": 3.5,
    "maneuver_lookahead_m": 6.0,
    "max_trajectory_segment_m": 10.0,
    "maximum_lateral_acceleration_mps2": 1.0,
    "maximum_longitudinal_acceleration_mps2": 1.5,
    "maximum_speed_mps": TARGET_SPEED_MPS,
    "nominal_cruise_speed_mps": TARGET_SPEED_MPS,
    "right_turn_outward_corridor_half_width_m": 0.0,
    "right_turn_trajectory_lateral_filter_gain": 0.0,
    "route_corridor_entry_distance_m": 0.0,
    "route_corridor_half_width_m": 0.2,
    "route_corridor_mode": "hard",
    "route_curvature_lookahead_m": 40.0,
    "route_projection_backtrack_m": 3.0,
    "route_projection_forward_m": 80.0,
    "trajectory_geometry_smoothing_interval_m": 0.25,
    "trajectory_geometry_smoothing_max_deviation_m": 0.1,
    "trajectory_geometry_smoothing_strength": BASELINE_STRENGTH,
    "trajectory_lateral_filter_activation_threshold_m": 0.0,
    "trajectory_lateral_filter_gain": 1.0,
    "trajectory_resample_interval_m": 0.5,
    "turn_inward_corridor_half_width_m": 0.2,
    "turn_outward_corridor_half_width_m": 0.2,
}


class PreflightError(RuntimeError):
    """Raised when immutable inputs or the reconstruction contract are invalid."""


@dataclass(frozen=True)
class BagRecord:
    topic: str
    storage_ns: int
    header_ns: int | None
    message: Any
    sequence: int


@dataclass(frozen=True)
class TrajectoryPair:
    raw: BagRecord
    final: BagRecord


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def percentile(values: Sequence[float], fraction: float) -> float | None:
    if not values:
        return None
    return float(np.percentile(np.asarray(values, dtype=float), fraction * 100.0))


def finite_number(value: object) -> bool:
    return (
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and math.isfinite(float(value))
    )


def same_value(actual: object, expected: object) -> bool:
    if isinstance(expected, float):
        return finite_number(actual) and math.isclose(
            float(actual), expected, rel_tol=0.0, abs_tol=1.0e-9
        )
    return actual == expected


def message_header_ns(message: Any) -> int | None:
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def yaw_from_point(point: Any) -> float:
    quaternion = point.pose.orientation
    return math.atan2(
        2.0
        * (
            quaternion.w * quaternion.z
            + quaternion.x * quaternion.y
        ),
        1.0
        - 2.0
        * (
            quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
        ),
    )


def angle_distance(left: float, right: float) -> float:
    return abs(math.atan2(math.sin(left - right), math.cos(left - right)))


def load_parameter_dump(path: Path) -> dict[str, Any]:
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
        parameters = document["/vad_route_manager"]["ros__parameters"]
    except (OSError, UnicodeDecodeError, yaml.YAMLError, KeyError, TypeError) as error:
        raise PreflightError(f"cannot load route-manager parameter dump: {error}") from error
    if not isinstance(parameters, dict):
        raise PreflightError("route-manager parameter dump is not a mapping")
    return parameters


def validate_immutable_inputs(
    trial: Path, *, allow_route_logic_prototype: bool = False
) -> tuple[dict[str, Any], dict[str, Any]]:
    failures: list[str] = []
    identities: dict[str, Any] = {}
    for relative, expected in PINNED_SHA256.items():
        path = trial / relative
        if path.is_symlink() or not path.is_file():
            failures.append(f"missing/non-regular pinned input: {relative}")
            continue
        actual = sha256_file(path)
        identities[relative] = {
            "path": str(path.resolve()),
            "sha256": actual,
            "expected_sha256": expected,
            "status": "PASS" if actual == expected else "FAILED",
        }
        if actual != expected:
            failures.append(f"pinned SHA-256 mismatch: {relative}")

    for current_relative, captured_relative in CURRENT_CODE_BINDINGS.items():
        current = REPO_ROOT / current_relative
        captured = trial / captured_relative
        if not current.is_file() or not captured.is_file():
            failures.append(f"current/captured production code is missing: {current_relative}")
            continue
        current_sha = sha256_file(current)
        captured_sha = sha256_file(captured)
        prototype_extension = (
            allow_route_logic_prototype
            and current_relative.endswith("/vad_route_logic.py")
        )
        identities[f"current:{current_relative}"] = {
            "path": str(current.resolve()),
            "sha256": current_sha,
            "captured_sha256": captured_sha,
            "status": (
                "PASS"
                if current_sha == captured_sha
                else (
                    "PROTOTYPE_EXTENSION_REQUIRES_BASELINE_ORACLE"
                    if prototype_extension
                    else "FAILED"
                )
            ),
        }
        if current_sha != captured_sha and not prototype_extension:
            failures.append(f"current production code differs from v4 capture: {current_relative}")

    try:
        aligned = json.loads((trial / "aligned_route.json").read_text(encoding="utf-8"))
        source = json.loads((trial / "source_route.json").read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise PreflightError(f"cannot read route evidence: {error}") from error
    alignment = aligned.get("coordinate_alignment")
    physical = source.get("physical_straight_preflight")
    commands = {
        point.get("vad_command")
        for point in aligned.get("route", [])
        if isinstance(point, dict)
    }
    if (
        aligned.get("town") != "Town06"
        or aligned.get("scenario") != "straight"
        or source.get("town") != "Town06"
        or source.get("scenario") != "straight"
    ):
        failures.append("source/aligned route identity is not Town06/straight")
    if not isinstance(physical, dict) or physical.get("status") != "PASS":
        failures.append("physical straight-route preflight is not PASS")
    if not commands or not commands.issubset({2, 3}):
        failures.append(f"straight-only route contains non-straight VAD commands: {commands}")
    expected_source_sha = PINNED_SHA256["source_route.json"]
    if (
        not isinstance(alignment, dict)
        or alignment.get("source_route_sha256") != expected_source_sha
    ):
        failures.append("aligned route is not bound to the pinned source route")

    parameters = load_parameter_dump(trial / "vad_route_manager.params.yaml")
    for name, expected in PARAMETER_CONTRACT.items():
        if not same_value(parameters.get(name), expected):
            failures.append(
                f"route-manager parameter changed: {name}={parameters.get(name)!r}"
            )
    recorded_route = Path(str(parameters.get("route_file", ""))).expanduser()
    if not recorded_route.is_absolute() or recorded_route.resolve() != (
        trial / "aligned_route.json"
    ).resolve():
        failures.append("route-manager parameter route_file is not the v4 aligned route")

    metadata_path = trial / "bag/metadata.yaml"
    try:
        metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
        bag_info = metadata["rosbag2_bagfile_information"]
    except (OSError, UnicodeDecodeError, yaml.YAMLError, KeyError, TypeError) as error:
        raise PreflightError(f"cannot read rosbag metadata: {error}") from error
    relative_files = bag_info.get("relative_file_paths")
    if relative_files != ["bag_0.db3"]:
        failures.append(f"unexpected rosbag storage files: {relative_files!r}")

    identities["route_contract"] = {
        "town": aligned.get("town"),
        "scenario": aligned.get("scenario"),
        "source_route_sha256": (
            alignment.get("source_route_sha256")
            if isinstance(alignment, dict)
            else None
        ),
        "physical_straight_status": (
            physical.get("status") if isinstance(physical, dict) else None
        ),
        "vad_commands": sorted(commands, key=lambda value: str(value)),
    }
    identities["preflight_script"] = {
        "path": str(Path(__file__).resolve()),
        "sha256": sha256_file(Path(__file__).resolve()),
    }
    if failures:
        raise PreflightError("; ".join(failures))
    return parameters, identities


def read_required_bag_records(bag: Path) -> tuple[dict[str, list[BagRecord]], dict[str, str]]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag.resolve()), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    missing = [topic for topic in REQUIRED_TOPICS if topic not in topic_types]
    if missing:
        raise PreflightError(f"bag is missing required topics: {missing}")
    message_types = {topic: get_message(topic_types[topic]) for topic in REQUIRED_TOPICS}
    records: dict[str, list[BagRecord]] = {topic: [] for topic in REQUIRED_TOPICS}
    sequence = 0
    while reader.has_next():
        topic, data, storage_ns = reader.read_next()
        if topic not in records:
            continue
        message = deserialize_message(data, message_types[topic])
        records[topic].append(
            BagRecord(
                topic=topic,
                storage_ns=int(storage_ns),
                header_ns=message_header_ns(message),
                message=message,
                sequence=sequence,
            )
        )
        sequence += 1
    empty = [topic for topic, values in records.items() if not values]
    if empty:
        raise PreflightError(f"bag has no messages for required topics: {empty}")
    return records, topic_types


def pair_trajectory_records(
    raw_records: Sequence[BagRecord],
    final_records: Sequence[BagRecord],
    tolerance_ns: int = PAIRING_TOLERANCE_NS,
) -> tuple[list[TrajectoryPair], list[str], int]:
    """Pair each raw callback with the nearest unused final sharing its ROS stamp."""
    finals_by_header: dict[int, list[BagRecord]] = defaultdict(list)
    for record in final_records:
        if record.header_ns is not None:
            finals_by_header[record.header_ns].append(record)
    for values in finals_by_header.values():
        values.sort(key=lambda item: (item.storage_ns, item.sequence))

    used: set[int] = set()
    pairs: list[TrajectoryPair] = []
    failures: list[str] = []
    for raw in sorted(raw_records, key=lambda item: (item.storage_ns, item.sequence)):
        if raw.header_ns is None:
            failures.append(f"raw trajectory sequence {raw.sequence} has no header stamp")
            continue
        available = [
            item
            for item in finals_by_header.get(raw.header_ns, [])
            if item.sequence not in used
        ]
        if not available:
            failures.append(
                f"raw trajectory sequence {raw.sequence} has no same-stamp final"
            )
            continue
        causal = [item for item in available if item.storage_ns >= raw.storage_ns]
        if not causal:
            failures.append(
                f"raw trajectory sequence {raw.sequence} has no causal same-stamp final"
            )
            continue
        earliest_storage_ns = min(item.storage_ns for item in causal)
        earliest = [
            item for item in causal if item.storage_ns == earliest_storage_ns
        ]
        if len(earliest) != 1:
            failures.append(
                f"raw trajectory sequence {raw.sequence} has ambiguous earliest causal finals"
            )
            continue
        selected = earliest[0]
        delta_ns = selected.storage_ns - raw.storage_ns
        if delta_ns > tolerance_ns:
            failures.append(
                f"raw trajectory sequence {raw.sequence} pairing delta "
                f"{delta_ns / 1.0e6:.3f} ms exceeds {tolerance_ns / 1.0e6:.1f} ms"
            )
            continue
        used.add(selected.sequence)
        pairs.append(TrajectoryPair(raw=raw, final=selected))
    return pairs, failures, len(final_records) - len(used)


def nearest_record(records: Sequence[BagRecord], storage_ns: int) -> BagRecord:
    return min(records, key=lambda item: abs(item.storage_ns - storage_ns))


def replay_progress(
    route: RoutePlan,
    pairs: Sequence[TrajectoryPair],
    odometry: Sequence[BagRecord],
    remaining: Sequence[BagRecord],
    parameters: Mapping[str, Any],
) -> tuple[list[tuple[TrajectoryPair, float, float, int]], dict[str, Any]]:
    if not pairs:
        raise PreflightError("no trajectory pairs are available")
    first_remaining = nearest_record(remaining, pairs[0].raw.storage_ns)
    seed_remaining_m = float(first_remaining.message.data)
    if not math.isfinite(seed_remaining_m) or not 0.0 <= seed_remaining_m <= route.length_m:
        raise PreflightError("initial remaining-distance seed is invalid")
    seed_delta_ns = abs(first_remaining.storage_ns - pairs[0].raw.storage_ns)
    if seed_delta_ns > PAIRING_TOLERANCE_NS:
        raise PreflightError("initial remaining-distance seed is not contemporaneous")

    progress_m = max(0.0, route.length_m - seed_remaining_m)
    ordered_odometry = sorted(
        (item for item in odometry if item.header_ns is not None),
        key=lambda item: (item.header_ns, item.storage_ns, item.sequence),
    )
    odometry_index = 0
    output: list[tuple[TrajectoryPair, float, float, int]] = []
    for pair in sorted(pairs, key=lambda item: item.raw.header_ns or -1):
        while (
            odometry_index < len(ordered_odometry)
            and ordered_odometry[odometry_index].header_ns <= pair.raw.header_ns
        ):
            position = ordered_odometry[odometry_index].message.pose.pose.position
            projection = route.project(
                position.x,
                position.y,
                progress_m,
                float(parameters["route_projection_backtrack_m"]),
                float(parameters["route_projection_forward_m"]),
            )
            progress_m = projection.progress_m
            odometry_index += 1
        remaining_m = route.remaining(progress_m)
        command = route.command_at(
            progress_m,
            float(parameters["maneuver_lookahead_m"]),
            float(parameters["maneuver_exit_lookahead_m"]),
        )
        output.append((pair, progress_m, remaining_m, command))
    return output, {
        "method": (
            "first recorded Float32 remaining-distance seed followed by causal "
            "header-time odometry replay through RoutePlan.project"
        ),
        "seed_remaining_m": seed_remaining_m,
        "seed_progress_m": max(0.0, route.length_m - seed_remaining_m),
        "seed_storage_delta_ms": seed_delta_ns / 1.0e6,
        "odometry_record_count": len(ordered_odometry),
    }


def route_polyline(route: RoutePlan) -> np.ndarray:
    return np.asarray([(point.x, point.y) for point in route.points], dtype=float)


def point_to_polyline_distances(points: np.ndarray, polyline: np.ndarray) -> np.ndarray:
    starts = polyline[:-1]
    vectors = polyline[1:] - starts
    length_squared = np.einsum("ij,ij->i", vectors, vectors)
    valid = length_squared > 1.0e-12
    if not np.any(valid):
        raise PreflightError("route polyline has no non-zero segment")
    relative = points[:, None, :] - starts[None, :, :]
    fractions = np.zeros((len(points), len(vectors)), dtype=float)
    fractions[:, valid] = np.clip(
        np.einsum("pij,ij->pi", relative[:, valid], vectors[valid])
        / length_squared[valid][None, :],
        0.0,
        1.0,
    )
    nearest = starts[None, :, :] + fractions[:, :, None] * vectors[None, :, :]
    squared = np.einsum(
        "pij,pij->pi",
        points[:, None, :] - nearest,
        points[:, None, :] - nearest,
    )
    squared[:, ~valid] = np.inf
    return np.sqrt(np.min(squared, axis=1))


def bidirectional_polyline_deviation_m(
    first_points: Sequence[Any], second_points: Sequence[Any]
) -> float:
    first_xy = trajectory_xy(first_points)
    second_xy = trajectory_xy(second_points)
    return max(
        float(np.max(point_to_polyline_distances(first_xy, second_xy))),
        float(np.max(point_to_polyline_distances(second_xy, first_xy))),
    )


def trajectory_xy(points: Sequence[Any]) -> np.ndarray:
    return np.asarray(
        [(point.pose.position.x, point.pose.position.y) for point in points],
        dtype=float,
    )


def count_self_intersections(points: Sequence[Any]) -> int:
    xy = trajectory_xy(points)
    if len(xy) < 4:
        return 0

    # A curve strictly monotone along its endpoint chord cannot self-cross.
    # Town06's straight-route horizons satisfy this cheap proof, avoiding an
    # O(N^2) Python loop for every one of the 399 snapshots.  Keep a vectorized
    # exact fallback so the helper remains fail-closed for unusual geometry.
    chord = xy[-1] - xy[0]
    chord_length = float(np.linalg.norm(chord))
    if chord_length > 1.0e-9:
        projected = xy @ (chord / chord_length)
        if np.all(np.diff(projected) > 1.0e-9):
            return 0

    starts = xy[:-1]
    ends = xy[1:]
    minimum = np.minimum(starts, ends)
    maximum = np.maximum(starts, ends)
    count = 0
    for left in range(len(starts)):
        indexes = np.arange(left + 2, len(starts))
        if not len(indexes):
            continue
        bbox = np.all(maximum[left] >= minimum[indexes], axis=1) & np.all(
            maximum[indexes] >= minimum[left], axis=1
        )
        indexes = indexes[bbox]
        if not len(indexes):
            continue
        a = starts[left]
        b = ends[left]
        c = starts[indexes]
        d = ends[indexes]

        def cross(left_vector: np.ndarray, right_vector: np.ndarray) -> np.ndarray:
            return (
                left_vector[..., 0] * right_vector[..., 1]
                - left_vector[..., 1] * right_vector[..., 0]
            )

        ab = b - a
        cd = d - c
        o1 = cross(ab, c - a)
        o2 = cross(ab, d - a)
        o3 = cross(cd, a - c)
        o4 = cross(cd, b - c)
        count += int(np.count_nonzero((o1 * o2 < -1.0e-12) & (o3 * o4 < -1.0e-12)))
    return count


def build_pre_smoothing_points(
    raw_trajectory: Any,
    route: RoutePlan,
    progress_m: float,
    remaining_m: float,
    command: int,
    parameters: Mapping[str, Any],
) -> tuple[list[Any], float, float, float, float]:
    stop_distance_m = zero_velocity_distance_for_goal(
        remaining_m,
        float(parameters["goal_tolerance_m"]),
        float(parameters["controller_stop_offset_m"]),
    )
    raw_distances = trajectory_arc_lengths(raw_trajectory.points)
    points = resample_trajectory_points(
        raw_trajectory.points,
        float(parameters["trajectory_resample_interval_m"]),
        extra_distances=(*raw_distances[1:-1], stop_distance_m),
    )
    route_width = float(parameters["route_corridor_half_width_m"])
    turn_inward = float(parameters["turn_inward_corridor_half_width_m"])
    turn_outward = float(parameters["turn_outward_corridor_half_width_m"])
    left_override = float(parameters["left_turn_outward_corridor_half_width_m"])
    right_override = float(parameters["right_turn_outward_corridor_half_width_m"])
    left_outward = turn_outward if left_override == 0.0 else left_override
    right_outward = turn_outward if right_override == 0.0 else right_override
    lateral_min, lateral_max = VadRouteManager._lateral_corridor_bounds(
        command,
        route_width,
        turn_inward,
        left_outward,
        right_outward,
    )
    corridor_correction_m = constrain_trajectory_points_to_route(
        points,
        route,
        progress_m,
        route_width,
        mode=str(parameters["route_corridor_mode"]),
        lateral_offset_min_m=lateral_min,
        lateral_offset_max_m=lateral_max,
        entry_distance_m=float(parameters["route_corridor_entry_distance_m"]),
    )
    # The pinned v4 profile intentionally disables temporal blending.  Refuse
    # to silently approximate a future profile where that is no longer true.
    if not math.isclose(
        float(parameters["trajectory_lateral_filter_gain"]),
        1.0,
        rel_tol=0.0,
        abs_tol=1.0e-12,
    ):
        raise PreflightError("stateful lateral filter replay is not implemented")
    return points, stop_distance_m, lateral_min, lateral_max, corridor_correction_m


def production_manager(parameters: Mapping[str, Any]) -> SimpleNamespace:
    return SimpleNamespace(
        comfortable_deceleration_mps2=float(
            parameters["comfortable_deceleration_mps2"]
        ),
        longitudinal_velocity_source=str(parameters["longitudinal_velocity_source"]),
        nominal_cruise_speed_mps=float(parameters["nominal_cruise_speed_mps"]),
        maximum_speed_mps=float(parameters["maximum_speed_mps"]),
        maximum_longitudinal_acceleration_mps2=float(
            parameters["maximum_longitudinal_acceleration_mps2"]
        ),
        max_trajectory_segment_m=float(parameters["max_trajectory_segment_m"]),
    )


def apply_velocity_pipeline(
    raw_trajectory: Any,
    points: Sequence[Any],
    adjusted_stop_distance_m: float,
    route: RoutePlan,
    progress_m: float,
    parameters: Mapping[str, Any],
) -> Any:
    output = copy.deepcopy(raw_trajectory)
    output.points = list(points)
    manager = production_manager(parameters)
    VadRouteManager._apply_velocity_profile(
        manager, output.points, adjusted_stop_distance_m
    )
    limit_trajectory_speed_for_curvature(
        output.points,
        float(parameters["maximum_lateral_acceleration_mps2"]),
        float(parameters["curvature_speed_preview_m"]),
        float(parameters["comfortable_deceleration_mps2"]),
    )
    limit_trajectory_speed_for_route_curvature(
        output.points,
        route,
        progress_m,
        float(parameters["maximum_lateral_acceleration_mps2"]),
        float(parameters["route_curvature_lookahead_m"]),
        float(parameters["comfortable_deceleration_mps2"]),
    )
    limit_trajectory_speed_recovery(
        output.points,
        float(parameters["maximum_longitudinal_acceleration_mps2"]),
    )
    VadRouteManager._recalculate_acceleration(manager, output)
    VadRouteManager._validate_output_trajectory(manager, output)
    return output


def first_zero_position(points: Sequence[Any]) -> tuple[float, float] | None:
    for point in points:
        if point.longitudinal_velocity_mps <= 1.0e-3:
            return (float(point.pose.position.x), float(point.pose.position.y))
    return None


def shape_variant(
    raw_trajectory: Any,
    pre_smoothing_points: Sequence[Any],
    route: RoutePlan,
    route_xy: np.ndarray,
    progress_m: float,
    stop_distance_m: float,
    lateral_min_m: float,
    lateral_max_m: float,
    strength: float,
    parameters: Mapping[str, Any],
    corridor_saturation_mode: str = "legacy",
    corridor_transition_width_m: float = 0.0,
    corridor_endpoint_taper_m: float = 0.0,
) -> tuple[Any, dict[str, Any]]:
    interval_m = float(parameters["trajectory_geometry_smoothing_interval_m"])
    max_deviation_m = float(
        parameters["trajectory_geometry_smoothing_max_deviation_m"]
    )
    comparison = resample_trajectory_points(
        pre_smoothing_points, interval_m, extra_distances=(stop_distance_m,)
    )
    comparison_distances = trajectory_sample_distances(
        trajectory_arc_lengths(pre_smoothing_points)[-1],
        interval_m,
        extra_distances=(stop_distance_m,),
    )
    stop_anchor_index = None
    if 0.0 <= stop_distance_m <= comparison_distances[-1]:
        candidate_index = min(
            range(len(comparison_distances)),
            key=lambda index: abs(comparison_distances[index] - stop_distance_m),
        )
        if abs(comparison_distances[candidate_index] - stop_distance_m) <= 1.0e-5:
            stop_anchor_index = candidate_index

    points = copy.deepcopy(list(pre_smoothing_points))
    started = time.perf_counter()
    rejection = None
    smoothing_result = None
    try:
        smoothing_result = smooth_trajectory_geometry(
            points,
            route,
            progress_m,
            strength,
            interval_m,
            max_deviation_m,
            float(parameters["route_corridor_half_width_m"]),
            mode=str(parameters["route_corridor_mode"]),
            lateral_offset_min_m=lateral_min_m,
            lateral_offset_max_m=lateral_max_m,
            stop_distance_m=stop_distance_m,
            entry_distance_m=float(parameters["route_corridor_entry_distance_m"]),
            corridor_saturation_mode=corridor_saturation_mode,
            corridor_transition_width_m=corridor_transition_width_m,
            corridor_endpoint_taper_m=corridor_endpoint_taper_m,
        )
    except ValueError as error:
        rejection = str(error)
    solve_ms = (time.perf_counter() - started) * 1000.0

    if smoothing_result is None:
        # This is exactly the route-manager's fail-open geometry behavior.  The
        # rejection remains a hard preflight failure and is never hidden by the
        # fallback metrics.
        adjusted_stop_distance_m = stop_distance_m
        smoothing_deviation_m = None
        endpoint_shift_m = None
        stop_anchor_shift_m = None
        start_yaw_change_rad = None
        end_yaw_change_rad = None
    else:
        adjusted_stop_distance_m = float(smoothing_result.stop_distance_m)
        smoothing_deviation_m = float(smoothing_result.correction_m)
        endpoint_shift_m = max(
            float(np.linalg.norm(trajectory_xy(points)[0] - trajectory_xy(comparison)[0])),
            float(np.linalg.norm(trajectory_xy(points)[-1] - trajectory_xy(comparison)[-1])),
        )
        stop_anchor_shift_m = (
            float(
                np.linalg.norm(
                    trajectory_xy(points)[stop_anchor_index]
                    - trajectory_xy(comparison)[stop_anchor_index]
                )
            )
            if stop_anchor_index is not None
            else None
        )
        start_yaw_change_rad = angle_distance(
            yaw_from_point(points[0]), yaw_from_point(comparison[0])
        )
        end_yaw_change_rad = angle_distance(
            yaw_from_point(points[-1]), yaw_from_point(comparison[-1])
        )

    output = apply_velocity_pipeline(
        raw_trajectory,
        points,
        adjusted_stop_distance_m,
        route,
        progress_m,
        parameters,
    )
    xy = trajectory_xy(output.points)
    route_errors = point_to_polyline_distances(xy, route_xy)
    curvature = np.abs(
        np.asarray(trajectory_planar_curvatures(output.points), dtype=float)
    )
    segments = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    return output, {
        "strength": strength,
        "corridor_saturation_mode": corridor_saturation_mode,
        "corridor_transition_width_m": corridor_transition_width_m,
        "corridor_endpoint_taper_m": corridor_endpoint_taper_m,
        "smoothing_applied": smoothing_result is not None,
        "rejection": rejection,
        "solve_time_ms": solve_ms,
        "point_count": len(output.points),
        "length_m": float(np.sum(segments)),
        "curvature_pointwise_p95_per_m": float(np.percentile(curvature, 95)),
        "curvature_peak_per_m": float(np.max(curvature)),
        "route_cte_max_m": float(np.max(route_errors)),
        "route_boundary_point_count": int(
            np.count_nonzero(
                route_errors
                >= float(parameters["route_corridor_half_width_m"])
                - CORRIDOR_TOLERANCE_M
            )
        ),
        "smoothing_deviation_max_m": smoothing_deviation_m,
        "fixed_endpoint_shift_m": endpoint_shift_m,
        "fixed_stop_anchor_shift_m": stop_anchor_shift_m,
        "start_yaw_change_rad": start_yaw_change_rad,
        "end_yaw_change_rad": end_yaw_change_rad,
        "self_intersection_count": count_self_intersections(output.points),
        "minimum_segment_m": float(np.min(segments)),
        "maximum_segment_m": float(np.max(segments)),
        "trajectory_horizon_minimum_mps": min(
            float(point.longitudinal_velocity_mps) for point in output.points
        ),
        "first_zero_position": first_zero_position(output.points),
        "adjusted_stop_distance_m": adjusted_stop_distance_m,
    }


def compare_trajectory_payload(actual: Any, expected: Any) -> dict[str, Any]:
    if len(actual.points) != len(expected.points):
        return {
            "point_count_match": False,
            "actual_point_count": len(actual.points),
            "expected_point_count": len(expected.points),
            "maximum_xy_error_m": None,
            "maximum_yaw_error_rad": None,
            "maximum_scalar_error": None,
            "time_from_start_exact": False,
            "geometry_status": "FAILED",
            "scalar_time_status": "FAILED",
            "status": "FAILED",
        }
    xy_errors: list[float] = []
    yaw_errors: list[float] = []
    scalar_errors: list[float] = []
    time_exact = True
    scalar_fields = (
        "longitudinal_velocity_mps",
        "lateral_velocity_mps",
        "acceleration_mps2",
        "heading_rate_rps",
        "front_wheel_angle_rad",
        "rear_wheel_angle_rad",
    )
    for left, right in zip(actual.points, expected.points):
        xy_errors.append(
            math.hypot(
                left.pose.position.x - right.pose.position.x,
                left.pose.position.y - right.pose.position.y,
            )
        )
        yaw_errors.append(angle_distance(yaw_from_point(left), yaw_from_point(right)))
        scalar_errors.append(abs(left.pose.position.z - right.pose.position.z))
        scalar_errors.extend(
            abs(float(getattr(left, name)) - float(getattr(right, name)))
            for name in scalar_fields
        )
        time_exact = time_exact and left.time_from_start == right.time_from_start
    maximum_xy = max(xy_errors, default=0.0)
    maximum_yaw = max(yaw_errors, default=0.0)
    maximum_scalar = max(scalar_errors, default=0.0)
    geometry_passed = (
        maximum_xy <= ORACLE_XY_TOLERANCE_M
        and maximum_yaw <= ORACLE_YAW_TOLERANCE_RAD
    )
    scalar_time_passed = maximum_scalar <= ORACLE_SCALAR_TOLERANCE and time_exact
    return {
        "point_count_match": True,
        "actual_point_count": len(actual.points),
        "expected_point_count": len(expected.points),
        "maximum_xy_error_m": maximum_xy,
        "maximum_yaw_error_rad": maximum_yaw,
        "maximum_scalar_error": maximum_scalar,
        "time_from_start_exact": time_exact,
        "geometry_status": "PASS" if geometry_passed else "FAILED",
        "scalar_time_status": "PASS" if scalar_time_passed else "FAILED",
        "status": "PASS" if geometry_passed and scalar_time_passed else "FAILED",
    }


def summary_distribution(values: Iterable[float | None]) -> dict[str, Any]:
    finite = [float(value) for value in values if value is not None and math.isfinite(value)]
    return {
        "count": len(finite),
        "minimum": min(finite) if finite else None,
        "median": percentile(finite, 0.5),
        "p95": percentile(finite, 0.95),
        "maximum": max(finite) if finite else None,
    }


def summarize_variant(rows: Sequence[Mapping[str, Any]], name: str) -> dict[str, Any]:
    values = [row[name] for row in rows]
    rejection_reasons = [
        item["rejection"] for item in values if item.get("rejection") is not None
    ]
    return {
        "snapshot_count": len(values),
        "smoothing_applied_count": sum(
            item["smoothing_applied"] is True for item in values
        ),
        "smoothing_rejection_count": len(rejection_reasons),
        "smoothing_rejection_reasons": sorted(set(rejection_reasons)),
        "snapshot_peak_curvature_per_m": summary_distribution(
            item["curvature_peak_per_m"] for item in values
        ),
        "pointwise_curvature_p95_per_m": summary_distribution(
            item["curvature_pointwise_p95_per_m"] for item in values
        ),
        "smoothing_deviation_max_m": summary_distribution(
            item["smoothing_deviation_max_m"] for item in values
        ),
        "route_cte_max_m": summary_distribution(
            item["route_cte_max_m"] for item in values
        ),
        "fixed_endpoint_shift_m": summary_distribution(
            item["fixed_endpoint_shift_m"] for item in values
        ),
        "fixed_stop_anchor_shift_m": summary_distribution(
            item["fixed_stop_anchor_shift_m"] for item in values
        ),
        "start_yaw_change_rad": summary_distribution(
            item["start_yaw_change_rad"] for item in values
        ),
        "end_yaw_change_rad": summary_distribution(
            item["end_yaw_change_rad"] for item in values
        ),
        "solve_time_ms": summary_distribution(item["solve_time_ms"] for item in values),
        "self_intersection_count": sum(
            int(item["self_intersection_count"]) for item in values
        ),
        "route_boundary_point_count": sum(
            int(item["route_boundary_point_count"]) for item in values
        ),
        "minimum_segment_m": summary_distribution(
            item["minimum_segment_m"] for item in values
        ),
        "trajectory_horizon_minimum_mps": summary_distribution(
            item["trajectory_horizon_minimum_mps"] for item in values
        ),
    }


def summarize_oracle(rows: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    oracle = [row["baseline_oracle"] for row in rows]
    if not rows:
        return {
            "status": "FAILED",
            "snapshot_count": 0,
            "failed_snapshot_count": 0,
            "failed_snapshot_indexes": [],
            "reason": "no active snapshots",
        }
    first_progress_m = float(rows[0]["progress_m"])
    startup_rows: list[Mapping[str, Any]] = []
    for row in rows:
        if float(row["progress_m"]) <= first_progress_m + 1.0e-4:
            startup_rows.append(row)
        else:
            break
    startup_indexes = [row["snapshot_index"] for row in startup_rows]
    startup_index_set = set(startup_indexes)
    geometry_failed = [
        row["snapshot_index"]
        for row in rows
        if row["baseline_oracle"]["geometry_status"] != "PASS"
    ]
    strict_payload_failed = [
        row["snapshot_index"]
        for row in rows
        if row["snapshot_index"] not in startup_index_set
        and row["baseline_oracle"]["scalar_time_status"] != "PASS"
    ]
    failed = sorted(set(geometry_failed + strict_payload_failed))
    startup_payload_mismatch = [
        row["snapshot_index"]
        for row in startup_rows
        if row["baseline_oracle"]["scalar_time_status"] != "PASS"
    ]
    strict_rows = [
        row for row in rows if row["snapshot_index"] not in startup_index_set
    ]
    startup_residual_indexes = [
        row["snapshot_index"]
        for row in rows
        if row["baseline_oracle"]["maximum_xy_error_m"] is not None
        and (
            row["baseline_oracle"]["maximum_xy_error_m"] > 1.0e-9
            or row["baseline_oracle"]["maximum_yaw_error_rad"] > 1.0e-9
        )
    ]
    return {
        "status": "PASS" if not failed and oracle else "FAILED",
        "snapshot_count": len(oracle),
        "failed_snapshot_count": len(failed),
        "failed_snapshot_indexes": failed,
        "tolerances": {
            "maximum_xy_error_m": ORACLE_XY_TOLERANCE_M,
            "maximum_yaw_error_rad": ORACLE_YAW_TOLERANCE_RAD,
            "maximum_scalar_error": ORACLE_SCALAR_TOLERANCE,
            "time_from_start_exact": True,
        },
        "geometry": {
            "status": "PASS" if not geometry_failed else "FAILED",
            "snapshot_count": len(rows),
            "failed_snapshot_indexes": geometry_failed,
            "point_count_exact": all(
                item["point_count_match"] is True for item in oracle
            ),
            "maximum_xy_error_m": max(
                (item["maximum_xy_error_m"] or 0.0 for item in oracle),
                default=None,
            ),
            "maximum_yaw_error_rad": max(
                (item["maximum_yaw_error_rad"] or 0.0 for item in oracle),
                default=None,
            ),
        },
        "scalar_time_after_startup": {
            "status": "PASS" if not strict_payload_failed else "FAILED",
            "snapshot_count": len(strict_rows),
            "failed_snapshot_indexes": strict_payload_failed,
            "maximum_scalar_error": max(
                (
                    row["baseline_oracle"]["maximum_scalar_error"] or 0.0
                    for row in strict_rows
                ),
                default=None,
            ),
            "all_time_from_start_exact": all(
                row["baseline_oracle"]["time_from_start_exact"] is True
                for row in strict_rows
            ),
        },
        "startup_context_missing": {
            "status": "DOCUMENTED_EXCLUSION",
            "reason": (
                "route-manager state already existed before rosbag capture; the "
                "initial stationary prefix cannot reproduce scalar/time payload "
                "exactly from recorded topics"
            ),
            "progress_threshold_m": first_progress_m + 1.0e-4,
            "excluded_snapshot_count": len(startup_rows),
            "excluded_snapshot_indexes": startup_indexes,
            "observed_scalar_time_mismatch_count": len(startup_payload_mismatch),
            "observed_scalar_time_mismatch_indexes": startup_payload_mismatch,
        },
        "maximum_xy_error_m": max(
            (item["maximum_xy_error_m"] or 0.0 for item in oracle), default=None
        ),
        "maximum_yaw_error_rad": max(
            (item["maximum_yaw_error_rad"] or 0.0 for item in oracle), default=None
        ),
        "maximum_scalar_error": max(
            (item["maximum_scalar_error"] or 0.0 for item in oracle), default=None
        ),
        "all_time_from_start_exact": all(
            item["time_from_start_exact"] is True for item in oracle
        ),
        "non_negligible_residual_snapshot_indexes": startup_residual_indexes,
        "note": (
            "The first progress state predates rosbag capture. It is recovered from "
            "the recorded Float32 remaining-distance value. A companion pairing "
            "probe bounded alternate bag-only startup reconstruction at 0.281 mm / "
            "20.8 urad; the explicit 1 mm / 50 urad geometry bounds conservatively "
            "cover that missing state. This report records its seeded replay maxima "
            "directly. Scalar/time equality is evaluated separately after the "
            "documented stationary startup prefix."
        ),
    }


def zero_position_shift(left: object, right: object) -> float | None:
    if not (
        isinstance(left, (tuple, list))
        and isinstance(right, (tuple, list))
        and len(left) == 2
        and len(right) == 2
    ):
        return None
    return math.hypot(float(left[0]) - float(right[0]), float(left[1]) - float(right[1]))


def gate_row(
    passed: bool,
    actual: Any,
    requirement: str,
    severity: str,
) -> dict[str, Any]:
    return {
        "status": "PASS" if passed else "FAIL",
        "severity": severity,
        "actual": actual,
        "requirement": requirement,
    }


def evaluate_gates(
    pairing: Mapping[str, Any],
    oracle: Mapping[str, Any],
    baseline: Mapping[str, Any],
    candidate: Mapping[str, Any],
    rows: Sequence[Mapping[str, Any]],
) -> tuple[str, dict[str, Any]]:
    checks: dict[str, dict[str, Any]] = {}
    active_count = int(pairing.get("active_shaped_pair_count", 0))
    checks["all_raw_trajectories_paired"] = gate_row(
        pairing.get("unpaired_raw_count") == 0,
        pairing,
        "every selected raw trajectory has one same-stamp callback result",
        "hard",
    )
    checks["active_snapshot_population"] = gate_row(
        active_count > 0 and baseline.get("snapshot_count") == active_count,
        active_count,
        "at least one active snapshot and no unexplained active exclusions",
        "hard",
    )
    checks["baseline_strength10_oracle"] = gate_row(
        oracle.get("status") == "PASS",
        oracle,
        "strength-10 geometry matches all active trajectories and scalar/time is exact after the documented startup prefix",
        "hard",
    )
    checks["candidate_no_smoothing_rejection"] = gate_row(
        candidate.get("smoothing_rejection_count") == 0,
        candidate.get("smoothing_rejection_count"),
        "strength-10000 produces zero guarded rejection/fail-open snapshots",
        "hard",
    )
    candidate_deviation = candidate["smoothing_deviation_max_m"]["maximum"]
    checks["candidate_hard_deviation_guard"] = gate_row(
        candidate_deviation is not None and candidate_deviation <= 0.1 + 1.0e-9,
        candidate_deviation,
        "all applied smoothing deviations are <= configured 0.10 m guard",
        "hard",
    )
    candidate_cte = candidate["route_cte_max_m"]["maximum"]
    checks["candidate_route_corridor"] = gate_row(
        candidate_cte is not None
        and candidate_cte <= 0.2 + CORRIDOR_TOLERANCE_M,
        candidate_cte,
        "every candidate point remains inside the 0.20 m production corridor",
        "hard",
    )
    endpoint_shift = candidate["fixed_endpoint_shift_m"]["maximum"]
    stop_shift = candidate["fixed_stop_anchor_shift_m"]["maximum"]
    checks["candidate_fixed_endpoints"] = gate_row(
        endpoint_shift is not None and endpoint_shift <= FIXED_POINT_TOLERANCE_M,
        endpoint_shift,
        "first/last positions move by <= 1e-8 m",
        "hard",
    )
    checks["candidate_fixed_stop_anchors"] = gate_row(
        stop_shift is None or stop_shift <= FIXED_POINT_TOLERANCE_M,
        stop_shift,
        "every in-horizon goal stop anchor moves by <= 1e-8 m",
        "hard",
    )
    checks["candidate_valid_geometry"] = gate_row(
        candidate.get("self_intersection_count") == 0
        and candidate["minimum_segment_m"]["minimum"] is not None
        and candidate["minimum_segment_m"]["minimum"] > 1.0e-4,
        {
            "self_intersections": candidate.get("self_intersection_count"),
            "minimum_segment_m": candidate["minimum_segment_m"]["minimum"],
        },
        "no self intersections and every segment is longer than 1e-4 m",
        "hard",
    )

    terminal_buffer_m = TARGET_SPEED_MPS**2 / (
        2.0 * float(PARAMETER_CONTRACT["comfortable_deceleration_mps2"])
    )
    eligible = [
        row
        for row in rows
        if row["progress_m"] >= 5.0 and row["remaining_m"] >= terminal_buffer_m
    ]
    baseline_peak = summary_distribution(
        row["baseline"]["curvature_peak_per_m"] for row in eligible
    )
    candidate_peak = summary_distribution(
        row["candidate"]["curvature_peak_per_m"] for row in eligible
    )
    accepted_eligible = [
        row for row in eligible if row["candidate"]["smoothing_applied"] is True
    ]
    rejected_eligible = [
        row for row in eligible if row["candidate"]["smoothing_applied"] is False
    ]
    accepted_candidate_peak = summary_distribution(
        row["candidate"]["curvature_peak_per_m"] for row in accepted_eligible
    )
    curvature_ratio = (
        candidate_peak["p95"] / baseline_peak["p95"]
        if candidate_peak["p95"] is not None
        and baseline_peak["p95"] is not None
        and baseline_peak["p95"] > 0.0
        else None
    )
    checks["candidate_curvature_p95_supports_15mps"] = gate_row(
        candidate_peak["p95"] is not None
        and candidate_peak["p95"] <= CURVATURE_LIMIT_PER_M,
        candidate_peak["p95"],
        f"moving-midroute adjacent-point trajectory-peak curvature p95 <= {CURVATURE_LIMIT_PER_M:.9f} 1/m",
        "acceptance",
    )
    checks["candidate_curvature_max_supports_15mps"] = gate_row(
        candidate_peak["maximum"] is not None
        and candidate_peak["maximum"] <= CURVATURE_LIMIT_PER_M,
        candidate_peak["maximum"],
        f"moving-midroute maximum adjacent-point trajectory peak curvature <= {CURVATURE_LIMIT_PER_M:.9f} 1/m",
        "acceptance",
    )
    checks["candidate_curvature_material_reduction"] = gate_row(
        curvature_ratio is not None and curvature_ratio <= 0.8,
        curvature_ratio,
        "candidate moving-midroute curvature p95 <= 80% of baseline",
        "acceptance",
    )
    checks["candidate_no_peak_curvature_regression"] = gate_row(
        candidate_peak["maximum"] is not None
        and baseline_peak["maximum"] is not None
        and candidate_peak["maximum"] <= baseline_peak["maximum"],
        {
            "baseline": baseline_peak["maximum"],
            "candidate": candidate_peak["maximum"],
        },
        "candidate maximum curvature does not exceed baseline",
        "acceptance",
    )
    cross_variant_deviation = max(
        (
            float(row["baseline_candidate_bidirectional_deviation_m"])
            for row in rows
        ),
        default=None,
    )
    checks["candidate_selection_deviation_envelope"] = gate_row(
        cross_variant_deviation is not None
        and cross_variant_deviation <= SELECTION_DEVIATION_LIMIT_M,
        cross_variant_deviation,
        "maximum strength-10 to strength-10000 XY deviation stays within 0.05 m",
        "acceptance",
    )

    endpoint_yaw_deltas = [
        max(
            angle_distance(
                yaw_from_point(row["baseline_output"].points[0]),
                yaw_from_point(row["candidate_output"].points[0]),
            ),
            angle_distance(
                yaw_from_point(row["baseline_output"].points[-1]),
                yaw_from_point(row["candidate_output"].points[-1]),
            ),
        )
        for row in rows
    ]
    maximum_endpoint_yaw_delta = max(endpoint_yaw_deltas, default=None)
    checks["candidate_endpoint_yaw_delta"] = gate_row(
        maximum_endpoint_yaw_delta is not None
        and maximum_endpoint_yaw_delta <= ENDPOINT_YAW_DELTA_LIMIT_RAD,
        maximum_endpoint_yaw_delta,
        "candidate endpoint tangent/yaw differs from v4 by <= 0.02 rad",
        "acceptance",
    )

    zero_shifts = [
        zero_position_shift(
            row["baseline"]["first_zero_position"],
            row["candidate"]["first_zero_position"],
        )
        for row in rows
    ]
    finite_zero_shifts = [value for value in zero_shifts if value is not None]
    maximum_zero_shift = max(finite_zero_shifts, default=None)
    checks["candidate_hard_stop_location_delta"] = gate_row(
        maximum_zero_shift is None or maximum_zero_shift <= SELECTION_DEVIATION_LIMIT_M,
        maximum_zero_shift,
        "first hard-stop sentinel location changes by <= 0.05 m",
        "acceptance",
    )

    baseline_caps = [
        row
        for row in eligible
        if 4.0 <= row["baseline"]["trajectory_horizon_minimum_mps"] <= 9.0
    ]
    candidate_caps = [
        row
        for row in eligible
        if 4.0 <= row["candidate"]["trajectory_horizon_minimum_mps"] <= 9.0
    ]
    checks["candidate_midroute_4_to_9mps_caps"] = gate_row(
        len(candidate_caps) == 0,
        {
            "eligible_snapshot_count": len(eligible),
            "baseline_cap_count": len(baseline_caps),
            "candidate_cap_count": len(candidate_caps),
            "terminal_buffer_m": terminal_buffer_m,
        },
        "zero 4-9 m/s horizon caps after 5 m progress and before terminal braking",
        "acceptance",
    )
    latency = candidate["solve_time_ms"]
    checks["candidate_solve_latency"] = gate_row(
        latency["p95"] is not None
        and latency["maximum"] is not None
        and latency["p95"] <= SOLVE_P95_LIMIT_MS
        and latency["maximum"] <= SOLVE_MAX_LIMIT_MS,
        latency,
        "single-thread solve p95 <= 20 ms and maximum <= 50 ms",
        "acceptance",
    )

    hard_failed = any(
        row["status"] == "FAIL" and row["severity"] == "hard"
        for row in checks.values()
    )
    acceptance_failed = any(
        row["status"] == "FAIL" and row["severity"] == "acceptance"
        for row in checks.values()
    )
    status = "FAILED" if hard_failed else ("HOLD" if acceptance_failed else "PASS")
    return status, {
        "status": status,
        "checks": checks,
        "curvature_candidate_to_baseline_p95_ratio": curvature_ratio,
        "maximum_endpoint_yaw_delta_rad": maximum_endpoint_yaw_delta,
        "maximum_first_zero_location_delta_m": maximum_zero_shift,
        "midroute": {
            "definition": {
                "minimum_progress_m": 5.0,
                "minimum_remaining_distance_m": terminal_buffer_m,
                "planning_horizon_speed_range_mps": [4.0, 9.0],
            },
            "eligible_snapshot_count": len(eligible),
            "baseline_adjacent_point_snapshot_peak_curvature_per_m": baseline_peak,
            "candidate_adjacent_point_snapshot_peak_curvature_per_m": candidate_peak,
            "candidate_accepted_only_snapshot_peak_curvature_per_m": (
                accepted_candidate_peak
            ),
            "candidate_accepted_snapshot_count": len(accepted_eligible),
            "candidate_rejected_snapshot_count": len(rejected_eligible),
            "candidate_accepted_curvature_limit_violation_count": sum(
                row["candidate"]["curvature_peak_per_m"] > CURVATURE_LIMIT_PER_M
                for row in accepted_eligible
            ),
            "candidate_curvature_limit_violation_count": sum(
                row["candidate"]["curvature_peak_per_m"] > CURVATURE_LIMIT_PER_M
                for row in eligible
            ),
            "candidate_worst_curvature_snapshots": [
                {
                    "snapshot_index": row["snapshot_index"],
                    "progress_m": row["progress_m"],
                    "remaining_m": row["remaining_m"],
                    "command": row["command"],
                    "curvature_peak_per_m": row["candidate"][
                        "curvature_peak_per_m"
                    ],
                }
                for row in sorted(
                    eligible,
                    key=lambda item: item["candidate"]["curvature_peak_per_m"],
                    reverse=True,
                )[:5]
            ],
            "baseline_cap_snapshot_indexes": [
                row["snapshot_index"] for row in baseline_caps
            ],
            "candidate_cap_snapshot_indexes": [
                row["snapshot_index"] for row in candidate_caps
            ],
        },
    }


def analyze(
    trial: Path,
    *,
    candidate_saturation_mode: str = "legacy",
    candidate_transition_width_m: float = 0.0,
    candidate_endpoint_taper_m: float = 0.0,
) -> dict[str, Any]:
    prototype_enabled = candidate_saturation_mode == "endpoint_tapered_c1"
    parameters, source_identities = validate_immutable_inputs(
        trial, allow_route_logic_prototype=prototype_enabled
    )
    records, topic_types = read_required_bag_records(trial / "bag")
    pairs, pairing_failures, unused_final_count = pair_trajectory_records(
        records[RAW_TRAJECTORY_TOPIC], records[FINAL_TRAJECTORY_TOPIC]
    )
    if pairing_failures:
        raise PreflightError("; ".join(pairing_failures))
    route = RoutePlan.load(trial / "aligned_route.json")
    replayed, progress_source = replay_progress(
        route,
        pairs,
        records[ODOMETRY_TOPIC],
        records[REMAINING_TOPIC],
        parameters,
    )
    route_xy = route_polyline(route)

    rows: list[dict[str, Any]] = []
    stopped_pairs = 0
    for snapshot_index, (pair, progress_m, remaining_m, command) in enumerate(replayed):
        if len(pair.final.message.points) <= 3:
            stopped_pairs += 1
            continue
        if command not in {2, 3}:
            raise PreflightError(
                f"active snapshot {snapshot_index} resolved directional command {command}"
            )
        (
            pre_smoothing,
            stop_distance_m,
            lateral_min_m,
            lateral_max_m,
            corridor_correction_m,
        ) = build_pre_smoothing_points(
            pair.raw.message,
            route,
            progress_m,
            remaining_m,
            command,
            parameters,
        )
        baseline_output, baseline_metrics = shape_variant(
            pair.raw.message,
            pre_smoothing,
            route,
            route_xy,
            progress_m,
            stop_distance_m,
            lateral_min_m,
            lateral_max_m,
            BASELINE_STRENGTH,
            parameters,
        )
        candidate_output, candidate_metrics = shape_variant(
            pair.raw.message,
            pre_smoothing,
            route,
            route_xy,
            progress_m,
            stop_distance_m,
            lateral_min_m,
            lateral_max_m,
            CANDIDATE_STRENGTH,
            parameters,
            corridor_saturation_mode=candidate_saturation_mode,
            corridor_transition_width_m=candidate_transition_width_m,
            corridor_endpoint_taper_m=candidate_endpoint_taper_m,
        )
        baseline_candidate_bidirectional_deviation_m = (
            bidirectional_polyline_deviation_m(
                baseline_output.points, candidate_output.points
            )
        )
        oracle = compare_trajectory_payload(baseline_output, pair.final.message)
        rows.append(
            {
                "snapshot_index": snapshot_index,
                "raw_sequence": pair.raw.sequence,
                "final_sequence": pair.final.sequence,
                "header_ns": pair.raw.header_ns,
                "pair_storage_delta_ms": (
                    pair.final.storage_ns - pair.raw.storage_ns
                )
                / 1.0e6,
                "progress_m": progress_m,
                "remaining_m": remaining_m,
                "command": command,
                "raw_point_count": len(pair.raw.message.points),
                "recorded_final_point_count": len(pair.final.message.points),
                "pre_smoothing_point_count": len(pre_smoothing),
                "pre_smoothing_corridor_correction_m": corridor_correction_m,
                "baseline_candidate_bidirectional_deviation_m": (
                    baseline_candidate_bidirectional_deviation_m
                ),
                "baseline": baseline_metrics,
                "candidate": candidate_metrics,
                "baseline_oracle": oracle,
                # Kept only while evaluating cross-variant gates, then removed
                # from the serializable report.
                "baseline_output": baseline_output,
                "candidate_output": candidate_output,
            }
        )

    pairing = {
        "raw_trajectory_count": len(records[RAW_TRAJECTORY_TOPIC]),
        "final_trajectory_count": len(records[FINAL_TRAJECTORY_TOPIC]),
        "paired_raw_count": len(pairs),
        "unpaired_raw_count": len(records[RAW_TRAJECTORY_TOPIC]) - len(pairs),
        "active_shaped_pair_count": len(rows),
        "three_point_stopped_pair_count": stopped_pairs,
        "unused_final_count": unused_final_count,
        "pairing_method": (
            "same ROS header stamp, then first unused causal bag record; "
            "timer stop refresh outputs remain unused"
        ),
        "maximum_allowed_storage_delta_ms": PAIRING_TOLERANCE_NS / 1.0e6,
        "maximum_observed_absolute_storage_delta_ms": max(
            (
                abs(pair.final.storage_ns - pair.raw.storage_ns) / 1.0e6
                for pair in pairs
            ),
            default=None,
        ),
    }
    baseline = summarize_variant(rows, "baseline")
    candidate = summarize_variant(rows, "candidate")
    oracle = summarize_oracle(rows)
    status, gates = evaluate_gates(pairing, oracle, baseline, candidate, rows)

    serialized_rows = []
    for row in rows:
        serialized = {
            key: value
            for key, value in row.items()
            if key not in {"baseline_output", "candidate_output"}
        }
        serialized_rows.append(serialized)
    return {
        "schema_version": 1,
        "analysis": (
            "town06_60kph_endpoint_tapered_c1_corridor_preflight_v1"
            if prototype_enabled
            else ANALYSIS_ID
        ),
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "status": status,
        "real_vehicle_ready": False,
        "validation_boundary": {
            "environment": "CARLA_offline_frozen_evidence",
            "live_ab_authorized": status == "PASS",
            "real_vehicle_ready": False,
            "statement": (
                "A PASS only unlocks one straight-only CARLA live A/B. It is not "
                "60 km/h qualification or real-vehicle calibration evidence."
            ),
        },
        "experiment": {
            "id": (
                "trajectory_geometry_smoothing_strength_10_to_10000_"
                "endpoint_tapered_c1_corridor"
                if prototype_enabled
                else "trajectory_geometry_smoothing_strength_10_to_10000"
            ),
            "baseline_strength": BASELINE_STRENGTH,
            "candidate_strength": CANDIDATE_STRENGTH,
            "isolated_parameter_change_count": 4 if prototype_enabled else 1,
            "fixed_route_corridor_half_width_m": 0.2,
            "fixed_turn_outward_corridor_half_width_m": 0.2,
            "route_scope": "Town06_straight_only",
            "curvature_15mps_limit_per_m": CURVATURE_LIMIT_PER_M,
            "candidate_corridor_saturation_mode": candidate_saturation_mode,
            "candidate_corridor_transition_width_m": (
                candidate_transition_width_m
            ),
            "candidate_corridor_endpoint_taper_m": candidate_endpoint_taper_m,
            "production_default_unchanged": True,
        },
        "production_pipeline": {
            "order": [
                "causal odometry -> RoutePlan.project/remaining/command_at",
                "resample raw at 0.5 m plus original stations and goal stop station",
                "hard route corridor at 0.20 m",
                "lateral stabilization (pinned gain 1.0: exact no-op)",
                "smooth_trajectory_geometry at 0.25 m with 0.10 m guard",
                "VadRouteManager._apply_velocity_profile",
                "limit_trajectory_speed_for_curvature",
                "limit_trajectory_speed_for_route_curvature",
                "limit_trajectory_speed_recovery",
                "VadRouteManager._recalculate_acceleration and output validation",
            ],
            "candidate_rejection_behavior": (
                "transactional fail-open to the common pre-smoothing geometry; any "
                "such rejection fails this preflight"
            ),
            "candidate_corridor_saturation": (
                "additive pure endpoint-tapered C1 offset helper selected only "
                "by this offline call; VadRouteManager and launch defaults do "
                "not expose or activate it"
                if prototype_enabled
                else "captured legacy production hard corridor"
            ),
            "parameters": {
                name: parameters[name] for name in PARAMETER_CONTRACT
            },
        },
        "sources": {
            "trial_dir": str(trial.resolve()),
            "immutable": source_identities,
            "topic_types": {topic: topic_types[topic] for topic in REQUIRED_TOPICS},
            "message_counts": {
                topic: len(records[topic]) for topic in REQUIRED_TOPICS
            },
            "progress_reconstruction": progress_source,
        },
        "pairing": pairing,
        "baseline_oracle": oracle,
        "variants": {
            "baseline_strength_10": baseline,
            "candidate_strength_10000": candidate,
        },
        "gates": gates,
        "limitations": [
            "The frozen vehicle motion remains the v4 motion; this is not a live closed-loop replay.",
            "The candidate is prohibited on turns and on every route containing commands other than STRAIGHT/LANEFOLLOW.",
            "Endpoint positions and an in-horizon goal-stop position are fixed, but endpoint tangent/yaw is recomputed and therefore gated separately.",
            "The earlier one-snapshot screen applied another smoother to an already strength-10 final trajectory and used a 0.25 m screen corridor; its numerical result is candidate-selection evidence only.",
            "The endpoint-tapered C1 helper is not wired into VadRouteManager, launch XML, or an execution wrapper.",
        ],
        "per_snapshot": serialized_rows,
    }


def failed_payload(
    trial: Path, reason: str, candidate_saturation_mode: str = "legacy"
) -> dict[str, Any]:
    return {
        "schema_version": 1,
        "analysis": (
            "town06_60kph_endpoint_tapered_c1_corridor_preflight_v1"
            if candidate_saturation_mode == "endpoint_tapered_c1"
            else ANALYSIS_ID
        ),
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "status": "FAILED",
        "real_vehicle_ready": False,
        "validation_boundary": {
            "environment": "CARLA_offline_frozen_evidence",
            "live_ab_authorized": False,
            "real_vehicle_ready": False,
        },
        "experiment": {
            "candidate_corridor_saturation_mode": candidate_saturation_mode,
        },
        "sources": {"trial_dir": str(trial.resolve())},
        "failure_reasons": [reason],
    }


def markdown_report(payload: Mapping[str, Any]) -> str:
    status = payload.get("status", "FAILED")
    experiment = payload.get("experiment", {})
    candidate_mode = experiment.get(
        "candidate_corridor_saturation_mode", "legacy"
    )
    lines = [
        (
            "# Town06 60 km/h endpoint-tapered C1 corridor preflight v1"
            if candidate_mode == "endpoint_tapered_c1"
            else "# Town06 60 km/h smoothing production preflight v1"
        ),
        "",
        f"- Preflight decision: **{status}**",
        "- Candidate: `trajectory_geometry_smoothing_strength 10 → 10000`",
        f"- Candidate corridor saturation: `{candidate_mode}`",
        "- Fixed corridor: `route=0.20 m`, `turn_outward=0.20 m`",
        f"- Live A/B authorized: **{str(payload.get('validation_boundary', {}).get('live_ab_authorized', False)).lower()}**",
        "- Real-vehicle ready: **false**",
        "",
    ]
    if "failure_reasons" in payload:
        lines.extend(["## Failure", ""])
        lines.extend(f"- {reason}" for reason in payload["failure_reasons"])
        lines.append("")
        return "\n".join(lines)

    if candidate_mode == "endpoint_tapered_c1":
        lines[6:6] = [
            "- C1 transition width: "
            f"`{experiment['candidate_corridor_transition_width_m']:.6g} m`",
            "- Endpoint taper distance: "
            f"`{experiment['candidate_corridor_endpoint_taper_m']:.6g} m`",
            "- Production/launch default changed: **false**",
        ]

    pairing = payload["pairing"]
    oracle = payload["baseline_oracle"]
    baseline = payload["variants"]["baseline_strength_10"]
    candidate = payload["variants"]["candidate_strength_10000"]
    midroute = payload["gates"]["midroute"]
    baseline_midroute_peak = midroute[
        "baseline_adjacent_point_snapshot_peak_curvature_per_m"
    ]
    candidate_midroute_peak = midroute[
        "candidate_adjacent_point_snapshot_peak_curvature_per_m"
    ]
    accepted_candidate_midroute_peak = midroute[
        "candidate_accepted_only_snapshot_peak_curvature_per_m"
    ]
    lines.extend(
        [
            "## Reconstruction integrity",
            "",
            f"- Raw/final pairs: `{pairing['paired_raw_count']}` / `{pairing['raw_trajectory_count']}`",
            f"- Active-shaped snapshots: `{pairing['active_shaped_pair_count']}`",
            f"- Goal/stopped snapshots: `{pairing['three_point_stopped_pair_count']}`",
            f"- Strength-10 oracle: **{oracle['status']}**, maximum XY error `{oracle['maximum_xy_error_m']:.9g} m`",
            f"- Geometry oracle: **{oracle['geometry']['status']}** across `{oracle['geometry']['snapshot_count']}` / `{oracle['snapshot_count']}` snapshots",
            f"- Post-startup scalar/time oracle: **{oracle['scalar_time_after_startup']['status']}** across `{oracle['scalar_time_after_startup']['snapshot_count']}` snapshots",
            f"- Startup context excluded from scalar/time equality only: `{oracle['startup_context_missing']['excluded_snapshot_count']}` snapshots (`{oracle['startup_context_missing']['excluded_snapshot_indexes'][0]}–{oracle['startup_context_missing']['excluded_snapshot_indexes'][-1]}`)",
            f"- Oracle bounds: XY `≤ {oracle['tolerances']['maximum_xy_error_m']:.6g} m`, yaw `≤ {oracle['tolerances']['maximum_yaw_error_rad']:.6g} rad`; point count is exact for all snapshots and time is exact after startup",
            "",
            "## Continuity contract",
            "",
            "- Scalar radial saturation is C1 across its transition band.",
            "- The spatial correction uses a quintic product whose value and station derivative are zero at each fixed endpoint/stop anchor.",
            "- A snapshot is rejected if preserving that anchor tangent would leave the 0.20 m corridor; rejected snapshots are never counted as an accepted C1 candidate.",
            "",
            "## All-snapshot outcome",
            "",
            *(
                [
                    f"> **Fail-open metrics warning:** `{candidate['smoothing_rejection_count']}` rejected snapshots below use the route manager's common pre-smoothing fallback. Aggregate candidate curvature/cap values are diagnostic mixtures, not authorization evidence.",
                    "",
                ]
                if candidate["smoothing_rejection_count"]
                else []
            ),
            "| Metric | v4 strength 10 | candidate strength 10000 |",
            "|---|---:|---:|",
            f"| Moving-midroute adjacent-point peak curvature p95 (1/m) | {baseline_midroute_peak['p95']:.9f} | {candidate_midroute_peak['p95']:.9f} |",
            f"| Moving-midroute adjacent-point peak curvature max (1/m) | {baseline_midroute_peak['maximum']:.9f} | {candidate_midroute_peak['maximum']:.9f} |",
            f"| Moving-midroute curvature violations (> {CURVATURE_LIMIT_PER_M:.9f} 1/m) | n/a | {midroute['candidate_curvature_limit_violation_count']} / {midroute['eligible_snapshot_count']} |",
            f"| Accepted-only moving-midroute peak curvature p95/max (1/m) | n/a | {accepted_candidate_midroute_peak['p95']:.9f} / {accepted_candidate_midroute_peak['maximum']:.9f} |",
            f"| Accepted-only curvature violations | n/a | {midroute['candidate_accepted_curvature_limit_violation_count']} / {midroute['candidate_accepted_snapshot_count']} (rejected midroute: {midroute['candidate_rejected_snapshot_count']}) |",
            f"| Smoothing deviation max (m) | {baseline['smoothing_deviation_max_m']['maximum']:.6f} | {candidate['smoothing_deviation_max_m']['maximum'] if candidate['smoothing_deviation_max_m']['maximum'] is not None else 'n/a'} |",
            f"| Strength-10 ↔ 10000 bidirectional deviation max (m) | n/a | {payload['gates']['checks']['candidate_selection_deviation_envelope']['actual']:.6f} |",
            f"| Route CTE max (m) | {baseline['route_cte_max_m']['maximum']:.6f} | {candidate['route_cte_max_m']['maximum']:.6f} |",
            f"| Guarded smoothing rejections | {baseline['smoothing_rejection_count']} | {candidate['smoothing_rejection_count']} |",
            f"| Solve time p95 / max (ms) | {baseline['solve_time_ms']['p95']:.3f} / {baseline['solve_time_ms']['maximum']:.3f} | {candidate['solve_time_ms']['p95']:.3f} / {candidate['solve_time_ms']['maximum']:.3f} |",
            "",
            "## Gates",
            "",
            "| Gate | Severity | Status | Requirement |",
            "|---|---|---|---|",
        ]
    )
    for name, gate in payload["gates"]["checks"].items():
        lines.append(
            f"| `{name}` | {gate['severity']} | {gate['status']} | {gate['requirement']} |"
        )
    failed_checks = [
        (name, gate)
        for name, gate in payload["gates"]["checks"].items()
        if gate["status"] == "FAIL"
    ]
    lines.extend(["", "## Failed gate measurements", ""])
    if failed_checks:
        lines.extend(
            f"- `{name}`: `{json.dumps(gate['actual'], sort_keys=True)}`"
            for name, gate in failed_checks
        )
    else:
        lines.append("- None")

    immutable = payload["sources"]["immutable"]
    lines.extend(["", "## Frozen input and code identity", ""])
    for name in (
        "source_route.json",
        "aligned_route.json",
        "bag/metadata.yaml",
        "bag/bag_0.db3",
        "vad_route_manager.params.yaml",
        "trajectory_code_provenance/vad_route_logic.py",
        "trajectory_code_provenance/vad_route_manager.py",
        "preflight_script",
    ):
        item = immutable[name]
        lines.append(f"- `{name}` SHA-256: `{item['sha256']}`")
    current_logic = immutable[
        "current:autoware_e2e_vad_launch/scripts/vad_route_logic.py"
    ]
    lines.append(
        "- current prototype `vad_route_logic.py` SHA-256/status: "
        f"`{current_logic['sha256']}` / `{current_logic['status']}`"
    )
    lines.extend(
        [
            "",
            "## Boundary",
            "",
            payload["validation_boundary"]["statement"],
            "",
            "The previous one-snapshot λ=10000 result is not treated as production-equivalent: it smoothed an already strength-10 output and allowed a 0.25 m screening corridor. This report instead starts from every recorded raw trajectory and uses the pinned production order and 0.20 m corridor.",
            "",
        ]
    )
    return "\n".join(lines)


def render_png(payload: Mapping[str, Any], path: Path) -> None:
    if not payload.get("per_snapshot"):
        fig, axis = plt.subplots(figsize=(11, 4.5), constrained_layout=True)
        axis.axis("off")
        axis.text(
            0.5,
            0.62,
            f"Production smoothing preflight: {payload.get('status', 'FAILED')}",
            ha="center",
            va="center",
            fontsize=20,
            weight="bold",
        )
        axis.text(
            0.5,
            0.36,
            "\n".join(payload.get("failure_reasons", ["No snapshot metrics"])),
            ha="center",
            va="center",
            fontsize=10,
            wrap=True,
        )
        fig.savefig(path, dpi=160, facecolor="white")
        plt.close(fig)
        return

    rows = payload["per_snapshot"]
    candidate_mode = payload.get("experiment", {}).get(
        "candidate_corridor_saturation_mode", "legacy"
    )
    candidate_label = (
        "strength 10000 + endpoint-tapered C1"
        if candidate_mode == "endpoint_tapered_c1"
        else "strength 10000"
    )
    indexes = [row["snapshot_index"] for row in rows]
    baseline_peak = [row["baseline"]["curvature_peak_per_m"] for row in rows]
    candidate_peak = [row["candidate"]["curvature_peak_per_m"] for row in rows]
    candidate_deviation = [
        row["candidate"]["smoothing_deviation_max_m"]
        if row["candidate"]["smoothing_deviation_max_m"] is not None
        else math.nan
        for row in rows
    ]
    candidate_cte = [row["candidate"]["route_cte_max_m"] for row in rows]
    candidate_latency = [row["candidate"]["solve_time_ms"] for row in rows]

    fig, axes = plt.subplots(2, 2, figsize=(14, 8), constrained_layout=True)
    axes[0, 0].plot(indexes, baseline_peak, label="strength 10", alpha=0.75)
    axes[0, 0].plot(indexes, candidate_peak, label=candidate_label, alpha=0.8)
    axes[0, 0].axhline(CURVATURE_LIMIT_PER_M, color="black", linestyle="--")
    axes[0, 0].set_ylabel("snapshot peak |curvature| [1/m]")
    axes[0, 0].legend()
    axes[0, 0].set_yscale("log")

    axes[0, 1].plot(indexes, candidate_deviation, color="#7b4ab5")
    axes[0, 1].axhline(0.05, color="#d18f00", linestyle="--", label="selection 0.05 m")
    axes[0, 1].axhline(0.10, color="#b22222", linestyle=":", label="hard guard 0.10 m")
    axes[0, 1].set_ylabel("candidate smoothing deviation [m]")
    axes[0, 1].legend()

    axes[1, 0].plot(indexes, candidate_cte, color="#14837d")
    axes[1, 0].axhline(0.20, color="black", linestyle="--")
    axes[1, 0].set_ylabel("candidate route CTE max [m]")

    axes[1, 1].plot(indexes, candidate_latency, color="#3b6ba5")
    axes[1, 1].axhline(SOLVE_P95_LIMIT_MS, color="#d18f00", linestyle="--")
    axes[1, 1].axhline(SOLVE_MAX_LIMIT_MS, color="#b22222", linestyle=":")
    axes[1, 1].set_ylabel("candidate smoothing solve [ms]")

    for axis in axes.flat:
        axis.set_xlabel("paired raw snapshot index")
        axis.grid(alpha=0.25)
        axis.spines[["top", "right"]].set_visible(False)
    fig.suptitle(
        "Town06 straight 60 km/h | "
        + (
            "endpoint-tapered C1 corridor preflight | "
            if candidate_mode == "endpoint_tapered_c1"
            else "production-equivalent smoothing preflight | "
        )
        + f"{payload['status']}",
        fontsize=14,
        weight="bold",
    )
    fig.savefig(path, dpi=170, facecolor="white")
    plt.close(fig)


def atomic_write_text(path: Path, content: str) -> None:
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".staged", dir=path.parent
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def write_report(output: Path, payload: Mapping[str, Any]) -> None:
    atomic_write_text(
        output / "summary.json",
        json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n",
    )
    atomic_write_text(output / "README.md", markdown_report(payload))
    render_png(payload, output / "comparison.png")
    checksum_lines = [
        f"{sha256_file(output / name)}  {name}"
        for name in ("summary.json", "README.md", "comparison.png")
    ]
    atomic_write_text(output / "SHA256SUMS", "\n".join(checksum_lines) + "\n")


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trial-dir", type=Path, default=CANONICAL_TRIAL)
    parser.add_argument("--output-dir", type=Path, default=CANONICAL_OUTPUT)
    parser.add_argument(
        "--candidate-corridor-saturation",
        choices=("legacy", "endpoint_tapered_c1"),
        default="legacy",
    )
    parser.add_argument("--candidate-transition-width-m", type=float, default=0.0)
    parser.add_argument("--candidate-endpoint-taper-m", type=float, default=0.0)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    trial = args.trial_dir.expanduser().resolve()
    output = args.output_dir.expanduser().resolve()
    if output.exists() and (output.is_symlink() or any(output.iterdir())):
        raise SystemExit(f"output directory must not already contain evidence: {output}")
    output.mkdir(parents=True, exist_ok=True)
    try:
        payload = analyze(
            trial,
            candidate_saturation_mode=args.candidate_corridor_saturation,
            candidate_transition_width_m=args.candidate_transition_width_m,
            candidate_endpoint_taper_m=args.candidate_endpoint_taper_m,
        )
    except (PreflightError, OSError, ValueError, KeyError, TypeError) as error:
        payload = failed_payload(
            trial, str(error), args.candidate_corridor_saturation
        )
    write_report(output, payload)
    print(
        f"SMOOTHING_PRODUCTION_PREFLIGHT {payload['status']} "
        f"real_vehicle_ready=false {output / 'summary.json'}",
        flush=True,
    )
    return 0 if payload["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
