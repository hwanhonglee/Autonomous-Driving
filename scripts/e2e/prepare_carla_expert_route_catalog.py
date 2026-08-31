#!/usr/bin/env python3
"""Build deterministic expert routes on one already-running CARLA server."""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import importlib
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import time
from typing import Any, Mapping, Sequence

import networkx as nx

try:
    from inventory_carla_training_maps import DEFAULT_MANIFEST, ManifestError, load_manifest
except ModuleNotFoundError:
    from scripts.e2e.inventory_carla_training_maps import (
        DEFAULT_MANIFEST,
        ManifestError,
        load_manifest,
    )


SCENARIOS = ("lane_follow", "straight", "left", "right")
RECOVERABLE_TRACE_EXCEPTIONS = (
    KeyError,
    RuntimeError,
    nx.NetworkXNoPath,
    nx.NodeNotFound,
)
MAX_TRACE_ERROR_SAMPLES = 10
TURN_MANEUVER_OPTIONS = frozenset(
    {
        "LEFT",
        "RIGHT",
        "STRAIGHT",
        "CHANGELANELEFT",
        "CHANGELANERIGHT",
    }
)
TURN_GEOMETRY_ARGUMENTS = (
    "minimum_turn_arc_length_m",
    "maximum_turn_arc_length_m",
    "minimum_turn_heading_change_deg",
    "maximum_turn_heading_change_deg",
    "maximum_turn_heading_excess_deg",
    "turn_alignment_heading_margin_deg",
    "maximum_turn_command_lead_m",
    "maximum_turn_command_tail_m",
    "maximum_turn_p95_curvature_per_m",
)


class CatalogError(RuntimeError):
    pass


def parse_integer_list(text: str) -> tuple[int, ...]:
    try:
        values = tuple(int(item.strip()) for item in text.split(",") if item.strip())
    except ValueError as error:
        raise CatalogError("seeds must be comma-separated integers") from error
    if not values or len(set(values)) != len(values):
        raise CatalogError("seeds must be a non-empty unique list")
    if any(value < 0 for value in values):
        raise CatalogError("seeds must be non-negative")
    return values


def parse_excluded_spawn_indices(text: str) -> tuple[int, ...]:
    if not text.strip():
        return ()
    items = text.split(",")
    if any(not item.strip() for item in items):
        raise CatalogError("excluded spawn indices must be comma-separated integers")
    try:
        values = tuple(int(item.strip()) for item in items)
    except ValueError as error:
        raise CatalogError(
            "excluded spawn indices must be comma-separated integers"
        ) from error
    if len(set(values)) != len(values):
        raise CatalogError("excluded spawn indices must be unique")
    if any(value < 0 for value in values):
        raise CatalogError("excluded spawn indices must be non-negative")
    return values


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--map-id", required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--host", default=os.environ.get("CARLA_HOST", "127.0.0.1"))
    parser.add_argument(
        "--port", type=int, default=int(os.environ.get("CARLA_PORT", "2000"))
    )
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument(
        "--map-load-settle-sec",
        type=float,
        default=10.0,
        help=(
            "pre-read delay for the externally started map; also applied after an "
            "explicitly allowed map load"
        ),
    )
    parser.add_argument(
        "--allow-map-load",
        action="store_true",
        help=(
            "UNSAFE opt-in: permit client.load_world when the current map mismatches; "
            "some packaged/custom CARLA maps crash during client map loading"
        ),
    )
    parser.add_argument(
        "--active-server-profile",
        help="explicit profile id of the already-running external CARLA server",
    )
    parser.add_argument("--weather", default="ClearNoon")
    parser.add_argument("--seeds", default="0,1,2")
    parser.add_argument("--pairs-per-seed", type=int, default=1)
    parser.add_argument("--min-distance", type=float, default=20.0)
    parser.add_argument("--max-distance", type=float, default=120.0)
    parser.add_argument("--preferred-distance", type=float, default=60.0)
    parser.add_argument("--sampling-resolution", type=float, default=1.0)
    parser.add_argument(
        "--max-endpoint-offset",
        type=float,
        default=2.0,
        help="maximum planar XY offset between a spawn and its route endpoint",
    )
    parser.add_argument(
        "--initial-approach-distance-m",
        type=float,
        default=0.0,
        help=(
            "optional route-start geometry preflight distance; zero disables the "
            "preflight"
        ),
    )
    parser.add_argument(
        "--maximum-initial-lateral-deviation-m",
        type=float,
        help=(
            "maximum route-centerline deviation from the initial tangent inside "
            "--initial-approach-distance-m"
        ),
    )
    parser.add_argument(
        "--maximum-initial-heading-change-deg",
        type=float,
        help=(
            "maximum absolute route heading change inside "
            "--initial-approach-distance-m"
        ),
    )
    parser.add_argument("--minimum-turn-arc-length-m", type=float)
    parser.add_argument("--maximum-turn-arc-length-m", type=float)
    parser.add_argument("--minimum-turn-heading-change-deg", type=float)
    parser.add_argument("--maximum-turn-heading-change-deg", type=float)
    parser.add_argument("--maximum-turn-heading-excess-deg", type=float)
    parser.add_argument("--turn-alignment-heading-margin-deg", type=float)
    parser.add_argument("--maximum-turn-command-lead-m", type=float)
    parser.add_argument("--maximum-turn-command-tail-m", type=float)
    parser.add_argument("--maximum-turn-p95-curvature-per-m", type=float)
    parser.add_argument("--max-traces", type=int, default=5000)
    parser.add_argument(
        "--exclude-spawn-indices",
        default="",
        help="comma-separated CARLA spawn indices excluded as starts and goals",
    )
    args = parser.parse_args(argv)
    try:
        args.seeds = parse_integer_list(args.seeds)
        args.exclude_spawn_indices = parse_excluded_spawn_indices(
            args.exclude_spawn_indices
        )
    except CatalogError as error:
        parser.error(str(error))
    if not 0 < args.port <= 65535 or args.timeout <= 0.0:
        parser.error("port and timeout must be positive")
    if not math.isfinite(args.map_load_settle_sec) or args.map_load_settle_sec < 0.0:
        parser.error("map-load-settle-sec must be finite and non-negative")
    if args.pairs_per_seed <= 0 or args.max_traces <= 0:
        parser.error("pairs-per-seed and max-traces must be positive")
    if not 0.0 < args.min_distance <= args.max_distance:
        parser.error("distance range is invalid")
    if args.preferred_distance <= 0.0 or args.sampling_resolution <= 0.0:
        parser.error("preferred distance and sampling resolution must be positive")
    if not math.isfinite(args.max_endpoint_offset) or args.max_endpoint_offset <= 0.0:
        parser.error("max-endpoint-offset must be positive and finite")
    try:
        initial_approach_contract(args)
        turn_geometry_contract(args)
    except CatalogError as error:
        parser.error(str(error))
    return args


def atomic_write_json(path: Path, payload: Mapping[str, Any]) -> Path:
    path = path.expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, indent=2, sort_keys=False, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise
    return path


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while block := stream.read(1024 * 1024):
            digest.update(block)
    return digest.hexdigest()


def _load_runtime_modules():
    try:
        carla = importlib.import_module("carla")
        helper = importlib.import_module("prepare_carla_route")
    except ModuleNotFoundError as error:
        raise CatalogError(
            "CARLA Python API and agents are required; source scripts/e2e/env.sh or "
            "prepend the selected profile's Python API to PYTHONPATH"
        ) from error
    return carla, helper


def _map_by_id(manifest: Mapping[str, Any], map_id: str) -> Mapping[str, Any]:
    for entry in manifest["maps"]:
        if entry["id"] == map_id:
            return entry
    raise CatalogError(f"unknown map id: {map_id}")


def validate_active_server_profile(
    manifest: Mapping[str, Any],
    map_entry: Mapping[str, Any],
    active_server_profile: str | None,
) -> None:
    profiles = manifest["server_profiles"]
    if active_server_profile is not None and active_server_profile not in profiles:
        raise CatalogError(f"unknown active server profile: {active_server_profile}")
    required_profile = map_entry.get("server_profile")
    if active_server_profile is not None and active_server_profile != required_profile:
        raise CatalogError(
            f"map {map_entry['id']} requires server profile {required_profile!r}, "
            f"not declared active profile {active_server_profile!r}"
        )
    if (
        map_entry.get("status") == "source_editor_required"
        and active_server_profile != required_profile
    ):
        raise CatalogError(
            f"map {map_entry['id']} requires explicit --active-server-profile "
            f"{required_profile}"
        )


def _location_distance(first: Any, second: Any) -> float:
    return float(first.location.distance(second.location))


def initial_approach_contract(args: argparse.Namespace) -> dict[str, Any]:
    """Return and validate the optional route-start geometry contract."""
    distance_m = float(getattr(args, "initial_approach_distance_m", 0.0))
    maximum_lateral = getattr(args, "maximum_initial_lateral_deviation_m", None)
    maximum_heading = getattr(args, "maximum_initial_heading_change_deg", None)
    if not math.isfinite(distance_m) or distance_m < 0.0:
        raise CatalogError("initial approach distance must be finite and non-negative")
    if distance_m == 0.0:
        if maximum_lateral is not None or maximum_heading is not None:
            raise CatalogError(
                "initial approach limits require a positive "
                "--initial-approach-distance-m"
            )
        return {
            "enabled": False,
            "distance_m": 0.0,
            "maximum_lateral_deviation_m": None,
            "maximum_heading_change_deg": None,
        }
    if maximum_lateral is None or maximum_heading is None:
        raise CatalogError(
            "a positive initial approach distance requires both lateral-deviation "
            "and heading-change limits"
        )
    maximum_lateral = float(maximum_lateral)
    maximum_heading = float(maximum_heading)
    if not math.isfinite(maximum_lateral) or maximum_lateral <= 0.0:
        raise CatalogError(
            "maximum initial lateral deviation must be positive and finite"
        )
    if (
        not math.isfinite(maximum_heading)
        or maximum_heading <= 0.0
        or maximum_heading > 180.0
    ):
        raise CatalogError(
            "maximum initial heading change must be finite and in (0, 180] degrees"
        )
    return {
        "enabled": True,
        "distance_m": distance_m,
        "maximum_lateral_deviation_m": maximum_lateral,
        "maximum_heading_change_deg": maximum_heading,
    }


def _normalize_angle_degrees(value: float) -> float:
    return (value + 180.0) % 360.0 - 180.0


def turn_geometry_contract(args: argparse.Namespace) -> dict[str, Any]:
    """Return the optional isolated-turn geometry contract."""
    values = {name: getattr(args, name, None) for name in TURN_GEOMETRY_ARGUMENTS}
    if all(value is None for value in values.values()):
        return {"enabled": False}
    missing = [name for name, value in values.items() if value is None]
    if missing:
        raise CatalogError(
            "turn geometry preflight requires every limit; missing "
            + ", ".join(sorted(missing))
        )
    numeric = {name: float(value) for name, value in values.items()}
    if any(not math.isfinite(value) or value <= 0.0 for value in numeric.values()):
        raise CatalogError("turn geometry limits must be positive and finite")
    if numeric["minimum_turn_arc_length_m"] >= numeric[
        "maximum_turn_arc_length_m"
    ]:
        raise CatalogError("minimum turn arc length must be below its maximum")
    if numeric["minimum_turn_heading_change_deg"] >= numeric[
        "maximum_turn_heading_change_deg"
    ]:
        raise CatalogError("minimum turn heading change must be below its maximum")
    if numeric["maximum_turn_heading_change_deg"] > 180.0:
        raise CatalogError("maximum turn heading change cannot exceed 180 degrees")
    if numeric["turn_alignment_heading_margin_deg"] >= numeric[
        "minimum_turn_heading_change_deg"
    ]:
        raise CatalogError(
            "turn alignment heading margin must be below the minimum heading change"
        )
    return {
        "enabled": True,
        "require_single_directional_block": True,
        "forbid_additional_maneuvers": True,
        "minimum_arc_length_m": numeric["minimum_turn_arc_length_m"],
        "maximum_arc_length_m": numeric["maximum_turn_arc_length_m"],
        "minimum_heading_change_deg": numeric[
            "minimum_turn_heading_change_deg"
        ],
        "maximum_heading_change_deg": numeric[
            "maximum_turn_heading_change_deg"
        ],
        "maximum_heading_excess_deg": numeric["maximum_turn_heading_excess_deg"],
        "alignment_heading_margin_deg": numeric[
            "turn_alignment_heading_margin_deg"
        ],
        "maximum_command_lead_m": numeric["maximum_turn_command_lead_m"],
        "maximum_command_tail_m": numeric["maximum_turn_command_tail_m"],
        "curvature_percentile": 95.0,
        "maximum_p95_abs_curvature_per_m": numeric[
            "maximum_turn_p95_curvature_per_m"
        ],
    }


def _road_option_name(option: Any) -> str:
    return str(getattr(option, "name", option))


def _linear_percentile(values: Sequence[float], percentile: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return 0.0
    position = (len(ordered) - 1) * percentile / 100.0
    lower = int(math.floor(position))
    upper = min(lower + 1, len(ordered) - 1)
    fraction = position - lower
    return ordered[lower] + fraction * (ordered[upper] - ordered[lower])


def _first_progress_crossing(
    samples: Sequence[tuple[float, float]], threshold: float
) -> float | None:
    for (start_distance, start_value), (end_distance, end_value) in zip(
        samples, samples[1:]
    ):
        if start_value >= threshold:
            return start_distance
        if end_value < threshold:
            continue
        if end_value <= start_value:
            return end_distance
        ratio = (threshold - start_value) / (end_value - start_value)
        return start_distance + ratio * (end_distance - start_distance)
    if samples and samples[-1][1] >= threshold:
        return samples[-1][0]
    return None


def _turn_block_geometry(
    route: Sequence[Any], start_index: int, end_index: int
) -> dict[str, Any]:
    distances = [0.0]
    for index in range(1, len(route)):
        distances.append(
            distances[-1]
            + float(
                route[index - 1][0].transform.location.distance(
                    route[index][0].transform.location
                )
            )
        )
    support_start = max(0, start_index - 1)
    support_end = min(len(route) - 1, end_index + 1)
    block_start = (
        0.5 * (distances[start_index - 1] + distances[start_index])
        if start_index
        else distances[start_index]
    )
    block_end = (
        0.5 * (distances[end_index] + distances[end_index + 1])
        if end_index + 1 < len(route)
        else distances[end_index]
    )
    yaws = [
        float(route[index][0].transform.rotation.yaw)
        for index in range(support_start, support_end + 1)
    ]
    unwrapped = [yaws[0]]
    for yaw in yaws[1:]:
        unwrapped.append(
            unwrapped[-1] + _normalize_angle_degrees(yaw - unwrapped[-1])
        )
    net_heading = unwrapped[-1] - unwrapped[0]
    absolute_net_heading = abs(net_heading)
    cumulative_heading = sum(
        abs(unwrapped[index] - unwrapped[index - 1])
        for index in range(1, len(unwrapped))
    )
    direction = 1.0 if net_heading >= 0.0 else -1.0
    progress = [
        (
            distances[support_start + index],
            max(0.0, direction * (yaw - unwrapped[0])),
        )
        for index, yaw in enumerate(unwrapped)
    ]
    curvatures = []
    for index in range(support_start + 1, support_end + 1):
        segment_length = distances[index] - distances[index - 1]
        if segment_length <= 1.0e-6:
            continue
        yaw_delta = _normalize_angle_degrees(
            float(route[index][0].transform.rotation.yaw)
            - float(route[index - 1][0].transform.rotation.yaw)
        )
        curvatures.append(abs(math.radians(yaw_delta)) / segment_length)
    arc_length = block_end - block_start
    return {
        "start_index": start_index,
        "end_index": end_index,
        "command_arc_length_m": arc_length,
        "absolute_net_heading_change_deg": absolute_net_heading,
        "cumulative_absolute_heading_change_deg": cumulative_heading,
        "heading_excess_deg": max(0.0, cumulative_heading - absolute_net_heading),
        "mean_absolute_curvature_per_m": (
            math.radians(cumulative_heading) / arc_length
            if arc_length > 0.0
            else math.inf
        ),
        "p95_absolute_curvature_per_m": _linear_percentile(curvatures, 95.0),
        "block_start_distance_m": block_start,
        "block_end_distance_m": block_end,
        "_heading_progress": progress,
    }


def analyze_turn_geometry(
    route: Sequence[Any], scenario: str, contract: Mapping[str, Any]
) -> dict[str, Any]:
    """Verify that a turn trial contains one compact, command-aligned turn."""
    if not contract.get("enabled"):
        raise CatalogError("turn geometry analysis requires an enabled contract")
    if scenario not in ("left", "right"):
        raise CatalogError("turn geometry analysis requires left or right scenario")
    if len(route) < 2:
        raise CatalogError("turn geometry analysis requires at least two points")

    target = scenario.upper()
    option_names = [_road_option_name(option) for _waypoint, option in route]
    indices = [index for index, option in enumerate(option_names) if option == target]
    blocks: list[list[int]] = []
    for index in indices:
        if not blocks or index != blocks[-1][-1] + 1:
            blocks.append([index])
        else:
            blocks[-1].append(index)
    additional = sorted((set(option_names) & TURN_MANEUVER_OPTIONS) - {target})
    reasons = []
    if len(blocks) != 1:
        reasons.append(
            f"expected exactly one contiguous {target} block, found {len(blocks)}"
        )
    if additional:
        reasons.append("additional maneuver commands present: " + ", ".join(additional))

    block_metrics = [
        _turn_block_geometry(route, block[0], block[-1]) for block in blocks
    ]
    selected = block_metrics[0] if len(block_metrics) == 1 else None
    if selected is not None:
        margin = float(contract["alignment_heading_margin_deg"])
        net_heading = float(selected["absolute_net_heading_change_deg"])
        lead_crossing = _first_progress_crossing(
            selected["_heading_progress"], margin
        )
        tail_crossing = _first_progress_crossing(
            selected["_heading_progress"], max(0.0, net_heading - margin)
        )
        command_lead = (
            math.inf
            if lead_crossing is None
            else max(
                0.0, lead_crossing - float(selected["block_start_distance_m"])
            )
        )
        command_tail = (
            math.inf
            if tail_crossing is None
            else max(0.0, float(selected["block_end_distance_m"]) - tail_crossing)
        )
        selected["command_lead_distance_m"] = command_lead
        selected["command_tail_distance_m"] = command_tail

        checks = (
            (
                float(selected["command_arc_length_m"])
                < float(contract["minimum_arc_length_m"]),
                "turn command arc length is below the minimum",
            ),
            (
                float(selected["command_arc_length_m"])
                > float(contract["maximum_arc_length_m"]),
                "turn command arc length exceeds the maximum",
            ),
            (
                net_heading < float(contract["minimum_heading_change_deg"]),
                "turn heading change is below the minimum",
            ),
            (
                net_heading > float(contract["maximum_heading_change_deg"]),
                "turn heading change exceeds the maximum",
            ),
            (
                float(selected["heading_excess_deg"])
                > float(contract["maximum_heading_excess_deg"]),
                "turn heading excess exceeds the maximum",
            ),
            (
                command_lead > float(contract["maximum_command_lead_m"]),
                "turn command lead distance exceeds the maximum",
            ),
            (
                command_tail > float(contract["maximum_command_tail_m"]),
                "turn command tail distance exceeds the maximum",
            ),
            (
                float(selected["p95_absolute_curvature_per_m"])
                > float(contract["maximum_p95_abs_curvature_per_m"]),
                "turn p95 absolute curvature exceeds the maximum",
            ),
        )
        reasons.extend(message for failed, message in checks if failed)

    for metrics in block_metrics:
        metrics.pop("_heading_progress", None)
    return {
        "status": "FAIL" if reasons else "PASS",
        "scenario": scenario,
        "directional_command": target,
        "directional_block_count": len(blocks),
        "additional_maneuver_commands": additional,
        "selected_block": selected,
        "directional_blocks": block_metrics,
        "limits": dict(contract),
        "failure_reasons": reasons,
    }


def analyze_initial_approach(
    route: Sequence[Any], contract: Mapping[str, Any]
) -> dict[str, Any]:
    """Measure route-start curvature without relying on RoadOption labels.

    CARLA marks non-junction bends as ``LANEFOLLOW``.  A route can therefore
    satisfy a maneuver-command contract while presenting an immediate sharp
    bend under command 3.  This preflight measures the actual centerline against
    its starting tangent, including an interpolated sample at the exact distance
    boundary.
    """
    if not contract.get("enabled"):
        raise CatalogError("initial approach analysis requires an enabled contract")
    if len(route) < 2:
        raise CatalogError("initial approach analysis requires at least two points")

    distance_limit = float(contract["distance_m"])
    first_transform = route[0][0].transform
    first_location = first_transform.location
    first_yaw_deg = float(first_transform.rotation.yaw)
    first_yaw_rad = math.radians(first_yaw_deg)
    normal_x = -math.sin(first_yaw_rad)
    normal_y = math.cos(first_yaw_rad)
    maximum_lateral = 0.0
    maximum_heading = 0.0
    covered_distance = 0.0

    def measure(x: float, y: float, yaw_deg: float) -> None:
        nonlocal maximum_lateral, maximum_heading
        dx = x - float(first_location.x)
        dy = y - float(first_location.y)
        maximum_lateral = max(maximum_lateral, abs(dx * normal_x + dy * normal_y))
        maximum_heading = max(
            maximum_heading,
            abs(_normalize_angle_degrees(yaw_deg - first_yaw_deg)),
        )

    previous = first_transform
    for waypoint, _option in route[1:]:
        current = waypoint.transform
        segment_length = float(previous.location.distance(current.location))
        if segment_length <= 1.0e-9:
            previous = current
            continue
        remaining = distance_limit - covered_distance
        if segment_length >= remaining:
            ratio = max(0.0, min(1.0, remaining / segment_length))
            yaw_delta = _normalize_angle_degrees(
                float(current.rotation.yaw) - float(previous.rotation.yaw)
            )
            measure(
                float(previous.location.x)
                + ratio * (float(current.location.x) - float(previous.location.x)),
                float(previous.location.y)
                + ratio * (float(current.location.y) - float(previous.location.y)),
                float(previous.rotation.yaw) + ratio * yaw_delta,
            )
            covered_distance = distance_limit
            break
        covered_distance += segment_length
        measure(
            float(current.location.x),
            float(current.location.y),
            float(current.rotation.yaw),
        )
        previous = current

    reasons = []
    if covered_distance < distance_limit - 1.0e-6:
        reasons.append(
            f"route covers only {covered_distance:.3f} m of the required "
            f"{distance_limit:.3f} m approach"
        )
    if maximum_lateral > float(contract["maximum_lateral_deviation_m"]):
        reasons.append(
            f"initial lateral deviation {maximum_lateral:.3f} m exceeds "
            f"{float(contract['maximum_lateral_deviation_m']):.3f} m"
        )
    if maximum_heading > float(contract["maximum_heading_change_deg"]):
        reasons.append(
            f"initial heading change {maximum_heading:.3f} deg exceeds "
            f"{float(contract['maximum_heading_change_deg']):.3f} deg"
        )
    return {
        "status": "FAIL" if reasons else "PASS",
        "distance_m": distance_limit,
        "covered_distance_m": covered_distance,
        "maximum_lateral_deviation_m": maximum_lateral,
        "maximum_heading_change_deg": maximum_heading,
        "limits": {
            "maximum_lateral_deviation_m": float(
                contract["maximum_lateral_deviation_m"]
            ),
            "maximum_heading_change_deg": float(
                contract["maximum_heading_change_deg"]
            ),
        },
        "failure_reasons": reasons,
    }


def deterministic_pairs(
    spawn_points: Sequence[Any],
    seed: int,
    minimum_distance: float,
    maximum_distance: float,
    preferred_distance: float,
    sampling_resolution: float,
    excluded_indices: Sequence[int] = (),
) -> list[tuple[int, int]]:
    excluded = frozenset(excluded_indices)
    ranked = []
    for start_index, start in enumerate(spawn_points):
        if start_index in excluded:
            continue
        for goal_index, goal in enumerate(spawn_points):
            if start_index == goal_index or goal_index in excluded:
                continue
            distance = _location_distance(start, goal)
            if not minimum_distance * 0.6 <= distance <= maximum_distance * 1.2:
                continue
            bucket = int(abs(distance - preferred_distance) / sampling_resolution)
            token = f"{seed}:{start_index}:{goal_index}".encode("ascii")
            tie_break = hashlib.sha256(token).digest()
            ranked.append((bucket, tie_break, start_index, goal_index))
    ranked.sort()
    return [(start, goal) for _, _, start, goal in ranked]


def validate_excluded_spawn_indices(
    excluded_indices: Sequence[int], spawn_point_count: int
) -> tuple[int, ...]:
    excluded = tuple(excluded_indices)
    if len(set(excluded)) != len(excluded):
        raise CatalogError("excluded spawn indices must be unique")
    if any(index < 0 for index in excluded):
        raise CatalogError("excluded spawn indices must be non-negative")
    out_of_range = tuple(index for index in excluded if index >= spawn_point_count)
    if out_of_range:
        raise CatalogError(
            "excluded spawn indices are out of range for "
            f"{spawn_point_count} spawn points: {list(out_of_range)}"
        )
    if spawn_point_count - len(excluded) < 2:
        raise CatalogError(
            "exclude-spawn-indices leaves fewer than two start/goal candidates"
        )
    return excluded


def _reset_planner(planner: Any, helper: Any) -> None:
    if hasattr(planner, "_intersection_end_node"):
        planner._intersection_end_node = -1
    if hasattr(planner, "_previous_decision") and hasattr(helper, "RoadOption"):
        planner._previous_decision = helper.RoadOption.VOID


def trace_route_candidate(planner: Any, start: Any, goal: Any) -> tuple[Any, Exception | None]:
    """Return one route or a recoverable pair-specific planner error."""
    try:
        return planner.trace_route(start, goal), None
    except RECOVERABLE_TRACE_EXCEPTIONS as error:
        return None, error


def _trace_error_sample(
    start_index: int, goal_index: int, error: Exception
) -> dict[str, Any]:
    return {
        "start_spawn_index": start_index,
        "goal_spawn_index": goal_index,
        "error_type": type(error).__name__,
        "message": str(error)[:240],
    }


def _route_payload(
    helper: Any,
    map_entry: Mapping[str, Any],
    weather: str,
    scenario: str,
    sampling_resolution: float,
    start_index: int,
    goal_index: int,
    spawn_points: Sequence[Any],
    route: Sequence[Any],
    initial_approach: Mapping[str, Any] | None = None,
    turn_geometry: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    start_transform = spawn_points[start_index]
    goal_transform = spawn_points[goal_index]
    route_points = helper.serialize_route(route, goal_transform)
    option_counts: dict[str, int] = {}
    for point in route_points:
        option = point["road_option"]
        option_counts[option] = option_counts.get(option, 0) + 1
    payload = {
        "schema_version": 1,
        "coordinate_reference": "base_link",
        "spawn_point_reference": "base_link",
        "town": map_entry["canonical_name"],
        "weather": weather,
        "scenario": scenario,
        "sampling_resolution_m": sampling_resolution,
        "route_length_m": route_points[-1]["distance_m"],
        "start_spawn_index": start_index,
        "goal_spawn_index": goal_index,
        "start_carla_transform": helper.transform_dict(start_transform),
        "start_ros_pose": helper.ros_pose_dict(start_transform),
        "goal_carla_transform": helper.transform_dict(goal_transform),
        "goal_ros_pose": helper.ros_pose_dict(goal_transform),
        "spawn_point": ",".join(
            f"{value:.6f}"
            for value in (
                start_transform.location.x,
                start_transform.location.y,
                start_transform.location.z,
                start_transform.rotation.roll,
                start_transform.rotation.pitch,
                start_transform.rotation.yaw,
            )
        ),
        "option_counts": option_counts,
        "route": route_points,
    }
    if initial_approach is not None:
        payload["initial_approach_preflight"] = dict(initial_approach)
    if turn_geometry is not None:
        payload["turn_geometry_preflight"] = dict(turn_geometry)
    return payload


def _planar_location_distance(first: Any, second: Any) -> float:
    return math.hypot(
        float(first.x) - float(second.x),
        float(first.y) - float(second.y),
    )


def normalize_route_endpoints(
    route: Sequence[Any], start_location: Any, goal_location: Any
) -> list[Any]:
    """Trim planner snapping and overshoot using planar endpoint proximity."""
    if not route:
        return []
    start_index = min(
        range(len(route)),
        key=lambda index: (
            _planar_location_distance(
                route[index][0].transform.location, start_location
            ),
            index,
        ),
    )
    goal_index = min(
        range(start_index, len(route)),
        key=lambda index: (
            _planar_location_distance(
                route[index][0].transform.location, goal_location
            ),
            -index,
        ),
    )
    return list(route[start_index : goal_index + 1])


def route_endpoint_offsets(
    route: Sequence[Any], start_location: Any, goal_location: Any
) -> tuple[float, float]:
    """Measure endpoint snapping in the map plane, independent of elevation."""
    first_location = route[0][0].transform.location
    last_location = route[-1][0].transform.location
    return (
        _planar_location_distance(first_location, start_location),
        _planar_location_distance(last_location, goal_location),
    )


def _world_map_matches(world: Any, map_entry: Mapping[str, Any]) -> bool:
    name = str(world.get_map().name).rstrip("/")
    return name == map_entry["load_name"] or name.rsplit("/", 1)[-1] == map_entry[
        "canonical_name"
    ]


def activate_map(
    client: Any,
    world: Any,
    map_entry: Mapping[str, Any],
    settle_sec: float,
    sleep: Any = time.sleep,
    *,
    allow_map_load: bool = False,
) -> tuple[Any, bool]:
    """Verify an external map, loading it only after an explicit unsafe opt-in."""
    if settle_sec > 0.0:
        sleep(settle_sec)
    loaded = False
    if not _world_map_matches(world, map_entry):
        if not allow_map_load:
            raise CatalogError(
                f"current CARLA map {world.get_map().name!r} does not match "
                f"required map {map_entry['load_name']!r}; automatic map loading is "
                "disabled, so start CARLA on the required map or pass the explicit "
                "unsafe --allow-map-load opt-in"
            )
        world = client.load_world(map_entry["load_name"])
        loaded = True
        if settle_sec > 0.0:
            sleep(settle_sec)
    if not _world_map_matches(world, map_entry):
        raise CatalogError(
            f"server did not activate {map_entry['load_name']!r}; "
            f"current={world.get_map().name!r}"
        )
    return world, loaded


def build_catalog(
    args: argparse.Namespace,
    manifest: Mapping[str, Any],
    carla: Any,
    helper: Any,
) -> dict[str, Any]:
    map_entry = _map_by_id(manifest, args.map_id)
    if map_entry["status"] == "unavailable":
        raise CatalogError(f"map {args.map_id} is unavailable: {map_entry['reason']}")
    validate_active_server_profile(
        manifest, map_entry, getattr(args, "active_server_profile", None)
    )

    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    world = client.get_world()
    initial_map_name = str(world.get_map().name)
    world, map_load_performed = activate_map(
        client,
        world,
        map_entry,
        getattr(args, "map_load_settle_sec", 10.0),
        allow_map_load=getattr(args, "allow_map_load", False),
    )

    carla_map = world.get_map()
    spawn_points = carla_map.get_spawn_points()
    if len(spawn_points) < 2:
        raise CatalogError(f"map {args.map_id} exposes fewer than two spawn points")
    excluded_spawn_indices = validate_excluded_spawn_indices(
        getattr(args, "exclude_spawn_indices", ()), len(spawn_points)
    )
    eligible_spawn_point_count = len(spawn_points) - len(excluded_spawn_indices)
    planner = helper.GlobalRoutePlanner(carla_map, args.sampling_resolution)
    output_root = args.output_root.expanduser().resolve()
    route_root = output_root / "routes" / args.map_id
    catalog_routes = []
    scenario_results = []
    used_pairs: set[tuple[int, int]] = set()
    total_coverage = Counter()
    total_error_types: Counter[str] = Counter()
    total_error_samples: list[dict[str, Any]] = []
    approach_contract = initial_approach_contract(args)
    geometry_contract = turn_geometry_contract(args)

    for scenario in SCENARIOS:
        created_before = len(catalog_routes)
        traces = 0
        coverage = Counter()
        error_types: Counter[str] = Counter()
        error_samples: list[dict[str, Any]] = []
        for seed in args.seeds:
            created_for_seed = 0
            pairs = deterministic_pairs(
                spawn_points,
                seed,
                args.min_distance,
                args.max_distance,
                args.preferred_distance,
                args.sampling_resolution,
                excluded_spawn_indices,
            )
            for start_index, goal_index in pairs:
                if traces >= args.max_traces or created_for_seed >= args.pairs_per_seed:
                    break
                if (start_index, goal_index) in used_pairs:
                    continue
                traces += 1
                coverage["attempted"] += 1
                _reset_planner(planner, helper)
                route, trace_error = trace_route_candidate(
                    planner,
                    spawn_points[start_index].location,
                    spawn_points[goal_index].location,
                )
                if trace_error is not None:
                    coverage["planner_error"] += 1
                    error_types[type(trace_error).__name__] += 1
                    if len(error_samples) < MAX_TRACE_ERROR_SAMPLES:
                        error_samples.append(
                            _trace_error_sample(start_index, goal_index, trace_error)
                        )
                    continue
                if not route or len(route) < 2:
                    coverage["no_route"] += 1
                    continue
                route = normalize_route_endpoints(
                    route,
                    spawn_points[start_index].location,
                    spawn_points[goal_index].location,
                )
                start_offset, goal_offset = route_endpoint_offsets(
                    route,
                    spawn_points[start_index].location,
                    spawn_points[goal_index].location,
                )
                maximum_endpoint_offset = float(
                    getattr(args, "max_endpoint_offset", 2.0)
                )
                if max(start_offset, goal_offset) > maximum_endpoint_offset:
                    coverage["endpoint_rejected"] += 1
                    continue
                if len(route) < 2:
                    coverage["no_route"] += 1
                    continue
                if not helper.route_matches(route, scenario):
                    coverage["scenario_mismatch"] += 1
                    continue
                length = helper.route_length(route)
                if not args.min_distance <= length <= args.max_distance:
                    coverage["distance_rejected"] += 1
                    continue
                initial_approach = None
                if approach_contract["enabled"] and scenario != "lane_follow":
                    initial_approach = analyze_initial_approach(
                        route, approach_contract
                    )
                    if initial_approach["status"] != "PASS":
                        coverage["initial_approach_rejected"] += 1
                        continue
                turn_geometry = None
                if geometry_contract["enabled"] and scenario in ("left", "right"):
                    turn_geometry = analyze_turn_geometry(
                        route, scenario, geometry_contract
                    )
                    if turn_geometry["status"] != "PASS":
                        coverage["turn_geometry_rejected"] += 1
                        continue
                pair_index = created_for_seed
                route_id = f"{args.map_id}_{scenario}_s{seed:04d}_p{pair_index:02d}"
                route_path = route_root / scenario / f"{route_id}.json"
                payload = _route_payload(
                    helper,
                    map_entry,
                    args.weather,
                    scenario,
                    args.sampling_resolution,
                    start_index,
                    goal_index,
                    spawn_points,
                    route,
                    initial_approach,
                    turn_geometry,
                )
                atomic_write_json(route_path, payload)
                used_pairs.add((start_index, goal_index))
                created_for_seed += 1
                coverage["accepted"] += 1
                catalog_routes.append(
                    {
                        "id": route_id,
                        "status": "ready",
                        "scenario": scenario,
                        "seed": seed,
                        "pair_index": pair_index,
                        "start_spawn_index": start_index,
                        "goal_spawn_index": goal_index,
                        "route_length_m": payload["route_length_m"],
                        "initial_approach_preflight": initial_approach,
                        **(
                            {"turn_geometry_preflight": turn_geometry}
                            if turn_geometry is not None
                            else {}
                        ),
                        "path": route_path.relative_to(output_root).as_posix(),
                        "sha256": sha256_file(route_path),
                    }
                )
        count = len(catalog_routes) - created_before
        skipped = (
            coverage["planner_error"]
            + coverage["no_route"]
            + coverage["endpoint_rejected"]
            + coverage["scenario_mismatch"]
            + coverage["distance_rejected"]
            + coverage["initial_approach_rejected"]
            + coverage["turn_geometry_rejected"]
        )
        total_coverage.update(coverage)
        total_error_types.update(error_types)
        remaining_samples = MAX_TRACE_ERROR_SAMPLES - len(total_error_samples)
        if remaining_samples > 0:
            total_error_samples.extend(error_samples[:remaining_samples])
        scenario_results.append(
            {
                "scenario": scenario,
                "status": "READY" if count else "SKIP",
                "route_count": count,
                "traces": traces,
                "trace_coverage": {
                    "attempted": coverage["attempted"],
                    "accepted": coverage["accepted"],
                    "skipped": skipped,
                    "planner_errors": coverage["planner_error"],
                    "no_route": coverage["no_route"],
                    "endpoint_rejected": coverage["endpoint_rejected"],
                    "scenario_mismatch": coverage["scenario_mismatch"],
                    "distance_rejected": coverage["distance_rejected"],
                    "initial_approach_rejected": coverage[
                        "initial_approach_rejected"
                    ],
                    **(
                        {
                            "turn_geometry_rejected": coverage[
                                "turn_geometry_rejected"
                            ]
                        }
                        if geometry_contract["enabled"]
                        else {}
                    ),
                    "planner_error_types": dict(sorted(error_types.items())),
                    "planner_error_samples": error_samples,
                },
                "reason": None if count else "no matching route in the requested constraints",
            }
        )

    catalog = {
        "schema_version": 1,
        "status": "complete" if catalog_routes else "blocked",
        "map_id": args.map_id,
        "canonical_name": map_entry["canonical_name"],
        "load_name": map_entry["load_name"],
        "map_status": map_entry["status"],
        "server_profile": map_entry["server_profile"],
        "server": {
            "host": args.host,
            "port": args.port,
            "client_version": client.get_client_version(),
            "server_version": client.get_server_version(),
            "active_server_profile": getattr(args, "active_server_profile", None),
            "initial_map_name": initial_map_name,
            "active_map_name": str(carla_map.name),
            "map_load_allowed": getattr(args, "allow_map_load", False),
            "map_load_performed": map_load_performed,
            "map_lifecycle_managed": map_load_performed,
            "pre_read_settle_sec": getattr(args, "map_load_settle_sec", 10.0),
            "map_load_settle_sec": (
                getattr(args, "map_load_settle_sec", 10.0)
                if map_load_performed
                else 0.0
            ),
            "map_load_warning": (
                "client.load_world was explicitly enabled and may crash packaged or "
                "custom CARLA maps"
                if getattr(args, "allow_map_load", False)
                else None
            ),
            "server_lifecycle_managed": False,
        },
        "generation": {
            "weather": args.weather,
            "seeds": list(args.seeds),
            "pairs_per_seed": args.pairs_per_seed,
            "minimum_distance_m": args.min_distance,
            "maximum_distance_m": args.max_distance,
            "preferred_distance_m": args.preferred_distance,
            "sampling_resolution_m": args.sampling_resolution,
            "maximum_endpoint_offset_m": float(
                getattr(args, "max_endpoint_offset", 2.0)
            ),
            "initial_approach_contract": approach_contract,
            **(
                {"turn_geometry_contract": geometry_contract}
                if geometry_contract["enabled"]
                else {}
            ),
            "max_traces_per_scenario": args.max_traces,
            "spawn_point_count": len(spawn_points),
            "excluded_spawn_indices": list(excluded_spawn_indices),
            "excluded_spawn_point_count": len(excluded_spawn_indices),
            "eligible_spawn_point_count": eligible_spawn_point_count,
            "trace_coverage": {
                "attempted": total_coverage["attempted"],
                "accepted": total_coverage["accepted"],
                "skipped": (
                    total_coverage["planner_error"]
                    + total_coverage["no_route"]
                    + total_coverage["endpoint_rejected"]
                    + total_coverage["scenario_mismatch"]
                    + total_coverage["distance_rejected"]
                    + total_coverage["initial_approach_rejected"]
                    + total_coverage["turn_geometry_rejected"]
                ),
                "planner_errors": total_coverage["planner_error"],
                "no_route": total_coverage["no_route"],
                "endpoint_rejected": total_coverage["endpoint_rejected"],
                "scenario_mismatch": total_coverage["scenario_mismatch"],
                "distance_rejected": total_coverage["distance_rejected"],
                "initial_approach_rejected": total_coverage[
                    "initial_approach_rejected"
                ],
                **(
                    {
                        "turn_geometry_rejected": total_coverage[
                            "turn_geometry_rejected"
                        ]
                    }
                    if geometry_contract["enabled"]
                    else {}
                ),
                "planner_error_types": dict(sorted(total_error_types.items())),
                "planner_error_samples": total_error_samples,
                "excluded_spawn_point_count": len(excluded_spawn_indices),
                "eligible_spawn_point_count": eligible_spawn_point_count,
            },
        },
        "scenario_results": scenario_results,
        "routes": catalog_routes,
    }
    atomic_write_json(output_root / "route_catalog.json", catalog)
    return catalog


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        manifest, _ = load_manifest(args.manifest)
        carla, helper = _load_runtime_modules()
        catalog = build_catalog(args, manifest, carla, helper)
    except (CatalogError, ManifestError, OSError, ValueError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    print(
        f"catalog={args.output_root.expanduser().resolve() / 'route_catalog.json'} "
        f"map={catalog['map_id']} routes={len(catalog['routes'])} status={catalog['status']}"
    )
    return 0 if catalog["status"] == "complete" else 1


if __name__ == "__main__":
    raise SystemExit(main())
