#!/usr/bin/env python3
"""Fail-closed comparison for the Town06 60 km/h corridor geometry A/B.

The experiment changes the active straight-route corridor from 0.50 m to 0.20 m.
``turn_outward_corridor_half_width_m`` follows the same value because the route
manager requires it not to exceed the route corridor.  On the enforced straight
route that second parameter is inactive, so this remains one behavioral knob.
"""

from __future__ import annotations

import argparse
from bisect import bisect_left
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
from typing import Any, Iterable, Mapping

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import yaml  # noqa: E402


PROFILE_ID = "carla_vad_60kph_straight_pilot_v1"
TARGET_SPEED_MPS = 60.0 / 3.6
MINIMUM_SUSTAINED_SPEED_MPS = 15.0
MINIMUM_SUSTAINED_SPEED_SEC = 1.0
MAXIMUM_OBSERVED_SPEED_MPS = 18.0
MAXIMUM_CTE_M = 1.0
MAXIMUM_LATERAL_ACCELERATION_MPS2 = 1.2
BASELINE_CORRIDOR_M = 0.50
CANDIDATE_CORRIDOR_M = 0.20
CURVATURE_15MPS_LIMIT_PER_M = 1.0 / (MINIMUM_SUSTAINED_SPEED_MPS**2)

EXPECTED_PROFILE_CONTEXT = {
    "longitudinal_velocity_source": "explicit_simulation_nominal",
    "vad_velocity_evaluated": False,
    "vad_geometry_evaluated": True,
}

STRING_INVARIANTS = {
    "SPEED_30KPH": "false",
    "SPEED_60KPH_PILOT": "true",
    "SPEED_PROFILE_ID": PROFILE_ID,
    "ROUTE_SCENARIO": "straight",
    "SPEED_EXPOSURE_MODE": "straight_target_required",
    "LONGITUDINAL_SPEED_SOURCE": "explicit_simulation_profile",
    "LONGITUDINAL_ACCELERATION_ROLE": "trajectory_internal_curve_exit_cap",
    "VAD_CRUISE_VELOCITY_EVALUATED": "false",
    "VAD_HARD_STOP_SENTINEL_PRESERVED": "true",
    "VAD_VELOCITY_EVALUATED": "false",
    "VAD_GEOMETRY_EVALUATED": "true",
    "VAD_GEOMETRY_SOURCE": "true",
    "SPEED_LIMIT_SOURCE": "explicit_simulation_profile",
    "REAL_VEHICLE_READY": "false",
    "SIMULATION_ONLY_EXPLORATORY": "true",
    "ROUTE_SCOPE": "straight_only",
    "ACTUATION_MAP_COVERAGE_STATUS": "EXPLORATORY",
    "ACTUATION_MAP_TARGET_ENVELOPE_CLASSIFICATION": (
        "TARGET_ENVELOPE_EXCEEDS_MAP_AXIS_CLAMP_IF_REACHED"
    ),
    "ACTUATION_TARGET_WITHIN_MAP_VELOCITY_AXIS": "false",
    "CONTROL_AB_CANDIDATE_ID": "baseline",
    "CONTROL_AB_PID_I40": "false",
    "CONTROL_AB_TURN_PREVIEW_5M": "false",
    "CONTROL_AB_ISOLATED_SINGLE_KNOB": "true",
    "TIGHT_CORRIDOR_CANDIDATE": "false",
    "TRAJECTORY_STABILITY_CANDIDATE": "false",
    "CAMERA_SOURCE_5HZ": "true",
    "CAMERA_TRANSPORT_PROFILE_ID": (
        "carla_vad_camera_source_5hz_best_effort_image_v2"
    ),
    "CAMERA_IMAGE_PUBLISH_QOS": "best_effort",
    "CAMERA_IMAGE_PUBLISH_HISTORY": "keep_last",
    "CAMERA_IMAGE_PUBLISH_DEPTH": "1",
    "VAD_IMAGE_SUBSCRIPTION_QOS": "best_effort",
    "VAD_IMAGE_SUBSCRIPTION_DEPTH": "1",
    "RVIZ_IMAGE_SUBSCRIPTION_QOS": "best_effort",
    "RVIZ_IMAGE_SUBSCRIPTION_DEPTH": "1",
    "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
    "ROS_LOCALHOST_ONLY": "0",
}

NUMERIC_INVARIANTS = {
    "TARGET_SPEED_MPS": TARGET_SPEED_MPS,
    "TARGET_SPEED_KPH": 60.0,
    "MINIMUM_SUSTAINED_SPEED_MPS": MINIMUM_SUSTAINED_SPEED_MPS,
    "MINIMUM_SUSTAINED_SPEED_SEC": MINIMUM_SUSTAINED_SPEED_SEC,
    "MAXIMUM_OBSERVED_SPEED_MPS": MAXIMUM_OBSERVED_SPEED_MPS,
    "MAXIMUM_LATERAL_ACCELERATION_LIMIT_MPS2": (
        MAXIMUM_LATERAL_ACCELERATION_MPS2
    ),
    "MAXIMUM_LONGITUDINAL_ACCELERATION_MPS2": 1.5,
    "MAXIMUM_LATERAL_ACCELERATION_MPS2": 1.0,
    "MAXIMUM_SPEED_SAMPLE_GAP_SEC": 0.25,
    "CONTROLLER_STOP_OFFSET_M": 0.60,
    "COMFORTABLE_DECELERATION_MPS2": 2.0,
    "MANEUVER_LOOKAHEAD_M": 6.0,
    "MANEUVER_EXIT_LOOKAHEAD_M": 3.5,
    "CURVATURE_SPEED_PREVIEW_M": 6.0,
    "ROUTE_CURVATURE_LOOKAHEAD_M": 40.0,
    "MAX_ROUTE_DEVIATION_M": 1.0,
    "MAX_CANDIDATE_AGE_SEC": 0.5,
    "CANDIDATE_TIMEOUT_SEC": 1.5,
    "LONGITUDINAL_PID_MAX_OUT_MPS2": 1.5,
    "LONGITUDINAL_PID_MAX_P_EFFORT_MPS2": 1.5,
    "COMMAND_GATE_NOMINAL_LONGITUDINAL_ACCELERATION_MPS2": 1.5,
    "CAMERA_SOURCE_SENSOR_TICK_SEC": 0.2,
    "CAMERA_ROS_PUBLISH_HZ": 5.0,
    "ACTUATION_MAP_VELOCITY_AXIS_MAXIMUM_MPS": 13.89,
}

EQUAL_HASH_ENV_KEYS = (
    "CAMERA_TRANSPORT_SENSOR_MAPPING_SHA256",
    "CAMERA_TRANSPORT_VAD_OVERRIDE_SHA256",
    "CAMERA_TRANSPORT_CYCLONEDDS_SHA256",
    "VAD_MODEL_OVERRIDE_SHA256",
    "SENSOR_MAPPING_SHA256",
    "MPC_PARAM_SHA256",
    "VEHICLE_CMD_GATE_PARAM_SHA256",
    "VEHICLE_CMD_GATE_METADATA_SHA256",
    "LONGITUDINAL_CONTROLLER_PARAM_SHA256",
    "LONGITUDINAL_CONTROLLER_METADATA_SHA256",
    "TRAJECTORY_LOGIC_SHA256",
    "VAD_ROUTE_MANAGER_SHA256",
)

PROVENANCE_FILES = (
    "speed_profile_provenance/longitudinal_controller.param.yaml",
    "speed_profile_provenance/longitudinal_controller.param.yaml.metadata.json",
    "speed_profile_provenance/vehicle_cmd_gate.param.yaml",
    "speed_profile_provenance/vehicle_cmd_gate.param.yaml.metadata.json",
    "actuation_config_provenance/raw_vehicle_cmd_converter.param.yaml",
    "actuation_config_provenance/accel_map.csv",
    "actuation_config_provenance/brake_map.csv",
    "actuation_config_provenance/steer_map.csv",
    "trajectory_code_provenance/vad_route_logic.py",
    "trajectory_code_provenance/vad_route_manager.py",
)


class ComparisonError(RuntimeError):
    """Raised when the A/B pair is incomplete, tampered, or confounded."""


def _read_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ComparisonError(f"cannot read {label} {path}: {error}") from error
    if not isinstance(value, dict):
        raise ComparisonError(f"{label} must be a JSON object: {path}")
    return value


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
    except OSError as error:
        raise ComparisonError(f"cannot hash {path}: {error}") from error
    return digest.hexdigest()


def _sha256_json(value: Any) -> str:
    try:
        encoded = json.dumps(
            value, sort_keys=True, separators=(",", ":"), allow_nan=False
        ).encode("utf-8")
    except (TypeError, ValueError) as error:
        raise ComparisonError(f"cannot canonicalize JSON evidence: {error}") from error
    return hashlib.sha256(encoded).hexdigest()


def _number(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ComparisonError(f"{label} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise ComparisonError(f"{label} must be finite")
    return result


def _nested(value: Mapping[str, Any], *keys: str) -> Any:
    current: Any = value
    for key in keys:
        if not isinstance(current, Mapping) or key not in current:
            raise ComparisonError(f"missing field {'.'.join(keys)}")
        current = current[key]
    return current


def _resolved_path(value: object, base: Path, label: str) -> Path:
    if not isinstance(value, str) or not value:
        raise ComparisonError(f"{label} path is missing")
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = base / path
    return path.resolve()


def _same_number(actual: object, expected: float, label: str) -> float:
    value = _number(actual, label)
    if not math.isclose(value, expected, rel_tol=0.0, abs_tol=1.0e-9):
        raise ComparisonError(f"{label} changed: {value!r} != {expected!r}")
    return value


def _parse_env(path: Path) -> dict[str, str]:
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError as error:
        raise ComparisonError(f"cannot read runtime environment {path}: {error}") from error
    output: dict[str, str] = {}
    for line in lines:
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        if key in output:
            raise ComparisonError(f"duplicate runtime.env key {key}: {path}")
        output[key] = value
    return output


def resolve_trial(path: Path) -> Path:
    root = path.expanduser().resolve()
    required = ("result.json", "runtime.env", "aligned_route.json")
    if root.is_dir() and all((root / name).is_file() for name in required):
        return root
    candidates: list[Path] = []
    for pattern in ("trial/attempt_*", "attempts/attempt_*"):
        candidates.extend(
            candidate.resolve()
            for candidate in sorted(root.glob(pattern))
            if candidate.is_dir()
            and all((candidate / name).is_file() for name in required)
        )
    unique = sorted(set(candidates))
    if len(unique) != 1:
        raise ComparisonError(
            f"expected exactly one complete trial under {root}, found {len(unique)}"
        )
    return unique[0]


def _bag_manifest(path: Path) -> dict[str, Any]:
    if path.is_symlink():
        raise ComparisonError(f"rosbag root must not be a symlink: {path}")
    root = path.resolve()
    if not root.is_dir():
        raise ComparisonError(f"rosbag directory does not exist: {root}")
    files: list[dict[str, Any]] = []
    for item in sorted(root.rglob("*")):
        if item.is_symlink():
            raise ComparisonError(f"rosbag evidence must not contain symlinks: {item}")
        if item.is_file():
            files.append(
                {
                    "path": item.relative_to(root).as_posix(),
                    "size_bytes": item.stat().st_size,
                    "sha256": _sha256(item),
                }
            )
    if not files or not any(item["path"] == "metadata.yaml" for item in files):
        raise ComparisonError(f"rosbag evidence has no metadata.yaml: {root}")
    payload: dict[str, Any] = {
        "schema_version": 1,
        "root": str(root),
        "files": files,
    }
    payload["sha256"] = _sha256_json({"schema_version": 1, "files": files})
    return payload


def _validate_bag_binding(
    trial: Path, speed: Mapping[str, Any], bag: Mapping[str, Any]
) -> None:
    identity = _nested(speed, "source_identity")
    if not isinstance(identity, Mapping):
        raise ComparisonError("speed profile source identity must be an object")
    identity_without_sha = dict(identity)
    recorded_identity_sha = identity_without_sha.pop("sha256", None)
    if recorded_identity_sha != _sha256_json(identity_without_sha):
        raise ComparisonError("speed profile source-identity digest mismatch")
    recorded_bag = _nested(identity, "rosbag")
    if not isinstance(recorded_bag, Mapping):
        raise ComparisonError("speed profile rosbag identity must be an object")
    if _resolved_path(recorded_bag.get("root"), trial, "bound rosbag") != Path(
        str(bag["root"])
    ):
        raise ComparisonError("speed profile is bound to another rosbag root")
    if recorded_bag.get("files") != bag["files"] or recorded_bag.get("sha256") != bag[
        "sha256"
    ]:
        raise ComparisonError("speed profile rosbag manifest does not match stored bag")


def _canonical_aligned_route(route: Mapping[str, Any]) -> dict[str, Any]:
    canonical = deepcopy(dict(route))
    alignment = canonical.get("coordinate_alignment")
    if not isinstance(alignment, dict):
        raise ComparisonError("aligned route lacks coordinate_alignment")
    alignment.pop("source_route", None)
    return canonical


def _load_route_identity(trial: Path, route: Mapping[str, Any]) -> dict[str, Any]:
    if route.get("town") != "Town06" or route.get("scenario") != "straight":
        raise ComparisonError("geometry A/B is restricted to Town06/straight")
    if _nested(route, "physical_straight_preflight", "status") != "PASS":
        raise ComparisonError("physical straight-route preflight did not pass")
    source_sha = _nested(route, "coordinate_alignment", "source_route_sha256")
    if not isinstance(source_sha, str) or len(source_sha) != 64:
        raise ComparisonError("aligned route source digest is invalid")
    copied_source = trial / "source_route.json"
    if not copied_source.is_file() or _sha256(copied_source) != source_sha:
        raise ComparisonError("copied source route does not match alignment provenance")
    alignment = _read_json(trial / "route_alignment.json", "route alignment")
    if (
        alignment.get("status") != "PASS"
        or alignment.get("source_route_sha256") != source_sha
        or alignment.get("aligned_route_sha256") != _sha256(trial / "aligned_route.json")
    ):
        raise ComparisonError("route-alignment digest/status mismatch")
    return {
        "town": route["town"],
        "scenario": route["scenario"],
        "route_length_m": _number(route.get("route_length_m"), "route length"),
        "source_route_sha256": source_sha,
        "aligned_route_file_sha256": _sha256(trial / "aligned_route.json"),
        "canonical_geometry_identity_sha256": _sha256_json(
            _canonical_aligned_route(route)
        ),
        "physical_straight_preflight": route["physical_straight_preflight"],
    }


def _validate_analysis_bindings(
    trial: Path,
    result: Mapping[str, Any],
    speed: Mapping[str, Any],
    diagnosis: Mapping[str, Any],
    route: Mapping[str, Any],
    bag: Mapping[str, Any],
) -> None:
    route_path = (trial / "aligned_route.json").resolve()
    result_path = (trial / "result.json").resolve()
    bag_path = (trial / "bag").resolve()
    if _resolved_path(result.get("route_file"), trial, "result route") != route_path:
        raise ComparisonError("result is bound to another effective route")
    if result.get("profile_context") != EXPECTED_PROFILE_CONTEXT:
        raise ComparisonError("route-result profile context changed")
    inputs = _nested(speed, "inputs")
    if not isinstance(inputs, Mapping):
        raise ComparisonError("speed profile inputs must be an object")
    if (
        inputs.get("profile_id") != PROFILE_ID
        or inputs.get("longitudinal_speed_source")
        != "explicit_simulation_nominal"
    ):
        raise ComparisonError("speed-profile input contract changed")
    _same_number(inputs.get("target_speed_mps"), TARGET_SPEED_MPS, "speed target")
    if _resolved_path(inputs.get("bag"), trial, "speed bag") != bag_path:
        raise ComparisonError("speed profile is bound to another rosbag")
    identity = _nested(speed, "source_identity")
    effective = _nested(identity, "effective_route")
    bound_result = _nested(identity, "route_result")
    if not isinstance(effective, Mapping) or not isinstance(bound_result, Mapping):
        raise ComparisonError("speed-profile source binding is incomplete")
    if (
        _resolved_path(effective.get("path"), trial, "bound route") != route_path
        or effective.get("sha256") != _sha256(route_path)
        or effective.get("town") != route.get("town")
        or effective.get("scenario") != route.get("scenario")
    ):
        raise ComparisonError("speed profile effective-route binding mismatch")
    if (
        _resolved_path(bound_result.get("path"), trial, "bound result")
        != result_path
        or bound_result.get("sha256") != _sha256(result_path)
        or bound_result.get("success") is not result.get("success")
        or bound_result.get("reason") != result.get("reason")
        or bound_result.get("profile_context") != EXPECTED_PROFILE_CONTEXT
    ):
        raise ComparisonError("speed profile route-result binding mismatch")
    diagnosis_inputs = _nested(diagnosis, "inputs")
    if not isinstance(diagnosis_inputs, Mapping):
        raise ComparisonError("diagnosis inputs must be an object")
    if (
        _resolved_path(diagnosis_inputs.get("bag"), trial, "diagnosis bag")
        != bag_path
        or _resolved_path(
            diagnosis_inputs.get("route_file"), trial, "diagnosis route"
        )
        != route_path
        or diagnosis_inputs.get("town") != "Town06"
        or diagnosis_inputs.get("scenario") != "straight"
    ):
        raise ComparisonError("diagnosis source binding mismatch")
    _validate_bag_binding(trial, speed, bag)


def _load_params(trial: Path) -> dict[str, Any]:
    path = trial / "vad_route_manager.params.yaml"
    try:
        payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, yaml.YAMLError) as error:
        raise ComparisonError(f"cannot read route-manager parameters {path}: {error}") from error
    try:
        params = payload["/vad_route_manager"]["ros__parameters"]
    except (KeyError, TypeError) as error:
        raise ComparisonError("route-manager parameter dump has unexpected shape") from error
    if not isinstance(params, dict):
        raise ComparisonError("route-manager parameters must be an object")
    expected_route = (trial / "aligned_route.json").resolve()
    if _resolved_path(params.get("route_file"), trial, "route-manager route") != expected_route:
        raise ComparisonError("route manager was configured with another route")
    return dict(params)


def _validate_sha256s_file(directory: Path, required_names: Iterable[str]) -> None:
    sums = directory / "SHA256SUMS"
    if not sums.is_file():
        raise ComparisonError(f"missing provenance checksum file: {sums}")
    records: dict[str, str] = {}
    for line in sums.read_text(encoding="utf-8").splitlines():
        fields = line.split(maxsplit=1)
        if len(fields) != 2:
            raise ComparisonError(f"invalid checksum record in {sums}: {line!r}")
        digest, name = fields
        name = name.lstrip("*")
        if name in records:
            raise ComparisonError(f"duplicate checksum record for {name}: {sums}")
        records[name] = digest
    for name in required_names:
        path = directory / name
        if not path.is_file() or records.get(name) != _sha256(path):
            raise ComparisonError(f"provenance checksum mismatch: {path}")


def _load_provenance(trial: Path, environment: Mapping[str, str]) -> dict[str, str]:
    _validate_sha256s_file(
        trial / "speed_profile_provenance",
        (
            "longitudinal_controller.param.yaml",
            "longitudinal_controller.param.yaml.metadata.json",
            "vehicle_cmd_gate.param.yaml",
            "vehicle_cmd_gate.param.yaml.metadata.json",
        ),
    )
    _validate_sha256s_file(
        trial / "trajectory_code_provenance",
        ("vad_route_logic.py", "vad_route_manager.py"),
    )
    manifest = _read_json(
        trial / "actuation_config_provenance/manifest.json", "actuation manifest"
    )
    manifest_files = manifest.get("files")
    if manifest.get("schema_version") != 1 or not isinstance(manifest_files, Mapping):
        raise ComparisonError("actuation provenance manifest has unexpected shape")
    execution = manifest.get("execution")
    if not isinstance(execution, Mapping) or (
        execution.get("uses_original_selected_config") is not True
        or execution.get("uses_artifact_copy") is not False
    ):
        raise ComparisonError("actuation converter execution provenance changed")
    records: dict[str, str] = {}
    for relative in PROVENANCE_FILES:
        path = trial / relative
        if not path.is_file():
            raise ComparisonError(f"missing provenance artifact: {path}")
        records[relative] = _sha256(path)
    actuation_names = {
        "config": "raw_vehicle_cmd_converter.param.yaml",
        "accel_map": "accel_map.csv",
        "brake_map": "brake_map.csv",
        "steer_map": "steer_map.csv",
    }
    for key, artifact in actuation_names.items():
        entry = manifest_files.get(key)
        if not isinstance(entry, Mapping) or entry.get("artifact") != artifact:
            raise ComparisonError(f"actuation manifest entry changed: {key}")
        actual = records[f"actuation_config_provenance/{artifact}"]
        if entry.get("sha256") != actual:
            raise ComparisonError(f"actuation manifest digest mismatch: {artifact}")
    env_bindings = {
        "LONGITUDINAL_CONTROLLER_PARAM_SHA256": (
            "speed_profile_provenance/longitudinal_controller.param.yaml"
        ),
        "LONGITUDINAL_CONTROLLER_METADATA_SHA256": (
            "speed_profile_provenance/longitudinal_controller.param.yaml.metadata.json"
        ),
        "VEHICLE_CMD_GATE_PARAM_SHA256": (
            "speed_profile_provenance/vehicle_cmd_gate.param.yaml"
        ),
        "VEHICLE_CMD_GATE_METADATA_SHA256": (
            "speed_profile_provenance/vehicle_cmd_gate.param.yaml.metadata.json"
        ),
    }
    for key, relative in env_bindings.items():
        if environment.get(key) != records[relative]:
            raise ComparisonError(f"runtime provenance digest mismatch: {key}")
    return records


def _validate_environment(environment: Mapping[str, str], role: str) -> None:
    for key, expected in STRING_INVARIANTS.items():
        if environment.get(key) != expected:
            raise ComparisonError(f"{role} environment {key} changed")
    for key, expected in NUMERIC_INVARIANTS.items():
        try:
            actual: object = float(environment[key])
        except (KeyError, ValueError) as error:
            raise ComparisonError(f"{role} environment {key} is missing/invalid") from error
        _same_number(actual, expected, f"{role} environment {key}")
    for key in EQUAL_HASH_ENV_KEYS:
        value = environment.get(key)
        if not isinstance(value, str) or len(value) != 64:
            raise ComparisonError(f"{role} environment {key} is missing/invalid")


def _geometry_metadata(environment: Mapping[str, str], role: str) -> dict[str, Any]:
    if role == "baseline":
        candidate_id = environment.get(
            "GEOMETRY_AB_CANDIDATE_ID", "legacy_v3_implicit_baseline"
        )
        flag = environment.get("GEOMETRY_AB_ROUTE_CORRIDOR_0P2", "false")
        if candidate_id not in {
            "baseline_corridor_0p5",
            "legacy_v3_implicit_baseline",
        } or flag != "false":
            raise ComparisonError("baseline geometry A/B metadata is invalid")
    elif role == "candidate":
        candidate_id = environment.get("GEOMETRY_AB_CANDIDATE_ID")
        flag = environment.get("GEOMETRY_AB_ROUTE_CORRIDOR_0P2")
        expected = {
            "GEOMETRY_AB_CANDIDATE_ID": "route_corridor_0p2",
            "GEOMETRY_AB_ROUTE_CORRIDOR_0P2": "true",
            "GEOMETRY_AB_ROUTE_CORRIDOR_BASELINE_M": "0.50",
            "GEOMETRY_AB_ROUTE_CORRIDOR_CANDIDATE_M": "0.20",
            "GEOMETRY_AB_BEHAVIORAL_SINGLE_KNOB": "true",
            "GEOMETRY_AB_PARAMETER_CHANGE_COUNT": "2",
            "GEOMETRY_AB_COUPLED_PARAMETER_REASON": (
                "turn_width_must_not_exceed_route_width"
            ),
            "GEOMETRY_AB_ROUTE_SCOPE": "straight_only",
            "ROUTE_CORRIDOR_HALF_WIDTH_M": "0.20",
            "TURN_OUTWARD_CORRIDOR_HALF_WIDTH_M": "0.20",
        }
        changed = [key for key, value in expected.items() if environment.get(key) != value]
        if changed:
            raise ComparisonError(
                f"candidate geometry A/B metadata changed: {', '.join(changed)}"
            )
    else:
        raise ComparisonError(f"unsupported role: {role}")
    return {
        "candidate_id": candidate_id,
        "route_corridor_0p2": flag,
        "behavioral_single_knob": environment.get(
            "GEOMETRY_AB_BEHAVIORAL_SINGLE_KNOB",
            "legacy_v3_inferred_from_parameter_dump" if role == "baseline" else None,
        ),
    }


def _percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    location = (len(ordered) - 1) * fraction
    lower = int(math.floor(location))
    upper = int(math.ceil(location))
    if lower == upper:
        return ordered[lower]
    weight = location - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def _project_remaining_m(x: float, y: float, route: Mapping[str, Any]) -> float:
    points = route.get("route")
    if not isinstance(points, list) or len(points) < 2:
        raise ComparisonError("aligned route has fewer than two points")
    best_distance2 = math.inf
    best_progress = 0.0
    for start, end in zip(points, points[1:]):
        try:
            x0 = float(start["x"])
            y0 = float(start["y"])
            x1 = float(end["x"])
            y1 = float(end["y"])
            d0 = float(start["distance_m"])
            d1 = float(end["distance_m"])
        except (KeyError, TypeError, ValueError) as error:
            raise ComparisonError("aligned route point is malformed") from error
        dx = x1 - x0
        dy = y1 - y0
        denominator = dx * dx + dy * dy
        ratio = 0.0 if denominator <= 1.0e-12 else (
            ((x - x0) * dx + (y - y0) * dy) / denominator
        )
        ratio = min(1.0, max(0.0, ratio))
        projected_x = x0 + ratio * dx
        projected_y = y0 + ratio * dy
        distance2 = (x - projected_x) ** 2 + (y - projected_y) ** 2
        if distance2 < best_distance2:
            best_distance2 = distance2
            best_progress = d0 + ratio * (d1 - d0)
    route_length = _number(route.get("route_length_m"), "route length")
    return max(0.0, route_length - best_progress)


def _midroute_caps(
    speed: Mapping[str, Any], result: Mapping[str, Any], route: Mapping[str, Any]
) -> dict[str, Any]:
    planning = _nested(speed, "series", "explicit_overlaid_planning")
    actual_path = result.get("actual_path")
    if not isinstance(planning, list) or not planning:
        raise ComparisonError("speed profile has no planning series")
    if not isinstance(actual_path, list) or len(actual_path) < 2:
        raise ComparisonError("result has insufficient actual-path samples")
    timed_path: list[tuple[float, float, float]] = []
    for item in actual_path:
        if not isinstance(item, Mapping):
            raise ComparisonError("actual-path sample is malformed")
        timed_path.append(
            (
                _number(item.get("sim_time_sec"), "actual-path simulation time"),
                _number(item.get("x"), "actual-path x"),
                _number(item.get("y"), "actual-path y"),
            )
        )
    timed_path.sort()
    path_times = [item[0] for item in timed_path]
    comfortable_deceleration = _number(
        _nested(result, "limits").get("comfortable_deceleration_mps2", 2.0)
        if isinstance(_nested(result, "limits"), Mapping)
        else 2.0,
        "comfortable deceleration",
    )
    if comfortable_deceleration <= 0.0:
        raise ComparisonError("comfortable deceleration must be positive")
    terminal_buffer_m = TARGET_SPEED_MPS**2 / (2.0 * comfortable_deceleration)
    eligible: list[dict[str, float]] = []
    capped: list[dict[str, float]] = []
    for item in planning:
        if not isinstance(item, Mapping):
            raise ComparisonError("planning-series sample is malformed")
        header_ns = item.get("header_time_ns")
        if isinstance(header_ns, bool) or not isinstance(header_ns, int):
            raise ComparisonError("planning sample has no integer header timestamp")
        sim_time = header_ns / 1.0e9
        index = bisect_left(path_times, sim_time)
        candidates = [candidate for candidate in (index - 1, index) if 0 <= candidate < len(timed_path)]
        if not candidates:
            continue
        nearest = min(candidates, key=lambda candidate: abs(path_times[candidate] - sim_time))
        if abs(path_times[nearest] - sim_time) > 0.30:
            continue
        _, x, y = timed_path[nearest]
        remaining = _project_remaining_m(x, y, route)
        progress = _number(route.get("route_length_m"), "route length") - remaining
        if progress < 5.0 or remaining < terminal_buffer_m:
            continue
        horizon_min = _number(
            item.get("trajectory_horizon_minimum_mps"), "planning horizon minimum"
        )
        sample = {
            "sim_time_sec": sim_time,
            "remaining_distance_m": remaining,
            "planning_horizon_minimum_mps": horizon_min,
        }
        eligible.append(sample)
        if 4.0 <= horizon_min <= 9.0:
            capped.append(sample)
    if not eligible:
        raise ComparisonError("no causally aligned mid-route planning samples")
    eligible.sort(key=lambda item: item["sim_time_sec"])
    capped_values = [item["planning_horizon_minimum_mps"] for item in capped]
    total_duration = 0.0
    longest_duration = 0.0
    current_duration = 0.0
    for index, item in enumerate(eligible):
        is_capped = 4.0 <= item["planning_horizon_minimum_mps"] <= 9.0
        if index + 1 < len(eligible):
            delta = eligible[index + 1]["sim_time_sec"] - item["sim_time_sec"]
        else:
            delta = 0.0
        if is_capped and 0.0 < delta <= 0.50:
            total_duration += delta
            current_duration += delta
            longest_duration = max(longest_duration, current_duration)
        else:
            current_duration = 0.0
    return {
        "definition": {
            "planning_horizon_minimum_speed_range_mps": [4.0, 9.0],
            "minimum_progress_m": 5.0,
            "minimum_remaining_distance_m": terminal_buffer_m,
            "terminal_buffer_basis": (
                "distance required to stop from 16.6667 m/s at configured "
                "comfortable deceleration"
            ),
            "maximum_actual_path_alignment_error_sec": 0.30,
        },
        "eligible_planning_sample_count": len(eligible),
        "cap_sample_count": len(capped),
        "cap_sample_fraction_percent": 100.0 * len(capped) / len(eligible),
        "estimated_exposure_duration_sec": total_duration,
        "longest_contiguous_exposure_sec": longest_duration,
        "minimum_cap_mps": min(capped_values) if capped_values else None,
        "median_cap_mps": _percentile(capped_values, 0.5),
        "examples": capped[:3],
    }


def _runtime_health(
    trial: Path,
    environment: Mapping[str, str],
    health: Mapping[str, Any],
    latency: Mapping[str, Any],
    camera: Mapping[str, Any],
    result: Mapping[str, Any],
) -> dict[str, Any]:
    if environment.get("RUNTIME_HEALTH_GATE_ENABLED") != "true":
        raise ComparisonError("runtime health gate was not enabled")
    health_sha = _sha256(trial / "runtime_health.json")
    if environment.get("RUNTIME_HEALTH_EVIDENCE_SHA256") != health_sha:
        raise ComparisonError("runtime health evidence digest mismatch")
    indexes = _nested(health, "sequence", "winning_window_indexes")
    windows = health.get("windows")
    if not isinstance(indexes, list) or not indexes or not isinstance(windows, list):
        raise ComparisonError("runtime health has no winning-window evidence")
    try:
        rtfs = [
            _number(windows[int(index)]["clock"]["rtf"], "runtime-health RTF")
            for index in indexes
        ]
    except (IndexError, KeyError, TypeError, ValueError) as error:
        raise ComparisonError("runtime-health winning windows are malformed") from error
    coverage = _number(
        _nested(latency, "camera_bundle", "bundle_coverage_percent"),
        "camera bundle coverage",
    )
    receipt_p95 = _number(
        _nested(latency, "camera_bundle", "receipt_span_sec", "p95"),
        "camera bundle receipt p95",
    )
    if _resolved_path(_nested(latency, "inputs", "bag"), trial, "latency bag") != (
        trial / "bag"
    ).resolve():
        raise ComparisonError("latency analysis is bound to another rosbag")
    camera_coverage = _number(
        camera.get("bundle_coverage_percent"), "camera validation coverage"
    )
    if not math.isclose(camera_coverage, coverage, rel_tol=0.0, abs_tol=1.0e-9):
        raise ComparisonError("camera validation and latency coverage disagree")
    metrics = _nested(result, "metrics")
    sim_elapsed = _number(metrics.get("sim_elapsed_sec"), "simulation elapsed time")
    wall_elapsed = _number(metrics.get("wall_elapsed_sec"), "wall elapsed time")
    if wall_elapsed <= 0.0:
        raise ComparisonError("wall elapsed time must be positive")
    return {
        "runtime_health_status": health.get("status"),
        "minimum_winning_window_rtf": min(rtfs),
        "route_rtf": sim_elapsed / wall_elapsed,
        "camera_validation_status": camera.get("status"),
        "camera_bundle_coverage_percent": coverage,
        "camera_bundle_receipt_p95_sec": receipt_p95,
        "status": (
            "PASS"
            if health.get("status") == "PASS"
            and environment.get("RUNTIME_HEALTH_GATE_STATUS") == "PASS"
            and min(rtfs) >= 0.90
            and sim_elapsed / wall_elapsed >= 0.90
            and camera.get("status") == "PASS"
            and coverage >= 99.0
            and receipt_p95 <= 0.04
            else "FAIL"
        ),
    }


def load_trial(path: Path, role: str) -> dict[str, Any]:
    trial = resolve_trial(path)
    required = (
        "result.json",
        "runtime.env",
        "runtime_health.json",
        "aligned_route.json",
        "source_route.json",
        "route_alignment.json",
        "speed_profile.json",
        "diagnosis.json",
        "latency/e2e_latency.json",
        "camera_source_5hz_validation.json",
        "vad_route_manager.params.yaml",
        "actuation_config_provenance/manifest.json",
    )
    missing = [name for name in required if not (trial / name).is_file()]
    if missing:
        raise ComparisonError(f"{role} trial is missing evidence: {missing}")
    result = _read_json(trial / "result.json", "route result")
    health = _read_json(trial / "runtime_health.json", "runtime health")
    route = _read_json(trial / "aligned_route.json", "aligned route")
    speed = _read_json(trial / "speed_profile.json", "speed profile")
    diagnosis = _read_json(trial / "diagnosis.json", "trajectory diagnosis")
    latency = _read_json(trial / "latency/e2e_latency.json", "latency analysis")
    camera = _read_json(
        trial / "camera_source_5hz_validation.json", "camera validation"
    )
    environment = _parse_env(trial / "runtime.env")
    _validate_environment(environment, role)
    geometry_metadata = _geometry_metadata(environment, role)
    bag = _bag_manifest(trial / "bag")
    route_identity = _load_route_identity(trial, route)
    _validate_analysis_bindings(trial, result, speed, diagnosis, route, bag)
    params = _load_params(trial)
    provenance = _load_provenance(trial, environment)
    if speed.get("status") != "complete" or speed.get("quality", {}).get("problems"):
        raise ComparisonError(f"{role} speed-profile analysis is incomplete/degraded")
    warnings = diagnosis.get("quality_warnings")
    if warnings not in (None, []):
        raise ComparisonError(f"{role} trajectory diagnosis has quality warnings")
    result_metrics = _nested(result, "metrics")
    final = _nested(result, "final")
    exposure = _nested(result, "speed_exposure")
    if not isinstance(result.get("success"), bool):
        raise ComparisonError("route result success must be boolean")
    if not isinstance(final, Mapping) or not isinstance(final.get("goal_reached"), bool):
        raise ComparisonError("route result goal outcome must be boolean")
    if not isinstance(exposure, Mapping) or exposure.get("status") not in {"PASS", "FAIL"}:
        raise ComparisonError("speed-exposure outcome is invalid")
    _same_number(
        exposure.get("minimum_sustained_speed_mps"),
        MINIMUM_SUSTAINED_SPEED_MPS,
        "result minimum sustained speed",
    )
    _same_number(
        exposure.get("minimum_sustained_speed_sec"),
        MINIMUM_SUSTAINED_SPEED_SEC,
        "result minimum sustained duration",
    )
    _same_number(
        exposure.get("maximum_observed_speed_limit_mps"),
        MAXIMUM_OBSERVED_SPEED_MPS,
        "result overspeed limit",
    )
    curvature = _nested(
        diagnosis, "metrics", "final_path", "snapshot_peak_curvature_per_m"
    )
    if not isinstance(curvature, Mapping):
        raise ComparisonError("conditioned-trajectory curvature summary is missing")
    curvature_count = curvature.get("count")
    if isinstance(curvature_count, bool) or not isinstance(curvature_count, int) or curvature_count <= 0:
        raise ComparisonError("conditioned-trajectory curvature has no samples")
    curvature_p95 = _number(
        curvature.get("p95_abs"), "conditioned peak-curvature p95"
    )
    curvature_maximum = _number(
        curvature.get("max_abs"), "conditioned peak-curvature maximum"
    )
    if curvature_p95 < 0.0 or curvature_maximum < curvature_p95:
        raise ComparisonError("conditioned-trajectory curvature summary is inconsistent")
    return {
        "role": role,
        "trial_directory": str(trial),
        "evidence_sha256": {
            relative: _sha256(trial / relative)
            for relative in (
                "result.json",
                "speed_profile.json",
                "diagnosis.json",
                "runtime_health.json",
                "latency/e2e_latency.json",
                "camera_source_5hz_validation.json",
                "aligned_route.json",
                "source_route.json",
                "route_alignment.json",
            )
        },
        "bag_manifest": bag,
        "route": route_identity,
        "environment": environment,
        "geometry_metadata": geometry_metadata,
        "route_manager_parameters": params,
        "provenance_sha256": provenance,
        "outcomes": {
            "route_result_success": result.get("success"),
            "reason": result.get("reason"),
            "goal_reached": final.get("goal_reached"),
            "speed_exposure_status": exposure.get("status"),
        },
        "health": _runtime_health(
            trial, environment, health, latency, camera, result
        ),
        "metrics": {
            "conditioned_peak_curvature_p95_per_m": curvature_p95,
            "conditioned_peak_curvature_max_per_m": curvature_maximum,
            "maximum_speed_mps": _number(
                result_metrics.get("maximum_observed_speed_mps"), "maximum speed"
            ),
            "sustained_15mps_duration_sec": _number(
                result_metrics.get("maximum_sustained_speed_duration_sec"),
                "sustained-speed duration",
            ),
            "maximum_cte_m": _number(
                result_metrics.get("maximum_absolute_cte_m"), "maximum CTE"
            ),
            "maximum_lateral_acceleration_mps2": _number(
                result_metrics.get("maximum_lateral_acceleration_mps2"),
                "maximum lateral acceleration",
            ),
            "maximum_trajectory_correction_m": _number(
                result_metrics.get("maximum_trajectory_correction_m"),
                "maximum trajectory correction",
            ),
        },
        "midroute_4_to_9mps_caps": _midroute_caps(speed, result, route),
    }


def _pair_contract(baseline: Mapping[str, Any], candidate: Mapping[str, Any]) -> dict[str, Any]:
    for key in ("source_route_sha256", "canonical_geometry_identity_sha256"):
        if baseline["route"][key] != candidate["route"][key]:
            raise ComparisonError(f"A/B effective-route identity mismatch: {key}")
    for key in STRING_INVARIANTS:
        if baseline["environment"][key] != candidate["environment"][key]:
            raise ComparisonError(f"A/B profile invariant changed: {key}")
    for key in NUMERIC_INVARIANTS:
        if not math.isclose(
            float(baseline["environment"][key]),
            float(candidate["environment"][key]),
            rel_tol=0.0,
            abs_tol=1.0e-9,
        ):
            raise ComparisonError(f"A/B numeric profile invariant changed: {key}")
    for key in EQUAL_HASH_ENV_KEYS:
        if baseline["environment"][key] != candidate["environment"][key]:
            raise ComparisonError(f"A/B provenance environment changed: {key}")
    if baseline["provenance_sha256"] != candidate["provenance_sha256"]:
        changed = sorted(
            key
            for key in baseline["provenance_sha256"]
            if baseline["provenance_sha256"].get(key)
            != candidate["provenance_sha256"].get(key)
        )
        raise ComparisonError(f"A/B controller/gate/converter/code changed: {changed}")

    baseline_params = deepcopy(baseline["route_manager_parameters"])
    candidate_params = deepcopy(candidate["route_manager_parameters"])
    baseline_route_file = baseline_params.pop("route_file", None)
    candidate_route_file = candidate_params.pop("route_file", None)
    expected_changes = {
        "route_corridor_half_width_m": (BASELINE_CORRIDOR_M, CANDIDATE_CORRIDOR_M),
        "turn_outward_corridor_half_width_m": (
            BASELINE_CORRIDOR_M,
            CANDIDATE_CORRIDOR_M,
        ),
    }
    observed: dict[str, dict[str, float]] = {}
    for key, (expected_a, expected_b) in expected_changes.items():
        actual_a = _same_number(baseline_params.pop(key, None), expected_a, f"baseline {key}")
        actual_b = _same_number(candidate_params.pop(key, None), expected_b, f"candidate {key}")
        observed[key] = {"baseline": actual_a, "candidate": actual_b}
    if baseline_params != candidate_params:
        all_keys = sorted(set(baseline_params) | set(candidate_params))
        changed = [key for key in all_keys if baseline_params.get(key) != candidate_params.get(key)]
        raise ComparisonError(f"A/B has non-corridor route-manager changes: {changed}")
    return {
        "status": "PASS",
        "effective_route_identity": {
            "source_route_sha256": baseline["route"]["source_route_sha256"],
            "canonical_geometry_identity_sha256": baseline["route"][
                "canonical_geometry_identity_sha256"
            ],
            "baseline_aligned_file_sha256": baseline["route"][
                "aligned_route_file_sha256"
            ],
            "candidate_aligned_file_sha256": candidate["route"][
                "aligned_route_file_sha256"
            ],
            "note": (
                "aligned-route file SHA may differ because coordinate_alignment."
                "source_route stores a run-local absolute path"
            ),
        },
        "behavioral_change": {
            "id": "straight_route_corridor_half_width_0p5_to_0p2",
            "behavioral_knob_count": 1,
            "serialized_parameter_change_count": 2,
            "changes": observed,
            "coupled_parameter_reason": (
                "turn_outward width must not exceed route width; it is inactive on "
                "the enforced straight route"
            ),
            "run_local_route_file_values": {
                "baseline": baseline_route_file,
                "candidate": candidate_route_file,
            },
        },
        "fixed_provenance_sha256": baseline["provenance_sha256"],
    }


def compare(baseline: dict[str, Any], candidate: dict[str, Any]) -> dict[str, Any]:
    contract = _pair_contract(baseline, candidate)
    a = baseline["metrics"]
    b = candidate["metrics"]
    a_caps = baseline["midroute_4_to_9mps_caps"]
    b_caps = candidate["midroute_4_to_9mps_caps"]
    checks: dict[str, dict[str, Any]] = {}

    def check(name: str, passed: bool, actual: Any, requirement: str) -> None:
        checks[name] = {
            "status": "PASS" if passed else "FAIL",
            "actual": actual,
            "requirement": requirement,
        }

    for label, trial in (("baseline", baseline), ("candidate", candidate)):
        check(
            f"{label}_runtime_camera_health",
            trial["health"]["status"] == "PASS",
            trial["health"],
            "runtime health PASS, RTF >= 0.90, camera coverage >= 99%, receipt p95 <= 40 ms",
        )
    check(
        "candidate_goal_reached",
        candidate["outcomes"]["goal_reached"] is True,
        candidate["outcomes"],
        "route goal must be reached even if the independent speed contract fails",
    )
    check(
        "conditioned_curvature_p95_material_reduction",
        b["conditioned_peak_curvature_p95_per_m"]
        <= 0.80 * a["conditioned_peak_curvature_p95_per_m"],
        {
            "baseline": a["conditioned_peak_curvature_p95_per_m"],
            "candidate": b["conditioned_peak_curvature_p95_per_m"],
            "candidate_to_baseline_ratio": (
                b["conditioned_peak_curvature_p95_per_m"]
                / a["conditioned_peak_curvature_p95_per_m"]
                if a["conditioned_peak_curvature_p95_per_m"] > 0.0
                else None
            ),
        },
        "candidate conditioned peak-curvature p95 <= 80% of baseline",
    )
    check(
        "conditioned_curvature_supports_15mps",
        b["conditioned_peak_curvature_p95_per_m"] <= CURVATURE_15MPS_LIMIT_PER_M,
        b["conditioned_peak_curvature_p95_per_m"],
        f"conditioned peak-curvature p95 <= {CURVATURE_15MPS_LIMIT_PER_M:.9f} 1/m",
    )
    check(
        "conditioned_curvature_maximum_supports_15mps",
        b["conditioned_peak_curvature_max_per_m"] <= CURVATURE_15MPS_LIMIT_PER_M,
        b["conditioned_peak_curvature_max_per_m"],
        f"maximum conditioned peak curvature <= {CURVATURE_15MPS_LIMIT_PER_M:.9f} 1/m",
    )
    check(
        "conditioned_curvature_maximum_no_kink_regression",
        b["conditioned_peak_curvature_max_per_m"]
        <= a["conditioned_peak_curvature_max_per_m"],
        {
            "baseline": a["conditioned_peak_curvature_max_per_m"],
            "candidate": b["conditioned_peak_curvature_max_per_m"],
        },
        "candidate maximum conditioned curvature must not exceed baseline",
    )
    check(
        "midroute_4_to_9mps_caps_eliminated",
        b_caps["cap_sample_count"] == 0,
        {
            "baseline_count": a_caps["cap_sample_count"],
            "candidate_count": b_caps["cap_sample_count"],
            "baseline_duration_sec": a_caps["estimated_exposure_duration_sec"],
            "candidate_duration_sec": b_caps["estimated_exposure_duration_sec"],
        },
        "candidate must have no 4-9 m/s horizon caps outside the terminal braking buffer",
    )
    check(
        "candidate_cte",
        b["maximum_cte_m"] <= MAXIMUM_CTE_M,
        b["maximum_cte_m"],
        f"maximum absolute CTE <= {MAXIMUM_CTE_M:.2f} m",
    )
    check(
        "candidate_lateral_acceleration",
        b["maximum_lateral_acceleration_mps2"]
        <= MAXIMUM_LATERAL_ACCELERATION_MPS2,
        b["maximum_lateral_acceleration_mps2"],
        f"maximum lateral acceleration <= {MAXIMUM_LATERAL_ACCELERATION_MPS2:.2f} m/s^2",
    )
    check(
        "candidate_correction_no_regression",
        b["maximum_trajectory_correction_m"]
        <= a["maximum_trajectory_correction_m"] + 0.05,
        {
            "baseline": a["maximum_trajectory_correction_m"],
            "candidate": b["maximum_trajectory_correction_m"],
            "delta": b["maximum_trajectory_correction_m"]
            - a["maximum_trajectory_correction_m"],
        },
        "maximum trajectory correction increase <= 0.05 m",
    )
    geometry_check_names = tuple(checks)
    geometry_accepted = all(checks[name]["status"] == "PASS" for name in geometry_check_names)

    speed_checks: dict[str, dict[str, Any]] = {}

    def speed_check(name: str, passed: bool, actual: Any, requirement: str) -> None:
        speed_checks[name] = {
            "status": "PASS" if passed else "FAIL",
            "actual": actual,
            "requirement": requirement,
        }

    speed_check(
        "exposure_status",
        candidate["outcomes"]["speed_exposure_status"] == "PASS",
        candidate["outcomes"]["speed_exposure_status"],
        "route result speed-exposure status PASS",
    )
    speed_check(
        "minimum_speed_reached",
        b["maximum_speed_mps"] >= MINIMUM_SUSTAINED_SPEED_MPS,
        b["maximum_speed_mps"],
        f"maximum speed >= {MINIMUM_SUSTAINED_SPEED_MPS:.1f} m/s",
    )
    speed_check(
        "minimum_sustained_duration",
        b["sustained_15mps_duration_sec"] >= MINIMUM_SUSTAINED_SPEED_SEC,
        b["sustained_15mps_duration_sec"],
        f">= 15 m/s sustained for >= {MINIMUM_SUSTAINED_SPEED_SEC:.1f} s",
    )
    speed_check(
        "overspeed_ceiling",
        b["maximum_speed_mps"] <= MAXIMUM_OBSERVED_SPEED_MPS,
        b["maximum_speed_mps"],
        f"maximum speed <= {MAXIMUM_OBSERVED_SPEED_MPS:.1f} m/s",
    )
    speed_passed = all(row["status"] == "PASS" for row in speed_checks.values())
    curvature_ratio = (
        b["conditioned_peak_curvature_p95_per_m"]
        / a["conditioned_peak_curvature_p95_per_m"]
        if a["conditioned_peak_curvature_p95_per_m"] > 0.0
        else math.inf
    )
    cap_ratio = (
        b_caps["cap_sample_count"] / a_caps["cap_sample_count"]
        if a_caps["cap_sample_count"] > 0
        else (0.0 if b_caps["cap_sample_count"] == 0 else math.inf)
    )
    if curvature_ratio <= 0.80 and cap_ratio <= 0.50:
        diagnostic_effect = "IMPROVED"
    elif curvature_ratio > 1.05 or cap_ratio > 1.05:
        diagnostic_effect = "REGRESSED"
    elif curvature_ratio < 0.95 or cap_ratio < 0.95:
        diagnostic_effect = "PARTIAL_IMPROVEMENT_INSUFFICIENT"
    else:
        diagnostic_effect = "NO_MATERIAL_IMPROVEMENT"
    return {
        "schema_version": 1,
        "analysis": "town06_60kph_geometry_only_corridor_ab",
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "pair_contract": contract,
        "baseline": baseline,
        "candidate": candidate,
        "geometry_outcome": {
            "decision": "ACCEPT" if geometry_accepted else "HOLD",
            "diagnostic_effect": diagnostic_effect,
            "checks": checks,
        },
        "speed_contract": {
            "decision": "PASS" if speed_passed else "FAIL",
            "checks": speed_checks,
            "independent_from_geometry_acceptance": True,
        },
        "real_vehicle_ready": False,
        "next_step": (
            "geometry accepted; retain simulation-only status and proceed to the separately isolated throttle/calibration stage"
            if geometry_accepted
            else "hold the corridor candidate; inspect clipping/correction and do not advance to throttle tuning"
        ),
    }


def render_markdown(payload: Mapping[str, Any]) -> str:
    a = payload["baseline"]
    b = payload["candidate"]
    am = a["metrics"]
    bm = b["metrics"]
    ac = a["midroute_4_to_9mps_caps"]
    bc = b["midroute_4_to_9mps_caps"]
    geometry = payload["geometry_outcome"]
    speed = payload["speed_contract"]
    lines = [
        "# Town06 60 km/h geometry-only A/B",
        "",
        f"- Geometry decision: **{geometry['decision']}** ({geometry['diagnostic_effect']})",
        f"- Independent 60 km/h speed contract: **{speed['decision']}**",
        "- Real-vehicle ready: **false**",
        "- Behavioral delta: straight-route corridor half-width 0.50 m → 0.20 m",
        "- Coupled serialized delta: turn-outward width 0.50 m → 0.20 m (inactive on straight route; route-manager invariant)",
        "",
        "## Pair integrity",
        "",
        f"- Effective source route SHA-256: `{payload['pair_contract']['effective_route_identity']['source_route_sha256']}`",
        f"- Canonical geometry/identity SHA-256: `{payload['pair_contract']['effective_route_identity']['canonical_geometry_identity_sha256']}`",
        "- Speed-profile configuration/invariants, controller, command gate, raw converter/maps, trajectory code: byte-identical",
        "- Each speed/diagnosis artifact is bound to its own aligned route and immutable rosbag manifest",
        "",
        "## Measured comparison",
        "",
        "| Metric | A baseline | B corridor 0.20 m |",
        "|---|---:|---:|",
        f"| Conditioned peak curvature p95 (1/m) | {am['conditioned_peak_curvature_p95_per_m']:.6f} | {bm['conditioned_peak_curvature_p95_per_m']:.6f} |",
        f"| Conditioned peak curvature max (1/m) | {am['conditioned_peak_curvature_max_per_m']:.6f} | {bm['conditioned_peak_curvature_max_per_m']:.6f} |",
        f"| Mid-route 4–9 m/s cap samples | {ac['cap_sample_count']} | {bc['cap_sample_count']} |",
        f"| Mid-route cap exposure (s) | {ac['estimated_exposure_duration_sec']:.3f} | {bc['estimated_exposure_duration_sec']:.3f} |",
        f"| Maximum speed (m/s / km/h) | {am['maximum_speed_mps']:.3f} / {am['maximum_speed_mps'] * 3.6:.2f} | {bm['maximum_speed_mps']:.3f} / {bm['maximum_speed_mps'] * 3.6:.2f} |",
        f"| ≥15 m/s sustained (s) | {am['sustained_15mps_duration_sec']:.3f} | {bm['sustained_15mps_duration_sec']:.3f} |",
        f"| Maximum CTE (m) | {am['maximum_cte_m']:.3f} | {bm['maximum_cte_m']:.3f} |",
        f"| Maximum lateral acceleration (m/s²) | {am['maximum_lateral_acceleration_mps2']:.3f} | {bm['maximum_lateral_acceleration_mps2']:.3f} |",
        f"| Maximum trajectory correction (m) | {am['maximum_trajectory_correction_m']:.3f} | {bm['maximum_trajectory_correction_m']:.3f} |",
        "",
        "## Geometry gates",
        "",
        "| Gate | Status | Requirement |",
        "|---|---|---|",
    ]
    for name, row in geometry["checks"].items():
        lines.append(f"| `{name}` | {row['status']} | {row['requirement']} |")
    lines.extend(
        [
            "",
            "## Independent speed gates",
            "",
            "| Gate | Status | Requirement |",
            "|---|---|---|",
        ]
    )
    for name, row in speed["checks"].items():
        lines.append(f"| `{name}` | {row['status']} | {row['requirement']} |")
    lines.extend(
        [
            "",
            "The geometry decision does not imply 60 km/h qualification. The result remains CARLA simulation-only and must not be transferred to a real vehicle.",
            "",
        ]
    )
    return "\n".join(lines)


def render_png(payload: Mapping[str, Any], output: Path) -> None:
    a = payload["baseline"]
    b = payload["candidate"]
    am = a["metrics"]
    bm = b["metrics"]
    ac = a["midroute_4_to_9mps_caps"]
    bc = b["midroute_4_to_9mps_caps"]
    fields = (
        (
            [am["conditioned_peak_curvature_p95_per_m"], bm["conditioned_peak_curvature_p95_per_m"]],
            "Conditioned peak curvature p95",
            "1/m",
            CURVATURE_15MPS_LIMIT_PER_M,
        ),
        (
            [am["conditioned_peak_curvature_max_per_m"], bm["conditioned_peak_curvature_max_per_m"]],
            "Conditioned peak curvature max",
            "1/m",
            None,
        ),
        (
            [ac["cap_sample_count"], bc["cap_sample_count"]],
            "Mid-route 4–9 m/s cap samples",
            "samples",
            0.0,
        ),
        (
            [am["maximum_speed_mps"], bm["maximum_speed_mps"]],
            "Maximum speed",
            "m/s",
            MINIMUM_SUSTAINED_SPEED_MPS,
        ),
        (
            [am["maximum_cte_m"], bm["maximum_cte_m"]],
            "Maximum CTE",
            "m",
            MAXIMUM_CTE_M,
        ),
        (
            [am["maximum_lateral_acceleration_mps2"], bm["maximum_lateral_acceleration_mps2"]],
            "Maximum lateral acceleration",
            "m/s²",
            MAXIMUM_LATERAL_ACCELERATION_MPS2,
        ),
    )
    fig, axes = plt.subplots(2, 3, figsize=(13.5, 7.7), constrained_layout=True)
    for axis, (values, title, unit, limit) in zip(axes.flat, fields):
        better = values[1] <= values[0] if title != "Maximum speed" else values[1] >= values[0]
        axis.bar((0, 1), values, color=("#68737d", "#14837d" if better else "#c44e3b"))
        if limit is not None:
            axis.axhline(limit, color="#d18f00", linestyle="--", linewidth=1.2)
        axis.set_xticks((0, 1), ("A 0.50 m", "B 0.20 m"))
        axis.set_title(title, weight="bold")
        axis.set_ylabel(unit)
        axis.grid(axis="y", alpha=0.25)
        axis.spines[["top", "right"]].set_visible(False)
    fig.suptitle(
        "Town06 straight | 60 km/h geometry-only A/B | "
        f"geometry {payload['geometry_outcome']['decision']} | speed {payload['speed_contract']['decision']}",
        fontsize=14,
        weight="bold",
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=170, facecolor="white")
    plt.close(fig)


def _atomic_write(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(content)
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except OSError:
            pass
        raise


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--output-markdown", type=Path, required=True)
    parser.add_argument("--output-png", type=Path)
    args = parser.parse_args()
    try:
        payload = compare(
            load_trial(args.baseline, "baseline"),
            load_trial(args.candidate, "candidate"),
        )
        _atomic_write(
            args.output_json,
            json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n",
        )
        _atomic_write(args.output_markdown, render_markdown(payload))
        if args.output_png is not None:
            render_png(payload, args.output_png)
    except ComparisonError as error:
        parser.error(str(error))
    print(
        "GEOMETRY_AB_DECISION "
        f"{payload['geometry_outcome']['decision']} "
        f"SPEED_CONTRACT {payload['speed_contract']['decision']} "
        f"{args.output_json}"
    )
    return 0 if payload["geometry_outcome"]["decision"] == "ACCEPT" else 1


if __name__ == "__main__":
    raise SystemExit(main())
