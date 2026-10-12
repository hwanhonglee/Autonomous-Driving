#!/usr/bin/env python3
"""Compare two SHA-bound Autoware/CARLA speed trials on one effective route."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import tempfile
from typing import Any, Mapping, Sequence

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


class ComparisonError(RuntimeError):
    """Raised when two trials cannot be compared without ambiguity."""


EXPECTED_PROFILE_CONTEXT = {
    "longitudinal_velocity_source": "explicit_simulation_nominal",
    "vad_velocity_evaluated": False,
    "vad_geometry_evaluated": True,
}
TRIAL_CONTRACTS = {
    "reference": {
        "label": "30 kph reference",
        "profile_id": "carla_vad_30kph_v2",
        "target_speed_mps": 30.0 / 3.6,
        "result_success": True,
        "speed_exposure_status": "PASS",
        "minimum_sustained_speed_mps": 7.5,
        "maximum_observed_speed_limit_mps": 9.0,
        "actuation_status": "PASS",
    },
    "pilot": {
        "label": "60 kph pilot",
        "profile_id": "carla_vad_60kph_straight_pilot_v1",
        "target_speed_mps": 60.0 / 3.6,
        "result_success": False,
        "speed_exposure_status": "FAIL",
        "minimum_sustained_speed_mps": 15.0,
        "maximum_observed_speed_limit_mps": 18.0,
        "actuation_status": "EXPLORATORY",
    },
}
CAMERA_CONTRACT = {
    "profile_id": "carla_camera_source_5hz_ab_v1",
    "baseline_selector": "speed_30kph",
    "sensor_count": 6,
    "sensor_tick_sec": 0.2,
    "source_frequency_hz": 5.0,
    "ros_publish_frequency_hz": 5.0,
    "maximum_stamp_gap_sec": 0.25,
    "qos_profile": "reliable",
    "only_semantic_delta": "camera_parameters_sensor_tick",
    "imu_gnss_unchanged": True,
}
CAMERA_IDS = (
    "CAM_BACK",
    "CAM_BACK_LEFT",
    "CAM_BACK_RIGHT",
    "CAM_FRONT",
    "CAM_FRONT_LEFT",
    "CAM_FRONT_RIGHT",
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _object(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ComparisonError(f"cannot read {label} {path}: {error}") from error
    if not isinstance(value, dict):
        raise ComparisonError(f"{label} must be a JSON object: {path}")
    return value


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
            return None
        current = current[key]
    return current


def _same_number(actual: object, expected: float, label: str) -> float:
    value = _number(actual, label)
    if not math.isclose(value, expected, rel_tol=0.0, abs_tol=1.0e-9):
        raise ComparisonError(f"{label} changed: {value!r} != {expected!r}")
    return value


def _sha256_json(value: Any) -> str:
    encoded = json.dumps(
        value, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _resolved_evidence_path(value: object, base: Path, label: str) -> Path:
    if not isinstance(value, str) or not value:
        raise ComparisonError(f"{label} path is missing")
    candidate = Path(value).expanduser()
    if not candidate.is_absolute():
        candidate = base / candidate
    return candidate.resolve()


def _bag_manifest(path: Path) -> dict[str, Any]:
    candidate = path.expanduser()
    if candidate.is_symlink():
        raise ComparisonError(f"rosbag evidence root must not be a symlink: {candidate}")
    root = candidate.resolve()
    if not root.is_dir():
        raise ComparisonError(f"rosbag directory does not exist: {root}")
    files: list[dict[str, Any]] = []
    for item in sorted(root.rglob("*")):
        if item.is_symlink():
            raise ComparisonError(f"rosbag evidence must not contain a symlink: {item}")
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
    manifest = {"schema_version": 1, "root": str(root), "files": files}
    manifest["sha256"] = _sha256_json(
        {"schema_version": 1, "files": files}
    )
    return manifest


def _file_record(path: Path) -> dict[str, Any]:
    return {
        "path": str(path),
        "size_bytes": path.stat().st_size,
        "sha256": _sha256(path),
    }


def _validate_source_identity(
    payload: Mapping[str, Any],
    *,
    label: str,
    result: Mapping[str, Any],
    result_path: Path,
    route: Mapping[str, Any],
    route_path: Path,
    bag: Mapping[str, Any],
    profile_id: str,
    target_speed_mps: float,
    longitudinal: bool,
) -> None:
    inputs = payload.get("inputs")
    identity = payload.get("source_identity")
    if not isinstance(inputs, Mapping) or not isinstance(identity, Mapping):
        raise ComparisonError(f"{label} lacks bound inputs/source identity")
    if inputs.get("profile_id") != profile_id:
        raise ComparisonError(f"{label} profile id changed")
    _same_number(inputs.get("target_speed_mps"), target_speed_mps, f"{label} target")
    _same_number(
        inputs.get("target_speed_kph"), target_speed_mps * 3.6, f"{label} target kph"
    )
    if inputs.get("longitudinal_speed_source") != "explicit_simulation_nominal":
        raise ComparisonError(f"{label} longitudinal speed source changed")
    if _resolved_evidence_path(inputs.get("bag"), result_path.parent, label) != Path(
        str(bag["root"])
    ):
        raise ComparisonError(f"{label} is bound to a different rosbag path")

    identity_without_sha = dict(identity)
    identity_sha = identity_without_sha.pop("sha256", None)
    if identity_sha != _sha256_json(identity_without_sha):
        raise ComparisonError(f"{label} source identity digest changed")
    effective = identity.get("effective_route")
    bound_result = identity.get("route_result")
    bound_bag = identity.get("rosbag")
    if not all(isinstance(value, Mapping) for value in (effective, bound_result)):
        raise ComparisonError(f"{label} source identity is incomplete")
    if dict(bound_bag or {}) != dict(bag):
        raise ComparisonError(f"{label} rosbag binding changed")
    if (
        _resolved_evidence_path(effective.get("path"), result_path.parent, label)
        != route_path
        or effective.get("sha256") != _sha256(route_path)
        or effective.get("town") != route.get("town")
        or effective.get("scenario") != route.get("scenario")
    ):
        raise ComparisonError(f"{label} effective-route binding changed")
    if (
        _resolved_evidence_path(bound_result.get("path"), result_path.parent, label)
        != result_path
        or bound_result.get("sha256") != _sha256(result_path)
        or bound_result.get("success") is not result.get("success")
        or bound_result.get("execution_mode") != result.get("execution_mode")
        or bound_result.get("reason") != result.get("reason")
        or bound_result.get("profile_context") != EXPECTED_PROFILE_CONTEXT
    ):
        raise ComparisonError(f"{label} route-result binding changed")
    exposure_status = _nested(result, "speed_exposure", "status")
    bound_exposure = bound_result.get("speed_exposure")
    if longitudinal:
        if not isinstance(bound_exposure, Mapping) or (
            bound_exposure.get("status") != exposure_status
        ):
            raise ComparisonError(f"{label} speed-exposure binding changed")
        result_exposure = result["speed_exposure"]
        for key in (
            "minimum_sustained_speed_mps",
            "maximum_observed_speed_limit_mps",
            "maximum_observed_speed_mps",
            "maximum_sustained_speed_duration_sec",
        ):
            _same_number(
                bound_exposure.get(key),
                _number(result_exposure.get(key), f"{label} result {key}"),
                f"{label} bound {key}",
            )
        profile = identity.get("profile")
        if not isinstance(profile, Mapping):
            raise ComparisonError(f"{label} profile identity is missing")
        if (
            profile.get("profile_id") != profile_id
            or profile.get("longitudinal_speed_source")
            != "explicit_simulation_nominal"
            or profile.get("requested_target_is_converter_lookup_velocity") is not False
            or profile.get("converter_lookup_velocity_source")
            != "absolute_current_odometry_longitudinal_speed_mps"
        ):
            raise ComparisonError(f"{label} profile identity changed")
        _same_number(
            profile.get("target_speed_mps"), target_speed_mps, f"{label} bound target"
        )
    elif bound_result.get("speed_exposure_status") != exposure_status:
        raise ComparisonError(f"{label} speed-exposure binding changed")


def _validate_camera_binding(
    trial: Path,
    camera: Mapping[str, Any],
    latency: Mapping[str, Any],
    bag: Mapping[str, Any],
    label: str,
) -> dict[str, dict[str, Any]]:
    if camera.get("contract") != CAMERA_CONTRACT:
        raise ComparisonError(f"{label} camera-source contract changed")
    if _resolved_evidence_path(
        _nested(latency, "inputs", "bag"), trial, f"{label} latency"
    ) != Path(str(bag["root"])):
        raise ComparisonError(f"{label} camera latency is bound to another rosbag")
    bundle = latency.get("camera_bundle")
    acceptance = latency.get("candidate_front_acceptance")
    rates = latency.get("event_rates")
    gaps = camera.get("camera_stamp_gap_sec")
    if not all(
        isinstance(value, Mapping) for value in (bundle, acceptance, rates, gaps)
    ):
        raise ComparisonError(f"{label} camera/latency evidence is incomplete")
    if bundle.get("available") is not True or acceptance.get("available") is not True:
        raise ComparisonError(f"{label} camera/latency evidence is unavailable")
    _same_number(
        camera.get("bundle_coverage_percent"),
        _number(bundle.get("bundle_coverage_percent"), f"{label} bundle coverage"),
        f"{label} camera bundle cross-check",
    )
    _same_number(
        camera.get("candidate_front_acceptance_percent"),
        _number(acceptance.get("acceptance_percent"), f"{label} acceptance"),
        f"{label} camera acceptance cross-check",
    )
    if (
        camera.get("candidate_count") != acceptance.get("candidate_count")
        or camera.get("front_count") != acceptance.get("front_count")
        or camera.get("front_count") != bundle.get("front_frame_count")
        or bundle.get("camera_count") != len(CAMERA_IDS)
    ):
        raise ComparisonError(f"{label} camera counts do not match latency evidence")
    observed_gaps: list[float] = []
    for camera_id in CAMERA_IDS:
        rate = rates.get(f"/sensing/camera/{camera_id}/camera_info")
        if not isinstance(rate, Mapping):
            raise ComparisonError(f"{label} latency lacks {camera_id}")
        gap = _number(
            _nested(rate, "stamp_period_sec", "max"), f"{label} {camera_id} gap"
        )
        _same_number(gaps.get(camera_id), gap, f"{label} {camera_id} gap cross-check")
        observed_gaps.append(gap)
    _same_number(
        camera.get("maximum_camera_stamp_gap_sec"),
        max(observed_gaps),
        f"{label} maximum camera gap",
    )

    mapping = trial / "sensor_mapping_provenance" / "sensor_mapping.yaml"
    sums = trial / "sensor_mapping_provenance" / "SHA256SUMS"
    runtime_env = trial / "runtime.env"
    stack = trial / "stack.log"
    for path in (mapping, sums, runtime_env, stack):
        if path.is_symlink() or not path.is_file():
            raise ComparisonError(f"{label} camera provenance is missing: {path}")
    mapping_sha = _sha256(mapping)
    if camera.get("mapping_sha256") != mapping_sha:
        raise ComparisonError(f"{label} camera mapping digest changed")
    manifest_lines = [line.split() for line in sums.read_text(encoding="utf-8").splitlines()]
    if manifest_lines != [[mapping_sha, "sensor_mapping.yaml"]]:
        raise ComparisonError(f"{label} camera mapping manifest changed")
    runtime_values = {}
    for line in runtime_env.read_text(encoding="utf-8").splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            runtime_values[key] = value
    if (
        runtime_values.get("CAMERA_SOURCE_5HZ") != "true"
        or runtime_values.get("CAMERA_SOURCE_SENSOR_TICK_SEC") != "0.2"
        or runtime_values.get("SENSOR_MAPPING_SHA256") != mapping_sha
    ):
        raise ComparisonError(f"{label} runtime camera mapping binding changed")
    stack_text = stack.read_text(encoding="utf-8", errors="replace")
    inference = re.findall(
        r"published_count=(\d+).*?mailbox_taken=(\d+).*?coalesced_drops=(\d+)",
        stack_text,
    )
    queued = re.findall(
        r"VAD frame queued: source_stamp_ns=\d+.*?assembled=(\d+).*?"
        r"capacity_pruned=(\d+).*?superseded=(\d+).*?"
        r"mailbox_submitted=(\d+).*?coalesced_drops=(\d+).*?"
        r"received_images_min=(\d+).*?received_images_max=(\d+)",
        stack_text,
    )
    counters = camera.get("vad_inference")
    if not inference or not queued or not isinstance(counters, Mapping):
        raise ComparisonError(f"{label} camera stack counters are unavailable")
    published, taken, drops = (int(value) for value in inference[-1])
    assembled, pruned, superseded, submitted, queued_drops, _, _ = (
        int(value) for value in queued[-1]
    )
    expected_counters = {
        "published_count": published,
        "mailbox_taken": taken,
        "mailbox_submitted": submitted,
        "capacity_pruned": pruned,
        "superseded": superseded,
        "coalesced_drops": max(drops, queued_drops),
    }
    if any(counters.get(key) != value for key, value in expected_counters.items()):
        raise ComparisonError(f"{label} camera counters do not match stack evidence")
    if assembled != submitted or published != taken or max(drops, queued_drops) != 0:
        raise ComparisonError(f"{label} camera stack reports dropped/incomplete work")
    return {
        "latency/e2e_latency.json": _file_record(trial / "latency/e2e_latency.json"),
        "sensor_mapping_provenance/sensor_mapping.yaml": _file_record(mapping),
        "sensor_mapping_provenance/SHA256SUMS": _file_record(sums),
        "runtime.env": _file_record(runtime_env),
        "stack.log": _file_record(stack),
    }


def _verify_runtime_record(record: object, expected: Path, label: str) -> None:
    if not isinstance(record, Mapping):
        raise ComparisonError(f"{label} runtime input record is missing")
    if _resolved_evidence_path(record.get("path"), expected.parent, label) != expected:
        raise ComparisonError(f"{label} runtime input path changed")
    if expected.is_symlink() or not expected.is_file():
        raise ComparisonError(f"{label} runtime input is unavailable")
    if (
        record.get("size_bytes") != expected.stat().st_size
        or record.get("sha256") != _sha256(expected)
    ):
        raise ComparisonError(f"{label} runtime input digest changed")


def _validate_runtime_binding(
    runtime: Mapping[str, Any],
    *,
    trial: Path,
    paths: Mapping[str, Path],
    bag: Mapping[str, Any],
    result: Mapping[str, Any],
    label: str,
) -> None:
    manifest = runtime.get("input_manifest")
    runtime_trial = runtime.get("trial")
    if not isinstance(manifest, Mapping) or not isinstance(runtime_trial, Mapping):
        raise ComparisonError(f"{label} runtime-load source identity is missing")
    for key, expected in (
        ("result", paths["result.json"]),
        ("latency", paths["latency/e2e_latency.json"]),
        ("stack", trial / "stack.log"),
    ):
        _verify_runtime_record(manifest.get(key), expected, f"{label} {key}")
    for key, record in manifest.items():
        if key in {"result", "latency", "stack", "bag"}:
            continue
        if not isinstance(record, Mapping) or "files" in record:
            raise ComparisonError(f"{label} runtime input is unsupported: {key}")
        source = _resolved_evidence_path(
            record.get("path"), trial, f"{label} runtime {key}"
        )
        _verify_runtime_record(record, source, f"{label} {key}")
    runtime_bag = manifest.get("bag")
    if not isinstance(runtime_bag, Mapping) or _resolved_evidence_path(
        runtime_bag.get("path"), trial, f"{label} runtime bag"
    ) != Path(str(bag["root"])):
        raise ComparisonError(f"{label} runtime load is bound to another rosbag")
    expected_files = [
        {"name": item["path"], "size_bytes": item["size_bytes"], "sha256": item["sha256"]}
        for item in bag["files"]
    ]
    if runtime_bag.get("files") != expected_files:
        raise ComparisonError(f"{label} runtime rosbag digest changed")
    runtime_schema = runtime.get("schema_version")
    if (
        isinstance(runtime_schema, bool)
        or not isinstance(runtime_schema, int)
        or runtime_schema < 2
        or runtime.get("analysis")
        != "CARLA/VAD pilot runtime-load phase reconstruction"
        or not isinstance(_nested(runtime, "findings", "classification"), str)
        or not isinstance(_nested(runtime, "vad_runtime", "runtime_pattern"), str)
    ):
        raise ComparisonError(f"{label} runtime-load analysis identity changed")
    metrics = result.get("metrics")
    if not isinstance(metrics, Mapping):
        raise ComparisonError(f"{label} result metrics are missing")
    for key in (
        "sim_elapsed_sec",
        "wall_elapsed_sec",
        "maximum_observed_speed_mps",
        "traveled_distance_m",
    ):
        _same_number(runtime_trial.get(key), _number(metrics.get(key), key), f"{label} runtime {key}")
    if (
        runtime_trial.get("success") is not result.get("success")
        or runtime_trial.get("reason") != result.get("reason")
        or runtime_trial.get("started_at") != result.get("started_at")
        or runtime_trial.get("finished_at") != result.get("finished_at")
    ):
        raise ComparisonError(f"{label} runtime trial identity changed")


def _load_trial(trial_dir: Path, role: str) -> dict[str, Any]:
    if role not in TRIAL_CONTRACTS:
        raise ComparisonError(f"unknown comparison role: {role}")
    contract = TRIAL_CONTRACTS[role]
    label = str(contract["label"])
    candidate = trial_dir.expanduser()
    if candidate.is_symlink():
        raise ComparisonError(f"{label} trial directory must not be a symlink")
    trial = candidate.resolve()
    if not trial.is_dir():
        raise ComparisonError(f"{label} trial directory is invalid: {trial}")
    names = (
        "result.json",
        "aligned_route.json",
        "speed_profile.json",
        "longitudinal_response.json",
        "actuation_map_runtime_coverage.json",
        "camera_source_5hz_validation.json",
        "desktop_capture.json",
        "latency/e2e_latency.json",
    )
    paths = {name: trial / name for name in names}
    missing = [
        name for name, path in paths.items() if path.is_symlink() or not path.is_file()
    ]
    if missing:
        raise ComparisonError(f"{label} is missing required evidence: {missing}")

    result = _object(paths["result.json"], f"{label} result")
    route = _object(paths["aligned_route.json"], f"{label} route")
    speed = _object(paths["speed_profile.json"], f"{label} speed analysis")
    longitudinal = _object(
        paths["longitudinal_response.json"], f"{label} longitudinal analysis"
    )
    actuation = _object(
        paths["actuation_map_runtime_coverage.json"],
        f"{label} actuation-map runtime audit",
    )
    camera = _object(
        paths["camera_source_5hz_validation.json"], f"{label} camera audit"
    )
    desktop = _object(paths["desktop_capture.json"], f"{label} desktop audit")
    latency = _object(paths["latency/e2e_latency.json"], f"{label} latency audit")

    result_success = contract["result_success"]
    exposure_status = contract["speed_exposure_status"]
    if (
        result.get("execution_mode") != "full_stack"
        or result.get("profile_context") != EXPECTED_PROFILE_CONTEXT
        or result.get("success") is not result_success
    ):
        raise ComparisonError(f"{label} result identity/outcome changed")
    if speed.get("status") != "complete":
        raise ComparisonError(f"{label} speed analysis is not complete")
    if longitudinal.get("status") != "complete":
        raise ComparisonError(f"{label} longitudinal analysis is not complete")
    for analysis_name, analysis in (
        ("speed", speed),
        ("longitudinal", longitudinal),
    ):
        quality = analysis.get("quality")
        if not isinstance(quality, Mapping) or quality.get("problems") != []:
            raise ComparisonError(f"{label} {analysis_name} analysis has quality problems")
    if camera.get("status") != "PASS":
        raise ComparisonError(f"{label} six-camera audit did not pass")
    view = desktop.get("rviz_view_contract")
    if (
        desktop.get("candidate_observed") is not True
        or not isinstance(view, dict)
        or view.get("vehicle_centered") is not True
    ):
        raise ComparisonError(f"{label} RViz capture is not candidate-backed/centered")

    route_sha = _sha256(paths["aligned_route.json"])
    if route.get("town") != "Town06" or route.get("scenario") != "straight":
        raise ComparisonError(f"{label} is not the contracted Town06 straight route")
    if _resolved_evidence_path(
        result.get("route_file"), paths["result.json"].parent, f"{label} result"
    ) != paths["aligned_route.json"]:
        raise ComparisonError(f"{label} result is bound to another route")
    bag = _bag_manifest(trial / "bag")
    profile_id = str(contract["profile_id"])
    target_contract = float(contract["target_speed_mps"])
    _validate_source_identity(
        speed,
        label=f"{label} speed analysis",
        result=result,
        result_path=paths["result.json"],
        route=route,
        route_path=paths["aligned_route.json"],
        bag=bag,
        profile_id=profile_id,
        target_speed_mps=target_contract,
        longitudinal=False,
    )
    _validate_source_identity(
        longitudinal,
        label=f"{label} longitudinal analysis",
        result=result,
        result_path=paths["result.json"],
        route=route,
        route_path=paths["aligned_route.json"],
        bag=bag,
        profile_id=profile_id,
        target_speed_mps=target_contract,
        longitudinal=True,
    )

    runtime_lookup = actuation.get("runtime_lookup_observation")
    if not isinstance(runtime_lookup, dict) or runtime_lookup.get("available") is not True:
        raise ComparisonError(f"{label} runtime actuation lookup audit is unavailable")

    metrics = result.get("metrics")
    exposure = result.get("speed_exposure")
    final = result.get("final")
    summary = longitudinal.get("summary")
    duty = longitudinal.get("saturation_and_duty")
    inputs = longitudinal.get("inputs")
    if not all(
        isinstance(value, dict)
        for value in (metrics, exposure, final, summary, duty, inputs)
    ):
        raise ComparisonError(f"{label} evidence has incomplete metric objects")

    target_mps = _same_number(
        inputs.get("target_speed_mps"), target_contract, f"{label} target speed"
    )
    maximum_mps = _number(
        metrics.get("maximum_observed_speed_mps"), f"{label} maximum speed"
    )
    wall_elapsed = _number(
        metrics.get("wall_elapsed_sec"), f"{label} wall duration"
    )
    sim_elapsed = _number(metrics.get("sim_elapsed_sec"), f"{label} sim duration")
    if target_mps <= 0.0 or wall_elapsed <= 0.0:
        raise ComparisonError(f"{label} target and wall duration must be positive")
    if (
        exposure.get("status") != exposure_status
        or (exposure_status == "PASS") is not result_success
        or final.get("goal_reached") is not True
        or any(
            exposure.get(key) != value
            for key, value in EXPECTED_PROFILE_CONTEXT.items()
        )
    ):
        raise ComparisonError(f"{label} result/speed-exposure verdict is contradictory")
    _same_number(
        exposure.get("minimum_sustained_speed_mps"),
        float(contract["minimum_sustained_speed_mps"]),
        f"{label} exposure minimum",
    )
    _same_number(
        exposure.get("maximum_observed_speed_limit_mps"),
        float(contract["maximum_observed_speed_limit_mps"]),
        f"{label} exposure maximum",
    )
    _same_number(
        exposure.get("maximum_observed_speed_mps"), maximum_mps, f"{label} result maximum"
    )
    minimum_duration = _number(
        exposure.get("minimum_sustained_speed_sec"), f"{label} exposure duration gate"
    )
    sustained_duration = _number(
        exposure.get("maximum_sustained_speed_duration_sec"),
        f"{label} sustained duration",
    )
    attained_minimum = maximum_mps >= float(contract["minimum_sustained_speed_mps"])
    sustained_minimum = sustained_duration >= minimum_duration
    if (exposure_status == "PASS") != (attained_minimum and sustained_minimum):
        raise ComparisonError(f"{label} speed-exposure measurements contradict status")
    if actuation.get("profile_id") != profile_id or actuation.get("status") != contract[
        "actuation_status"
    ]:
        raise ComparisonError(f"{label} actuation profile/status changed")
    _same_number(
        actuation.get("target_speed_mps"), target_mps, f"{label} actuation target"
    )
    _same_number(
        runtime_lookup.get("maximum_absolute_current_speed_mps"),
        maximum_mps,
        f"{label} actuation runtime maximum",
    )
    actuation_source = _nested(longitudinal, "actuation_map_coverage", "source")
    if (
        not isinstance(actuation_source, Mapping)
        or _resolved_evidence_path(
            actuation_source.get("path"), trial, f"{label} actuation source"
        )
        != paths["actuation_map_runtime_coverage.json"]
        or actuation_source.get("sha256")
        != _sha256(paths["actuation_map_runtime_coverage.json"])
        or _nested(
            longitudinal,
            "actuation_map_coverage",
            "runtime_cross_check",
            "consistent_with_bag",
        )
        is not True
    ):
        raise ComparisonError(f"{label} actuation audit is not bound to this bag/result")

    auxiliary_records = _validate_camera_binding(trial, camera, latency, bag, label)

    def duty_record(name: str) -> dict[str, Any]:
        record = duty.get(name)
        if not isinstance(record, dict) or record.get("available") is not True:
            raise ComparisonError(f"{label} duty metric is unavailable: {name}")
        return {
            "time_fraction_percent": record.get("time_fraction_percent"),
            "sample_fraction_percent": record.get("sample_fraction_percent"),
            "longest_contiguous_duration_sec": record.get(
                "longest_contiguous_duration_sec"
            ),
        }

    runtime_path = trial / "runtime_load_analysis.json"
    runtime: dict[str, Any] | None = None
    if runtime_path.is_symlink():
        raise ComparisonError(f"{label} runtime-load analysis must not be a symlink")
    if runtime_path.is_file():
        runtime_payload = _object(runtime_path, f"{label} runtime-load analysis")
        _validate_runtime_binding(
            runtime_payload,
            trial=trial,
            paths=paths,
            bag=bag,
            result=result,
            label=label,
        )
        runtime = {
            "source": _file_record(runtime_path),
            "runtime_pattern": _nested(
                runtime_payload, "vad_runtime", "runtime_pattern"
            ),
            "phase_detection": _nested(
                runtime_payload, "vad_runtime", "phase_detection"
            ),
            "findings_classification": _nested(
                runtime_payload, "findings", "classification"
            ),
        }

    return {
        "label": label,
        "trial_directory": str(trial),
        "profile_id": profile_id,
        "target_speed_mps": target_mps,
        "target_speed_kph": target_mps * 3.6,
        "trial_verdict": "PASS" if result.get("success") is True else "FAIL",
        "reason": result.get("reason"),
        "goal_reached": final.get("goal_reached"),
        "speed_exposure_status": exposure.get("status"),
        "route": {
            "sha256": route_sha,
            "town": route.get("town"),
            "scenario": route.get("scenario"),
            "route_length_m": route.get("route_length_m"),
        },
        "motion": {
            "maximum_observed_speed_mps": maximum_mps,
            "maximum_observed_speed_kph": maximum_mps * 3.6,
            "target_attainment_percent": maximum_mps / target_mps * 100.0,
            "maximum_sustained_speed_duration_sec": metrics.get(
                "maximum_sustained_speed_duration_sec"
            ),
            "maximum_absolute_cte_m": metrics.get("maximum_absolute_cte_m"),
            "maximum_lateral_acceleration_mps2": metrics.get(
                "maximum_lateral_acceleration_mps2"
            ),
            "traveled_distance_m": metrics.get("traveled_distance_m"),
            "sim_elapsed_sec": sim_elapsed,
            "wall_elapsed_sec": wall_elapsed,
            "real_time_factor": sim_elapsed / wall_elapsed,
        },
        "longitudinal": {
            "actual_speed_mps": summary.get("actual_speed_mps"),
            "raw_target_speed_mps": summary.get("raw_target_speed_mps"),
            "gated_target_speed_mps": summary.get("gated_target_speed_mps"),
            "raw_acceleration_above_gate_limit": duty_record(
                "raw_acceleration_above_gate_limit"
            ),
            "gated_positive_acceleration_limit": duty_record(
                "gated_positive_acceleration_limit"
            ),
            "accel_command_near_saturation": duty_record(
                "accel_command_near_saturation"
            ),
            "brake_command_active": duty_record("brake_command_active"),
        },
        "actuation_map": {
            "status": actuation.get("status"),
            "target_envelope_classification": actuation.get(
                "target_envelope_classification"
            ),
            "runtime_lookup_classification": runtime_lookup.get("classification"),
            "velocity_axis_clamping_observed": runtime_lookup.get(
                "velocity_axis_clamping_observed"
            ),
            "converter_lookup_velocity_source": _nested(
                actuation,
                "validation_boundary",
                "converter_lookup_velocity_source",
            ),
        },
        "camera": {
            "status": camera.get("status"),
            "bundle_coverage_percent": camera.get("bundle_coverage_percent"),
            "maximum_camera_stamp_gap_sec": camera.get(
                "maximum_camera_stamp_gap_sec"
            ),
            "candidate_front_acceptance_percent": camera.get(
                "candidate_front_acceptance_percent"
            ),
        },
        "rviz": {
            "candidate_observed": True,
            "vehicle_centered": True,
            "png_dimensions": desktop.get("png_dimensions"),
            "gif_dimensions": desktop.get("gif_dimensions"),
        },
        "runtime_load": runtime,
        "source_records": {
            **{name: _file_record(path) for name, path in paths.items()},
            **auxiliary_records,
        },
        "rosbag_identity": bag,
    }


def _validate_trial_contract(trial: Mapping[str, Any], role: str) -> None:
    contract = TRIAL_CONTRACTS[role]
    label = str(contract["label"])
    expected_verdict = "PASS" if contract["result_success"] else "FAIL"
    if (
        trial.get("label") != label
        or trial.get("profile_id") != contract["profile_id"]
        or trial.get("trial_verdict") != expected_verdict
        or trial.get("speed_exposure_status")
        != contract["speed_exposure_status"]
        or trial.get("goal_reached") is not True
    ):
        raise ComparisonError(f"{label} strict identity/outcome contract changed")
    _same_number(
        trial.get("target_speed_mps"),
        float(contract["target_speed_mps"]),
        f"{label} comparison target",
    )
    _same_number(
        trial.get("target_speed_kph"),
        float(contract["target_speed_mps"]) * 3.6,
        f"{label} comparison target kph",
    )
    route = trial.get("route")
    if not isinstance(route, Mapping) or (
        route.get("town") != "Town06" or route.get("scenario") != "straight"
    ):
        raise ComparisonError(f"{label} route identity changed")
    route_length = _number(route.get("route_length_m"), f"{label} route length")
    if route_length <= 0.0:
        raise ComparisonError(f"{label} route length must be positive")


def _validate_pair(reference: Mapping[str, Any], pilot: Mapping[str, Any]) -> None:
    _validate_trial_contract(reference, "reference")
    _validate_trial_contract(pilot, "pilot")
    reference_route = reference["route"]
    pilot_route = pilot["route"]
    if reference_route["sha256"] != pilot_route["sha256"]:
        raise ComparisonError("effective routes are not byte-identical")
    if (
        reference_route.get("town") != pilot_route.get("town")
        or reference_route.get("scenario") != pilot_route.get("scenario")
    ):
        raise ComparisonError("effective route town/scenario identity differs")
    if reference_route.get("scenario") != "straight":
        raise ComparisonError("this speed comparison requires a straight route")
    _same_number(
        pilot_route.get("route_length_m"),
        _number(reference_route.get("route_length_m"), "reference route length"),
        "pilot route length",
    )


def build_comparison(reference: dict[str, Any], pilot: dict[str, Any]) -> dict[str, Any]:
    _validate_pair(reference, pilot)
    reference_route = reference["route"]

    reference_rtf = reference["motion"]["real_time_factor"]
    pilot_rtf = pilot["motion"]["real_time_factor"]
    reference_max = reference["motion"]["maximum_observed_speed_mps"]
    pilot_max = pilot["motion"]["maximum_observed_speed_mps"]
    reference_pass = True
    pilot_pass = False
    low_target_slower_rtf = (
        reference["target_speed_mps"] < pilot["target_speed_mps"]
        and reference_rtf < pilot_rtf
        and reference_rtf < 0.5
    )
    return {
        "schema_version": 1,
        "analysis": "same_route_autoware_speed_trial_comparison",
        "status": "complete",
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "comparison_id": "town06_same_effective_route_30_vs_60_v1",
        "strict_contract": {
            "reference_profile_id": TRIAL_CONTRACTS["reference"]["profile_id"],
            "pilot_profile_id": TRIAL_CONTRACTS["pilot"]["profile_id"],
            "reference_outcome": "PASS",
            "pilot_outcome": "FAIL_SPEED_EXPOSURE",
        },
        "real_vehicle_ready": False,
        "same_route": {
            "verified": True,
            **reference_route,
        },
        "trials": {
            "reference_30kph": reference,
            "pilot_60kph": pilot,
        },
        "comparison": {
            "reference_speed_exposure_passed": reference_pass,
            "pilot_speed_exposure_passed": pilot_pass,
            "pilot_goal_reached_despite_speed_failure": (
                pilot.get("goal_reached") is True and not pilot_pass
            ),
            "maximum_observed_speed_increase_mps": pilot_max - reference_max,
            "maximum_observed_speed_increase_kph": (
                pilot_max - reference_max
            )
            * 3.6,
            "pilot_target_shortfall_mps": pilot["target_speed_mps"] - pilot_max,
            "pilot_target_shortfall_kph": (
                pilot["target_speed_mps"] - pilot_max
            )
            * 3.6,
            "reference_real_time_factor": reference_rtf,
            "pilot_real_time_factor": pilot_rtf,
            "lower_target_run_was_slower_in_wall_time": low_target_slower_rtf,
        },
        "findings": {
            "classification": "60KPH_TARGET_NOT_REACHED_REFERENCE_30KPH_PASS",
            "target_speed_alone_does_not_explain_low_rtf": {
                "supported": low_target_slower_rtf,
                "reason": (
                    "The byte-identical lower-target reference run had the lower RTF. "
                    "This excludes requested target speed as a sufficient explanation, "
                    "but does not identify a single renderer/transport cause."
                ),
            },
            "actuation_velocity_axis_clamping_observed": any(
                trial["actuation_map"]["velocity_axis_clamping_observed"] is True
                for trial in (reference, pilot)
            ),
            "interpretation": (
                "The requested speed is an explicit simulation profile envelope. "
                "Raw VAD cruise velocity and real-vehicle readiness are not evaluated."
            ),
        },
    }


def _atomic_json(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, staged_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".staged", dir=path.parent
    )
    staged = Path(staged_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(staged, path)
    finally:
        staged.unlink(missing_ok=True)


def _plot(report: Mapping[str, Any], output: Path) -> None:
    labels = ("30 kph reference", "60 kph pilot")
    trials = (
        report["trials"]["reference_30kph"],
        report["trials"]["pilot_60kph"],
    )
    figure, axes = plt.subplots(2, 2, figsize=(16, 9), constrained_layout=True)
    figure.suptitle(
        "Town06 byte-identical route: 30 vs 60 kph Autoware/CARLA",
        fontsize=17,
        fontweight="bold",
    )
    x = (0, 1)
    targets = [trial["target_speed_kph"] for trial in trials]
    observed = [trial["motion"]["maximum_observed_speed_kph"] for trial in trials]
    axes[0, 0].bar([item - 0.18 for item in x], targets, 0.36, label="target")
    axes[0, 0].bar([item + 0.18 for item in x], observed, 0.36, label="observed max")
    axes[0, 0].set_xticks(x, labels)
    axes[0, 0].set_ylabel("speed [kph]")
    axes[0, 0].set_title("Speed target and measured maximum")
    axes[0, 0].grid(axis="y", alpha=0.25)
    axes[0, 0].legend()

    rtf = [trial["motion"]["real_time_factor"] for trial in trials]
    axes[0, 1].bar(x, rtf, color=("#2980b9", "#e67e22"))
    axes[0, 1].axhline(1.0, color="#333333", linestyle="--", linewidth=1)
    axes[0, 1].set_xticks(x, labels)
    axes[0, 1].set_ylabel("simulation / wall")
    axes[0, 1].set_title("Aggregate real-time factor")
    axes[0, 1].grid(axis="y", alpha=0.25)

    metric_names = (
        "gated_positive_acceleration_limit",
        "accel_command_near_saturation",
        "brake_command_active",
    )
    metric_labels = ("gate +accel limit", "throttle near max", "brake active")
    positions = range(len(metric_names))
    width = 0.36
    for index, trial in enumerate(trials):
        values = [
            trial["longitudinal"][name]["time_fraction_percent"]
            for name in metric_names
        ]
        axes[1, 0].bar(
            [position + (index - 0.5) * width for position in positions],
            values,
            width,
            label=labels[index],
        )
    axes[1, 0].set_xticks(list(positions), metric_labels)
    axes[1, 0].set_ylabel("eligible simulation time [%]")
    axes[1, 0].set_title("Longitudinal gate and pedal duty")
    axes[1, 0].grid(axis="y", alpha=0.25)
    axes[1, 0].legend()

    reference, pilot = trials
    text = (
        "Verdict\n"
        f"30 kph: {reference['trial_verdict']} | "
        f"max {reference['motion']['maximum_observed_speed_kph']:.2f} kph\n"
        f"60 kph: {pilot['trial_verdict']} | "
        f"max {pilot['motion']['maximum_observed_speed_kph']:.2f} kph\n"
        f"60 target attainment: {pilot['motion']['target_attainment_percent']:.2f}%\n"
        f"60 goal reached: {pilot['goal_reached']}\n\n"
        "Bounded conclusion\n"
        "The lower-target run was slower in wall time, so target speed alone\n"
        "is not a sufficient explanation for stutter. Both camera audits pass.\n"
        "No observed converter velocity-axis clamp. Simulation-only evidence."
    )
    axes[1, 1].axis("off")
    axes[1, 1].text(
        0.02,
        0.98,
        text,
        va="top",
        family="monospace",
        fontsize=11,
        bbox={"boxstyle": "round", "facecolor": "#f6f6f6"},
    )
    figure.savefig(output, format="png", dpi=120, facecolor="white")
    plt.close(figure)


def _atomic_plot(path: Path, report: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, staged_name = tempfile.mkstemp(
        prefix=f".{path.stem}.", suffix=".png", dir=path.parent
    )
    os.close(descriptor)
    staged = Path(staged_name)
    try:
        _plot(report, staged)
        with staged.open("rb") as stream:
            os.fsync(stream.fileno())
        os.replace(staged, path)
    finally:
        staged.unlink(missing_ok=True)


def _pilot_summary(report: Mapping[str, Any], comparison_json: Path) -> dict[str, Any]:
    trials = report.get("trials")
    if not isinstance(trials, Mapping):
        raise ComparisonError("comparison report has no trials")
    reference = trials.get("reference_30kph")
    pilot = trials.get("pilot_60kph")
    if not isinstance(reference, Mapping) or not isinstance(pilot, Mapping):
        raise ComparisonError("comparison report has incomplete trials")
    _validate_pair(reference, pilot)
    comparison = report.get("comparison")
    findings = report.get("findings")
    if (
        report.get("status") != "complete"
        or report.get("analysis") != "same_route_autoware_speed_trial_comparison"
        or report.get("comparison_id") != "town06_same_effective_route_30_vs_60_v1"
        or not isinstance(comparison, Mapping)
        or comparison.get("reference_speed_exposure_passed") is not True
        or comparison.get("pilot_speed_exposure_passed") is not False
        or not isinstance(findings, Mapping)
        or findings.get("classification")
        != "60KPH_TARGET_NOT_REACHED_REFERENCE_30KPH_PASS"
    ):
        raise ComparisonError("comparison report contradicts the strict verdict")
    comparison_json = comparison_json.expanduser().resolve()
    if _object(comparison_json, "comparison report") != dict(report):
        raise ComparisonError("pilot summary source does not match the comparison report")
    return {
        "schema_version": 1,
        "status": "EVIDENCE_COMPLETE_TRIAL_FAILED_SPEED_EXPOSURE",
        "trial_verdict": "FAIL",
        "profile_id": pilot["profile_id"],
        "real_vehicle_ready": False,
        "reason": pilot["reason"],
        "goal_reached": pilot["goal_reached"],
        "speed_exposure_status": pilot["speed_exposure_status"],
        "summary": pilot["motion"],
        "actuation_map": pilot["actuation_map"],
        "camera": pilot["camera"],
        "rviz": pilot["rviz"],
        "same_route_reference": report["trials"]["reference_30kph"][
            "trial_directory"
        ],
        "comparison": _file_record(comparison_json),
        "supersedes_for_interpretation": {
            "file": "pilot_run.json",
            "reason": (
                "The original wrapper summary predates failure-tolerant speed analysis "
                "and the corrected current-odometry actuation lookup audit."
            ),
            "original_file_preserved": True,
        },
    }


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference-trial", required=True, type=Path)
    parser.add_argument("--pilot-trial", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--pilot-summary-output", type=Path)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    reference = _load_trial(args.reference_trial, "reference")
    pilot = _load_trial(args.pilot_trial, "pilot")
    report = build_comparison(reference, pilot)
    output = args.output_dir.expanduser().resolve()
    json_path = output / "same_route_30_vs_60.json"
    plot_path = output / "same_route_30_vs_60.png"
    _atomic_json(json_path, report)
    _atomic_plot(plot_path, report)
    if args.pilot_summary_output:
        _atomic_json(
            args.pilot_summary_output.expanduser().resolve(),
            _pilot_summary(report, json_path),
        )
    print(f"speed comparison JSON: {json_path}")
    print(f"speed comparison plot: {plot_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
