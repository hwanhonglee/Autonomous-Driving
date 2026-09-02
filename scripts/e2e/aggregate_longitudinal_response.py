#!/usr/bin/env python3
"""Aggregate fail-closed longitudinal-response evidence for a Town matrix.

Only PASS attempts selected by the matrix ``aggregate.json`` are admitted.
The output remains a response analysis rather than an actuation-calibration
claim: the cruise target is a profile envelope, while converter CSV lookup
uses absolute current odometry speed.
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import io
import json
import math
import os
from pathlib import Path
import struct
import sys
import tempfile
from typing import Any, Sequence

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402


OUTPUT_BASENAME = "matrix_longitudinal_response"
TRIAL_IDS = ("straight", "turn")
ANALYSIS_ID = "carla_longitudinal_response_matrix"
INDIVIDUAL_ANALYSIS_ID = "carla_longitudinal_response"
EXPECTED_LOOKUP_SOURCE = "absolute_current_odometry_longitudinal_speed_mps"
CSV_COLUMNS = (
    "map_id",
    "canonical_name",
    "trial_id",
    "route_scenario",
    "attempt_directory",
    "route_length_m",
    "target_speed_mps",
    "target_speed_kph",
    "actual_maximum_speed_mps",
    "actual_maximum_speed_kph",
    "maximum_target_attainment_percent",
    "contract_minimum_speed_mps",
    "contract_required_duration_sec",
    "contract_observed_duration_sec",
    "gate_accel_limit_sample_percent",
    "gate_accel_limit_time_percent",
    "gate_accel_limit_longest_sec",
    "raw_accel_above_gate_sample_percent",
    "raw_accel_above_gate_time_percent",
    "raw_accel_above_gate_longest_sec",
    "throttle_near_saturation_sample_percent",
    "throttle_near_saturation_time_percent",
    "throttle_near_saturation_longest_sec",
    "brake_active_sample_percent",
    "brake_active_time_percent",
    "brake_active_longest_sec",
    "robust_acceleration_minimum_mps2",
    "robust_acceleration_p95_mps2",
    "robust_acceleration_maximum_mps2",
    "measured_acceleration_outlier_count",
    "measured_acceleration_outlier_percent",
    "gated_target_minus_actual_mean_mps",
    "gated_target_minus_actual_rmse_mps",
    "gated_target_minus_actual_p95_abs_mps",
    "gated_target_minus_actual_max_abs_mps",
    "accel_command_status_rmse",
    "brake_command_status_rmse",
    "map_axis_maximum_mps",
    "runtime_lookup_classification",
    "runtime_velocity_clamping_observed",
    "analysis_json_sha256",
    "analysis_png_sha256",
)

SUMMARY_METRICS = (
    "actual_maximum_speed_mps",
    "maximum_target_attainment_percent",
    "gate_accel_limit_sample_percent",
    "gate_accel_limit_time_percent",
    "gate_accel_limit_longest_sec",
    "raw_accel_above_gate_sample_percent",
    "throttle_near_saturation_sample_percent",
    "brake_active_sample_percent",
    "gated_target_minus_actual_rmse_mps",
    "measured_acceleration_outlier_percent",
)


class AggregateError(RuntimeError):
    """Raised when selected matrix evidence is absent or inconsistent."""


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
    except OSError as error:
        raise AggregateError(f"cannot hash {path}: {error}") from error
    return digest.hexdigest()


def _sha256_json(value: Any) -> str:
    try:
        encoded = json.dumps(
            value,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
    except (TypeError, ValueError) as error:
        raise AggregateError(f"cannot canonicalize JSON: {error}") from error
    return hashlib.sha256(encoded).hexdigest()


def _required_file(path: Path, label: str) -> Path:
    if path.is_symlink() or not path.is_file():
        raise AggregateError(f"missing or unsafe {label}: {path}")
    return path


def _read_object(path: Path, label: str) -> dict[str, Any]:
    _required_file(path, label)
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise AggregateError(f"cannot read {label} {path}: {error}") from error
    if not isinstance(value, dict):
        raise AggregateError(f"{label} must contain a JSON object: {path}")
    return value


def _number(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise AggregateError(f"{label} must be numeric")
    output = float(value)
    if not math.isfinite(output):
        raise AggregateError(f"{label} must be finite")
    return output


def _integer(value: object, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise AggregateError(f"{label} must be a non-negative integer")
    return value


def _mapping(value: object, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise AggregateError(f"{label} must be an object")
    return value


def _path_inside(root: Path, declared: object, label: str) -> Path:
    if not isinstance(declared, str) or not declared:
        raise AggregateError(f"{label} path is missing")
    candidate = Path(declared).expanduser()
    if not candidate.is_absolute():
        candidate = root / candidate
    if candidate.is_symlink():
        raise AggregateError(f"{label} path must not be a symlink: {candidate}")
    resolved = candidate.resolve()
    try:
        resolved.relative_to(root)
    except ValueError as error:
        raise AggregateError(f"{label} path escapes matrix root: {resolved}") from error
    if not resolved.is_dir():
        raise AggregateError(f"{label} directory does not exist: {resolved}")
    return resolved


def _png_dimensions(path: Path) -> tuple[int, int]:
    _required_file(path, "longitudinal response PNG")
    try:
        with path.open("rb") as stream:
            header = stream.read(24)
    except OSError as error:
        raise AggregateError(f"cannot read PNG {path}: {error}") from error
    if (
        len(header) != 24
        or header[:8] != b"\x89PNG\r\n\x1a\n"
        or header[12:16] != b"IHDR"
    ):
        raise AggregateError(f"invalid PNG header: {path}")
    width, height = struct.unpack(">II", header[16:24])
    if width < 1200 or height < 900:
        raise AggregateError(
            f"longitudinal response PNG is unexpectedly small: {width}x{height}"
        )
    return width, height


def _assert_close(actual: float, expected: float, label: str) -> None:
    if not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1.0e-9):
        raise AggregateError(f"{label} changed: expected {expected}, got {actual}")


def _duty(
    response: dict[str, Any], key: str, label: str
) -> dict[str, Any]:
    duty_root = _mapping(response.get("saturation_and_duty"), "duty metrics")
    value = _mapping(duty_root.get(key), label)
    if value.get("available") is not True:
        raise AggregateError(f"{label} is unavailable")
    return value


def _tracking(
    response: dict[str, Any], key: str, label: str
) -> dict[str, Any]:
    tracking_root = _mapping(response.get("tracking"), "tracking metrics")
    value = _mapping(tracking_root.get(key), label)
    if value.get("available") is not True:
        raise AggregateError(f"{label} is unavailable")
    return value


def _validate_rosbag_identity(attempt: Path, bag: dict[str, Any]) -> None:
    root_value = bag.get("root")
    if not isinstance(root_value, str) or not root_value:
        raise AggregateError(f"{attempt}: rosbag root is missing")
    candidate = Path(root_value).expanduser()
    if candidate.is_symlink():
        raise AggregateError(f"{attempt}: rosbag root must not be a symlink")
    bag_root = candidate.resolve()
    if bag_root != (attempt / "bag").resolve():
        raise AggregateError(f"{attempt}: rosbag root is not attempt-local")
    if not bag_root.is_dir():
        raise AggregateError(f"{attempt}: rosbag root does not exist")

    schema_version = bag.get("schema_version")
    if (
        isinstance(schema_version, bool)
        or not isinstance(schema_version, int)
        or schema_version != 1
    ):
        raise AggregateError(f"{attempt}: unsupported rosbag manifest schema")
    declared_files = bag.get("files")
    if not isinstance(declared_files, list) or not declared_files:
        raise AggregateError(f"{attempt}: rosbag manifest has no files")
    if not all(isinstance(record, dict) for record in declared_files):
        raise AggregateError(f"{attempt}: rosbag manifest file record is invalid")

    expected_manifest_sha = bag.get("sha256")
    declared_manifest_sha = _sha256_json(
        {"schema_version": schema_version, "files": declared_files}
    )
    if (
        not isinstance(expected_manifest_sha, str)
        or expected_manifest_sha != declared_manifest_sha
    ):
        raise AggregateError(f"{attempt}: rosbag manifest SHA-256 is invalid")

    actual_files: list[dict[str, Any]] = []
    try:
        descendants = sorted(bag_root.rglob("*"))
    except OSError as error:
        raise AggregateError(
            f"{attempt}: cannot enumerate rosbag evidence: {error}"
        ) from error
    for path in descendants:
        if path.is_symlink():
            raise AggregateError(
                f"{attempt}: rosbag evidence contains a symlink: {path}"
            )
        if not path.is_file():
            continue
        try:
            size_bytes = path.stat().st_size
        except OSError as error:
            raise AggregateError(
                f"{attempt}: cannot stat rosbag file {path}: {error}"
            ) from error
        actual_files.append(
            {
                "path": path.relative_to(bag_root).as_posix(),
                "size_bytes": size_bytes,
                "sha256": _sha256_file(path),
            }
        )
    if not actual_files or not any(
        record["path"] == "metadata.yaml" for record in actual_files
    ):
        raise AggregateError(f"{attempt}: rosbag evidence has no metadata.yaml")

    declared_by_path: dict[str, dict[str, Any]] = {}
    for record in declared_files:
        relative = record.get("path")
        if (
            not isinstance(relative, str)
            or not relative
            or Path(relative).is_absolute()
            or Path(relative).as_posix() != relative
            or ".." in Path(relative).parts
            or relative in declared_by_path
        ):
            raise AggregateError(
                f"{attempt}: rosbag manifest contains an invalid file path"
            )
        declared_by_path[relative] = record
    actual_by_path = {record["path"]: record for record in actual_files}
    if declared_by_path.keys() != actual_by_path.keys():
        raise AggregateError(f"{attempt}: rosbag file inventory changed")
    for relative, actual in actual_by_path.items():
        declared = declared_by_path[relative]
        if declared.get("size_bytes") != actual["size_bytes"]:
            raise AggregateError(
                f"{attempt}: rosbag file size changed: {relative}"
            )
        if declared.get("sha256") != actual["sha256"]:
            raise AggregateError(
                f"{attempt}: rosbag file SHA-256 changed: {relative}"
            )
    if declared_files != actual_files:
        raise AggregateError(f"{attempt}: rosbag file manifest changed")
    actual_manifest_sha = _sha256_json(
        {"schema_version": schema_version, "files": actual_files}
    )
    if expected_manifest_sha != actual_manifest_sha:
        raise AggregateError(f"{attempt}: rosbag manifest SHA-256 changed")


def _validate_source_files(
    attempt: Path, source: dict[str, Any], trial_id: str
) -> tuple[str, float]:
    expected_identity_sha = source.get("sha256")
    if not isinstance(expected_identity_sha, str) or expected_identity_sha != (
        _sha256_json({key: value for key, value in source.items() if key != "sha256"})
    ):
        raise AggregateError(f"{attempt}: source identity SHA-256 is invalid")

    route = _mapping(source.get("effective_route"), "effective route identity")
    route_path = Path(str(route.get("path", ""))).expanduser().resolve()
    if route_path != (attempt / "aligned_route.json").resolve():
        raise AggregateError(f"{attempt}: effective route path is not attempt-local")
    _required_file(route_path, "effective route")
    if route.get("sha256") != _sha256_file(route_path):
        raise AggregateError(f"{attempt}: effective route SHA-256 changed")
    route_scenario = route.get("scenario")
    if trial_id == "straight" and route_scenario != "straight":
        raise AggregateError(f"{attempt}: straight trial has non-straight route")
    if trial_id == "turn" and route_scenario not in {"left", "right"}:
        raise AggregateError(f"{attempt}: turn trial lacks left/right route")
    if route.get("trial_id") != trial_id:
        raise AggregateError(f"{attempt}: source trial id does not match matrix")

    result = _mapping(source.get("route_result"), "route result identity")
    result_path = Path(str(result.get("path", ""))).expanduser().resolve()
    if result_path != (attempt / "result.json").resolve():
        raise AggregateError(f"{attempt}: route result path is not attempt-local")
    _required_file(result_path, "route result")
    if result.get("sha256") != _sha256_file(result_path):
        raise AggregateError(f"{attempt}: route result SHA-256 changed")
    exposure = _mapping(result.get("speed_exposure"), "route speed exposure")
    if result.get("success") is not True or exposure.get("status") != "PASS":
        raise AggregateError(f"{attempt}: selected response is not a PASS trial")

    bag = _mapping(source.get("rosbag"), "rosbag identity")
    _validate_rosbag_identity(attempt, bag)
    route_length = _number(route.get("route_length_m"), "route length")
    return str(route_scenario), route_length


def _extract_row(
    matrix_root: Path,
    map_record: dict[str, Any],
    trial_id: str,
    trial: dict[str, Any],
    *,
    profile_id: str,
    target_speed_mps: float,
    expected_longitudinal_source: str,
) -> dict[str, Any]:
    map_id = map_record.get("map_id")
    canonical_name = map_record.get("canonical_name")
    if not isinstance(map_id, str) or not isinstance(canonical_name, str):
        raise AggregateError("PASS map lacks map identity")
    if trial.get("status") != "PASS":
        raise AggregateError(f"{map_id}/{trial_id}: selected trial is not PASS")
    attempt = _path_inside(
        matrix_root, trial.get("attempt_directory"), f"{map_id}/{trial_id} attempt"
    )
    response_path = attempt / "longitudinal_response.json"
    plot_path = attempt / "longitudinal_response.png"
    response = _read_object(response_path, "longitudinal response")
    dimensions = _png_dimensions(plot_path)
    if (
        response.get("analysis") != INDIVIDUAL_ANALYSIS_ID
        or response.get("status") != "complete"
    ):
        raise AggregateError(f"{map_id}/{trial_id}: response is not complete")
    quality = _mapping(response.get("quality"), "response quality")
    if quality.get("problems") != [] or quality.get("warnings") != []:
        raise AggregateError(
            f"{map_id}/{trial_id}: response contains quality findings"
        )

    inputs = _mapping(response.get("inputs"), "response inputs")
    if inputs.get("profile_id") != profile_id:
        raise AggregateError(f"{map_id}/{trial_id}: profile id changed")
    _assert_close(
        _number(inputs.get("target_speed_mps"), "response target speed"),
        target_speed_mps,
        f"{map_id}/{trial_id} target speed",
    )
    if inputs.get("longitudinal_speed_source") != expected_longitudinal_source:
        raise AggregateError(f"{map_id}/{trial_id}: longitudinal source changed")

    source = _mapping(response.get("source_identity"), "response source identity")
    profile = _mapping(source.get("profile"), "source profile identity")
    if (
        profile.get("profile_id") != profile_id
        or profile.get("requested_target_is_converter_lookup_velocity") is not False
        or profile.get("converter_lookup_velocity_source") != EXPECTED_LOOKUP_SOURCE
    ):
        raise AggregateError(f"{map_id}/{trial_id}: source profile is invalid")
    route_scenario, route_length = _validate_source_files(attempt, source, trial_id)
    route_identity = _mapping(source.get("effective_route"), "route identity")
    if route_identity.get("town") != canonical_name:
        raise AggregateError(
            f"{map_id}/{trial_id}: route town does not match {canonical_name}"
        )

    coverage = _mapping(
        response.get("actuation_map_coverage"), "actuation map coverage"
    )
    runtime = _mapping(
        coverage.get("runtime_lookup_observation"), "runtime lookup observation"
    )
    cross_check = _mapping(coverage.get("runtime_cross_check"), "map cross-check")
    if (
        coverage.get("provided") is not True
        or coverage.get("status") != "PASS"
        or coverage.get("target_within_map_velocity_axis") is not True
        or runtime.get("classification") != "OBSERVED_LOOKUPS_WITHIN_MAP_AXIS"
        or runtime.get("velocity_axis_clamping_observed") is not False
        or cross_check.get("consistent_with_bag") is not True
    ):
        raise AggregateError(f"{map_id}/{trial_id}: map coverage is not PASS")
    interpretation = _mapping(response.get("interpretation"), "interpretation")
    if interpretation.get("converter_lookup_velocity_source") != EXPECTED_LOOKUP_SOURCE:
        raise AggregateError(f"{map_id}/{trial_id}: lookup semantics changed")

    summary = _mapping(response.get("summary"), "response summary")
    actual = _mapping(summary.get("actual_speed_mps"), "actual speed summary")
    robust = _mapping(
        summary.get("robust_measured_acceleration_mps2"),
        "robust acceleration summary",
    )
    robust_quality = _mapping(
        response.get("robust_measured_acceleration"),
        "robust acceleration quality",
    )
    exposure_root = _mapping(response.get("target_exposure"), "target exposure")
    exposure_reported = _mapping(
        exposure_root.get("route_result_reported"), "reported exposure"
    )
    exposure_check = _mapping(
        exposure_root.get("route_result_cross_check"), "exposure cross-check"
    )
    if exposure_check.get("maximum_speed_consistent_with_bag") is not True:
        raise AggregateError(f"{map_id}/{trial_id}: result speed does not match bag")

    gate = _duty(
        response, "gated_positive_acceleration_limit", "gate saturation"
    )
    raw_above = _duty(
        response, "raw_acceleration_above_gate_limit", "raw gate clipping demand"
    )
    throttle = _duty(
        response, "accel_command_near_saturation", "throttle saturation"
    )
    brake = _duty(response, "brake_command_active", "brake activity")
    target_tracking = _tracking(
        response,
        "gated_target_minus_actual_speed_mps",
        "gated target tracking",
    )
    accel_tracking = _tracking(
        response, "accel_command_minus_status", "accel command tracking"
    )
    brake_tracking = _tracking(
        response, "brake_command_minus_status", "brake command tracking"
    )
    actual_maximum = _number(actual.get("maximum"), "actual maximum speed")

    row = {
        "map_id": map_id,
        "canonical_name": canonical_name,
        "trial_id": trial_id,
        "route_scenario": route_scenario,
        "attempt_directory": attempt.relative_to(matrix_root).as_posix(),
        "route_length_m": route_length,
        "target_speed_mps": target_speed_mps,
        "target_speed_kph": target_speed_mps * 3.6,
        "actual_maximum_speed_mps": actual_maximum,
        "actual_maximum_speed_kph": actual_maximum * 3.6,
        "maximum_target_attainment_percent": (
            actual_maximum / target_speed_mps * 100.0
        ),
        "contract_minimum_speed_mps": _number(
            exposure_reported.get("minimum_sustained_speed_mps"),
            "contract minimum speed",
        ),
        "contract_required_duration_sec": _number(
            exposure_reported.get("minimum_sustained_speed_sec"),
            "contract required duration",
        ),
        "contract_observed_duration_sec": _number(
            exposure_reported.get("maximum_sustained_speed_duration_sec"),
            "contract observed duration",
        ),
        "gate_accel_limit_sample_percent": _number(
            gate.get("sample_fraction_percent"), "gate sample duty"
        ),
        "gate_accel_limit_time_percent": _number(
            gate.get("time_fraction_percent"), "gate time duty"
        ),
        "gate_accel_limit_longest_sec": _number(
            gate.get("longest_contiguous_duration_sec"), "gate longest duration"
        ),
        "raw_accel_above_gate_sample_percent": _number(
            raw_above.get("sample_fraction_percent"), "raw clipping sample duty"
        ),
        "raw_accel_above_gate_time_percent": _number(
            raw_above.get("time_fraction_percent"), "raw clipping time duty"
        ),
        "raw_accel_above_gate_longest_sec": _number(
            raw_above.get("longest_contiguous_duration_sec"),
            "raw clipping longest duration",
        ),
        "throttle_near_saturation_sample_percent": _number(
            throttle.get("sample_fraction_percent"), "throttle sample duty"
        ),
        "throttle_near_saturation_time_percent": _number(
            throttle.get("time_fraction_percent"), "throttle time duty"
        ),
        "throttle_near_saturation_longest_sec": _number(
            throttle.get("longest_contiguous_duration_sec"),
            "throttle longest duration",
        ),
        "brake_active_sample_percent": _number(
            brake.get("sample_fraction_percent"), "brake sample duty"
        ),
        "brake_active_time_percent": _number(
            brake.get("time_fraction_percent"), "brake time duty"
        ),
        "brake_active_longest_sec": _number(
            brake.get("longest_contiguous_duration_sec"), "brake longest duration"
        ),
        "robust_acceleration_minimum_mps2": _number(
            robust.get("minimum"), "robust acceleration minimum"
        ),
        "robust_acceleration_p95_mps2": _number(
            robust.get("p95"), "robust acceleration p95"
        ),
        "robust_acceleration_maximum_mps2": _number(
            robust.get("maximum"), "robust acceleration maximum"
        ),
        "measured_acceleration_outlier_count": _integer(
            robust_quality.get("outlier_count"), "acceleration outlier count"
        ),
        "measured_acceleration_outlier_percent": _number(
            robust_quality.get("outlier_sample_percent"),
            "acceleration outlier percent",
        ),
        "gated_target_minus_actual_mean_mps": _number(
            target_tracking.get("mean"), "target tracking mean"
        ),
        "gated_target_minus_actual_rmse_mps": _number(
            target_tracking.get("rmse"), "target tracking RMSE"
        ),
        "gated_target_minus_actual_p95_abs_mps": _number(
            target_tracking.get("p95_absolute"), "target tracking p95 absolute"
        ),
        "gated_target_minus_actual_max_abs_mps": _number(
            target_tracking.get("maximum_absolute"),
            "target tracking maximum absolute",
        ),
        "accel_command_status_rmse": _number(
            accel_tracking.get("rmse"), "accel command/status RMSE"
        ),
        "brake_command_status_rmse": _number(
            brake_tracking.get("rmse"), "brake command/status RMSE"
        ),
        "map_axis_maximum_mps": _number(
            coverage.get("map_velocity_axis_maximum_mps"), "map axis maximum"
        ),
        "runtime_lookup_classification": runtime["classification"],
        "runtime_velocity_clamping_observed": False,
        "analysis_json_sha256": _sha256_file(response_path),
        "analysis_png_sha256": _sha256_file(plot_path),
        "analysis_png_dimensions": {"width": dimensions[0], "height": dimensions[1]},
        "source_identity_sha256": source["sha256"],
    }
    return row


def _distribution(rows: list[dict[str, Any]], key: str) -> dict[str, Any]:
    values = np.asarray([_number(row[key], key) for row in rows], dtype=float)
    return {
        "count": int(len(values)),
        "minimum": float(np.min(values)),
        "mean": float(np.mean(values)),
        "median": float(np.median(values)),
        "p95": float(np.percentile(values, 95)),
        "maximum": float(np.max(values)),
    }


def _extreme(
    rows: list[dict[str, Any]], key: str, *, highest: bool
) -> dict[str, Any]:
    selected = sorted(
        rows,
        key=lambda row: (
            (-1.0 if highest else 1.0) * _number(row[key], key),
            str(row["map_id"]),
            str(row["trial_id"]),
        ),
    )[0]
    return {
        "map_id": selected["map_id"],
        "trial_id": selected["trial_id"],
        "value": selected[key],
    }


def build_aggregate(
    matrix_root: Path, *, expected_trial_count: int = 18
) -> dict[str, Any]:
    candidate = matrix_root.expanduser()
    if candidate.is_symlink():
        raise AggregateError(f"matrix root must not be a symlink: {candidate}")
    root = candidate.resolve()
    if not root.is_dir():
        raise AggregateError(f"matrix root does not exist: {root}")
    matrix_path = root / "aggregate.json"
    matrix = _read_object(matrix_path, "matrix aggregate")
    if matrix.get("status") != "COMPLETE":
        raise AggregateError("matrix aggregate is not COMPLETE")
    profile_root = _mapping(matrix.get("runtime_profile"), "runtime profile")
    speed_contract = _mapping(profile_root.get("speed_contract"), "speed contract")
    profile_id = speed_contract.get("profile_id")
    if not isinstance(profile_id, str) or not profile_id:
        raise AggregateError("matrix speed profile id is missing")
    target_speed_mps = _number(
        speed_contract.get("target_speed_mps"), "matrix target speed"
    )
    route_parameters = _mapping(
        speed_contract.get("route_manager_parameters"), "route manager parameters"
    )
    longitudinal_source = route_parameters.get("longitudinal_velocity_source")
    if longitudinal_source != "explicit_simulation_nominal":
        raise AggregateError("matrix longitudinal speed source is unsupported")
    maps = matrix.get("maps")
    if not isinstance(maps, list):
        raise AggregateError("matrix aggregate has no maps array")

    rows: list[dict[str, Any]] = []
    selected_directories: set[Path] = set()
    for raw_map in maps:
        map_record = _mapping(raw_map, "matrix map record")
        if map_record.get("status") != "PASS":
            continue
        trials = _mapping(map_record.get("trials"), "matrix trials")
        for trial_id in TRIAL_IDS:
            trial = _mapping(trials.get(trial_id), f"{trial_id} trial")
            row = _extract_row(
                root,
                map_record,
                trial_id,
                trial,
                profile_id=profile_id,
                target_speed_mps=target_speed_mps,
                expected_longitudinal_source=longitudinal_source,
            )
            attempt = (root / row["attempt_directory"]).resolve()
            if attempt in selected_directories:
                raise AggregateError(f"duplicate selected attempt: {attempt}")
            selected_directories.add(attempt)
            rows.append(row)
    rows.sort(key=lambda row: (str(row["map_id"]), str(row["trial_id"])))
    if len(rows) != expected_trial_count:
        raise AggregateError(
            f"expected {expected_trial_count} selected trials, found {len(rows)}"
        )
    runnable_pass_count = _integer(
        matrix.get("runnable_pass_count"), "matrix runnable PASS count"
    )
    if len(rows) != runnable_pass_count * len(TRIAL_IDS):
        raise AggregateError("selected trial count does not match runnable PASS maps")

    by_trial: dict[str, Any] = {}
    for trial_id in TRIAL_IDS:
        selected = [row for row in rows if row["trial_id"] == trial_id]
        by_trial[trial_id] = {
            "trial_count": len(selected),
            "metrics": {
                key: _distribution(selected, key) for key in SUMMARY_METRICS
            },
        }
    result = {
        "schema_version": 1,
        "analysis": ANALYSIS_ID,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "status": "COMPLETE",
        "matrix": {
            "root": str(root),
            "aggregate": {
                "path": str(matrix_path),
                "sha256": _sha256_file(matrix_path),
            },
            "matrix_id": matrix.get("matrix_id"),
            "runtime_profile_selector": matrix.get("runtime_profile_selector"),
            "runnable_pass_count": runnable_pass_count,
        },
        "profile": {
            "profile_id": profile_id,
            "target_speed_mps": target_speed_mps,
            "target_speed_kph": target_speed_mps * 3.6,
            "longitudinal_speed_source": longitudinal_source,
            "requested_target_is_converter_lookup_velocity": False,
            "converter_lookup_velocity_source": EXPECTED_LOOKUP_SOURCE,
            "real_vehicle_ready": False,
        },
        "quality": {
            "expected_trial_count": expected_trial_count,
            "observed_trial_count": len(rows),
            "complete_trial_count": len(rows),
            "problem_count": 0,
            "warning_count": 0,
            "all_route_results_pass": True,
            "all_runtime_map_lookups_within_axis": True,
            "runtime_velocity_clamping_observed_count": 0,
        },
        "summary_by_trial": by_trial,
        "extremes": {
            "lowest_actual_maximum_speed_mps": _extreme(
                rows, "actual_maximum_speed_mps", highest=False
            ),
            "highest_gate_accel_limit_sample_percent": _extreme(
                rows, "gate_accel_limit_sample_percent", highest=True
            ),
            "highest_throttle_near_saturation_sample_percent": _extreme(
                rows, "throttle_near_saturation_sample_percent", highest=True
            ),
            "highest_gated_target_minus_actual_rmse_mps": _extreme(
                rows, "gated_target_minus_actual_rmse_mps", highest=True
            ),
            "highest_acceleration_outlier_percent": _extreme(
                rows, "measured_acceleration_outlier_percent", highest=True
            ),
        },
        "trials": rows,
        "outputs": {
            "json": f"{OUTPUT_BASENAME}.json",
            "csv": f"{OUTPUT_BASENAME}.csv",
            "plot": f"{OUTPUT_BASENAME}.png",
        },
    }
    return result


def _csv_text(evidence: dict[str, Any]) -> str:
    output = io.StringIO(newline="")
    writer = csv.DictWriter(
        output,
        fieldnames=CSV_COLUMNS,
        extrasaction="ignore",
        lineterminator="\n",
    )
    writer.writeheader()
    for row in evidence.get("trials", []):
        writer.writerow(row)
    return output.getvalue()


def _grouped_bars(
    axis: Any,
    rows: list[dict[str, Any]],
    maps: list[str],
    field: str,
    ylabel: str,
    title: str,
) -> None:
    x = np.arange(len(maps), dtype=float)
    width = 0.38
    colors = {"straight": "#2980b9", "turn": "#e67e22"}
    for offset, trial_id in ((-width / 2.0, "straight"), (width / 2.0, "turn")):
        values = []
        for map_id in maps:
            selected = next(
                row
                for row in rows
                if row["map_id"] == map_id and row["trial_id"] == trial_id
            )
            values.append(float(selected[field]))
        axis.bar(
            x + offset,
            values,
            width,
            label=trial_id,
            color=colors[trial_id],
            alpha=0.88,
        )
    axis.set_title(title)
    axis.set_ylabel(ylabel)
    axis.set_xticks(x, maps, rotation=28, ha="right")
    axis.grid(axis="y", alpha=0.25)
    axis.legend(fontsize=9)


def _plot(evidence: dict[str, Any], path: Path) -> None:
    rows = evidence.get("trials", [])
    if not rows:
        figure, axis = plt.subplots(figsize=(14, 7), constrained_layout=True)
        axis.axis("off")
        problems = evidence.get("quality", {}).get("problems", [])
        axis.text(
            0.5,
            0.58,
            "MATRIX LONGITUDINAL RESPONSE — ERROR",
            ha="center",
            va="center",
            fontsize=22,
            fontweight="bold",
            color="#c0392b",
        )
        axis.text(
            0.5,
            0.42,
            "\n".join(str(problem) for problem in problems),
            ha="center",
            va="center",
            fontsize=11,
            wrap=True,
        )
        figure.savefig(path, dpi=150, format="png")
        plt.close(figure)
        return
    maps = sorted({str(row["map_id"]) for row in rows})
    figure, axes = plt.subplots(2, 2, figsize=(18, 12), constrained_layout=True)
    figure.suptitle(
        "30 km/h CARLA Town matrix — longitudinal response (18 selected PASS trials)",
        fontsize=17,
        fontweight="bold",
    )
    _grouped_bars(
        axes[0, 0],
        rows,
        maps,
        "actual_maximum_speed_kph",
        "maximum actual speed [km/h]",
        "Observed speed by Town and route type",
    )
    axes[0, 0].axhline(
        float(evidence["profile"]["target_speed_kph"]),
        color="#16a085",
        linestyle="--",
        linewidth=1.4,
        label="profile target",
    )
    axes[0, 0].legend(fontsize=8)
    _grouped_bars(
        axes[0, 1],
        rows,
        maps,
        "gate_accel_limit_sample_percent",
        "sample duty [%]",
        "Vehicle-command-gate positive acceleration limit duty",
    )
    _grouped_bars(
        axes[1, 0],
        rows,
        maps,
        "brake_active_sample_percent",
        "sample duty [%]",
        "Brake command active duty "
        f"(max throttle near-saturation duty: "
        f"{max(float(row['throttle_near_saturation_sample_percent']) for row in rows):.3f}%)",
    )
    _grouped_bars(
        axes[1, 1],
        rows,
        maps,
        "gated_target_minus_actual_rmse_mps",
        "RMSE [m/s]",
        "Gated target minus actual speed tracking error",
    )
    figure.text(
        0.99,
        0.003,
        "All selected trials: route PASS, complete response evidence, current-speed map "
        "lookups inside the CSV axis; target is not a converter lookup velocity.",
        ha="right",
        fontsize=8,
        color="#555555",
    )
    figure.savefig(path, dpi=150, format="png")
    plt.close(figure)


def _atomic_outputs(evidence: dict[str, Any], output_dir: Path) -> None:
    directory = output_dir.expanduser().resolve()
    directory.mkdir(parents=True, exist_ok=True)
    staged: dict[str, Path] = {}
    descriptors: dict[str, int] = {}
    try:
        for suffix in ("json", "csv", "png"):
            descriptor, name = tempfile.mkstemp(
                prefix=f".{OUTPUT_BASENAME}.{suffix}.",
                suffix=".staged",
                dir=directory,
            )
            descriptors[suffix] = descriptor
            staged[suffix] = Path(name)
        with os.fdopen(descriptors.pop("json"), "w", encoding="utf-8") as stream:
            json.dump(evidence, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        with os.fdopen(
            descriptors.pop("csv"), "w", encoding="utf-8", newline=""
        ) as stream:
            stream.write(_csv_text(evidence))
            stream.flush()
            os.fsync(stream.fileno())
        os.close(descriptors.pop("png"))
        _plot(evidence, staged["png"])
        for suffix in ("json", "csv", "png"):
            os.replace(staged[suffix], directory / f"{OUTPUT_BASENAME}.{suffix}")
    finally:
        for descriptor in descriptors.values():
            os.close(descriptor)
        for path in staged.values():
            path.unlink(missing_ok=True)


def _failure(error: BaseException, matrix_root: Path) -> dict[str, Any]:
    return {
        "schema_version": 1,
        "analysis": ANALYSIS_ID,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "status": "ERROR",
        "matrix": {"root": str(matrix_root.expanduser().resolve())},
        "quality": {
            "problem_count": 1,
            "problems": [f"{type(error).__name__}: {error}"],
        },
        "trials": [],
        "outputs": {
            "json": f"{OUTPUT_BASENAME}.json",
            "csv": f"{OUTPUT_BASENAME}.csv",
            "plot": f"{OUTPUT_BASENAME}.png",
        },
    }


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--matrix-root", required=True, type=Path)
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="output directory; defaults to the matrix root",
    )
    parser.add_argument("--expected-trial-count", type=int, default=18)
    args = parser.parse_args(argv)
    if args.expected_trial_count <= 0:
        parser.error("--expected-trial-count must be positive")
    return args


def run(args: argparse.Namespace) -> int:
    output_dir = args.output_dir or args.matrix_root
    try:
        evidence = build_aggregate(
            args.matrix_root, expected_trial_count=args.expected_trial_count
        )
    except Exception as error:
        evidence = _failure(error, args.matrix_root)
    _atomic_outputs(evidence, output_dir)
    print(f"matrix longitudinal response status: {evidence['status']}")
    for suffix in ("json", "csv", "png"):
        print(
            f"matrix longitudinal response {suffix}: "
            f"{output_dir / f'{OUTPUT_BASENAME}.{suffix}'}"
        )
    if evidence["status"] != "COMPLETE":
        for problem in evidence.get("quality", {}).get("problems", []):
            print(f"matrix longitudinal response problem: {problem}", file=sys.stderr)
        return 1
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    return run(parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
