#!/usr/bin/env python3
"""Analyze the commanded and measured CARLA longitudinal response.

The report keeps trajectory-follower output, vehicle-command-gate output,
converter pedal output, applied pedal status, and vehicle response separate.
The requested cruise target is a profile envelope; it is *not* the velocity
used to index the raw-vehicle-command-converter maps.  Those maps are indexed
by absolute current odometry velocity.

ROS imports are deliberately lazy so the evidence calculations can be unit
tested without a sourced ROS installation.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import tempfile
from typing import Any, Callable, Sequence

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402


RAW_CONTROL_TOPICS = (
    "/control/trajectory_follower/control_cmd",
    "/trajectory_follower/control_cmd",
)
GATED_CONTROL_TOPIC = "/control/command/control_cmd"
ODOMETRY_TOPIC = "/localization/kinematic_state"
ACCELERATION_TOPIC = "/localization/acceleration"
ACTUATION_COMMAND_TOPIC = "/control/command/actuation_cmd"
ACTUATION_STATUS_TOPIC = "/vehicle/status/actuation_status"

EXPECTED_PROFILE_CONTEXT = {
    "longitudinal_velocity_source": "explicit_simulation_nominal",
    "vad_velocity_evaluated": False,
    "vad_geometry_evaluated": True,
}

REQUIRED_SERIES = (
    "raw_control",
    "gated_control",
    "actual_odometry",
    "measured_acceleration",
    "actuation_command",
    "actuation_status",
)


class AnalysisError(RuntimeError):
    """Raised when evidence cannot be bound or interpreted safely."""


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _sha256_json(value: Any) -> str:
    encoded = json.dumps(
        value, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _read_json_object(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise AnalysisError(f"cannot read {label} {path}: {error}") from error
    if not isinstance(value, dict):
        raise AnalysisError(f"{label} must contain a JSON object: {path}")
    return value


def _finite_number(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise AnalysisError(f"{label} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise AnalysisError(f"{label} must be finite")
    return result


def _bag_manifest(bag: Path) -> dict[str, Any]:
    candidate = bag.expanduser()
    if candidate.is_symlink():
        raise AnalysisError(f"rosbag evidence root must not be a symlink: {candidate}")
    root = candidate.resolve()
    if not root.is_dir():
        raise AnalysisError(f"rosbag directory does not exist: {root}")
    files: list[dict[str, Any]] = []
    for path in sorted(root.rglob("*")):
        if path.is_symlink():
            raise AnalysisError(f"rosbag evidence must not contain a symlink: {path}")
        if path.is_file():
            files.append(
                {
                    "path": path.relative_to(root).as_posix(),
                    "size_bytes": path.stat().st_size,
                    "sha256": _sha256_file(path),
                }
            )
    if not files or not any(item["path"] == "metadata.yaml" for item in files):
        raise AnalysisError(f"rosbag evidence has no metadata.yaml: {root}")
    manifest = {"schema_version": 1, "root": str(root), "files": files}
    manifest["sha256"] = _sha256_json(
        {"schema_version": manifest["schema_version"], "files": files}
    )
    return manifest


def _resolved_result_route(result_path: Path, result: dict[str, Any]) -> Path:
    route_value = result.get("route_file")
    if not isinstance(route_value, str) or not route_value:
        raise AnalysisError("route result has no route_file")
    route_path = Path(route_value).expanduser()
    if not route_path.is_absolute():
        route_path = result_path.parent / route_path
    return route_path.resolve()


def _source_identity(
    bag: Path,
    route_path: Path,
    result_path: Path,
    *,
    profile_id: str,
    target_speed_mps: float,
    longitudinal_speed_source: str,
) -> dict[str, Any]:
    route_path = route_path.expanduser().resolve()
    result_path = result_path.expanduser().resolve()
    if not route_path.is_file():
        raise AnalysisError(f"effective route does not exist: {route_path}")
    if not result_path.is_file():
        raise AnalysisError(f"route result does not exist: {result_path}")
    if not profile_id:
        raise AnalysisError("profile id must not be empty")
    if not math.isfinite(target_speed_mps) or target_speed_mps <= 0.0:
        raise AnalysisError("target speed must be finite and positive")
    if longitudinal_speed_source != "explicit_simulation_nominal":
        raise AnalysisError("unsupported longitudinal speed source")

    route = _read_json_object(route_path, "effective route")
    result = _read_json_object(result_path, "route result")
    result_route = _resolved_result_route(result_path, result)
    if result_route != route_path:
        raise AnalysisError(
            "route result is bound to a different effective route: "
            f"{result_route} != {route_path}"
        )
    scenario = route.get("scenario")
    town = route.get("town")
    if not isinstance(town, str) or not town or scenario not in {
        "straight",
        "left",
        "right",
    }:
        raise AnalysisError("effective route has invalid town/scenario identity")

    exposure = result.get("speed_exposure")
    limits = result.get("limits")
    result_success = result.get("success")
    if (
        not isinstance(result_success, bool)
        or result.get("execution_mode") != "full_stack"
        or result.get("profile_context") != EXPECTED_PROFILE_CONTEXT
        or not isinstance(exposure, dict)
        or not isinstance(limits, dict)
        or exposure.get("status") not in {"PASS", "FAIL"}
        or any(
            exposure.get(key) != value
            for key, value in EXPECTED_PROFILE_CONTEXT.items()
        )
    ):
        raise AnalysisError("route result is not an explicit-simulation speed trial")

    minimum_speed = _finite_number(
        exposure.get("minimum_sustained_speed_mps"),
        "speed exposure minimum sustained speed",
    )
    maximum_speed = _finite_number(
        exposure.get("maximum_observed_speed_limit_mps"),
        "speed exposure maximum speed limit",
    )
    if not minimum_speed <= target_speed_mps <= maximum_speed:
        raise AnalysisError(
            "profile target is outside the route result speed-exposure envelope"
        )
    route_length = route.get("route_length_m")
    if route_length is not None:
        route_length = _finite_number(route_length, "route length")

    profile = {
        "profile_id": profile_id,
        "target_speed_mps": target_speed_mps,
        "target_speed_kph": target_speed_mps * 3.6,
        "longitudinal_speed_source": longitudinal_speed_source,
        "requested_target_is_converter_lookup_velocity": False,
        "converter_lookup_velocity_source": (
            "absolute_current_odometry_longitudinal_speed_mps"
        ),
    }
    identity = {
        "schema_version": 1,
        "profile": profile,
        "effective_route": {
            "path": str(route_path),
            "sha256": _sha256_file(route_path),
            "town": town,
            "scenario": scenario,
            "trial_id": "straight" if scenario == "straight" else "turn",
            "route_length_m": route_length,
        },
        "route_result": {
            "path": str(result_path),
            "sha256": _sha256_file(result_path),
            "success": result_success,
            "execution_mode": "full_stack",
            "reason": result.get("reason"),
            "profile_context": EXPECTED_PROFILE_CONTEXT,
            "speed_exposure": {
                "status": exposure["status"],
                "minimum_sustained_speed_mps": minimum_speed,
                "minimum_sustained_speed_sec": _finite_number(
                    exposure.get("minimum_sustained_speed_sec"),
                    "speed exposure minimum sustained duration",
                ),
                "maximum_observed_speed_limit_mps": maximum_speed,
                "maximum_observed_speed_mps": _finite_number(
                    exposure.get("maximum_observed_speed_mps"),
                    "speed exposure maximum observed speed",
                ),
                "maximum_sustained_speed_duration_sec": _finite_number(
                    exposure.get("maximum_sustained_speed_duration_sec"),
                    "speed exposure maximum sustained duration",
                ),
                "continuity_maximum_gap_sec": _finite_number(
                    limits.get("maximum_speed_sample_gap_sec"),
                    "speed exposure continuity maximum sample gap",
                ),
            },
        },
        "rosbag": _bag_manifest(bag),
    }
    identity["sha256"] = _sha256_json(
        {key: value for key, value in identity.items() if key != "sha256"}
    )
    return identity


def _time_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _storage_identifier(bag: Path) -> str:
    try:
        import yaml
    except ImportError as error:  # pragma: no cover - sourced ROS has PyYAML
        raise AnalysisError("PyYAML is required to read rosbag metadata") from error
    metadata_path = bag / "metadata.yaml"
    if not metadata_path.is_file():
        raise AnalysisError(f"rosbag metadata does not exist: {metadata_path}")
    try:
        metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
        storage_id = metadata["rosbag2_bagfile_information"]["storage_identifier"]
    except (OSError, UnicodeDecodeError, TypeError, KeyError) as error:
        raise AnalysisError(
            f"cannot resolve storage_identifier from {metadata_path}: {error}"
        ) from error
    if not storage_id:
        raise AnalysisError(f"storage_identifier is missing from {metadata_path}")
    return str(storage_id)


def _read_bag(
    bag: Path,
) -> tuple[dict[str, list[dict[str, Any]]], dict[str, str]]:
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as error:  # pragma: no cover - requires sourced ROS
        raise AnalysisError(
            "ROS 2 Python modules are unavailable; source install/setup.bash first"
        ) from error

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(bag), storage_id=_storage_identifier(bag)
        ),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    requested = {
        *RAW_CONTROL_TOPICS,
        GATED_CONTROL_TOPIC,
        ODOMETRY_TOPIC,
        ACCELERATION_TOPIC,
        ACTUATION_COMMAND_TOPIC,
        ACTUATION_STATUS_TOPIC,
    }
    available = sorted(requested.intersection(topic_types))
    if hasattr(reader, "set_filter"):
        reader.set_filter(rosbag2_py.StorageFilter(topics=available))
    records: dict[str, list[dict[str, Any]]] = {topic: [] for topic in available}
    message_classes = {topic: get_message(topic_types[topic]) for topic in available}

    while reader.has_next():
        topic, serialized, bag_ns = reader.read_next()
        if topic not in message_classes:
            continue
        message = deserialize_message(serialized, message_classes[topic])
        if topic in RAW_CONTROL_TOPICS or topic == GATED_CONTROL_TOPIC:
            record = {
                "bag_ns": int(bag_ns),
                "stamp_ns": _time_ns(message.stamp),
                "speed": float(message.longitudinal.velocity),
                "acceleration": float(message.longitudinal.acceleration),
                "jerk": float(message.longitudinal.jerk),
            }
        elif topic == ODOMETRY_TOPIC:
            record = {
                "bag_ns": int(bag_ns),
                "stamp_ns": _time_ns(message.header.stamp),
                "speed": float(message.twist.twist.linear.x),
            }
        elif topic == ACCELERATION_TOPIC:
            record = {
                "bag_ns": int(bag_ns),
                "stamp_ns": _time_ns(message.header.stamp),
                "acceleration": float(message.accel.accel.linear.x),
            }
        elif topic == ACTUATION_COMMAND_TOPIC:
            record = {
                "bag_ns": int(bag_ns),
                "stamp_ns": _time_ns(message.header.stamp),
                "accel": float(message.actuation.accel_cmd),
                "brake": float(message.actuation.brake_cmd),
            }
        else:
            record = {
                "bag_ns": int(bag_ns),
                "stamp_ns": _time_ns(message.header.stamp),
                "accel": float(message.status.accel_status),
                "brake": float(message.status.brake_status),
            }
        records[topic].append(record)
    for values in records.values():
        values.sort(key=lambda value: value["bag_ns"])
    return records, topic_types


def _physics_ns(record: dict[str, Any]) -> int:
    stamp_ns = int(record.get("stamp_ns", 0))
    return stamp_ns if stamp_ns > 0 else int(record["bag_ns"])


def _sample_time(record: dict[str, Any], origin_ns: int) -> dict[str, Any]:
    stamp_ns = int(record.get("stamp_ns", 0))
    physics_ns = _physics_ns(record)
    return {
        "time_sec": (physics_ns - origin_ns) * 1.0e-9,
        "header_time_ns": stamp_ns if stamp_ns > 0 else None,
        "bag_time_ns": int(record["bag_ns"]),
        "time_source": "header_stamp" if stamp_ns > 0 else "bag_receipt_fallback",
    }


def _samples(
    records: list[dict[str, Any]],
    origin_ns: int,
    fields: tuple[str, ...],
) -> list[dict[str, Any]]:
    output: list[dict[str, Any]] = []
    for record in records:
        values: dict[str, float] = {}
        valid = True
        for field in fields:
            try:
                value = float(record.get(field, math.nan))
            except (TypeError, ValueError):
                valid = False
                break
            if not math.isfinite(value):
                valid = False
                break
            values[field] = value
        if valid:
            output.append({**_sample_time(record, origin_ns), **values})
    return sorted(output, key=lambda item: (item["time_sec"], item["bag_time_ns"]))


def _deduplicate(
    samples: list[dict[str, Any]], field: str
) -> tuple[np.ndarray, np.ndarray]:
    times = np.asarray([sample["time_sec"] for sample in samples], dtype=float)
    values = np.asarray([sample[field] for sample in samples], dtype=float)
    order = np.argsort(times, kind="stable")
    times = times[order]
    values = values[order]
    if len(times) > 1:
        keep = np.r_[times[1:] != times[:-1], True]
        times = times[keep]
        values = values[keep]
    return times, values


def _value_summary(values: Sequence[float]) -> dict[str, float | int | None]:
    array = np.asarray(values, dtype=float)
    array = array[np.isfinite(array)]
    if not len(array):
        return {
            "count": 0,
            "minimum": None,
            "mean": None,
            "median": None,
            "p95": None,
            "maximum": None,
        }
    return {
        "count": int(len(array)),
        "minimum": float(np.min(array)),
        "mean": float(np.mean(array)),
        "median": float(np.median(array)),
        "p95": float(np.percentile(array, 95)),
        "maximum": float(np.max(array)),
    }


def _series_summary(
    samples: list[dict[str, Any]], field: str
) -> dict[str, Any]:
    times = np.asarray([sample["time_sec"] for sample in samples], dtype=float)
    positive_gaps = np.diff(np.unique(times)) if len(times) > 1 else np.empty(0)
    return {
        **_value_summary([float(sample[field]) for sample in samples]),
        "first_time_sec": float(times[0]) if len(times) else None,
        "last_time_sec": float(times[-1]) if len(times) else None,
        "duration_sec": float(times[-1] - times[0]) if len(times) else None,
        "maximum_sample_gap_sec": (
            float(np.max(positive_gaps)) if len(positive_gaps) else None
        ),
        "header_stamp_count": sum(
            sample["time_source"] == "header_stamp" for sample in samples
        ),
        "bag_receipt_fallback_count": sum(
            sample["time_source"] == "bag_receipt_fallback" for sample in samples
        ),
    }


def _robust_acceleration(
    samples: list[dict[str, Any]],
    *,
    window_sec: float = 0.30,
    sigma_threshold: float = 6.0,
    minimum_residual_threshold_mps2: float = 0.75,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    """Return a local-median acceleration trace and transparent outlier metrics."""
    if not samples:
        return [], {
            "algorithm": "centered_time_window_median",
            "window_sec": window_sec,
            "outlier_count": 0,
            "outlier_sample_percent": None,
        }
    times = np.asarray([sample["time_sec"] for sample in samples], dtype=float)
    raw = np.asarray([sample["acceleration"] for sample in samples], dtype=float)
    local_median = np.empty(len(raw), dtype=float)
    half_window = window_sec / 2.0
    for index, time_sec in enumerate(times):
        selected = (times >= time_sec - half_window) & (
            times <= time_sec + half_window
        )
        local_median[index] = float(np.median(raw[selected]))
    residual = raw - local_median
    residual_center = float(np.median(residual))
    residual_mad = float(np.median(np.abs(residual - residual_center)))
    robust_sigma = 1.4826 * residual_mad
    threshold = max(
        minimum_residual_threshold_mps2, sigma_threshold * robust_sigma
    )
    outlier = np.abs(residual - residual_center) > threshold
    output = [
        {
            **sample,
            "raw_acceleration_mps2": float(raw[index]),
            "robust_acceleration_mps2": float(local_median[index]),
            "residual_mps2": float(residual[index]),
            "outlier": bool(outlier[index]),
        }
        for index, sample in enumerate(samples)
    ]
    return output, {
        "algorithm": "centered_time_window_median",
        "window_sec": window_sec,
        "residual_outlier_rule": (
            "abs(residual - median(residual)) > max(minimum, sigma * 1.4826*MAD)"
        ),
        "sigma_threshold": sigma_threshold,
        "minimum_residual_threshold_mps2": minimum_residual_threshold_mps2,
        "residual_mad_mps2": residual_mad,
        "robust_sigma_mps2": robust_sigma,
        "applied_residual_threshold_mps2": threshold,
        "outlier_count": int(np.count_nonzero(outlier)),
        "outlier_sample_percent": float(np.mean(outlier) * 100.0),
        "raw_summary_mps2": _value_summary(raw.tolist()),
        "robust_summary_mps2": _value_summary(local_median.tolist()),
    }


def _duty_metrics(
    samples: list[dict[str, Any]],
    field: str,
    predicate: Callable[[float], bool],
    *,
    maximum_gap_sec: float,
    definition: str,
) -> dict[str, Any]:
    if len(samples) < 2:
        return {
            "available": False,
            "definition": definition,
            "sample_count": len(samples),
        }
    times, values = _deduplicate(samples, field)
    flags = np.asarray([predicate(float(value)) for value in values], dtype=bool)
    active_duration = 0.0
    eligible_duration = 0.0
    excluded_gap_duration = 0.0
    longest = 0.0
    current = 0.0
    for index, interval in enumerate(np.diff(times)):
        dt = float(interval)
        if dt <= 0.0:
            continue
        if dt > maximum_gap_sec:
            excluded_gap_duration += dt
            current = 0.0
            continue
        eligible_duration += dt
        if bool(flags[index]) and bool(flags[index + 1]):
            active_duration += dt
            current += dt
            longest = max(longest, current)
        else:
            current = 0.0
    return {
        "available": True,
        "definition": definition,
        "sample_count": int(len(flags)),
        "active_sample_count": int(np.count_nonzero(flags)),
        "sample_fraction_percent": float(np.mean(flags) * 100.0),
        "eligible_interval_duration_sec": eligible_duration,
        "active_interval_duration_sec": active_duration,
        "time_fraction_percent": (
            active_duration / eligible_duration * 100.0
            if eligible_duration > 0.0
            else None
        ),
        "longest_contiguous_duration_sec": longest,
        "maximum_counted_gap_sec": maximum_gap_sec,
        "excluded_gap_duration_sec": excluded_gap_duration,
        "interval_rule": (
            "both endpoints must satisfy the predicate; intervals beyond the "
            "maximum gap are excluded"
        ),
    }


def _aligned_error_summary(
    left: list[dict[str, Any]],
    left_field: str,
    right: list[dict[str, Any]],
    right_field: str,
) -> dict[str, Any]:
    if len(left) < 2 or len(right) < 2:
        return {"available": False, "count": 0}
    left_time, left_values = _deduplicate(left, left_field)
    right_time, right_values = _deduplicate(right, right_field)
    begin = max(float(left_time[0]), float(right_time[0]))
    end = min(float(left_time[-1]), float(right_time[-1]))
    selected = (left_time >= begin) & (left_time <= end)
    grid = left_time[selected]
    if end <= begin or not len(grid):
        return {"available": False, "count": 0, "overlap_sec": max(0.0, end - begin)}
    error = left_values[selected] - np.interp(grid, right_time, right_values)
    return {
        "available": True,
        **_value_summary(error.tolist()),
        "rmse": float(np.sqrt(np.mean(np.square(error)))),
        "p95_absolute": float(np.percentile(np.abs(error), 95)),
        "maximum_absolute": float(np.max(np.abs(error))),
        "overlap_sec": end - begin,
        "definition": "left minus linearly-interpolated right at left timestamps",
    }


def _map_coverage_context(
    path: Path,
    *,
    profile_id: str,
    target_speed_mps: float,
) -> dict[str, Any]:
    resolved = path.expanduser().resolve()
    payload = _read_json_object(resolved, "actuation map coverage")
    if payload.get("analysis") != "raw_vehicle_command_converter_velocity_coverage":
        raise AnalysisError("actuation map coverage has an unexpected analysis type")
    if payload.get("status") not in {"PASS", "EXPLORATORY", "BLOCKED"}:
        raise AnalysisError("actuation map coverage has an invalid status")
    if payload.get("profile_id") != profile_id:
        raise AnalysisError("actuation map coverage profile id does not match")
    coverage_target = _finite_number(
        payload.get("target_speed_mps"), "actuation map coverage target speed"
    )
    if not math.isclose(coverage_target, target_speed_mps, abs_tol=1.0e-9):
        raise AnalysisError("actuation map coverage target speed does not match")
    runtime = payload.get("runtime_lookup_observation")
    if not isinstance(runtime, dict):
        raise AnalysisError("actuation map coverage lacks runtime lookup observation")
    boundary = payload.get("validation_boundary")
    if (
        not isinstance(boundary, dict)
        or boundary.get("target_speed_is_converter_lookup_velocity") is not False
        or boundary.get("converter_lookup_velocity_source")
        != "absolute_current_odometry_longitudinal_speed_mps"
    ):
        raise AnalysisError(
            "actuation map coverage does not prove current-odometry lookup semantics"
        )
    return {
        "provided": True,
        "source": {"path": str(resolved), "sha256": _sha256_file(resolved)},
        "status": payload["status"],
        "target_envelope_classification": payload.get(
            "target_envelope_classification"
        ),
        "target_within_map_velocity_axis": payload.get(
            "target_within_map_velocity_axis"
        ),
        "map_velocity_axis_minimum_mps": payload.get(
            "map_velocity_axis_minimum_mps"
        ),
        "map_velocity_axis_maximum_mps": payload.get(
            "map_velocity_axis_maximum_mps"
        ),
        "runtime_lookup_observation": runtime,
        "interpretation": {
            "requested_target_is_converter_lookup_velocity": False,
            "converter_lookup_velocity_source": (
                "absolute_current_odometry_longitudinal_speed_mps"
            ),
            "target_axis_excess_is_runtime_clamping": False,
            "runtime_clamping_requires_observed_current_speed_beyond_axis": True,
        },
    }


def _select_raw_control(
    records: dict[str, list[dict[str, Any]]]
) -> tuple[str | None, list[dict[str, Any]]]:
    for topic in RAW_CONTROL_TOPICS:
        if records.get(topic):
            return topic, records[topic]
    return None, []


def build_evidence(
    records: dict[str, list[dict[str, Any]]],
    topic_types: dict[str, str],
    *,
    bag: Path,
    profile_id: str,
    target_speed_mps: float,
    longitudinal_speed_source: str,
    source_identity: dict[str, Any] | None = None,
    map_coverage: dict[str, Any] | None = None,
    gated_acceleration_limit_mps2: float = 1.5,
    accel_pedal_saturation: float = 0.399,
    brake_pedal_saturation: float = 0.599,
    pedal_activity_epsilon: float = 1.0e-3,
) -> dict[str, Any]:
    numeric_inputs = {
        "target_speed_mps": target_speed_mps,
        "gated_acceleration_limit_mps2": gated_acceleration_limit_mps2,
        "accel_pedal_saturation": accel_pedal_saturation,
        "brake_pedal_saturation": brake_pedal_saturation,
        "pedal_activity_epsilon": pedal_activity_epsilon,
    }
    if not profile_id:
        raise AnalysisError("profile id must not be empty")
    if longitudinal_speed_source != "explicit_simulation_nominal":
        raise AnalysisError("unsupported longitudinal speed source")
    for label, value in numeric_inputs.items():
        if not math.isfinite(value) or value <= 0.0:
            raise AnalysisError(f"{label} must be finite and positive")

    raw_topic, raw_records = _select_raw_control(records)
    selected_records = {
        "raw_control": raw_records,
        "gated_control": records.get(GATED_CONTROL_TOPIC, []),
        "actual_odometry": records.get(ODOMETRY_TOPIC, []),
        "measured_acceleration": records.get(ACCELERATION_TOPIC, []),
        "actuation_command": records.get(ACTUATION_COMMAND_TOPIC, []),
        "actuation_status": records.get(ACTUATION_STATUS_TOPIC, []),
    }
    physics_times = [
        _physics_ns(record)
        for values in selected_records.values()
        for record in values
    ]
    if not physics_times:
        raise AnalysisError("bag has no messages for any longitudinal-response topic")
    origin_ns = min(physics_times)
    series = {
        "raw_control": _samples(
            selected_records["raw_control"],
            origin_ns,
            ("speed", "acceleration", "jerk"),
        ),
        "gated_control": _samples(
            selected_records["gated_control"],
            origin_ns,
            ("speed", "acceleration", "jerk"),
        ),
        "actual_odometry": _samples(
            selected_records["actual_odometry"], origin_ns, ("speed",)
        ),
        "measured_acceleration": _samples(
            selected_records["measured_acceleration"],
            origin_ns,
            ("acceleration",),
        ),
        "actuation_command": _samples(
            selected_records["actuation_command"],
            origin_ns,
            ("accel", "brake"),
        ),
        "actuation_status": _samples(
            selected_records["actuation_status"],
            origin_ns,
            ("accel", "brake"),
        ),
    }
    robust_acceleration, robust_metrics = _robust_acceleration(
        series["measured_acceleration"]
    )
    series["measured_acceleration"] = robust_acceleration

    problems: list[str] = []
    warnings: list[str] = []
    topic_by_series = {
        "raw_control": raw_topic or " | ".join(RAW_CONTROL_TOPICS),
        "gated_control": GATED_CONTROL_TOPIC,
        "actual_odometry": ODOMETRY_TOPIC,
        "measured_acceleration": ACCELERATION_TOPIC,
        "actuation_command": ACTUATION_COMMAND_TOPIC,
        "actuation_status": ACTUATION_STATUS_TOPIC,
    }
    for key in REQUIRED_SERIES:
        if len(series[key]) < 2:
            problems.append(
                f"{topic_by_series[key]} has {len(series[key])} finite sample(s); "
                "at least 2 are required"
            )
        fallback_count = sum(
            sample["time_source"] == "bag_receipt_fallback"
            for sample in series[key]
        )
        if fallback_count:
            warnings.append(
                f"{topic_by_series[key]} used bag receipt time for "
                f"{fallback_count} sample(s)"
            )

    intervals = [
        (values[0]["time_sec"], values[-1]["time_sec"])
        for values in series.values()
        if len(values) >= 2
    ]
    common_begin = max((interval[0] for interval in intervals), default=math.nan)
    common_end = min((interval[1] for interval in intervals), default=math.nan)
    common_duration = (
        max(0.0, common_end - common_begin)
        if math.isfinite(common_begin) and math.isfinite(common_end)
        else None
    )
    if len(intervals) == len(REQUIRED_SERIES) and not (
        common_duration and common_duration > 0.0
    ):
        problems.append("required series have no common simulation-time interval")

    exposure_contract = (
        source_identity.get("route_result", {}).get("speed_exposure", {})
        if isinstance(source_identity, dict)
        else {}
    )
    exposure_threshold = float(
        exposure_contract.get("minimum_sustained_speed_mps", target_speed_mps)
    )
    maximum_gap_sec = float(
        exposure_contract.get("continuity_maximum_gap_sec", 0.25)
    )
    actual = series["actual_odometry"]
    target_exposure = {
        "requested_target": _duty_metrics(
            actual,
            "speed",
            lambda value: value >= target_speed_mps,
            maximum_gap_sec=maximum_gap_sec,
            definition=f"actual odometry speed >= {target_speed_mps:.9g} m/s",
        ),
        "route_contract_minimum": _duty_metrics(
            actual,
            "speed",
            lambda value: value >= exposure_threshold,
            maximum_gap_sec=maximum_gap_sec,
            definition=f"actual odometry speed >= {exposure_threshold:.9g} m/s",
        ),
        "route_result_reported": exposure_contract or None,
    }
    if exposure_contract:
        computed_contract = target_exposure["route_contract_minimum"]
        computed_duration = computed_contract.get(
            "longest_contiguous_duration_sec"
        )
        reported_duration = float(
            exposure_contract["maximum_sustained_speed_duration_sec"]
        )
        required_duration = float(exposure_contract["minimum_sustained_speed_sec"])
        duration_comparison_applicable = (
            required_duration > 0.0 and exposure_threshold > 0.0
        )
        bag_maximum = max(
            (float(sample["speed"]) for sample in actual), default=math.nan
        )
        reported_maximum = float(exposure_contract["maximum_observed_speed_mps"])
        target_exposure["route_result_cross_check"] = {
            "bag_maximum_odometry_speed_mps": (
                bag_maximum if math.isfinite(bag_maximum) else None
            ),
            "reported_maximum_observed_speed_mps": reported_maximum,
            "maximum_speed_absolute_difference_mps": (
                abs(bag_maximum - reported_maximum)
                if math.isfinite(bag_maximum)
                else None
            ),
            "maximum_speed_consistent_with_bag": (
                math.isfinite(bag_maximum)
                and math.isclose(bag_maximum, reported_maximum, abs_tol=0.05)
            ),
            "bag_longest_contract_exposure_sec": computed_duration,
            "reported_longest_contract_exposure_sec": reported_duration,
            "duration_absolute_difference_sec": (
                abs(float(computed_duration) - reported_duration)
                if isinstance(computed_duration, (int, float))
                else None
            ),
            "duration_consistent_within_one_continuity_gap": (
                (
                    isinstance(computed_duration, (int, float))
                    and abs(float(computed_duration) - reported_duration)
                    <= maximum_gap_sec + 1.0e-9
                )
                if duration_comparison_applicable
                else None
            ),
            "minimum_duration_condition_met_from_bag": (
                (
                    isinstance(computed_duration, (int, float))
                    and float(computed_duration) >= required_duration
                )
                if duration_comparison_applicable
                else None
            ),
            "duration_comparison_applicable": duration_comparison_applicable,
            "note": (
                "The route result may include additional lateral-acceleration and "
                "maximum-speed checks; this comparison covers longitudinal speed "
                "exposure only."
            ),
        }
        if not target_exposure["route_result_cross_check"][
            "maximum_speed_consistent_with_bag"
        ]:
            problems.append("route result maximum speed does not match this bag")
        duration_consistency = target_exposure["route_result_cross_check"][
            "duration_consistent_within_one_continuity_gap"
        ]
        if duration_consistency is False:
            warnings.append(
                "route result and bag sustained-speed durations differ by more "
                "than one allowed continuity gap"
            )

    saturation = {
        "raw_acceleration_above_gate_limit": _duty_metrics(
            series["raw_control"],
            "acceleration",
            lambda value: value > gated_acceleration_limit_mps2 + 1.0e-6,
            maximum_gap_sec=maximum_gap_sec,
            definition=(
                "raw controller acceleration > configured gate positive limit "
                f"({gated_acceleration_limit_mps2:.9g} m/s^2)"
            ),
        ),
        "gated_positive_acceleration_limit": _duty_metrics(
            series["gated_control"],
            "acceleration",
            lambda value: value
            >= gated_acceleration_limit_mps2
            - max(1.0e-3, gated_acceleration_limit_mps2 * 1.0e-3),
            maximum_gap_sec=maximum_gap_sec,
            definition=(
                "gated acceleration within max(0.001, 0.1%) of positive limit "
                f"({gated_acceleration_limit_mps2:.9g} m/s^2)"
            ),
        ),
        "accel_command_active": _duty_metrics(
            series["actuation_command"],
            "accel",
            lambda value: value > pedal_activity_epsilon,
            maximum_gap_sec=maximum_gap_sec,
            definition=f"accel command > {pedal_activity_epsilon:.9g}",
        ),
        "accel_command_near_saturation": _duty_metrics(
            series["actuation_command"],
            "accel",
            lambda value: value >= accel_pedal_saturation,
            maximum_gap_sec=maximum_gap_sec,
            definition=f"accel command >= {accel_pedal_saturation:.9g}",
        ),
        "brake_command_active": _duty_metrics(
            series["actuation_command"],
            "brake",
            lambda value: value > pedal_activity_epsilon,
            maximum_gap_sec=maximum_gap_sec,
            definition=f"brake command > {pedal_activity_epsilon:.9g}",
        ),
        "brake_command_near_saturation": _duty_metrics(
            series["actuation_command"],
            "brake",
            lambda value: value >= brake_pedal_saturation,
            maximum_gap_sec=maximum_gap_sec,
            definition=f"brake command >= {brake_pedal_saturation:.9g}",
        ),
        "accel_status_near_saturation": _duty_metrics(
            series["actuation_status"],
            "accel",
            lambda value: value >= accel_pedal_saturation,
            maximum_gap_sec=maximum_gap_sec,
            definition=f"applied accel status >= {accel_pedal_saturation:.9g}",
        ),
        "accel_status_active": _duty_metrics(
            series["actuation_status"],
            "accel",
            lambda value: value > pedal_activity_epsilon,
            maximum_gap_sec=maximum_gap_sec,
            definition=f"applied accel status > {pedal_activity_epsilon:.9g}",
        ),
        "brake_status_active": _duty_metrics(
            series["actuation_status"],
            "brake",
            lambda value: value > pedal_activity_epsilon,
            maximum_gap_sec=maximum_gap_sec,
            definition=f"applied brake status > {pedal_activity_epsilon:.9g}",
        ),
        "brake_status_near_saturation": _duty_metrics(
            series["actuation_status"],
            "brake",
            lambda value: value >= brake_pedal_saturation,
            maximum_gap_sec=maximum_gap_sec,
            definition=f"applied brake status >= {brake_pedal_saturation:.9g}",
        ),
    }

    actual_summary = _series_summary(actual, "speed")
    if map_coverage is not None:
        runtime = map_coverage.get("runtime_lookup_observation", {})
        if runtime.get("available") is True:
            reported_maximum = _finite_number(
                runtime.get("maximum_absolute_current_speed_mps"),
                "map runtime maximum current speed",
            )
            measured_maximum = max(
                (abs(float(sample["speed"])) for sample in actual), default=math.nan
            )
            map_coverage["runtime_cross_check"] = {
                "bag_maximum_absolute_odometry_speed_mps": (
                    measured_maximum if math.isfinite(measured_maximum) else None
                ),
                "coverage_reported_maximum_absolute_current_speed_mps": (
                    reported_maximum
                ),
                "absolute_difference_mps": (
                    abs(measured_maximum - reported_maximum)
                    if math.isfinite(measured_maximum)
                    else None
                ),
                "consistent_with_bag": (
                    math.isfinite(measured_maximum)
                    and math.isclose(measured_maximum, reported_maximum, abs_tol=0.05)
                ),
            }
            if not map_coverage["runtime_cross_check"]["consistent_with_bag"]:
                problems.append(
                    "actuation map runtime maximum speed does not match this bag"
                )

    summaries = {
        "raw_target_speed_mps": _series_summary(series["raw_control"], "speed"),
        "gated_target_speed_mps": _series_summary(
            series["gated_control"], "speed"
        ),
        "actual_speed_mps": actual_summary,
        "raw_acceleration_mps2": _series_summary(
            series["raw_control"], "acceleration"
        ),
        "gated_acceleration_mps2": _series_summary(
            series["gated_control"], "acceleration"
        ),
        "robust_measured_acceleration_mps2": _value_summary(
            [
                sample["robust_acceleration_mps2"]
                for sample in series["measured_acceleration"]
            ]
        ),
        "accel_command": _series_summary(series["actuation_command"], "accel"),
        "brake_command": _series_summary(series["actuation_command"], "brake"),
        "accel_status": _series_summary(series["actuation_status"], "accel"),
        "brake_status": _series_summary(series["actuation_status"], "brake"),
    }

    return {
        "schema_version": 1,
        "analysis": "carla_longitudinal_response",
        "status": "complete" if not problems else "incomplete",
        "inputs": {
            "bag": str(bag.expanduser().resolve()),
            "profile_id": profile_id,
            "target_speed_mps": target_speed_mps,
            "target_speed_kph": target_speed_mps * 3.6,
            "longitudinal_speed_source": longitudinal_speed_source,
            "thresholds": {
                "gated_acceleration_limit_mps2": gated_acceleration_limit_mps2,
                "accel_pedal_saturation": accel_pedal_saturation,
                "brake_pedal_saturation": brake_pedal_saturation,
                "pedal_activity_epsilon": pedal_activity_epsilon,
                "maximum_contiguous_sample_gap_sec": maximum_gap_sec,
            },
        },
        "source_identity": source_identity,
        "interpretation": {
            "requested_target_role": "simulation speed-profile envelope",
            "requested_target_is_converter_lookup_velocity": False,
            "converter_lookup_velocity_source": (
                "absolute_current_odometry_longitudinal_speed_mps"
            ),
            "raw_control_stage": "trajectory follower before vehicle command gate",
            "gated_control_stage": "vehicle command gate output",
            "actuation_command_stage": "raw vehicle command converter pedal output",
            "actuation_status_stage": "CARLA interface applied pedal status",
            "real_vehicle_ready": False,
        },
        "alignment": {
            "primary_time": "ROS message/header stamp (CARLA simulation clock)",
            "fallback_time": "rosbag receipt timestamp when header stamp is zero",
            "origin_time_ns": origin_ns,
            "reported_time": "seconds relative to earliest required source sample",
            "common_interval_start_sec": (
                common_begin if math.isfinite(common_begin) else None
            ),
            "common_interval_end_sec": (
                common_end if math.isfinite(common_end) else None
            ),
            "common_interval_duration_sec": common_duration,
        },
        "sources": {
            "raw_control": {
                "topic": raw_topic,
                "topic_type": topic_types.get(raw_topic) if raw_topic else None,
                "fields": "longitudinal.velocity/acceleration/jerk",
            },
            "gated_control": {
                "topic": GATED_CONTROL_TOPIC,
                "topic_type": topic_types.get(GATED_CONTROL_TOPIC),
                "fields": "longitudinal.velocity/acceleration/jerk",
            },
            "actual_odometry": {
                "topic": ODOMETRY_TOPIC,
                "topic_type": topic_types.get(ODOMETRY_TOPIC),
                "fields": "twist.twist.linear.x",
            },
            "measured_acceleration": {
                "topic": ACCELERATION_TOPIC,
                "topic_type": topic_types.get(ACCELERATION_TOPIC),
                "fields": "accel.accel.linear.x",
            },
            "actuation_command": {
                "topic": ACTUATION_COMMAND_TOPIC,
                "topic_type": topic_types.get(ACTUATION_COMMAND_TOPIC),
                "fields": "actuation.accel_cmd/brake_cmd",
            },
            "actuation_status": {
                "topic": ACTUATION_STATUS_TOPIC,
                "topic_type": topic_types.get(ACTUATION_STATUS_TOPIC),
                "fields": "status.accel_status/brake_status",
            },
        },
        "message_counts": {
            topic: len(records.get(topic, []))
            for topic in (
                *RAW_CONTROL_TOPICS,
                GATED_CONTROL_TOPIC,
                ODOMETRY_TOPIC,
                ACCELERATION_TOPIC,
                ACTUATION_COMMAND_TOPIC,
                ACTUATION_STATUS_TOPIC,
            )
        },
        "quality": {
            "required_series": list(REQUIRED_SERIES),
            "problems": problems,
            "warnings": warnings,
        },
        "summary": summaries,
        "robust_measured_acceleration": robust_metrics,
        "target_exposure": target_exposure,
        "saturation_and_duty": saturation,
        "tracking": {
            "raw_minus_gated_target_speed_mps": _aligned_error_summary(
                series["raw_control"],
                "speed",
                series["gated_control"],
                "speed",
            ),
            "gated_target_minus_actual_speed_mps": _aligned_error_summary(
                series["gated_control"], "speed", actual, "speed"
            ),
            "raw_minus_gated_acceleration_mps2": _aligned_error_summary(
                series["raw_control"],
                "acceleration",
                series["gated_control"],
                "acceleration",
            ),
            "accel_command_minus_status": _aligned_error_summary(
                series["actuation_command"],
                "accel",
                series["actuation_status"],
                "accel",
            ),
            "brake_command_minus_status": _aligned_error_summary(
                series["actuation_command"],
                "brake",
                series["actuation_status"],
                "brake",
            ),
        },
        "actuation_map_coverage": map_coverage or {"provided": False},
        "series": series,
        "outputs": {
            "json": "longitudinal_response.json",
            "plot": "longitudinal_response.png",
        },
    }


def _plot_series(
    axis: Any,
    samples: list[dict[str, Any]],
    field: str,
    *,
    label: str,
    color: str,
    linestyle: str = "-",
    linewidth: float = 1.7,
) -> None:
    if samples:
        axis.plot(
            [sample["time_sec"] for sample in samples],
            [sample[field] for sample in samples],
            label=label,
            color=color,
            linestyle=linestyle,
            linewidth=linewidth,
            alpha=0.93,
        )


def _plot_evidence(evidence: dict[str, Any], output: Path) -> None:
    figure, axes = plt.subplots(3, 1, figsize=(16, 13), constrained_layout=True)
    inputs = evidence.get("inputs", {})
    source = evidence.get("source_identity") or {}
    route_result = source.get("route_result", {})
    trial_verdict = (
        route_result.get("speed_exposure", {}).get("status", "UNBOUND")
        if isinstance(route_result, dict)
        else "UNBOUND"
    )
    figure.suptitle(
        f"CARLA longitudinal response | {inputs.get('profile_id', 'unknown')} | "
        f"route speed exposure {trial_verdict} | analysis "
        f"{str(evidence.get('status', 'incomplete')).upper()}",
        fontsize=16,
        fontweight="bold",
    )
    series = evidence.get("series", {})

    axis = axes[0]
    _plot_series(
        axis,
        series.get("raw_control", []),
        "speed",
        label="Raw trajectory-follower target",
        color="#2980b9",
    )
    _plot_series(
        axis,
        series.get("gated_control", []),
        "speed",
        label="Gated target",
        color="#8e44ad",
    )
    _plot_series(
        axis,
        series.get("actual_odometry", []),
        "speed",
        label="Actual odometry",
        color="#111111",
        linewidth=2.4,
    )
    target = inputs.get("target_speed_mps")
    if isinstance(target, (int, float)) and math.isfinite(float(target)):
        axis.axhline(
            float(target),
            color="#16a085",
            linestyle="--",
            linewidth=1.5,
            label=f"Requested profile target ({float(target):.3f} m/s)",
        )
    contract = source.get("route_result", {}).get("speed_exposure", {})
    minimum = contract.get("minimum_sustained_speed_mps")
    if isinstance(minimum, (int, float)) and math.isfinite(float(minimum)):
        axis.axhline(
            float(minimum),
            color="#27ae60",
            linestyle=":",
            linewidth=1.4,
            label=f"Exposure minimum ({float(minimum):.3f} m/s)",
        )
    axis.set_title("Longitudinal target propagation and actual speed")
    axis.set_ylabel("speed [m/s]")
    axis.grid(alpha=0.25)
    axis.legend(fontsize=9, loc="best")

    axis = axes[1]
    _plot_series(
        axis,
        series.get("raw_control", []),
        "acceleration",
        label="Raw controller acceleration",
        color="#2980b9",
    )
    _plot_series(
        axis,
        series.get("gated_control", []),
        "acceleration",
        label="Gated acceleration",
        color="#8e44ad",
    )
    _plot_series(
        axis,
        series.get("measured_acceleration", []),
        "robust_acceleration_mps2",
        label="Robust measured longitudinal acceleration",
        color="#c0392b",
        linewidth=2.0,
    )
    gate_limit = inputs.get("thresholds", {}).get(
        "gated_acceleration_limit_mps2"
    )
    if isinstance(gate_limit, (int, float)):
        axis.axhline(
            float(gate_limit),
            color="#7f8c8d",
            linestyle="--",
            linewidth=1.2,
            label=f"Configured gate positive limit ({float(gate_limit):.3f})",
        )
    axis.axhline(0.0, color="#555555", linewidth=0.8)
    axis.set_title("Acceleration request, gate output, and robust vehicle response")
    axis.set_ylabel("acceleration [m/s²]")
    axis.grid(alpha=0.25)
    axis.legend(fontsize=9, loc="best")

    axis = axes[2]
    _plot_series(
        axis,
        series.get("actuation_command", []),
        "accel",
        label="Throttle command",
        color="#27ae60",
    )
    _plot_series(
        axis,
        series.get("actuation_status", []),
        "accel",
        label="Applied throttle status",
        color="#145a32",
        linestyle="--",
    )
    if series.get("actuation_command"):
        axis.plot(
            [sample["time_sec"] for sample in series["actuation_command"]],
            [-sample["brake"] for sample in series["actuation_command"]],
            label="Brake command (negative for display)",
            color="#e74c3c",
            linewidth=1.7,
        )
    if series.get("actuation_status"):
        axis.plot(
            [sample["time_sec"] for sample in series["actuation_status"]],
            [-sample["brake"] for sample in series["actuation_status"]],
            label="Applied brake status (negative for display)",
            color="#922b21",
            linestyle="--",
            linewidth=1.7,
        )
    axis.axhline(0.0, color="#555555", linewidth=0.8)
    axis.set_title("Converter pedal output and CARLA-applied status")
    axis.set_xlabel("simulation/header time from first evidence sample [s]")
    axis.set_ylabel("normalized pedal (brake shown negative)")
    axis.grid(alpha=0.25)
    axis.legend(fontsize=9, loc="best")

    problems = evidence.get("quality", {}).get("problems", [])
    if problems:
        figure.text(
            0.01,
            0.006,
            "INCOMPLETE: " + " | ".join(str(item) for item in problems),
            color="#c0392b",
            fontsize=8,
        )
    figure.text(
        0.99,
        0.006,
        "Requested target is a profile envelope; converter map lookup uses current "
        "odometry speed.",
        ha="right",
        color="#555555",
        fontsize=8,
    )
    figure.savefig(output, dpi=150, format="png")
    plt.close(figure)


def _failure_evidence(args: argparse.Namespace, error: BaseException) -> dict[str, Any]:
    target = getattr(args, "target_speed_mps", None)
    return {
        "schema_version": 1,
        "analysis": "carla_longitudinal_response",
        "status": "incomplete",
        "inputs": {
            "bag": str(Path(args.bag).expanduser().resolve()),
            "profile_id": getattr(args, "profile_id", None),
            "target_speed_mps": target,
            "target_speed_kph": (
                float(target) * 3.6
                if isinstance(target, (int, float)) and math.isfinite(float(target))
                else None
            ),
            "longitudinal_speed_source": getattr(
                args, "longitudinal_speed_source", None
            ),
        },
        "source_identity": None,
        "quality": {
            "required_series": list(REQUIRED_SERIES),
            "problems": [f"{type(error).__name__}: {error}"],
            "warnings": [],
        },
        "series": {key: [] for key in REQUIRED_SERIES},
        "outputs": {
            "json": "longitudinal_response.json",
            "plot": "longitudinal_response.png",
        },
    }


def _atomic_outputs(evidence: dict[str, Any], output_dir: Path) -> None:
    directory = output_dir.expanduser().resolve()
    directory.mkdir(parents=True, exist_ok=True)
    json_path = directory / "longitudinal_response.json"
    plot_path = directory / "longitudinal_response.png"
    json_fd, json_name = tempfile.mkstemp(
        prefix=".longitudinal_response.json.", suffix=".staged", dir=directory
    )
    plot_fd, plot_name = tempfile.mkstemp(
        prefix=".longitudinal_response.png.", suffix=".staged", dir=directory
    )
    os.close(plot_fd)
    staged_json = Path(json_name)
    staged_plot = Path(plot_name)
    try:
        with os.fdopen(json_fd, "w", encoding="utf-8") as stream:
            json.dump(evidence, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        _plot_evidence(evidence, staged_plot)
        with staged_plot.open("rb") as stream:
            os.fsync(stream.fileno())
        os.replace(staged_plot, plot_path)
        os.replace(staged_json, json_path)
    finally:
        staged_json.unlink(missing_ok=True)
        staged_plot.unlink(missing_ok=True)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--route-file", required=True, type=Path)
    parser.add_argument("--result", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--profile-id", required=True)
    parser.add_argument("--target-speed-mps", required=True, type=float)
    parser.add_argument(
        "--longitudinal-speed-source",
        required=True,
        choices=("explicit_simulation_nominal",),
    )
    parser.add_argument("--actuation-map-coverage", type=Path)
    parser.add_argument(
        "--gated-acceleration-limit-mps2", type=float, default=1.5
    )
    parser.add_argument("--accel-pedal-saturation", type=float, default=0.399)
    parser.add_argument("--brake-pedal-saturation", type=float, default=0.599)
    parser.add_argument("--pedal-activity-epsilon", type=float, default=1.0e-3)
    return parser.parse_args(argv)


def run(
    args: argparse.Namespace,
    read_bag: Callable[
        [Path], tuple[dict[str, list[dict[str, Any]]], dict[str, str]]
    ] = _read_bag,
) -> int:
    try:
        source_identity = _source_identity(
            args.bag,
            args.route_file,
            args.result,
            profile_id=args.profile_id,
            target_speed_mps=args.target_speed_mps,
            longitudinal_speed_source=args.longitudinal_speed_source,
        )
        map_coverage = None
        coverage_path = getattr(args, "actuation_map_coverage", None)
        if coverage_path is not None:
            map_coverage = _map_coverage_context(
                coverage_path,
                profile_id=args.profile_id,
                target_speed_mps=args.target_speed_mps,
            )
        records, topic_types = read_bag(args.bag)
        evidence = build_evidence(
            records,
            topic_types,
            bag=args.bag,
            profile_id=args.profile_id,
            target_speed_mps=args.target_speed_mps,
            longitudinal_speed_source=args.longitudinal_speed_source,
            source_identity=source_identity,
            map_coverage=map_coverage,
            gated_acceleration_limit_mps2=getattr(
                args, "gated_acceleration_limit_mps2", 1.5
            ),
            accel_pedal_saturation=getattr(
                args, "accel_pedal_saturation", 0.399
            ),
            brake_pedal_saturation=getattr(
                args, "brake_pedal_saturation", 0.599
            ),
            pedal_activity_epsilon=getattr(
                args, "pedal_activity_epsilon", 1.0e-3
            ),
        )
    except Exception as error:  # Preserve machine-readable failure evidence.
        evidence = _failure_evidence(args, error)
    _atomic_outputs(evidence, args.output_dir)
    json_path = args.output_dir / "longitudinal_response.json"
    plot_path = args.output_dir / "longitudinal_response.png"
    print(f"longitudinal response status: {evidence['status']}")
    print(f"longitudinal response JSON: {json_path}")
    print(f"longitudinal response plot: {plot_path}")
    if evidence["status"] != "complete":
        for problem in evidence.get("quality", {}).get("problems", []):
            print(f"longitudinal response problem: {problem}", file=sys.stderr)
        return 1
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    return run(parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
