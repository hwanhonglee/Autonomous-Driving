#!/usr/bin/env python3
"""Validate and summarize the CTrack camera-source cadence A/B campaign.

The comparison is intentionally fail-closed.  It accepts only a terminal
``speed_30kph`` baseline and a terminal
``speed_30kph_camera_source_5hz`` candidate, both with passing CTrack straight
and turn trials.  The only allowed sensor-mapping semantic change is the six
RGB camera ``sensor_tick`` values changing from 0.0 to 0.2 seconds.

RTF is measured over the route evaluator's interval.  Camera receipt and stamp
rates come from ``analyze_e2e_latency.py`` and therefore cover the whole
rosbag.  This analyzer does not silently present those two windows as identical.
"""

from __future__ import annotations

import argparse
import copy
import csv
import hashlib
import io
import json
import math
import os
from pathlib import Path
import re
import statistics
import sys
import tempfile
from typing import Any, Mapping, Sequence

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402


SCHEMA_VERSION = 1
ANALYSIS_ID = "c_track_camera_source_cadence_ab_v1"
MAP_ID = "c_track_1_0_7"
CANONICAL_MAP = "C_track_1_0_7"
TRIAL_IDS = ("straight", "turn")
BASELINE_SELECTOR = "speed_30kph"
CANDIDATE_SELECTOR = "speed_30kph_camera_source_5hz"
BASELINE_MAPPING_NAME = "sensor_mapping_vad_fast_reliable_imu.yaml"
CANDIDATE_MAPPING_NAME = (
    "sensor_mapping_vad_fast_reliable_imu_camera_source_5hz.yaml"
)
OUTPUT_BASENAME = "camera_cadence_ab"
CAMERA_IDS = (
    "CAM_FRONT",
    "CAM_BACK",
    "CAM_FRONT_LEFT",
    "CAM_BACK_LEFT",
    "CAM_FRONT_RIGHT",
    "CAM_BACK_RIGHT",
)
MAXIMUM_CAMERA_STAMP_GAP_SEC = 0.25
ROUTE_MANAGER_THREAD_ENV = (
    "VAD_ROUTE_MANAGER_OPENBLAS_NUM_THREADS",
    "VAD_ROUTE_MANAGER_OMP_NUM_THREADS",
    "VAD_ROUTE_MANAGER_MKL_NUM_THREADS",
    "VAD_ROUTE_MANAGER_NUMEXPR_NUM_THREADS",
)

EXPECTED_BASELINE_WRAPPER_OPTIONS = (
    "--recommended",
    "--speed-30kph",
    "--visualize",
    "--capture-desktop",
)
EXPECTED_CANDIDATE_WRAPPER_OPTIONS = (
    "--recommended",
    "--speed-30kph",
    "--camera-source-5hz",
    "--visualize",
    "--capture-desktop",
)

INFERENCE_MARKER = "VAD inference complete:"
QUEUE_MARKER = "VAD frame queued:"
KEY_VALUE_RE = re.compile(r"([a-z][a-z0-9_]*)=([^\s]+)")
LOG_WALL_TIME_RE = re.compile(r"\[INFO\s+([0-9]+(?:\.[0-9]+)?)\]")
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")


class AnalysisError(RuntimeError):
    """Raised when an input cannot support a valid A/B comparison."""


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as error:
        raise AnalysisError(f"cannot hash {path}: {error}") from error
    return digest.hexdigest()


def _sha256_json(value: Any) -> str:
    try:
        encoded = json.dumps(
            value,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=True,
            allow_nan=False,
        ).encode("utf-8")
    except (TypeError, ValueError) as error:
        raise AnalysisError(f"cannot canonicalize JSON payload: {error}") from error
    return hashlib.sha256(encoded).hexdigest()


def _required_file(path: Path, label: str) -> Path:
    if path.is_symlink():
        raise AnalysisError(f"{label} must not be a symlink: {path}")
    if not path.is_file():
        raise AnalysisError(f"missing {label}: {path}")
    return path


def _read_json(path: Path, label: str) -> dict[str, Any]:
    _required_file(path, label)
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise AnalysisError(f"cannot read {label} {path}: {error}") from error
    if not isinstance(value, dict):
        raise AnalysisError(f"{label} must contain a JSON object: {path}")
    return value


def _read_yaml(path: Path, label: str) -> dict[str, Any]:
    _required_file(path, label)
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, yaml.YAMLError) as error:
        raise AnalysisError(f"cannot read {label} {path}: {error}") from error
    if not isinstance(value, dict):
        raise AnalysisError(f"{label} must contain a YAML mapping: {path}")
    return value


def _matrix_root(path: Path, label: str) -> Path:
    path = path.expanduser()
    if path.is_symlink():
        raise AnalysisError(f"{label} matrix root must not be a symlink: {path}")
    path = path.resolve()
    if not path.is_dir():
        raise AnalysisError(f"{label} matrix root is not a directory: {path}")
    return path


def _number(value: Any, label: str, *, positive: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise AnalysisError(f"{label} must be numeric, got {value!r}")
    result = float(value)
    if not math.isfinite(result):
        raise AnalysisError(f"{label} must be finite, got {value!r}")
    if positive and result <= 0.0:
        raise AnalysisError(f"{label} must be positive, got {result}")
    return result


def _integer(value: Any, label: str, *, positive: bool = False) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise AnalysisError(f"{label} must be an integer, got {value!r}")
    if positive and value <= 0:
        raise AnalysisError(f"{label} must be positive, got {value}")
    if not positive and value < 0:
        raise AnalysisError(f"{label} must be non-negative, got {value}")
    return value


def _close(actual: float, expected: float, label: str, tolerance: float = 1.0e-9) -> None:
    if not math.isclose(actual, expected, rel_tol=0.0, abs_tol=tolerance):
        raise AnalysisError(f"{label} changed: expected {expected}, got {actual}")


def _runtime_env(path: Path) -> dict[str, str]:
    _required_file(path, "runtime environment")
    values: dict[str, str] = {}
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except (OSError, UnicodeDecodeError) as error:
        raise AnalysisError(f"cannot read runtime environment {path}: {error}") from error
    for line_number, line in enumerate(lines, 1):
        if not line:
            continue
        key, separator, value = line.partition("=")
        if not separator or not key or key in values:
            raise AnalysisError(
                f"invalid or duplicate runtime.env entry at {path}:{line_number}"
            )
        values[key] = value
    return values


def _runtime_number(runtime: Mapping[str, str], key: str, label: str) -> float:
    value = runtime.get(key)
    if value is None:
        raise AnalysisError(f"{label} is missing")
    try:
        parsed = float(value)
    except ValueError as error:
        raise AnalysisError(f"{label} must be numeric, got {value!r}") from error
    return _number(parsed, label)


def _sha256_manifest(path: Path) -> dict[str, str]:
    _required_file(path, "SHA256 manifest")
    entries: dict[str, str] = {}
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except (OSError, UnicodeDecodeError) as error:
        raise AnalysisError(f"cannot read SHA256 manifest {path}: {error}") from error
    for line_number, line in enumerate(lines, 1):
        if not line.strip():
            continue
        fields = line.split(maxsplit=1)
        if len(fields) != 2:
            raise AnalysisError(f"invalid SHA256 manifest line {path}:{line_number}")
        digest, filename = fields
        filename = filename.lstrip("*")
        if not SHA256_RE.fullmatch(digest) or not filename or filename in entries:
            raise AnalysisError(f"invalid SHA256 manifest line {path}:{line_number}")
        entries[filename] = digest
    return entries


def _status_attempt_dir(
    root: Path, trial_id: str, trial_status: Mapping[str, Any]
) -> Path:
    declared = trial_status.get("attempt_directory")
    if not isinstance(declared, str) or not declared:
        raise AnalysisError(f"{trial_id} PASS status has no attempt_directory")
    declared_path = Path(declared)
    if (
        declared_path.name in {"", ".", ".."}
        or declared_path.parent.name != trial_id
        or declared_path.parent.parent.name != "trials"
    ):
        raise AnalysisError(f"unsafe {trial_id} attempt_directory: {declared}")
    attempt = root / "maps" / MAP_ID / "trials" / trial_id / declared_path.name
    if attempt.is_symlink() or not attempt.is_dir():
        raise AnalysisError(f"missing {trial_id} attempt directory: {attempt}")

    validation = trial_status.get("validation")
    if not isinstance(validation, str) or Path(validation).name != "matrix_validation.json":
        raise AnalysisError(f"{trial_id} PASS status has invalid validation path")
    if Path(validation).parent.name != declared_path.name:
        raise AnalysisError(f"{trial_id} validation is bound to a different attempt")
    return attempt


def _aggregate_map(aggregate: Mapping[str, Any]) -> Mapping[str, Any]:
    maps = aggregate.get("maps")
    if not isinstance(maps, list):
        raise AnalysisError("matrix aggregate has no maps array")
    matches = [entry for entry in maps if isinstance(entry, dict) and entry.get("map_id") == MAP_ID]
    if len(matches) != 1:
        raise AnalysisError(f"matrix aggregate must contain exactly one {MAP_ID} entry")
    return matches[0]


def _validate_terminal_status(value: Mapping[str, Any], label: str) -> None:
    if (
        value.get("map_id") != MAP_ID
        or value.get("canonical_name") != CANONICAL_MAP
        or value.get("runnable") is not True
        or value.get("status") != "PASS"
        or value.get("stage") != "complete"
    ):
        raise AnalysisError(f"{label} is not terminal {MAP_ID} PASS")
    trials = value.get("trials")
    if not isinstance(trials, dict) or set(trials) != set(TRIAL_IDS):
        raise AnalysisError(f"{label} does not contain exactly straight and turn")
    for trial_id in TRIAL_IDS:
        entry = trials.get(trial_id)
        if not isinstance(entry, dict) or entry.get("status") != "PASS":
            raise AnalysisError(f"{label} {trial_id} is not PASS")


def _validate_result(result: Mapping[str, Any], trial_id: str) -> dict[str, Any]:
    assessment = result.get("assessment")
    final = result.get("final")
    exposure = result.get("speed_exposure")
    context = result.get("profile_context")
    metrics = result.get("metrics")
    limits = result.get("limits")
    if (
        result.get("success") is not True
        or result.get("execution_mode") != "full_stack"
        or not isinstance(assessment, dict)
        or assessment.get("planning_architecture") != "vad_route_manager_hybrid"
        or assessment.get("route_completion") != "PASS"
        or not isinstance(final, dict)
        or final.get("goal_reached") is not True
        or final.get("route_status") != "goal_reached"
        or not isinstance(exposure, dict)
        or exposure.get("status") != "PASS"
        or not isinstance(context, dict)
        or context.get("longitudinal_velocity_source")
        != "explicit_simulation_nominal"
        or context.get("vad_geometry_evaluated") is not True
        or context.get("vad_velocity_evaluated") is not False
        or not isinstance(metrics, dict)
        or not isinstance(limits, dict)
    ):
        raise AnalysisError(f"{trial_id} result is not a passing full-stack speed trial")

    sim_elapsed = _number(
        metrics.get("sim_elapsed_sec"), f"{trial_id} sim elapsed", positive=True
    )
    wall_elapsed = _number(
        metrics.get("wall_elapsed_sec"), f"{trial_id} wall elapsed", positive=True
    )

    bounded_metrics = (
        ("maximum_absolute_cte_m", "maximum_absolute_cte_m"),
        ("maximum_trajectory_correction_m", "maximum_trajectory_correction_m"),
        ("maximum_lateral_acceleration_mps2", "maximum_lateral_acceleration_mps2"),
        ("maximum_observed_speed_mps", "maximum_observed_speed_mps"),
        ("maximum_speed_sample_gap_sec", "maximum_speed_sample_gap_sec"),
    )
    for metric_key, limit_key in bounded_metrics:
        observed = _number(metrics.get(metric_key), f"{trial_id} {metric_key}")
        limit = _number(limits.get(limit_key), f"{trial_id} limit {limit_key}")
        if observed > limit + 1.0e-9:
            raise AnalysisError(
                f"{trial_id} functional gate failed: {metric_key}={observed} > {limit}"
            )

    required_duration = _number(
        limits.get("minimum_sustained_speed_sec"),
        f"{trial_id} minimum sustained-speed duration",
    )
    observed_duration = _number(
        metrics.get("maximum_sustained_speed_duration_sec"),
        f"{trial_id} sustained-speed duration",
    )
    if observed_duration + 1.0e-9 < required_duration:
        raise AnalysisError(
            f"{trial_id} sustained-speed gate failed: {observed_duration} < {required_duration}"
        )

    return {
        "success": True,
        "reason": result.get("reason"),
        "route_completion": "PASS",
        "goal_reached": True,
        "speed_status": "PASS",
        "sim_elapsed_sec": sim_elapsed,
        "wall_elapsed_sec": wall_elapsed,
        "rtf": sim_elapsed / wall_elapsed,
        "maximum_absolute_cte_m": _number(
            metrics.get("maximum_absolute_cte_m"), f"{trial_id} max CTE"
        ),
        "maximum_trajectory_correction_m": _number(
            metrics.get("maximum_trajectory_correction_m"),
            f"{trial_id} max trajectory correction",
        ),
        "maximum_lateral_acceleration_mps2": _number(
            metrics.get("maximum_lateral_acceleration_mps2"),
            f"{trial_id} max lateral acceleration",
        ),
        "maximum_observed_speed_mps": _number(
            metrics.get("maximum_observed_speed_mps"),
            f"{trial_id} max observed speed",
        ),
        "maximum_speed_sample_gap_sec": _number(
            metrics.get("maximum_speed_sample_gap_sec"),
            f"{trial_id} max speed sample gap",
        ),
        "maximum_sustained_speed_duration_sec": observed_duration,
        "minimum_sustained_speed_sec": required_duration,
    }


def _validate_latency(latency: Mapping[str, Any], trial_id: str) -> dict[str, Any]:
    selected = latency.get("selected_topics")
    event_rates = latency.get("event_rates")
    bundle = latency.get("camera_bundle")
    acceptance = latency.get("candidate_front_acceptance")
    if not isinstance(selected, dict) or not isinstance(event_rates, dict):
        raise AnalysisError(f"{trial_id} latency lacks selected topics/event rates")
    front_topic = selected.get("sensor")
    if not isinstance(front_topic, str) or not front_topic.endswith(
        "/CAM_FRONT/camera_info"
    ):
        raise AnalysisError(f"{trial_id} latency did not select front camera_info")
    front_rate = event_rates.get(front_topic)
    if not isinstance(front_rate, dict):
        raise AnalysisError(f"{trial_id} latency lacks front-camera event rate")
    front_count = _integer(
        front_rate.get("count"), f"{trial_id} front camera count", positive=True
    )
    if front_count < 2:
        raise AnalysisError(f"{trial_id} front camera count must be at least two")
    effective_rate = _number(
        front_rate.get("effective_receipt_rate_hz"),
        f"{trial_id} front effective receipt rate",
        positive=True,
    )
    stamp_rate = _number(
        front_rate.get("stamp_rate_hz"),
        f"{trial_id} front stamp rate",
        positive=True,
    )
    receipt_period = front_rate.get("receipt_period_sec")
    stamp_period = front_rate.get("stamp_period_sec")
    if (
        not isinstance(receipt_period, dict)
        or receipt_period.get("available") is not True
        or not isinstance(stamp_period, dict)
        or stamp_period.get("available") is not True
    ):
        raise AnalysisError(f"{trial_id} front camera period evidence is unavailable")
    receipt_mean = _number(
        receipt_period.get("mean"), f"{trial_id} front mean receipt period", positive=True
    )
    stamp_median = _number(
        stamp_period.get("median"), f"{trial_id} front median stamp period", positive=True
    )
    _close(
        effective_rate,
        1.0 / receipt_mean,
        f"{trial_id} front effective receipt rate formula",
    )
    _close(
        stamp_rate,
        1.0 / stamp_median,
        f"{trial_id} front stamp rate formula",
    )
    if not 4.9 <= stamp_rate <= 5.1:
        raise AnalysisError(f"{trial_id} front stamp rate is not approximately 5 Hz")

    camera_stamp_gaps: dict[str, float] = {}
    for camera_id in CAMERA_IDS:
        topic = f"/sensing/camera/{camera_id}/camera_info"
        rate = event_rates.get(topic)
        if not isinstance(rate, dict):
            raise AnalysisError(f"{trial_id} latency lacks camera event rate for {topic}")
        camera_stamp_period = rate.get("stamp_period_sec")
        if (
            not isinstance(camera_stamp_period, dict)
            or camera_stamp_period.get("available") is not True
        ):
            raise AnalysisError(
                f"{trial_id} camera stamp-period evidence is unavailable for {topic}"
            )
        maximum_gap = _number(
            camera_stamp_period.get("max"),
            f"{trial_id} maximum camera stamp gap for {topic}",
            positive=True,
        )
        camera_stamp_rate = _number(
            rate.get("stamp_rate_hz"),
            f"{trial_id} camera stamp rate for {topic}",
            positive=True,
        )
        if maximum_gap > MAXIMUM_CAMERA_STAMP_GAP_SEC + 1.0e-9:
            raise AnalysisError(
                f"{trial_id} maximum camera stamp gap exceeds "
                f"{MAXIMUM_CAMERA_STAMP_GAP_SEC:.3f} s for {topic}: "
                f"{maximum_gap:.6f} s"
            )
        if not 4.9 <= camera_stamp_rate <= 5.1:
            raise AnalysisError(
                f"{trial_id} camera stamp rate is not approximately 5 Hz for {topic}"
            )
        camera_stamp_gaps[camera_id] = maximum_gap

    if not isinstance(bundle, dict) or bundle.get("available") is not True:
        raise AnalysisError(f"{trial_id} six-camera bundle evidence is unavailable")
    if _integer(bundle.get("camera_count"), f"{trial_id} camera count") != 6:
        raise AnalysisError(f"{trial_id} bundle does not contain six cameras")
    bundle_front = _integer(
        bundle.get("front_frame_count"),
        f"{trial_id} bundle front count",
        positive=True,
    )
    matched = _integer(
        bundle.get("matched_bundle_count"),
        f"{trial_id} matched bundle count",
        positive=True,
    )
    if matched > bundle_front or bundle_front != front_count:
        raise AnalysisError(f"{trial_id} bundle/front counts are inconsistent")
    coverage = _number(
        bundle.get("bundle_coverage_percent"), f"{trial_id} bundle coverage"
    )
    recomputed_coverage = 100.0 * matched / bundle_front
    _close(coverage, recomputed_coverage, f"{trial_id} bundle coverage")
    if coverage < 99.0:
        raise AnalysisError(
            f"{trial_id} six-camera bundle coverage is below 99%: {coverage}"
        )

    if not isinstance(acceptance, dict) or acceptance.get("available") is not True:
        raise AnalysisError(f"{trial_id} candidate/front acceptance is unavailable")
    candidate_count = _integer(
        acceptance.get("candidate_count"),
        f"{trial_id} candidate count",
        positive=True,
    )
    acceptance_front = _integer(
        acceptance.get("front_count"),
        f"{trial_id} acceptance front count",
        positive=True,
    )
    acceptance_percent = _number(
        acceptance.get("acceptance_percent"), f"{trial_id} candidate acceptance"
    )
    if acceptance_front != front_count:
        raise AnalysisError(f"{trial_id} candidate/front input counts are inconsistent")
    _close(
        acceptance_percent,
        100.0 * candidate_count / acceptance_front,
        f"{trial_id} candidate acceptance",
    )
    count_delta = candidate_count - acceptance_front
    if acceptance_percent < 99.0 or abs(count_delta) > 1:
        raise AnalysisError(
            f"{trial_id} whole-bag candidate/front boundary gate failed: "
            f"{candidate_count}/{acceptance_front}={acceptance_percent}%"
        )

    stamp_span = bundle.get("stamp_span_sec")
    stamp_span_max = None
    if isinstance(stamp_span, dict) and stamp_span.get("available") is True:
        stamp_span_max = _number(
            stamp_span.get("max"), f"{trial_id} maximum bundle stamp span"
        )

    warnings = latency.get("data_quality_warnings")
    if warnings is None:
        warnings = []
    if not isinstance(warnings, list) or not all(isinstance(item, str) for item in warnings):
        raise AnalysisError(f"{trial_id} latency data-quality warnings are malformed")

    return {
        "window": "whole_rosbag",
        "front_topic": front_topic,
        "front_count": front_count,
        "front_effective_receipt_rate_hz": effective_rate,
        "front_stamp_rate_hz": stamp_rate,
        "maximum_camera_stamp_gap_sec": max(camera_stamp_gaps.values()),
        "camera_stamp_gap_sec": camera_stamp_gaps,
        "bundle": {
            "camera_count": 6,
            "front_frame_count": bundle_front,
            "matched_bundle_count": matched,
            "coverage_percent": coverage,
            "maximum_stamp_span_sec": stamp_span_max,
        },
        "candidate_front_acceptance": {
            "candidate_count": candidate_count,
            "front_count": acceptance_front,
            "count_delta": count_delta,
            "acceptance_percent": acceptance_percent,
            "gate": "whole-bag acceptance >=99% and absolute count delta <=1",
            "interpretation": (
                "whole-bag boundary-compatible count ratio only; this is not a "
                "route-window exactness claim and the VAD output does not preserve "
                "causal front-camera source identity"
            ),
        },
        "route_window": {
            "status": "UNAVAILABLE",
            "reason": "this analyzer does not reread the rosbag",
            "exact_bundle_or_acceptance_claimed": False,
        },
        "data_quality_warnings": list(warnings),
    }


def _parse_log_fields(line: str, marker: str, label: str) -> dict[str, str]:
    suffix = line.split(marker, 1)[1]
    pairs = KEY_VALUE_RE.findall(suffix)
    values: dict[str, str] = {}
    for key, value in pairs:
        if key in values:
            raise AnalysisError(f"duplicate {key} in {label} log line")
        values[key] = value
    return values


def _log_int(fields: Mapping[str, str], key: str, label: str) -> int:
    value = fields.get(key)
    if value is None or not re.fullmatch(r"[0-9]+", value):
        raise AnalysisError(f"{label} log line lacks integer {key}")
    return int(value)


def _log_wall_time(line: str, label: str) -> float:
    match = LOG_WALL_TIME_RE.search(line)
    if match is None:
        raise AnalysisError(f"{label} log line lacks an INFO wall timestamp")
    value = float(match.group(1))
    if not math.isfinite(value) or value <= 0.0:
        raise AnalysisError(f"{label} log line has an invalid wall timestamp")
    return value


def _percentile(values: Sequence[float], percentile: float) -> float:
    if not values:
        raise AnalysisError("cannot summarize an empty numeric sequence")
    ordered = sorted(values)
    position = (len(ordered) - 1) * percentile / 100.0
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return float(ordered[lower])
    fraction = position - lower
    return float(ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction)


def _inference_summary(values: Sequence[float]) -> dict[str, Any]:
    if not values:
        raise AnalysisError("stack log has no inference latency samples")
    return {
        "count": len(values),
        "min": min(values),
        "mean": statistics.fmean(values),
        "median": statistics.median(values),
        "p95": _percentile(values, 95.0),
        "p99": _percentile(values, 99.0),
        "max": max(values),
        "cold_start_included": True,
    }


def _validate_stack_log(
    path: Path,
    recorder_path: Path,
    trial_id: str,
    camera_evidence: Mapping[str, Any],
) -> dict[str, Any]:
    _required_file(path, "VAD stack log")
    try:
        lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError as error:
        raise AnalysisError(f"cannot read VAD stack log {path}: {error}") from error

    inference_records: list[dict[str, int | float | bool]] = []
    queue_records: list[dict[str, int | float]] = []
    for line in lines:
        if INFERENCE_MARKER in line:
            fields = _parse_log_fields(line, INFERENCE_MARKER, f"{trial_id} inference")
            required = {
                "inference_ms",
                "published",
                "published_count",
                "mailbox_taken",
                "coalesced_drops",
            }
            if not required.issubset(fields):
                missing = sorted(required - set(fields))
                raise AnalysisError(
                    f"{trial_id} inference log line lacks fields: {', '.join(missing)}"
                )
            try:
                inference_ms = float(fields["inference_ms"])
            except ValueError as error:
                raise AnalysisError(f"{trial_id} has invalid inference_ms") from error
            if not math.isfinite(inference_ms) or inference_ms < 0.0:
                raise AnalysisError(f"{trial_id} has invalid inference_ms={inference_ms}")
            if fields["published"] not in {"true", "false"}:
                raise AnalysisError(f"{trial_id} has invalid published flag")
            record: dict[str, int | float | bool] = {
                "inference_ms": inference_ms,
                "published": fields["published"] == "true",
                "published_count": _log_int(fields, "published_count", trial_id),
                "mailbox_taken": _log_int(fields, "mailbox_taken", trial_id),
                "coalesced_drops": _log_int(fields, "coalesced_drops", trial_id),
            }
            inference_records.append(record)
        if QUEUE_MARKER in line:
            fields = _parse_log_fields(line, QUEUE_MARKER, f"{trial_id} queue")
            required = {
                "source_stamp_ns",
                "assembled",
                "capacity_pruned",
                "superseded",
                "mailbox_submitted",
                "coalesced_drops",
                "received_images_min",
                "received_images_max",
            }
            if not required.issubset(fields):
                missing = sorted(required - set(fields))
                raise AnalysisError(
                    f"{trial_id} queue log line lacks fields: {', '.join(missing)}"
                )
            record: dict[str, int | float] = {
                key: _log_int(fields, key, trial_id) for key in sorted(required)
            }
            record["wall_time_sec"] = _log_wall_time(line, f"{trial_id} queue")
            queue_records.append(record)

    if not inference_records or not queue_records:
        raise AnalysisError(f"{trial_id} stack log lacks VAD inference/queue evidence")
    if any(record["published"] is not True for record in inference_records):
        raise AnalysisError(f"{trial_id} VAD inference contains unpublished output")
    if any(record["coalesced_drops"] != 0 for record in inference_records):
        raise AnalysisError(f"{trial_id} VAD inference reports coalesced drops")
    for previous, current in zip(inference_records, inference_records[1:]):
        if (
            current["published_count"] < previous["published_count"]
            or current["mailbox_taken"] < previous["mailbox_taken"]
        ):
            raise AnalysisError(f"{trial_id} VAD inference counters are not monotonic")
    for previous, current in zip(queue_records, queue_records[1:]):
        if (
            current["assembled"] < previous["assembled"]
            or current["mailbox_submitted"] < previous["mailbox_submitted"]
        ):
            raise AnalysisError(f"{trial_id} VAD queue counters are not monotonic")
    if any(
        record["capacity_pruned"] != 0
        or record["coalesced_drops"] != 0
        or record["assembled"] != record["mailbox_submitted"]
        for record in queue_records
    ):
        raise AnalysisError(
            f"{trial_id} VAD queue reports pruning, coalesced drops, or mailbox loss"
        )

    initial_superseded = int(queue_records[0]["superseded"])
    if initial_superseded not in {0, 1} or any(
        int(record["superseded"]) != initial_superseded
        for record in queue_records
    ):
        raise AnalysisError(
            f"{trial_id} VAD queue reports runtime supersession or more than one "
            "startup supersession"
        )
    superseded_classification = "none"
    recorder_start_wall_sec = None
    if initial_superseded == 1:
        _required_file(recorder_path, "rosbag recorder log")
        try:
            recorder_log = recorder_path.read_text(
                encoding="utf-8", errors="replace"
            )
        except OSError as error:
            raise AnalysisError(
                f"cannot read {trial_id} rosbag recorder log: {error}"
            ) from error
        recorder_match = re.search(
            r"\[INFO\s+([0-9]+(?:\.[0-9]+)?)\].*?Recording\.\.\.",
            recorder_log,
        )
        if recorder_match is None:
            raise AnalysisError(
                f"{trial_id} recorder log lacks a timestamped Recording marker"
            )
        recorder_start_wall_sec = float(recorder_match.group(1))
        first_queue_wall_sec = float(queue_records[0]["wall_time_sec"])
        bundle = camera_evidence["bundle"]
        acceptance = camera_evidence["candidate_front_acceptance"]
        startup_relation_valid = all(
            int(record["received_images_min"])
            == int(record["received_images_max"])
            == int(record["assembled"]) + 1
            for record in queue_records
        )
        if (
            first_queue_wall_sec >= recorder_start_wall_sec
            or not startup_relation_valid
            or not math.isclose(
                float(bundle["coverage_percent"]),
                100.0,
                rel_tol=0.0,
                abs_tol=1.0e-9,
            )
            or int(acceptance["candidate_count"]) != int(acceptance["front_count"])
            or not math.isclose(
                float(acceptance["acceptance_percent"]),
                100.0,
                rel_tol=0.0,
                abs_tol=1.0e-9,
            )
        ):
            raise AnalysisError(
                f"{trial_id} startup supersession is not proven to precede the "
                "recorded evaluation window"
            )
        superseded_classification = "startup_before_recorder"

    final = inference_records[-1]
    published = int(final["published_count"])
    mailbox_taken = int(final["mailbox_taken"])
    if published <= 0 or published != mailbox_taken:
        raise AnalysisError(
            f"{trial_id} VAD published/mailbox counters differ: "
            f"{published} != {mailbox_taken}"
        )
    last_queue = queue_records[-1]
    return {
        "scope": "full_stack_process_lifetime",
        "inference_complete_observation_count": len(inference_records),
        "queue_observation_count": len(queue_records),
        "published_count": published,
        "mailbox_taken": mailbox_taken,
        "capacity_pruned_max": max(item["capacity_pruned"] for item in queue_records),
        "superseded_max": max(item["superseded"] for item in queue_records),
        "superseded_classification": superseded_classification,
        "recorder_start_wall_sec": recorder_start_wall_sec,
        "coalesced_drops_max": max(
            max(item["coalesced_drops"] for item in queue_records),
            max(int(item["coalesced_drops"]) for item in inference_records),
        ),
        "last_observed_assembled": last_queue["assembled"],
        "last_observed_mailbox_submitted": last_queue["mailbox_submitted"],
        "inference_ms": _inference_summary(
            [float(item["inference_ms"]) for item in inference_records]
        ),
    }


def _load_mapping_provenance(
    attempt: Path, runtime: Mapping[str, str], trial_id: str, arm_name: str
) -> tuple[dict[str, Any], dict[str, Any]]:
    provenance = attempt / "sensor_mapping_provenance"
    mapping_path = provenance / "sensor_mapping.yaml"
    sums_path = provenance / "SHA256SUMS"
    mapping = _read_yaml(mapping_path, f"{arm_name} {trial_id} sensor mapping")
    digest = _sha256_file(mapping_path)
    sums = _sha256_manifest(sums_path)
    if sums != {"sensor_mapping.yaml": digest}:
        raise AnalysisError(f"{arm_name} {trial_id} sensor-mapping manifest changed")
    if runtime.get("SENSOR_MAPPING_SHA256") != digest:
        raise AnalysisError(f"{arm_name} {trial_id} runtime mapping SHA256 changed")
    runtime_file = runtime.get("SENSOR_MAPPING_FILE")
    if not isinstance(runtime_file, str) or not runtime_file:
        raise AnalysisError(f"{arm_name} {trial_id} runtime has no sensor mapping path")
    expected_name = (
        BASELINE_MAPPING_NAME if arm_name == "baseline" else CANDIDATE_MAPPING_NAME
    )
    if Path(runtime_file).name != expected_name:
        raise AnalysisError(
            f"{arm_name} {trial_id} selected unexpected mapping {runtime_file}"
        )
    return mapping, {
        "path": str(mapping_path.resolve()),
        "sha256": digest,
        "runtime_file": runtime_file,
    }


def _load_trial(
    root: Path,
    arm_name: str,
    selector: str,
    trial_id: str,
    trial_status: Mapping[str, Any],
) -> tuple[dict[str, Any], dict[str, Any]]:
    attempt = _status_attempt_dir(root, trial_id, trial_status)
    validation_path = attempt / "matrix_validation.json"
    result_path = attempt / "result.json"
    route_path = attempt / "source_route.json"
    latency_path = attempt / "latency" / "e2e_latency.json"
    stack_path = attempt / "stack.log"
    recorder_path = attempt / "recorder.log"
    runtime_path = attempt / "runtime.env"

    validation = _read_json(validation_path, f"{arm_name} {trial_id} validation")
    if (
        validation.get("status") != "PASS"
        or validation.get("map_id") != MAP_ID
        or validation.get("trial_id") != trial_id
        or validation.get("runtime_profile_selector") != selector
        or not isinstance(validation.get("result"), dict)
        or validation["result"].get("route_completion") != "PASS"
        or validation["result"].get("goal_reached") is not True
        or not isinstance(validation.get("speed_contract"), dict)
        or validation["speed_contract"].get("status") != "PASS"
    ):
        raise AnalysisError(f"{arm_name} {trial_id} matrix validation is not PASS")

    result = _read_json(result_path, f"{arm_name} {trial_id} result")
    route = _read_json(route_path, f"{arm_name} {trial_id} source route")
    expected_scenario = "straight" if trial_id == "straight" else {"left", "right"}
    scenario = route.get("scenario")
    if (
        route.get("town") != CANONICAL_MAP
        or route.get("weather") != "ClearNoon"
        or (
            scenario != expected_scenario
            if isinstance(expected_scenario, str)
            else scenario not in expected_scenario
        )
        or not isinstance(route.get("route"), list)
        or not route["route"]
    ):
        raise AnalysisError(f"{arm_name} {trial_id} source-route identity is invalid")

    functional = _validate_result(result, trial_id)
    latency = _validate_latency(
        _read_json(latency_path, f"{arm_name} {trial_id} latency"), trial_id
    )
    vad = _validate_stack_log(stack_path, recorder_path, trial_id, latency)
    runtime = _runtime_env(runtime_path)
    for field in ROUTE_MANAGER_THREAD_ENV:
        if runtime.get(field) != "1":
            raise AnalysisError(
                f"{arm_name} {trial_id} runtime does not attest {field}=1"
            )
    mapping, mapping_identity = _load_mapping_provenance(
        attempt, runtime, trial_id, arm_name
    )

    if arm_name == "candidate":
        if runtime.get("CAMERA_SOURCE_5HZ") != "true":
            raise AnalysisError(f"candidate {trial_id} did not enable camera source 5 Hz")
        _close(
            _runtime_number(
                runtime,
                "CAMERA_SOURCE_SENSOR_TICK_SEC",
                f"candidate {trial_id} runtime sensor tick",
            ),
            0.2,
            f"candidate {trial_id} runtime sensor tick",
        )
        _close(
            _runtime_number(
                runtime,
                "CAMERA_ROS_PUBLISH_HZ",
                f"candidate {trial_id} ROS publish rate",
            ),
            5.0,
            f"candidate {trial_id} ROS publish rate",
        )
    else:
        if runtime.get("CAMERA_SOURCE_5HZ") not in {None, "false"}:
            raise AnalysisError(
                f"baseline {trial_id} unexpectedly enabled camera source 5 Hz"
            )
        if "CAMERA_SOURCE_SENSOR_TICK_SEC" in runtime:
            _close(
                _runtime_number(
                    runtime,
                    "CAMERA_SOURCE_SENSOR_TICK_SEC",
                    f"baseline {trial_id} runtime sensor tick",
                ),
                0.0,
                f"baseline {trial_id} runtime sensor tick",
            )
        if "CAMERA_ROS_PUBLISH_HZ" in runtime:
            _close(
                _runtime_number(
                    runtime,
                    "CAMERA_ROS_PUBLISH_HZ",
                    f"baseline {trial_id} ROS publish rate",
                ),
                5.0,
                f"baseline {trial_id} ROS publish rate",
            )

    route_sha = _sha256_file(route_path)
    trial = {
        "trial_id": trial_id,
        "scenario": scenario,
        "attempt_directory": str(attempt.resolve()),
        "source_route": {
            "path": str(route_path.resolve()),
            "sha256": route_sha,
            "canonical_payload_sha256": _sha256_json(route),
            "route_length_m": _number(
                route.get("route_length_m"), f"{arm_name} {trial_id} route length", positive=True
            ),
            "start_spawn_index": route.get("start_spawn_index"),
            "goal_spawn_index": route.get("goal_spawn_index"),
        },
        "functional": functional,
        "camera_vad": latency,
        "vad_runtime": vad,
        "measurement": {
            "rtf": functional["rtf"],
            "front_effective_receipt_rate_hz": latency[
                "front_effective_receipt_rate_hz"
            ],
            "front_stamp_rate_hz": latency["front_stamp_rate_hz"],
            "maximum_camera_stamp_gap_sec": latency[
                "maximum_camera_stamp_gap_sec"
            ],
            "receipt_efficiency_ratio": latency[
                "front_effective_receipt_rate_hz"
            ]
            / (latency["front_stamp_rate_hz"] * functional["rtf"]),
        },
        "input_files": {
            "matrix_validation_sha256": _sha256_file(validation_path),
            "result_sha256": _sha256_file(result_path),
            "latency_sha256": _sha256_file(latency_path),
            "stack_log_sha256": _sha256_file(stack_path),
            "recorder_log_sha256": _sha256_file(recorder_path),
            "runtime_env_sha256": _sha256_file(runtime_path),
        },
        "sensor_mapping": mapping_identity,
    }
    return trial, mapping


def _runtime_profile_invariants(
    baseline: Mapping[str, Any], candidate: Mapping[str, Any]
) -> dict[str, Any]:
    baseline_options = baseline.get("wrapper_options")
    candidate_options = candidate.get("wrapper_options")
    if baseline_options != list(EXPECTED_BASELINE_WRAPPER_OPTIONS):
        raise AnalysisError("baseline wrapper options are not pinned")
    if candidate_options != list(EXPECTED_CANDIDATE_WRAPPER_OPTIONS):
        raise AnalysisError("candidate wrapper options are not pinned")
    camera_contract = candidate.get("camera_source_contract")
    if (
        not isinstance(camera_contract, dict)
        or camera_contract.get("sensor_count") != 6
        or camera_contract.get("sensor_tick_sec") != 0.2
        or camera_contract.get("ros_publish_frequency_hz") != 5.0
        or camera_contract.get("maximum_stamp_gap_sec")
        != MAXIMUM_CAMERA_STAMP_GAP_SEC
    ):
        raise AnalysisError("candidate camera-source contract is not pinned")
    normalized_baseline = copy.deepcopy(dict(baseline))
    normalized_candidate = copy.deepcopy(dict(candidate))
    for profile in (normalized_baseline, normalized_candidate):
        profile.pop("id", None)
        profile.pop("wrapper_options", None)
        profile.pop("camera_source_contract", None)
    if normalized_baseline != normalized_candidate:
        raise AnalysisError(
            "runtime profiles differ outside candidate identity/camera-source contract"
        )
    return {
        "baseline_wrapper_options": list(EXPECTED_BASELINE_WRAPPER_OPTIONS),
        "candidate_wrapper_options": list(EXPECTED_CANDIDATE_WRAPPER_OPTIONS),
        "other_runtime_profile_fields_equal": True,
    }


def _load_arm(root: Path, arm_name: str, selector: str) -> tuple[dict[str, Any], dict[str, Any]]:
    root = _matrix_root(root, arm_name)
    plan_path = root / "matrix_plan.json"
    aggregate_path = root / "aggregate.json"
    status_path = root / "maps" / MAP_ID / "status.json"
    plan = _read_json(plan_path, f"{arm_name} matrix plan")
    aggregate = _read_json(aggregate_path, f"{arm_name} aggregate")
    status = _read_json(status_path, f"{arm_name} CTrack status")
    if plan.get("runtime_profile_selector") != selector:
        raise AnalysisError(
            f"{arm_name} selector must be {selector}, got "
            f"{plan.get('runtime_profile_selector')!r}"
        )
    if aggregate.get("runtime_profile_selector") != selector:
        raise AnalysisError(f"{arm_name} aggregate selector differs from its plan")
    profile = plan.get("runtime_profile")
    if not isinstance(profile, dict) or aggregate.get("runtime_profile") != profile:
        raise AnalysisError(f"{arm_name} runtime profile is not plan/aggregate bound")
    _validate_terminal_status(status, f"{arm_name} map status")
    aggregate_entry = _aggregate_map(aggregate)
    _validate_terminal_status(aggregate_entry, f"{arm_name} aggregate map")
    if aggregate_entry != status:
        raise AnalysisError(f"{arm_name} aggregate CTrack status differs from status.json")

    status_trials = status["trials"]
    trials: dict[str, Any] = {}
    mappings: list[dict[str, Any]] = []
    for trial_id in TRIAL_IDS:
        trial, mapping = _load_trial(
            root, arm_name, selector, trial_id, status_trials[trial_id]
        )
        trials[trial_id] = trial
        mappings.append(mapping)
    if mappings[0] != mappings[1]:
        raise AnalysisError(f"{arm_name} straight/turn sensor mappings differ")
    mapping_hashes = {trials[item]["sensor_mapping"]["sha256"] for item in TRIAL_IDS}
    if len(mapping_hashes) != 1:
        raise AnalysisError(f"{arm_name} straight/turn sensor-mapping SHA256 differs")

    arm = {
        "matrix_root": str(root),
        "runtime_profile_selector": selector,
        "runtime_profile_id": profile.get("id"),
        "matrix_plan_sha256": _sha256_file(plan_path),
        "aggregate_sha256": _sha256_file(aggregate_path),
        "status_sha256": _sha256_file(status_path),
        "aggregate_status": aggregate.get("status"),
        "runtime_profile": profile,
        "sensor_mapping_sha256": next(iter(mapping_hashes)),
        "trials": trials,
    }
    return arm, mappings[0]


def _semantic_differences(
    baseline: Any, candidate: Any, path: tuple[str, ...] = ()
) -> list[dict[str, Any]]:
    if isinstance(baseline, dict) and isinstance(candidate, dict):
        if set(baseline) != set(candidate):
            return [
                {
                    "path": ".".join(path),
                    "baseline": sorted(str(key) for key in baseline),
                    "candidate": sorted(str(key) for key in candidate),
                }
            ]
        output: list[dict[str, Any]] = []
        for key in sorted(baseline, key=str):
            output.extend(
                _semantic_differences(
                    baseline[key], candidate[key], (*path, str(key))
                )
            )
        return output
    if isinstance(baseline, list) and isinstance(candidate, list):
        if len(baseline) != len(candidate):
            return [{"path": ".".join(path), "baseline": baseline, "candidate": candidate}]
        output = []
        for index, (left, right) in enumerate(zip(baseline, candidate)):
            output.extend(
                _semantic_differences(left, right, (*path, str(index)))
            )
        return output
    if baseline != candidate or type(baseline) is not type(candidate):
        return [
            {
                "path": ".".join(path),
                "baseline": baseline,
                "candidate": candidate,
            }
        ]
    return []


def _mapping_delta(
    baseline: Mapping[str, Any], candidate: Mapping[str, Any]
) -> dict[str, Any]:
    baseline_sensors = baseline.get("sensor_mappings")
    candidate_sensors = candidate.get("sensor_mappings")
    if not isinstance(baseline_sensors, dict) or not isinstance(candidate_sensors, dict):
        raise AnalysisError("sensor mappings lack sensor_mappings dictionaries")
    camera_keys = sorted(
        key
        for key, value in baseline_sensors.items()
        if isinstance(value, dict) and value.get("carla_type") == "sensor.camera.rgb"
    )
    candidate_camera_keys = sorted(
        key
        for key, value in candidate_sensors.items()
        if isinstance(value, dict) and value.get("carla_type") == "sensor.camera.rgb"
    )
    if len(camera_keys) != 6 or candidate_camera_keys != camera_keys:
        raise AnalysisError("A/B sensor mappings do not contain the same six RGB cameras")
    expected_paths = {
        f"sensor_mappings.{key}.parameters.sensor_tick" for key in camera_keys
    }
    for key in camera_keys:
        try:
            baseline_tick = _number(
                baseline_sensors[key]["parameters"]["sensor_tick"],
                f"baseline {key} sensor_tick",
            )
            candidate_tick = _number(
                candidate_sensors[key]["parameters"]["sensor_tick"],
                f"candidate {key} sensor_tick",
            )
        except (KeyError, TypeError) as error:
            raise AnalysisError(f"camera {key} lacks sensor_tick") from error
        _close(baseline_tick, 0.0, f"baseline {key} sensor_tick")
        _close(candidate_tick, 0.2, f"candidate {key} sensor_tick")

    differences = _semantic_differences(dict(baseline), dict(candidate))
    actual_paths = {item["path"] for item in differences}
    if actual_paths != expected_paths or len(differences) != 6:
        unexpected = sorted(actual_paths - expected_paths)
        missing = sorted(expected_paths - actual_paths)
        raise AnalysisError(
            "sensor mapping delta is not exactly six camera sensor_tick values; "
            f"unexpected={unexpected}, missing={missing}"
        )
    return {
        "status": "PASS",
        "camera_count": 6,
        "camera_keys": camera_keys,
        "field": "parameters.sensor_tick",
        "baseline_sec": 0.0,
        "candidate_sec": 0.2,
        "other_fields_equal": True,
        "semantic_changes": differences,
    }


def _metric_comparison(baseline: float, candidate: float) -> dict[str, float]:
    if not math.isfinite(baseline) or baseline <= 0.0:
        raise AnalysisError(f"comparison baseline must be positive, got {baseline}")
    if not math.isfinite(candidate) or candidate <= 0.0:
        raise AnalysisError(f"comparison candidate must be positive, got {candidate}")
    return {
        "baseline": baseline,
        "candidate": candidate,
        "candidate_to_baseline_ratio": candidate / baseline,
        "candidate_change_percent": 100.0 * (candidate / baseline - 1.0),
    }


def analyze(baseline_root: Path, candidate_root: Path) -> dict[str, Any]:
    """Build a validated, deterministic comparison report."""
    baseline, baseline_mapping = _load_arm(
        baseline_root, "baseline", BASELINE_SELECTOR
    )
    candidate, candidate_mapping = _load_arm(
        candidate_root, "candidate", CANDIDATE_SELECTOR
    )
    profile_invariants = _runtime_profile_invariants(
        baseline["runtime_profile"], candidate["runtime_profile"]
    )
    mapping_delta = _mapping_delta(baseline_mapping, candidate_mapping)

    route_identity: dict[str, Any] = {}
    comparisons: dict[str, Any] = {}
    for trial_id in TRIAL_IDS:
        baseline_trial = baseline["trials"][trial_id]
        candidate_trial = candidate["trials"][trial_id]
        baseline_route = baseline_trial["source_route"]
        candidate_route = candidate_trial["source_route"]
        if baseline_route["sha256"] != candidate_route["sha256"]:
            raise AnalysisError(f"{trial_id} source_route SHA256 differs across arms")
        if (
            baseline_route["canonical_payload_sha256"]
            != candidate_route["canonical_payload_sha256"]
        ):
            raise AnalysisError(f"{trial_id} source_route payload differs across arms")
        route_identity[trial_id] = {
            "status": "PASS",
            "sha256": baseline_route["sha256"],
            "canonical_payload_sha256": baseline_route[
                "canonical_payload_sha256"
            ],
            "payload_equal": True,
            "route_length_m": baseline_route["route_length_m"],
            "scenario": baseline_trial["scenario"],
            "start_spawn_index": baseline_route["start_spawn_index"],
            "goal_spawn_index": baseline_route["goal_spawn_index"],
        }
        baseline_measurement = baseline_trial["measurement"]
        candidate_measurement = candidate_trial["measurement"]
        comparisons[trial_id] = {
            "rtf": _metric_comparison(
                baseline_measurement["rtf"], candidate_measurement["rtf"]
            ),
            "front_effective_receipt_rate_hz": _metric_comparison(
                baseline_measurement["front_effective_receipt_rate_hz"],
                candidate_measurement["front_effective_receipt_rate_hz"],
            ),
            "front_stamp_rate_hz": _metric_comparison(
                baseline_measurement["front_stamp_rate_hz"],
                candidate_measurement["front_stamp_rate_hz"],
            ),
            "maximum_camera_stamp_gap_sec": _metric_comparison(
                baseline_measurement["maximum_camera_stamp_gap_sec"],
                candidate_measurement["maximum_camera_stamp_gap_sec"],
            ),
            "receipt_efficiency_ratio": _metric_comparison(
                baseline_measurement["receipt_efficiency_ratio"],
                candidate_measurement["receipt_efficiency_ratio"],
            ),
            "inference_mean_ms": _metric_comparison(
                baseline_trial["vad_runtime"]["inference_ms"]["mean"],
                candidate_trial["vad_runtime"]["inference_ms"]["mean"],
            ),
            "inference_p95_ms": _metric_comparison(
                baseline_trial["vad_runtime"]["inference_ms"]["p95"],
                candidate_trial["vad_runtime"]["inference_ms"]["p95"],
            ),
        }

    # The normalized profiles have already been compared; omit their large copies
    # from the compact per-arm output while retaining their immutable plan digests.
    baseline.pop("runtime_profile")
    candidate.pop("runtime_profile")
    gates = {
        "terminal_c_track_straight_turn_pass": True,
        "runtime_profile_invariants_equal": True,
        "source_route_sha_and_payload_equal": True,
        "mapping_delta_exactly_six_camera_sensor_ticks": True,
        "whole_bag_six_camera_coverage_at_least_99_percent": True,
        "all_six_camera_stamp_gaps_at_most_0_25_sec": True,
        "whole_bag_candidate_front_boundary_gate": True,
        "vad_no_runtime_supersession_pruning_or_coalescing": True,
        "route_manager_blas_threads_pinned_to_one": True,
        "vad_published_equals_mailbox_taken": True,
        "functional_route_and_speed_pass": True,
    }
    return {
        "schema_version": SCHEMA_VERSION,
        "analysis_id": ANALYSIS_ID,
        "status": "PASS",
        "comparison_valid": True,
        "decision": "VALID_DESCRIPTIVE_COMPARISON",
        "decision_scope": (
            "Functional/integrity gates passed. Performance deltas are descriptive; "
            "this analyzer does not impose an optimization-adoption threshold."
        ),
        "map_id": MAP_ID,
        "trial_ids": list(TRIAL_IDS),
        "measurement_windows": {
            "rtf": (
                "route evaluation window from result.metrics.sim_elapsed_sec and "
                "wall_elapsed_sec"
            ),
            "camera_rates": (
                "whole rosbag from latency/e2e_latency.json; no rosbag reread"
            ),
            "route_window_camera_metrics": (
                "UNAVAILABLE without rosbag reread; whole-bag coverage/acceptance are "
                "boundary-compatible proxies, not route-window exactness"
            ),
            "vad_runtime": "full stack process lifetime from stack.log",
            "receipt_efficiency_ratio": (
                "front whole-bag effective receipt Hz / (front whole-bag stamp Hz * "
                "route-window RTF); cross-window descriptive sanity ratio"
            ),
        },
        "single_variable_contract": {
            "baseline_selector": BASELINE_SELECTOR,
            "candidate_selector": CANDIDATE_SELECTOR,
            "runtime_profile": profile_invariants,
            "sensor_mapping": mapping_delta,
        },
        "route_identity": route_identity,
        "arms": {"baseline": baseline, "candidate": candidate},
        "comparisons": comparisons,
        "acceptance_gates": gates,
    }


CSV_COLUMNS = (
    "trial_id",
    "arm",
    "selector",
    "route_sha256",
    "sensor_mapping_sha256",
    "sensor_tick_sec",
    "rtf_window",
    "camera_rate_window",
    "sim_elapsed_sec",
    "wall_elapsed_sec",
    "rtf",
    "front_count",
    "front_effective_receipt_rate_hz",
    "front_stamp_rate_hz",
    "maximum_camera_stamp_gap_sec",
    "receipt_efficiency_ratio",
    "bundle_matched_count",
    "bundle_front_count",
    "bundle_coverage_percent",
    "candidate_count",
    "candidate_front_count",
    "candidate_acceptance_percent",
    "inference_count",
    "inference_mean_ms",
    "inference_median_ms",
    "inference_p95_ms",
    "inference_p99_ms",
    "inference_max_ms",
    "published_count",
    "mailbox_taken",
    "capacity_pruned_max",
    "superseded_max",
    "coalesced_drops_max",
    "route_success",
    "speed_status",
    "candidate_rtf_change_percent",
    "candidate_wall_receipt_change_percent",
)


def _csv_text(report: Mapping[str, Any]) -> str:
    stream = io.StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=CSV_COLUMNS, lineterminator="\n")
    writer.writeheader()
    for trial_id in TRIAL_IDS:
        for arm_name, sensor_tick in (("baseline", 0.0), ("candidate", 0.2)):
            arm = report["arms"][arm_name]
            trial = arm["trials"][trial_id]
            functional = trial["functional"]
            camera = trial["camera_vad"]
            bundle = camera["bundle"]
            acceptance = camera["candidate_front_acceptance"]
            vad = trial["vad_runtime"]
            inference = vad["inference_ms"]
            comparison = report["comparisons"][trial_id]
            row = {
                "trial_id": trial_id,
                "arm": arm_name,
                "selector": arm["runtime_profile_selector"],
                "route_sha256": trial["source_route"]["sha256"],
                "sensor_mapping_sha256": arm["sensor_mapping_sha256"],
                "sensor_tick_sec": sensor_tick,
                "rtf_window": "route_evaluation",
                "camera_rate_window": "whole_rosbag",
                "sim_elapsed_sec": functional["sim_elapsed_sec"],
                "wall_elapsed_sec": functional["wall_elapsed_sec"],
                "rtf": trial["measurement"]["rtf"],
                "front_count": camera["front_count"],
                "front_effective_receipt_rate_hz": camera[
                    "front_effective_receipt_rate_hz"
                ],
                "front_stamp_rate_hz": camera["front_stamp_rate_hz"],
                "maximum_camera_stamp_gap_sec": camera[
                    "maximum_camera_stamp_gap_sec"
                ],
                "receipt_efficiency_ratio": trial["measurement"][
                    "receipt_efficiency_ratio"
                ],
                "bundle_matched_count": bundle["matched_bundle_count"],
                "bundle_front_count": bundle["front_frame_count"],
                "bundle_coverage_percent": bundle["coverage_percent"],
                "candidate_count": acceptance["candidate_count"],
                "candidate_front_count": acceptance["front_count"],
                "candidate_acceptance_percent": acceptance["acceptance_percent"],
                "inference_count": inference["count"],
                "inference_mean_ms": inference["mean"],
                "inference_median_ms": inference["median"],
                "inference_p95_ms": inference["p95"],
                "inference_p99_ms": inference["p99"],
                "inference_max_ms": inference["max"],
                "published_count": vad["published_count"],
                "mailbox_taken": vad["mailbox_taken"],
                "capacity_pruned_max": vad["capacity_pruned_max"],
                "superseded_max": vad["superseded_max"],
                "coalesced_drops_max": vad["coalesced_drops_max"],
                "route_success": str(functional["success"]).lower(),
                "speed_status": functional["speed_status"],
                "candidate_rtf_change_percent": comparison["rtf"][
                    "candidate_change_percent"
                ],
                "candidate_wall_receipt_change_percent": comparison[
                    "front_effective_receipt_rate_hz"
                ]["candidate_change_percent"],
            }
            writer.writerow(row)
    return stream.getvalue()


def _fmt(value: float, digits: int = 6) -> str:
    return f"{value:.{digits}f}"


def _markdown_text(report: Mapping[str, Any]) -> str:
    lines = [
        "# CTrack camera-source cadence A/B",
        "",
        f"- Status: **{report['status']}**",
        f"- Decision scope: {report['decision_scope']}",
        "- Baseline: `speed_30kph` (`sensor_tick=0.0 s`)",
        "- Candidate: `speed_30kph_camera_source_5hz` (`sensor_tick=0.2 s`)",
        "",
        "## Window contract",
        "",
        "RTF uses the route-evaluation interval. Camera rates use the whole rosbag as "
        "reported by `latency/e2e_latency.json`; this analyzer does not reread the bag. "
        "Receipt efficiency therefore remains an explicitly cross-window descriptive "
        "sanity ratio.",
        "",
        "## Comparison",
        "",
        "| Trial | Arm | RTF | Front wall Hz | Front stamp Hz | Max camera gap s | "
        "Receipt efficiency | Bundle coverage | Candidate acceptance | Inference p95 ms |",
        "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for trial_id in TRIAL_IDS:
        for arm_name in ("baseline", "candidate"):
            trial = report["arms"][arm_name]["trials"][trial_id]
            measurement = trial["measurement"]
            camera = trial["camera_vad"]
            lines.append(
                "| {trial} | {arm} | {rtf} | {wall} | {stamp} | {max_gap} | {efficiency} | "
                "{coverage}% | {acceptance}% | {p95} |".format(
                    trial=trial_id,
                    arm=arm_name,
                    rtf=_fmt(measurement["rtf"]),
                    wall=_fmt(measurement["front_effective_receipt_rate_hz"]),
                    stamp=_fmt(measurement["front_stamp_rate_hz"]),
                    max_gap=_fmt(camera["maximum_camera_stamp_gap_sec"]),
                    efficiency=_fmt(measurement["receipt_efficiency_ratio"]),
                    coverage=_fmt(camera["bundle"]["coverage_percent"], 3),
                    acceptance=_fmt(
                        camera["candidate_front_acceptance"]["acceptance_percent"], 3
                    ),
                    p95=_fmt(trial["vad_runtime"]["inference_ms"]["p95"], 3),
                )
            )
    lines.extend(
        [
            "",
            "## Candidate change from baseline",
            "",
            "| Trial | RTF | Front wall receipt | Receipt efficiency | Inference p95 |",
            "|---|---:|---:|---:|---:|",
        ]
    )
    for trial_id in TRIAL_IDS:
        comparison = report["comparisons"][trial_id]
        lines.append(
            "| {trial} | {rtf:+.3f}% | {wall:+.3f}% | {eff:+.3f}% | {p95:+.3f}% |".format(
                trial=trial_id,
                rtf=comparison["rtf"]["candidate_change_percent"],
                wall=comparison["front_effective_receipt_rate_hz"][
                    "candidate_change_percent"
                ],
                eff=comparison["receipt_efficiency_ratio"][
                    "candidate_change_percent"
                ],
                p95=comparison["inference_p95_ms"]["candidate_change_percent"],
            )
        )
    lines.extend(["", "## Integrity gates", ""])
    for name, passed in report["acceptance_gates"].items():
        lines.append(f"- [{'x' if passed else ' '}] `{name}`")
    lines.extend(["", "## Route identity", ""])
    for trial_id in TRIAL_IDS:
        route = report["route_identity"][trial_id]
        lines.append(
            f"- `{trial_id}`: `{route['sha256']}` (byte SHA and canonical payload equal)"
        )
    lines.extend(
        [
            "",
            "A single straight/turn pair is descriptive. Use paired repeated runs and a "
            "predeclared performance threshold before making an optimization-adoption claim.",
            "",
        ]
    )
    return "\n".join(lines)


def _render_png(report: Mapping[str, Any], output: Path) -> None:
    panels = (
        ("rtf", "Route real-time factor", "RTF"),
        (
            "front_effective_receipt_rate_hz",
            "Front camera wall receipt (whole bag)",
            "Hz",
        ),
        (
            "maximum_camera_stamp_gap_sec",
            "Worst six-camera stamp gap (whole bag)",
            "s",
        ),
        ("inference_p95_ms", "VAD inference p95 (process lifetime)", "ms"),
    )
    colors = {"baseline": "#59636e", "candidate": "#167c80"}
    figure, axes = plt.subplots(2, 2, figsize=(12.8, 8.0), constrained_layout=True)
    x = np.arange(len(TRIAL_IDS), dtype=float)
    width = 0.34
    for axis, (key, title, unit) in zip(axes.flat, panels):
        for offset, arm_name in ((-width / 2.0, "baseline"), (width / 2.0, "candidate")):
            values = []
            for trial_id in TRIAL_IDS:
                trial = report["arms"][arm_name]["trials"][trial_id]
                if key == "inference_p95_ms":
                    value = trial["vad_runtime"]["inference_ms"]["p95"]
                else:
                    value = trial["measurement"][key]
                values.append(value)
            bars = axis.bar(
                x + offset,
                values,
                width,
                label=arm_name.capitalize(),
                color=colors[arm_name],
                alpha=0.9,
                zorder=2,
            )
            axis.bar_label(bars, fmt="%.3f", fontsize=8, padding=2)
        for index, trial_id in enumerate(TRIAL_IDS):
            change = report["comparisons"][trial_id][key]["candidate_change_percent"]
            axis.text(
                index,
                0.97,
                f"B/A {change:+.1f}%",
                ha="center",
                va="top",
                transform=axis.get_xaxis_transform(),
                fontsize=9,
                weight="bold",
            )
        axis.set_title(title, fontsize=11.5, weight="bold")
        axis.set_ylabel(unit)
        axis.set_xticks(x, ["Straight", "Turn"])
        axis.set_ylim(bottom=0.0)
        axis.grid(axis="y", color="#d9dde2", linewidth=0.8, zorder=1)
        axis.spines[["top", "right"]].set_visible(False)
    axes.flat[0].legend(loc="lower right", frameon=False)
    figure.suptitle(
        "CTrack camera source cadence A/B | RTF: route window; camera: whole bag",
        fontsize=14,
        weight="bold",
    )
    figure.savefig(
        output,
        format="png",
        dpi=160,
        facecolor="white",
        metadata={"Software": "analyze_camera_cadence_ab.py"},
    )
    plt.close(figure)


def write_outputs(report: Mapping[str, Any], output_dir: Path) -> dict[str, Path]:
    """Write deterministic JSON, CSV, Markdown, and PNG artifacts."""
    output_dir = output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    outputs = {
        "json": output_dir / f"{OUTPUT_BASENAME}.json",
        "csv": output_dir / f"{OUTPUT_BASENAME}.csv",
        "markdown": output_dir / f"{OUTPUT_BASENAME}.md",
        "png": output_dir / f"{OUTPUT_BASENAME}.png",
    }
    for label, path in outputs.items():
        if path.is_symlink():
            raise AnalysisError(f"refusing to replace symlinked {label} output: {path}")

    json_text = json.dumps(
        report, indent=2, sort_keys=True, ensure_ascii=True, allow_nan=False
    ) + "\n"
    csv_text = _csv_text(report)
    markdown_text = _markdown_text(report)
    with tempfile.TemporaryDirectory(prefix=".camera_cadence_ab.", dir=output_dir) as stage:
        stage_root = Path(stage)
        staged = {label: stage_root / path.name for label, path in outputs.items()}
        staged["json"].write_text(json_text, encoding="utf-8")
        staged["csv"].write_text(csv_text, encoding="utf-8", newline="")
        staged["markdown"].write_text(markdown_text, encoding="utf-8")
        _render_png(report, staged["png"])
        for label in ("json", "csv", "markdown", "png"):
            os.replace(staged[label], outputs[label])
    return outputs


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline-root", required=True, type=Path)
    parser.add_argument("--candidate-root", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    return parser.parse_args(argv)


def run(args: argparse.Namespace) -> int:
    report = analyze(args.baseline_root, args.candidate_root)
    outputs = write_outputs(report, args.output_dir)
    print(f"status:   {report['status']}")
    for label in ("json", "csv", "markdown", "png"):
        print(f"{label}: {outputs[label]}")
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    try:
        return run(args)
    except AnalysisError as error:
        print(f"camera cadence A/B validation failed: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
