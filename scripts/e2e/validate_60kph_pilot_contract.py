#!/usr/bin/env python3
"""HH_260906 - Gate the 60 km/h CARLA pilot fail-closed.

The evidence-integrity and camera-transport verdicts are deliberately separate
from the simulation acceptance verdict.  A route may qualify the strict camera
transport while still failing the required speed exposure.  Neither outcome is
a real-vehicle calibration claim.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import tempfile
from typing import Any, Callable, Sequence

import yaml


PROFILE_ID = "carla_vad_60kph_straight_pilot_v1"
CAMERA_PROFILE_5HZ_ID = "carla_vad_camera_source_5hz_best_effort_image_v2"
# HH_260906 - Preserve v1 evidence semantics while v2 adds recorder-boundary proof.
CAMERA_PROFILE_10HZ_STRICT_LEGACY_ID = "carla_vad_camera_source_10hz_strict_v1"
CAMERA_PROFILE_10HZ_STRICT_ID = "carla_vad_camera_source_10hz_strict_v2"
CAMERA_PROFILE_10HZ_STRICT_IDS = frozenset(
    {CAMERA_PROFILE_10HZ_STRICT_LEGACY_ID, CAMERA_PROFILE_10HZ_STRICT_ID}
)
CAMERA_PROFILE_ID = CAMERA_PROFILE_5HZ_ID
CAMERA_EVIDENCE_BY_PROFILE = {
    CAMERA_PROFILE_5HZ_ID: "camera_source_5hz_validation.json",
    CAMERA_PROFILE_10HZ_STRICT_LEGACY_ID: (
        "camera_source_10hz_strict_validation.json"
    ),
    CAMERA_PROFILE_10HZ_STRICT_ID: "camera_source_10hz_strict_validation.json",
}
TARGET_SPEED_MPS = 16.666666666666668
MINIMUM_SUSTAINED_SPEED_MPS = 15.0
MINIMUM_SUSTAINED_SPEED_SEC = 1.0
MAXIMUM_OBSERVED_SPEED_MPS = 18.0
MAXIMUM_LATERAL_ACCELERATION_MPS2 = 1.2
MAXIMUM_GATED_LONGITUDINAL_ACCELERATION_MPS2 = 1.5
MAXIMUM_SPEED_SAMPLE_GAP_SEC = 0.25
MINIMUM_RUNTIME_RTF = 0.9
MINIMUM_CAMERA_RATE_HZ = 4.0
MINIMUM_CAMERA_BUNDLE_COVERAGE_PERCENT = 99.0
MAXIMUM_CAMERA_BUNDLE_RECEIPT_P95_SEC = 0.04
MAXIMUM_STRICT_CAMERA_BUNDLE_STAMP_SPAN_SEC = 0.000005
# HH_260906 - Keep strict pre-engagement source and receipt cadence bounded.
MAXIMUM_STRICT_CAMERA_WALL_RATE_HZ = 11.0
MINIMUM_STRICT_CAMERA_SOURCE_RATE_HZ = 9.5
MAXIMUM_STRICT_CAMERA_SOURCE_RATE_HZ = 10.5
# HH_260906 - Reject accelerated source intervals as well as missing-frame gaps.
MINIMUM_STRICT_CAMERA_SOURCE_GAP_SEC = 0.099995
MAXIMUM_STRICT_CAMERA_SOURCE_GAP_SEC = 0.100005
PORTABLE_SHADOW_NODE = "/portable_e2e_shadow"
PORTABLE_OUTPUT_TOPICS = (
    "/planning/portable_e2e/shadow_trajectory",
    "/planning/portable_e2e/shadow_path",
    "/planning/portable_e2e/status",
    "/planning/portable_e2e/latency_ms",
    "/planning/portable_e2e/selected_candidate",
)
STRICT_CAMERA_INFO_TOPICS = tuple(
    f"/sensing/camera/{camera}/camera_info"
    for camera in (
        "CAM_FRONT",
        "CAM_BACK",
        "CAM_FRONT_LEFT",
        "CAM_BACK_LEFT",
        "CAM_FRONT_RIGHT",
        "CAM_BACK_RIGHT",
    )
)

PROFILE_CONTEXT = {
    "longitudinal_velocity_source": "explicit_simulation_nominal",
    "vad_velocity_evaluated": False,
    "vad_geometry_evaluated": True,
}

REQUIRED_JSON = (
    "result.json",
    "route_alignment.json",
    "aligned_route.json",
    "source_route.json",
    "runtime_health.json",
    "runtime_load_analysis.json",
    "speed_profile.json",
    "longitudinal_response.json",
    "diagnosis.json",
    "actuation_map_coverage.json",
    "actuation_map_runtime_coverage.json",
    "carla_preflight_health.json",
    "carla_completion_health.json",
    "carla_cleanup_health.json",
)


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


def _finite(value: object) -> bool:
    return (
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and math.isfinite(float(value))
    )


def _close(left: object, right: float, tolerance: float = 1.0e-9) -> bool:
    return _finite(left) and math.isclose(
        float(left), right, rel_tol=0.0, abs_tol=tolerance
    )


def _parsed_number(value: object) -> float | None:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def _parsed_positive_integer(value: object, maximum: int | None = None) -> int | None:
    if not isinstance(value, str):
        return None
    try:
        parsed = int(value)
    except ValueError:
        return None
    if str(parsed) != value or parsed <= 0 or (maximum is not None and parsed > maximum):
        return None
    return parsed


def _resolved_recorded_path(value: object, base: Path) -> Path | None:
    if not isinstance(value, str) or not value:
        return None
    candidate = Path(value).expanduser()
    if not candidate.is_absolute():
        candidate = base / candidate
    return candidate.resolve()


def _parse_env(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    for number, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not raw or raw.startswith("#"):
            continue
        if "=" not in raw:
            raise ValueError(f"runtime.env line {number} has no '='")
        key, value = raw.split("=", 1)
        if not key or key in values:
            raise ValueError(f"runtime.env line {number} has an invalid/duplicate key")
        values[key] = value
    return values


def _nested(value: Any, *keys: str) -> Any:
    current = value
    for key in keys:
        if not isinstance(current, dict):
            return None
        current = current.get(key)
    return current


def _strict_v2_edge_bounded_camera_integrity_valid(
    value: object,
    matched_bundle_count: object,
) -> bool:
    """HH_260906 - Validate bounded recorder-edge trimming and exact interior bundles."""

    def nonnegative_integer(item: object) -> bool:
        return (
            isinstance(item, int)
            and not isinstance(item, bool)
            and item >= 0
        )

    def positive_integer(item: object) -> bool:
        return nonnegative_integer(item) and int(item) > 0

    if not isinstance(value, dict):
        return False
    topics = value.get("topics")
    boundary = value.get("boundary_trim")
    retained = value.get("retained_interior")
    if (
        not positive_integer(value.get("schema_version"))
        or value.get("schema_version") != 1
        or value.get("qualification_id")
        != "camera_source_stamp_edge_bounded_whole_bag_v1"
        or value.get("status") != "PASS"
        or not positive_integer(value.get("camera_count"))
        or value.get("camera_count") != len(STRICT_CAMERA_INFO_TOPICS)
        or value.get("expected_topics") != list(STRICT_CAMERA_INFO_TOPICS)
        or value.get("failures") != []
        or not isinstance(topics, dict)
        or set(topics) != set(STRICT_CAMERA_INFO_TOPICS)
        or not isinstance(boundary, dict)
        or not isinstance(retained, dict)
    ):
        return False

    topic_totals = {
        "record_count": 0,
        "positive_stamp_count": 0,
        "zero_or_negative_stamp_count": 0,
        "duplicate_positive_stamp_count": 0,
        "non_increasing_positive_stamp_delta_count": 0,
        "trimmed_boundary_record_count": 0,
        "retained_interior_record_count": 0,
    }
    for topic in STRICT_CAMERA_INFO_TOPICS:
        report = topics.get(topic)
        if not isinstance(report, dict):
            return False
        counts = {
            key: report.get(key)
            for key in topic_totals
        }
        if (
            not all(nonnegative_integer(item) for item in counts.values())
            or not positive_integer(counts["record_count"])
            or counts["positive_stamp_count"] != counts["record_count"]
            or not positive_integer(report.get("unique_positive_stamp_count"))
            or report.get("unique_positive_stamp_count")
            != counts["positive_stamp_count"]
            or counts["zero_or_negative_stamp_count"] != 0
            or counts["duplicate_positive_stamp_count"] != 0
            or counts["non_increasing_positive_stamp_delta_count"] != 0
            or report.get("strictly_increasing_unique_positive_stamps") is not True
            or counts["trimmed_boundary_record_count"]
            + counts["retained_interior_record_count"]
            != counts["positive_stamp_count"]
        ):
            return False
        for key, count in counts.items():
            topic_totals[key] += int(count)

    leading_count = boundary.get("leading_incomplete_union_stamp_count")
    trailing_count = boundary.get("trailing_incomplete_union_stamp_count")
    trimmed_count = boundary.get("trimmed_union_stamp_count")
    trimmed_positive_count = boundary.get("trimmed_positive_record_count")
    trimmed_stamps = boundary.get("trimmed_source_stamps_ns")
    if (
        boundary.get("policy")
        != "at_most_one_incomplete_union_stamp_per_bag_edge_v1"
        or not positive_integer(
            boundary.get("maximum_incomplete_union_stamp_count_per_edge")
        )
        or boundary.get("maximum_incomplete_union_stamp_count_per_edge") != 1
        or not nonnegative_integer(leading_count)
        or int(leading_count) > 1
        or not nonnegative_integer(trailing_count)
        or int(trailing_count) > 1
        or not nonnegative_integer(trimmed_count)
        or int(trimmed_count) != int(leading_count) + int(trailing_count)
        or int(trimmed_count) > 2
        or not nonnegative_integer(trimmed_positive_count)
        or not isinstance(trimmed_stamps, list)
        or len(trimmed_stamps) != int(trimmed_count)
        or not all(positive_integer(stamp) for stamp in trimmed_stamps)
        or trimmed_stamps != sorted(set(trimmed_stamps))
    ):
        return False

    descriptor_record_count = 0
    descriptor_stamps: list[int] = []
    for edge, expected_count in (
        ("leading", leading_count),
        ("trailing", trailing_count),
    ):
        descriptor = boundary.get(edge)
        if expected_count == 0:
            if descriptor is not None:
                return False
            continue
        if not isinstance(descriptor, dict):
            return False
        per_topic = descriptor.get("per_topic_record_counts")
        if (
            not positive_integer(descriptor.get("source_stamp_ns"))
            or not isinstance(per_topic, dict)
            or set(per_topic) != set(STRICT_CAMERA_INFO_TOPICS)
            or not all(
                nonnegative_integer(count) and int(count) <= 1
                for count in per_topic.values()
            )
        ):
            return False
        present = [
            topic for topic in STRICT_CAMERA_INFO_TOPICS if per_topic[topic] == 1
        ]
        missing = [
            topic for topic in STRICT_CAMERA_INFO_TOPICS if per_topic[topic] == 0
        ]
        record_count = sum(per_topic.values())
        if (
            not present
            or not missing
            or not positive_integer(descriptor.get("record_count"))
            or descriptor.get("record_count") != record_count
            or descriptor.get("present_topics") != present
            or descriptor.get("missing_topics") != missing
            or descriptor.get("duplicate_topics") != []
            or descriptor.get("incomplete") is not True
            or descriptor.get("exact_six_camera_one_to_one") is not False
        ):
            return False
        descriptor_record_count += record_count
        descriptor_stamps.append(int(descriptor["source_stamp_ns"]))

    source_stamp_count = retained.get("source_stamp_count")
    source_range = retained.get("source_stamp_range_ns")
    retained_positive_count = retained.get("positive_record_count")
    expected_record_count = retained.get("expected_record_count")
    if (
        not positive_integer(source_stamp_count)
        or not isinstance(source_range, dict)
        or not positive_integer(source_range.get("minimum"))
        or not positive_integer(source_range.get("maximum"))
        or int(source_range["minimum"]) > int(source_range["maximum"])
        or not positive_integer(retained_positive_count)
        or not positive_integer(expected_record_count)
        or retained_positive_count != int(source_stamp_count)
        * len(STRICT_CAMERA_INFO_TOPICS)
        or expected_record_count != retained_positive_count
        or not positive_integer(retained.get("complete_bundle_count"))
        or retained.get("complete_bundle_count") != source_stamp_count
        or not nonnegative_integer(retained.get("incomplete_union_stamp_count"))
        or retained.get("incomplete_union_stamp_count") != 0
        or not nonnegative_integer(retained.get("non_exact_union_stamp_count"))
        or retained.get("non_exact_union_stamp_count") != 0
        or retained.get("all_source_stamps_exact_six_camera_one_to_one") is not True
        or not positive_integer(matched_bundle_count)
        or matched_bundle_count != source_stamp_count
    ):
        return False
    if leading_count and descriptor_stamps[0] >= int(source_range["minimum"]):
        return False
    if trailing_count and descriptor_stamps[-1] <= int(source_range["maximum"]):
        return False
    if sorted(descriptor_stamps) != trimmed_stamps:
        return False

    union_count = value.get("union_positive_source_stamp_count")
    if not all(
        nonnegative_integer(value.get(key))
        for key in (
            "raw_record_count",
            "positive_record_count",
            "nonpositive_record_count",
            "duplicate_positive_record_count",
            "non_increasing_positive_stamp_delta_count",
        )
    ) or not positive_integer(union_count):
        return False
    return bool(
        value.get("nonpositive_record_count") == 0
        and value.get("duplicate_positive_record_count") == 0
        and value.get("non_increasing_positive_stamp_delta_count") == 0
        and value.get("raw_record_count") == topic_totals["record_count"]
        and value.get("positive_record_count")
        == topic_totals["positive_stamp_count"]
        and value.get("nonpositive_record_count")
        == topic_totals["zero_or_negative_stamp_count"]
        and value.get("duplicate_positive_record_count")
        == topic_totals["duplicate_positive_stamp_count"]
        and value.get("non_increasing_positive_stamp_delta_count")
        == topic_totals["non_increasing_positive_stamp_delta_count"]
        and trimmed_positive_count == descriptor_record_count
        and trimmed_positive_count
        == topic_totals["trimmed_boundary_record_count"]
        and retained_positive_count
        == topic_totals["retained_interior_record_count"]
        and union_count == int(source_stamp_count) + int(trimmed_count)
        and value.get("positive_record_count")
        == int(trimmed_positive_count) + int(retained_positive_count)
    )


def _series_jerk_observability(longitudinal: dict[str, Any]) -> dict[str, Any]:
    output: dict[str, Any] = {}
    series = longitudinal.get("series")
    for name in ("raw_control", "gated_control"):
        samples = series.get(name) if isinstance(series, dict) else None
        values = []
        if isinstance(samples, list):
            for sample in samples:
                value = sample.get("jerk") if isinstance(sample, dict) else None
                if _finite(value):
                    values.append(float(value))
        output[name] = {
            "finite_sample_count": len(values),
            "nonzero_sample_count": sum(abs(value) > 1.0e-9 for value in values),
            "maximum_absolute_message_jerk_mps3": (
                max((abs(value) for value in values), default=None)
            ),
        }
    observable = all(
        output[name]["finite_sample_count"] >= 2
        and output[name]["nonzero_sample_count"] > 0
        for name in output
    )
    return {
        "message_field_observable": observable,
        "series": output,
        "acceptance_gate_applied": False,
        "reason": (
            "no phase-aware jerk acceptance contract is defined; zero-filled "
            "Control.longitudinal.jerk fields cannot prove bounded plant jerk"
        ),
    }


def _validate_manifest(
    manifest: object,
    expected_root: Path,
    failures: list[str],
    label: str,
) -> None:
    if not isinstance(manifest, dict):
        failures.append(f"{label} rosbag manifest is missing")
        return
    root = _resolved_recorded_path(manifest.get("root"), expected_root.parent)
    files = manifest.get("files")
    if root != expected_root or manifest.get("schema_version") != 1:
        failures.append(f"{label} rosbag root/schema does not match the trial bag")
        return
    if not isinstance(files, list) or not files:
        failures.append(f"{label} rosbag manifest has no files")
        return
    canonical_files: list[dict[str, Any]] = []
    seen: set[str] = set()
    for item in files:
        if not isinstance(item, dict):
            failures.append(f"{label} rosbag manifest contains a non-object entry")
            return
        relative = item.get("path")
        if (
            not isinstance(relative, str)
            or not relative
            or relative in seen
            or Path(relative).is_absolute()
            or ".." in Path(relative).parts
        ):
            failures.append(f"{label} rosbag manifest contains an unsafe path")
            return
        seen.add(relative)
        path = expected_root / relative
        try:
            if path.is_symlink() or not path.is_file():
                raise OSError("not a regular file")
            size = path.stat().st_size
            digest = _sha256_file(path)
        except OSError as error:
            failures.append(f"{label} rosbag file {relative} is invalid: {error}")
            return
        if item.get("size_bytes") != size or item.get("sha256") != digest:
            failures.append(f"{label} rosbag file {relative} digest/size mismatch")
            return
        canonical_files.append(
            {"path": relative, "size_bytes": size, "sha256": digest}
        )
    expected_digest = _sha256_json(
        {"schema_version": 1, "files": canonical_files}
    )
    if manifest.get("sha256") != expected_digest:
        failures.append(f"{label} rosbag manifest digest mismatch")


def _validate_runtime_load_input_manifest(
    manifest: object,
    attempt: Path,
    failures: list[str],
) -> None:
    # HH_260906 - Rehash every v2 runtime-load input before trusting its analysis.
    telemetry = attempt.parents[1] / "host_telemetry"
    expected_paths = {
        "result": attempt / "result.json",
        "latency": attempt / "latency/e2e_latency.json",
        "stack": attempt / "stack.log",
        "recorder": attempt / "recorder.log",
        "vmstat": telemetry / "vmstat.log",
        "nvidia_dmon": telemetry / "nvidia_smi_dmon.log",
        "pidstat": telemetry / "pidstat.log",
        "bag": attempt / "bag",
    }
    if not isinstance(manifest, dict) or set(manifest) != set(expected_paths):
        failures.append("strict runtime-load input manifest label set mismatch")
        return
    for label, expected_path in expected_paths.items():
        item = manifest.get(label)
        resolved = expected_path.resolve()
        if not isinstance(item, dict) or item.get("path") != str(resolved):
            failures.append(f"strict runtime-load {label} manifest path mismatch")
            continue
        if label == "bag":
            recorded_files = item.get("files")
            if not resolved.is_dir() or not isinstance(recorded_files, list):
                failures.append("strict runtime-load bag manifest is invalid")
                continue
            actual_files = []
            invalid_entry = False
            for path in sorted(resolved.iterdir()):
                if path.is_symlink():
                    invalid_entry = True
                    continue
                if path.is_file():
                    actual_files.append(
                        {
                            "name": path.name,
                            "size_bytes": path.stat().st_size,
                            "sha256": _sha256_file(path),
                        }
                    )
            if invalid_entry or recorded_files != actual_files or not actual_files:
                failures.append("strict runtime-load bag manifest content mismatch")
            continue
        if resolved.is_symlink() or not resolved.is_file():
            failures.append(f"strict runtime-load {label} input is not a regular file")
            continue
        expected_item = {
            "path": str(resolved),
            "size_bytes": resolved.stat().st_size,
            "sha256": _sha256_file(resolved),
        }
        if item != expected_item:
            failures.append(f"strict runtime-load {label} digest/size mismatch")


def _file_identity(
    path: Path,
    failures: list[str],
    label: str,
) -> dict[str, Any]:
    identity = {"path": str(path.resolve()), "sha256": None}
    try:
        if path.is_symlink() or not path.is_file():
            raise OSError("not a regular file")
        identity["sha256"] = _sha256_file(path)
    except OSError as error:
        failures.append(f"{label} is invalid: {error}")
    return identity


def _aware_timestamp(value: object) -> datetime | None:
    if not isinstance(value, str) or not value:
        return None
    try:
        parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError:
        return None
    return parsed if parsed.tzinfo is not None else None


def _validate_strict_recorder_measurement(
    runtime_env: dict[str, str],
    attempt: Path,
    result: dict[str, Any],
    failures: list[str],
) -> dict[str, Any]:
    # HH_260906 - Bind the measured bag window to the acknowledged PTY resume edge.
    path = attempt / "recorder_measurement_start.json"
    identity = _file_identity(path, failures, "strict recorder measurement evidence")
    evidence: dict[str, Any] = {}
    if identity["sha256"] is not None:
        try:
            loaded = json.loads(path.read_text(encoding="utf-8"))
            if not isinstance(loaded, dict):
                raise ValueError("top level is not an object")
            evidence = loaded
        except (OSError, UnicodeDecodeError, json.JSONDecodeError, ValueError) as error:
            failures.append(f"strict recorder measurement evidence is malformed: {error}")

    expected_environment = {
        "STRICT_RECORDER_START_PAUSED": "true",
        "STRICT_RECORDER_CAMERA_INFO_SUBSCRIPTIONS_ACKNOWLEDGED": "6",
        "STRICT_RECORDER_RESUME_CONTROL": "humble_rosbag2_owned_pty_space_key_v1",
        "STRICT_RECORDER_MEASUREMENT_RESUME_STATUS": "pass",
        "STRICT_RECORDER_MEASUREMENT_EVIDENCE_FILE": (
            "recorder_measurement_start.json"
        ),
        "RECORDER_ROUTE_EVALUATION_LIVENESS_STATUS": "pass",
        "RECORDER_ROUTE_COMPLETION_LIVENESS_STATUS": "pass",
        "RECORDER_OWNED_GROUP_CLEANUP_STATUS": "pass",
        "RECORDER_CONTROL_FIFO_REMOVED": "true",
        "STACK_OWNED_GROUP_CLEANUP_STATUS": "pass",
        "STACK_POST_SHUTDOWN_CRITICAL_PROCESS_CHECK": "pass",
    }
    for key, expected in expected_environment.items():
        if runtime_env.get(key) != expected:
            failures.append(f"strict recorder runtime.env {key} mismatch")

    if (
        runtime_env.get("STRICT_RECORDER_MEASUREMENT_EVIDENCE_SHA256")
        != identity["sha256"]
    ):
        failures.append("strict recorder measurement evidence SHA-256 mismatch")
    if (attempt / "recorder_control.fifo").exists() or (
        attempt / "recorder_control.fifo"
    ).is_symlink():
        failures.append("strict recorder control FIFO remains after cleanup")

    expected_evidence = {
        "schema_version": 1,
        "status": "PASS",
        "policy": "paused_until_all_six_camera_info_subscriptions_acknowledged_v1",
        "recorder_started_paused": True,
        "camera_info_subscription_count": 6,
        "pause_state_proof": "Waiting for recording: Press SPACE to start.",
        "resume_control": "humble_rosbag2_owned_pty_space_key_v1",
        "resume_acknowledgement": "Resuming recording.",
        "measurement_boundary": "owned_pty_resume_acknowledged_before_route_evaluation",
    }
    for key, expected in expected_evidence.items():
        if evidence.get(key) != expected:
            failures.append(f"strict recorder measurement evidence {key} mismatch")

    timestamp_values = {
        "resume_requested": runtime_env.get(
            "STRICT_RECORDER_MEASUREMENT_RESUME_REQUESTED_AT"
        ),
        "resumed": runtime_env.get("STRICT_RECORDER_MEASUREMENT_RESUMED_AT"),
        "route_started": runtime_env.get("ROUTE_EVALUATION_STARTED_AT"),
        "result_started": result.get("started_at"),
        "result_finished": result.get("finished_at"),
        "route_finished": runtime_env.get("ROUTE_EVALUATION_FINISHED_AT"),
    }
    timestamps = {
        key: _aware_timestamp(value) for key, value in timestamp_values.items()
    }
    if any(value is None for value in timestamps.values()):
        failures.append("strict recorder measurement timestamps are missing or invalid")
    else:
        if not (
            timestamps["resume_requested"]
            <= timestamps["resumed"]
            < timestamps["route_started"]
            <= timestamps["result_started"]
            <= timestamps["result_finished"]
            <= timestamps["route_finished"]
        ):
            failures.append("strict recorder measurement timestamps are out of order")
        resume_latency = (
            timestamps["resumed"] - timestamps["resume_requested"]
        ).total_seconds()
        if not 0.0 <= resume_latency <= 5.0:
            failures.append("strict recorder resume acknowledgement exceeded 5 seconds")
    if (
        evidence.get("resume_requested_at") != timestamp_values["resume_requested"]
        or evidence.get("resumed_at") != timestamp_values["resumed"]
    ):
        failures.append("strict recorder JSON/runtime.env timestamps differ")

    return {
        **identity,
        "evidence": evidence,
        "control_fifo_removed": not (attempt / "recorder_control.fifo").exists(),
    }


def _bound_camera_file(
    runtime_env: dict[str, str],
    path_key: str,
    sha_key: str,
    attempt: Path,
    failures: list[str],
    label: str,
) -> dict[str, Any]:
    declared = runtime_env.get(path_key)
    path = _resolved_recorded_path(declared, attempt)
    expected_sha = runtime_env.get(sha_key)
    identity = {
        "declared_path": declared,
        "path": str(path) if path is not None else None,
        "sha256": None,
    }
    if (
        path is None
        or not path.is_file()
        or not isinstance(expected_sha, str)
        or re.fullmatch(r"[0-9a-f]{64}", expected_sha) is None
    ):
        failures.append(f"strict camera {label} path/hash provenance is invalid")
        return identity
    actual_sha = _sha256_file(path)
    identity["sha256"] = actual_sha
    if actual_sha != expected_sha:
        failures.append(f"strict camera {label} SHA-256 mismatch")
    return identity


def _validate_strict_camera_delivery_provenance(
    runtime_env: dict[str, str],
    attempt: Path,
    failures: list[str],
) -> dict[str, Any]:
    records = {
        "bundle_dispatch_source": _bound_camera_file(
            runtime_env,
            "CARLA_CAMERA_BUNDLE_DISPATCH_SOURCE_FILE",
            "CARLA_CAMERA_BUNDLE_DISPATCH_SHA256",
            attempt,
            failures,
            "bundle dispatch source",
        ),
        "bundle_dispatch_runtime": _bound_camera_file(
            runtime_env,
            "CARLA_CAMERA_BUNDLE_DISPATCH_RUNTIME_FILE",
            "CARLA_CAMERA_BUNDLE_DISPATCH_SHA256",
            attempt,
            failures,
            "bundle dispatch runtime",
        ),
        "bridge_source": _bound_camera_file(
            runtime_env,
            "CARLA_CAMERA_BRIDGE_SOURCE_FILE",
            "CARLA_CAMERA_BRIDGE_SHA256",
            attempt,
            failures,
            "bridge source",
        ),
        "bridge_runtime": _bound_camera_file(
            runtime_env,
            "CARLA_CAMERA_BRIDGE_RUNTIME_FILE",
            "CARLA_CAMERA_BRIDGE_SHA256",
            attempt,
            failures,
            "bridge runtime",
        ),
        "entrypoint_source": _bound_camera_file(
            runtime_env,
            "CARLA_CAMERA_ENTRYPOINT_SOURCE_FILE",
            "CARLA_CAMERA_ENTRYPOINT_SHA256",
            attempt,
            failures,
            "entrypoint source",
        ),
        "entrypoint_runtime": _bound_camera_file(
            runtime_env,
            "CARLA_CAMERA_ENTRYPOINT_RUNTIME_FILE",
            "CARLA_CAMERA_ENTRYPOINT_SHA256",
            attempt,
            failures,
            "entrypoint runtime",
        ),
        "publish_worker_source": _bound_camera_file(
            runtime_env,
            "CARLA_CAMERA_PUBLISH_WORKER_SOURCE_FILE",
            "CARLA_CAMERA_PUBLISH_WORKER_SHA256",
            attempt,
            failures,
            "publish worker source",
        ),
        "publish_worker_runtime": _bound_camera_file(
            runtime_env,
            "CARLA_CAMERA_PUBLISH_WORKER_RUNTIME_FILE",
            "CARLA_CAMERA_PUBLISH_WORKER_SHA256",
            attempt,
            failures,
            "publish worker runtime",
        ),
        "interface_launch_source": _bound_camera_file(
            runtime_env,
            "CARLA_CAMERA_INTERFACE_LAUNCH_SOURCE_FILE",
            "CARLA_CAMERA_INTERFACE_LAUNCH_SHA256",
            attempt,
            failures,
            "interface launch source",
        ),
        "interface_launch_runtime": _bound_camera_file(
            runtime_env,
            "CARLA_CAMERA_INTERFACE_LAUNCH_RUNTIME_FILE",
            "CARLA_CAMERA_INTERFACE_LAUNCH_SHA256",
            attempt,
            failures,
            "interface launch runtime",
        ),
        "delivery_patch": _bound_camera_file(
            runtime_env,
            "CARLA_CAMERA_DELIVERY_PATCH_FILE",
            "CARLA_CAMERA_DELIVERY_PATCH_SHA256",
            attempt,
            failures,
            "delivery patch",
        ),
    }
    patch = _resolved_recorded_path(
        runtime_env.get("CARLA_CAMERA_DELIVERY_PATCH_FILE"), attempt
    )
    worktree = _resolved_recorded_path(
        runtime_env.get("CARLA_CAMERA_DELIVERY_PATCH_WORKTREE"), attempt
    )
    reverse_status = "FAILED"
    if (
        patch is not None
        and patch.is_file()
        and worktree is not None
        and worktree.is_dir()
        and runtime_env.get("CARLA_CAMERA_DELIVERY_PATCH_REVERSE_CHECK") == "PASS"
    ):
        completed = subprocess.run(
            [
                "git",
                "-C",
                str(worktree),
                "apply",
                "--reverse",
                "--check",
                str(patch),
            ],
            check=False,
            capture_output=True,
            text=True,
        )
        if completed.returncode == 0:
            reverse_status = "PASS"
    if reverse_status != "PASS":
        failures.append("strict camera delivery patch reverse-check failed")
    records["patch_application"] = {
        "status": reverse_status,
        "worktree": str(worktree) if worktree is not None else None,
        "verification": "git_apply_reverse_check",
    }
    return records


def _validate_legacy_strict_camera_delivery_provenance(
    runtime_env: dict[str, str],
    recorded: object,
    failures: list[str],
) -> dict[str, Any] | None:
    # HH_260906 - Validate v1 recorded identities without rebinding them to v2 files.
    if not isinstance(recorded, dict):
        failures.append("legacy strict camera delivery provenance is missing")
        return None
    identity_fields = {
        "bundle_dispatch_source": (
            "CARLA_CAMERA_BUNDLE_DISPATCH_SOURCE_FILE",
            "CARLA_CAMERA_BUNDLE_DISPATCH_SHA256",
        ),
        "bundle_dispatch_runtime": (
            "CARLA_CAMERA_BUNDLE_DISPATCH_RUNTIME_FILE",
            "CARLA_CAMERA_BUNDLE_DISPATCH_SHA256",
        ),
        "bridge_source": (
            "CARLA_CAMERA_BRIDGE_SOURCE_FILE",
            "CARLA_CAMERA_BRIDGE_SHA256",
        ),
        "bridge_runtime": (
            "CARLA_CAMERA_BRIDGE_RUNTIME_FILE",
            "CARLA_CAMERA_BRIDGE_SHA256",
        ),
        "entrypoint_source": (
            "CARLA_CAMERA_ENTRYPOINT_SOURCE_FILE",
            "CARLA_CAMERA_ENTRYPOINT_SHA256",
        ),
        "entrypoint_runtime": (
            "CARLA_CAMERA_ENTRYPOINT_RUNTIME_FILE",
            "CARLA_CAMERA_ENTRYPOINT_SHA256",
        ),
        "publish_worker_source": (
            "CARLA_CAMERA_PUBLISH_WORKER_SOURCE_FILE",
            "CARLA_CAMERA_PUBLISH_WORKER_SHA256",
        ),
        "publish_worker_runtime": (
            "CARLA_CAMERA_PUBLISH_WORKER_RUNTIME_FILE",
            "CARLA_CAMERA_PUBLISH_WORKER_SHA256",
        ),
        "interface_launch_source": (
            "CARLA_CAMERA_INTERFACE_LAUNCH_SOURCE_FILE",
            "CARLA_CAMERA_INTERFACE_LAUNCH_SHA256",
        ),
        "interface_launch_runtime": (
            "CARLA_CAMERA_INTERFACE_LAUNCH_RUNTIME_FILE",
            "CARLA_CAMERA_INTERFACE_LAUNCH_SHA256",
        ),
        "delivery_patch": (
            "CARLA_CAMERA_DELIVERY_PATCH_FILE",
            "CARLA_CAMERA_DELIVERY_PATCH_SHA256",
        ),
    }
    for label, (path_key, sha_key) in identity_fields.items():
        item = recorded.get(label)
        expected_sha = runtime_env.get(sha_key)
        if (
            not isinstance(item, dict)
            or item.get("declared_path") != runtime_env.get(path_key)
            or not isinstance(item.get("path"), str)
            or not item.get("path")
            or re.fullmatch(r"[0-9a-f]{64}", str(expected_sha)) is None
            or item.get("sha256") != expected_sha
        ):
            failures.append(f"legacy strict camera {label} recorded identity mismatch")
    patch_application = recorded.get("patch_application")
    expected_worktree = runtime_env.get("CARLA_CAMERA_DELIVERY_PATCH_WORKTREE")
    if (
        not isinstance(patch_application, dict)
        or patch_application.get("status") != "PASS"
        or patch_application.get("verification") != "git_apply_reverse_check"
        or patch_application.get("worktree") != expected_worktree
        or runtime_env.get("CARLA_CAMERA_DELIVERY_PATCH_REVERSE_CHECK") != "PASS"
    ):
        failures.append("legacy strict camera recorded patch application mismatch")
    return recorded


def _validate_strict_runtime_parameters(
    runtime_env: dict[str, str],
    attempt: Path,
    failures: list[str],
) -> dict[str, Any]:
    path = _resolved_recorded_path(
        runtime_env.get("CARLA_CAMERA_RUNTIME_PARAMETERS_FILE"), attempt
    )
    expected_sha = runtime_env.get("CARLA_CAMERA_RUNTIME_PARAMETERS_SHA256")
    source = {
        "path": str(path) if path is not None else None,
        "sha256": None,
        "evidence": None,
    }
    if (
        path is None
        or path.is_symlink()
        or not path.is_file()
        or not isinstance(expected_sha, str)
        or re.fullmatch(r"[0-9a-f]{64}", expected_sha) is None
    ):
        failures.append("strict live camera runtime-parameter evidence is invalid")
        return source
    actual_sha = _sha256_file(path)
    source["sha256"] = actual_sha
    if actual_sha != expected_sha:
        failures.append("strict live camera runtime-parameter evidence SHA-256 mismatch")
        return source
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        failures.append(f"cannot read strict live camera runtime parameters: {error}")
        return source
    source["evidence"] = document
    expected_parameters = {
        "fixed_delta_seconds": 0.05,
        "sync_mode": True,
        "camera_frame_barrier_enabled": True,
        "camera_frame_wait_timeout_sec": 0.25,
        "camera_publish_deadline_sec": 0.25,
        "camera_pending_frame_limit": 8,
    }
    if (
        not isinstance(document, dict)
        or document.get("schema_version") != 1
        or document.get("status") != "PASS"
        or document.get("node") != "/autoware_carla_interface"
        or document.get("parameters") != expected_parameters
        or document.get("derived_camera_frame_stride") != 2
        or document.get("derivation")
        != "sensor_tick_0.1_sec_divided_by_fixed_delta_0.05_sec"
        or document.get("read_only") is not True
    ):
        failures.append("strict live camera runtime-parameter contract mismatch")
    return source


def _validate_checksum_manifest(
    manifest_path: Path,
    expected_root: Path,
    required_files: set[str],
    failures: list[str],
    label: str,
) -> dict[str, Any]:
    source = {
        "manifest": _file_identity(manifest_path, failures, f"{label} checksum manifest"),
        "files": {},
    }
    if source["manifest"]["sha256"] is None:
        return source
    try:
        lines = manifest_path.read_text(encoding="utf-8").splitlines()
    except (OSError, UnicodeDecodeError) as error:
        failures.append(f"cannot read {label} checksum manifest: {error}")
        return source
    recorded: dict[str, str] = {}
    for number, raw in enumerate(lines, 1):
        parts = raw.split(maxsplit=1)
        if len(parts) != 2:
            failures.append(f"{label} checksum line {number} is malformed")
            continue
        digest, relative = parts[0].lower(), parts[1].strip().lstrip("*")
        candidate = Path(relative)
        if (
            len(digest) != 64
            or any(character not in "0123456789abcdef" for character in digest)
            or not relative
            or relative in recorded
            or candidate.is_absolute()
            or ".." in candidate.parts
        ):
            failures.append(f"{label} checksum line {number} is unsafe or invalid")
            continue
        recorded[relative] = digest
    missing = sorted(required_files.difference(recorded))
    if missing:
        failures.append(f"{label} checksum manifest is missing: {', '.join(missing)}")
    for relative, expected_digest in sorted(recorded.items()):
        path = expected_root / relative
        identity = _file_identity(path, failures, f"{label} file {relative}")
        source["files"][relative] = identity
        if identity["sha256"] is not None and identity["sha256"] != expected_digest:
            failures.append(f"{label} file {relative} checksum mismatch")
    return source


def _validate_carla_lifecycle(
    attempt: Path,
    evidence: dict[str, dict[str, Any]],
    runtime_env: dict[str, str],
    failures: list[str],
) -> dict[str, Any]:
    definitions = {
        "preflight": ("carla_preflight_health.json", "trial_preflight", "running"),
        "completion": ("carla_completion_health.json", "trial_completion", "running"),
        "cleanup": ("carla_cleanup_health.json", "trial_cleanup", "stopped"),
    }
    sources: dict[str, Any] = {}
    expected_generation = runtime_env.get("CARLA_GENERATION_ID")
    expected_owner_pid = _parsed_positive_integer(runtime_env.get("CARLA_OWNER_PID"))
    expected_owner_pgid = _parsed_positive_integer(runtime_env.get("CARLA_OWNER_PGID"))
    expected_port = _parsed_positive_integer(runtime_env.get("CARLA_PORT"), 65535)
    server_log_path = (attempt / "carla_server.log").resolve()
    if (
        runtime_env.get("CARLA_HOST") != "127.0.0.1"
        or runtime_env.get("CARLA_LIFECYCLE")
        != "cold_start_owned_process_group_per_trial"
        or runtime_env.get("CARLA_MATRIX_OWNED") != "true"
        or _resolved_recorded_path(runtime_env.get("CARLA_SERVER_LOG"), attempt)
        != server_log_path
    ):
        failures.append("runtime.env CARLA lifecycle ownership contract mismatch")
    if runtime_env.get("CARLA_EXPECTED_MAP") != "Town06":
        failures.append("runtime.env CARLA_EXPECTED_MAP is not Town06")
    if (
        not isinstance(expected_generation, str)
        or not expected_generation
        or expected_owner_pid is None
        or expected_owner_pgid is None
        or expected_owner_pid != expected_owner_pgid
        or expected_port is None
    ):
        failures.append("runtime.env CARLA generation/owner/port identity is incomplete")

    recorded_log_sizes: list[int] = []
    for key, (name, stage, mode) in definitions.items():
        path = attempt / name
        sources[key] = _file_identity(path, failures, f"CARLA {key} health evidence")
        document = evidence.get(name, {})
        if (
            document.get("schema_version") != 1
            or document.get("stage") != stage
            or document.get("status") != "PASS"
            or document.get("mode") != mode
            or document.get("expected_map") != "Town06"
            or document.get("generation_id") != expected_generation
            or document.get("owner_pid") != expected_owner_pid
            or document.get("owner_pgid") != expected_owner_pgid
            or document.get("port") != expected_port
            or document.get("read_only") is not True
            or document.get("error") is not None
        ):
            failures.append(f"CARLA {key} lifecycle contract mismatch")
        if key != "cleanup" and (
            document.get("active_map_basename") != "Town06"
            or document.get("active_map_name") != "Carla/Maps/Town06"
            or not isinstance(document.get("owner_process_state"), str)
            or not document.get("owner_process_state")
        ):
            failures.append(f"CARLA {key} running-state/map contract mismatch")
        if key == "cleanup" and (
            document.get("port_released") is not True
            or document.get("owner_process_state") is not None
        ):
            failures.append(
                "CARLA cleanup did not prove owner exit and RPC port release"
            )
        server_log = document.get("server_log")
        recorded_size = (
            server_log.get("size_bytes") if isinstance(server_log, dict) else None
        )
        if (
            not isinstance(server_log, dict)
            or _resolved_recorded_path(server_log.get("path"), attempt)
            != server_log_path
            or not isinstance(recorded_size, int)
            or isinstance(recorded_size, bool)
            or recorded_size < 0
            or not isinstance(server_log.get("sha256"), str)
        ):
            failures.append(f"CARLA {key} server-log provenance is incomplete")
            continue
        recorded_log_sizes.append(recorded_size)
        try:
            if server_log_path.is_symlink() or not server_log_path.is_file():
                raise OSError("not a regular file")
            actual_size = server_log_path.stat().st_size
            with server_log_path.open("rb") as stream:
                prefix = stream.read(recorded_size)
        except OSError as error:
            failures.append(f"CARLA {key} server log is invalid: {error}")
            continue
        if (
            len(prefix) != recorded_size
            or hashlib.sha256(prefix).hexdigest() != server_log.get("sha256")
            or (key == "cleanup" and actual_size != recorded_size)
        ):
            failures.append(f"CARLA {key} server-log digest/size mismatch")
    if len(recorded_log_sizes) == 3 and recorded_log_sizes != sorted(recorded_log_sizes):
        failures.append("CARLA lifecycle server-log snapshot sizes are not monotonic")
    sources["server_log"] = _file_identity(
        server_log_path, failures, "CARLA server log"
    )
    return sources


def _validate_actuation_provenance(
    attempt: Path,
    documents: list[tuple[str, dict[str, Any]]],
    failures: list[str],
) -> dict[str, Any]:
    root = (attempt / "actuation_config_provenance").resolve()
    manifest_path = root / "manifest.json"
    source: dict[str, Any] = {
        "root": str(root),
        "manifest": _file_identity(
            manifest_path, failures, "actuation configuration manifest"
        ),
        "files": {},
    }
    try:
        if manifest_path.is_symlink() or not manifest_path.is_file():
            raise OSError("not a regular file")
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        failures.append(f"cannot read actuation configuration manifest: {error}")
        return source
    records = manifest.get("files") if isinstance(manifest, dict) else None
    required = {"config", "accel_map", "brake_map", "steer_map"}
    execution = manifest.get("execution") if isinstance(manifest, dict) else None
    if (
        manifest.get("schema_version") != 1
        or not isinstance(records, dict)
        or not isinstance(execution, dict)
        or execution.get("uses_original_selected_config") is not True
        or execution.get("uses_artifact_copy") is not False
    ):
        failures.append("actuation configuration manifest contract mismatch")
        return source
    missing = sorted(required.difference(records))
    if missing:
        failures.append(
            "actuation configuration manifest is missing: " + ", ".join(missing)
        )
    expected_artifacts = {
        "config": "raw_vehicle_cmd_converter.param.yaml",
        "accel_map": "accel_map.csv",
        "brake_map": "brake_map.csv",
        "steer_map": "steer_map.csv",
    }
    for key, record in sorted(records.items()):
        if not isinstance(record, dict):
            failures.append(f"actuation manifest {key} entry is not an object")
            continue
        artifact = record.get("artifact")
        relative = Path(artifact) if isinstance(artifact, str) else Path(".")
        if (
            not isinstance(artifact, str)
            or not artifact
            or relative.is_absolute()
            or ".." in relative.parts
        ):
            failures.append(f"actuation manifest {key} artifact path is unsafe")
            continue
        if key in expected_artifacts and artifact != expected_artifacts[key]:
            failures.append(f"actuation manifest {key} artifact name changed")
        path = root / relative
        identity = _file_identity(path, failures, f"actuation artifact {key}")
        source["files"][key] = {"artifact": artifact, **identity}
        try:
            actual_size = path.stat().st_size
        except OSError:
            actual_size = None
        if (
            identity["sha256"] is not None
            and (
                record.get("sha256") != identity["sha256"]
                or record.get("size_bytes") != actual_size
            )
        ):
            failures.append(f"actuation artifact {key} digest/size mismatch")

    for name, document in documents:
        provenance = document.get("provenance")
        if not isinstance(provenance, dict):
            failures.append(f"{name} actuation provenance is missing")
            continue
        declared_manifest = provenance.get("manifest")
        if (
            _resolved_recorded_path(provenance.get("root"), attempt) != root
            or not isinstance(declared_manifest, dict)
            or _resolved_recorded_path(declared_manifest.get("path"), attempt)
            != manifest_path
            or declared_manifest.get("sha256") != source["manifest"]["sha256"]
        ):
            failures.append(f"{name} actuation manifest provenance mismatch")
        maps = provenance.get("maps")
        for map_key in ("accel_map", "brake_map"):
            declared = maps.get(map_key) if isinstance(maps, dict) else None
            record = records.get(map_key)
            actual = source["files"].get(map_key)
            if (
                not isinstance(declared, dict)
                or not isinstance(record, dict)
                or not isinstance(actual, dict)
                or declared.get("artifact") != record.get("artifact")
                or declared.get("sha256") != actual.get("sha256")
            ):
                failures.append(f"{name} {map_key} provenance mismatch")
    if len(documents) == 2:
        left = documents[0][1].get("provenance")
        right = documents[1][1].get("provenance")
        if left != right:
            failures.append("pre/post actuation-map provenance differs")
    return source


def evaluate_trial(attempt_dir: Path, source_route: Path) -> dict[str, Any]:
    integrity_failures: list[str] = []
    acceptance_failures: list[str] = []
    camera_transport_failures: list[str] = []
    readiness_blockers: list[str] = []
    evidence: dict[str, dict[str, Any]] = {}

    attempt_input = attempt_dir.expanduser()
    source_input = source_route.expanduser()
    if attempt_input.is_symlink():
        integrity_failures.append("attempt directory must not be a symlink")
    attempt = attempt_input.resolve()
    source = source_input.resolve()
    if not attempt.is_dir():
        integrity_failures.append(f"attempt directory does not exist: {attempt}")
    if source_input.is_symlink() or not source.is_file():
        integrity_failures.append(f"source route is not a regular file: {source}")

    for name in REQUIRED_JSON:
        path = attempt / name
        try:
            if path.is_symlink() or not path.is_file():
                raise OSError("not a regular file")
            value = json.loads(path.read_text(encoding="utf-8"))
            if not isinstance(value, dict):
                raise ValueError("top level is not an object")
            evidence[name] = value
        except (OSError, UnicodeDecodeError, json.JSONDecodeError, ValueError) as error:
            integrity_failures.append(f"cannot read {name}: {error}")

    runtime_env: dict[str, str] = {}
    runtime_env_path = attempt / "runtime.env"
    geometry_variant: dict[str, Any] = {
        "provenance_status": "LEGACY_NOT_RECORDED",
        "candidate_id": None,
        "route_corridor_0p2": None,
        "route_corridor_half_width_m": None,
        "turn_outward_corridor_half_width_m": None,
        "parameter_dump_sha256": None,
        "trajectory_checksum_manifest_sha256": None,
    }
    try:
        if runtime_env_path.is_symlink() or not runtime_env_path.is_file():
            raise OSError("not a regular file")
        runtime_env = _parse_env(runtime_env_path)
    except (OSError, UnicodeDecodeError, ValueError) as error:
        integrity_failures.append(f"cannot read runtime.env: {error}")

    camera_profile_id = runtime_env.get("CAMERA_TRANSPORT_PROFILE_ID")
    camera_evidence_name = CAMERA_EVIDENCE_BY_PROFILE.get(camera_profile_id)
    if camera_evidence_name is None:
        integrity_failures.append(
            f"unsupported camera transport profile: {camera_profile_id!r}"
        )
    else:
        camera_path = attempt / camera_evidence_name
        try:
            if camera_path.is_symlink() or not camera_path.is_file():
                raise OSError("not a regular file")
            camera_value = json.loads(camera_path.read_text(encoding="utf-8"))
            if not isinstance(camera_value, dict):
                raise ValueError("top level is not an object")
            evidence[camera_evidence_name] = camera_value
        except (
            OSError,
            UnicodeDecodeError,
            json.JSONDecodeError,
            ValueError,
        ) as error:
            integrity_failures.append(f"cannot read {camera_evidence_name}: {error}")
    if (
        camera_profile_id in CAMERA_PROFILE_10HZ_STRICT_IDS
        and "runtime_load_analysis.json" not in evidence
    ):
        camera_transport_failures.append(
            "strict full-run runtime-load evidence is missing or malformed"
        )

    source_sha = None
    aligned_sha = None
    result_sha = None
    if source.is_file():
        source_sha = _sha256_file(source)
    source_copy = attempt / "source_route.json"
    aligned = attempt / "aligned_route.json"
    result_path = attempt / "result.json"
    if source_copy.is_file() and not source_copy.is_symlink():
        copied_sha = _sha256_file(source_copy)
        if source_sha is not None and copied_sha != source_sha:
            integrity_failures.append("source_route.json does not match selected catalog route")
    if aligned.is_file() and not aligned.is_symlink():
        aligned_sha = _sha256_file(aligned)
    if result_path.is_file() and not result_path.is_symlink():
        result_sha = _sha256_file(result_path)

    alignment = evidence.get("route_alignment.json", {})
    if alignment:
        if alignment.get("status") != "PASS":
            acceptance_failures.append("route alignment status is not PASS")
        if (
            _resolved_recorded_path(alignment.get("source_route"), attempt) != source
            or alignment.get("source_route_sha256") != source_sha
            or _resolved_recorded_path(alignment.get("aligned_route"), attempt)
            != aligned.resolve()
            or alignment.get("aligned_route_sha256") != aligned_sha
        ):
            integrity_failures.append("route-alignment path/SHA provenance mismatch")

    route = evidence.get("aligned_route.json", {})
    source_payload = evidence.get("source_route.json", {})
    if route and source_payload:
        for label, payload in (("source", source_payload), ("aligned", route)):
            if payload.get("town") != "Town06" or payload.get("scenario") != "straight":
                integrity_failures.append(f"{label} route is not Town06/straight")
        physical = source_payload.get("physical_straight_preflight")
        if not isinstance(physical, dict) or physical.get("status") != "PASS":
            acceptance_failures.append("source route physical-straight preflight is not PASS")
        length = source_payload.get("route_length_m")
        if not _finite(length) or not 430.0 <= float(length) <= 460.0:
            integrity_failures.append("source route length is outside [430, 460] m")

    expected_env = {
        "SPEED_60KPH_PILOT": "true",
        "SPEED_PROFILE_ID": PROFILE_ID,
        "ROUTE_SCENARIO": "straight",
        "SPEED_EXPOSURE_MODE": "straight_target_required",
        "TARGET_SPEED_MPS": str(TARGET_SPEED_MPS),
        "MINIMUM_SUSTAINED_SPEED_MPS": str(MINIMUM_SUSTAINED_SPEED_MPS),
        "MINIMUM_SUSTAINED_SPEED_SEC": str(MINIMUM_SUSTAINED_SPEED_SEC),
        "MAXIMUM_OBSERVED_SPEED_MPS": str(MAXIMUM_OBSERVED_SPEED_MPS),
        "MAXIMUM_LATERAL_ACCELERATION_LIMIT_MPS2": str(
            MAXIMUM_LATERAL_ACCELERATION_MPS2
        ),
        "MAXIMUM_SPEED_SAMPLE_GAP_SEC": str(MAXIMUM_SPEED_SAMPLE_GAP_SEC),
        "RUNTIME_HEALTH_GATE_ENABLED": "true",
        "RUNTIME_HEALTH_GATE_STATUS": "PASS",
        "RUNTIME_HEALTH_REQUIRED_CONSECUTIVE_PASSES": "3",
        "REAL_VEHICLE_READY": "false",
        "SIMULATION_ONLY_EXPLORATORY": "true",
        "ROUTE_SCOPE": "straight_only",
    }
    if camera_profile_id == CAMERA_PROFILE_5HZ_ID:
        expected_env.update(
            {
                "CAMERA_SOURCE_5HZ": "true",
                "CAMERA_TRANSPORT_PROFILE_ID": CAMERA_PROFILE_5HZ_ID,
            }
        )
    elif camera_profile_id in CAMERA_PROFILE_10HZ_STRICT_IDS:
        expected_env.update(
            {
                "CAMERA_SOURCE_5HZ": "false",
                "CAMERA_SOURCE_10HZ_STRICT": "true",
                "CAMERA_SOURCE_SENSOR_TICK_SEC": "0.1",
                "CAMERA_ROS_PUBLISH_HZ": "10.0",
                "CAMERA_TRANSPORT_PROFILE_ID": camera_profile_id,
                "CAMERA_TRANSPORT_QUALIFICATION_SCOPE": (
                    "strict_six_camera_10hz_transport_runtime_only"
                ),
                "CAMERA_TRANSPORT_SIMULATION_ONLY": "true",
                "CAMERA_TRANSPORT_VEHICLE_BEHAVIOR_VALIDATED": "false",
                "CAMERA_TRANSPORT_REAL_VEHICLE_READY": "false",
                "CAMERA_TRANSPORT_SENSOR_MAPPING_ROLE": (
                    "six_camera_source_abi_only"
                ),
                "CAMERA_TRANSPORT_SENSOR_MAPPING_REUSE": (
                    "portable_common10_mapping_without_portable_runtime"
                ),
                "PORTABLE_SHADOW_LAUNCH_REQUESTED": "false",
                "PORTABLE_SHADOW_ENABLED": "false",
                "PORTABLE_RUNTIME_BUNDLE_PROVIDED": "false",
                "PORTABLE_MODEL_LOADED": "false",
                "PORTABLE_MODEL_60KPH_VALIDATED": "false",
                "PORTABLE_MODEL_60KPH_CLAIM_ALLOWED": "false",
                "CARLA_CAMERA_BUNDLE_DISPATCH_POLICY": (
                    "exact_due_frame_barrier_fail_closed_v1"
                ),
                "CARLA_CAMERA_FRAME_BARRIER_ENABLED": "true",
                "CARLA_CAMERA_FRAME_STRIDE": "2",
                "CARLA_CAMERA_FRAME_WAIT_TIMEOUT_SEC": "0.25",
                "CARLA_CAMERA_PUBLISH_DEADLINE_SEC": "0.25",
                "CARLA_CAMERA_PENDING_FRAME_LIMIT": "8",
                "CARLA_CAMERA_EXPECTED_RGB_COUNT": "6",
                "CARLA_CAMERA_BARRIER_SCOPE": (
                    "strict_six_camera_10hz_transport_only"
                ),
                "CARLA_CAMERA_DELIVERY_CONTRACT_ID": (
                    "strict10_exact_due_frame_fail_closed_v1"
                ),
                "CARLA_FIXED_DELTA_SECONDS_PINNED": "0.05",
                "CARLA_SYNC_MODE_PINNED": "true",
                "CARLA_CAMERA_RUNTIME_PARAMETERS_STATUS": "PASS",
                "CARLA_CAMERA_DELIVERY_PATCH_REVERSE_CHECK": "PASS",
            }
        )
        if camera_profile_id == CAMERA_PROFILE_10HZ_STRICT_ID:
            # HH_260906 - Require recorder-boundary proof only from newly issued v2 runs.
            expected_env.update(
                {
                    "STRICT_RECORDER_START_PAUSED": "true",
                    "STRICT_RECORDER_CAMERA_INFO_SUBSCRIPTIONS_ACKNOWLEDGED": "6",
                    "STRICT_RECORDER_RESUME_CONTROL": (
                        "humble_rosbag2_owned_pty_space_key_v1"
                    ),
                    "STRICT_RECORDER_MEASUREMENT_RESUME_STATUS": "pass",
                    "STRICT_RECORDER_MEASUREMENT_EVIDENCE_FILE": (
                        "recorder_measurement_start.json"
                    ),
                    "RECORDER_ROUTE_EVALUATION_LIVENESS_STATUS": "pass",
                    "RECORDER_ROUTE_COMPLETION_LIVENESS_STATUS": "pass",
                    "RECORDER_OWNED_GROUP_CLEANUP_STATUS": "pass",
                    "RECORDER_CONTROL_FIFO_REMOVED": "true",
                    "STACK_OWNED_GROUP_CLEANUP_STATUS": "pass",
                    "STACK_POST_SHUTDOWN_CRITICAL_PROCESS_CHECK": "pass",
                }
            )
    geometry_contract_present = False
    geometry_failures: list[str] = []
    expected_variant: tuple[str, float] | None = None
    parsed_route_width = None
    parsed_turn_width = None
    if runtime_env:
        for key, expected in expected_env.items():
            if runtime_env.get(key) != expected:
                message = (
                    f"runtime.env {key} mismatch: "
                    f"{runtime_env.get(key)!r} != {expected!r}"
                )
                integrity_failures.append(message)
                if (
                    camera_profile_id in CAMERA_PROFILE_10HZ_STRICT_IDS
                    and (
                        key.startswith("CAMERA_")
                        or key.startswith("CARLA_CAMERA_")
                        or key.startswith("PORTABLE_")
                        or key.startswith("STRICT_RECORDER_")
                        or key.startswith("RECORDER_")
                    )
                ):
                    camera_transport_failures.append(message)
        if camera_profile_id == CAMERA_PROFILE_5HZ_ID:
            for key in ("CAMERA_SOURCE_10HZ_STRICT", "PORTABLE_SHADOW_ENABLED"):
                if runtime_env.get(key) not in (None, "false"):
                    message = (
                        f"runtime.env legacy {key} must be absent or false: "
                        f"{runtime_env.get(key)!r}"
                    )
                    integrity_failures.append(message)
                    camera_transport_failures.append(message)
        if _resolved_recorded_path(runtime_env.get("SOURCE_ROUTE_FILE"), attempt) != source:
            integrity_failures.append("runtime.env SOURCE_ROUTE_FILE mismatch")
        if _resolved_recorded_path(runtime_env.get("EFFECTIVE_ROUTE_FILE"), attempt) != aligned.resolve():
            integrity_failures.append("runtime.env EFFECTIVE_ROUTE_FILE mismatch")
        geometry_keys = {
            "GEOMETRY_AB_CANDIDATE_ID",
            "GEOMETRY_AB_ROUTE_CORRIDOR_0P2",
            "GEOMETRY_AB_ROUTE_CORRIDOR_BASELINE_M",
            "GEOMETRY_AB_ROUTE_CORRIDOR_CANDIDATE_M",
            "GEOMETRY_AB_BEHAVIORAL_SINGLE_KNOB",
            "GEOMETRY_AB_PARAMETER_CHANGE_COUNT",
            "GEOMETRY_AB_COUPLED_PARAMETER_REASON",
            "GEOMETRY_AB_ROUTE_SCOPE",
            "ROUTE_CORRIDOR_HALF_WIDTH_M",
            "TURN_OUTWARD_CORRIDOR_HALF_WIDTH_M",
        }
        present_geometry_keys = geometry_keys.intersection(runtime_env)
        if present_geometry_keys:
            geometry_contract_present = True
            if present_geometry_keys != geometry_keys:
                geometry_failures.append(
                    "runtime.env has partial 60 km/h geometry A/B provenance"
                )
            candidate_id = runtime_env.get("GEOMETRY_AB_CANDIDATE_ID")
            enabled = runtime_env.get("GEOMETRY_AB_ROUTE_CORRIDOR_0P2")
            expected_variant = {
                "baseline_corridor_0p5": ("false", 0.50),
                "route_corridor_0p2": ("true", 0.20),
            }.get(candidate_id)
            route_width = runtime_env.get("ROUTE_CORRIDOR_HALF_WIDTH_M")
            turn_width = runtime_env.get("TURN_OUTWARD_CORRIDOR_HALF_WIDTH_M")
            parsed_route_width = _parsed_number(route_width)
            parsed_turn_width = _parsed_number(turn_width)
            valid_widths = (
                expected_variant is not None
                and _close(parsed_route_width, expected_variant[1])
                and _close(parsed_turn_width, expected_variant[1])
            )
            if (
                expected_variant is None
                or enabled != expected_variant[0]
                or not valid_widths
                or not _close(
                    _parsed_number(
                        runtime_env.get("GEOMETRY_AB_ROUTE_CORRIDOR_BASELINE_M")
                    ),
                    0.50,
                )
                or not _close(
                    _parsed_number(
                        runtime_env.get("GEOMETRY_AB_ROUTE_CORRIDOR_CANDIDATE_M")
                    ),
                    0.20,
                )
                or runtime_env.get("GEOMETRY_AB_BEHAVIORAL_SINGLE_KNOB") != "true"
                or runtime_env.get("GEOMETRY_AB_PARAMETER_CHANGE_COUNT") != "2"
                or runtime_env.get("GEOMETRY_AB_COUPLED_PARAMETER_REASON")
                != "turn_width_must_not_exceed_route_width"
                or runtime_env.get("GEOMETRY_AB_ROUTE_SCOPE") != "straight_only"
            ):
                geometry_failures.append(
                    "runtime.env 60 km/h geometry A/B provenance is inconsistent"
                )
            geometry_variant = {
                "provenance_status": "FAILED",
                "candidate_id": candidate_id,
                "route_corridor_0p2": enabled == "true",
                "route_corridor_half_width_m": parsed_route_width,
                "turn_outward_corridor_half_width_m": parsed_turn_width,
                "parameter_dump_sha256": None,
                "trajectory_checksum_manifest_sha256": None,
            }

    camera_delivery_provenance_source = None
    strict_runtime_parameters_source = None
    strict_recorder_measurement_source = None
    if (
        camera_profile_id in CAMERA_PROFILE_10HZ_STRICT_IDS
        and runtime_env
    ):
        strict_provenance_failures: list[str] = []
        if camera_profile_id == CAMERA_PROFILE_10HZ_STRICT_LEGACY_ID:
            camera_delivery_provenance_source = (
                _validate_legacy_strict_camera_delivery_provenance(
                    runtime_env,
                    _nested(
                        evidence,
                        str(camera_evidence_name),
                        "camera_delivery_contract_provenance",
                    ),
                    strict_provenance_failures,
                )
            )
        else:
            camera_delivery_provenance_source = (
                _validate_strict_camera_delivery_provenance(
                    runtime_env, attempt, strict_provenance_failures
                )
            )
        strict_runtime_parameters_source = _validate_strict_runtime_parameters(
            runtime_env, attempt, strict_provenance_failures
        )
        integrity_failures.extend(strict_provenance_failures)
        camera_transport_failures.extend(strict_provenance_failures)

    lifecycle_sources = _validate_carla_lifecycle(
        attempt, evidence, runtime_env, integrity_failures
    )
    trajectory_failures: list[str] = []
    trajectory_root = (attempt / "trajectory_code_provenance").resolve()
    trajectory_source = _validate_checksum_manifest(
        trajectory_root / "SHA256SUMS",
        trajectory_root,
        {"vad_route_logic.py", "vad_route_manager.py"},
        trajectory_failures,
        "trajectory code provenance",
    )
    for name, env_key in (
        ("vad_route_logic.py", "TRAJECTORY_LOGIC_SHA256"),
        ("vad_route_manager.py", "VAD_ROUTE_MANAGER_SHA256"),
    ):
        actual_digest = _nested(trajectory_source, "files", name, "sha256")
        if runtime_env.get(env_key) != actual_digest:
            trajectory_failures.append(
                f"runtime.env {env_key} does not match captured trajectory code"
            )
    integrity_failures.extend(trajectory_failures)

    geometry_parameter_source: dict[str, Any] = {
        "path": str((attempt / "vad_route_manager.params.yaml").resolve()),
        "sha256": None,
    }
    if geometry_contract_present:
        parameter_path = attempt / "vad_route_manager.params.yaml"
        geometry_parameter_source = _file_identity(
            parameter_path,
            geometry_failures,
            "VAD route-manager parameter dump",
        )
        if geometry_parameter_source["sha256"] is not None:
            try:
                parameter_document = yaml.safe_load(
                    parameter_path.read_text(encoding="utf-8")
                )
            except (OSError, UnicodeDecodeError, yaml.YAMLError) as error:
                geometry_failures.append(
                    f"cannot read VAD route-manager parameter dump: {error}"
                )
                parameter_document = None
            parameters = _nested(
                parameter_document, "/vad_route_manager", "ros__parameters"
            )
            if (
                not isinstance(parameters, dict)
                or parsed_route_width is None
                or parsed_turn_width is None
                or not _close(
                    parameters.get("route_corridor_half_width_m"),
                    parsed_route_width,
                )
                or not _close(
                    parameters.get("turn_outward_corridor_half_width_m"),
                    parsed_turn_width,
                )
                or _resolved_recorded_path(parameters.get("route_file"), attempt)
                != aligned.resolve()
            ):
                geometry_failures.append(
                    "runtime geometry does not match vad_route_manager.params.yaml"
                )
        geometry_variant["parameter_dump_sha256"] = geometry_parameter_source[
            "sha256"
        ]
        geometry_variant["trajectory_checksum_manifest_sha256"] = _nested(
            trajectory_source, "manifest", "sha256"
        )
        if not geometry_failures and not trajectory_failures:
            geometry_variant["provenance_status"] = "PASS"
        integrity_failures.extend(geometry_failures)

    result = evidence.get("result.json", {})
    metrics: dict[str, Any] = {}
    exposure: dict[str, Any] = {}
    physical_goal_completion_status = "FAILED"
    speed_exposure_contract_status = "FAILED"
    full_stack_route_test_status = "FAILED"
    if result:
        metrics_value = result.get("metrics")
        exposure_value = result.get("speed_exposure")
        limits = result.get("limits")
        metrics = metrics_value if isinstance(metrics_value, dict) else {}
        exposure = exposure_value if isinstance(exposure_value, dict) else {}
        if (
            result.get("schema_version") != 1
            or result.get("execution_mode") != "full_stack"
            or result.get("profile_context") != PROFILE_CONTEXT
            or not isinstance(limits, dict)
            or not isinstance(metrics_value, dict)
            or not isinstance(exposure_value, dict)
        ):
            integrity_failures.append("route result top-level/profile contract mismatch")
        if _resolved_recorded_path(result.get("route_file"), attempt) != aligned.resolve():
            integrity_failures.append("route result is not bound to aligned_route.json")
        expected_limits = {
            "minimum_sustained_speed_mps": MINIMUM_SUSTAINED_SPEED_MPS,
            "minimum_sustained_speed_sec": MINIMUM_SUSTAINED_SPEED_SEC,
            "maximum_observed_speed_mps": MAXIMUM_OBSERVED_SPEED_MPS,
            "maximum_lateral_acceleration_mps2": MAXIMUM_LATERAL_ACCELERATION_MPS2,
            "maximum_speed_sample_gap_sec": MAXIMUM_SPEED_SAMPLE_GAP_SEC,
        }
        if isinstance(limits, dict):
            for key, expected in expected_limits.items():
                if not _close(limits.get(key), expected):
                    integrity_failures.append(f"route result limit {key} changed")
        exposure_limits = {
            "minimum_sustained_speed_mps": MINIMUM_SUSTAINED_SPEED_MPS,
            "minimum_sustained_speed_sec": MINIMUM_SUSTAINED_SPEED_SEC,
            "maximum_observed_speed_limit_mps": MAXIMUM_OBSERVED_SPEED_MPS,
            "maximum_lateral_acceleration_limit_mps2": (
                MAXIMUM_LATERAL_ACCELERATION_MPS2
            ),
        }
        for key, expected in exposure_limits.items():
            if not _close(exposure.get(key), expected):
                integrity_failures.append(f"speed exposure contract {key} changed")
        if any(exposure.get(key) != value for key, value in PROFILE_CONTEXT.items()):
            integrity_failures.append("speed exposure profile context mismatch")
        for exposure_key, metric_key in (
            ("maximum_observed_speed_mps", "maximum_observed_speed_mps"),
            (
                "maximum_sustained_speed_duration_sec",
                "maximum_sustained_speed_duration_sec",
            ),
            (
                "maximum_lateral_acceleration_mps2",
                "maximum_lateral_acceleration_mps2",
            ),
            ("maximum_speed_sample_gap_sec", "maximum_speed_sample_gap_sec"),
        ):
            left = exposure.get(exposure_key)
            right = metrics.get(metric_key)
            if not _finite(left) or not _finite(right) or not math.isclose(
                float(left), float(right), rel_tol=0.0, abs_tol=1.0e-6
            ):
                integrity_failures.append(
                    f"result metric and speed exposure disagree for {metric_key}"
                )
        final = result.get("final")
        assessment = result.get("assessment")
        goal_confirmed = (
            isinstance(final, dict)
            and final.get("goal_reached") is True
            and final.get("route_status") == "goal_reached"
        )
        speed_exposure_passed = exposure.get("status") == "PASS"
        result_success = result.get("success")
        physical_goal_completion_status = "PASS" if goal_confirmed else "FAILED"
        speed_exposure_contract_status = (
            "PASS" if speed_exposure_passed else "FAILED"
        )
        full_stack_route_test_status = "PASS" if result_success is True else "FAILED"
        if not isinstance(result_success, bool):
            integrity_failures.append("route-test overall verdict is not boolean")
        expected_assessment = "PASS" if result_success is True else "FAIL"
        if (
            not isinstance(assessment, dict)
            or assessment.get("route_completion") != expected_assessment
        ):
            integrity_failures.append(
                "route-test assessment is inconsistent with its overall verdict"
            )
        if result_success is True and (not goal_confirmed or not speed_exposure_passed):
            integrity_failures.append(
                "route-test PASS contradicts physical-goal or speed-exposure evidence"
            )
        if not goal_confirmed:
            acceptance_failures.append("physical route goal was not reached")
        if exposure.get("status") != "PASS":
            acceptance_failures.append("60 kph speed exposure contract status is FAIL")
        if (
            result_success is False
            and goal_confirmed
            and speed_exposure_passed
        ):
            acceptance_failures.append(
                "full-stack route-test verdict failed for a non-speed reason"
            )
        if (
            result_success is False
            and goal_confirmed
            and not speed_exposure_passed
            and "speed exposure" not in str(result.get("reason", "")).lower()
        ):
            integrity_failures.append(
                "route-test failure reason does not identify the speed-exposure failure"
            )
        if (
            not _finite(exposure.get("maximum_sustained_speed_duration_sec"))
            or float(exposure.get("maximum_sustained_speed_duration_sec", -1.0))
            < MINIMUM_SUSTAINED_SPEED_SEC
        ):
            acceptance_failures.append("15 m/s speed exposure was not sustained for 1.0 s")
        if (
            not _finite(metrics.get("maximum_observed_speed_mps"))
            or float(metrics.get("maximum_observed_speed_mps", math.inf))
            > MAXIMUM_OBSERVED_SPEED_MPS
        ):
            acceptance_failures.append("maximum observed speed is unavailable or exceeds 18 m/s")
        if (
            not _finite(metrics.get("maximum_lateral_acceleration_mps2"))
            or float(metrics.get("maximum_lateral_acceleration_mps2", math.inf))
            > MAXIMUM_LATERAL_ACCELERATION_MPS2
        ):
            acceptance_failures.append("lateral acceleration is unavailable or exceeds 1.2 m/s^2")
        sim_elapsed = metrics.get("sim_elapsed_sec")
        wall_elapsed = metrics.get("wall_elapsed_sec")
        if not _finite(sim_elapsed) or not _finite(wall_elapsed) or float(wall_elapsed) <= 0.0:
            acceptance_failures.append("route real-time factor is unavailable")
        elif float(sim_elapsed) / float(wall_elapsed) < MINIMUM_RUNTIME_RTF:
            acceptance_failures.append("route real-time factor is below 0.9")

    if camera_profile_id == CAMERA_PROFILE_10HZ_STRICT_ID:
        recorder_measurement_failures: list[str] = []
        strict_recorder_measurement_source = _validate_strict_recorder_measurement(
            runtime_env,
            attempt,
            result,
            recorder_measurement_failures,
        )
        integrity_failures.extend(recorder_measurement_failures)
        camera_transport_failures.extend(recorder_measurement_failures)

    health = evidence.get("runtime_health.json", {})
    if health:
        strict_10hz = camera_profile_id in CAMERA_PROFILE_10HZ_STRICT_IDS
        minimum_camera_rate_hz = 9.0 if strict_10hz else MINIMUM_CAMERA_RATE_HZ
        minimum_complete_bundle_count = 70 if strict_10hz else 20
        thresholds = _nested(health, "contract", "thresholds")
        expected_thresholds = {
            "maximum_bundle_receipt_p95_seconds": (
                MAXIMUM_CAMERA_BUNDLE_RECEIPT_P95_SEC
            ),
            "minimum_bundle_coverage_percent": (
                MINIMUM_CAMERA_BUNDLE_COVERAGE_PERCENT
            ),
            "minimum_camera_wall_rate_hz": minimum_camera_rate_hz,
            "minimum_complete_bundle_count": minimum_complete_bundle_count,
            "minimum_rtf": MINIMUM_RUNTIME_RTF,
        }
        if strict_10hz:
            expected_thresholds.update(
                {
                    "maximum_bundle_stamp_span_seconds": (
                        MAXIMUM_STRICT_CAMERA_BUNDLE_STAMP_SPAN_SEC
                    ),
                    "maximum_camera_wall_rate_hz": (
                        MAXIMUM_STRICT_CAMERA_WALL_RATE_HZ
                    ),
                    "minimum_camera_source_rate_hz": (
                        MINIMUM_STRICT_CAMERA_SOURCE_RATE_HZ
                    ),
                    "maximum_camera_source_rate_hz": (
                        MAXIMUM_STRICT_CAMERA_SOURCE_RATE_HZ
                    ),
                    "maximum_camera_source_gap_seconds": (
                        MAXIMUM_STRICT_CAMERA_SOURCE_GAP_SEC
                    ),
                }
            )
            if camera_profile_id == CAMERA_PROFILE_10HZ_STRICT_ID:
                expected_thresholds["minimum_camera_source_gap_seconds"] = (
                    MINIMUM_STRICT_CAMERA_SOURCE_GAP_SEC
                )
        if (
            health.get("schema_version") != 1
            or health.get("probe_id") != "pre_engagement_runtime_health_v1"
            or not isinstance(thresholds, dict)
            or thresholds != expected_thresholds
            or (
                strict_10hz
                and (
                    _nested(
                        health,
                        "contract",
                        "strict_camera_source_integrity_required",
                    )
                    is not True
                    or not _close(
                        _nested(
                            health, "contract", "bundle_match_tolerance_seconds"
                        ),
                        MAXIMUM_STRICT_CAMERA_BUNDLE_STAMP_SPAN_SEC,
                        tolerance=1.0e-12,
                    )
                )
            )
        ):
            message = "runtime-health fixed contract mismatch"
            integrity_failures.append(message)
            camera_transport_failures.append(message)
        health_transport = _nested(health, "contract", "camera_transport")
        if (
            not isinstance(health_transport, dict)
            or health_transport.get("profile_id") != camera_profile_id
        ):
            message = "runtime-health camera transport profile mismatch"
            integrity_failures.append(message)
            camera_transport_failures.append(message)
        if strict_10hz and (
            not isinstance(health_transport, dict)
            or health_transport.get("qualification_scope")
            != "strict_six_camera_10hz_transport_runtime_only"
            or health_transport.get("simulation_only") is not True
            or health_transport.get("vehicle_behavior_validated") is not False
            or health_transport.get("portable_model_loaded") is not False
            or health_transport.get("portable_model_60kph_validated") is not False
            or health_transport.get("portable_model_60kph_claim_allowed") is not False
            or health_transport.get("portable_runtime_graph_absence_required") is not True
            or health_transport.get("portable_shadow_node") != PORTABLE_SHADOW_NODE
            or health_transport.get("portable_output_topics")
            != list(PORTABLE_OUTPUT_TOPICS)
            or health_transport.get("real_vehicle_ready") is not False
        ):
            message = "runtime-health strict 10 Hz qualification boundary mismatch"
            integrity_failures.append(message)
            camera_transport_failures.append(message)
        if runtime_env:
            actual_health_sha = _sha256_file(attempt / "runtime_health.json")
            if runtime_env.get("RUNTIME_HEALTH_EVIDENCE_SHA256") != actual_health_sha:
                message = "runtime-health JSON digest does not match runtime.env"
                integrity_failures.append(message)
                camera_transport_failures.append(message)
            source_record = health.get("source")
            if (
                not isinstance(source_record, dict)
                or source_record.get("sha256")
                != runtime_env.get("RUNTIME_HEALTH_PROBE_SHA256")
            ):
                message = "runtime-health probe provenance mismatch"
                integrity_failures.append(message)
                camera_transport_failures.append(message)
        sequence = health.get("sequence")
        if health.get("status") != "PASS":
            message = "pre-engagement runtime-health status is not PASS"
            acceptance_failures.append(message)
            camera_transport_failures.append(message)
        # HH_260906 - Replay three distinct consecutive health-window indexes.
        winning_indexes = (
            sequence.get("winning_window_indexes")
            if isinstance(sequence, dict)
            else None
        )
        strict_v2_winning_indexes_valid = (
            isinstance(winning_indexes, list)
            and len(winning_indexes) == 3
            and all(
                isinstance(index, int) and not isinstance(index, bool)
                for index in winning_indexes
            )
            and winning_indexes
            == list(range(winning_indexes[0], winning_indexes[0] + 3))
        )
        if (
            not isinstance(sequence, dict)
            or sequence.get("status") != "PASS"
            or sequence.get("timed_out") is not False
            or sequence.get("maximum_consecutive_passes", 0) < 3
            or len(sequence.get("winning_window_indexes", [])) != 3
            or (
                camera_profile_id == CAMERA_PROFILE_10HZ_STRICT_ID
                and not strict_v2_winning_indexes_valid
            )
        ):
            message = "runtime-health lacks three consecutive PASS windows"
            acceptance_failures.append(message)
            camera_transport_failures.append(message)
        graph = health.get("camera_image_graph")
        if not isinstance(graph, dict) or graph.get("status") != "PASS":
            message = "runtime-health camera endpoint graph is not PASS"
            acceptance_failures.append(message)
            camera_transport_failures.append(message)
        transport_environment = _nested(health, "runtime", "transport_environment")
        if not isinstance(transport_environment, dict) or transport_environment.get("status") != "PASS":
            message = "runtime-health DDS environment is not PASS"
            acceptance_failures.append(message)
            camera_transport_failures.append(message)
        windows = health.get("windows")
        winning = sequence.get("winning_window_indexes", []) if isinstance(sequence, dict) else []
        if isinstance(windows, list):
            by_index = {
                item.get("index"): item for item in windows if isinstance(item, dict)
            }
            for index in winning:
                window = by_index.get(index)
                if not isinstance(window, dict) or window.get("status") != "PASS":
                    message = f"runtime-health winning window {index} is not PASS"
                    acceptance_failures.append(message)
                    camera_transport_failures.append(message)
                    continue
                if not _finite(_nested(window, "clock", "rtf")) or float(
                    _nested(window, "clock", "rtf")
                ) < MINIMUM_RUNTIME_RTF:
                    message = f"runtime-health window {index} RTF is below 0.9"
                    acceptance_failures.append(message)
                    camera_transport_failures.append(message)
                if not _finite(window.get("minimum_observed_camera_wall_rate_hz")) or float(
                    window.get("minimum_observed_camera_wall_rate_hz")
                ) < minimum_camera_rate_hz:
                    message = (
                        f"runtime-health window {index} camera rate is below "
                        f"{minimum_camera_rate_hz:g} Hz"
                    )
                    acceptance_failures.append(message)
                    camera_transport_failures.append(message)
                if not _finite(_nested(window, "bundles", "coverage_percent")) or float(
                    _nested(window, "bundles", "coverage_percent")
                ) < MINIMUM_CAMERA_BUNDLE_COVERAGE_PERCENT:
                    message = f"runtime-health window {index} bundle coverage is below 99%"
                    acceptance_failures.append(message)
                    camera_transport_failures.append(message)
                if not _finite(_nested(window, "bundles", "receipt_span_seconds", "p95")) or float(
                    _nested(window, "bundles", "receipt_span_seconds", "p95")
                ) > MAXIMUM_CAMERA_BUNDLE_RECEIPT_P95_SEC:
                    message = f"runtime-health window {index} bundle receipt p95 exceeds 40 ms"
                    acceptance_failures.append(message)
                    camera_transport_failures.append(message)
                if strict_10hz:
                    stamp_span = _nested(
                        window, "bundles", "stamp_span_seconds", "maximum"
                    )
                    source_integrity = window.get("camera_source_stamp_integrity")
                    source_topics = (
                        source_integrity.get("topics")
                        if isinstance(source_integrity, dict)
                        else None
                    )
                    absence = window.get("portable_runtime_absence")
                    output_publishers = (
                        absence.get("observed_output_publishers")
                        if isinstance(absence, dict)
                        else None
                    )
                    if (
                        not _finite(stamp_span)
                        or float(stamp_span)
                        > MAXIMUM_STRICT_CAMERA_BUNDLE_STAMP_SPAN_SEC
                    ):
                        message = (
                            f"runtime-health window {index} camera bundle source-stamp "
                            "span exceeds 5 us"
                        )
                        acceptance_failures.append(message)
                        camera_transport_failures.append(message)
                    if camera_profile_id == CAMERA_PROFILE_10HZ_STRICT_ID:
                        # HH_260906 - Bind each index to one full 8-second sample window.
                        matching_window_count = sum(
                            isinstance(item, dict) and item.get("index") == index
                            for item in windows
                        )
                        if (
                            matching_window_count != 1
                            or not _close(
                                _nested(window, "window", "duration_seconds"),
                                8.0,
                            )
                            or not isinstance(
                                _nested(window, "bundles", "complete_bundle_count"),
                                int,
                            )
                            or isinstance(
                                _nested(window, "bundles", "complete_bundle_count"),
                                bool,
                            )
                            or _nested(
                                window, "bundles", "complete_bundle_count"
                            )
                            < 70
                        ):
                            message = (
                                f"runtime-health window {index} is not one unique "
                                "8-second window with at least 70 complete bundles"
                            )
                            acceptance_failures.append(message)
                            camera_transport_failures.append(message)
                    invalid_source_topics = (
                        not isinstance(source_topics, dict)
                        or set(source_topics) != {
                            f"/sensing/camera/{camera}/camera_info"
                            for camera in (
                                "CAM_FRONT",
                                "CAM_BACK",
                                "CAM_FRONT_LEFT",
                                "CAM_BACK_LEFT",
                                "CAM_FRONT_RIGHT",
                                "CAM_BACK_RIGHT",
                            )
                        }
                        or any(
                            not isinstance(item, dict)
                            or item.get(
                                "strictly_increasing_unique_positive_arrival_stamps"
                            )
                            is not True
                            or item.get("zero_or_negative_window_stamp_count") != 0
                            or item.get(
                                "duplicate_positive_source_range_stamp_count"
                            )
                            != 0
                            or item.get(
                                "non_increasing_source_range_arrival_stamp_delta_count"
                            )
                            != 0
                            or not _finite(item.get("source_rate_hz_from_span"))
                            or not MINIMUM_STRICT_CAMERA_SOURCE_RATE_HZ
                            <= float(item.get("source_rate_hz_from_span", -1.0))
                            <= MAXIMUM_STRICT_CAMERA_SOURCE_RATE_HZ
                            or not _finite(item.get("maximum_source_gap_seconds"))
                            or float(item.get("maximum_source_gap_seconds", math.inf))
                            > MAXIMUM_STRICT_CAMERA_SOURCE_GAP_SEC + 1.0e-12
                            or (
                                camera_profile_id == CAMERA_PROFILE_10HZ_STRICT_ID
                                and (
                                    item.get("window_unmatched_record_count") != 0
                                    or item.get("window_matched_record_count")
                                    != item.get("window_record_count")
                                    or not _finite(
                                        item.get("minimum_source_gap_seconds")
                                    )
                                    or float(
                                        item.get(
                                            "minimum_source_gap_seconds",
                                            -math.inf,
                                        )
                                    )
                                    < MINIMUM_STRICT_CAMERA_SOURCE_GAP_SEC
                                    - 1.0e-12
                                )
                            )
                            for item in (
                                source_topics.values()
                                if isinstance(source_topics, dict)
                                else ()
                            )
                        )
                    )
                    if (
                        not isinstance(source_integrity, dict)
                        or source_integrity.get("status") != "PASS"
                        or source_integrity.get("record_count_parity") is not True
                        or source_integrity.get("all_records_used_exactly_once")
                        is not True
                        or (
                            camera_profile_id == CAMERA_PROFILE_10HZ_STRICT_ID
                            and source_integrity.get(
                                "all_window_records_used_exactly_once"
                            )
                            is not True
                        )
                        or invalid_source_topics
                        or not _finite(
                            window.get("maximum_observed_camera_wall_rate_hz")
                        )
                        or float(
                            window.get(
                                "maximum_observed_camera_wall_rate_hz", math.inf
                            )
                        )
                        > MAXIMUM_STRICT_CAMERA_WALL_RATE_HZ
                    ):
                        message = (
                            f"runtime-health window {index} lacks strict unique, "
                            "arrival-ordered, one-to-one 10 Hz source evidence"
                        )
                        acceptance_failures.append(message)
                        camera_transport_failures.append(message)
                    if (
                        not isinstance(absence, dict)
                        or absence.get("status") != "PASS"
                        or absence.get("required_absent_node") != PORTABLE_SHADOW_NODE
                        or absence.get("required_absent_output_topics")
                        != list(PORTABLE_OUTPUT_TOPICS)
                        or absence.get("observed_portable_nodes") != []
                        or not isinstance(output_publishers, dict)
                        or set(output_publishers) != set(PORTABLE_OUTPUT_TOPICS)
                        or any(output_publishers.values())
                        or (
                            camera_profile_id == CAMERA_PROFILE_10HZ_STRICT_ID
                            and (
                                absence.get("observation_scope")
                                != "window_end_graph_snapshot"
                                or absence.get("continuous_window_observation")
                                is not False
                            )
                        )
                    ):
                        message = (
                            f"runtime-health window {index} lacks observed Portable "
                            "node/output absence"
                        )
                        acceptance_failures.append(message)
                        camera_transport_failures.append(message)
        else:
            message = "runtime-health windows are missing"
            integrity_failures.append(message)
            camera_transport_failures.append(message)

    camera = evidence.get(camera_evidence_name, {}) if camera_evidence_name else {}
    if camera:
        contract = camera.get("contract")
        transport = camera.get("transport_provenance")
        strict_10hz = camera_profile_id in CAMERA_PROFILE_10HZ_STRICT_IDS
        if strict_10hz:
            boundary = camera.get("qualification_boundary")
            barrier = camera.get("barrier_contract")
            runtime_health_source = camera.get("runtime_health")
            expected_boundary = {
                "scope": "strict_six_camera_10hz_transport_runtime_only",
                "simulation_only": True,
                "vehicle_behavior_validated": False,
                "portable_model_loaded": False,
                "portable_model_60kph_validated": False,
                "portable_model_60kph_claim_allowed": False,
                "real_vehicle_ready": False,
            }
            expected_barrier = {
                "dispatch_policy": "exact_due_frame_barrier_fail_closed_v1",
                "delivery_contract_id": "strict10_exact_due_frame_fail_closed_v1",
                "scope": "strict_six_camera_10hz_transport_only",
                "frame_stride": 2,
                "expected_rgb_count": 6,
                "wait_timeout_sec": 0.25,
                "publish_deadline_sec": 0.25,
                "pending_frame_limit": 8,
            }
            runtime_health_path = attempt / "runtime_health.json"
            runtime_health_sha256 = (
                _sha256_file(runtime_health_path)
                if runtime_health_path.is_file()
                and not runtime_health_path.is_symlink()
                else None
            )
            if (
                camera.get("schema_version") != 1
                or camera.get("qualification_id")
                != (
                    "carla_six_camera_10hz_strict_transport_runtime_v2"
                    if camera_profile_id == CAMERA_PROFILE_10HZ_STRICT_ID
                    else "carla_six_camera_10hz_strict_transport_runtime_v1"
                )
                or not isinstance(contract, dict)
                or contract.get("profile_id") != camera_profile_id
                or contract.get("sensor_count") != 6
                or not _close(contract.get("source_frequency_hz"), 10.0)
                or not _close(contract.get("minimum_camera_rate_hz"), 9.5)
                or not _close(contract.get("maximum_camera_rate_hz"), 10.5)
                or not _close(contract.get("maximum_camera_stamp_gap_sec"), 0.100005)
                or (
                    camera_profile_id == CAMERA_PROFILE_10HZ_STRICT_ID
                    and not _close(
                        contract.get("minimum_camera_stamp_gap_sec"),
                        MINIMUM_STRICT_CAMERA_SOURCE_GAP_SEC,
                    )
                )
                or not _close(
                    contract.get("maximum_camera_bundle_stamp_span_sec"),
                    MAXIMUM_STRICT_CAMERA_BUNDLE_STAMP_SPAN_SEC,
                    tolerance=1.0e-12,
                )
                or contract.get("camera_image_publish_qos") != "best_effort"
                or contract.get("camera_image_publish_depth") != 1
                or contract.get("camera_info_publish_qos") != "reliable"
                or boundary != expected_boundary
                or barrier != expected_barrier
                or not isinstance(transport, dict)
                or not isinstance(runtime_health_source, dict)
                or Path(runtime_health_source.get("path", "")).resolve()
                != runtime_health_path.resolve()
                or runtime_health_source.get("sha256")
                != runtime_health_sha256
                or runtime_health_source.get("status") != "PASS"
                or camera.get("camera_delivery_contract_provenance")
                != camera_delivery_provenance_source
                or camera.get("runtime_parameters")
                != strict_runtime_parameters_source
            ):
                message = "post-run strict 10 Hz camera transport contract mismatch"
                integrity_failures.append(message)
                camera_transport_failures.append(message)
            portable_absence = camera.get("portable_runtime_absence")
            if (
                not isinstance(portable_absence, dict)
                or portable_absence.get("status") != "PASS"
                or portable_absence.get("observation_scope")
                != (
                    "winning_window_end_graph_snapshots"
                    if camera_profile_id == CAMERA_PROFILE_10HZ_STRICT_ID
                    else "winning_runtime_health_windows"
                )
                or (
                    camera_profile_id == CAMERA_PROFILE_10HZ_STRICT_ID
                    and portable_absence.get("continuous_window_observation")
                    is not False
                )
                or portable_absence.get("observed_window_indexes")
                != (
                    sequence.get("winning_window_indexes", [])
                    if isinstance(sequence, dict)
                    else []
                )
                or portable_absence.get("portable_shadow_node_observed") is not False
                or portable_absence.get("portable_output_publisher_observed") is not False
                or not isinstance(portable_absence.get("observations"), list)
                or len(portable_absence.get("observations", [])) != 3
            ):
                message = "post-run strict 10 Hz Portable absence proof mismatch"
                integrity_failures.append(message)
                camera_transport_failures.append(message)
        elif (
            camera.get("schema_version") != 1
            or not isinstance(contract, dict)
            or contract.get("profile_id") != CAMERA_PROFILE_5HZ_ID
            or contract.get("sensor_count") != 6
            or not _close(contract.get("source_frequency_hz"), 5.0)
            or contract.get("camera_image_publish_qos") != "best_effort"
            or contract.get("camera_image_publish_depth") != 1
            or contract.get("real_vehicle_ready") is not False
            or not isinstance(transport, dict)
        ):
            message = "post-run camera transport contract mismatch"
            integrity_failures.append(message)
            camera_transport_failures.append(message)
        health_transport = _nested(health, "contract", "camera_transport")
        if isinstance(transport, dict) and isinstance(health_transport, dict):
            for key in (
                "profile_id",
                "sensor_mapping_sha256",
                "vad_model_override_sha256",
                "cyclonedds_config_sha256",
                "rmw_implementation",
            ):
                if transport.get(key) != health_transport.get(key):
                    message = f"camera pre/post transport provenance differs for {key}"
                    integrity_failures.append(message)
                    camera_transport_failures.append(message)
        if camera.get("status") != "PASS":
            message = "post-run six-camera integrity is not PASS"
            acceptance_failures.append(message)
            camera_transport_failures.append(message)
        if not _finite(camera.get("bundle_coverage_percent")) or float(
            camera.get("bundle_coverage_percent", -1.0)
        ) < MINIMUM_CAMERA_BUNDLE_COVERAGE_PERCENT:
            message = "post-run camera bundle coverage is below 99%"
            acceptance_failures.append(message)
            camera_transport_failures.append(message)
        maximum_camera_gap = 0.100005 if strict_10hz else MAXIMUM_SPEED_SAMPLE_GAP_SEC
        if not _finite(camera.get("maximum_camera_stamp_gap_sec")) or float(
            camera.get("maximum_camera_stamp_gap_sec", math.inf)
        ) > maximum_camera_gap + (1.0e-12 if strict_10hz else 0.0):
            message = (
                "post-run camera stamp gap exceeds 0.100005 s"
                if strict_10hz
                else "post-run camera stamp gap exceeds 0.25 s"
            )
            acceptance_failures.append(message)
            camera_transport_failures.append(message)
        if strict_10hz:
            if camera_profile_id == CAMERA_PROFILE_10HZ_STRICT_ID:
                minimum_camera_gap = camera.get("minimum_camera_stamp_gap_sec")
                if (
                    not _finite(minimum_camera_gap)
                    or float(minimum_camera_gap)
                    < MINIMUM_STRICT_CAMERA_SOURCE_GAP_SEC - 1.0e-12
                ):
                    message = (
                        "post-run camera stamp gap is below 0.099995 s"
                    )
                    acceptance_failures.append(message)
                    camera_transport_failures.append(message)
            maximum_stamp_span = camera.get(
                "maximum_camera_bundle_stamp_span_sec"
            )
            if (
                not _finite(maximum_stamp_span)
                or float(maximum_stamp_span)
                > MAXIMUM_STRICT_CAMERA_BUNDLE_STAMP_SPAN_SEC
            ):
                message = "post-run camera bundle source-stamp span exceeds 5 us"
                acceptance_failures.append(message)
                camera_transport_failures.append(message)
            minimum_rate = camera.get("minimum_camera_stamp_rate_hz")
            maximum_rate = camera.get("maximum_camera_stamp_rate_hz")
            if (
                not _finite(minimum_rate)
                or float(minimum_rate) < 9.5
                or not _finite(maximum_rate)
                or float(maximum_rate) > 10.5
            ):
                message = "post-run camera cadence is outside [9.5, 10.5] Hz"
                acceptance_failures.append(message)
                camera_transport_failures.append(message)
        else:
            if not _finite(camera.get("candidate_front_acceptance_percent")) or float(
                camera.get("candidate_front_acceptance_percent", -1.0)
            ) < 99.0:
                message = "post-run candidate/front acceptance is below 99%"
                acceptance_failures.append(message)
                camera_transport_failures.append(message)
            if _nested(camera, "raw_six_image_queue_integrity", "status") != "PASS":
                message = "raw six-image queue integrity is not PASS"
                acceptance_failures.append(message)
                camera_transport_failures.append(message)

    runtime_load = evidence.get("runtime_load_analysis.json", {})
    if runtime_load:
        if camera_profile_id == CAMERA_PROFILE_10HZ_STRICT_ID:
            runtime_load_manifest_failures: list[str] = []
            _validate_runtime_load_input_manifest(
                runtime_load.get("input_manifest"),
                attempt,
                runtime_load_manifest_failures,
            )
            integrity_failures.extend(runtime_load_manifest_failures)
            camera_transport_failures.extend(runtime_load_manifest_failures)
        if (
            runtime_load.get("schema_version") != 3
            or runtime_load.get("status") != "complete"
            or runtime_load.get("problems") != []
        ):
            message = "runtime-load analysis is incomplete"
            integrity_failures.append(message)
            if camera_profile_id in CAMERA_PROFILE_10HZ_STRICT_IDS:
                camera_transport_failures.append(message)
        full_run = _nested(runtime_load, "vad_runtime", "phases", "full_run")
        if not isinstance(full_run, dict):
            message = "runtime-load full-run phase is missing"
            integrity_failures.append(message)
            if camera_profile_id in CAMERA_PROFILE_10HZ_STRICT_IDS:
                camera_transport_failures.append(message)
        else:
            if not _finite(full_run.get("aggregate_rtf")) or float(
                full_run.get("aggregate_rtf", -1.0)
            ) < MINIMUM_RUNTIME_RTF:
                acceptance_failures.append("full-run VAD RTF is below 0.9")
            if not _finite(full_run.get("wall_output_rate_hz")) or float(
                full_run.get("wall_output_rate_hz", -1.0)
            ) < MINIMUM_CAMERA_RATE_HZ:
                acceptance_failures.append("full-run VAD output rate is below 4 Hz")
        receipt_p95_ms = _nested(
            runtime_load, "camera_delivery", "phases", "full_run", "receipt_span_ms", "p95"
        )
        if not _finite(receipt_p95_ms) or float(receipt_p95_ms) > 40.0:
            message = "full-run six-camera receipt p95 exceeds 40 ms"
            acceptance_failures.append(message)
            camera_transport_failures.append(message)
        if camera_profile_id in CAMERA_PROFILE_10HZ_STRICT_IDS:
            camera_delivery = runtime_load.get("camera_delivery")
            source_rate_hz = (
                camera_delivery.get("source_rate_hz_from_median_period")
                if isinstance(camera_delivery, dict)
                else None
            )
            bundle_coverage = (
                camera_delivery.get("bundle_coverage_percent")
                if isinstance(camera_delivery, dict)
                else None
            )
            if (
                not _finite(source_rate_hz)
                or not 9.5 <= float(source_rate_hz) <= 10.5
            ):
                message = "full-run six-camera source cadence is outside [9.5, 10.5] Hz"
                acceptance_failures.append(message)
                camera_transport_failures.append(message)
            if camera_profile_id == CAMERA_PROFILE_10HZ_STRICT_ID:
                minimum_source_period = _nested(
                    camera_delivery, "source_period_sec", "min"
                )
                if (
                    not _finite(minimum_source_period)
                    or float(minimum_source_period)
                    < MINIMUM_STRICT_CAMERA_SOURCE_GAP_SEC - 1.0e-12
                ):
                    message = (
                        "full-run six-camera source period is below 0.099995 s"
                    )
                    acceptance_failures.append(message)
                    camera_transport_failures.append(message)
            if (
                not _finite(bundle_coverage)
                or float(bundle_coverage) < MINIMUM_CAMERA_BUNDLE_COVERAGE_PERCENT
            ):
                message = "full-run six-camera bundle coverage is below 99%"
                acceptance_failures.append(message)
                camera_transport_failures.append(message)
            if camera_profile_id == CAMERA_PROFILE_10HZ_STRICT_ID:
                edge_integrity = (
                    camera_delivery.get("edge_bounded_source_stamp_integrity")
                    if isinstance(camera_delivery, dict)
                    else None
                )
                if not _strict_v2_edge_bounded_camera_integrity_valid(
                    edge_integrity,
                    (
                        camera_delivery.get("matched_bundle_count")
                        if isinstance(camera_delivery, dict)
                        else None
                    ),
                ):
                    message = (
                        "full-run six-camera recorder edges or retained interior "
                        "one-to-one stamp proof are invalid"
                    )
                    acceptance_failures.append(message)
                    camera_transport_failures.append(message)
            else:
                stamp_integrity = (
                    camera_delivery.get("source_stamp_integrity")
                    if isinstance(camera_delivery, dict)
                    else None
                )
                topic_integrity = (
                    stamp_integrity.get("topics")
                    if isinstance(stamp_integrity, dict)
                    else None
                )
                invalid_topic_integrity = (
                    not isinstance(topic_integrity, dict)
                    or set(topic_integrity) != set(STRICT_CAMERA_INFO_TOPICS)
                    or any(
                        not isinstance(item, dict)
                        or item.get("strictly_increasing_unique_positive_stamps")
                        is not True
                        or item.get("zero_or_negative_stamp_count") != 0
                        or item.get("duplicate_positive_stamp_count") != 0
                        or item.get("non_increasing_positive_stamp_delta_count") != 0
                        or item.get("unused_record_count") != 0
                        for item in (
                            topic_integrity.values()
                            if isinstance(topic_integrity, dict)
                            else ()
                        )
                    )
                )
                if (
                    not isinstance(stamp_integrity, dict)
                    or stamp_integrity.get("status") != "PASS"
                    or stamp_integrity.get("record_count_parity") is not True
                    or stamp_integrity.get("all_records_used_exactly_once") is not True
                    or stamp_integrity.get("matched_bundle_count")
                    != camera_delivery.get("matched_bundle_count")
                    or invalid_topic_integrity
                ):
                    message = (
                        "full-run six-camera stamps are not strictly monotonic, unique, "
                        "count-parity, and one-to-one bundled"
                    )
                    acceptance_failures.append(message)
                    camera_transport_failures.append(message)

    speed = evidence.get("speed_profile.json", {})
    longitudinal = evidence.get("longitudinal_response.json", {})
    for label, document, analysis in (
        ("speed-profile", speed, "carla_speed_source_evidence"),
        ("longitudinal", longitudinal, "carla_longitudinal_response"),
    ):
        if document:
            inputs = document.get("inputs")
            if (
                document.get("schema_version") != 1
                or document.get("analysis") != analysis
                or document.get("status") != "complete"
                or not isinstance(inputs, dict)
                or inputs.get("profile_id") != PROFILE_ID
                or not _close(inputs.get("target_speed_mps"), TARGET_SPEED_MPS)
                or inputs.get("longitudinal_speed_source")
                != "explicit_simulation_nominal"
                or _nested(document, "quality", "problems") != []
            ):
                integrity_failures.append(f"{label} analysis contract is incomplete")
            identity = document.get("source_identity")
            if not isinstance(identity, dict):
                integrity_failures.append(f"{label} source identity is missing")
                continue
            recorded_identity_sha = identity.get("sha256")
            canonical = {key: value for key, value in identity.items() if key != "sha256"}
            if recorded_identity_sha != _sha256_json(canonical):
                integrity_failures.append(f"{label} source-identity digest mismatch")
            if (
                _resolved_recorded_path(_nested(identity, "effective_route", "path"), attempt)
                != aligned.resolve()
                or _nested(identity, "effective_route", "sha256") != aligned_sha
                or _resolved_recorded_path(_nested(identity, "route_result", "path"), attempt)
                != result_path.resolve()
                or _nested(identity, "route_result", "sha256") != result_sha
                or _nested(identity, "route_result", "success")
                is not result.get("success")
            ):
                integrity_failures.append(f"{label} route/result provenance mismatch")
            _validate_manifest(
                identity.get("rosbag"),
                (attempt / "bag").resolve(),
                integrity_failures,
                label,
            )
    if speed and longitudinal:
        for path in (
            ("effective_route", "sha256"),
            ("route_result", "sha256"),
            ("rosbag", "sha256"),
        ):
            if _nested(speed, "source_identity", *path) != _nested(
                longitudinal, "source_identity", *path
            ):
                integrity_failures.append(
                    f"speed/longitudinal provenance differs for {'/'.join(path)}"
                )
        bag_cross_check = _nested(
            longitudinal, "target_exposure", "route_result_cross_check"
        )
        if not isinstance(bag_cross_check, dict):
            integrity_failures.append("longitudinal bag/result speed cross-check is missing")
        else:
            if bag_cross_check.get("maximum_speed_consistent_with_bag") is not True:
                integrity_failures.append("bag and result maximum speed disagree")
            if (
                bag_cross_check.get("duration_consistent_within_one_continuity_gap")
                is not True
            ):
                integrity_failures.append("bag and result sustained-speed duration disagree")
            if bag_cross_check.get("minimum_duration_condition_met_from_bag") is not True:
                acceptance_failures.append("rosbag does not independently prove 15 m/s for 1.0 s")
        gated_acceleration = _nested(
            longitudinal, "summary", "gated_acceleration_mps2"
        )
        if not isinstance(gated_acceleration, dict):
            integrity_failures.append("gated longitudinal acceleration summary is missing")
        elif (
            not _finite(gated_acceleration.get("maximum"))
            or float(gated_acceleration.get("maximum", math.inf))
            > MAXIMUM_GATED_LONGITUDINAL_ACCELERATION_MPS2 + 1.0e-6
        ):
            acceptance_failures.append("gated positive acceleration exceeds 1.5 m/s^2")

    diagnosis = evidence.get("diagnosis.json", {})
    curvature_summary = None
    if diagnosis:
        curvature_summary = _nested(
            diagnosis, "metrics", "final_path", "snapshot_peak_curvature_per_m"
        )
        if (
            diagnosis.get("schema_version") != 2
            or _nested(diagnosis, "inputs", "town") != "Town06"
            or _nested(diagnosis, "inputs", "scenario") != "straight"
            or not isinstance(curvature_summary, dict)
            or not _finite(curvature_summary.get("p95_abs"))
            or not _finite(curvature_summary.get("max_abs"))
        ):
            integrity_failures.append("dynamic curvature diagnosis is incomplete")

    actuation_documents = [
        ("actuation_map_coverage.json", evidence.get("actuation_map_coverage.json", {})),
        (
            "actuation_map_runtime_coverage.json",
            evidence.get("actuation_map_runtime_coverage.json", {}),
        ),
    ]
    for name, coverage in actuation_documents:
        if not coverage:
            continue
        axis_minimum = coverage.get("map_velocity_axis_minimum_mps")
        axis_maximum = coverage.get("map_velocity_axis_maximum_mps")
        target_within_axis = (
            _finite(axis_minimum)
            and _finite(axis_maximum)
            and float(axis_minimum) <= TARGET_SPEED_MPS <= float(axis_maximum)
        )
        expected_coverage_status = "PASS" if target_within_axis else "EXPLORATORY"
        expected_envelope = (
            "TARGET_ENVELOPE_COVERED_BY_MAP_AXIS"
            if target_within_axis
            else "TARGET_ENVELOPE_EXCEEDS_MAP_AXIS_CLAMP_IF_REACHED"
        )
        boundary = coverage.get("validation_boundary")
        if (
            coverage.get("schema_version") != 1
            or coverage.get("analysis")
            != "raw_vehicle_command_converter_velocity_coverage"
            or coverage.get("profile_id") != PROFILE_ID
            or not _close(coverage.get("target_speed_mps"), TARGET_SPEED_MPS)
            or coverage.get("status") != expected_coverage_status
            or coverage.get("target_within_map_velocity_axis")
            is not target_within_axis
            or coverage.get("target_envelope_classification") != expected_envelope
            or coverage.get(
                "target_envelope_extension_authorized_for_exploratory_simulation"
            )
            is not (not target_within_axis)
            or not isinstance(boundary, dict)
            or boundary.get("simulation_only") is not True
            or boundary.get("map_coverage_above_axis") is not False
            or boundary.get("real_vehicle_ready") is not False
            or boundary.get("route_result_pass_is_high_speed_actuation_calibration")
            is not False
            or boundary.get("target_speed_is_converter_lookup_velocity") is not False
            or boundary.get("converter_lookup_velocity_source")
            != "absolute_current_odometry_longitudinal_speed_mps"
        ):
            integrity_failures.append(f"{name} contract mismatch")
    actuation_source = _validate_actuation_provenance(
        attempt,
        actuation_documents,
        integrity_failures,
    )
    preflight_coverage = evidence.get("actuation_map_coverage.json", {})
    if runtime_env:
        env_axis_maximum = _parsed_number(
            runtime_env.get("ACTUATION_MAP_VELOCITY_AXIS_MAXIMUM_MPS")
        )
        coverage_axis_maximum = preflight_coverage.get(
            "map_velocity_axis_maximum_mps"
        )
        if (
            runtime_env.get("ACTUATION_MAP_COVERAGE_STATUS")
            != preflight_coverage.get("status")
            or runtime_env.get("ACTUATION_MAP_TARGET_ENVELOPE_CLASSIFICATION")
            != preflight_coverage.get("target_envelope_classification")
            or not _finite(env_axis_maximum)
            or not _finite(coverage_axis_maximum)
            or not math.isclose(
                float(env_axis_maximum),
                float(coverage_axis_maximum),
                rel_tol=0.0,
                abs_tol=1.0e-9,
            )
            or runtime_env.get("ACTUATION_TARGET_WITHIN_MAP_VELOCITY_AXIS")
            != (
                "true"
                if preflight_coverage.get("target_within_map_velocity_axis") is True
                else "false"
            )
        ):
            integrity_failures.append("runtime.env actuation-map contract mismatch")
    runtime_coverage = evidence.get("actuation_map_runtime_coverage.json", {})
    if runtime_coverage:
        lookup = runtime_coverage.get("runtime_lookup_observation")
        if not isinstance(lookup, dict) or lookup.get("available") is not True:
            integrity_failures.append("actuation-map runtime lookup observation is missing")
        axis_max = runtime_coverage.get("map_velocity_axis_maximum_mps")
        observed_maximum = (
            lookup.get("maximum_absolute_current_speed_mps")
            if isinstance(lookup, dict)
            else None
        )
        observed_within_axis = (
            _finite(observed_maximum)
            and _finite(axis_max)
            and float(observed_maximum) <= float(axis_max)
        )
        expected_lookup_classification = (
            "OBSERVED_LOOKUPS_WITHIN_MAP_AXIS"
            if observed_within_axis
            else "OBSERVED_SPEED_REACHED_CLAMPED_MAP_REGION"
        )
        if (
            not _finite(observed_maximum)
            or not _finite(metrics.get("maximum_observed_speed_mps"))
            or not math.isclose(
                float(observed_maximum),
                float(metrics.get("maximum_observed_speed_mps")),
                rel_tol=0.0,
                abs_tol=1.0e-6,
            )
            or lookup.get("within_map_velocity_axis") is not observed_within_axis
            or lookup.get("velocity_axis_clamping_observed")
            is not (not observed_within_axis)
            or lookup.get("classification") != expected_lookup_classification
        ):
            integrity_failures.append(
                "actuation-map runtime observation does not match route-result speed"
            )
        if not _finite(axis_max) or float(axis_max) < TARGET_SPEED_MPS:
            readiness_blockers.append(
                "actuation-map velocity axis does not cover the 60 km/h target"
            )

    jerk = _series_jerk_observability(longitudinal)
    if not jerk["message_field_observable"]:
        readiness_blockers.append(
            "phase-aware measured longitudinal jerk acceptance is not yet observable/gated"
        )
    readiness_blockers.append(
        "this profile is simulation-only and is not real-vehicle calibration evidence"
    )
    if camera_profile_id in CAMERA_PROFILE_10HZ_STRICT_IDS:
        readiness_blockers.append(
            "strict 10 Hz qualification covers camera transport/runtime only; "
            "no Portable model was loaded or validated at 60 km/h"
        )
    if camera_profile_id == CAMERA_PROFILE_10HZ_STRICT_LEGACY_ID:
        readiness_blockers.append(
            "legacy strict v1 camera provenance validation checks recorded "
            "declaration consistency only; archived source bytes are unavailable"
        )

    runtime_env_source = _file_identity(
        runtime_env_path, integrity_failures, "runtime.env provenance"
    )
    camera_evidence_source = {
        "path": str((attempt / camera_evidence_name).resolve())
        if camera_evidence_name
        else None,
        "sha256": (
            _sha256_file(attempt / camera_evidence_name)
            if camera_evidence_name
            and (attempt / camera_evidence_name).is_file()
            and not (attempt / camera_evidence_name).is_symlink()
            else None
        ),
    }
    camera_transport_qualification_status = (
        "PASS"
        if camera_profile_id in CAMERA_EVIDENCE_BY_PROFILE
        and bool(camera)
        and not camera_transport_failures
        else "FAILED"
    )
    integrity_status = "PASS" if not integrity_failures else "FAILED"
    acceptance_status = (
        "PASS"
        if integrity_status == "PASS" and not acceptance_failures
        else "FAILED"
    )
    maximum_speed = metrics.get("maximum_observed_speed_mps")
    sustained = metrics.get("maximum_sustained_speed_duration_sec")
    sim_elapsed = metrics.get("sim_elapsed_sec")
    wall_elapsed = metrics.get("wall_elapsed_sec")
    route_rtf = (
        float(sim_elapsed) / float(wall_elapsed)
        if _finite(sim_elapsed) and _finite(wall_elapsed) and float(wall_elapsed) > 0.0
        else None
    )
    return {
        "schema_version": 1,
        "gate_id": "carla_vad_60kph_pilot_acceptance_v1",
        "status": acceptance_status,
        "evidence_integrity_status": integrity_status,
        "simulation_pilot_acceptance_status": acceptance_status,
        "physical_goal_completion_status": physical_goal_completion_status,
        "speed_exposure_contract_status": speed_exposure_contract_status,
        "full_stack_route_test_status": full_stack_route_test_status,
        "camera_transport_qualification_status": (
            camera_transport_qualification_status
        ),
        "real_vehicle_readiness_status": "BLOCKED",
        "real_vehicle_ready": False,
        "evaluated_at": datetime.now(timezone.utc).isoformat(),
        "profile_id": PROFILE_ID,
        "contract": {
            "target_speed_mps": TARGET_SPEED_MPS,
            "minimum_sustained_speed_mps": MINIMUM_SUSTAINED_SPEED_MPS,
            "minimum_sustained_speed_sec": MINIMUM_SUSTAINED_SPEED_SEC,
            "maximum_observed_speed_mps": MAXIMUM_OBSERVED_SPEED_MPS,
            "maximum_lateral_acceleration_mps2": (
                MAXIMUM_LATERAL_ACCELERATION_MPS2
            ),
            "maximum_gated_longitudinal_acceleration_mps2": (
                MAXIMUM_GATED_LONGITUDINAL_ACCELERATION_MPS2
            ),
            "minimum_runtime_rtf": MINIMUM_RUNTIME_RTF,
            "minimum_camera_rate_hz": (
                9.5
                if camera_profile_id in CAMERA_PROFILE_10HZ_STRICT_IDS
                else MINIMUM_CAMERA_RATE_HZ
            ),
            "minimum_camera_bundle_coverage_percent": (
                MINIMUM_CAMERA_BUNDLE_COVERAGE_PERCENT
            ),
            "maximum_camera_bundle_receipt_p95_sec": (
                MAXIMUM_CAMERA_BUNDLE_RECEIPT_P95_SEC
            ),
            "maximum_camera_bundle_stamp_span_sec": (
                MAXIMUM_STRICT_CAMERA_BUNDLE_STAMP_SPAN_SEC
                if camera_profile_id in CAMERA_PROFILE_10HZ_STRICT_IDS
                else None
            ),
        },
        "sources": {
            "attempt_dir": str(attempt),
            "source_route": {
                "path": str(source),
                "sha256": source_sha,
            },
            "aligned_route": {
                "path": str(aligned.resolve()),
                "sha256": aligned_sha,
            },
            "route_result": {
                "path": str(result_path.resolve()),
                "sha256": result_sha,
            },
            "runtime_env": runtime_env_source,
            "camera_transport_qualification": camera_evidence_source,
            "camera_delivery_contract_provenance": (
                camera_delivery_provenance_source
            ),
            "legacy_camera_provenance_validation_scope": (
                {
                    "status": "DECLARATION_CONSISTENCY_ONLY",
                    "offline_legacy_gate_semantics": True,
                    "archived_source_bytes_available": False,
                    "current_source_bytes_rehashed": False,
                    "byte_identical_runtime_replay_claim_allowed": False,
                }
                if camera_profile_id == CAMERA_PROFILE_10HZ_STRICT_LEGACY_ID
                else None
            ),
            "strict_camera_runtime_parameters": (
                strict_runtime_parameters_source
            ),
            "strict_recorder_measurement": strict_recorder_measurement_source,
            "carla_lifecycle": lifecycle_sources,
            "geometry_parameter_dump": (
                geometry_parameter_source if geometry_contract_present else None
            ),
            "trajectory_code_provenance": trajectory_source,
            "actuation_config_provenance": actuation_source,
        },
        "geometry_variant": geometry_variant,
        "camera_transport_qualification": {
            "status": camera_transport_qualification_status,
            "profile_id": camera_profile_id,
            "evidence_file": camera_evidence_name,
            "scope": (
                "strict_six_camera_10hz_transport_runtime_only"
                if camera_profile_id in CAMERA_PROFILE_10HZ_STRICT_IDS
                else "six_camera_5hz_transport_and_vad_runtime"
            ),
            "simulation_only": True,
            "vehicle_behavior_validated": False,
            "portable_model_loaded": False,
            "portable_model_60kph_validated": False,
            "portable_model_60kph_claim_allowed": False,
            "real_vehicle_ready": False,
            "failures": list(dict.fromkeys(camera_transport_failures)),
        },
        "integrity_failures": integrity_failures,
        "acceptance_failures": acceptance_failures,
        "readiness_blockers": list(dict.fromkeys(readiness_blockers)),
        "summary": {
            "goal_reached": _nested(result, "final", "goal_reached"),
            "route_status": _nested(result, "final", "route_status"),
            "route_result_success": result.get("success"),
            "speed_exposure_status": exposure.get("status"),
            "physical_goal_completion_status": physical_goal_completion_status,
            "speed_exposure_contract_status": speed_exposure_contract_status,
            "full_stack_route_test_status": full_stack_route_test_status,
            "maximum_observed_speed_mps": maximum_speed,
            "maximum_sustained_speed_duration_sec": sustained,
            "maximum_lateral_acceleration_mps2": metrics.get(
                "maximum_lateral_acceleration_mps2"
            ),
            "route_real_time_factor": route_rtf,
            "pre_engagement_runtime_health_status": health.get("status"),
            "post_run_camera_status": camera.get("status"),
            "camera_transport_profile_id": camera_profile_id,
            "camera_transport_qualification_status": (
                camera_transport_qualification_status
            ),
            "full_run_vad_rtf": _nested(
                runtime_load, "vad_runtime", "phases", "full_run", "aggregate_rtf"
            ),
            "full_run_vad_output_rate_hz": _nested(
                runtime_load, "vad_runtime", "phases", "full_run", "wall_output_rate_hz"
            ),
            "final_path_snapshot_peak_curvature_per_m": curvature_summary,
            "jerk_observability": jerk,
        },
    }


def atomic_write_json(path: Path, payload: dict[str, Any]) -> None:
    target = path.expanduser().resolve()
    target.parent.mkdir(parents=True, exist_ok=True)
    descriptor, staged_name = tempfile.mkstemp(
        prefix=f".{target.name}.", suffix=".staged", dir=target.parent
    )
    staged = Path(staged_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(staged, target)
    finally:
        staged.unlink(missing_ok=True)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--attempt-dir", required=True, type=Path)
    parser.add_argument("--source-route", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    return parser.parse_args(argv)


def run(
    args: argparse.Namespace,
    evaluator: Callable[[Path, Path], dict[str, Any]] = evaluate_trial,
) -> int:
    try:
        payload = evaluator(args.attempt_dir, args.source_route)
    except Exception as error:  # Always preserve machine-readable failure evidence.
        payload = {
            "schema_version": 1,
            "gate_id": "carla_vad_60kph_pilot_acceptance_v1",
            "status": "FAILED",
            "evidence_integrity_status": "FAILED",
            "simulation_pilot_acceptance_status": "FAILED",
            "physical_goal_completion_status": "UNKNOWN",
            "speed_exposure_contract_status": "UNKNOWN",
            "full_stack_route_test_status": "UNKNOWN",
            "real_vehicle_readiness_status": "BLOCKED",
            "real_vehicle_ready": False,
            "evaluated_at": datetime.now(timezone.utc).isoformat(),
            "profile_id": PROFILE_ID,
            "integrity_failures": [f"{type(error).__name__}: {error}"],
            "acceptance_failures": [],
            "readiness_blockers": [
                "validation crashed before the evidence contract could be established"
            ],
        }
    atomic_write_json(args.output, payload)
    print(
        "60 kph pilot gate: "
        f"integrity={payload['evidence_integrity_status']} "
        f"acceptance={payload['simulation_pilot_acceptance_status']}"
    )
    return 0 if payload.get("simulation_pilot_acceptance_status") == "PASS" else 1


def main(argv: Sequence[str] | None = None) -> int:
    return run(parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
