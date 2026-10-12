#!/usr/bin/env python3
"""Inspect the official legacy Bench2Drive Mini archives without extracting.

The structural result and the temporal qualification are deliberately
separate.  A structurally valid archive can pass while remaining unsuitable
for the repository's native-timestamp Common-10Hz contract.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import gzip
import json
import math
import os
from pathlib import Path
import re
import stat
import statistics
import struct
import sys
import tarfile
from typing import Any, BinaryIO, Mapping, Sequence

try:
    from scripts.e2e.verify_bench2drive_mini import ArchiveSpec
    from scripts.e2e.verify_bench2drive_mini import OFFICIAL_LEGACY_MINI_ARCHIVES
    from scripts.e2e.verify_tar_archive import AuditLimits
    from scripts.e2e.verify_tar_archive import verify_tar_archive
except ModuleNotFoundError as error:
    # ``python scripts/e2e/this_file.py`` places only this directory on
    # sys.path, while module imports/tests place the repository root there.
    if error.name not in {"scripts", "scripts.e2e"}:
        raise
    from verify_bench2drive_mini import ArchiveSpec
    from verify_bench2drive_mini import OFFICIAL_LEGACY_MINI_ARCHIVES
    from verify_tar_archive import AuditLimits
    from verify_tar_archive import verify_tar_archive


SCHEMA_VERSION = 1
MAX_ERRORS = 100
MAX_TARGET_MEMBER_BYTES = 32 * 1024 * 1024
MAX_ANNOTATION_BYTES = 32 * 1024 * 1024
MAX_JPEG_HEADER_BYTES = 256 * 1024
MAX_FRAME_COUNT = 100_000
MAX_JSON_SCAN_NODES = 2_000_000
MAX_JSON_PATH_CHARACTERS = 512
MAX_NONFINITE_SAMPLES_PER_ANNOTATION = 10
MAX_NONFINITE_SAMPLES_PER_ARCHIVE = 50
MAX_NONFINITE_SAMPLES_PER_SET = 100

SURROUND_RGB_DIRECTORIES: Mapping[str, str] = {
    "rgb_front": "CAM_FRONT",
    "rgb_front_left": "CAM_FRONT_LEFT",
    "rgb_front_right": "CAM_FRONT_RIGHT",
    "rgb_back": "CAM_BACK",
    "rgb_back_left": "CAM_BACK_LEFT",
    "rgb_back_right": "CAM_BACK_RIGHT",
}
AUXILIARY_RGB_DIRECTORIES: Mapping[str, str] = {
    "rgb_top_down": "TOP_DOWN",
}
ALL_RGB_DIRECTORIES = {
    **SURROUND_RGB_DIRECTORIES,
    **AUXILIARY_RGB_DIRECTORIES,
}

REQUIRED_ANNOTATION_KEYS = {
    "acceleration",
    "angular_velocity",
    "bounding_boxes",
    "brake",
    "command_far",
    "command_near",
    "next_command",
    "only_ap_brake",
    "reverse",
    "sensors",
    "should_brake",
    "speed",
    "steer",
    "theta",
    "throttle",
    "weather",
    "x",
    "x_command_far",
    "x_command_near",
    "x_target",
    "y",
    "y_command_far",
    "y_command_near",
    "y_target",
}
REQUIRED_CAMERA_CALIBRATION_KEYS = {
    "cam2ego",
    "fov",
    "image_size_x",
    "image_size_y",
    "intrinsic",
    "location",
    "rotation",
    "world2cam",
}

FRAME_FILE_RE = re.compile(r"^(?P<frame>[0-9]{5})\.(?P<suffix>jpg|json\.gz)$")
SCENE_RE = re.compile(
    r"^(?P<scenario>.+)_(?P<town>Town[^_]+)_Route(?P<route>[0-9]+)"
    r"_Weather(?P<weather>[0-9]+)$"
)

# The upstream statement is informative.  Qualification below still requires
# timestamp evidence carried by the archive itself.
UPSTREAM_NOMINAL_10HZ_SOURCE = (
    "https://github.com/Thinklab-SJTU/Bench2Drive-Jittor/blob/uniad/vad/"
    "docs/CONVERT_GUIDE.md"
)

# CARLA leaderboard RoadOption integer convention used by Bench2Drive.  These
# values are only hints and never replace trajectory-geometry analysis.
COMMAND_HINTS = {
    1: "left",
    2: "right",
    3: "straight",
    4: "lane_follow",
    5: "lane_change_left",
    6: "lane_change_right",
}
COMMAND_CONVENTION_SOURCE = (
    "https://github.com/carla-simulator/carla/blob/ue5-dev/PythonAPI/carla/"
    "agents/navigation/local_planner.py"
)


class InspectionFailure(Exception):
    """A bounded, reportable inspection failure."""


class _NonFiniteJSONConstant:
    """Marker retained while locating non-standard JSON numeric constants."""

    __slots__ = ("token",)

    def __init__(self, token: str) -> None:
        self.token = token


class Errors:
    """Retain a bounded number of errors while counting every failure."""

    def __init__(self) -> None:
        self.items: list[str] = []
        self.total = 0

    def add(self, message: str) -> None:
        self.total += 1
        if len(self.items) < MAX_ERRORS:
            self.items.append(message)


def _absolute_path(value: Path | str) -> Path:
    return Path(os.path.abspath(os.fspath(Path(value).expanduser())))


def _identity(metadata: os.stat_result) -> tuple[int, int, int, int, int]:
    return (
        metadata.st_dev,
        metadata.st_ino,
        metadata.st_size,
        metadata.st_mtime_ns,
        metadata.st_ctime_ns,
    )


def _identity_report(metadata: os.stat_result) -> dict[str, int]:
    return {
        "device": metadata.st_dev,
        "inode": metadata.st_ino,
        "size_bytes": metadata.st_size,
        "mtime_ns": metadata.st_mtime_ns,
        "ctime_ns": metadata.st_ctime_ns,
    }


def _scene_root(archive_name: str) -> str:
    suffix = ".tar.gz"
    if not archive_name.endswith(suffix):
        raise InspectionFailure(f"archive name is not a .tar.gz file: {archive_name!r}")
    root = archive_name[: -len(suffix)]
    if not root or root != Path(root).name:
        raise InspectionFailure(f"archive name has an unsafe scene root: {archive_name!r}")
    return root


def _normalise_member_name(name: str) -> str:
    while name.startswith("./"):
        name = name[2:]
    return name.rstrip("/")


def _jpeg_dimensions(payload: bytes) -> tuple[int, int]:
    """Return JPEG width and height using only marker headers."""
    if len(payload) < 4 or payload[:2] != b"\xff\xd8":
        raise InspectionFailure("JPEG does not start with an SOI marker")
    start_of_frame = {
        0xC0,
        0xC1,
        0xC2,
        0xC3,
        0xC5,
        0xC6,
        0xC7,
        0xC9,
        0xCA,
        0xCB,
        0xCD,
        0xCE,
        0xCF,
    }
    offset = 2
    while offset < len(payload):
        if payload[offset] != 0xFF:
            offset += 1
            continue
        while offset < len(payload) and payload[offset] == 0xFF:
            offset += 1
        if offset >= len(payload):
            break
        marker = payload[offset]
        offset += 1
        if marker in {0x00, 0x01, 0xD8, 0xD9, *range(0xD0, 0xD8)}:
            continue
        if offset + 2 > len(payload):
            break
        segment_bytes = struct.unpack(">H", payload[offset : offset + 2])[0]
        if segment_bytes < 2:
            raise InspectionFailure("JPEG contains an invalid marker length")
        if offset + segment_bytes > len(payload):
            break
        if marker in start_of_frame:
            if segment_bytes < 7:
                raise InspectionFailure("JPEG SOF marker is too short")
            height, width = struct.unpack(">HH", payload[offset + 3 : offset + 7])
            if width <= 0 or height <= 0:
                raise InspectionFailure("JPEG SOF declares a non-positive dimension")
            return width, height
        offset += segment_bytes
    raise InspectionFailure(
        f"JPEG dimensions were not found in the first {len(payload)} bytes"
    )


def _read_member_prefix(member: BinaryIO, member_size: int) -> bytes:
    if member_size <= 0:
        raise InspectionFailure("JPEG member is empty")
    if member_size > MAX_TARGET_MEMBER_BYTES:
        raise InspectionFailure(
            f"JPEG member size {member_size} exceeds limit {MAX_TARGET_MEMBER_BYTES}"
        )
    return member.read(min(member_size, MAX_JPEG_HEADER_BYTES))


def _read_annotation(
    member: BinaryIO,
    member_size: int,
) -> tuple[Mapping[str, Any], dict[str, Any]]:
    if member_size <= 0:
        raise InspectionFailure("annotation member is empty")
    if member_size > MAX_TARGET_MEMBER_BYTES:
        raise InspectionFailure(
            f"compressed annotation size {member_size} exceeds limit "
            f"{MAX_TARGET_MEMBER_BYTES}"
        )
    try:
        with gzip.GzipFile(fileobj=member, mode="rb") as stream:
            payload = stream.read(MAX_ANNOTATION_BYTES + 1)
            if len(payload) > MAX_ANNOTATION_BYTES:
                raise InspectionFailure(
                    f"annotation expands beyond limit {MAX_ANNOTATION_BYTES}"
                )
            # Force gzip EOF/CRC processing even if the first bounded read was exact.
            if stream.read(1):
                raise InspectionFailure(
                    f"annotation expands beyond limit {MAX_ANNOTATION_BYTES}"
                )
        value = json.loads(
            payload.decode("utf-8"),
            object_pairs_hook=_unique_json_object,
            parse_constant=_retain_nonfinite_json_constant,
        )
    except InspectionFailure:
        raise
    except (
        EOFError,
        MemoryError,
        OSError,
        OverflowError,
        RecursionError,
        UnicodeDecodeError,
        ValueError,
        gzip.BadGzipFile,
    ) as error:
        raise InspectionFailure(f"annotation gzip/JSON is invalid: {error}") from error
    if not isinstance(value, Mapping):
        raise InspectionFailure("annotation JSON root is not an object")
    return value, _scan_nonfinite_json(value)


def _unique_json_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    value: dict[str, Any] = {}
    for key, item in pairs:
        if key in value:
            raise InspectionFailure(f"annotation JSON repeats object key {key!r}")
        value[key] = item
    return value


def _retain_nonfinite_json_constant(token: str) -> _NonFiniteJSONConstant:
    if token not in {"NaN", "Infinity", "-Infinity"}:
        raise InspectionFailure(f"unknown non-standard JSON constant {token!r}")
    return _NonFiniteJSONConstant(token)


def _bounded_json_pointer(parent: str, component: Any) -> str:
    escaped = str(component).replace("~", "~0").replace("/", "~1")
    candidate = f"{parent}/{escaped}"
    if len(candidate) <= MAX_JSON_PATH_CHARACTERS:
        return candidate
    return candidate[: MAX_JSON_PATH_CHARACTERS - 3] + "..."


def _scan_nonfinite_json(value: Any) -> dict[str, Any]:
    """Count non-standard JSON constants and retain bounded JSON-pointer samples."""
    stack: list[tuple[str, Any]] = [("$", value)]
    count = 0
    tokens: Counter[str] = Counter()
    samples: list[dict[str, str]] = []
    visited_nodes = 0
    while stack:
        path, item = stack.pop()
        visited_nodes += 1
        if visited_nodes > MAX_JSON_SCAN_NODES:
            raise InspectionFailure(
                f"annotation JSON node count exceeds limit {MAX_JSON_SCAN_NODES}"
            )
        if isinstance(item, _NonFiniteJSONConstant):
            count += 1
            tokens[item.token] += 1
            if len(samples) < MAX_NONFINITE_SAMPLES_PER_ANNOTATION:
                samples.append({"json_pointer": path, "token": item.token})
            continue
        if isinstance(item, Mapping):
            for key, child in reversed(list(item.items())):
                stack.append((_bounded_json_pointer(path, key), child))
        elif isinstance(item, list):
            for index in range(len(item) - 1, -1, -1):
                stack.append((_bounded_json_pointer(path, index), item[index]))
    return {
        "count": count,
        "by_token": dict(sorted(tokens.items())),
        "samples": samples,
        "samples_truncated": count > len(samples),
        "visited_json_nodes": visited_nodes,
        "maximum_json_nodes": MAX_JSON_SCAN_NODES,
    }


def _is_number(value: Any) -> bool:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    try:
        return math.isfinite(float(value))
    except (OverflowError, ValueError):
        return False


def _matrix_has_shape(value: Any, rows: int, columns: int) -> bool:
    return (
        isinstance(value, list)
        and len(value) == rows
        and all(
            isinstance(row, list)
            and len(row) == columns
            and all(_is_number(item) for item in row)
            for row in value
        )
    )


def _vector_has_length(value: Any, length: int) -> bool:
    return (
        isinstance(value, list)
        and len(value) == length
        and all(_is_number(item) for item in value)
    )


def _timestamp_keys(annotation: Mapping[str, Any]) -> set[str]:
    found: set[str] = set()
    exact = {
        "elapsed_seconds",
        "frame_time",
        "game_timestamp",
        "simulation_time",
        "timestamp",
        "timestamp_ms",
        "timestamp_ns",
        "timestamp_s",
        "timestamp_us",
    }
    for key in annotation:
        lowered = str(key).casefold()
        if (
            lowered in exact
            or lowered.endswith("_timestamp")
            or lowered.startswith("timestamp_")
        ):
            found.add(str(key))
    return found


def _validate_calibration(
    annotation: Mapping[str, Any],
    frame_id: str,
    calibration_dimensions: dict[str, set[tuple[int, int]]],
    errors: Errors,
) -> None:
    sensors = annotation.get("sensors")
    if not isinstance(sensors, Mapping):
        errors.add(f"annotation {frame_id} has no sensor calibration object")
        return
    for rgb_directory, sensor_name in ALL_RGB_DIRECTORIES.items():
        calibration = sensors.get(sensor_name)
        if not isinstance(calibration, Mapping):
            errors.add(
                f"annotation {frame_id} lacks calibration for {sensor_name} "
                f"({rgb_directory})"
            )
            continue
        missing = sorted(REQUIRED_CAMERA_CALIBRATION_KEYS - set(calibration))
        if missing:
            errors.add(
                f"annotation {frame_id} calibration {sensor_name} misses {missing}"
            )
            continue
        if not _matrix_has_shape(calibration["intrinsic"], 3, 3):
            errors.add(
                f"annotation {frame_id} calibration {sensor_name} has invalid intrinsic"
            )
        if not _matrix_has_shape(calibration["cam2ego"], 4, 4):
            errors.add(
                f"annotation {frame_id} calibration {sensor_name} has invalid cam2ego"
            )
        if not _matrix_has_shape(calibration["world2cam"], 4, 4):
            errors.add(
                f"annotation {frame_id} calibration {sensor_name} has invalid world2cam"
            )
        if not _vector_has_length(calibration["location"], 3):
            errors.add(
                f"annotation {frame_id} calibration {sensor_name} has invalid location"
            )
        if not _vector_has_length(calibration["rotation"], 3):
            errors.add(
                f"annotation {frame_id} calibration {sensor_name} has invalid rotation"
            )
        width = calibration["image_size_x"]
        height = calibration["image_size_y"]
        if not _is_number(width) or not _is_number(height):
            errors.add(
                f"annotation {frame_id} calibration {sensor_name} has invalid image size"
            )
        elif int(width) != width or int(height) != height or width <= 0 or height <= 0:
            errors.add(
                f"annotation {frame_id} calibration {sensor_name} has invalid image size"
            )
        else:
            calibration_dimensions[rgb_directory].add((int(width), int(height)))
        if not _is_number(calibration["fov"]):
            errors.add(f"annotation {frame_id} calibration {sensor_name} has invalid fov")


def _continuous_frame_report(frame_ids: set[str]) -> dict[str, Any]:
    if not frame_ids:
        return {
            "count": 0,
            "first": None,
            "last": None,
            "continuous_from_zero": False,
            "missing": [],
        }
    numeric = sorted(int(value) for value in frame_ids)
    expected = set(range(numeric[-1] + 1))
    missing = sorted(expected - set(numeric))
    return {
        "count": len(numeric),
        "first": f"{numeric[0]:05d}",
        "last": f"{numeric[-1]:05d}",
        "continuous_from_zero": numeric[0] == 0 and not missing,
        "missing": [f"{value:05d}" for value in missing[:MAX_ERRORS]],
        "missing_count": len(missing),
    }


def _timestamp_unit(field: str) -> float | None:
    lowered = field.casefold()
    if lowered.endswith("_ns"):
        return 1e-9
    if lowered.endswith("_us"):
        return 1e-6
    if lowered.endswith("_ms"):
        return 1e-3
    if lowered.endswith("_s") or lowered in {
        "elapsed_seconds",
        "frame_time",
        "simulation_time",
    }:
        return 1.0
    return None


def _native_timestamp_cadence_evidence(
    frame_ids: set[str],
    timestamp_fields_by_frame: Mapping[str, set[str]],
    timestamp_values: Mapping[str, Mapping[str, Any]],
) -> dict[str, Any]:
    field_sets = list(timestamp_fields_by_frame.values())
    all_fields = sorted(set().union(*field_sets)) if field_sets else []
    common_fields = sorted(set.intersection(*field_sets)) if field_sets else []
    report: dict[str, Any] = {
        "status": "NOT_PROVEN",
        "scope": "native annotation timestamp cadence and duration only",
        "full_common_10hz_qualification": False,
        "official_nominal_frequency_hz": 10.0,
        "official_claim_source": UPSTREAM_NOMINAL_10HZ_SOURCE,
        "official_claim_is_native_timestamp_evidence": False,
        "recognized_timestamp_scope": "top-level annotation keys only",
        "native_timestamp_fields_union": all_fields,
        "native_timestamp_fields_common_to_all_frames": common_fields,
        "native_timestamp_availability": "NONE" if not all_fields else "PRESENT",
        "evaluated_explicit_unit_fields": [],
        "reason": "RECOGNIZED_TOP_LEVEL_NATIVE_TIMESTAMPS_NOT_PRESENT",
    }
    if not frame_ids or not all_fields:
        return report
    if not common_fields or any(
        timestamp_fields_by_frame.get(frame_id, set()) != set(common_fields)
        for frame_id in frame_ids
    ):
        report["native_timestamp_availability"] = "PARTIAL_OR_INCONSISTENT"
        report["reason"] = "NATIVE_TIMESTAMP_FIELDS_ARE_PARTIAL_OR_INCONSISTENT"
        return report

    ordered_frames = sorted(frame_ids)
    for field in common_fields:
        multiplier = _timestamp_unit(field)
        if multiplier is None:
            continue
        raw_values = [timestamp_values[frame_id].get(field) for frame_id in ordered_frames]
        if not all(_is_number(value) for value in raw_values):
            continue
        try:
            seconds = [float(value) * multiplier for value in raw_values]
        except (OverflowError, ValueError):
            continue
        intervals = [right - left for left, right in zip(seconds, seconds[1:])]
        if not intervals or any(
            not math.isfinite(interval) or interval <= 0.0 for interval in intervals
        ):
            continue
        ordered_intervals = sorted(intervals)
        duration_s = seconds[-1] - seconds[0]
        effective_rate_hz = (
            (len(seconds) - 1) / duration_s if duration_s > 0.0 else 0.0
        )
        median_gap_ms = statistics.median(intervals) * 1000.0
        p99_index = max(0, math.ceil(0.99 * len(ordered_intervals)) - 1)
        p99_gap_ms = ordered_intervals[p99_index] * 1000.0
        maximum_gap_ms = ordered_intervals[-1] * 1000.0
        field_report = {
            "candidate_field": field,
            "duration_s": duration_s,
            "effective_rate_hz": effective_rate_hz,
            "median_gap_ms": median_gap_ms,
            "p99_gap_ms": p99_gap_ms,
            "maximum_gap_ms": maximum_gap_ms,
            "interval_seconds_min": ordered_intervals[0],
            "interval_seconds_max": ordered_intervals[-1],
        }
        report["evaluated_explicit_unit_fields"].append(field_report)
        cadence_pass = (
            duration_s >= 30.0
            and effective_rate_hz >= 9.5
            and 80.0 <= median_gap_ms <= 120.0
            and p99_gap_ms <= 150.0
            and maximum_gap_ms <= 250.0
        )
        if cadence_pass:
            report.update(
                {
                    "status": "PASS",
                    "native_timestamp_availability": "PRESENT_ALL_FRAMES",
                    "evidence_field": field,
                    "passing_explicit_unit_field": field_report,
                    "reason": "NATIVE_TIMESTAMP_CADENCE_AND_DURATION_PASS",
                }
            )
            return report
    report["native_timestamp_availability"] = "PRESENT_ALL_FRAMES"
    report["reason"] = "NO_EXPLICIT_UNIT_FIELD_PASSES_CADENCE_AND_DURATION_GATES"
    return report


def _maneuver_hint(scene_root: str, command_values: set[int]) -> dict[str, Any]:
    lowered = scene_root.casefold()
    decoded = [COMMAND_HINTS.get(value, f"unknown_{value}") for value in sorted(command_values)]
    if "turn" in lowered or command_values.intersection({1, 2}):
        value = "turn"
    elif command_values == {3}:
        value = "straight"
    elif command_values == {4}:
        value = "lane_follow_only"
    elif command_values.intersection({5, 6}):
        value = "lane_change_or_mixed"
    else:
        value = "unknown_or_mixed"
    return {
        "value": value,
        "basis": "scene filename tokens and raw command integers only",
        "command_values": sorted(command_values),
        "decoded_command_hints": decoded,
        "command_integer_convention_source": COMMAND_CONVENTION_SOURCE,
        "caveat": (
            "This is not trajectory-geometry evidence; lane_follow may include curves, "
            "and route labels do not prove the driven maneuver."
        ),
    }


def _conversion_readiness(
    *,
    structural_error_count: int,
    native_cadence_status: str | None,
    nonfinite_constant_count: int,
) -> dict[str, Any]:
    reasons: list[str] = []
    required_actions: list[str] = []
    if structural_error_count:
        reasons.append("UPSTREAM_RAW_STRUCTURE_INVALID")
        required_actions.append("Resolve every structural inspection error.")
    if native_cadence_status != "PASS":
        reasons.append("NATIVE_TIMESTAMP_10HZ_EVIDENCE_MISSING")
        required_actions.append(
            "Provide source-carried timestamps with explicit units and pass the documented "
            "duration, effective-rate, median-gap, p99-gap, and maximum-gap gates."
        )
    if nonfinite_constant_count:
        reasons.append("NON_STANDARD_JSON_NUMERIC_CONSTANTS_REQUIRE_POLICY")
        required_actions.append(
            "Define, test, and provenance-record an explicit NaN/Infinity handling policy; "
            "the converter must not silently pass or replace these values."
        )
    reasons.append("PREPARED_DATASET_CANONICAL_VALIDATION_NOT_RUN")
    required_actions.append(
        "Convert into a new prepared tree and pass the complete common_10hz_v1 validator."
    )
    return {
        "status": "BLOCKED_FAIL_CLOSED",
        "ready": False,
        "scope": "common10 release readiness, not raw archive structure",
        "target_contract": "common_10hz_v1",
        "blocking_reasons": reasons,
        "required_actions": required_actions,
        "silent_nonfinite_passthrough_permitted": False,
        "silent_timestamp_synthesis_permitted": False,
    }


def _empty_archive_report(path: Path, scene_root: str) -> dict[str, Any]:
    return {
        "archive": path.name,
        "path": str(path),
        "scene_root_expected": scene_root,
        "status": "FAIL",
        "valid": False,
        "archive_identity": None,
        "tar_safety_audit": None,
        "scene": None,
        "frames": None,
        "rgb_surround": [],
        "rgb_auxiliary": [],
        "annotations": None,
        "maneuver_hint": None,
        "common_10hz_qualification": None,
        "conversion_readiness": None,
        "errors": [],
        "error_count": 0,
        "errors_truncated": False,
        "read_only": True,
        "extracted": False,
    }


def _inspect_verified_archive(
    path: Path,
    scene_root: str,
    initial_identity: os.stat_result,
    tar_audit: Mapping[str, Any],
) -> dict[str, Any]:
    report = _empty_archive_report(path, scene_root)
    errors = Errors()
    report["tar_safety_audit"] = {
        "status": tar_audit.get("status"),
        "actual_size": tar_audit.get("actual_size"),
        "actual_sha256": tar_audit.get("actual_sha256"),
        "member_count": (tar_audit.get("tar") or {}).get("member_count"),
        "bound_to_this_inspection": True,
    }

    flags = os.O_RDONLY
    if hasattr(os, "O_CLOEXEC"):
        flags |= os.O_CLOEXEC
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    try:
        archive_fd = os.open(path, flags)
    except OSError as error:
        errors.add(f"cannot open verified archive safely: {error}")
        return _finish_archive_report(report, errors)

    rgb_frames: dict[str, set[str]] = defaultdict(set)
    jpeg_dimensions: dict[str, set[tuple[int, int]]] = defaultdict(set)
    annotation_frames: set[str] = set()
    annotation_required_key_intersection: set[str] | None = None
    timestamp_fields_by_frame: dict[str, set[str]] = {}
    timestamp_values: dict[str, dict[str, Any]] = {}
    calibration_dimensions: dict[str, set[tuple[int, int]]] = defaultdict(set)
    command_values: set[int] = set()
    nonfinite_constant_count = 0
    nonfinite_by_token: Counter[str] = Counter()
    nonfinite_frame_ids: set[str] = set()
    nonfinite_samples: list[dict[str, str]] = []
    roots: set[str] = set()
    member_count = 0

    try:
        before = os.fstat(archive_fd)
        report["archive_identity"] = {
            **_identity_report(before),
            "sha256": tar_audit.get("actual_sha256"),
            "stable_during_audit_and_inspection": False,
        }
        if not stat.S_ISREG(before.st_mode):
            errors.add("opened archive is not a regular file")
        if _identity(before) != _identity(initial_identity):
            errors.add("archive identity changed between safety audit and inspection")
        if errors.total:
            return _finish_archive_report(report, errors)

        with os.fdopen(archive_fd, "rb", closefd=False) as raw_stream:
            with tarfile.open(fileobj=raw_stream, mode="r:gz") as archive:
                for member in archive:
                    member_count += 1
                    if member_count > AuditLimits.max_members:
                        raise InspectionFailure(
                            f"member count exceeds limit {AuditLimits.max_members}"
                        )
                    normalised = _normalise_member_name(member.name)
                    if not normalised:
                        continue
                    components = normalised.split("/")
                    roots.add(components[0])
                    if not member.isfile():
                        continue
                    if member.size < 0:
                        errors.add(f"member {member.name!r} has a negative size")
                        continue

                    if (
                        len(components) == 4
                        and components[0] == scene_root
                        and components[1] == "camera"
                        and components[2].startswith("rgb_")
                    ):
                        camera = components[2]
                        match = FRAME_FILE_RE.fullmatch(components[3])
                        if match is None or match.group("suffix") != "jpg":
                            errors.add(
                                f"RGB member has a non-canonical frame name: {member.name!r}"
                            )
                            continue
                        frame_id = match.group("frame")
                        if frame_id in rgb_frames[camera]:
                            errors.add(
                                f"RGB camera {camera} repeats frame ID {frame_id}"
                            )
                            continue
                        rgb_frames[camera].add(frame_id)
                        if len(rgb_frames[camera]) > MAX_FRAME_COUNT:
                            raise InspectionFailure(
                                f"RGB camera {camera} exceeds frame limit {MAX_FRAME_COUNT}"
                            )
                        extracted = archive.extractfile(member)
                        if extracted is None:
                            errors.add(f"cannot read JPEG member {member.name!r}")
                            continue
                        try:
                            dimensions = _jpeg_dimensions(
                                _read_member_prefix(extracted, member.size)
                            )
                            jpeg_dimensions[camera].add(dimensions)
                        except InspectionFailure as error:
                            errors.add(f"JPEG {member.name!r}: {error}")
                        finally:
                            extracted.close()
                        continue

                    if (
                        len(components) == 3
                        and components[0] == scene_root
                        and components[1] == "anno"
                    ):
                        match = FRAME_FILE_RE.fullmatch(components[2])
                        if match is None or match.group("suffix") != "json.gz":
                            errors.add(
                                f"annotation member has a non-canonical frame name: "
                                f"{member.name!r}"
                            )
                            continue
                        frame_id = match.group("frame")
                        if frame_id in annotation_frames:
                            errors.add(f"annotation repeats frame ID {frame_id}")
                            continue
                        annotation_frames.add(frame_id)
                        if len(annotation_frames) > MAX_FRAME_COUNT:
                            raise InspectionFailure(
                                f"annotation count exceeds limit {MAX_FRAME_COUNT}"
                            )
                        extracted = archive.extractfile(member)
                        if extracted is None:
                            errors.add(f"cannot read annotation member {member.name!r}")
                            continue
                        try:
                            annotation, nonfinite = _read_annotation(
                                extracted, member.size
                            )
                        except InspectionFailure as error:
                            errors.add(f"annotation {member.name!r}: {error}")
                            extracted.close()
                            continue
                        finally:
                            if not extracted.closed:
                                extracted.close()
                        keys = set(annotation)
                        frame_nonfinite_count = int(nonfinite["count"])
                        nonfinite_constant_count += frame_nonfinite_count
                        if frame_nonfinite_count:
                            nonfinite_frame_ids.add(frame_id)
                        nonfinite_by_token.update(nonfinite["by_token"])
                        for sample in nonfinite["samples"]:
                            if (
                                len(nonfinite_samples)
                                >= MAX_NONFINITE_SAMPLES_PER_ARCHIVE
                            ):
                                break
                            nonfinite_samples.append(
                                {
                                    "frame_id": frame_id,
                                    "json_pointer": sample["json_pointer"],
                                    "token": sample["token"],
                                }
                            )
                        if annotation_required_key_intersection is None:
                            annotation_required_key_intersection = keys
                        else:
                            annotation_required_key_intersection &= keys
                        missing_keys = sorted(REQUIRED_ANNOTATION_KEYS - keys)
                        if missing_keys:
                            errors.add(
                                f"annotation {frame_id} misses required keys {missing_keys}"
                            )
                        _validate_calibration(
                            annotation,
                            frame_id,
                            calibration_dimensions,
                            errors,
                        )
                        fields = _timestamp_keys(annotation)
                        timestamp_fields_by_frame[frame_id] = fields
                        timestamp_values[frame_id] = {
                            field: annotation.get(field) for field in fields
                        }
                        for key in ("command_near", "command_far", "next_command"):
                            value = annotation.get(key)
                            if isinstance(value, bool) or not isinstance(value, int):
                                errors.add(
                                    f"annotation {frame_id} field {key} is not an integer"
                                )
                            else:
                                command_values.add(value)

        after = os.fstat(archive_fd)
        if _identity(after) != _identity(before):
            errors.add("archive changed while its structure was inspected")
        try:
            final_path = os.lstat(path)
        except OSError as error:
            errors.add(f"archive path changed during inspection: {error}")
        else:
            if _identity(final_path) != _identity(after):
                errors.add("archive path no longer refers to the inspected identity")
        if report["archive_identity"] is not None:
            identity_stable = (
                _identity(before) == _identity(initial_identity)
                and _identity(after) == _identity(before)
            )
            if "final_path" in locals():
                identity_stable = identity_stable and (
                    _identity(final_path) == _identity(after)
                )
            else:
                identity_stable = False
            report["archive_identity"][
                "stable_during_audit_and_inspection"
            ] = identity_stable
    except (
        EOFError,
        InspectionFailure,
        MemoryError,
        OSError,
        OverflowError,
        RecursionError,
        tarfile.TarError,
        UnicodeError,
        ValueError,
    ) as error:
        errors.add(f"archive structure inspection failed: {error}")
    finally:
        os.close(archive_fd)

    audit_member_count = (tar_audit.get("tar") or {}).get("member_count")
    if audit_member_count != member_count:
        errors.add(
            f"member count differs from bound safety audit: "
            f"{member_count} != {audit_member_count}"
        )
    if roots != {scene_root}:
        errors.add(
            f"archive top-level roots must be exactly {[scene_root]}, found {sorted(roots)}"
        )

    observed_rgb = set(rgb_frames)
    expected_rgb = set(ALL_RGB_DIRECTORIES)
    if observed_rgb != expected_rgb:
        errors.add(
            f"RGB camera folders must be exactly {sorted(expected_rgb)}, "
            f"found {sorted(observed_rgb)}"
        )

    annotation_frame_report = _continuous_frame_report(annotation_frames)
    if not annotation_frame_report["continuous_from_zero"]:
        errors.add("annotation frame IDs are not continuous from 00000")
    for camera in sorted(expected_rgb):
        camera_frames = rgb_frames.get(camera, set())
        if camera_frames != annotation_frames:
            missing = sorted(annotation_frames - camera_frames)[:MAX_ERRORS]
            extra = sorted(camera_frames - annotation_frames)[:MAX_ERRORS]
            errors.add(
                f"RGB camera {camera} frame set differs from annotations; "
                f"missing={missing}, extra={extra}"
            )
        if not _continuous_frame_report(camera_frames)["continuous_from_zero"]:
            errors.add(f"RGB camera {camera} frame IDs are not continuous from 00000")
        dimensions = jpeg_dimensions.get(camera, set())
        if len(dimensions) != 1:
            errors.add(
                f"RGB camera {camera} must have one consistent JPEG dimension, "
                f"found {sorted(dimensions)}"
            )
        calibration = calibration_dimensions.get(camera, set())
        if len(calibration) != 1:
            errors.add(
                f"RGB camera {camera} must have one consistent calibration image size, "
                f"found {sorted(calibration)}"
            )
        if dimensions and calibration and dimensions != calibration:
            errors.add(
                f"RGB camera {camera} JPEG dimensions {sorted(dimensions)} differ from "
                f"calibration {sorted(calibration)}"
            )

    scene_match = SCENE_RE.fullmatch(scene_root)
    if scene_match is None:
        errors.add(f"scene root does not match the official naming contract: {scene_root!r}")
        scene = {"root": scene_root}
    else:
        scene = {
            "root": scene_root,
            "scenario_hint": scene_match.group("scenario"),
            "town": scene_match.group("town"),
            "route_id": int(scene_match.group("route")),
            "weather_id": int(scene_match.group("weather")),
            "hint_caveat": "Values are parsed only from the archive filename.",
        }

    cadence_evidence = _native_timestamp_cadence_evidence(
        annotation_frames,
        timestamp_fields_by_frame,
        timestamp_values,
    )
    qualification = {
        "status": "NOT_QUALIFIED_COMMON10",
        "qualified": False,
        "scope": "complete common_10hz_v1 prepared-dataset contract",
        "reason": "RAW_ARCHIVE_REQUIRES_CONVERSION_AND_CANONICAL_VALIDATION",
        "canonical_validator_required": True,
        "native_timestamp_cadence_evidence": cadence_evidence,
        "unvalidated_gates": [
            "six-camera decoded payload and per-frame timestamp alignment",
            "ego-state, command, route, and future-trajectory semantics",
            "minimum usable duration, motion, and split leakage",
            "converter provenance and post-conversion manifest integrity",
        ],
    }
    nonfinite_report = {
        "status": (
            "PRESENT_BLOCKING_CONVERSION"
            if nonfinite_constant_count
            else "ABSENT"
        ),
        "count": nonfinite_constant_count,
        "annotation_frame_count": len(nonfinite_frame_ids),
        "by_token": dict(sorted(nonfinite_by_token.items())),
        "samples": nonfinite_samples,
        "samples_truncated": nonfinite_constant_count > len(nonfinite_samples),
        "maximum_reported_samples": MAX_NONFINITE_SAMPLES_PER_ARCHIVE,
        "note": (
            "NaN, Infinity, and -Infinity are not JSON numbers under RFC 8259. "
            "Their presence does not invalidate this upstream archive structure, "
            "but conversion must apply an explicit, provenance-recorded policy."
        ),
    }
    readiness = _conversion_readiness(
        structural_error_count=errors.total,
        native_cadence_status=cadence_evidence["status"],
        nonfinite_constant_count=nonfinite_constant_count,
    )
    report.update(
        {
            "scene": scene,
            "frames": annotation_frame_report,
            "rgb_surround": [
                {
                    "directory": camera,
                    "sensor": SURROUND_RGB_DIRECTORIES[camera],
                    "frame_count": len(rgb_frames.get(camera, set())),
                    "jpeg_dimensions": [
                        {"width": width, "height": height}
                        for width, height in sorted(jpeg_dimensions.get(camera, set()))
                    ],
                }
                for camera in sorted(SURROUND_RGB_DIRECTORIES)
            ],
            "rgb_auxiliary": [
                {
                    "directory": camera,
                    "sensor": AUXILIARY_RGB_DIRECTORIES[camera],
                    "frame_count": len(rgb_frames.get(camera, set())),
                    "jpeg_dimensions": [
                        {"width": width, "height": height}
                        for width, height in sorted(jpeg_dimensions.get(camera, set()))
                    ],
                }
                for camera in sorted(AUXILIARY_RGB_DIRECTORIES)
            ],
            "annotations": {
                "frame_count": len(annotation_frames),
                "required_keys": sorted(REQUIRED_ANNOTATION_KEYS),
                "required_keys_common_to_all_frames": sorted(
                    (annotation_required_key_intersection or set())
                    & REQUIRED_ANNOTATION_KEYS
                ),
                "camera_calibration_required_keys": sorted(
                    REQUIRED_CAMERA_CALIBRATION_KEYS
                ),
                "native_timestamp_fields_union": cadence_evidence[
                    "native_timestamp_fields_union"
                ],
                "native_timestamp_fields_common_to_all_frames": cadence_evidence[
                    "native_timestamp_fields_common_to_all_frames"
                ],
                "raw_command_values": sorted(command_values),
                "non_standard_json_constants": nonfinite_report,
            },
            "maneuver_hint": _maneuver_hint(scene_root, command_values),
            "common_10hz_qualification": qualification,
            "conversion_readiness": readiness,
        }
    )
    return _finish_archive_report(report, errors)


def _finish_archive_report(report: dict[str, Any], errors: Errors) -> dict[str, Any]:
    report["errors"] = errors.items
    report["error_count"] = errors.total
    report["errors_truncated"] = errors.total > len(errors.items)
    report["valid"] = errors.total == 0
    report["status"] = "PASS" if report["valid"] else "FAIL"
    if report.get("conversion_readiness") is None:
        report["conversion_readiness"] = _conversion_readiness(
            structural_error_count=max(errors.total, 1),
            native_cadence_status=None,
            nonfinite_constant_count=0,
        )
    return report


def inspect_archive(
    archive_path: Path | str,
    spec: ArchiveSpec,
) -> dict[str, Any]:
    """Safety-audit and structurally inspect one archive, read-only."""
    path = _absolute_path(archive_path)
    try:
        scene_root = _scene_root(path.name)
    except InspectionFailure as error:
        report = _empty_archive_report(path, "")
        errors = Errors()
        errors.add(str(error))
        return _finish_archive_report(report, errors)

    report = _empty_archive_report(path, scene_root)
    errors = Errors()
    if not isinstance(spec, ArchiveSpec):
        errors.add("archive manifest entry is not an ArchiveSpec")
        return _finish_archive_report(report, errors)
    try:
        initial = os.lstat(path)
    except OSError as error:
        errors.add(f"cannot inspect archive path: {error}")
        return _finish_archive_report(report, errors)
    if stat.S_ISLNK(initial.st_mode):
        errors.add("archive path must not be a symbolic link")
    elif not stat.S_ISREG(initial.st_mode):
        errors.add("archive path must be a regular file")
    if errors.total:
        return _finish_archive_report(report, errors)

    audit = verify_tar_archive(
        path,
        expected_size=spec.size,
        expected_sha256=spec.sha256,
    )
    if not audit.get("valid"):
        report["tar_safety_audit"] = {
            "status": audit.get("status"),
            "actual_size": audit.get("actual_size"),
            "actual_sha256": audit.get("actual_sha256"),
            "errors": audit.get("errors", []),
            "error_count": audit.get("error_count"),
            "bound_to_this_inspection": False,
        }
        errors.add("full-stream TAR+gzip safety/integrity audit did not pass")
        for message in audit.get("errors", [])[:MAX_ERRORS]:
            errors.add(f"TAR audit: {message}")
        return _finish_archive_report(report, errors)
    try:
        after_audit = os.lstat(path)
    except OSError as error:
        errors.add(f"archive path changed after safety audit: {error}")
        return _finish_archive_report(report, errors)
    if _identity(after_audit) != _identity(initial):
        errors.add("archive identity changed during or immediately after safety audit")
        return _finish_archive_report(report, errors)
    return _inspect_verified_archive(path, scene_root, after_audit, audit)


def _empty_set_report(directory: Path, expected_count: int) -> dict[str, Any]:
    return {
        "schema_version": SCHEMA_VERSION,
        "status": "FAIL",
        "valid": False,
        "archive_directory": str(directory),
        "expected_archive_count": expected_count,
        "inspected_archive_count": 0,
        "archives": [],
        "totals": None,
        "non_standard_json_constants": None,
        "common_10hz_qualification": None,
        "conversion_readiness": None,
        "errors": [],
        "error_count": 0,
        "errors_truncated": False,
        "read_only": True,
        "extracted": False,
    }


def inspect_archive_set(
    archive_directory: Path | str,
    manifest: Mapping[str, ArchiveSpec] = OFFICIAL_LEGACY_MINI_ARCHIVES,
) -> dict[str, Any]:
    """Inspect an exact manifest-bound Bench2Drive archive directory."""
    directory = _absolute_path(archive_directory)
    report = _empty_set_report(directory, len(manifest))
    errors = Errors()
    if not manifest:
        errors.add("archive manifest must not be empty")
        return _finish_set_report(report, errors)
    try:
        before = os.lstat(directory)
    except OSError as error:
        errors.add(f"cannot inspect archive directory: {error}")
        return _finish_set_report(report, errors)
    if stat.S_ISLNK(before.st_mode):
        errors.add("archive directory must not be a symbolic link")
    elif not stat.S_ISDIR(before.st_mode):
        errors.add("archive directory is not a directory")
    if errors.total:
        return _finish_set_report(report, errors)

    try:
        entries = {entry.name: entry for entry in os.scandir(directory)}
    except OSError as error:
        errors.add(f"cannot enumerate archive directory: {error}")
        return _finish_set_report(report, errors)
    expected = set(manifest)
    observed = set(entries)
    missing = sorted(expected - observed)
    extra = sorted(observed - expected)
    if missing:
        errors.add(f"missing expected archives: {missing}")
    if extra:
        errors.add(f"unexpected archive-directory entries: {extra}")

    archives = []
    for name in sorted(expected & observed):
        archive_report = inspect_archive(directory / name, manifest[name])
        archives.append(archive_report)
        if not archive_report["valid"]:
            errors.add(f"archive {name} failed structural inspection")
    report["archives"] = archives
    report["inspected_archive_count"] = len(archives)

    try:
        after = os.lstat(directory)
    except OSError as error:
        errors.add(f"archive directory changed during inspection: {error}")
    else:
        if _identity(after) != _identity(before):
            errors.add("archive directory identity changed during inspection")

    structurally_valid = [item for item in archives if item["valid"]]
    cadence_proven = [
        item
        for item in structurally_valid
        if (
            (item.get("common_10hz_qualification") or {}).get(
                "native_timestamp_cadence_evidence"
            )
            or {}
        ).get("status")
        == "PASS"
    ]
    total_frames = sum((item.get("frames") or {}).get("count", 0) for item in archives)
    observed_surround_images = sum(
        int(camera.get("frame_count", 0))
        for item in archives
        for camera in item.get("rgb_surround", [])
    )
    observed_auxiliary_images = sum(
        int(camera.get("frame_count", 0))
        for item in archives
        for camera in item.get("rgb_auxiliary", [])
    )
    nonfinite_constant_count = 0
    nonfinite_annotation_frame_count = 0
    nonfinite_by_token: Counter[str] = Counter()
    nonfinite_archives: list[str] = []
    nonfinite_samples: list[dict[str, str]] = []
    for item in archives:
        diagnostics = (
            (item.get("annotations") or {}).get("non_standard_json_constants") or {}
        )
        count = int(diagnostics.get("count", 0))
        nonfinite_constant_count += count
        nonfinite_annotation_frame_count += int(
            diagnostics.get("annotation_frame_count", 0)
        )
        nonfinite_by_token.update(diagnostics.get("by_token", {}))
        if count:
            nonfinite_archives.append(item["archive"])
        for sample in diagnostics.get("samples", []):
            if len(nonfinite_samples) >= MAX_NONFINITE_SAMPLES_PER_SET:
                break
            nonfinite_samples.append(
                {
                    "archive": item["archive"],
                    "frame_id": sample["frame_id"],
                    "json_pointer": sample["json_pointer"],
                    "token": sample["token"],
                }
            )
    conversion_ready = [
        item
        for item in archives
        if (item.get("conversion_readiness") or {}).get("ready") is True
    ]
    report["totals"] = {
        "structurally_valid_archive_count": len(structurally_valid),
        "annotation_frame_count_observed": total_frames,
        "surround_rgb_image_count_observed": observed_surround_images,
        "auxiliary_rgb_image_count_observed": observed_auxiliary_images,
        "native_timestamp_10hz_cadence_evidence_pass_count": len(cadence_proven),
        "non_standard_json_constant_count": nonfinite_constant_count,
        "archives_with_non_standard_json_constants": len(nonfinite_archives),
        "conversion_ready_archive_count": len(conversion_ready),
    }
    report["non_standard_json_constants"] = {
        "status": (
            "PRESENT_BLOCKING_CONVERSION"
            if nonfinite_constant_count
            else "ABSENT"
        ),
        "count": nonfinite_constant_count,
        "annotation_frame_count": nonfinite_annotation_frame_count,
        "by_token": dict(sorted(nonfinite_by_token.items())),
        "affected_archives": nonfinite_archives,
        "samples": nonfinite_samples,
        "samples_truncated": nonfinite_constant_count > len(nonfinite_samples),
        "maximum_reported_samples": MAX_NONFINITE_SAMPLES_PER_SET,
        "conversion_policy": "EXPLICIT_AUDITED_POLICY_REQUIRED",
    }
    report["common_10hz_qualification"] = {
        "status": "NOT_QUALIFIED_COMMON10",
        "qualified": False,
        "scope": "complete common_10hz_v1 prepared-dataset contract",
        "official_nominal_frequency_hz": 10.0,
        "official_claim_source": UPSTREAM_NOMINAL_10HZ_SOURCE,
        "official_claim_is_native_timestamp_evidence": False,
        "reason": "RAW_ARCHIVE_SET_REQUIRES_CONVERSION_AND_CANONICAL_VALIDATION",
        "canonical_validator_required": True,
        "native_timestamp_cadence_evidence_pass_count": len(cadence_proven),
        "expected_archive_count": len(manifest),
    }
    set_blocking_reasons: list[str] = []
    set_required_actions: list[str] = []
    if errors.total or len(structurally_valid) != len(manifest):
        set_blocking_reasons.append("UPSTREAM_RAW_STRUCTURE_INVALID_OR_INCOMPLETE")
        set_required_actions.append("Resolve every archive-set structural error.")
    if len(cadence_proven) != len(manifest):
        set_blocking_reasons.append("NATIVE_TIMESTAMP_10HZ_EVIDENCE_MISSING")
        set_required_actions.append(
            "Supply source-carried timestamps; do not synthesize them from frame indices."
        )
    if nonfinite_constant_count:
        set_blocking_reasons.append(
            "NON_STANDARD_JSON_NUMERIC_CONSTANTS_REQUIRE_POLICY"
        )
        set_required_actions.append(
            "Sanitize NaN/Infinity only under an explicit tested policy and record "
            "the transformation in conversion provenance."
        )
    set_blocking_reasons.append("PREPARED_DATASET_CANONICAL_VALIDATION_NOT_RUN")
    set_required_actions.append(
        "Convert into a new prepared tree and pass the complete common_10hz_v1 validator."
    )
    report["conversion_readiness"] = {
        "status": "BLOCKED_FAIL_CLOSED",
        "ready": False,
        "target_contract": "common_10hz_v1",
        "blocking_reasons": set_blocking_reasons,
        "required_actions": set_required_actions,
        "non_standard_json_constant_count": nonfinite_constant_count,
        "affected_archives": nonfinite_archives,
        "silent_nonfinite_passthrough_permitted": False,
        "silent_timestamp_synthesis_permitted": False,
    }
    return _finish_set_report(report, errors)


def _finish_set_report(report: dict[str, Any], errors: Errors) -> dict[str, Any]:
    report["errors"] = errors.items
    report["error_count"] = errors.total
    report["errors_truncated"] = errors.total > len(errors.items)
    report["valid"] = errors.total == 0
    report["status"] = "PASS" if report["valid"] else "FAIL"
    if report.get("conversion_readiness") is None:
        report["conversion_readiness"] = {
            "status": "BLOCKED_FAIL_CLOSED",
            "ready": False,
            "target_contract": "common_10hz_v1",
            "blocking_reasons": ["ARCHIVE_SET_INSPECTION_DID_NOT_COMPLETE"],
            "required_actions": ["Complete a passing archive-set inspection."],
            "non_standard_json_constant_count": None,
            "affected_archives": [],
            "silent_nonfinite_passthrough_permitted": False,
            "silent_timestamp_synthesis_permitted": False,
        }
    return report


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Read-only, no-extraction structure inspection for the exact official "
            "legacy Bench2Drive Mini-10 archive set."
        )
    )
    parser.add_argument("archive_directory", type=Path)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    report = inspect_archive_set(
        args.archive_directory,
        OFFICIAL_LEGACY_MINI_ARCHIVES,
    )
    json.dump(report, sys.stdout, indent=2, sort_keys=True)
    sys.stdout.write("\n")
    return 0 if report["valid"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
