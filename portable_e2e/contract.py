"""Standard-library validator for the portable 10 Hz E2E dataset contract.

The validator deliberately performs the cross-file checks that a JSON Schema
cannot express: hashes, safe paths, calibration geometry, synchronized camera
bundles, episode cadence, future-label masks, and split leakage.
"""

from __future__ import annotations

from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import stat
from statistics import median
from typing import Any, Iterable, Mapping, Sequence


CAMERA_ORDER = (
    "CAM_FRONT",
    "CAM_BACK",
    "CAM_FRONT_LEFT",
    "CAM_BACK_LEFT",
    "CAM_FRONT_RIGHT",
    "CAM_BACK_RIGHT",
)
CONTRACT_ID = "common_10hz_v1"
CONTRACT_SCHEMA_ID = "autoware-e2e.common10.contract.v1"
DATASET_SCHEMA_ID = "autoware-e2e.common10.dataset.v1"
RIG_SCHEMA_ID = "autoware-e2e.common10.rig.v1"
EPISODE_SCHEMA_ID = "autoware-e2e.common10.episode.v1"
SAMPLE_SCHEMA_ID = "autoware-e2e.common10.sample.v1"
SOURCE_MANIFEST_SCHEMA_ID = "autoware-e2e.common10.source-manifest.v1"
REPORT_SCHEMA_ID = "autoware-e2e.common10.validation-report.v1"
DEFAULT_CONTRACT_PATH = Path(__file__).with_name("config") / "common_10hz_v1.contract.json"
SHA256_PATTERN = re.compile(r"[0-9a-f]{64}")
MAX_ROUTE_POLYLINE_POINTS = 100_000
MAX_ROUTE_ARC_LENGTH_M = 100_000.0
MAX_SOURCE_MANIFEST_FRAMES = 500_000
MIB = 1024**2
READ_CHUNK_BYTES = 8 * MIB
MAX_JSON_FILE_BYTES = 256 * MIB
MAX_JSONL_LINE_BYTES = 8 * MIB
MAX_JPEG_FILE_BYTES = 16 * MIB


class ContractError(ValueError):
    """Raised when a portable dataset fails closed against the contract."""


def _resource_limits_report() -> dict[str, int]:
    return {
        "max_json_file_bytes": MAX_JSON_FILE_BYTES,
        "max_jsonl_line_bytes": MAX_JSONL_LINE_BYTES,
        "max_jpeg_file_bytes": MAX_JPEG_FILE_BYTES,
        "max_source_manifest_frames": MAX_SOURCE_MANIFEST_FRAMES,
    }


def _reject_duplicate_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    value: dict[str, Any] = {}
    for key, child in pairs:
        if key in value:
            raise ContractError(f"duplicate JSON key: {key!r}")
        value[key] = child
    return value


def _loads_json(text: str, context: str) -> dict[str, Any]:
    try:
        value = json.loads(
            text,
            object_pairs_hook=_reject_duplicate_keys,
            parse_constant=lambda token: (_ for _ in ()).throw(
                ContractError(f"{context}: non-standard JSON number {token!r}")
            ),
        )
    except MemoryError as error:
        raise ContractError(f"{context}: JSON parsing exceeded available memory") from error
    except (RecursionError, OverflowError) as error:
        raise ContractError(f"{context}: JSON nesting exceeds the safe parser depth") from error
    except json.JSONDecodeError as error:
        raise ContractError(f"{context}: invalid JSON: {error}") from error
    if not isinstance(value, dict):
        raise ContractError(f"{context}: root must be a JSON object")
    return value


def _file_identity(metadata: os.stat_result) -> tuple[int, int, int, int, int, int]:
    return (
        metadata.st_dev,
        metadata.st_ino,
        metadata.st_mode,
        metadata.st_size,
        metadata.st_mtime_ns,
        metadata.st_ctime_ns,
    )


def _open_regular_read_only(path: Path, context: str) -> tuple[int, os.stat_result]:
    try:
        path_metadata = os.lstat(path)
    except (OSError, ValueError) as error:
        raise ContractError(f"cannot inspect {context}: {error}") from error
    if stat.S_ISLNK(path_metadata.st_mode):
        raise ContractError(f"{context} must not be a symbolic link")
    if not stat.S_ISREG(path_metadata.st_mode):
        raise ContractError(f"{context} must be a regular file")
    flags = os.O_RDONLY
    if hasattr(os, "O_CLOEXEC"):
        flags |= os.O_CLOEXEC
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    try:
        descriptor = os.open(path, flags)
    except (OSError, ValueError) as error:
        raise ContractError(f"cannot open {context} safely: {error}") from error
    try:
        opened = os.fstat(descriptor)
        if not stat.S_ISREG(opened.st_mode):
            raise ContractError(f"opened {context} is not a regular file")
        if _file_identity(opened) != _file_identity(path_metadata):
            raise ContractError(f"{context} changed before it could be opened")
        return descriptor, opened
    except BaseException:
        os.close(descriptor)
        raise


def _verify_open_file_identity(
    descriptor: int,
    path: Path,
    expected: os.stat_result,
    context: str,
) -> None:
    after = os.fstat(descriptor)
    if _file_identity(after) != _file_identity(expected):
        raise ContractError(f"{context} changed while it was read")
    try:
        final_path = os.lstat(path)
    except (OSError, ValueError) as error:
        raise ContractError(f"{context} path changed while it was read: {error}") from error
    if _file_identity(final_path) != _file_identity(after):
        raise ContractError(f"{context} path refers to a different object after reading")


def _read_regular_file_bounded(
    path: Path,
    maximum_bytes: int,
    context: str,
) -> bytearray:
    """Read one stable regular file with a hard allocation bound."""
    descriptor, before = _open_regular_read_only(path, context)
    try:
        if before.st_size > maximum_bytes:
            raise ContractError(
                f"{context} size {before.st_size} exceeds limit {maximum_bytes} bytes"
            )
        try:
            payload = bytearray(before.st_size)
            offset = 0
            while offset < before.st_size:
                block = os.read(
                    descriptor,
                    min(READ_CHUNK_BYTES, before.st_size - offset),
                )
                if not block:
                    raise ContractError(f"{context} ended before its declared size")
                payload[offset : offset + len(block)] = block
                offset += len(block)
            if os.read(descriptor, 1):
                raise ContractError(f"{context} grew beyond its declared size")
        except MemoryError as error:
            raise ContractError(
                f"{context} could not be read within the bounded memory budget"
            ) from error
        _verify_open_file_identity(descriptor, path, before, context)
        return payload
    except ContractError:
        raise
    except (OSError, ValueError) as error:
        raise ContractError(f"cannot read {context}: {error}") from error
    finally:
        os.close(descriptor)


def _read_json_and_sha256(
    path: Path, context: str | None = None
) -> tuple[dict[str, Any], str]:
    label = context or f"JSON file {path}"
    payload = _read_regular_file_bounded(path, MAX_JSON_FILE_BYTES, label)
    try:
        text = payload.decode("utf-8")
    except UnicodeDecodeError as error:
        raise ContractError(f"{label}: JSON must be UTF-8: {error}") from error
    except MemoryError as error:
        raise ContractError(f"{label}: JSON decoding exceeded available memory") from error
    try:
        digest = hashlib.sha256(payload).hexdigest()
    except MemoryError as error:
        raise ContractError(f"{label}: JSON hashing exceeded available memory") from error
    return _loads_json(text, label), digest


def _read_json(path: Path, context: str | None = None) -> dict[str, Any]:
    return _read_json_and_sha256(path, context)[0]


def _mapping(value: Any, context: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ContractError(f"{context} must be an object")
    return value


def _sequence(value: Any, context: str) -> Sequence[Any]:
    if not isinstance(value, list):
        raise ContractError(f"{context} must be an array")
    return value


def _string(value: Any, context: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ContractError(f"{context} must be a non-empty string")
    return value


def _utc_datetime(value: Any, context: str) -> datetime:
    text = _string(value, context)
    try:
        parsed = datetime.fromisoformat(text.replace("Z", "+00:00"))
    except ValueError as error:
        raise ContractError(f"{context} must be an ISO-8601 timestamp") from error
    if parsed.tzinfo is None or parsed.utcoffset() != timezone.utc.utcoffset(parsed):
        raise ContractError(f"{context} must include UTC timezone information")
    return parsed


def _integer(value: Any, context: str, minimum: int | None = None) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ContractError(f"{context} must be an integer")
    if minimum is not None and value < minimum:
        raise ContractError(f"{context} must be >= {minimum}")
    return value


def _number(value: Any, context: str, minimum: float | None = None) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ContractError(f"{context} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise ContractError(f"{context} must be finite")
    if minimum is not None and result < minimum:
        raise ContractError(f"{context} must be >= {minimum}")
    return result


def _boolean(value: Any, context: str) -> bool:
    if not isinstance(value, bool):
        raise ContractError(f"{context} must be boolean")
    return value


def _vector(
    value: Any,
    length: int,
    context: str,
    *,
    allow_null: bool = False,
) -> tuple[float | None, ...]:
    items = _sequence(value, context)
    if len(items) != length:
        raise ContractError(f"{context} must contain exactly {length} values")
    output: list[float | None] = []
    for index, item in enumerate(items):
        if item is None and allow_null:
            output.append(None)
        else:
            output.append(_number(item, f"{context}[{index}]"))
    return tuple(output)


def _sha256(value: Any, context: str) -> str:
    text = _string(value, context)
    if SHA256_PATTERN.fullmatch(text) is None:
        raise ContractError(f"{context} must be a lowercase SHA256 digest")
    return text


def sha256_file(path: Path, block_size: int = 8 * 1024 * 1024) -> str:
    if isinstance(block_size, bool) or not isinstance(block_size, int) or block_size <= 0:
        raise ContractError("SHA-256 block_size must be a positive integer")
    read_size = min(block_size, READ_CHUNK_BYTES)
    context = f"file {path}"
    descriptor, before = _open_regular_read_only(path, context)
    digest = hashlib.sha256()
    try:
        while block := os.read(descriptor, read_size):
            try:
                digest.update(block)
            except MemoryError as error:
                raise ContractError(
                    f"{context} hashing exceeded the bounded memory budget"
                ) from error
        _verify_open_file_identity(descriptor, path, before, context)
    except ContractError:
        raise
    except (OSError, ValueError) as error:
        raise ContractError(f"cannot hash {path}: {error}") from error
    finally:
        os.close(descriptor)
    try:
        return digest.hexdigest()
    except MemoryError as error:
        raise ContractError(f"{context} hashing exceeded available memory") from error


def contract_fingerprint(contract: Mapping[str, Any]) -> str:
    """Hash the effective contract, excluding loader-only provenance keys."""
    effective = {key: value for key, value in contract.items() if not str(key).startswith("_")}
    encoded = json.dumps(
        effective,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _safe_file(root: Path, value: Any, context: str, *, nonempty: bool = True) -> Path:
    text = _string(value, context)
    try:
        if "\0" in text:
            raise ContractError(f"{context} contains a NUL byte")
        relative = Path(text)
        if relative.is_absolute() or ".." in relative.parts or "\\" in text:
            raise ContractError(f"{context} must be a safe POSIX path below {root}")
        unresolved = root / relative
        current = root
        for part in relative.parts:
            current /= part
            if current.is_symlink():
                raise ContractError(f"{context} must not traverse a symbolic link")
        candidate = unresolved.resolve()
        resolved_root = root.resolve()
        if candidate != resolved_root and resolved_root not in candidate.parents:
            raise ContractError(f"{context} escapes {root}")
        if not candidate.is_file():
            raise ContractError(f"{context} does not exist: {candidate}")
        if nonempty and candidate.stat().st_size == 0:
            raise ContractError(f"{context} is empty: {candidate}")
        return candidate
    except ContractError:
        raise
    except (OSError, RuntimeError, ValueError) as error:
        raise ContractError(f"{context} cannot be resolved safely: {error}") from error


def _jpeg_dimensions_impl(path: Path, context: str) -> tuple[int, int, int, str, str]:
    """Return SOF dimensions plus file/scan hashes without decoding pixels."""
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
    data = _read_regular_file_bounded(
        path,
        MAX_JPEG_FILE_BYTES,
        f"{context} JPEG file",
    )
    if len(data) < 20 or data[:2] != b"\xff\xd8":
        raise ContractError(f"{context} is not a JPEG file")
    if data[-2:] != b"\xff\xd9":
        raise ContractError(f"{context} is a truncated JPEG file")

    offset = 2
    dimensions: tuple[int, int, int] | None = None
    while offset < len(data) - 2:
        if data[offset] != 0xFF:
            raise ContractError(f"{context} has data before its JPEG scan")
        while offset < len(data) and data[offset] == 0xFF:
            offset += 1
        if offset >= len(data):
            break
        marker = data[offset]
        offset += 1
        if marker in (0x00, 0xD8, 0xD9):
            raise ContractError(f"{context} has an invalid JPEG marker sequence")
        if marker == 0x01 or 0xD0 <= marker <= 0xD7:
            continue
        if offset + 2 > len(data) - 2:
            break
        segment_length = int.from_bytes(data[offset : offset + 2], "big")
        if segment_length < 2 or offset + segment_length > len(data) - 2:
            raise ContractError(f"{context} has an invalid JPEG segment length")
        payload = data[offset + 2 : offset + segment_length]
        offset += segment_length
        if marker in start_of_frame:
            if len(payload) < 6:
                raise ContractError(f"{context} has an invalid JPEG frame header")
            precision = payload[0]
            height = int.from_bytes(payload[1:3], "big")
            width = int.from_bytes(payload[3:5], "big")
            components = payload[5]
            if precision != 8 or len(payload) != 6 + 3 * components:
                raise ContractError(f"{context} has an invalid JPEG component table")
            component_ids: set[int] = set()
            for component_index in range(components):
                base = 6 + component_index * 3
                component_id, sampling, table_id = payload[base : base + 3]
                if component_id in component_ids or sampling == 0 or table_id > 3:
                    raise ContractError(f"{context} has an invalid JPEG component descriptor")
                component_ids.add(component_id)
            if width <= 0 or height <= 0 or components <= 0:
                raise ContractError(f"{context} has invalid JPEG dimensions")
            dimensions = (width, height, components)
        if marker == 0xDA:
            if dimensions is None:
                raise ContractError(f"{context} has a JPEG scan before its frame header")
            if not payload:
                raise ContractError(f"{context} has an empty JPEG scan header")
            scan_components = payload[0]
            if scan_components <= 0 or len(payload) != 1 + 2 * scan_components + 3:
                raise ContractError(f"{context} has an invalid JPEG scan header")
            entropy_payload = data[offset:-2]
            if len(entropy_payload) < 4 or all(value == 0xFF for value in entropy_payload):
                raise ContractError(f"{context} has no JPEG entropy payload")
            scan_digest = hashlib.sha256(payload)
            scan_digest.update(entropy_payload)
            scan_hash = scan_digest.hexdigest()
            return (*dimensions, hashlib.sha256(data).hexdigest(), scan_hash)
    raise ContractError(f"{context} has no complete JPEG frame and scan metadata")


def _jpeg_dimensions(path: Path, context: str) -> tuple[int, int, int, str, str]:
    try:
        return _jpeg_dimensions_impl(path, context)
    except MemoryError as error:
        raise ContractError(
            f"{context} JPEG inspection exceeded the bounded memory budget"
        ) from error


def _route_geometry_semantic_hash(
    path: Path,
    *,
    episode: Mapping[str, Any],
    contract: Mapping[str, Any],
    expected_sha256: str | None = None,
) -> str:
    geometry, actual_sha256 = _read_json_and_sha256(path)
    if expected_sha256 is not None and actual_sha256 != expected_sha256:
        raise ContractError(f"{path} byte-level SHA256 mismatch")
    if geometry.get("route_id") != episode["route_id"]:
        raise ContractError(f"{path}.route_id does not match the episode")
    if geometry.get("frame_id") != contract["coordinates"]["map_frame"]:
        raise ContractError(f"{path}.frame_id must be map")
    polyline = _sequence(geometry.get("polyline_m"), f"{path}.polyline_m")
    if len(polyline) < 2:
        raise ContractError(f"{path}.polyline_m needs at least two points")
    if len(polyline) > MAX_ROUTE_POLYLINE_POINTS:
        raise ContractError(
            f"{path}.polyline_m exceeds the {MAX_ROUTE_POLYLINE_POINTS}-point limit"
        )
    route_points: list[tuple[float, float]] = []
    for index, point in enumerate(polyline):
        values = _vector(point, 2, f"{path}.polyline_m[{index}]")
        parsed = (float(values[0]), float(values[1]))
        if not route_points or math.dist(route_points[-1], parsed) > 1.0e-9:
            route_points.append(parsed)
    if len(route_points) < 2:
        raise ContractError(f"{path}.polyline_m must span a nonzero route")

    cumulative_distance = [0.0]
    for first, second in zip(route_points, route_points[1:]):
        cumulative_distance.append(cumulative_distance[-1] + math.dist(first, second))
    total_distance = cumulative_distance[-1]
    if not math.isfinite(total_distance):
        raise ContractError(f"{path}.polyline_m has a non-finite arc length")
    if total_distance <= 0.0:
        raise ContractError(f"{path}.polyline_m must span a nonzero route")
    if total_distance > MAX_ROUTE_ARC_LENGTH_M:
        raise ContractError(
            f"{path}.polyline_m exceeds the {MAX_ROUTE_ARC_LENGTH_M:g} m arc-length limit"
        )
    sample_spacing_m = 1.0
    sample_distances = [
        float(index)
        for index in range(int(math.floor(total_distance / sample_spacing_m)) + 1)
    ]
    if total_distance - sample_distances[-1] > 1.0e-6:
        sample_distances.append(total_distance)

    normalized_points: list[list[float]] = []
    segment_index = 0
    for target_distance in sample_distances:
        while (
            segment_index + 1 < len(cumulative_distance) - 1
            and cumulative_distance[segment_index + 1] < target_distance
        ):
            segment_index += 1
        start_distance = cumulative_distance[segment_index]
        end_distance = cumulative_distance[segment_index + 1]
        ratio = (target_distance - start_distance) / (end_distance - start_distance)
        first = route_points[segment_index]
        second = route_points[segment_index + 1]
        interpolated = (
            first[0] + ratio * (second[0] - first[0]),
            first[1] + ratio * (second[1] - first[1]),
        )
        quantized = [
            0.0 if abs(value) < 0.05 else round(value, 1) for value in interpolated
        ]
        if not normalized_points or quantized != normalized_points[-1]:
            normalized_points.append(quantized)
    canonical = {
        "map_id": episode["map_id"],
        "frame_id": geometry["frame_id"],
        "arc_length_sample_spacing_m": sample_spacing_m,
        "polyline_m_rounded_10cm": normalized_points,
    }
    encoded = json.dumps(
        canonical, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _validated_route_points(
    path: Path,
    *,
    expected_sha256: str,
) -> tuple[tuple[float, float], ...]:
    """Load route points after the episode-level semantic validation has passed."""
    geometry, actual_sha256 = _read_json_and_sha256(path)
    if actual_sha256 != expected_sha256:
        raise ContractError(f"{path} changed after route semantic validation")
    polyline = _sequence(geometry.get("polyline_m"), f"{path}.polyline_m")
    route_points: list[tuple[float, float]] = []
    for index, point in enumerate(polyline):
        values = _vector(point, 2, f"{path}.polyline_m[{index}]")
        parsed = (float(values[0]), float(values[1]))
        if not route_points or math.dist(route_points[-1], parsed) > 1.0e-9:
            route_points.append(parsed)
    if len(route_points) < 2:
        raise ContractError(f"{path}.polyline_m must span a nonzero route")
    return tuple(route_points)


def _canonical_route_in_base(
    route_points: Sequence[tuple[float, float]],
    *,
    position_m: Sequence[float],
    orientation_xyzw: Sequence[float],
    contract: Mapping[str, Any],
    context: str,
) -> tuple[tuple[tuple[float, float], ...], tuple[float, float], float]:
    """Derive the only accepted local route from map route and anchor pose."""
    # HH_260906 - Remove consecutive zero-length segments before arc interpolation.
    deduplicated_route: list[tuple[float, float]] = []
    for index, point in enumerate(route_points):
        if len(point) != 2:
            raise ContractError(f"{context} route point {index} must contain x and y")
        parsed_point = (float(point[0]), float(point[1]))
        if not all(math.isfinite(value) for value in parsed_point):
            raise ContractError(f"{context} route point {index} must be finite")
        if (
            not deduplicated_route
            or math.dist(deduplicated_route[-1], parsed_point) > 1.0e-9
        ):
            deduplicated_route.append(parsed_point)
    if len(deduplicated_route) < 2:
        raise ContractError(f"{context} route has no nonzero segment")
    route_points = tuple(deduplicated_route)
    cumulative = [0.0]
    for first, second in zip(route_points, route_points[1:]):
        cumulative.append(cumulative[-1] + math.dist(first, second))

    qx, qy, qz, qw = (float(value) for value in orientation_xyzw)
    forward_x = 1.0 - 2.0 * (qy * qy + qz * qz)
    forward_y = 2.0 * (qx * qy + qw * qz)
    forward_norm = math.hypot(forward_x, forward_y)
    if forward_norm < 1.0e-6:
        raise ContractError(f"{context} ego heading is undefined in the map plane")
    forward_x /= forward_norm
    forward_y /= forward_norm
    ego_x, ego_y = float(position_m[0]), float(position_m[1])

    best: tuple[tuple[float, float, int], float, tuple[float, float]] | None = None
    for segment_index, (first, second) in enumerate(
        zip(route_points, route_points[1:])
    ):
        delta_x = second[0] - first[0]
        delta_y = second[1] - first[1]
        length_squared = delta_x * delta_x + delta_y * delta_y
        if length_squared <= 0.0:
            continue
        ratio = max(
            0.0,
            min(
                1.0,
                ((ego_x - first[0]) * delta_x + (ego_y - first[1]) * delta_y)
                / length_squared,
            ),
        )
        projected = (first[0] + ratio * delta_x, first[1] + ratio * delta_y)
        distance_squared = (ego_x - projected[0]) ** 2 + (ego_y - projected[1]) ** 2
        segment_length = math.sqrt(length_squared)
        heading_alignment = (
            forward_x * delta_x + forward_y * delta_y
        ) / segment_length
        # Rounded distance makes an exact crossing deterministic by heading, then index.
        key = (round(distance_squared, 12), -heading_alignment, segment_index)
        anchor_arc = cumulative[segment_index] + ratio * segment_length
        if best is None or key < best[0]:
            best = (key, anchor_arc, projected)
    if best is None:
        raise ContractError(f"{context} route has no nonzero segment")

    maximum_anchor_distance = float(
        contract["navigation"]["maximum_anchor_distance_m"]
    )
    anchor_distance = math.sqrt(best[0][0])
    if anchor_distance > maximum_anchor_distance:
        raise ContractError(
            f"{context} ego is {anchor_distance:.3f} m from its declared route"
        )
    anchor_arc = best[1]
    total_arc = cumulative[-1]
    spacing = float(contract["navigation"]["route_resample_spacing_m"])
    lookahead = float(contract["navigation"]["route_lookahead_m"])
    end_arc = min(total_arc, anchor_arc + lookahead)
    remaining = end_arc - anchor_arc
    if remaining <= 1.0e-6:
        raise ContractError(f"{context} has no route remaining after the anchor")
    sample_arcs = [
        anchor_arc + index * spacing
        for index in range(int(math.floor(remaining / spacing)) + 1)
    ]
    if end_arc - sample_arcs[-1] > 1.0e-6:
        sample_arcs.append(end_arc)

    world_samples: list[tuple[float, float]] = []
    segment_index = 0
    for sample_arc in sample_arcs:
        while (
            segment_index + 1 < len(cumulative) - 1
            and cumulative[segment_index + 1] < sample_arc
        ):
            segment_index += 1
        segment_start = cumulative[segment_index]
        segment_end = cumulative[segment_index + 1]
        ratio = (sample_arc - segment_start) / (segment_end - segment_start)
        first = route_points[segment_index]
        second = route_points[segment_index + 1]
        world_samples.append(
            (
                first[0] + ratio * (second[0] - first[0]),
                first[1] + ratio * (second[1] - first[1]),
            )
        )

    def to_base(point: tuple[float, float]) -> tuple[float, float]:
        delta_x = point[0] - ego_x
        delta_y = point[1] - ego_y
        return (
            forward_x * delta_x + forward_y * delta_y,
            -forward_y * delta_x + forward_x * delta_y,
        )

    return tuple(to_base(point) for point in world_samples), to_base(route_points[-1]), anchor_arc


def _validate_source_manifest(
    document: Mapping[str, Any],
    *,
    episode: Mapping[str, Any],
    route_geometry_sha256: str,
    path: Path,
) -> dict[str, Any]:
    """Validate immutable raw-frame and selected-bundle provenance bindings."""
    context = str(path)
    if document.get("schema_id") != SOURCE_MANIFEST_SCHEMA_ID:
        raise ContractError(f"{context}.schema_id must be {SOURCE_MANIFEST_SCHEMA_ID}")
    if document.get("source_session_id") != episode["source_session_id"]:
        raise ContractError(f"{context}.source_session_id does not match the episode")
    metadata = {}
    for key in ("source_dataset_id", "source_dataset_version", "license_id"):
        metadata[key] = _string(document.get(key), f"{context}.{key}")

    artifacts = _sequence(document.get("source_artifacts"), f"{context}.source_artifacts")
    if not artifacts:
        raise ContractError(f"{context}.source_artifacts cannot be empty")
    artifact_keys: set[tuple[str, str]] = set()
    artifact_hashes: set[str] = set()
    for index, raw_artifact in enumerate(artifacts):
        artifact_context = f"{context}.source_artifacts[{index}]"
        artifact = _mapping(raw_artifact, artifact_context)
        uri = _string(artifact.get("uri"), f"{artifact_context}.uri")
        digest = _sha256(artifact.get("sha256"), f"{artifact_context}.sha256")
        key = (uri, digest)
        if key in artifact_keys:
            raise ContractError(f"{artifact_context} duplicates a source artifact")
        artifact_keys.add(key)
        artifact_hashes.add(digest)

    route_source = _mapping(document.get("route_source"), f"{context}.route_source")
    route_source_time = _integer(
        route_source.get("available_timestamp_ns"),
        f"{context}.route_source.available_timestamp_ns",
        0,
    )
    _string(route_source.get("uri"), f"{context}.route_source.uri")
    _sha256(
        route_source.get("source_payload_sha256"),
        f"{context}.route_source.source_payload_sha256",
    )
    route_artifact_hash = _sha256(
        route_source.get("source_artifact_sha256"),
        f"{context}.route_source.source_artifact_sha256",
    )
    if route_artifact_hash not in artifact_hashes:
        raise ContractError(
            f"{context}.route_source.source_artifact_sha256 is not declared"
        )
    if (
        _sha256(
            route_source.get("route_geometry_sha256"),
            f"{context}.route_source.route_geometry_sha256",
        )
        != route_geometry_sha256
    ):
        raise ContractError(
            f"{context}.route_source.route_geometry_sha256 does not match the episode"
        )

    raw_frames = _sequence(document.get("camera_frames"), f"{context}.camera_frames")
    if not raw_frames:
        raise ContractError(f"{context}.camera_frames cannot be empty")
    if len(raw_frames) > MAX_SOURCE_MANIFEST_FRAMES:
        raise ContractError(
            f"{context}.camera_frames exceeds {MAX_SOURCE_MANIFEST_FRAMES} entries"
        )
    frames: dict[tuple[str, str], dict[str, Any]] = {}
    per_camera: dict[str, list[tuple[int, int]]] = {
        name: [] for name in CAMERA_ORDER
    }
    for index, raw_frame in enumerate(raw_frames):
        frame_context = f"{context}.camera_frames[{index}]"
        frame = _mapping(raw_frame, frame_context)
        camera_name = _string(frame.get("camera_name"), f"{frame_context}.camera_name")
        if camera_name not in per_camera:
            raise ContractError(f"{frame_context}.camera_name is not in the camera ABI")
        frame_id = _string(frame.get("source_frame_id"), f"{frame_context}.source_frame_id")
        timestamp = _integer(
            frame.get("source_timestamp_ns"),
            f"{frame_context}.source_timestamp_ns",
            0,
        )
        sequence_index = _integer(
            frame.get("source_sequence_index"),
            f"{frame_context}.source_sequence_index",
            0,
        )
        _string(frame.get("source_uri"), f"{frame_context}.source_uri")
        source_hash = _sha256(
            frame.get("source_payload_sha256"),
            f"{frame_context}.source_payload_sha256",
        )
        source_artifact_hash = _sha256(
            frame.get("source_artifact_sha256"),
            f"{frame_context}.source_artifact_sha256",
        )
        if source_artifact_hash not in artifact_hashes:
            raise ContractError(
                f"{frame_context}.source_artifact_sha256 is not declared"
            )
        prepared_value = frame.get("prepared_payload_sha256")
        prepared_hash = (
            None
            if prepared_value is None
            else _sha256(prepared_value, f"{frame_context}.prepared_payload_sha256")
        )
        transform_id = _string(
            frame.get("image_transform_id"), f"{frame_context}.image_transform_id"
        )
        if transform_id not in ("identity_jpeg", "resize_rectify_rgb_to_jpeg_v1"):
            raise ContractError(f"{frame_context}.image_transform_id is not supported")
        if (
            transform_id == "identity_jpeg"
            and prepared_hash is not None
            and source_hash != prepared_hash
        ):
            raise ContractError(
                f"{frame_context} identity_jpeg hashes must be identical"
            )
        key = (camera_name, frame_id)
        if key in frames:
            raise ContractError(f"{frame_context} duplicates native frame {key!r}")
        frames[key] = {
            "source_timestamp_ns": timestamp,
            "source_sequence_index": sequence_index,
            "source_payload_sha256": source_hash,
            "prepared_payload_sha256": prepared_hash,
            "image_transform_id": transform_id,
        }
        per_camera[camera_name].append((sequence_index, timestamp))

    for camera_name, sequence in per_camera.items():
        if not sequence:
            raise ContractError(f"{context}.camera_frames has no {camera_name} entries")
        expected_indices = list(range(len(sequence)))
        actual_indices = [item[0] for item in sequence]
        if actual_indices != expected_indices:
            raise ContractError(
                f"{context} {camera_name} source_sequence_index must be ordered "
                "and contiguous from zero"
            )
        timestamps = [item[1] for item in sequence]
        if any(second <= first for first, second in zip(timestamps, timestamps[1:])):
            raise ContractError(
                f"{context} {camera_name} source timestamps must be strictly increasing"
            )

    selected_values = _sequence(
        document.get("selected_samples"), f"{context}.selected_samples"
    )
    if len(selected_values) != int(episode["sample_count"]):
        raise ContractError(
            f"{context}.selected_samples does not match episode.sample_count"
        )
    selected_samples: dict[str, dict[str, Any]] = {}
    for index, raw_selected in enumerate(selected_values):
        selected_context = f"{context}.selected_samples[{index}]"
        selected = _mapping(raw_selected, selected_context)
        sample_id = _string(
            selected.get("source_sample_id"), f"{selected_context}.source_sample_id"
        )
        if sample_id in selected_samples:
            raise ContractError(f"{selected_context} duplicates source_sample_id")
        timestamp = _integer(
            selected.get("anchor_timestamp_ns"),
            f"{selected_context}.anchor_timestamp_ns",
            0,
        )
        frame_ids = tuple(
            _string(value, f"{selected_context}.camera_source_frame_ids[{item_index}]")
            for item_index, value in enumerate(
                _sequence(
                    selected.get("camera_source_frame_ids"),
                    f"{selected_context}.camera_source_frame_ids",
                )
            )
        )
        if len(frame_ids) != len(CAMERA_ORDER):
            raise ContractError(
                f"{selected_context}.camera_source_frame_ids must contain six entries"
            )
        for camera_name, frame_id in zip(CAMERA_ORDER, frame_ids):
            if (camera_name, frame_id) not in frames:
                raise ContractError(
                    f"{selected_context} references an unknown native camera frame"
                )
        selected_samples[sample_id] = {
            "anchor_timestamp_ns": timestamp,
            "camera_source_frame_ids": frame_ids,
        }

    return {
        "metadata": metadata,
        "route_source_timestamp_ns": route_source_time,
        "frames": frames,
        "raw_camera_frame_counts": {
            name: len(values) for name, values in per_camera.items()
        },
        "selected_samples": selected_samples,
    }


def _percentile_99(values: Sequence[float]) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = max(0, math.ceil(0.99 * len(ordered)) - 1)
    return ordered[index]


def _rotation_determinant(values: Sequence[float]) -> float:
    a, b, c = values[0], values[1], values[2]
    d, e, f = values[4], values[5], values[6]
    g, h, i = values[8], values[9], values[10]
    return a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)


def _validate_rotation(values: Sequence[float], context: str) -> None:
    rows = (
        (values[0], values[1], values[2]),
        (values[4], values[5], values[6]),
        (values[8], values[9], values[10]),
    )
    for first_index, first in enumerate(rows):
        for second_index, second in enumerate(rows):
            dot = sum(left * right for left, right in zip(first, second))
            expected = 1.0 if first_index == second_index else 0.0
            if abs(dot - expected) > 1.0e-3:
                raise ContractError(f"{context} rotation is not orthonormal")
    if _rotation_determinant(values) <= 0.0:
        raise ContractError(f"{context} rotation must have positive determinant")


def load_contract(path: Path | str | None = None) -> dict[str, Any]:
    contract_path = DEFAULT_CONTRACT_PATH if path is None else Path(path).expanduser().resolve()
    contract, contract_hash = _read_json_and_sha256(contract_path)
    validate_contract(contract)
    contract["_path"] = str(contract_path)
    contract["_sha256"] = contract_hash
    contract["_resource_limits"] = _resource_limits_report()
    return contract


def validate_contract(contract: Mapping[str, Any]) -> None:
    if contract.get("schema_id") != CONTRACT_SCHEMA_ID or contract.get("schema_version") != 1:
        raise ContractError(f"contract must use {CONTRACT_SCHEMA_ID}")
    if contract.get("contract_id") != CONTRACT_ID:
        raise ContractError(f"contract_id must be {CONTRACT_ID}")
    if tuple(_sequence(contract.get("camera_order"), "contract.camera_order")) != CAMERA_ORDER:
        raise ContractError("contract.camera_order does not match the deployed VAD camera order")

    image = _mapping(contract.get("image"), "contract.image")
    if image.get("width_px") != 640 or image.get("height_px") != 360:
        raise ContractError("contract image size must be 640x360")
    expected_image = {
        "width_px": 640,
        "height_px": 360,
        "source_encoding": "bgr8",
        "storage_codec": "jpeg",
        "decoded_training_encoding": "rgb8",
        "rectified": True,
    }
    if dict(image) != expected_image:
        raise ContractError("contract image storage/decoding ABI is not common_10hz_v1")

    coordinates = _mapping(contract.get("coordinates"), "contract.coordinates")
    expected_coordinates = {
        "world_handedness": "right_handed",
        "map_frame": "map",
        "base_frame": "base_link",
        "base_axes": "x_forward_y_left_z_up",
        "camera_optical_axes": "x_right_y_down_z_forward",
        "extrinsic": "T_base_from_camera_row_major",
    }
    for key, expected in expected_coordinates.items():
        if coordinates.get(key) != expected:
            raise ContractError(f"contract.coordinates.{key} must be {expected!r}")

    capture = _mapping(contract.get("capture"), "contract.capture")
    for key in (
        "nominal_rate_hz",
        "minimum_effective_rate_hz",
        "minimum_episode_duration_s",
        "minimum_motion_distance_m",
        "minimum_median_period_ms",
        "maximum_median_period_ms",
        "maximum_p99_gap_ms",
        "maximum_absolute_gap_ms",
        "dataset_bundle_skew_ms",
        "runtime_bundle_skew_ms",
        "maximum_state_delta_ms",
        "maximum_command_age_ms",
        "minimum_bundle_coverage_percent",
        "maximum_count_imbalance_percent",
        "maximum_pose_step_m",
        "maximum_speed_consistency_error_mps",
    ):
        _number(capture.get(key), f"contract.capture.{key}", minimum=0.0)
    expected_capture = {
        "nominal_rate_hz": 10.0,
        "minimum_effective_rate_hz": 9.5,
        "minimum_episode_duration_s": 30.0,
        "minimum_motion_distance_m": 5.0,
        "minimum_median_period_ms": 80.0,
        "maximum_median_period_ms": 120.0,
        "maximum_p99_gap_ms": 150.0,
        "maximum_absolute_gap_ms": 250.0,
        "dataset_bundle_skew_ms": 20.0,
        "runtime_bundle_skew_ms": 1.0,
        "maximum_state_delta_ms": 100.0,
        "maximum_command_age_ms": 500.0,
        "minimum_bundle_coverage_percent": 99.0,
        "maximum_count_imbalance_percent": 1.0,
        "maximum_pose_step_m": 10.0,
        "maximum_speed_consistency_error_mps": 5.0,
    }
    if dict(capture) != expected_capture:
        raise ContractError("contract.capture thresholds are not common_10hz_v1")
    if float(capture["runtime_bundle_skew_ms"]) > float(capture["dataset_bundle_skew_ms"]):
        raise ContractError("runtime bundle tolerance cannot exceed dataset tolerance")
    if not 0.0 < float(capture["minimum_bundle_coverage_percent"]) <= 100.0:
        raise ContractError("minimum bundle coverage must be in (0, 100]")

    trajectory = _mapping(contract.get("trajectory"), "contract.trajectory")
    step = _number(trajectory.get("step_s"), "contract.trajectory.step_s", minimum=0.0)
    horizon = _number(
        trajectory.get("horizon_s"), "contract.trajectory.horizon_s", minimum=0.0
    )
    point_count = _integer(
        trajectory.get("point_count"), "contract.trajectory.point_count", minimum=1
    )
    if not math.isclose(step * point_count, horizon, rel_tol=0.0, abs_tol=1.0e-9):
        raise ContractError("trajectory step_s * point_count must equal horizon_s")
    if trajectory.get("frame") != "base_link_at_anchor":
        raise ContractError("trajectory frame must be base_link_at_anchor")
    if not math.isclose(
        _number(trajectory.get("first_horizon_s"), "contract.trajectory.first_horizon_s"),
        step,
        rel_tol=0.0,
        abs_tol=1.0e-9,
    ):
        raise ContractError("trajectory first_horizon_s must equal step_s")
    if trajectory.get("legacy_view_step_s") != 0.5:
        raise ContractError("trajectory legacy_view_step_s must be 0.5")
    if trajectory.get("legacy_view_horizon_s") != 3.0:
        raise ContractError("trajectory legacy_view_horizon_s must be 3.0")
    if trajectory.get("minimum_valid_point_percent") != 95.0:
        raise ContractError("trajectory minimum_valid_point_percent must be 95")
    if trajectory.get("invalid_value", "not-null") is not None:
        raise ContractError("invalid trajectory values must be represented as JSON null")
    expected_trajectory = {
        "frame": "base_link_at_anchor",
        "step_s": 0.1,
        "horizon_s": 6.4,
        "point_count": 64,
        "first_horizon_s": 0.1,
        "legacy_view_step_s": 0.5,
        "legacy_view_horizon_s": 3.0,
        "minimum_valid_point_percent": 95.0,
        "invalid_value": None,
    }
    if dict(trajectory) != expected_trajectory:
        raise ContractError("contract.trajectory is not common_10hz_v1")

    navigation = _mapping(contract.get("navigation"), "contract.navigation")
    expected_navigation = {
        "route_source": "episode_route_geometry",
        "route_resample_spacing_m": 1.0,
        "route_lookahead_m": 120.0,
        "maximum_anchor_distance_m": 5.0,
        "maximum_coordinate_error_m": 0.05,
    }
    if dict(navigation) != expected_navigation:
        raise ContractError("contract.navigation is not common_10hz_v1")

    if tuple(_sequence(contract.get("domains"), "contract.domains")) != ("carla", "real"):
        raise ContractError("contract.domains must be ['carla', 'real']")
    if tuple(_sequence(contract.get("splits"), "contract.splits")) != (
        "train",
        "val",
        "test",
    ):
        raise ContractError("contract.splits must be ['train', 'val', 'test']")
    if contract.get("split_policy") != "route_site_day_holdout_v1":
        raise ContractError("contract.split_policy must be route_site_day_holdout_v1")
    if contract.get("missing_label_policy") != "available_flag_and_valid_mask_never_zero_fill":
        raise ContractError("contract missing-label policy must reject fabricated zero labels")
    commands = _mapping(contract.get("commands"), "contract.commands")
    expected_commands = {
        "0": "left",
        "1": "right",
        "2": "straight",
        "3": "lane_follow",
        "4": "change_left",
        "5": "change_right",
    }
    if dict(commands) != expected_commands:
        raise ContractError("contract.commands must match the deployed VAD command enum")


def _validate_rig(rig: Mapping[str, Any], contract: Mapping[str, Any], path: Path) -> None:
    context = str(path)
    if rig.get("schema_id") != RIG_SCHEMA_ID:
        raise ContractError(f"{context}.schema_id must be {RIG_SCHEMA_ID}")
    rig_id = _string(rig.get("rig_id"), f"{context}.rig_id")
    domain = _string(rig.get("domain"), f"{context}.domain")
    if domain not in contract["domains"]:
        raise ContractError(f"{context}.domain is not allowed")
    if rig.get("base_frame") != contract["coordinates"]["base_frame"]:
        raise ContractError(f"{context}.base_frame must match the contract")
    if rig.get("rectified") is not True:
        raise ContractError(f"{context}.rectified must be true")
    if tuple(_sequence(rig.get("camera_order"), f"{context}.camera_order")) != CAMERA_ORDER:
        raise ContractError(f"{context}.camera_order is invalid")

    cameras = _sequence(rig.get("cameras"), f"{context}.cameras")
    if len(cameras) != len(CAMERA_ORDER):
        raise ContractError(f"{context}.cameras must contain exactly six entries")
    frames: set[str] = set()
    physical_ids: set[str] = set()
    for index, value in enumerate(cameras):
        camera_context = f"{context}.cameras[{index}]"
        camera = _mapping(value, camera_context)
        name = CAMERA_ORDER[index]
        if camera.get("name") != name or camera.get("model_index") != index:
            raise ContractError(f"{camera_context} must be {name} at model_index {index}")
        physical_id = _string(camera.get("physical_id"), f"{camera_context}.physical_id")
        optical_frame = _string(camera.get("optical_frame"), f"{camera_context}.optical_frame")
        if physical_id in physical_ids or optical_frame in frames:
            raise ContractError(f"{camera_context} repeats a physical ID or optical frame")
        physical_ids.add(physical_id)
        frames.add(optical_frame)
        if camera.get("width_px") != contract["image"]["width_px"]:
            raise ContractError(f"{camera_context}.width_px does not match the contract")
        if camera.get("height_px") != contract["image"]["height_px"]:
            raise ContractError(f"{camera_context}.height_px does not match the contract")
        if camera.get("source_encoding") != contract["image"]["source_encoding"]:
            raise ContractError(f"{camera_context}.source_encoding does not match the contract")
        if camera.get("storage_codec") != contract["image"]["storage_codec"]:
            raise ContractError(f"{camera_context}.storage_codec does not match the contract")
        intrinsics = _vector(camera.get("K"), 9, f"{camera_context}.K")
        if float(intrinsics[0]) <= 0.0 or float(intrinsics[4]) <= 0.0:
            raise ContractError(f"{camera_context}.K must have positive fx and fy")
        if (
            abs(float(intrinsics[1])) > 1.0e-9
            or abs(float(intrinsics[3])) > 1.0e-9
            or tuple(float(value) for value in intrinsics[6:]) != (0.0, 0.0, 1.0)
        ):
            raise ContractError(f"{camera_context}.K has an unsupported skew or bottom row")
        if not 0.0 <= float(intrinsics[2]) < float(camera["width_px"]):
            raise ContractError(f"{camera_context}.K cx is outside the image")
        if not 0.0 <= float(intrinsics[5]) < float(camera["height_px"]):
            raise ContractError(f"{camera_context}.K cy is outside the image")
        horizontal_fov = _number(
            camera.get("horizontal_fov_rad"), f"{camera_context}.horizontal_fov_rad", 0.0
        )
        expected_fov = 2.0 * math.atan(float(camera["width_px"]) / (2.0 * float(intrinsics[0])))
        if abs(horizontal_fov - expected_fov) > 0.05:
            raise ContractError(f"{camera_context}.horizontal_fov_rad disagrees with K")
        distortion = _sequence(camera.get("D"), f"{camera_context}.D")
        if any(abs(_number(item, f"{camera_context}.D")) > 1.0e-9 for item in distortion):
            raise ContractError(f"{camera_context}.D must be zero after rectification")
        transform = _vector(
            camera.get("T_base_from_camera"), 16, f"{camera_context}.T_base_from_camera"
        )
        matrix = tuple(float(item) for item in transform)
        if any(
            abs(actual - expected) > 1.0e-9
            for actual, expected in zip(matrix[12:], (0, 0, 0, 1))
        ):
            raise ContractError(f"{camera_context}.T_base_from_camera has an invalid bottom row")
        _validate_rotation(matrix, f"{camera_context}.T_base_from_camera")
        view_x, view_y, view_z = matrix[2], matrix[6], matrix[10]
        horizontal_norm = math.hypot(view_x, view_y)
        if horizontal_norm < 0.2 or abs(view_z) > 0.8:
            raise ContractError(f"{camera_context} optical forward axis has an invalid pitch")
        expected_yaw = {
            "CAM_FRONT": 0.0,
            "CAM_BACK": math.pi,
            "CAM_FRONT_LEFT": math.pi / 4.0,
            "CAM_BACK_LEFT": 3.0 * math.pi / 4.0,
            "CAM_FRONT_RIGHT": -math.pi / 4.0,
            "CAM_BACK_RIGHT": -3.0 * math.pi / 4.0,
        }[name]
        view_yaw = math.atan2(view_y, view_x)
        yaw_error = math.atan2(
            math.sin(view_yaw - expected_yaw), math.cos(view_yaw - expected_yaw)
        )
        if abs(yaw_error) > math.radians(60.0):
            raise ContractError(f"{camera_context} optical forward axis faces the wrong role")
        for key in ("timestamp_source", "trigger_mode", "firmware", "exposure_mode"):
            _string(camera.get(key), f"{camera_context}.{key}")
        if domain == "real" and physical_id.startswith("virtual:"):
            raise ContractError(f"{camera_context}.physical_id must identify real hardware")
    if not rig_id:
        raise ContractError(f"{context}.rig_id must be set")
    if domain == "real":
        data_kind = _string(rig.get("real_data_kind"), f"{context}.real_data_kind")
        if data_kind == "live_vehicle":
            commissioning = _mapping(
                rig.get("commissioning"), f"{context}.commissioning"
            )
            if commissioning.get("approved") is not True:
                raise ContractError(f"{context}.commissioning.approved must be true")
            valid_until = _utc_datetime(
                commissioning.get("valid_until_utc"),
                f"{context}.commissioning.valid_until_utc",
            )
            if valid_until <= datetime.now(timezone.utc):
                raise ContractError(f"{context}.commissioning approval is expired")
            _string(
                commissioning.get("ptp_clock_source"),
                f"{context}.commissioning.ptp_clock_source",
            )
            ptp_offset = _number(
                commissioning.get("maximum_observed_ptp_offset_ns"),
                f"{context}.commissioning.maximum_observed_ptp_offset_ns",
                0.0,
            )
            if ptp_offset > 100_000.0:
                raise ContractError(
                    f"{context}.commissioning PTP offset exceeds 100000 ns"
                )
            evidence = commissioning
            evidence_prefixes = ("calibration", "rectification", "commissioning")
            evidence_context = f"{context}.commissioning"
        elif data_kind == "recorded_dataset":
            evidence = _mapping(
                rig.get("recorded_dataset"), f"{context}.recorded_dataset"
            )
            evidence_prefixes = (
                "calibration",
                "rectification",
                "source_sensor_metadata",
            )
            evidence_context = f"{context}.recorded_dataset"
        else:
            raise ContractError(
                f"{context}.real_data_kind must be live_vehicle or recorded_dataset"
            )
        for prefix in evidence_prefixes:
            evidence_path = _safe_file(
                path.parent,
                evidence.get(f"{prefix}_file"),
                f"{evidence_context}.{prefix}_file",
            )
            expected_hash = _sha256(
                evidence.get(f"{prefix}_sha256"),
                f"{evidence_context}.{prefix}_sha256",
            )
            if sha256_file(evidence_path) != expected_hash:
                raise ContractError(f"{evidence_context}.{prefix}_sha256 mismatch")


def _iter_jsonl(
    path: Path, *, expected_sha256: str | None = None
) -> Iterable[tuple[int, dict[str, Any]]]:
    context = f"JSONL file {path}"
    descriptor, before = _open_regular_read_only(path, context)
    try:
        found = False
        digest = hashlib.sha256()
        try:
            line_number = 0
            with os.fdopen(descriptor, "rb", closefd=False) as stream:
                while True:
                    payload = stream.readline(MAX_JSONL_LINE_BYTES + 1)
                    if not payload:
                        break
                    line_number += 1
                    if len(payload) > MAX_JSONL_LINE_BYTES:
                        raise ContractError(
                            f"{path}:{line_number}: JSONL line exceeds limit "
                            f"{MAX_JSONL_LINE_BYTES} bytes"
                        )
                    digest.update(payload)
                    if not payload.strip():
                        continue
                    found = True
                    try:
                        line = payload.decode("utf-8")
                    except UnicodeDecodeError as error:
                        raise ContractError(
                            f"{path}:{line_number}: JSONL must be UTF-8: {error}"
                        ) from error
                    yield line_number, _loads_json(line, f"{path}:{line_number}")
            _verify_open_file_identity(descriptor, path, before, context)
        except MemoryError as error:
            raise ContractError(
                f"{context} could not be processed within the bounded memory budget"
            ) from error
        except OSError as error:
            raise ContractError(f"cannot read {path}: {error}") from error
        if not found:
            raise ContractError(f"{path} contains no samples")
        if expected_sha256 is not None and digest.hexdigest() != expected_sha256:
            raise ContractError(f"{path} SHA256 changed or does not match its manifest")
    finally:
        os.close(descriptor)


def _validate_file_reference(
    root: Path,
    value: Any,
    context: str,
) -> tuple[Path, dict[str, Any], str]:
    reference = _mapping(value, context)
    path = _safe_file(root, reference.get("manifest"), f"{context}.manifest")
    expected = _sha256(reference.get("sha256"), f"{context}.sha256")
    document, actual = _read_json_and_sha256(path)
    if actual != expected:
        raise ContractError(f"{context}.sha256 mismatch for {path}")
    return path, document, actual


def _validate_sample(
    sample: Mapping[str, Any],
    *,
    context: str,
    episode: Mapping[str, Any],
    episode_root: Path,
    rig: Mapping[str, Any],
    contract: Mapping[str, Any],
    route_geometry_sha256: str,
    route_geometry_points: Sequence[tuple[float, float]],
    source_manifest: Mapping[str, Any],
    expected_index: int,
    check_image_hashes: bool,
) -> dict[str, Any]:
    if sample.get("schema_id") != SAMPLE_SCHEMA_ID:
        raise ContractError(f"{context}.schema_id must be {SAMPLE_SCHEMA_ID}")
    sample_id = _string(sample.get("sample_id"), f"{context}.sample_id")
    source_sample_id = _string(sample.get("source_sample_id"), f"{context}.source_sample_id")
    if sample.get("episode_id") != episode["episode_id"]:
        raise ContractError(f"{context}.episode_id does not match its episode")
    if (
        _integer(sample.get("sequence_index"), f"{context}.sequence_index", minimum=0)
        != expected_index
    ):
        raise ContractError(f"{context}.sequence_index must be contiguous from zero")
    anchor = _integer(sample.get("anchor_timestamp_ns"), f"{context}.anchor_timestamp_ns", 0)
    if not int(episode["start_timestamp_ns"]) <= anchor <= int(episode["end_timestamp_ns"]):
        raise ContractError(f"{context}.anchor_timestamp_ns is outside the episode")
    if sample.get("rig_id") != rig["rig_id"]:
        raise ContractError(f"{context}.rig_id does not match the episode rig")

    bundle = _sequence(sample.get("camera_bundle"), f"{context}.camera_bundle")
    if len(bundle) != len(CAMERA_ORDER):
        raise ContractError(f"{context}.camera_bundle must contain exactly six cameras")
    timestamps: list[int] = []
    source_timestamps: list[int] = []
    source_frame_ids: list[str] = []
    frame_counters: list[int] = []
    image_hashes: list[str] = []
    image_scan_hashes: list[str] = []
    for index, raw_camera in enumerate(bundle):
        camera_context = f"{context}.camera_bundle[{index}]"
        camera = _mapping(raw_camera, camera_context)
        rig_camera = rig["cameras"][index]
        if camera.get("name") != CAMERA_ORDER[index] or camera.get("model_index") != index:
            raise ContractError(f"{camera_context} violates the fixed camera order")
        if camera.get("frame_id") != rig_camera["optical_frame"]:
            raise ContractError(f"{camera_context}.frame_id does not match the rig")
        camera_time = _integer(camera.get("timestamp_ns"), f"{camera_context}.timestamp_ns", 0)
        if not int(episode["start_timestamp_ns"]) <= camera_time <= int(
            episode["end_timestamp_ns"]
        ):
            raise ContractError(f"{camera_context}.timestamp_ns is outside the episode")
        source_time = _integer(
            camera.get("source_timestamp_ns"),
            f"{camera_context}.source_timestamp_ns",
            0,
        )
        source_frame_id = _string(
            camera.get("source_frame_id"), f"{camera_context}.source_frame_id"
        )
        frame_counter = _integer(
            camera.get("frame_counter"), f"{camera_context}.frame_counter", 0
        )
        image_path = _safe_file(episode_root, camera.get("path"), f"{camera_context}.path")
        if image_path.suffix.lower() not in (".jpg", ".jpeg"):
            raise ContractError(f"{camera_context}.path must use JPEG storage")
        width, height, components, actual_hash, scan_hash = _jpeg_dimensions(
            image_path, f"{camera_context}.path"
        )
        if (width, height, components) != (
            int(contract["image"]["width_px"]),
            int(contract["image"]["height_px"]),
            3,
        ):
            raise ContractError(
                f"{camera_context}.path must contain a 640x360 three-component JPEG"
            )
        declared_hash = _sha256(camera.get("sha256"), f"{camera_context}.sha256")
        if check_image_hashes and actual_hash != declared_hash:
            raise ContractError(f"{camera_context}.sha256 mismatch for {image_path}")
        if camera_time != source_time:
            raise ContractError(
                f"{camera_context}.timestamp_ns must equal the native source timestamp"
            )
        source_frame = source_manifest["frames"].get(
            (CAMERA_ORDER[index], source_frame_id)
        )
        if source_frame is None:
            raise ContractError(f"{camera_context} is absent from the source manifest")
        if int(source_frame["source_timestamp_ns"]) != source_time:
            raise ContractError(
                f"{camera_context}.source_timestamp_ns does not match the source manifest"
            )
        if int(source_frame["source_sequence_index"]) != frame_counter:
            raise ContractError(
                f"{camera_context}.frame_counter does not match the source manifest"
            )
        if source_frame["prepared_payload_sha256"] != declared_hash:
            raise ContractError(
                f"{camera_context}.sha256 does not match its source-manifest derivation"
            )
        timestamps.append(camera_time)
        source_timestamps.append(source_time)
        source_frame_ids.append(source_frame_id)
        frame_counters.append(frame_counter)
        image_hashes.append(declared_hash)
        image_scan_hashes.append(scan_hash)
    bundle_median = median(timestamps)
    if abs(float(anchor) - float(bundle_median)) > 1.0:
        raise ContractError(f"{context}.anchor_timestamp_ns must equal the camera median")
    selected_source = source_manifest["selected_samples"].get(source_sample_id)
    if selected_source is None:
        raise ContractError(f"{context}.source_sample_id is absent from the source manifest")
    if int(selected_source["anchor_timestamp_ns"]) != anchor:
        raise ContractError(
            f"{context}.anchor_timestamp_ns does not match the source manifest"
        )
    if tuple(selected_source["camera_source_frame_ids"]) != tuple(source_frame_ids):
        raise ContractError(
            f"{context}.camera_bundle does not match its source-manifest selection"
        )
    bundle_skew_ms = (max(timestamps) - min(timestamps)) * 1.0e-6
    if bundle_skew_ms > float(contract["capture"]["dataset_bundle_skew_ms"]):
        raise ContractError(f"{context} camera bundle exceeds dataset skew tolerance")

    ego = _mapping(sample.get("ego"), f"{context}.ego")
    ego_time = _integer(ego.get("timestamp_ns"), f"{context}.ego.timestamp_ns", 0)
    if ego_time > anchor:
        raise ContractError(f"{context}.ego timestamp is in the future")
    state_delta_ms = (anchor - ego_time) * 1.0e-6
    if state_delta_ms > float(contract["capture"]["maximum_state_delta_ms"]):
        raise ContractError(f"{context}.ego timestamp exceeds state tolerance")
    if ego.get("frame_id") != contract["coordinates"]["map_frame"]:
        raise ContractError(f"{context}.ego.frame_id must be map")
    if ego.get("child_frame_id") != contract["coordinates"]["base_frame"]:
        raise ContractError(f"{context}.ego.child_frame_id must be base_link")
    position = _vector(ego.get("position_m"), 3, f"{context}.ego.position_m")
    quaternion = _vector(ego.get("orientation_xyzw"), 4, f"{context}.ego.orientation_xyzw")
    quaternion_norm = math.sqrt(sum(float(item) ** 2 for item in quaternion))
    if abs(quaternion_norm - 1.0) > 1.0e-3:
        raise ContractError(f"{context}.ego.orientation_xyzw must be normalized")
    velocity = _vector(
        ego.get("linear_velocity_base_mps"), 3, f"{context}.ego.linear_velocity_base_mps"
    )
    _vector(
        ego.get("angular_velocity_base_radps"),
        3,
        f"{context}.ego.angular_velocity_base_radps",
    )
    _vector(
        ego.get("linear_acceleration_base_mps2"),
        3,
        f"{context}.ego.linear_acceleration_base_mps2",
    )
    steering = _number(
        ego.get("steering_tire_angle_rad"), f"{context}.ego.steering_tire_angle_rad"
    )

    navigation = _mapping(sample.get("navigation"), f"{context}.navigation")
    if navigation.get("route_id") != episode["route_id"]:
        raise ContractError(f"{context}.navigation.route_id does not match the episode")
    if navigation.get("route_source") != contract["navigation"]["route_source"]:
        raise ContractError(
            f"{context}.navigation.route_source must be episode_route_geometry"
        )
    if (
        _sha256(
            navigation.get("route_geometry_sha256"),
            f"{context}.navigation.route_geometry_sha256",
        )
        != route_geometry_sha256
    ):
        raise ContractError(
            f"{context}.navigation.route_geometry_sha256 does not match the episode"
        )
    route_source_time = _integer(
        navigation.get("route_source_timestamp_ns"),
        f"{context}.navigation.route_source_timestamp_ns",
        0,
    )
    if route_source_time > anchor:
        raise ContractError(f"{context}.navigation route source timestamp is in the future")
    if route_source_time != int(source_manifest["route_source_timestamp_ns"]):
        raise ContractError(
            f"{context}.navigation route source timestamp does not match the source manifest"
        )
    command = _integer(navigation.get("command"), f"{context}.navigation.command", 0)
    if str(command) not in contract["commands"]:
        raise ContractError(f"{context}.navigation.command must be in [0, 5]")
    command_time = _integer(
        navigation.get("command_timestamp_ns"), f"{context}.navigation.command_timestamp_ns", 0
    )
    if command_time > anchor:
        raise ContractError(f"{context}.navigation command timestamp is in the future")
    command_age_ms = (anchor - command_time) * 1.0e-6
    if command_age_ms > float(contract["capture"]["maximum_command_age_ms"]):
        raise ContractError(f"{context}.navigation command is stale")
    _number(navigation.get("speed_limit_mps"), f"{context}.navigation.speed_limit_mps", 0.0)
    route_values = _sequence(
        navigation.get("route_polyline_base_m"), f"{context}.navigation.route_polyline_base_m"
    )
    if len(route_values) < 2:
        raise ContractError(f"{context}.navigation.route_polyline_base_m needs two points")
    route = tuple(
        tuple(
            float(value)
            for value in _vector(
                point,
                2,
                f"{context}.navigation.route_polyline_base_m[{index}]",
            )
        )
        for index, point in enumerate(route_values)
    )
    goal = tuple(
        float(value)
        for value in _vector(
            navigation.get("goal_base_m"), 2, f"{context}.navigation.goal_base_m"
        )
    )
    expected_route, expected_goal, expected_anchor_arc = _canonical_route_in_base(
        route_geometry_points,
        position_m=position,
        orientation_xyzw=quaternion,
        contract=contract,
        context=f"{context}.navigation",
    )
    coordinate_tolerance = float(
        contract["navigation"]["maximum_coordinate_error_m"]
    )
    if len(route) != len(expected_route) or any(
        math.dist(actual, expected) > coordinate_tolerance
        for actual, expected in zip(route, expected_route)
    ):
        raise ContractError(
            f"{context}.navigation.route_polyline_base_m is not the canonical "
            "causal projection of the episode route"
        )
    if math.dist(goal, expected_goal) > coordinate_tolerance:
        raise ContractError(
            f"{context}.navigation.goal_base_m does not match the mission route endpoint"
        )
    declared_anchor_arc = _number(
        navigation.get("route_anchor_arc_m"),
        f"{context}.navigation.route_anchor_arc_m",
        0.0,
    )
    if abs(declared_anchor_arc - expected_anchor_arc) > coordinate_tolerance:
        raise ContractError(
            f"{context}.navigation.route_anchor_arc_m does not match the anchor projection"
        )

    labels = _mapping(sample.get("labels"), f"{context}.labels")
    planning = _mapping(labels.get("planning"), f"{context}.labels.planning")
    if planning.get("available") is not True:
        raise ContractError(f"{context}.labels.planning.available must be true")
    step_s = _number(planning.get("dt_s"), f"{context}.labels.planning.dt_s", 0.0)
    if not math.isclose(step_s, float(contract["trajectory"]["step_s"]), abs_tol=1.0e-9):
        raise ContractError(f"{context}.labels.planning.dt_s does not match the contract")
    point_count = int(contract["trajectory"]["point_count"])
    positions = _sequence(
        planning.get("positions_base_xy_m"), f"{context}.labels.planning.positions_base_xy_m"
    )
    yaws = _sequence(planning.get("yaw_rad"), f"{context}.labels.planning.yaw_rad")
    speeds = _sequence(planning.get("speed_mps"), f"{context}.labels.planning.speed_mps")
    valid = _sequence(planning.get("valid"), f"{context}.labels.planning.valid")
    target_times = _sequence(
        planning.get("target_timestamp_ns"), f"{context}.labels.planning.target_timestamp_ns"
    )
    invalid_reasons = _sequence(
        planning.get("invalid_reason"), f"{context}.labels.planning.invalid_reason"
    )
    arrays = (positions, yaws, speeds, valid, target_times, invalid_reasons)
    if any(len(items) != point_count for items in arrays):
        raise ContractError(f"{context}.labels.planning arrays must contain {point_count} points")
    valid_count = 0
    invalid_tail_started = False
    for index, flag_value in enumerate(valid):
        flag = _boolean(flag_value, f"{context}.labels.planning.valid[{index}]")
        target_time = anchor + int(round((index + 1) * step_s * 1.0e9))
        if flag:
            if invalid_tail_started:
                raise ContractError(f"{context}.labels.planning valid mask must be a prefix")
            _vector(positions[index], 2, f"{context}.labels.planning.positions[{index}]")
            _number(yaws[index], f"{context}.labels.planning.yaw_rad[{index}]")
            _number(speeds[index], f"{context}.labels.planning.speed_mps[{index}]", 0.0)
            target_timestamp = _integer(
                target_times[index], f"{context}.labels.planning.target_timestamp_ns[{index}]", 0
            )
            if target_timestamp != target_time:
                raise ContractError(f"{context}.labels.planning target timestamp is off-grid")
            if target_timestamp > int(episode["end_timestamp_ns"]):
                raise ContractError(f"{context}.labels.planning crosses the episode boundary")
            if invalid_reasons[index] is not None:
                raise ContractError(f"{context}.labels.planning valid target has an invalid reason")
            valid_count += 1
        else:
            invalid_tail_started = True
            if positions[index] != [None, None] or yaws[index] is not None:
                raise ContractError(f"{context}.labels.planning invalid targets must be null")
            if speeds[index] is not None or target_times[index] is not None:
                raise ContractError(f"{context}.labels.planning invalid metadata must be null")
            if invalid_reasons[index] not in ("episode_end", "sensor_gap", "label_unavailable"):
                raise ContractError(f"{context}.labels.planning invalid target needs a reason")
    if valid_count == 0:
        raise ContractError(f"{context}.labels.planning must contain a valid target")
    for task in ("objects", "occupancy"):
        task_value = _mapping(labels.get(task), f"{context}.labels.{task}")
        available = _boolean(task_value.get("available"), f"{context}.labels.{task}.available")
        if available:
            raise ContractError(
                f"{context}.labels.{task} cannot be marked available before "
                "its payload schema exists"
            )

    events = _mapping(sample.get("events"), f"{context}.events")
    for key in ("collision", "lane_invasion", "manual_intervention", "fallback_active"):
        _boolean(events.get(key), f"{context}.events.{key}")

    speed_mps = math.sqrt(sum(float(item) ** 2 for item in velocity))
    return {
        "sample_id": sample_id,
        "source_sample_id": source_sample_id,
        "anchor_timestamp_ns": anchor,
        "position_m": tuple(float(item) for item in position),
        "speed_mps": speed_mps,
        "steering_tire_angle_rad": steering,
        "command": command,
        "bundle_skew_ms": bundle_skew_ms,
        "state_delta_ms": state_delta_ms,
        "command_age_ms": command_age_ms,
        "route_anchor_arc_m": expected_anchor_arc,
        "image_hashes": image_hashes,
        "image_scan_hashes": image_scan_hashes,
        "camera_timestamps_ns": timestamps,
        "camera_source_timestamps_ns": source_timestamps,
        "camera_source_frame_ids": source_frame_ids,
        "camera_frame_counters": frame_counters,
        "valid_future_points": valid_count,
    }


def _claim_split(
    claims: dict[str, dict[str, str]],
    category: str,
    identifier: str,
    split: str,
) -> None:
    previous = claims.setdefault(category, {}).get(identifier)
    if previous is not None and previous != split:
        raise ContractError(
            f"split leakage: {category} {identifier!r} appears in both {previous} and {split}"
        )
    claims[category][identifier] = split


def validate_dataset(
    root: Path | str,
    *,
    contract_path: Path | str | None = None,
    contract: Mapping[str, Any] | None = None,
    mode: str = "planning",
    check_image_hashes: bool = True,
) -> dict[str, Any]:
    """Validate a common10 dataset and return an auditable summary.

    ``planning`` enforces the offline 10 Hz training-data gates. ``runtime``
    additionally requires 1 ms offline bundle readiness. The explicit
    ``schema`` mode is only for tiny fixtures and adapter development.
    """
    if mode not in ("planning", "runtime", "schema"):
        raise ContractError("mode must be 'planning', 'runtime', or 'schema'")
    if contract is not None and contract_path is not None:
        raise ContractError("provide contract or contract_path, not both")
    loaded_contract = dict(contract) if contract is not None else load_contract(contract_path)
    validate_contract(loaded_contract)

    dataset_root = Path(root).expanduser().resolve()
    if not dataset_root.is_dir():
        raise ContractError(f"dataset root is not a directory: {dataset_root}")
    manifest_path = dataset_root / "dataset.json"
    manifest, manifest_hash = _read_json_and_sha256(manifest_path)
    if manifest.get("schema_id") != DATASET_SCHEMA_ID:
        raise ContractError(f"dataset.schema_id must be {DATASET_SCHEMA_ID}")
    if manifest.get("contract_id") != CONTRACT_ID:
        raise ContractError(f"dataset.contract_id must be {CONTRACT_ID}")
    effective_contract_hash = contract_fingerprint(loaded_contract)
    if (
        _sha256(manifest.get("contract_sha256"), "dataset.contract_sha256")
        != effective_contract_hash
    ):
        raise ContractError("dataset.contract_sha256 does not match the effective contract")
    dataset_id = _string(manifest.get("dataset_id"), "dataset.dataset_id")
    _utc_datetime(manifest.get("created_at_utc"), "dataset.created_at_utc")
    if tuple(_sequence(manifest.get("camera_order"), "dataset.camera_order")) != CAMERA_ORDER:
        raise ContractError("dataset.camera_order does not match the contract")
    if manifest.get("split_policy") != loaded_contract["split_policy"]:
        raise ContractError("dataset.split_policy does not match the contract")

    rig_references = _sequence(manifest.get("rigs"), "dataset.rigs")
    if not rig_references:
        raise ContractError("dataset.rigs cannot be empty")
    rigs: dict[str, Mapping[str, Any]] = {}
    rig_hashes: list[str] = []
    for index, reference in enumerate(rig_references):
        context = f"dataset.rigs[{index}]"
        path, rig, rig_hash = _validate_file_reference(dataset_root, reference, context)
        _validate_rig(rig, loaded_contract, path)
        rig_id = str(rig["rig_id"])
        if rig_id != _string(
            _mapping(reference, context).get("rig_id"), f"{context}.rig_id"
        ):
            raise ContractError(f"{context}.rig_id does not match its manifest")
        if rig_id in rigs:
            raise ContractError(f"duplicate rig_id: {rig_id}")
        rigs[rig_id] = rig
        rig_hashes.append(rig_hash)

    episode_references = _sequence(manifest.get("episodes"), "dataset.episodes")
    if not episode_references:
        raise ContractError("dataset.episodes cannot be empty")
    split_claims: dict[str, dict[str, str]] = {}
    sample_ids: set[str] = set()
    image_split_by_hash: dict[str, str] = {}
    source_split_by_id: dict[str, str] = {}
    source_frame_owners: dict[tuple[str, str, str], str] = {}
    split_counts: Counter[str] = Counter()
    domain_counts: Counter[str] = Counter()
    scenario_counts: Counter[str] = Counter()
    episode_reports: list[dict[str, Any]] = []
    episode_ids: set[str] = set()
    dataset_fingerprint = hashlib.sha256()
    dataset_fingerprint.update(manifest_hash.encode("ascii"))
    for digest in rig_hashes:
        dataset_fingerprint.update(digest.encode("ascii"))

    for reference_index, reference in enumerate(episode_references):
        reference_context = f"dataset.episodes[{reference_index}]"
        episode_path, episode, episode_manifest_hash = _validate_file_reference(
            dataset_root, reference, reference_context
        )
        episode_root = episode_path.parent
        if episode.get("schema_id") != EPISODE_SCHEMA_ID:
            raise ContractError(f"{episode_path}.schema_id must be {EPISODE_SCHEMA_ID}")
        episode_id = _string(episode.get("episode_id"), f"{episode_path}.episode_id")
        if episode_id in episode_ids:
            raise ContractError(f"duplicate episode_id: {episode_id}")
        episode_ids.add(episode_id)
        if episode_id != _string(
            _mapping(reference, reference_context).get("episode_id"),
            f"{reference_context}.episode_id",
        ):
            raise ContractError(f"{reference_context}.episode_id does not match its manifest")
        domain = _string(episode.get("domain"), f"{episode_path}.domain")
        split = _string(episode.get("split"), f"{episode_path}.split")
        if domain not in loaded_contract["domains"] or split not in loaded_contract["splits"]:
            raise ContractError(f"{episode_path} has an unsupported domain or split")
        rig_id = _string(episode.get("rig_id"), f"{episode_path}.rig_id")
        if rig_id not in rigs or rigs[rig_id]["domain"] != domain:
            raise ContractError(f"{episode_path}.rig_id is unknown or has the wrong domain")
        for key in (
            "source_session_id",
            "split_group_id",
            "scene_group_id",
            "vehicle_id",
            "site_id",
            "map_id",
            "route_id",
            "weather",
            "clock_domain",
        ):
            _string(episode.get(key), f"{episode_path}.{key}")
        scenario_tags = _sequence(episode.get("scenario_tags"), f"{episode_path}.scenario_tags")
        if not scenario_tags:
            raise ContractError(f"{episode_path}.scenario_tags cannot be empty")
        for index, tag in enumerate(scenario_tags):
            _string(tag, f"{episode_path}.scenario_tags[{index}]")
        start = _integer(
            episode.get("start_timestamp_ns"), f"{episode_path}.start_timestamp_ns", 0
        )
        end = _integer(episode.get("end_timestamp_ns"), f"{episode_path}.end_timestamp_ns", 0)
        if end <= start:
            raise ContractError(f"{episode_path} must have a positive duration")
        declared_count = _integer(
            episode.get("sample_count"), f"{episode_path}.sample_count", 1
        )
        provenance = _mapping(
            episode.get("source_provenance"), f"{episode_path}.source_provenance"
        )
        for key in ("adapter_id", "git_commit", "source_manifest_file", "collection_config_file"):
            _string(provenance.get(key), f"{episode_path}.source_provenance.{key}")
        if re.fullmatch(r"[0-9a-f]{40}", str(provenance["git_commit"])) is None:
            raise ContractError(f"{episode_path}.source_provenance.git_commit must be 40 hex")
        provenance_files: dict[str, tuple[Path, str]] = {}
        for prefix in ("source_manifest", "collection_config"):
            source_path = _safe_file(
                episode_root,
                provenance.get(f"{prefix}_file"),
                f"{episode_path}.source_provenance.{prefix}_file",
            )
            source_hash = _sha256(
                provenance.get(f"{prefix}_sha256"),
                f"{episode_path}.source_provenance.{prefix}_sha256",
            )
            if sha256_file(source_path) != source_hash:
                raise ContractError(
                    f"{episode_path}.source_provenance.{prefix}_sha256 mismatch"
                )
            provenance_files[prefix] = (source_path, source_hash)
            dataset_fingerprint.update(source_hash.encode("ascii"))

        route_geometry_path = _safe_file(
            episode_root,
            episode.get("route_geometry_file"),
            f"{episode_path}.route_geometry_file",
        )
        route_geometry_hash = _sha256(
            episode.get("route_geometry_sha256"), f"{episode_path}.route_geometry_sha256"
        )
        route_semantic_hash = _route_geometry_semantic_hash(
            route_geometry_path,
            episode=episode,
            contract=loaded_contract,
            expected_sha256=route_geometry_hash,
        )
        route_geometry_points = _validated_route_points(
            route_geometry_path,
            expected_sha256=route_geometry_hash,
        )
        source_manifest_document, source_manifest_actual_hash = _read_json_and_sha256(
            provenance_files["source_manifest"][0]
        )
        if source_manifest_actual_hash != provenance_files["source_manifest"][1]:
            raise ContractError(
                f"{episode_path}.source_provenance.source_manifest_sha256 mismatch"
            )
        source_manifest = _validate_source_manifest(
            source_manifest_document,
            episode=episode,
            route_geometry_sha256=route_geometry_hash,
            path=provenance_files["source_manifest"][0],
        )
        if domain == "carla":
            traffic_seed = _integer(
                episode.get("traffic_seed"), f"{episode_path}.traffic_seed", 0
            )
            derived_split_group = f"carla:{episode['map_id']}:{route_semantic_hash}"
            derived_scene_group = (
                f"{derived_split_group}:traffic={traffic_seed}:weather={episode['weather']}"
            )
            _claim_split(split_claims, "carla_route_geometry", derived_split_group, split)
        else:
            collection_date = _string(
                episode.get("collection_date"), f"{episode_path}.collection_date"
            )
            if re.fullmatch(r"\d{4}-\d{2}-\d{2}", collection_date) is None:
                raise ContractError(f"{episode_path}.collection_date must use YYYY-MM-DD")
            continuous_drive_id = _string(
                episode.get("continuous_drive_id"), f"{episode_path}.continuous_drive_id"
            )
            derived_split_group = (
                f"real:{episode['site_id']}:{collection_date}:{continuous_drive_id}"
            )
            derived_scene_group = f"{derived_split_group}:route={route_semantic_hash}"
            _claim_split(split_claims, "real_site_day_drive", derived_split_group, split)
            _claim_split(
                split_claims,
                "real_site_day",
                f"{episode['site_id']}:{collection_date}",
                split,
            )
            _claim_split(
                split_claims,
                "real_route_geometry",
                f"{episode['map_id']}:{route_semantic_hash}",
                split,
            )
        if episode["split_group_id"] != derived_split_group:
            raise ContractError(
                f"{episode_path}.split_group_id must be derived from immutable episode fields"
            )
        if episode["scene_group_id"] != derived_scene_group:
            raise ContractError(
                f"{episode_path}.scene_group_id must be derived from immutable episode fields"
            )

        for category in ("episode_id", "source_session_id", "split_group_id", "scene_group_id"):
            _claim_split(split_claims, category, str(episode[category]), split)

        samples_path = _safe_file(
            episode_root, episode.get("sample_jsonl"), f"{episode_path}.sample_jsonl"
        )
        expected_samples_hash = _sha256(
            episode.get("sample_jsonl_sha256"), f"{episode_path}.sample_jsonl_sha256"
        )
        dataset_fingerprint.update(episode_manifest_hash.encode("ascii"))
        dataset_fingerprint.update(route_geometry_hash.encode("ascii"))
        dataset_fingerprint.update(route_semantic_hash.encode("ascii"))

        anchors: list[int] = []
        positions: list[tuple[float, float, float]] = []
        bundle_skews: list[float] = []
        state_deltas: list[float] = []
        command_ages: list[float] = []
        speeds_mps: list[float] = []
        camera_times: list[list[int]] = [[] for _ in CAMERA_ORDER]
        camera_source_times: list[list[int]] = [[] for _ in CAMERA_ORDER]
        camera_counters: list[list[int]] = [[] for _ in CAMERA_ORDER]
        camera_scan_hashes: list[list[str]] = [[] for _ in CAMERA_ORDER]
        cross_camera_duplicate_bundle_count = 0
        valid_future_points = 0
        command_counts: Counter[int] = Counter()
        sample_count = 0
        for line_number, sample in _iter_jsonl(
            samples_path, expected_sha256=expected_samples_hash
        ):
            sample_context = f"{samples_path}:{line_number}"
            summary = _validate_sample(
                sample,
                context=sample_context,
                episode=episode,
                episode_root=episode_root,
                rig=rigs[rig_id],
                contract=loaded_contract,
                route_geometry_sha256=route_geometry_hash,
                route_geometry_points=route_geometry_points,
                source_manifest=source_manifest,
                expected_index=sample_count,
                check_image_hashes=check_image_hashes,
            )
            if summary["sample_id"] in sample_ids:
                raise ContractError(f"duplicate sample_id: {summary['sample_id']}")
            sample_ids.add(summary["sample_id"])
            source_id = str(summary["source_sample_id"])
            previous_source_split = source_split_by_id.get(source_id)
            if previous_source_split is not None:
                if previous_source_split != split:
                    raise ContractError(
                        f"split leakage: source sample {source_id!r} crosses splits"
                    )
                raise ContractError(f"duplicate source sample: {source_id!r}")
            source_split_by_id[source_id] = split
            for image_hash in summary["image_hashes"]:
                previous_image_split = image_split_by_hash.get(image_hash)
                if previous_image_split is not None and previous_image_split != split:
                    raise ContractError(
                        f"split leakage: image SHA256 {image_hash} crosses splits"
                    )
                image_split_by_hash[image_hash] = split
            anchors.append(int(summary["anchor_timestamp_ns"]))
            positions.append(summary["position_m"])
            bundle_skews.append(float(summary["bundle_skew_ms"]))
            state_deltas.append(float(summary["state_delta_ms"]))
            command_ages.append(float(summary["command_age_ms"]))
            speeds_mps.append(float(summary["speed_mps"]))
            scan_hashes = summary["image_scan_hashes"]
            if len(set(scan_hashes)) != len(CAMERA_ORDER):
                cross_camera_duplicate_bundle_count += 1
            for camera_index in range(len(CAMERA_ORDER)):
                source_frame_id = summary["camera_source_frame_ids"][camera_index]
                source_frame_key = (
                    str(episode["source_session_id"]),
                    CAMERA_ORDER[camera_index],
                    source_frame_id,
                )
                previous_source_frame = source_frame_owners.get(source_frame_key)
                if previous_source_frame is not None:
                    raise ContractError(
                        "duplicate native source frame "
                        f"{source_frame_key!r} in {previous_source_frame} and "
                        f"{summary['sample_id']}"
                    )
                source_frame_owners[source_frame_key] = str(summary["sample_id"])
                camera_times[camera_index].append(summary["camera_timestamps_ns"][camera_index])
                camera_source_times[camera_index].append(
                    summary["camera_source_timestamps_ns"][camera_index]
                )
                camera_counters[camera_index].append(
                    summary["camera_frame_counters"][camera_index]
                )
                camera_scan_hashes[camera_index].append(scan_hashes[camera_index])
            valid_future_points += int(summary["valid_future_points"])
            command_counts[int(summary["command"])] += 1
            sample_count += 1

        dataset_fingerprint.update(expected_samples_hash.encode("ascii"))

        if sample_count != declared_count:
            raise ContractError(
                f"{episode_path}.sample_count declares {declared_count}, found {sample_count}"
            )
        if any(second <= first for first, second in zip(anchors, anchors[1:])):
            raise ContractError(f"{episode_path} sample timestamps must be strictly increasing")
        for camera_index, name in enumerate(CAMERA_ORDER):
            if any(
                second <= first
                for first, second in zip(
                    camera_times[camera_index], camera_times[camera_index][1:]
                )
            ):
                raise ContractError(f"{episode_path} {name} timestamps must be strictly increasing")
            if any(
                second <= first
                for first, second in zip(
                    camera_counters[camera_index], camera_counters[camera_index][1:]
                )
            ):
                raise ContractError(f"{episode_path} {name} frame counters must be increasing")
            if any(
                second <= first
                for first, second in zip(
                    camera_source_times[camera_index],
                    camera_source_times[camera_index][1:],
                )
            ):
                raise ContractError(
                    f"{episode_path} {name} native source timestamps must be strictly increasing"
                )
        gaps_ms = [
            (second - first) * 1.0e-6 for first, second in zip(anchors, anchors[1:])
        ]
        duration_s = (anchors[-1] - anchors[0]) * 1.0e-9 if len(anchors) > 1 else 0.0
        effective_rate_hz = (len(anchors) - 1) / duration_s if duration_s > 0.0 else 0.0
        median_period_ms = float(median(gaps_ms)) if gaps_ms else 0.0
        p99_gap_ms = _percentile_99(gaps_ms)
        maximum_gap_ms = max(gaps_ms, default=0.0)
        motion_distance_m = sum(
            math.dist(first, second) for first, second in zip(positions, positions[1:])
        )
        pose_steps_m = [
            math.dist(first, second) for first, second in zip(positions, positions[1:])
        ]
        speed_consistency_errors = [
            abs(
                distance / (gap_ms * 1.0e-3)
                - 0.5 * (first_speed + second_speed)
            )
            for distance, gap_ms, first_speed, second_speed in zip(
                pose_steps_m, gaps_ms, speeds_mps, speeds_mps[1:]
            )
            if gap_ms > 0.0
        ]
        source_cadence = []
        for source_times in camera_source_times:
            source_gaps_ms = [
                (second - first) * 1.0e-6
                for first, second in zip(source_times, source_times[1:])
            ]
            source_duration_s = (
                (source_times[-1] - source_times[0]) * 1.0e-9
                if len(source_times) > 1
                else 0.0
            )
            source_cadence.append(
                {
                    "effective_rate_hz": (
                        (len(source_times) - 1) / source_duration_s
                        if source_duration_s > 0.0
                        else 0.0
                    ),
                    "median_period_ms": (
                        float(median(source_gaps_ms)) if source_gaps_ms else 0.0
                    ),
                    "p99_gap_ms": _percentile_99(source_gaps_ms),
                    "maximum_gap_ms": max(source_gaps_ms, default=0.0),
                }
            )
        valid_point_percent = 100.0 * valid_future_points / (sample_count * int(
            loaded_contract["trajectory"]["point_count"]
        ))

        accounting = _mapping(
            episode.get("capture_accounting"), f"{episode_path}.capture_accounting"
        )
        raw_anchor_count = _integer(
            accounting.get("raw_anchor_count"),
            f"{episode_path}.capture_accounting.raw_anchor_count",
            1,
        )
        eligible_count = _integer(
            accounting.get("eligible_anchor_count"),
            f"{episode_path}.capture_accounting.eligible_anchor_count",
            1,
        )
        selected_count = _integer(
            accounting.get("selected_bundle_count"),
            f"{episode_path}.capture_accounting.selected_bundle_count",
            1,
        )
        dropped_count = _integer(
            accounting.get("dropped_incomplete_bundle_count"),
            f"{episode_path}.capture_accounting.dropped_incomplete_bundle_count",
            0,
        )
        ineligible_reasons = _mapping(
            accounting.get("ineligible_anchor_reasons"),
            f"{episode_path}.capture_accounting.ineligible_anchor_reasons",
        )
        ineligible_count = 0
        for reason, count in ineligible_reasons.items():
            _string(reason, f"{episode_path}.capture_accounting.ineligible reason")
            ineligible_count += _integer(
                count,
                f"{episode_path}.capture_accounting.ineligible_anchor_reasons.{reason}",
                0,
            )
        if raw_anchor_count != eligible_count + ineligible_count:
            raise ContractError(
                f"{episode_path}.capture_accounting raw count does not match "
                "eligible plus the ineligible reason ledger"
            )
        if selected_count != sample_count or dropped_count != eligible_count - selected_count:
            raise ContractError(f"{episode_path}.capture_accounting counts are inconsistent")
        bundle_coverage_percent = 100.0 * selected_count / eligible_count
        raw_camera_counts = _mapping(
            accounting.get("raw_camera_frame_counts"),
            f"{episode_path}.capture_accounting.raw_camera_frame_counts",
        )
        if tuple(raw_camera_counts) != CAMERA_ORDER:
            raise ContractError(
                f"{episode_path}.capture_accounting.raw_camera_frame_counts order is invalid"
            )
        raw_counts = [
            _integer(
                raw_camera_counts[name],
                f"{episode_path}.capture_accounting.raw_camera_frame_counts.{name}",
                eligible_count,
            )
            for name in CAMERA_ORDER
        ]
        manifest_raw_counts = source_manifest["raw_camera_frame_counts"]
        if any(
            raw_camera_counts[name] != manifest_raw_counts[name]
            for name in CAMERA_ORDER
        ):
            raise ContractError(
                f"{episode_path}.capture_accounting.raw_camera_frame_counts "
                "does not match the source manifest"
            )
        maximum_raw_count = max(raw_counts)
        count_imbalance_percent = (
            100.0 * (maximum_raw_count - min(raw_counts)) / maximum_raw_count
        )
        camera_unique_scan_percent = [
            100.0 * len(set(hashes)) / len(hashes) for hashes in camera_scan_hashes
        ]
        minimum_unique_scan_percent = min(camera_unique_scan_percent)
        maximum_identical_scan_run = 1
        for hashes in camera_scan_hashes:
            current_run = 1
            for first, second in zip(hashes, hashes[1:]):
                current_run = current_run + 1 if first == second else 1
                maximum_identical_scan_run = max(
                    maximum_identical_scan_run, current_run
                )
        cross_camera_duplicate_bundle_percent = (
            100.0 * cross_camera_duplicate_bundle_count / sample_count
        )
        capture = loaded_contract["capture"]
        if mode in ("planning", "runtime"):
            gates = (
                (
                    duration_s >= float(capture["minimum_episode_duration_s"]),
                    f"duration {duration_s:.3f}s is below the capture minimum",
                ),
                (
                    effective_rate_hz >= float(capture["minimum_effective_rate_hz"]),
                    f"effective rate {effective_rate_hz:.3f}Hz is below the capture minimum",
                ),
                (
                    float(capture["minimum_median_period_ms"])
                    <= median_period_ms
                    <= float(capture["maximum_median_period_ms"]),
                    f"median period {median_period_ms:.3f}ms is outside the capture range",
                ),
                (
                    p99_gap_ms <= float(capture["maximum_p99_gap_ms"]),
                    f"p99 gap {p99_gap_ms:.3f}ms exceeds the capture maximum",
                ),
                (
                    maximum_gap_ms <= float(capture["maximum_absolute_gap_ms"]),
                    f"maximum gap {maximum_gap_ms:.3f}ms exceeds the capture maximum",
                ),
                (
                    min(item["effective_rate_hz"] for item in source_cadence)
                    >= float(capture["minimum_effective_rate_hz"]),
                    "native source camera rate is below the capture minimum",
                ),
                (
                    all(
                        float(capture["minimum_median_period_ms"])
                        <= item["median_period_ms"]
                        <= float(capture["maximum_median_period_ms"])
                        for item in source_cadence
                    ),
                    "native source camera median period is outside the capture range",
                ),
                (
                    max(item["p99_gap_ms"] for item in source_cadence)
                    <= float(capture["maximum_p99_gap_ms"]),
                    "native source camera p99 gap exceeds the capture maximum",
                ),
                (
                    max(item["maximum_gap_ms"] for item in source_cadence)
                    <= float(capture["maximum_absolute_gap_ms"]),
                    "native source camera maximum gap exceeds the capture maximum",
                ),
                (
                    motion_distance_m >= float(capture["minimum_motion_distance_m"]),
                    f"motion {motion_distance_m:.3f}m is below the capture minimum",
                ),
                (
                    bundle_coverage_percent
                    >= float(capture["minimum_bundle_coverage_percent"]),
                    f"bundle coverage {bundle_coverage_percent:.3f}% is below the minimum",
                ),
                (
                    count_imbalance_percent
                    <= float(capture["maximum_count_imbalance_percent"]),
                    f"camera count imbalance {count_imbalance_percent:.3f}% exceeds the maximum",
                ),
                (
                    max(pose_steps_m, default=0.0)
                    <= float(capture["maximum_pose_step_m"]),
                    "pose step exceeds the localization-jump guard",
                ),
                (
                    max(speed_consistency_errors, default=0.0)
                    <= float(capture["maximum_speed_consistency_error_mps"]),
                    "pose displacement and reported speed are inconsistent",
                ),
                (
                    valid_point_percent
                    >= float(loaded_contract["trajectory"]["minimum_valid_point_percent"]),
                    f"valid future coverage {valid_point_percent:.3f}% is below the minimum",
                ),
            )
            for passed, message in gates:
                if not passed:
                    raise ContractError(f"{episode_path}: {message}")

        runtime_limit = float(capture["runtime_bundle_skew_ms"])
        runtime_coverage = (
            100.0 * sum(value <= runtime_limit for value in bundle_skews) / len(bundle_skews)
        )
        if mode == "runtime" and runtime_coverage < float(
            capture["minimum_bundle_coverage_percent"]
        ):
            raise ContractError(
                f"{episode_path}: offline 1 ms bundle readiness is "
                f"{runtime_coverage:.3f}%"
            )
        episode_report = {
            "episode_id": episode_id,
            "domain": domain,
            "split": split,
            "source_dataset_id": source_manifest["metadata"]["source_dataset_id"],
            "source_dataset_version": source_manifest["metadata"][
                "source_dataset_version"
            ],
            "license_id": source_manifest["metadata"]["license_id"],
            "sample_count": sample_count,
            "duration_s": duration_s,
            "effective_rate_hz": effective_rate_hz,
            "median_period_ms": median_period_ms,
            "p99_gap_ms": p99_gap_ms,
            "maximum_gap_ms": maximum_gap_ms,
            "native_source_camera_cadence": {
                name: source_cadence[index]
                for index, name in enumerate(CAMERA_ORDER)
            },
            "motion_distance_m": motion_distance_m,
            "maximum_pose_step_m": max(pose_steps_m, default=0.0),
            "maximum_speed_consistency_error_mps": max(
                speed_consistency_errors, default=0.0
            ),
            "raw_anchor_count": raw_anchor_count,
            "eligible_anchor_count": eligible_count,
            "selected_bundle_count": selected_count,
            "dropped_incomplete_bundle_count": dropped_count,
            "ineligible_anchor_reasons": dict(sorted(ineligible_reasons.items())),
            "bundle_coverage_percent": bundle_coverage_percent,
            "camera_count_imbalance_percent": count_imbalance_percent,
            "camera_unique_jpeg_scan_percent": {
                name: camera_unique_scan_percent[index]
                for index, name in enumerate(CAMERA_ORDER)
            },
            "minimum_unique_jpeg_scan_percent": minimum_unique_scan_percent,
            "maximum_consecutive_identical_jpeg_scans": maximum_identical_scan_run,
            "cross_camera_duplicate_bundle_count": cross_camera_duplicate_bundle_count,
            "cross_camera_duplicate_bundle_percent": (
                cross_camera_duplicate_bundle_percent
            ),
            "valid_future_point_percent": valid_point_percent,
            "maximum_bundle_skew_ms": max(bundle_skews),
            "runtime_bundle_coverage_percent": runtime_coverage,
            "maximum_state_delta_ms": max(state_deltas),
            "maximum_command_age_ms": max(command_ages),
            "command_counts": {str(key): value for key, value in sorted(command_counts.items())},
        }
        episode_reports.append(episode_report)
        split_counts[split] += sample_count
        domain_counts[domain] += sample_count
        for tag in scenario_tags:
            scenario_counts[str(tag)] += sample_count

    minimum_coverage = float(loaded_contract["capture"]["minimum_bundle_coverage_percent"])
    runtime_pass = all(
        report["runtime_bundle_coverage_percent"] >= minimum_coverage
        for report in episode_reports
    )
    return {
        "schema_id": REPORT_SCHEMA_ID,
        "schema_version": 1,
        "status": "PASS",
        "dataset_id": dataset_id,
        "dataset_root": str(dataset_root),
        "contract_id": CONTRACT_ID,
        "contract_sha256": effective_contract_hash,
        "contract_file_sha256": loaded_contract.get("_sha256"),
        "resource_limits": _resource_limits_report(),
        "dataset_manifest_sha256": manifest_hash,
        "mode": mode,
        "image_hashes_checked": check_image_hashes,
        "dataset_fingerprint_sha256": dataset_fingerprint.hexdigest(),
        "episode_count": len(episode_reports),
        "sample_count": sum(report["sample_count"] for report in episode_reports),
        "split_sample_counts": dict(sorted(split_counts.items())),
        "domain_sample_counts": dict(sorted(domain_counts.items())),
        "scenario_sample_counts": dict(sorted(scenario_counts.items())),
        "qualification": {
            "schema": "PASS",
            "common_10hz_planning": "PASS" if mode in ("planning", "runtime") else "NOT_RUN",
            "image_payload_sha256": "PASS" if check_image_hashes else "NOT_RUN",
            "native_source_frame_identity": "PASS",
            "native_source_manifest_binding": "PASS",
            "camera_timestamp_equals_native_source": "PASS",
            "raw_source_artifact_content": "NOT_RUN_REQUIRES_RAW_SOURCE_REVIEW",
            "canonical_causal_route_reconstruction": "PASS",
            "jpeg_scan_payload_reuse_diagnostic": "MEASURED_NOT_A_QUALIFICATION_GATE",
            "image_pixel_decode": "NOT_RUN_REQUIRES_APPROVED_IMAGE_LIBRARY",
            "offline_1ms_bundle_readiness": (
                "NOT_RUN"
                if mode == "schema"
                else ("PASS" if runtime_pass else "FAIL")
            ),
            "runtime_execution_tested": False,
            "trajectory_geometry_recomputed_from_source_state": False,
        },
        "manual_release_review_required": True,
        "episodes": episode_reports,
    }
