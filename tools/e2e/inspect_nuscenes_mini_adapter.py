#!/usr/bin/env python3
"""Read-only nuScenes v1.0-mini and CAN-bus adapter audit.

The audit reads only the bounded JSON metadata needed to prove camera-table
joins, camera timing, calibration/ego-pose references, and CAN route
availability.  It never extracts archive members and never decodes an image.
Passing reports from ``verify_tar_archive.py`` and ``verify_zip_archive.py`` are
mandatory and rebound to the current archive bytes, so a stale report cannot
validate a changed archive.

Structural validity and ``common_10hz_v1`` planning qualification are separate:
the official mini archives are useful for adapter smoke tests, but their roughly
20-second scenes and asynchronous roughly-12-Hz camera streams are not silently
retimed or padded into the project's 10-Hz planning contract.
"""

from __future__ import annotations

import argparse
import bisect
from collections import Counter, defaultdict
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path, PurePosixPath, PureWindowsPath
import re
import stat
import statistics
import struct
import sys
import tarfile
from typing import Any, Iterable, Mapping, Sequence
import unicodedata
import zipfile


MIB = 1024 * 1024
GIB = 1024 * MIB
MAX_AUDIT_REPORT_BYTES = 4 * MIB
HASH_CHUNK_BYTES = 8 * MIB
MAX_ERRORS = 100

CAMERA_CHANNELS = (
    "CAM_FRONT",
    "CAM_BACK",
    "CAM_FRONT_LEFT",
    "CAM_BACK_LEFT",
    "CAM_FRONT_RIGHT",
    "CAM_BACK_RIGHT",
)
CAMERA_CHANNEL_SET = frozenset(CAMERA_CHANNELS)
CAMERA_MAPPING = tuple((channel, channel) for channel in CAMERA_CHANNELS)

EXPECTED_SCENE_SAMPLES = (
    ("scene-0061", 39),
    ("scene-0103", 40),
    ("scene-0553", 41),
    ("scene-0655", 41),
    ("scene-0757", 41),
    ("scene-0796", 40),
    ("scene-0916", 41),
    ("scene-1077", 41),
    ("scene-1094", 40),
    ("scene-1100", 40),
)

METADATA_MEMBERS = {
    "scene": "v1.0-mini/scene.json",
    "sample": "v1.0-mini/sample.json",
    "sample_data": "v1.0-mini/sample_data.json",
    "sensor": "v1.0-mini/sensor.json",
    "calibrated_sensor": "v1.0-mini/calibrated_sensor.json",
    "ego_pose": "v1.0-mini/ego_pose.json",
    "log": "v1.0-mini/log.json",
}
CAN_MEMBER = re.compile(
    r"can_bus/(?P<scene>scene-[0-9]{4})_(?P<kind>[a-z0-9_]+)\.json",
    re.ASCII,
)
TOKEN = re.compile(r"[0-9a-f]{32}", re.ASCII)
SHA256 = re.compile(r"[0-9a-f]{64}", re.ASCII)
SUPPORTED_ZIP_METHODS = {
    zipfile.ZIP_STORED,
    zipfile.ZIP_DEFLATED,
    zipfile.ZIP_BZIP2,
    zipfile.ZIP_LZMA,
}
WINDOWS_RESERVED_NAMES = {
    "AUX",
    "CON",
    "NUL",
    "PRN",
    *(f"COM{index}" for index in range(1, 10)),
    *(f"LPT{index}" for index in range(1, 10)),
}


@dataclass(frozen=True)
class AuditLimits:
    """Fail-closed archive and metadata resource limits."""

    max_nuscenes_archive_bytes: int = 8 * GIB
    max_can_archive_bytes: int = 2 * GIB
    max_tar_members: int = 100_000
    max_zip_members: int = 50_000
    max_zip_central_directory_bytes: int = 64 * MIB
    max_member_uncompressed_bytes: int = 2 * GIB
    max_total_uncompressed_bytes: int = 16 * GIB
    max_total_compression_ratio: float = 100.0
    max_member_compression_ratio: float = 1_000.0
    max_metadata_member_bytes: int = 32 * MIB
    max_metadata_total_bytes: int = 64 * MIB
    max_can_route_member_bytes: int = 8 * MIB
    max_route_points: int = 100_000
    max_path_bytes: int = 4096
    max_component_bytes: int = 255
    max_aggregate_path_bytes: int = 16 * MIB


@dataclass(frozen=True)
class MiniExpectations:
    """Pinned official-mini cardinalities; replaceable only by tiny tests."""

    scene_samples: tuple[tuple[str, int], ...] = EXPECTED_SCENE_SAMPLES
    sample_count: int = 404
    sample_data_count: int = 31_206
    sensor_count: int = 12
    calibrated_sensor_count: int = 120
    ego_pose_count: int = 31_206
    log_count: int = 8
    camera_frame_count: int = 14_008
    camera_frames_by_channel: tuple[tuple[str, int], ...] = (
        ("CAM_FRONT", 2_342),
        ("CAM_BACK", 2_315),
        ("CAM_FRONT_LEFT", 2_344),
        ("CAM_BACK_LEFT", 2_327),
        ("CAM_FRONT_RIGHT", 2_338),
        ("CAM_BACK_RIGHT", 2_342),
    )
    camera_calibrations_per_channel: int = 10

    def samples_by_scene(self) -> dict[str, int]:
        return dict(self.scene_samples)


DEFAULT_LIMITS = AuditLimits()
DEFAULT_EXPECTATIONS = MiniExpectations()


@dataclass(frozen=True)
class FileIdentity:
    path: Path
    device: int
    inode: int
    size: int
    mtime_ns: int
    ctime_ns: int

    @classmethod
    def from_stat(cls, path: Path, metadata: os.stat_result) -> "FileIdentity":
        return cls(
            path=path,
            device=metadata.st_dev,
            inode=metadata.st_ino,
            size=metadata.st_size,
            mtime_ns=metadata.st_mtime_ns,
            ctime_ns=metadata.st_ctime_ns,
        )

    def signature(self) -> tuple[int, int, int, int, int]:
        return self.device, self.inode, self.size, self.mtime_ns, self.ctime_ns


@dataclass(frozen=True)
class GenericAuditProof:
    report_path: Path
    report: Mapping[str, Any]
    archive_identity: FileIdentity
    archive_kind: str
    digest: str


class InspectionFailure(Exception):
    """A deterministic failure that is safe to serialize into the report."""


class JsonArgumentParser(argparse.ArgumentParser):
    """Convert CLI usage failures into the same JSON failure path."""

    def error(self, message: str) -> None:
        raise InspectionFailure(f"argument error: {message}")


def _absolute_path(value: Path | str) -> Path:
    try:
        return Path(os.path.abspath(os.fspath(Path(value).expanduser())))
    except (OSError, RuntimeError, TypeError, ValueError) as error:
        raise InspectionFailure(f"invalid filesystem path: {error}") from error


def _same_identity(metadata: os.stat_result, identity: FileIdentity) -> bool:
    return (
        metadata.st_dev,
        metadata.st_ino,
        metadata.st_size,
        metadata.st_mtime_ns,
        metadata.st_ctime_ns,
    ) == identity.signature()


def _open_regular_read_only(
    path: Path,
    label: str,
    maximum_bytes: int,
) -> tuple[int, FileIdentity]:
    try:
        path_metadata = os.lstat(path)
    except OSError as error:
        raise InspectionFailure(f"{label} cannot be inspected: {error}") from error
    if stat.S_ISLNK(path_metadata.st_mode):
        raise InspectionFailure(f"{label} path must not be a symbolic link")
    if not stat.S_ISREG(path_metadata.st_mode):
        raise InspectionFailure(f"{label} path must be a regular file")
    if path_metadata.st_size <= 0 or path_metadata.st_size > maximum_bytes:
        raise InspectionFailure(
            f"{label} size {path_metadata.st_size} is outside 1..{maximum_bytes} bytes"
        )

    flags = os.O_RDONLY
    if hasattr(os, "O_CLOEXEC"):
        flags |= os.O_CLOEXEC
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    try:
        descriptor = os.open(path, flags)
    except OSError as error:
        raise InspectionFailure(f"{label} cannot be opened safely: {error}") from error
    try:
        opened = os.fstat(descriptor)
        if not stat.S_ISREG(opened.st_mode):
            raise InspectionFailure(f"opened {label} is not a regular file")
        if (opened.st_dev, opened.st_ino) != (
            path_metadata.st_dev,
            path_metadata.st_ino,
        ):
            raise InspectionFailure(f"{label} changed while it was opened")
        identity = FileIdentity.from_stat(path, opened)
        if identity.size <= 0 or identity.size > maximum_bytes:
            raise InspectionFailure(
                f"opened {label} size {identity.size} is outside 1..{maximum_bytes} bytes"
            )
        return descriptor, identity
    except Exception:
        os.close(descriptor)
        raise


def _recheck_identity(identity: FileIdentity, label: str) -> None:
    try:
        current = os.lstat(identity.path)
    except OSError as error:
        raise InspectionFailure(f"{label} cannot be rechecked: {error}") from error
    if stat.S_ISLNK(current.st_mode) or not stat.S_ISREG(current.st_mode):
        raise InspectionFailure(f"{label} is no longer the same regular file")
    if not _same_identity(current, identity):
        raise InspectionFailure(f"{label} changed during inspection")


def _sha256_same_identity(identity: FileIdentity, label: str) -> str:
    descriptor, current = _open_regular_read_only(
        identity.path,
        label,
        max(identity.size, 1),
    )
    if current.signature() != identity.signature():
        os.close(descriptor)
        raise InspectionFailure(f"{label} changed before hashing")
    digest = hashlib.sha256()
    try:
        while True:
            block = os.read(descriptor, HASH_CHUNK_BYTES)
            if not block:
                break
            digest.update(block)
        if not _same_identity(os.fstat(descriptor), identity):
            raise InspectionFailure(f"{label} changed while hashing")
    except OSError as error:
        raise InspectionFailure(f"{label} could not be hashed: {error}") from error
    finally:
        os.close(descriptor)
    _recheck_identity(identity, label)
    return digest.hexdigest()


def _validate_limits(limits: AuditLimits) -> None:
    integer_fields = (
        limits.max_nuscenes_archive_bytes,
        limits.max_can_archive_bytes,
        limits.max_tar_members,
        limits.max_zip_members,
        limits.max_zip_central_directory_bytes,
        limits.max_member_uncompressed_bytes,
        limits.max_total_uncompressed_bytes,
        limits.max_metadata_member_bytes,
        limits.max_metadata_total_bytes,
        limits.max_can_route_member_bytes,
        limits.max_route_points,
        limits.max_path_bytes,
        limits.max_component_bytes,
        limits.max_aggregate_path_bytes,
    )
    if any(isinstance(value, bool) or not isinstance(value, int) or value <= 0
           for value in integer_fields):
        raise InspectionFailure("every integer resource limit must be positive")
    for value in (
        limits.max_total_compression_ratio,
        limits.max_member_compression_ratio,
    ):
        if not isinstance(value, (int, float)) or isinstance(value, bool):
            raise InspectionFailure("compression-ratio limits must be numeric")
        if not math.isfinite(float(value)) or value <= 0:
            raise InspectionFailure("compression-ratio limits must be finite and positive")


def _validate_expectations(expectations: MiniExpectations) -> None:
    by_scene = expectations.samples_by_scene()
    if not by_scene or len(by_scene) != len(expectations.scene_samples):
        raise InspectionFailure("expected scene names must be unique and non-empty")
    if any(
        re.fullmatch(r"scene-[0-9]{4}", scene, re.ASCII) is None
        or isinstance(count, bool)
        or not isinstance(count, int)
        or count <= 0
        for scene, count in expectations.scene_samples
    ):
        raise InspectionFailure("expected scenes and sample counts are invalid")
    counts = (
        expectations.sample_count,
        expectations.sample_data_count,
        expectations.sensor_count,
        expectations.calibrated_sensor_count,
        expectations.ego_pose_count,
        expectations.log_count,
        expectations.camera_frame_count,
        expectations.camera_calibrations_per_channel,
    )
    if any(isinstance(value, bool) or not isinstance(value, int) or value <= 0
           for value in counts):
        raise InspectionFailure("expected table counts must be positive integers")
    if sum(by_scene.values()) != expectations.sample_count:
        raise InspectionFailure("expected per-scene samples do not sum to sample_count")
    camera_counts = dict(expectations.camera_frames_by_channel)
    if (
        len(camera_counts) != len(expectations.camera_frames_by_channel)
        or set(camera_counts) != CAMERA_CHANNEL_SET
        or any(
            isinstance(value, bool) or not isinstance(value, int) or value <= 0
            for value in camera_counts.values()
        )
    ):
        raise InspectionFailure("expected per-channel camera counts are invalid")
    if sum(camera_counts.values()) != expectations.camera_frame_count:
        raise InspectionFailure(
            "expected per-channel camera counts do not sum to camera_frame_count"
        )


def _safe_member_path(
    raw_name: str,
    *,
    limits: AuditLimits,
) -> tuple[str, int]:
    if not isinstance(raw_name, str) or not raw_name:
        raise InspectionFailure("archive contains an empty or non-text member path")
    if "\x00" in raw_name or "\\" in raw_name:
        raise InspectionFailure(f"unsafe archive member path {raw_name!r}")
    normalized = unicodedata.normalize("NFC", raw_name.rstrip("/"))
    if not normalized:
        raise InspectionFailure(f"unsafe archive member path {raw_name!r}")
    posix = PurePosixPath(normalized)
    windows = PureWindowsPath(normalized)
    if posix.is_absolute() or windows.is_absolute() or windows.drive:
        raise InspectionFailure(f"absolute archive member path {raw_name!r}")
    if any(part in ("", ".", "..") for part in posix.parts):
        raise InspectionFailure(f"traversal archive member path {raw_name!r}")
    for part in posix.parts:
        if part.endswith((" ", ".")):
            raise InspectionFailure(f"unsafe trailing archive path character in {raw_name!r}")
        stem = part.split(".", 1)[0].upper()
        if stem in WINDOWS_RESERVED_NAMES:
            raise InspectionFailure(f"reserved archive member path {raw_name!r}")
    encoded_bytes = len(normalized.encode("utf-8"))
    if encoded_bytes > limits.max_path_bytes:
        raise InspectionFailure(f"archive member path exceeds {limits.max_path_bytes} bytes")
    if any(len(part.encode("utf-8")) > limits.max_component_bytes for part in posix.parts):
        raise InspectionFailure(
            f"archive member component exceeds {limits.max_component_bytes} bytes"
        )
    return normalized, encoded_bytes


def _reject_json_constant(value: str) -> Any:
    raise InspectionFailure(f"JSON contains forbidden constant {value!r}")


def _unique_json_object(pairs: Iterable[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise InspectionFailure(f"JSON contains duplicate key {key!r}")
        result[key] = value
    return result


def _decode_json_list(payload: bytes, label: str, record_cap: int) -> list[Any]:
    try:
        decoded = payload.decode("utf-8")
        parsed = json.loads(
            decoded,
            object_pairs_hook=_unique_json_object,
            parse_constant=_reject_json_constant,
        )
    except InspectionFailure:
        raise
    except Exception as error:
        raise InspectionFailure(f"{label} is not strict UTF-8 JSON: {error}") from error
    if not isinstance(parsed, list):
        raise InspectionFailure(f"{label} root must be a JSON array")
    if len(parsed) > record_cap:
        raise InspectionFailure(f"{label} exceeds the {record_cap}-record cap")
    return parsed


def _top_level_counts(paths: Iterable[str]) -> dict[str, int]:
    counts: Counter[str] = Counter()
    for path in paths:
        counts[path.split("/", 1)[0]] += 1
    return dict(sorted(counts.items()))


def _scan_nuscenes_tar(
    path: Path,
    limits: AuditLimits,
) -> tuple[dict[str, Any], FileIdentity]:
    descriptor, identity = _open_regular_read_only(
        path,
        "nuScenes archive",
        limits.max_nuscenes_archive_bytes,
    )
    tables: dict[str, list[Any]] = {}
    all_paths: set[str] = set()
    regular_file_paths: set[str] = set()
    regular_file_sizes: dict[str, int] = {}
    camera_image_member_candidates: set[str] = set()
    casefold_paths: dict[str, str] = {}
    path_bytes = 0
    member_count = 0
    file_count = 0
    directory_count = 0
    declared_regular_bytes = 0
    metadata_bytes_read = 0
    metadata_names = {value: key for key, value in METADATA_MEMBERS.items()}
    record_caps = {
        "scene": 2_000,
        "sample": 10_000,
        "sample_data": 100_000,
        "sensor": 1_000,
        "calibrated_sensor": 10_000,
        "ego_pose": 100_000,
        "log": 2_000,
    }
    try:
        with os.fdopen(os.dup(descriptor), "rb") as stream:
            try:
                archive = tarfile.open(fileobj=stream, mode="r:gz")
            except (OSError, tarfile.TarError) as error:
                raise InspectionFailure(f"nuScenes TAR+gzip cannot be opened: {error}") from error
            with archive:
                try:
                    for member in archive:
                        member_count += 1
                        if member_count > limits.max_tar_members:
                            raise InspectionFailure(
                                f"nuScenes TAR exceeds {limits.max_tar_members} members"
                            )
                        normalized, encoded_bytes = _safe_member_path(
                            member.name,
                            limits=limits,
                        )
                        path_bytes += encoded_bytes
                        if path_bytes > limits.max_aggregate_path_bytes:
                            raise InspectionFailure(
                                "nuScenes TAR exceeds the aggregate normalized-path cap"
                            )
                        folded = normalized.casefold()
                        if normalized in all_paths:
                            raise InspectionFailure(
                                f"nuScenes TAR contains duplicate path {normalized!r}"
                            )
                        if folded in casefold_paths and casefold_paths[folded] != normalized:
                            raise InspectionFailure(
                                "nuScenes TAR contains a case-folding path collision"
                            )
                        all_paths.add(normalized)
                        casefold_paths[folded] = normalized

                        if member.isdir():
                            directory_count += 1
                            continue
                        if not member.isfile():
                            raise InspectionFailure(
                                f"nuScenes TAR contains forbidden link/special member {normalized!r}"
                            )
                        file_count += 1
                        if member.size < 0 or member.size > limits.max_member_uncompressed_bytes:
                            raise InspectionFailure(
                                f"nuScenes TAR member {normalized!r} exceeds its size cap"
                            )
                        regular_file_paths.add(normalized)
                        regular_file_sizes[normalized] = member.size
                        parts = PurePosixPath(normalized).parts
                        if (
                            len(parts) >= 2
                            and parts[0] in ("samples", "sweeps")
                            and (
                                parts[1].startswith("CAM_")
                                or normalized.lower().endswith((".jpg", ".jpeg"))
                            )
                        ):
                            camera_image_member_candidates.add(normalized)
                        declared_regular_bytes += member.size
                        if declared_regular_bytes > limits.max_total_uncompressed_bytes:
                            raise InspectionFailure(
                                "nuScenes TAR exceeds the total declared-byte cap"
                            )

                        table_name = metadata_names.get(normalized)
                        if table_name is None:
                            continue
                        if table_name in tables:
                            raise InspectionFailure(
                                f"nuScenes metadata member {normalized!r} is duplicated"
                            )
                        if member.size > limits.max_metadata_member_bytes:
                            raise InspectionFailure(
                                f"nuScenes metadata member {normalized!r} exceeds its cap"
                            )
                        metadata_bytes_read += member.size
                        if metadata_bytes_read > limits.max_metadata_total_bytes:
                            raise InspectionFailure(
                                "nuScenes metadata exceeds the aggregate read cap"
                            )
                        extracted = archive.extractfile(member)
                        if extracted is None:
                            raise InspectionFailure(
                                f"nuScenes metadata member {normalized!r} cannot be read"
                            )
                        try:
                            payload = extracted.read(member.size + 1)
                        finally:
                            extracted.close()
                        if len(payload) != member.size:
                            raise InspectionFailure(
                                f"nuScenes metadata member {normalized!r} has a short read"
                            )
                        tables[table_name] = _decode_json_list(
                            payload,
                            normalized,
                            record_caps[table_name],
                        )
                except (OSError, tarfile.TarError) as error:
                    raise InspectionFailure(f"nuScenes TAR inspection failed: {error}") from error
        if not _same_identity(os.fstat(descriptor), identity):
            raise InspectionFailure("nuScenes archive changed while it was inspected")
    finally:
        os.close(descriptor)
    _recheck_identity(identity, "nuScenes archive")

    missing = sorted(set(METADATA_MEMBERS) - set(tables))
    if missing:
        raise InspectionFailure(f"nuScenes archive is missing metadata tables: {missing}")
    ratio = declared_regular_bytes / identity.size
    if ratio > limits.max_total_compression_ratio:
        raise InspectionFailure("nuScenes TAR exceeds the total compression-ratio cap")
    summary = {
        "member_count": member_count,
        "file_count": file_count,
        "directory_count": directory_count,
        "declared_regular_bytes": declared_regular_bytes,
        "top_level_entries": _top_level_counts(all_paths),
        "metadata_members_read": sorted(METADATA_MEMBERS.values()),
        "metadata_bytes_read": metadata_bytes_read,
        "image_payloads_read": 0,
        "member_paths": all_paths,
        "regular_file_paths": regular_file_paths,
        "regular_file_sizes": regular_file_sizes,
        "camera_image_member_candidates": camera_image_member_candidates,
        "tables": tables,
    }
    return summary, identity


def _zip_member_is_forbidden(info: zipfile.ZipInfo) -> bool:
    unix_mode = (info.external_attr >> 16) & 0xFFFF
    member_type = stat.S_IFMT(unix_mode)
    if member_type == stat.S_IFLNK:
        return True
    if member_type not in (0, stat.S_IFREG, stat.S_IFDIR):
        return True
    return False


def _read_zip_json_list(
    archive: zipfile.ZipFile,
    info: zipfile.ZipInfo,
    label: str,
    byte_cap: int,
    record_cap: int,
) -> list[Any]:
    if info.file_size > byte_cap:
        raise InspectionFailure(f"{label} exceeds the {byte_cap}-byte read cap")
    try:
        with archive.open(info, "r") as stream:
            payload = stream.read(byte_cap + 1)
    except (OSError, RuntimeError, zipfile.BadZipFile) as error:
        raise InspectionFailure(f"{label} could not be read safely: {error}") from error
    if len(payload) != info.file_size or len(payload) > byte_cap:
        raise InspectionFailure(f"{label} has an invalid uncompressed length")
    return _decode_json_list(payload, label, record_cap)


def _valid_route_points(values: list[Any], label: str) -> None:
    for index, point in enumerate(values):
        if not isinstance(point, list) or len(point) != 2:
            raise InspectionFailure(f"{label} point {index} is not an [x, y] pair")
        for coordinate in point:
            if isinstance(coordinate, bool) or not isinstance(coordinate, (int, float)):
                raise InspectionFailure(f"{label} point {index} has a non-numeric coordinate")
            if not math.isfinite(float(coordinate)):
                raise InspectionFailure(f"{label} point {index} has a non-finite coordinate")


def _preflight_zip_eocd(
    descriptor: int,
    identity: FileIdentity,
    limits: AuditLimits,
) -> dict[str, int]:
    """Bound a classic single-disk central directory before ``ZipFile`` parses it."""
    maximum_tail = 22 + 65_535
    tail_size = min(identity.size, maximum_tail)
    try:
        tail = os.pread(descriptor, tail_size, identity.size - tail_size)
    except OSError as error:
        raise InspectionFailure(f"CAN-bus ZIP EOCD cannot be read: {error}") from error
    if len(tail) != tail_size:
        raise InspectionFailure("CAN-bus ZIP EOCD tail has a short read")
    signature = b"PK\x05\x06"
    offset = tail.rfind(signature)
    fields: tuple[Any, ...] | None = None
    while offset >= 0:
        if offset + 22 <= len(tail):
            candidate = struct.unpack_from("<4s4H2LH", tail, offset)
            comment_bytes = candidate[-1]
            if offset + 22 + comment_bytes == len(tail):
                fields = candidate
                break
        offset = tail.rfind(signature, 0, offset)
    if fields is None:
        raise InspectionFailure("CAN-bus ZIP has no terminal classic EOCD record")
    (
        _signature,
        disk_number,
        directory_disk,
        entries_on_disk,
        entry_count,
        central_bytes,
        central_offset,
        _comment_bytes,
    ) = fields
    if disk_number != 0 or directory_disk != 0 or entries_on_disk != entry_count:
        raise InspectionFailure("CAN-bus ZIP must be a single-disk archive")
    if entry_count == 0xFFFF or central_bytes == 0xFFFFFFFF or central_offset == 0xFFFFFFFF:
        raise InspectionFailure("CAN-bus ZIP64 is outside this pinned adapter profile")
    if entry_count <= 0 or entry_count > limits.max_zip_members:
        raise InspectionFailure("CAN-bus ZIP EOCD member count exceeds its resource cap")
    if central_bytes <= 0 or central_bytes > limits.max_zip_central_directory_bytes:
        raise InspectionFailure("CAN-bus ZIP EOCD central directory exceeds its resource cap")
    eocd_offset = identity.size - tail_size + offset
    if central_offset + central_bytes != eocd_offset:
        raise InspectionFailure("CAN-bus ZIP central-directory bounds are inconsistent")
    return {
        "entry_count": entry_count,
        "central_directory_bytes": central_bytes,
        "central_directory_offset": central_offset,
    }


def _scan_can_zip(
    path: Path,
    expected_scenes: frozenset[str],
    limits: AuditLimits,
) -> tuple[dict[str, Any], FileIdentity]:
    descriptor, identity = _open_regular_read_only(
        path,
        "CAN-bus archive",
        limits.max_can_archive_bytes,
    )
    all_paths: set[str] = set()
    casefold_paths: dict[str, str] = {}
    by_scene: dict[str, dict[str, zipfile.ZipInfo]] = defaultdict(dict)
    declared_uncompressed = 0
    declared_compressed = 0
    directory_count = 0
    file_count = 0
    path_bytes = 0
    route_points: dict[str, int] = {}
    route_members_read: list[str] = []
    try:
        eocd = _preflight_zip_eocd(descriptor, identity, limits)
        with os.fdopen(os.dup(descriptor), "rb") as stream:
            try:
                archive = zipfile.ZipFile(stream, mode="r", allowZip64=True)
            except (OSError, zipfile.BadZipFile) as error:
                raise InspectionFailure(f"CAN-bus ZIP cannot be opened: {error}") from error
            with archive:
                if archive.start_dir != eocd["central_directory_offset"]:
                    raise InspectionFailure(
                        "CAN-bus ZIP parser disagrees with the EOCD central-directory offset"
                    )
                try:
                    members = archive.infolist()
                except (OSError, zipfile.BadZipFile) as error:
                    raise InspectionFailure(
                        f"CAN-bus ZIP central directory cannot be read: {error}"
                    ) from error
                if len(members) > limits.max_zip_members:
                    raise InspectionFailure(
                        f"CAN-bus ZIP exceeds {limits.max_zip_members} members"
                    )
                if len(members) != eocd["entry_count"]:
                    raise InspectionFailure(
                        "CAN-bus ZIP parser disagrees with the EOCD member count"
                    )
                for info in members:
                    normalized, encoded_bytes = _safe_member_path(
                        info.filename,
                        limits=limits,
                    )
                    path_bytes += encoded_bytes
                    if path_bytes > limits.max_aggregate_path_bytes:
                        raise InspectionFailure(
                            "CAN-bus ZIP exceeds the aggregate normalized-path cap"
                        )
                    folded = normalized.casefold()
                    if normalized in all_paths:
                        raise InspectionFailure(
                            f"CAN-bus ZIP contains duplicate path {normalized!r}"
                        )
                    if folded in casefold_paths and casefold_paths[folded] != normalized:
                        raise InspectionFailure(
                            "CAN-bus ZIP contains a case-folding path collision"
                        )
                    all_paths.add(normalized)
                    casefold_paths[folded] = normalized
                    if info.flag_bits & 0x1:
                        raise InspectionFailure(
                            f"CAN-bus ZIP contains encrypted member {normalized!r}"
                        )
                    if info.compress_type not in SUPPORTED_ZIP_METHODS:
                        raise InspectionFailure(
                            f"CAN-bus ZIP member {normalized!r} uses unsupported compression"
                        )
                    if _zip_member_is_forbidden(info):
                        raise InspectionFailure(
                            f"CAN-bus ZIP contains forbidden link/special member {normalized!r}"
                        )
                    if info.is_dir():
                        directory_count += 1
                        continue
                    file_count += 1
                    if info.file_size < 0 or info.file_size > limits.max_member_uncompressed_bytes:
                        raise InspectionFailure(
                            f"CAN-bus ZIP member {normalized!r} exceeds its size cap"
                        )
                    declared_uncompressed += info.file_size
                    declared_compressed += info.compress_size
                    if declared_uncompressed > limits.max_total_uncompressed_bytes:
                        raise InspectionFailure(
                            "CAN-bus ZIP exceeds the total declared-byte cap"
                        )
                    if info.file_size and info.compress_size == 0:
                        raise InspectionFailure(
                            f"CAN-bus ZIP member {normalized!r} has an invalid compressed size"
                        )
                    if info.compress_size:
                        member_ratio = info.file_size / info.compress_size
                        if member_ratio > limits.max_member_compression_ratio:
                            raise InspectionFailure(
                                f"CAN-bus ZIP member {normalized!r} exceeds its ratio cap"
                            )
                    match = CAN_MEMBER.fullmatch(normalized)
                    if match is not None:
                        scene = match.group("scene")
                        kind = match.group("kind")
                        if kind in by_scene[scene]:
                            raise InspectionFailure(
                                f"CAN-bus ZIP repeats {scene!r} message kind {kind!r}"
                            )
                        by_scene[scene][kind] = info

                ratio = declared_uncompressed / max(declared_compressed, 1)
                if ratio > limits.max_total_compression_ratio:
                    raise InspectionFailure(
                        "CAN-bus ZIP exceeds the total compression-ratio cap"
                    )
                for scene in sorted(expected_scenes):
                    info = by_scene.get(scene, {}).get("route")
                    if info is None:
                        continue
                    label = f"CAN route {scene}"
                    values = _read_zip_json_list(
                        archive,
                        info,
                        label,
                        limits.max_can_route_member_bytes,
                        limits.max_route_points,
                    )
                    if not values:
                        raise InspectionFailure(f"{label} must contain at least one point")
                    _valid_route_points(values, label)
                    route_points[scene] = len(values)
                    route_members_read.append(info.filename)
        if not _same_identity(os.fstat(descriptor), identity):
            raise InspectionFailure("CAN-bus archive changed while it was inspected")
    finally:
        os.close(descriptor)
    _recheck_identity(identity, "CAN-bus archive")

    summary = {
        "member_count": len(all_paths),
        "file_count": file_count,
        "directory_count": directory_count,
        "declared_uncompressed_bytes": declared_uncompressed,
        "declared_compressed_bytes": declared_compressed,
        "central_directory_bytes": eocd["central_directory_bytes"],
        "central_directory_offset": eocd["central_directory_offset"],
        "top_level_entries": _top_level_counts(all_paths),
        "scene_kinds": {
            scene: sorted(kinds)
            for scene, kinds in sorted(by_scene.items())
        },
        "route_point_counts": route_points,
        "route_members_read": sorted(route_members_read),
        "other_payloads_read": 0,
    }
    return summary, identity


def _require_object(record: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(record, dict):
        raise InspectionFailure(f"{label} must be a JSON object")
    return record


def _require_text(record: Mapping[str, Any], key: str, label: str) -> str:
    value = record.get(key)
    if not isinstance(value, str):
        raise InspectionFailure(f"{label}.{key} must be text")
    return value


def _require_token(record: Mapping[str, Any], key: str, label: str) -> str:
    value = _require_text(record, key, label)
    if TOKEN.fullmatch(value) is None:
        raise InspectionFailure(f"{label}.{key} must be a lowercase 32-hex token")
    return value


def _require_integer(record: Mapping[str, Any], key: str, label: str) -> int:
    value = record.get(key)
    if isinstance(value, bool) or not isinstance(value, int):
        raise InspectionFailure(f"{label}.{key} must be an integer")
    return value


def _require_finite_vector(value: Any, length: int, label: str) -> None:
    if not isinstance(value, list) or len(value) != length:
        raise InspectionFailure(f"{label} must contain exactly {length} values")
    if any(
        isinstance(item, bool)
        or not isinstance(item, (int, float))
        or not math.isfinite(float(item))
        for item in value
    ):
        raise InspectionFailure(f"{label} must contain only finite numbers")


def _token_table(records: list[Any], table: str) -> dict[str, Mapping[str, Any]]:
    result: dict[str, Mapping[str, Any]] = {}
    for index, item in enumerate(records):
        label = f"{table}[{index}]"
        record = _require_object(item, label)
        token = _require_token(record, "token", label)
        if token in result:
            raise InspectionFailure(f"{table} repeats token {token!r}")
        result[token] = record
    return result


def _percentile(values: Sequence[float], quantile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = max(0, math.ceil(quantile * len(ordered)) - 1)
    return ordered[index]


def _metric_summary(values: Sequence[float]) -> dict[str, float | int | None]:
    if not values:
        return {"count": 0, "min": None, "median": None, "p95": None, "p99": None,
                "max": None}
    return {
        "count": len(values),
        "min": min(values),
        "median": statistics.median(values),
        "p95": _percentile(values, 0.95),
        "p99": _percentile(values, 0.99),
        "max": max(values),
    }


def _selection_only_10hz(timestamps: Sequence[int]) -> list[int]:
    """Select nearest native frames on a 100-ms grid without retiming or synthesis."""
    source = sorted(set(timestamps))
    if not source:
        return []
    selected: list[int] = []
    first_eligible = 0
    target = source[0]
    while target <= source[-1] and first_eligible < len(source):
        insertion = bisect.bisect_left(source, target, first_eligible)
        candidates = [
            index
            for index in (insertion - 1, insertion)
            if first_eligible <= index < len(source)
        ]
        if not candidates:
            break
        chosen = min(candidates, key=lambda index: (abs(source[index] - target), source[index]))
        selected.append(source[chosen])
        first_eligible = chosen + 1
        target += 100_000
    return selected


def _validate_table_counts(
    tables: Mapping[str, list[Any]],
    expectations: MiniExpectations,
) -> None:
    expected = {
        "scene": len(expectations.scene_samples),
        "sample": expectations.sample_count,
        "sample_data": expectations.sample_data_count,
        "sensor": expectations.sensor_count,
        "calibrated_sensor": expectations.calibrated_sensor_count,
        "ego_pose": expectations.ego_pose_count,
        "log": expectations.log_count,
    }
    for name, count in expected.items():
        if len(tables[name]) != count:
            raise InspectionFailure(
                f"nuScenes {name} count is {len(tables[name])}, expected {count}"
            )


def _inspect_nuscenes_tables(
    archive_summary: Mapping[str, Any],
    expectations: MiniExpectations,
    limits: AuditLimits,
) -> dict[str, Any]:
    tables = archive_summary["tables"]
    member_paths = archive_summary["member_paths"]
    regular_file_paths = archive_summary["regular_file_paths"]
    regular_file_sizes = archive_summary["regular_file_sizes"]
    camera_image_member_candidates = archive_summary[
        "camera_image_member_candidates"
    ]
    _validate_table_counts(tables, expectations)
    scenes = _token_table(tables["scene"], "scene")
    samples = _token_table(tables["sample"], "sample")
    sample_data = _token_table(tables["sample_data"], "sample_data")
    sensors = _token_table(tables["sensor"], "sensor")
    calibrations = _token_table(tables["calibrated_sensor"], "calibrated_sensor")
    ego_poses = _token_table(tables["ego_pose"], "ego_pose")
    logs = _token_table(tables["log"], "log")

    expected_by_name = expectations.samples_by_scene()
    scenes_by_name: dict[str, tuple[str, Mapping[str, Any]]] = {}
    for token, scene in scenes.items():
        label = f"scene[{token}]"
        name = _require_text(scene, "name", label)
        if name in scenes_by_name:
            raise InspectionFailure(f"scene table repeats name {name!r}")
        scenes_by_name[name] = (token, scene)
        log_token = _require_token(scene, "log_token", label)
        if log_token not in logs:
            raise InspectionFailure(f"{label} refers to missing log {log_token!r}")
    if set(scenes_by_name) != set(expected_by_name):
        raise InspectionFailure(
            "nuScenes mini scene names do not exactly match the pinned manifest"
        )

    samples_by_scene: dict[str, list[tuple[int, str, Mapping[str, Any]]]] = defaultdict(list)
    for token, sample in samples.items():
        label = f"sample[{token}]"
        scene_token = _require_token(sample, "scene_token", label)
        if scene_token not in scenes:
            raise InspectionFailure(f"{label} refers to missing scene {scene_token!r}")
        timestamp = _require_integer(sample, "timestamp", label)
        samples_by_scene[scene_token].append((timestamp, token, sample))

    scene_metrics: dict[str, dict[str, Any]] = {}
    sample_scene_name: dict[str, str] = {}
    for name in sorted(scenes_by_name):
        scene_token, scene = scenes_by_name[name]
        records = sorted(samples_by_scene.get(scene_token, []))
        expected_count = expected_by_name[name]
        declared_count = _require_integer(scene, "nbr_samples", f"scene[{name}]")
        if declared_count != expected_count or len(records) != expected_count:
            raise InspectionFailure(f"scene {name!r} sample count does not match its manifest")
        if len({timestamp for timestamp, _token, _sample in records}) != len(records):
            raise InspectionFailure(f"scene {name!r} has duplicate sample timestamps")
        first = _require_token(scene, "first_sample_token", f"scene[{name}]")
        last = _require_token(scene, "last_sample_token", f"scene[{name}]")
        if records[0][1] != first or records[-1][1] != last:
            raise InspectionFailure(f"scene {name!r} first/last sample tokens are inconsistent")
        for index, (_timestamp, token, sample) in enumerate(records):
            expected_prev = "" if index == 0 else records[index - 1][1]
            expected_next = "" if index + 1 == len(records) else records[index + 1][1]
            if _require_text(sample, "prev", f"sample[{token}]") != expected_prev:
                raise InspectionFailure(f"scene {name!r} has a broken sample.prev chain")
            if _require_text(sample, "next", f"sample[{token}]") != expected_next:
                raise InspectionFailure(f"scene {name!r} has a broken sample.next chain")
            sample_scene_name[token] = name
        duration_s = (records[-1][0] - records[0][0]) / 1_000_000.0
        scene_metrics[name] = {
            "sample_count": len(records),
            "first_sample_timestamp_us": records[0][0],
            "last_sample_timestamp_us": records[-1][0],
            "sample_duration_s": duration_s,
        }

    camera_sensor_tokens: dict[str, str] = {}
    for token, sensor in sensors.items():
        label = f"sensor[{token}]"
        channel = _require_text(sensor, "channel", label)
        modality = _require_text(sensor, "modality", label)
        if modality == "camera":
            if channel not in CAMERA_CHANNEL_SET:
                raise InspectionFailure(f"unexpected nuScenes camera channel {channel!r}")
            if channel in camera_sensor_tokens:
                raise InspectionFailure(f"camera channel {channel!r} has multiple sensor tokens")
            camera_sensor_tokens[channel] = token
    if set(camera_sensor_tokens) != CAMERA_CHANNEL_SET:
        raise InspectionFailure("nuScenes camera sensors are not the exact six-channel mapping")

    calibration_channel: dict[str, str] = {}
    calibration_counts: Counter[str] = Counter()
    calibration_tokens_by_channel: dict[str, set[str]] = defaultdict(set)
    for token, calibration in calibrations.items():
        label = f"calibrated_sensor[{token}]"
        sensor_token = _require_token(calibration, "sensor_token", label)
        if sensor_token not in sensors:
            raise InspectionFailure(f"{label} refers to missing sensor {sensor_token!r}")
        _require_finite_vector(calibration.get("translation"), 3, f"{label}.translation")
        _require_finite_vector(calibration.get("rotation"), 4, f"{label}.rotation")
        sensor = sensors[sensor_token]
        if sensor.get("modality") != "camera":
            continue
        channel = _require_text(sensor, "channel", f"sensor[{sensor_token}]")
        intrinsic = calibration.get("camera_intrinsic")
        if not isinstance(intrinsic, list) or len(intrinsic) != 3:
            raise InspectionFailure(f"{label}.camera_intrinsic must be a 3x3 matrix")
        for row in intrinsic:
            _require_finite_vector(row, 3, f"{label}.camera_intrinsic row")
        calibration_channel[token] = channel
        calibration_counts[channel] += 1
        calibration_tokens_by_channel[channel].add(token)
    if set(calibration_counts) != CAMERA_CHANNEL_SET:
        raise InspectionFailure("one or more camera channels have no calibration records")
    for channel in CAMERA_CHANNELS:
        if calibration_counts[channel] != expectations.camera_calibrations_per_channel:
            raise InspectionFailure(
                f"camera channel {channel!r} has {calibration_counts[channel]} calibration "
                f"records, expected {expectations.camera_calibrations_per_channel}"
            )

    for token, ego_pose in ego_poses.items():
        label = f"ego_pose[{token}]"
        _require_integer(ego_pose, "timestamp", label)
        _require_finite_vector(ego_pose.get("translation"), 3, f"{label}.translation")
        _require_finite_vector(ego_pose.get("rotation"), 4, f"{label}.rotation")

    stream_records: dict[
        tuple[str, str], list[tuple[int, str, Mapping[str, Any]]]
    ] = defaultdict(list)
    key_bundles: dict[str, dict[str, int]] = defaultdict(dict)
    camera_counts: Counter[str] = Counter()
    referenced_calibrations: dict[str, set[str]] = defaultdict(set)
    calibration_scenes: dict[str, set[str]] = defaultdict(set)
    scene_calibrations: dict[tuple[str, str], set[str]] = defaultdict(set)
    referenced_ego_poses: set[str] = set()
    referenced_image_paths: set[str] = set()
    camera_frame_count = 0
    for token, datum in sample_data.items():
        label = f"sample_data[{token}]"
        calibration_token = _require_token(datum, "calibrated_sensor_token", label)
        if calibration_token not in calibrations:
            raise InspectionFailure(
                f"{label} refers to missing calibration {calibration_token!r}"
            )
        channel = calibration_channel.get(calibration_token)
        if channel is None:
            continue
        camera_frame_count += 1
        sample_token = _require_token(datum, "sample_token", label)
        if sample_token not in sample_scene_name:
            raise InspectionFailure(f"{label} refers to missing sample {sample_token!r}")
        ego_token = _require_token(datum, "ego_pose_token", label)
        if ego_token not in ego_poses:
            raise InspectionFailure(f"{label} refers to missing ego pose {ego_token!r}")
        timestamp = _require_integer(datum, "timestamp", label)
        if ego_poses[ego_token].get("timestamp") != timestamp:
            raise InspectionFailure(f"{label} timestamp does not match its ego pose")
        filename = _require_text(datum, "filename", label)
        normalized, _encoded = _safe_member_path(filename, limits=limits)
        if normalized != filename:
            raise InspectionFailure(f"{label} image path is not canonical NFC text")
        if normalized not in member_paths:
            raise InspectionFailure(f"{label} image path is absent from the TAR manifest")
        if normalized not in regular_file_paths:
            raise InspectionFailure(f"{label} image path is not a regular TAR member")
        if regular_file_sizes.get(normalized, 0) <= 0:
            raise InspectionFailure(f"{label} image member must be non-empty")
        if _require_integer(datum, "width", label) <= 0:
            raise InspectionFailure(f"{label}.width must be positive")
        if _require_integer(datum, "height", label) <= 0:
            raise InspectionFailure(f"{label}.height must be positive")
        if _require_text(datum, "fileformat", label).lower() not in ("jpg", "jpeg"):
            raise InspectionFailure(f"{label}.fileformat is not JPEG")
        is_key_frame = datum.get("is_key_frame")
        if not isinstance(is_key_frame, bool):
            raise InspectionFailure(f"{label}.is_key_frame must be boolean")
        image_parts = PurePosixPath(normalized).parts
        expected_root = "samples" if is_key_frame else "sweeps"
        if (
            len(image_parts) != 3
            or image_parts[0] != expected_root
            or image_parts[1] != channel
            or not image_parts[2].lower().endswith(".jpg")
        ):
            raise InspectionFailure(
                f"{label} image path must be {expected_root}/{channel}/<unique>.jpg"
            )
        if normalized in referenced_image_paths:
            raise InspectionFailure(
                f"camera image member {normalized!r} is referenced more than once"
            )
        referenced_image_paths.add(normalized)
        scene_name = sample_scene_name[sample_token]
        stream_records[(scene_name, channel)].append((timestamp, token, datum))
        camera_counts[channel] += 1
        referenced_calibrations[channel].add(calibration_token)
        calibration_scenes[calibration_token].add(scene_name)
        scene_calibrations[(scene_name, channel)].add(calibration_token)
        referenced_ego_poses.add(ego_token)
        if is_key_frame:
            if channel in key_bundles[sample_token]:
                raise InspectionFailure(
                    f"sample {sample_token!r} repeats key camera channel {channel!r}"
                )
            key_bundles[sample_token][channel] = timestamp
    if camera_frame_count != expectations.camera_frame_count:
        raise InspectionFailure(
            f"camera frame count is {camera_frame_count}, expected "
            f"{expectations.camera_frame_count}"
        )
    expected_channel_counts = dict(expectations.camera_frames_by_channel)
    for channel in CAMERA_CHANNELS:
        if camera_counts[channel] != expected_channel_counts[channel]:
            raise InspectionFailure(
                f"camera channel {channel!r} has {camera_counts[channel]} frames, "
                f"expected {expected_channel_counts[channel]}"
            )
        if referenced_calibrations[channel] != calibration_tokens_by_channel[channel]:
            raise InspectionFailure(
                f"camera channel {channel!r} does not reference every declared calibration"
            )
        for token in sorted(calibration_tokens_by_channel[channel]):
            if len(calibration_scenes[token]) != 1:
                raise InspectionFailure(
                    f"camera calibration {token!r} is reused across scenes or unassigned"
                )
        for scene_name in sorted(expected_by_name):
            if len(scene_calibrations[(scene_name, channel)]) != 1:
                raise InspectionFailure(
                    f"{scene_name}/{channel} must use exactly one camera calibration"
                )

    malformed_candidates = []
    for candidate in sorted(camera_image_member_candidates):
        parts = PurePosixPath(candidate).parts
        if (
            len(parts) != 3
            or parts[1] not in CAMERA_CHANNEL_SET
            or not parts[2].lower().endswith(".jpg")
        ):
            malformed_candidates.append(candidate)
            if len(malformed_candidates) == MAX_ERRORS:
                break
    if malformed_candidates:
        raise InspectionFailure(
            "camera image directory contains malformed regular members: "
            f"{malformed_candidates}"
        )
    if referenced_image_paths != camera_image_member_candidates:
        missing = sorted(camera_image_member_candidates - referenced_image_paths)[:MAX_ERRORS]
        absent = sorted(referenced_image_paths - camera_image_member_candidates)[:MAX_ERRORS]
        raise InspectionFailure(
            "camera sample_data/image-member bijection failed: "
            f"unreferenced_member_count="
            f"{len(camera_image_member_candidates - referenced_image_paths)}, "
            f"missing_member_count="
            f"{len(referenced_image_paths - camera_image_member_candidates)}, "
            f"unreferenced_samples={missing}, missing_samples={absent}"
        )

    bundle_skews_ms: list[float] = []
    for sample_token in samples:
        bundle = key_bundles.get(sample_token, {})
        if set(bundle) != CAMERA_CHANNEL_SET:
            raise InspectionFailure(
                f"sample {sample_token!r} does not have exactly six key camera frames"
            )
        bundle_skews_ms.append((max(bundle.values()) - min(bundle.values())) / 1_000.0)

    stream_reports: dict[str, dict[str, Any]] = {}
    source_gaps_ms: list[float] = []
    source_rates_hz: list[float] = []
    selected_gaps_ms: list[float] = []
    selected_stream_p99_ms: list[float] = []
    for scene_name in sorted(expected_by_name):
        channel_reports: dict[str, Any] = {}
        for channel in CAMERA_CHANNELS:
            records = sorted(stream_records.get((scene_name, channel), []))
            timestamps = [record[0] for record in records]
            if len(timestamps) < 2 or len(set(timestamps)) != len(timestamps):
                raise InspectionFailure(
                    f"{scene_name}/{channel} needs unique, increasing camera timestamps"
                )
            gaps = [
                (later - earlier) / 1_000.0
                for earlier, later in zip(timestamps, timestamps[1:])
            ]
            if any(gap <= 0 for gap in gaps):
                raise InspectionFailure(f"{scene_name}/{channel} has non-positive gaps")
            for index, (_timestamp, token, datum) in enumerate(records):
                expected_prev = "" if index == 0 else records[index - 1][1]
                expected_next = "" if index + 1 == len(records) else records[index + 1][1]
                actual_prev = _require_text(datum, "prev", f"sample_data[{token}]")
                actual_next = _require_text(datum, "next", f"sample_data[{token}]")
                if actual_prev != expected_prev:
                    raise InspectionFailure(
                        f"{scene_name}/{channel} has a broken sample_data.prev chain"
                    )
                if actual_next != expected_next:
                    raise InspectionFailure(
                        f"{scene_name}/{channel} has a broken sample_data.next chain"
                    )
            duration_s = (timestamps[-1] - timestamps[0]) / 1_000_000.0
            rate_hz = (len(timestamps) - 1) / duration_s
            selected = _selection_only_10hz(timestamps)
            selected_gaps = [
                (later - earlier) / 1_000.0
                for earlier, later in zip(selected, selected[1:])
            ]
            source_gaps_ms.extend(gaps)
            source_rates_hz.append(rate_hz)
            selected_gaps_ms.extend(selected_gaps)
            selected_gap_summary = _metric_summary(selected_gaps)
            if selected_gap_summary["p99"] is None:
                raise InspectionFailure(
                    f"{scene_name}/{channel} has no selection-only gap evidence"
                )
            selected_stream_p99_ms.append(float(selected_gap_summary["p99"]))
            channel_reports[channel] = {
                "frame_count": len(timestamps),
                "first_timestamp_us": timestamps[0],
                "last_timestamp_us": timestamps[-1],
                "duration_s": duration_s,
                "effective_rate_hz": rate_hz,
                "source_gap_ms": _metric_summary(gaps),
                "selection_only_10hz": {
                    "operation": "native_frame_selection_only_no_retime_no_synthesis",
                    "selected_frame_count": len(selected),
                    "gap_ms": selected_gap_summary,
                },
            }
        stream_reports[scene_name] = channel_reports

    return {
        "scene_count": len(scenes),
        "sample_count": len(samples),
        "sample_data_count": len(sample_data),
        "sensor_count": len(sensors),
        "calibrated_sensor_count": len(calibrations),
        "ego_pose_count": len(ego_poses),
        "log_count": len(logs),
        "scenes": scene_metrics,
        "camera": {
            "mapping": [
                {
                    "nuscenes_channel": source,
                    "common_10hz_channel": target,
                    "sensor_token": camera_sensor_tokens[source],
                }
                for source, target in CAMERA_MAPPING
            ],
            "frame_count": camera_frame_count,
            "frame_counts_by_channel": {
                channel: camera_counts[channel] for channel in CAMERA_CHANNELS
            },
            "keyframe_bundle_count": len(key_bundles),
            "keyframe_bundle_skew_ms": _metric_summary(bundle_skews_ms),
            "bundles_over_20ms": sum(skew > 20.0 for skew in bundle_skews_ms),
            "source_effective_rate_hz": _metric_summary(source_rates_hz),
            "source_gap_ms": _metric_summary(source_gaps_ms),
            "selection_only_10hz_gap_ms_pooled": _metric_summary(selected_gaps_ms),
            "selection_only_10hz_stream_p99_ms": _metric_summary(
                selected_stream_p99_ms
            ),
            "calibration_records_referenced_by_channel": {
                channel: len(referenced_calibrations[channel])
                for channel in CAMERA_CHANNELS
            },
            "referenced_ego_pose_count": len(referenced_ego_poses),
            "streams": stream_reports,
        },
    }


def _can_correspondence(
    can_summary: Mapping[str, Any],
    scene_names: Iterable[str],
) -> dict[str, Any]:
    all_kinds = can_summary["scene_kinds"]
    scenes: dict[str, Any] = {}
    missing_correspondence: list[str] = []
    missing_routes: list[str] = []
    for scene in sorted(scene_names):
        kinds = all_kinds.get(scene, [])
        if not kinds:
            missing_correspondence.append(scene)
        route_present = "route" in kinds
        if not route_present:
            missing_routes.append(scene)
        scenes[scene] = {
            "message_kinds": kinds,
            "meta_present": "meta" in kinds,
            "route_present": route_present,
            "route_point_count": can_summary["route_point_counts"].get(scene),
        }
    if missing_correspondence:
        raise InspectionFailure(
            f"CAN-bus archive has no records for mini scenes: {missing_correspondence}"
        )
    return {
        "status": "PASS",
        "can_scene_count": len(all_kinds),
        "mini_scene_count": len(scenes),
        "missing_correspondence": missing_correspondence,
        "route_available_count": len(scenes) - len(missing_routes),
        "route_missing_count": len(missing_routes),
        "route_missing_scenes": missing_routes,
        "scenes": scenes,
    }


def _strict_json_file(path: Path, label: str) -> Mapping[str, Any]:
    descriptor, identity = _open_regular_read_only(path, label, MAX_AUDIT_REPORT_BYTES)
    try:
        with os.fdopen(os.dup(descriptor), "rb") as stream:
            payload = stream.read(MAX_AUDIT_REPORT_BYTES + 1)
        if len(payload) != identity.size:
            raise InspectionFailure(f"{label} has a short or oversized read")
        if not _same_identity(os.fstat(descriptor), identity):
            raise InspectionFailure(f"{label} changed while it was read")
    finally:
        os.close(descriptor)
    _recheck_identity(identity, label)
    try:
        parsed = json.loads(
            payload.decode("utf-8"),
            object_pairs_hook=_unique_json_object,
            parse_constant=_reject_json_constant,
        )
    except InspectionFailure:
        raise
    except Exception as error:
        raise InspectionFailure(f"{label} is not strict UTF-8 JSON: {error}") from error
    if not isinstance(parsed, dict):
        raise InspectionFailure(f"{label} root must be an object")
    return parsed


def _exact_integer(value: Any) -> int | None:
    return value if isinstance(value, int) and not isinstance(value, bool) else None


def _positive_report_integer(
    record: Mapping[str, Any],
    key: str,
    label: str,
    *,
    allow_zero: bool = False,
) -> int:
    value = _exact_integer(record.get(key))
    minimum = 0 if allow_zero else 1
    if value is None or value < minimum:
        raise InspectionFailure(f"{label}.{key} must be an integer >= {minimum}")
    return value


def _finite_report_number(record: Mapping[str, Any], key: str, label: str) -> float:
    value = record.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise InspectionFailure(f"{label}.{key} must be numeric")
    converted = float(value)
    if not math.isfinite(converted) or converted < 0:
        raise InspectionFailure(f"{label}.{key} must be finite and non-negative")
    return converted


def _identity_only(path: Path, label: str, maximum_bytes: int) -> FileIdentity:
    descriptor, identity = _open_regular_read_only(path, label, maximum_bytes)
    os.close(descriptor)
    _recheck_identity(identity, label)
    return identity


def _preflight_generic_audit(
    report_path: Path,
    archive_path: Path,
    archive_kind: str,
    limits: AuditLimits,
) -> GenericAuditProof:
    """Prove full-stream audit and exact bytes before adapter metadata parsing."""
    label = f"{archive_kind} generic audit"
    maximum_bytes = (
        limits.max_nuscenes_archive_bytes
        if archive_kind == "tar"
        else limits.max_can_archive_bytes
    )
    archive_identity = _identity_only(archive_path, f"{archive_kind} archive", maximum_bytes)
    report = _strict_json_file(report_path, f"{label} report")
    if _exact_integer(report.get("schema_version")) != 1:
        raise InspectionFailure(f"{label} must use schema_version 1")
    if report.get("status") != "PASS" or report.get("valid") is not True:
        raise InspectionFailure(f"{label} does not declare PASS and valid true")
    if report.get("read_only") is not True or report.get("extracted") is not False:
        raise InspectionFailure(f"{label} does not prove read-only verification")
    if report.get("errors") != [] or _exact_integer(report.get("error_count")) != 0:
        raise InspectionFailure(f"{label} contains verification errors")
    if report.get("errors_truncated") is not False:
        raise InspectionFailure(f"{label} has an invalid errors_truncated value")
    reported_archive = report.get("archive")
    if not isinstance(reported_archive, str):
        raise InspectionFailure(f"{label}.archive must be text")
    if _absolute_path(reported_archive) != archive_identity.path:
        raise InspectionFailure(f"{label} refers to a different archive path")
    if _exact_integer(report.get("actual_size")) != archive_identity.size:
        raise InspectionFailure(f"{label} size does not match the current archive")
    if report.get("expected_size") != report.get("actual_size"):
        raise InspectionFailure(f"{label} expected_size and actual_size differ")
    digest = report.get("actual_sha256")
    if not isinstance(digest, str) or SHA256.fullmatch(digest) is None:
        raise InspectionFailure(f"{label}.actual_sha256 is invalid")
    if report.get("expected_sha256") != digest:
        raise InspectionFailure(f"{label} expected_sha256 and actual_sha256 differ")

    if archive_kind == "tar":
        detail = report.get("tar")
        if report.get("gzip_stream_status") != "PASS" or not isinstance(detail, dict):
            raise InspectionFailure(f"{label} lacks a passing TAR+gzip stream audit")
        if detail.get("format") != "tar+gzip":
            raise InspectionFailure(f"{label} does not identify TAR+gzip format")
        if detail.get("forbidden_type_count") != 0:
            raise InspectionFailure(f"{label} reports forbidden TAR member types")
        if _positive_report_integer(detail, "member_count", label) > limits.max_tar_members:
            raise InspectionFailure(f"{label} member count exceeds adapter limits")
        if (
            _positive_report_integer(detail, "declared_regular_bytes", label, allow_zero=True)
            > limits.max_total_uncompressed_bytes
        ):
            raise InspectionFailure(f"{label} declared bytes exceed adapter limits")
        if (
            _positive_report_integer(detail, "decompressed_stream_bytes", label)
            > limits.max_total_uncompressed_bytes
        ):
            raise InspectionFailure(f"{label} decompressed stream exceeds adapter limits")
        if (
            _positive_report_integer(
                detail,
                "aggregate_normalized_path_bytes",
                label,
                allow_zero=True,
            )
            > limits.max_aggregate_path_bytes
        ):
            raise InspectionFailure(f"{label} aggregate path bytes exceed adapter limits")
        if (
            _positive_report_integer(detail, "metadata_bytes", label, allow_zero=True)
            > limits.max_metadata_total_bytes
        ):
            raise InspectionFailure(f"{label} TAR metadata bytes exceed adapter limits")
        if (
            _finite_report_number(detail, "decompressed_stream_compression_ratio", label)
            > limits.max_total_compression_ratio
        ):
            raise InspectionFailure(f"{label} compression ratio exceeds adapter limits")
    elif archive_kind == "zip":
        detail = report.get("zip")
        if not isinstance(detail, dict):
            raise InspectionFailure(f"{label} lacks ZIP detail")
        payload_crc = report.get("payload_crc")
        if not isinstance(payload_crc, dict):
            raise InspectionFailure(f"{label} lacks payload CRC detail")
        if payload_crc.get("requested") is not True or payload_crc.get("status") != "PASS":
            raise InspectionFailure(f"{label} lacks a passing full payload CRC audit")
        member_count = _positive_report_integer(detail, "member_count", label)
        declared_bytes = _positive_report_integer(
            detail,
            "declared_uncompressed_bytes",
            label,
            allow_zero=True,
        )
        if member_count > limits.max_zip_members:
            raise InspectionFailure(f"{label} member count exceeds adapter limits")
        if declared_bytes > limits.max_total_uncompressed_bytes:
            raise InspectionFailure(f"{label} declared bytes exceed adapter limits")
        if (
            _positive_report_integer(detail, "central_directory_bytes", label)
            > limits.max_zip_central_directory_bytes
        ):
            raise InspectionFailure(f"{label} central directory exceeds adapter limits")
        if (
            _finite_report_number(detail, "compression_ratio", label)
            > limits.max_total_compression_ratio
        ):
            raise InspectionFailure(f"{label} compression ratio exceeds adapter limits")
        if _exact_integer(payload_crc.get("verified_members")) != member_count:
            raise InspectionFailure(f"{label} payload member count differs")
        if _exact_integer(payload_crc.get("verified_uncompressed_bytes")) != declared_bytes:
            raise InspectionFailure(f"{label} payload byte count differs")
    else:
        raise InspectionFailure(f"unknown generic audit kind {archive_kind!r}")

    current_digest = _sha256_same_identity(archive_identity, f"archive bound to {label}")
    if current_digest != digest:
        raise InspectionFailure(
            f"{label} actual_sha256 does not match the currently inspected archive"
        )
    return GenericAuditProof(
        report_path=report_path,
        report=report,
        archive_identity=archive_identity,
        archive_kind=archive_kind,
        digest=digest,
    )


def _bind_generic_audit_summary(
    proof: GenericAuditProof,
    archive_identity: FileIdentity,
    archive_summary: Mapping[str, Any],
) -> dict[str, Any]:
    """Bind the preflight proof to the independently parsed archive manifest."""
    label = f"{proof.archive_kind} generic audit"
    if proof.archive_identity.signature() != archive_identity.signature():
        raise InspectionFailure(f"{label} archive identity changed after preflight")
    detail = proof.report.get(proof.archive_kind)
    if not isinstance(detail, dict):
        raise InspectionFailure(f"{label} detail disappeared after preflight")
    if proof.archive_kind == "tar":
        expected_fields = {
            "member_count": archive_summary["member_count"],
            "file_count": archive_summary["file_count"],
            "directory_count": archive_summary["directory_count"],
            "declared_regular_bytes": archive_summary["declared_regular_bytes"],
        }
    else:
        expected_fields = {
            "member_count": archive_summary["member_count"],
            "file_count": archive_summary["file_count"],
            "directory_count": archive_summary["directory_count"],
            "declared_uncompressed_bytes": archive_summary["declared_uncompressed_bytes"],
            "central_directory_bytes": archive_summary["central_directory_bytes"],
            "central_directory_offset": archive_summary["central_directory_offset"],
        }
    for key, value in expected_fields.items():
        if _exact_integer(detail.get(key)) != value:
            raise InspectionFailure(f"{label} {proof.archive_kind}.{key} differs")
    if detail.get("top_level_entries") != archive_summary["top_level_entries"]:
        raise InspectionFailure(f"{label} top-level entry counts differ")
    _recheck_identity(archive_identity, f"archive bound to {label}")
    return {
        "status": "PASS",
        "report": str(proof.report_path),
        "archive_sha256": proof.digest,
        "archive_raw_bytes_hashed": True,
        "full_payload_safety_preflight": True,
    }


def _common10_qualification(
    nuscenes: Mapping[str, Any],
    correspondence: Mapping[str, Any],
) -> dict[str, Any]:
    durations = [scene["sample_duration_s"] for scene in nuscenes["scenes"].values()]
    camera = nuscenes["camera"]
    rates = camera["source_effective_rate_hz"]
    selection_stream_p99 = camera["selection_only_10hz_stream_p99_ms"]
    selection_gaps_pooled = camera["selection_only_10hz_gap_ms_pooled"]
    bundle_skews = camera["keyframe_bundle_skew_ms"]
    duration_pass = bool(durations) and min(durations) >= 30.0
    native_rate_pass = bool(rates["count"]) and rates["min"] >= 9.5 and rates["max"] <= 10.5
    selection_p99_pass = (
        selection_stream_p99["max"] is not None
        and selection_stream_p99["max"] <= 150.0
    )
    selection_max_pass = (
        selection_gaps_pooled["max"] is not None
        and selection_gaps_pooled["max"] <= 250.0
    )
    bundle_pass = bundle_skews["max"] is not None and bundle_skews["max"] <= 20.0
    route_available = correspondence["route_missing_count"] == 0
    gates = {
        "minimum_episode_duration_30s": {
            "status": "PASS" if duration_pass else "FAIL",
            "minimum_observed_s": min(durations) if durations else None,
        },
        "native_camera_rate_10hz": {
            "status": "PASS" if native_rate_pass else "FAIL",
            "observed_effective_rate_hz": rates,
            "reason": "nuScenes cameras are asynchronous approximately-12-Hz source streams",
        },
        "selection_only_max_stream_p99_gap_150ms": {
            "status": "PASS" if selection_p99_pass else "FAIL",
            "observed_max_stream_p99_ms": selection_stream_p99["max"],
            "stream_p99_summary_ms": selection_stream_p99,
            "operation": "native_frame_selection_only_no_retime_no_synthesis",
        },
        "selection_only_absolute_gap_250ms": {
            "status": "PASS" if selection_max_pass else "FAIL",
            "observed_max_ms": selection_gaps_pooled["max"],
        },
        "six_camera_bundle_skew_20ms": {
            "status": "PASS" if bundle_pass else "FAIL",
            "observed_max_ms": bundle_skews["max"],
        },
        "can_route_available_for_each_scene": {
            "status": "PASS" if route_available else "FAIL",
            "missing_scenes": correspondence["route_missing_scenes"],
        },
        "canonical_route_reconstruction": {
            "status": "NOT_RUN",
            "reason": "route projection and source-manifest binding require a prepared adapter",
        },
    }
    qualified = all(value["status"] == "PASS" for value in gates.values())
    return {
        "status": "QUALIFIED" if qualified else "NOT_QUALIFIED",
        "qualified": qualified,
        "contract_id": "common_10hz_v1",
        "allowed_use": "planning_training" if qualified else "schema_adapter_smoke_only",
        "gates": gates,
        "policy": {
            "retimed_frames": 0,
            "invented_or_duplicated_frames": 0,
            "cross_scene_concatenation": False,
            "selection_only_10hz": {
                "id": "first_timestamp_nearest_unused_v1",
                "grid_period_us": 100_000,
                "grid_anchor": "first_source_timestamp_per_scene_camera_stream",
                "selection": "nearest_unused_native_frame_at_or_after_previous_choice",
                "equal_distance_tie_break": "earlier_native_timestamp",
                "percentile_method": "nearest_rank_ceiling",
            },
        },
    }


def _public_tar_summary(summary: Mapping[str, Any]) -> dict[str, Any]:
    return {
        key: summary[key]
        for key in (
            "member_count",
            "file_count",
            "directory_count",
            "declared_regular_bytes",
            "top_level_entries",
            "metadata_members_read",
            "metadata_bytes_read",
            "image_payloads_read",
        )
    }


def _public_zip_summary(summary: Mapping[str, Any]) -> dict[str, Any]:
    return {
        key: summary[key]
        for key in (
            "member_count",
            "file_count",
            "directory_count",
            "declared_uncompressed_bytes",
            "declared_compressed_bytes",
            "central_directory_bytes",
            "central_directory_offset",
            "top_level_entries",
            "route_members_read",
            "other_payloads_read",
        )
    }


def _failure_report(
    nuscenes_path: Path | str,
    can_path: Path | str,
    message: str,
) -> dict[str, Any]:
    return {
        "schema_version": 1,
        "status": "FAIL",
        "valid": False,
        "structural_validation": {"status": "FAIL"},
        "common_10hz_v1": {
            "status": "NOT_EVALUATED",
            "qualified": False,
        },
        "nuscenes_archive": str(nuscenes_path),
        "can_bus_archive": str(can_path),
        "read_only": True,
        "extracted": False,
        "image_payloads_read": 0,
        "errors": [message],
        "error_count": 1,
        "errors_truncated": False,
    }


def inspect_nuscenes_mini_adapter(
    nuscenes_archive: Path | str,
    can_bus_archive: Path | str,
    *,
    nuscenes_audit_report: Path | str | None = None,
    can_bus_audit_report: Path | str | None = None,
    limits: AuditLimits = DEFAULT_LIMITS,
    expectations: MiniExpectations = DEFAULT_EXPECTATIONS,
) -> dict[str, Any]:
    """Inspect official-mini metadata and CAN routes without archive extraction."""
    try:
        _validate_limits(limits)
        _validate_expectations(expectations)
        if nuscenes_audit_report is None or can_bus_audit_report is None:
            raise InspectionFailure(
                "both passing generic TAR and full-payload ZIP audit reports are required"
            )
        nuscenes_path = _absolute_path(nuscenes_archive)
        can_path = _absolute_path(can_bus_archive)
        nuscenes_audit_path = _absolute_path(nuscenes_audit_report)
        can_audit_path = _absolute_path(can_bus_audit_report)

        tar_proof = _preflight_generic_audit(
            nuscenes_audit_path,
            nuscenes_path,
            "tar",
            limits,
        )
        zip_proof = _preflight_generic_audit(
            can_audit_path,
            can_path,
            "zip",
            limits,
        )
        tar_summary, tar_identity = _scan_nuscenes_tar(nuscenes_path, limits)
        nuscenes = _inspect_nuscenes_tables(tar_summary, expectations, limits)
        scene_names = frozenset(nuscenes["scenes"])
        zip_summary, zip_identity = _scan_can_zip(can_path, scene_names, limits)
        correspondence = _can_correspondence(zip_summary, scene_names)
        generic_audits = {
            "nuscenes": _bind_generic_audit_summary(
                tar_proof,
                tar_identity,
                tar_summary,
            ),
            "can_bus": _bind_generic_audit_summary(
                zip_proof,
                zip_identity,
                zip_summary,
            ),
        }
        _recheck_identity(tar_identity, "nuScenes archive")
        _recheck_identity(zip_identity, "CAN-bus archive")

        qualification = _common10_qualification(nuscenes, correspondence)
        return {
            "schema_version": 1,
            "status": "PASS",
            "valid": True,
            "structural_validation": {"status": "PASS"},
            "common_10hz_v1": qualification,
            "nuscenes_archive": str(nuscenes_path),
            "can_bus_archive": str(can_path),
            "archive_audits": generic_audits,
            "nuscenes_tar": _public_tar_summary(tar_summary),
            "can_bus_zip": _public_zip_summary(zip_summary),
            "dataset": nuscenes,
            "can_correspondence": correspondence,
            "scope": {
                "camera_sample_data_stream_semantics": "PASS",
                "lidar_radar_sample_data_stream_semantics": {
                    "status": "NOT_EVALUATED_OUT_OF_SCOPE",
                    "reason": (
                        "this adapter audit targets the six-camera planning input; "
                        "the mandatory generic audits still cover every archive payload"
                    ),
                },
                "can_zip_container_profile": {
                    "status": "PASS",
                    "profile": "classic_single_disk_zip",
                    "zip64_policy": "FAIL_CLOSED_OUT_OF_PINNED_PROFILE",
                    "raw_eocd_bounded_before_zipfile": True,
                },
            },
            "read_only": True,
            "extracted": False,
            "image_payloads_read": 0,
            "errors": [],
            "error_count": 0,
            "errors_truncated": False,
        }
    except InspectionFailure as error:
        return _failure_report(nuscenes_archive, can_bus_archive, str(error))
    except Exception as error:
        return _failure_report(
            nuscenes_archive,
            can_bus_archive,
            f"unexpected fail-closed inspection error: {type(error).__name__}: {error}",
        )


def _parser() -> argparse.ArgumentParser:
    parser = JsonArgumentParser(
        description=(
            "Read-only nuScenes v1.0-mini/CAN metadata audit; never extracts or decodes images."
        )
    )
    parser.add_argument("nuscenes_archive", type=Path)
    parser.add_argument("can_bus_archive", type=Path)
    parser.add_argument("--nuscenes-audit-report", required=True, type=Path)
    parser.add_argument("--can-bus-audit-report", required=True, type=Path)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    try:
        args = _parser().parse_args(argv)
        report = inspect_nuscenes_mini_adapter(
            args.nuscenes_archive,
            args.can_bus_archive,
            nuscenes_audit_report=args.nuscenes_audit_report,
            can_bus_audit_report=args.can_bus_audit_report,
        )
    except InspectionFailure as error:
        report = _failure_report("", "", str(error))
    json.dump(report, sys.stdout, indent=2, sort_keys=True, allow_nan=False)
    sys.stdout.write("\n")
    return 0 if report["valid"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
