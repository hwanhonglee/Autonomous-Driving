#!/usr/bin/env python3
"""Read-only structural inspection for the pinned nuPlan v1.1 mini subset.

This tool deliberately does not extract files or open the SQLite databases.  It
checks the exact database-log manifest and the exact camera-group-0 directory
shape from ZIP central-directory metadata.  Optional reports produced by
``verify_zip_archive.py --verify-payload-crc`` can be bound to the archives as
additional evidence that payload CRCs and archive hashes were already checked.
"""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path, PureWindowsPath
import re
import stat
import sys
from typing import Any, Callable, Iterable, Mapping, Sequence
import unicodedata
import zipfile


MAX_ERRORS = 100
MAX_PATH_BYTES = 4096
MAX_COMPONENT_BYTES = 255
MAX_AUDIT_REPORT_BYTES = 4 * 1024 * 1024
ARCHIVE_HASH_CHUNK_BYTES = 8 * 1024 * 1024

DATABASE_DIRECTORY = "data/cache/mini/"
CAMERA_TOP_DIRECTORY = "nuplan-v1.1_mini_camera_0"
SQLITE_VALIDATION_STATUS = "NOT_RUN_REQUIRES_SAFE_STAGING"
JPEG_NAME = re.compile(r"[0-9a-f]{16}\.jpg", re.ASCII)
SUPPORTED_COMPRESSION_METHODS = {
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


EXPECTED_DATABASE_STEMS = frozenset(
    {
        "2021.05.12.22.00.38_veh-35_01008_01518",
        "2021.05.12.22.28.35_veh-35_00620_01164",
        "2021.05.12.23.36.44_veh-35_00152_00504",
        "2021.05.12.23.36.44_veh-35_01133_01535",
        "2021.05.12.23.36.44_veh-35_02035_02387",
        "2021.05.25.14.16.10_veh-35_01690_02183",
        "2021.06.03.12.02.06_veh-35_00233_00609",
        "2021.06.03.13.55.17_veh-35_00073_00426",
        "2021.06.07.12.54.00_veh-35_01843_02314",
        "2021.06.07.18.53.26_veh-26_00005_00427",
        "2021.06.08.12.54.54_veh-26_04262_04732",
        "2021.06.08.14.35.24_veh-26_02555_03004",
        "2021.06.08.16.31.33_veh-38_01589_02072",
        "2021.06.09.11.54.15_veh-12_04366_04810",
        "2021.06.09.12.39.51_veh-26_01943_02303",
        "2021.06.09.12.39.51_veh-26_05620_06003",
        "2021.06.09.14.03.17_veh-12_02584_02970",
        "2021.06.09.14.58.55_veh-35_01095_01484",
        "2021.06.09.14.58.55_veh-35_01894_02311",
        "2021.06.09.17.23.18_veh-38_00773_01140",
        "2021.06.09.17.23.18_veh-38_02526_03027",
        "2021.06.09.17.37.09_veh-12_00404_00864",
        "2021.06.14.16.32.09_veh-35_05038_05402",
        "2021.06.14.16.48.02_veh-12_04057_04438",
        "2021.06.14.16.48.02_veh-12_04978_05337",
        "2021.06.14.17.26.26_veh-38_04544_04920",
        "2021.06.14.18.33.41_veh-35_03901_04264",
        "2021.06.14.18.42.45_veh-12_03445_03902",
        "2021.06.14.19.22.11_veh-38_01480_01860",
        "2021.06.23.15.56.12_veh-16_00839_01285",
        "2021.06.23.16.54.19_veh-35_00808_01256",
        "2021.06.23.17.31.36_veh-16_00016_00377",
        "2021.06.23.20.43.31_veh-16_03607_04007",
        "2021.06.28.15.02.02_veh-38_02398_02848",
        "2021.06.28.16.29.11_veh-38_01415_01821",
        "2021.06.28.16.29.11_veh-38_03263_03766",
        "2021.06.28.16.57.59_veh-26_00016_00484",
        "2021.07.09.17.06.37_veh-35_00258_00748",
        "2021.07.09.20.59.12_veh-38_01208_01692",
        "2021.07.16.00.51.05_veh-17_01352_01901",
        "2021.07.16.18.06.21_veh-38_03231_03712",
        "2021.07.16.18.06.21_veh-38_04471_04922",
        "2021.07.16.18.06.21_veh-38_04933_05307",
        "2021.07.16.18.19.22_veh-35_00440_00858",
        "2021.07.16.20.45.29_veh-35_00600_01084",
        "2021.07.16.20.45.29_veh-35_01095_01486",
        "2021.07.24.20.37.45_veh-17_00015_00375",
        "2021.07.24.23.50.16_veh-17_01696_02071",
        "2021.08.09.17.55.59_veh-28_00021_00307",
        "2021.08.17.16.57.11_veh-08_01200_01636",
        "2021.08.17.17.17.01_veh-45_02314_02798",
        "2021.08.17.18.54.02_veh-45_00665_01065",
        "2021.08.24.13.12.55_veh-45_00386_00472",
        "2021.08.30.14.54.34_veh-40_00439_00835",
        "2021.09.16.15.12.03_veh-42_01037_01434",
        "2021.10.01.19.16.42_veh-28_02011_02410",
        "2021.10.01.19.16.42_veh-28_03307_03808",
        "2021.10.05.07.10.04_veh-52_01442_01802",
        "2021.10.06.07.26.10_veh-52_00006_00398",
        "2021.10.06.17.43.07_veh-28_00508_00877",
        "2021.10.11.02.57.41_veh-50_00352_00535",
        "2021.10.11.02.57.41_veh-50_01522_02088",
        "2021.10.11.07.12.18_veh-50_00211_00304",
        "2021.10.11.08.31.07_veh-50_01750_01948",
    }
)

EXPECTED_CAMERA_LOGS = frozenset(
    {
        "2021.05.12.22.00.38_veh-35_01008_01518",
        "2021.05.12.22.28.35_veh-35_00620_01164",
        "2021.05.12.23.36.44_veh-35_00152_00504",
        "2021.05.12.23.36.44_veh-35_01133_01535",
        "2021.05.12.23.36.44_veh-35_02035_02387",
        "2021.05.25.14.16.10_veh-35_01690_02183",
        "2021.06.03.12.02.06_veh-35_00233_00609",
    }
)

EXPECTED_CAMERA_CHANNELS = (
    "CAM_B0",
    "CAM_F0",
    "CAM_L0",
    "CAM_L1",
    "CAM_L2",
    "CAM_R0",
    "CAM_R1",
    "CAM_R2",
)

EXPECTED_JPEGS_PER_CHANNEL_BY_LOG = (
    ("2021.05.12.22.00.38_veh-35_01008_01518", 5100),
    ("2021.05.12.22.28.35_veh-35_00620_01164", 5440),
    ("2021.05.12.23.36.44_veh-35_00152_00504", 3520),
    ("2021.05.12.23.36.44_veh-35_01133_01535", 4020),
    ("2021.05.12.23.36.44_veh-35_02035_02387", 3520),
    ("2021.05.25.14.16.10_veh-35_01690_02183", 4930),
    ("2021.06.03.12.02.06_veh-35_00233_00609", 3760),
)


@dataclass(frozen=True)
class SubsetExpectations:
    """Pinned archive expectations; custom values are useful for tiny tests only."""

    database_stems: frozenset[str]
    camera_logs: frozenset[str]
    camera_channels: tuple[str, ...]
    jpegs_per_channel_by_log: tuple[tuple[str, int], ...]

    def jpeg_counts(self) -> dict[str, int]:
        return dict(self.jpegs_per_channel_by_log)


DEFAULT_EXPECTATIONS = SubsetExpectations(
    database_stems=EXPECTED_DATABASE_STEMS,
    camera_logs=EXPECTED_CAMERA_LOGS,
    camera_channels=EXPECTED_CAMERA_CHANNELS,
    jpegs_per_channel_by_log=EXPECTED_JPEGS_PER_CHANNEL_BY_LOG,
)


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
        return (self.device, self.inode, self.size, self.mtime_ns, self.ctime_ns)


class InspectionFailure(Exception):
    """A deterministic validation failure safe to include in JSON output."""


class ErrorCollector:
    """Retain a bounded error sample while counting every detected failure."""

    def __init__(self) -> None:
        self.items: list[str] = []
        self.total = 0

    def add(self, message: str) -> None:
        self.total += 1
        if len(self.items) < MAX_ERRORS:
            self.items.append(message)


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


def _open_regular_read_only(path: Path, label: str) -> tuple[int, FileIdentity]:
    try:
        path_metadata = os.lstat(path)
    except OSError as error:
        raise InspectionFailure(f"{label} cannot be inspected: {error}") from error
    if stat.S_ISLNK(path_metadata.st_mode):
        raise InspectionFailure(f"{label} path must not be a symbolic link")
    if not stat.S_ISREG(path_metadata.st_mode):
        raise InspectionFailure(f"{label} path must be a regular file")

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
        opened_metadata = os.fstat(descriptor)
        if not stat.S_ISREG(opened_metadata.st_mode):
            raise InspectionFailure(f"opened {label} is not a regular file")
        if (opened_metadata.st_dev, opened_metadata.st_ino) != (
            path_metadata.st_dev,
            path_metadata.st_ino,
        ):
            raise InspectionFailure(f"{label} path changed before it could be opened")
        return descriptor, FileIdentity.from_stat(path, opened_metadata)
    except Exception:
        os.close(descriptor)
        raise


def _recheck_identity(identity: FileIdentity, label: str) -> None:
    try:
        metadata = os.lstat(identity.path)
    except OSError as error:
        raise InspectionFailure(f"{label} path changed during inspection: {error}") from error
    if stat.S_ISLNK(metadata.st_mode):
        raise InspectionFailure(f"{label} path became a symbolic link during inspection")
    if not _same_identity(metadata, identity):
        raise InspectionFailure(f"{label} changed or now refers to a different inode")


def _sha256_same_identity(identity: FileIdentity, label: str) -> str:
    """Hash the archive while requiring the identity seen by the manifest scan."""
    descriptor, opened_identity = _open_regular_read_only(identity.path, label)
    try:
        if opened_identity.signature() != identity.signature():
            raise InspectionFailure(f"{label} changed before it could be hashed")
        digest = hashlib.sha256()
        try:
            while True:
                block = os.read(descriptor, ARCHIVE_HASH_CHUNK_BYTES)
                if not block:
                    break
                digest.update(block)
            after = os.fstat(descriptor)
        except OSError as error:
            raise InspectionFailure(f"{label} could not be hashed: {error}") from error
        if not _same_identity(after, identity):
            raise InspectionFailure(f"{label} changed while it was hashed")
        _recheck_identity(identity, label)
        return digest.hexdigest()
    finally:
        os.close(descriptor)


def _member_kind(info: zipfile.ZipInfo, label: str, errors: ErrorCollector) -> str:
    kind = "directory" if info.is_dir() else "file"
    unix_mode = (info.external_attr >> 16) & 0xFFFF
    file_type = stat.S_IFMT(unix_mode)
    if info.create_system == 3:
        if stat.S_ISLNK(unix_mode):
            errors.add(f"{label} member {info.filename!r} is a symbolic link")
        elif file_type not in (0, stat.S_IFREG, stat.S_IFDIR):
            errors.add(f"{label} member {info.filename!r} has a forbidden special-file mode")
        if info.is_dir() and file_type not in (0, stat.S_IFDIR):
            errors.add(f"{label} member {info.filename!r} has conflicting type metadata")
        if not info.is_dir() and file_type == stat.S_IFDIR:
            errors.add(f"{label} member {info.filename!r} has conflicting type metadata")
    return kind


def _safe_member_name(info: zipfile.ZipInfo, label: str, errors: ErrorCollector) -> str | None:
    original = getattr(info, "orig_filename", info.filename)
    if not isinstance(original, str) or not original:
        errors.add(f"{label} contains an empty or non-text member name")
        return None
    if "\x00" in original:
        errors.add(f"{label} member name {original!r} contains a NUL byte")
        return None
    if any(ord(character) < 32 or ord(character) == 127 for character in original):
        errors.add(f"{label} member name {original!r} contains a control character")
        return None
    if "\\" in original:
        errors.add(f"{label} member name {original!r} contains a backslash")
        return None
    try:
        encoded = original.encode("utf-8")
    except UnicodeEncodeError:
        errors.add(f"{label} member name {original!r} is not valid Unicode text")
        return None
    if len(encoded) > MAX_PATH_BYTES:
        errors.add(f"{label} member name {original!r} exceeds the path-length limit")
        return None
    windows_path = PureWindowsPath(original)
    if original.startswith("/") or windows_path.is_absolute() or windows_path.drive:
        errors.add(f"{label} member name {original!r} is absolute or drive-qualified")
        return None

    trimmed = original[:-1] if original.endswith("/") else original
    if not trimmed:
        errors.add(f"{label} member name {original!r} does not identify a safe path")
        return None
    components = trimmed.split("/")
    if any(component in ("", ".", "..") for component in components):
        errors.add(f"{label} member name {original!r} has an unsafe path component")
        return None
    for component in components:
        if len(component.encode("utf-8")) > MAX_COMPONENT_BYTES:
            errors.add(f"{label} member name {original!r} has an overlong component")
            return None
        if component != component.strip() or component.endswith((".", " ")):
            errors.add(f"{label} member name {original!r} has ambiguous whitespace or dots")
            return None
        if ":" in component:
            errors.add(f"{label} member name {original!r} has an alternate-stream separator")
            return None
        if component.rstrip(" .").split(".", 1)[0].upper() in WINDOWS_RESERVED_NAMES:
            errors.add(f"{label} member name {original!r} has a reserved device component")
            return None
    normalized = unicodedata.normalize("NFC", trimmed)
    if normalized != trimmed:
        errors.add(f"{label} member name {original!r} is not NFC-normalized")
        return None
    return original


def _validated_members(
    archive: zipfile.ZipFile,
    label: str,
    errors: ErrorCollector,
) -> tuple[list[tuple[zipfile.ZipInfo, str, str]], dict[str, Any]]:
    members = archive.infolist()
    exact_names: set[str] = set()
    portable_names: dict[str, str] = {}
    header_offsets: set[int] = set()
    records: list[tuple[zipfile.ZipInfo, str, str]] = []
    files = 0
    directories = 0
    declared_bytes = 0
    top_levels: Counter[str] = Counter()

    for info in members:
        name = _safe_member_name(info, label, errors)
        kind = _member_kind(info, label, errors)
        original = getattr(info, "orig_filename", info.filename)
        if original in exact_names:
            errors.add(f"{label} contains duplicate member name {original!r}")
        exact_names.add(original)
        if isinstance(original, str):
            portable = unicodedata.normalize("NFC", original.rstrip("/")).casefold()
            previous = portable_names.get(portable)
            if previous is not None and previous != original:
                errors.add(
                    f"{label} has a portable path collision between {previous!r} and {original!r}"
                )
            portable_names.setdefault(portable, original)
        if info.header_offset in header_offsets:
            errors.add(f"{label} has multiple members at header offset {info.header_offset}")
        header_offsets.add(info.header_offset)
        if info.header_offset < 0:
            errors.add(f"{label} member {original!r} has a negative header offset")
        if getattr(info, "volume", 0) != 0:
            errors.add(f"{label} member {original!r} belongs to a split archive")
        if info.flag_bits & 0x1:
            errors.add(f"{label} member {original!r} is encrypted")
        if info.compress_type not in SUPPORTED_COMPRESSION_METHODS:
            errors.add(
                f"{label} member {original!r} uses unsupported compression "
                f"method {info.compress_type}"
            )
        if kind == "directory":
            directories += 1
            if info.file_size != 0:
                errors.add(f"{label} directory {original!r} declares payload bytes")
        else:
            files += 1
        declared_bytes += info.file_size
        if name is not None:
            top_levels[name.rstrip("/").split("/", 1)[0]] += 1
            records.append((info, name, kind))

    return records, {
        "member_count": len(members),
        "file_count": files,
        "directory_count": directories,
        "declared_uncompressed_bytes": declared_bytes,
        "top_level_entries": dict(sorted(top_levels.items())),
    }


def _scan_database_members(
    archive: zipfile.ZipFile,
    expectations: SubsetExpectations,
    errors: ErrorCollector,
) -> dict[str, Any]:
    records, common = _validated_members(archive, "database archive", errors)
    stem_counts: Counter[str] = Counter()
    database_directory_count = 0
    license_count = 0
    unexpected_samples: list[str] = []
    unexpected_total = 0

    for info, name, kind in records:
        if name == DATABASE_DIRECTORY and kind == "directory":
            database_directory_count += 1
            continue
        if name == "LICENSE" and kind == "file":
            license_count += 1
            if info.file_size <= 0:
                errors.add("database archive LICENSE must not be empty")
            continue
        prefix = DATABASE_DIRECTORY
        if kind == "file" and name.startswith(prefix) and name.endswith(".db"):
            stem = name[len(prefix) : -3]
            if stem and "/" not in stem:
                stem_counts[stem] += 1
                if info.file_size <= 0:
                    errors.add(f"database archive DB member {name!r} must not be empty")
                continue
        unexpected_total += 1
        if len(unexpected_samples) < MAX_ERRORS:
            unexpected_samples.append(name)
        errors.add(f"database archive contains unexpected member {name!r}")

    expected = expectations.database_stems
    observed = set(stem_counts)
    missing = sorted(expected - observed)
    unexpected_stems = sorted(observed - expected)
    for stem in missing:
        errors.add(f"database archive is missing expected DB stem {stem!r}")
    for stem in unexpected_stems:
        errors.add(f"database archive contains unexpected DB stem {stem!r}")
    for stem, count in sorted(stem_counts.items()):
        if count != 1:
            errors.add(f"database archive DB stem {stem!r} occurs {count} times, expected 1")
    if database_directory_count != 1:
        errors.add(
            "database archive directory marker occurs "
            f"{database_directory_count} times, expected 1"
        )
    if license_count != 1:
        errors.add(f"database archive LICENSE occurs {license_count} times, expected 1")
    expected_members = len(expected) + 2
    if common["member_count"] != expected_members:
        errors.add(
            f"database archive has {common['member_count']} members, expected {expected_members}"
        )

    common.update(
        {
            "expected_member_count": expected_members,
            "database_count": sum(stem_counts.values()),
            "expected_database_count": len(expected),
            "full_stems": sorted(observed & expected),
            "missing_full_stems": missing,
            "unexpected_full_stems": unexpected_stems,
            "unexpected_member_count": unexpected_total,
            "unexpected_member_samples": unexpected_samples,
            "stem_occurrences": {
                stem: stem_counts.get(stem, 0) for stem in sorted(expected)
            },
            "sqlite_validation": {
                "status": SQLITE_VALIDATION_STATUS,
                "reason": "DB payloads are never extracted or opened by this inspector",
            },
        }
    )
    return common


def _scan_camera_members(
    archive: zipfile.ZipFile,
    expectations: SubsetExpectations,
    errors: ErrorCollector,
) -> dict[str, Any]:
    records, common = _validated_members(archive, "camera archive", errors)
    logs = expectations.camera_logs
    channels = set(expectations.camera_channels)
    expected_frames = expectations.jpeg_counts()
    root_count = 0
    license_count = 0
    log_directory_counts: Counter[str] = Counter()
    channel_directory_counts: Counter[tuple[str, str]] = Counter()
    jpeg_counts: Counter[tuple[str, str]] = Counter()
    observed_logs: set[str] = set()
    observed_channels: set[str] = set()
    unexpected_samples: list[str] = []
    unexpected_total = 0

    def unexpected(name: str, reason: str) -> None:
        nonlocal unexpected_total
        unexpected_total += 1
        if len(unexpected_samples) < MAX_ERRORS:
            unexpected_samples.append(name)
        errors.add(f"camera archive member {name!r} {reason}")

    for info, name, kind in records:
        if name == "LICENSE" and kind == "file":
            license_count += 1
            if info.file_size <= 0:
                errors.add("camera archive LICENSE must not be empty")
            continue
        trimmed = name.rstrip("/")
        parts = trimmed.split("/")
        if parts[0] != CAMERA_TOP_DIRECTORY:
            unexpected(name, "has an unexpected top-level path")
            continue
        if kind == "directory" and len(parts) == 1 and name.endswith("/"):
            root_count += 1
            continue
        if len(parts) < 2 or parts[1] not in logs:
            unexpected(name, "has an unexpected camera log")
            continue
        log = parts[1]
        observed_logs.add(log)
        if kind == "directory" and len(parts) == 2 and name.endswith("/"):
            log_directory_counts[log] += 1
            continue
        if len(parts) < 3 or parts[2] not in channels:
            unexpected(name, "has an unexpected camera channel")
            continue
        channel = parts[2]
        observed_channels.add(channel)
        if kind == "directory" and len(parts) == 3 and name.endswith("/"):
            channel_directory_counts[(log, channel)] += 1
            continue
        if (
            kind != "file"
            or len(parts) != 4
            or name.endswith("/")
            or JPEG_NAME.fullmatch(parts[3]) is None
        ):
            unexpected(name, "does not match top/log/channel/16-lowercase-hex.jpg")
            continue
        if info.file_size <= 0:
            errors.add(f"camera archive JPEG member {name!r} must not be empty")
        jpeg_counts[(log, channel)] += 1

    for log in sorted(logs):
        if log_directory_counts[log] != 1:
            errors.add(
                f"camera archive log directory {log!r} occurs "
                f"{log_directory_counts[log]} times, expected 1"
            )
        for channel in expectations.camera_channels:
            directory_count = channel_directory_counts[(log, channel)]
            if directory_count != 1:
                errors.add(
                    f"camera archive directory {log}/{channel} occurs "
                    f"{directory_count} times, expected 1"
                )
            actual_count = jpeg_counts[(log, channel)]
            expected_count = expected_frames[log]
            if actual_count != expected_count:
                errors.add(
                    f"camera archive {log}/{channel} has {actual_count} JPEGs, "
                    f"expected {expected_count}"
                )
    if root_count != 1:
        errors.add(f"camera archive root directory occurs {root_count} times, expected 1")
    if license_count != 1:
        errors.add(f"camera archive LICENSE occurs {license_count} times, expected 1")
    missing_logs = sorted(logs - observed_logs)
    missing_channels = sorted(channels - observed_channels)
    for log in missing_logs:
        errors.add(f"camera archive is missing expected log {log!r}")
    for channel in missing_channels:
        errors.add(f"camera archive is missing expected channel {channel!r}")

    expected_jpegs = sum(expected_frames[log] for log in logs) * len(channels)
    expected_members = 2 + len(logs) + len(logs) * len(channels) + expected_jpegs
    if common["member_count"] != expected_members:
        errors.add(
            f"camera archive has {common['member_count']} members, expected {expected_members}"
        )
    nested_counts = {
        log: {
            channel: jpeg_counts[(log, channel)] for channel in expectations.camera_channels
        }
        for log in sorted(logs)
    }
    common.update(
        {
            "top_directory": CAMERA_TOP_DIRECTORY,
            "expected_member_count": expected_members,
            "jpeg_count": sum(jpeg_counts.values()),
            "expected_jpeg_count": expected_jpegs,
            "logs": sorted(observed_logs & logs),
            "expected_logs": sorted(logs),
            "missing_logs": missing_logs,
            "channels": sorted(observed_channels & channels),
            "expected_channels": sorted(channels),
            "missing_channels": missing_channels,
            "jpeg_counts_by_log_channel": nested_counts,
            "unexpected_member_count": unexpected_total,
            "unexpected_member_samples": unexpected_samples,
        }
    )
    return common


def _inspect_archive(
    path: Path,
    label: str,
    scanner: Callable[[zipfile.ZipFile, SubsetExpectations, ErrorCollector], dict[str, Any]],
    expectations: SubsetExpectations,
    errors: ErrorCollector,
) -> tuple[dict[str, Any] | None, FileIdentity | None]:
    try:
        descriptor, identity = _open_regular_read_only(path, label)
    except InspectionFailure as error:
        errors.add(str(error))
        return None, None
    summary: dict[str, Any] | None = None
    try:
        with os.fdopen(descriptor, "rb", closefd=False) as stream:
            try:
                with zipfile.ZipFile(stream, mode="r", allowZip64=True) as archive:
                    summary = scanner(archive, expectations, errors)
            except Exception as error:
                errors.add(f"{label} ZIP inspection failed: {error}")
        after = os.fstat(descriptor)
        if not _same_identity(after, identity):
            errors.add(f"{label} changed while its central directory was inspected")
        try:
            _recheck_identity(identity, label)
        except InspectionFailure as error:
            errors.add(str(error))
    finally:
        os.close(descriptor)
    return summary, identity


def _reject_json_constant(value: str) -> Any:
    raise InspectionFailure(f"audit report contains forbidden JSON constant {value!r}")


def _unique_json_object(pairs: Iterable[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise InspectionFailure(f"audit report contains duplicate JSON key {key!r}")
        result[key] = value
    return result


def _read_audit_report(path: Path, label: str) -> tuple[Mapping[str, Any], FileIdentity]:
    descriptor, identity = _open_regular_read_only(path, label)
    try:
        if identity.size <= 0 or identity.size > MAX_AUDIT_REPORT_BYTES:
            raise InspectionFailure(
                f"{label} size {identity.size} is outside 1..{MAX_AUDIT_REPORT_BYTES} bytes"
            )
        with os.fdopen(descriptor, "rb", closefd=False) as stream:
            try:
                payload = stream.read(MAX_AUDIT_REPORT_BYTES + 1)
            except OSError as error:
                raise InspectionFailure(f"{label} could not be read: {error}") from error
        after = os.fstat(descriptor)
        if not _same_identity(after, identity):
            raise InspectionFailure(f"{label} changed while it was read")
        _recheck_identity(identity, label)
    finally:
        os.close(descriptor)
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
    if not isinstance(parsed, dict):
        raise InspectionFailure(f"{label} root must be a JSON object")
    return parsed, identity


def _exact_integer(value: Any) -> int | None:
    return value if isinstance(value, int) and not isinstance(value, bool) else None


def _validate_audit_report(
    report_path: Path,
    archive_identity: FileIdentity | None,
    archive_summary: Mapping[str, Any] | None,
    label: str,
    errors: ErrorCollector,
) -> dict[str, Any]:
    result: dict[str, Any] = {
        "status": "FAIL",
        "report": str(report_path),
        "archive_sha256": None,
        "archive_raw_bytes_hashed": False,
    }
    if archive_identity is None or archive_summary is None:
        errors.add(f"{label} cannot be bound because its archive inspection did not complete")
        return result
    try:
        report, _identity = _read_audit_report(report_path, f"{label} report")
        if _exact_integer(report.get("schema_version")) != 1:
            raise InspectionFailure(f"{label} must use generic audit schema_version 1")
        if report.get("status") != "PASS" or report.get("valid") is not True:
            raise InspectionFailure(f"{label} does not declare status PASS and valid true")
        if report.get("read_only") is not True or report.get("extracted") is not False:
            raise InspectionFailure(f"{label} does not prove read-only, non-extracted verification")
        if _exact_integer(report.get("error_count")) != 0 or report.get("errors") != []:
            raise InspectionFailure(f"{label} contains verification errors")
        if report.get("errors_truncated") is not False:
            raise InspectionFailure(f"{label} has an invalid errors_truncated field")
        reported_archive = report.get("archive")
        if not isinstance(reported_archive, str):
            raise InspectionFailure(f"{label} archive field must be text")
        if _absolute_path(reported_archive) != archive_identity.path:
            raise InspectionFailure(f"{label} refers to a different archive path")
        if _exact_integer(report.get("actual_size")) != archive_identity.size:
            raise InspectionFailure(f"{label} actual_size does not match the inspected archive")
        if report.get("expected_size") != report.get("actual_size"):
            raise InspectionFailure(f"{label} expected_size and actual_size differ")
        digest = report.get("actual_sha256")
        if (
            not isinstance(digest, str)
            or re.fullmatch(r"[0-9a-f]{64}", digest, re.ASCII) is None
        ):
            raise InspectionFailure(f"{label} actual_sha256 is not lowercase SHA-256 text")
        if report.get("expected_sha256") != digest:
            raise InspectionFailure(f"{label} expected_sha256 and actual_sha256 differ")
        zip_report = report.get("zip")
        if not isinstance(zip_report, dict):
            raise InspectionFailure(f"{label} zip field must be an object")
        for key in (
            "member_count",
            "file_count",
            "directory_count",
            "declared_uncompressed_bytes",
        ):
            if _exact_integer(zip_report.get(key)) != archive_summary.get(key):
                raise InspectionFailure(f"{label} zip.{key} does not match this inspection")
        if zip_report.get("top_level_entries") != archive_summary.get("top_level_entries"):
            raise InspectionFailure(f"{label} top-level entry counts do not match this inspection")
        payload_crc = report.get("payload_crc")
        if not isinstance(payload_crc, dict):
            raise InspectionFailure(f"{label} payload_crc field must be an object")
        if payload_crc.get("requested") is not True or payload_crc.get("status") != "PASS":
            raise InspectionFailure(f"{label} does not contain a passing full payload CRC audit")
        if _exact_integer(payload_crc.get("verified_members")) != archive_summary.get(
            "member_count"
        ):
            raise InspectionFailure(f"{label} payload member count does not match")
        if _exact_integer(payload_crc.get("verified_uncompressed_bytes")) != archive_summary.get(
            "declared_uncompressed_bytes"
        ):
            raise InspectionFailure(f"{label} payload byte count does not match")
        current_digest = _sha256_same_identity(
            archive_identity,
            f"archive bound to {label}",
        )
        result["archive_raw_bytes_hashed"] = True
        if current_digest != digest:
            raise InspectionFailure(
                f"{label} actual_sha256 does not match the currently inspected archive"
            )
    except InspectionFailure as error:
        errors.add(str(error))
        return result
    result["status"] = "PASS"
    result["archive_sha256"] = digest
    return result


def _validated_expectations(expectations: SubsetExpectations) -> None:
    if not expectations.database_stems:
        raise InspectionFailure("expected database stem set must not be empty")
    if not expectations.camera_logs:
        raise InspectionFailure("expected camera log set must not be empty")
    if not expectations.camera_logs.issubset(expectations.database_stems):
        raise InspectionFailure("every expected camera log must have an expected database stem")
    if len(set(expectations.camera_channels)) != len(expectations.camera_channels):
        raise InspectionFailure("expected camera channels must be unique")
    if not expectations.camera_channels:
        raise InspectionFailure("expected camera channels must not be empty")
    frame_counts = expectations.jpeg_counts()
    if set(frame_counts) != set(expectations.camera_logs):
        raise InspectionFailure("JPEG-count logs must exactly equal expected camera logs")
    if any(isinstance(count, bool) or not isinstance(count, int) or count <= 0 for count in frame_counts.values()):
        raise InspectionFailure("every expected JPEG count must be a positive integer")


def inspect_nuplan_mini_subset(
    database_archive: Path | str,
    camera_archive: Path | str,
    *,
    database_audit_report: Path | str | None = None,
    camera_audit_report: Path | str | None = None,
    expectations: SubsetExpectations = DEFAULT_EXPECTATIONS,
) -> dict[str, Any]:
    """Inspect exact nuPlan mini DB/camera manifests without reading payloads."""
    errors = ErrorCollector()
    try:
        database_path = _absolute_path(database_archive)
        camera_path = _absolute_path(camera_archive)
        _validated_expectations(expectations)
    except InspectionFailure as error:
        errors.add(str(error))
        database_path = Path(str(database_archive))
        camera_path = Path(str(camera_archive))

    report: dict[str, Any] = {
        "schema_version": 1,
        "status": "FAIL",
        "valid": False,
        "database_archive": str(database_path),
        "camera_archive": str(camera_path),
        "database": None,
        "camera": None,
        "correspondence": None,
        "generic_audits": {
            "database": {
                "status": "NOT_PROVIDED",
                "report": None,
                "archive_sha256": None,
                "archive_raw_bytes_hashed": False,
            },
            "camera": {
                "status": "NOT_PROVIDED",
                "report": None,
                "archive_sha256": None,
                "archive_raw_bytes_hashed": False,
            },
        },
        "sqlite_validation": {
            "status": SQLITE_VALIDATION_STATUS,
            "reason": "SQLite checks require a separately approved, safely staged extraction",
        },
        "read_only": True,
        "extracted": False,
        "archive_payloads_read": False,
        "errors": [],
        "error_count": 0,
        "errors_truncated": False,
    }
    if errors.total:
        report["errors"] = errors.items
        report["error_count"] = errors.total
        return report

    database, database_identity = _inspect_archive(
        database_path,
        "database archive",
        _scan_database_members,
        expectations,
        errors,
    )
    camera, camera_identity = _inspect_archive(
        camera_path,
        "camera archive",
        _scan_camera_members,
        expectations,
        errors,
    )
    report["database"] = database
    report["camera"] = camera

    if database is not None and camera is not None:
        occurrences = database["stem_occurrences"]
        observed_camera_logs = set(camera["logs"])
        mapping = {
            log: {
                "camera_present": log in observed_camera_logs,
                "database_occurrences": occurrences.get(log, 0),
            }
            for log in sorted(expectations.camera_logs)
        }
        correspondence_valid = all(
            value["camera_present"] and value["database_occurrences"] == 1
            for value in mapping.values()
        )
        if not correspondence_valid:
            errors.add("camera-to-database log correspondence is not exactly one-to-one")
        report["correspondence"] = {
            "status": "PASS" if correspondence_valid else "FAIL",
            "logs": mapping,
        }

    if database_audit_report is not None:
        try:
            audit_path = _absolute_path(database_audit_report)
        except InspectionFailure as error:
            errors.add(f"database generic audit path is invalid: {error}")
        else:
            report["generic_audits"]["database"] = _validate_audit_report(
                audit_path,
                database_identity,
                database,
                "database generic audit",
                errors,
            )
    if camera_audit_report is not None:
        try:
            audit_path = _absolute_path(camera_audit_report)
        except InspectionFailure as error:
            errors.add(f"camera generic audit path is invalid: {error}")
        else:
            report["generic_audits"]["camera"] = _validate_audit_report(
                audit_path,
                camera_identity,
                camera,
                "camera generic audit",
                errors,
            )

    for identity, label in (
        (database_identity, "database archive"),
        (camera_identity, "camera archive"),
    ):
        if identity is not None:
            try:
                _recheck_identity(identity, label)
            except InspectionFailure as error:
                errors.add(str(error))

    report["errors"] = errors.items
    report["error_count"] = errors.total
    report["errors_truncated"] = errors.total > len(errors.items)
    report["valid"] = errors.total == 0
    report["status"] = "PASS" if report["valid"] else "FAIL"
    return report


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Read-only exact-manifest inspection for nuPlan v1.1 mini DB and camera group 0."
        )
    )
    parser.add_argument("database_archive", type=Path)
    parser.add_argument("camera_archive", type=Path)
    parser.add_argument(
        "--database-audit-report",
        type=Path,
        help="optional PASS JSON from verify_zip_archive.py --verify-payload-crc",
    )
    parser.add_argument(
        "--camera-audit-report",
        type=Path,
        help="optional PASS JSON from verify_zip_archive.py --verify-payload-crc",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    report = inspect_nuplan_mini_subset(
        args.database_archive,
        args.camera_archive,
        database_audit_report=args.database_audit_report,
        camera_audit_report=args.camera_audit_report,
        expectations=DEFAULT_EXPECTATIONS,
    )
    json.dump(report, sys.stdout, indent=2, sort_keys=True, allow_nan=False)
    sys.stdout.write("\n")
    return 0 if report["valid"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
