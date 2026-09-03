#!/usr/bin/env python3
"""Fail-closed, read-only verification for a downloaded ZIP archive.

The verifier never extracts or modifies archive content.  It first bounds and
validates the end-of-central-directory metadata, then audits every central
directory entry.  Optional payload verification streams each entry only to
check local headers, declared lengths, and CRCs.
"""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path, PureWindowsPath
import stat
import struct
import sys
from typing import Any, BinaryIO, Mapping, Sequence
import unicodedata
import zipfile


GIB = 1024**3
TIB = 1024**4
READ_CHUNK_BYTES = 8 * 1024 * 1024
MAX_REPORTED_ERRORS = 100
MAX_PATH_BYTES = 4096
MAX_COMPONENT_BYTES = 255

EOCD_SIGNATURE = b"PK\x05\x06"
ZIP64_EOCD_SIGNATURE = b"PK\x06\x06"
ZIP64_LOCATOR_SIGNATURE = b"PK\x06\x07"
EOCD = struct.Struct("<4s4H2LH")
ZIP64_EOCD = struct.Struct("<4sQ2H2L4Q")
ZIP64_LOCATOR = struct.Struct("<4sLQL")
EOCD_MAX_BYTES = EOCD.size + 65_535

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


@dataclass(frozen=True)
class AuditLimits:
    """Resource limits applied before extraction is ever considered."""

    max_members: int = 500_000
    max_central_directory_bytes: int = 128 * 1024**2
    max_total_uncompressed_bytes: int = TIB
    max_member_uncompressed_bytes: int = 256 * GIB
    max_total_compression_ratio: float = 200.0
    max_member_compression_ratio: float = 10_000.0


@dataclass(frozen=True)
class EndRecord:
    """Validated single-disk ZIP end-record values."""

    member_count: int
    central_directory_size: int
    central_directory_offset: int
    eocd_offset: int
    zip64: bool
    comment_bytes: int


class AuditFailure(Exception):
    """A deterministic validation failure, safe to expose in a report."""


class ErrorCollector:
    """Bound memory use while retaining a useful sample of validation errors."""

    def __init__(self) -> None:
        self.items: list[str] = []
        self.total = 0

    def add(self, message: str) -> None:
        self.total += 1
        if len(self.items) < MAX_REPORTED_ERRORS:
            self.items.append(message)


def _positive_integer(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise AuditFailure(f"{label} must be a positive integer")
    return value


def _positive_ratio(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise AuditFailure(f"{label} must be a finite number greater than or equal to 1")
    parsed = float(value)
    if not math.isfinite(parsed) or parsed < 1.0:
        raise AuditFailure(f"{label} must be a finite number greater than or equal to 1")
    return parsed


def _validated_limits(limits: AuditLimits) -> AuditLimits:
    return AuditLimits(
        max_members=_positive_integer(limits.max_members, "max_members"),
        max_central_directory_bytes=_positive_integer(
            limits.max_central_directory_bytes, "max_central_directory_bytes"
        ),
        max_total_uncompressed_bytes=_positive_integer(
            limits.max_total_uncompressed_bytes, "max_total_uncompressed_bytes"
        ),
        max_member_uncompressed_bytes=_positive_integer(
            limits.max_member_uncompressed_bytes, "max_member_uncompressed_bytes"
        ),
        max_total_compression_ratio=_positive_ratio(
            limits.max_total_compression_ratio, "max_total_compression_ratio"
        ),
        max_member_compression_ratio=_positive_ratio(
            limits.max_member_compression_ratio, "max_member_compression_ratio"
        ),
    )


def _validated_sha256(value: str | None) -> str | None:
    if value is None:
        return None
    if not isinstance(value, str):
        raise AuditFailure("expected_sha256 must be text")
    normalized = value.strip().lower()
    if len(normalized) != 64 or any(character not in "0123456789abcdef" for character in normalized):
        raise AuditFailure("expected_sha256 must contain exactly 64 hexadecimal characters")
    return normalized


def _file_identity(metadata: os.stat_result) -> tuple[int, int, int, int, int]:
    return (
        metadata.st_dev,
        metadata.st_ino,
        metadata.st_size,
        metadata.st_mtime_ns,
        metadata.st_ctime_ns,
    )


def _read_exact_at(stream: BinaryIO, offset: int, size: int) -> bytes:
    if offset < 0 or size < 0:
        raise AuditFailure("ZIP metadata contains a negative offset or length")
    stream.seek(offset)
    value = stream.read(size)
    if len(value) != size:
        raise AuditFailure("ZIP metadata points outside the archive")
    return value


def _find_eocd(stream: BinaryIO, archive_size: int) -> tuple[int, tuple[Any, ...]]:
    if archive_size < EOCD.size:
        raise AuditFailure("archive is too small to contain a ZIP end record")
    tail_size = min(archive_size, EOCD_MAX_BYTES)
    tail_offset = archive_size - tail_size
    tail = _read_exact_at(stream, tail_offset, tail_size)
    # CPython's zipfile parser selects the final EOCD signature in the tail,
    # even when an earlier signature has a self-consistent comment length.  We
    # must validate that exact record before handing the stream to zipfile;
    # otherwise a later fake signature inside the archive comment could bypass
    # the resource limits checked here and make zipfile read a different,
    # attacker-sized central directory.
    relative_offset = tail.rfind(EOCD_SIGNATURE)
    if relative_offset < 0:
        raise AuditFailure("ZIP end-of-central-directory record is missing")
    if relative_offset + EOCD.size > len(tail):
        raise AuditFailure("final ZIP end-of-central-directory signature is truncated")
    values = EOCD.unpack_from(tail, relative_offset)
    comment_size = int(values[-1])
    absolute_offset = tail_offset + relative_offset
    if absolute_offset + EOCD.size + comment_size != archive_size:
        raise AuditFailure(
            "final ZIP end-of-central-directory record has an inconsistent comment length"
        )
    return absolute_offset, values


def _read_end_record(
    stream: BinaryIO,
    archive_size: int,
    limits: AuditLimits,
) -> EndRecord:
    eocd_offset, values = _find_eocd(stream, archive_size)
    (
        signature,
        disk_number,
        central_directory_disk,
        disk_member_count,
        total_member_count,
        central_directory_size,
        central_directory_offset,
        comment_size,
    ) = values
    if signature != EOCD_SIGNATURE:
        raise AuditFailure("invalid ZIP end-record signature")
    if disk_number not in (0, 0xFFFF) or central_directory_disk not in (0, 0xFFFF):
        raise AuditFailure("multi-disk ZIP archives are forbidden")

    uses_zip64 = any(
        value == sentinel
        for value, sentinel in (
            (disk_number, 0xFFFF),
            (central_directory_disk, 0xFFFF),
            (disk_member_count, 0xFFFF),
            (total_member_count, 0xFFFF),
            (central_directory_size, 0xFFFFFFFF),
            (central_directory_offset, 0xFFFFFFFF),
        )
    )
    central_directory_end = eocd_offset

    if uses_zip64:
        locator_offset = eocd_offset - ZIP64_LOCATOR.size
        locator = ZIP64_LOCATOR.unpack(
            _read_exact_at(stream, locator_offset, ZIP64_LOCATOR.size)
        )
        locator_signature, zip64_disk, zip64_offset, total_disks = locator
        if locator_signature != ZIP64_LOCATOR_SIGNATURE:
            raise AuditFailure("ZIP64 locator is missing or malformed")
        if zip64_disk != 0 or total_disks != 1:
            raise AuditFailure("multi-disk ZIP64 archives are forbidden")
        if zip64_offset >= locator_offset:
            raise AuditFailure("ZIP64 end record has an invalid offset")
        fixed_record = ZIP64_EOCD.unpack(
            _read_exact_at(stream, int(zip64_offset), ZIP64_EOCD.size)
        )
        (
            zip64_signature,
            zip64_record_size,
            _version_made_by,
            _version_needed,
            zip64_disk_number,
            zip64_central_disk,
            zip64_disk_members,
            zip64_total_members,
            zip64_central_size,
            zip64_central_offset,
        ) = fixed_record
        if zip64_signature != ZIP64_EOCD_SIGNATURE or zip64_record_size < 44:
            raise AuditFailure("ZIP64 end record is malformed")
        if int(zip64_offset) + 12 + int(zip64_record_size) != locator_offset:
            raise AuditFailure("ZIP64 end record length is inconsistent")
        if zip64_disk_number != 0 or zip64_central_disk != 0:
            raise AuditFailure("multi-disk ZIP64 archives are forbidden")
        if zip64_disk_members != zip64_total_members:
            raise AuditFailure("split ZIP64 archives are forbidden")
        total_member_count = int(zip64_total_members)
        central_directory_size = int(zip64_central_size)
        central_directory_offset = int(zip64_central_offset)
        central_directory_end = int(zip64_offset)
    elif disk_member_count != total_member_count:
        raise AuditFailure("split ZIP archives are forbidden")

    total_member_count = int(total_member_count)
    central_directory_size = int(central_directory_size)
    central_directory_offset = int(central_directory_offset)
    if total_member_count <= 0:
        raise AuditFailure("ZIP archive must contain at least one member")
    if total_member_count > limits.max_members:
        raise AuditFailure(
            f"declared member count {total_member_count} exceeds limit {limits.max_members}"
        )
    if central_directory_size <= 0:
        raise AuditFailure("central directory must not be empty")
    if central_directory_size > limits.max_central_directory_bytes:
        raise AuditFailure(
            "declared central-directory size "
            f"{central_directory_size} exceeds limit {limits.max_central_directory_bytes}"
        )
    if central_directory_offset + central_directory_size != central_directory_end:
        raise AuditFailure(
            "central-directory offset/size is inconsistent or archive has unexpected prefix data"
        )
    if central_directory_end > archive_size:
        raise AuditFailure("central directory extends beyond the archive")
    return EndRecord(
        member_count=total_member_count,
        central_directory_size=central_directory_size,
        central_directory_offset=central_directory_offset,
        eocd_offset=eocd_offset,
        zip64=uses_zip64,
        comment_bytes=int(comment_size),
    )


def _hash_stream(stream: BinaryIO) -> str:
    stream.seek(0)
    digest = hashlib.sha256()
    for block in iter(lambda: stream.read(READ_CHUNK_BYTES), b""):
        digest.update(block)
    return digest.hexdigest()


def _validate_extra_fields(info: zipfile.ZipInfo, errors: ErrorCollector) -> None:
    position = 0
    while position < len(info.extra):
        if len(info.extra) - position < 4:
            errors.add(f"member {info.filename!r} has a malformed ZIP extra field header")
            return
        _field_id, data_size = struct.unpack_from("<HH", info.extra, position)
        position += 4
        if data_size > len(info.extra) - position:
            errors.add(f"member {info.filename!r} has a truncated ZIP extra field")
            return
        position += data_size


def _portable_path(info: zipfile.ZipInfo, errors: ErrorCollector) -> tuple[str, str] | None:
    original = getattr(info, "orig_filename", info.filename)
    if not isinstance(original, str) or not original:
        errors.add("archive contains an empty or non-text member name")
        return None
    if "\x00" in original:
        errors.add("archive contains a member name with a NUL byte")
        return None
    if any(ord(character) < 32 or ord(character) == 127 for character in original):
        errors.add(f"member name {original!r} contains a control character")
        return None
    try:
        encoded = original.encode("utf-8")
    except UnicodeEncodeError:
        errors.add(f"member name {original!r} is not valid Unicode text")
        return None
    if len(encoded) > MAX_PATH_BYTES:
        errors.add(f"member name {original!r} exceeds {MAX_PATH_BYTES} UTF-8 bytes")
        return None
    if "\\" in original:
        errors.add(f"member name {original!r} contains a backslash")
        return None
    windows_path = PureWindowsPath(original)
    if original.startswith("/") or windows_path.is_absolute() or windows_path.drive:
        errors.add(f"member name {original!r} is absolute or drive-qualified")
        return None

    trimmed = original[:-1] if original.endswith("/") else original
    if not trimmed:
        errors.add(f"member name {original!r} does not identify a safe path")
        return None
    components = trimmed.split("/")
    if any(component in ("", ".", "..") for component in components):
        errors.add(f"member name {original!r} has an unsafe path component")
        return None
    for component in components:
        try:
            component_size = len(component.encode("utf-8"))
        except UnicodeEncodeError:
            errors.add(f"member name {original!r} is not valid Unicode text")
            return None
        if component_size > MAX_COMPONENT_BYTES:
            errors.add(
                f"member name {original!r} has a component over {MAX_COMPONENT_BYTES} bytes"
            )
            return None
        if component != component.strip() or component.endswith((".", " ")):
            errors.add(f"member name {original!r} has ambiguous whitespace or trailing dots")
            return None
        if ":" in component:
            errors.add(f"member name {original!r} contains a Windows alternate-stream separator")
            return None
        reserved_base = component.rstrip(" .").split(".", 1)[0].upper()
        if reserved_base in WINDOWS_RESERVED_NAMES:
            errors.add(f"member name {original!r} contains a reserved Windows device name")
            return None

    normalized = unicodedata.normalize("NFC", "/".join(components))
    return normalized, normalized.casefold()


def _entry_type(info: zipfile.ZipInfo, errors: ErrorCollector) -> str:
    is_directory = info.is_dir()
    unix_mode = (info.external_attr >> 16) & 0xFFFF
    file_type = stat.S_IFMT(unix_mode)
    if info.create_system == 3:
        if stat.S_ISLNK(unix_mode):
            errors.add(f"member {info.filename!r} is a symbolic link")
        elif file_type not in (0, stat.S_IFREG, stat.S_IFDIR):
            errors.add(f"member {info.filename!r} has a forbidden special-file mode")
        if is_directory and file_type not in (0, stat.S_IFDIR):
            errors.add(f"member {info.filename!r} has conflicting file/directory metadata")
        if not is_directory and file_type == stat.S_IFDIR:
            errors.add(f"member {info.filename!r} has conflicting file/directory metadata")
    return "directory" if is_directory else "file"


def _compression_ratio(uncompressed: int, compressed: int) -> float | None:
    if uncompressed == 0:
        return 0.0
    if compressed == 0:
        return None
    return uncompressed / compressed


def _audit_members(
    archive: zipfile.ZipFile,
    end_record: EndRecord,
    limits: AuditLimits,
    errors: ErrorCollector,
) -> tuple[dict[str, Any], list[zipfile.ZipInfo]]:
    members = archive.infolist()
    if len(members) != end_record.member_count:
        errors.add(
            "central-directory member count mismatch: "
            f"end record declares {end_record.member_count}, parser found {len(members)}"
        )
    if len(members) > limits.max_members:
        errors.add(f"parsed member count {len(members)} exceeds limit {limits.max_members}")

    exact_names: set[str] = set()
    normalized_paths: dict[str, str] = {}
    portable_paths: dict[str, str] = {}
    path_kinds: dict[str, str] = {}
    portable_kinds: dict[str, str] = {}
    header_offsets: set[int] = set()
    top_levels: Counter[str] = Counter()
    compression_methods: Counter[str] = Counter()
    file_count = 0
    directory_count = 0
    total_uncompressed = 0
    total_compressed = 0

    for info in members:
        original = getattr(info, "orig_filename", info.filename)
        if original in exact_names:
            errors.add(f"duplicate member name {original!r}")
        exact_names.add(original)
        if info.header_offset in header_offsets:
            errors.add(f"multiple members share local-header offset {info.header_offset}")
        header_offsets.add(info.header_offset)
        if info.header_offset < 0 or info.header_offset >= end_record.central_directory_offset:
            errors.add(f"member {original!r} has a local header outside the payload region")
        if getattr(info, "volume", 0) != 0:
            errors.add(f"member {original!r} belongs to a forbidden split archive")
        if info.flag_bits & 0x1:
            errors.add(f"member {original!r} is encrypted")
        if info.compress_type not in SUPPORTED_COMPRESSION_METHODS:
            errors.add(
                f"member {original!r} uses unsupported compression method {info.compress_type}"
            )
        _validate_extra_fields(info, errors)

        path = _portable_path(info, errors)
        kind = _entry_type(info, errors)
        if kind == "directory":
            directory_count += 1
            # A valid compressed empty stream has zero uncompressed bytes but
            # a small non-zero compressed representation.  Its local header,
            # stream termination, and CRC are checked in payload-verification
            # mode just like a regular file.
            if info.file_size != 0:
                errors.add(f"directory member {original!r} declares non-zero payload bytes")
            total_uncompressed += info.file_size
            total_compressed += info.compress_size
        else:
            file_count += 1
            if info.file_size < 0 or info.compress_size < 0:
                errors.add(f"member {original!r} has a negative declared size")
            total_uncompressed += info.file_size
            total_compressed += info.compress_size
            if info.file_size > limits.max_member_uncompressed_bytes:
                errors.add(
                    f"member {original!r} declared size {info.file_size} exceeds limit "
                    f"{limits.max_member_uncompressed_bytes}"
                )
            ratio = _compression_ratio(info.file_size, info.compress_size)
            if ratio is None:
                errors.add(f"non-empty member {original!r} has zero compressed bytes")
            elif ratio > limits.max_member_compression_ratio:
                errors.add(
                    f"member {original!r} compression ratio {ratio:.3f} exceeds limit "
                    f"{limits.max_member_compression_ratio:.3f}"
                )
        compression_methods[str(info.compress_type)] += 1

        if path is None:
            continue
        normalized, portable = path
        top_levels[normalized.split("/", 1)[0]] += 1
        if normalized in normalized_paths:
            errors.add(
                f"normalized path collision between {normalized_paths[normalized]!r} and {original!r}"
            )
        else:
            normalized_paths[normalized] = original
        if portable in portable_paths and portable_paths[portable] != original:
            errors.add(
                f"portable case/Unicode collision between {portable_paths[portable]!r} "
                f"and {original!r}"
            )
        else:
            portable_paths[portable] = original

        previous_kind = path_kinds.get(normalized)
        if previous_kind is not None and previous_kind != kind:
            errors.add(f"path {normalized!r} has a file/directory conflict")
        path_kinds[normalized] = kind

        previous_portable_kind = portable_kinds.get(portable)
        if previous_portable_kind is not None and previous_portable_kind != kind:
            errors.add(f"portable path {portable!r} has a file/directory conflict")
        portable_kinds[portable] = kind

    _report_parent_file_conflicts(path_kinds, errors, portable=False)
    _report_parent_file_conflicts(portable_kinds, errors, portable=True)

    if total_uncompressed > limits.max_total_uncompressed_bytes:
        errors.add(
            f"total declared size {total_uncompressed} exceeds limit "
            f"{limits.max_total_uncompressed_bytes}"
        )
    total_ratio = _compression_ratio(total_uncompressed, total_compressed)
    if total_ratio is None:
        errors.add("non-empty archive payload has zero total compressed bytes")
    elif total_ratio > limits.max_total_compression_ratio:
        errors.add(
            f"total compression ratio {total_ratio:.3f} exceeds limit "
            f"{limits.max_total_compression_ratio:.3f}"
        )

    summary = {
        "member_count": len(members),
        "file_count": file_count,
        "directory_count": directory_count,
        "declared_compressed_bytes": total_compressed,
        "declared_uncompressed_bytes": total_uncompressed,
        "compression_ratio": total_ratio,
        "compression_methods": dict(sorted(compression_methods.items())),
        "top_level_entries": dict(sorted(top_levels.items())),
    }
    return summary, members


def _report_parent_file_conflicts(
    path_kinds: Mapping[str, str],
    errors: ErrorCollector,
    *,
    portable: bool,
) -> None:
    """Detect file-as-directory conflicts without retaining every parent prefix."""
    ordered = sorted(path_kinds)
    label = "portable file path" if portable else "file member"
    for path, following in zip(ordered, ordered[1:]):
        if path_kinds[path] == "file" and following.startswith(path + "/"):
            errors.add(f"{label} {path!r} is also a parent directory")


def _verify_payload_crc(
    archive: zipfile.ZipFile,
    members: Sequence[zipfile.ZipInfo],
    errors: ErrorCollector,
) -> dict[str, Any]:
    verified_members = 0
    verified_bytes = 0
    try:
        for info in members:
            member_bytes = 0
            with archive.open(info, "r") as source:
                for block in iter(lambda: source.read(READ_CHUNK_BYTES), b""):
                    member_bytes += len(block)
            if member_bytes != info.file_size:
                raise AuditFailure(
                    f"member {info.filename!r} yielded {member_bytes} bytes, "
                    f"expected {info.file_size}"
                )
            verified_members += 1
            verified_bytes += member_bytes
    except Exception as error:
        # Decompressors expose unrelated exception families (for example
        # zlib.error and lzma.LZMAError).  Treat every ordinary data/parser
        # exception as an invalid archive while still allowing process-control
        # BaseException subclasses such as KeyboardInterrupt to propagate.
        errors.add(f"payload CRC/local-header verification failed: {error}")
        return {
            "requested": True,
            "status": "FAIL",
            "verified_members": verified_members,
            "verified_uncompressed_bytes": verified_bytes,
        }
    return {
        "requested": True,
        "status": "PASS",
        "verified_members": verified_members,
        "verified_uncompressed_bytes": verified_bytes,
    }


def _empty_report(
    archive_path: Path,
    expected_size: Any,
    expected_sha256: str | None,
    limits: AuditLimits,
    verify_payload_crc: bool,
) -> dict[str, Any]:
    return {
        "schema_version": 1,
        "status": "FAIL",
        "valid": False,
        "archive": str(archive_path),
        "expected_size": expected_size,
        "actual_size": None,
        "expected_sha256": expected_sha256,
        "actual_sha256": None,
        "zip": None,
        "limits": {
            "max_members": limits.max_members,
            "max_central_directory_bytes": limits.max_central_directory_bytes,
            "max_total_uncompressed_bytes": limits.max_total_uncompressed_bytes,
            "max_member_uncompressed_bytes": limits.max_member_uncompressed_bytes,
            "max_total_compression_ratio": limits.max_total_compression_ratio,
            "max_member_compression_ratio": limits.max_member_compression_ratio,
        },
        "payload_crc": {
            "requested": verify_payload_crc,
            "status": "SKIPPED" if verify_payload_crc else "NOT_REQUESTED",
            "verified_members": 0,
            "verified_uncompressed_bytes": 0,
        },
        "errors": [],
        "error_count": 0,
        "errors_truncated": False,
        "read_only": True,
        "extracted": False,
    }


def verify_zip_archive(
    archive_path: Path | str,
    *,
    expected_size: int,
    expected_sha256: str | None = None,
    limits: AuditLimits = AuditLimits(),
    verify_payload_crc: bool = False,
) -> dict[str, Any]:
    """Return a machine-readable, fail-closed ZIP verification report."""
    path = Path(os.path.abspath(os.fspath(Path(archive_path).expanduser())))
    report = _empty_report(path, expected_size, expected_sha256, limits, verify_payload_crc)
    errors = ErrorCollector()
    try:
        expected_size = _positive_integer(expected_size, "expected_size")
        expected_sha256 = _validated_sha256(expected_sha256)
        limits = _validated_limits(limits)
    except AuditFailure as error:
        errors.add(str(error))
        report["errors"] = errors.items
        report["error_count"] = errors.total
        return report
    report["expected_size"] = expected_size
    report["expected_sha256"] = expected_sha256
    report["limits"] = _empty_report(
        path, expected_size, expected_sha256, limits, verify_payload_crc
    )["limits"]

    try:
        path_metadata = os.lstat(path)
    except OSError as error:
        errors.add(f"cannot inspect archive path: {error}")
        report["errors"] = errors.items
        report["error_count"] = errors.total
        return report
    if stat.S_ISLNK(path_metadata.st_mode):
        errors.add("archive path must not be a symbolic link")
    elif not stat.S_ISREG(path_metadata.st_mode):
        errors.add("archive path must be a regular file")
    if errors.total:
        report["errors"] = errors.items
        report["error_count"] = errors.total
        return report

    flags = os.O_RDONLY
    if hasattr(os, "O_CLOEXEC"):
        flags |= os.O_CLOEXEC
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    try:
        archive_fd = os.open(path, flags)
    except OSError as error:
        errors.add(f"cannot open archive safely: {error}")
        report["errors"] = errors.items
        report["error_count"] = errors.total
        return report

    try:
        before = os.fstat(archive_fd)
        if not stat.S_ISREG(before.st_mode):
            errors.add("opened archive is not a regular file")
        if (before.st_dev, before.st_ino) != (path_metadata.st_dev, path_metadata.st_ino):
            errors.add("archive path changed before it could be opened")
        report["actual_size"] = before.st_size
        if before.st_size != expected_size:
            errors.add(f"size mismatch: expected {expected_size}, found {before.st_size}")

        if not errors.total:
            with os.fdopen(archive_fd, "rb", closefd=False) as stream:
                try:
                    end_record = _read_end_record(stream, before.st_size, limits)
                    actual_sha256 = _hash_stream(stream)
                    report["actual_sha256"] = actual_sha256
                    after_hash = os.fstat(archive_fd)
                    if _file_identity(after_hash) != _file_identity(before):
                        errors.add("archive changed while it was being hashed")
                    if expected_sha256 is not None and actual_sha256 != expected_sha256:
                        errors.add(
                            f"SHA-256 mismatch: expected {expected_sha256}, found {actual_sha256}"
                        )
                    report["zip"] = {
                        "zip64": end_record.zip64,
                        "comment_bytes": end_record.comment_bytes,
                        "central_directory_bytes": end_record.central_directory_size,
                        "central_directory_offset": end_record.central_directory_offset,
                    }
                    if not errors.total:
                        stream.seek(0)
                        with zipfile.ZipFile(stream, mode="r", allowZip64=True) as archive:
                            if archive.start_dir != end_record.central_directory_offset:
                                raise AuditFailure(
                                    "ZIP parser selected a central-directory offset that differs "
                                    "from the preflight end record"
                                )
                            if len(archive.comment) != end_record.comment_bytes:
                                raise AuditFailure(
                                    "ZIP parser selected an end record with a different comment length"
                                )
                            summary, members = _audit_members(
                                archive, end_record, limits, errors
                            )
                            report["zip"].update(summary)
                            if verify_payload_crc and not errors.total:
                                report["payload_crc"] = _verify_payload_crc(
                                    archive, members, errors
                                )
                except Exception as error:
                    errors.add(f"ZIP verification failed: {error}")

        after = os.fstat(archive_fd)
        if _file_identity(after) != _file_identity(before):
            errors.add("archive changed while it was being verified")
        try:
            final_path_metadata = os.lstat(path)
        except OSError as error:
            errors.add(f"archive path changed while it was being verified: {error}")
        else:
            if (after.st_dev, after.st_ino) != (
                final_path_metadata.st_dev,
                final_path_metadata.st_ino,
            ):
                errors.add("archive path now refers to a different inode")
    finally:
        os.close(archive_fd)

    report["errors"] = errors.items
    report["error_count"] = errors.total
    report["errors_truncated"] = errors.total > len(errors.items)
    report["valid"] = errors.total == 0
    report["status"] = "PASS" if report["valid"] else "FAIL"
    return report


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Read-only ZIP central-directory audit with optional streaming payload CRC verification."
        )
    )
    parser.add_argument("archive", type=Path)
    parser.add_argument("--expected-size", required=True, type=int)
    parser.add_argument("--expected-sha256")
    parser.add_argument("--max-members", type=int, default=AuditLimits.max_members)
    parser.add_argument(
        "--max-central-directory-bytes",
        type=int,
        default=AuditLimits.max_central_directory_bytes,
    )
    parser.add_argument(
        "--max-total-uncompressed-bytes",
        type=int,
        default=AuditLimits.max_total_uncompressed_bytes,
    )
    parser.add_argument(
        "--max-member-uncompressed-bytes",
        type=int,
        default=AuditLimits.max_member_uncompressed_bytes,
    )
    parser.add_argument(
        "--max-total-compression-ratio",
        type=float,
        default=AuditLimits.max_total_compression_ratio,
    )
    parser.add_argument(
        "--max-member-compression-ratio",
        type=float,
        default=AuditLimits.max_member_compression_ratio,
    )
    parser.add_argument(
        "--verify-payload-crc",
        action="store_true",
        help="stream every member to verify local headers, length, and CRC without extraction",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    limits = AuditLimits(
        max_members=args.max_members,
        max_central_directory_bytes=args.max_central_directory_bytes,
        max_total_uncompressed_bytes=args.max_total_uncompressed_bytes,
        max_member_uncompressed_bytes=args.max_member_uncompressed_bytes,
        max_total_compression_ratio=args.max_total_compression_ratio,
        max_member_compression_ratio=args.max_member_compression_ratio,
    )
    report = verify_zip_archive(
        args.archive,
        expected_size=args.expected_size,
        expected_sha256=args.expected_sha256,
        limits=limits,
        verify_payload_crc=args.verify_payload_crc,
    )
    json.dump(report, sys.stdout, indent=2, sort_keys=True, allow_nan=False)
    sys.stdout.write("\n")
    return 0 if report["valid"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
