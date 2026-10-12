#!/usr/bin/env python3
"""Fail-closed, read-only verification for a gzip-compressed TAR archive.

The verifier parses TAR records itself so that size limits are applied before
any extended header or member payload is read into memory.  It never extracts,
creates, repairs, or rewrites archive members.
"""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import dataclass
import gzip
import hashlib
import json
import math
import os
from pathlib import Path, PureWindowsPath
import stat
import sys
from typing import Any, BinaryIO, Mapping, Sequence
import unicodedata
import zlib


MIB = 1024**2
GIB = 1024**3
TIB = 1024**4
TAR_BLOCK_BYTES = 512
READ_CHUNK_BYTES = 8 * MIB
MAX_REPORTED_ERRORS = 100

REGULAR_TYPES = {b"\0", b"0"}
DIRECTORY_TYPE = b"5"
PAX_LOCAL_TYPE = b"x"
PAX_GLOBAL_TYPE = b"g"
GNU_LONGNAME_TYPE = b"L"
GNU_LONGLINK_TYPE = b"K"
METADATA_TYPES = {
    PAX_LOCAL_TYPE,
    PAX_GLOBAL_TYPE,
    GNU_LONGNAME_TYPE,
    GNU_LONGLINK_TYPE,
}
FORBIDDEN_TYPES = {
    b"1": "hard link",
    b"2": "symbolic link",
    b"3": "character device",
    b"4": "block device",
    b"6": "FIFO",
    b"7": "contiguous/special file",
    b"S": "GNU sparse file",
}
FORBIDDEN_PAX_SPARSE_PREFIXES = (
    "gnu.sparse",
    "libarchive.sparse",
    "schily.sparse",
    "sun.sparse",
)
FORBIDDEN_PAX_SIZE_KEYS = {
    "schily.holes",
    "schily.realsize",
    "sun.holesdata",
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
    """Resource and expansion limits enforced while streaming."""

    max_members: int = 500_000
    max_total_uncompressed_bytes: int = TIB
    max_member_uncompressed_bytes: int = 256 * GIB
    max_tar_stream_bytes: int = 2 * TIB
    max_metadata_bytes: int = 64 * MIB
    max_total_compression_ratio: float = 200.0
    max_path_bytes: int = 4096
    max_component_bytes: int = 255
    max_aggregate_normalized_path_bytes: int = 128 * MIB


class AuditFailure(Exception):
    """A deterministic validation failure that is safe to report."""


class ErrorCollector:
    """Bound report memory while retaining the total error count."""

    def __init__(self) -> None:
        self.items: list[str] = []
        self.total = 0

    def add(self, message: str) -> None:
        self.total += 1
        if len(self.items) < MAX_REPORTED_ERRORS:
            self.items.append(message)


class BoundedGzipReader:
    """Count decompressed bytes and stop expansion beyond a hard limit."""

    def __init__(
        self,
        source: gzip.GzipFile,
        maximum_bytes: int,
        limit_suffix: str = "",
    ) -> None:
        self.source = source
        self.maximum_bytes = maximum_bytes
        self.limit_suffix = limit_suffix
        self.bytes_read = 0

    def read(self, size: int) -> bytes:
        if size < 0:
            raise AuditFailure("internal error: unbounded gzip read was requested")
        remaining = self.maximum_bytes - self.bytes_read
        if remaining < 0:
            raise AuditFailure(
                f"decompressed TAR stream exceeds limit {self.maximum_bytes}"
                f"{self.limit_suffix}"
            )
        request = min(size, remaining + 1)
        value = self.source.read(request)
        self.bytes_read += len(value)
        if self.bytes_read > self.maximum_bytes:
            raise AuditFailure(
                f"decompressed TAR stream exceeds limit {self.maximum_bytes}"
                f"{self.limit_suffix}"
            )
        return value

    def read_exact(self, size: int, context: str) -> bytes:
        if size < 0:
            raise AuditFailure(f"{context} has a negative length")
        chunks: list[bytes] = []
        remaining = size
        while remaining:
            chunk = self.read(min(remaining, READ_CHUNK_BYTES))
            if not chunk:
                raise AuditFailure(f"gzip/TAR stream ended inside {context}")
            chunks.append(chunk)
            remaining -= len(chunk)
        return b"".join(chunks)

    def discard_exact(self, size: int, context: str) -> None:
        if size < 0:
            raise AuditFailure(f"{context} has a negative length")
        remaining = size
        while remaining:
            chunk = self.read(min(remaining, READ_CHUNK_BYTES))
            if not chunk:
                raise AuditFailure(f"gzip/TAR stream ended inside {context}")
            remaining -= len(chunk)


@dataclass
class PathRegistry:
    """Track extraction-equivalent portable path collisions."""

    exact_names: set[str]
    normalized_paths: dict[str, str]
    portable_paths: dict[str, str]
    path_kinds: dict[str, str]
    portable_kinds: dict[str, str]
    aggregate_normalized_path_bytes: int

    @classmethod
    def create(cls) -> PathRegistry:
        return cls(set(), {}, {}, {}, {}, 0)


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
    if not isinstance(limits, AuditLimits):
        raise AuditFailure("limits must be an AuditLimits instance")
    return AuditLimits(
        max_members=_positive_integer(limits.max_members, "max_members"),
        max_total_uncompressed_bytes=_positive_integer(
            limits.max_total_uncompressed_bytes, "max_total_uncompressed_bytes"
        ),
        max_member_uncompressed_bytes=_positive_integer(
            limits.max_member_uncompressed_bytes, "max_member_uncompressed_bytes"
        ),
        max_tar_stream_bytes=_positive_integer(
            limits.max_tar_stream_bytes, "max_tar_stream_bytes"
        ),
        max_metadata_bytes=_positive_integer(
            limits.max_metadata_bytes, "max_metadata_bytes"
        ),
        max_total_compression_ratio=_positive_ratio(
            limits.max_total_compression_ratio, "max_total_compression_ratio"
        ),
        max_path_bytes=_positive_integer(limits.max_path_bytes, "max_path_bytes"),
        max_component_bytes=_positive_integer(
            limits.max_component_bytes, "max_component_bytes"
        ),
        max_aggregate_normalized_path_bytes=_positive_integer(
            limits.max_aggregate_normalized_path_bytes,
            "max_aggregate_normalized_path_bytes",
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


def _hash_stream(stream: BinaryIO) -> str:
    stream.seek(0)
    digest = hashlib.sha256()
    for block in iter(lambda: stream.read(READ_CHUNK_BYTES), b""):
        digest.update(block)
    return digest.hexdigest()


def _tar_number(field: bytes, context: str) -> int:
    if not field:
        raise AuditFailure(f"{context} is empty")
    if field[0] in (0x80, 0xFF):
        value = int.from_bytes(field[1:], "big", signed=False)
        if field[0] == 0xFF:
            value -= 256 ** (len(field) - 1)
    else:
        raw = field.split(b"\0", 1)[0].strip(b" ")
        if not raw:
            return 0
        if any(character not in b"01234567" for character in raw):
            raise AuditFailure(f"{context} is not a valid TAR number")
        value = int(raw, 8)
    if value < 0:
        raise AuditFailure(f"{context} must not be negative")
    return value


def _header_text(field: bytes, context: str) -> str:
    terminator = field.find(b"\0")
    if terminator >= 0:
        if any(field[terminator + 1 :]):
            raise AuditFailure(f"{context} contains bytes after a NUL terminator")
        field = field[:terminator]
    try:
        return field.decode("utf-8", "strict")
    except UnicodeDecodeError as error:
        raise AuditFailure(f"{context} is not valid UTF-8") from error


def _parse_header(block: bytes, index: int) -> dict[str, Any]:
    if len(block) != TAR_BLOCK_BYTES:
        raise AuditFailure(f"TAR header {index} is incomplete")
    checksum = _tar_number(block[148:156], f"TAR header {index} checksum")
    checksum_block = block[:148] + (b" " * 8) + block[156:]
    unsigned_checksum = sum(checksum_block)
    signed_checksum = sum(value if value < 128 else value - 256 for value in checksum_block)
    if checksum not in (unsigned_checksum, signed_checksum):
        raise AuditFailure(f"TAR header {index} checksum mismatch")

    magic = block[257:263]
    version = block[263:265]
    if magic == b"ustar\0" and version == b"00":
        header_format = "posix_ustar"
    elif magic == b"ustar " and version == b" \0":
        header_format = "old_gnu"
    elif magic == b"\0" * 6 and version == b"\0" * 2:
        header_format = "v7"
    else:
        raise AuditFailure(
            f"TAR header {index} has an unsupported format signature/version"
        )
    name = _header_text(block[0:100], f"TAR header {index} name")
    if header_format == "posix_ustar":
        prefix = _header_text(block[345:500], f"TAR header {index} prefix")
        if prefix:
            name = f"{prefix}/{name}"
    if not name:
        raise AuditFailure(f"TAR header {index} has an empty member name")
    return {
        "format": header_format,
        "name": name,
        "size": _tar_number(block[124:136], f"TAR header {index} size"),
        "type": block[156:157],
        "linkname": _header_text(block[157:257], f"TAR header {index} link name"),
    }


def _parse_pax(data: bytes, context: str) -> dict[str, str]:
    values: dict[str, str] = {}
    position = 0
    while position < len(data):
        separator = data.find(b" ", position)
        if separator < 0:
            raise AuditFailure(f"{context} has a malformed record length")
        raw_length = data[position:separator]
        if not raw_length or any(character not in b"0123456789" for character in raw_length):
            raise AuditFailure(f"{context} has a malformed record length")
        record_length = int(raw_length)
        record_end = position + record_length
        if record_length <= separator - position + 2 or record_end > len(data):
            raise AuditFailure(f"{context} has an out-of-range record length")
        record = data[separator + 1 : record_end]
        if not record.endswith(b"\n"):
            raise AuditFailure(f"{context} record does not end with a newline")
        key_bytes, marker, value_bytes = record[:-1].partition(b"=")
        if not marker or not key_bytes:
            raise AuditFailure(f"{context} record has no key/value separator")
        try:
            key = key_bytes.decode("ascii", "strict")
            value = value_bytes.decode("utf-8", "strict")
        except UnicodeDecodeError as error:
            raise AuditFailure(f"{context} contains invalid text") from error
        if key in values:
            raise AuditFailure(f"{context} repeats key {key!r}")
        if "\x00" in value or any(ord(character) == 127 for character in value):
            raise AuditFailure(f"{context} key {key!r} contains unsafe text")
        values[key] = value
        position = record_end
    return values


def _reject_sparse_pax(values: Mapping[str, str], context: str) -> None:
    """Reject vendor metadata that extractors can treat as a sparse file."""
    for key, value in values.items():
        normalized_key = key.casefold()
        if normalized_key in FORBIDDEN_PAX_SIZE_KEYS or normalized_key.startswith(
            FORBIDDEN_PAX_SPARSE_PREFIXES
        ):
            raise AuditFailure(
                f"{context} contains forbidden sparse/size-changing key {key!r}"
            )
        if normalized_key == "schily.filetype" and value.casefold() == "sparse":
            raise AuditFailure(
                f"{context} contains forbidden sparse file type in key {key!r}"
            )


def _padded_size(size: int) -> int:
    return (-size) % TAR_BLOCK_BYTES


def _read_payload(
    reader: BoundedGzipReader,
    size: int,
    context: str,
    *,
    retain: bool,
) -> bytes | None:
    value = reader.read_exact(size, context) if retain else None
    if not retain:
        reader.discard_exact(size, context)
    padding_size = _padded_size(size)
    if padding_size:
        padding = reader.read_exact(padding_size, f"padding after {context}")
        if any(padding):
            raise AuditFailure(f"padding after {context} contains non-zero bytes")
    return value


def _portable_path(
    original: str,
    kind: str,
    limits: AuditLimits,
    errors: ErrorCollector,
) -> tuple[str, str] | None:
    if not isinstance(original, str) or not original:
        errors.add("archive contains an empty or non-text member name")
        return None
    if "\x00" in original:
        errors.add("archive contains a member name with a NUL byte")
        return None
    if any(ord(character) < 32 or ord(character) == 127 for character in original):
        errors.add(f"member name {original!r} contains a control character")
        return None
    if "\\" in original:
        errors.add(f"member name {original!r} contains a backslash")
        return None
    windows_path = PureWindowsPath(original)
    if original.startswith("/") or windows_path.is_absolute() or windows_path.drive:
        errors.add(f"member name {original!r} is absolute or drive-qualified")
        return None

    canonical = original
    while canonical.startswith("./"):
        canonical = canonical[2:]
    if kind == "directory":
        if canonical.endswith("//"):
            errors.add(f"member name {original!r} has an unsafe empty path component")
            return None
        if canonical.endswith("/"):
            canonical = canonical[:-1]
    elif canonical.endswith("/"):
        errors.add(f"non-directory member name {original!r} has a trailing slash")
        return None
    if not canonical:
        errors.add(f"member name {original!r} does not identify a safe path")
        return None
    try:
        encoded = canonical.encode("utf-8", "strict")
    except UnicodeEncodeError:
        errors.add(f"member name {original!r} is not valid Unicode text")
        return None
    if len(encoded) > limits.max_path_bytes:
        errors.add(
            f"member name {original!r} exceeds path limit {limits.max_path_bytes} bytes"
        )
        return None

    components = canonical.split("/")
    if any(component in ("", ".", "..") for component in components):
        errors.add(f"member name {original!r} has an unsafe path component")
        return None
    for component in components:
        component_size = len(component.encode("utf-8"))
        if component_size > limits.max_component_bytes:
            errors.add(
                f"member name {original!r} has a component over "
                f"{limits.max_component_bytes} bytes"
            )
            return None
        if component != component.strip() or component.endswith((".", " ")):
            errors.add(f"member name {original!r} has ambiguous whitespace or trailing dots")
            return None
        if ":" in component:
            errors.add(f"member name {original!r} contains a Windows stream separator")
            return None
        reserved_base = component.rstrip(" .").split(".", 1)[0].upper()
        if reserved_base in WINDOWS_RESERVED_NAMES:
            errors.add(f"member name {original!r} contains a reserved Windows device name")
            return None
    normalized = unicodedata.normalize("NFC", "/".join(components))
    return normalized, normalized.casefold()


def _register_path(
    original: str,
    normalized: str,
    portable: str,
    kind: str,
    registry: PathRegistry,
    errors: ErrorCollector,
) -> None:
    if original in registry.exact_names:
        errors.add(f"duplicate member name {original!r}")
    registry.exact_names.add(original)
    if normalized in registry.normalized_paths:
        errors.add(
            "normalized path collision between "
            f"{registry.normalized_paths[normalized]!r} and {original!r}"
        )
    else:
        registry.normalized_paths[normalized] = original
    if portable in registry.portable_paths and registry.portable_paths[portable] != original:
        errors.add(
            "portable case/Unicode collision between "
            f"{registry.portable_paths[portable]!r} and {original!r}"
        )
    else:
        registry.portable_paths[portable] = original

    previous_kind = registry.path_kinds.get(normalized)
    if previous_kind is not None and previous_kind != kind:
        errors.add(f"path {normalized!r} has a file/directory conflict")
    registry.path_kinds[normalized] = kind

    previous_portable_kind = registry.portable_kinds.get(portable)
    if previous_portable_kind is not None and previous_portable_kind != kind:
        errors.add(f"portable path {portable!r} has a file/directory conflict")
    registry.portable_kinds[portable] = kind


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


def _effective_size(header_size: int, pax: Mapping[str, str], context: str) -> int:
    if "size" not in pax:
        return header_size
    value = pax["size"]
    if not value or any(character not in "0123456789" for character in value):
        raise AuditFailure(f"{context} has an invalid PAX size")
    return int(value)


def _check_declared_resources(
    size: int,
    projected_total: int,
    archive_size: int,
    limits: AuditLimits,
    context: str,
) -> None:
    if size > limits.max_member_uncompressed_bytes:
        raise AuditFailure(
            f"{context} declared size {size} exceeds member limit "
            f"{limits.max_member_uncompressed_bytes}"
        )
    if projected_total > limits.max_total_uncompressed_bytes:
        raise AuditFailure(
            f"total declared size {projected_total} exceeds limit "
            f"{limits.max_total_uncompressed_bytes}"
        )
    ratio = projected_total / archive_size
    if ratio > limits.max_total_compression_ratio:
        raise AuditFailure(
            f"declared compression ratio {ratio:.3f} exceeds limit "
            f"{limits.max_total_compression_ratio:.3f}"
        )


def _metadata_payload(
    reader: BoundedGzipReader,
    header: Mapping[str, Any],
    metadata_total: int,
    limits: AuditLimits,
    index: int,
) -> tuple[bytes, int]:
    size = int(header["size"])
    projected = metadata_total + size
    if size > limits.max_metadata_bytes or projected > limits.max_metadata_bytes:
        raise AuditFailure(
            f"TAR metadata exceeds limit {limits.max_metadata_bytes} bytes"
        )
    payload = _read_payload(
        reader,
        size,
        f"metadata payload at header {index}",
        retain=True,
    )
    if payload is None:
        raise AuditFailure("internal error: metadata payload was not retained")
    return payload, projected


def _audit_tar_gzip(
    stream: BinaryIO,
    archive_size: int,
    limits: AuditLimits,
    errors: ErrorCollector,
) -> dict[str, Any]:
    stream.seek(0)
    registry = PathRegistry.create()
    top_levels: Counter[str] = Counter()
    member_types: Counter[str] = Counter()
    member_count = 0
    header_count = 0
    file_count = 0
    directory_count = 0
    forbidden_count = 0
    metadata_count = 0
    metadata_bytes = 0
    declared_bytes = 0
    declared_regular_bytes = 0
    global_pax: dict[str, str] = {}
    local_pax: dict[str, str] = {}
    pending_long_name: str | None = None
    pending_long_link: str | None = None

    ratio_byte_limit = limits.max_tar_stream_bytes
    ratio_limit_applied = False
    if limits.max_total_compression_ratio < (
        limits.max_tar_stream_bytes / archive_size
    ):
        ratio_byte_limit = max(
            1,
            math.floor(archive_size * limits.max_total_compression_ratio),
        )
        ratio_limit_applied = True
    effective_stream_limit = min(limits.max_tar_stream_bytes, ratio_byte_limit)
    with gzip.GzipFile(fileobj=stream, mode="rb") as decompressed:
        limit_suffix = ""
        if ratio_limit_applied:
            limit_suffix = (
                " imposed by actual compression ratio limit "
                f"{limits.max_total_compression_ratio:.3f}"
            )
        reader = BoundedGzipReader(
            decompressed,
            effective_stream_limit,
            limit_suffix,
        )
        while True:
            block = reader.read_exact(TAR_BLOCK_BYTES, "TAR header")
            if block == b"\0" * TAR_BLOCK_BYTES:
                second = reader.read_exact(TAR_BLOCK_BYTES, "second TAR end block")
                if second != b"\0" * TAR_BLOCK_BYTES:
                    raise AuditFailure("TAR archive does not end with two zero blocks")
                break

            header_count += 1
            if header_count > limits.max_members:
                raise AuditFailure(
                    f"TAR header count exceeds member limit {limits.max_members}"
                )
            header = _parse_header(block, header_count)
            type_flag = header["type"]
            if type_flag in METADATA_TYPES:
                metadata_count += 1
                payload, metadata_bytes = _metadata_payload(
                    reader, header, metadata_bytes, limits, header_count
                )
                if type_flag in (PAX_LOCAL_TYPE, PAX_GLOBAL_TYPE):
                    values = _parse_pax(payload, f"PAX header {header_count}")
                    _reject_sparse_pax(values, f"PAX header {header_count}")
                    if type_flag == PAX_GLOBAL_TYPE:
                        if any(key in values for key in ("path", "linkpath", "size")):
                            raise AuditFailure(
                                "global PAX path/linkpath/size overrides are forbidden"
                            )
                        global_pax.update(values)
                    else:
                        if local_pax or pending_long_name is not None:
                            raise AuditFailure("stacked local TAR name metadata is forbidden")
                        local_pax = values
                elif type_flag == GNU_LONGNAME_TYPE:
                    if local_pax or pending_long_name is not None:
                        raise AuditFailure("stacked local TAR name metadata is forbidden")
                    raw_name = payload.rstrip(b"\0")
                    if b"\0" in raw_name:
                        raise AuditFailure("GNU long name contains an embedded NUL byte")
                    try:
                        pending_long_name = raw_name.decode("utf-8", "strict")
                    except UnicodeDecodeError as error:
                        raise AuditFailure("GNU long name is not valid UTF-8") from error
                else:
                    if pending_long_link is not None:
                        raise AuditFailure("stacked GNU long-link metadata is forbidden")
                    raw_link = payload.rstrip(b"\0")
                    if b"\0" in raw_link:
                        raise AuditFailure("GNU long link contains an embedded NUL byte")
                    try:
                        pending_long_link = raw_link.decode("utf-8", "strict")
                    except UnicodeDecodeError as error:
                        raise AuditFailure("GNU long link is not valid UTF-8") from error
                continue

            member_count += 1
            if member_count > limits.max_members:
                raise AuditFailure(f"member count exceeds limit {limits.max_members}")
            pax = dict(global_pax)
            pax.update(local_pax)
            local_pax = {}
            original_name = pending_long_name or pax.get("path") or str(header["name"])
            pending_long_name = None
            link_name = pending_long_link or pax.get("linkpath") or str(header["linkname"])
            pending_long_link = None
            size = _effective_size(int(header["size"]), pax, f"member {original_name!r}")
            projected_total = declared_bytes + size
            _check_declared_resources(
                size,
                projected_total,
                archive_size,
                limits,
                f"member {original_name!r}",
            )
            declared_bytes = projected_total

            if type_flag in REGULAR_TYPES:
                kind = "file"
                file_count += 1
                declared_regular_bytes += size
                member_types["regular"] += 1
            elif type_flag == DIRECTORY_TYPE:
                kind = "directory"
                directory_count += 1
                member_types["directory"] += 1
                if size != 0:
                    errors.add(f"directory member {original_name!r} declares non-zero payload")
            else:
                kind = "file"
                forbidden_count += 1
                description = FORBIDDEN_TYPES.get(
                    type_flag, f"unknown type flag 0x{type_flag.hex()}"
                )
                member_types[str(description)] += 1
                errors.add(f"member {original_name!r} is a forbidden {description}")
                if link_name and type_flag in (b"1", b"2"):
                    errors.add(
                        f"member {original_name!r} carries forbidden link target {link_name!r}"
                    )

            path = _portable_path(original_name, kind, limits, errors)
            if path is not None:
                normalized, portable = path
                normalized_path_bytes = len(normalized.encode("utf-8"))
                projected_path_bytes = (
                    registry.aggregate_normalized_path_bytes
                    + normalized_path_bytes
                )
                if projected_path_bytes > limits.max_aggregate_normalized_path_bytes:
                    raise AuditFailure(
                        "aggregate normalized path bytes "
                        f"{projected_path_bytes} exceed limit "
                        f"{limits.max_aggregate_normalized_path_bytes}"
                    )
                registry.aggregate_normalized_path_bytes = projected_path_bytes
                _register_path(
                    original_name,
                    normalized,
                    portable,
                    kind,
                    registry,
                    errors,
                )
                top_levels[normalized.split("/", 1)[0]] += 1
            _read_payload(
                reader,
                size,
                f"member {original_name!r}",
                retain=False,
            )

        if local_pax or pending_long_name is not None or pending_long_link is not None:
            raise AuditFailure("TAR archive ends with unapplied local metadata")

        _report_parent_file_conflicts(registry.path_kinds, errors, portable=False)
        _report_parent_file_conflicts(registry.portable_kinds, errors, portable=True)

        nonzero_trailing = False
        while True:
            trailing = reader.read(READ_CHUNK_BYTES)
            if not trailing:
                break
            if any(trailing):
                nonzero_trailing = True
        if nonzero_trailing:
            errors.add("decompressed TAR stream has non-zero data after its end blocks")

    if member_count == 0:
        errors.add("TAR archive must contain at least one logical member")
    declared_payload_ratio = declared_bytes / archive_size
    decompressed_stream_ratio = reader.bytes_read / archive_size
    if decompressed_stream_ratio > limits.max_total_compression_ratio:
        raise AuditFailure(
            f"decompressed TAR stream ratio {decompressed_stream_ratio:.3f} exceeds limit "
            f"{limits.max_total_compression_ratio:.3f}"
        )
    return {
        "format": "tar+gzip",
        "member_count": member_count,
        "header_count": header_count,
        "file_count": file_count,
        "directory_count": directory_count,
        "forbidden_type_count": forbidden_count,
        "metadata_record_count": metadata_count,
        "metadata_bytes": metadata_bytes,
        "declared_member_bytes": declared_bytes,
        "declared_regular_bytes": declared_regular_bytes,
        "decompressed_stream_bytes": reader.bytes_read,
        "compression_ratio": declared_payload_ratio,
        "declared_payload_compression_ratio": declared_payload_ratio,
        "decompressed_stream_compression_ratio": decompressed_stream_ratio,
        "aggregate_normalized_path_bytes": (
            registry.aggregate_normalized_path_bytes
        ),
        "member_types": dict(sorted(member_types.items())),
        "top_level_entries": dict(sorted(top_levels.items())),
    }


def _json_safe_number(value: Any) -> Any:
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    return value


def _limits_report(limits: AuditLimits) -> dict[str, Any]:
    return {
        "max_members": _json_safe_number(limits.max_members),
        "max_total_uncompressed_bytes": _json_safe_number(
            limits.max_total_uncompressed_bytes
        ),
        "max_member_uncompressed_bytes": _json_safe_number(
            limits.max_member_uncompressed_bytes
        ),
        "max_tar_stream_bytes": _json_safe_number(limits.max_tar_stream_bytes),
        "max_metadata_bytes": _json_safe_number(limits.max_metadata_bytes),
        "max_total_compression_ratio": _json_safe_number(
            limits.max_total_compression_ratio
        ),
        "max_path_bytes": _json_safe_number(limits.max_path_bytes),
        "max_component_bytes": _json_safe_number(limits.max_component_bytes),
        "max_aggregate_normalized_path_bytes": _json_safe_number(
            limits.max_aggregate_normalized_path_bytes
        ),
    }


def _empty_report(
    archive_path: Path,
    expected_size: Any,
    expected_sha256: Any,
    limits: AuditLimits,
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
        "gzip_stream_status": "NOT_RUN",
        "tar": None,
        "limits": _limits_report(limits),
        "errors": [],
        "error_count": 0,
        "errors_truncated": False,
        "read_only": True,
        "extracted": False,
    }


def _finish_report(report: dict[str, Any], errors: ErrorCollector) -> dict[str, Any]:
    report["errors"] = errors.items
    report["error_count"] = errors.total
    report["errors_truncated"] = errors.total > len(errors.items)
    report["valid"] = errors.total == 0
    report["status"] = "PASS" if report["valid"] else "FAIL"
    return report


def verify_tar_archive(
    archive_path: Path | str,
    *,
    expected_size: int,
    expected_sha256: str | None = None,
    limits: AuditLimits = AuditLimits(),
) -> dict[str, Any]:
    """Return a JSON-compatible, fail-closed TAR+gzip verification report."""
    path = Path(os.path.abspath(os.fspath(Path(archive_path).expanduser())))
    report = _empty_report(path, expected_size, expected_sha256, limits)
    errors = ErrorCollector()
    try:
        expected_size = _positive_integer(expected_size, "expected_size")
        expected_sha256 = _validated_sha256(expected_sha256)
        limits = _validated_limits(limits)
    except AuditFailure as error:
        errors.add(str(error))
        return _finish_report(report, errors)
    report["expected_size"] = expected_size
    report["expected_sha256"] = expected_sha256
    report["limits"] = _limits_report(limits)

    try:
        path_metadata = os.lstat(path)
    except OSError as error:
        errors.add(f"cannot inspect archive path: {error}")
        return _finish_report(report, errors)
    if stat.S_ISLNK(path_metadata.st_mode):
        errors.add("archive path must not be a symbolic link")
    elif not stat.S_ISREG(path_metadata.st_mode):
        errors.add("archive path must be a regular file")
    if errors.total:
        return _finish_report(report, errors)

    flags = os.O_RDONLY
    if hasattr(os, "O_CLOEXEC"):
        flags |= os.O_CLOEXEC
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    try:
        archive_fd = os.open(path, flags)
    except OSError as error:
        errors.add(f"cannot open archive safely: {error}")
        return _finish_report(report, errors)

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
                actual_sha256 = _hash_stream(stream)
                report["actual_sha256"] = actual_sha256
                after_hash = os.fstat(archive_fd)
                if _file_identity(after_hash) != _file_identity(before):
                    errors.add("archive changed while it was being hashed")
                if expected_sha256 is not None and actual_sha256 != expected_sha256:
                    errors.add(
                        f"SHA-256 mismatch: expected {expected_sha256}, found {actual_sha256}"
                    )
                if not errors.total:
                    try:
                        report["tar"] = _audit_tar_gzip(
                            stream, before.st_size, limits, errors
                        )
                        report["gzip_stream_status"] = "PASS"
                    except (
                        AuditFailure,
                        EOFError,
                        OSError,
                        UnicodeError,
                        ValueError,
                        gzip.BadGzipFile,
                        zlib.error,
                    ) as error:
                        report["gzip_stream_status"] = "FAIL"
                        errors.add(f"TAR/gzip verification failed: {error}")

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
    except OSError as error:
        errors.add(f"archive verification I/O failed: {error}")
    finally:
        os.close(archive_fd)

    return _finish_report(report, errors)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Read-only full-stream TAR+gzip safety and integrity verification."
    )
    parser.add_argument("archive", type=Path)
    parser.add_argument("--expected-size", required=True, type=int)
    parser.add_argument("--expected-sha256")
    parser.add_argument("--max-members", type=int, default=AuditLimits.max_members)
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
        "--max-tar-stream-bytes",
        type=int,
        default=AuditLimits.max_tar_stream_bytes,
    )
    parser.add_argument(
        "--max-metadata-bytes",
        type=int,
        default=AuditLimits.max_metadata_bytes,
    )
    parser.add_argument(
        "--max-total-compression-ratio",
        type=float,
        default=AuditLimits.max_total_compression_ratio,
    )
    parser.add_argument("--max-path-bytes", type=int, default=AuditLimits.max_path_bytes)
    parser.add_argument(
        "--max-component-bytes",
        type=int,
        default=AuditLimits.max_component_bytes,
    )
    parser.add_argument(
        "--max-aggregate-normalized-path-bytes",
        type=int,
        default=AuditLimits.max_aggregate_normalized_path_bytes,
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    report = verify_tar_archive(
        args.archive,
        expected_size=args.expected_size,
        expected_sha256=args.expected_sha256,
        limits=AuditLimits(
            max_members=args.max_members,
            max_total_uncompressed_bytes=args.max_total_uncompressed_bytes,
            max_member_uncompressed_bytes=args.max_member_uncompressed_bytes,
            max_tar_stream_bytes=args.max_tar_stream_bytes,
            max_metadata_bytes=args.max_metadata_bytes,
            max_total_compression_ratio=args.max_total_compression_ratio,
            max_path_bytes=args.max_path_bytes,
            max_component_bytes=args.max_component_bytes,
            max_aggregate_normalized_path_bytes=(
                args.max_aggregate_normalized_path_bytes
            ),
        ),
    )
    json.dump(report, sys.stdout, indent=2, sort_keys=True, allow_nan=False)
    sys.stdout.write("\n")
    return 0 if report["valid"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
