#!/usr/bin/env python3
"""Create and compare fail-closed content manifests for directory trees.

Only regular files are inventoried.  Directories are traversal containers;
symbolic links and every other filesystem object are rejected.  Files are
opened without following links where the platform supports it, and their
identity is checked before and after hashing to detect replacement or writes.

The command is read-only and emits its JSON report exclusively on stdout.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import stat
import sys
from typing import Any, Sequence
import unicodedata


READ_CHUNK_BYTES = 8 * 1024 * 1024
MAX_REPORTED_ERRORS = 100
MAX_COMPONENT_BYTES = 255
MAX_RELATIVE_PATH_BYTES = 4096
MAX_SAFE_DEPTH = 128
MANIFEST_SCHEMA = "autoware-e2e.tree-manifest.v1"


@dataclass(frozen=True)
class ManifestLimits:
    """Memory and traversal limits for an untrusted directory tree."""

    max_files: int = 1_000_000
    max_directories: int = 1_000_000
    max_total_path_bytes: int = 256 * 1024**2
    max_depth: int = 64


class ManifestFailure(Exception):
    """A deterministic validation failure that is safe to report."""


class ErrorCollector:
    """Count every error while retaining a bounded diagnostic sample."""

    def __init__(self) -> None:
        self.items: list[str] = []
        self.total = 0

    def add(self, message: str) -> None:
        self.total += 1
        if len(self.items) < MAX_REPORTED_ERRORS:
            self.items.append(message)


def _identity(metadata: os.stat_result) -> tuple[int, int, int, int, int, int]:
    return (
        metadata.st_dev,
        metadata.st_ino,
        metadata.st_mode,
        metadata.st_size,
        metadata.st_mtime_ns,
        metadata.st_ctime_ns,
    )


def _positive_integer(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ManifestFailure(f"{label} must be a positive integer")
    return value


def _validated_limits(limits: ManifestLimits) -> ManifestLimits:
    validated = ManifestLimits(
        max_files=_positive_integer(limits.max_files, "max_files"),
        max_directories=_positive_integer(limits.max_directories, "max_directories"),
        max_total_path_bytes=_positive_integer(
            limits.max_total_path_bytes, "max_total_path_bytes"
        ),
        max_depth=_positive_integer(limits.max_depth, "max_depth"),
    )
    if validated.max_depth > MAX_SAFE_DEPTH:
        raise ManifestFailure(f"max_depth must not exceed safe limit {MAX_SAFE_DEPTH}")
    return validated


def _display_name(name: str) -> str:
    return ascii(name)


def _normalized_relative_path(parts: tuple[str, ...]) -> tuple[str, bytes]:
    if not parts:
        raise ManifestFailure("internal error: an empty relative path was requested")
    for component in parts:
        if component in ("", ".", ".."):
            raise ManifestFailure(f"forbidden path component: {_display_name(component)}")
        if "/" in component or "\\" in component:
            raise ManifestFailure(
                f"path component contains a separator: {_display_name(component)}"
            )
        if any(ord(character) < 32 or ord(character) == 127 for character in component):
            raise ManifestFailure(
                f"path component contains a control character: {_display_name(component)}"
            )
        if unicodedata.normalize("NFC", component) != component:
            raise ManifestFailure(
                f"path component is not Unicode NFC-normalized: {_display_name(component)}"
            )
        try:
            component_bytes = component.encode("utf-8", errors="strict")
        except UnicodeError as error:
            raise ManifestFailure(
                f"path component is not valid UTF-8 text: {_display_name(component)}"
            ) from error
        if len(component_bytes) > MAX_COMPONENT_BYTES:
            raise ManifestFailure(
                f"path component exceeds {MAX_COMPONENT_BYTES} UTF-8 bytes: "
                f"{_display_name(component)}"
            )

    relative = PurePosixPath(*parts)
    if relative.is_absolute() or relative.parts != parts:
        raise ManifestFailure(f"relative path did not normalize safely: {relative.as_posix()!r}")
    text = relative.as_posix()
    encoded = text.encode("utf-8", errors="strict")
    if len(encoded) > MAX_RELATIVE_PATH_BYTES:
        raise ManifestFailure(
            f"relative path exceeds {MAX_RELATIVE_PATH_BYTES} UTF-8 bytes: {text!r}"
        )
    return text, encoded


def _open_flags(*, directory: bool = False) -> int:
    flags = os.O_RDONLY
    if hasattr(os, "O_CLOEXEC"):
        flags |= os.O_CLOEXEC
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    if directory and hasattr(os, "O_DIRECTORY"):
        flags |= os.O_DIRECTORY
    return flags


def _hash_open_file(file_descriptor: int) -> str:
    digest = hashlib.sha256()
    while True:
        chunk = os.read(file_descriptor, READ_CHUNK_BYTES)
        if not chunk:
            return digest.hexdigest()
        digest.update(chunk)


def _hash_regular_file(
    parent_fd: int,
    name: str,
    expected: os.stat_result,
    relative_path: str,
) -> tuple[int, str]:
    try:
        file_fd = os.open(name, _open_flags(), dir_fd=parent_fd)
    except OSError as error:
        raise ManifestFailure(f"cannot open regular file {relative_path!r}: {error}") from error
    try:
        before = os.fstat(file_fd)
        if not stat.S_ISREG(before.st_mode):
            raise ManifestFailure(f"opened object is not a regular file: {relative_path!r}")
        if _identity(before) != _identity(expected):
            raise ManifestFailure(
                f"file changed before it could be opened: {relative_path!r}"
            )
        digest = _hash_open_file(file_fd)
        after = os.fstat(file_fd)
        if _identity(after) != _identity(before):
            raise ManifestFailure(f"file changed while it was hashed: {relative_path!r}")
        try:
            final_path = os.stat(name, dir_fd=parent_fd, follow_symlinks=False)
        except OSError as error:
            raise ManifestFailure(
                f"file path changed while it was hashed: {relative_path!r}: {error}"
            ) from error
        if _identity(final_path) != _identity(after):
            raise ManifestFailure(
                f"file path refers to a different object after hashing: {relative_path!r}"
            )
        return before.st_size, digest
    finally:
        os.close(file_fd)


def _entry_kind(mode: int) -> str:
    if stat.S_ISLNK(mode):
        return "symbolic link"
    if stat.S_ISFIFO(mode):
        return "FIFO"
    if stat.S_ISSOCK(mode):
        return "socket"
    if stat.S_ISCHR(mode):
        return "character device"
    if stat.S_ISBLK(mode):
        return "block device"
    return "unsupported filesystem object"


def _scan_entry(
    directory_fd: int,
    name: str,
    parts: tuple[str, ...],
    limits: ManifestLimits,
    entries: list[dict[str, Any]],
    counters: dict[str, int],
) -> None:
    child_parts = (*parts, name)
    relative_path, encoded_path = _normalized_relative_path(child_parts)
    counters["path_bytes"] += len(encoded_path)
    if counters["path_bytes"] > limits.max_total_path_bytes:
        raise ManifestFailure(
            "aggregate relative-path bytes exceed configured maximum "
            f"{limits.max_total_path_bytes}"
        )
    try:
        metadata = os.lstat(name, dir_fd=directory_fd)
    except OSError as error:
        raise ManifestFailure(f"cannot inspect path {relative_path!r}: {error}") from error

    if stat.S_ISREG(metadata.st_mode):
        counters["files"] += 1
        if counters["files"] > limits.max_files:
            raise ManifestFailure(
                f"file count exceeds configured maximum {limits.max_files}"
            )
        size, digest = _hash_regular_file(directory_fd, name, metadata, relative_path)
        entries.append({"path": relative_path, "size": size, "sha256": digest})
        counters["bytes"] += size
        return

    if stat.S_ISDIR(metadata.st_mode):
        depth = len(child_parts)
        if depth > limits.max_depth:
            raise ManifestFailure(
                f"directory depth {depth} exceeds configured maximum {limits.max_depth}: "
                f"{relative_path!r}"
            )
        try:
            child_fd = os.open(
                name,
                _open_flags(directory=True),
                dir_fd=directory_fd,
            )
        except OSError as error:
            raise ManifestFailure(
                f"cannot open directory {relative_path!r}: {error}"
            ) from error
        try:
            opened = os.fstat(child_fd)
            if not stat.S_ISDIR(opened.st_mode):
                raise ManifestFailure(
                    f"opened traversal object is not a directory: {relative_path!r}"
                )
            if _identity(opened) != _identity(metadata):
                raise ManifestFailure(
                    f"directory changed before it could be opened: {relative_path!r}"
                )
            _scan_directory(child_fd, child_parts, limits, entries, counters)
            after = os.fstat(child_fd)
            if _identity(after) != _identity(opened):
                raise ManifestFailure(
                    f"directory changed while it was scanned: {relative_path!r}"
                )
            try:
                final_path = os.stat(
                    name, dir_fd=directory_fd, follow_symlinks=False
                )
            except OSError as error:
                raise ManifestFailure(
                    f"directory path changed while it was scanned: "
                    f"{relative_path!r}: {error}"
                ) from error
            if _identity(final_path) != _identity(after):
                raise ManifestFailure(
                    "directory path refers to a different object after scanning: "
                    f"{relative_path!r}"
                )
        finally:
            os.close(child_fd)
        return

    raise ManifestFailure(
        f"forbidden {_entry_kind(metadata.st_mode)} at {relative_path!r}"
    )


def _scan_directory(
    directory_fd: int,
    parts: tuple[str, ...],
    limits: ManifestLimits,
    entries: list[dict[str, Any]],
    counters: dict[str, int],
) -> None:
    if len(parts) > limits.max_depth:
        raise ManifestFailure(
            f"directory depth exceeds configured maximum {limits.max_depth}"
        )
    before = os.fstat(directory_fd)
    if not stat.S_ISDIR(before.st_mode):
        raise ManifestFailure("opened traversal object is not a directory")
    counters["directories"] += 1
    if counters["directories"] > limits.max_directories:
        raise ManifestFailure(
            f"directory count exceeds configured maximum {limits.max_directories}"
        )

    location = PurePosixPath(*parts).as_posix() if parts else "."
    try:
        iterator = os.scandir(directory_fd)
    except OSError as error:
        raise ManifestFailure(f"cannot scan directory {location!r}: {error}") from error
    with iterator:
        while True:
            try:
                child = next(iterator)
            except StopIteration:
                break
            except OSError as error:
                raise ManifestFailure(
                    f"cannot continue scanning directory {location!r}: {error}"
                ) from error
            _scan_entry(
                directory_fd,
                child.name,
                parts,
                limits,
                entries,
                counters,
            )

    after = os.fstat(directory_fd)
    if _identity(after) != _identity(before):
        location = PurePosixPath(*parts).as_posix() if parts else "."
        raise ManifestFailure(f"directory changed while it was scanned: {location!r}")


def _manifest_fingerprint(entries: list[dict[str, Any]]) -> str:
    """Hash an unambiguous, versioned binary encoding of sorted file records."""
    digest = hashlib.sha256()
    digest.update(b"autoware-e2e.tree-manifest.fingerprint.v1\0")
    for entry in entries:
        path_bytes = entry["path"].encode("utf-8", errors="strict")
        file_digest = bytes.fromhex(entry["sha256"])
        digest.update(len(path_bytes).to_bytes(8, "big"))
        digest.update(path_bytes)
        digest.update(entry["size"].to_bytes(16, "big"))
        digest.update(file_digest)
    return digest.hexdigest()


def build_tree_manifest(
    root: Path | str,
    *,
    limits: ManifestLimits = ManifestLimits(),
) -> dict[str, Any]:
    """Return a read-only manifest report for ``root`` without following links."""
    path = Path(root)
    errors = ErrorCollector()
    report: dict[str, Any] = {
        "root": os.fspath(path),
        "status": "FAIL",
        "valid": False,
        "read_only": True,
        "limits": {
            "max_files": limits.max_files,
            "max_directories": limits.max_directories,
            "max_total_path_bytes": limits.max_total_path_bytes,
            "max_depth": limits.max_depth,
            "hard_max_depth": MAX_SAFE_DEPTH,
        },
        "file_count": 0,
        "directory_count": 0,
        "total_size_bytes": 0,
        "total_path_bytes": 0,
        "manifest_sha256": None,
        "files": [],
        "errors": [],
        "error_count": 0,
        "errors_truncated": False,
    }
    try:
        limits = _validated_limits(limits)
    except ManifestFailure as error:
        errors.add(str(error))
        report["errors"] = errors.items
        report["error_count"] = errors.total
        return report

    try:
        path_metadata = os.lstat(path)
    except (OSError, ValueError) as error:
        errors.add(f"cannot inspect tree root: {error}")
        report["errors"] = errors.items
        report["error_count"] = errors.total
        return report
    if stat.S_ISLNK(path_metadata.st_mode):
        errors.add("tree root must not be a symbolic link")
    elif not stat.S_ISDIR(path_metadata.st_mode):
        errors.add("tree root must be a directory")
    if errors.total:
        report["errors"] = errors.items
        report["error_count"] = errors.total
        return report

    try:
        root_fd = os.open(path, _open_flags(directory=True))
    except (OSError, ValueError) as error:
        errors.add(f"cannot open tree root safely: {error}")
        report["errors"] = errors.items
        report["error_count"] = errors.total
        return report

    entries: list[dict[str, Any]] = []
    counters = {"files": 0, "directories": 0, "bytes": 0, "path_bytes": 0}
    try:
        try:
            opened = os.fstat(root_fd)
        except OSError as error:
            errors.add(f"cannot inspect opened tree root: {error}")
        else:
            if not stat.S_ISDIR(opened.st_mode):
                errors.add("opened tree root is not a directory")
            elif _identity(opened) != _identity(path_metadata):
                errors.add("tree root changed before it could be opened")
            else:
                try:
                    _scan_directory(root_fd, (), limits, entries, counters)
                except RecursionError:
                    errors.add(
                        "tree traversal reached the interpreter recursion boundary "
                        f"before configured max_depth {limits.max_depth}"
                    )
                except (ManifestFailure, OSError, ValueError) as error:
                    errors.add(str(error))

            try:
                after = os.fstat(root_fd)
            except OSError as error:
                errors.add(f"cannot re-inspect opened tree root: {error}")
            else:
                if _identity(after) != _identity(opened):
                    errors.add("tree root changed while it was inventoried")
        try:
            final_path = os.lstat(path)
        except (OSError, ValueError) as error:
            errors.add(f"tree root path changed while it was inventoried: {error}")
        else:
            if _identity(final_path) != _identity(path_metadata):
                errors.add("tree root path refers to a different object after inventory")
    finally:
        try:
            os.close(root_fd)
        except OSError as error:
            errors.add(f"cannot close opened tree root: {error}")

    entries.sort(key=lambda entry: entry["path"])
    report.update(
        {
            "file_count": counters["files"],
            "directory_count": counters["directories"],
            "total_size_bytes": counters["bytes"],
            "total_path_bytes": counters["path_bytes"],
            "files": entries,
            "errors": errors.items,
            "error_count": errors.total,
            "errors_truncated": errors.total > len(errors.items),
        }
    )
    if not errors.total:
        report["manifest_sha256"] = _manifest_fingerprint(entries)
        report["valid"] = True
        report["status"] = "PASS"
    return report


def _comparison(source: dict[str, Any], destination: dict[str, Any]) -> dict[str, Any]:
    report: dict[str, Any] = {
        "requested": True,
        "status": "NOT_RUN",
        "matching": False,
        "missing_file_count": 0,
        "extra_file_count": 0,
        "different_file_count": 0,
        "missing_files": [],
        "extra_files": [],
        "different_files": [],
    }
    if not source["valid"] or not destination["valid"]:
        return report

    source_files = {entry["path"]: entry for entry in source["files"]}
    destination_files = {entry["path"]: entry for entry in destination["files"]}
    missing = sorted(source_files.keys() - destination_files.keys())
    extra = sorted(destination_files.keys() - source_files.keys())
    different = []
    for path in sorted(source_files.keys() & destination_files.keys()):
        source_entry = source_files[path]
        destination_entry = destination_files[path]
        if (
            source_entry["size"] != destination_entry["size"]
            or source_entry["sha256"] != destination_entry["sha256"]
        ):
            different.append(
                {
                    "path": path,
                    "source_size": source_entry["size"],
                    "destination_size": destination_entry["size"],
                    "source_sha256": source_entry["sha256"],
                    "destination_sha256": destination_entry["sha256"],
                }
            )

    matching = not missing and not extra and not different
    report.update(
        {
            "status": "PASS" if matching else "FAIL",
            "matching": matching,
            "missing_file_count": len(missing),
            "extra_file_count": len(extra),
            "different_file_count": len(different),
            "missing_files": missing,
            "extra_files": extra,
            "different_files": different,
        }
    )
    return report


def verify_tree_manifest(
    source_root: Path | str,
    destination_root: Path | str | None = None,
    *,
    limits: ManifestLimits = ManifestLimits(),
) -> dict[str, Any]:
    """Build one manifest or compare two directory trees exactly by content."""
    source = build_tree_manifest(source_root, limits=limits)
    destination = (
        build_tree_manifest(destination_root, limits=limits)
        if destination_root is not None
        else None
    )
    comparison = (
        _comparison(source, destination)
        if destination is not None
        else {
            "requested": False,
            "status": "NOT_REQUESTED",
            "matching": None,
        }
    )
    valid = source["valid"] and (
        destination is None or (destination["valid"] and comparison["matching"])
    )
    errors = [f"source: {error}" for error in source["errors"]]
    if destination is not None:
        errors.extend(f"destination: {error}" for error in destination["errors"])
        if destination["valid"] and source["valid"] and not comparison["matching"]:
            errors.append("source and destination file manifests differ")
    return {
        "schema_version": MANIFEST_SCHEMA,
        "status": "PASS" if valid else "FAIL",
        "valid": valid,
        "read_only": True,
        "source": source,
        "destination": destination,
        "comparison": comparison,
        "errors": errors[:MAX_REPORTED_ERRORS],
        "error_count": len(errors),
        "errors_truncated": len(errors) > MAX_REPORTED_ERRORS,
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Create a read-only SHA-256 tree manifest, optionally requiring an exact "
            "source/destination content match. JSON is written only to stdout."
        )
    )
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path, nargs="?")
    parser.add_argument("--max-files", type=int, default=ManifestLimits.max_files)
    parser.add_argument(
        "--max-directories", type=int, default=ManifestLimits.max_directories
    )
    parser.add_argument(
        "--max-total-path-bytes",
        type=int,
        default=ManifestLimits.max_total_path_bytes,
    )
    parser.add_argument(
        "--max-depth",
        type=int,
        default=ManifestLimits.max_depth,
        help=f"maximum directory depth (hard safety ceiling: {MAX_SAFE_DEPTH})",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    report = verify_tree_manifest(
        args.source,
        args.destination,
        limits=ManifestLimits(
            max_files=args.max_files,
            max_directories=args.max_directories,
            max_total_path_bytes=args.max_total_path_bytes,
            max_depth=args.max_depth,
        ),
    )
    json.dump(report, sys.stdout, indent=2, sort_keys=True, allow_nan=False)
    sys.stdout.write("\n")
    return 0 if report["valid"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
