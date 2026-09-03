#!/usr/bin/env python3
"""Verify an offline Bench2Drive legacy Mini archive set, without mutating it.

This program only enumerates and reads an existing directory.  It never
downloads, installs, extracts, repairs, renames, or deletes anything.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import stat
import sys
from typing import Any, Mapping, Sequence


@dataclass(frozen=True)
class ArchiveSpec:
    """Immutable size and digest contract for one official archive."""

    size: int
    sha256: str


# Source: Thinklab-SJTU/Bench2Drive tag 0.0.3,
# docs/bench2drive_mini_10.json.  These are the legacy PNG-depth archives
# expected by the upstream Bench2DriveZoo uniad/vad data preparation code.
OFFICIAL_LEGACY_MINI_ARCHIVES: Mapping[str, ArchiveSpec] = {
    "AccidentTwoWays_Town12_Route1444_Weather0.tar.gz": ArchiveSpec(
        size=341_845_960,
        sha256="2690cb0053a2e9208f50f9d48c4a1c5befae4179c07e6fb8f06dc4434fb82178",
    ),
    "Accident_Town03_Route156_Weather0.tar.gz": ArchiveSpec(
        size=264_497_958,
        sha256="09780cfaae07b13f65f11addfffbda8dea7723a6dfbbe0b49f493303805d24fa",
    ),
    "ConstructionObstacle_Town05_Route68_Weather8.tar.gz": ArchiveSpec(
        size=287_017_092,
        sha256="7a1e93168780f5dc4d38b6b5b357e74a62736c935adc3fa74b1ed35028c61e76",
    ),
    "ControlLoss_Town11_Route401_Weather11.tar.gz": ArchiveSpec(
        size=99_078_817,
        sha256="9e3abe0c9b25f0e50e597ac0f62a5cf681acd9631e439b23cabac77b85355043",
    ),
    "DynamicObjectCrossing_Town02_Route13_Weather6.tar.gz": ArchiveSpec(
        size=280_176_944,
        sha256="b040f5993726dfb3a1dc1aaf5e2e4f93404a0124272d6f47f2469fca53ca25f2",
    ),
    "HardBreakRoute_Town01_Route30_Weather3.tar.gz": ArchiveSpec(
        size=426_576_889,
        sha256="2f6593d05e288a88cf37d8043d1e448842a4b1ceaed5544fe13100c174a4cb04",
    ),
    "OppositeVehicleTakingPriority_Town13_Route600_Weather2.tar.gz": ArchiveSpec(
        size=177_697_783,
        sha256="757a8e7415081447c2d172592723de194b05ad3fb9416e9fc215844e1dac58dc",
    ),
    "ParkedObstacle_Town10HD_Route371_Weather7.tar.gz": ArchiveSpec(
        size=264_725_091,
        sha256="78464d659875f8b2bc5901c763bc9ba154c3eaf021196f4d6077d9ef17c705e4",
    ),
    "VehicleTurningRoute_Town15_Route443_Weather1.tar.gz": ArchiveSpec(
        size=487_312_838,
        sha256="97a066963571c10fd91ec0ee45bd5fc598e79ea2f1cf153e8617887ad83b2b5a",
    ),
    "YieldToEmergencyVehicle_Town04_Route165_Weather7.tar.gz": ArchiveSpec(
        size=183_804_370,
        sha256="feb52345f09a5358728dd2ed32db93cf64a5d2b0d04d4719a38d408c60c212df",
    ),
}

OFFICIAL_MANIFEST_SOURCE = (
    "https://github.com/Thinklab-SJTU/Bench2Drive/blob/0.0.3/"
    "docs/bench2drive_mini_10.json"
)
READ_CHUNK_BYTES = 8 * 1024 * 1024


def _empty_report(directory: Path, manifest: Mapping[str, ArchiveSpec]) -> dict[str, Any]:
    return {
        "schema_version": 1,
        "status": "FAIL",
        "valid": False,
        "archive_directory": str(directory),
        "manifest_source": OFFICIAL_MANIFEST_SOURCE,
        "expected_archive_count": len(manifest),
        "expected_total_bytes": sum(item.size for item in manifest.values()),
        "verified_archive_count": 0,
        "verified_total_bytes": 0,
        "missing": [],
        "extra": [],
        "partial": [],
        "symlinks": [],
        "invalid_entries": [],
        "files": [],
        "errors": [],
        "read_only": True,
    }


def _manifest_errors(manifest: Mapping[str, ArchiveSpec]) -> list[str]:
    errors: list[str] = []
    for name, item in manifest.items():
        if not name or name != Path(name).name or not name.endswith(".tar.gz"):
            errors.append(f"manifest has an unsafe archive name: {name!r}")
        if not isinstance(item, ArchiveSpec):
            errors.append(f"manifest entry {name!r} is not an ArchiveSpec")
            continue
        if isinstance(item.size, bool) or not isinstance(item.size, int) or item.size <= 0:
            errors.append(f"manifest entry {name!r} has an invalid size")
        if (
            not isinstance(item.sha256, str)
            or len(item.sha256) != 64
            or any(character not in "0123456789abcdef" for character in item.sha256)
        ):
            errors.append(f"manifest entry {name!r} has an invalid SHA-256")
    return errors


def _directory_path(value: Path | str) -> Path:
    expanded = Path(value).expanduser()
    return Path(os.path.abspath(os.fspath(expanded)))


def _entry_kind(mode: int) -> str:
    if stat.S_ISDIR(mode):
        return "directory"
    if stat.S_ISREG(mode):
        return "regular_file"
    if stat.S_ISFIFO(mode):
        return "fifo"
    if stat.S_ISSOCK(mode):
        return "socket"
    if stat.S_ISCHR(mode):
        return "character_device"
    if stat.S_ISBLK(mode):
        return "block_device"
    return "other"


def _hash_open_regular_file(
    directory_fd: int,
    name: str,
) -> tuple[int, str, bool]:
    """Hash a directory-relative regular file without following a symlink."""
    flags = os.O_RDONLY
    if hasattr(os, "O_CLOEXEC"):
        flags |= os.O_CLOEXEC
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    file_fd = os.open(name, flags, dir_fd=directory_fd)
    try:
        before = os.fstat(file_fd)
        if not stat.S_ISREG(before.st_mode):
            raise OSError("entry is not a regular file")
        digest = hashlib.sha256()
        with os.fdopen(file_fd, "rb", closefd=False) as stream:
            for block in iter(lambda: stream.read(READ_CHUNK_BYTES), b""):
                digest.update(block)
        after = os.fstat(file_fd)
        unchanged = (
            before.st_dev,
            before.st_ino,
            before.st_size,
            before.st_mtime_ns,
            before.st_ctime_ns,
        ) == (
            after.st_dev,
            after.st_ino,
            after.st_size,
            after.st_mtime_ns,
            after.st_ctime_ns,
        )
        return after.st_size, digest.hexdigest(), unchanged
    finally:
        os.close(file_fd)


def verify_archive_directory(
    archive_directory: Path | str,
    manifest: Mapping[str, ArchiveSpec] = OFFICIAL_LEGACY_MINI_ARCHIVES,
) -> dict[str, Any]:
    """Return a deterministic, fail-closed report for an offline archive directory."""
    directory = _directory_path(archive_directory)
    report = _empty_report(directory, manifest)
    report["errors"].extend(_manifest_errors(manifest))
    if report["errors"]:
        return report

    try:
        directory_metadata = directory.lstat()
    except OSError as error:
        report["errors"].append(f"cannot inspect archive directory: {error}")
        return report
    if stat.S_ISLNK(directory_metadata.st_mode):
        report["errors"].append("archive directory must not be a symlink")
        return report
    if not stat.S_ISDIR(directory_metadata.st_mode):
        report["errors"].append("archive directory is not a directory")
        return report

    directory_flags = os.O_RDONLY
    if hasattr(os, "O_DIRECTORY"):
        directory_flags |= os.O_DIRECTORY
    if hasattr(os, "O_CLOEXEC"):
        directory_flags |= os.O_CLOEXEC
    if hasattr(os, "O_NOFOLLOW"):
        directory_flags |= os.O_NOFOLLOW

    try:
        directory_fd = os.open(directory, directory_flags)
    except OSError as error:
        report["errors"].append(f"cannot open archive directory safely: {error}")
        return report

    expected_names = set(manifest)
    entries: dict[str, os.DirEntry[str]] = {}
    try:
        try:
            with os.scandir(directory_fd) as iterator:
                entries = {entry.name: entry for entry in iterator}
        except OSError as error:
            report["errors"].append(f"cannot enumerate archive directory: {error}")
            return report

        observed_names = set(entries)
        report["missing"] = sorted(expected_names - observed_names)
        report["extra"] = sorted(observed_names - expected_names)
        report["partial"] = sorted(
            name
            for name in observed_names
            if name.endswith(".part") or ".part." in name
        )

        for name in sorted(observed_names):
            entry = entries[name]
            try:
                metadata = entry.stat(follow_symlinks=False)
            except OSError as error:
                report["invalid_entries"].append(
                    {"name": name, "reason": f"cannot stat entry: {error}"}
                )
                continue
            if stat.S_ISLNK(metadata.st_mode):
                report["symlinks"].append(name)
                report["invalid_entries"].append(
                    {"name": name, "reason": "symlink is forbidden"}
                )
            elif not stat.S_ISREG(metadata.st_mode):
                report["invalid_entries"].append(
                    {"name": name, "reason": _entry_kind(metadata.st_mode)}
                )

        for name in sorted(expected_names & observed_names):
            expected = manifest[name]
            file_report: dict[str, Any] = {
                "name": name,
                "status": "FAIL",
                "expected_size": expected.size,
                "actual_size": None,
                "expected_sha256": expected.sha256,
                "actual_sha256": None,
                "errors": [],
            }
            report["files"].append(file_report)
            entry = entries[name]
            try:
                metadata = entry.stat(follow_symlinks=False)
            except OSError as error:
                file_report["errors"].append(f"cannot stat file: {error}")
                continue
            if stat.S_ISLNK(metadata.st_mode):
                file_report["errors"].append("symlink is forbidden")
                continue
            if not stat.S_ISREG(metadata.st_mode):
                file_report["errors"].append(
                    f"entry is not a regular file: {_entry_kind(metadata.st_mode)}"
                )
                continue
            try:
                actual_size, actual_sha256, unchanged = _hash_open_regular_file(
                    directory_fd, name
                )
            except OSError as error:
                file_report["errors"].append(f"cannot read file safely: {error}")
                continue
            file_report["actual_size"] = actual_size
            file_report["actual_sha256"] = actual_sha256
            if not unchanged:
                file_report["errors"].append("file changed while it was being verified")
            if actual_size != expected.size:
                file_report["errors"].append("size mismatch")
            if actual_sha256 != expected.sha256:
                file_report["errors"].append("SHA-256 mismatch")
            if not file_report["errors"]:
                file_report["status"] = "PASS"
                report["verified_archive_count"] += 1
                report["verified_total_bytes"] += actual_size
    finally:
        os.close(directory_fd)

    if report["missing"]:
        report["errors"].append("one or more official archives are missing")
    if report["extra"]:
        report["errors"].append("unexpected directory entries are present")
    if report["partial"]:
        report["errors"].append("partial download files are present")
    if report["symlinks"]:
        report["errors"].append("symlinks are present")
    if report["invalid_entries"]:
        report["errors"].append("non-regular or unreadable entries are present")
    if any(item["status"] != "PASS" for item in report["files"]):
        report["errors"].append("one or more official archives failed verification")

    report["valid"] = not report["errors"]
    report["status"] = "PASS" if report["valid"] else "FAIL"
    return report


def _human_report(report: Mapping[str, Any]) -> str:
    lines = [
        "Bench2Drive legacy Mini archive verification: " + str(report["status"]),
        "Directory: " + str(report["archive_directory"]),
        "Verified: {}/{} archives ({} / {} bytes)".format(
            report["verified_archive_count"],
            report["expected_archive_count"],
            report["verified_total_bytes"],
            report["expected_total_bytes"],
        ),
    ]
    for key, label in (
        ("missing", "Missing"),
        ("extra", "Extra"),
        ("partial", "Partial"),
        ("symlinks", "Symlinks"),
    ):
        if report[key]:
            lines.append(f"{label}: " + ", ".join(report[key]))
    for item in report["files"]:
        if item["status"] != "PASS":
            lines.append(f"Invalid {item['name']}: " + "; ".join(item["errors"]))
    for error in report["errors"]:
        lines.append("Error: " + str(error))
    return "\n".join(lines)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Read-only, fail-closed verification of the ten official legacy "
            "Bench2Drive Mini tar.gz archives."
        )
    )
    parser.add_argument("archive_directory", type=Path)
    parser.add_argument(
        "--json",
        action="store_true",
        help="emit the complete machine-readable verification report",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    report = verify_archive_directory(
        args.archive_directory, OFFICIAL_LEGACY_MINI_ARCHIVES
    )
    if args.json:
        json.dump(report, sys.stdout, indent=2, sort_keys=True)
        sys.stdout.write("\n")
    else:
        print(_human_report(report))
    return 0 if report["valid"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
