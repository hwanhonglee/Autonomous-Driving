from __future__ import annotations

from dataclasses import replace
import hashlib
import json
import os
from pathlib import Path
import warnings
import zipfile

import pytest

from scripts.e2e import inspect_nuplan_mini_subset as inspector
from scripts.e2e.inspect_nuplan_mini_subset import inspect_nuplan_mini_subset
from scripts.e2e.verify_zip_archive import verify_zip_archive


@pytest.fixture
def tiny_expectations() -> inspector.SubsetExpectations:
    return replace(
        inspector.DEFAULT_EXPECTATIONS,
        jpegs_per_channel_by_log=tuple(
            (log, 1) for log in sorted(inspector.EXPECTED_CAMERA_LOGS)
        ),
    )


def _write_zip(
    path: Path,
    members: list[tuple[str | zipfile.ZipInfo, bytes]],
) -> None:
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)
        with zipfile.ZipFile(path, "w", compression=zipfile.ZIP_STORED) as archive:
            for name, payload in members:
                archive.writestr(name, payload)


def _database_members(*, omit: str | None = None) -> list[tuple[str, bytes]]:
    members: list[tuple[str, bytes]] = [(inspector.DATABASE_DIRECTORY, b"")]
    members.extend(
        (
            f"{inspector.DATABASE_DIRECTORY}{stem}.db",
            f"synthetic-{stem}".encode(),
        )
        for stem in sorted(inspector.EXPECTED_DATABASE_STEMS)
        if stem != omit
    )
    members.append(("LICENSE", b"synthetic license"))
    return members


def _camera_members(
    expectations: inspector.SubsetExpectations,
    *,
    omit: tuple[str, str] | None = None,
) -> list[tuple[str, bytes]]:
    root = inspector.CAMERA_TOP_DIRECTORY
    members: list[tuple[str, bytes]] = [(f"{root}/", b"")]
    image_index = 0
    for log in sorted(expectations.camera_logs):
        members.append((f"{root}/{log}/", b""))
        for channel in expectations.camera_channels:
            members.append((f"{root}/{log}/{channel}/", b""))
            if omit == (log, channel):
                continue
            filename = f"{image_index:016x}.jpg"
            members.append((f"{root}/{log}/{channel}/{filename}", b"jpeg"))
            image_index += 1
    members.append(("LICENSE", b"synthetic license"))
    return members


def _fixtures(
    tmp_path: Path,
    expectations: inspector.SubsetExpectations,
    *,
    database_members: list[tuple[str | zipfile.ZipInfo, bytes]] | None = None,
    camera_members: list[tuple[str | zipfile.ZipInfo, bytes]] | None = None,
) -> tuple[Path, Path]:
    database = tmp_path / "nuplan-v1.1_mini.zip"
    camera = tmp_path / "nuplan-v1.1_mini_camera_0.zip"
    _write_zip(database, database_members or _database_members())
    _write_zip(camera, camera_members or _camera_members(expectations))
    return database, camera


def _write_generic_audit(path: Path, archive: Path) -> None:
    digest = hashlib.sha256(archive.read_bytes()).hexdigest()
    report = verify_zip_archive(
        archive,
        expected_size=archive.stat().st_size,
        expected_sha256=digest,
        verify_payload_crc=True,
    )
    assert report["valid"] is True
    path.write_text(json.dumps(report), encoding="utf-8")


def _mutate_stored_member_payload_without_changing_size(path: Path) -> None:
    with zipfile.ZipFile(path, "r") as archive:
        info = next(
            member
            for member in archive.infolist()
            if not member.is_dir() and member.file_size > 0
        )
    assert info.compress_type == zipfile.ZIP_STORED

    payload = bytearray(path.read_bytes())
    offset = info.header_offset
    assert payload[offset : offset + 4] == b"PK\x03\x04"
    filename_size = int.from_bytes(payload[offset + 26 : offset + 28], "little")
    extra_size = int.from_bytes(payload[offset + 28 : offset + 30], "little")
    payload_offset = offset + 30 + filename_size + extra_size
    payload[payload_offset] ^= 0x01
    original_size = path.stat().st_size
    path.write_bytes(payload)
    assert path.stat().st_size == original_size


def test_valid_exact_subset_passes_without_extracting_or_sqlite_access(
    tmp_path: Path,
    tiny_expectations: inspector.SubsetExpectations,
) -> None:
    database, camera = _fixtures(tmp_path, tiny_expectations)
    before = {
        path: (path.read_bytes(), path.stat().st_mtime_ns, path.stat().st_ctime_ns)
        for path in (database, camera)
    }

    report = inspect_nuplan_mini_subset(
        database,
        camera,
        expectations=tiny_expectations,
    )

    assert report["valid"] is True
    assert report["status"] == "PASS"
    assert report["database"]["database_count"] == 64
    assert report["camera"]["jpeg_count"] == 56
    assert report["camera"]["logs"] == sorted(inspector.EXPECTED_CAMERA_LOGS)
    assert report["camera"]["channels"] == sorted(inspector.EXPECTED_CAMERA_CHANNELS)
    assert report["correspondence"]["status"] == "PASS"
    assert report["sqlite_validation"]["status"] == "NOT_RUN_REQUIRES_SAFE_STAGING"
    assert report["database"]["sqlite_validation"]["status"] == (
        "NOT_RUN_REQUIRES_SAFE_STAGING"
    )
    assert report["read_only"] is True
    assert report["extracted"] is False
    assert report["archive_payloads_read"] is False
    for path, (payload, mtime_ns, ctime_ns) in before.items():
        after = path.stat()
        assert path.read_bytes() == payload
        assert (after.st_mtime_ns, after.st_ctime_ns) == (mtime_ns, ctime_ns)


def test_official_manifest_dimensions_are_pinned() -> None:
    expected_counts = dict(inspector.EXPECTED_JPEGS_PER_CHANNEL_BY_LOG)

    assert len(inspector.EXPECTED_DATABASE_STEMS) == 64
    assert len(inspector.EXPECTED_CAMERA_LOGS) == 7
    assert len(inspector.EXPECTED_CAMERA_CHANNELS) == 8
    assert inspector.EXPECTED_CAMERA_LOGS.issubset(inspector.EXPECTED_DATABASE_STEMS)
    assert sum(expected_counts.values()) * len(inspector.EXPECTED_CAMERA_CHANNELS) == 242_320


def test_passing_generic_crc_reports_are_bound_to_both_archives(
    tmp_path: Path,
    tiny_expectations: inspector.SubsetExpectations,
) -> None:
    database, camera = _fixtures(tmp_path, tiny_expectations)
    database_audit = tmp_path / "database-audit.json"
    camera_audit = tmp_path / "camera-audit.json"
    _write_generic_audit(database_audit, database)
    _write_generic_audit(camera_audit, camera)

    report = inspect_nuplan_mini_subset(
        database,
        camera,
        database_audit_report=database_audit,
        camera_audit_report=camera_audit,
        expectations=tiny_expectations,
    )

    assert report["valid"] is True
    assert report["generic_audits"]["database"]["status"] == "PASS"
    assert report["generic_audits"]["camera"]["status"] == "PASS"
    assert len(report["generic_audits"]["database"]["archive_sha256"]) == 64
    assert report["generic_audits"]["database"]["archive_raw_bytes_hashed"] is True
    assert report["generic_audits"]["camera"]["archive_raw_bytes_hashed"] is True


def test_stale_generic_audit_rejects_same_size_payload_mutation(
    tmp_path: Path,
    tiny_expectations: inspector.SubsetExpectations,
) -> None:
    database, camera = _fixtures(tmp_path, tiny_expectations)
    audit_path = tmp_path / "database-audit.json"
    _write_generic_audit(audit_path, database)
    _mutate_stored_member_payload_without_changing_size(database)

    report = inspect_nuplan_mini_subset(
        database,
        camera,
        database_audit_report=audit_path,
        expectations=tiny_expectations,
    )

    assert report["valid"] is False
    assert report["generic_audits"]["database"]["status"] == "FAIL"
    assert report["generic_audits"]["database"]["archive_raw_bytes_hashed"] is True
    assert any(
        "actual_sha256 does not match the currently inspected archive" in error
        for error in report["errors"]
    )


def test_missing_camera_database_fails_exact_set_and_correspondence(
    tmp_path: Path,
    tiny_expectations: inspector.SubsetExpectations,
) -> None:
    missing = sorted(inspector.EXPECTED_CAMERA_LOGS)[0]
    database, camera = _fixtures(
        tmp_path,
        tiny_expectations,
        database_members=_database_members(omit=missing),
    )

    report = inspect_nuplan_mini_subset(
        database,
        camera,
        expectations=tiny_expectations,
    )

    assert report["valid"] is False
    assert missing in report["database"]["missing_full_stems"]
    assert report["correspondence"]["status"] == "FAIL"
    assert report["correspondence"]["logs"][missing]["database_occurrences"] == 0


def test_missing_jpeg_and_unexpected_log_channel_or_extension_fail_closed(
    tmp_path: Path,
    tiny_expectations: inspector.SubsetExpectations,
) -> None:
    log = sorted(inspector.EXPECTED_CAMERA_LOGS)[0]
    channel = inspector.EXPECTED_CAMERA_CHANNELS[0]
    members = _camera_members(tiny_expectations, omit=(log, channel))
    root = inspector.CAMERA_TOP_DIRECTORY
    members.extend(
        [
            (f"{root}/unexpected-log/{channel}/00000000000000aa.jpg", b"jpeg"),
            (f"{root}/{log}/CAM_X0/00000000000000bb.jpg", b"jpeg"),
            (f"{root}/{log}/{channel}/00000000000000cc.png", b"png"),
        ]
    )
    database, camera = _fixtures(tmp_path, tiny_expectations, camera_members=members)

    report = inspect_nuplan_mini_subset(
        database,
        camera,
        expectations=tiny_expectations,
    )

    assert report["valid"] is False
    errors = "\n".join(report["errors"])
    assert "unexpected camera log" in errors
    assert "unexpected camera channel" in errors
    assert "16-lowercase-hex.jpg" in errors
    assert f"{log}/{channel} has 0 JPEGs" in errors


def test_unsafe_and_duplicate_members_fail_closed(
    tmp_path: Path,
    tiny_expectations: inspector.SubsetExpectations,
) -> None:
    database_members = _database_members()
    duplicate_name = f"{inspector.DATABASE_DIRECTORY}{sorted(inspector.EXPECTED_DATABASE_STEMS)[0]}.db"
    database_members.extend([(duplicate_name, b"duplicate"), ("../escape.db", b"unsafe")])
    database, camera = _fixtures(
        tmp_path,
        tiny_expectations,
        database_members=database_members,
    )

    report = inspect_nuplan_mini_subset(
        database,
        camera,
        expectations=tiny_expectations,
    )

    assert report["valid"] is False
    errors = "\n".join(report["errors"])
    assert "duplicate member name" in errors
    assert "unsafe path component" in errors
    assert "occurs 2 times" in errors


@pytest.mark.parametrize("which", ["database", "camera"])
def test_archive_symlinks_are_rejected(
    tmp_path: Path,
    tiny_expectations: inspector.SubsetExpectations,
    which: str,
) -> None:
    database, camera = _fixtures(tmp_path, tiny_expectations)
    target = database if which == "database" else camera
    link = tmp_path / f"{which}-link.zip"
    link.symlink_to(target)

    report = inspect_nuplan_mini_subset(
        link if which == "database" else database,
        link if which == "camera" else camera,
        expectations=tiny_expectations,
    )

    assert report["valid"] is False
    assert any(f"{which} archive path must not be a symbolic link" in e for e in report["errors"])


def test_archive_replacement_during_scan_is_detected(
    tmp_path: Path,
    tiny_expectations: inspector.SubsetExpectations,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    database, camera = _fixtures(tmp_path, tiny_expectations)
    replacement = tmp_path / "replacement.zip"
    _write_zip(replacement, _database_members())
    original_scan = inspector._scan_database_members

    def scan_then_replace(
        archive: zipfile.ZipFile,
        expectations: inspector.SubsetExpectations,
        errors: inspector.ErrorCollector,
    ) -> dict[str, object]:
        result = original_scan(archive, expectations, errors)
        os.replace(replacement, database)
        return result

    monkeypatch.setattr(inspector, "_scan_database_members", scan_then_replace)
    report = inspect_nuplan_mini_subset(
        database,
        camera,
        expectations=tiny_expectations,
    )

    assert report["valid"] is False
    assert any("different inode" in error for error in report["errors"])


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        (lambda report: report.update(actual_size=1), "actual_size"),
        (lambda report: report.update(archive="/wrong/archive.zip"), "different archive path"),
        (
            lambda report: report["payload_crc"].update(status="NOT_REQUESTED"),
            "passing full payload CRC",
        ),
        (lambda report: report["zip"].update(member_count=1), "zip.member_count"),
    ],
)
def test_generic_audit_report_mismatches_fail_closed(
    tmp_path: Path,
    tiny_expectations: inspector.SubsetExpectations,
    mutation: object,
    message: str,
) -> None:
    database, camera = _fixtures(tmp_path, tiny_expectations)
    audit_path = tmp_path / "database-audit.json"
    _write_generic_audit(audit_path, database)
    payload = json.loads(audit_path.read_text(encoding="utf-8"))
    mutation(payload)
    audit_path.write_text(json.dumps(payload), encoding="utf-8")

    report = inspect_nuplan_mini_subset(
        database,
        camera,
        database_audit_report=audit_path,
        expectations=tiny_expectations,
    )

    assert report["valid"] is False
    assert report["generic_audits"]["database"]["status"] == "FAIL"
    assert any(message in error for error in report["errors"])


def test_generic_report_symlink_and_duplicate_json_keys_are_rejected(
    tmp_path: Path,
    tiny_expectations: inspector.SubsetExpectations,
) -> None:
    database, camera = _fixtures(tmp_path, tiny_expectations)
    real_report = tmp_path / "real-report.json"
    real_report.write_text('{"schema_version": 1, "schema_version": 1}', encoding="utf-8")
    report_link = tmp_path / "report-link.json"
    report_link.symlink_to(real_report)

    symlink_report = inspect_nuplan_mini_subset(
        database,
        camera,
        database_audit_report=report_link,
        expectations=tiny_expectations,
    )
    duplicate_report = inspect_nuplan_mini_subset(
        database,
        camera,
        database_audit_report=real_report,
        expectations=tiny_expectations,
    )

    assert any("report path must not be a symbolic link" in e for e in symlink_report["errors"])
    assert any("duplicate JSON key" in e for e in duplicate_report["errors"])


def test_cli_emits_json_and_uses_zero_or_two_exit_status(
    tmp_path: Path,
    tiny_expectations: inspector.SubsetExpectations,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    database, camera = _fixtures(tmp_path, tiny_expectations)
    monkeypatch.setattr(inspector, "DEFAULT_EXPECTATIONS", tiny_expectations)

    passed = inspector.main([str(database), str(camera)])
    passed_report = json.loads(capsys.readouterr().out)
    database.unlink()
    failed = inspector.main([str(database), str(camera)])
    failed_report = json.loads(capsys.readouterr().out)

    assert passed == 0
    assert passed_report["status"] == "PASS"
    assert failed == 2
    assert failed_report["status"] == "FAIL"
