from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import stat
import struct
import warnings
import zipfile

import pytest

from scripts.e2e import verify_zip_archive as verifier
from scripts.e2e.verify_zip_archive import AuditLimits, verify_zip_archive


def _digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write_zip(
    path: Path,
    members: list[tuple[str | zipfile.ZipInfo, bytes]],
    *,
    compression: int = zipfile.ZIP_STORED,
) -> None:
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)
        with zipfile.ZipFile(path, "w", compression=compression, allowZip64=True) as archive:
            for name, payload in members:
                archive.writestr(name, payload)


def _verify(
    path: Path,
    *,
    expected_sha256: str | None = None,
    limits: AuditLimits = AuditLimits(),
    verify_payload_crc: bool = False,
) -> dict[str, object]:
    return verify_zip_archive(
        path,
        expected_size=path.stat().st_size,
        expected_sha256=expected_sha256,
        limits=limits,
        verify_payload_crc=verify_payload_crc,
    )


def _patch_first_central_field(path: Path, offset: int, value: int) -> None:
    payload = bytearray(path.read_bytes())
    central_offset = payload.index(verifier.EOCD_SIGNATURE)
    end_values = verifier.EOCD.unpack_from(payload, central_offset)
    directory_offset = int(end_values[6])
    assert payload[directory_offset : directory_offset + 4] == b"PK\x01\x02"
    struct.pack_into("<H", payload, directory_offset + offset, value)
    path.write_bytes(payload)


def _write_small_zip64_with_member_extra(path: Path) -> bytes:
    """Write genuine ZIP64 member/end metadata without allocating a >4 GiB fixture."""
    original_limit = zipfile.ZIP64_LIMIT
    try:
        zipfile.ZIP64_LIMIT = 32
        _write_zip(
            path,
            [("payload.bin", b"x" * 128)],
            compression=zipfile.ZIP_DEFLATED,
        )
    finally:
        zipfile.ZIP64_LIMIT = original_limit

    payload = bytearray(path.read_bytes())
    eocd_offset = payload.rfind(verifier.EOCD_SIGNATURE)
    values = list(verifier.EOCD.unpack_from(payload, eocd_offset))
    assert verifier.ZIP64_EOCD_SIGNATURE in payload
    assert verifier.ZIP64_LOCATOR_SIGNATURE in payload
    values[3] = 0xFFFF
    values[4] = 0xFFFF
    values[5] = 0xFFFFFFFF
    values[6] = 0xFFFFFFFF
    payload[eocd_offset : eocd_offset + verifier.EOCD.size] = verifier.EOCD.pack(*values)
    path.write_bytes(payload)

    central_offset = payload.index(b"PK\x01\x02")
    filename_size, extra_size = struct.unpack_from("<HH", payload, central_offset + 28)
    extra_offset = central_offset + 46 + filename_size
    return bytes(payload[extra_offset : extra_offset + extra_size])


def _first_payload_offset(path: Path) -> tuple[int, int]:
    payload = path.read_bytes()
    with zipfile.ZipFile(path, "r") as archive:
        info = archive.infolist()[0]
    filename_size, extra_size = struct.unpack_from("<HH", payload, info.header_offset + 26)
    return info.header_offset + 30 + filename_size + extra_size, info.compress_size


def test_valid_archive_passes_and_reports_declared_sizes(tmp_path: Path) -> None:
    archive = tmp_path / "valid.zip"
    _write_zip(
        archive,
        [("dataset/", b""), ("dataset/a.txt", b"alpha"), ("dataset/b.bin", b"beta")],
    )

    report = _verify(archive, expected_sha256=_digest(archive))

    assert report["valid"] is True
    assert report["status"] == "PASS"
    assert report["actual_sha256"] == _digest(archive)
    assert report["read_only"] is True
    assert report["extracted"] is False
    assert report["payload_crc"]["status"] == "NOT_REQUESTED"
    assert report["zip"]["member_count"] == 3
    assert report["zip"]["file_count"] == 2
    assert report["zip"]["directory_count"] == 1
    assert report["zip"]["declared_uncompressed_bytes"] == 9
    assert report["zip"]["top_level_entries"] == {"dataset": 3}


def test_payload_crc_streaming_passes_without_extracting(tmp_path: Path) -> None:
    archive = tmp_path / "crc-pass.zip"
    _write_zip(archive, [("safe/payload.bin", b"payload" * 1000)])
    before = set(tmp_path.iterdir())

    report = _verify(archive, verify_payload_crc=True)

    assert report["valid"] is True
    assert report["payload_crc"] == {
        "requested": True,
        "status": "PASS",
        "verified_members": 1,
        "verified_uncompressed_bytes": 7000,
    }
    assert set(tmp_path.iterdir()) == before


@pytest.mark.parametrize(
    "compression",
    [zipfile.ZIP_DEFLATED, zipfile.ZIP_BZIP2, zipfile.ZIP_LZMA],
)
def test_compressed_empty_directory_is_valid_and_its_local_header_is_checked(
    tmp_path: Path, compression: int
) -> None:
    archive = tmp_path / f"compressed-directory-{compression}.zip"
    _write_zip(archive, [("dataset/", b"")], compression=compression)

    report = _verify(archive, verify_payload_crc=True)

    assert report["valid"] is True
    assert report["zip"]["directory_count"] == 1
    assert report["zip"]["declared_compressed_bytes"] > 0
    assert report["payload_crc"] == {
        "requested": True,
        "status": "PASS",
        "verified_members": 1,
        "verified_uncompressed_bytes": 0,
    }


def test_compressed_directory_with_mismatched_local_name_is_rejected(tmp_path: Path) -> None:
    archive = tmp_path / "bad-directory-local-header.zip"
    _write_zip(archive, [("dataset/", b"")], compression=zipfile.ZIP_DEFLATED)
    payload = bytearray(archive.read_bytes())
    payload[30] = ord("D")
    archive.write_bytes(payload)

    report = _verify(archive, verify_payload_crc=True)

    assert report["valid"] is False
    assert report["payload_crc"]["status"] == "FAIL"
    assert any("File name in directory" in error for error in report["errors"])


def test_size_and_digest_mismatches_fail_clearly(tmp_path: Path) -> None:
    archive = tmp_path / "mismatch.zip"
    _write_zip(archive, [("data.txt", b"payload")])

    size_report = verify_zip_archive(archive, expected_size=archive.stat().st_size + 1)
    digest_report = _verify(archive, expected_sha256="0" * 64)

    assert size_report["valid"] is False
    assert any("size mismatch" in error for error in size_report["errors"])
    assert size_report["actual_sha256"] is None
    assert digest_report["valid"] is False
    assert any("SHA-256 mismatch" in error for error in digest_report["errors"])


def test_archive_symlink_and_non_regular_path_are_rejected(tmp_path: Path) -> None:
    archive = tmp_path / "real.zip"
    _write_zip(archive, [("data.txt", b"payload")])
    link = tmp_path / "link.zip"
    link.symlink_to(archive)

    link_report = verify_zip_archive(link, expected_size=archive.stat().st_size)
    directory_report = verify_zip_archive(tmp_path, expected_size=archive.stat().st_size)

    assert link_report["errors"] == ["archive path must not be a symbolic link"]
    assert directory_report["errors"] == ["archive path must be a regular file"]


def test_safe_open_requests_nofollow_when_platform_supports_it(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    archive = tmp_path / "nofollow.zip"
    _write_zip(archive, [("data.txt", b"payload")])
    original_open = verifier.os.open
    observed_flags: list[int] = []

    def recording_open(path: os.PathLike[str] | str, flags: int) -> int:
        observed_flags.append(flags)
        return original_open(path, flags)

    monkeypatch.setattr(verifier.os, "open", recording_open)
    report = _verify(archive)

    assert report["valid"] is True
    assert observed_flags
    if hasattr(os, "O_NOFOLLOW"):
        assert observed_flags[0] & os.O_NOFOLLOW


def test_path_replacement_during_hash_is_rejected_by_inode_check(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    archive = tmp_path / "race.zip"
    replacement = tmp_path / "replacement.zip"
    _write_zip(archive, [("data.txt", b"original")])
    _write_zip(replacement, [("data.txt", b"replacement")])
    original_hash_stream = verifier._hash_stream

    def hash_then_replace(stream: object) -> str:
        digest = original_hash_stream(stream)
        os.replace(replacement, archive)
        return digest

    monkeypatch.setattr(verifier, "_hash_stream", hash_then_replace)
    report = verify_zip_archive(
        archive,
        expected_size=archive.stat().st_size,
    )

    assert report["valid"] is False
    assert any("different inode" in error for error in report["errors"])


@pytest.mark.parametrize(
    ("member_name", "message"),
    [
        ("../escape.txt", "unsafe path component"),
        ("/absolute.txt", "absolute or drive-qualified"),
        ("C:/drive.txt", "absolute or drive-qualified"),
        ("safe\\..\\escape.txt", "contains a backslash"),
        ("safe//empty.txt", "unsafe path component"),
        ("safe/./dot.txt", "unsafe path component"),
        ("safe/control\n.txt", "control character"),
        ("safe/NUL.txt", "reserved Windows device name"),
        ("safe/trailing./file.txt", "ambiguous whitespace or trailing dots"),
        ("safe/name:stream", "alternate-stream separator"),
    ],
)
def test_unsafe_member_paths_are_rejected(
    tmp_path: Path, member_name: str, message: str
) -> None:
    archive = tmp_path / "unsafe.zip"
    _write_zip(archive, [(member_name, b"payload")])

    report = _verify(archive)

    assert report["valid"] is False
    assert any(message in error for error in report["errors"])


def test_exact_normalized_and_case_unicode_duplicates_are_rejected(tmp_path: Path) -> None:
    exact = tmp_path / "exact.zip"
    _write_zip(exact, [("same.txt", b"one"), ("same.txt", b"two")])
    portable = tmp_path / "portable.zip"
    _write_zip(
        portable,
        [("Root/A.txt", b"one"), ("root/a.txt", b"two"), ("café", b"x"), ("café", b"y")],
    )

    exact_report = _verify(exact)
    portable_report = _verify(portable)

    assert any("duplicate member name" in error for error in exact_report["errors"])
    assert any("normalized path collision" in error for error in exact_report["errors"])
    assert sum("portable case/Unicode collision" in error for error in portable_report["errors"]) == 2


@pytest.mark.parametrize(
    "members",
    [
        [("parent", b"file"), ("parent/child", b"child")],
        [("parent/child", b"child"), ("parent", b"file")],
        [("Parent", b"file"), ("parent/child", b"child")],
        [("same/", b""), ("same", b"file")],
    ],
)
def test_file_directory_and_parent_conflicts_are_rejected(
    tmp_path: Path, members: list[tuple[str, bytes]]
) -> None:
    archive = tmp_path / "conflict.zip"
    _write_zip(archive, members)

    report = _verify(archive)

    assert report["valid"] is False
    assert any("parent directory" in error or "conflict" in error for error in report["errors"])


def test_deep_path_parent_conflict_is_detected_without_prefix_registry(
    tmp_path: Path,
) -> None:
    archive = tmp_path / "deep-parent.zip"
    deep_parent = "/".join("d" for _ in range(1_000))
    _write_zip(
        archive,
        [(f"{deep_parent}/child", b"child"), (deep_parent, b"file")],
    )

    report = _verify(archive)

    assert report["valid"] is False
    assert any("parent directory" in error for error in report["errors"])


@pytest.mark.parametrize("file_type", [stat.S_IFLNK, stat.S_IFIFO, stat.S_IFCHR])
def test_symlink_and_special_unix_modes_are_rejected(tmp_path: Path, file_type: int) -> None:
    archive = tmp_path / "mode.zip"
    info = zipfile.ZipInfo("unsafe-entry")
    info.create_system = 3
    info.external_attr = (file_type | 0o777) << 16
    _write_zip(archive, [(info, b"target")])

    report = _verify(archive)

    assert report["valid"] is False
    if file_type == stat.S_IFLNK:
        assert any("symbolic link" in error for error in report["errors"])
    else:
        assert any("forbidden special-file mode" in error for error in report["errors"])


def test_encryption_and_unsupported_compression_are_rejected(tmp_path: Path) -> None:
    encrypted = tmp_path / "encrypted.zip"
    _write_zip(encrypted, [("data.txt", b"payload")])
    _patch_first_central_field(encrypted, 8, 1)
    unsupported = tmp_path / "unsupported.zip"
    _write_zip(unsupported, [("data.txt", b"payload")])
    _patch_first_central_field(unsupported, 10, 99)

    encrypted_report = _verify(encrypted)
    unsupported_report = _verify(unsupported)

    assert any("is encrypted" in error for error in encrypted_report["errors"])
    assert any("unsupported compression method 99" in error for error in unsupported_report["errors"])


def test_member_count_central_size_declared_size_and_ratio_limits(tmp_path: Path) -> None:
    archive = tmp_path / "limits.zip"
    _write_zip(
        archive,
        [("a.txt", b"A" * 20_000), ("b.txt", b"B")],
        compression=zipfile.ZIP_DEFLATED,
    )

    member_report = _verify(archive, limits=AuditLimits(max_members=1))
    central_report = _verify(
        archive, limits=AuditLimits(max_central_directory_bytes=1)
    )
    declared_report = _verify(
        archive, limits=AuditLimits(max_total_uncompressed_bytes=10_000)
    )
    ratio_report = _verify(
        archive,
        limits=AuditLimits(
            max_total_compression_ratio=2.0,
            max_member_compression_ratio=2.0,
        ),
    )

    assert any("declared member count" in error for error in member_report["errors"])
    assert any("central-directory size" in error for error in central_report["errors"])
    assert any("total declared size" in error for error in declared_report["errors"])
    assert any("compression ratio" in error for error in ratio_report["errors"])


def test_corrupt_payload_only_fails_when_crc_streaming_is_requested(tmp_path: Path) -> None:
    archive = tmp_path / "corrupt.zip"
    original = b"unique-payload-for-crc"
    _write_zip(archive, [("payload.bin", original)])
    payload = bytearray(archive.read_bytes())
    payload_offset = payload.index(original)
    payload[payload_offset] ^= 0x01
    archive.write_bytes(payload)

    central_only = _verify(archive)
    crc_report = _verify(archive, verify_payload_crc=True)

    assert central_only["valid"] is True
    assert crc_report["valid"] is False
    assert crc_report["payload_crc"]["status"] == "FAIL"
    assert any("Bad CRC-32" in error for error in crc_report["errors"])


@pytest.mark.parametrize(
    ("compression", "corrupt_offset"),
    [
        (zipfile.ZIP_DEFLATED, 0),
        (zipfile.ZIP_BZIP2, 0),
        (zipfile.ZIP_LZMA, 2),
    ],
)
def test_decompressor_errors_are_returned_as_json_failure(
    tmp_path: Path, compression: int, corrupt_offset: int
) -> None:
    archive = tmp_path / f"corrupt-compressor-{compression}.zip"
    original = b"".join(hashlib.sha256(str(index).encode()).digest() for index in range(512))
    _write_zip(archive, [("payload.bin", original)], compression=compression)
    payload_offset, compressed_size = _first_payload_offset(archive)
    assert corrupt_offset < compressed_size
    payload = bytearray(archive.read_bytes())
    payload[payload_offset + corrupt_offset] ^= 0xFF
    archive.write_bytes(payload)

    report = _verify(archive, verify_payload_crc=True)

    assert report["valid"] is False
    assert report["payload_crc"]["status"] == "FAIL"
    assert any("payload CRC/local-header verification failed" in error for error in report["errors"])
    json.dumps(report, allow_nan=False)


def test_multidisk_end_record_is_rejected_before_member_parsing(tmp_path: Path) -> None:
    archive = tmp_path / "multidisk.zip"
    _write_zip(archive, [("payload.bin", b"payload")])
    payload = bytearray(archive.read_bytes())
    eocd_offset = payload.rfind(verifier.EOCD_SIGNATURE)
    struct.pack_into("<H", payload, eocd_offset + 4, 1)
    archive.write_bytes(payload)

    report = _verify(archive)

    assert report["valid"] is False
    assert any("multi-disk ZIP archives are forbidden" in error for error in report["errors"])


def test_zip64_end_record_and_member_extra_are_supported(tmp_path: Path) -> None:
    archive = tmp_path / "zip64.zip"
    member_extra = _write_small_zip64_with_member_extra(archive)

    report = _verify(archive, verify_payload_crc=True)

    assert member_extra.startswith(b"\x01\x00")
    assert report["valid"] is True
    assert report["zip"]["zip64"] is True
    assert report["payload_crc"]["status"] == "PASS"


def test_fake_eocd_in_comment_is_rejected_before_zipfile_parser(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    archive = tmp_path / "fake-eocd-comment.zip"
    _write_zip(archive, [("payload.bin", b"payload")])
    fake_eocd = verifier.EOCD.pack(
        verifier.EOCD_SIGNATURE,
        0,
        0,
        1,
        1,
        0xFFFFFFFF,
        0,
        0,
    )
    with zipfile.ZipFile(archive, "a") as writer:
        writer.comment = b"metadata:" + fake_eocd + b"!"

    original_zipfile = verifier.zipfile.ZipFile
    calls = 0

    def recording_zipfile(*args: object, **kwargs: object) -> zipfile.ZipFile:
        nonlocal calls
        calls += 1
        return original_zipfile(*args, **kwargs)

    monkeypatch.setattr(verifier.zipfile, "ZipFile", recording_zipfile)
    report = _verify(archive)

    assert calls == 0
    assert report["valid"] is False
    assert any("inconsistent comment length" in error for error in report["errors"])


def test_default_limits_leave_headroom_for_nuplan_mini_camera_archive() -> None:
    limits = AuditLimits()

    assert limits.max_members == 500_000
    assert limits.max_central_directory_bytes == 128 * 1024**2
    assert limits.max_members > 242_385
    assert limits.max_central_directory_bytes > 41_944_810


def test_malformed_archive_and_invalid_contract_values_fail_as_json(tmp_path: Path) -> None:
    malformed = tmp_path / "malformed.zip"
    malformed.write_bytes(b"not a zip archive but long enough")

    malformed_report = _verify(malformed)
    sha_report = verify_zip_archive(
        malformed,
        expected_size=malformed.stat().st_size,
        expected_sha256="invalid",
    )
    limit_report = verify_zip_archive(
        malformed,
        expected_size=malformed.stat().st_size,
        limits=AuditLimits(max_members=0),
    )

    assert any("end-of-central-directory" in error for error in malformed_report["errors"])
    assert sha_report["errors"] == [
        "expected_sha256 must contain exactly 64 hexadecimal characters"
    ]
    assert limit_report["errors"] == ["max_members must be a positive integer"]


def test_cli_always_emits_json_and_uses_zero_or_two_exit_status(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    archive = tmp_path / "cli.zip"
    _write_zip(archive, [("payload.bin", b"payload")])

    passed = verifier.main(
        [
            str(archive),
            "--expected-size",
            str(archive.stat().st_size),
            "--expected-sha256",
            _digest(archive),
            "--verify-payload-crc",
        ]
    )
    pass_report = json.loads(capsys.readouterr().out)
    failed = verifier.main(
        [str(archive), "--expected-size", str(archive.stat().st_size + 1)]
    )
    fail_report = json.loads(capsys.readouterr().out)

    assert passed == 0
    assert pass_report["status"] == "PASS"
    assert pass_report["payload_crc"]["status"] == "PASS"
    assert failed == 2
    assert fail_report["status"] == "FAIL"
    assert fail_report["error_count"] == 1


def test_verification_does_not_change_archive_bytes_or_metadata(tmp_path: Path) -> None:
    archive = tmp_path / "immutable.zip"
    _write_zip(archive, [("payload.bin", b"payload" * 100)])
    before_bytes = archive.read_bytes()
    before = os.stat(archive)

    report = _verify(archive, verify_payload_crc=True)
    after = os.stat(archive)

    assert report["valid"] is True
    assert archive.read_bytes() == before_bytes
    assert (after.st_size, after.st_mtime_ns, after.st_ctime_ns) == (
        before.st_size,
        before.st_mtime_ns,
        before.st_ctime_ns,
    )
