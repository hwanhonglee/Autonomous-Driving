from __future__ import annotations

import gzip
import hashlib
import io
import json
import os
from pathlib import Path
import tarfile

import pytest

from scripts.e2e import verify_tar_archive as verifier
from scripts.e2e.verify_tar_archive import AuditLimits, verify_tar_archive


def _digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write_tar(
    path: Path,
    members: list[tuple[str | tarfile.TarInfo, bytes]],
    *,
    tar_format: int = tarfile.GNU_FORMAT,
) -> None:
    with tarfile.open(path, "w:gz", format=tar_format) as archive:
        for value, payload in members:
            if isinstance(value, tarfile.TarInfo):
                info = value
            else:
                info = tarfile.TarInfo(value)
                info.size = len(payload)
                info.mode = 0o644
            archive.addfile(info, io.BytesIO(payload) if info.size else None)


def _directory(name: str) -> tarfile.TarInfo:
    info = tarfile.TarInfo(name)
    info.type = tarfile.DIRTYPE
    info.mode = 0o755
    info.size = 0
    return info


def _refresh_header_checksum(block: bytearray) -> None:
    block[148:156] = b" " * 8
    checksum = sum(block)
    block[148:156] = f"{checksum:06o}\0 ".encode("ascii")


def _verify(
    path: Path,
    *,
    expected_sha256: str | None = None,
    limits: AuditLimits = AuditLimits(),
) -> dict[str, object]:
    return verify_tar_archive(
        path,
        expected_size=path.stat().st_size,
        expected_sha256=expected_sha256,
        limits=limits,
    )


def test_valid_archive_full_stream_passes_without_extraction(tmp_path: Path) -> None:
    archive = tmp_path / "valid.tar.gz"
    _write_tar(
        archive,
        [
            (_directory("./dataset/"), b""),
            ("./dataset/a.txt", b"alpha"),
            ("./dataset/b.bin", b"beta"),
        ],
    )
    before_entries = set(tmp_path.iterdir())
    before_bytes = archive.read_bytes()
    before_stat = archive.stat()

    report = _verify(archive, expected_sha256=_digest(archive))
    after_stat = archive.stat()

    assert report["status"] == "PASS"
    assert report["valid"] is True
    assert report["gzip_stream_status"] == "PASS"
    assert report["actual_sha256"] == _digest(archive)
    assert report["read_only"] is True
    assert report["extracted"] is False
    assert report["tar"]["member_count"] == 3
    assert report["tar"]["file_count"] == 2
    assert report["tar"]["directory_count"] == 1
    assert report["tar"]["declared_regular_bytes"] == 9
    assert report["tar"]["declared_payload_compression_ratio"] == (
        report["tar"]["compression_ratio"]
    )
    assert report["tar"]["decompressed_stream_compression_ratio"] == (
        report["tar"]["decompressed_stream_bytes"] / archive.stat().st_size
    )
    assert set(tmp_path.iterdir()) == before_entries
    assert archive.read_bytes() == before_bytes
    assert (after_stat.st_size, after_stat.st_mtime_ns, after_stat.st_ctime_ns) == (
        before_stat.st_size,
        before_stat.st_mtime_ns,
        before_stat.st_ctime_ns,
    )


def test_pax_long_utf8_name_is_supported(tmp_path: Path) -> None:
    archive = tmp_path / "pax.tar.gz"
    long_name = "dataset/" + ("긴이름" * 20) + "/payload.txt"
    _write_tar(archive, [(long_name, b"payload")], tar_format=tarfile.PAX_FORMAT)

    report = _verify(
        archive,
        limits=AuditLimits(max_total_compression_ratio=10_000.0),
    )

    assert report["valid"] is True
    assert report["tar"]["metadata_record_count"] >= 1


@pytest.mark.parametrize(
    "pax_headers",
    (
        {"SCHILY.realsize": str(300 * 1024**3)},
        {"SUN.holesdata": "0 1"},
        {"GNU.sparse.realsize": str(300 * 1024**3)},
        {"SCHILY.filetype": "sparse"},
    ),
)
def test_vendor_sparse_pax_cannot_hide_huge_logical_size(
    tmp_path: Path,
    pax_headers: dict[str, str],
) -> None:
    archive = tmp_path / "vendor-sparse.tar.gz"
    info = tarfile.TarInfo("payload.bin")
    info.size = 1
    info.mode = 0o644
    info.pax_headers = pax_headers
    _write_tar(archive, [(info, b"x")], tar_format=tarfile.PAX_FORMAT)

    report = _verify(archive)

    assert report["valid"] is False
    assert any("forbidden sparse" in error for error in report["errors"])


def test_posix_ustar_prefix_is_part_of_the_member_path(tmp_path: Path) -> None:
    archive = tmp_path / "ustar-prefix.tar.gz"
    name = "dataset/" + ("nested" * 16) + "/payload.txt"
    assert len(name.encode("utf-8")) > 100
    _write_tar(archive, [(name, b"payload")], tar_format=tarfile.USTAR_FORMAT)

    report = _verify(archive)

    assert report["valid"] is True
    assert report["tar"]["top_level_entries"] == {"dataset": 1}


def test_old_gnu_extension_bytes_cannot_hide_duplicate_member(tmp_path: Path) -> None:
    archive = tmp_path / "old-gnu-duplicate.tar.gz"
    _write_tar(archive, [("same.txt", b"first"), ("same.txt", b"second")])
    payload = bytearray(gzip.decompress(archive.read_bytes()))
    first_header = bytearray(payload[:512])
    assert first_header[257:263] == b"ustar "
    assert first_header[263:265] == b" \0"

    # In old-GNU layout this range starts with atime, not the POSIX prefix.
    # A verifier that treats it as prefix sees "00000000000/same.txt" and can
    # miss that both logical members actually target the same path.
    first_header[345:357] = b"00000000000\0"
    _refresh_header_checksum(first_header)
    payload[:512] = first_header
    archive.write_bytes(gzip.compress(bytes(payload), mtime=0))

    report = _verify(archive)

    assert report["valid"] is False
    assert any("duplicate member name 'same.txt'" in error for error in report["errors"])


@pytest.mark.parametrize(
    ("magic", "version"),
    (
        (b"ustar\0", b" \0"),
        (b"ustar ", b"00"),
        (b"\0" * 6, b"00"),
    ),
)
def test_header_magic_and_version_must_identify_one_layout(
    tmp_path: Path, magic: bytes, version: bytes
) -> None:
    archive = tmp_path / "mixed-layout.tar.gz"
    _write_tar(archive, [("payload", b"data")])
    payload = bytearray(gzip.decompress(archive.read_bytes()))
    first_header = bytearray(payload[:512])
    first_header[257:263] = magic
    first_header[263:265] = version
    _refresh_header_checksum(first_header)
    payload[:512] = first_header
    archive.write_bytes(gzip.compress(bytes(payload), mtime=0))

    report = _verify(archive)

    assert report["valid"] is False
    assert any("format signature/version" in error for error in report["errors"])


def test_expected_size_and_sha256_mismatches_fail_clearly(tmp_path: Path) -> None:
    archive = tmp_path / "mismatch.tar.gz"
    _write_tar(archive, [("payload", b"data")])

    size_report = verify_tar_archive(
        archive,
        expected_size=archive.stat().st_size + 1,
    )
    sha_report = _verify(archive, expected_sha256="0" * 64)

    assert size_report["valid"] is False
    assert any("size mismatch" in error for error in size_report["errors"])
    assert size_report["actual_sha256"] is None
    assert sha_report["valid"] is False
    assert any("SHA-256 mismatch" in error for error in sha_report["errors"])
    assert sha_report["gzip_stream_status"] == "NOT_RUN"


def test_truncated_gzip_fails_full_stream_verification(tmp_path: Path) -> None:
    archive = tmp_path / "truncated.tar.gz"
    _write_tar(archive, [("payload.bin", b"payload" * 10_000)])
    archive.write_bytes(archive.read_bytes()[:-8])

    report = _verify(
        archive,
        limits=AuditLimits(max_total_compression_ratio=10_000.0),
    )

    assert report["valid"] is False
    assert report["gzip_stream_status"] == "FAIL"
    assert any("ended before" in error or "end-of-stream" in error for error in report["errors"])


def test_corrupt_gzip_crc_is_rejected(tmp_path: Path) -> None:
    archive = tmp_path / "bad-crc.tar.gz"
    _write_tar(archive, [("payload.bin", b"payload")])
    payload = bytearray(archive.read_bytes())
    payload[-8] ^= 0x01
    archive.write_bytes(payload)

    report = _verify(archive)

    assert report["valid"] is False
    assert report["gzip_stream_status"] == "FAIL"
    assert any("CRC check failed" in error for error in report["errors"])


def test_archive_symlink_and_non_regular_path_are_rejected(tmp_path: Path) -> None:
    archive = tmp_path / "real.tar.gz"
    _write_tar(archive, [("payload", b"data")])
    link = tmp_path / "link.tar.gz"
    link.symlink_to(archive)

    link_report = verify_tar_archive(link, expected_size=archive.stat().st_size)
    directory_report = verify_tar_archive(tmp_path, expected_size=archive.stat().st_size)

    assert link_report["errors"] == ["archive path must not be a symbolic link"]
    assert directory_report["errors"] == ["archive path must be a regular file"]


def test_safe_open_requests_nofollow_when_available(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    archive = tmp_path / "nofollow.tar.gz"
    _write_tar(archive, [("payload", b"data")])
    original_open = verifier.os.open
    flags_seen: list[int] = []

    def recording_open(path: os.PathLike[str] | str, flags: int) -> int:
        flags_seen.append(flags)
        return original_open(path, flags)

    monkeypatch.setattr(verifier.os, "open", recording_open)
    report = _verify(archive)

    assert report["valid"] is True
    assert flags_seen
    if hasattr(os, "O_NOFOLLOW"):
        assert flags_seen[0] & os.O_NOFOLLOW


def test_path_replacement_during_hash_is_rejected(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    archive = tmp_path / "race.tar.gz"
    replacement = tmp_path / "replacement.tar.gz"
    _write_tar(archive, [("payload", b"original")])
    _write_tar(replacement, [("payload", b"replacement")])
    expected_size = archive.stat().st_size
    original_hash = verifier._hash_stream

    def hash_then_replace(stream: object) -> str:
        digest = original_hash(stream)
        os.replace(replacement, archive)
        return digest

    monkeypatch.setattr(verifier, "_hash_stream", hash_then_replace)
    report = verify_tar_archive(archive, expected_size=expected_size)

    assert report["valid"] is False
    assert any("different inode" in error for error in report["errors"])


def test_same_inode_change_during_hash_is_rejected(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    archive = tmp_path / "changed.tar.gz"
    _write_tar(archive, [("payload", b"original")])
    expected_size = archive.stat().st_size
    original_hash = verifier._hash_stream

    def hash_then_touch(stream: object) -> str:
        digest = original_hash(stream)
        metadata = archive.stat()
        os.utime(archive, ns=(metadata.st_atime_ns, metadata.st_mtime_ns + 1_000_000))
        return digest

    monkeypatch.setattr(verifier, "_hash_stream", hash_then_touch)
    report = verify_tar_archive(archive, expected_size=expected_size)

    assert report["valid"] is False
    assert any("changed while it was being hashed" in error for error in report["errors"])


@pytest.mark.parametrize(
    ("name", "message"),
    [
        ("../escape", "unsafe path component"),
        ("/absolute", "absolute or drive-qualified"),
        ("C:/drive", "absolute or drive-qualified"),
        ("safe\\escape", "contains a backslash"),
        ("safe\x00evil", "bytes after a NUL terminator"),
        ("safe//empty", "unsafe path component"),
        ("safe/./dot", "unsafe path component"),
        ("safe/control\n", "control character"),
        ("safe/NUL.txt", "reserved Windows device name"),
        ("safe/trailing./file", "ambiguous whitespace or trailing dots"),
        ("safe/name:stream", "Windows stream separator"),
    ],
)
def test_unsafe_paths_are_rejected(tmp_path: Path, name: str, message: str) -> None:
    archive = tmp_path / "unsafe.tar.gz"
    _write_tar(archive, [(name, b"payload")])

    report = _verify(archive)

    assert report["valid"] is False
    assert any(message in error for error in report["errors"])


def test_duplicate_case_unicode_and_file_directory_collisions(tmp_path: Path) -> None:
    archive = tmp_path / "collisions.tar.gz"
    _write_tar(
        archive,
        [
            ("same", b"one"),
            ("same", b"two"),
            ("Root/A", b"a"),
            ("root/a", b"b"),
            ("café", b"x"),
            ("café", b"y"),
            (_directory("both"), b""),
            ("both", b"file"),
            ("parent", b"file"),
            ("parent/child", b"child"),
        ],
    )

    report = _verify(archive)

    assert report["valid"] is False
    assert any("duplicate member name" in error for error in report["errors"])
    assert sum("portable case/Unicode collision" in error for error in report["errors"]) >= 2
    assert any("file/directory conflict" in error for error in report["errors"])
    assert any("parent directory" in error for error in report["errors"])


@pytest.mark.parametrize(
    ("type_flag", "description"),
    [
        (tarfile.SYMTYPE, "symbolic link"),
        (tarfile.LNKTYPE, "hard link"),
        (tarfile.CHRTYPE, "character device"),
        (tarfile.BLKTYPE, "block device"),
        (tarfile.FIFOTYPE, "FIFO"),
    ],
)
def test_links_devices_fifo_and_special_types_are_rejected(
    tmp_path: Path, type_flag: bytes, description: str
) -> None:
    archive = tmp_path / "special.tar.gz"
    info = tarfile.TarInfo("unsafe")
    info.type = type_flag
    info.linkname = "../target" if type_flag in (tarfile.SYMTYPE, tarfile.LNKTYPE) else ""
    info.devmajor = 1
    info.devminor = 3
    info.size = 0
    _write_tar(archive, [(info, b"")])

    report = _verify(archive)

    assert report["valid"] is False
    assert any(description in error for error in report["errors"])


def test_member_total_count_stream_metadata_path_and_ratio_limits(tmp_path: Path) -> None:
    archive = tmp_path / "limits.tar.gz"
    long_name = "dataset/" + ("long" * 30) + "/payload"
    _write_tar(
        archive,
        [(long_name, b"A" * 20_000), ("second", b"B")],
        tar_format=tarfile.PAX_FORMAT,
    )

    member_report = _verify(archive, limits=AuditLimits(max_members=1))
    member_size_report = _verify(
        archive, limits=AuditLimits(max_member_uncompressed_bytes=10_000)
    )
    total_report = _verify(
        archive, limits=AuditLimits(max_total_uncompressed_bytes=10_000)
    )
    stream_report = _verify(archive, limits=AuditLimits(max_tar_stream_bytes=1024))
    metadata_report = _verify(archive, limits=AuditLimits(max_metadata_bytes=10))
    path_report = _verify(archive, limits=AuditLimits(max_path_bytes=20))
    aggregate_path_report = _verify(
        archive,
        limits=AuditLimits(max_aggregate_normalized_path_bytes=20),
    )
    ratio_report = _verify(
        archive, limits=AuditLimits(max_total_compression_ratio=2.0)
    )

    assert any("member limit" in error for error in member_report["errors"])
    assert any("member limit" in error for error in member_size_report["errors"])
    assert any("total declared size" in error for error in total_report["errors"])
    assert any("decompressed TAR stream" in error for error in stream_report["errors"])
    assert any("metadata exceeds" in error for error in metadata_report["errors"])
    assert any("path limit" in error for error in path_report["errors"])
    assert any(
        "aggregate normalized path bytes" in error
        for error in aggregate_path_report["errors"]
    )
    assert any("compression ratio" in error for error in ratio_report["errors"])


def test_trailing_zero_stream_obeys_actual_expansion_ratio(tmp_path: Path) -> None:
    archive = tmp_path / "trailing-zero-expansion.tar.gz"
    uncompressed = io.BytesIO()
    with tarfile.open(fileobj=uncompressed, mode="w", format=tarfile.GNU_FORMAT) as writer:
        info = tarfile.TarInfo("payload.bin")
        info.size = 1
        writer.addfile(info, io.BytesIO(b"x"))
    expanded = uncompressed.getvalue() + b"\0" * (100 * 1024**2)
    archive.write_bytes(gzip.compress(expanded, compresslevel=9, mtime=0))
    assert len(expanded) / archive.stat().st_size > AuditLimits.max_total_compression_ratio

    report = _verify(archive)

    assert report["valid"] is False
    assert any("decompressed TAR stream exceeds limit" in error for error in report["errors"])


def test_default_member_and_path_memory_limits_cover_large_official_set() -> None:
    limits = AuditLimits()

    assert limits.max_members == 500_000
    assert limits.max_members > 242_385
    assert limits.max_aggregate_normalized_path_bytes == 128 * 1024**2


def test_invalid_contract_values_return_json_safe_failure(tmp_path: Path) -> None:
    archive = tmp_path / "valid.tar.gz"
    _write_tar(archive, [("payload", b"data")])

    sha_report = verify_tar_archive(
        archive,
        expected_size=archive.stat().st_size,
        expected_sha256="invalid",
    )
    limit_report = verify_tar_archive(
        archive,
        expected_size=archive.stat().st_size,
        limits=AuditLimits(max_total_compression_ratio=float("nan")),
    )

    assert sha_report["errors"] == [
        "expected_sha256 must contain exactly 64 hexadecimal characters"
    ]
    assert limit_report["valid"] is False
    json.dumps(limit_report, allow_nan=False)


def test_cli_emits_json_and_uses_zero_or_two_exit_status(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    archive = tmp_path / "cli.tar.gz"
    _write_tar(archive, [("payload", b"data")])

    passed = verifier.main(
        [
            str(archive),
            "--expected-size",
            str(archive.stat().st_size),
            "--expected-sha256",
            _digest(archive),
        ]
    )
    pass_report = json.loads(capsys.readouterr().out)
    failed = verifier.main(
        [str(archive), "--expected-size", str(archive.stat().st_size + 1)]
    )
    fail_report = json.loads(capsys.readouterr().out)

    assert passed == 0
    assert pass_report["status"] == "PASS"
    assert failed == 2
    assert fail_report["status"] == "FAIL"


def test_verifier_never_calls_tar_extraction_api(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    archive = tmp_path / "no-extract.tar.gz"
    _write_tar(archive, [("payload", b"data")])

    def forbidden(*args: object, **kwargs: object) -> None:
        raise AssertionError("extraction API must never be called")

    monkeypatch.setattr(tarfile.TarFile, "extract", forbidden)
    monkeypatch.setattr(tarfile.TarFile, "extractall", forbidden)

    report = _verify(archive)

    assert report["valid"] is True
