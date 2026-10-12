"""HH_261005 - Compare exact every-entry census and reject symlinks, tampering and unsafe build targets."""

import os
from pathlib import Path
import socket

import pytest

from scripts.e2e import full_tree_quota_native_v1_20261005 as native
from scripts.e2e import audit_scandir_episode_quota_parity_v1_20261004 as original


@pytest.fixture(scope="module")
def library(tmp_path_factory):
    path = tmp_path_factory.mktemp("HH_native_quota") / "libquota.so"
    build = native.compile_new(path, expected_source_sha256=native.digest(native.SOURCE))
    return path, build


@pytest.fixture
def census(library):
    path, build = library
    return native.NativeFullCensus(path, expected_source_sha256=build["source_sha256"],
                                  expected_library_sha256=build["library_sha256"])


@pytest.mark.parametrize("count", [0, 1, 7, 41, 511])
def test_every_nested_hidden_file_and_exact_counts(census, tmp_path, count):
    # HH_261005 - Fixture writes are synthetic; no actual raw capture or frozen output is modified.
    root = tmp_path / "episode"
    root.mkdir()
    (root / ".nested").mkdir()
    for index in range(count):
        (root / ".nested" / str(index)).write_bytes(b"x" * (index % 13))
    os.mkfifo(root / "special")
    a, b = {}, {}
    assert census.directory_bytes(root, receipt=a) == original.scandir_directory_bytes(root, receipt=b)
    for key in ("entries", "directories_including_root", "regular_files", "regular_file_bytes", "special_nodes_ignored_like_original"):
        assert a[key] == b[key]
    assert a["complete_walk"] and not a["cached_or_incremental_byte_count_used"]


def test_new_membership_and_size_changes_are_counted(census, tmp_path):
    root = tmp_path / "episode"
    root.mkdir()
    assert census.directory_bytes(root) == 0
    (root / "new").write_bytes(b"four")
    assert census.directory_bytes(root) == 4
    (root / "new").write_bytes(b"123456")
    assert census.directory_bytes(root) == 6


@pytest.mark.parametrize("kind", ["dangling", "file", "dir", "root"])
def test_symlinks_rejected(census, tmp_path, kind):
    root = tmp_path / "episode"
    root.mkdir()
    outside = tmp_path / "outside"
    outside.mkdir()
    (outside / "file").write_bytes(b"data")
    if kind == "root":
        root = tmp_path / "alias"
        root.symlink_to(outside, target_is_directory=True)
    else:
        target = outside / ("missing" if kind == "dangling" else "file") if kind != "dir" else outside
        (root / "alias").symlink_to(target)
    with pytest.raises((RuntimeError, OSError)):
        census.directory_bytes(root)


def test_missing_root_original_zero(census, tmp_path):
    assert census.directory_bytes(tmp_path / "missing") == 0


def test_bad_library_seal_rejected(library):
    path, build = library
    with pytest.raises(RuntimeError, match="seal"):
        native.NativeFullCensus(path, expected_source_sha256=build["source_sha256"], expected_library_sha256="0" * 64)


def test_existing_compile_target_rejected(library):
    path, build = library
    with pytest.raises(RuntimeError, match="fresh"):
        native.compile_new(path, expected_source_sha256=build["source_sha256"])


def test_raced_compile_target_never_overwritten(tmp_path, monkeypatch):
    target = tmp_path / "raced.so"
    run = native.subprocess.run
    def raced(command, **kwargs):
        result = run(command, **kwargs)
        target.write_bytes(b"preserve raced target")
        return result
    monkeypatch.setattr(native.subprocess, "run", raced)
    with pytest.raises(FileExistsError):
        native.compile_new(target, expected_source_sha256=native.digest(native.SOURCE))
    assert target.read_bytes() == b"preserve raced target"


def test_hardlinks_and_socket_original_semantics(census, tmp_path):
    root = tmp_path / "episode"
    root.mkdir()
    (root / "file").write_bytes(b"data")
    os.link(root / "file", root / "hardlink")
    with socket.socket(socket.AF_UNIX) as sock:
        sock.bind(str(root / "socket"))
        a, b = {}, {}
        assert census.directory_bytes(root, receipt=a) == original.scandir_directory_bytes(root, receipt=b) == 8
        assert a["regular_files"] == 2 and a["special_nodes_ignored_like_original"] == 1


def test_unreadable_nested_directory_fails(census, tmp_path):
    root = tmp_path / "episode"
    root.mkdir()
    private = root / "unreadable"
    private.mkdir()
    private.chmod(0)
    try:
        with pytest.raises(PermissionError):
            census.directory_bytes(root)
    finally:
        private.chmod(0o700)


def test_no_descriptor_leak_for_success_and_failure(census, tmp_path):
    root = tmp_path / "episode"
    root.mkdir()
    initial = len(list(Path("/proc/self/fd").iterdir()))
    for _ in range(30):
        assert census.directory_bytes(root) == 0
    (root / "dangling").symlink_to(tmp_path / "missing")
    for _ in range(30):
        with pytest.raises(OSError):
            census.directory_bytes(root)
    assert len(list(Path("/proc/self/fd").iterdir())) == initial


def test_root_substitution_after_call_rejected(census, tmp_path, monkeypatch):
    root = tmp_path / "episode"
    root.mkdir()
    call = census._call
    def substituted(path, counts):
        status = call(path, counts)
        root.rename(tmp_path / "old")
        root.mkdir()
        return status
    monkeypatch.setattr(census, "_call", substituted)
    with pytest.raises(RuntimeError, match="generation"):
        census.directory_bytes(root)


def test_recursion_limit_fails_closed_and_closes_descriptors(census, tmp_path):
    root = tmp_path / "episode"
    root.mkdir()
    descendant = root
    for _ in range(514):
        descendant = descendant / "d"
        descendant.mkdir()
    initial = len(list(Path("/proc/self/fd").iterdir()))
    with pytest.raises(OSError):
        census.directory_bytes(root)
    assert len(list(Path("/proc/self/fd").iterdir())) == initial
