"""HH_261004 - Test complete synthetic quota accounting and fail-closed metadata faults without live runtime or dataset access."""

import ast
import errno
import json
import os
from pathlib import Path
import socket
from types import SimpleNamespace

import pytest

from scripts.e2e import audit_scandir_episode_quota_parity_v1_20261004 as quota


def make_tree(tmp_path):
    root = tmp_path / "owned_episode"
    root.mkdir()
    for name, data in (("plain", b"abc"), (".hidden", b"12345"), ("CAM_FRONT/a.jpg", b"image" * 31),
                       ("CAM_BACK/nested/b.jpg", b"other" * 49), (".hidden_dir/nested/state", b"journal" * 17)):
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
    return root


def test_stable_readable_owned_nested_hidden_tree_matches_exact_original_bytes(tmp_path):
    root = make_tree(tmp_path)
    receipt = {}
    before = {str(path): quota.digest(path) for path in root.rglob("*") if path.is_file()}
    result = quota.scandir_directory_bytes(root, receipt=receipt)
    assert result == quota.original_directory_bytes()(root) == 527
    assert receipt["regular_files"] == 5 and receipt["complete_walk"] is True
    assert receipt["entries"] == 10 and receipt["directories_including_root"] == 6
    assert receipt["cached_or_incremental_byte_count_used"] is False and receipt["source_payload_read"] is False
    assert before == {str(path): quota.digest(path) for path in root.rglob("*") if path.is_file()}


def test_absent_and_empty_owned_roots_match_original(tmp_path):
    absent = tmp_path / "absent"
    assert quota.scandir_directory_bytes(absent) == quota.original_directory_bytes()(absent) == 0
    absent.mkdir()
    assert quota.scandir_directory_bytes(absent) == quota.original_directory_bytes()(absent) == 0


@pytest.mark.parametrize("kind", ["root_directory", "root_file", "root_dangling", "subdirectory", "file", "dangling_file", "hidden"])
def test_every_symlink_is_denied_and_no_foreign_member_is_counted(tmp_path, kind):
    root = make_tree(tmp_path)
    foreign = tmp_path / "foreign"
    foreign.mkdir()
    (foreign / "user_file").write_bytes(b"untouched")
    if kind.startswith("root_"):
        target = foreign if kind == "root_directory" else foreign / ("user_file" if kind == "root_file" else "missing")
        root = tmp_path / "root_alias"
        root.symlink_to(target)
    else:
        name = ".hidden_link" if kind == "hidden" else "alias"
        target = foreign if kind == "subdirectory" else foreign / ("missing" if kind == "dangling_file" else "user_file")
        (root / name).symlink_to(target)
    with pytest.raises(RuntimeError):
        quota.scandir_directory_bytes(root)
    if kind != "root_dangling":
        with pytest.raises(RuntimeError):
            quota.original_directory_bytes()(root)
    else:
        # HH_261004 - Explicitly disclose stricter fail-closed handling rather than claiming false universal equivalence.
        assert quota.original_directory_bytes()(root) == 0
    assert (foreign / "user_file").read_bytes() == b"untouched"


def test_FIFO_and_unix_socket_are_visited_but_counted_like_original_without_opening_payload(tmp_path):
    root = make_tree(tmp_path)
    os.mkfifo(root / "fifo")
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as channel:
        channel.bind(str(root / "socket"))
        receipt = {}
        assert quota.scandir_directory_bytes(root, receipt=receipt) == quota.original_directory_bytes()(root) == 527
        assert receipt["special_nodes_ignored_like_original"] == 2 and receipt["entries"] == 12


def test_sparse_large_file_bytes_are_counted_not_blocks_or_regular_file_count(tmp_path):
    root = tmp_path / "owned"
    root.mkdir()
    with (root / "sparse").open("wb") as stream:
        stream.truncate(600 * 1024**2 + 7)
    assert quota.scandir_directory_bytes(root) == quota.original_directory_bytes()(root) == 600 * 1024**2 + 7


def test_hardlinked_paths_are_each_counted_like_original_without_inode_deduplication(tmp_path):
    root = make_tree(tmp_path)
    os.link(root / "plain", root / "plain_again")
    assert quota.scandir_directory_bytes(root) == quota.original_directory_bytes()(root) == 530


def test_deep_tree_and_empty_hidden_directories_are_not_omitted(tmp_path):
    root = tmp_path / "owned"
    root.mkdir()
    leaf = root
    for index in range(40):
        leaf = leaf / (".hidden" if index % 2 else "nested")
        leaf.mkdir()
    (leaf / "deep").write_bytes(b"all40levels")
    (root / ".empty").mkdir()
    receipt = {}
    assert quota.scandir_directory_bytes(root, receipt=receipt) == quota.original_directory_bytes()(root) == 11
    assert receipt["directories_including_root"] == 42 and receipt["regular_files"] == 1


def test_every_call_rewalks_new_deleted_and_resized_members_without_cached_counts(tmp_path):
    root = make_tree(tmp_path)
    reference = quota.original_directory_bytes()
    assert quota.scandir_directory_bytes(root) == reference(root)
    (root / "new").write_bytes(b"new" * 71)
    assert quota.scandir_directory_bytes(root) == reference(root) == 740
    (root / "plain").unlink()
    (root / ".hidden").write_bytes(b"resized")
    assert quota.scandir_directory_bytes(root) == reference(root) == 739


@pytest.mark.parametrize("failure", [PermissionError(errno.EACCES, "denied"), OSError(errno.EIO, "I/O fault")])
def test_unreadable_or_IO_fault_never_returns_a_partial_accepted_count(tmp_path, monkeypatch, failure):
    root = make_tree(tmp_path)
    opened = []
    original_open, original_scan = quota.os.open, quota.os.scandir
    def observed_open(*args, **kwargs):
        fd = original_open(*args, **kwargs)
        opened.append(fd)
        return fd
    def denied_scan(fd):
        if len(opened) > 1:
            raise failure
        return original_scan(fd)
    monkeypatch.setattr(quota.os, "open", observed_open)
    monkeypatch.setattr(quota.os, "scandir", denied_scan)
    with pytest.raises(type(failure)):
        quota.scandir_directory_bytes(root)
    for fd in opened:
        with pytest.raises(OSError): os.fstat(fd)


def test_directory_generation_replacement_is_denied_and_all_descriptors_close(tmp_path, monkeypatch):
    root = make_tree(tmp_path)
    original = quota.os.fstat
    def changed(fd):
        current = original(fd)
        return SimpleNamespace(st_mode=current.st_mode, st_dev=current.st_dev, st_ino=current.st_ino + 1)
    monkeypatch.setattr(quota.os, "fstat", changed)
    with pytest.raises(RuntimeError, match="identity changed"):
        quota.scandir_directory_bytes(root)


@pytest.mark.parametrize("failure", [FileNotFoundError(errno.ENOENT, "removed during traversal"),
    PermissionError(errno.EACCES, "metadata unreadable"), OSError(errno.EIO, "metadata I/O fault")])
def test_per_entry_metadata_fault_is_not_silently_skipped(tmp_path, monkeypatch, failure):
    root = make_tree(tmp_path)
    original = quota.os.scandir
    class Entry:
        def __init__(self, actual): self.actual, self.name = actual, actual.name
        def is_symlink(self): return self.actual.is_symlink()
        def is_dir(self, **kwargs): return self.actual.is_dir(**kwargs)
        def is_file(self, **kwargs): return self.actual.is_file(**kwargs)
        def stat(self, **kwargs):
            if self.name == "plain": raise failure
            return self.actual.stat(**kwargs)
    class Scan:
        def __init__(self, fd): self.actual = original(fd)
        def __next__(self): return Entry(next(self.actual))
        def close(self): self.actual.close()
    monkeypatch.setattr(quota.os, "scandir", Scan)
    with pytest.raises(type(failure)):
        quota.scandir_directory_bytes(root)


def test_regular_file_replaced_by_symlink_after_classification_is_denied(tmp_path, monkeypatch):
    root = make_tree(tmp_path)
    original = quota.os.scandir
    class Entry:
        def __init__(self, actual): self.actual, self.name = actual, actual.name
        def is_symlink(self): return self.actual.is_symlink()
        def is_dir(self, **kwargs): return self.actual.is_dir(**kwargs)
        def is_file(self, **kwargs): return self.actual.is_file(**kwargs)
        def stat(self, **kwargs):
            current = self.actual.stat(**kwargs)
            return SimpleNamespace(st_mode=0o120777) if self.name == "plain" else current
    class Scan:
        def __init__(self, fd): self.actual = original(fd)
        def __next__(self): return Entry(next(self.actual))
        def close(self): self.actual.close()
    monkeypatch.setattr(quota.os, "scandir", Scan)
    with pytest.raises(RuntimeError, match="regular-file type changed"):
        quota.scandir_directory_bytes(root)


def test_symlink_replacement_after_type_check_is_denied_by_NOFOLLOW(tmp_path, monkeypatch):
    root = make_tree(tmp_path)
    foreign = tmp_path / "foreign"
    foreign.mkdir()
    original_open = quota.os.open
    def replaced(name, flags, **kwargs):
        if name == "CAM_FRONT":
            actual = root / "CAM_FRONT"
            actual.rename(root / "CAM_FRONT_retained")
            actual.symlink_to(foreign)
        return original_open(name, flags, **kwargs)
    monkeypatch.setattr(quota.os, "open", replaced)
    with pytest.raises(OSError):
        quota.scandir_directory_bytes(root)


def test_no_owner_or_model_import_and_no_payload_read_or_incremental_cache_in_candidate():
    tree = ast.parse(Path(quota.__file__).read_text())
    imports = [ast.unparse(node) for node in ast.walk(tree) if isinstance(node, (ast.Import, ast.ImportFrom))]
    assert not any("torch" in item or "carla" in item or "portable_e2e" in item for item in imports)
    function = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "scandir_directory_bytes")
    calls = [ast.unparse(node.func) for node in ast.walk(function) if isinstance(node, ast.Call)]
    assert "os.scandir" in ast.unparse(function)
    assert not any(name.endswith("read_bytes") or name.endswith("read_text") or name.endswith("read") for name in calls)
    assert not any(isinstance(node, (ast.Global, ast.Nonlocal)) for node in ast.walk(function))


def test_prospective_contract_retains_every_bundle_quota_floor_reserves_and_new_actual_timing_requirement():
    plan = quota.prospective_contract()
    assert plan["incremental_byte_count_or_membership_cache"] is False
    assert plan["original_quota_or_free_floor_removed"] is False
    assert plan["original_12MiB_next_bundle_and_closing_reserve_checks_unchanged"] is True
    assert plan["original_per_bundle_free_disk_check_unchanged"] is True
    assert plan["real_tree_counterfactual"]["original_exact_per_bundle_full_directory_membership_not_recorded"] is True
    assert plan["later_live_intervention"]["new_per_quota_walk_and_disk_check_monotonic_boundaries_required"] is True
    assert plan["actual_runtime_edited_or_activated"] is False and plan["formal_function_approval"] is False


def test_prospective_CLI_has_no_real_tree_count_or_timing_execution(monkeypatch, capsys):
    monkeypatch.setattr(quota, "scandir_directory_bytes", lambda *args, **kwargs: pytest.fail("unapproved real tree traversal"))
    quota.main(["--prospective-only"])
    assert json.loads(capsys.readouterr().out)["actual_runtime_edited_or_activated"] is False
    with pytest.raises(SystemExit): quota.main([])
