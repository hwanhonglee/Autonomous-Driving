"""HH_261002 - Synthetic backup/cleanup fixtures never access the four real temporary trees or SSH."""
import hashlib
import importlib.util
import io
import json
import os
from pathlib import Path
import shlex
import shutil
import stat
import sys
from types import SimpleNamespace

import pytest

FILE = Path(__file__).resolve().parents[2] / "tools/e2e/recoverable_tmp_carla_contract_cleanup_20261002.py"
spec = importlib.util.spec_from_file_location("tmp_cleanup_under_test", FILE)
tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tool)


@pytest.fixture
def trees(tmp_path, monkeypatch):
    roots = []
    for index in range(4):
        root = tmp_path / ("snapshot_%d" % index)
        root.mkdir(mode=0o700)
        (root / "universe/.git").mkdir(parents=True)
        (root / "universe/.git/config").write_text("original local Git source " + str(index))
        (root / "universe/dirty.py").write_text("unique dirty change " + str(index))
        (root / "universe/untracked.py").write_text("untracked user code")
        (root / "empty").mkdir()
        (root / "internal-link").symlink_to("universe/dirty.py")
        roots.append(str(root))
    monkeypatch.setattr(tool, "TARGETS", tuple(roots))
    monkeypatch.setattr(tool, "TARGET_INODES", {p: Path(p).stat().st_ino for p in roots})
    monkeypatch.setattr(tool, "SOURCE_UID", os.getuid())
    return roots


def test_full_git_dirty_untracked_empty_and_link_manifest(trees):
    result = tool.census(trees, local=True)
    entries = result["entries"]
    assert len([row for row in entries.values() if row["kind"] == "file"]) == 12
    for root in trees:
        prefix = Path(root).name
        assert entries[prefix + "/universe/.git/config"]["sha256"] == tool.digest(Path(root) / "universe/.git/config")
        assert entries[prefix + "/internal-link"]["target"] == "universe/dirty.py"
        assert entries[prefix + "/empty"]["kind"] == "directory"
        assert len(result["identities"][prefix]) == 9
    assert tool.census(trees, local=True) == result


def test_cross_tree_hardlink_topology_is_preserved_in_payload(trees):
    os.link(Path(trees[0]) / "universe/dirty.py", Path(trees[1]) / "shared.py")
    result = tool.census(trees, local=True)
    assert result["entries"]["snapshot_1/shared.py"]["hardlink_first"] == "snapshot_0/universe/dirty.py"


@pytest.mark.parametrize("mutation", ["mode", "inode", "owner", "absolute_link", "escaping_link", "special"])
def test_wrong_generation_and_unsupported_entries_fail_closed(trees, monkeypatch, mutation):
    root = Path(trees[0])
    if mutation == "mode": root.chmod(0o755)
    elif mutation == "inode": monkeypatch.setitem(tool.TARGET_INODES, str(root), root.stat().st_ino + 1)
    elif mutation == "owner": monkeypatch.setattr(tool, "SOURCE_UID", os.getuid() + 1)
    elif mutation == "absolute_link": (root / "bad").symlink_to(root / "universe/dirty.py")
    elif mutation == "escaping_link": (root / "bad").symlink_to("../snapshot_1/universe/dirty.py")
    else: os.mkfifo(root / "pipe")
    with pytest.raises(ValueError): tool.census(trees, local=True)


def test_directory_link_is_recorded_but_not_walked(trees):
    (Path(trees[0]) / "directory-link").symlink_to("universe", target_is_directory=True)
    rows = tool.census(trees, local=True)["entries"]
    assert rows["snapshot_0/directory-link"]["kind"] == "symlink"
    assert not any(name.startswith("snapshot_0/directory-link/") for name in rows)


def test_ancestor_symlinks_and_declared_references_fail_closed(trees, tmp_path, monkeypatch):
    link = tmp_path / "alias"
    link.symlink_to(trees[0], target_is_directory=True)
    with pytest.raises(ValueError): tool.canonical(link / "universe/dirty.py")
    monkeypatch.setattr(tool, "ROOT", tmp_path)
    ref = tmp_path / "plan.json"
    ref.write_text(json.dumps({"source": trees[0]}))
    with pytest.raises(ValueError): tool.references([ref])
    ref.write_text("no temporary dependency")
    assert tool.references([ref]) == {str(ref): tool.digest(ref)}


def make_proc(tmp_path):
    proc = tmp_path / "proc"
    folder = proc / "12"
    (folder / "fd").mkdir(parents=True)
    (folder / "stat").write_text("12 (name with spaces) " + " ".join(["S"] + ["0"] * 18 + ["12345"] + ["0"] * 4))
    for field in ("cwd", "root", "exe"): (folder / field).symlink_to("/not-a-target")
    for field in ("cmdline", "maps"): (folder / field).write_bytes(b"not referenced")
    return proc, folder


def test_actual_fd_reference_and_permission_denominators(tmp_path, trees, monkeypatch):
    proc, folder = make_proc(tmp_path)
    (folder / "fd/9").symlink_to(Path(trees[0]) / "universe/dirty.py")
    assert tool.process_usage(proc)["hits"][0]["field"] == "fd"
    (folder / "fd/9").unlink()
    original = Path.iterdir
    def denied(path):
        if path == folder / "fd": raise PermissionError("synthetic /proc denial")
        return original(path)
    monkeypatch.setattr(Path, "iterdir", denied)
    report = tool.process_usage(proc)
    assert report["all_uid_inspection_complete"] is False
    assert report["same_uid_inspection_complete"] is False
    assert report["inaccessible"][0]["field"] == "fd"


def test_new_pid_cannot_be_called_complete(tmp_path, monkeypatch):
    proc, folder = make_proc(tmp_path)
    original = Path.iterdir
    calls = 0
    def changing(path):
        nonlocal calls
        if path == proc:
            calls += 1
            return iter([folder] + ([proc / "99"] if calls > 1 else []))
        return original(path)
    monkeypatch.setattr(Path, "iterdir", changing)
    result = tool.process_usage(proc)
    assert result["all_uid_inspection_complete"] is False
    assert result["inaccessible"][-1]["field"] == "new_process_after_scan"


def test_incomplete_process_proof_needs_exact_separate_root_acceptance():
    usage = dict(hits=[], inaccessible=[dict(pid=42, uid=0, field="fd")],
                 all_uid_inspection_complete=False, same_uid_inspection_complete=True)
    with pytest.raises(ValueError): tool.removal_policy(dict(process_policy="ALL_UID_COMPLETE"), usage)
    auth = dict(process_policy="ROOT_ACCEPTS_EXACT_DISCLOSED_LIMITATIONS",
                accepted_process_usage_sha256=tool.sha(tool.encoded(usage)), other_uid_absence_not_proven=True,
                same_uid_incomplete_also_disclosed=False)
    tool.removal_policy(auth, usage)
    with pytest.raises(ValueError): tool.removal_policy(dict(auth, accepted_process_usage_sha256="0" * 64), usage)
    with pytest.raises(ValueError): tool.removal_policy(auth, dict(usage, hits=[{"reference": "live"}]))


def test_remote_command_no_install_no_gpu_no_link_follow_and_create_only():
    command = tool.remote_program("create")
    words = shlex.split(command[-1])
    assert "-I" in words and "-B" in words and "CUDA_VISIBLE_DEVICES=" in words
    assert "archive.mkdir(mode=0o700)" in words[-1]
    assert "require(not archive.exists()" in words[-1]
    assert "pip" not in words and "torch" not in words[-1]


def test_real_remote_program_full_create_verify_and_mutation_denial(tmp_path, trees, monkeypatch, capsys):
    personal = tmp_path / "private_remote"
    personal.mkdir(mode=0o700)
    archive = personal / "archives/snapshots"
    monkeypatch.setattr(tool, "REMOTE_PERSONAL", personal)
    monkeypatch.setattr(tool, "REMOTE", archive)
    data = dict(census=tool.census(trees, local=True), authority={"ROOT": "backup only"},
                source_sha256={"helper": hashlib.sha256(b"frozen source").hexdigest()},
                source_archive={"helper": b"frozen source".hex()}, references={"current-plan": "a" * 64})
    def execute(phase, payload):
        monkeypatch.setattr(sys, "stdin", SimpleNamespace(buffer=io.BytesIO(tool.encoded(payload))))
        exec(shlex.split(tool.remote_program(phase)[-1])[-1], {})
    execute("create", data)
    assert json.loads(capsys.readouterr().out)["created"] is True
    assert stat.S_IMODE(archive.stat().st_mode) == 0o700
    with pytest.raises(ValueError, match="create-only"): execute("create", data)
    for root in trees:
        shutil.copytree(root, archive / Path(root).name, symlinks=True)
    data["names"] = [Path(path).name for path in trees]
    execute("verify", data)
    assert json.loads(capsys.readouterr().out)["entries"] == data["census"]["entries"]
    (archive / Path(trees[0]).name / "universe/dirty.py").write_text("corrupt backup")
    with pytest.raises(ValueError, match="census differs"): execute("verify", data)


def test_remote_source_archive_cannot_launder_different_helper_bytes(tmp_path, monkeypatch):
    personal = tmp_path / "private_remote"
    personal.mkdir(mode=0o700)
    monkeypatch.setattr(tool, "REMOTE_PERSONAL", personal)
    monkeypatch.setattr(tool, "REMOTE", personal / "archives/snapshots")
    payload = dict(source_sha256={"helper": "0" * 64}, source_archive={"helper": b"wrong source".hex()})
    monkeypatch.setattr(sys, "stdin", SimpleNamespace(buffer=io.BytesIO(tool.encoded(payload))))
    with pytest.raises(ValueError, match="helper bytes differ"):
        exec(shlex.split(tool.remote_program("create")[-1])[-1], {})
    assert not tool.REMOTE.exists()


@pytest.fixture
def prepared(tmp_path, trees, monkeypatch):
    receipt = tmp_path / "receipts"
    receipt.mkdir(mode=0o700)
    monkeypatch.setattr(tool, "RECEIPT", receipt)
    expected = tool.census(trees, local=True)
    refs = {"ROOT-current-plan": "a" * 64}
    monkeypatch.setattr(tool, "references", lambda files: refs)
    usage = dict(hits=[], inaccessible=[], all_uid_inspection_complete=True, same_uid_inspection_complete=True)
    monkeypatch.setattr(tool, "process_usage", lambda: usage)
    inspection = dict(schema=tool.SCHEMA, census=expected, references=refs, source_sha256=tool.sources())
    tool.write_new(receipt / "inspection.json", inspection)
    def auth(mode):
        value = dict(schema=tool.AUTH_SCHEMA, mode=mode, source_sha256=tool.sources(), targets=list(trees),
                     remote_backup=str(tool.REMOTE), inspection_sha256=tool.digest(receipt / "inspection.json"),
                     payload_sha256=expected["payload_sha256"], generation_sha256=expected["generation_sha256"],
                     tree_pins=tool.tree_pins(expected),
                     declared_reference_sha256=refs, current_root_declaration_set_complete=True)
        if mode == "remove":
            value.update(backup_receipt_sha256=tool.digest(receipt / "backup_complete.json"), process_policy="ALL_UID_COMPLETE")
        path = tmp_path / (mode + "_authority.json")
        tool.write_new(path, value)
        return ["--mode", mode, "--expected-tool-sha256", tool.digest(FILE), "--authorization", str(path),
                "--authorization-sha256", tool.digest(path)]
    calls = []
    monkeypatch.setattr(tool, "remote_program", lambda phase: ["FAKE_REMOTE", phase])
    remote = dict(expected, identities={"independent_remote_generation": [1]}, generation_sha256="b" * 64)
    def run(command, **kwargs):
        calls.append(command)
        payload = b""
        if command[0] == "FAKE_REMOTE":
            payload = tool.encoded(dict(created=True) if command[1] == "create" else remote)
        return SimpleNamespace(returncode=0, stdout=payload, stderr=b"")
    monkeypatch.setattr(tool.subprocess, "run", run)
    return receipt, auth, calls


def test_full_backup_stage_never_removes_local_and_has_no_exclusions(prepared, trees):
    receipt, auth, calls = prepared
    assert tool.main(auth("backup")) == 0
    assert all(Path(path).is_dir() for path in trees)
    result = tool.read(receipt / "backup_complete.json")
    assert result["local_removed"] is False and result["deletion_authorized"] is False
    rsync = next(row for row in calls if row[0] == "rsync")
    assert all(path in rsync for path in trees)
    assert "--hard-links" in rsync and "--safe-links" in rsync and "--ignore-existing" in rsync
    assert not any("exclude" in word or "copy-links" in word for word in rsync)


def test_root_authorized_exact_cleanup_retains_remote_receipts(prepared, trees, monkeypatch):
    receipt, auth, calls = prepared
    tool.main(auth("backup"))
    removed = []
    original = shutil.rmtree
    def narrow(path):
        assert str(path) in trees
        removed.append(str(path))
        original(path)
    narrow.avoids_symlink_attacks = True
    monkeypatch.setattr(tool.shutil, "rmtree", narrow)
    assert tool.main(auth("remove")) == 0
    assert removed == trees and not any(Path(p).exists() for p in trees)
    assert len(list(receipt.glob("removed_*.json"))) == 4
    assert tool.read(receipt / "removal_result.json")["remote_backup_retained"] is True


def test_changed_tree_blocks_before_remote_or_deletion(prepared, trees):
    receipt, auth, calls = prepared
    args = auth("backup")
    (Path(trees[2]) / "universe/untracked.py").write_text("new user edit")
    with pytest.raises(ValueError, match="changed since ROOT seal"): tool.main(args)
    assert not calls and all(Path(path).exists() for path in trees)


def test_second_backup_cannot_overwrite_receipts(prepared):
    receipt, auth, calls = prepared
    args = auth("backup")
    tool.main(args)
    previous_calls = len(calls)
    with pytest.raises(FileExistsError): tool.main(args)
    assert len(calls) == previous_calls


def test_partial_removal_failure_preserves_backup_and_completed_receipts(prepared, trees, monkeypatch):
    receipt, auth, calls = prepared
    tool.main(auth("backup"))
    original = shutil.rmtree
    def fail_second(path):
        if str(path) == trees[1]: raise OSError("synthetic removal failure")
        original(path)
    fail_second.avoids_symlink_attacks = True
    monkeypatch.setattr(tool.shutil, "rmtree", fail_second)
    with pytest.raises(OSError, match="synthetic removal failure"): tool.main(auth("remove"))
    assert not Path(trees[0]).exists() and all(Path(p).exists() for p in trees[1:])
    assert (receipt / "removed_01.json").exists() and (receipt / "backup_complete.json").exists()
    assert tool.read(receipt / "remove_failure.json")["partial_remote_backup_retained"] is True
    assert not (receipt / "removal_result.json").exists()


def test_child_mutation_during_first_removal_blocks_second_target(prepared, trees, monkeypatch):
    receipt, auth, calls = prepared
    tool.main(auth("backup"))
    root_before = tool.stamp(Path(trees[1]).stat())
    original = shutil.rmtree
    def mutate_sibling(path):
        assert str(path) == trees[0]
        (Path(trees[1]) / "universe/dirty.py").write_text("unique new user change during prior removal")
        original(path)
    mutate_sibling.avoids_symlink_attacks = True
    monkeypatch.setattr(tool.shutil, "rmtree", mutate_sibling)
    with pytest.raises(ValueError, match="per-target payload changed"): tool.main(auth("remove"))
    assert tool.stamp(Path(trees[1]).stat()) == root_before
    assert (Path(trees[1]) / "universe/dirty.py").read_text() == "unique new user change during prior removal"
    assert all(Path(path).exists() for path in trees[1:])
    assert (receipt / "backup_complete.json").exists() and (receipt / "removed_01.json").exists()
    assert not (receipt / "removal_result.json").exists()


def test_owned_cross_tree_unlink_changes_ctime_not_payload_or_inode(trees):
    source = Path(trees[0]) / "universe/dirty.py"
    first = Path(trees[1]) / "first.py"
    second = Path(trees[1]) / "second.py"
    os.link(source, first); os.link(source, second)
    expected = tool.census(trees, local=True)
    shutil.rmtree(trees[0])
    assert tool.census([trees[1]], local=True)["identities"]["snapshot_1/first.py"][-1] == 2
    tool.verify_single_target(trees[1], expected)
    assert tool.single_tree_payload(expected["entries"], trees[1])["snapshot_1/second.py"]["hardlink_first"] == "snapshot_1/first.py"


def test_same_bytes_replaced_inode_still_blocked(trees):
    expected = tool.census(trees, local=True)
    source = Path(trees[0]) / "universe/dirty.py"
    old, parent_before = source.stat(), source.parent.stat()
    raw = source.read_bytes()
    source.rename(source.with_suffix(".old"))
    source.write_bytes(raw); source.chmod(stat.S_IMODE(old.st_mode))
    os.utime(source, ns=(old.st_atime_ns, old.st_mtime_ns))
    source.with_suffix(".old").unlink()
    os.utime(source.parent, ns=(parent_before.st_atime_ns, parent_before.st_mtime_ns))
    with pytest.raises(ValueError, match="inode/owner/mode/mtime"):
        tool.verify_single_target(trees[0], expected)
