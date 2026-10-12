"""HH_261002 - Synthetic-only V4 process-proof and full verified-backup removal tests; no real SSH or deletion."""
import copy
import hashlib
import json
import os
from pathlib import Path
import shutil
from types import SimpleNamespace

import pytest
from scripts.e2e import remove_verified_tmp_source_snapshots_v4_20261002 as tool

FILE = Path(tool.__file__)


def usage(users=None, kernels=None, hits=None):
    rows = [dict(pid=42, uid=os.getuid(), start="123", field="fd", kernel_thread_proven=False)] if users is None else users
    kernel_rows = [dict(pid=90, uid=0, start="200", field="cwd", kernel_thread_proven=True)] if kernels is None else kernels
    return dict(hits=[] if hits is None else hits, inaccessible=rows + kernel_rows,
        process_metadata=[], PID_generation_validation_complete=True, generation_errors=[],
        all_uid_inspection_complete=not (rows + kernel_rows),
        same_uid_inspection_complete=not any(r["uid"] in (None, os.getuid()) for r in rows + kernel_rows),
        other_UID_absence_proven=False)


def policy_authority(before):
    expected = tool.userspace_limitations(before)
    return dict(process_policy=tool.POLICY, proven_UID0_Kthread1_PID_churn_accepted=True,
        accepted_userspace_limitations=expected, accepted_userspace_limitations_sha256=tool.sha(tool.encoded(expected)),
        other_uid_absence_not_proven=True, same_uid_incomplete_also_disclosed=not before["same_uid_inspection_complete"])


def test_only_kernel_churn_allowed_all_actual_limitations_retained():
    before = usage(); current = usage(kernels=[dict(pid=91, uid=0, start="201", field="fd", kernel_thread_proven=True)])
    tool.removal_policy(policy_authority(before), current, dict(process_usage=before))
    assert tool.userspace_limitations(before) == tool.userspace_limitations(current)
    assert len(current["inaccessible"]) == 2 and current["other_UID_absence_proven"] is False


@pytest.mark.parametrize("fault", ["PID", "generation", "UID", "field", "extra_user", "missing_user", "unproven_kernel", "target", "generation_incomplete", "waiver", "absence_claim", "same_uid_claim", "hash"])
def test_every_userspace_change_target_and_unapproved_scope_rejected(fault):
    before = usage(); current = copy.deepcopy(before); authority = policy_authority(before)
    if fault == "PID": current["inaccessible"][0]["pid"] += 1
    elif fault == "generation": current["inaccessible"][0]["start"] = "124"
    elif fault == "UID": current["inaccessible"][0]["uid"] += 1
    elif fault == "field": current["inaccessible"][0]["field"] = "cwd"
    elif fault == "extra_user": current["inaccessible"].append(dict(pid=43, uid=1, start="1", field="fd", kernel_thread_proven=False))
    elif fault == "missing_user": current["inaccessible"].pop(0)
    elif fault == "unproven_kernel": current["inaccessible"][1]["kernel_thread_proven"] = False
    elif fault == "target": current["hits"] = [{"reference": "actual target"}]
    elif fault == "generation_incomplete": current["PID_generation_validation_complete"] = False
    elif fault == "waiver": authority["proven_UID0_Kthread1_PID_churn_accepted"] = False
    elif fault == "absence_claim": authority["other_uid_absence_not_proven"] = False
    elif fault == "same_uid_claim": authority["same_uid_incomplete_also_disclosed"] = False
    else: authority["accepted_userspace_limitations_sha256"] = "0" * 64
    with pytest.raises(ValueError): tool.removal_policy(authority, current, dict(process_usage=before))


def proc_folder(base, pid=12, kthread="0", uids=None):
    folder = base / str(pid); (folder / "fd").mkdir(parents=True)
    (folder / "stat").write_text(str(pid) + " (worker with spaces) " + " ".join(["S"] + ["0"] * 18 + ["12345"] + ["0"] * 4))
    ids = [str(os.getuid())] * 4 if uids is None else uids
    (folder / "status").write_text("Name:\tkworker-looking-user\nUid:\t" + "\t".join(ids) + "\nKthread:\t" + kthread + "\n")
    for field in ("cwd", "root", "exe"): (folder / field).symlink_to("/not-a-target")
    for field in ("cmdline", "maps"): (folder / field).write_text("not a target")
    return folder


@pytest.mark.parametrize("kthread,uids,owner,proven", [("1", ["0"] * 4, 0, True), ("0", ["0"] * 4, 0, False),
    ("1", ["0", "0", "1", "0"], 0, False), ("1", ["0"] * 4, 1000, False), ("", ["0"] * 4, 0, False)])
def test_kernel_requires_status_flag_all_four_UIDs_and_directory_owner(tmp_path, monkeypatch, kthread, uids, owner, proven):
    proc = tmp_path / "proc"; folder = proc_folder(proc, kthread=kthread, uids=uids)
    original_stat, original_readlink = Path.stat, tool.os.readlink
    def st(path, *args, **kwargs):
        return SimpleNamespace(st_uid=owner) if path == folder else original_stat(path, *args, **kwargs)
    monkeypatch.setattr(Path, "stat", st)
    def link(path, *args, **kwargs):
        if Path(path) == folder / "cwd": raise PermissionError("synthetic inaccessible cwd")
        return original_readlink(path, *args, **kwargs)
    monkeypatch.setattr(tool.os, "readlink", link)
    result = tool.process_usage(proc)
    assert result["inaccessible"] == [dict(pid=12, uid=owner, start="12345", kernel_thread_proven=proven, field="cwd")]
    assert result["PID_generation_validation_complete"] and result["other_UID_absence_proven"] is False


def test_real_process_reader_preserves_actual_fd_target_hits(tmp_path, monkeypatch):
    proc = tmp_path / "proc"; folder = proc_folder(proc)
    monkeypatch.setattr(tool.frozen, "TARGETS", ("/synthetic/owned/tree",))
    (folder / "fd/8").symlink_to("/synthetic/owned/tree/source.py")
    result = tool.process_usage(proc)
    assert result["hits"][0]["field"] == "fd"
    assert result["hits"][0]["reference"] == "/synthetic/owned/tree/source.py"


def test_PID_generation_change_blocks_before_removal_policy(tmp_path, monkeypatch):
    proc = tmp_path / "proc"; folder = proc_folder(proc); original = Path.read_text; calls = 0
    def changed(path, *args, **kwargs):
        nonlocal calls
        value = original(path, *args, **kwargs)
        if path == folder / "stat":
            calls += 1
            if calls > 1: value = value.replace("12345", "12346")
        return value
    monkeypatch.setattr(Path, "read_text", changed)
    result = tool.process_usage(proc)
    assert not result["PID_generation_validation_complete"] and result["generation_errors"]


def test_source_manifest_preserves_both_original_frozen_files_and_new_sources():
    result = tool.sources()
    assert len(result) == 4 and all(result[name] == pin for name, pin in tool.OLD_SOURCE_SHA.items())
    assert tool.OLD_BACKUP_SHA == "9b7ad93a8c40b790d8d3699c46ca1574cb5c20ba02f59e298cd6dd0141396ab2"


@pytest.fixture
def prepared(tmp_path, monkeypatch):
    roots = []
    for index in range(8):
        root = tmp_path / ("snapshot_%d" % index); root.mkdir(mode=0o700)
        (root / "source.txt").write_text("full original source " + str(index)); (root / "link").symlink_to("source.txt")
        roots.append(str(root))
    monkeypatch.setattr(tool.frozen, "TARGETS", tuple(roots))
    monkeypatch.setattr(tool.frozen, "TARGET_INODES", {path: Path(path).stat().st_ino for path in roots})
    monkeypatch.setattr(tool.frozen, "SOURCE_UID", os.getuid())
    monkeypatch.setattr(tool.frozen, "ROOT", tmp_path); monkeypatch.setattr(tool, "ROOT", tmp_path)
    receipt = tmp_path / "new_receipt"; receipt.mkdir(mode=0o700); monkeypatch.setattr(tool, "RECEIPT", receipt)
    oldfile = tmp_path / "old_inspection.json"; backupfile = tmp_path / "old_backup.json"
    oldfile.write_text("old immutable inspection"); backupfile.write_text("old immutable backup")
    monkeypatch.setattr(tool, "OLD_INSPECTION", oldfile); monkeypatch.setattr(tool, "OLD_BACKUP", backupfile)
    monkeypatch.setattr(tool, "OLD_INSPECTION_SHA", tool.digest(oldfile)); monkeypatch.setattr(tool, "OLD_BACKUP_SHA", tool.digest(backupfile))
    ref = tmp_path / "ROOT_current_plan.json"; ref.write_text("no temporary reference")
    maps = tuple(tmp_path / name for name in ("map_train.json", "map_launch.json"))
    for path in maps: path.write_text("no temporary reference")
    monkeypatch.setattr(tool, "REQUIRED_NEW_REFERENCES", maps)
    refs = {str(path): tool.digest(path) for path in (ref, *maps)}
    sourcepaths = {"old_helper.py": b"old helper", "old_test.py": b"old test"}
    pins = {name: hashlib.sha256(raw).hexdigest() for name, raw in sourcepaths.items()}
    for name, raw in sourcepaths.items(): (tmp_path / name).write_bytes(raw)
    monkeypatch.setattr(tool, "OLD_SOURCE_SHA", pins)
    four = dict(pins, new_helper="a" * 64, new_test="b" * 64); monkeypatch.setattr(tool, "sources", lambda: four)
    expected = tool.frozen.census(roots, local=True)
    old = dict(census=expected, references={str(ref): refs[str(ref)]})
    remote_census = dict(expected, identities={"remote independent inode": [1]}, generation_sha256="c" * 64)
    backup = dict(census=expected, backup_authority={"ROOT": "old backup only"}, remote_census=remote_census)
    monkeypatch.setattr(tool, "original_evidence", lambda: (old, backup))
    before = usage(); monkeypatch.setattr(tool, "process_usage", lambda: copy.deepcopy(before))
    inspection = dict(schema=tool.SCHEMA, census=expected, references=refs, process_usage=before, source_sha256=four)
    tool.frozen.write_new(receipt / "inspection.json", inspection)
    authority = dict(policy_authority(before), schema=tool.AUTH_SCHEMA, mode="remove", source_sha256=four,
        targets=roots, remote_backup=str(tool.frozen.REMOTE), original_inspection_sha256=tool.OLD_INSPECTION_SHA,
        backup_receipt_sha256=tool.OLD_BACKUP_SHA, fresh_inspection_sha256=tool.digest(receipt / "inspection.json"),
        payload_sha256=expected["payload_sha256"], generation_sha256=expected["generation_sha256"],
        tree_pins=tool.frozen.tree_pins(expected), declared_reference_sha256=refs,
        current_root_declaration_set_complete=True, deletion_authorized=True, user_approved_full_backup_then_exact8_removal=True)
    authority_path = tmp_path / "ROOT_remove.json"; tool.frozen.write_new(authority_path, authority)
    args = ["--mode", "remove", "--expected-tool-sha256", tool.digest(FILE), "--authorization", str(authority_path),
        "--authorization-sha256", tool.digest(authority_path)]
    calls = []; monkeypatch.setattr(tool.frozen, "remote_program", lambda phase: ["FAKE_REMOTE", phase])
    def run(command, **options):
        assert command == ["FAKE_REMOTE", "verify"]
        package = json.loads(options["input"]); assert package["census"]["entries"] == expected["entries"]
        calls.append(package)
        return SimpleNamespace(returncode=0, stdout=tool.encoded(remote_census), stderr=b"")
    monkeypatch.setattr(tool.subprocess, "run", run)
    return SimpleNamespace(roots=roots, receipt=receipt, args=args, before=before, calls=calls,
        authority=authority, authority_path=authority_path, inspection=inspection, old=old, backup=backup, refs=refs)


def test_full_eight_removal_uses_old_backup_three_verifications_and_all_usage_receipts(prepared):
    p = prepared
    assert tool.main(p.args) == 0 and not any(Path(path).exists() for path in p.roots)
    assert len(p.calls) == 3 and len(list(p.receipt.glob("removed_*.json"))) == 8
    assert len(list(p.receipt.glob("observed_process_usage_*.json"))) == 10
    result = tool.read(p.receipt / "removal_result.json")
    assert result["all_uid_absence_claimed"] is False and result["remote_backup_retained"]
    assert tool.digest(tool.OLD_BACKUP) == tool.OLD_BACKUP_SHA
    assert tool.digest(tool.OLD_INSPECTION) == tool.OLD_INSPECTION_SHA


def test_kernel_churn_throughout_remote_and_each_target_still_preserves_all_userspace(prepared, monkeypatch):
    n = 0
    def changing():
        nonlocal n
        n += 1
        return usage(kernels=[dict(pid=90 + n, uid=0, start=str(200 + n), field="cwd", kernel_thread_proven=True)])
    monkeypatch.setattr(tool, "process_usage", changing)
    assert tool.main(prepared.args) == 0 and n == 10


@pytest.mark.parametrize("fault", ["before_remote", "after_remote", "before_target", "target_hit"])
def test_changed_userspace_or_target_reference_always_blocks_and_preserves_failed_observation(prepared, monkeypatch, fault):
    n = 0
    def changed():
        nonlocal n
        n += 1; value = copy.deepcopy(prepared.before)
        active = n == {"before_remote": 1, "after_remote": 2, "before_target": 3, "target_hit": 3}[fault]
        if active:
            if fault == "target_hit": value["hits"] = [{"reference": prepared.roots[0]}]
            else: value["inaccessible"][0]["start"] = "different_generation"
        return value
    monkeypatch.setattr(tool, "process_usage", changed)
    with pytest.raises(ValueError): tool.main(prepared.args)
    assert all(Path(path).exists() for path in prepared.roots)
    assert tool.read(prepared.receipt / "remove_failure.json")["removed"] == []
    assert list(prepared.receipt.glob("observed_process_usage_*.json"))


@pytest.mark.parametrize("fault", ["source", "backup", "inspection", "refs", "targets", "churn", "generation", "authorization"])
def test_changed_root_authority_blocks_before_any_remote_or_removal(prepared, fault):
    p = prepared; value = copy.deepcopy(p.authority)
    if fault == "source": value["source_sha256"]["new_helper"] = "f" * 64
    elif fault == "backup": value["backup_receipt_sha256"] = "f" * 64
    elif fault == "inspection": value["fresh_inspection_sha256"] = "f" * 64
    elif fault == "refs": value["declared_reference_sha256"] = {}
    elif fault == "targets": value["targets"] = p.roots[:-1]
    elif fault == "churn": value["proven_UID0_Kthread1_PID_churn_accepted"] = False
    elif fault == "generation": value["generation_sha256"] = "f" * 64
    else: value["deletion_authorized"] = False
    p.authority_path.write_text(json.dumps(value)); p.args[-1] = tool.digest(p.authority_path)
    with pytest.raises(ValueError): tool.main(p.args)
    assert not p.calls and all(Path(path).exists() for path in p.roots)


def test_source_mutation_and_failed_remote_check_never_delete(prepared, monkeypatch):
    p = prepared
    (Path(p.roots[2]) / "source.txt").write_text("new user source edit")
    with pytest.raises(ValueError, match="original local bytes"): tool.main(p.args)
    assert not p.calls and all(Path(path).exists() for path in p.roots)


def test_actual_failed_remote_preserves_all_sources(prepared, monkeypatch):
    monkeypatch.setattr(tool.subprocess, "run", lambda *a, **k: SimpleNamespace(returncode=1, stdout=b"", stderr=b"synthetic failure"))
    with pytest.raises(ValueError, match="remote backup verification failed"): tool.main(prepared.args)
    assert all(Path(path).exists() for path in prepared.roots)
    assert (prepared.receipt / "remove_remote_before_stderr.txt").read_bytes() == b"synthetic failure"


def test_one_attempt_only_and_partial_removal_preserves_receipts(prepared, monkeypatch):
    original = shutil.rmtree
    def fail(path):
        if str(path) == prepared.roots[1]: raise OSError("synthetic second removal failure")
        original(path)
    fail.avoids_symlink_attacks = True; monkeypatch.setattr(tool.shutil, "rmtree", fail)
    with pytest.raises(OSError): tool.main(prepared.args)
    assert not Path(prepared.roots[0]).exists() and all(Path(path).exists() for path in prepared.roots[1:])
    assert tool.read(prepared.receipt / "remove_failure.json")["removed"] == prepared.roots[:1]
    with pytest.raises(ValueError, match="one declared attempt"): tool.main(prepared.args)


def test_missing_current_MAP_authority_and_existing_target_reference_block(prepared):
    p = prepared
    with pytest.raises(ValueError, match="both current V3 map"): tool.current_references([next(iter(p.old["references"]))], p.old)
    path = Path(next(iter(p.refs))); path.write_text(p.roots[0])
    with pytest.raises(ValueError, match="temporary tree"): tool.current_references(p.refs, p.old)


def test_fresh_inspection_creates_no_SSH_or_removal_and_retains_original_backup(prepared, monkeypatch):
    p = prepared; fresh = p.receipt.with_name("fresh_inspection"); monkeypatch.setattr(tool, "RECEIPT", fresh)
    monkeypatch.setattr(tool.subprocess, "run", lambda *a, **k: pytest.fail("inspection cannot use SSH"))
    args = ["--mode", "inspect", "--expected-tool-sha256", tool.digest(FILE)]
    for path in p.refs: args.extend(["--reference-file", path])
    assert tool.main(args) == 0
    assert tool.read(fresh / "inspection.json")["deletion_authorized"] is False
    assert all(Path(path).exists() for path in p.roots) and tool.digest(tool.OLD_BACKUP) == tool.OLD_BACKUP_SHA


def test_no_backup_create_auto_retry_installs_or_frozen_source_rewrites():
    source = FILE.read_text()
    assert 'remote_program("verify")' in source and 'remote_program("create")' not in source
    assert '"rsync"' not in source and "shell=True" not in source and "os.kill" not in source
    assert 'choices=("inspect", "remove")' in source
    assert 'all_uid_absence_claimed=False' in source


@pytest.mark.parametrize("fault", [None, "source", "remote_entries", "removed", "approved", "remote_path", "authority_schema", "authority_mode", "authority_targets", "old_inspection_pin", "tree_pins", "references"])
def test_original_backup_reader_all_frozen_evidence_guards_without_real_payload(monkeypatch, fault):
    pins = dict(tool.OLD_SOURCE_SHA); census = dict(entries={"synthetic": {"kind": "file", "sha256": "a" * 64}})
    old = dict(schema=tool.frozen.SCHEMA, source_sha256=pins, census=census, references={"synthetic-plan": "b" * 64})
    tree_pins = {"synthetic-tree": "c" * 64}
    authority = dict(schema=tool.frozen.AUTH_SCHEMA, mode="backup", targets=list(tool.frozen.TARGETS),
        source_sha256=pins, inspection_sha256=tool.OLD_INSPECTION_SHA, tree_pins=tree_pins,
        declared_reference_sha256=old["references"])
    backup = dict(schema=tool.frozen.SCHEMA, census=census, remote_census=dict(entries=copy.deepcopy(census["entries"])),
        local_removed=False, deletion_authorized=False, remote_backup=str(tool.frozen.REMOTE), backup_authority=authority)
    if fault == "source": old["source_sha256"] = {}
    elif fault == "remote_entries": backup["remote_census"]["entries"] = {}
    elif fault == "removed": backup["local_removed"] = True
    elif fault == "approved": backup["deletion_authorized"] = True
    elif fault == "remote_path": backup["remote_backup"] = "/wrong/private/archive"
    elif fault == "authority_schema": authority["schema"] = "wrong"
    elif fault == "authority_mode": authority["mode"] = "remove"
    elif fault == "authority_targets": authority["targets"] = []
    elif fault == "old_inspection_pin": authority["inspection_sha256"] = "f" * 64
    elif fault == "tree_pins": authority["tree_pins"] = {}
    elif fault == "references": authority["declared_reference_sha256"] = {}
    monkeypatch.setattr(tool, "sources", lambda: {**pins, "fresh-helper": "d" * 64, "fresh-test": "e" * 64})
    monkeypatch.setattr(tool.frozen, "tree_pins", lambda actual: tree_pins)
    def read(path, pin):
        assert (path, pin) in ((tool.OLD_INSPECTION, tool.OLD_INSPECTION_SHA), (tool.OLD_BACKUP, tool.OLD_BACKUP_SHA))
        return old if path == tool.OLD_INSPECTION else backup
    monkeypatch.setattr(tool, "read", read)
    if fault is None: assert tool.original_evidence() == (old, backup)
    else:
        with pytest.raises(ValueError): tool.original_evidence()


def test_process_scan_failure_is_preserved_without_fabricated_absence(prepared, monkeypatch):
    def missing(): raise PermissionError("synthetic entire proc scan failure")
    monkeypatch.setattr(tool, "process_usage", missing)
    with pytest.raises(PermissionError): tool.main(prepared.args)
    report = tool.read(prepared.receipt / "observed_process_usage_00_before_remote.json.scan_failure.json")
    assert report["observed_process_usage_available"] is False
    assert tool.read(prepared.receipt / "remove_failure.json")["removed"] == []
    assert all(Path(path).exists() for path in prepared.roots) and not prepared.calls


def test_nonprivate_receipt_directory_rejected_before_process_or_remote(prepared):
    prepared.receipt.chmod(0o755)
    with pytest.raises(ValueError, match="0700"): tool.main(prepared.args)
    assert not prepared.calls and all(Path(path).exists() for path in prepared.roots)


def test_child_mutation_after_first_removal_blocks_next_target_with_backup_intact(prepared, monkeypatch):
    original = shutil.rmtree
    def mutate(path):
        if str(path) == prepared.roots[0]:
            (Path(prepared.roots[1]) / "source.txt").write_text("new user edit during first removal")
        original(path)
    mutate.avoids_symlink_attacks = True; monkeypatch.setattr(tool.shutil, "rmtree", mutate)
    with pytest.raises(ValueError, match="per-target payload changed"): tool.main(prepared.args)
    assert not Path(prepared.roots[0]).exists() and all(Path(path).exists() for path in prepared.roots[1:])
    assert tool.digest(tool.OLD_BACKUP) == tool.OLD_BACKUP_SHA

# HH_261002 - Keep the reviewed V3 proposal unchanged while closing only its final-census timing gap.
def test_frozen_v3_source_and_tests_remain_exactly_preserved():
    root = Path(__file__).resolve().parents[2]
    pins = {
        "tools/e2e/remove_verified_tmp_source_snapshots_v3_20261002.py": "85aa06a038ec783c7af1ca6df606e912bdf706ebe53c6395167bf3571ad14b1d",
        "learning/tests/test_remove_verified_tmp_source_snapshots_v3_20261002.py": "e7faa4019a8b56f9db8273e820f781273c3268b68df924e7e03c3179b6f16743",
    }
    assert all(tool.digest(root / name) == pin for name, pin in pins.items())


@pytest.mark.parametrize("fault", ["bytes", "inode", "link", "ROOT_reference", "ROOT_authority"])
def test_mutation_during_last_process_scan_fails_before_any_target_removal(prepared, monkeypatch, fault):
    # HH_261002 - A process scan must not leave an unverified source or declaration change before rmtree.
    calls = 0
    def changed():
        nonlocal calls
        calls += 1
        if calls == 3:
            root = Path(prepared.roots[0])
            if fault == "bytes":
                (root / "source.txt").write_text("new user bytes during process scan")
            elif fault == "inode":
                original = root / "source.txt"
                original.rename(root / "preserved_source.txt")
                original.write_text("full original source 0")
            elif fault == "link":
                (root / "link").unlink()
                (root / "link").symlink_to(".")
            elif fault == "ROOT_reference":
                Path(next(iter(prepared.refs))).write_text("changed ROOT declaration")
            else:
                prepared.authority_path.write_text("changed ROOT authority")
        return copy.deepcopy(prepared.before)
    monkeypatch.setattr(tool, "process_usage", changed)
    with pytest.raises(ValueError):
        tool.main(prepared.args)
    assert all(Path(path).exists() for path in prepared.roots)
    assert tool.read(prepared.receipt / "remove_failure.json")["removed"] == []
    assert len(prepared.calls) == 2
    assert tool.digest(tool.OLD_BACKUP) == tool.OLD_BACKUP_SHA
    assert tool.read(prepared.receipt / "observed_process_usage_02_before_target.json") == prepared.before


@pytest.mark.parametrize("field,value", [("pid", 43), ("uid", 0), ("start", "124"), ("field", "maps")])
def test_unknown_userspace_limitations_are_never_normalized_or_waived(field, value):
    # HH_261002 - Unknown identity is not a kernel exemption; each disclosed userspace field remains exact.
    unknown = dict(pid=42, uid=None, start=None, field="identity", kernel_thread_proven=False)
    before = usage(users=[unknown]); current = copy.deepcopy(before)
    current["inaccessible"][0][field] = value
    with pytest.raises(ValueError, match="every userspace limitation"):
        tool.removal_policy(policy_authority(before), current, dict(process_usage=before))
