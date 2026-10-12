"""HH_261005 - Exercise exact object backup guards using synthetic temporary files, never live build deletion."""
from __future__ import annotations

import copy
import io
import json
import os
from pathlib import Path
import sys

import pytest

from tools.maintenance import recoverable_autonomy_build_objects_v1_20261005 as m


@pytest.fixture
def tree(tmp_path):
    build = tmp_path / "build"
    obj = build / "pkg/CMakeFiles/demo.dir/src/demo.cpp.o"
    obj.parent.mkdir(parents=True)
    obj.write_bytes(b"synthetic object\x00\xff")
    install = tmp_path / "install"
    install.mkdir()
    return build, obj, install


@pytest.mark.parametrize("name", ["pkg/CMakeFiles/x.o", "pkg/CMakeFiles/x.dir/src/a.cpp.o"])
def test_exact_scope_accepts(name):
    assert m.scoped(name)


@pytest.mark.parametrize("name", ["/pkg/CMakeFiles/x.o", "pkg/CMakeFiles/../x.o", "pkg/x.o", "pkg/CMakeFiles/x.so", "canonical_rebuild/CMakeFiles/x.o", "pkg/CMakeFiles/a\nx.o", "", "pkg/CMakeFiles"])
def test_scope_denies_every_unapproved_artifact(name):
    assert not m.scoped(name)


def test_census_exact_no_follow_and_allocation(tree):
    build, obj, _ = tree
    (build / "pkg/CMakeFiles/ignored.so").write_bytes(b"installed library stays")
    (build / "pkg/CMakeFiles/link.o").symlink_to(obj)
    canonical = build / "canonical_rebuild/CMakeFiles"
    canonical.mkdir(parents=True)
    (canonical / "other.o").write_bytes(b"excluded")
    result = m.inventory(build)
    assert result["object_count"] == 1
    assert result["logical_bytes"] == obj.stat().st_size
    assert result["allocated_bytes"] == obj.stat().st_blocks * 512
    assert result["excluded_symlinks"] == ["pkg/CMakeFiles/link.o"]
    assert m.verify_object_before_unlink("pkg/CMakeFiles/demo.dir/src/demo.cpp.o", next(iter(result["objects"].values())), result)["removed"] is False


def test_hardlinked_object_refused(tree):
    build, obj, _ = tree
    os.link(obj, obj.with_name("alias.o"))
    with pytest.raises(ValueError, match="single-link"):
        m.inventory(build)


def test_symlink_build_root_refused(tree, tmp_path):
    build, _, _ = tree
    alias = tmp_path / "alias"
    alias.symlink_to(build, target_is_directory=True)
    with pytest.raises(ValueError, match="symlink-free"):
        m.inventory(alias)


@pytest.mark.parametrize("mutation", ["payload", "inode", "mtime", "symlink", "parent"])
def test_generation_or_reference_change_denied(tree, monkeypatch, mutation):
    build, obj, _ = tree
    manifest = m.inventory(build)
    relative, expected = next(iter(manifest["objects"].items()))
    if mutation == "payload":
        obj.write_bytes(b"different")
    elif mutation == "inode":
        obj.rename(obj.with_name("prior"))
        obj.write_bytes(b"synthetic object\x00\xff")
    elif mutation == "mtime":
        os.utime(obj, ns=(obj.stat().st_atime_ns, obj.stat().st_mtime_ns + 1))
    elif mutation == "symlink":
        obj.rename(obj.with_name("prior"))
        obj.symlink_to(obj.with_name("prior"))
    else:
        obj.parent.rename(obj.parent.with_name("prior"))
        obj.parent.mkdir()
        obj.write_bytes(b"synthetic object\x00\xff")
    unlinks = []
    monkeypatch.setattr(m.os, "unlink", lambda *a, **k: unlinks.append((a, k)))
    with pytest.raises((ValueError, OSError)):
        m.verify_object_before_unlink(relative, expected, manifest, do_unlink=True, before_unlink=lambda r: None)
    assert unlinks == []


def test_intent_is_durable_before_unlink_and_failure_retains_intent(tree, monkeypatch):
    build, _, _ = tree
    manifest = m.inventory(build)
    relative, expected = next(iter(manifest["objects"].items()))
    rows = []
    def denied_unlink(name, **kwargs):
        assert rows and rows[0]["unlink_attempted"] is True
        assert name == Path(relative).name and type(kwargs["dir_fd"]) is int
        raise PermissionError("synthetic failure")
    monkeypatch.setattr(m.os, "unlink", denied_unlink)
    with pytest.raises(PermissionError):
        m.verify_object_before_unlink(relative, expected, manifest, do_unlink=True, before_unlink=rows.append)
    assert len(rows) == 1 and rows[0]["sha256"] == expected["sha256"]


def test_missing_durable_intent_blocks_unlink(tree, monkeypatch):
    build, _, _ = tree
    manifest = m.inventory(build)
    relative, expected = next(iter(manifest["objects"].items()))
    monkeypatch.setattr(m.os, "unlink", lambda *a, **k: pytest.fail("must never unlink"))
    with pytest.raises(ValueError, match="durable"):
        m.verify_object_before_unlink(relative, expected, manifest, do_unlink=True)


def test_install_symlink_and_text_refs_fail_closed(tree, monkeypatch):
    build, obj, install = tree
    manifest = m.inventory(build)
    monkeypatch.setattr(m, "BUILD", build)
    (install / "object.o").symlink_to(obj)
    (install / "import.cmake").write_text(f"set(IMPORT_OBJECTS \"{obj}\")\n")
    audit = m.install_audit(manifest, install)
    assert audit["symlink_count"] == 1 and audit["dangling_symlink_count"] == 0
    assert {row["kind"] for row in audit["hits"]} == {"symlink_to_object", "metadata_object_reference"}
    assert not audit["no_unknown_or_object_reference"]


def test_dangling_nonobject_install_link_is_disclosed_not_deleted(tree):
    build, _, install = tree
    manifest = m.inventory(build)
    (install / "missing").symlink_to(install / "not-present")
    audit = m.install_audit(manifest, install)
    assert audit["dangling_symlink_count"] == 1
    assert audit["no_unknown_or_object_reference"]


def test_install_object_inode_alias_denied(tree):
    build, obj, install = tree
    manifest = m.inventory(build)
    os.link(obj, install / "alias")
    assert m.install_audit(manifest, install)["hits"][0]["kind"] == "object_inode_alias"


def fake_proc(folder, pid, *, uid=1000, argv=b"/usr/bin/sleep\x0060\x00", kernel=False):
    p = folder / str(pid)
    p.mkdir()
    # HH_261005 - State is stat field 3 and starttime is field 22; no actual process is modified.
    (p / "stat").write_text(f"{pid} (synthetic) S " + " ".join(["0"]*18 + ["99"] + ["0"]*10))
    (p / "status").write_text(f"Uid:\t{uid}\t{uid}\t{uid}\t{uid}\nKthread:\t{int(kernel)}\nPPid:\t0\n")
    (p / "comm").write_text("sleep\n")
    (p / "cgroup").write_text("0::/user.slice/synthetic.scope\n")
    (p / "cmdline").write_bytes(argv)
    (p / "maps").write_text("")
    for role in ("cwd", "root", "exe"):
        (p / role).symlink_to("/usr/bin")
    (p / "fd").mkdir()
    return p


def test_process_full_readable_userspace_and_kernel_separate(tree, tmp_path):
    build, _, _ = tree
    proc = tmp_path / "proc"
    proc.mkdir()
    fake_proc(proc, 12)
    fake_proc(proc, 13, uid=0, kernel=True)
    result = m.process_audit(build, proc)
    assert result["no_unknown_or_active_reference"]
    assert len(result["userspace"]) == 1 and len(result["proven_kernel_threads"]) == 1


@pytest.mark.parametrize("argv", [b"/usr/bin/g++\x00a.cpp", b"/usr/bin/python3\x00/usr/bin/colcon\x00build"])
def test_build_tool_argv_any_token_blocks(tree, tmp_path, argv):
    build, _, _ = tree
    proc = tmp_path / "proc"
    proc.mkdir()
    fake_proc(proc, 12, argv=argv)
    assert m.process_audit(build, proc)["hits"][0]["field"] == "cmdline"


def test_process_object_fd_blocks(tree, tmp_path):
    build, obj, _ = tree
    proc = tmp_path / "proc"
    proc.mkdir()
    p = fake_proc(proc, 12)
    (p / "fd/3").symlink_to(obj)
    assert any(row["field"] == "fd" for row in m.process_audit(build, proc)["hits"])


def test_permission_denial_is_unknown_not_absence(tree, tmp_path, monkeypatch):
    build, _, _ = tree
    proc = tmp_path / "proc"
    proc.mkdir()
    p = fake_proc(proc, 12, uid=0)
    original = m.os.readlink
    def denied(path, *a, **kw):
        if Path(path) == p / "cwd":
            raise PermissionError("synthetic protected proc")
        return original(path, *a, **kw)
    monkeypatch.setattr(m.os, "readlink", denied)
    result = m.process_audit(build, proc)
    assert result["unknown"] == [dict(pid=12, start="99", uid_four=[0]*4, cmdline="/usr/bin/sleep\x0060\x00", field="cwd", error="PermissionError")]
    assert not result["no_unknown_or_active_reference"]


@pytest.mark.parametrize("argv,comm,service", [
    (["/usr/sbin/cron", "-f", "-P"], "cron", "cron"),
    (["/usr/bin/python3", "/usr/bin/networkd-dispatcher", "--run-startup-triggers"], "networkd-dispat", "networkd-dispatcher"),
    (["/usr/bin/python3", "/usr/share/unattended-upgrades/unattended-upgrade-shutdown", "--wait-for-signal"], "unattended-upgr", "unattended-upgrades"),
])
def test_exact_root_reviewed_service_retains_denials_not_fd_proof(tree, tmp_path, monkeypatch, argv, comm, service):
    build, _, _ = tree
    proc = tmp_path / "proc"
    proc.mkdir()
    p = fake_proc(proc, 12, uid=0, argv=("\0".join(argv)+"\0").encode())
    (p / "comm").write_text(comm+"\n")
    (p / "cgroup").write_text("0::/system.slice/"+service+".service\n")
    monkeypatch.setattr(m, "trusted_installed_file", lambda raw, **kw: dict(requested=str(raw), generation="synthetic-root-reviewed"))
    original = m.os.readlink
    def denied(path, *a, **kw):
        if Path(path) == p / "cwd":
            raise PermissionError("synthetic protected proc")
        return original(path, *a, **kw)
    monkeypatch.setattr(m.os, "readlink", denied)
    audit = m.process_audit(build, proc)
    assert audit["no_unresolved_potential_build_consumer"]
    assert audit["unknown"] == [] and len(audit["system_service_permission_denials"]) == 1
    assert not audit["all_uid_userspace_inspection_complete"]
    assert audit["exact_root_reviewed_system_services"][0]["fd_maps_proof_available"] is False


@pytest.mark.parametrize("fault", ["uid", "comm", "cgroup", "extra_argument", "script", "installed_trust"])
def test_service_nearmiss_cannot_bypass_unknown(tree, tmp_path, monkeypatch, fault):
    build, _, _ = tree
    proc = tmp_path / "proc"
    proc.mkdir()
    argv = ["/usr/bin/python3", "/usr/bin/networkd-dispatcher", "--run-startup-triggers"]
    if fault == "extra_argument": argv.append("/home/a/evil.py")
    if fault == "script": argv[1] = "/home/a/networkd-dispatcher"
    p = fake_proc(proc, 12, uid=1000 if fault == "uid" else 0, argv=("\0".join(argv)+"\0").encode())
    (p / "comm").write_text("evil\n" if fault == "comm" else "networkd-dispat\n")
    (p / "cgroup").write_text("0::/unknown\n" if fault == "cgroup" else "0::/system.slice/networkd-dispatcher.service\n")
    def trusted(raw, **kw):
        if fault == "installed_trust": raise ValueError("untrusted installed source")
        return dict(requested=str(raw))
    monkeypatch.setattr(m, "trusted_installed_file", trusted)
    original = m.os.readlink
    def denied(path, *a, **kw):
        if Path(path) == p / "cwd": raise PermissionError("synthetic denial")
        return original(path, *a, **kw)
    monkeypatch.setattr(m.os, "readlink", denied)
    audit = m.process_audit(build, proc)
    assert audit["unknown"] and not audit["no_unknown_or_active_reference"]
    assert audit["exact_root_reviewed_system_services"] == []


def test_gdm_worker_requires_actual_parent_generation(tree, tmp_path, monkeypatch):
    build, _, _ = tree
    proc = tmp_path / "proc"
    proc.mkdir()
    parent = fake_proc(proc, 11, uid=0, argv=b"/usr/sbin/gdm3\0")
    (parent / "cgroup").write_text("0::/system.slice/gdm.service\n")
    p = fake_proc(proc, 12, uid=0, argv=b"gdm-session-worker [pam/gdm-password]\0")
    (p / "status").write_text("Uid:\t0\t0\t0\t0\nPPid:\t11\nKthread:\t0\n")
    (p / "comm").write_text("gdm-session-wor\n")
    (p / "cgroup").write_text("0::/user.slice/user-1000.slice/session-499.scope\n")
    monkeypatch.setattr(m, "trusted_installed_file", lambda raw, **kw: dict(requested=str(raw)))
    row = dict(pid=12, start="99", uid_four=[0]*4, cmdline="gdm-session-worker [pam/gdm-password]\0")
    result = m.exact_system_service(p, row, (p / "status").read_text(), proc=proc)
    assert result["parent"]["pid"] == 11 and result["parent"]["start"] == "99"
    (parent / "cmdline").write_bytes(b"/bin/sh\0")
    with pytest.raises(ValueError, match="real gdm"):
        m.exact_system_service(p, row, (p / "status").read_text(), proc=proc)


def test_user_file_not_trusted_installed_source(tmp_path):
    p = tmp_path / "daemon"
    p.write_bytes(b"synthetic")
    with pytest.raises(ValueError, match="trusted prefix"):
        m.trusted_installed_file(p)


def test_receipts_create_only_and_exact_hash(tmp_path):
    receipt = tmp_path / "receipt.json"
    m.write_new(receipt, {"a": 1})
    assert m.read(receipt, m.digest(receipt)) == {"a": 1}
    with pytest.raises(FileExistsError):
        m.write_new(receipt, {"a": 2})
    with pytest.raises(ValueError, match="pinned"):
        m.read(receipt, "0"*64)


@pytest.mark.parametrize("raw", [b'{"a":1,"a":2}', b'{"a":NaN}'])
def test_malformed_receipt_denied(tmp_path, raw):
    receipt = tmp_path / "receipt.json"
    receipt.write_bytes(raw)
    with pytest.raises(ValueError):
        m.read(receipt, m.sha(raw))


@pytest.mark.parametrize("phase", ["create", "verify"])
def test_remote_program_valid_syntax_without_ssh(phase):
    compile(m.remote_program(phase), "synthetic_remote_only", "exec")


def test_remote_actual_verifier_checks_every_synthetic_object(tree, tmp_path, monkeypatch, capsys):
    build, obj, _ = tree
    personal = tmp_path / "personal"
    personal.mkdir(mode=0o700)
    remote = personal / "archive"
    monkeypatch.setattr(m, "REMOTE_PERSONAL", personal)
    monkeypatch.setattr(m, "REMOTE", remote)
    payload = dict(manifest=m.inventory(build), sources={"helper.py": m.sha(b"source")}, source_archive={"helper.py": b"source".hex()}, authority={"test": True})
    class Input:
        buffer = io.BytesIO(m.encoded(payload))
    monkeypatch.setattr(sys, "stdin", Input())
    exec(m.remote_program("create"), {})
    relative = next(iter(payload["manifest"]["objects"]))
    archived = remote / "objects" / relative
    archived.parent.mkdir(parents=True)
    archived.write_bytes(obj.read_bytes())
    archived.chmod(obj.stat().st_mode & 0o777)
    monkeypatch.setattr(sys, "stdin", type("Input", (), {"buffer": io.BytesIO(m.encoded(payload))})())
    exec(m.remote_program("verify"), {})
    assert json.loads(capsys.readouterr().out.splitlines()[-1])["all_object_hashes_verified"] is True
    archived.write_bytes(b"tampered")
    monkeypatch.setattr(sys, "stdin", type("Input", (), {"buffer": io.BytesIO(m.encoded(payload))})())
    with pytest.raises(ValueError, match="size/mode|hash"):
        exec(m.remote_program("verify"), {})
