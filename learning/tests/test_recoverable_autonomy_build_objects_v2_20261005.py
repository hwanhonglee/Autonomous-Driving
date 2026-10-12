"""HH_261005 - Prove five exact service extensions and the unchanged V1 object/archive/unlink lifecycle on synthetic data."""
from __future__ import annotations

import ast
import inspect
from pathlib import Path
import stat

import pytest

from tools.maintenance import recoverable_autonomy_build_objects_v2_20261005 as m
from learning.tests.test_recoverable_autonomy_build_objects_v1_20261005 import fake_proc


CASES = [
    dict(uid=[0]*4, comm="anydesk", argv=["/usr/bin/anydesk", "--service"], cgroup="/system.slice/anydesk.service", parent_argv=["/lib/systemd/systemd", "--system", "--deserialize", "31", "splash"], parent_uid=[0]*4, parent_cgroup="/init.scope", parent_pid=1),
    dict(uid=[1000]*4, comm="(sd-pam)", argv=["(sd-pam)"], cgroup="/user.slice/user-1000.slice/user@1000.service/init.scope", parent_argv=["/lib/systemd/systemd", "--user"], parent_uid=[1000]*4, parent_cgroup="/user.slice/user-1000.slice/user@1000.service/init.scope", parent_pid=11),
    dict(uid=[1000,0,0,0], comm="fusermount3", argv=["fusermount3", "-o", "rw,nosuid,nodev,fsname=portal,auto_unmount,subtype=portal", "--", "/run/user/1000/doc"], cgroup="/user.slice/user-1000.slice/user@1000.service/session.slice/xdg-document-portal.service", parent_argv=["/usr/libexec/xdg-document-portal"], parent_uid=[1000]*4, parent_cgroup="/user.slice/user-1000.slice/user@1000.service/session.slice/xdg-document-portal.service", parent_pid=11),
    dict(uid=[1000]*4, comm="ssh-agent", argv=["/usr/bin/ssh-agent", "-D", "-a", "/run/user/1000/keyring/.ssh"], cgroup="/user.slice/user-1000.slice/session-499.scope", parent_argv=["/usr/bin/gnome-keyring-daemon", "--daemonize", "--login"], parent_uid=[1000]*4, parent_cgroup="/user.slice/user-1000.slice/session-499.scope", parent_pid=11),
    dict(uid=[1000]*4, comm="ssh-agent", argv=["ssh-agent", "-s"], cgroup="/user.slice/user-1000.slice/user@1000.service/app.slice/vte-spawn-a7a9504f-ba5d-4b2b-8fb1-d4ac343632e3.scope", parent_argv=["/lib/systemd/systemd", "--user"], parent_uid=[1000]*4, parent_cgroup="/user.slice/user-1000.slice/user@1000.service/init.scope", parent_pid=11),
]


def prepared(tmp_path, monkeypatch, case):
    proc = tmp_path / "proc"
    proc.mkdir()
    parent = fake_proc(proc, case["parent_pid"], uid=case["parent_uid"][0], argv=("\0".join(case["parent_argv"])+"\0").encode())
    (parent / "cgroup").write_text("0::"+case["parent_cgroup"]+"\n")
    p = fake_proc(proc, 12, uid=case["uid"][0], argv=("\0".join(case["argv"])+"\0").encode())
    status = "Uid:\t"+"\t".join(map(str, case["uid"]))+"\nPPid:\t"+str(case["parent_pid"])+"\nKthread:\t0\n"
    (p / "status").write_text(status)
    (p / "comm").write_text(case["comm"]+"\n")
    (p / "cgroup").write_text("0::"+case["cgroup"]+"\n")
    row = dict(pid=12, start="99", uid_four=case["uid"], cmdline="\0".join(case["argv"])+"\0")
    def trusted(raw, **kw):
        mode = stat.S_IFREG | 0o755
        if str(raw).endswith("fusermount3"): mode |= stat.S_ISUID
        if str(raw).endswith("ssh-agent"): mode |= stat.S_ISGID
        return dict(requested=str(raw), generation=dict(mode=mode))
    monkeypatch.setattr(m.base, "trusted_installed_file", trusted)
    monkeypatch.setattr(m, "trusted_unit", lambda raw: dict(requested=raw, exact_root_owned=True))
    monkeypatch.setattr(m, "ORIGINAL_SERVICE", lambda *a, **k: None)
    return p, row, status, parent, proc


@pytest.mark.parametrize("case", CASES)
def test_five_exact_installed_parent_services_are_not_generic_exemptions(tmp_path, monkeypatch, case):
    p, row, status, parent, proc = prepared(tmp_path, monkeypatch, case)
    result = m.exact_system_service(p, row, status, proc=proc)
    assert result["exact_argv"] == case["argv"]
    assert result["parent"]["pid"] == case["parent_pid"] and result["parent"]["start"] == "99"
    assert result["fd_maps_proof_available"] is False


@pytest.mark.parametrize("case", CASES)
@pytest.mark.parametrize("fault", ["argv", "comm", "cgroup", "uid", "parent_argv", "parent_uid", "parent_cgroup", "generation"])
def test_every_nearmiss_keeps_unresolved_consumer_refusal(tmp_path, monkeypatch, case, fault):
    p, row, status, parent, proc = prepared(tmp_path, monkeypatch, case)
    if fault == "argv": row["cmdline"] += "extra\0"
    elif fault == "comm": (p / "comm").write_text("generic-sh\n")
    elif fault == "cgroup": (p / "cgroup").write_text("0::/unknown\n")
    elif fault == "uid": row["uid_four"] = [999]*4
    elif fault == "parent_argv": (parent / "cmdline").write_bytes(b"/bin/sh\0")
    elif fault == "parent_uid": (parent / "status").write_text("Uid:\t999\t999\t999\t999\n")
    elif fault == "parent_cgroup": (parent / "cgroup").write_text("0::/unknown\n")
    else: row["start"] = "100"
    try:
        result = m.exact_system_service(p, row, status, proc=proc)
    except ValueError:
        return
    assert result is None


@pytest.mark.parametrize("index", [2,3,4])
def test_special_executable_mode_is_required(tmp_path, monkeypatch, index):
    p, row, status, parent, proc = prepared(tmp_path, monkeypatch, CASES[index])
    monkeypatch.setattr(m.base, "trusted_installed_file", lambda raw, **kw: dict(requested=str(raw), generation=dict(mode=stat.S_IFREG|0o755)))
    with pytest.raises(ValueError, match="setuid|setgid"):
        m.exact_system_service(p, row, status, proc=proc)


def test_original_exact_service_is_delegated_without_relabeling(tmp_path, monkeypatch):
    original = dict(original=True, fd_maps_proof_available=False)
    monkeypatch.setattr(m, "ORIGINAL_SERVICE", lambda *a, **k: original)
    assert m.exact_system_service(tmp_path, {}, "") is original


def test_main_inverse_ast_single_archive_tuple_seam():
    original = inspect.getsource(m.base.main)
    old = "for p in (Path(__file__).resolve(), TEST)"
    assert original.count(old) == 1
    amended = original.replace(old, "for p in ARCHIVE_SOURCES")
    assert ast.dump(ast.parse(amended.replace("for p in ARCHIVE_SOURCES", old))) == ast.dump(ast.parse(original))
    assert callable(m.prepared_main())


def test_source_closure_preserves_frozen_v1_and_actual_new_code():
    pins = m.sources()
    assert len(pins) == 4
    for path, value in m.PINS.items():
        assert pins[str(path.relative_to(m.ROOT))] == value


def test_scope_restores_even_exception_and_private_main_has_v2_actual_identity():
    names = ["OUT", "REMOTE", "SCHEMA", "AUTH_SCHEMA", "TEST", "sources", "exact_system_service"]
    before = {name:getattr(m.base,name) for name in names}
    with pytest.raises(RuntimeError):
        with m.activated():
            inner = m.prepared_main()
            assert inner.__globals__["OUT"] == m.OUT and inner.__globals__["SCHEMA"] == m.SCHEMA
            assert inner.__globals__["ARCHIVE_SOURCES"] == m.ARCHIVE_SOURCES
            assert m.base.exact_system_service is m.exact_system_service
            assert m.base.REMOTE == m.REMOTE
            raise RuntimeError("synthetic setup failure")
    assert all(getattr(m.base,name) is before[name] for name in names)
