"""HH_261005 - Validate scalar import-only seals and private staging using non-importable tiny bytes, never real CARLA."""

import ast
import hashlib
import json
import os
from pathlib import Path
import sys
import time
from types import SimpleNamespace as NS
import zipfile

import pytest

from scripts.e2e import probe_carla_import_only_v1_20261005 as probe


@pytest.fixture
def tiny_native(tmp_path, monkeypatch):
    # HH_261005 - This fake native is not loadable; neither real egg/cache nor production failed-prefix bytes are accessed.
    egg = tmp_path / "carla-0.9.15-py3.10-linux-x86_64.egg"
    source = tmp_path / "existing" / "carla" / "libcarla.so"
    source.parent.mkdir(parents=True)
    payload = b"synthetic native, never imported"*100
    source.write_bytes(payload)
    with zipfile.ZipFile(egg, "w") as archive:
        archive.writestr(zipfile.ZipInfo(probe.MEMBER, date_time=(2025, 5, 20, 20, 15, 0)), payload)
        archive.writestr("carla/__init__.py", b"synthetic init source")
        archive.writestr("carla/libcarla.py", b"synthetic bootstrap source")
    output = tmp_path / "probe"
    output.mkdir()
    private_cache = output / "Python-Eggs"
    private = private_cache / (egg.name+"-tmp") / probe.MEMBER
    monkeypatch.setattr(probe, "ROOT", tmp_path)
    monkeypatch.setattr(probe, "EGG", egg)
    monkeypatch.setattr(probe, "CACHE_NATIVE", source)
    monkeypatch.setattr(probe, "OUTPUT", output)
    monkeypatch.setattr(probe, "PRIVATE_CACHE", private_cache)
    monkeypatch.setattr(probe, "PRIVATE_NATIVE", private)
    monkeypatch.setattr(probe, "NATIVE_SIZE", len(payload))
    monkeypatch.setattr(probe, "NATIVE_SHA", hashlib.sha256(payload).hexdigest())
    monkeypatch.setattr(probe, "SMALL_MEMBERS", {"carla/__init__.py": hashlib.sha256(b"synthetic init source").hexdigest(),
        "carla/libcarla.py": hashlib.sha256(b"synthetic bootstrap source").hexdigest()})
    return source, private, payload


def test_private_full_copy_matches_native_zip_mtime_and_preserves_global_bytes(tiny_native):
    source, private, payload = tiny_native
    before = source.stat()
    probe.stage_native()
    probe.verify_private_native()
    assert private.read_bytes() == source.read_bytes() == payload
    assert source.stat().st_mtime_ns == before.st_mtime_ns and source.stat().st_ctime_ns == before.st_ctime_ns
    assert private.stat().st_mtime == time.mktime((2025, 5, 20, 20, 15, 0, 0, 0, -1))
    with pytest.raises(FileExistsError): probe.stage_native()


@pytest.mark.parametrize("fault", ["truncated", "same_size_corrupt", "stale_mtime", "symlink"])
def test_private_cache_denies_truncated_corrupt_or_stale_native_before_import(tiny_native, fault):
    _, private, payload = tiny_native
    probe.stage_native()
    if fault == "truncated": private.write_bytes(payload[:16])
    elif fault == "same_size_corrupt": private.write_bytes(b"x"*len(payload))
    elif fault == "stale_mtime": os.utime(private, (0, 0))
    else:
        target = private.with_name("unsealed.so")
        target.write_bytes(payload)
        private.unlink(); private.symlink_to(target)
    with pytest.raises(RuntimeError): probe.verify_private_native()
    assert "carla" not in sys.modules


def test_original_V1_partial_native_size_is_not_full_declared_native():
    # HH_261005 - Preserve the measured failed-prefix size as a diagnostic fact; never import/hash/rewrite that failed file.
    assert 16_777_216 < probe.NATIVE_SIZE == 153_958_688
    assert probe.NATIVE_SHA == "d60de268813db6e1c94b8f613888ed5f9eaed39cb02ac2debb4e1a05aedb8966"


@pytest.mark.parametrize("event,args", [
    ("open", ("forbidden", "wb", os.O_WRONLY|os.O_CREAT)),
    ("open", ("forbidden", "r+", os.O_RDWR)),
    ("os.remove", ("forbidden", -1)), ("os.rename", ("a", "b", -1, -1)),
    ("socket.__new__", (None, 2, 1, 0)), ("socket.connect", (None, ("127.0.0.1", 2440))),
    ("subprocess.Popen", ("forbidden", [], None, None)), ("os.utime", ("forbidden", None, None, -1)),
])
def test_audit_policy_denies_Python_extraction_mutation_or_endpoint_without_global_hook(event, args):
    with pytest.raises(RuntimeError): probe.audit_policy(event, args)


def test_audit_allows_reads_existing_private_directories_only(tiny_native):
    _, private, _ = tiny_native
    probe.stage_native()
    probe.audit_policy("open", (str(private), "rb", os.O_RDONLY))
    probe.audit_policy("os.mkdir", (str(private.parent), 511, -1))
    with pytest.raises((RuntimeError, FileNotFoundError)): probe.audit_policy("os.mkdir", (str(private.parent.parent / "new"), 511, -1))
    with pytest.raises(RuntimeError): probe.audit_policy("os.mkdir", (str(probe.CACHE_NATIVE.parent), 511, -1))


def test_environment_is_private_CPU_only_and_strips_original_ld_ros_vars(monkeypatch):
    for key in ("PYTHONPATH", "LD_LIBRARY_PATH", "AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH"):
        monkeypatch.setenv(key, "synthetic unwanted")
    value = probe.environment()
    assert value["CUDA_VISIBLE_DEVICES"] == "" and value["PYTHON_EGG_CACHE"] == str(probe.PRIVATE_CACHE)
    assert all(key not in value for key in ("PYTHONPATH", "LD_LIBRARY_PATH", "AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH"))
    assert value["PYTHONDONTWRITEBYTECODE"] == "1"


@pytest.mark.parametrize("fault", ["duplicate", "NaN", "SHA", "symlink"])
def test_pinned_plan_denies_malformed_or_unsealed_authority(tmp_path, fault):
    path = tmp_path / "plan.json"
    payload = {"duplicate": b'{"x":1,"x":2}', "NaN": b'{"x":NaN}', "SHA": b'{"x":1}', "symlink": b'{}'}[fault]
    path.write_bytes(payload)
    sha = hashlib.sha256(payload).hexdigest()
    if fault == "symlink":
        target = tmp_path / "actual.json"
        target.write_bytes(payload); path.unlink(); path.symlink_to(target)
    with pytest.raises(RuntimeError): probe.read_pinned(path, "0"*64 if fault == "SHA" else sha)


def test_missing_ROOT_authority_denies_before_stage_child_or_assets(tmp_path, monkeypatch):
    plan, authority = tmp_path / "plan.json", tmp_path / "ROOT_execution_authority.json"
    plan.write_text('{}')
    monkeypatch.setattr(probe, "PLAN", plan)
    monkeypatch.setattr(probe, "AUTHORITY", authority)
    monkeypatch.setattr(probe, "contract", lambda: {})
    monkeypatch.setattr(probe, "stage_native", lambda: pytest.fail("no stage without ROOT authority"))
    monkeypatch.setattr(probe.subprocess, "Popen", lambda *a, **k: pytest.fail("no import child without ROOT authority"))
    with pytest.raises(FileNotFoundError): probe._execute(hashlib.sha256(plan.read_bytes()).hexdigest(), "a"*64)


@pytest.mark.parametrize("fault", [None, "pid_generation", "PPID", "PGID", "SID"])
def test_only_exact_owned_generation_can_be_signalled_and_joined(monkeypatch, fault):
    core = dict(pid=42, start_ticks=77, ppid=os.getpid(), pgid=42, sid=42)
    actual = dict(core)
    if fault == "pid_generation": actual["start_ticks"] += 1
    elif fault is not None: actual[{"PPID":"ppid", "PGID":"pgid", "SID":"sid"}[fault]] += 1
    calls = []
    process = NS(pid=42, poll=lambda: None, wait=lambda **kwargs: calls.append("joined"))
    monkeypatch.setattr(probe, "process_core", lambda pid: actual)
    monkeypatch.setattr(probe.os, "killpg", lambda pid, sig: calls.append((pid, sig)))
    if fault is None:
        probe.stop_owned(process, core)
        assert calls[0][0] == 42 and calls[-1] == "joined"
    else:
        with pytest.raises(RuntimeError, match="generation/session"): probe.stop_owned(process, core)
        assert not calls


TRACE = '51 execve("/usr/bin/python3.10", ["python3.10"], []) = 0\n51 exit_group(0) = ?\n'
PHASES = '{"phase":"before_import"}\n{"phase":"after_import"}\n'


@pytest.mark.parametrize("fault", [None, "connect", "IPv6_socket", "unfinished", "missing_exit", "missing_phase"])
def test_actual_trace_claim_requires_complete_phases_and_no_endpoint_syscalls(fault):
    trace, phases = TRACE, PHASES
    if fault == "connect": trace += '51 connect(4, {sa_family=AF_INET, sin_port=htons(2440)}, 16) = 0\n'
    elif fault == "IPv6_socket": trace += '51 socket(AF_INET6, SOCK_STREAM, 0) = 4\n'
    elif fault == "unfinished": trace += '51 openat(4, "x", 0 <unfinished ...>\n'
    elif fault == "missing_exit": trace = trace.split("exit_group")[0]
    elif fault == "missing_phase": phases = phases.splitlines()[0]
    if fault is None:
        result = probe.audit_trace(trace, phases)
        assert result["actual_trace_complete"] and not result["native_endpoint_calls_observed"]
        assert not result["native_syscalls_OS_sandboxed"]
    else:
        with pytest.raises(RuntimeError): probe.audit_trace(trace, phases)


def test_runtime_lease_is_the_existing_lock_and_refuses_a_second_owner(tmp_path, monkeypatch):
    lock = tmp_path / "runtime.lock"
    lock.touch()
    monkeypatch.setattr(probe, "LOCK", lock)
    with probe.runtime_lease():
        with pytest.raises(BlockingIOError):
            with probe.runtime_lease(): pytest.fail("second lease admitted")
    with probe.runtime_lease(): pass


def test_free_floor_shared_and_closing_reserves_are_not_relaxed(tiny_native, monkeypatch):
    floor = 5*1024**3 + 64*1024**2 + 16*1024**2
    monkeypatch.setattr(probe.shutil, "disk_usage", lambda path: NS(free=floor))
    probe.budget()
    monkeypatch.setattr(probe.shutil, "disk_usage", lambda path: NS(free=floor-1))
    with pytest.raises(RuntimeError): probe.budget()


def test_source_full_import_has_no_Client_call_resource_override_or_zip_extraction():
    tree = ast.parse(Path(probe.__file__).read_text())
    forbidden = {"Client", "connect", "load_world", "get_world", "spawn_actor", "extract", "extractall", "set_num_threads"}
    assert not any(isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) and node.func.attr in forbidden for node in ast.walk(tree))
    assert not any(isinstance(node, ast.Attribute) and node.attr == "resource_filename" for node in ast.walk(tree))
    assert any(isinstance(node, ast.Import) and node.names[0].name == "carla" for node in ast.walk(tree))
    assert "torch" not in sys.modules and "carla" not in sys.modules
