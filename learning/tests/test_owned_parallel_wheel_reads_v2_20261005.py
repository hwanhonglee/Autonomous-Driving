"""HH_261005 - Retain V1's 31 pure guards and deny unbound V2 CARLA path, ROOT/plan seals and actual/archive assets before import."""

import ast
import importlib.util
import json
import os
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import pytest

from scripts.e2e import owned_parallel_wheel_reads_v2_20261005 as adapter


@pytest.fixture
def sealed_asset_fixture(tmp_path, monkeypatch):
    # HH_261005 - Only tiny temporary synthetic bytes are read; production CARLA assets and servers are never opened by these tests.
    carla_root = tmp_path / "packaged_CARLA"
    egg = carla_root / "PythonAPI/carla/dist/carla-0.9.15-py3.10-linux-x86_64.egg"
    run = tmp_path / "storage/experiments/validation/2026-10-05/synthetic_fresh_owned_probe"
    archive = run / "source_external" / egg.relative_to(carla_root)
    for path in (egg, archive):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"synthetic pinned asset, never an importable CARLA egg")
    sha = adapter.digest(egg)
    monkeypatch.setattr(adapter, "ROOT", tmp_path)
    monkeypatch.setattr(adapter, "CARLA_ROOT", carla_root)
    monkeypatch.setattr(adapter, "CARLA_EGG", egg)
    monkeypatch.setattr(adapter, "CARLA_EGG_SHA256", sha)
    proof = dict(carla_egg_path=str(egg), carla_egg_sha256=sha)
    authority = dict(proof, carla_egg_archive_path=str(archive))
    plan = dict(output=str(run), asset_sha256={str(egg): sha})
    return proof, authority, plan, egg, archive


def test_fixed_asset_is_exact_same_packaged_egg_as_actual_V5():
    assert str(adapter.CARLA_EGG) == "/home/a/carla-autoware-universe/CARLA_0.9.15/PythonAPI/carla/dist/carla-0.9.15-py3.10-linux-x86_64.egg"
    assert adapter.CARLA_EGG_SHA256 == "6eb690ef4304b34a919b27a23129273029008c8e95a5f164d52a1bd494a96c3a"


def test_asset_admission_requires_ROOT_plan_and_actual_owned_archive_bytes(sealed_asset_fixture):
    proof, authority, plan, egg, archive = sealed_asset_fixture
    assert adapter.admit_carla_asset(proof, authority, plan) == dict(proof,
        carla_egg_archive_path=str(archive), actual_and_archived_bytes_verified=True)


@pytest.mark.parametrize("fault", ("wrong_path", "wrong_SHA", "unsealed_asset", "plan_vs_ROOT", "symlink", "archive_SHA", "archive_path", "actual_SHA"))
def test_unsealed_or_misbound_asset_denies_before_any_CARLA_import(sealed_asset_fixture, monkeypatch, fault):
    proof, authority, plan, egg, archive = sealed_asset_fixture
    if fault == "wrong_path": proof["carla_egg_path"] = str(egg) + ".other"
    elif fault == "wrong_SHA": proof["carla_egg_sha256"] = "not_a_hexadecimal_SHA".ljust(64, "z")
    elif fault == "unsealed_asset": plan["asset_sha256"] = {}
    elif fault == "plan_vs_ROOT": authority["carla_egg_sha256"] = "0" * 64
    elif fault == "symlink":
        target = egg.with_name("synthetic_target")
        target.write_bytes(egg.read_bytes())
        egg.unlink()
        egg.symlink_to(target)
    elif fault == "archive_SHA": archive.write_bytes(b"changed archive")
    elif fault == "archive_path": authority["carla_egg_archive_path"] = str(archive) + ".other"
    elif fault == "actual_SHA": egg.write_bytes(b"changed actual")
    with pytest.raises(RuntimeError):
        adapter.admit_carla_asset(proof, authority, plan)
    assert "carla" not in sys.modules


def test_real_constructor_cannot_import_CARLA_without_worker_asset_admission():
    spec = importlib.util.spec_from_file_location("test_real_wheel_asset_admission", adapter.REAL_WORKER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    with pytest.raises(RuntimeError, match="ABI/asset"):
        module.CarlaWheelBackend(dict(synthetic_only=False), "FL", None)
    assert "carla" not in sys.modules


def proof(tmp_path, **extra):
    identity = adapter.process_record(os.getpid())
    value = dict(capture=identity, owner=identity, server=identity, socket_owner=identity,
        host="127.0.0.1", port=2440, actor_id=42, type_id="vehicle.synthetic", world_id=4, first_frame=10,
        synthetic_only=True, declaration_sha256="e" * 64, **extra)
    receipt = tmp_path / "synthetic_owner_started.json"
    receipt.write_text(json.dumps({key: value[key] for key in ("owner", "server", "socket_owner", "declaration_sha256")}))
    value.update(owner_receipt_path=str(receipt), owner_receipt_sha256=adapter.digest(receipt))
    return value


def parent_identity():
    return 10, 4, 42, "vehicle.synthetic"


def assert_joined_generations(receipt):
    # HH_261005 - Independently reread every retained synthetic PID after wait; a reused foreign PID is not mistaken for our child.
    assert receipt["all_owned_children_joined"]
    for child in receipt["owned_children"].values():
        assert child["returncode"] is not None
        actual = adapter.process_record(child["pid"])
        assert actual is None or actual["start_ticks"] != child["creation_core"]["start_ticks"]


def test_creation_core_waits_for_exact_exec_argv_before_admitting(monkeypatch):
    core = dict(pid=51, start_ticks=91, ppid=os.getpid(), pgid=os.getpgrp())
    argv = ["/usr/bin/python3", "fixed_worker.py", "7"]
    observations = iter((dict(core, state="R", argv=[]), dict(core, state="S", argv=argv), dict(core, state="R", argv=argv)))
    monkeypatch.setattr(adapter, "process_record", lambda pid: next(observations))
    monkeypatch.setattr(adapter.time, "sleep", lambda seconds: None)
    assert adapter.admit_child(core, argv, time.monotonic() + 1) == dict(core, argv=argv)


@pytest.mark.parametrize("field", adapter.CHILD_CORE)
def test_exec_transition_cannot_change_creation_generation_or_ancestry(monkeypatch, field):
    core = dict(pid=51, start_ticks=91, ppid=os.getpid(), pgid=os.getpgrp())
    actual = dict(core, state="R", argv=["/usr/bin/python3", "fixed_worker.py", "7"])
    actual[field] += 1
    monkeypatch.setattr(adapter, "process_record", lambda pid: actual)
    with pytest.raises(RuntimeError, match="generation/ancestry"):
        adapter.admit_child(core, actual["argv"], time.monotonic() + 1)


def test_exact_exec_argv_after_startup_deadline_is_not_admitted(monkeypatch):
    monkeypatch.setattr(adapter, "process_record", lambda pid: pytest.fail("expired admission must not inspect or admit a child"))
    with pytest.raises(RuntimeError, match="readiness deadline"):
        adapter.admit_child(dict(pid=51), [], time.monotonic() - 1)


def test_unadmitted_real_fresh_fake_child_is_closed_joined_and_receipted(tmp_path, monkeypatch):
    def denied(*args):
        raise RuntimeError("synthetic pre-admission failure")
    monkeypatch.setattr(adapter, "admit_child", denied)
    with pytest.raises(adapter.WheelReadError) as caught:
        adapter.OwnedWheelReader(proof(tmp_path), synthetic=True)
    receipt = caught.value.receipt
    assert_joined_generations(receipt)
    assert len(receipt["owned_children"]) == 1 and receipt["cleanup_errors"]
    assert all(child["admitted_identity"] is None for child in receipt["owned_children"].values())
    assert receipt["lifecycle"][0]["status"] == "FAIL"


def test_pinned_guard_dependency_closure_is_only_preparation_plus_stdlib():
    guard = ast.parse((adapter.ROOT / "tools/e2e/run_owned_ctrack_isolated_probe_20261001.py").read_text())
    preparation = ast.parse((adapter.ROOT / "tools/e2e/prepare_ctrack_isolated_probe_20261001.py").read_text())
    def executed_nodes(node):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            return
        yield node
        for child in ast.iter_child_nodes(node):
            yield from executed_nodes(child)
    for tree in (guard, preparation):
        for node in executed_nodes(tree):
            if isinstance(node, ast.Import):
                assert all(alias.name.split(".")[0] in sys.stdlib_module_names
                    or alias.name == "prepare_ctrack_isolated_probe_20261001" for alias in node.names)
            elif isinstance(node, ast.ImportFrom):
                assert (node.level == 1 and node.module is None and all(alias.name == "prepare_ctrack_isolated_probe_20261001" for alias in node.names)) or (
                    node.level == 0 and node.module.split(".")[0] in sys.stdlib_module_names)
    assert not (adapter.ROOT / "tools/e2e/__init__.py").exists()
    assert len(adapter.sources()) == 8


def test_two_actual_fresh_process_fake_reads_start_before_either_finishes(tmp_path):
    value = proof(tmp_path)
    with adapter.OwnedWheelReader(value, synthetic=True) as reader:
        assert reader.read(10, parent_identity) == (-1.25, 2.5)
        rows = reader.rows[0]["role_results"]
        assert max(row["wheel_call_started_monotonic_ns"] for row in rows.values()) < min(
            row["wheel_call_finished_monotonic_ns"] for row in rows.values())
        assert len({row["worker"]["pid"] for row in rows.values()}) == 2
        assert all(row["worker"]["pid"] != os.getpid() for row in rows.values())
        assert reader.read(11, lambda: (11, 4, 42, "vehicle.synthetic")) == (-1.25, 2.5)
    assert_joined_generations(reader.receipt())
    assert not reader.cleanup_errors
    assert len(reader.rows) == 2 and [row["sequence"] for row in reader.rows] == [1, 2]


@pytest.mark.parametrize("fault", ("wheel_error", "nonfinite", "deadline", "parent_frame", "parent_actor", "parent_epoch"))
def test_every_failed_pair_is_retained_without_fabricated_measurement_and_children_join(tmp_path, fault):
    extra = {}
    if fault == "wheel_error": extra["synthetic_failed_role"] = "FL"
    elif fault == "nonfinite": extra["synthetic_nonfinite_role"] = "FR"
    elif fault == "deadline": extra["synthetic_delay_seconds"] = .15
    reader = adapter.OwnedWheelReader(proof(tmp_path, **extra), synthetic=True)
    identity = parent_identity
    if fault == "parent_frame": identity = lambda: (11, 4, 42, "vehicle.synthetic")
    elif fault == "parent_actor": identity = lambda: (10, 4, 43, "vehicle.synthetic")
    elif fault == "parent_epoch": identity = lambda: (10, 5, 42, "vehicle.synthetic")
    try:
        with pytest.raises(RuntimeError):
            reader.read(10, identity)
        assert len(reader.rows) == 1 and reader.rows[0]["status"] == "FAIL" and reader.rows[0]["error"]
        with pytest.raises(RuntimeError):
            reader.read(10, parent_identity)
    finally:
        reader.close()
    assert_joined_generations(reader.receipt())
    if fault in ("wheel_error", "nonfinite"):
        assert set(reader.rows[0]["role_results"]) == {"FL", "FR"}


@pytest.mark.parametrize("fault", ("pid_generation", "foreign_host", "receipt", "real_mode"))
def test_admission_denies_before_any_process_or_CARLA_connection(tmp_path, fault, monkeypatch):
    value = proof(tmp_path)
    if fault == "pid_generation": value["server"] = dict(value["server"], start_ticks=-1)
    elif fault == "foreign_host": value["host"] = "192.0.2.10"
    elif fault == "receipt": value["owner_receipt_sha256"] = "0" * 64
    calls = []
    monkeypatch.setattr(adapter.subprocess, "Popen", lambda *a, **k: calls.append((a, k)))
    with pytest.raises(RuntimeError):
        adapter.OwnedWheelReader(value, synthetic=fault != "real_mode")
    assert not calls


def test_fixed_real_and_fake_executable_sources_have_no_actuation_or_Torch_calls():
    forbidden = {"tick", "wait_for_tick", "load_world", "reload_world", "spawn_actor", "try_spawn_actor",
        "apply_control", "apply_batch", "apply_batch_sync", "set_autopilot", "apply_settings", "set_weather"}
    for path in (adapter.HERE, adapter.REAL_WORKER, adapter.SYNTHETIC_WORKER):
        tree = ast.parse(path.read_text())
        assert not any(isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
            and node.func.attr in forbidden for node in ast.walk(tree))
        assert not any(isinstance(node, ast.Import) and any(alias.name == "torch" for alias in node.names) for node in ast.walk(tree))
    fake_tree = ast.parse(adapter.SYNTHETIC_WORKER.read_text())
    assert not any(isinstance(node, ast.Import) and any(alias.name == "carla" for alias in node.names) for node in ast.walk(fake_tree))
    assert adapter.REQUEST_SECONDS == .1 and adapter.RPC_SECONDS == .05 and adapter.MAX_REQUESTS == 2200


@pytest.mark.parametrize("field", ("token", "role", "sequence", "frame"))
def test_cross_role_or_stale_protocol_cannot_return_a_pair(tmp_path, monkeypatch, field):
    reader = adapter.OwnedWheelReader(proof(tmp_path), synthetic=True)
    original = adapter.send
    def amended(connection, message):
        if message.get("kind") == "read" and message["role"] == "FL":
            message = dict(message)
            message[field] = {"token": "wrong", "role": "FR", "sequence": 2, "frame": 11}[field]
        original(connection, message)
    monkeypatch.setattr(adapter, "send", amended)
    try:
        with pytest.raises(RuntimeError): reader.read(10, parent_identity)
        assert reader.rows[0]["status"] == "FAIL" and set(reader.rows[0]["role_requests"]) == {"FL", "FR"}
    finally:
        reader.close()
    assert_joined_generations(reader.receipt())


def test_no_skipped_original_native_sample_and_failure_row_retained(tmp_path):
    reader = adapter.OwnedWheelReader(proof(tmp_path), synthetic=True)
    try:
        assert reader.read(10, parent_identity) == (-1.25, 2.5)
        with pytest.raises(RuntimeError): reader.read(12, lambda: (12, 4, 42, "vehicle.synthetic"))
    finally:
        reader.close()
    assert [row["status"] for row in reader.rows] == ["PASS", "FAIL"]
    assert_joined_generations(reader.receipt())


def test_startup_failure_carries_all_cleanup_receipts_no_orphan(tmp_path):
    with pytest.raises(adapter.WheelReadError) as caught:
        adapter.OwnedWheelReader(proof(tmp_path, synthetic_fail_initialization=True), synthetic=True)
    receipt = caught.value.receipt
    assert_joined_generations(receipt)
    assert receipt["cleanup_errors"]
    assert receipt["lifecycle"][0]["status"] == "FAIL" and receipt["all_requests_including_failures"] == []


@pytest.mark.parametrize("fault", ("actor", "type", "epoch", "future_frame", "frame_changed"))
def test_real_read_method_guards_with_only_fake_Client_world_actor_no_CARLA_import(fault):
    spec = importlib.util.spec_from_file_location("test_real_wheel_source", adapter.REAL_WORKER)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    class FakeVehicle:
        id, type_id = 42, "vehicle.synthetic"
        def get_wheel_steer_angle(self, role):
            if fault == "frame_changed": world.frame = 11
            return 1.25
    class FakeWorld:
        id, frame = 4, 10
        def get_snapshot(self): return SimpleNamespace(frame=self.frame)
        def get_actor(self, ident): return actor
    world, actor = FakeWorld(), FakeVehicle()
    if fault == "actor": actor.id = 43
    elif fault == "type": actor.type_id = "vehicle.other"
    elif fault == "epoch": world.id = 5
    elif fault == "future_frame": world.frame = 11
    backend = module.CarlaWheelBackend.__new__(module.CarlaWheelBackend)
    backend.proof = dict(world_id=4, actor_id=42, type_id="vehicle.synthetic")
    backend.role = "FL"
    backend.carla = SimpleNamespace(Vehicle=FakeVehicle, VehicleWheelLocation=SimpleNamespace(FL_Wheel=0))
    backend.client = SimpleNamespace(get_world=lambda: world)
    with pytest.raises(RuntimeError):
        backend.read(dict(frame=10, deadline_monotonic_ns=10**30), .05)
