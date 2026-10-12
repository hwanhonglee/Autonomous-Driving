"""HH_261005 - Check fresh stationary ownership, immutable admission and bounded quota without CARLA."""

import hashlib
import json
from pathlib import Path

import pytest

from scripts.e2e import run_owned_town03_stationary_wheel_rpc_v1_20261005 as owner

GPU = "GPU-11111111-2222-3333-4444-555555555555"


@pytest.fixture
def contract(monkeypatch):
    monkeypatch.setattr(owner, "source_pins", lambda: {"synthetic.py": "a" * 64})
    return owner.fixed_contract(GPU)


def test_explicit_non_driving_contract(contract):
    assert contract["measured_frames"] == 30
    assert contract["variant_calls"] == 60 and contract["wheel_rpc_calls"] == 120
    assert contract["physics_hz"] == 20 and contract["fixed_delta_seconds"] == .05
    assert contract["bootstrap_native_ticks"] == 1
    assert contract["vehicle_blueprint"] == "vehicle.toyota.prius"
    assert contract["actorphysics_disabled"] is True
    assert contract["moving_physics_equivalence_claimed"] is False
    assert all(contract[key] is False for key in ("images", "NN", "weights", "train", "test", "teacher_drive", "retry", "world_reload"))
    assert contract["formal_main_domains_approved"] == contract["formal_subfunctions_approved"] == 0
    assert contract["first_cold_calls_retained"] is True
    assert contract["server_command"][0] == str(owner.BINARY)
    assert "-carla-port=2440" in contract["server_command"]
    assert "-EngineINI=" + str(owner.OUTPUT / "Town03.Engine.ini") in contract["server_command"]
    assert "--plan-sha256" in contract["worker_command"]
    assert "--launch-authority-sha256" in contract["worker_command"]
    assert contract["worker_command"][5] == contract["owner_command"][5] == "--"
    assert "torch" not in owner.__dict__


@pytest.mark.parametrize("uuid", [None, "0", GPU + "x", "GPU-123", False])
def test_uuid_must_be_exact(uuid):
    with pytest.raises(RuntimeError):
        owner.fixed_contract(uuid)


def test_budget_exact_floor_and_reserve(tmp_path):
    needed = owner.FLOOR + owner.ATTEMPT + owner.SHARED
    assert needed == 6241124352
    assert owner.check_budget(tmp_path, startup=True, free=needed)["used_bytes"] == 0
    with pytest.raises(RuntimeError):
        owner.check_budget(tmp_path, startup=True, free=needed - 1)
    closing = owner.FLOOR + owner.SHARED + owner.CLOSING
    assert owner.check_budget(tmp_path, free=closing)["available_bytes"] == closing
    with pytest.raises(RuntimeError):
        owner.check_budget(tmp_path, free=closing - 1)
    with pytest.raises(RuntimeError):
        owner.check_budget(tmp_path, free=True)


def test_whole_walk_preserves_file_and_symlink_guards(tmp_path):
    (tmp_path / "nested").mkdir()
    (tmp_path / "nested/file").write_bytes(b"abc")
    assert owner.directory_bytes(tmp_path) == 3
    (tmp_path / "alias").symlink_to(tmp_path / "nested/file")
    with pytest.raises(RuntimeError):
        owner.directory_bytes(tmp_path)


def test_special_entry_rejected(tmp_path):
    import os
    os.mkfifo(tmp_path / "fifo")
    with pytest.raises(RuntimeError):
        owner.directory_bytes(tmp_path)


def test_quota_keeps_closing_reserve(monkeypatch, tmp_path):
    monkeypatch.setattr(owner, "directory_bytes", lambda path: owner.ATTEMPT - owner.CLOSING)
    owner.check_budget(tmp_path, free=owner.FLOOR + owner.SHARED + owner.CLOSING)
    monkeypatch.setattr(owner, "directory_bytes", lambda path: owner.ATTEMPT - owner.CLOSING + 1)
    with pytest.raises(RuntimeError):
        owner.check_budget(tmp_path, free=owner.FLOOR + owner.ATTEMPT + owner.SHARED)


def test_create_only_and_json_faults(tmp_path):
    path = tmp_path / "receipt.json"
    owner.write_new(path, {"one": 1})
    assert owner.read_json(path) == {"one": 1}
    with pytest.raises(FileExistsError):
        owner.write_new(path, {})
    alias = tmp_path / "alias.json"
    alias.symlink_to(path)
    with pytest.raises(RuntimeError):
        owner.read_json(alias)
    for payload in ('{"a":1,"a":2}', '{"a":NaN}'):
        bad = tmp_path / (hashlib.sha256(payload.encode()).hexdigest() + ".json")
        bad.write_text(payload)
        with pytest.raises(RuntimeError):
            owner.read_json(bad)
    with pytest.raises(RuntimeError):
        owner.read_json(path, limit=1)


@pytest.mark.parametrize("argv", [[], ["--plan-sha256"], ["--plan-sha256", "a" * 64, "--plan-sha256", "a" * 64], ["--plan-sha256", "bad"]])
def test_cli_sha_is_not_a_self_claim(argv):
    with pytest.raises(RuntimeError):
        owner.argv_value(argv, "--plan-sha256")


def test_cli_substitution(contract):
    actual = owner.substituted(contract["worker_command"], "c" * 64, "d" * 64)
    assert owner.argv_value(actual, "--plan-sha256") == "c" * 64
    assert owner.argv_value(actual, "--launch-authority-sha256") == "d" * 64
    assert owner.PLAN_TOKEN not in actual and owner.AUTH_TOKEN not in actual


@pytest.fixture
def launch(tmp_path, monkeypatch, contract):
    path = tmp_path / "ROOT_launch_authority.json"
    monkeypatch.setattr(owner, "LAUNCH_AUTHORITY", path)
    body = dict(schema=owner.SCHEMA + ".ROOT_launch_authority", explicit_owned_stationary_launch_approved=True,
                declaration_path=str(owner.PLAN), declaration_sha256="c" * 64,
                source_sha256=contract["source_sha256"], asset_sha256=contract["asset_sha256"])
    owner.write_new(path, body)
    return contract, path, body


def test_launch_authority_exact_scoped_receipt(launch):
    plan, path, body = launch
    assert owner.verify_launch_authority(plan, "c" * 64, owner.digest(path)) == body
    with pytest.raises(RuntimeError):
        owner.verify_launch_authority(plan, "c" * 64, "e" * 64)


@pytest.mark.parametrize("key,value", [("schema", "fake"), ("explicit_owned_stationary_launch_approved", False),
                                    ("declaration_path", "/tmp/other_plan.json"), ("declaration_sha256", "e" * 64),
                                    ("source_sha256", {}), ("asset_sha256", {})])
def test_launch_guard_does_not_accept_partial_approval(launch, key, value):
    plan, path, body = launch
    body[key] = value
    # HH_261005 - This temporary synthetic authority is outside the real fixed ROOT path and never starts a process.
    path.write_text(json.dumps(body))
    with pytest.raises(RuntimeError):
        owner.verify_launch_authority(plan, "c" * 64, owner.digest(path))


def test_gpu_identity_and_idle_guards(monkeypatch, contract):
    class Result:
        def __init__(self, stdout):
            self.stdout = stdout
    outputs = iter(["0, " + GPU + ", 512, 30\n", "", "synthetic NVIDIA full graphics inventory"])
    monkeypatch.setattr(owner.subprocess, "run", lambda *args, **kwargs: Result(next(outputs)))
    monkeypatch.setattr(owner.guards, "all_processes", lambda: [])
    assert owner.gpu_available(contract)["gpu_uuid"] == GPU
    for inventory, apps in (("0, " + GPU + ", 513, 0\n", ""), ("0, " + GPU + ", 0, 31\n", ""),
                            ("0, GPU-foreign, 0, 0\n", ""), ("0, " + GPU + ", 0, 0\n", GPU + ", 123, other-job\n")):
        outputs = iter([inventory, apps])
        with pytest.raises(RuntimeError):
            owner.gpu_available(contract)


def test_no_other_carla_never_kills(monkeypatch):
    monkeypatch.setattr(owner.guards, "all_processes", lambda: [{"argv": [str(owner.BINARY)], "pid": 123}])
    with pytest.raises(RuntimeError):
        owner.no_other_carla()


def test_signal_refuses_generation_reuse(monkeypatch):
    class Child:
        pid = 123
        def poll(self):
            return None
    identity = dict(pid=123, ppid=1, pgid=123, start_ticks=55, argv=["sealed"])
    monkeypatch.setattr(owner, "process_identity", lambda pid: dict(identity, start_ticks=56))
    monkeypatch.setattr(owner.os, "killpg", lambda *args: pytest.fail("must not signal reused PID"))
    with pytest.raises(RuntimeError):
        owner.stop_owned(Child(), identity)


def test_exec_admission_accepts_only_same_creation_generation(monkeypatch):
    class Child:
        pid = 123
        def poll(self):
            return None
    core = dict(pid=123, ppid=owner.os.getpid(), pgid=123, start_ticks=55)
    rows = iter([dict(core, state="R", argv=[]), dict(core, state="R", argv=["preexec-parent"]),
                 dict(core, state="S", argv=["sealed-child"])])
    monkeypatch.setattr(owner.guards, "process_record", lambda pid: next(rows))
    monkeypatch.setattr(owner.os, "getsid", lambda pid: pid)
    monkeypatch.setattr(owner.time, "sleep", lambda delay: None)
    identities = {}
    result = owner.admit_owned_child(Child(), ["sealed-child"], identities, "capture")
    assert result == dict(core, argv=["sealed-child"])
    assert identities["capture"] == result


def test_preexec_failure_keeps_core_and_can_clean_only_that_core(monkeypatch):
    class Child:
        pid = 123
        returncode = None
        def poll(self):
            return self.returncode
        def wait(self, timeout):
            self.returncode = -15
            return self.returncode
    core = dict(pid=123, ppid=owner.os.getpid(), pgid=123, start_ticks=55)
    row = dict(core, state="R", argv=["still-before-exec"])
    monkeypatch.setattr(owner.guards, "process_record", lambda pid: row)
    monkeypatch.setattr(owner.os, "getsid", lambda pid: pid)
    clocks = iter([0., 0., 6.])
    monkeypatch.setattr(owner.time, "monotonic", lambda: next(clocks))
    monkeypatch.setattr(owner.time, "sleep", lambda delay: None)
    identities, child = {}, Child()
    with pytest.raises(RuntimeError):
        owner.admit_owned_child(child, ["sealed-child"], identities, "capture")
    assert identities["capture"] == core
    signals = []
    monkeypatch.setattr(owner.os, "killpg", lambda pid, sig: signals.append((pid, sig)))
    monkeypatch.setattr(owner.guards, "all_processes", lambda: [])
    owner.stop_owned(child, identities["capture"])
    assert signals == [(123, owner.signal.SIGTERM)]


@pytest.mark.parametrize("field,value", [("start_ticks", 56), ("ppid", 99), ("pgid", 124)])
def test_exec_admission_rejects_reuse_and_retains_original_core(monkeypatch, field, value):
    class Child:
        pid = 123
        def poll(self):
            return None
    core = dict(pid=123, ppid=owner.os.getpid(), pgid=123, start_ticks=55)
    rows = iter([dict(core, state="R", argv=[]), dict(core, **{field: value}, state="R", argv=["sealed-child"])])
    monkeypatch.setattr(owner.guards, "process_record", lambda pid: next(rows))
    monkeypatch.setattr(owner.os, "getsid", lambda pid: pid)
    identities = {}
    with pytest.raises(RuntimeError):
        owner.admit_owned_child(Child(), ["sealed-child"], identities, "capture")
    assert identities["capture"] == core


def test_unadmitted_wrong_generation_is_never_signalled(monkeypatch):
    class Child:
        pid = 123
        def poll(self):
            return None
    core = dict(pid=123, ppid=owner.os.getpid(), pgid=123, start_ticks=55)
    monkeypatch.setattr(owner.guards, "process_record", lambda pid: dict(core, start_ticks=56, state="R", argv=[]))
    monkeypatch.setattr(owner.os, "killpg", lambda *args: pytest.fail("unadmitted reused core must not be signalled"))
    with pytest.raises(RuntimeError):
        owner.stop_owned(Child(), core)


def test_direct_engine_ini_is_small_and_only_town03():
    text = owner.engine_ini()
    assert text.count("=/Game/Carla/Maps/Town03.Town03") == 4
    assert len(text.encode()) < 1024


def test_declared_source_pins_include_actual_measurement_and_real_v2():
    import ast
    tree = ast.parse(Path(owner.__file__).read_text())
    names = {node.value for node in ast.walk(tree) if isinstance(node, ast.Constant) and type(node.value) is str}
    assert "tools/e2e/collect_carla_vad_expert.py" in names
    assert "tools/e2e/collect_owned_town03_stationary_wheel_rpc_v1_20261005.py" in names
    assert owner.COLLECTOR_SHA256 == "a41eac9019c644fec03396a50645fa021b63230a4ca86a664e8044dfd170ae08"


def test_actual_and_archive_guard_not_only_an_unused_source_alias(tmp_path, monkeypatch):
    root, carla = tmp_path / "workspace", tmp_path / "carla"
    root.mkdir(); carla.mkdir()
    source, asset = root / "synthetic.py", carla / "synthetic.egg"
    source.write_bytes(b"synthetic scalar source\n")
    asset.write_bytes(b"synthetic CARLA asset, never imported\n")
    output = root / "fresh_output"; output.mkdir()
    plan_path, authority = root / "synthetic_plan.json", root / "ROOT_launch_authority.json"
    plan = dict(source_sha256={"synthetic.py": owner.digest(source)}, asset_sha256={str(asset): owner.digest(asset)})
    owner.write_new(plan_path, plan); owner.write_new(authority, {"synthetic_not_actual_authority": True})
    expected, launch_sha = owner.digest(plan_path), owner.digest(authority)
    for name, value in (("ROOT", root), ("CARLA_ROOT", carla), ("OUTPUT", output), ("PLAN", plan_path),
                        ("LAUNCH_AUTHORITY", authority)):
        monkeypatch.setattr(owner, name, value)
    records = owner.archive_sources(plan, expected, launch_sha)
    assert len(records) == 4
    assert owner.verify_archived_sources(plan, expected, launch_sha) is True
    archived_asset = output / "source_external/synthetic.egg"
    archived_asset.write_bytes(b"corrupt retained synthetic archive")
    with pytest.raises(RuntimeError):
        owner.verify_archived_sources(plan, expected, launch_sha)


def test_plan_exact_types_are_sealed_not_python_bool_equality(tmp_path, monkeypatch, contract):
    plan_path = tmp_path / "synthetic_plan.json"
    monkeypatch.setattr(owner, "PLAN", plan_path)
    monkeypatch.setattr(owner, "ASSET_SHA256", {})
    original = dict(contract, asset_sha256={})
    monkeypatch.setattr(owner, "fixed_contract", lambda uuid: original)
    owner.write_new(plan_path, original)
    assert owner.verify_plan(owner.digest(plan_path)) == original
    changed = dict(original, gpu_index=False)
    plan_path.write_text(json.dumps(changed))
    assert changed == original
    with pytest.raises(RuntimeError):
        owner.verify_plan(owner.digest(plan_path))


@pytest.mark.parametrize("module", [owner.MODULE, owner.CAPTURE_MODULE])
def test_genuine_bootstrap_help_smoke_only(module):
    # HH_261005 - Exercise the actual argparse/REMAINDER route with --help; no declaration, admission or CARLA connection can run.
    import subprocess
    command = ["/usr/bin/python3", "-B", str(owner.ROOT / "tools/workspace_python.py"), "-m", module, "--", "--help"]
    result = subprocess.run(command, cwd=owner.ROOT, env=dict(owner.os.environ, CUDA_VISIBLE_DEVICES="", PYTHONDONTWRITEBYTECODE="1"),
                            text=True, capture_output=True, timeout=5)
    assert result.returncode == 0, result.stderr
    assert "--plan-sha256" in result.stdout and "--launch-authority-sha256" in result.stdout
