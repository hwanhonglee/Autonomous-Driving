"""HH_261005 - Check the fresh admission-state logging revision, frozen predecessors and unchanged stationary bounds without CARLA."""

import hashlib
import json
from pathlib import Path

import pytest

from scripts.e2e import run_owned_town03_stationary_wheel_rpc_v4_20261005 as owner

GPU = "GPU-11111111-2222-3333-4444-555555555555"


@pytest.fixture
def contract(monkeypatch):
    monkeypatch.setattr(owner, "source_pins", lambda: {"synthetic.py": "a" * 64})
    monkeypatch.setattr(owner, "native_runtime_input_contract", lambda: {"synthetic_only": True, "cache_path": "/synthetic/cache"})
    return owner.fixed_contract(GPU)


def test_explicit_non_driving_contract(contract):
    assert contract["measured_frames"] == 30
    assert contract["admission_snapshot_before_stationary_guard"] is True and contract["physical_setup_unchanged_from_V3"] is True
    assert contract["velocity_reset_calls"] == 0 and contract["prior_V3_admission_velocity_values_remain_unknown"] is True
    assert contract["variant_calls"] == 60 and contract["wheel_rpc_calls"] == 120
    assert contract["physics_hz"] == 20 and contract["fixed_delta_seconds"] == .05
    assert contract["bootstrap_native_ticks"] == 1
    assert contract["vehicle_blueprint"] == "vehicle.toyota.prius"
    assert contract["actorphysics_disabled"] is True
    assert contract["moving_physics_equivalence_claimed"] is False
    assert all(contract[key] is False for key in ("images", "NN", "weights", "train", "test", "teacher_drive", "retry"))
    assert contract["formal_main_domains_approved"] == contract["formal_subfunctions_approved"] == 0
    assert contract["world_reload"] is True and contract["initialize_owned_map_if_needed"] == "Town03"
    assert contract["maximum_load_world_calls"] == 1 and contract["before_any_actor_or_worker"] is True
    assert contract["worker_definition"] == "wheel_read_worker; capture_setup_process_exists"
    assert contract["initialization_RPC_timeout_seconds"] == 30. and contract["normal_stationary_RPC_timeout_seconds"] == .5
    assert contract["no_initial_non_Town03_payload_access"] is True
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
                source_sha256=contract["source_sha256"], asset_sha256=contract["asset_sha256"],
                initialize_owned_map_if_needed="Town03", maximum_load_world_calls=1, before_any_actor_or_worker=True,
                worker_definition="wheel_read_worker; capture_setup_process_exists",
                initialization_RPC_timeout_seconds=30., normal_stationary_RPC_timeout_seconds=.5)
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
    assert "tools/e2e/collect_owned_town03_stationary_wheel_rpc_v4_20261005.py" in names
    assert owner.COLLECTOR_SHA256 == "a41eac9019c644fec03396a50645fa021b63230a4ca86a664e8044dfd170ae08"


def test_actual_and_archive_guard_not_only_an_unused_source_alias(tmp_path, monkeypatch):
    root, carla = tmp_path / "workspace", tmp_path / "carla"
    root.mkdir(); carla.mkdir()
    source, asset = root / "synthetic.py", carla / "synthetic.egg"
    source.write_bytes(b"synthetic scalar source\n")
    asset.write_bytes(b"synthetic CARLA asset, never imported\n")
    output = root / "fresh_output"; output.mkdir()
    plan_path, authority = root / "synthetic_plan.json", root / "ROOT_launch_authority.json"
    plan = dict(source_sha256={"synthetic.py": owner.digest(source)}, asset_sha256={str(asset): owner.digest(asset)},
                native_runtime_input={"producer_provenance_sha256": {}, "synthetic_only": True},
                asset_archive_paths={str(asset): str(output / "source_external/synthetic.egg")},
                reused_SDK_archive_generations={}, metadata_evidence_sha256={})
    monkeypatch.setattr(owner, "verify_sdk_reused_inputs", lambda value: True)
    monkeypatch.setattr(owner, "verify_native_runtime_input", lambda value: {"synthetic_only": True})
    owner.write_new(plan_path, plan); owner.write_new(authority, {"synthetic_not_actual_authority": True})
    expected, launch_sha = owner.digest(plan_path), owner.digest(authority)
    for name, value in (("ROOT", root), ("CARLA_ROOT", carla), ("OUTPUT", output), ("PLAN", plan_path),
                        ("LAUNCH_AUTHORITY", authority)):
        monkeypatch.setattr(owner, name, value)
    records = owner.archive_sources(plan, expected, launch_sha)
    assert len(records) == 5
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
    monkeypatch.setattr(owner, "verify_sdk_reused_inputs", lambda plan: True)
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


def test_exact_unchanged_guard_functions_are_static_V1_AST():
    # HH_261005 - Compare the real copied function bodies, not aliases that the actual lifecycle never calls.
    import ast
    original = ast.parse((owner.ROOT / "tools/e2e/run_owned_town03_stationary_wheel_rpc_v1_20261005.py").read_text())
    revised = ast.parse(Path(owner.__file__).read_text())
    bodies = lambda tree: {node.name: ast.dump(node, include_attributes=False) for node in tree.body if isinstance(node, ast.FunctionDef)}
    old, new = bodies(original), bodies(revised)
    changed = {"source_pins", "fixed_contract", "environment", "archive_sources", "verify_archived_sources", "preflight", "execute", "verify_plan", "verify_launch_authority", "main"}
    assert all(old[name] == new[name] for name in set(old) - changed)
    assert owner.file_limit.__code__.co_filename == owner.__file__
    assert owner.execute.__globals__ is owner.__dict__
    assert owner.verify_plan.__globals__["fixed_contract"] is owner.fixed_contract
    assert owner.execute.__globals__["environment"] is owner.environment
    assert owner.execute.__globals__["file_limit"] is owner.file_limit
    assert owner.source_pins().keys() >= owner.FROZEN_V1_PINS.keys() | owner.PROBE_SOURCE_PINS.keys()
    assert len(owner.source_pins()) == 27


def test_private_runtime_input_is_explicit_and_original_limits_unchanged(contract):
    assert contract["native_runtime_input_copied"] is False
    assert contract["working_engine_ini_is_mutable"] is True
    assert contract["pristine_engine_ini_retained"] is True
    assert contract["original_16MiB_child_file_limit_unchanged"] is True
    assert owner.environment(contract)["PYTHON_EGG_CACHE"] == contract["native_runtime_input"]["cache_path"]
    assert contract["individual_log_file_limit_bytes"] == owner.CLOSING == 16777216


@pytest.fixture
def tiny_native_input(tmp_path, monkeypatch):
    # HH_261005 - Three synthetic bytes exercise provenance and truncation denials; no real native library is copied or imported.
    import os
    import time
    import zipfile
    from types import SimpleNamespace as NS
    egg, native = tmp_path / "synthetic.egg", tmp_path / "cache/synthetic.egg-tmp/carla/native.so"
    native.parent.mkdir(parents=True)
    content = b"abc"
    member = owner.NATIVE_MEMBER
    info = zipfile.ZipInfo(member, (2025, 5, 20, 20, 15, 58))
    with zipfile.ZipFile(egg, "w") as archive:
        archive.writestr(info, content)
    native.write_bytes(content)
    timestamp = time.mktime(info.date_time + (0, 0, -1))
    os.utime(native, (timestamp, timestamp))
    output, plans = tmp_path / "producer", tmp_path / "plans"
    output.mkdir(); plans.mkdir()
    probe = NS(OUTPUT=output, PRIVATE_CACHE=tmp_path / "cache", PRIVATE_NATIVE=native,
               PLAN=plans / "plan.json", AUTHORITY=plans / "authority.json", SCHEMA="synthetic.import_only")
    plan = dict(source_sha256={"synthetic.py": "a" * 64}, asset_sha256={str(egg): owner.digest(egg)})
    probe.contract = lambda: plan
    owner.write_new(probe.PLAN, plan)
    plan_sha = owner.digest(probe.PLAN)
    authority = dict(schema=probe.SCHEMA + ".ROOT_execution_authority", explicit_import_only_execution_approved=True,
                     plan_path=str(probe.PLAN), plan_sha256=plan_sha, source_sha256=plan["source_sha256"], asset_sha256=plan["asset_sha256"])
    owner.write_new(probe.AUTHORITY, authority)
    authority_sha = owner.digest(probe.AUTHORITY)
    result = dict(schema=probe.SCHEMA + ".result", status="IMPORT_ONLY_COMPLETE_NOT_DRIVING", exit_code=0,
                  native_endpoint_calls_verified_absent=True,
                  trace_audit=dict(actual_trace_complete=True, native_endpoint_calls_observed=False),
                  actual_child_argv=["synthetic_import_only", "--plan-sha256", plan_sha, "--authority-sha256", authority_sha])
    owner.write_new(output / "result.json", result)
    for name in ("strace.log", "stdout.log", "stderr.log"):
        (output / name).write_bytes(b"synthetic bounded trace; never an actual native import")
    monkeypatch.setattr(owner, "_probe", lambda: probe)
    monkeypatch.setattr(owner, "EGG", egg)
    monkeypatch.setattr(owner, "NATIVE_SIZE", len(content))
    monkeypatch.setattr(owner, "NATIVE_SHA256", hashlib.sha256(content).hexdigest())
    return NS(probe=probe, result=result, native=native, egg=egg, timestamp=timestamp)


def test_native_input_exact_provenance_generation_and_bytes(tiny_native_input):
    input_contract = owner.native_runtime_input_contract()
    receipt = owner.verify_native_runtime_input({"native_runtime_input": input_contract})
    assert input_contract["extra_native_copy"] is False and len(input_contract["producer_provenance_sha256"]) == 6
    assert receipt["size_bytes"] == 3 and receipt["full_private_input_verified"]
    assert receipt["native_generation"] == input_contract["native_generation"]
    assert receipt["original_file_limit_bytes"] == 16777216


@pytest.mark.parametrize("fault", ["truncated", "same_size_corruption", "wrong_mtime", "native_alias", "result_alias", "failed_result", "missing_result", "producer_plan", "producer_authority", "wrong_argv", "incomplete_trace", "network", "result_changed", "source_changed"])
def test_native_input_denies_before_any_import(tiny_native_input, monkeypatch, fault):
    import os
    value = tiny_native_input
    plan = {"native_runtime_input": owner.native_runtime_input_contract()}
    result_path = value.probe.OUTPUT / "result.json"
    if fault == "truncated": value.native.write_bytes(b"ab")
    elif fault == "same_size_corruption": value.native.write_bytes(b"abd"); os.utime(value.native, (value.timestamp, value.timestamp))
    elif fault == "wrong_mtime": os.utime(value.native, (value.timestamp + 1, value.timestamp + 1))
    elif fault in ("native_alias", "result_alias"):
        original = value.native if fault == "native_alias" else result_path
        preserved = original.with_name(original.name + ".synthetic_preserved")
        original.rename(preserved); original.symlink_to(preserved)
    elif fault == "missing_result": result_path.rename(result_path.with_suffix(".synthetic_preserved"))
    elif fault == "producer_plan": value.probe.PLAN.write_text('{}')
    elif fault == "producer_authority": value.probe.AUTHORITY.write_text('{}')
    elif fault == "source_changed": monkeypatch.setattr(value.probe, "contract", lambda: {})
    else:
        result = dict(value.result)
        if fault == "failed_result": result["status"] = "FAIL_RETAINED"
        elif fault == "wrong_argv": result["actual_child_argv"] = []
        elif fault == "incomplete_trace": result["trace_audit"] = {"actual_trace_complete": False, "native_endpoint_calls_observed": False}
        elif fault == "network": result["trace_audit"] = {"actual_trace_complete": True, "native_endpoint_calls_observed": True}
        elif fault == "result_changed": result["synthetic_new_metadata"] = True
        result_path.write_text(json.dumps(result))
    with pytest.raises((RuntimeError, FileNotFoundError)):
        owner.verify_native_runtime_input(plan)


def test_native_exact_bytes_not_only_generations(tiny_native_input, monkeypatch):
    expected = owner.native_runtime_input_contract()
    monkeypatch.setattr(owner, "digest", lambda path: "0" * 64 if Path(path) == tiny_native_input.native else hashlib.sha256(Path(path).read_bytes()).hexdigest())
    with pytest.raises(RuntimeError, match="bytes differ"):
        owner.verify_native_runtime_input({"native_runtime_input": expected})


def test_native_single_inode_no_hardlink(tiny_native_input):
    import os
    os.link(tiny_native_input.native, tiny_native_input.native.with_name("synthetic_hardlink"))
    with pytest.raises(RuntimeError, match="single regular inode"):
        owner.native_runtime_input_contract()


@pytest.fixture
def synthetic_ini(tmp_path, monkeypatch):
    monkeypatch.setattr(owner, "OUTPUT", tmp_path)
    path = tmp_path / "Town03.Engine.ini"
    path.write_text(owner.engine_ini())
    plan = {"generated_engine_ini_sha256": owner.digest(path)}
    owner.preserve_engine_ini_input(plan)
    return tmp_path, path, plan


def test_mutable_working_ini_keeps_pristine_input_and_actual_map(synthetic_ini):
    output, working, plan = synthetic_ini
    working.write_text(owner.engine_ini() + "\n[UnrealExpandedSynthetic]\nUnrelated=1\n")
    owner.write_new(output / "capture_result.json", {"actual_map_identity": "Town03"})
    result = owner.engine_ini_postcheck(plan, require_actual_map=True)
    assert result["working_INI_mutated"] is True
    assert result["pristine_sha256"] == plan["generated_engine_ini_sha256"]
    assert result["working_sha256"] == owner.digest(working) and result["actual_map_identity"] == "Town03"
    assert result["shared_INI_overwritten"] is False
    with pytest.raises(FileExistsError): owner.engine_ini_postcheck(plan, require_actual_map=True)


@pytest.mark.parametrize("fault", ["missing_key", "wrong_map", "duplicate_key", "wrong_section", "pristine_changed", "no_actual_map", "foreign_actual_map"])
def test_ini_semantic_or_input_corruption_is_not_excused_as_Unreal_mutation(synthetic_ini, fault):
    output, working, plan = synthetic_ini
    text = working.read_text()
    if fault == "missing_key": working.write_text(text.replace("TransitionMap=/Game/Carla/Maps/Town03.Town03\n", ""))
    elif fault == "wrong_map": working.write_text(text.replace("GameDefaultMap=/Game/Carla/Maps/Town03.Town03", "GameDefaultMap=/Game/Carla/Maps/Town07.Town07"))
    elif fault == "duplicate_key": working.write_text(text + "GameDefaultMap=/Game/Carla/Maps/Town03.Town03\n")
    elif fault == "wrong_section": working.write_text(text.replace("[/Script/EngineSettings.GameMapsSettings]", "[syntheticWrongSection]"))
    elif fault == "pristine_changed": (output / "source_generated/Town03.Engine.pristine.ini").write_bytes(b"synthetic corruption")
    elif fault == "foreign_actual_map": owner.write_new(output / "capture_result.json", {"actual_map_identity": "Town07"})
    with pytest.raises(RuntimeError): owner.engine_ini_postcheck(plan, require_actual_map=True)


def test_failed_pre_world_capture_keeps_unknown_actual_map_not_fabricated(synthetic_ini):
    _, _, plan = synthetic_ini
    result = owner.engine_ini_postcheck(plan, require_actual_map=False)
    assert result["actual_map_identity"] is None and result["capture_result_sha256"] is None


@pytest.mark.parametrize("key,value", [("initialize_owned_map_if_needed", "Town07"), ("maximum_load_world_calls", 2),
                                      ("maximum_load_world_calls", True), ("before_any_actor_or_worker", False),
                                      ("worker_definition", "all workers absent"),
                                      ("initialization_RPC_timeout_seconds", None), ("initialization_RPC_timeout_seconds", 60.),
                                      ("initialization_RPC_timeout_seconds", True), ("normal_stationary_RPC_timeout_seconds", None),
                                      ("normal_stationary_RPC_timeout_seconds", 30.), ("normal_stationary_RPC_timeout_seconds", True)])
def test_one_owned_map_initialization_is_not_self_authorized(launch, key, value):
    plan, path, body = launch
    body[key] = value
    path.write_text(json.dumps(body))
    with pytest.raises(RuntimeError, match="ROOT approval"):
        owner.verify_launch_authority(plan, "c" * 64, owner.digest(path))


def test_all_frozen_V3_pins_are_included_without_reinterpreting_the_original_failure():
    # HH_261005 - The prior attempt stays frozen; V4 records the previously absent admission data without changing physical conditions.
    assert set(owner.FROZEN_V3_PINS) == {
        "tools/e2e/run_owned_town03_stationary_wheel_rpc_v3_20261005.py",
        "tools/e2e/collect_owned_town03_stationary_wheel_rpc_v3_20261005.py",
        "learning/tests/test_owned_town03_stationary_wheel_rpc_owner_v3_20261005.py",
        "learning/tests/test_owned_town03_stationary_wheel_rpc_capture_v3_20261005.py",
    }
    pins = owner.source_pins()
    assert all(pins[name] == sha for name, sha in owner.FROZEN_V3_PINS.items())


def test_unchanged_shipping_argv_no_unsupported_INI_or_URL_candidate(contract):
    assert contract["server_command"][1:3] == ["CarlaUE4", "-prefernvidia"]
    assert not any("DEFENGINEINI" in value or value.startswith("/Game/") for value in contract["server_command"])
    assert contract["exact_map_identifiers"] == ["Town03", "/Game/Carla/Maps/Town03", "Carla/Maps/Town03"]
    assert contract["expected_Town03_XODR_size_bytes"] == 4589872
    assert contract["expected_Town03_XODR_sha256"] == "abf177bcd9b66cfefb7ac4919f6456815f1384f4229f1343c8a6a90bbb7713c9"
    assert contract["V2_actual_rejected_map_remains_unknown"] is True
    assert len(contract["reused_immutable_SDK_assets"]) == 4
    assert contract["freshly_copied_SDK_assets"] == [str(owner.EGG)]


@pytest.fixture
def tiny_hybrid_sdk(tmp_path, monkeypatch):
    # HH_261005 - Five tiny synthetic package assets validate real archive lookup/generation guards without opening SDK binaries or driving data.
    from types import SimpleNamespace as NS
    root, sdk = tmp_path / "workspace", tmp_path / "sdk"
    root.mkdir(); sdk.mkdir()
    old, fresh = root / "old_v2", root / "new_v4"
    old.mkdir(); fresh.mkdir()
    assets = {}
    paths = [sdk / "egg.egg"] + [sdk / ("sdk" + str(index)) for index in range(4)]
    for index, path in enumerate(paths):
        path.write_bytes(("synthetic sealed package " + str(index)).encode())
        assets[str(path)] = owner.digest(path)
        archive = old / "source_external" / path.relative_to(sdk)
        archive.parent.mkdir(exist_ok=True)
        archive.write_bytes(path.read_bytes())
    old_plan, current_plan, authority = root / "old_v2_plan.json", root / "new_v4_plan.json", root / "ROOT_synthetic_authority.json"
    owner.write_new(old_plan, {"schema": "portable_e2e.owned_town03_stationary_wheel_rpc.20261005.v2",
        "output": str(old), "asset_sha256": assets})
    evidence = root / "V5_synthetic_metadata.json"
    owner.write_new(evidence, {"capture_contract": {key: {"map_name": "Carla/Maps/Town03"}
        for key in ("live_xodr_preflight", "active_map_opendrive")}})
    failed = root / "original_synthetic_failure.json"
    owner.write_new(failed, {"status": "FAIL_RETAINED", "actual_map": "UNKNOWN"})
    metadata = {str(path): owner.digest(path) for path in (old_plan, evidence, failed)}
    for key, value in (("ROOT", root), ("CARLA_ROOT", sdk), ("EGG", paths[0]), ("ASSET_SHA256", assets),
                       ("OLD_V2_OUTPUT", old), ("OLD_V2_PLAN", old_plan), ("OUTPUT", fresh), ("PLAN", current_plan),
                       ("LAUNCH_AUTHORITY", authority), ("V5_MAP_EVIDENCE", evidence), ("METADATA_EVIDENCE_PINS", metadata)):
        monkeypatch.setattr(owner, key, value)
    source = root / "synthetic.py"; source.write_bytes(b"synthetic source, no API")
    plan = dict(source_sha256={"synthetic.py": owner.digest(source)}, asset_sha256=assets,
                native_runtime_input={"producer_provenance_sha256": {}, "synthetic_only": True},
                asset_archive_paths=owner.asset_archive_paths(), reused_SDK_archive_generations=owner.sdk_archive_generations(),
                current_SDK_asset_generations=owner.sdk_asset_generations(),
                metadata_evidence_sha256=metadata)
    owner.write_new(current_plan, plan); owner.write_new(authority, {"synthetic_only": True})
    monkeypatch.setattr(owner, "verify_native_runtime_input", lambda plan: {"synthetic_only": True})
    return NS(root=root, sdk=sdk, old=old, fresh=fresh, assets=assets, paths=paths, plan=plan, old_plan=old_plan,
              evidence=evidence, current_plan=current_plan, authority=authority)


def test_hybrid_archive_copies_fresh_egg_but_reuses_four_exact_old_SDK_files(tiny_hybrid_sdk):
    value = tiny_hybrid_sdk
    before = {path: (owner.digest(path), path.stat().st_ctime_ns) for path in (value.old / "source_external").iterdir()}
    assert owner.verify_sdk_reused_inputs(value.plan)
    records = owner.archive_sources(value.plan, owner.digest(value.current_plan), owner.digest(value.authority))
    assert owner.verify_archived_sources(value.plan, owner.digest(value.current_plan), owner.digest(value.authority))
    for index, asset in enumerate(value.paths):
        target = Path(value.plan["asset_archive_paths"][str(asset)])
        assert target.is_relative_to(value.fresh if index == 0 else value.old)
        assert owner.digest(target) == value.assets[str(asset)]
    assert list((value.fresh / "source_external").iterdir()) == [value.fresh / "source_external/egg.egg"]
    assert before == {path: (owner.digest(path), path.stat().st_ctime_ns) for path in (value.old / "source_external").iterdir()}
    assert len([row for row in records if str(row[1]) in value.plan["reused_SDK_archive_generations"]]) == 4


@pytest.mark.parametrize("fault", ["archive_bytes", "same_bytes_new_generation", "archive_alias", "current_bytes", "plan_path", "generation",
                                      "old_plan", "metadata", "map_evidence"])
def test_hybrid_reuse_refuses_any_changed_input_or_provenance(tiny_hybrid_sdk, fault):
    value = tiny_hybrid_sdk
    archive = Path(value.plan["asset_archive_paths"][str(value.paths[1])])
    if fault == "archive_bytes": archive.write_bytes(b"synthetic corrupt retained archive")
    elif fault == "same_bytes_new_generation": archive.write_bytes(archive.read_bytes())
    elif fault == "archive_alias":
        preserved = archive.with_name("synthetic_preserved"); archive.rename(preserved); archive.symlink_to(preserved)
    elif fault == "current_bytes": value.paths[1].write_bytes(b"synthetic current corruption")
    elif fault == "plan_path": value.plan["asset_archive_paths"][str(value.paths[1])] = str(value.paths[1])
    elif fault == "generation": value.plan["reused_SDK_archive_generations"][str(archive)]["st_ino"] += 1
    elif fault == "old_plan": value.old_plan.write_text('{}')
    elif fault == "metadata": value.plan["metadata_evidence_sha256"] = {}
    else: value.evidence.write_text('{}')
    with pytest.raises((RuntimeError, FileNotFoundError)):
        owner.verify_sdk_reused_inputs(value.plan)


@pytest.mark.parametrize("identifier", ["Town03", "/Game/Carla/Maps/Town03", "Carla/Maps/Town03"])
def test_exact_evidenced_API_identifier_kept_in_working_INI_receipt(synthetic_ini, identifier):
    output, _, plan = synthetic_ini
    owner.write_new(output / "capture_result.json", {"actual_map_identity": identifier})
    assert owner.engine_ini_postcheck(plan, require_actual_map=True)["actual_map_identity"] == identifier
