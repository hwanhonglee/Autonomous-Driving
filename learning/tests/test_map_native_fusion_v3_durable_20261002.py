"""HH_261002 - Synthetic-only V3 identity and durable-launch guards; no actual raw/model/GPU/SSH execution."""
import ast
import copy
import hashlib
import inspect
import json
import multiprocessing
from pathlib import Path
import shutil
import sys
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "learning")); sys.path.insert(0, str(ROOT / "tools/e2e"))
import train_map_native_fusion_pair_v3_20261002 as revision
import launch_map_native_fusion_pair_v3_durable_20261002 as durable
import test_map_native_fusion_gpu_v2_20261002 as old_tests
ENVIRONMENT_CHECK = durable.environment_check


def authority(workers=2):
    p = revision.protocol
    source = {**revision.FROZEN_SOURCE_SHA, Path(revision.__file__).name: "a" * 64}
    proof = {"synthetic_not_actual_schedule": True}
    value = dict(schema=p.AUTHORIZATION_SCHEMA, contract=p.prepared_contract(workers), source_sha256=source,
        protocol_sha256=p.document_sha(p.prepared_contract(workers)), equal_global_schedule_sha256=p.document_sha(proof),
        runtime_identity=old_tests.runtime(), reviewer="synthetic-not-real-authority", research_fit_authorized=True,
        native_denials_preserved=True, DEV_or_HOLDOUT_used_for_fit=False,
        automatic_model_promotion=False, vehicle_control_approved=False)
    return value, source, proof


@pytest.mark.parametrize("workers", [0, 2])
def test_v3_changes_only_identity_and_explicit_interruption_provenance(workers):
    old, new = revision.prior.protocol.prepared_contract(workers), revision.protocol.prepared_contract(workers)
    assert {k for k in old if old[k] != new[k]} == {
        "schema", "study_id", "source", "output", "source_count", "numeric_export_schema"}
    assert new["source_count"] == 15 and new["gpu_preflight_revision"] == old["gpu_preflight_revision"]
    assert new["interrupted_V2_study_preserved"] == revision.prior.NAME
    assert new["fresh_both_arms_fixed20000_not_resume"] is True
    assert new["interrupted_optimizer_or_endpoint_reused"] is False
    assert new["examples_per_arm"] == 160000 and new["physical_TRAIN_slots"] == 1209
    assert new["semantic_loss"] == old["semantic_loss"] and new["all_existing_27_screens_unchanged"] is True
    for name in ("schedule", "schedule_proof", "train_package", "postcheck", "environment_contract", "original_27_screen"):
        assert getattr(revision.protocol, name).__code__ is getattr(revision.prior.protocol, name).__code__
    value, source, proof = authority(workers)
    revision.protocol.validate_authorization(value, source, proof, workers)


def test_inverse_authority_AST_retains_every_original_predicate():
    node = revision.count_revision_ast()
    hits = [n for n in ast.walk(node) if isinstance(n, ast.Constant) and type(n.value) is int and n.value == 15]
    assert len(hits) == 1; hits[0].value = 13
    tree = ast.parse(Path(revision.previous.__file__).read_bytes())
    old = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "validate_authorization")
    assert ast.dump(node) == ast.dump(old)


@pytest.mark.parametrize("fault", ["oldschema", "count", "missing_wrapper", "changed_V2", "resume", "gpu", "math", "fit", "DEV"])
def test_external_v3_authority_rejects_changed_identity_math_or_scope(fault):
    value, source, proof = authority()
    if fault == "oldschema": value["schema"] = revision.prior.protocol.AUTHORIZATION_SCHEMA
    elif fault == "count": value["contract"]["source_count"] = 14
    elif fault == "missing_wrapper": source.pop(Path(revision.__file__).name)
    elif fault == "changed_V2": source[Path(revision.prior.__file__).name] = "c" * 64
    elif fault == "resume": value["contract"]["interrupted_optimizer_or_endpoint_reused"] = True
    elif fault == "gpu": value["contract"]["gpu_preflight_revision"]["maximum_driver_residual_MiB"] = 17
    elif fault == "math": value["contract"]["learning_rate"] = 1e-4
    elif fault == "fit": value["research_fit_authorized"] = False
    else: value["DEV_or_HOLDOUT_used_for_fit"] = True
    with pytest.raises(ValueError): revision.protocol.validate_authorization(value, source, proof, 2)


def test_full_fifteen_snapshot_and_frozen_v2(tmp_path):
    revision.verify_frozen()
    for name in revision.SOURCE_NAMES:
        path = Path(revision.__file__).parent / name
        if not path.is_file(): path = ROOT / "learning/portable_e2e" / name
        shutil.copy2(path, tmp_path / name)
    actual = revision.source_snapshot(tmp_path.resolve())
    assert len(actual) == 15 and all(actual[k] == v for k, v in revision.FROZEN_SOURCE_SHA.items())
    assert actual[Path(revision.__file__).name] == hashlib.sha256(Path(revision.__file__).read_bytes()).hexdigest()
    (tmp_path / Path(revision.prior.__file__).name).write_bytes(b"changed")
    with pytest.raises(ValueError): revision.source_snapshot(tmp_path.resolve())


def test_activation_exact_frozen_functions_unchanged_and_restored():
    original = revision.original
    old = original.protocol, original.providers, original.gpu_idle_before_cuda, original.CropBatches
    with pytest.raises(RuntimeError, match="synthetic"):
        with revision.activated():
            assert original.protocol is revision.protocol and original.providers is revision.prior.providers
            assert original.gpu_idle_before_cuda is revision.prior.gpu_idle_before_cuda
            assert original.CropBatches is revision.CropBatches
            with pytest.raises(ValueError):
                with revision.activated(): pass
            with pytest.raises(ValueError):
                with revision.prior.activated(): pass
            raise RuntimeError("synthetic")
    assert old == (original.protocol, original.providers, original.gpu_idle_before_cuda, original.CropBatches)
    assert not revision._active and not revision.prior._active


@pytest.mark.parametrize("binding", ["protocol", "providers", "gpu_idle_before_cuda", "CropBatches"])
def test_unknown_original_binding_is_not_overwritten(monkeypatch, binding):
    sentinel = object(); monkeypatch.setattr(revision.original, binding, sentinel)
    with pytest.raises(ValueError):
        with revision.activated(): pass
    assert getattr(revision.original, binding) is sentinel and not revision._active


def spawned_reader(connection, descriptor):
    try:
        assert descriptor.arrays is None and type(descriptor) is revision.CropBatches
        assert descriptor.__getitem__.__globals__["protocol"] is revision.protocol
        assert revision.protocol.SOURCE.name.endswith("v3") and len(revision.SOURCE_NAMES) == 15
        sys.modules["torch"] = SimpleNamespace(cuda=SimpleNamespace(is_initialized=lambda: False))
        revision.protocol.source_snapshot = lambda root: descriptor.source
        revision.protocol.train_package = lambda: (None, np.zeros((1, 1), np.uint8), None, None)
        revision.protocol.previous.previous.batch_arrays = lambda arrays, ids, boxes, bits, arm: (arrays, arrays, bits.copy())
        step, rgb, target, bits = descriptor[0]
        connection.send(dict(schema=revision.SCHEMA, step=step, byte_equal=np.array_equal(rgb, target), FALSE_only=not bits.any()))
    except Exception as error: connection.send(dict(error=type(error).__name__ + ": " + str(error)))
    finally: connection.close()


def test_genuine_spawn_keeps_exact_crop_bytecode_and_new_namespace():
    assert revision.CropBatches.__getitem__.__code__ is revision.prior.CropBatches.__getitem__.__code__
    assert revision.CropBatches.__getstate__ is revision.prior.CropBatches.__getstate__
    schedule = {k: np.zeros((1, 1), np.uint8) for k in ("control_indices", "shared_crops", "shared_FALSE_view_bits")}
    descriptor = revision.CropBatches(np.zeros((1, 1), np.uint8), schedule, {"synthetic.py": "a" * 64})
    context = multiprocessing.get_context("spawn"); receiver, sender = context.Pipe(duplex=False)
    process = context.Process(target=spawned_reader, args=(sender, descriptor)); process.start(); sender.close()
    try:
        assert receiver.poll(20); value = receiver.recv(); process.join(20)
        assert process.exitcode == 0 and value == dict(schema=revision.SCHEMA, step=0, byte_equal=True, FALSE_only=True)
    finally:
        receiver.close()
        if process.is_alive(): process.terminate(); process.join(5)


def test_query_reads_no_payload_GPU_or_output(monkeypatch, capsys):
    monkeypatch.setattr(revision.original, "run", lambda _: pytest.fail("query is not actual fit"))
    assert revision.main(["--mode", "contract", "--loader-workers", "2"]) == 0
    value = json.loads(capsys.readouterr().out)
    assert value["source_count"] == 15 and not value["training_authorized"]
    assert durable.main(["--mode", "contract"]) == 0
    value = json.loads(capsys.readouterr().out)
    assert value["detached_session"] and not value["resume"] and not value["automatic_retry"]


def sandbox(monkeypatch, tmp_path):
    task = tmp_path.resolve() / "private"
    source = task / "source_worktrees" / durable.NAME
    helper = task / "plans/launches" / durable.NAME / Path(durable.__file__).name
    train_auth = task / "plans/training" / durable.NAME / "ROOT_fixed20k_structure_pair_authorization.json"
    logs = task / "runs/launches" / durable.NAME
    output = task / "runs/diagnostics" / durable.NAME
    venv = task / "venvs/py312"
    lease = task / "runs/campaigns/.gpu0_training.lock"
    for path in (source, helper.parent, train_auth.parent, logs.parent, output.parent, venv / "bin", lease.parent):
        path.mkdir(parents=True, exist_ok=True)
    helper.write_bytes(b"synthetic helper"); train_auth.write_text("{}"); lease.write_bytes(b"")
    trainer = source / "train_map_native_fusion_pair_v3_20261002.py"; trainer.write_bytes(b"synthetic trainer")
    pins = {**revision.FROZEN_SOURCE_SHA, Path(revision.__file__).name: "b" * 64}
    bindings = dict(WORKSPACE=task, SOURCE=source, VENV=venv, PYTHON=venv / "bin/python", TRAINER=trainer,
        TRAIN_AUTHORITY=train_auth, HELPER=helper, LAUNCH_AUTHORITY=helper.parent / "ROOT_detached_launch_authorization.json",
        LOGS=logs, OUTPUT=output, LEASE=lease)
    for key, value in bindings.items(): monkeypatch.setattr(durable, key, value)
    value = dict(schema=durable.SCHEMA + ".authorization", contract=durable.contract(),
        helper_sha256=durable.digest(helper), trainer_authorization_sha256=durable.digest(train_auth), source_sha256=pins,
        detached_launch_authorized=True, reviewer="synthetic ROOT", automatic_model_promotion=False, vehicle_control_approved=False)
    durable.LAUNCH_AUTHORITY.write_text(json.dumps(value))
    args = SimpleNamespace(helper_sha256=durable.digest(helper), authorization_sha256=durable.digest(durable.LAUNCH_AUTHORITY), validate_only=False)
    monkeypatch.setattr(durable, "environment_check", lambda: None)
    monkeypatch.setattr(durable, "source_and_authority", lambda v: pins)
    monkeypatch.setattr(durable, "process_generation", lambda pid: 123456789)
    seen = []
    def query(command, **kwargs):
        seen.append(tuple(command)); assert command[:3] == ["nvidia-smi", "-i", "0"]
        return "" if "compute-apps" in command[3] else f"0, {durable.GPU_UUID}, 1, 0\n"
    monkeypatch.setattr(durable.subprocess, "check_output", query)
    return args, value, seen


def test_actual_launch_call_uses_regular_files_no_pipes_and_detached_session(monkeypatch, tmp_path):
    args, _, seen = sandbox(monkeypatch, tmp_path)
    calls = []
    def popen(command, **options):
        calls.append((command, options))
        assert options["stdin"] == durable.subprocess.DEVNULL and options["start_new_session"] is True
        assert options["close_fds"] is True and options["cwd"] == durable.SOURCE
        assert options["stdout"].writable() and options["stderr"].writable()
        assert Path(options["stdout"].name) == durable.LOGS / "stdout.log"
        assert Path(options["stderr"].name) == durable.LOGS / "stderr.log"
        options["stdout"].write(b"synthetic durable stdout\n"); options["stderr"].write(b"synthetic stderr\n")
        return SimpleNamespace(pid=43210)
    monkeypatch.setattr(durable.subprocess, "Popen", popen)
    value = durable.launch(args)
    assert len(calls) == 1 and len(seen) == 2 and value["actual_optimizer_updates_claimed"] == 0
    assert value["process_start_ticks"] == 123456789
    assert (durable.LOGS / "stdout.log").read_bytes() == b"synthetic durable stdout\n"
    assert (durable.LOGS / "launch_receipt.json").is_file()
    assert value["owned_process_generation_confirmed"] is True and value["boot_id"]
    assert calls[0][0] == [str(durable.PYTHON), "-B", "-u", str(durable.TRAINER), "--mode", "fit", "--loader-workers", "2",
        "--authorization", str(durable.TRAIN_AUTHORITY), "--authorization-sha256", durable.digest(durable.TRAIN_AUTHORITY)]


def test_immediate_child_exit_still_retains_nontraining_receipt(monkeypatch, tmp_path):
    args, _, _ = sandbox(monkeypatch, tmp_path)
    monkeypatch.setattr(durable.subprocess, "Popen", lambda *a, **k: SimpleNamespace(pid=43210))
    def missing(pid): raise FileNotFoundError("synthetic child already exited")
    monkeypatch.setattr(durable, "process_generation", missing)
    value = durable.launch(args)
    assert value["owned_process_generation_confirmed"] is False and value["process_start_ticks"] is None
    assert value["generation_read_error"]["error_type"] == "FileNotFoundError"
    assert value["actual_optimizer_updates_claimed"] == 0
    assert (durable.LOGS / "launch_receipt.json").is_file() and (durable.LOGS / "stderr.log").is_file()


def test_validate_only_never_forks_or_creates_logs(monkeypatch, tmp_path):
    args, _, _ = sandbox(monkeypatch, tmp_path); args.validate_only = True
    monkeypatch.setattr(durable.subprocess, "Popen", lambda *a, **k: pytest.fail("preflight cannot spawn"))
    value = durable.launch(args)
    assert value["status"] == "DETACHED_LAUNCH_PREFLIGHT_ONLY_NO_PROCESS_NO_TRAIN_NO_OUTPUT"
    assert not durable.LOGS.exists() and not durable.OUTPUT.exists()


@pytest.mark.parametrize("fault", ["schema", "helper", "source_contract", "resume", "GPU1", "scope", "reviewer", "promotion", "vehicle"])
def test_changed_external_launch_authority_rejected_before_fork(monkeypatch, tmp_path, fault):
    args, value, _ = sandbox(monkeypatch, tmp_path)
    if fault == "schema": value["schema"] = "old"
    elif fault == "helper": value["helper_sha256"] = "c" * 64
    elif fault == "source_contract": value["contract"]["source"] = "/foreign"
    elif fault == "resume": value["contract"]["resume"] = True
    elif fault == "GPU1": value["contract"]["physical_GPU_index"] = 1
    elif fault == "scope": value["detached_launch_authorized"] = False
    elif fault == "reviewer": value["reviewer"] = " "
    elif fault == "promotion": value["automatic_model_promotion"] = True
    else: value["vehicle_control_approved"] = True
    durable.LAUNCH_AUTHORITY.write_text(json.dumps(value)); args.authorization_sha256 = durable.digest(durable.LAUNCH_AUTHORITY)
    monkeypatch.setattr(durable.subprocess, "Popen", lambda *a, **k: pytest.fail("denied cannot spawn"))
    with pytest.raises(ValueError): durable.launch(args)
    assert not durable.LOGS.exists()


@pytest.mark.parametrize("gpu", ["0, GPU-foreign, 1, 0\n", f"0, {durable.GPU_UUID}, 17, 0\n",
    f"0, {durable.GPU_UUID}, 1, 1\n", f"1, {durable.GPU_UUID}, 1, 0\n", "", "malformed\n"])
def test_busy_or_wrong_GPU_rejects_without_spawn(monkeypatch, tmp_path, gpu):
    args, _, _ = sandbox(monkeypatch, tmp_path)
    monkeypatch.setattr(durable.subprocess, "check_output", lambda command, **k: "" if "compute-apps" in command[3] else gpu)
    monkeypatch.setattr(durable.subprocess, "Popen", lambda *a, **k: pytest.fail("busy cannot spawn"))
    with pytest.raises(ValueError): durable.launch(args)
    assert not durable.LOGS.exists()


def test_compute_process_and_held_original_lease_rejected(monkeypatch, tmp_path):
    args, _, _ = sandbox(monkeypatch, tmp_path)
    monkeypatch.setattr(durable.subprocess, "check_output", lambda command, **k:
        "9876, GPU-foreign, 1\n" if "compute-apps" in command[3] else f"0, {durable.GPU_UUID}, 1, 0\n")
    with pytest.raises(ValueError): durable.launch(args)
    with durable.LEASE.open("r+b") as stream:
        durable.fcntl.flock(stream, durable.fcntl.LOCK_EX | durable.fcntl.LOCK_NB)
        with pytest.raises(BlockingIOError): durable.lease_and_GPU_idle()
    assert not durable.LOGS.exists()


def test_log_paths_ancestor_symlink_and_escape_are_denied(monkeypatch, tmp_path):
    sandbox(monkeypatch, tmp_path)
    for path in (Path("/foreign"), durable.WORKSPACE / ".." / "escape"):
        with pytest.raises(ValueError): durable.canonical(path)
    link = durable.WORKSPACE / "link"; link.symlink_to(durable.LOGS.parent, target_is_directory=True)
    with pytest.raises(ValueError): durable.canonical(link / "fresh")


def test_environment_guard_checks_host_venv_cwd_all_paths_and_no_contaminants(monkeypatch, tmp_path):
    sandbox(monkeypatch, tmp_path)
    method = inspect.getsource(ENVIRONMENT_CHECK)
    for required in ("pro6000", "sys.prefix", "Path.cwd()", "sys.flags.optimize", "canonical(LOGS)",
        "canonical(OUTPUT)", "not LOGS.exists()", "not OUTPUT.exists()", "PYTHONPATH", "PYTHONHOME"):
        assert required in method
    ast_body = ast.parse(inspect.getsource(durable.launch))
    call_names = [n.func.id for n in ast.walk(ast_body) if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)]
    assert call_names.index("environment_check") < call_names.index("source_and_authority") < call_names.index("lease_and_GPU_idle")


@pytest.mark.parametrize("fault", [None, "host", "venv", "cwd", "helper", "env", "PYTHONPATH", "logs", "output", "symlink"])
def test_executable_environment_checks_reject_wrong_host_or_nonfresh_private_paths(monkeypatch, tmp_path, fault):
    sandbox(monkeypatch, tmp_path)
    monkeypatch.setattr(durable, "__file__", str(durable.HELPER))
    monkeypatch.setattr(durable.sys, "prefix", str(durable.VENV))
    monkeypatch.setattr(durable.os, "uname", lambda: SimpleNamespace(nodename="pro6000"))
    monkeypatch.chdir(durable.SOURCE)
    for key, value in durable.ENVIRONMENT.items(): monkeypatch.setenv(key, value)
    for key in ("PYTHONPATH", "PYTHONHOME", "PYTHONUSERBASE", "PYTHONSTARTUP"): monkeypatch.delenv(key, raising=False)
    if fault == "host": monkeypatch.setattr(durable.os, "uname", lambda: SimpleNamespace(nodename="foreign"))
    elif fault == "venv": monkeypatch.setattr(durable.sys, "prefix", "/foreign/venv")
    elif fault == "cwd": monkeypatch.chdir(durable.WORKSPACE)
    elif fault == "helper": monkeypatch.setattr(durable, "__file__", "/foreign/helper.py")
    elif fault == "env": monkeypatch.setenv("CUDA_VISIBLE_DEVICES", "0,1")
    elif fault == "PYTHONPATH": monkeypatch.setenv("PYTHONPATH", "/foreign/imports")
    elif fault == "logs": durable.LOGS.mkdir()
    elif fault == "output": durable.OUTPUT.mkdir()
    elif fault == "symlink": durable.LOGS.symlink_to(durable.SOURCE, target_is_directory=True)
    if fault is None: ENVIRONMENT_CHECK()
    else:
        with pytest.raises(ValueError): ENVIRONMENT_CHECK()


def test_source_read_boundary_verifies_all_fifteen_and_original_authorization_before_runtime():
    method = inspect.getsource(durable.source_and_authority)
    for required in ("revision.verify_frozen()", "revision.source_snapshot(SOURCE)", "validate_authorization",
        "len(source) == 15", "runtime_identity()"):
        assert required in method
    assert method.index("source bytes changed before import") < method.index("import train_map_native_fusion_pair_v3")
    assert "read_npz" not in method and "train_package" not in method and "torch.cuda" not in method


def test_no_unbounded_retry_pipe_install_or_foreign_kill_exists():
    method = Path(durable.__file__).read_text()
    assert "subprocess.PIPE" not in method and "os.kill" not in method and "shell=True" not in method
    body = ast.parse(method)
    assert not any(isinstance(n, (ast.While, ast.AsyncFor)) for n in ast.walk(body))
    assert sum(isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr == "Popen" for n in ast.walk(body)) == 1
