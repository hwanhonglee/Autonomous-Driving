"""HH_261002 - Tiny synthetic launch/source/CPU/MAP proofs; no real weights, raw data, CUDA, SSH or child process."""
import hashlib
import importlib.util
import inspect
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("hh_balanced_durable", ROOT / "tools/e2e/launch_temporal_left_right_joint_v2_durable_20261002.py")
d = importlib.util.module_from_spec(spec); spec.loader.exec_module(d)
ENVIRONMENT_CHECK = d.environment_check


def json_sha(value): return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


@pytest.fixture
def fixture(tmp_path, monkeypatch):
    # HH_261002 - A compact JSON chronology is not the large synthetic update journal or any actual model/dataset.
    personal = tmp_path / "personal"; w = personal / "portable_e2e"; s = w / "source_worktrees" / d.NAME
    bindings = dict(PERSONAL=personal, WORKSPACE=w, SOURCE=s, VENV=w / "venvs/py312", PYTHON=w / "venvs/py312/bin/python",
        TRAINER=s / d.TRAINER_REL, TRAIN_AUTHORITY=w / "plans/training" / d.NAME / "ROOT_fit.json",
        HELPER=w / "plans/launches" / d.NAME / "helper.py", LAUNCH_AUTHORITY=w / "plans/launches" / d.NAME / "ROOT_launch.json",
        LOGS=w / "runs/launches" / d.NAME, OUTPUT=w / "runs/training" / d.NAME,
        LEASE=w / "runs/campaigns/.gpu0_training.lock", CPU_PREFLIGHT=w / "plans/training" / d.NAME / "CPU.json",
        MAP_COMPLETION=w / "plans/training" / d.NAME / "MAP_complete.json", MAP_OUTPUT=w / "runs/diagnostics/MAP_V3")
    for key, value in bindings.items(): monkeypatch.setattr(d, key, value)
    for path in (s, d.VENV / "bin", d.HELPER.parent, d.TRAIN_AUTHORITY.parent, d.LOGS.parent, d.OUTPUT.parent, d.LEASE.parent, d.MAP_OUTPUT):
        path.mkdir(parents=True, exist_ok=True)
    d.HELPER.write_bytes(b"synthetic helper"); d.LEASE.write_bytes(b"")
    def write(path, value): path.write_text(json.dumps(value)); return d.digest(path)
    pins = {}
    for name in [d.TRAINER_REL, "tools/workspace_python.py", *[f"learning/dependency{i}.py" for i in range(29)]]:
        path = s / name; path.parent.mkdir(parents=True, exist_ok=True); path.write_bytes(b"synthetic source " + name.encode()); pins[name] = d.digest(path)
    monkeypatch.setattr(d, "TRAINER_SHA", pins[d.TRAINER_REL])
    history = "3c1fe1e4f9e18017a2db31b5371001873dece4a2b8e78bb3cdb21f9aad5ebac3"
    proposed = dict(sources=pins, new_LEFT_conversion_sha256="a" * 64, new_RIGHT_conversion_sha256="b" * 64, actual_parent_history_sha256=history)
    authsha = write(d.TRAIN_AUTHORITY, dict(contract=proposed))
    arguments, assets = {}, {str(d.TRAIN_AUTHORITY): authsha}
    for name in d.ARGUMENT_NAMES:
        if name == "right-train": path = personal / "datasets/prepared/RIGHT697"
        elif name == "left-train": path = personal / "dataset/prepared/LEFT347"
        elif name.endswith("train") or name.endswith("holdout") or name == "parent-run": path = personal / name
        else: path = personal / (name + ".synthetic")
        path.parent.mkdir(parents=True, exist_ok=True)
        if name.endswith("train") or name.endswith("holdout") or name == "parent-run": path.mkdir(exist_ok=True)
        else: path.write_bytes(b"synthetic immutable metadata only"); assets[str(path)] = d.digest(path, personal)
        arguments[name] = str(path)
    def validate(path, sha, contract): assert d.read_pinned(path, sha)["contract"] == contract
    revision = SimpleNamespace(SCHEMA="portable_e2e.temporal_actual_left_right_joint.20261002.v2", PARENT_HISTORY_SHA=history,
        RIGHT_BASENAME="RIGHT697", LEFT_BASENAME="LEFT347", verify_frozen=lambda: None, source_snapshot=lambda: pins,
        contract=lambda left: proposed, validate_authorization=validate, json_sha=json_sha)
    monkeypatch.setattr(d, "trainer_revision", lambda: revision)
    runtime = dict(synthetic_no_actual_binary_read=True); monkeypatch.setattr(d, "runtime_identity", lambda _: runtime)
    samples = [["r", "o", "g", "l", "t"]] * 20000
    cpu = dict(status="VALIDATED_ALL20_TRAIN_CPU_NO_CUDA_NO_OUTPUT", schema=revision.SCHEMA, protocol=proposed,
        source_sha256=pins, immutable_input_sha256=assets, authorization_sha256=authsha, training_episode_count=20,
        effective_batch_size=5, trainable_state_tensor_count=54, frozen_state_tensor_count=47, parent_history_sha256=history,
        holdout_payload_loaded=False, fixed_sample_ids=samples, fixed_sample_ids_sha256=json_sha(samples))
    cpusha = write(d.CPU_PREFLIGHT, cpu)
    arms = {arm: dict(steps=20000, examples_seen=160000, model_promoted=False, vehicle_control_approved=False) for arm in ("control", "candidate")}
    boundary = {arm: dict(steps=20000, examples_seen=160000) for arm in arms}
    evaluation = {split: {arm: dict(evaluated_frames=n, count=n, full_FOV=True) for arm in ("parent", "control", "candidate")}
        for split, n in (("dev_town01", 118), ("holdout_town02_p02", 232))}
    summary = dict(schema="portable_e2e.map_native_fusion_pair.20261002.v3", status="FIXED20K_ARCHITECTURE_PAIR_DIAGNOSTIC_NOT_PROMOTED",
        source_and_inputs_unchanged=True, model_promoted=False, vehicle_control_approved=False, arms_result=arms, actual_boundary=boundary,
        known_evaluation=evaluation, exploratory_screen_result=dict(checks={f"check{i}": i != 0 for i in range(27)}))
    fit = dict(status="BOTH_FIXED_FINAL20000_NUMERIC_ENDPOINTS_VERIFIED_BEFORE_KNOWN_EVALUATION", known_payload_opened=False,
        arms=arms, actual_full_journal_evidence=boundary)
    proof = dict(schema=d.SCHEMA + ".MAP_completion", PID=d.MAP_PID, process_start_ticks=d.MAP_START_TICKS,
        boot_id=Path("/proc/sys/kernel/random/boot_id").read_text().strip(), source_asset_numeric_endpoints_and_full_journals_verified=True,
        model_promoted=False, vehicle_control_approved=False, summary_sha256=write(d.MAP_OUTPUT / "summary.json", summary),
        known_evaluation_sha256=write(d.MAP_OUTPUT / "known_evaluation.json", evaluation), fit_complete_sha256=write(d.MAP_OUTPUT / "fit_complete.json", fit))
    mapsha = write(d.MAP_COMPLETION, proof)
    value = dict(schema=d.SCHEMA + ".authorization", contract=d.contract(), helper_sha256=d.digest(d.HELPER), trainer_authorization_sha256=authsha,
        source_sha256=pins, asset_sha256=assets, asset_arguments=arguments, runtime_identity=runtime, cpu_preflight_sha256=cpusha,
        MAP_completion_sha256=mapsha, detached_launch_authorized=True, reviewer="synthetic ROOT", automatic_model_promotion=False, vehicle_control_approved=False)
    args = SimpleNamespace(helper_sha256=d.digest(d.HELPER), authorization_sha256=write(d.LAUNCH_AUTHORITY, value), validate_only=False)
    monkeypatch.setattr(d, "environment_check", lambda: None)
    monkeypatch.setattr(d, "process_generation", lambda pid: d.MAP_START_TICKS + 1)
    queries = []
    def query(command, **kwargs):
        queries.append(command); assert command[:3] == ["nvidia-smi", "-i", "0"]
        return "" if "compute-apps" in command[3] else f"0, {d.GPU_UUID}, 1, 0\n"
    monkeypatch.setattr(d.subprocess, "check_output", query)
    monkeypatch.setattr(d.subprocess, "Popen", lambda *a, **k: pytest.fail("unmocked child process forbidden"))
    return SimpleNamespace(args=args, value=value, pins=pins, proposed=proposed, cpu=cpu, proof=proof, summary=summary,
        evaluation=evaluation, fit=fit, write=write, queries=queries)


def repin(f): f.args.authorization_sha256 = f.write(d.LAUNCH_AUTHORITY, f.value)


def test_full_source_cpu_assets_map_and_regular_detached_launch(fixture, monkeypatch):
    f = fixture; calls = []
    def popen(command, **opts):
        assert opts["stdin"] == d.subprocess.DEVNULL and opts["start_new_session"] and opts["close_fds"]
        assert opts["cwd"] == d.SOURCE and opts["stdout"].writable() and opts["stderr"].writable()
        opts["stdout"].write(b"synthetic no actual updates\n"); calls.append(command); return SimpleNamespace(pid=123456)
    monkeypatch.setattr(d.subprocess, "Popen", popen)
    receipt = d.launch(f.args)
    assert len(calls) == 1 and len(f.queries) == 2 and receipt["actual_optimizer_updates_claimed"] == 0
    assert calls[0] == d.fit_command(f.value, f.proposed) and calls[0][:3] == [str(d.PYTHON), "-B", "-u"]
    assert "scripts.e2e.train_temporal_left_right_joint_v2_20261002" in calls[0]
    assert (d.LOGS / "stdout.log").read_bytes() == b"synthetic no actual updates\n" and (d.LOGS / "launch_receipt.json").is_file()


def test_query_and_validate_only_never_spawn_or_create_outputs(fixture, capsys):
    assert d.main(["--mode", "contract"]) == 0; assert json.loads(capsys.readouterr().out)["source_count"] == 31
    f = fixture; f.args.validate_only = True; receipt = d.launch(f.args)
    assert receipt["status"] == "DETACHED_LAUNCH_PREFLIGHT_ONLY_NO_PROCESS_NO_TRAIN_NO_OUTPUT"
    assert not d.LOGS.exists() and not d.OUTPUT.exists()


@pytest.mark.parametrize("fault", ["schema", "scope", "GPUbool", "promotion", "helper", "reviewer", "source_count", "source_changed",
    "asset_changed", "runtime", "singularRIGHT", "pluralLEFT"])
def test_changed_authority_sources_assets_runtime_or_canonical_roles_block_before_fork(fixture, fault):
    f = fixture
    if fault == "schema": f.value["schema"] = "old"
    elif fault == "scope": f.value["detached_launch_authorized"] = False
    elif fault == "GPUbool": f.value["contract"]["physical_GPU_index"] = False
    elif fault == "promotion": f.value["automatic_model_promotion"] = True
    elif fault == "helper": f.value["helper_sha256"] = "a" * 64
    elif fault == "reviewer": f.value["reviewer"] = " "
    elif fault == "source_count": f.value["source_sha256"].pop(next(k for k in f.pins if k != d.TRAINER_REL))
    elif fault == "source_changed": (d.SOURCE / "learning/dependency0.py").write_bytes(b"changed")
    elif fault == "asset_changed": Path(f.value["asset_arguments"]["warm-parent-weights"]).write_bytes(b"changed")
    elif fault == "runtime": f.value["runtime_identity"] = dict(foreign=True)
    else:
        name = "right-train" if fault == "singularRIGHT" else "left-train"
        path = d.PERSONAL / ("dataset/prepared/RIGHT697" if fault == "singularRIGHT" else "datasets/prepared/LEFT347")
        path.mkdir(parents=True); f.value["asset_arguments"][name] = str(path)
    repin(f)
    with pytest.raises(ValueError): d.launch(f.args)
    assert not d.LOGS.exists()


@pytest.mark.parametrize("key,value", [("status", "prepared"), ("effective_batch_size", 4), ("effective_batch_size", 5.),
    ("training_episode_count", 19), ("trainable_state_tensor_count", 53), ("holdout_payload_loaded", True),
    ("parent_history_sha256", "b" * 64), ("fixed_sample_ids_sha256", "a" * 64)])
def test_actual_cpu_preflight_required_not_claim_or_shortened_scope(fixture, key, value):
    f = fixture; f.cpu[key] = value; f.value["cpu_preflight_sha256"] = f.write(d.CPU_PREFLIGHT, f.cpu); repin(f)
    with pytest.raises(ValueError): d.launch(f.args)
    assert not d.LOGS.exists()


@pytest.mark.parametrize("fault", ["root_unverified", "wrongPID", "wrongticks", "oldboot", "unfinished", "missingarm", "lost350", "lost27", "no_fov"])
def test_MAP_predecessor_full_completed_both20k35027_required(fixture, fault):
    f = fixture
    if fault == "root_unverified": f.proof["source_asset_numeric_endpoints_and_full_journals_verified"] = False
    elif fault == "wrongPID": f.proof["PID"] += 1
    elif fault == "wrongticks": f.proof["process_start_ticks"] += 1
    elif fault == "oldboot": f.proof["boot_id"] = "wrong"
    elif fault == "unfinished": f.summary["arms_result"]["candidate"]["steps"] = 19999
    elif fault == "missingarm": f.summary["arms_result"].pop("candidate")
    elif fault == "lost350": f.evaluation["holdout_town02_p02"]["candidate"]["evaluated_frames"] = 231
    elif fault == "lost27": f.summary["exploratory_screen_result"]["checks"].pop("check0")
    else: f.evaluation["dev_town01"]["control"]["full_FOV"] = False
    f.proof.update(summary_sha256=f.write(d.MAP_OUTPUT / "summary.json", f.summary),
        known_evaluation_sha256=f.write(d.MAP_OUTPUT / "known_evaluation.json", f.evaluation), fit_complete_sha256=f.write(d.MAP_OUTPUT / "fit_complete.json", f.fit))
    f.value["MAP_completion_sha256"] = f.write(d.MAP_COMPLETION, f.proof); repin(f)
    with pytest.raises(ValueError): d.launch(f.args)
    assert not d.LOGS.exists()


def test_still_running_owned_MAP_cannot_launch_wait_or_kill(fixture, monkeypatch):
    f = fixture; exists = Path.exists
    monkeypatch.setattr(Path, "exists", lambda path: True if path == Path(f"/proc/{d.MAP_PID}/stat") else exists(path))
    monkeypatch.setattr(d, "process_generation", lambda pid: d.MAP_START_TICKS)
    with pytest.raises(ValueError, match="still running"): d.launch(f.args)
    assert not d.LOGS.exists() and not f.queries


@pytest.mark.parametrize("gpu", ["", "malformed", "1, WRONG, 0, 0", f"0, {d.GPU_UUID}, 17, 0", f"0, {d.GPU_UUID}, 1, 1"])
def test_busy_or_wrong_GPU_idle_blocks(fixture, monkeypatch, gpu):
    monkeypatch.setattr(d.subprocess, "check_output", lambda cmd, **k: "" if "compute-apps" in cmd[3] else gpu)
    with pytest.raises(ValueError): d.launch(fixture.args)
    assert not d.LOGS.exists()


def test_compute_process_or_original_lease_held_blocks(fixture, monkeypatch):
    monkeypatch.setattr(d.subprocess, "check_output", lambda cmd, **k: "777, foreign, 1" if "compute-apps" in cmd[3] else f"0, {d.GPU_UUID}, 1, 0")
    with pytest.raises(ValueError): d.launch(fixture.args)
    with d.LEASE.open("r+b") as stream:
        d.fcntl.flock(stream, d.fcntl.LOCK_EX | d.fcntl.LOCK_NB)
        with pytest.raises(BlockingIOError): d.lease_and_GPU_idle()
    assert not d.LOGS.exists()


def test_first_buffer_authority_duplicate_key_and_concurrent_replace(fixture, monkeypatch):
    path = d.WORKSPACE / "first.json"; raw = b'{"authorized":false}'; path.write_bytes(raw); read = Path.read_bytes; seen = []
    def raced(current):
        data = read(current)
        if current == path: seen.append(data); path.write_bytes(b'{"authorized":true}')
        return data
    monkeypatch.setattr(Path, "read_bytes", raced)
    assert d.read_pinned(path, hashlib.sha256(raw).hexdigest()) == dict(authorized=False) and seen == [raw]
    path.write_bytes(b'{"a":0,"a":1}')
    with pytest.raises(ValueError): d.read_pinned(path, hashlib.sha256(read(path)).hexdigest())


def test_changed_runtime_before_fork_retains_no_created_output(fixture, monkeypatch):
    f = fixture; calls = []
    def runtime(_): calls.append(1); return f.value["runtime_identity"] if len(calls) == 1 else dict(changed=True)
    monkeypatch.setattr(d, "runtime_identity", runtime)
    with pytest.raises(ValueError, match="before fork"): d.launch(f.args)
    assert not d.LOGS.exists()


@pytest.mark.parametrize("fault", [None, "host", "venv", "cwd", "helper", "environment", "contaminant", "logs", "output", "alias"])
def test_executable_environment_guard(fixture, monkeypatch, fault):
    monkeypatch.setattr(d, "__file__", str(d.HELPER)); monkeypatch.setattr(d.sys, "prefix", str(d.VENV))
    monkeypatch.setattr(d.os, "uname", lambda: SimpleNamespace(nodename="pro6000")); monkeypatch.chdir(d.SOURCE)
    for key, value in d.ENVIRONMENT.items(): monkeypatch.setenv(key, value)
    for key in ("PYTHONPATH", "PYTHONHOME", "PYTHONUSERBASE", "PYTHONSTARTUP"): monkeypatch.delenv(key, raising=False)
    if fault == "host": monkeypatch.setattr(d.os, "uname", lambda: SimpleNamespace(nodename="wrong"))
    elif fault == "venv": monkeypatch.setattr(d.sys, "prefix", "/foreign")
    elif fault == "cwd": monkeypatch.chdir(d.WORKSPACE)
    elif fault == "helper": monkeypatch.setattr(d, "__file__", "/foreign.py")
    elif fault == "environment": monkeypatch.setenv("CUDA_VISIBLE_DEVICES", "0,1")
    elif fault == "contaminant": monkeypatch.setenv("PYTHONPATH", "/foreign")
    elif fault == "logs": d.LOGS.mkdir()
    elif fault == "output": d.OUTPUT.mkdir()
    elif fault == "alias": d.LOGS.symlink_to(d.SOURCE, target_is_directory=True)
    if fault is None: ENVIRONMENT_CHECK()
    else:
        with pytest.raises(ValueError): ENVIRONMENT_CHECK()


def test_source_only_has_one_popen_no_wait_retry_ssh_install_or_foreign_kill():
    text = Path(d.__file__).read_text(); assert text.count("subprocess.Popen(") == 1
    assert not any(token in text for token in ("subprocess.PIPE", "shell=True", "os.kill", "time.sleep", "pip install", '"ssh"'))
    source = inspect.getsource(d.source_and_authority)
    assert source.index("verify_sources(") < source.index("trainer_revision(")
    assert "read_npz" not in source and "load_LEFT(" not in source and "load_RIGHT(" not in source
