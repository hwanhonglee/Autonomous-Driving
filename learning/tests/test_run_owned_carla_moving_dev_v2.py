# HH_260906 - Locate the repository independently of the relocated test directory.
"""HH_260906 - Check the DEV v2 owner using harmless private-repository children only."""

from __future__ import annotations

import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import shutil
import socket
import subprocess
import sys

import pytest

# HH_260906 - Keep logical source IDs, but open canonical nonsymlink files in the moved workspace.
from learning.portable_e2e.workspace_paths import workspace_path as _hh_source_path

ROOT = next(p for p in Path(__file__).resolve().parents if (p / ".git").exists())
WRAPPER = _hh_source_path(ROOT, "scripts/e2e/run_owned_carla_moving_dev_v2.sh")
SCHEMA = "carla.moving_task_capture_plan.v2"

# HH_260906 - Reuse only the legacy test fixture, without changing or executing its real entrypoints.
_spec = importlib.util.spec_from_file_location(
    "_moving_dev_owner_fixture", ROOT / "learning/tests/test_run_owned_carla_expert_trial.py")
legacy = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(legacy)


@pytest.fixture
def harness(tmp_path):
    value = legacy.harness.__wrapped__(tmp_path)
    path, _, manifest = legacy.moving_harness(value)
    scripts = value["root"] / "tools/e2e"
    shutil.copy2(WRAPPER, scripts / WRAPPER.name)
    value["runner"] = scripts / WRAPPER.name
    value["route"].write_text(json.dumps({"town": "Town03"}))
    helper = (scripts / "carla_moving_capture.py").read_text().replace(
        "carla.moving_task_capture_plan.v1", SCHEMA)
    helper += (
        "\n# HH_260906 - Validate the separate selection receipt in the fixture preflight.\n"
        "_configuration_from_args = configuration_from_args\n"
        "def configuration_from_args(a):\n"
        " d=_configuration_from_args(a)\n"
        " ref=d['route_selection_evidence']\n"
        " if hashlib.sha256(Path(ref['path']).read_bytes()).hexdigest()!=ref['sha256']: raise ValueError('selection receipt changed')\n"
        " return d\n")
    (scripts / "carla_moving_capture_dev_v2.py").write_text(helper)
    worker = (scripts / "collect_carla_vad_expert.py").read_text().replace(
        "carla_moving_capture", "carla_moving_capture_dev_v2")
    (scripts / "collect_carla_moving_dev_v2.py").write_text(worker)
    selection = value["root"] / "route_selection.json"
    selection.write_text(json.dumps({"fixture_only": True, "selected_town": "Town03"}))
    plan = json.loads(path.read_text())
    plan["schema"] = SCHEMA
    plan["route_selection_evidence"] = {
        "path": str(selection), "sha256": hashlib.sha256(selection.read_bytes()).hexdigest()}
    names = (
        "run_owned_carla_moving_dev_v2.sh", "collect_carla_vad_expert.py",
        "collect_carla_moving_dev_v2.py", "carla_moving_capture.py",
        "carla_moving_capture_dev_v2.py", "carla_goal_stop_profile.py", "carla_physics_substeps.py")
    plan["source_sha256"] = {
        "scripts/e2e/" + name: hashlib.sha256((scripts / name).read_bytes()).hexdigest()
        for name in names}
    plan["input_sha256"] = {
        name: hashlib.sha256(Path(source).read_bytes()).hexdigest()
        for name, source in plan["input_paths"].items()}
    path.write_text(json.dumps(plan))
    value.update(plan_path=path, manifest=manifest, selection=selection)
    return value


def options(harness, *, equals=False):
    path = harness["plan_path"]
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if equals:
        return [f"--moving-task-plan={path}", f"--moving-task-plan-sha256={digest}"]
    return ["--moving-task-plan", str(path), "--moving-task-plan-sha256", digest]


def run(harness, *owner_options, collector_options=None):
    return legacy._run(
        harness, "--quality", "Epic", "--wall-timeout-sec", "300", *owner_options,
        "--", *(options(harness) if collector_options is None else collector_options))


def test_owner_shell_and_embedded_preflight_programs_compile_without_execution():
    result = subprocess.run(["bash", "-n", str(WRAPPER)], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    text = WRAPPER.read_text()
    programs = re.findall(r"<<'PY'\n(.*?)\nPY(?:\n|$)", text, re.DOTALL)
    assert len(programs) == 8
    for index, program in enumerate(programs):
        compile(program, f"{WRAPPER.name}:embedded-{index}", "exec")
    assert "worker_module=scripts.e2e.collect_carla_moving_dev_v2" in text
    assert "from scripts.e2e.collect_carla_vad_expert import" not in text
    assert text.count("setsid bash tools/e2e/run_carla_map.sh") == 1
    assert text.count("setsid timeout --signal=INT --kill-after=15") == 1


@pytest.mark.parametrize("missing", ["both", "plan", "pin"])
def test_both_plan_flags_are_mandatory_before_output_claim_or_children(harness, missing):
    full = options(harness)
    selected = [] if missing == "both" else full[2:] if missing == "plan" else full[:2]
    result = run(harness, collector_options=selected)
    assert result.returncode == 2
    assert "both explicit" in result.stderr
    assert not harness["output"].exists() and not harness["events"].exists()


@pytest.mark.parametrize("owner_options", [
    ["--capture-mode", "actuation-response"], ["--capture-mode", "stationary-camera"],
    ["--capture-mode", "stationary-motion"], ["--capture-mode", "low-speed-coast"],
    ["--capture-mode", "coast-brake-transition"], ["--quality", "Low"],
    ["--wall-timeout-sec", "900"], ["--finish-before-utc", "2000-01-01T00:00:00Z"],
])
def test_legacy_modes_and_drifted_owner_budget_never_launch(harness, owner_options):
    assert run(harness, *owner_options).returncode == 2
    assert not harness["output"].exists() and not harness["events"].exists()


@pytest.mark.parametrize("override", [
    ["--host", "remote"], ["--host=remote"], ["--ho", "remote"],
    ["--port", "2999"], ["--po", "2999"], ["--allow-map-load"], ["--allow-m"],
])
def test_endpoint_or_world_redirect_is_rejected_before_launch(harness, override):
    result = run(harness, collector_options=options(harness) + override)
    assert result.returncode != 0
    assert not harness["output"].exists() and not harness["events"].exists()


@pytest.mark.parametrize("equals", [False, True])
def test_v2_capture_archives_complete_sources_and_separate_route_selection(harness, equals):
    result = run(harness, "--capture-mode", "expert", collector_options=options(harness, equals=equals))
    assert result.returncode == 0, result.stderr
    plan = json.loads((harness["output"] / "owner_plan.json").read_text())
    assert plan["worker_path"] == "scripts/e2e/collect_carla_moving_dev_v2.py"
    assert plan["capture_mode"] == "expert"
    assert plan["collector_argv"][0] == str(harness["output"] / "episode")
    assert len(plan["source_sha256"]) == 14
    expected = json.loads(harness["plan_path"].read_text())["source_sha256"]
    assert expected.items() <= plan["source_sha256"].items()
    assert "scripts/e2e/run_owned_carla_expert_trial.sh" not in plan["source_sha256"]
    for name, digest in plan["source_sha256"].items():
        assert hashlib.sha256((harness["output"] / "provenance" / name).read_bytes()).hexdigest() == digest
    task = plan["moving_task_capture"]
    assert task["schema"] == SCHEMA
    assert set(task["inputs"]) == {"route_file", "mapping", "calibration"}
    for ref in [task["plan"], *task["inputs"].values(), task["route_selection_evidence"]]:
        assert Path(ref["path"]).read_bytes() == (harness["output"] / ref["archived_path"]).read_bytes()
    owner = json.loads((harness["output"] / "owner_result.json").read_text())
    assert owner["exit_code"] == 0 and owner["source_bytes_unchanged_and_archived"] is True
    assert owner["moving_task_capture"]["input_checks"]["route_selection_evidence"] is True
    assert owner["moving_task_capture"]["capture_complete_pending_qa"] is True
    assert owner["moving_task_capture"]["dataset_admission_granted"] is False
    assert json.loads((harness["output"] / "lifecycle/stopped.json").read_text())["status"] == "PASS"


def test_legacy_plan_cannot_enter_v2_owner(harness):
    plan = json.loads(harness["plan_path"].read_text())
    plan["schema"] = "carla.moving_task_capture_plan.v1"
    harness["plan_path"].write_text(json.dumps(plan))
    assert run(harness).returncode != 0
    assert not harness["output"].exists() and not harness["events"].exists()


@pytest.mark.parametrize("offset", [0, 1, 2])
def test_occupied_rpc_or_auxiliary_port_remains_untouched(harness, offset):
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", harness["port"] + offset))
        listener.listen()
        result = run(harness)
        assert result.returncode != 0 and "already occupied" in result.stderr
        assert listener.getsockname()[1] == harness["port"] + offset
    assert not harness["output"].exists() and not harness["events"].exists()


def test_runtime_lock_denial_precedes_output_claim(harness):
    harness["env"]["E2E_TRIAL_TEST_LOCKED"] = "1"
    assert run(harness).returncode != 0
    assert not harness["output"].exists() and not harness["events"].exists()


def test_existing_output_is_never_retried_or_overwritten(harness):
    harness["output"].mkdir()
    sentinel = harness["output"] / "server.log"
    sentinel.write_text("keep-original")
    assert run(harness).returncode == 2
    assert sentinel.read_text() == "keep-original"
    assert not harness["events"].exists()


@pytest.mark.parametrize("target", ["plan", "selection", "helper"])
def test_startup_mutation_blocks_collector_and_cleans_owned_server(harness, target):
    paths = {"plan": harness["plan_path"], "selection": harness["selection"],
             "helper": harness["root"] / "tools/e2e/carla_moving_capture_dev_v2.py"}
    server = harness["root"] / "tools/e2e/fake_server.py"
    server.write_text(server.read_text().replace(
        "print('CARLA_READY simulated_only=true',flush=True)",
        f"with open({str(paths[target])!r},'a') as f: f.write(' ')\n"
        "print('CARLA_READY simulated_only=true',flush=True)"))
    result = run(harness)
    assert result.returncode == 1, result.stderr
    events = [json.loads(row) for row in harness["events"].read_text().splitlines()]
    assert [row["role"] for row in events] == ["server"]
    assert json.loads((harness["output"] / "lifecycle/stopped.json").read_text())["status"] == "PASS"
    owner = json.loads((harness["output"] / "owner_result.json").read_text())
    if target == "helper":
        assert owner["source_bytes_unchanged_and_archived"] is False
    else:
        check = "route_selection_evidence" if target == "selection" else "plan"
        assert owner["moving_task_capture"]["input_checks"][check] is False


@pytest.mark.parametrize("field,value", [
    ("task_complete", False), ("task_complete", 1), ("goal_reached", True),
    ("admission_status", "APPROVED"), ("dataset_conversion_performed", True),
    ("development_only", True), ("training_data_approved", False),
])
def test_success_requires_exact_pending_candidate_semantics(harness, field, value):
    harness["manifest"]["result"][field] = value
    harness["env"]["E2E_TRIAL_TEST_MOVING_MANIFEST"] = json.dumps(harness["manifest"])
    result = run(harness)
    assert result.returncode == 1, result.stderr
    proof = json.loads((harness["output"] / "owner_result.json").read_text())
    assert proof["exit_code"] == 1
    assert proof["moving_task_capture"]["capture_complete_pending_qa"] is False
    assert json.loads((harness["output"] / "lifecycle/stopped.json").read_text())["status"] == "PASS"


def test_failed_worker_preserves_failure_and_only_cleans_owned_children(harness):
    unrelated = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)"], start_new_session=True)
    try:
        result = run(harness, collector_options=options(harness) + ["--seed", "98"])
        assert result.returncode == 7, result.stderr
        assert unrelated.poll() is None
        events = [json.loads(row) for row in harness["events"].read_text().splitlines()]
        assert {row["role"] for row in events} == {"server", "collector"}
        for event in events:
            with pytest.raises(ProcessLookupError):
                os.kill(event["pid"], 0)
        owner = json.loads((harness["output"] / "owner_result.json").read_text())
        assert owner["exit_code"] == 7
        assert owner["learned_model_control"] is owner["vehicle_control_approved"] is False
    finally:
        unrelated.terminate()
        unrelated.wait(timeout=5)
