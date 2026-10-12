# HH_260906 - Locate the repository independently of the relocated test directory.
"""HH_260906 - CPU/mock tests for the create-only E subgroup recovery wrapper."""
from contextlib import contextmanager
from copy import deepcopy
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

# HH_260906 - Keep logical source IDs, but open canonical nonsymlink files in the moved workspace.
from learning.portable_e2e.workspace_paths import workspace_path as _hh_source_path


ROOT = next(p for p in Path(__file__).resolve().parents if (p / ".git").exists())
PATH = _hh_source_path(ROOT, "scripts/e2e/run_moving_turn_replay_subgroup_recovery.py")
SPEC = importlib.util.spec_from_file_location("moving_turn_replay_subgroup_recovery_test", PATH)
m = importlib.util.module_from_spec(SPEC); SPEC.loader.exec_module(m)


def put(path, value):
    path.parent.mkdir(parents=True, exist_ok=True); path.write_bytes(m.encoded(value))


def file_sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def configure_fixture(tmp_path, monkeypatch):
    workspace = tmp_path / "portable_e2e"; repo = workspace / "repo"; repo.mkdir(parents=True)
    original = workspace / m.ORIGINAL_ROOT; arm = original / m.ARM
    runner = original / m.ORIGINAL_RUNNER_PATH; runner.parent.mkdir(parents=True); runner.write_text("# runner fixture\n")
    old_evaluator = original / m.ORIGINAL_EVALUATOR_PATH; old_evaluator.write_text("# old evaluator fixture\n")
    checkpoint = arm / "training/checkpoints/latest.pt"; checkpoint.parent.mkdir(parents=True); checkpoint.write_bytes(b"checkpoint")
    patched = workspace / "declarations/evaluate_portable_moving_turn_replay_subgroups.py"
    patched.parent.mkdir(parents=True); patched.write_text("# patched evaluator fixture\n")
    runner_sha, old_sha, checkpoint_sha, patched_sha = map(file_sha, (runner, old_evaluator, checkpoint, patched))
    monkeypatch.setattr(m, "RUNNER_SHA256", runner_sha); monkeypatch.setattr(m, "OLD_EVALUATOR_SHA256", old_sha)
    monkeypatch.setattr(m, "CHECKPOINT_SHA256", checkpoint_sha); monkeypatch.setattr(m, "PATCHED_EVALUATOR_SHA256", patched_sha)
    allowed = deepcopy(m.ALLOWED_FIX); allowed["old_evaluator_sha256"] = old_sha
    allowed["patched_evaluator_sha256"] = patched_sha; monkeypatch.setattr(m, "ALLOWED_FIX", allowed)
    source = {"current_sha256": {m.PIPELINE_PATH: "a" * 64}}
    original_plan = {"campaign_id": m.ORIGINAL_CAMPAIGN_ID, "output": m.ORIGINAL_ROOT,
        "runner_sha256": runner_sha, "evaluator_sha256": old_sha, "source": source,
        "test_read_or_use": False, "automatic_promotion": False}
    plan_path = original / "plan.json"; put(plan_path, original_plan); plan_sha = file_sha(plan_path)
    monkeypatch.setattr(m, "ORIGINAL_PLAN_SHA256", plan_sha)
    train = {"status": "TRAINING_TARGET_REACHED",
        "state": {"epoch": 20, "next_batch_index": 0, "global_step": 9020,
            "samples_seen": 36040, "domain_samples_seen": {"carla": 36040}},
        "sampling_execution_schedule_row_count": 36040,
        "sampling_execution_schedule_sha256": "b" * 64}
    evaluation = {"status": "OPEN_LOOP_EVALUATION_COMPLETE", "checkpoint_sha256": checkpoint_sha,
        "evaluation_split": "val", "sample_count": 665}
    audit = {"status": "RUNTIME_GEOMETRY_AUDIT_COMPLETE", "checkpoint_sha256": checkpoint_sha,
        "evaluation_split": "val", "geometry": {"sample_count": 665}}
    put(arm / "training/run.json", train); put(arm / "evaluation/metrics.json", evaluation); put(arm / "gate_v8.json", audit)
    stages = [{"stage": stage, "status": "COMPLETE", "checkpoint_sha256": checkpoint_sha}
              for stage in ("train", "evaluate", "audit")]
    put(original / "failed.json", {"status": "FAIL", "plan_sha256": plan_sha,
        "error_type": m.KNOWN_FAILURE["error_type"], "error": m.KNOWN_FAILURE["error"],
        "arm": "FAIL", "stages": stages})
    log = arm / "subgroup.log"; log.parent.mkdir(parents=True, exist_ok=True); log.write_text(m.KNOWN_FAILURE["subgroup_log_exact"])
    owner = workspace / m.OWNER_PATH; owner.parent.mkdir(parents=True)
    owner.write_bytes((_hh_source_path(ROOT, "scripts/e2e/profile_portable_training.py")).read_bytes())
    pipeline = repo / m.PIPELINE_PATH; pipeline.parent.mkdir(parents=True); pipeline.write_text("# pipeline fixture\n")
    fake_runner = SimpleNamespace(validate_plan=lambda plan, runner_sha256: plan, verify_sources=lambda *_: None)
    declarations = workspace / "declarations"; recovery_plan = declarations / "recovery_plan.json"
    execution_plan = declarations / "execution_plan.json"
    return SimpleNamespace(workspace=workspace, repo=repo, original=original, patched=patched,
        runner=runner, checkpoint=checkpoint, fake_runner=fake_runner,
        recovery_plan=recovery_plan, execution_plan=execution_plan, original_plan=original_plan)


def declare(fixture):
    return m.declare_plan(workspace=fixture.workspace, repo_root=fixture.repo,
        original_run_root=fixture.original, patched_evaluator=fixture.patched,
        output_plan=fixture.recovery_plan, output_execution_plan=fixture.execution_plan,
        runner_module=fixture.fake_runner)


def test_declare_binds_known_failure_and_changes_only_evaluator(tmp_path, monkeypatch):
    fixture = configure_fixture(tmp_path, monkeypatch); plan, execution = declare(fixture)
    assert plan["original_run"]["known_failure"] == m.KNOWN_FAILURE
    assert plan["stage_order"] == ["subgroup"] and plan["automatic_retry"] is False
    assert plan["training_executed"] is plan["evaluate_stage_executed"] is plan["audit_executed"] is False
    assert execution["evaluator_sha256"] == m.PATCHED_EVALUATOR_SHA256
    for key in fixture.original_plan:
        if key != "evaluator_sha256": assert execution[key] == fixture.original_plan[key]
    assert file_sha(fixture.execution_plan) == plan["execution_plan"]["sha256"]
    assert m.validate_plan(m.read(fixture.recovery_plan)) == plan


def test_declare_rejects_any_known_failure_text_change(tmp_path, monkeypatch):
    fixture = configure_fixture(tmp_path, monkeypatch)
    (fixture.original / m.REQUIRED_FILES["subgroup_log"]).write_text("different failure\n")
    with pytest.raises(ValueError, match="failure log"):
        declare(fixture)
    assert not fixture.recovery_plan.exists() and not fixture.execution_plan.exists()


@pytest.mark.parametrize("field,value", [("automatic_retry", True), ("training_executed", True),
    ("evaluate_stage_executed", True), ("audit_executed", True), ("test_read_or_use", True),
    ("automatic_promotion", True), ("stage_order", ["train", "subgroup"])])
def test_recovery_plan_rejects_scope_expansion(tmp_path, monkeypatch, field, value):
    fixture = configure_fixture(tmp_path, monkeypatch); plan, _ = declare(fixture)
    mutated = deepcopy(plan); mutated[field] = value
    with pytest.raises(ValueError): m.validate_plan(mutated)


@contextmanager
def nothing(*_args):
    yield


def mock_owners(fixture, *, rc=0):
    owner = SimpleNamespace(check_environment=lambda *_: None, gpu_lease=nothing,
        gpu_idle=lambda *_: dict(m.GPU, pre_cuda_compute_processes=0), bounded_signals=nothing,
        finalization_signals=nothing)
    calls = []
    def stage(command, _repo, _env, _log, timeout):
        calls.append((command, timeout))
        if rc: return rc
        output = Path(command[command.index("--output-dir") + 1]); output.mkdir(parents=True)
        plan_sha = command[command.index("--plan-sha256") + 1]
        summary = {"status": "COMPLETE_NOT_PROMOTED", "arm": m.ARM, "plan_sha256": plan_sha,
            "runner_sha256": m.RUNNER_SHA256, "evaluator_sha256": m.PATCHED_EVALUATOR_SHA256,
            "checkpoint_sha256": m.CHECKPOINT_SHA256, "combined_val_count": 665,
            "test_read_or_use": False, "training_executed_by_evaluator": False,
            "automatic_promotion": False, "vehicle_control_approved": False,
            "closed_loop_evaluation": False, "episodes": {
                "town03_right_turn_dev": {"count": 337}, "town03_straight_dev": {"count": 328}}}
        put(output / "summary.json", summary); return 0
    return owner, SimpleNamespace(run_owned_stage=stage), calls


def test_run_executes_only_subgroup_and_preserves_original_tree(tmp_path, monkeypatch):
    fixture = configure_fixture(tmp_path, monkeypatch); plan, _ = declare(fixture)
    before = m.tree_inventory(fixture.original); owner, pipeline, calls = mock_owners(fixture)
    output = m.run(recovery_plan=fixture.recovery_plan, execution_plan=fixture.execution_plan,
        workspace=fixture.workspace, repo_root=fixture.repo, patched_evaluator=fixture.patched,
        runner_module=fixture.fake_runner, owner_module=owner, pipeline_module=pipeline)
    assert m.tree_inventory(fixture.original) == before and len(calls) == 1
    command = calls[0][0]
    assert "--checkpoint" in command and "--output-dir" in command
    assert all(value not in command for value in ("portable_e2e.train", "portable_e2e.evaluate",
                                                    "portable_e2e.audit_runtime", "--resume"))
    summary = m.read(output / "recovery_summary.json")
    assert summary["status"] == "COMPLETE_NOT_PROMOTED" and summary["original_failed_run_unchanged"] is True
    assert summary["training_executed"] is summary["evaluate_stage_executed"] is summary["audit_executed"] is False
    assert not (output / "training").exists() and not (output / "evaluation").exists()
    assert (output / "completed.json").exists() and not (output / "failed.json").exists()


def test_run_failure_is_create_only_and_does_not_touch_original(tmp_path, monkeypatch):
    fixture = configure_fixture(tmp_path, monkeypatch); declare(fixture)
    before = m.tree_inventory(fixture.original); owner, pipeline, calls = mock_owners(fixture, rc=17)
    with pytest.raises(ValueError, match="exited 17"):
        m.run(recovery_plan=fixture.recovery_plan, execution_plan=fixture.execution_plan,
            workspace=fixture.workspace, repo_root=fixture.repo, patched_evaluator=fixture.patched,
            runner_module=fixture.fake_runner, owner_module=owner, pipeline_module=pipeline)
    output = fixture.workspace / m.OUTPUT_PATH
    assert m.tree_inventory(fixture.original) == before and len(calls) == 1
    assert (output / "failed.json").exists() and not (output / "completed.json").exists()
    assert not (output / "recovery_summary.json").exists()


def test_run_refuses_original_mutation_before_creating_output(tmp_path, monkeypatch):
    fixture = configure_fixture(tmp_path, monkeypatch); declare(fixture)
    with (fixture.original / "failed.json").open("a") as stream: stream.write("\n")
    owner, pipeline, _ = mock_owners(fixture)
    with pytest.raises(ValueError, match="original failed run changed"):
        m.run(recovery_plan=fixture.recovery_plan, execution_plan=fixture.execution_plan,
            workspace=fixture.workspace, repo_root=fixture.repo, patched_evaluator=fixture.patched,
            runner_module=fixture.fake_runner, owner_module=owner, pipeline_module=pipeline)
    assert not (fixture.workspace / m.OUTPUT_PATH).exists()


def test_load_module_does_not_create_bytecode_in_sealed_tree(tmp_path):
    source = tmp_path / "sealed/source/helper.py"; source.parent.mkdir(parents=True)
    source.write_text("VALUE = 17\n")
    helper = m.load_module(source, file_sha(source), "_recovery_no_bytecode_fixture")
    assert helper.VALUE == 17
    assert not (source.parent / "__pycache__").exists()


def test_run_refuses_wrong_gpu0_uuid_before_creating_output(tmp_path, monkeypatch):
    fixture = configure_fixture(tmp_path, monkeypatch); declare(fixture)
    owner, pipeline, calls = mock_owners(fixture)
    owner.gpu_idle = lambda *_: {"physical_index": 0, "uuid": "GPU-wrong",
                                  "pre_cuda_compute_processes": 0}
    with pytest.raises(ValueError, match="physical GPU0"):
        m.run(recovery_plan=fixture.recovery_plan, execution_plan=fixture.execution_plan,
            workspace=fixture.workspace, repo_root=fixture.repo, patched_evaluator=fixture.patched,
            runner_module=fixture.fake_runner, owner_module=owner, pipeline_module=pipeline)
    assert calls == [] and not (fixture.workspace / m.OUTPUT_PATH).exists()


def test_cli_freezes_separate_declaration_and_execution_files():
    args = m.main
    assert callable(args)
    assert m.OUTPUT_PATH != m.ORIGINAL_ROOT and m.TIMEOUT_SECONDS < m.WHOLE_TIMEOUT_SECONDS
