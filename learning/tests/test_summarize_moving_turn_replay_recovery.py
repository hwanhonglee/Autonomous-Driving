# HH_260906 - Locate the repository independently of the relocated test directory.
"""HH_260906 - Focused tests for failed-E plus posthoc subgroup comparison."""
from copy import deepcopy
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil

import pytest

# HH_260906 - Keep logical source IDs, but open canonical nonsymlink files in the moved workspace.
from learning.portable_e2e.workspace_paths import workspace_path as _hh_source_path


ROOT = next(p for p in Path(__file__).resolve().parents if (p / ".git").exists())
SCRIPT = _hh_source_path(ROOT, "scripts/e2e/summarize_moving_turn_replay_recovery.py")
SPEC = importlib.util.spec_from_file_location("moving_turn_replay_recovery_summary_test", SCRIPT)
m = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(m)

HELPER_PATH = ROOT / "learning/tests/test_summarize_moving_turn_replay.py"
HELPER_SPEC = importlib.util.spec_from_file_location("moving_turn_replay_base_fixture", HELPER_PATH)
h = importlib.util.module_from_spec(HELPER_SPEC)
assert HELPER_SPEC.loader is not None
HELPER_SPEC.loader.exec_module(h)
h.m = m.base


def put(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(m._encoded(value))


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def add_png_evidence(root: Path, arm: str) -> None:
    subgroup_path = root / arm / "subgroup/summary.json"
    subgroup = json.loads(subgroup_path.read_text())
    for label, row in subgroup["episodes"].items():
        values = {}
        for index in range(6):
            name = f"{index:04d}_{label}.png"
            path = subgroup_path.parent / "episodes" / label / "trajectories" / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(f"png-{arm}-{label}-{index}".encode())
            values[name] = sha(path)
        row["trajectory_png_sha256"] = values
    put(subgroup_path, subgroup)


def recovery_denials() -> dict:
    return {field: False for field in m.RECOVERY_DENIAL_FIELDS}


def build_fixture(tmp_path: Path, monkeypatch):
    reference, experiment = h.make_pair(tmp_path, monkeypatch)
    add_png_evidence(reference, m.base.REFERENCE_ARM)
    h.refresh_stage(reference, m.base.REFERENCE_ARM, "subgroup")
    add_png_evidence(experiment, m.ARM)

    subgroup_path = experiment / m.ARM / "subgroup/summary.json"
    subgroup = json.loads(subgroup_path.read_text())
    right = subgroup["episodes"]["town03_right_turn_dev"]
    right["metrics"].update({"ade_6p4s_m": 11.0, "fde_6p4s_m": 21.0,
                             "speed_mae_6p4s_mps": 4.3})
    report_path = subgroup_path.parent / right["report_path"]
    report = json.loads(report_path.read_text())
    report["metrics"] = right["metrics"]
    put(report_path, report)
    right["report_sha256"] = sha(report_path)
    put(subgroup_path, subgroup)

    evaluation_path = experiment / m.ARM / "evaluation/metrics.json"
    evaluation = json.loads(evaluation_path.read_text())
    for key in evaluation["metrics"]:
        evaluation["metrics"][key] = sum(
            subgroup["episodes"][spec["label"]]["metrics"][key] * spec["count"]
            for spec in m.base.EPISODES
        ) / 665
    evaluation["auxiliary_loss_metrics"] = {"candidate_regret_loss": 1.25}
    evaluation["auxiliary_loss_metric_counts"] = {"candidate_regret_loss": 665}
    put(evaluation_path, evaluation)

    experiment_plan_path = experiment / "plan.json"
    experiment_plan = json.loads(experiment_plan_path.read_text())
    experiment_plan["reference_C"]["files"]["summary"]["sha256"] = sha(reference / "summary.json")
    experiment_plan["reference_C"]["files"]["subgroup"]["sha256"] = sha(
        reference / m.base.REFERENCE_ARM / "subgroup/summary.json")
    put(experiment_plan_path, experiment_plan)
    h.refresh_plan_binding(experiment)
    h.refresh_stage(experiment, m.ARM, "evaluate")

    successful_summary = json.loads((experiment / "summary.json").read_text())
    for row in successful_summary["stages"]:
        if row["stage"] == "train":
            row["sampling_execution_schedule_sha256"] = (
                experiment_plan["arm"]["execution_schedule"]["sha256"])
            row["sampling_execution_schedule_row_count"] = 36_040
            put(experiment / m.ARM / "train_completed.json", row)
    stages = successful_summary["stages"][:3]
    recovery = tmp_path / "recovery"
    shutil.copytree(experiment / m.ARM / "subgroup", recovery / "subgroup")
    shutil.rmtree(experiment / m.ARM / "subgroup")
    (experiment / m.ARM / "subgroup_completed.json").unlink()
    (experiment / "summary.json").unlink()

    checkpoint_sha = sha(experiment / m.ARM / "training/checkpoints/latest.pt")
    put(experiment / m.ARM / "subgroup_started.json", {
        "status": "RUNNING", "stage": "subgroup", "input_checkpoint_sha256": checkpoint_sha,
    })
    (experiment / m.ARM / "subgroup.log").write_text(m.KNOWN_LOG)
    failed = {
        "schema": m.base.EXPERIMENT_SUMMARY_SCHEMA,
        "status": "FAIL",
        "arm": "FAIL",
        "plan_sha256": sha(experiment_plan_path),
        "error_type": "ValueError",
        "error": f"{m.ARM} subgroup exited 2",
        "pre_cuda_gpu": {"physical_index": 0, "uuid": "GPU-fixture",
                         "pre_cuda_compute_processes": 0},
        "stages": stages,
        **{field: False for field in m.base.DENIAL_FIELDS},
    }
    put(experiment / "failed.json", failed)

    original_plan = json.loads(experiment_plan_path.read_text())
    old_evaluator = original_plan["evaluator_sha256"]
    patched_bytes = b"# canonical tuple/list evaluator fixture\n"
    patched_sha = hashlib.sha256(patched_bytes).hexdigest()
    execution = deepcopy(original_plan)
    execution["evaluator_sha256"] = patched_sha
    put(recovery / "execution_plan.json", execution)
    execution_sha = sha(recovery / "execution_plan.json")

    recovered_subgroup_path = recovery / "subgroup/summary.json"
    recovered_subgroup = json.loads(recovered_subgroup_path.read_text())
    recovered_subgroup["plan_sha256"] = execution_sha
    recovered_subgroup["evaluator_sha256"] = patched_sha
    put(recovered_subgroup_path, recovered_subgroup)

    runner_source = experiment / "source/run_moving_turn_replay_seed_pilot.py"
    (recovery / "source").mkdir(parents=True)
    shutil.copyfile(runner_source, recovery / "source/original_runner.py")
    (recovery / "source/patched_evaluator.py").write_bytes(patched_bytes)
    (recovery / "subgroup.log").write_text("recovered\n")

    original_tree = m._inventory(experiment)
    required_paths = {
        "plan": "plan.json",
        "failed": "failed.json",
        "subgroup_log": f"{m.ARM}/subgroup.log",
        "checkpoint": f"{m.ARM}/training/checkpoints/latest.pt",
        "training_run": f"{m.ARM}/training/run.json",
        "evaluation": f"{m.ARM}/evaluation/metrics.json",
        "audit": f"{m.ARM}/gate_v8.json",
        "runner": "source/run_moving_turn_replay_seed_pilot.py",
        "old_evaluator": "source/evaluate_portable_moving_turn_replay_subgroups.py",
    }
    required = {}
    for key, relative in required_paths.items():
        path = experiment / relative
        required[key] = {"path": relative, "size": path.stat().st_size, "sha256": sha(path)}
    recovery_plan = {
        "schema": m.RECOVERY_PLAN_SCHEMA,
        "recovery_id": m.RECOVERY_ID,
        "gpu": {"physical_index": 0, "uuid": "GPU-fixture"},
        "stage_order": ["subgroup"],
        "subgroup_evaluation_executed": True,
        "execution_plan": {"path": "execution_plan.json", "sha256": execution_sha,
                           "original_plan_sha256": sha(experiment_plan_path),
                           "changed_fields": ["evaluator_sha256"]},
        "allowed_fix": {
            "scope": "subgroup_evaluator_train_config_representation_only",
            "changed_execution_plan_fields": ["evaluator_sha256"],
            "old_evaluator_sha256": old_evaluator,
            "patched_evaluator_sha256": patched_sha,
            "reason": "checkpoint tuple and plan list encode the same value",
        },
        "original_run": {"tree": original_tree, "required": required},
        **recovery_denials(),
    }
    put(recovery / "recovery_plan.json", recovery_plan)
    recovery_plan_sha = sha(recovery / "recovery_plan.json")
    command = ["python", "subgroup_evaluator.py", "--runner-sha256", sha(runner_source),
               "--plan-sha256", execution_sha, "--checkpoint-sha256", checkpoint_sha]
    started = {
        "schema": m.RECOVERY_SUMMARY_SCHEMA, "status": "RUNNING", "stage": "subgroup",
        "recovery_plan_sha256": recovery_plan_sha,
        "execution_plan_sha256": execution_sha,
        "original_plan_sha256": sha(experiment_plan_path),
        "checkpoint_sha256": checkpoint_sha,
        "pre_cuda_gpu": {"physical_index": 0, "uuid": "GPU-fixture",
                         "pre_cuda_compute_processes": 0},
        **recovery_denials(),
    }
    put(recovery / "started.json", started)
    stage_started = {
        "status": "RUNNING", "stage": "subgroup", "command": command,
        "input_checkpoint_sha256": checkpoint_sha, **recovery_denials(),
    }
    put(recovery / "subgroup_started.json", stage_started)
    subgroup_sha = sha(recovered_subgroup_path)
    completed = {
        **stage_started, "status": "COMPLETE", "path": "subgroup/summary.json",
        "sha256": subgroup_sha, "checkpoint_sha256": checkpoint_sha,
    }
    put(recovery / "completed.json", completed)
    recovery_summary = {
        "schema": m.RECOVERY_SUMMARY_SCHEMA,
        "status": "COMPLETE_NOT_PROMOTED",
        "recovery_plan_sha256": recovery_plan_sha,
        "execution_plan_sha256": execution_sha,
        "original_plan_sha256": sha(experiment_plan_path),
        "original_failed_run_unchanged": True,
        "stage_order": ["subgroup"],
        "subgroup_evaluation_executed": True,
        "subgroup": {"path": "subgroup/summary.json", "sha256": subgroup_sha,
                     "checkpoint_sha256": checkpoint_sha},
        "final_gpu": {"physical_index": 0, "uuid": "GPU-fixture",
                      "pre_cuda_compute_processes": 0},
        **recovery_denials(),
    }
    put(recovery / "recovery_summary.json", recovery_summary)

    monkeypatch.setattr(m, "GPU", {"physical_index": 0, "uuid": "GPU-fixture"})
    monkeypatch.setattr(m, "ORIGINAL_PLAN_SHA256", sha(experiment_plan_path))
    monkeypatch.setattr(m, "ORIGINAL_FAILED_SHA256", sha(experiment / "failed.json"))
    monkeypatch.setattr(m, "ORIGINAL_CHECKPOINT_SHA256", checkpoint_sha)
    monkeypatch.setattr(m, "ORIGINAL_TRAIN_SHA256", sha(experiment / m.ARM / "training/run.json"))
    monkeypatch.setattr(m, "ORIGINAL_HISTORY_SHA256", sha(experiment / m.ARM / "training/metrics.jsonl"))
    monkeypatch.setattr(m, "ORIGINAL_EVALUATION_SHA256", sha(evaluation_path))
    monkeypatch.setattr(m, "ORIGINAL_AUDIT_SHA256", sha(experiment / m.ARM / "gate_v8.json"))
    monkeypatch.setattr(m, "ORIGINAL_SCHEDULE_SHA256",
                        original_plan["arm"]["execution_schedule"]["sha256"])
    monkeypatch.setattr(m, "ORIGINAL_TREE_FILE_COUNT", original_tree["file_count"])
    monkeypatch.setattr(m, "ORIGINAL_TREE_TOTAL_SIZE", original_tree["total_size_bytes"])
    monkeypatch.setattr(m, "ORIGINAL_TREE_INVENTORY_SHA256", original_tree["inventory_sha256"])
    monkeypatch.setattr(m, "OLD_EVALUATOR_SHA256", old_evaluator)
    monkeypatch.setattr(m, "PATCHED_EVALUATOR_SHA256", patched_sha)
    monkeypatch.setattr(m, "RUNNER_SHA256", sha(runner_source))
    monkeypatch.setattr(m, "EXPECTED_MODEL_CONFIG", original_plan["model_config"])
    monkeypatch.setattr(m, "EXPECTED_MODEL_CONFIG_CANONICAL_SHA256",
                        original_plan["model_config_canonical_sha256"])
    monkeypatch.setattr(m, "EXPECTED_LOSS_CONFIG", original_plan["loss_config"])
    monkeypatch.setattr(m, "RECOVERY_PLAN_SHA256", recovery_plan_sha)
    monkeypatch.setattr(m, "RECOVERY_EXECUTION_PLAN_SHA256", execution_sha)
    monkeypatch.setattr(m, "RECOVERY_SUMMARY_SHA256", sha(recovery / "recovery_summary.json"))
    monkeypatch.setattr(m, "RECOVERY_SUBGROUP_SHA256", subgroup_sha)
    monkeypatch.setattr(m, "RECOVERY_COMPLETED_SHA256", sha(recovery / "completed.json"))
    recovery_tree = m._inventory(recovery)
    monkeypatch.setattr(m, "RECOVERY_FILE_COUNT", recovery_tree["file_count"])
    monkeypatch.setattr(m, "RECOVERY_TOTAL_SIZE", recovery_tree["total_size_bytes"])
    monkeypatch.setattr(m, "REFERENCE_PLAN_SHA256", sha(reference / "plan.json"))
    monkeypatch.setattr(m, "REFERENCE_SUMMARY_SHA256", sha(reference / "summary.json"))
    monkeypatch.setattr(m, "REFERENCE_CHECKPOINT_SHA256",
                        sha(reference / m.base.REFERENCE_ARM / "training/checkpoints/latest.pt"))
    monkeypatch.setattr(m, "REFERENCE_TRAIN_SHA256",
                        sha(reference / m.base.REFERENCE_ARM / "training/run.json"))
    monkeypatch.setattr(m, "REFERENCE_SUBGROUP_SHA256",
                        sha(reference / m.base.REFERENCE_ARM / "subgroup/summary.json"))
    monkeypatch.setattr(m, "REFERENCE_AUDIT_SHA256",
                        sha(reference / m.base.REFERENCE_ARM / "gate_v8.json"))
    return reference, experiment, recovery


def test_complete_recovery_is_compared_but_not_promoted(tmp_path, monkeypatch):
    reference, experiment, recovery = build_fixture(tmp_path, monkeypatch)
    result = m.summarize(reference_root=reference, experiment_root=experiment,
                         recovery_root=recovery)
    e_input = result["inputs"]["E_experiment"]
    assert set(e_input) == {"campaign_id", "plan_sha256", "failed_sha256",
                            "original_status", "posthoc_subgroup_recovery"}
    assert e_input["original_status"] == "FAIL_SUBGROUP"
    assert result["arm_provenance"]["E"]["subgroup_source"] == "posthoc_recovery"
    gate = result["preregistered_C_E_gate"]
    assert gate["runner_summary_exact_match"] is False
    assert gate["posthoc_recovery_subgroup_used"] is True
    assert gate["result"]["pass"] is False
    assert result["scope"]["unseen_confirmation_seeds_authorized"] is False
    assert result["scope"]["model_promotion"] is False
    assert result["verification"]["evaluation_contract"]["candidate_regret_count"] == 665


def test_only_evaluator_change_rejects_other_plan_change():
    original = {"evaluator_sha256": "1" * 64, "train_config": {"max_steps": 9020}}
    execution = {"evaluator_sha256": "2" * 64, "train_config": {"max_steps": 9021}}
    old, patched = m.OLD_EVALUATOR_SHA256, m.PATCHED_EVALUATOR_SHA256
    try:
        m.OLD_EVALUATOR_SHA256, m.PATCHED_EVALUATOR_SHA256 = "1" * 64, "2" * 64
        with pytest.raises(m.base.SummaryError, match="forbidden"):
            m._only_evaluator_changed(original, execution)
    finally:
        m.OLD_EVALUATOR_SHA256, m.PATCHED_EVALUATOR_SHA256 = old, patched


def test_candidate_regret_accounting_is_required(tmp_path, monkeypatch):
    reference, experiment, _ = build_fixture(tmp_path, monkeypatch)
    del reference
    evaluation_path = experiment / m.ARM / "evaluation/metrics.json"
    evaluation = json.loads(evaluation_path.read_text())
    evaluation.pop("auxiliary_loss_metric_counts")
    put(evaluation_path, evaluation)
    with pytest.raises(m.base.SummaryError, match="auxiliary loss counts"):
        m._verify_evaluation_contract(json.loads((experiment / "plan.json").read_text()), experiment)


def test_recovery_tree_is_bound_to_original_failed_tree(tmp_path, monkeypatch):
    _, experiment, recovery = build_fixture(tmp_path, monkeypatch)
    (experiment / m.ARM / "subgroup.log").write_text("tampered\n")
    with pytest.raises(m.base.SummaryError, match="full tree changed"):
        m._verify_recovery(recovery, experiment,
                           json.loads((experiment / "plan.json").read_text()))
