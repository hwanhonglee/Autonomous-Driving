# HH_260906 - Locate the repository independently of the relocated test directory.
"""HH_260906 - CPU/mock-only tests for fresh C/D fits at 7,380 steps; never train or use SSH."""
from contextlib import contextmanager
from copy import deepcopy
from datetime import datetime, timezone
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
PATH = _hh_source_path(ROOT, "scripts/e2e/run_moving_data_exposure_match_pilot.py")
SPEC = importlib.util.spec_from_file_location("moving_data_exposure_match_pilot_test", PATH)
m = importlib.util.module_from_spec(SPEC); SPEC.loader.exec_module(m)


def put(path, value):
    path.parent.mkdir(parents=True, exist_ok=True); path.write_bytes(m.encoded(value))


def fixture_plan():
    common_val_examples = "e" * 64; common_val_tokens = "f" * 64
    arms = {}
    for index, arm in enumerate(m.ARMS):
        counts = m.ARM_COUNTS[arm]
        train_examples = ("a" if index == 0 else "b") * 64
        split_examples = {"train": train_examples, "val": common_val_examples}
        split_tensor = {split: hashlib.sha256((value + "\0" + split).encode()).hexdigest()
                        for split, value in split_examples.items()}
        val_ids = [row["episode_id"] for row in m.VALIDATION_EPISODES]
        train_ids = [f"train_{index}_{item}" for item in range(3 + index)]
        arms[arm] = {"dataset": {"path": f"datasets/prepared/{arm}",
            "manifest_sha256": str(index + 1) * 64, "corpus_fingerprint_sha256": str(index + 3) * 64,
            "split_fingerprint_sha256": split_tensor,
            "split_loader_sha256": {"train": str(index + 5) * 64, "val": str(index + 7) * 64},
            "split_examples_sha256": split_examples,
            "split_tokens_sha256": {"train": str(index + 8) * 64, "val": common_val_tokens},
            "counts": counts, "episode_ids": {"train": train_ids, "val": val_ids},
            "tree": {"manifest_sha256": "9" * 64, "file_count": 1, "directory_count": 1,
                "total_size_bytes": 2, "files": [{"path": "dataset.json", "size": 2,
                    "sha256": str(index + 1) * 64}]}},
            "expected_train_state": m.EXPECTED_TRAIN_STATE[arm], "train_config": dict(m.ARM_TRAIN_CONFIG[arm])}
    return {"schema": m.PLAN_SCHEMA, "campaign_id": m.CAMPAIGN_ID,
        "declared_at_utc": "2026-09-21T00:00:00Z", "source_commit": m.SOURCE_COMMIT,
        "gpu": dict(m.GPU), "source_sha256": {name: (m.MODEL_SHA256 if name == m.MODEL_PATH else "c" * 64)
            for name in m.SOURCE_PATHS}, "runner_sha256": m.digest(PATH), "evaluator_sha256": "d" * 64,
        "owner": {"path": m.OWNER_PATH, "sha256": m.OWNER_SHA256},
        "model_config": m.MODEL_PATH, "model_config_sha256": m.MODEL_SHA256,
        "model_config_canonical_sha256": m.MODEL_CANONICAL_SHA256, "model_id": m.MODEL_ID,
        "view_report_sha256": "b" * 64, "output": m.OUTPUT_PATH, "arms": arms,
        "validation": {"episodes": list(m.VALIDATION_EPISODES), "count": 665,
            "fingerprint_sha256": common_val_examples, "tokens_sha256": common_val_tokens},
        "train_config": dict(m.TRAIN_CONFIG), "loss_config": dict(m.LOSS_CONFIG),
        "stage_order": list(m.STAGES), "stage_timeout_seconds": dict(m.TIMEOUTS),
        "whole_timeout_seconds": m.WHOLE_TIMEOUT_SECONDS,
        "decision": dict(m.DECISION), **m.DENIALS}


def test_exact_plan_and_commands_freeze_one_data_variable(tmp_path):
    plan = fixture_plan()
    m.validate_plan(plan, runner_sha256=m.digest(PATH), observed=datetime(2026, 9, 21, 1, tzinfo=timezone.utc))
    evaluator = tmp_path / "evaluate.py"; evaluator.write_text("# fixture\n")
    output = tmp_path / "out"; plan_path = tmp_path / "plan.json"; put(plan_path, plan)
    for arm in m.ARMS:
        train = m.stage_command(plan, tmp_path, ROOT, output, plan_path, evaluator, arm, "train")
        assert train[train.index("--seed") + 1] == "20260906"
        assert train[train.index("--max-steps") + 1] == str(m.EXPECTED_TRAIN_STATE[arm]["global_step"])
        assert train[train.index("--candidate-regret-weight") + 1] == "0.1"
        assert train[train.index("--checkpoint-interval") + 1] == "154"
        assert "--resume" not in train and "test" not in train
        checkpoint = output / arm / "training/checkpoints/latest.pt"
        checkpoint.parent.mkdir(parents=True); checkpoint.write_bytes(b"opaque command fixture")
        subgroup = m.stage_command(plan, tmp_path, ROOT, output, plan_path, evaluator, arm, "subgroup")
        for flag in ("--repo-root", "--plan", "--plan-sha256", "--arm", "--checkpoint",
                     "--checkpoint-sha256", "--output-dir"):
            assert subgroup.count(flag) == 1
    assert m.EXPECTED_TRAIN_STATE[m.ARMS[0]] == {"epoch": 25, "next_batch_index": 205,
        "global_step": 7380, "samples_seen": 29495, "domain_samples_seen": {"carla": 29495}}
    assert m.EXPECTED_TRAIN_STATE[m.ARMS[1]] == {"epoch": 20, "next_batch_index": 0,
        "global_step": 7380, "samples_seen": 29500, "domain_samples_seen": {"carla": 29500}}


def test_exact_runner_plan_is_accepted_by_real_subgroup_evaluator():
    # HH_260906 - Keep the independently owned runner/evaluator plan projection executable together.
    path = _hh_source_path(ROOT, "scripts/e2e/evaluate_portable_moving_data_subgroups.py")
    spec = importlib.util.spec_from_file_location("exposure_match_subgroup_contract_integration", path)
    evaluator = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = evaluator; spec.loader.exec_module(evaluator)
    parsed = evaluator.parse_plan(fixture_plan())
    assert tuple(parsed.arms) == m.ARMS
    assert parsed.validation_count == 665
    assert parsed.arms[m.ARMS[0]].train_fingerprint_sha256 == \
        fixture_plan()["arms"][m.ARMS[0]]["dataset"]["split_fingerprint_sha256"]["train"]


def test_tensor_fingerprint_is_not_loader_fingerprint():
    plan = fixture_plan(); data = plan["arms"][m.ARMS[0]]["dataset"]
    expected = hashlib.sha256((data["split_examples_sha256"]["train"] + "\0train").encode()).hexdigest()
    assert data["split_fingerprint_sha256"]["train"] == expected
    assert data["split_fingerprint_sha256"]["train"] != data["split_loader_sha256"]["train"]
    broken = deepcopy(plan)
    broken["arms"][m.ARMS[0]]["dataset"]["split_fingerprint_sha256"]["train"] = \
        broken["arms"][m.ARMS[0]]["dataset"]["split_loader_sha256"]["train"]
    # HH_260906 - The declaration validator rejects a loader/checkpoint fingerprint substitution.
    with pytest.raises(ValueError, match="tensor fingerprint"):
        m.validate_plan(broken, runner_sha256=m.digest(PATH))


@pytest.mark.parametrize("field,value", [
    ("resume", True), ("automatic_retry", True), ("test_read_or_use", True),
    ("automatic_promotion", True), ("model_export", True), ("source_commit", "f" * 40),
    ("whole_timeout_seconds", 18001), ("stage_order", ["train", "evaluate"]),
])
def test_plan_rejects_scope_or_safety_mutation(field, value):
    plan = fixture_plan(); plan[field] = value
    with pytest.raises(ValueError): m.validate_plan(plan, runner_sha256=m.digest(PATH))


def test_full_inventory_refuses_unplanned_and_test_entries(tmp_path):
    root = tmp_path / "view"; root.mkdir(); (root / "dataset.json").write_text("{}")
    expected = {"dataset.json": {"path": "dataset.json", "size": 2,
        "sha256": m.digest(root / "dataset.json")}}
    m.verify_tree(root, expected)
    (root / "extra.json").write_text("{}")
    with pytest.raises(ValueError, match="unplanned"): m.verify_tree(root, expected)
    (root / "extra.json").unlink(); (root / "test_payload.json").write_text("{}")
    with pytest.raises(ValueError, match="TEST"): m.verify_tree(root, expected)


def test_probe_is_cpu_only_and_uses_personal_venv(tmp_path, monkeypatch):
    workspace = tmp_path / "workspace"; repo = workspace / "repo"; root = workspace / "dataset"
    repo.mkdir(parents=True); root.mkdir(); (workspace / "venvs/py312/bin").mkdir(parents=True)
    captured = {}
    def check(command, **kwargs):
        captured.update(command=command, kwargs=kwargs)
        return json.dumps({"train": {}, "val": {}})
    monkeypatch.setattr(m.subprocess, "check_output", check)
    assert m.probe_view(workspace, repo, root) == {"train": {}, "val": {}}
    assert captured["command"][0] == str(workspace / "venvs/py312/bin/python")
    assert captured["kwargs"]["env"]["CUDA_VISIBLE_DEVICES"] == ""
    assert "PYTHONPATH" not in captured["kwargs"]["env"]


@contextmanager
def nothing(*_args):
    yield


@pytest.mark.parametrize("fail", [False, True])
def test_owned_mock_pipeline_is_sequential_fresh_and_retains_failure(tmp_path, monkeypatch, fail):
    workspace = tmp_path / "personal/hwanhong/portable_e2e"; repo = workspace / "autoware_e2e"
    repo.mkdir(parents=True); external = workspace / "benchmarks/moving_ab"; external.mkdir(parents=True)
    runner = external / PATH.name; runner.write_bytes(PATH.read_bytes())
    evaluator = external / "evaluate_portable_moving_data_subgroups.py"; evaluator.write_text("# fixture evaluator\n")
    owner = workspace / m.OWNER_PATH; owner.parent.mkdir(parents=True)
    owner.write_bytes((_hh_source_path(ROOT, "scripts/e2e/profile_portable_training.py")).read_bytes())
    pipeline_path = repo / m.PIPELINE_PATH; pipeline_path.parent.mkdir(parents=True)
    pipeline_path.write_bytes((_hh_source_path(ROOT, m.PIPELINE_PATH)).read_bytes())
    plan = fixture_plan(); plan["runner_sha256"] = m.digest(runner); plan["evaluator_sha256"] = m.digest(evaluator)
    plan["source_sha256"][m.PIPELINE_PATH] = m.digest(pipeline_path)
    plan_path = external / "plan.json"; put(plan_path, plan)
    report = external / "view_report.json"; put(report, {"schema": m.VIEW_SCHEMA})
    plan["view_report_sha256"] = m.digest(report); put(plan_path, plan)
    calls = []
    def stage(command, _repo, _env, _log, timeout):
        arm = next(value for value in m.ARMS if value in " ".join(command)); calls.append((arm, command, timeout))
        item = workspace / m.OUTPUT_PATH / arm
        if len(calls) == 1 and fail: return 17
        if "portable_e2e.train" in command:
            checkpoint = item / "training/checkpoints/latest.pt"; checkpoint.parent.mkdir(parents=True)
            checkpoint.write_bytes(b"opaque mock checkpoint")
        return 0
    owner_api = SimpleNamespace(check_environment=lambda *_: None, gpu_lease=nothing,
        gpu_idle=lambda *_: dict(m.GPU, pre_cuda_compute_processes=0), bounded_signals=nothing,
        finalization_signals=nothing)
    pipeline_api = SimpleNamespace(run_owned_stage=stage)
    monkeypatch.setattr(m, "verify_sources", lambda *_: None)
    monkeypatch.setattr(m, "verify_views", lambda *_: None)
    monkeypatch.setattr(m, "_archive", lambda *_: None)
    monkeypatch.setattr(m, "load_module", lambda path, _sha, name: owner_api if "gpu_owner" in name else pipeline_api)
    def verify(_plan, output, arm, stage_name):
        item = output / arm; target = item / (stage_name + "_mock.json"); put(target, {"stage": stage_name})
        checkpoint = item / "training/checkpoints/latest.pt"
        return {"path": target.relative_to(output).as_posix(), "sha256": m.digest(target),
            "checkpoint_sha256": m.digest(checkpoint)}
    monkeypatch.setattr(m, "verify_stage", verify)
    monkeypatch.setattr(m, "comparison", lambda *_: {"single_seed_exploratory_only": True})
    if fail:
        with pytest.raises(ValueError, match="exited 17"):
            m.run(plan_path=plan_path, repo_root=repo, workspace=workspace,
                  view_report=report, evaluator=evaluator, runner=runner)
        failed = m.read(workspace / m.OUTPUT_PATH / "failed.json")
        assert len(calls) == 1 and failed["arms"][m.ARMS[0]] == "FAIL"
        assert failed["arms"][m.ARMS[1]] == "NOT_RUN" and not (workspace / m.OUTPUT_PATH / "summary.json").exists()
    else:
        m.run(plan_path=plan_path, repo_root=repo, workspace=workspace,
              view_report=report, evaluator=evaluator, runner=runner)
        summary = m.read(workspace / m.OUTPUT_PATH / "summary.json")
        assert len(calls) == 8 and summary["status"] == "COMPLETE_NOT_PROMOTED"
        assert [name for _, command, _ in calls for name in [command[2] if len(command) > 2 else ""]].count("portable_e2e.train") == 2
        assert set(summary["arms"].values()) == {"COMPLETE"}
        assert summary["automatic_retry"] is False and summary["test_read_or_use"] is False


def test_higher_shared_step_budget_preserves_the_historical_runner():
    original = _hh_source_path(ROOT, "scripts/e2e/run_moving_data_ab_seed_pilot.py")
    assert m.digest(original) == "a6852f9fec12c24201abf9ac45b8661aad6fe91f2f71d51fecb8f96f4106d70e"
    assert "max_steps" not in m.TRAIN_CONFIG
    assert m.OUTPUT_PATH != "runs/diagnostics/moving_data_ab_seed20260906_20260921_v1"
    assert m.CAMPAIGN_ID != "hh260921-moving-data-ab-seed20260906-v1"
    for arm in m.ARMS:
        config = m.ARM_TRAIN_CONFIG[arm]
        state = m.EXPECTED_TRAIN_STATE[arm]
        size = m.ARM_COUNTS[arm]["train"]
        assert config == dict(m.TRAIN_CONFIG, max_steps=7380)
        assert state["global_step"] == 7380
        assert state["samples_seen"] == state["epoch"] * size + state["next_batch_index"] * 4
    assert m.DECISION["same_optimizer_steps"] is True
    assert m.DECISION["optimizer_steps_per_arm"] == 7380
    assert m.DECISION["same_complete_epochs"] is False
    assert m.DECISION["three_seed_confirmation_required"] is True
    assert m.DECISION["adoption_allowed"] is False
    assert 2 * sum(m.TIMEOUTS.values()) + 600 <= m.WHOLE_TIMEOUT_SECONDS


@pytest.mark.parametrize("change", ["B_steps", "A_steps", "B_epoch", "threshold", "promotion", "val_tokens"])
def test_exposure_or_diagnostic_contract_mutation_is_rejected(change):
    plan = deepcopy(fixture_plan())
    if change == "B_steps":
        plan["arms"][m.ARMS[1]]["train_config"]["max_steps"] = 5740
    elif change == "A_steps":
        plan["arms"][m.ARMS[0]]["train_config"]["max_steps"] = 5740
    elif change == "B_epoch":
        plan["arms"][m.ARMS[1]]["expected_train_state"]["epoch"] = 19
    elif change == "threshold":
        plan["decision"]["maximum_D_over_C"]["town03_straight_dev"]["ade_6p4s_m"] = 0.95
    elif change == "promotion":
        plan["decision"]["adoption_allowed"] = True
    else:
        plan["arms"][m.ARMS[1]]["dataset"]["split_tokens_sha256"]["val"] = "0" * 64
    with pytest.raises(ValueError):
        m.validate_plan(plan, runner_sha256=m.digest(PATH))


@pytest.mark.parametrize("arm", m.ARMS)
def test_full_history_requires_the_arm_specific_final_step(tmp_path, arm):
    state = m.EXPECTED_TRAIN_STATE[arm]
    path = tmp_path / "metrics.jsonl"
    with path.open("w") as stream:
        for step in range(1, state["global_step"] + 1):
            stream.write(json.dumps({"global_step": step, "samples_seen": state["samples_seen"]}) + "\n")
    assert m.verify_history(path, state) == m.digest(path)
    with path.open("a") as stream:
        stream.write(json.dumps({"global_step": state["global_step"] + 1, "samples_seen": state["samples_seen"]}) + "\n")
    with pytest.raises(ValueError, match="endpoint"):
        m.verify_history(path, state)


def gate_inputs():
    reports, audits = {}, {}
    for arm in m.ARMS:
        rows = {}
        for episode in m.VALIDATION_EPISODES:
            label = episode["label"]
            metrics = {key: (10.0 if arm == m.ARMS[0] else 10.0 * ratio)
                       for key, ratio in m.DECISION["maximum_D_over_C"][label].items()}
            rows[label] = dict(episode_id=episode["episode_id"], count=episode["count"], metrics=metrics,
                metric_counts={key: episode["count"] for key in metrics})
        reports[arm] = {"episodes": rows}
        audits[arm] = {"geometry": {"sample_count": 665,
            "any_candidate_geometry_pass_count": 664, "all_candidates_geometry_pass_count": 600,
            "selected_result": {"sample_count": 665, "geometry_pass_count": 664}}}
    return reports, audits


def test_diagnostic_threshold_boundaries_pass_without_adoption():
    reports, audits = gate_inputs()
    result = m.diagnostic_gate(reports, audits)
    assert result["status"] == "PASS_SINGLE_SEED_DIAGNOSTIC"
    assert result["pass"] is True
    assert result["adoption_allowed"] is result["automatic_promotion"] is False
    assert result["three_seed_confirmation_required"] is True


@pytest.mark.parametrize("label,key", [("town03_straight_dev", "ade_6p4s_m"),
    ("town03_straight_dev", "fde_6p4s_m"), ("town03_right_turn_dev", "ade_6p4s_m"),
    ("town03_right_turn_dev", "fde_6p4s_m"), ("town03_right_turn_dev", "speed_mae_6p4s_mps")])
def test_each_metric_can_fail_the_diagnostic_gate(label, key):
    reports, audits = gate_inputs()
    reports[m.ARMS[1]]["episodes"][label]["metrics"][key] += 0.001
    result = m.diagnostic_gate(reports, audits)
    assert result["status"] == "FAIL_SINGLE_SEED_DIAGNOSTIC"
    assert result["pass"] is result["adoption_allowed"] is False


@pytest.mark.parametrize("case", ["lower_pass_count", "lower_any_count", "lower_all_count",
    "denominator", "metric_count", "missing", "nonfinite"])
def test_geometry_regression_or_incomplete_evidence_cannot_pass(case):
    reports, audits = gate_inputs()
    if case == "lower_pass_count":
        audits[m.ARMS[1]]["geometry"]["selected_result"]["geometry_pass_count"] = 663
        assert m.diagnostic_gate(reports, audits)["pass"] is False
        return
    if case in ("lower_any_count", "lower_all_count"):
        field = "any_candidate_geometry_pass_count" if case == "lower_any_count" else "all_candidates_geometry_pass_count"
        audits[m.ARMS[1]]["geometry"][field] -= 1
        assert m.diagnostic_gate(reports, audits)["pass"] is False
        return
    if case == "denominator":
        audits[m.ARMS[1]]["geometry"]["selected_result"]["sample_count"] = 664
    elif case == "metric_count":
        reports[m.ARMS[1]]["episodes"]["town03_straight_dev"]["metric_counts"]["ade_6p4s_m"] = 327
    elif case == "missing":
        del reports[m.ARMS[1]]["episodes"]["town03_straight_dev"]["metrics"]["ade_6p4s_m"]
    else:
        reports[m.ARMS[1]]["episodes"]["town03_straight_dev"]["metrics"]["ade_6p4s_m"] = float("nan")
    with pytest.raises(ValueError, match="diagnostic gate"):
        m.diagnostic_gate(reports, audits)
