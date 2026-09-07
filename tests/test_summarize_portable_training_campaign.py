"""HH_260906 - Verify campaign comparison gates using synthetic evidence without GPU execution."""

from __future__ import annotations

import copy
from dataclasses import asdict
import importlib.util
import json
from pathlib import Path

import pytest

from portable_e2e.contract import ContractError
from test_portable_e2e_compare import _report


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "summarize_campaign", ROOT / "scripts/e2e/summarize_portable_training_campaign.py"
)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _write(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding="utf-8")


@pytest.fixture
def campaign(tmp_path: Path, monkeypatch) -> Path:
    # HH_260906 - Keep source-byte checks active while replacing git access with deterministic fixture bytes.
    plan = json.loads((ROOT / "config/portable_e2e_lr_ab_20260907.json").read_text())
    config_payload = (ROOT / plan["model_config"]).read_bytes()
    config = json.loads(config_payload)
    source_bytes = {
        plan["model_config"]: config_payload,
        "portable_e2e/audit_runtime.py": b"fixture audit",
        "portable_e2e/runtime_contract.py": b"fixture gate",
    }
    monkeypatch.setattr(MODULE, "_git_bytes", lambda commit, path: source_bytes[path])
    root = tmp_path / "campaign"
    _write(root / "plan.json", plan)
    stages = []
    for seed in MODULE.SEEDS:
        for arm, lr in MODULE.ARMS.items():
            relative = f"seed_{seed}/{arm}"
            item = root / relative
            path = _report(item / "evaluation/metrics.json", checkpoint="a.pt",
                           ade=1.0 if arm == "A_baseline" else 0.8, device="cuda:0")
            evaluation = json.loads(path.read_text())
            evaluation["hardware"]["device_uuid"] = plan["gpu_uuid"].removeprefix("GPU-")
            evaluation["vehicle_control_approved"] = False
            evaluation["model_config_sha256"] = MODULE._sha(json.dumps(config, sort_keys=True, separators=(",", ":")).encode())
            evaluation["checkpoint_sha256"] = MODULE._sha(relative.encode())
            evaluation["training_episode_count"] = 2
            evaluation["evaluation_episode_count"] = 1
            evaluation["metrics"].update(ade_6p4s_m=evaluation["metrics"]["selected_ade_m"],
                                         fde_6p4s_m=evaluation["metrics"]["selected_fde_m"])
            evaluation["metric_counts"].update(ade_6p4s_m=20, fde_6p4s_m=20)
            for domain in evaluation["per_domain_metrics"].values():
                domain["metrics"].update(ade_6p4s_m=domain["metrics"]["selected_ade_m"],
                                          fde_6p4s_m=domain["metrics"]["selected_fde_m"])
                domain["metric_counts"].update(ade_6p4s_m=domain["sample_count"], fde_6p4s_m=domain["sample_count"])
            evaluation["sample_count"] = 337
            evaluation["domain_sample_counts"] = {"carla": 337}
            evaluation["metric_counts"] = {name: 337 for name in evaluation["metrics"]}
            evaluation["per_domain_metrics"] = {"carla": {"sample_count": 337,
                "metrics": copy.deepcopy(evaluation["metrics"]), "metric_counts": copy.deepcopy(evaluation["metric_counts"])}}
            evaluation["training_domain_samples_seen"] = {"carla": 6130}
            evaluation["timing"]["model_forward_ms_per_sample"] = 1000 * evaluation["timing"]["model_forward_seconds"] / 337
            _write(path, evaluation)
            training = {
                "status": "TRAINING_TARGET_REACHED", "training_split": "train",
                "dataset_size": 613, "training_episode_ids": ["train-0", "train-1"],
                "model_config": config, "model_parameter_count": evaluation["model_parameter_count"],
                "loss_config": {"xy_weight": 1.0, "speed_weight": 0.2, "yaw_weight": 0.1,
                                "kinematic_speed_weight": 0.05, "final_displacement_weight": 0.5,
                                "candidate_score_weight": 0.1},
                "train_config": {"seed": seed, "learning_rate": lr, "batch_size": 4,
                    "max_steps": 1540, "weight_decay": 0.0001, "checkpoint_interval": 154,
                    "num_workers": 0, "maximum_gradient_norm": 5.0, "verify_image_sha256": True,
                    "sampling_policy": "uniform_without_replacement", "domain_ratios": []},
                "state": {"global_step": 1540, "domain_samples_seen": evaluation["training_domain_samples_seen"]},
                "corpus_fingerprint_sha256": evaluation["corpus_fingerprint_sha256"],
                "dataset_fingerprint_sha256": evaluation["training_dataset_fingerprint_sha256"],
                "sampling_plan_sha256": evaluation["training_sampling_plan_sha256"],
                "runtime": evaluation["runtime"], "hardware": evaluation["hardware"], "device": "cuda:0",
            }
            _write(item / "training/run.json", training)
            audit = {name: evaluation[name] for name in (
                "checkpoint_sha256", "model_config_sha256", "corpus_fingerprint_sha256",
                "dataset_fingerprint_sha256", "training_dataset_fingerprint_sha256",
                "training_episode_count", "evaluation_episode_count", "training_sampling_policy",
                "training_sampling_plan_sha256", "training_domain_samples_seen", "domain_sample_counts",
                "runtime", "device", "batch_size", "evaluation_split", "vehicle_control_approved", "model_parameter_count")}
            audit.update(audit_id="portable_e2e.runtime_geometry_audit.v8", status="RUNTIME_GEOMETRY_AUDIT_COMPLETE",
                gate={"source": MODULE.RUNTIME_GATE_ID, "thresholds": asdict(MODULE.RuntimeGateConfig()), "threshold_overrides": False},
                implementation={f"{name}_sha256": MODULE._sha(source_bytes[f"portable_e2e/{name}.py"])
                                for name in ("audit_runtime", "runtime_contract")},
                geometry={"sample_count": 337, "selected_result": {"sample_count": 337,
                    "geometry_pass_count": 336, "geometry_reject_count": 1, "geometry_pass_rate": 336 / 337,
                    "failure_counts": {"speed": 1}}, "selector": {
                        "selection_counts": {str(index): 337 if index == 2 else 0 for index in range(6)},
                        "invalid_selection_count": 0}})
            _write(item / "gate_v8.json", audit)
            for stage in ("train", "evaluate", "audit"):
                command = ["python", "-m", f"portable_e2e.{'audit_runtime' if stage == 'audit' else stage}",
                           "--split", "train" if stage == "train" else "val", "--device", "cuda:0", "--batch-size", "4"]
                if stage == "train":
                    command.extend(["--seed", str(seed), "--learning-rate", str(lr), "--max-steps", "1540"])
                stages.append({"run": relative, "stage": stage, "status": "COMPLETE", "returncode": 0,
                               "command": command})
    _write(root / "status.json", {"status": "TRAIN_EVAL_AUDIT_COMPLETE_NOT_PROMOTED", "source_commit": plan["source_commit"],
        "plan": plan, "plan_sha256": MODULE._sha((root / "plan.json").read_bytes()),
        "model_config_sha256": MODULE._sha(config_payload), "vehicle_control_approved": False, "stages": stages})
    return root


def test_complete_paired_screen_never_promotes(campaign: Path) -> None:
    report = MODULE.summarize_campaign(campaign)
    assert report["status"] == "COMPLETE_NOT_PROMOTED"
    assert report["candidate_screen"] == report["absolute_quality"] == "PASS"
    assert len(report["pairs"]) == 3
    assert report["vehicle_control_approved"] is False
    assert report["automatic_promotion"] is False
    assert report["test_opened_by_this_campaign"] is False
    assert len(report["input_manifest"]) == 20
    assert report["pairs"][0]["candidate"]["metrics"]["selection_regret_ade_m"] == pytest.approx(0.16)
    assert report["pairs"][0]["candidate"]["geometry"]["selection_histogram"]["2"] == 337
    assert report["pairs"][0]["candidate"]["checkpoint_bytes_locally_verified"] is False
    assert "Selection regret" in MODULE.render_markdown(report)


def _mutate(root: Path, relative: str, mutate) -> None:
    path = root / relative
    value = json.loads(path.read_text())
    mutate(value)
    _write(path, value)


@pytest.mark.parametrize("relative,mutate,error", [
    ("seed_20260903/B_lower_lr/training/run.json", lambda x: x["train_config"].update(seed=20260904), "train config"),
    ("seed_20260903/B_lower_lr/training/run.json", lambda x: x["train_config"].update(learning_rate=0.001), "train config"),
    ("seed_20260903/B_lower_lr/training/run.json", lambda x: x["state"].update(global_step=1539), "step mismatch"),
    ("seed_20260903/B_lower_lr/training/run.json", lambda x: x["loss_config"].update(candidate_score_weight=0.2), "loss config"),
    ("seed_20260903/B_lower_lr/training/run.json", lambda x: x.update(dataset_size=612), "full train613"),
    ("seed_20260903/B_lower_lr/training/run.json", lambda x: x.update(training_episode_ids=["train-0"]), "unique training episodes"),
    ("seed_20260903/B_lower_lr/evaluation/metrics.json", lambda x: x.update(training_episode_count=3), "episode counts"),
    ("seed_20260903/B_lower_lr/evaluation/metrics.json", lambda x: x.update(evaluation_episode_count=2), "episode counts"),
    ("seed_20260903/B_lower_lr/gate_v8.json", lambda x: x.update(checkpoint_sha256="f" * 64), "checkpoint_sha256 mismatch"),
    ("seed_20260903/B_lower_lr/gate_v8.json", lambda x: x["gate"]["thresholds"].update(maximum_speed_mps=20), "gate changed"),
    ("seed_20260903/B_lower_lr/gate_v8.json", lambda x: x["geometry"]["selector"]["selection_counts"].update({"2": 19}), "histogram denominator"),
    ("seed_20260903/B_lower_lr/gate_v8.json", lambda x: x["implementation"].update(audit_runtime_sha256="f" * 64), "pinned source"),
    ("seed_20260903/B_lower_lr/evaluation/metrics.json", lambda x: x.update(corpus_fingerprint_sha256="f" * 64), "comparison is not fair"),
    ("status.json", lambda x: x.update(plan_sha256="f" * 64), "plan SHA-256"),
    ("status.json", lambda x: x["stages"][1].update(command=["python", "--split", "test"]), "different split"),
    ("status.json", lambda x: x["stages"].append(copy.deepcopy(x["stages"][0])), "stage order"),
])
def test_tampered_evidence_rejected(campaign: Path, relative, mutate, error) -> None:
    _mutate(campaign, relative, mutate)
    with pytest.raises(ContractError, match=error):
        MODULE.summarize_campaign(campaign)


@pytest.mark.parametrize("missing", ("status.json", "seed_20260905/B_lower_lr/gate_v8.json"))
def test_missing_artifacts_never_pass(campaign: Path, missing: str) -> None:
    (campaign / missing).unlink()
    report = MODULE.summarize_campaign(campaign)
    assert report["status"] == "INCOMPLETE"
    assert report["candidate_screen"] == "NOT_EVALUATED"
    assert missing in report["missing_artifacts"]
    assert report["test_opened_by_this_campaign"] is None


def test_running_campaign_never_passes(campaign: Path) -> None:
    _mutate(campaign, "status.json", lambda x: x.update(status="RUNNING"))
    report = MODULE.summarize_campaign(campaign)
    assert report["status"] == "INCOMPLETE"
    assert report["candidate_screen"] == "NOT_EVALUATED"


def test_consistent_partial_split_still_violates_frozen_val337(campaign: Path) -> None:
    # HH_260906 - Six mutually compatible partial reports cannot replace the predeclared complete validation split.
    for seed in MODULE.SEEDS:
        for arm in MODULE.ARMS:
            def mutation(value):
                value["sample_count"] = 336
                value["domain_sample_counts"] = {"carla": 336}
                value["metric_counts"] = {name: 336 for name in value["metrics"]}
                value["per_domain_metrics"]["carla"]["sample_count"] = 336
                value["per_domain_metrics"]["carla"]["metric_counts"] = copy.deepcopy(value["metric_counts"])
                value["timing"]["model_forward_ms_per_sample"] = 1000 * value["timing"]["model_forward_seconds"] / 336
            _mutate(campaign, f"seed_{seed}/{arm}/evaluation/metrics.json", mutation)
    with pytest.raises(ContractError, match="full CARLA val337"):
        MODULE.summarize_campaign(campaign)


def test_single_seed_geometry_regression_fails_entire_screen(campaign: Path) -> None:
    def mutation(value):
        value["geometry"]["selected_result"].update(geometry_pass_count=335, geometry_reject_count=2, geometry_pass_rate=335 / 337)
    _mutate(campaign, "seed_20260905/B_lower_lr/gate_v8.json", mutation)
    report = MODULE.summarize_campaign(campaign)
    assert report["candidate_screen"] == "FAIL"
    assert report["pairs"][2]["relative_checks"]["geometry_does_not_regress"] is False
    assert report["absolute_quality"] == "PASS"


@pytest.mark.parametrize("metric,factor,check", [
    ("selected_ade_m", 1.25, "selected_ade_improves"),
    ("selected_fde_m", 1.25, "selected_fde_improves"),
    ("selected_speed_mae_mps", 1.314, "speed_mae_within_5_percent"),
])
def test_one_bad_pair_blocks_aggregate_screen(campaign: Path, metric: str, factor: float, check: str) -> None:
    # HH_260906 - An improved mean must not hide one seed's tied primary metric or speed regression.
    def mutation(value):
        value["metrics"][metric] *= factor
        for domain in value["per_domain_metrics"].values():
            domain["metrics"][metric] *= factor
    _mutate(campaign, "seed_20260904/B_lower_lr/evaluation/metrics.json", mutation)
    report = MODULE.summarize_campaign(campaign)
    assert report["candidate_screen"] == "FAIL"
    assert report["pairs"][1]["relative_checks"][check] is False
    assert report["pairs"][0]["candidate_screen"] == report["pairs"][2]["candidate_screen"] == "PASS"


def test_relative_pass_does_not_override_absolute_failure(campaign: Path) -> None:
    for seed in MODULE.SEEDS:
        for arm in MODULE.ARMS:
            path = f"seed_{seed}/{arm}/evaluation/metrics.json"
            def mutation(value):
                for metrics in [value["metrics"], *[domain["metrics"] for domain in value["per_domain_metrics"].values()]]:
                    for name in metrics:
                        metrics[name] *= 10
            _mutate(campaign, path, mutation)
    report = MODULE.summarize_campaign(campaign)
    assert report["candidate_screen"] == "PASS"
    assert report["absolute_quality"] == "FAIL"
    assert report["vehicle_control_approved"] is False


def test_downloaded_checkpoint_must_match(campaign: Path) -> None:
    path = campaign / "seed_20260903/A_baseline/training/checkpoints/latest.pt"
    path.parent.mkdir()
    path.write_bytes(b"not the pinned checkpoint")
    with pytest.raises(ContractError, match="checkpoint hash mismatch"):
        MODULE.summarize_campaign(campaign)


def test_cli_preserves_existing_outputs(campaign: Path, tmp_path: Path) -> None:
    output = tmp_path / "summary"
    args = [str(campaign), "--output-dir", str(output)]
    assert MODULE.main(args) == 0
    original = (output / "summary.json").read_bytes()
    assert MODULE.main(args) == 2
    assert (output / "summary.json").read_bytes() == original
