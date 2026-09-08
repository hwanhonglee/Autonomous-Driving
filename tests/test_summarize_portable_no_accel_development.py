"""HH_260906 - Verify fresh no-accel paired evidence using synthetic reports without running training or inference."""

from __future__ import annotations

import copy
from datetime import datetime, timedelta, timezone
import json
from pathlib import Path
import shutil

import pytest

from portable_e2e.contract import ContractError
from scripts.e2e import summarize_portable_no_accel_development as module
import test_summarize_portable_training_campaign as first
from test_summarize_portable_training_campaign import campaign
from test_summarize_portable_data_expansion import expansion


ROOT = Path(__file__).resolve().parents[1]


def commands(relative, stage, checksum):
    # HH_260906 - The fixture uses a deliberately synthetic personal prefix; no real remote account is contacted.
    workspace = Path("/synthetic/personal/hwanhong/portable_e2e")
    python = str(workspace / "venvs/py312/bin/python")
    item = workspace / "runs/campaigns" / module.CAMPAIGN_ID / relative
    dataset = str(workspace / module.DATASET)
    common = ["--device", "cuda:0", "--batch-size", "4"]
    seed, arm = relative.split("/")
    if stage == "train":
        return [python, "-m", "portable_e2e.train", dataset, "--run-dir", str(item / "training"), "--model-config",
            str(workspace / "autoware_e2e" / module.MODELS[arm]), "--split", "train", *common,
            "--seed", seed.removeprefix("seed_"), "--learning-rate", "0.0001", "--weight-decay", "0.0001",
            "--max-steps", "1540", "--checkpoint-interval", "154", "--num-workers", "0",
            "--maximum-gradient-norm", "5", "--sampling-policy", "uniform_without_replacement"]
    if stage == "evaluate":
        return [python, "-m", "portable_e2e.evaluate", dataset, "--checkpoint", str(item / "training/checkpoints/latest.pt"),
            "--output-dir", str(item / "evaluation"), "--split", "val", *common, "--num-workers", "0", "--render-count", "12"]
    return [python, "-m", "portable_e2e.audit_runtime", dataset, "--checkpoint", str(item / "training/checkpoints/latest.pt"),
        "--output-json", str(item / "gate_v8.json"), "--split", "val", *common, "--checkpoint-sha256", checksum]


@pytest.fixture
def no_accel(expansion, tmp_path, monkeypatch):
    root = tmp_path / "no_accel"
    plan = json.loads((ROOT / "config/portable_e2e_no_accel_development_20260909.json").read_text())
    old_git = module.base._git_bytes
    source = {name: (ROOT / name).read_bytes() if name in module.MODELS.values()
        else old_git(plan["source_commit"], name) if name in ("portable_e2e/audit_runtime.py", "portable_e2e/runtime_contract.py")
        else ("synthetic pinned source " + name).encode() for name in module.CORE_FILES}
    monkeypatch.setattr(module.base, "_git_bytes", lambda commit, path: source[path])
    state = copy.deepcopy(json.loads((expansion / "status.json").read_text()))
    first._write(root / "plan.json", plan)
    state.update(plan=plan, source_commit=plan["source_commit"], plan_sha256=module.base._sha((root / "plan.json").read_bytes()),
        prerequisite=None, runner_sha256=module.base._sha(source[module.WORKER]),
        reviewed_contract={"train_samples": 1147, "val_samples": 337, "run_count": 6, "stage_count": 18},
        created_at_utc="2026-09-09T00:00:00+00:00", stages=[])
    state.pop("model_config_sha256")
    state["model_configs"] = {}
    for arm, name in module.MODELS.items():
        config = json.loads(source[name])
        state["model_configs"][arm] = {"path": name, "sha256": module.base._sha(source[name]),
            "canonical_sha256": module.base._sha(module.canonical(config).encode()), "model_config": config}
    for seed in module.base.SEEDS:
        for arm in module.ARMS:
            relative = f"seed_{seed}/{arm}"
            shutil.copytree(expansion / f"seed_{seed}/C_expanded_data", root / relative)
            for stage, name in module.expansion.STAGE_FILES.items():
                path = root / relative / name
                value = json.loads(path.read_text())
                if stage == "train":
                    value["model_config"] = state["model_configs"][arm]["model_config"]
                else:
                    value["model_config_sha256"] = state["model_configs"][arm]["canonical_sha256"]
                if stage == "evaluate" and arm == "B_no_accel_input":
                    for metrics in [value["metrics"], *[domain["metrics"] for domain in value["per_domain_metrics"].values()]]:
                        for key in metrics:
                            metrics[key] *= .8
                first._write(path, value)
                when = datetime(2026, 9, 9, tzinfo=timezone.utc) + timedelta(seconds=len(state["stages"]) * 2)
                state["stages"].append({"run": relative, "stage": stage, "status": "COMPLETE", "returncode": 0,
                    "command": commands(relative, stage, value.get("checkpoint_sha256")),
                    "started_at_utc": when.isoformat(), "finished_at_utc": (when + timedelta(seconds=1)).isoformat(),
                    "report": {"path": name, "sha256": module.base._sha(path.read_bytes()),
                        "corpus_fingerprint_sha256": value["corpus_fingerprint_sha256"],
                        "dataset_fingerprint_sha256": value["dataset_fingerprint_sha256"]}})
    archive = root / module.ARCHIVE
    archive.parent.mkdir(parents=True)
    archive.write_bytes(source[module.WORKER])
    first._write(root / "status.json", state)
    return root


def mutate(root, path, callback):
    first._mutate(root, path, callback)


def seal(root, stage, callback, arm="B_no_accel_input", seed=20260903):
    relative = f"seed_{seed}/{arm}"
    name = module.expansion.STAGE_FILES[stage]
    mutate(root, relative + "/" + name, callback)
    value = json.loads((root / "status.json").read_text())
    record = next(r for r in value["stages"] if r["run"] == relative and r["stage"] == stage)
    record["report"]["sha256"] = module.sha_file(root / relative / name)
    first._write(root / "status.json", value)


def metric(value, key, number):
    value["metrics"][key] = number
    for domain in value["per_domain_metrics"].values():
        domain["metrics"][key] = number


def test_complete_pairs_never_promoted(no_accel):
    result = module.summarize_campaign(no_accel)
    assert result["status"] == "COMPLETE_NOT_PROMOTED"
    assert result["candidate_screen"] == result["absolute_quality"] == "PASS"
    assert len(result["pairs"]) == 3 and len(result["stages"]) == 18
    assert result["test_evaluated"] is result["test_used_for_training_or_selection"] is False
    assert result["automatic_promotion"] is result["vehicle_control_approved"] is False
    assert result["source_proof"]["initial_tensor_hashes_available"] is False
    assert len(result["input_manifest"]) == 21
    assert str(no_accel) not in json.dumps(result)
    assert "/synthetic/" not in json.dumps(result)
    assert "test 파일을 읽을 수" in module.render_markdown(result)


@pytest.mark.parametrize("stage,callback,error", [
    ("train", lambda v: v.update(dataset_size=613), "full train1147"),
    ("train", lambda v: v["train_config"].update(seed=20260904), "train config"),
    ("train", lambda v: v["state"].update(global_step=1539), "step mismatch"),
    ("train", lambda v: v["model_config"].update(model_id=module.MODEL_IDS["A_physical_input"]), "model config"),
    ("train", lambda v: v["loss_config"].update(candidate_score_weight=.5), "loss config"),
    ("evaluate", lambda v: v.update(sample_count=336), "counts|count|timing|sample"),
    ("evaluate", lambda v: v.update(training_episode_count=2), "episode counts"),
    ("audit", lambda v: v.update(model_config_sha256="f" * 64), "model_config_sha256"),
    ("audit", lambda v: v.update(checkpoint_sha256="f" * 64), "checkpoint_sha256"),
    ("audit", lambda v: v["gate"]["thresholds"].update(maximum_deceleration_mps2=7), "gate changed"),
])
def test_semantic_mismatch_rejected(no_accel, stage, callback, error):
    seal(no_accel, stage, callback)
    with pytest.raises(ContractError, match=error):
        module.summarize_campaign(no_accel)


@pytest.mark.parametrize("callback,error", [
    (lambda s: s.update(plan_sha256="f" * 64), "plan/source"),
    (lambda s: s.update(runner_sha256="f" * 64), "runner"),
    (lambda s: s["model_configs"]["B_no_accel_input"].update(sha256="f" * 64), "config proof"),
    (lambda s: s["stages"].reverse(), "order/scope"),
    (lambda s: s["stages"].append(copy.deepcopy(s["stages"][-1])), "order/scope"),
    (lambda s: s["stages"][0]["command"].append("--resume"), "recorded command"),
    (lambda s: s["stages"][1]["command"].extend(["--limit-samples", "10"]), "recorded command"),
    (lambda s: s["stages"][0]["report"].update(sha256="f" * 64), "report SHA"),
    (lambda s: s["stages"][1].update(started_at_utc="2026-09-08T00:00:00+00:00"), "chronology"),
    (lambda s: s["stages"][0].update(returncode=1), "exit status"),
    (lambda s: s.update(corpus_fingerprint_sha256="f" * 64), "corpus fingerprint"),
])
def test_provenance_mismatch_rejected(no_accel, callback, error):
    mutate(no_accel, "status.json", callback)
    with pytest.raises(ContractError, match=error):
        module.summarize_campaign(no_accel)


@pytest.mark.parametrize("missing", ["plan.json", "status.json", module.ARCHIVE,
    "seed_20260905/B_no_accel_input/gate_v8.json"])
def test_missing_is_incomplete(no_accel, missing):
    (no_accel / missing).unlink()
    result = module.summarize_campaign(no_accel)
    assert result["status"] == "INCOMPLETE"
    assert result["candidate_screen"] == "NOT_EVALUATED"
    assert missing in result["missing_artifacts"]


@pytest.mark.parametrize("status", ["RUNNING", "STOPPED_FAILURE_NO_PROMOTION"])
def test_running_or_failed_preserves_no_pass(no_accel, status):
    mutate(no_accel, "status.json", lambda value: value.update(status=status))
    result = module.summarize_campaign(no_accel)
    assert result["status"] == "INCOMPLETE"
    assert result["runner_status"] == status
    assert len(result["stages"]) == 18


def test_failed_partial_stage_and_remaining_denominator(no_accel):
    def failure(value):
        value["stages"] = value["stages"][:2]
        value["stages"][-1].update(status="FAILED", returncode=1)
        value["stages"][-1].pop("report")
        value["status"] = "STOPPED_FAILURE_NO_PROMOTION"
    mutate(no_accel, "status.json", failure)
    result = module.summarize_campaign(no_accel)
    assert result["status"] == "INCOMPLETE" and len(result["stages"]) == 2
    assert result["stages"][-1]["status"] == "FAILED"
    assert result["expected_stage_count"] == 18
    assert len(result["not_started_stages"]) == 16


@pytest.mark.parametrize("name,value,check", [("selected_ade_m", 1., "selected_ade_improves"),
    ("selected_fde_m", 2., "selected_fde_improves"), ("selected_speed_mae_mps", .21001, "speed_mae_within_5_percent")])
def test_one_pair_blocks_aggregate(no_accel, name, value, check):
    seal(no_accel, "evaluate", lambda report: metric(report, name, value), seed=20260905)
    report = module.summarize_campaign(no_accel)
    assert report["candidate_screen"] == "FAIL"
    assert report["pairs"][2]["relative_checks"][check] is False
    assert report["pairs"][0]["candidate_screen"] == "PASS"


def test_geometry_regression_and_absolute_failure_separate(no_accel):
    seal(no_accel, "audit", lambda report: report["geometry"]["selected_result"].update(
        geometry_pass_count=335, geometry_reject_count=2, geometry_pass_rate=335/337))
    seal(no_accel, "evaluate", lambda report: metric(report, "ade_6p4s_m", 2.01))
    report = module.summarize_campaign(no_accel)
    assert report["candidate_screen"] == report["absolute_quality"] == "FAIL"
    assert report["status"] == "COMPLETE_NOT_PROMOTED"


def test_actual_checkpoint_bytes_when_present(no_accel):
    checkpoint = no_accel / "seed_20260903/A_physical_input/training/checkpoints/latest.pt"
    checkpoint.parent.mkdir()
    checkpoint.write_bytes(b"wrong checkpoint")
    with pytest.raises(ContractError, match="checkpoint hash mismatch"):
        module.summarize_campaign(no_accel)


def test_verified_local_checkpoint_is_in_manifest(no_accel):
    relative = "seed_20260903/A_physical_input/training/checkpoints/latest.pt"
    checkpoint = no_accel / relative
    checkpoint.parent.mkdir()
    # HH_260906 - The inherited evaluation fixture authenticates these exact synthetic bytes, not a loadable torch model.
    checkpoint.write_bytes(b"seed_20260903/A_baseline")
    report = module.summarize_campaign(no_accel)
    assert report["pairs"][0]["baseline"]["checkpoint_bytes_locally_verified"] is True
    assert relative in {entry["path"] for entry in report["input_manifest"]}


def test_other_source_commit_is_not_this_reviewed_study(no_accel):
    plan = json.loads((no_accel / "plan.json").read_text())
    plan["source_commit"] = "f" * 40
    first._write(no_accel / "plan.json", plan)
    mutate(no_accel, "status.json", lambda value: value.update(plan=plan, source_commit=plan["source_commit"],
        plan_sha256=module.sha_file(no_accel / "plan.json")))
    with pytest.raises(ContractError, match="reviewed matrix"):
        module.summarize_campaign(no_accel)


def test_checkpoint_symlink_rejected(no_accel, tmp_path):
    checkpoint = no_accel / "seed_20260903/A_physical_input/training/checkpoints/latest.pt"
    checkpoint.parent.mkdir()
    other = tmp_path / "other_checkpoint"
    other.write_bytes(b"not a real trained model")
    checkpoint.symlink_to(other)
    with pytest.raises(ContractError, match="symlink"):
        module.summarize_campaign(no_accel)


def test_archived_worker_corruption_rejected(no_accel):
    (no_accel / module.ARCHIVE).write_bytes(b"wrong worker")
    with pytest.raises(ContractError, match="archived worker"):
        module.summarize_campaign(no_accel)


def test_cli_public_paths_fresh_outputs_and_dataset_alias(no_accel, tmp_path, monkeypatch):
    output = tmp_path / "summary"
    assert module.main([str(no_accel), "--output-dir", str(output)]) == 0
    before = (output / "summary.json").read_bytes()
    assert module.main([str(no_accel), "--output-dir", str(output)]) == 2
    assert (output / "summary.json").read_bytes() == before
    assert module.main([str(no_accel), "--output-dir", str(no_accel / "nested")]) == 2
    repo, dataset = tmp_path / "repo", tmp_path / "actual_dataset"
    repo.mkdir(); dataset.mkdir()
    (repo / "datasets").symlink_to(dataset, target_is_directory=True)
    monkeypatch.setattr(module.base, "REPO", repo)
    assert module.main([str(no_accel), "--output-dir", str(dataset / "summary")]) == 2


def test_post_read_artifact_mutation_rejected(no_accel, monkeypatch):
    original = module.base.compare_reports
    def changed(paths):
        result = original(paths)
        mutate(no_accel, "status.json", lambda value: value.update(injected=True))
        return result
    monkeypatch.setattr(module.base, "compare_reports", changed)
    with pytest.raises(ContractError, match="changed during summary"):
        module.summarize_campaign(no_accel)
