"""HH_260906 - Verify candidate-aware architecture comparisons using isolated synthetic evidence."""

from __future__ import annotations

import copy
import importlib.util
import json
from pathlib import Path
import shutil

import pytest

from portable_e2e.contract import ContractError
import test_summarize_portable_training_campaign as first
from test_summarize_portable_training_campaign import campaign
from test_summarize_portable_data_expansion import expansion

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("candidate_rank_summary", ROOT / "scripts/e2e/summarize_portable_candidate_rank.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


@pytest.fixture
def candidate(expansion: Path, tmp_path: Path, monkeypatch) -> Path:
    # HH_260906 - Keep the baseline reports intact; candidate config and capacity differ without changing reviewed data.
    root = tmp_path / "candidate_rank"
    config = json.loads((ROOT / MODULE.BASELINE_CONFIG).read_text())
    config["model_id"] = MODULE.MODEL_ID
    config_bytes = json.dumps(config).encode()
    source = "\n".join(f"{name} = {index + 1}.0" for index, name in enumerate(MODULE.DECODER_CONSTANTS))
    source += "\nclass PerspectiveTrajectoryModel:\n    def _decode_physical_v1(self, values):\n        return values\n"
    old_git = MODULE.base._git_bytes
    def git_bytes(commit, path):
        if path == MODULE.MODEL_CONFIG:
            return config_bytes
        if path == "portable_e2e/model.py":
            return source.encode() + (b"\n# HH_260906 - New candidate-aware scorer.\n" if commit != MODULE.BASELINE_COMMIT else b"")
        if path in ("portable_e2e/losses.py", "portable_e2e/evaluate.py"):
            return b"same implementation " + path.encode()
        return old_git(commit, path)
    monkeypatch.setattr(MODULE.base, "_git_bytes", git_bytes)
    baseline = json.loads((expansion / "status.json").read_text())
    plan = copy.deepcopy(baseline["plan"])
    plan.update(schema="portable_e2e.candidate_rank_campaign.v1", campaign_id="hh260907-candidate-rank-3seeds-v1",
        source_commit="f" * 40, model_config=MODULE.MODEL_CONFIG, arms={MODULE.ARM: 0.0001}, candidate_score_weight=0.1,
        prerequisite_campaign_id=MODULE.BASELINE_ID, baseline_campaign_id=MODULE.BASELINE_ID,
        baseline_source_commit=MODULE.BASELINE_COMMIT)
    plan["decision"]["absolute_limits_m"] = {key.removesuffix("_m"): value for key, value in MODULE.base.ABSOLUTE_LIMITS.items()}
    first._write(root / "plan.json", plan)
    state = copy.deepcopy(baseline)
    state.update(plan=plan, source_commit=plan["source_commit"], plan_sha256=MODULE.base._sha((root / "plan.json").read_bytes()),
        model_config_sha256=MODULE.base._sha(config_bytes), prerequisite={"status": "TRAIN_EVAL_AUDIT_COMPLETE_NOT_PROMOTED",
        "completed_stages": 9, "sha256": MODULE.base._sha((expansion / "status.json").read_bytes())})
    for seed in MODULE.base.SEEDS:
        relative = f"seed_{seed}/{MODULE.ARM}"
        shutil.copytree(expansion / f"seed_{seed}/C_expanded_data", root / relative)
        for stage, filename in MODULE.expansion.STAGE_FILES.items():
            path = root / relative / filename
            value = json.loads(path.read_text())
            value["model_parameter_count"] += 100
            if stage == "train":
                value["model_config"] = config
            else:
                value["model_config_sha256"] = MODULE.base._sha(json.dumps(config, sort_keys=True, separators=(",", ":")).encode())
            if stage == "evaluate":
                for metrics in [value["metrics"], *[domain["metrics"] for domain in value["per_domain_metrics"].values()]]:
                    for name in metrics:
                        metrics[name] *= 0.8
            first._write(path, value)
            record = next(item for item in state["stages"] if item["run"] == f"seed_{seed}/C_expanded_data" and item["stage"] == stage)
            record["run"] = relative
            record["report"]["sha256"] = MODULE.base._sha(path.read_bytes())
            if stage == "train":
                record["command"].extend(["--candidate-score-weight", "0.1"])
    first._write(root / "status.json", state)
    return root


def _seal(root: Path, stage: str, mutate, *, seed: int = 20260903) -> None:
    # HH_260906 - Re-seal only synthetic fixture reports to distinguish semantic checks from integrity checks.
    relative = f"seed_{seed}/{MODULE.ARM}"
    path = root / relative / MODULE.expansion.STAGE_FILES[stage]
    value = json.loads(path.read_text())
    mutate(value)
    first._write(path, value)
    state = json.loads((root / "status.json").read_text())
    record = next(item for item in state["stages"] if item["run"] == relative and item["stage"] == stage)
    record["report"]["sha256"] = MODULE.base._sha(path.read_bytes())
    first._write(root / "status.json", state)


def _metrics(value: dict, name: str, number: float) -> None:
    value["metrics"][name] = number
    for domain in value["per_domain_metrics"].values():
        domain["metrics"][name] = number


def test_different_architectures_share_strict_data_contract_without_promotion(expansion: Path, candidate: Path) -> None:
    report = MODULE.summarize_campaign(expansion, candidate)
    assert report["status"] == "COMPLETE_NOT_PROMOTED"
    assert report["candidate_screen"] == report["absolute_quality"] == "PASS"
    assert report["automatic_promotion"] is report["vehicle_control_approved"] is False
    assert report["test_opened_by_this_campaign"] is False
    assert report["experiment"]["same_model_architecture"] is False
    assert len(report["comparison"]["reports"]) == 6
    assert len(report["pairs"]) == 3 and len(report["input_manifest"]) == 22
    for pair in report["pairs"]:
        assert pair["baseline"]["model_config_sha256"] != pair["candidate"]["model_config_sha256"]
        assert pair["baseline"]["model_parameter_count"] != pair["candidate"]["model_parameter_count"]
        assert pair["candidate"]["training_dataset_size"] == 1147
        assert pair["candidate"]["training_episode_count"] == 3
        assert pair["candidate"]["validation_sample_count"] == 337
    assert len(report["source_proof"]["decoder_segments"]) == 7
    assert len(report["source_proof"]["unchanged_files"]) == 4
    assert "서로 다른 모델 구조" in MODULE.render_markdown(report)


@pytest.mark.parametrize("metric,value,check", [
    ("selected_ade_m", 1.0, "selected_ade_improves"),
    ("selected_fde_m", 2.0, "selected_fde_improves"),
    ("selected_speed_mae_mps", 0.21001, "speed_mae_within_5_percent"),
])
def test_one_seed_regression_or_tie_fails_all_seed_screen(expansion: Path, candidate: Path, metric, value, check) -> None:
    _seal(candidate, "evaluate", lambda item: _metrics(item, metric, value), seed=20260905)
    report = MODULE.summarize_campaign(expansion, candidate)
    assert report["candidate_screen"] == "FAIL"
    assert report["pairs"][0]["candidate_screen"] == "PASS"
    assert report["pairs"][2]["relative_checks"][check] is False


def test_speed_exact_five_percent_boundary_passes(expansion: Path, candidate: Path) -> None:
    _seal(candidate, "evaluate", lambda value: _metrics(value, "selected_speed_mae_mps", 1.05 * 0.2))
    assert MODULE.summarize_campaign(expansion, candidate)["candidate_screen"] == "PASS"


def test_geometry_regression_fails(expansion: Path, candidate: Path) -> None:
    _seal(candidate, "audit", lambda value: value["geometry"]["selected_result"].update(
        geometry_pass_count=335, geometry_reject_count=2, geometry_pass_rate=335 / 337))
    assert MODULE.summarize_campaign(expansion, candidate)["candidate_screen"] == "FAIL"


def test_absolute_failure_does_not_become_relative_pass_or_promotion(expansion: Path, candidate: Path) -> None:
    _seal(candidate, "evaluate", lambda value: _metrics(value, "ade_6p4s_m", 2.01))
    report = MODULE.summarize_campaign(expansion, candidate)
    assert report["candidate_screen"] == "PASS" and report["absolute_quality"] == "FAIL"
    assert report["status"] == "COMPLETE_NOT_PROMOTED"
    assert report["vehicle_control_approved"] is False


@pytest.mark.parametrize("stage,mutation,error", [
    ("train", lambda value: value["loss_config"].update(candidate_score_weight=0.5), "loss config"),
    ("train", lambda value: value.update(dataset_size=613), "full train1147"),
    ("train", lambda value: value["train_config"].update(seed=20260904), "train config"),
    ("train", lambda value: value["model_config"].update(model_id="portable_e2e.perspective_trajectory.physical.v1"), "model config"),
    ("evaluate", lambda value: value.update(model_config_sha256="e" * 64), "config hash"),
    ("audit", lambda value: value.update(model_parameter_count=1234), "model_parameter_count mismatch"),
    ("evaluate", lambda value: value["hardware"].update(device_name="other GPU"), "comparison is not fair"),
    ("audit", lambda value: value["gate"]["thresholds"].update(maximum_speed_mps=100), "gate changed"),
])
def test_report_semantics_remain_strict(expansion: Path, candidate: Path, stage, mutation, error) -> None:
    _seal(candidate, stage, mutation)
    with pytest.raises(ContractError, match=error):
        MODULE.summarize_campaign(expansion, candidate)


@pytest.mark.parametrize("mutation,error", [
    (lambda state: state["prerequisite"].update(sha256="e" * 64), "predecessor status byte hash"),
    (lambda state: state["stages"][0]["report"].update(sha256="e" * 64), "report SHA-256"),
    (lambda state: state["stages"].reverse(), "nine-stage order"),
    (lambda state: state["stages"][0]["command"].__setitem__(-1, "0.5"), "candidate-score weight"),
    (lambda state: state["stages"][0]["command"].extend(["--candidate-score-weight", "0.1"]), "exactly one"),
    (lambda state: state["stages"][1]["command"].__setitem__(4, "test"), "unreviewed split"),
    (lambda state: state.update(source_commit="e" * 40), "plan/source"),
])
def test_worker_proofs_are_bound_to_frozen_plan(expansion: Path, candidate: Path, mutation, error) -> None:
    first._mutate(candidate, "status.json", mutation)
    with pytest.raises(ContractError, match=error):
        MODULE.summarize_campaign(expansion, candidate)


@pytest.mark.parametrize("path", MODULE.UNCHANGED_FILES)
def test_loss_evaluation_and_gate_source_changes_rejected(expansion: Path, candidate: Path, monkeypatch, path: str) -> None:
    original = MODULE.base._git_bytes
    monkeypatch.setattr(MODULE.base, "_git_bytes", lambda commit, name:
        b"changed formula" if commit != MODULE.BASELINE_COMMIT and name == path else original(commit, name))
    with pytest.raises(ContractError, match="frozen implementation changed"):
        MODULE.summarize_campaign(expansion, candidate)


@pytest.mark.parametrize("before,after", [(b"return values", b"return values * 2"),
    (b"PHYSICAL_MAXIMUM_SPEED_MPS = 2.0", b"PHYSICAL_MAXIMUM_SPEED_MPS = 99.0")])
def test_decoder_method_and_physical_constants_cannot_change(expansion: Path, candidate: Path, monkeypatch, before, after) -> None:
    original = MODULE.base._git_bytes
    monkeypatch.setattr(MODULE.base, "_git_bytes", lambda commit, name:
        original(commit, name).replace(before, after) if commit != MODULE.BASELINE_COMMIT and name == "portable_e2e/model.py" else original(commit, name))
    with pytest.raises(ContractError, match="physical decoder source changed"):
        MODULE.summarize_campaign(expansion, candidate)


def test_other_model_capacity_config_change_is_not_hidden(expansion: Path, candidate: Path, monkeypatch) -> None:
    original = MODULE.base._git_bytes
    modified = json.loads(original("f" * 40, MODULE.MODEL_CONFIG))
    modified["hidden_width"] = 512
    payload = json.dumps(modified).encode()
    monkeypatch.setattr(MODULE.base, "_git_bytes", lambda commit, name: payload if name == MODULE.MODEL_CONFIG else original(commit, name))
    first._mutate(candidate, "status.json", lambda state: state.update(model_config_sha256=MODULE.base._sha(payload)))
    with pytest.raises(ContractError, match="changes more than the reviewed model ID"):
        MODULE.summarize_campaign(expansion, candidate)


def test_absolute_plan_limits_cannot_be_relaxed(expansion: Path, candidate: Path) -> None:
    plan = json.loads((candidate / "plan.json").read_text())
    plan["decision"]["absolute_limits_m"]["ade_6p4s"] = 200.0
    first._write(candidate / "plan.json", plan)
    first._mutate(candidate, "status.json", lambda state: state.update(plan=plan,
        plan_sha256=MODULE.base._sha((candidate / "plan.json").read_bytes())))
    with pytest.raises(ContractError, match="absolute gates"):
        MODULE.summarize_campaign(expansion, candidate)


def test_archived_worker_bytes_are_never_assumed_verified(expansion: Path, candidate: Path) -> None:
    report = MODULE.summarize_campaign(expansion, candidate)
    assert report["source_proof"]["workers"][1]["bytes_locally_verified"] is False
    path = candidate / "provenance/active_runner.py"
    path.parent.mkdir()
    path.write_bytes(b"fixture worker")
    with pytest.raises(ContractError, match="archived worker SHA-256"):
        MODULE.summarize_campaign(expansion, candidate)


@pytest.mark.parametrize("missing", ["status.json", "seed_20260905/E_candidate_rank/gate_v8.json"])
def test_missing_candidate_reports_cannot_pass(expansion: Path, candidate: Path, missing: str) -> None:
    (candidate / missing).unlink()
    report = MODULE.summarize_campaign(expansion, candidate)
    assert report["status"] == "INCOMPLETE"
    assert report["candidate_screen"] == report["absolute_quality"] == "NOT_EVALUATED"
    assert report["test_opened_by_this_campaign"] is None


def test_cli_preserves_existing_output(expansion: Path, candidate: Path, tmp_path: Path) -> None:
    output = tmp_path / "summary"
    args = [str(expansion), str(candidate), "--output-dir", str(output)]
    assert MODULE.main(args) == 0
    payload = (output / "summary.json").read_bytes()
    assert MODULE.main(args) == 2
    assert (output / "summary.json").read_bytes() == payload
