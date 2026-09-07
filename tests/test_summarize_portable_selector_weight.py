"""HH_260906 - Exercise paired selector-weight evidence without training, remote access or published-file edits."""

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
SPEC = importlib.util.spec_from_file_location("selector_summary", ROOT / "scripts/e2e/summarize_portable_selector_weight.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


@pytest.fixture
def selector(expansion: Path, tmp_path: Path, monkeypatch) -> Path:
    # HH_260906 - Reuse the full v3 baseline contract, changing only the score weight and synthetic outcome.
    old_git = MODULE.base._git_bytes
    monkeypatch.setattr(MODULE.base, "_git_bytes", lambda commit, path:
                        b"same core " + path.encode() if path in MODULE.CORE_FILES[:2] else old_git(commit, path))
    root = tmp_path / "selector"
    plan = json.loads((ROOT / "config/portable_e2e_selector_weight_20260907.json").read_text())
    first._write(root / "plan.json", plan)
    state = copy.deepcopy(json.loads((expansion / "status.json").read_text()))
    state.update(plan=plan, source_commit=plan["source_commit"], plan_sha256=MODULE.base._sha((root / "plan.json").read_bytes()),
        prerequisite={"status": "TRAIN_EVAL_AUDIT_COMPLETE_NOT_PROMOTED", "completed_stages": 9,
                      "sha256": MODULE.base._sha((expansion / "status.json").read_bytes())})
    for seed in MODULE.base.SEEDS:
        relative = f"seed_{seed}/{MODULE.ARM}"
        shutil.copytree(expansion / f"seed_{seed}/C_expanded_data", root / relative)
        for stage, filename in MODULE.expansion.STAGE_FILES.items():
            path = root / relative / filename
            value = json.loads(path.read_text())
            if stage == "train":
                value["loss_config"]["candidate_score_weight"] = 0.5
            elif stage == "evaluate":
                for metrics in [value["metrics"], *[domain["metrics"] for domain in value["per_domain_metrics"].values()]]:
                    for name in metrics:
                        metrics[name] *= 0.8
            first._write(path, value)
            record = next(item for item in state["stages"] if item["run"] == f"seed_{seed}/C_expanded_data" and item["stage"] == stage)
            record["run"] = relative
            record["report"]["sha256"] = MODULE.base._sha(path.read_bytes())
            if stage == "train":
                record["command"].extend(["--candidate-score-weight", "0.5"])
    first._write(root / "status.json", state)
    return root


def _seal(root: Path, stage: str, mutate, *, arm: str = MODULE.ARM, seed: int = 20260903) -> None:
    # HH_260906 - Separate semantic rejection from report-byte tampering by explicitly updating fixture seals.
    relative = f"seed_{seed}/{arm}"
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


def test_same_corpus_three_pairs_pass_without_promotion(expansion: Path, selector: Path) -> None:
    report = MODULE.summarize_campaign(expansion, selector)
    assert report["status"] == "COMPLETE_NOT_PROMOTED"
    assert report["candidate_screen"] == report["absolute_quality"] == "PASS"
    assert report["automatic_promotion"] is report["vehicle_control_approved"] is False
    assert report["test_opened_by_this_campaign"] is False
    assert len(report["comparison"]["reports"]) == 6
    assert len(report["pairs"]) == 3 and len(report["input_manifest"]) == 22
    assert len(report["source_proof"]["files"]) == 5
    assert all(run["training_dataset_size"] == 1147 and run["training_episode_count"] == 3
               for pair in report["pairs"] for run in (pair["baseline"], pair["candidate"]))
    assert "동일 v3" in MODULE.render_markdown(report)


@pytest.mark.parametrize("metric,value,check", [
    ("selected_ade_m", 1.0, "selected_ade_improves"),
    ("selected_fde_m", 2.0, "selected_fde_improves"),
    ("selected_speed_mae_mps", 0.21001, "speed_mae_within_5_percent"),
])
def test_one_seed_tie_or_regression_fails_entire_screen(expansion: Path, selector: Path, metric, value, check) -> None:
    _seal(selector, "evaluate", lambda item: _metrics(item, metric, value), seed=20260905)
    report = MODULE.summarize_campaign(expansion, selector)
    assert report["candidate_screen"] == "FAIL"
    assert report["pairs"][0]["candidate_screen"] == "PASS"
    assert report["pairs"][2]["relative_checks"][check] is False
    assert report["automatic_promotion"] is False


def test_speed_exact_five_percent_boundary_passes(expansion: Path, selector: Path) -> None:
    _seal(selector, "evaluate", lambda value: _metrics(value, "selected_speed_mae_mps", 1.05 * 0.2))
    assert MODULE.summarize_campaign(expansion, selector)["candidate_screen"] == "PASS"


def test_geometry_regression_fails(expansion: Path, selector: Path) -> None:
    _seal(selector, "audit", lambda value: value["geometry"]["selected_result"].update(
        geometry_pass_count=335, geometry_reject_count=2, geometry_pass_rate=335 / 337))
    report = MODULE.summarize_campaign(expansion, selector)
    assert report["candidate_screen"] == "FAIL"
    assert report["pairs"][0]["relative_checks"]["geometry_does_not_regress"] is False


def test_absolute_failure_is_independent_of_relative_pass(expansion: Path, selector: Path) -> None:
    _seal(selector, "evaluate", lambda value: _metrics(value, "ade_6p4s_m", 2.01))
    report = MODULE.summarize_campaign(expansion, selector)
    assert report["candidate_screen"] == "PASS"
    assert report["absolute_quality"] == "FAIL"
    assert report["status"] == "COMPLETE_NOT_PROMOTED"


@pytest.mark.parametrize("mutation,error", [
    (lambda state: state["prerequisite"].update(sha256="f" * 64), "predecessor status byte hash"),
    (lambda state: state["prerequisite"].update(completed_stages=18), "predecessor completion"),
    (lambda state: state["stages"][0]["report"].update(sha256="f" * 64), "report SHA-256"),
    (lambda state: state["stages"].reverse(), "nine-stage order"),
    (lambda state: state["stages"][0]["command"].__setitem__(-1, "0.1"), "selector weight"),
    (lambda state: state["stages"][0]["command"].extend(["--candidate-score-weight", "0.5"]), "exactly one"),
    (lambda state: state["stages"][1]["command"].__setitem__(4, "test"), "unreviewed split"),
    (lambda state: state.update(source_commit="f" * 40), "plan/source"),
])
def test_stage_and_source_proofs_reject_tampering(expansion: Path, selector: Path, mutation, error) -> None:
    first._mutate(selector, "status.json", mutation)
    with pytest.raises(ContractError, match=error):
        MODULE.summarize_campaign(expansion, selector)


@pytest.mark.parametrize("root_name,arm,weight", [("candidate", MODULE.ARM, 0.1), ("baseline", "C_expanded_data", 0.5)])
def test_score_weight_is_not_weakened_for_old_or_new_schema(expansion: Path, selector: Path, root_name, arm, weight) -> None:
    root = selector if root_name == "candidate" else expansion
    _seal(root, "train", lambda value: value["loss_config"].update(candidate_score_weight=weight), arm=arm)
    with pytest.raises(ContractError, match="loss config"):
        MODULE.summarize_campaign(expansion, selector)


def test_core_git_byte_change_rejects_comparison(expansion: Path, selector: Path, monkeypatch) -> None:
    old_git = MODULE.base._git_bytes
    monkeypatch.setattr(MODULE.base, "_git_bytes", lambda commit, path:
        b"changed formula" if commit != MODULE.BASELINE_COMMIT and path == "portable_e2e/losses.py" else old_git(commit, path))
    with pytest.raises(ContractError, match="core source changed"):
        MODULE.summarize_campaign(expansion, selector)


def test_absolute_plan_limits_cannot_be_relaxed(expansion: Path, selector: Path) -> None:
    plan = json.loads((selector / "plan.json").read_text())
    plan["decision"]["absolute_limits_m"]["ade_6p4s"] = 200.0
    first._write(selector / "plan.json", plan)
    first._mutate(selector, "status.json", lambda state: state.update(plan=plan,
        plan_sha256=MODULE.base._sha((selector / "plan.json").read_bytes())))
    with pytest.raises(ContractError, match="absolute gates"):
        MODULE.summarize_campaign(expansion, selector)


def test_archived_worker_hash_mismatch_is_rejected(expansion: Path, selector: Path) -> None:
    path = selector / "provenance/active_runner.py"
    path.parent.mkdir()
    path.write_bytes(b"fixture worker")
    with pytest.raises(ContractError, match="archived worker SHA-256"):
        MODULE.summarize_campaign(expansion, selector)
    first._mutate(selector, "status.json", lambda state: state.update(runner_sha256=MODULE.base._sha(path.read_bytes())))
    assert MODULE.summarize_campaign(expansion, selector)["source_proof"]["workers"][1]["bytes_locally_verified"] is True


@pytest.mark.parametrize("missing", ["status.json", "seed_20260905/D_selector_weight/gate_v8.json"])
def test_missing_candidate_evidence_cannot_pass(expansion: Path, selector: Path, missing: str) -> None:
    (selector / missing).unlink()
    report = MODULE.summarize_campaign(expansion, selector)
    assert report["status"] == "INCOMPLETE"
    assert report["candidate_screen"] == report["absolute_quality"] == "NOT_EVALUATED"
    assert report["test_opened_by_this_campaign"] is None


def test_cli_is_create_only(expansion: Path, selector: Path, tmp_path: Path) -> None:
    output = tmp_path / "summary"
    args = [str(expansion), str(selector), "--output-dir", str(output)]
    assert MODULE.main(args) == 0
    payload = (output / "summary.json").read_bytes()
    assert MODULE.main(args) == 2
    assert (output / "summary.json").read_bytes() == payload
