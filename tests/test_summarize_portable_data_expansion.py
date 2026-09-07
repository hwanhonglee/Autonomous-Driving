"""HH_260906 - Verify expanded-corpus evidence, stage seals and non-promotion boundaries."""

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


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("expansion_summary", ROOT / "scripts/e2e/summarize_portable_data_expansion.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


@pytest.fixture
def expansion(campaign: Path, tmp_path: Path, monkeypatch) -> Path:
    # HH_260906 - Build a complete second-study fixture from the shared full-val337 report contract.
    monkeypatch.setattr(MODULE.base, "_git_bytes", first.MODULE._git_bytes)
    root = tmp_path / "expanded"
    plan = json.loads((ROOT / "config/portable_e2e_data_expansion_20260907.json").read_text())
    first._write(root / "plan.json", plan)
    original = json.loads((campaign / "status.json").read_text())
    state = {"status": "TRAIN_EVAL_AUDIT_COMPLETE_NOT_PROMOTED", "source_commit": plan["source_commit"], "plan": plan,
        "plan_sha256": MODULE.base._sha((root / "plan.json").read_bytes()), "model_config_sha256": original["model_config_sha256"],
        "dataset_manifest_sha256": MODULE.MANIFEST_SHA256, "vehicle_control_approved": False, "runner_sha256": "e" * 64,
        "reviewed_contract": {"train_samples": 1147, "val_samples": 337, "run_count": 3, "stage_count": 9},
        "prerequisite": {"status": "TRAIN_EVAL_AUDIT_COMPLETE_NOT_PROMOTED", "completed_stages": 18, "sha256": "d" * 64},
        "corpus_fingerprint_sha256": MODULE.CORPUS_SHA256, "train_fingerprint_sha256": "4" * 64,
        "val_fingerprint_sha256": "1" * 64, "stages": []}
    for seed in MODULE.base.SEEDS:
        relative = f"seed_{seed}/{MODULE.ARM}"
        item = root / relative
        shutil.copytree(campaign / f"seed_{seed}/A_baseline", item)
        for stage, filename in MODULE.STAGE_FILES.items():
            path = item / filename
            value = json.loads(path.read_text())
            value["corpus_fingerprint_sha256"] = MODULE.CORPUS_SHA256
            if stage == "train":
                value["dataset_size"] = 1147
                value["training_episode_ids"].append("train-2-town01-right")
                value["dataset_fingerprint_sha256"] = "4" * 64
                value["state"]["domain_samples_seen"] = {"carla": 6155}
            else:
                value["training_episode_count"] = 3
                value["training_dataset_fingerprint_sha256"] = "4" * 64
                value["training_domain_samples_seen"] = {"carla": 6155}
            first._write(path, value)
            record = copy.deepcopy(next(record for record in original["stages"]
                if record["run"] == f"seed_{seed}/A_baseline" and record["stage"] == stage))
            record["run"] = relative
            record["report"] = {"path": filename, "sha256": MODULE.base._sha(path.read_bytes()),
                "corpus_fingerprint_sha256": MODULE.CORPUS_SHA256, "dataset_fingerprint_sha256": value["dataset_fingerprint_sha256"]}
            state["stages"].append(record)
    first._write(root / "status.json", state)
    return root


def test_expansion_three_seed_quality_never_promotes(expansion: Path, campaign: Path) -> None:
    report = MODULE.summarize_campaign(expansion)
    assert report["status"] == "COMPLETE_NOT_PROMOTED"
    assert report["absolute_quality"] == "PASS"
    assert len(report["runs"]) == 3
    assert report["automatic_promotion"] is report["vehicle_control_approved"] is False
    assert report["test_opened_by_this_campaign"] is False
    assert report["budget"]["approximate_epochs"] == pytest.approx(1540 / 287)
    assert report["cross_corpus_comparison"]["status"] == "NOT_PERFORMED"
    assert report["cross_corpus_comparison"]["automatic_pass"] is False
    assert all(run["training_dataset_size"] == 1147 and run["training_episode_count"] == 3
               and run["validation_episode_count"] == 1 for run in report["runs"])
    assert len(report["input_manifest"]) == 11
    with pytest.raises(ContractError, match="comparison is not fair"):
        MODULE.base.compare_reports([campaign / "seed_20260903/A_baseline/evaluation/metrics.json",
                                    expansion / "seed_20260903/C_expanded_data/evaluation/metrics.json"])


def _mutate_and_seal(root: Path, stage: str, mutate) -> None:
    # HH_260906 - Re-seal fixture reports so semantic checks are tested independently of byte-hash checks.
    relative = "seed_20260903/C_expanded_data"
    path = root / relative / MODULE.STAGE_FILES[stage]
    value = json.loads(path.read_text())
    mutate(value)
    first._write(path, value)
    state = json.loads((root / "status.json").read_text())
    record = next(record for record in state["stages"] if record["run"] == relative and record["stage"] == stage)
    record["report"]["sha256"] = MODULE.base._sha(path.read_bytes())
    first._write(root / "status.json", state)


@pytest.mark.parametrize("stage,mutate,error", [
    ("train", lambda value: value.update(dataset_size=613), "full train1147"),
    ("train", lambda value: value.update(training_episode_ids=["train-0", "train-1"]), "unique training episodes"),
    ("evaluate", lambda value: value.update(evaluation_episode_count=2), "episode counts"),
    ("evaluate", lambda value: value.update(training_episode_count=2), "episode counts"),
    ("audit", lambda value: value.update(checkpoint_sha256="f" * 64), "checkpoint_sha256 mismatch"),
])
def test_semantic_report_mismatch_rejected(expansion: Path, stage, mutate, error) -> None:
    _mutate_and_seal(expansion, stage, mutate)
    with pytest.raises(ContractError, match=error):
        MODULE.summarize_campaign(expansion)


@pytest.mark.parametrize("mutation,error", [
    (lambda state: state.update(dataset_manifest_sha256="f" * 64), "manifest proof"),
    (lambda state: state["reviewed_contract"].update(train_samples=613), "contract mismatch"),
    (lambda state: state["prerequisite"].update(completed_stages=17), "predecessor completion"),
    (lambda state: state["stages"][0]["report"].update(sha256="f" * 64), "report SHA-256"),
    (lambda state: state["stages"][0]["report"].update(corpus_fingerprint_sha256="f" * 64), "corpus fingerprint"),
    (lambda state: state.update(val_fingerprint_sha256="f" * 64), "split fingerprint"),
    (lambda state: state["stages"].reverse(), "nine-stage order"),
    (lambda state: state["stages"][1]["command"].__setitem__(4, "test"), "unreviewed split"),
])
def test_status_proof_mutation_rejected(expansion: Path, mutation, error) -> None:
    first._mutate(expansion, "status.json", mutation)
    with pytest.raises(ContractError, match=error):
        MODULE.summarize_campaign(expansion)


def test_unsealed_report_edit_is_rejected(expansion: Path) -> None:
    first._mutate(expansion, "seed_20260903/C_expanded_data/training/run.json", lambda value: value.update(dataset_size=613))
    with pytest.raises(ContractError, match="report SHA-256"):
        MODULE.summarize_campaign(expansion)


@pytest.mark.parametrize("missing", ["status.json", "seed_20260905/C_expanded_data/gate_v8.json"])
def test_missing_evidence_has_no_quality_pass(expansion: Path, missing: str) -> None:
    (expansion / missing).unlink()
    report = MODULE.summarize_campaign(expansion)
    assert report["status"] == "INCOMPLETE"
    assert report["absolute_quality"] == "NOT_EVALUATED"
    assert report["test_opened_by_this_campaign"] is None
    assert missing in report["missing_artifacts"]


def test_quality_failure_stays_complete_not_promoted(expansion: Path) -> None:
    def mutation(value):
        for metrics in [value["metrics"], *[domain["metrics"] for domain in value["per_domain_metrics"].values()]]:
            metrics["ade_6p4s_m"] = 3.0
    _mutate_and_seal(expansion, "evaluate", mutation)
    report = MODULE.summarize_campaign(expansion)
    assert report["status"] == "COMPLETE_NOT_PROMOTED"
    assert report["absolute_quality"] == "FAIL"
    assert report["runs"][0]["absolute_checks"]["ade_6p4s_m"] is False
    assert report["vehicle_control_approved"] is False


def test_expansion_cli_does_not_overwrite(expansion: Path, tmp_path: Path) -> None:
    output = tmp_path / "summary"
    args = [str(expansion), "--output-dir", str(output)]
    assert MODULE.main(args) == 0
    payload = (output / "summary.json").read_bytes()
    assert "5.37 epoch" in (output / "README.md").read_text()
    assert MODULE.main(args) == 2
    assert (output / "summary.json").read_bytes() == payload
