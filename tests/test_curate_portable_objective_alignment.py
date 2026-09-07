"""HH_260906 - Test aggregate-only objective publication with synthetic private reports and no model execution."""

from __future__ import annotations

import copy
import importlib.util
import json
from pathlib import Path

import pytest

from portable_e2e.contract import ContractError

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("objective_curator", ROOT / "scripts/e2e/curate_portable_objective_alignment.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _write(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value))


def make_summary(count: int, split="train") -> dict:
    # HH_260906 - Three distinct candidate choices expose accidental agreement, confusion-axis and averaging errors.
    samples = []
    for index in range(count):
        episode = f"train-{index % 3}" if split == "train" else "validation"
        samples.append({"index": index, "sample_id": f"{episode}:{index:06d}", "selected_index": 2,
            "composite_oracle_index": 1, "ade_oracle_index": 0, "target_valid": [True] * 64, "valid_point_count": 64,
            "candidate_ade_m": [0., 1., 2., 3., 4., 5.], "selected_ade_m": 2., "composite_oracle_ade_m": 1.,
            "ade_oracle_ade_m": 0., "selected_minus_ade_oracle_m": 2., "composite_oracle_minus_ade_oracle_m": 1.,
            "selected_minus_composite_oracle_ade_m": 1.})
    summary = {"sample_count": count, "candidate_count": 6, "predicted_points": 64, "per_sample": samples}
    chosen = {"selected": 2, "composite_oracle": 1, "ade_oracle": 0}
    for name, index in chosen.items():
        summary[f"{name}_histogram"] = [count if value == index else 0 for value in range(6)]
    for left, right in MODULE.PAIRS:
        key = f"{left}_{right}"
        matrix = [[0] * 6 for _ in range(6)]
        matrix[chosen[left]][chosen[right]] = count
        summary.update({f"{key}_agreement_count": 0, f"{key}_agreement_rate": 0.,
                        f"{key}_confusion_rows_left_columns_right": matrix})
    for field in MODULE.MEAN_FIELDS:
        summary[f"mean_{field}"] = samples[0][field]
    return summary


@pytest.fixture
def evidence(tmp_path: Path, monkeypatch):
    raw, baseline, candidate = [tmp_path / name for name in ("raw", "baseline", "candidate")]
    source = {"portable_e2e/losses.py": "a" * 64, "scripts/e2e/diagnose_portable_selector.py": "b" * 64}
    monkeypatch.setattr(MODULE, "source_hashes", lambda commit: source)
    monkeypatch.setattr(MODULE.campaign_summary, "summarize_campaign", lambda *args: {"status": "COMPLETE_NOT_PROMOTED"})
    commit, script = "c" * 40, MODULE.sha(ROOT / MODULE.SCRIPT)
    summaries = {split: make_summary(count, split) for split, (count, _) in MODULE.SPLITS.items()}
    for seed in MODULE.SEEDS:
        for arm, weight in MODULE.ARMS.items():
            relative = f"seed_{seed}/{arm}"
            run = (baseline if arm == "C_expanded_data" else candidate) / relative
            loss = {"xy_weight": 1., "speed_weight": .2, "yaw_weight": .1, "kinematic_speed_weight": .05,
                    "final_displacement_weight": .5, "candidate_score_weight": weight}
            metadata = {"checkpoint_sha256": MODULE.hashlib.sha256(relative.encode()).hexdigest(), "model_config_sha256": "d" * 64,
                "model_parameter_count": 954590, "training_dataset_fingerprint_sha256": MODULE.SPLITS["train"][1],
                "training_sampling_plan_sha256": "e" * 64, "training_sampling_policy": "uniform_without_replacement",
                "training_domain_samples_seen": {"carla": 6155}}
            _write(run / "evaluation/metrics.json", {**metadata, "metrics": {"selected_ade_m": 2.}})
            _write(run / "gate_v8.json", {"geometry": {"selector": {"selection_counts": {
                str(index): 337 if index == 2 else 0 for index in range(6)}}}})
            _write(run / "training/run.json", {"train_config": {"seed": seed}, "loss_config": loss,
                "training_episode_ids": ["train-0", "train-1", "train-2"]})
            report = {"schema": MODULE.SCHEMA, "status": "DIAGNOSIS_COMPLETE", "diagnostic_source_commit": commit,
                "diagnostic_script_sha256": script, "source_sha256": source, "manifest_sha256": MODULE.MANIFEST_SHA256,
                "corpus_fingerprint_sha256": MODULE.CORPUS_SHA256, "device": "cpu", "torch_num_threads": 4,
                "torch_num_interop_threads": 4, "vehicle_control_approved": False, "training_episode_count": 3,
                "evaluation_episode_count": 1, "loss_config": loss, **metadata, "checkpoint_id": "portable_e2e.pytorch_checkpoint.v1",
                "diagnostic_wall_seconds": 10., "torch_version": "fixture-torch", "private_path": "/home/sample/private/result",
                "splits": {split: {"split": split, "dataset_fingerprint_sha256": fingerprint,
                    "interpretation": "Train and val remain separate.", "summary": summaries[split]}
                    for split, (_, fingerprint) in MODULE.SPLITS.items()}}
            _write(raw / relative / "objective_alignment.json", report)
    return raw, baseline, candidate, commit, script


def _mutate(evidence, mutate, *, arm="C_expanded_data", seed=20260903) -> None:
    path = evidence[0] / f"seed_{seed}/{arm}/objective_alignment.json"
    value = json.loads(path.read_text())
    mutate(value)
    _write(path, value)


def test_recompute_all_aggregates_without_mutating_original() -> None:
    original = make_summary(3)
    snapshot = copy.deepcopy(original)
    summary, ids = MODULE.validate_summary(original, 3)
    assert original == snapshot and "per_sample" not in summary
    assert summary["selected_composite_oracle_agreement_rate"] == 0
    assert summary["selected_composite_oracle_confusion_rows_left_columns_right"][2][1] == 3
    assert summary["mean_selected_minus_ade_oracle_m"] == 2.
    assert len(ids) == 3


@pytest.mark.parametrize("mutation,error", [
    (lambda value: value.update(sample_count=4), "denominator"),
    (lambda value: value["per_sample"].pop(), "per-sample denominator"),
    (lambda value: value["per_sample"][0].update(index=1), "sample order"),
    (lambda value: value["per_sample"][0].update(selected_index=True), "candidate index"),
    (lambda value: value["per_sample"][0].update(ade_oracle_index=1), "ADE oracle"),
    (lambda value: value["per_sample"][0].update(selected_ade_m=3), "recomputed metric"),
    (lambda value: value["per_sample"][0]["candidate_ade_m"].__setitem__(0, float("nan")), "finite metric"),
    (lambda value: value["per_sample"][0].update(target_valid=[False, True] + [False] * 62), "prefix mask"),
    (lambda value: value["per_sample"][0].update(target_valid=[1] * 64), "prefix mask"),
    (lambda value: value["per_sample"][0].update(valid_point_count=63), "valid-point denominator"),
    (lambda value: value["per_sample"][1].update(sample_id=value["per_sample"][0]["sample_id"]), "duplicate sample"),
    (lambda value: value["selected_histogram"].__setitem__(2, 2), "histogram"),
    (lambda value: value["selected_ade_oracle_confusion_rows_left_columns_right"][2].__setitem__(0, 2), "confusion"),
    (lambda value: value.update(selected_ade_oracle_agreement_count=1), "agreement count"),
    (lambda value: value.update(selected_ade_oracle_agreement_rate=.01), "recomputed metric"),
    (lambda value: value.update(mean_selected_ade_m=2.01), "recomputed metric"),
    (lambda value: value.update(unknown=1), "unexpected summary"),
])
def test_bad_sample_or_aggregate_is_rejected(mutation, error) -> None:
    summary = make_summary(3)
    mutation(summary)
    with pytest.raises(ContractError, match=error):
        MODULE.validate_summary(summary, 3)


def test_all_six_exact_private_reports_validate_and_omit_rows(evidence) -> None:
    validated = MODULE.validate_inputs(*evidence)
    assert len(validated["records"]) == 6
    for record in validated["records"]:
        assert record["public"]["raw_source_sha256"] == MODULE.sha(record["source"])
        assert record["public"]["omitted_per_sample_counts"] == {"train": 1147, "val": 337}
        assert record["public"]["private_path"] == "${USER_HOME}/private/result"
        assert all("per_sample" not in item["summary"] for item in record["public"]["splits"].values())
        assert "per_sample" in MODULE.read_json(record["source"])["splits"]["train"]["summary"]


@pytest.mark.parametrize("mutation,error", [
    (lambda report: report.update(diagnostic_source_commit="d" * 40), "scope/source"),
    (lambda report: report.update(diagnostic_script_sha256="d" * 64), "scope/source"),
    (lambda report: report["source_sha256"].update({"portable_e2e/losses.py": "d" * 64}), "scope/source"),
    (lambda report: report.update(manifest_sha256="d" * 64), "scope/source"),
    (lambda report: report.update(corpus_fingerprint_sha256="d" * 64), "scope/source"),
    (lambda report: report.update(checkpoint_sha256="d" * 64), "checkpoint/evaluation"),
    (lambda report: report["loss_config"].update(candidate_score_weight=.5), "loss config"),
    (lambda report: report["splits"].update(test=report["splits"]["val"]), "partial splits forbidden"),
    (lambda report: report["splits"]["val"].update(split="train"), "split label"),
    (lambda report: report["splits"]["train"].update(dataset_fingerprint_sha256=MODULE.SPLITS["val"][1]), "fingerprint"),
])
def test_provenance_checkpoint_and_split_changes_rejected(evidence, mutation, error) -> None:
    _mutate(evidence, mutation)
    with pytest.raises(ContractError, match=error):
        MODULE.validate_inputs(*evidence)


def test_missing_or_extra_episode_diagnostics_fail_before_output(evidence, tmp_path) -> None:
    path = evidence[0] / "seed_20260905/D_selector_weight/objective_alignment.json"
    path.rename(path.with_suffix(".private-incomplete"))
    output = tmp_path / "publication"
    with pytest.raises(ContractError, match="exactly six"):
        MODULE.publish(*evidence[:3], output, *evidence[3:])
    assert not output.exists()


def test_sample_coverage_must_match_between_models(evidence) -> None:
    _mutate(evidence, lambda report: report["splits"]["train"]["summary"]["per_sample"][0].update(sample_id="train-0:different"),
            arm="D_selector_weight", seed=20260905)
    with pytest.raises(ContractError, match="coverage/order"):
        MODULE.validate_inputs(*evidence)


def test_exact_training_episode_identity_required(evidence) -> None:
    _mutate(evidence, lambda report: report["splits"]["train"]["summary"]["per_sample"][0].update(sample_id="unrelated:000000"))
    with pytest.raises(ContractError, match="episode identities"):
        MODULE.validate_inputs(*evidence)


def test_incomplete_paired_campaign_rejected(evidence, monkeypatch) -> None:
    monkeypatch.setattr(MODULE.campaign_summary, "summarize_campaign", lambda *args: {"status": "INCOMPLETE"})
    with pytest.raises(ContractError, match="campaign evidence is incomplete"):
        MODULE.validate_inputs(*evidence)


@pytest.mark.parametrize("kind", ["directory", "file", "symlink"])
def test_existing_output_refused_before_any_source_read(tmp_path, monkeypatch, kind) -> None:
    path = tmp_path / "output"
    if kind == "directory": path.mkdir()
    elif kind == "file": path.write_text("user file")
    else: path.symlink_to(tmp_path / "missing")
    monkeypatch.setattr(MODULE, "validate_inputs", lambda *args: pytest.fail("existing output must reject first"))
    with pytest.raises(ContractError, match="must be new"):
        MODULE.publish(tmp_path, tmp_path, tmp_path, path, "c" * 40, "d" * 64)


def test_complete_publication_has_exact_hashes_no_private_samples_and_real_png(evidence, tmp_path) -> None:
    from PIL import Image
    output = tmp_path / "publication"
    result = MODULE.publish(*evidence[:3], output, *evidence[3:])
    assert result["source_reports"] == 6 and result["summary_rows"] == 12 and result["published_files"] == 11
    summary = MODULE.read_json(output / "summary.json")
    assert summary["automatic_promotion"] is summary["vehicle_control_approved"] is summary["test_opened"] is False
    assert summary["raw_per_sample_records_omitted"] == 8904
    with Image.open(output / "01_train_val_objective_agreement.png") as image:
        image.load()
        assert image.size == (1920, 1080)
    for line in (output / "SHA256SUMS").read_text().splitlines():
        digest, relative = line.split("  ", 1)
        assert MODULE.sha(output / relative) == digest
    for record in MODULE.read_json(output / "publication_manifest.json")["files"]:
        assert MODULE.sha(output / record["published_path"]) == record["sha256"]
        assert MODULE.sha(Path(record["source_path"].replace("${TMP_ROOT}", "/tmp"))) == record["raw_source_sha256"]
    for path in output.rglob("*.json"):
        text = path.read_text()
        assert '"per_sample":' not in text and "/home/" not in text and '"sample_id":' not in text
