"""HH_260906 - Use synthetic numeric fixtures only; never treat test plots as actual experiment evidence."""

from copy import deepcopy
import json

import pytest

from scripts.e2e import plot_portable_candidate_regret as module
from test_render_portable_no_accel_learning import rows as base_rows, encoded


def rows(arm):
    values = base_rows()
    for row in values:
        row["regression_loss"] = 2.0 if row["batch_domain_sample_counts"]["carla"] == 3 else 1.0
        if arm == module.ARMS[1]: row[module.AUXILIARY] = row["global_step"] / 1000
    return values


@pytest.mark.parametrize("arm", module.ARMS)
def test_all_batches_weighted_shared_objectives_and_separate_auxiliary(arm):
    values, bins = module.history(encoded(rows(arm)), arm)
    assert len(values) == 1540 and len(bins) == 16 and sum(b["sample_exposures"] for b in bins) == 6155
    assert bins[-1]["batch_count"] == 40 and bins[-1]["sample_exposures"] == 160
    assert bins[-1]["partial_display_bin"]
    assert bins[2]["regression_loss"] == pytest.approx((396 + 6) / 399)
    assert all("loss" not in b and (module.AUXILIARY in b) == (arm == module.ARMS[1]) for b in bins)
    if arm == module.ARMS[1]:
        assert bins[-1][module.AUXILIARY] == pytest.approx(sum(i / 1000 for i in range(1501, 1541)) / 40)


@pytest.mark.parametrize("fault", ["missing", "duplicate", "epoch", "batch", "samples", "nan", "negative", "bool", "missing_aux", "extra_aux", "unknown_arm"])
def test_bad_history_or_arm_rejected(fault):
    arm = module.ARMS[1] if fault == "missing_aux" else module.ARMS[0]
    data = rows(arm)
    if fault == "missing": data.pop()
    elif fault == "duplicate": data[12] = deepcopy(data[11])
    elif fault == "epoch": data[287]["epoch"] = 0
    elif fault == "batch": data[286]["batch_domain_sample_counts"]["carla"] = 4
    elif fault == "samples": data[9]["samples_seen"] += 1
    elif fault == "nan": data[0]["regression_loss"] = float("nan")
    elif fault == "negative": data[0]["regression_loss"] = -.1
    elif fault == "bool": data[0]["regression_loss"] = True
    elif fault == "missing_aux": data[0].pop(module.AUXILIARY)
    elif fault == "extra_aux": data[0][module.AUXILIARY] = 0.0
    else: arm = "old_arm"
    with pytest.raises((module.campaign.ContractError, ValueError)):
        module.history(encoded(data), arm)


@pytest.fixture
def inputs(tmp_path, monkeypatch):
    root = tmp_path / "campaign"; root.mkdir()
    summary = tmp_path / "summary/summary.json"; summary.parent.mkdir()
    output = tmp_path / "plots"
    report = {"schema": module.campaign.SCHEMA, "status": "COMPLETE_NOT_PROMOTED", "automatic_promotion": False,
        "vehicle_control_approved": False, "training_data_approved_by_this_report": False,
        "test_evaluated": False, "test_used_for_training_or_selection": False, "completed_stage_count": 18,
        "candidate_screen": "FAIL", "absolute_quality": "FAIL", "stages": [{"status": "COMPLETE"}] * 18,
        "pairs": [], "input_manifest": []}
    for seed in module.SEEDS:
        pair = {"seed": seed, "candidate_screen": "FAIL", "absolute_quality": "FAIL"}
        for label, arm in zip(("baseline", "candidate"), module.ARMS):
            prefix = f"seed_{seed}/{arm}"; item = root / prefix
            (item / "training").mkdir(parents=True); (item / "evaluation").mkdir()
            values = rows(arm)
            (item / "training/metrics.jsonl").write_bytes(encoded(values))
            training = {"last_metrics": values[-1]}
            metrics = {"selected_ade_m": 4., "selected_fde_m": 10., "selected_speed_mae_mps": .8}
            evaluation = {"metrics": {**metrics, "loss": 3., "regression_loss": 2.}}
            if arm == module.ARMS[1]:
                evaluation.update(loss_config=dict(module.campaign.LOSS, candidate_regret_weight=.1),
                    auxiliary_loss_metrics={"candidate_regret_loss": .75},
                    auxiliary_loss_metric_counts={"candidate_regret_loss": 337})
            pair[label] = {"arm": arm, "training_dataset_size": 1147, "validation_sample_count": 337,
                "metrics": {**metrics, "selection_regret_ade_m": .5}, "geometry": {"selected_pass_count": 336, "sample_count": 337},
                "auxiliary_diagnostics": module.campaign.auxiliary_diagnostics(training, evaluation, arm)}
            for name, value in (("training/run.json", training), ("evaluation/metrics.json", evaluation)):
                path = item / name; path.write_text(json.dumps(value))
                report["input_manifest"].append({"path": prefix + "/" + name, "sha256": module.campaign.sha_file(path)})
        report["pairs"].append(pair)
    summary.write_text(json.dumps(report))
    monkeypatch.setattr(module.campaign, "summarize_campaign", lambda *_: deepcopy(report))
    return root, summary, output, report


def test_render_numeric_fixture_two_images_and_complete_binding(inputs):
    pytest.importorskip("matplotlib"); image = pytest.importorskip("PIL.Image")
    root, summary, output, report = inputs
    proof = module.render(root, summary, output)
    assert len(proof["training_bins"]) == 6 and len(proof["inputs"]) == 18
    assert proof["raw_total_loss_compared"] is proof["model_loaded"] is proof["model_training"] is False
    assert proof["common_training_metrics"] == ["regression_loss", "selected_ade_m"]
    assert proof["candidate_only_auxiliary_metric"] == "candidate_regret_loss"
    assert proof["summary_sha256"] == module.campaign.sha_file(summary)
    for name, height in (("01_common_learning_and_auxiliary.png", 1440), ("02_paired_validation.png", 1200)):
        with image.open(output / name) as frame:
            assert frame.size == (1920, height); frame.verify()
    for line in (output / "SHA256SUMS").read_text().splitlines():
        sha, name = line.split("  ")
        assert module.campaign.sha_file(output / name) == sha


@pytest.mark.parametrize("fault", ["incomplete", "stale", "duplicate_manifest", "final_row", "raw_val", "auxiliary_count",
    "mutated_history", "mutated_summary", "existing", "inside_campaign", "inside_summary", "symlink"])
def test_fail_closed_publication(inputs, monkeypatch, fault):
    root, summary, output, report = inputs
    monkeypatch.setattr(module, "draw", lambda *_: None)
    if fault == "incomplete": report["status"] = "INCOMPLETE"; summary.write_text(json.dumps(report))
    elif fault == "stale": report["candidate_screen"] = "PASS"
    elif fault == "duplicate_manifest": report["input_manifest"].append(report["input_manifest"][0]); summary.write_text(json.dumps(report))
    elif fault in ("final_row", "raw_val", "auxiliary_count"):
        suffix = "training/run.json" if fault == "final_row" else "evaluation/metrics.json"
        entry = next(e for e in report["input_manifest"] if e["path"].endswith(suffix) and module.ARMS[1] in e["path"])
        path = root / entry["path"]; value = json.loads(path.read_text())
        if fault == "final_row": value["last_metrics"]["regression_loss"] += 1
        elif fault == "raw_val": value["metrics"]["selected_ade_m"] += 1
        else: value["auxiliary_loss_metric_counts"]["candidate_regret_loss"] = 336
        path.write_text(json.dumps(value)); entry["sha256"] = module.campaign.sha_file(path); summary.write_text(json.dumps(report))
    elif fault == "mutated_history":
        path = root / f"seed_{module.SEEDS[0]}/{module.ARMS[0]}/training/metrics.jsonl"
        monkeypatch.setattr(module, "draw", lambda *_: path.write_text("changed\n"))
    elif fault == "mutated_summary": monkeypatch.setattr(module, "draw", lambda *_: summary.write_text("changed\n"))
    elif fault == "existing": output.mkdir()
    elif fault == "inside_campaign": output = root / "plots"
    elif fault == "inside_summary": output = summary.parent / "plots"
    else: output.symlink_to(output.parent / "absent", target_is_directory=True)
    with pytest.raises((module.campaign.ContractError, ValueError)):
        module.render(root, summary, output)
    assert not (output / "SHA256SUMS").exists()


def test_dataset_physical_alias_rejected(inputs, tmp_path, monkeypatch):
    root, summary, _, _ = inputs
    repo = tmp_path / "repo"; repo.mkdir()
    physical = tmp_path / "physical"; physical.mkdir()
    (repo / "datasets").symlink_to(physical, target_is_directory=True)
    monkeypatch.setattr(module, "ROOT", repo)
    with pytest.raises(module.campaign.ContractError, match="outside"):
        module.render(root, summary, physical / "plots")
