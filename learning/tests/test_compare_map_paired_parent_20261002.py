"""HH_261002 - Synthetic-only full49 parent comparisons, immutable pointers and create-only publication tests."""

import copy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts.e2e import compare_map_paired_parent_20261002 as comparison


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, sort_keys=True, allow_nan=False), encoding="utf-8")
    return digest(path)


def full_matrix(count):
    total = count*360*640
    values = [total//7]*7
    values[0] += total-sum(values)
    return [[values[i] if i == j else 0 for j in range(7)] for i in range(7)]


def old_row(matrix):
    calculated = comparison.metrics_from_confusion(matrix)
    return {key: calculated[key] for key in ("confusion", "iou", "precision", "pixel_accuracy")} | {
        "true_class_pixels": list(calculated["target_pixels"].values())}


def new_row(matrix, count, sha):
    return comparison.metrics_from_confusion(matrix) | {
        "count": count, "source_sha256": sha, "known_exposed_diagnostic_not_independent": True}


def cohort():
    parent, published = {}, {}
    for split, (old_name, count, sha) in comparison.SPLITS.items():
        old = full_matrix(count)
        old[5][5] -= 20; old[5][0] += 20
        control = full_matrix(count)
        control[5][5] -= 5; control[5][0] += 5
        balanced = full_matrix(count)
        balanced[5][5] -= 10; balanced[5][0] += 10
        parent[old_name] = old_row(old)
        published[split] = {"control": new_row(control, count, sha), "balanced": new_row(balanced, count, sha)}
    return parent, published


@pytest.fixture
def archive(tmp_path, monkeypatch):
    """HH_261002 - Opaque synthetic endpoint bytes are only hashed; no actual NPZ, PT or model loads occur."""
    parent_root, run, publication = tmp_path/"parent", tmp_path/"run", tmp_path/"docs"
    publication.mkdir(); parent_root.mkdir(); run.mkdir()
    monkeypatch.setattr(comparison, "ROOT", tmp_path)
    monkeypatch.setattr(comparison, "PARENT_ROOT", parent_root)
    monkeypatch.setattr(comparison, "RUN_ROOT", run)
    monkeypatch.setattr(comparison, "PUBLICATION", publication)
    parent, published = cohort()
    (parent_root/"B").mkdir()
    (parent_root/"B/weights.npz").write_bytes(b"opaque synthetic parent NPZ, never parse")
    (parent_root/"B/checkpoint.pt").write_bytes(b"opaque synthetic parent PT, never parse")
    weight_sha, pt_sha = digest(parent_root/"B/weights.npz"), digest(parent_root/"B/checkpoint.pt")
    monkeypatch.setattr(comparison, "PARENT_WEIGHTS_SHA", weight_sha)
    monkeypatch.setattr(comparison, "PARENT_CHECKPOINT_SHA", pt_sha)
    parent.update(parameter_count=245831, weights_sha256=weight_sha, checkpoint_sha256=pt_sha)
    monkeypatch.setattr(comparison, "PARENT_METRICS_SHA", write_json(parent_root/"B/metrics.json", parent))
    monkeypatch.setattr(comparison, "PARENT_SUMMARY_SHA", write_json(parent_root/"summary.json", {"B": parent}))
    publisher = tmp_path/"tools/e2e/publish_map_paired_balanced_results_20261002.py"
    publisher.parent.mkdir(parents=True)
    publisher.write_bytes(b"synthetic publisher source, never execute")
    monkeypatch.setattr(comparison, "PUBLISHER_SHA", digest(publisher))
    (publication/"source").mkdir()
    (publication/"source"/publisher.name).write_bytes(publisher.read_bytes())
    (publication/"README.md").write_text("Existing synthetic finished publication.\n")
    arms, summary_arms, first = {}, {}, {}
    for arm in ("control", "balanced"):
        (run/arm).mkdir()
        info = {}
        for filename, key in (("weights.npz", "weights_sha256"), ("checkpoint.pt", "checkpoint_sha256")):
            (run/arm/filename).write_bytes(("opaque synthetic " + arm + filename).encode())
            info[key] = digest(run/arm/filename)
            first[str(run/arm/filename)] = info[key]
        arms[arm] = info
        summary_arms[arm] = info | {"steps": 10000, "examples_seen": 80000}
    summary = {"parent_weights_sha256": weight_sha, "arms_result": summary_arms, "known_evaluation": published}
    summary_sha = write_json(run/"summary.json", summary)
    first[str(run/"summary.json")] = summary_sha
    report = {"schema": "portable_e2e.map_paired_balanced_results.20261002.v1",
        "status": "FULL_ARCHIVE_AND_PIXEL_EVALUATION_VERIFIED_NOT_MODEL_APPROVAL",
        "study_id": "map_paired_balanced_20261002_v1", "parent_weights_sha256": weight_sha,
        "labels": list(comparison.LABELS), "independent_new_dev_sites": 0,
        "formal_main_functions_approved": 0, "formal_sub_functions_approved": 0,
        "native_diagnostic_denials_preserved": True, "RGB_only_seven_class_perception_not_HD_AMG": True,
        "known_evaluation_used_for_fit_or_checkpoint_selection": False, "model_promoted": False,
        "vehicle_control_approved": False, "caller_summary_sha256": summary_sha, "arms": arms,
        "first_read_source_and_artifact_pins": first, "evaluation": published,
        "archived_publication_sources": {"source/"+publisher.name: {"original_path": str(publisher), "sha256": digest(publisher)}}}
    args = SimpleNamespace(results_json=publication/"results.json", results_sha256=write_json(publication/"results.json", report),
                           caller_summary_sha256=summary_sha, output=publication/"PARENT_COMPARISON.md", validate_only=True)
    return args, parent, report, summary


def test_all49_independent_ratios_and_absent_preserved():
    matrix = [[0]*7 for _ in range(7)]
    matrix[0][0], matrix[0][5], matrix[4][0], matrix[4][4] = 12, 3, 2, 6
    result = comparison.metrics_from_confusion(matrix)
    assert result["iou"]["traffic_sign"] is None
    assert result["recall"]["traffic_sign"] is None
    assert result["precision"]["traffic_sign"] == 0
    assert result["false_positive_pixels"]["traffic_sign"] == 3
    assert result["precision"]["pole"] is None
    assert result["recall"]["traffic_light_fixture"] == .75
    assert result["confusion"] == matrix
    assert result["pixel_count"] == 23


@pytest.mark.parametrize("mutation", [lambda x: x.pop(), lambda x: x[0].pop(), lambda x: x[0].__setitem__(0, True),
                                      lambda x: x[0].__setitem__(0, -1), lambda x: x[0].__setitem__(0, 1.5)])
def test_malformed_confusion_rejected(mutation):
    matrix = full_matrix(118); mutation(matrix)
    with pytest.raises(ValueError): comparison.metrics_from_confusion(matrix)


def test_full_target_denominator_not_partial():
    with pytest.raises(ValueError): comparison.metrics_from_confusion(full_matrix(118), expected_pixels=1)


def test_parent_gain_vs_balanced_regression_not_collapsed():
    parent, published = cohort()
    result = comparison.comparison(parent, published)
    for row in result.values():
        delta = row["iou_delta_percentage_points"]
        assert delta["control_minus_parent"]["traffic_sign"] > 0
        assert delta["balanced_minus_parent"]["traffic_sign"] > 0
        assert delta["balanced_minus_control"]["traffic_sign"] < 0
        assert set(row["models"]) == {"parent", "control", "balanced"}


@pytest.mark.parametrize("kind", ["old_iou", "new_fp", "same_support", "source", "independence", "missing_class"])
def test_saved_accounting_scope_mutations_rejected(kind):
    parent, published = cohort()
    row = published["dev_town01"]["control"]
    if kind == "old_iou": parent["town01"]["iou"]["road"] = .3
    if kind == "new_fp": row["false_positive_pixels"]["pole"] += 1
    if kind == "source": row["source_sha256"] = "a"*64
    if kind == "independence": row["known_exposed_diagnostic_not_independent"] = False
    if kind == "missing_class": row["iou"].pop("pole")
    if kind == "same_support":
        matrix = copy.deepcopy(row["confusion"]); matrix[0][0] -= 1; matrix[1][1] += 1
        published["dev_town01"]["control"] = new_row(matrix, 118, comparison.SPLITS["dev_town01"][2])
    with pytest.raises(ValueError): comparison.comparison(parent, published)


def test_full_synthetic_audit_and_validation_only(archive):
    args, _, _, _ = archive
    result = comparison.publish(args)
    assert result["published"] is False
    assert not args.output.exists()
    report, _ = comparison.audit(args)
    assert report["NPZ_or_PT_loaded"] is False
    assert report["model_inference_executed"] is False
    assert sum(len(row) for row in report["evaluation"]["dev_town01"]["models"]["parent"]["confusion"]) == 49


def test_create_only_readable_complete_markdown(archive):
    args, _, _, _ = archive
    args.validate_only = False
    result = comparison.publish(args)
    assert result["published"] is True
    body = args.output.read_text()
    for label in comparison.LABELS: assert label in body
    assert "control − 기존" in body and "balanced − control" in body
    assert "0/9" in body and "0/30" in body and "0개" in body
    assert body.count("<details>") == 2 and body.count("### parent") == 2
    original = args.output.read_bytes()
    with pytest.raises(ValueError): comparison.publish(args)
    assert args.output.read_bytes() == original


@pytest.mark.parametrize("name", ["source", "parent_weights", "parent_metrics", "new_weights", "summary"])
def test_actual_first_read_hash_mutation_rejected(archive, name):
    args, _, _, _ = archive
    paths = {"source": comparison.ROOT/"tools/e2e/publish_map_paired_balanced_results_20261002.py",
             "parent_weights": comparison.PARENT_ROOT/"B/weights.npz",
             "parent_metrics": comparison.PARENT_ROOT/"B/metrics.json",
             "new_weights": comparison.RUN_ROOT/"control/weights.npz", "summary": comparison.RUN_ROOT/"summary.json"}
    paths[name].write_bytes(paths[name].read_bytes()+b"\nchanged")
    with pytest.raises(ValueError): comparison.publish(args)
    assert not args.output.exists()


def test_mutation_after_first_read_before_create_rejected(archive, monkeypatch):
    args, _, _, _ = archive
    original = comparison.markdown
    def changed(report):
        body = original(report)
        path = comparison.RUN_ROOT/"balanced/weights.npz"
        path.write_bytes(b"late mutated synthetic archive")
        return body
    monkeypatch.setattr(comparison, "markdown", changed)
    args.validate_only = False
    with pytest.raises(ValueError): comparison.publish(args)
    assert not args.output.exists()


def test_publisher_must_be_finished_before_optional_write(archive):
    args, _, _, _ = archive
    (comparison.PUBLICATION/"README.md").unlink()
    with pytest.raises(ValueError): comparison.publish(args)
    assert not args.output.exists()


def test_symlink_ancestor_rejected(tmp_path):
    (tmp_path/"real").mkdir()
    (tmp_path/"link").symlink_to(tmp_path/"real", target_is_directory=True)
    with pytest.raises(ValueError): comparison.canonical(tmp_path/"link/fresh.md")


def test_caller_confusion_must_match_publication(archive):
    args, _, _, summary = archive
    matrix = full_matrix(118)
    summary["known_evaluation"]["dev_town01"]["control"]["confusion"] = matrix
    args.caller_summary_sha256 = write_json(comparison.RUN_ROOT/"summary.json", summary)
    report = json.loads(args.results_json.read_text())
    report["caller_summary_sha256"] = args.caller_summary_sha256
    report["first_read_source_and_artifact_pins"][str(comparison.RUN_ROOT/"summary.json")] = args.caller_summary_sha256
    args.results_sha256 = write_json(args.results_json, report)
    with pytest.raises(ValueError): comparison.publish(args)
