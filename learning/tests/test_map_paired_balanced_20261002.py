"""HH_261002 - Synthetic matched-slot, source-boundary and native review tests."""

import hashlib
import json
import copy
import os
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from scripts.e2e import map_paired_balanced_contract_20261002 as contract
from scripts.e2e import prepare_map_paired_balanced_20261002 as prepare
from scripts.e2e import train_map_paired_balanced_20261002 as trainer


@pytest.fixture(scope="module")
def scheduled():
    return contract.schedule()


def test_fixed_matched_schedule(scheduled):
    indices, crops, bits = scheduled
    proof = contract.schedule_proof(indices, crops, bits)
    assert indices.shape == bits.shape == (10000, 8)
    assert crops.shape == (10000, 8, 4)
    assert proof["total_exposures_per_arm"] == 80000
    assert proof["original_exposures"] + proof["paired_exposures"] == 80000
    assert sum(proof["balanced_FALSE_exposures_per_pair"]) + sum(proof["balanced_TRUE_exposures_per_pair"]) == proof["paired_exposures"]
    assert max(abs(a-b) for a, b in zip(proof["balanced_FALSE_exposures_per_pair"], proof["balanced_TRUE_exposures_per_pair"])) <= 1
    assert all(bits.ravel()[indices.ravel() < 729] == 0)
    for start in range(0, 80000-1209+1, 1209):
        assert np.array_equal(np.sort(indices.ravel()[start:start+1209]), np.arange(1209))


@pytest.mark.parametrize("member", [0, 1, 2])
def test_schedule_mutation_rejected(scheduled, member):
    values = [value.copy() for value in scheduled]
    values[member].flat[0] += 1
    with pytest.raises(ValueError, match="schedule differs"):
        contract.schedule_proof(*values)


def test_pair_views_alternate_not_duplicate_slots(scheduled):
    indices, _, bits = scheduled
    for pair in (0, 119, 239, 240, 479):
        selected = bits.ravel()[indices.ravel() == 729+pair]
        assert len(selected) >= 66
        assert (selected[1:] != selected[:-1]).all()


def test_frozen_crop_math_is_reused():
    assert contract.apply_crop is contract.frozen.apply_crop
    assert contract.common_weights is contract.frozen.common_weights
    old_ids, old_crops = contract.frozen.schedule(contract.SEED)
    new_ids, new_crops, _ = contract.schedule()
    assert np.array_equal(new_ids, old_ids) and np.array_equal(new_crops, old_crops)


def test_actual_frozen_helper_pins():
    root = Path(__file__).resolve().parents[2] / "tools/e2e"
    for name in ("map_appearance_matched_contract_20261001.py", "train_map_appearance_matched_20261001.py"):
        assert contract.digest(root / name) == contract.FROZEN_SOURCE_SHA[name]


def test_exact_read_payload_pin(tmp_path):
    path = tmp_path / "input.json"
    value = b'{"authorized":false}'
    path.write_bytes(value)
    sha = hashlib.sha256(value).hexdigest()
    assert contract.read_json(path, sha) == {"authorized": False}
    path.write_bytes(b'{"authorized":true}')
    with pytest.raises(ValueError, match="pinned bytes"):
        contract.read_json(path, sha)


@pytest.mark.parametrize("kind", ["ancestor", "leaf", "lexical"])
def test_symlinks_and_lexical_escape_rejected(tmp_path, kind):
    folder = tmp_path / "real"
    folder.mkdir()
    (folder / "input").write_text("bytes")
    link = tmp_path / "link"
    link.symlink_to(folder, target_is_directory=True)
    if kind == "ancestor":
        path = link / "fresh"
    elif kind == "leaf":
        path = tmp_path / "leaf"
        path.symlink_to(folder / "input")
    else:
        path = folder / ".." / "fresh"
    with pytest.raises(ValueError):
        contract.canonical(path)


def test_native_review_keeps_original_rgb_and_seven_classes():
    false = np.full((360, 640, 3), 19, dtype=np.uint8)
    true = np.full_like(false, 237)
    target = np.indices((360, 640))[1].astype(np.uint8) % 7
    image = np.asarray(prepare.review_canvas("p04", 119, false, true, target))
    assert image.shape == (850, 1920, 3)
    np.testing.assert_array_equal(image[56:416, :640], false)
    np.testing.assert_array_equal(image[56:416, 640:1280], true)
    np.testing.assert_array_equal(image[56:416, 1280:], prepare.frozen.COLORS[target])
    expected = np.rint(.6 * false.astype(np.float64) + .4 * prepare.frozen.COLORS[target]).astype(np.uint8)
    np.testing.assert_array_equal(image[448:808, :640], expected)


def test_native_review_invalid_label_rejected():
    rgb = np.zeros((360, 640, 3), dtype=np.uint8)
    with pytest.raises(ValueError, match="ABI"):
        prepare.review_canvas("p02", 0, rgb, rgb, np.full((360, 640), 7, dtype=np.uint8))


def test_render_create_only_before_any_input_read(tmp_path, monkeypatch):
    output = tmp_path / "existing"
    output.mkdir()
    monkeypatch.setattr(prepare, "REVIEW_OUTPUT", output)
    monkeypatch.setattr(prepare, "capture_contract", lambda _: pytest.fail("must not read input"))
    with pytest.raises(ValueError, match="fresh canonical"):
        prepare.render_review(output)


def test_low_storage_before_output_or_qa(tmp_path, monkeypatch):
    output = tmp_path / "fresh"
    monkeypatch.setattr(prepare, "REVIEW_OUTPUT", output)
    monkeypatch.setattr(prepare.shutil, "disk_usage", lambda _: type("Space", (), {"free": 1})())
    monkeypatch.setattr(prepare, "capture_contract", lambda _: pytest.fail("no QA on low storage"))
    with pytest.raises(ValueError, match="reserve"):
        prepare.render_review(output)
    assert not output.exists()


def test_first_read_source_mutation_rejected(tmp_path):
    path = tmp_path / "data"
    path.write_bytes(b"first")
    pins = {str(path): contract.digest(path)}
    path.write_bytes(b"changed")
    with pytest.raises(ValueError, match="first-read"):
        prepare.recheck(pins)


def valid_review():
    return {"schema": contract.REVIEW_SCHEMA, "study_id": contract.NAME,
            "parent_weights_sha256": contract.PARENT_SHA, "render_manifest_sha256": "render",
            "training_plan_sha256": "plan", "research_fit_authorized": True,
            "native_diagnostic_denials_preserved": True, "vehicle_control_approved": False,
            "dev_or_test_used_for_fit": False, "ctrack_train_eligible": False,
            "independent_new_dev_sites": 0, "counts": copy.deepcopy(contract.COUNTS),
            "arms": copy.deepcopy(contract.ARMS), "optimization": contract.optimization(),
            "reviewer": "actual root image inspector", "review_note": "Actual six native full-FOV panels reviewed; no model approval claimed.",
            "source_reviews": {name: {"source_capture": str(prepare.CAPTURES[name]),
                                      "qa_sha256": contract.QA_SHA[name], "route_sha256": contract.ROUTES[name],
                                      "reviewed_indices": [0, 119, 239], "rgb_conditions": ["false", "true"],
                                      "semantic_target_overlay_reviewed": True, "all_240_retained": True,
                                      "review_note": "Actual paired RGB and shared seven-label semantic overlay reviewed."}
                               for name in contract.ROUTES}}


def test_actual_manual_permission_is_separate_from_unapproved_native_sources():
    contract.review_contract(valid_review(), "render", "plan", prepare.CAPTURES)
    assert "human_approval" not in valid_review()


@pytest.mark.parametrize("field,value", [
    ("research_fit_authorized", False), ("native_diagnostic_denials_preserved", False),
    ("vehicle_control_approved", True), ("dev_or_test_used_for_fit", True),
    ("ctrack_train_eligible", True), ("independent_new_dev_sites", 2),
    ("independent_new_dev_sites", False), ("parent_weights_sha256", "new-best"),
    ("render_manifest_sha256", "not-viewed"), ("training_plan_sha256", "changed"),
    ("reviewer", ""), ("review_note", "OK")])
def test_manual_permission_rejects_missing_review_or_scope_expansion(field, value):
    review = valid_review(); review[field] = value
    with pytest.raises(ValueError):
        contract.review_contract(review, "render", "plan", prepare.CAPTURES)


@pytest.mark.parametrize("field,value", [
    ("semantic_target_overlay_reviewed", False), ("all_240_retained", False),
    ("reviewed_indices", [119]), ("rgb_conditions", ["true"]),
    ("qa_sha256", "new-QA"), ("route_sha256", "old-p01"),
    ("source_capture", "other-capture"), ("review_note", "OK")])
def test_each_route_semantic_review_is_mandatory(field, value):
    review = valid_review(); review["source_reviews"]["p04"][field] = value
    with pytest.raises(ValueError):
        contract.review_contract(review, "render", "plan", prepare.CAPTURES)


@pytest.mark.parametrize("field,value", [("steps_per_arm", 2000), ("batch", 16),
    ("learning_rate", .0003), ("seed", 20261001), ("checkpoint_selection", "best_DEV")])
def test_optimizer_and_fixed_checkpoint_are_prospective_not_tuned(field, value):
    review = valid_review(); review["optimization"][field] = value
    with pytest.raises(ValueError):
        contract.review_contract(review, "render", "plan", prepare.CAPTURES)


def valid_manifest_registry():
    files = {name: {"sha256": sha} for name, sha in
             (("original_train729.npz", contract.ORIGINAL_SHA), ("parent_weights.npz", contract.PARENT_SHA),
              ("paired_train480.npz", "pair"), ("training_schedule.npz", "schedule"))}
    common = {"study_id": contract.NAME, "counts": copy.deepcopy(contract.COUNTS), "arms": copy.deepcopy(contract.ARMS),
              "route_sha256": dict(contract.ROUTES), "parent_weights_sha256": contract.PARENT_SHA,
              "native_diagnostic_denials_preserved": True, "ctrack_train_eligible": False,
              "dev_or_test_used_for_fit": False, "vehicle_control_approved": False, "independent_new_dev_sites": 0}
    manifest = dict(common, schema=contract.DATA_SCHEMA, labels=list(contract.LABELS), files=files,
                    raw_model_inputs=["RGB"], depth_or_pose_model_input=False, source_receipts_modified=False,
                    schedule_contract={}, paired_sample_ids=[f"{name}:frames/{index:04d}" for name in contract.ROUTES for index in range(240)])
    registry = dict(copy.deepcopy(common), schema=contract.REGISTRY_SCHEMA, research_fit_authorized=True,
                    optimization=contract.optimization(), broad_registry_modified=False, data_files=copy.deepcopy(files), common_schedule={})
    return manifest, registry


def test_manifest_whole480_two_views_not_960_independent_slots():
    manifest, registry = valid_manifest_registry()
    contract.validate_manifest(manifest, registry)
    assert manifest["counts"]["available_rgb_views"] == 1689
    assert manifest["counts"]["logical_slots_per_arm"] == 1209


@pytest.mark.parametrize("field,value", [("depth_or_pose_model_input", True), ("raw_model_inputs", ["RGB", "depth"]),
    ("source_receipts_modified", True), ("vehicle_control_approved", True), ("ctrack_train_eligible", True),
    ("dev_or_test_used_for_fit", True), ("native_diagnostic_denials_preserved", False)])
def test_manifest_rejects_non_RGB_and_changed_native_role(field, value):
    manifest, registry = valid_manifest_registry(); manifest[field] = value
    with pytest.raises(ValueError): contract.validate_manifest(manifest, registry)


def test_predeclared_physical_ids_cannot_be_filtered_reordered_or_duplicated():
    for kind in ("filter", "reverse", "duplicate"):
        manifest, registry = valid_manifest_registry()
        ids = manifest["paired_sample_ids"]
        if kind == "filter": ids.pop()
        elif kind == "reverse": ids.reverse()
        else: ids[0] = ids[1]
        with pytest.raises(ValueError): contract.validate_manifest(manifest, registry)


def test_only_view_changes_same_slots_targets_and_crops(monkeypatch):
    monkeypatch.setattr(contract, "ORIGINAL_COUNT", 1)
    original = np.full((1, 360, 640, 3), 10, dtype=np.uint8)
    false, true = np.full_like(original, 40), np.full_like(original, 200)
    target = (np.indices((360, 640))[1] % 7).astype(np.uint8)[None]
    arrays = original, target, false, true, target
    ids, boxes, bits = np.asarray([0, 1], dtype=np.int64), np.asarray([[0, 0, 360, 640], [30, 45, 270, 480]]), np.asarray([0, 1], dtype=np.uint8)
    a, ya, actual_a = trainer.batch_arrays(arrays, ids, boxes, bits, "control")
    b, yb, actual_b = trainer.batch_arrays(arrays, ids, boxes, bits, "balanced")
    np.testing.assert_array_equal(a[0], b[0]); np.testing.assert_array_equal(ya, yb)
    assert not np.array_equal(a[1], b[1])
    assert actual_a.tolist() == [0, 0] and actual_b.tolist() == [0, 1]


@pytest.mark.parametrize("arm,bits,ids", [("third", [0], [0]), ("balanced", [2], [0]),
    ("balanced", [1], [0]), ("balanced", [0], [-1]), ("control", [0], [1209])])
def test_batch_rejects_unknown_bits_original_rewrites_and_invalid_slots(arm, bits, ids):
    dummy = np.zeros((1, 360, 640, 3), dtype=np.uint8)
    target = np.zeros((1, 360, 640), dtype=np.uint8)
    with pytest.raises(ValueError):
        trainer.batch_arrays((dummy, target, dummy, dummy, target), np.asarray(ids, dtype=np.int64),
                             np.asarray([[0, 0, 360, 640]]), np.asarray(bits, dtype=np.uint8), arm)


def test_eval_absent_GT_class_reports_null_and_false_positive_pixels():
    matrix = np.zeros((7, 7), dtype=np.int64); matrix[1, 1] = 80; matrix[1, 3] = 20
    result = trainer.confusion_metrics(matrix)
    assert result["iou"]["lane_marking"] is None
    assert result["false_positive_pixels"]["lane_marking"] == 20


def test_evaluation_cannot_open_DEV_until_both_fixed_endpoints_exist(monkeypatch):
    monkeypatch.setattr(contract, "read_npz", lambda *args: pytest.fail("DEV must not open"))
    with pytest.raises(ValueError, match="both fixed final"):
        trainer.evaluate_endpoints({}, None, None)


def test_prepare_never_creates_output_without_external_manual_and_plan_pin(tmp_path, monkeypatch):
    output = tmp_path / "fresh"
    monkeypatch.setattr(prepare, "OUTPUT", output)
    with pytest.raises(ValueError): prepare.prepare("missing-review", "missing-plan", True)
    assert not output.exists()


def test_npz_exact_payload_hash_and_allow_pickle_false(tmp_path):
    path = tmp_path / "safe.npz"
    np.savez(path, value=np.asarray([1, 2], dtype=np.float32))
    sha = contract.digest(path)
    np.testing.assert_array_equal(contract.read_npz(path, sha)["value"], [1, 2])
    np.savez(path, value=np.asarray(["object"], dtype=object))
    with pytest.raises(ValueError): contract.read_npz(path, contract.digest(path))
    with pytest.raises(ValueError, match="pinned bytes"): contract.read_npz(path, sha)


def test_runner_reserve_before_any_cuda_import(tmp_path, monkeypatch):
    monkeypatch.setattr(trainer.shutil, "disk_usage", lambda _: type("Space", (), {"free": 1})())
    with pytest.raises(ValueError, match="reserve"): trainer.reserve(tmp_path / "fresh")


def test_fresh_standalone_source_clone_cli_without_Torch_or_workspace_alias(tmp_path):
    source = tmp_path / "source"
    source.mkdir()
    root = Path(__file__).resolve().parents[2]
    for name in contract.SOURCE_FILES:
        if name == "plan.md":
            (source / name).write_text("Prospective fixed final10000 plan.\n")
        elif (root / "tools/e2e" / name).is_file():
            shutil.copyfile(root / "tools/e2e" / name, source / name)
        else:
            shutil.copyfile(prepare.frozen.RUN / "source" / name, source / name)
    env = {key: value for key, value in os.environ.items() if key not in
           ("PYTHONPATH", "PYTHONHOME", "PYTHONSTARTUP", "PYTHONUSERBASE")}
    result = subprocess.run([sys.executable, str(source / "train_map_paired_balanced_20261002.py"), "--help"],
                            cwd=source, env=env, capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stderr
    assert "--expected-data-sha256" in result.stdout and "--validate-only" in result.stdout
    assert "torch" not in result.stderr


def test_source_bundle_exact_closure_and_original_architecture_pins(tmp_path):
    source = tmp_path / "source"; source.mkdir()
    root = Path(__file__).resolve().parents[2]
    for name in contract.SOURCE_FILES:
        if name == "plan.md": (source / name).write_text("Fixed10000 prospective plan.\n")
        elif (root / "tools/e2e" / name).is_file(): shutil.copyfile(root / "tools/e2e" / name, source / name)
        else: shutil.copyfile(prepare.frozen.RUN / "source" / name, source / name)
    files = {name: contract.digest(source / name) for name in contract.SOURCE_FILES}
    bundle = {"schema": contract.SOURCE_SCHEMA, "study_id": contract.NAME, "files": files,
              "data_manifest_sha256": "data", "study_registry_sha256": "registry",
              "training_plan_sha256": files["plan.md"]}
    contract.write_new(source / "bundle.json", bundle)
    sha = contract.digest(source / "bundle.json")
    assert trainer.source_contract(source, sha, "data", "registry") == bundle
    (source / "map_context_recovery.py").write_text("changed architecture")
    with pytest.raises(ValueError, match="source changed"):
        trainer.source_contract(source, sha, "data", "registry")


def test_minimum_space_checked_again_before_checkpoint_write():
    import ast
    source = Path(trainer.__file__).read_text()
    tree = ast.parse(source)
    run = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "run")
    segment = ast.get_source_segment(source, run)
    assert "reserve(OUTPUT)\n            state =" in segment
    assert 'reserve(OUTPUT)\n            with (arm_dir / "weights.npz")' in segment
    assert 'torch.nn.utils.clip_grad_norm_(model.parameters(), 5.)' in segment
    assert 'bool(torch.isfinite(norm))' in segment


def test_runner_does_not_open_known_DEV_or_resume_before_two_fixed_exports():
    source = Path(trainer.__file__).read_text()
    assert source.index('guard.write_new(OUTPUT / "fit_complete.json"') < source.index('evaluated = evaluate_endpoints(')
    assert 'args.resume' not in source and 'load_state_dict(checkpoint' not in source
    assert contract.optimization()["checkpoint_selection"] == "fixed_final_10000_only"


def test_build_archive_uses_actual_owner_provenance_location_not_invented_source_dir():
    source = Path(prepare.__file__).read_text()
    assert 'source.parent / "provenance" / logical' in source
    assert 'source.parent / "provenance/source"' not in source
