"""HH_261002 - Synthetic-only saved-map comparison proofs; never open actual datasets or run inference."""

import copy
import hashlib
import io
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from PIL import ImageFont
import pytest

from scripts.e2e import publish_map_sign_sampler_v2_comparison_20261002 as p


def digest(path): return hashlib.sha256(path.read_bytes()).hexdigest()


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(value if isinstance(value, bytes) else json.dumps(value, sort_keys=True, allow_nan=False).encode())
    return digest(path)


def npz(path, **arrays):
    buffer = io.BytesIO(); np.savez_compressed(buffer, **arrays)
    return write(path, buffer.getvalue())


@pytest.fixture
def archives(tmp_path, monkeypatch):
    """HH_261002 - Complete synthetic43/47 member trees retain real source pins but use no actual image/model artifacts."""
    runs = {v: tmp_path / v for v in ("v1", "v2")}; evaluation = tmp_path / "evaluation"
    monkeypatch.setattr(p, "RUNS", runs); monkeypatch.setattr(p, "EVAL", evaluation)
    monkeypatch.setattr(p, "OUTPUT", tmp_path / "publication")
    monkeypatch.setattr(p, "SHAPE", (2, 4)); monkeypatch.setattr(p, "INDICES", {k: [0, 1] for k in p.SPLITS})
    splits = {}
    target = np.tile(np.arange(8, dtype=np.uint8).reshape(1, 2, 4) % 7, (2, 1, 1))
    for split in p.SPLITS:
        sha = npz(evaluation / (split + ".npz"), rgb=np.full((2, 2, 4, 3), 91, np.uint8), target=target)
        splits[split] = (2, sha)
    monkeypatch.setattr(p, "SPLITS", splits)
    manifest_sha = write(evaluation / "manifest.json", {"files": {k: dict(file=k + ".npz", count=2, sha256=v[1], towns=["synthetic"], class_pixels=[1]*7) for k, v in splits.items()}})
    monkeypatch.setattr(p.frozen, "EVAL_MANIFEST_SHA", manifest_sha)
    initial_path = tmp_path / "initial.npz"
    initial_sha = npz(initial_path, synthetic=np.zeros(245831, np.float32)); monkeypatch.setattr(p, "INITIAL_SHA", initial_sha)
    data_sha = hashlib.sha256(b"synthetic manifest").hexdigest(); monkeypatch.setattr(p, "DATA_SHA", data_sha)
    registry_sha = hashlib.sha256(b"synthetic registry").hexdigest(); monkeypatch.setattr(p, "REGISTRY_SHA", registry_sha)
    schedule = dict(control_indices=(np.arange(32000) % 1209).reshape(4000, 8).astype(np.int64),
        candidate_indices=(np.arange(32000) % 1209).reshape(4000, 8).astype(np.int64),
        shared_crops=np.tile(np.asarray([0, 0, 360, 640], np.int16), (4000, 8, 1)),
        shared_FALSE_view_bits=np.zeros((4000, 8), np.uint8))
    for version, root in runs.items():
        for name in p.required_run_files(version): write(root / name, b"synthetic placeholder")
        sources = p.TRAIN_SOURCE if version == "v1" else p.V2_SOURCE
        for name in sources: write(root / "source" / name, p.local_training_source(name).read_bytes())
        write(root / "initial_model_weights.npz", initial_path.read_bytes())
        write(root / "inputs/manifest.json", b"synthetic manifest"); write(root / "inputs/study_train_registry.json", b"synthetic registry")
        proof = dict(schema="portable_e2e.map_train_hardnegative_sampler.20261002." + version,
            steps_per_arm=4000, batch=8, examples_per_arm=32000,
            candidate_slot_roles=["all_TRAIN"] * 4 + (["small_sign", "large_sign", "hard_negative", "hard_negative"] if version == "v1" else ["small_sign", "small_sign", "large_sign", "hard_negative"]),
            tensor_sha256={k: hashlib.sha256(v.tobytes()).hexdigest() for k, v in schedule.items()})
        schedule_path = "mining/proposed_schedule.npz" if version == "v1" else "new_declared_schedule.npz"
        schedule_sha = npz(root / schedule_path, **schedule)
        auth = dict(schema="portable_e2e.map_hardnegative_continuation.20261002." + version + ".authorization",
            source_sha256=sources, research_fit_authorized=True, native_denials_preserved=True,
            DEV_or_HOLDOUT_used_for_fit=False, automatic_model_promotion=False, vehicle_control_approved=False)
        if version == "v2":
            proof_sha = hashlib.sha256(json.dumps(proof, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
            auth["new_sampler_proof_sha256"] = proof_sha
            file_sha = write(root / "new_declared_schedule_proof.json", proof)
            write(root / "new_schedule_archive_receipt.json", dict(new_sampler_proof_sha256=proof_sha,
                new_schedule_proof_file_sha256=file_sha, new_schedule_npz_sha256=schedule_sha))
        auth_sha = write(root / "authorization.json", auth)
        arms = {}
        for arm in ("control", "candidate"):
            rows = [dict(step=i+1, physical_slot_indices=schedule[arm + "_indices"][i].tolist(),
                shared_crop_boxes=schedule["shared_crops"][i].tolist(), actual_paired_TRUE_bits=[0]*8) for i in range(4000)]
            exposure_sha = write(root / arm / "exposures.jsonl", b"\n".join(json.dumps(r).encode() for r in rows))
            write(root / arm / "history.jsonl", b"\n".join(json.dumps(dict(arm=arm, step=(i+1)*100, mean_loss_100=.1, wall_seconds=i+1.)).encode() for i in range(40)))
            write(root / arm / "weights.npz", initial_path.read_bytes())
            arms[arm] = dict(steps=4000, examples_seen=32000, parameter_count=245831, initial_model_weights_sha256=initial_sha,
                optimizer_initial_state_entries=0, initial_state_sha256="a"*64, weights_sha256=initial_sha,
                checkpoint_sha256=digest(root / arm / "checkpoint.pt"), full_exposure_journal_sha256=exposure_sha,
                physical_slot_exposures=np.bincount(schedule[arm + "_indices"].ravel(), minlength=1209).tolist(),
                **dict(zip(("sample_sequence_sha256", "crop_sequence_sha256", "view_sequence_sha256"),
                    [hashlib.sha256(schedule[k].tobytes()).hexdigest() for k in (arm + "_indices", "shared_crops", "shared_FALSE_view_bits")])))
            write(root / arm / "training.json", arms[arm])
        metrics = {}
        for split, (_, sha) in splits.items():
            metrics[split] = {}
            for arm in ("parent", "control", "candidate"):
                folder = "parent_diagnostic" if arm == "parent" else arm
                npz(root / folder / (split + "_predictions.npz"), prediction=target)
                metrics[split][arm] = dict(count=2, source_sha256=sha, weights_sha256=initial_sha,
                    known_exposed_diagnostic_not_independent=True, **p.confusion_metrics(target, target))
        summary = dict(schema="portable_e2e.map_hardnegative_continuation.20261002." + version,
            study_id="map_hardnegative_continuation_20261002_" + version, status="FIXED_CURRENT_PARENT_SAMPLER_AB_COMPLETE_DIAGNOSTIC_NOT_PROMOTED",
            source_sha256=sources, initial_model_weights_sha256=initial_sha, data_manifest_sha256=data_sha, data_registry_sha256=registry_sha,
            additional_updates_per_arm=4000, examples_per_arm=32000, initial_completed_updates=10000, total_lineage_updates_per_arm=14000,
            physical_TRAIN_slots=1209, parameter_count=245831, batch=8, seed=2026100203, learning_rate=3e-5, weight_decay=1e-4,
            gradient_norm_cap=5., stratum_frame_counts=dict(small_sign=391, large_sign=193, hard_negative=145),
            source_and_inputs_unchanged=True, model_promoted=False, vehicle_control_approved=False, known_evaluation_not_independent=True,
            evaluation_opens_only_after_both_final4000=True, validation_used_during_optimization=False, independent_new_dev_sites=0,
            labels_changed=False, architecture_unchanged=True, semantic_loss_unchanged=True, native_denials_preserved=True,
            training_authorized=True, main_capabilities_approved=0, subfunctions_approved=0,
            authorization_sha256=auth_sha, sampler_proof=proof, arms_result=arms, known_evaluation=metrics,
            exploratory_screen_result=p.diagnostic_screen(metrics), class_pixels=[1]*7, class_weights=[1.]*7,
            optimizer="synthetic same", shared_crop_law="synthetic same", RGB_views="FALSE", final_selection="fixed4000",
            mining_artifact_sha256={str(f.relative_to(root / "mining")): digest(f) for f in (root / "mining").iterdir()},
            initial_model_receipt_sha256={str(f.relative_to(root / "initial_lineage")): digest(f) for f in (root / "initial_lineage").rglob("*") if f.is_file()})
        write(root / "summary.json", summary); write(root / "known_evaluation.json", metrics)
        write(root / "fit_complete.json", dict(status="BOTH_FIXED_FINAL4000_ENDPOINTS_SAVED_BEFORE_KNOWN_EVALUATION", arms=arms))
    caller = dict(schema=p.CALLER_SCHEMA, output=str(p.OUTPUT), fixed_indices=p.INDICES,
        publication_source_sha256={k: digest(v) for k, v in p.publication_sources().items()},
        font=dict(path=str(p.FONT), sha256=digest(p.FONT)), runs={v: dict(root=str(root), files={name: digest(root / name) for name in p.inventory(root)}) for v, root in runs.items()},
        evaluation=dict(root=str(evaluation), files={k + ".npz": v[1] for k, v in splits.items()}))
    path = tmp_path / "caller.json"; sha = write(path, caller)
    return path, sha, caller, schedule


def test_full_synthetic43_47_readers_and_all_metrics(archives):
    path, sha, _, _ = archives
    report, panels, pins = p.audit(path, sha)
    assert report["source_counts"] == {"v1": 8, "v2": 9}
    assert report["V1_V2_control_numeric_exact_parity"] is True
    assert len(report["exploratory_screens"]["v2"]["checks"]) == 20
    assert report["exploratory_screens"]["v2"]["passed_exploratory_only"] is False
    assert report["formal_main_functions_approved"] == report["formal_subfunctions_approved"] == 0
    assert len(panels) == 4 and len(report["evaluation"]["v1"]["dev_town01"]["parent"]["confusion"]) == 7
    pins.recheck()


def test_fixed_indices_and_actual_fov_before_any_evaluation():
    assert p.INDICES == {"dev_town01": [0, 29, 58, 88, 117], "holdout_town02_p02": [0, 57, 115, 174, 231]}
    assert p.SHAPE == (360, 640)
    assert len(p.required_run_files("v1")) == 43 and len(p.required_run_files("v2")) == 47


@pytest.mark.parametrize("name", ["../x", "/x", "source/../x", "", "x//y"])
def test_path_escape_rejected(name):
    with pytest.raises(Exception): p.relative_name(name)


@pytest.mark.parametrize("bad", [True, 1, "a"*63, "A"*64])
def test_caller_sha_must_be_exact(bad, tmp_path):
    path = tmp_path / "x"; write(path, {})
    with pytest.raises(Exception): p.audit(path, bad)


def test_torch_process_cannot_claim_inference_free_publication(monkeypatch, tmp_path):
    monkeypatch.setitem(p.sys.modules, "torch", SimpleNamespace())
    with pytest.raises(Exception): p.audit(tmp_path / "never_read", "a"*64)


def test_reused_publisher_frozen_pin_is_required(monkeypatch, tmp_path):
    monkeypatch.setattr(p, "FROZEN_PUBLISHER_SHA", "a"*64)
    with pytest.raises(Exception): p.audit(tmp_path / "never_read", "a"*64)


@pytest.mark.parametrize("kind", ["missing", "extra", "symlink", "mutation"])
def test_complete_archive_source_guards(archives, kind):
    path, sha, caller, _ = archives
    root = p.RUNS["v1"]
    if kind == "missing": (root / "control/checkpoint.pt").unlink()
    elif kind == "extra": write(root / "unaccounted.json", {})
    elif kind == "symlink": (root / "linked").symlink_to(root / "summary.json")
    else: write(root / "source/map_perception.py", b"changed")
    with pytest.raises(Exception): p.audit(path, sha)


def test_boolean_exposure_is_not_integer():
    a = np.zeros((4000, 8), np.int64)
    schedule = dict(control_indices=a, candidate_indices=a, shared_crops=np.zeros((4000, 8, 4), np.int16), shared_FALSE_view_bits=np.zeros((4000, 8), np.uint8))
    row = dict(step=1, physical_slot_indices=[False]*8, shared_crop_boxes=[[0]*4]*8, actual_paired_TRUE_bits=[0]*8)
    with pytest.raises(Exception): p.exposure_proof(b"\n".join([json.dumps(row).encode()]*4000), schedule, "control", {})


def test_numeric_parity_false_is_disclosed_not_assumed():
    a = dict(state=np.zeros(245831, np.float32)); b = dict(state=a["state"].copy()); b["state"][0] = 1.
    assert p.numeric_model(a, b) is False


def test_confusion_keeps_absent_classes_and_false_positives():
    result = p.confusion_metrics(np.asarray([0, 1], np.uint8), np.asarray([5, 1], np.uint8))
    assert result["iou"]["traffic_sign"] is None
    assert result["false_positive_pixels"]["traffic_sign"] == result["other_to_sign_false_positive_pixels"] == 1
    assert result["recall"]["traffic_sign"] is None


def test_synthetic_canvas_preserves_each_native_fov(monkeypatch):
    shape = p.SHAPE; rgb = np.full((*shape, 3), 123, np.uint8); target = np.full(shape, 5, np.uint8)
    image = np.asarray(p.canvas(("dev_town01", 0, rgb, target, target, target, target, target), ImageFont.load_default()))
    assert image.shape == (2 * shape[0] + 180, 3 * shape[1], 3)
    assert np.array_equal(image[74:74 + shape[0], :shape[1]], rgb)
    assert np.array_equal(image[74:74 + shape[0], shape[1]:2*shape[1]], p.COLORS[target])


def test_validate_only_creates_no_outputs(archives):
    path, sha, _, _ = archives
    result = p.publish(SimpleNamespace(caller_pins=str(path), caller_pins_sha256=sha, validate_only=True))
    assert not result["published"] and not p.OUTPUT.exists()


def test_complete_synthetic_publication_is_create_only_and_archives_sources(archives, monkeypatch):
    path, sha, _, _ = archives
    monkeypatch.setattr(p.shutil, "disk_usage", lambda _: SimpleNamespace(free=10*1024**3))
    result = p.publish(SimpleNamespace(caller_pins=str(path), caller_pins_sha256=sha, validate_only=False))
    assert result["published"] and result["png_count"] == 4
    report = json.loads((p.OUTPUT / "results.json").read_text())
    assert report["original_sources_and_inputs_unchanged"] is True
    for name, expected in report["publication_source_archive"].items(): assert digest(p.OUTPUT / name) == expected
    with pytest.raises(Exception): p.publish(SimpleNamespace(caller_pins=str(path), caller_pins_sha256=sha, validate_only=False))


def test_existing_output_never_overwritten(archives):
    path, sha, _, _ = archives
    p.OUTPUT.mkdir(); sentinel = p.OUTPUT / "user"; sentinel.write_text("retained")
    with pytest.raises(Exception): p.publish(SimpleNamespace(caller_pins=str(path), caller_pins_sha256=sha, validate_only=False))
    assert sentinel.read_text() == "retained"


def test_late_input_mutation_is_fail_closed(archives):
    path, sha, _, _ = archives
    _, _, pins = p.audit(path, sha)
    write(p.RUNS["v2"] / "candidate/training.json", {})
    with pytest.raises(Exception): pins.recheck()


def test_markdown_preserves_failed_checks_and_parity(archives):
    path, sha, _, _ = archives
    report, _, _ = p.audit(path, sha); report["V1_V2_control_numeric_exact_parity"] = False
    text = p.markdown(report, {})
    assert "FAIL / 미채택" in text and "False" in text and "전체 350장" in text
    assert "신호 상태" in text and "메인 0/9" in text
