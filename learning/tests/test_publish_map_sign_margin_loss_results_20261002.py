"""HH_261002 - Test saved-loss publication with complete synthetic51 archives and no actual evaluation or models."""
import hashlib
import io
import json
from types import SimpleNamespace

import numpy as np
from PIL import ImageFont
import pytest

from scripts.e2e import publish_map_sign_margin_loss_results_20261002 as p


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(value if isinstance(value, bytes) else json.dumps(value, sort_keys=True, allow_nan=False).encode())
    return hashlib.sha256(path.read_bytes()).hexdigest()


def npz(path, **values):
    buffer = io.BytesIO(); np.savez_compressed(buffer, **values); return write(path, buffer.getvalue())


@pytest.fixture
def archives(tmp_path, monkeypatch):
    # HH_261002 - The tiny two-image fixture still retains every real4000 exposure,1209slot,10source, and zero-update witness.
    run, evaluation = tmp_path / "run", tmp_path / "evaluation"
    monkeypatch.setattr(p, "RUN", run); monkeypatch.setattr(p, "EVAL", evaluation); monkeypatch.setattr(p, "OUTPUT", tmp_path / "publication")
    monkeypatch.setattr(p, "SHAPE", (2, 4)); monkeypatch.setattr(p, "INDICES", {s: [0, 1] for s in p.SPLITS})
    target = np.tile((np.arange(8).reshape(1, 2, 4) % 7).astype(np.uint8), (2, 1, 1)); splits = {}
    for split in p.SPLITS:
        splits[split] = (2, npz(evaluation / (split + ".npz"), rgb=np.full((2, 2, 4, 3), 91, np.uint8), target=target))
    monkeypatch.setattr(p, "SPLITS", splits); monkeypatch.setattr(p.previous, "SPLITS", splits)
    manifest_sha = write(evaluation / "manifest.json", dict(files={s: dict(file=s + ".npz", count=2, sha256=sha) for s, (_, sha) in splits.items()}))
    monkeypatch.setattr(p.frozen, "EVAL_MANIFEST_SHA", manifest_sha)
    for name in p.required_run_files(): write(run / name, b"placeholder")
    for name in p.SOURCE: write(run / "source" / name, p.local_source(name).read_bytes())
    initial_sha = npz(run / "initial_model_weights.npz", fixture=np.zeros(245831, np.float32))
    monkeypatch.setattr(p, "INITIAL_SHA", initial_sha); monkeypatch.setattr(p.previous, "INITIAL_SHA", initial_sha)
    data_sha = write(run / "inputs/manifest.json", b"synthetic TRAIN metadata")
    registry_sha = write(run / "inputs/study_train_registry.json", b"synthetic registry")
    monkeypatch.setattr(p, "DATA_SHA", data_sha); monkeypatch.setattr(p, "REGISTRY_SHA", registry_sha)
    schedule = dict(control_indices=(np.arange(32000) % 1209).reshape(4000, 8).astype(np.int64),
        candidate_indices=(np.arange(32000) % 1209).reshape(4000, 8).astype(np.int64),
        shared_crops=np.tile(np.array([0, 0, 360, 640], np.int16), (4000, 8, 1)), shared_FALSE_view_bits=np.zeros((4000, 8), np.uint8))
    old = {key: value.copy() for key, value in schedule.items()}; old["candidate_indices"] = np.roll(old["candidate_indices"], 1, axis=0)
    npz(run / "mining/proposed_schedule.npz", **old)
    mining = {name: hashlib.sha256((run / "mining" / name).read_bytes()).hexdigest() for name in p.MINING_SHA}
    lineage = {name: hashlib.sha256((run / "initial_lineage" / name).read_bytes()).hexdigest() for name in p.LINEAGE_SHA}
    monkeypatch.setattr(p, "MINING_SHA", mining); monkeypatch.setattr(p, "LINEAGE_SHA", lineage)
    proof = dict(schema=p.FIT_SCHEMA + ".equal_global_schedule", both_arm_slot_roles=["all_TRAIN"] * 8,
        tensor_sha256={k: hashlib.sha256(v.tobytes()).hexdigest() for k, v in schedule.items()})
    proof_sha = p.document_sha(proof); monkeypatch.setattr(p, "EQUAL_PROOF_SHA", proof_sha)
    loss_contract = dict(coefficient=.1, seven_original_logits=True, original_CE_retained=True)
    loss_sha = p.document_sha(loss_contract); monkeypatch.setattr(p, "LOSS_CONTRACT_SHA", loss_sha)
    receipt = dict(schema=p.FIT_SCHEMA + ".archive", original_c019_unmodified=True, training_authorized_by_this_receipt=False,
        schedule_npz_sha256=npz(run / "equal_global_schedule.npz", **schedule),
        schedule_proof_file_sha256=write(run / "equal_global_schedule_proof.json", proof), equal_global_proof_sha256=proof_sha,
        loss_contract_file_sha256=write(run / "loss_contract.json", loss_contract), loss_contract_sha256=loss_sha)
    write(run / "intervention_archive_receipt.json", receipt)
    auth = dict(schema=p.FIT_SCHEMA + ".authorization", source_sha256=p.SOURCE, research_fit_authorized=True,
        native_denials_preserved=True, DEV_or_HOLDOUT_used_for_fit=False, automatic_model_promotion=False, vehicle_control_approved=False,
        equal_global_proof_sha256=proof_sha, loss_contract_sha256=loss_sha, contract=dict(training_authorized=False, sampler_unchanged=True))
    auth_sha = write(run / "authorization.json", auth); monkeypatch.setattr(p, "AUTH_SHA", auth_sha)
    arms = {}
    for arm in ("control", "candidate"):
        rows = [dict(step=i + 1, physical_slot_indices=schedule[arm + "_indices"][i].tolist(),
            shared_crop_boxes=schedule["shared_crops"][i].tolist(), actual_paired_TRUE_bits=[0] * 8) for i in range(4000)]
        exposure_sha = write(run / arm / "exposures.jsonl", b"\n".join(json.dumps(row).encode() for row in rows))
        write(run / arm / "history.jsonl", b"\n".join(json.dumps(dict(arm=arm, step=(i + 1) * 100, mean_loss_100=.1, wall_seconds=i + 1.)).encode() for i in range(40)))
        write(run / arm / "weights.npz", (run / "initial_model_weights.npz").read_bytes())
        checkpoint_sha = hashlib.sha256((run / arm / "checkpoint.pt").read_bytes()).hexdigest()
        arms[arm] = dict(steps=4000, examples_seen=32000, parameter_count=245831, initial_model_weights_sha256=initial_sha,
            initial_state_sha256=p.INITIAL_STATE_SHA, optimizer_initial_state_entries=0, weights_sha256=initial_sha, checkpoint_sha256=checkpoint_sha,
            full_exposure_journal_sha256=exposure_sha, physical_slot_exposures=np.bincount(schedule[arm + "_indices"].ravel(), minlength=1209).tolist(),
            **dict(zip(("sample_sequence_sha256", "crop_sequence_sha256", "view_sequence_sha256"),
                [hashlib.sha256(schedule[k].tobytes()).hexdigest() for k in (arm + "_indices", "shared_crops", "shared_FALSE_view_bits")])))
        write(run / arm / "training.json", arms[arm])
        witness = dict(schema=p.FIT_SCHEMA + ".strict_CUDA_preflight", arm=arm, status="STRICT_CUDA_FORWARD_BACKWARD_PASS_ZERO_UPDATES",
            parameter_count=245831, all_parameter_gradients_present_and_finite=True, initial_state_sha256=p.INITIAL_STATE_SHA,
            optimizer_updates=0, all_arm_optimizer_updates_so_far=0, completed_fit_arms=[], extra_TRAIN_diagnostic_exposures=8,
            known_DEV_or_HOLDOUT_opened=False, strict_deterministic_algorithms=True, TF32=False,
            actual_TRAIN_slot_indices=schedule[arm + "_indices"][0].tolist(), shared_crop_boxes=schedule["shared_crops"][0].tolist(),
            actual_paired_TRUE_bits=[0] * 8, rgb_array_sha256="a" * 64, target_array_sha256="b" * 64)
        write(run / (arm + "_strict_CUDA_preflight.json"), witness)
    metrics = {}
    for split, (count, sha) in splits.items():
        metrics[split] = {}
        for arm in ("parent", "control", "candidate"):
            npz(run / ("parent_diagnostic" if arm == "parent" else arm) / (split + "_predictions.npz"), prediction=target)
            metrics[split][arm] = dict(count=count, source_sha256=sha, weights_sha256=initial_sha,
                known_exposed_diagnostic_not_independent=True, **p.previous.confusion_metrics(target, target))
    summary = dict(schema=p.FIT_SCHEMA, study_id=p.NAME, status="FIXED_CURRENT_PARENT_SIGN_MARGIN_LOSS_AB_COMPLETE_DIAGNOSTIC_NOT_PROMOTED",
        source_sha256=p.SOURCE, initial_model_weights_sha256=initial_sha, data_manifest_sha256=data_sha, data_registry_sha256=registry_sha,
        additional_updates_per_arm=4000, examples_per_arm=32000, initial_completed_updates=10000, total_lineage_updates_per_arm=14000,
        physical_TRAIN_slots=1209, parameter_count=245831, batch=8, seed=2026100203, learning_rate=3e-5, weight_decay=1e-4, gradient_norm_cap=5.,
        stratum_frame_counts=dict(small_sign=391, large_sign=193, hard_negative=145), source_and_inputs_unchanged=True,
        model_promoted=False, vehicle_control_approved=False, known_evaluation_not_independent=True, evaluation_opens_only_after_both_final4000=True,
        validation_used_during_optimization=False, independent_new_dev_sites=0, labels_changed=False, architecture_unchanged=True,
        semantic_loss_unchanged=False, original_weighted_CE_unchanged=True, sampler_unchanged=True, native_denials_preserved=True,
        training_authorized=True, main_capabilities_approved=0, subfunctions_approved=0,
        both_arm_indices_crops_FALSE_bits_byte_identical=True, both_arm_strict_CUDA_preflights_before_ANY_optimizer_update=True,
        preflight_exposures_per_arm=8, preflight_optimizer_updates=0, preflight_extra_TRAIN_only_diagnostic_exposures_not_in_32000_optimizer_exposures=True,
        historical_runtime_binary_identity_claimed=False, authorization_sha256=auth_sha, mining_artifact_sha256=mining,
        initial_model_receipt_sha256=lineage, arms_result=arms, sampler_proof=proof, loss_contract=loss_contract,
        loss_contract_sha256=loss_sha, known_evaluation=metrics, exploratory_screen_result=p.diagnostic_screen(metrics))
    write(run / "summary.json", summary); write(run / "known_evaluation.json", metrics)
    write(run / "fit_complete.json", dict(status="BOTH_FIXED_FINAL4000_ENDPOINTS_SAVED_BEFORE_KNOWN_EVALUATION", arms=arms))
    caller = dict(schema=p.CALLER_SCHEMA, output=str(p.OUTPUT), fixed_indices=p.INDICES,
        remote_originals_independently_hashed=True, remote_run_root=p.REMOTE_RUN,
        publication_source_sha256={name: hashlib.sha256(path.read_bytes()).hexdigest() for name, path in p.publication_sources().items()},
        font=dict(path=str(p.FONT), sha256=hashlib.sha256(p.FONT.read_bytes()).hexdigest()),
        run=dict(root=str(run), files={name: hashlib.sha256((run / name).read_bytes()).hexdigest() for name in p.previous.inventory(run)}),
        evaluation=dict(root=str(evaluation), files={s + ".npz": sha for s, (_, sha) in splits.items()}))
    path = tmp_path / "caller.json"; sha = write(path, caller)
    return path, sha, caller, schedule


def repin(archives, name, value):
    path, _, caller, _ = archives; caller["run"]["files"][name] = write(p.RUN / name, value)
    return path, write(path, caller)


def test_complete_synthetic_reader_retains_denominators_and_failures(archives):
    report, panels, pins = p.audit(*archives[:2]); pins.recheck()
    assert len(panels) == 4 and report["source_count"] == 10
    assert len(report["mirrored_run_inventory"]) == 51
    assert len(report["exploratory_screen"]["checks"]) == 27 and not report["exploratory_screen"]["passed_exploratory_only"]
    assert report["actual_optimizer_exposures"]["candidate"]["examples_seen"] == 32000
    assert report["additional_CUDA_preflight_exposures_each_arm"] == 8
    assert report["formal_main_functions_approved"] == report["formal_subfunctions_approved"] == 0


def test_prospective_fixed_full_fov_and_complete51():
    assert p.INDICES == {"dev_town01": [0, 29, 58, 88, 117], "holdout_town02_p02": [0, 57, 115, 174, 231]}
    assert p.SHAPE == (360, 640) and len(p.required_run_files()) == 51 and len(p.SOURCE) == 10


@pytest.mark.parametrize("name", ["../escape", "/abs", "source/../x", "x//y", ""])
def test_canonical_relative_archive_names(name):
    with pytest.raises(Exception): p.previous.relative_name(name)


@pytest.mark.parametrize("bad", [True, 1, "A" * 64, "a" * 63])
def test_exact_external_sha_required(tmp_path, bad):
    path = tmp_path / "caller"; write(path, {})
    with pytest.raises(Exception): p.audit(path, bad)


@pytest.mark.parametrize("kind", ["missing", "extra", "symlink", "source"])
def test_complete_mirror_negative_guards(archives, kind):
    if kind == "missing": (p.RUN / "candidate/checkpoint.pt").unlink()
    elif kind == "extra": write(p.RUN / "extra", b"x")
    elif kind == "symlink": (p.RUN / "link").symlink_to(p.RUN / "summary.json")
    else: write(p.RUN / "source/map_sign_margin_loss_v1_20261002.py", b"changed")
    with pytest.raises(Exception): p.audit(*archives[:2])


@pytest.mark.parametrize("key,value", [("optimizer_updates", 1), ("optimizer_updates", False), ("completed_fit_arms", ["control"]),
    ("strict_deterministic_algorithms", False), ("TF32", True), ("extra_TRAIN_diagnostic_exposures", 0)])
def test_both_strict_zero_update_witnesses_are_not_summary_only(archives, key, value):
    name = "candidate_strict_CUDA_preflight.json"; row = json.loads((p.RUN / name).read_bytes()); row[key] = value
    path, sha = repin(archives, name, row)
    with pytest.raises(Exception): p.audit(path, sha)


def test_original_schedule_equal_arm_and_FALSE_are_not_relaxed(archives):
    path, _, caller, schedule = archives; changed = {k: v.copy() for k, v in schedule.items()}; changed["candidate_indices"][0, 0] = 1
    caller["run"]["files"]["equal_global_schedule.npz"] = npz(p.RUN / "equal_global_schedule.npz", **changed)
    with pytest.raises(Exception): p.audit(path, write(path, caller))


def test_all49_confusion_and_null_absent_class_retained():
    m = p.previous.confusion_metrics(np.asarray([0, 1], np.uint8), np.asarray([5, 1], np.uint8))
    assert m["iou"]["traffic_sign"] is None and m["recall"]["traffic_sign"] is None
    assert m["other_to_sign_false_positive_pixels"] == m["false_positive_pixels"]["traffic_sign"] == 1
    assert np.asarray(m["confusion"]).shape == (7, 7)


def test_false_reported_diagnostic_or_metric_is_rejected(archives):
    name = "summary.json"; summary = json.loads((p.RUN / name).read_bytes())
    key = next(iter(summary["exploratory_screen_result"]["checks"])); summary["exploratory_screen_result"]["checks"][key] ^= True
    path, sha = repin(archives, name, summary)
    with pytest.raises(Exception): p.audit(path, sha)


def test_original_model_or_pickle_never_loaded(monkeypatch, tmp_path):
    monkeypatch.setitem(p.sys.modules, "torch", SimpleNamespace())
    with pytest.raises(Exception): p.audit(tmp_path / "absent", "a" * 64)


@pytest.mark.parametrize("key,value", [("remote_originals_independently_hashed", False),
    ("remote_originals_independently_hashed", 1), ("remote_run_root", "/foreign/run")])
def test_external_remote_original_provenance_is_explicit(archives, key, value):
    path, _, caller, _ = archives; caller[key] = value
    with pytest.raises(Exception): p.audit(path, write(path, caller))


def test_validate_only_creates_nothing(archives):
    result = p.publish(SimpleNamespace(caller_pins=str(archives[0]), caller_pins_sha256=archives[1], validate_only=True))
    assert not result["published"] and not p.OUTPUT.exists()


def test_full_native_panel_pixels_are_not_resized_or_GT_fallback():
    rgb = np.full((*p.SHAPE, 3), 123, np.uint8); gt = np.full(p.SHAPE, 5, np.uint8); pred = np.full(p.SHAPE, 3, np.uint8)
    screen = dict(checks={str(i): False for i in range(27)}, passed_exploratory_only=False)
    report = dict(exploratory_screen=screen, model_weights_sha256={a: "a" * 64 for a in ("parent", "control", "candidate")})
    image = np.asarray(p.canvas(("dev_town01", 0, rgb, gt, pred, pred, pred), ImageFont.load_default(), report))
    assert np.array_equal(image[74:434, :640], rgb)
    assert np.array_equal(image[74:434, 640:1280], p.COLORS[gt])
    assert np.array_equal(image[474:834, 640:1280], p.COLORS[pred])


def test_create_only_full_synthetic_publication_and_archived_sources(archives, monkeypatch):
    monkeypatch.setattr(p.shutil, "disk_usage", lambda _: SimpleNamespace(free=10 * 1024**3))
    args = SimpleNamespace(caller_pins=str(archives[0]), caller_pins_sha256=archives[1], validate_only=False)
    result = p.publish(args); assert result["png_count"] == 4 and result["published"]
    report = json.loads((p.OUTPUT / "results.json").read_bytes())
    for name, sha in report["publication_source_archive"].items(): assert hashlib.sha256((p.OUTPUT / name).read_bytes()).hexdigest() == sha
    assert report["original_sources_and_inputs_unchanged"] is True
    with pytest.raises(Exception): p.publish(args)


def test_late_source_or_input_mutation_fails_first_read_postcheck(archives):
    _, _, pins = p.audit(*archives[:2]); write(p.RUN / "candidate/training.json", {})
    with pytest.raises(Exception): pins.recheck()


def test_existing_user_output_is_never_overwritten(archives):
    p.OUTPUT.mkdir(); sentinel = p.OUTPUT / "user"; sentinel.write_text("preserve")
    with pytest.raises(Exception): p.publish(SimpleNamespace(caller_pins=str(archives[0]), caller_pins_sha256=archives[1], validate_only=False))
    assert sentinel.read_text() == "preserve"


def test_markdown_is_conditional_not_a_success_template(archives):
    report, _, _ = p.audit(*archives[:2]); text = p.markdown(report, {})
    assert "FAIL / 미채택" in text and "27" in text and "전체를 검산" in text and "메인 0/9" in text
    report["exploratory_screen"] = dict(checks={str(i): True for i in range(27)}, passed_exploratory_only=True)
    assert "탐색 기준 PASS(기능 승인 아님)" in p.markdown(report, {})
