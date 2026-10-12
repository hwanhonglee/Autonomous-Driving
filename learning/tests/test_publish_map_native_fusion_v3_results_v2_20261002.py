"""HH_261002 - Actual-shaped numeric metadata receipts in tiny CPU synthetic52 galleries, never real data or jobs."""
from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from PIL import Image

from scripts.e2e import publish_map_native_fusion_v3_results_v2_20261002 as p


def document(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(json.dumps(value, sort_keys=True, allow_nan=False).encode())
    return p.frozen.contract.digest(path)


def numeric(path, **values):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("xb") as stream: np.savez_compressed(stream, **values)
    return p.frozen.contract.digest(path)


@pytest.fixture
def study(tmp_path, monkeypatch):
    # HH_261002 - Two/three tiny16x32 frames are explicitly synthetic; public350/native640x360 constants are tested separately.
    root = tmp_path / "workspace"; run = root / "run"; evaluation = root / "eval"
    monkeypatch.setattr(p, "ROOT", root); monkeypatch.setattr(p, "RUN", run)
    monkeypatch.setattr(p, "EVAL", evaluation); monkeypatch.setattr(p, "OUTPUT", root / "gallery")
    monkeypatch.setattr(p, "COMPLETION", root / "ROOT_completion.json")
    monkeypatch.setattr(p, "SHAPE", (16, 32)); monkeypatch.setattr(p, "INDICES", {"dev_town01": [0, 1], "holdout_town02_p02": [0, 1, 2]})
    source_file = root / "fake_publication.py"; source_file.parent.mkdir(parents=True); source_file.write_bytes(b"# synthetic source only\n")
    monkeypatch.setattr(p, "publication_sources", lambda: {"fake_publication.py": source_file})
    source = {}
    for name in p.SOURCE:
        folder = "learning/portable_e2e" if name in ("map_context_recovery.py", "map_perception.py", "map_sign_margin_loss_v1_20261002.py", "map_native_fusion_v1_20261002.py") else "tools/e2e"
        path = root / folder / name; path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(("# synthetic scientific source " + name).encode()); source[name] = p.frozen.contract.digest(path)
    monkeypatch.setattr(p, "SOURCE", source)
    for name in p.required_run_files():
        path = run / name; path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(("SYNTHETIC_HASH_ONLY_NOT_MODEL_OR_DATA:" + name).encode())
    metrics = {}; splits = {}; manifest = {"files": {}}
    for split, count in (("dev_town01", 2), ("holdout_town02_p02", 3)):
        target = np.zeros((count, *p.SHAPE), np.uint8)
        target.reshape(count, -1)[:, :48] = np.repeat(np.arange(1, 7, dtype=np.uint8), 8)
        rgb = np.stack([target * 17, target * 23, target * 31], axis=-1)
        sha = numeric(evaluation / (split + ".npz"), rgb=rgb, target=target)
        splits[split] = (count, sha); manifest["files"][split] = dict(file=split + ".npz", count=count, sha256=sha)
        metrics[split] = {}
        for arm in ("parent", "control", "candidate"):
            pred = target.copy(); flat = pred.reshape(count, -1)
            if arm == "candidate": flat[:, 32] = 0
            else: flat[:, 48:64] = 5
            relative = ("parent_diagnostic" if arm == "parent" else arm) + "/" + split + "_predictions.npz"
            (run / relative).unlink(); pred_sha = numeric(run / relative, prediction=pred)
            row = p.previous.previous.confusion_metrics(target, pred)
            row.update(count=count, evaluated_frames=count, source_sha256=sha, full_FOV=True,
                prediction_file_sha256=pred_sha, known_exposed_diagnostic_not_independent=True,
                weights_sha256=p.frozen.contract.digest(run / ("initial_model_weights.npz" if arm == "parent" else arm + "/weights.npz")))
            metrics[split][arm] = row
    monkeypatch.setattr(p, "SPLITS", splits)
    monkeypatch.setattr(p.frozen, "EVAL_MANIFEST_SHA", document(evaluation / "manifest.json", manifest))
    for name in source: (run / "source" / name).write_bytes(("# synthetic scientific source " + name).encode())
    journals = {a: dict(steps=20000, examples_seen=160000) for a in ("control", "candidate")}
    arms = {a: dict(steps=20000, examples_seen=160000, weights_sha256=p.frozen.contract.digest(run / a / "weights.npz")) for a in journals}
    # HH_261002 - Match the actual writer: training.json contains info, while summary/fit arms additionally bind its SHA.
    for a, value in arms.items(): value["metadata_sha256"] = document(run / a / "training.json", value)
    screen = p.previous.diagnostic_screen(metrics)
    assert sum(screen["checks"].values()) == 23
    assert sorted(k for k, value in screen["checks"].items() if not value) == p.FAILED
    summary = dict(schema=p.FIT_SCHEMA, study_id=p.NAME, status="FIXED20K_ARCHITECTURE_PAIR_DIAGNOSTIC_NOT_PROMOTED",
        source_sha256=source, source_count=15, source_and_inputs_unchanged=True, training_authorized=True,
        model_promoted=False, vehicle_control_approved=False, known_evaluation=metrics, arms_result=arms,
        actual_boundary=journals, exploratory_screen_result=screen)
    proof = dict(schema="portable_e2e.temporal_left_right_detached_launch.20261002.v2.MAP_completion", PID=3472102,
        process_start_ticks=140240638, source_asset_numeric_endpoints_and_full_journals_verified=True,
        all350_prediction_confusions_and_metrics_rebuilt_on_CPU=True, all27_original_screens_recomputed_including_failures=True,
        source15_and_original_archived_assets_runtime_unchanged=True, owned_predecessor_generation_finished=True,
        model_promoted=False, vehicle_control_approved=False, official_main_features_approved=0,
        official_subfeatures_approved=0, passed_diagnostic_screens=23, diagnostic_screen_denominator=27,
        failed_diagnostic_screens=list(p.FAILED), actual_both_final_journals=journals)
    proof["summary_sha256"] = document(run / "summary.json", summary)
    proof["known_evaluation_sha256"] = document(run / "known_evaluation.json", metrics)
    proof["fit_complete_sha256"] = document(run / "fit_complete.json", dict(
        status="BOTH_FIXED_FINAL20000_NUMERIC_ENDPOINTS_VERIFIED_BEFORE_KNOWN_EVALUATION",
        known_payload_opened=False, arms=arms, actual_full_journal_evidence=journals))
    proof["complete_output_file_sha256"] = {name: dict(sha256=p.frozen.contract.digest(run / name), bytes=(run / name).stat().st_size) for name in p.required_run_files()}
    completion_sha = document(p.COMPLETION, proof); monkeypatch.setattr(p, "COMPLETION_SHA", completion_sha)
    caller = dict(schema=p.CALLER_SCHEMA, output=str(p.OUTPUT), fixed_indices=p.INDICES,
        remote_run_root=p.REMOTE_RUN, remote_originals_independently_hashed=True,
        MAP_completion=dict(path=str(p.COMPLETION), sha256=completion_sha),
        publication_source_sha256={"fake_publication.py": p.frozen.contract.digest(source_file)},
        font=dict(path=str(p.FONT), sha256=p.frozen.contract.digest(p.FONT)),
        run=dict(root=str(run), files={k: v["sha256"] for k, v in proof["complete_output_file_sha256"].items()}),
        evaluation=dict(root=str(evaluation), files={s + ".npz": sha for s, (_, sha) in splits.items()}))
    path = root / "ROOT_caller.json"; pin = document(path, caller)
    return SimpleNamespace(caller=caller, path=path, sha=pin, proof=proof, summary=summary)


def reseal(study):
    # HH_261002 - Re-seal synthetic receipts only to exercise semantic guards independently from immutable ROOT SHA guards.
    study.proof["summary_sha256"] = document(p.RUN / "summary.json", study.summary)
    study.proof["complete_output_file_sha256"] = {name: dict(sha256=p.frozen.contract.digest(p.RUN / name), bytes=(p.RUN / name).stat().st_size) for name in p.required_run_files()}
    p.COMPLETION_SHA = document(p.COMPLETION, study.proof)
    study.caller["MAP_completion"]["sha256"] = p.COMPLETION_SHA
    study.caller["run"]["files"] = {k: v["sha256"] for k, v in study.proof["complete_output_file_sha256"].items()}
    study.sha = document(study.path, study.caller)


def test_complete52_small_synthetic_pixels_four_failures_no_models(study):
    report, panels, pins = p.audit(study.path, study.sha); pins.recheck()
    assert len(report["complete_output_file_sha256"]) == 52 and report["source_count"] == 15
    assert len(panels) == 5 and report["full_cohort_counts"] == {"dev_town01": 2, "holdout_town02_p02": 3}
    assert sum(report["exploratory_screen"]["checks"].values()) == 23
    assert all(report[key] is False for key in ("model_promoted", "vehicle_control_approved", "model_inference_executed", "Torch_imported", "pickle_loaded", "raw_TRAIN_reopened", "endpoint_numeric_weights_loaded", "full_journals_independently_reparsed_here", "live_mapping_latency_measured"))
    assert report["recorded_diagnostic_rendering_not_Autoware_or_RViz_screenshot"]
    assert report["endpoint_and_full_journal_verification_attested_by_fixed_ROOT_completion"]
    for rows in report["evaluation"].values():
        assert rows["candidate"]["recall"]["traffic_sign"] < rows["parent"]["recall"]["traffic_sign"]
        assert rows["candidate"]["iou"]["traffic_sign"] > rows["parent"]["iou"]["traffic_sign"]


@pytest.mark.parametrize("fault", ["caller_sha", "completion_sha", "old_caller", "selected_frames", "remote_claim", "missing_file", "extra_file", "changed_file", "census_size", "census_hash", "symlink", "source_changed"])
def test_source_and_complete_root_seal_fail_closed_before_arrays(study, monkeypatch, fault):
    if fault == "caller_sha": study.sha = "0" * 64
    elif fault == "completion_sha": study.caller["MAP_completion"]["sha256"] = "0" * 64
    elif fault == "old_caller": study.caller["schema"] = p.previous.CALLER_SCHEMA
    elif fault == "selected_frames": study.caller["fixed_indices"] = {"dev_town01": [1], "holdout_town02_p02": [1]}
    elif fault == "remote_claim": study.caller["remote_originals_independently_hashed"] = False
    elif fault == "missing_file": (p.RUN / "loader_parity.json").unlink()
    elif fault == "extra_file": (p.RUN / "extra.bin").write_bytes(b"not allowed")
    elif fault == "changed_file": (p.RUN / "initial_model_weights.npz").write_bytes(b"changed opaque bytes")
    elif fault in ("census_size", "census_hash"):
        row = study.proof["complete_output_file_sha256"]["loader_parity.json"]
        row["bytes" if fault == "census_size" else "sha256"] = -1 if fault == "census_size" else "0" * 64
        monkeypatch.setattr(p, "COMPLETION_SHA", document(p.COMPLETION, study.proof)); study.caller["MAP_completion"]["sha256"] = p.COMPLETION_SHA
    elif fault == "symlink":
        file = p.RUN / "loader_parity.json"; file.unlink(); file.symlink_to(study.path)
    elif fault == "source_changed": (p.ROOT / "tools/e2e/train_map_native_fusion_pair_v3_20261002.py").write_bytes(b"changed source")
    if fault != "caller_sha": study.sha = document(study.path, study.caller)
    monkeypatch.setattr(p.frozen.Pins, "npz", lambda *args: pytest.fail("unproven inventory cannot open arrays"))
    with pytest.raises(Exception): p.audit(study.path, study.sha)


@pytest.mark.parametrize("fault", ["missing_completion_proof", "promoted", "formal_approval", "lost_failure", "lower_denominator", "wrong_source_count", "wrong_study", "early_known", "partial_training", "wrong_endpoint", "known_link", "screen_changed"])
def test_authenticated_synthetic_semantic_limits(study, fault):
    if fault == "missing_completion_proof": study.proof["all350_prediction_confusions_and_metrics_rebuilt_on_CPU"] = False
    elif fault == "promoted": study.proof["model_promoted"] = True
    elif fault == "formal_approval": study.proof["official_main_features_approved"] = True
    elif fault == "lost_failure": study.proof["failed_diagnostic_screens"].pop()
    elif fault == "lower_denominator": study.proof["diagnostic_screen_denominator"] = 23
    elif fault == "wrong_source_count": study.summary["source_count"] = 14
    elif fault == "wrong_study": study.summary["study_id"] = "old_v2"
    elif fault == "early_known":
        value = json.loads((p.RUN / "fit_complete.json").read_bytes()); value["known_payload_opened"] = True
        study.proof["fit_complete_sha256"] = document(p.RUN / "fit_complete.json", value)
    elif fault == "partial_training": study.summary["arms_result"]["candidate"]["steps"] = 13300
    elif fault == "wrong_endpoint": study.summary["arms_result"]["candidate"]["weights_sha256"] = "0" * 64
    elif fault == "known_link": study.summary["known_evaluation"]["dev_town01"]["candidate"]["recall"]["traffic_sign"] = 1.0
    elif fault == "screen_changed": study.summary["exploratory_screen_result"]["checks"][p.FAILED[0]] = True
    reseal(study)
    with pytest.raises(Exception): p.audit(study.path, study.sha)


@pytest.mark.parametrize("fault", ["missing_frame", "cropped_FOV", "invalid_class", "wrong_dtype", "saved_metric", "wrong_pred_sha", "wrong_weight", "changed_tail"])
def test_entire_prediction_and_metrics_not_only_preview(study, fault):
    split = "holdout_town02_p02"; name = "candidate/" + split + "_predictions.npz"; path = p.RUN / name
    if fault in ("missing_frame", "cropped_FOV", "invalid_class", "wrong_dtype", "changed_tail"):
        with np.load(path, allow_pickle=False) as arrays: value = arrays["prediction"].copy()
        if fault == "missing_frame": value = value[:-1]
        elif fault == "cropped_FOV": value = value[:, :, :-1]
        elif fault == "invalid_class": value[-1, -1, -1] = 7
        elif fault == "wrong_dtype": value = value.astype(np.int64)
        elif fault == "changed_tail": value[-1, -1, -1] = 1
        path.unlink(); numeric(path, prediction=value)
    else:
        rows = study.summary["known_evaluation"]
        if fault == "saved_metric": rows[split]["candidate"]["false_negative_pixels"]["traffic_sign"] += 1
        elif fault == "wrong_pred_sha": rows[split]["candidate"]["prediction_file_sha256"] = "0" * 64
        elif fault == "wrong_weight": rows[split]["candidate"]["weights_sha256"] = "0" * 64
        study.proof["known_evaluation_sha256"] = document(p.RUN / "known_evaluation.json", rows)
    reseal(study)
    with pytest.raises(Exception): p.audit(study.path, study.sha)


def test_validate_create_only_synthetic_pngs_readable_regressions_sources(study, monkeypatch):
    old = {key: getattr(p.previous, key) for key in ("OUTPUT", "RUN", "audit", "canvas", "markdown", "publish")}
    args = SimpleNamespace(caller_pins=str(study.path), caller_pins_sha256=study.sha, validate_only=True)
    assert p.publish(args)["published"] is False and not p.OUTPUT.exists()
    monkeypatch.setattr(p.previous.shutil, "disk_usage", lambda *args: SimpleNamespace(free=10 * 1024**3))
    args.validate_only = False; result = p.publish(args)
    assert result["published"] and result["png_count"] == 5
    report = json.loads((p.OUTPUT / "results.json").read_bytes())
    text = (p.OUTPUT / "README.md").read_text()
    assert "23/27" in text and "4개 미달" in text and "재현율−대조군" in text and "FP−대조군" in text
    assert "0/9" in text and "0/30" in text and "실제 Autoware/RViz 화면이 아닌" in text
    assert all(name in text for name in p.FAILED)
    for name, sha in report["files"].items():
        assert p.frozen.contract.digest(p.OUTPUT / name) == sha
        with Image.open(p.OUTPUT / name) as im: assert im.size == (96, 212)
    assert report["original_sources_and_inputs_unchanged"]
    assert all(p.frozen.contract.digest(p.OUTPUT / name) == sha for name, sha in report["publication_source_archive"].items())
    with pytest.raises(Exception): p.publish(args)
    assert old == {key: getattr(p.previous, key) for key in old}


def test_public_fixed_native350_exact10_sources52_root_pin():
    assert p.INDICES == {"dev_town01": [0, 29, 58, 88, 117], "holdout_town02_p02": [0, 57, 115, 174, 231]}
    assert sum(len(v) for v in p.INDICES.values()) == 10 and p.SHAPE == (360, 640)
    assert [v[0] for v in p.SPLITS.values()] == [118, 232]
    assert len(p.SOURCE) == 15 and len(p.required_run_files()) == 52
    assert not any(name.endswith(".pt") for name in p.required_run_files())
    assert p.COMPLETION_SHA == "84452c157c221964545a1a9753adf9ca8f74daf098351308661d4c9cd0b1ab17"
    assert p.RUN.name == "map_native_fusion_pair_v3" and p.OUTPUT.name == "25_map_native_fusion_v3_results"
    assert len(p.publication_sources()) == 6


def test_cli_only_dispatch_and_storage_floor(study, monkeypatch, capsys):
    monkeypatch.setattr(p.previous.shutil, "disk_usage", lambda *args: SimpleNamespace(free=5 * 1024**3))
    with pytest.raises(Exception): p.publish(SimpleNamespace(caller_pins=str(study.path), caller_pins_sha256=study.sha, validate_only=False))
    assert not p.OUTPUT.exists()
    seen = []; monkeypatch.setattr(p, "publish", lambda args: seen.append(vars(args)) or dict(published=False))
    assert p.main(["--caller-pins", str(study.path), "--caller-pins-sha256", study.sha, "--validate-only"]) == 0
    assert seen[0]["validate_only"] and json.loads(capsys.readouterr().out) == {"published": False}



@pytest.mark.parametrize("fault", ["missing", "wrong_sha", "field_in_training"])
def test_actual_writer_metadata_separation_and_exact_file_pin(study, monkeypatch, fault):
    value = study.summary["arms_result"]["candidate"]
    if fault == "missing": value.pop("metadata_sha256")
    elif fault == "wrong_sha": value["metadata_sha256"] = "0" * 64
    else:
        path = p.RUN / "candidate/training.json"
        row = json.loads(path.read_bytes()); row["metadata_sha256"] = value["metadata_sha256"]
        value["metadata_sha256"] = document(path, row)
    fit = json.loads((p.RUN / "fit_complete.json").read_bytes())
    fit["arms"] = study.summary["arms_result"]
    study.proof["fit_complete_sha256"] = document(p.RUN / "fit_complete.json", fit)
    reseal(study)
    monkeypatch.setattr(p.frozen.Pins, "npz", lambda *args: pytest.fail("incorrect endpoint metadata must fail before pixels"))
    with pytest.raises(Exception): p.audit(study.path, study.sha)


def test_actual_writer_positive_and_no_frozen_v1_change(study):
    for arm, value in study.summary["arms_result"].items():
        path = p.RUN / arm / "training.json"
        assert value["metadata_sha256"] == p.frozen.contract.digest(path)
        stored = json.loads(path.read_bytes())
        assert "metadata_sha256" not in stored
        assert stored == {k: v for k, v in value.items() if k != "metadata_sha256"}
    p.audit(study.path, study.sha)
    original = Path(p.__file__).with_name("publish_map_native_fusion_v3_results_20261002.py")
    assert p.frozen.contract.digest(original) == "bcd5fcca8c19c2db1ca571418452c241e200b8d9ddf0d6e6e1bd28d5bcabe044"
    oldtests = Path(__file__).with_name("test_publish_map_native_fusion_v3_results_20261002.py")
    assert p.frozen.contract.digest(oldtests) == "7bd11f3342f5a5110a1e7b777df594d5c67c68c23bb621030693105d414151ee"


def test_whole_source_inverse_only_receipt_abi_and_revision_provenance():
    old = Path(p.__file__).with_name("publish_map_native_fusion_v3_results_20261002.py").read_text()
    new = Path(p.__file__).read_text()
    new = new.replace('"""HH_261002 - Publish sealed MAP V3 stored predictions with the actual numeric training-receipt ABI."""',
        '"""HH_261002 - Publish ROOT-sealed fixed20k MAP V3 stored predictions, never fit, infer, or select frames."""')
    new = new.replace('SCHEMA = "portable_e2e.map_native_fusion_v3_results.20261002.v2"',
        'SCHEMA = "portable_e2e.map_native_fusion_v3_results.20261002.v1"')
    new = new.replace('    # HH_261002 - Frozen writer adds metadata_sha256 only to summary arms; bind its exact file then compare the stored info without that field.\n', '')
    new = new.replace('and value.get("metadata_sha256") == ref["files"][arm + "/training.json"]\n            and pins.json(RUN / arm / "training.json", value["metadata_sha256"]) == {k: v for k, v in value.items() if k != "metadata_sha256"},',
        'and pins.json(RUN / arm / "training.json", ref["files"][arm + "/training.json"]) == value,')
    assert new == old
