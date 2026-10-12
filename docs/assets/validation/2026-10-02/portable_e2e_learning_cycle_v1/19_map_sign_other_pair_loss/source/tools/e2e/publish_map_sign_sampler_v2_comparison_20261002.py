#!/usr/bin/env python3
"""HH_261002 - Compare caller-pinned stored map predictions; no Torch, inference, or checkpoint selection."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from . import publish_map_paired_balanced_results_20261002 as frozen

ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / "docs/assets/validation/2026-10-02/portable_e2e_learning_cycle_v1/11_map_sign_sampler_comparison"
RUNS = {version: ROOT / ("storage/experiments/training/2026-10-02/map_hardnegative_continuation_" + version)
        for version in ("v1", "v2")}
EVAL = frozen.EVAL
SPLITS = frozen.SPLITS
INDICES = {"dev_town01": [0, 29, 58, 88, 117], "holdout_town02_p02": [0, 57, 115, 174, 231]}
SHAPE = (360, 640)
SCHEMA = "portable_e2e.map_sign_sampler_comparison.20261002.v1"
CALLER_SCHEMA = SCHEMA + ".caller_pins"
INITIAL_SHA = "2284511d2b28c273ab731fcbc7b1326c8e040f4c1d84d57faacf2df86afd8310"
DATA_SHA = "614840471e938a0e9ecd169368d6240e96c01fdf76c74b56f82863c9d49d2b1b"
REGISTRY_SHA = "ca892a6b6e34602b2c983fdec83972a3693bfd763baafb4abfac2f1a488bad8d"
TRAIN_SOURCE = {
    "map_appearance_matched_contract_20261001.py": "9fb7b057a09d955a8952699f3a5868a8f6a6d04cdf78ea66ada7eea742c53b96",
    "map_context_recovery.py": "c8e2a8f0026852cc2b68ff653f78ed4666a1d2d2bad81997d1f5c16798fbc50d",
    "map_perception.py": "1b0076c8f9d34192ab5aedf6cdf96ce2c14458e2bb49986685a6811376b0750b",
    "map_paired_balanced_contract_20261002.py": "9ceab19d111c56bda27b53cdc4bb4e39b42380d0ade7e3eb2dc618d12a028d69",
    "map_train_hardnegative_inventory_sampler_20261002.py": "bb9f93f047cb3ffc46f60431e4c3c63a93de620e09ddbca2f1bb7900ad4fe0eb",
    "train_map_hardnegative_continuation_20261002.py": "0b6261a3bcf474cf7b02f81b6856d6cfd7a0c64b948b3f9d8c48769a24ca5dd3",
    "train_map_paired_balanced_20261002.py": "3773493fd141afdb50dbf467e9d8622fd74a4a4474c83266fa99a1c72352e5be",
    "train_map_photometric_pilot.py": "4de567323d3964ec877da831ec2acb08e445fe77c9da659f052edcf52ca53484",
}
V2_SOURCE = {**TRAIN_SOURCE, "train_map_hardnegative_continuation_v2_20261002.py":
             "0af1e282e0f117b7e1fa53a694fc4f62cf5f6650e9cb757f4b3d18aba53a10c8"}
FONT = Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")
FROZEN_PUBLISHER_SHA = "33c717ddf51c1dc6aa2e56c15c8e131d5866983c97fc19b0421a5c0bf9ba1808"
LABELS, COLORS = frozen.contract.LABELS, frozen.COLORS
require = frozen.require


def publication_sources():
    # HH_261002 - Enumerate the entire local imported publication closure; never import training or Torch modules.
    return {str(Path(module.__file__).resolve().relative_to(ROOT)): Path(module.__file__).resolve()
            for module in (frozen, frozen.contract, frozen.contract.frozen)} | {
                str(Path(__file__).resolve().relative_to(ROOT)): Path(__file__).resolve()}


def local_training_source(name):
    return ROOT / ("learning/portable_e2e" if name in ("map_context_recovery.py", "map_perception.py") else "tools/e2e") / name


def relative_name(name):
    require(type(name) is str and name and Path(name).as_posix() == name and not Path(name).is_absolute()
            and ".." not in Path(name).parts and "." not in Path(name).parts, "canonical relative artifact name required")
    return name


def inventory(root):
    frozen.contract.canonical(root)
    require(root.is_dir(), "completed mirrored run directory required")
    paths = list(root.rglob("*"))
    require(not any(p.is_symlink() for p in paths), "symlink archive member forbidden")
    return {str(p.relative_to(root)) for p in paths if p.is_file()}


def required_run_files(version):
    # HH_261002 - Complete43/47 mirrored artifacts include hashed-only pickle files; no checkpoint is loaded.
    sources = TRAIN_SOURCE if version == "v1" else V2_SOURCE
    result = {"authorization.json", "started.json", "summary.json", "fit_complete.json", "known_evaluation.json", "initial_model_weights.npz"}
    result |= {arm + "/" + name for arm in ("control", "candidate") for name in
        ("training.json", "weights.npz", "checkpoint.pt", "history.jsonl", "exposures.jsonl", *(s + "_predictions.npz" for s in SPLITS))}
    result |= {"parent_diagnostic/" + s + "_predictions.npz" for s in SPLITS}
    result |= {"source/" + s for s in sources}
    result |= {"inputs/" + s for s in ("manifest.json", "study_train_registry.json", "paired_samples.json")}
    result |= {"mining/" + s for s in ("inventory.json", "started.json", "authorization.json", "records.jsonl", "parent_TRAIN_predictions.npz", "proposed_schedule.npz")}
    result |= {"initial_lineage/" + s for s in ("control/training.json", "fit_complete.json", "summary.json", "source/bundle.json")}
    if version == "v2": result |= {"new_declared_schedule.npz", "new_declared_schedule_proof.json", "new_schedule_archive_receipt.json"}
    return result


def confusion_metrics(target, prediction):
    result = frozen.metrics(target, prediction)
    matrix = np.asarray(result["confusion"], dtype=np.int64)
    actual, predicted, diagonal = matrix.sum(1), matrix.sum(0), np.diag(matrix)
    result.update(true_positive_pixels=dict(zip(LABELS, diagonal.tolist())),
        precision={k: float(diagonal[i] / predicted[i]) if predicted[i] else None for i, k in enumerate(LABELS)},
        recall={k: float(diagonal[i] / actual[i]) if actual[i] else None for i, k in enumerate(LABELS)},
        other_to_sign_false_positive_pixels=int(matrix[0, 5]))
    return result


def diagnostic_screen(evaluation):
    # HH_261002 - Preserve all twenty frozen exploratory predicates; no selection or threshold relaxation.
    checks = {}
    candidate = evaluation["dev_town01"]["candidate"]
    for comparator in ("parent", "control"):
        reference = evaluation["dev_town01"][comparator]
        for key, allowance in (("precision", 0.), ("recall", -.02)):
            a, b = reference[key]["traffic_sign"], candidate[key]["traffic_sign"]
            checks["Town01_sign_" + key + "_vs_" + comparator] = a is not None and b is not None and (b > a if key == "precision" else b >= a + allowance)
        checks["Town01_other_to_sign_FP_reduction_vs_" + comparator] = candidate["other_to_sign_false_positive_pixels"] < reference["other_to_sign_false_positive_pixels"]
        for split, cohort in evaluation.items():
            for label in ("road", "lane_marking", "traffic_light_fixture"):
                a, b = cohort[comparator]["iou"][label], cohort["candidate"]["iou"][label]
                checks[split + "_" + label + "_retention_vs_" + comparator] = a is not None and b is not None and b >= a - .01
        cohort = evaluation["holdout_town02_p02"]
        a, b = cohort[comparator]["iou"]["traffic_sign"], cohort["candidate"]["iou"]["traffic_sign"]
        checks["Town02_sign_IoU_retention_vs_" + comparator] = a is not None and b is not None and b >= a - .01
    require(len(checks) == 20, "all twenty exploratory diagnostics required")
    return dict(checks=checks, passed_exploratory_only=all(checks.values()), independent_validation=False,
                model_promoted=False, vehicle_control_approved=False)


def numeric_model(arrays, reference):
    frozen.numeric_weights(arrays, reference)
    return all(np.array_equal(arrays[name], reference[name]) for name in arrays)


def exposure_proof(payload, schedule, arm, info):
    rows = payload.splitlines()
    require(len(rows) == 4000, "every actual4000 optimizer exposure required")
    digests = [hashlib.sha256() for _ in range(3)]
    for step, line in enumerate(rows):
        row = frozen.strict_json(line)
        values = (schedule[arm + "_indices"][step], schedule["shared_crops"][step], schedule["shared_FALSE_view_bits"][step])
        require(set(row) == {"step", "physical_slot_indices", "shared_crop_boxes", "actual_paired_TRUE_bits"}
                and type(row["step"]) is int and row["step"] == step + 1
                and type(row["physical_slot_indices"]) is list and len(row["physical_slot_indices"]) == 8
                and all(type(v) is int for v in row["physical_slot_indices"])
                and type(row["actual_paired_TRUE_bits"]) is list and len(row["actual_paired_TRUE_bits"]) == 8
                and all(type(v) is int for v in row["actual_paired_TRUE_bits"])
                and type(row["shared_crop_boxes"]) is list and len(row["shared_crop_boxes"]) == 8
                and all(type(box) is list and len(box) == 4 and all(type(v) is int for v in box) for box in row["shared_crop_boxes"])
                and row["physical_slot_indices"] == values[0].tolist()
                and row["shared_crop_boxes"] == values[1].tolist()
                and row["actual_paired_TRUE_bits"] == values[2].tolist(), "complete exposure/crop/FALSE chronology differs")
        for digest, value in zip(digests, values): digest.update(value.tobytes())
    for key, digest in zip(("sample_sequence_sha256", "crop_sequence_sha256", "view_sequence_sha256"), digests):
        require(info[key] == digest.hexdigest(), "observed full exposure hash differs")
    counts = np.bincount(schedule[arm + "_indices"].ravel(), minlength=1209).tolist()
    require(info["physical_slot_exposures"] == counts and all(v > 0 for v in counts), "all1209 TRAIN slots must be retained")
    return dict(steps=4000, examples_seen=32000, every_actual_exposure_verified=True,
                physical_slot_count=1209, all_slots_exposed=True)


def validate_schedule(schedule, proof):
    abi = {"control_indices": ((4000, 8), np.int64), "candidate_indices": ((4000, 8), np.int64),
           "shared_crops": ((4000, 8, 4), np.int16), "shared_FALSE_view_bits": ((4000, 8), np.uint8)}
    require(set(schedule) == set(abi), "exact four-array schedule required")
    for key, (shape, dtype) in abi.items():
        value = schedule[key]
        require(value.shape == shape and value.dtype == dtype and proof["tensor_sha256"][key] == hashlib.sha256(value.tobytes()).hexdigest(), "schedule ABI/hash differs")
    require(bool(((schedule["control_indices"] >= 0) & (schedule["control_indices"] < 1209)).all())
            and bool(((schedule["candidate_indices"] >= 0) & (schedule["candidate_indices"] < 1209)).all())
            and not bool(schedule["shared_FALSE_view_bits"].any()), "actual TRAIN indices/FALSE-only scope differs")
    boxes = schedule["shared_crops"]
    require(bool(((boxes[..., 0] >= 0) & (boxes[..., 1] >= 0) & (boxes[..., 2] > 0) & (boxes[..., 3] > 0)
        & (boxes[..., 0] + boxes[..., 2] <= 360) & (boxes[..., 1] + boxes[..., 3] <= 640)).all()), "native shared crop geometry differs")


def read_study(version, ref, pins):
    root = frozen.contract.canonical(Path(ref["root"]))
    require(root == RUNS[version] and set(ref["files"]) == inventory(root) == required_run_files(version), "external complete mirrored inventory differs")
    for name, sha in ref["files"].items(): pins.file(root / relative_name(name), sha)
    summary = pins.json(root / "summary.json", ref["files"]["summary.json"])
    expected_source = TRAIN_SOURCE if version == "v1" else V2_SOURCE
    require(summary["schema"] == "portable_e2e.map_hardnegative_continuation.20261002." + version
            and summary["study_id"] == "map_hardnegative_continuation_20261002_" + version
            and summary["status"] == "FIXED_CURRENT_PARENT_SAMPLER_AB_COMPLETE_DIAGNOSTIC_NOT_PROMOTED"
            and summary["source_sha256"] == expected_source, "exact completed source8/source9 study required")
    for name, sha in expected_source.items():
        pins.file(root / "source" / name, sha); pins.file(local_training_source(name), sha)
    for key, wanted in dict(initial_model_weights_sha256=INITIAL_SHA, data_manifest_sha256=DATA_SHA,
        data_registry_sha256=REGISTRY_SHA, additional_updates_per_arm=4000, examples_per_arm=32000,
        initial_completed_updates=10000, total_lineage_updates_per_arm=14000, physical_TRAIN_slots=1209,
        parameter_count=245831, batch=8, seed=2026100203, learning_rate=3e-5, weight_decay=1e-4,
        gradient_norm_cap=5., stratum_frame_counts=dict(small_sign=391, large_sign=193, hard_negative=145)).items():
        require(type(summary.get(key)) is type(wanted) and summary[key] == wanted, "same initial228/fixed4000 contract differs: " + key)
    for key, wanted in dict(source_and_inputs_unchanged=True, model_promoted=False, vehicle_control_approved=False,
        known_evaluation_not_independent=True, evaluation_opens_only_after_both_final4000=True,
        validation_used_during_optimization=False, independent_new_dev_sites=0, labels_changed=False,
        architecture_unchanged=True, semantic_loss_unchanged=True, native_denials_preserved=True,
        training_authorized=True, main_capabilities_approved=0, subfunctions_approved=0).items():
        require(type(summary.get(key)) is type(wanted) and summary[key] == wanted, "truthful diagnostic/postcheck scope differs: " + key)
    authorization = pins.json(root / "authorization.json", summary["authorization_sha256"])
    require(authorization["schema"] == summary["schema"] + ".authorization" and authorization["source_sha256"] == expected_source
        and authorization["research_fit_authorized"] is True and authorization["native_denials_preserved"] is True
        and authorization["DEV_or_HOLDOUT_used_for_fit"] is False and authorization["automatic_model_promotion"] is False
        and authorization["vehicle_control_approved"] is False,
        "actual separately authenticated fit authorization required")
    require(set(summary["mining_artifact_sha256"]) == {n.removeprefix("mining/") for n in required_run_files(version) if n.startswith("mining/")}
        and set(summary["initial_model_receipt_sha256"]) == {n.removeprefix("initial_lineage/") for n in required_run_files(version) if n.startswith("initial_lineage/")},
        "complete original mining and initial lineage receipt closure required")
    for name, sha in summary["mining_artifact_sha256"].items(): pins.file(root / "mining" / relative_name(name), sha)
    for name, sha in summary["initial_model_receipt_sha256"].items(): pins.file(root / "initial_lineage" / relative_name(name), sha)
    pins.file(root / "inputs/manifest.json", DATA_SHA); pins.file(root / "inputs/study_train_registry.json", REGISTRY_SHA)
    initial = pins.npz(root / "initial_model_weights.npz", INITIAL_SHA)
    frozen.numeric_weights(initial, initial)
    info = summary["arms_result"]
    require(set(info) == {"control", "candidate"}, "both fixed endpoints required")
    complete = pins.json(root / "fit_complete.json", ref["files"]["fit_complete.json"])
    require(complete["status"] == "BOTH_FIXED_FINAL4000_ENDPOINTS_SAVED_BEFORE_KNOWN_EVALUATION" and complete["arms"] == info,
        "both fixed final4000 before evaluation receipt differs")
    schedule_path = "mining/proposed_schedule.npz" if version == "v1" else "new_declared_schedule.npz"
    schedule = pins.npz(root / schedule_path, ref["files"][schedule_path])
    proof = summary["sampler_proof"]; validate_schedule(schedule, proof)
    require(proof["schema"] == "portable_e2e.map_train_hardnegative_sampler.20261002." + version
        and proof["steps_per_arm"] == 4000 and proof["batch"] == 8 and proof["examples_per_arm"] == 32000
        and proof["candidate_slot_roles"] == (["all_TRAIN"] * 4 + ["small_sign", "large_sign"] + ["hard_negative"] * 2
            if version == "v1" else ["all_TRAIN"] * 4 + ["small_sign"] * 2 + ["large_sign", "hard_negative"]),
        "prospectively declared exact sampler quota required")
    if version == "v2":
        receipt = pins.json(root / "new_schedule_archive_receipt.json", ref["files"]["new_schedule_archive_receipt.json"])
        stored = pins.json(root / "new_declared_schedule_proof.json", receipt["new_schedule_proof_file_sha256"])
        proof_sha = hashlib.sha256(json.dumps(proof, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()
        require(stored == proof and authorization["new_sampler_proof_sha256"] == receipt["new_sampler_proof_sha256"] == proof_sha
            and receipt["new_schedule_npz_sha256"] == ref["files"][schedule_path]
            and proof["candidate_slot_roles"] == ["all_TRAIN"] * 4 + ["small_sign"] * 2 + ["large_sign", "hard_negative"],
            "separate externally committed balanced sampler proof differs")
    models, exposures = {}, {}
    for arm, value in info.items():
        require(value["steps"] == 4000 and value["examples_seen"] == 32000 and value["parameter_count"] == 245831
                and value["initial_model_weights_sha256"] == INITIAL_SHA and value["optimizer_initial_state_entries"] == 0
                and value["initial_state_sha256"] == info["control"]["initial_state_sha256"], "same warmstart/fixed4000 endpoint differs")
        require(pins.json(root / arm / "training.json", ref["files"][arm + "/training.json"]) == value, "complete endpoint training receipt differs")
        require(ref["files"][arm + "/checkpoint.pt"] == value["checkpoint_sha256"], "hashed-only original checkpoint identity differs")
        models[arm] = pins.npz(root / arm / "weights.npz", value["weights_sha256"])
        frozen.numeric_weights(models[arm], initial)
        exposures[arm] = exposure_proof(pins.payload(root / arm / "exposures.jsonl", value["full_exposure_journal_sha256"]), schedule, arm, value)
        history = [frozen.strict_json(row) for row in pins.payload(root / arm / "history.jsonl", ref["files"][arm + "/history.jsonl"]).splitlines()]
        require(len(history) == 40 and all(row.get("arm") == arm and type(row.get("step")) is int and row["step"] == (i + 1) * 100
            and frozen.finite_number(row.get("mean_loss_100")) and frozen.finite_number(row.get("wall_seconds"))
            and (not i or row["wall_seconds"] >= history[i - 1]["wall_seconds"]) for i, row in enumerate(history)), "all forty actual training history rows required")
    require(pins.json(root / "known_evaluation.json", ref["files"]["known_evaluation.json"]) == summary["known_evaluation"], "complete evaluation/summary linkage differs")
    return dict(root=root, summary=summary, models=models, exposures=exposures, schedule=schedule)


def read_prediction(pins, study, ref, split, arm, target):
    relative = ("parent_diagnostic" if arm == "parent" else arm) + "/" + split + "_predictions.npz"
    arrays = pins.npz(study["root"] / relative, ref["files"][relative])
    require(set(arrays) == {"prediction"}, "stored prediction keys differ")
    prediction = arrays["prediction"]
    require(prediction.shape == target.shape and prediction.dtype == np.uint8 and int(prediction.max()) < 7, "full-FOV stored prediction ABI differs")
    metrics = confusion_metrics(target, prediction)
    row = study["summary"]["known_evaluation"][split][arm]
    frozen.reported_metrics(row, metrics, SPLITS[split][0], SPLITS[split][1])
    for key in ("true_positive_pixels", "false_negative_pixels", "precision", "recall", "other_to_sign_false_positive_pixels"):
        require(row[key] == metrics[key], "independent all350 derived metric differs: " + key)
    expected_weight = INITIAL_SHA if arm == "parent" else study["summary"]["arms_result"][arm]["weights_sha256"]
    require(row["weights_sha256"] == expected_weight, "prediction endpoint identity differs")
    return prediction, row


def audit(caller_path, caller_sha):
    # HH_261002 - An imported model process or a changed reused authenticator cannot claim inference-free publication.
    require("torch" not in sys.modules and frozen.contract.digest(Path(frozen.__file__).resolve()) == FROZEN_PUBLISHER_SHA,
        "unchanged frozen publication provider and no Torch process required")
    pins = frozen.Pins(); caller = pins.json(caller_path, caller_sha)
    require(caller.get("schema") == CALLER_SCHEMA and caller.get("output") == str(OUTPUT)
        and caller.get("fixed_indices") == INDICES and set(caller.get("runs", {})) == set(RUNS), "prospective exact comparison caller required")
    require(caller.get("publication_source_sha256", {}).keys() == publication_sources().keys(), "complete local publication source closure required")
    for name, path in publication_sources().items(): pins.file(path, caller["publication_source_sha256"][name])
    require(caller["font"]["path"] == str(FONT), "fixed readable font required")
    pins.file(FONT, caller["font"]["sha256"])
    studies = {v: read_study(v, ref, pins) for v, ref in caller["runs"].items()}
    a, b = studies["v1"], studies["v2"]
    for key in ("class_pixels", "class_weights", "optimizer", "shared_crop_law", "RGB_views", "final_selection"):
        require(a["summary"][key] == b["summary"][key], "sampler-only comparison changed another factor: " + key)
    schedule_parity = {k: np.array_equal(a["schedule"][k], b["schedule"][k]) for k in ("control_indices", "shared_crops", "shared_FALSE_view_bits")}
    require(all(schedule_parity.values()), "control/crop/view setup is not the same")
    control_parity = numeric_model(a["models"]["control"], b["models"]["control"])
    evaluation = caller["evaluation"]
    require(evaluation["root"] == str(EVAL) and evaluation["files"] == {name + ".npz": sha for name, (_, sha) in SPLITS.items()}, "exact known350 evaluation references required")
    manifest = pins.json(EVAL / "manifest.json", frozen.EVAL_MANIFEST_SHA)
    all_metrics, panels, prediction_parity = {"v1": {}, "v2": {}}, [], {}
    for split, (count, sha) in SPLITS.items():
        ref = manifest["files"][split]
        require(all(ref.get(k) == v for k, v in dict(file=split + ".npz", count=count, sha256=sha).items()), "known evaluation manifest identity differs")
        arrays = pins.npz(EVAL / (split + ".npz"), sha)
        require(set(arrays) == {"rgb", "target"}, "RGB/GT archive keys differ")
        rgb, target = arrays["rgb"], arrays["target"]
        require(rgb.shape == (count, *SHAPE, 3) and target.shape == (count, *SHAPE)
                and rgb.dtype == target.dtype == np.uint8 and int(target.max()) < 7, "actual native360x640 full evaluation FOV required")
        predictions = {}
        for version, study in studies.items():
            all_metrics[version][split] = {}; predictions[version] = {}
            for arm in ("parent", "control", "candidate"):
                pred, row = read_prediction(pins, study, caller["runs"][version], split, arm, target)
                predictions[version][arm] = pred; all_metrics[version][split][arm] = row
        prediction_parity[split] = {arm: bool(np.array_equal(predictions["v1"][arm], predictions["v2"][arm])) for arm in ("parent", "control")}
        for index in INDICES[split]:
            panels.append((split, index, rgb[index].copy(), target[index].copy(), predictions["v1"]["parent"][index].copy(),
                predictions["v1"]["control"][index].copy(), predictions["v1"]["candidate"][index].copy(), predictions["v2"]["candidate"][index].copy()))
    screens = {v: diagnostic_screen(rows) for v, rows in all_metrics.items()}
    require(all(screens[v] == studies[v]["summary"]["exploratory_screen_result"] for v in studies), "all twenty saved diagnostic predicates differ")
    pins.recheck()
    require(all(inventory(RUNS[v]) == set(caller["runs"][v]["files"]) for v in RUNS), "complete mirrored inventory changed during audit")
    report = dict(schema=SCHEMA, status="FULL350_STORED_PREDICTIONS_VERIFIED_EXPLORATORY_NOT_APPROVED", evaluation=all_metrics,
        exploratory_screens=screens, fixed_indices=INDICES, native_FOV_hw=list(SHAPE), full_cohort_counts={k: n for k, (n, _) in SPLITS.items()},
        source_counts={v: len(study["summary"]["source_sha256"]) for v, study in studies.items()},
        actual_exposures={v: s["exposures"] for v, s in studies.items()}, control_schedule_parity=schedule_parity,
        V1_V2_control_numeric_exact_parity=control_parity, V1_V2_stored_prediction_exact_parity=prediction_parity,
        panel_control="V1 control; V2 control retained in all350 metrics and exact numeric/prediction parity fields",
        no_control_parity_assumed=True, first_read_source_and_artifact_pins=dict(pins.values), caller_sha256=caller_sha,
        mirrored_run_inventory={v: sorted(caller["runs"][v]["files"]) for v in RUNS},
        known_exposed_diagnostic_not_independent=True, hypothesis_adapted_after_V1=True, independent_new_dev_sites=0,
        failed_diagnostic_checks_retained=True, model_inference_executed=False, Torch_imported=False, pickle_loaded=False,
        raw_TRAIN_reopened=False, frozen_fit_postcheck_receipts_authenticated=True, RGB_only_seven_class_perception_not_HD_AMG=True,
        traffic_light_fixture_not_signal_state=True, GT_is_not_a_model_map=True, model_promoted=False,
        vehicle_control_approved=False, formal_main_functions_approved=0, formal_subfunctions_approved=0)
    return report, panels, pins


def canvas(panel, font):
    split, index, rgb, *maps = panel
    height, width = SHAPE
    require(rgb.shape == (*SHAPE, 3) and rgb.dtype == np.uint8 and len(maps) == 5, "full native RGB/five semantic panels required")
    require(all(v.shape == SHAPE and v.dtype == np.uint8 and int(v.max()) < 7 for v in maps), "unmodified seven-class panel required")
    out = Image.new("RGB", (3 * width, 2 * height + 180), (18, 24, 35)); draw = ImageDraw.Draw(out)
    draw.text((12, 8), f"{split} | fixed index {index} | actual saved predictions | known-site diagnostic, NOT model approval", font=font, fill="white")
    titles = ["Original RGB / full FOV, no crop", "Ground truth / not model input", "Parent228 / before additional updates",
        "V1 control / global-only4000", "V1 candidate / negative-heavy4000", "V2 candidate / small-sign-balanced4000"]
    pixels = [rgb, *(COLORS[v] for v in maps)]
    for i, (title, image) in enumerate(zip(titles, pixels)):
        x, y = (i % 3) * width, 44 + (i // 3) * (height + 40)
        draw.text((x + 10, y), title, font=font, fill="white"); out.paste(Image.fromarray(image), (x, y + 30))
    for i, (label, color) in enumerate(zip(LABELS, COLORS)):
        x = 12 + i * 272; y = 2 * height + 133
        draw.rectangle((x, y, x + 12, y + 12), fill=tuple(int(c) for c in color)); draw.text((x + 17, y - 4), label, font=font, fill="white")
    draw.text((12, 2 * height + 155), "7 RGB classes only; light fixture is NOT signal state; GT/predictions are NOT HD map topology; main0/9 sub0/30.", font=font, fill="white")
    return out


def markdown(report, files):
    pct = lambda v: "미측정" if v is None else f"{100 * v:.2f}%"
    lines = ["# 표지 누락·오탐 복습 비율 비교", "", "RGB·정답·부모228·일반 복습 대조군·배경 중심 V1 후보·작은 표지 균형 V2 후보를 같은 화면으로 비교했습니다.", "",
        "Town01 118장·Town02 232장 **전체 350장**으로 모든 분류와 실패를 검산했습니다. 아래 10장은 V2 평가 전에 정한 고정 인덱스이며 좋은 표지나 성공 화면을 골라낸 것이 아닙니다.", "",
        "두 비교는 같은 부모에서 각각 4,000회·32,000개 노출입니다. 7분류 영상 인식의 기존 장소 진단이며, 신호 상태·좌표 누적·차선 연결을 갖춘 정밀지도 완료가 아닙니다. 신규 독립 장소 0개, 메인 0/9·하위 0/30 및 자동 채택 금지를 유지합니다.", "",
        f"V1/V2 대조군 전체 숫자 가중치 정확 일치: **{report['V1_V2_control_numeric_exact_parity']}**. 대조군 화면은 V1이며 V2 대조군의 전체 수치와 저장 예측 일치 여부도 results.json에 보존합니다. 불일치를 숨기거나 동일하다고 가정하지 않습니다.", ""]
    for version, result in report["exploratory_screens"].items():
        failed = [k for k, v in result["checks"].items() if not v]
        lines += [f"- {version}: {20 - len(failed)}/20 탐색 진단, {'PASS(기능 승인 아님)' if not failed else 'FAIL / 미채택'}. 미달: {', '.join(failed) or '없음'}"]
    for split in SPLITS:
        lines += ["", f"## {split}: 전체 분류", "", "| 분류 | 부모 IoU | V1 대조군 | V1 후보 | V2 대조군 | V2 후보 |", "|---|---:|---:|---:|---:|---:|"]
        for label in LABELS:
            values = [report["evaluation"][v][split][a]["iou"][label] for v, a in (("v1", "parent"), ("v1", "control"), ("v1", "candidate"), ("v2", "control"), ("v2", "candidate"))]
            lines.append("| " + label + " | " + " | ".join(pct(v) for v in values) + " |")
        lines += ["", "| 표지 지표 | 부모 | V1 대조군 | V1 후보 | V2 대조군 | V2 후보 |", "|---|---:|---:|---:|---:|---:|"]
        rows = [report["evaluation"][v][split][a] for v, a in (("v1", "parent"), ("v1", "control"), ("v1", "candidate"), ("v2", "control"), ("v2", "candidate"))]
        for title, values in (("정밀도(오탐과 함께 읽기)", [pct(r["precision"]["traffic_sign"]) for r in rows]),
            ("재현율(누락)", [pct(r["recall"]["traffic_sign"]) for r in rows]),
            ("other→표지 오탐 픽셀", [str(r["other_to_sign_false_positive_pixels"]) for r in rows])):
            lines.append("| " + title + " | " + " | ".join(values) + " |")
    lines += ["", "## 평가 전 고정한 10개 원본 화면", "", "모든 패널은 실제 360×640 전체 화각입니다. 위: RGB·GT·부모. 아래: V1 대조군·V1 후보·V2 후보. GT는 모델이 만든 지도가 아닙니다.", ""]
    for name in files: lines += [f"![{name}]({name})", ""]
    lines += ["전체 49칸 confusion·7분류 FN/FP·정밀도·재현율·두 모델의 20개 판정과 모든 출처는 [results.json](results.json)에 있습니다. 저장된 예측만 읽었으며 새 모델 추론·pickle·GPU 작업은 실행하지 않았습니다.", ""]
    return "\n".join(lines)


def publish(args):
    frozen.contract.canonical(OUTPUT); require(not OUTPUT.exists(), "fresh fixed comparison output only")
    report, panels, pins = audit(Path(args.caller_pins), args.caller_pins_sha256)
    if args.validate_only: return dict(status=report["status"], published=False, fixed_preview_count=len(panels),
        full_evaluation_count=sum(n for n, _ in SPLITS.values()))
    ancestor = next(p for p in OUTPUT.parents if p.exists())
    require(shutil.disk_usage(ancestor).free >= 5 * 1024**3 + 128 * 1024**2, "5GiB floor plus128MiB image/report reserve required")
    source_bytes = {name: pins.payload(path, pins.values[str(path)]) for name, path in publication_sources().items()}
    pins.recheck(); OUTPUT.mkdir(parents=True)
    font = ImageFont.truetype(str(FONT), 18)
    files = {}
    for panel in panels:
        name = f"{panel[0]}_{panel[1]:04d}.png"
        with (OUTPUT / name).open("xb") as stream: canvas(panel, font).save(stream, format="PNG")
        files[name] = frozen.contract.digest(OUTPUT / name)
    for name, payload in source_bytes.items():
        path = OUTPUT / "source" / name; path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("xb") as stream: stream.write(payload)
        require(frozen.contract.digest(path) == pins.values[str(publication_sources()[name])], "publication source archive bytes differ")
    pins.recheck()
    require(all(inventory(RUNS[v]) == set(report["mirrored_run_inventory"][v]) for v in RUNS), "complete mirrored inventory changed during rendering")
    report.update(created_at_utc=datetime.now(timezone.utc).isoformat(), files=files, original_sources_and_inputs_unchanged=True,
        publication_source_archive={"source/" + k: pins.values[str(v)] for k, v in publication_sources().items()})
    frozen.contract.write_new(OUTPUT / "results.json", report)
    with (OUTPUT / "README.md").open("x") as stream: stream.write(markdown(report, files))
    pins.recheck()
    return dict(status=report["status"], published=True, output=str(OUTPUT), png_count=len(files),
        results_sha256=frozen.contract.digest(OUTPUT / "results.json"), README_sha256=frozen.contract.digest(OUTPUT / "README.md"))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--caller-pins", required=True); parser.add_argument("--caller-pins-sha256", required=True)
    parser.add_argument("--validate-only", action="store_true")
    print(json.dumps(publish(parser.parse_args(argv)), ensure_ascii=False, sort_keys=True), flush=True)
    return 0


if __name__ == "__main__": raise SystemExit(main())
