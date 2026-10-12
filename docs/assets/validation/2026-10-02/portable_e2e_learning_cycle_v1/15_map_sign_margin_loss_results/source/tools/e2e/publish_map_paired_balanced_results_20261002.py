#!/usr/bin/env python3
"""HH_261002 - Audit fixed paired map endpoints and render archived predictions without Torch or inference."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import io
import json
import math
from pathlib import Path
import re
import shutil
import zipfile

import numpy as np
from PIL import Image, ImageDraw

from . import map_paired_balanced_contract_20261002 as contract

ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / "docs/assets/validation/2026-10-02/map_paired_balanced_results_v1"
DATA = ROOT / "storage/experiments/training/2026-10-02" / contract.NAME
EVAL = ROOT / "storage/experiments/training/2026-09-29/map_town02_p00_route_holdout_data_v1"
REMOTE_RUN = "/home/pro6000/personal/hwanhong/portable_e2e/runs/diagnostics/" + contract.NAME
SCHEMA = "portable_e2e.map_paired_balanced_results.20261002.v1"
ARTIFACT_SCHEMA = "portable_e2e.map_paired_balanced_remote_artifact_pins.20261002.v1"
EVAL_MANIFEST_SHA = "f4764b9aa9d015f3d136a145f6194dd60bda8900c27dd2423de6145b9f472834"
SPLITS = {
    "dev_town01": (118, "417c60f924abb044617e2aba5b1afd70b83f5da7f905e54513e9c220172ba4b3"),
    "holdout_town02_p02": (232, "858f92ccd874c154576d3cf6a85132508801a22671baf6d0c07abe3936eb16cb"),
}
SOURCE_SHA = {
    "map_paired_balanced_contract_20261002.py": "9ceab19d111c56bda27b53cdc4bb4e39b42380d0ade7e3eb2dc618d12a028d69",
    "train_map_paired_balanced_20261002.py": "3773493fd141afdb50dbf467e9d8622fd74a4a4474c83266fa99a1c72352e5be",
    **contract.FROZEN_SOURCE_SHA,
}
COLORS = np.asarray(((30, 30, 40), (67, 92, 123), (141, 88, 147), (246, 222, 90),
                     (246, 100, 70), (60, 220, 180), (179, 190, 199)), dtype=np.uint8)
REQUIRED_ARTIFACTS = {"known_evaluation.json", *(
    f"{arm}/{name}" for arm in contract.ARMS for name in
    ("weights.npz", "checkpoint.pt", *(split + "_predictions.npz" for split in SPLITS)))}
require = contract.require


def strict_json(value):
    """HH_261002 - Reject duplicate keys and nonfinite JSON instead of normalizing corrupt evidence."""
    def pairs(items):
        result = {}
        for key, item in items:
            require(key not in result, "duplicate JSON key")
            result[key] = item
        return result
    return json.loads(value, object_pairs_hook=pairs,
                      parse_constant=lambda _: require(False, "nonfinite JSON"))


def sha_string(value):
    require(type(value) is str and re.fullmatch(r"[0-9a-f]{64}", value) is not None,
            "external SHA256 must be exactly64 lowercase hexadecimal characters")
    return value


class Pins:
    """HH_261002 - Bind exactly parsed/read bytes on first read and recheck before any output."""
    def __init__(self): self.values = {}

    def remember(self, path, value):
        path = contract.canonical(Path(path))
        value = sha_string(value)
        key = str(path)
        require(key not in self.values or self.values[key] == value, "first-read pin changed")
        self.values[key] = value
        return path

    def payload(self, path, expected=None):
        path = contract.canonical(Path(path))
        value, digest = contract.payload(path, sha_string(expected) if expected is not None else None)
        self.remember(path, digest)
        return value

    def json(self, path, expected=None): return strict_json(self.payload(path, expected))

    def file(self, path, expected):
        path = contract.canonical(Path(path))
        require(contract.digest(path) == sha_string(expected), "declared artifact bytes differ: " + str(path))
        self.remember(path, expected)

    def npz(self, path, expected):
        value = self.payload(path, expected)
        require(len(value) <= 1024**3, "bounded numeric archive input required")
        with zipfile.ZipFile(io.BytesIO(value)) as archive:
            members = archive.infolist()
            require(len({row.filename for row in members}) == len(members)
                    and sum(row.file_size for row in members) <= 1024**3,
                    "duplicate or oversized numeric ZIP members")
        with np.load(io.BytesIO(value), allow_pickle=False) as archive:
            require(len(set(archive.files)) == len(archive.files), "duplicate NPZ tensors")
            return {name: archive[name].copy() for name in archive.files}

    def recheck(self):
        for filename, value in self.values.items():
            require(contract.digest(Path(filename)) == value, "first-read input changed before publication: " + filename)


def metrics(target, prediction):
    """HH_261002 - Retain all49 confusion cells and absent-target null IoU with its false positives."""
    require(target.shape == prediction.shape and target.dtype == prediction.dtype == np.uint8
            and target.size > 0 and int(target.max()) < 7 and int(prediction.max()) < 7,
            "seven-class prediction/target ABI differs")
    matrix = np.bincount((target.astype(np.int64)*7 + prediction).ravel(), minlength=49).reshape(7, 7)
    actual, predicted, diagonal = matrix.sum(1), matrix.sum(0), np.diag(matrix)
    union = actual + predicted - diagonal
    return {
        "confusion": matrix.tolist(), "target_pixels": dict(zip(contract.LABELS, actual.tolist())),
        "predicted_pixels": dict(zip(contract.LABELS, predicted.tolist())),
        "false_negative_pixels": dict(zip(contract.LABELS, (actual-diagonal).tolist())),
        "false_positive_pixels": dict(zip(contract.LABELS, (predicted-diagonal).tolist())),
        "iou": {label: float(diagonal[i]/union[i]) if actual[i] else None for i, label in enumerate(contract.LABELS)},
        "pixel_accuracy": float(diagonal.sum()/matrix.sum()),
        "absent_target_class_iou": "null; false positives retained",
        "pixel_count": int(matrix.sum()),
    }


def reported_metrics(row, calculated, count, source_sha):
    require(row.get("count") == count and row.get("source_sha256") == source_sha
            and row.get("known_exposed_diagnostic_not_independent") is True,
            "saved evaluation scope/count/source differs")
    matrix = row.get("confusion")
    require(type(matrix) is list and len(matrix) == 7
            and all(type(values) is list and len(values) == 7
                    and all(type(value) is int and value >= 0 for value in values) for values in matrix),
            "all49 typed confusion cells required")
    for key in ("target_pixels", "predicted_pixels", "false_positive_pixels"):
        require(type(row.get(key)) is dict and set(row[key]) == set(contract.LABELS)
                and all(type(value) is int and value >= 0 for value in row[key].values()),
                "all seven typed pixel count fields required")
    for key in ("confusion", "target_pixels", "predicted_pixels", "false_positive_pixels", "iou",
                "pixel_accuracy", "absent_target_class_iou"):
        require(row.get(key) == calculated[key], "saved full-FOV metrics differ: " + key)


def artifact_receipt(value, summary_sha, data_sha, registry_sha, source_sha):
    require(value.get("schema") == ARTIFACT_SCHEMA and value.get("study_id") == contract.NAME
            and value.get("remote_run_root") == REMOTE_RUN
            and value.get("remote_originals_independently_hashed") is True,
            "independent remote-original artifact receipt required")
    for key, sha in (("summary_sha256", summary_sha), ("data_manifest_sha256", data_sha),
                     ("study_registry_sha256", registry_sha), ("source_bundle_sha256", source_sha)):
        require(value.get(key) == sha_string(sha), "caller artifact receipt pointer differs: " + key)
    require(type(value.get("files")) is dict and set(value["files"]) == REQUIRED_ARTIFACTS,
            "both final endpoints/four predictions/known evaluation must be externally pinned")
    for name, value in value["files"].items():
        require(Path(name).as_posix() == name and not Path(name).is_absolute() and ".." not in Path(name).parts,
                "caller artifact path escaped run")
        sha_string(value)


def finite_number(value):
    return type(value) in (int, float) and math.isfinite(value) and value >= 0


def exposure_proof(payload, info, arm, indices, crops, bits):
    """HH_261002 - Verify all10000 actual rows, not summary-only counters or a favorable subset."""
    lines = payload.splitlines()
    require(len(lines) == contract.STEPS, "exact10000 actual exposure rows required")
    digests = [hashlib.sha256() for _ in range(3)]
    off, on = np.zeros(480, np.int64), np.zeros(480, np.int64)
    for step, line in enumerate(lines):
        row = strict_json(line)
        require(set(row) == {"step", "physical_slot_indices", "shared_crop_boxes", "actual_paired_TRUE_bits"}
                and type(row["step"]) is int and row["step"] == step+1, "exposure step/order/schema differs")
        ids, boxes, views = row["physical_slot_indices"], row["shared_crop_boxes"], row["actual_paired_TRUE_bits"]
        require(type(ids) is type(boxes) is type(views) is list and len(ids) == len(boxes) == len(views) == contract.BATCH
                and all(type(i) is int for i in ids + views)
                and all(type(box) is list and len(box) == 4 and all(type(i) is int for i in box) for box in boxes),
                "actual exposure typed batch ABI differs")
        expected_views = bits[step] if arm == "balanced" else np.zeros(contract.BATCH, np.uint8)
        require(ids == indices[step].tolist() and boxes == crops[step].tolist() and views == expected_views.tolist(),
                "actual complete input/crop/view sequence differs")
        for digest, array in zip(digests, (np.asarray(ids, np.int64), np.asarray(boxes, np.int16), np.asarray(views, np.uint8))):
            digest.update(array.tobytes())
        for index, bit in zip(ids, views):
            if index >= 729: (on if bit else off)[index-729] += 1
    for key, digest in zip(("sample_sequence_sha256", "crop_sequence_sha256", "view_sequence_sha256"), digests):
        require(info.get(key) == digest.hexdigest(), "observed exposure sequence SHA differs")
    require(info.get("paired_FALSE_exposures_per_pair") == off.tolist()
            and info.get("paired_TRUE_exposures_per_pair") == on.tolist(), "actual per-pair bit counts differ")
    return {"rows": len(lines), "examples_seen": len(lines)*contract.BATCH,
            "paired_FALSE_exposures": int(off.sum()), "paired_TRUE_exposures": int(on.sum()),
            "all_actual_indices_crops_and_bits_verified": True}


def history_proof(payload, arm):
    rows = [strict_json(line) for line in payload.splitlines()]
    require(len(rows) == 100, "all100 complete mean-loss rows required")
    previous = 0.
    for index, row in enumerate(rows):
        require(row.get("arm") == arm and type(row.get("step")) is int and row["step"] == (index+1)*100
                and finite_number(row.get("mean_loss_100")) and finite_number(row.get("wall_seconds"))
                and row["wall_seconds"] >= previous, "actual training history incomplete/nonfinite")
        previous = row["wall_seconds"]
    return {"row_count": 100, "last_mean_loss_100": rows[-1]["mean_loss_100"], "last_wall_seconds": previous}


def numeric_weights(archive, parent):
    require(set(archive) == set(parent) and bool(parent), "final numeric model learned-tensor keys differ")
    for name, value in archive.items():
        require(value.shape == parent[name].shape and value.dtype == parent[name].dtype == np.float32
                and bool(np.isfinite(value).all()) and bool(np.isfinite(parent[name]).all()), "final numeric model ABI differs: " + name)
    count = sum(value.size for value in parent.values())
    require(count == 245831, "original245831-parameter architecture required")
    return {"tensor_count": len(archive), "parameter_count": count, "finite_float32_and_parent_shapes_verified": True,
            "pickle_checkpoint_loaded": False, "model_inference_executed": False}


def audit(args):
    """HH_261002 - Read only the caller-pinned completed run; no remote access or mutable fit operations."""
    run, data, evaluation = (contract.canonical(Path(value)) for value in (args.run_root, args.data_root, args.eval_root))
    require(all(path.is_relative_to(ROOT) and path.is_dir() for path in (run, data, evaluation)), "local canonical workspace inputs required")
    pins = Pins()
    summary = pins.json(run / "summary.json", args.caller_summary_sha256)
    require(summary.get("status") == "MATCHED_BALANCED_PAIR_COMPLETE_DIAGNOSTIC_NOT_PROMOTED"
            and summary.get("study_id") == contract.NAME and summary.get("counts") == contract.COUNTS
            and summary.get("arms") == contract.ARMS and summary.get("optimization") == contract.optimization()
            and summary.get("parent_weights_sha256") == contract.PARENT_SHA,
            "completed fixed same-parent paired fit required")
    for key, sha in (("data_manifest_sha256", args.expected_data_sha256), ("study_registry_sha256", args.expected_registry_sha256),
                     ("source_bundle_sha256", args.expected_source_sha256)):
        require(summary.get(key) == sha_string(sha), "summary external input pointer differs")
    for key, value in (("native_diagnostic_denials_preserved", True), ("ctrack_train_eligible", False),
                       ("model_promoted", False), ("vehicle_control_approved", False),
                       ("dev_used_during_optimization", False), ("source_and_data_unchanged", True), ("independent_new_dev_sites", 0)):
        require(type(summary.get(key)) is type(value) and summary[key] == value, "completed fit research scope differs: " + key)
    external = pins.json(args.artifact_pins_json, args.artifact_pins_sha256)
    artifact_receipt(external, args.caller_summary_sha256, args.expected_data_sha256,
                     args.expected_registry_sha256, args.expected_source_sha256)
    for name, sha in external["files"].items(): pins.file(run / name, sha)
    manifest = pins.json(data / "manifest.json", args.expected_data_sha256)
    registry = pins.json(data / "study_train_registry.json", args.expected_registry_sha256)
    contract.validate_manifest(manifest, registry)
    require(manifest["study_registry_sha256"] == args.expected_registry_sha256
            and summary["schedule_contract"] == manifest["schedule_contract"]
            and summary["common_class_weights"] == manifest["common_class_weights"], "common schedule/weights registry differs")
    for name, sha in (("manifest.json", args.expected_data_sha256), ("study_train_registry.json", args.expected_registry_sha256),
                      ("paired_samples.json", manifest["paired_samples_sha256"])):
        pins.file(run / "inputs" / name, sha)
        pins.file(data / name, sha)
    rows = pins.json(data / "paired_samples.json", manifest["paired_samples_sha256"])
    require([row["sample_id"] for row in rows] == manifest["paired_sample_ids"] and len(rows) == 480,
            "all480 physical pairs required, not960 independent sites")
    for name, ref in manifest["files"].items(): pins.file(data / name, ref["sha256"])
    provenance = registry.get("copied_provenance_files", {})
    require(type(provenance) is dict and bool(provenance), "native source evidence closure missing")
    for name, sha in provenance.items():
        require(name.startswith("provenance/") and ".." not in Path(name).parts, "provenance path escape")
        pins.file(data / name, sha)
    review = pins.json(data / "provenance/manual_review_and_fit_authorization.json", registry["authorization_source_sha256"])
    captures = {name: Path(ref["source_capture"]) for name, ref in registry["source_captures"].items()}
    contract.review_contract(review, registry["render_manifest_sha256"], registry["training_plan_sha256"], captures)
    for name in contract.ROUTES:
        for filename in ("raw_qa.json", "owner_result.json", "summary.json", "owner_plan.json"):
            document = pins.json(data / "provenance" / name / filename)
            require(document.get("dataset_admitted") is False and document.get("train_eligible") is False,
                    "native explicit denial must remain false")
    bundle = pins.json(run / "source/bundle.json", args.expected_source_sha256)
    require(bundle.get("schema") == contract.SOURCE_SCHEMA and bundle.get("study_id") == contract.NAME
            and set(bundle.get("files", {})) == contract.SOURCE_FILES
            and bundle.get("data_manifest_sha256") == args.expected_data_sha256
            and bundle.get("study_registry_sha256") == args.expected_registry_sha256
            and bundle.get("training_plan_sha256") == registry["training_plan_sha256"] == bundle["files"]["plan.md"],
            "exact original source/data/plan bundle required")
    pins.file(data / "source/bundle.json", args.expected_source_sha256)
    for name, sha in bundle["files"].items():
        require(name in contract.SOURCE_FILES and (name == "plan.md" or SOURCE_SHA.get(name) == sha), "frozen source revision differs")
        for folder in (run / "source", data / "source"): pins.file(folder / name, sha)
    for module in (Path(__file__).resolve(), Path(contract.__file__).resolve(), Path(contract.frozen.__file__).resolve()):
        pins.remember(module, contract.digest(module))
    require(contract.digest(Path(contract.__file__).resolve()) == SOURCE_SHA[Path(contract.__file__).name]
            and contract.digest(Path(contract.frozen.__file__).resolve()) == SOURCE_SHA[Path(contract.frozen.__file__).name],
            "publisher imported frozen contract provider changed")
    schedule = pins.npz(data / "training_schedule.npz", manifest["files"]["training_schedule.npz"]["sha256"])
    require(set(schedule) == {"indices", "crops", "balanced_view_bits"}, "common schedule tensor keys differ")
    ids, crops, bits = schedule["indices"], schedule["crops"], schedule["balanced_view_bits"]
    require(contract.schedule_proof(ids, crops, bits) == manifest["schedule_contract"], "exact original schedule proof differs")
    parent = pins.npz(data / "parent_weights.npz", contract.PARENT_SHA)
    completed = pins.json(run / "fit_complete.json")
    require(completed.get("status") == "BOTH_FIXED_FINAL10000_ENDPOINTS_SAVED_BEFORE_DEV"
            and completed.get("arms") == summary["arms_result"] and set(summary["arms_result"]) == set(contract.ARMS),
            "both fixed final endpoints before DEV receipt required")
    arms = {}
    for arm, info in summary["arms_result"].items():
        require(info.get("steps") == 10000 and info.get("examples_seen") == 80000 and info.get("parameter_count") == 245831
                and type(info.get("optimizer_initial_state_entries")) is int
                and info["optimizer_initial_state_entries"] == 0 and finite_number(info.get("training_wall_seconds")),
                "fixed final10000/fresh optimizer/80000 exposure endpoint required")
        sha_string(info.get("initial_state_sha256"))
        require(pins.json(run / arm / "training.json") == info, "endpoint training receipt differs from caller summary")
        for filename, key in (("weights.npz", "weights_sha256"), ("checkpoint.pt", "checkpoint_sha256")):
            require(external["files"][arm + "/" + filename] == info[key], "caller final endpoint SHA differs")
        model = numeric_weights(pins.npz(run / arm / "weights.npz", info["weights_sha256"]), parent)
        exposures = exposure_proof(pins.payload(run / arm / "exposures.jsonl", info["full_exposure_journal_sha256"]),
                                   info, arm, ids, crops, bits)
        history = history_proof(pins.payload(run / arm / "history.jsonl"), arm)
        require(history["last_wall_seconds"] <= info["training_wall_seconds"], "history exceeds endpoint wall duration")
        arms[arm] = {"weights_sha256": info["weights_sha256"], "checkpoint_sha256": info["checkpoint_sha256"],
                     "numeric_model": model, "exposures": exposures, "history": history}
    require(summary["arms_result"]["control"]["initial_state_sha256"] == summary["arms_result"]["balanced"]["initial_state_sha256"],
            "same parent initialization differs")
    require(pins.json(run / "known_evaluation.json", external["files"]["known_evaluation.json"]) == summary["known_evaluation"],
            "independent caller evaluation/summary linkage differs")
    eval_manifest = pins.json(evaluation / "manifest.json", EVAL_MANIFEST_SHA)
    comparisons, panels = {}, []
    for split, (count, source_sha) in SPLITS.items():
        ref = eval_manifest["files"][split]
        require(ref.get("file") == split + ".npz" and ref.get("count") == count and ref.get("sha256") == source_sha,
                "known118/232 evaluation manifest pointer differs")
        arrays = pins.npz(evaluation / (split + ".npz"), source_sha)
        require(set(arrays) == {"rgb", "target"}, "evaluation tensor keys differ")
        rgb, target = arrays["rgb"], arrays["target"]
        require(rgb.shape == (count, 360, 640, 3) and target.shape == (count, 360, 640)
                and rgb.dtype == target.dtype == np.uint8 and int(target.max()) < 7, "known full-FOV evaluation ABI differs")
        predictions, comparison = {}, {}
        for arm in contract.ARMS:
            name = arm + "/" + split + "_predictions.npz"
            archive = pins.npz(run / name, external["files"][name])
            require(set(archive) == {"prediction"}, "saved prediction tensor keys differ")
            prediction = archive["prediction"]
            require(prediction.shape == target.shape and prediction.dtype == np.uint8 and int(prediction.max()) < 7,
                    "complete full-FOV actual prediction ABI differs")
            result = metrics(target, prediction)
            reported_metrics(summary["known_evaluation"][split][arm], result, count, source_sha)
            comparisons.setdefault(split, {})[arm] = {"count": count, "source_sha256": source_sha,
                "prediction_sha256": external["files"][name], "known_exposed_diagnostic_not_independent": True, **result}
            predictions[arm] = prediction
        for index in (0, count//2, count-1):
            panels.append((split, index, rgb[index].copy(), target[index].copy(),
                           predictions["control"][index].copy(), predictions["balanced"][index].copy()))
    pins.recheck()
    return {"schema": SCHEMA, "status": "FULL_ARCHIVE_AND_PIXEL_EVALUATION_VERIFIED_NOT_MODEL_APPROVAL",
            "study_id": contract.NAME, "parent_weights_sha256": contract.PARENT_SHA,
            "data_manifest_sha256": args.expected_data_sha256, "study_registry_sha256": args.expected_registry_sha256,
            "source_bundle_sha256": args.expected_source_sha256, "caller_summary_sha256": args.caller_summary_sha256,
            "remote_artifact_pins_sha256": args.artifact_pins_sha256, "counts": contract.COUNTS,
            "optimization": contract.optimization(), "common_schedule": manifest["schedule_contract"],
            "arms": arms, "evaluation": comparisons, "labels": list(contract.LABELS),
            "semantic_palette_rgb": COLORS.tolist(), "first_read_source_and_artifact_pins": dict(pins.values),
            "native_diagnostic_denials_preserved": True, "research_fit_authorized_separately": True,
            "RGB_only_seven_class_perception_not_HD_AMG": True, "independent_new_dev_sites": 0,
            "known_evaluation_used_for_fit_or_checkpoint_selection": False,
            "formal_main_functions_approved": 0, "formal_sub_functions_approved": 0,
            "model_promoted": False, "vehicle_control_approved": False,
            "Torch_imported": False, "model_inference_executed": False, "pickle_loaded": False}, panels, pins


def canvas(split, index, rgb, target, control, balanced, weights):
    """HH_261002 - All four native640x360 panels retain full FOV and all seven original classes."""
    require(rgb.shape == (360, 640, 3) and rgb.dtype == np.uint8, "native original RGB panel required")
    for value in (target, control, balanced):
        require(value.shape == (360, 640) and value.dtype == np.uint8 and int(value.max()) < 7, "native semantic panel required")
    output = Image.new("RGB", (1280, 850), (18, 24, 35)); draw = ImageDraw.Draw(output)
    draw.text((12, 8), f"{split} fixed index{index} | ACTUAL saved predictions | known-exposed diagnostic; NOT HD map/approval", fill="white")
    for x, title in ((0, "Original RGB / no crop"), (640, "Same seven-class GT / not model input")):
        draw.text((x+12, 30), title, fill="white")
    for x, arm, title in ((0, "control", "Control: new480 always OFF"),
                           (640, "balanced", "Balanced: same480 alternating OFF/ON")):
        draw.text((x+12, 420), title, fill="white")
        draw.text((x+12, 436), "weights SHA256 " + weights[arm], fill="white")
    for x, y, pixels in ((0, 56, rgb), (640, 56, COLORS[target]), (0, 448, COLORS[control]), (640, 448, COLORS[balanced])):
        output.paste(Image.fromarray(pixels), (x, y))
    for i, (label, color) in enumerate(zip(contract.LABELS, COLORS)):
        x = i*180+12
        draw.rectangle((x, 823, x+10, 833), fill=tuple(int(v) for v in color))
        draw.text((x+15, 821), label, fill="white")
    return output


def markdown(report, files):
    lines = ["# 지도 A/B: 동일 장면에서 OFF 학습과 OFF/ON 균형 학습 비교", "",
             "두 모델을 같은 부모 가중치에서 각각 10,000회 학습했습니다. 두 팔 모두 실제 입력 80,000개를 확인했고, 원본·정답·crop·학습 순서가 같습니다.", "",
             "차이는 새 480개 장면의 RGB 조건뿐입니다. 대조군은 OFF만, 균형군은 같은 장면의 OFF/ON을 교대로 사용합니다. 960개 RGB가 960개 독립 장소라는 뜻은 아닙니다.", "",
             "이는 RGB 영상의 7개 분류 모델입니다. 정밀지도·차선 연결·신호 상태를 완성한 AMG나 주행 모델 승인이 아닙니다. 메인 0/9, 서브 0/30 공식 승인 상태는 유지됩니다.", "",
             "평가는 이미 여러 번 확인한 Town01 DEV 118장과 Town02 p02 holdout 232장 전체 화면입니다. 독립 신규 지역은 0개이며, 평가 자료를 학습·checkpoint 선택에 사용하지 않았습니다.", "",
             "## 모든 분류의 결과", "",
             "IoU는 정답과 예측의 겹침 비율입니다. FN은 놓친 정답 픽셀, FP는 잘못 표시한 픽셀입니다. 정답이 없는 분류의 IoU는 미측정(null)이며, 오탐은 숨기지 않습니다.", ""]
    def value(item): return "미측정" if item is None else f"{100*item:.2f}%"
    for split, arms in report["evaluation"].items():
        lines += [f"### {split}", "", "| 분류 | 대조군 IoU | 균형군 IoU | 대조군 FN / FP | 균형군 FN / FP |",
                  "|---|---:|---:|---:|---:|"]
        for label in contract.LABELS:
            a, b = arms["control"], arms["balanced"]
            lines.append(f"| {label} | {value(a['iou'][label])} | {value(b['iou'][label])} | {a['false_negative_pixels'][label]} / {a['false_positive_pixels'][label]} | {b['false_negative_pixels'][label]} / {b['false_positive_pixels'][label]} |")
        lines += ["", f"전체 픽셀 정확도: 대조군 {value(arms['control']['pixel_accuracy'])}, 균형군 {value(arms['balanced']['pixel_accuracy'])}.", ""]
    lines += ["## 동일 프레임 비교", "", "각 자료의 고정 첫·중간·마지막 프레임입니다. 위쪽은 원본 RGB·동일 정답, 아래쪽은 대조군·균형군 예측이며 자르거나 실패 분류를 빼지 않았습니다.", ""]
    for filename in files: lines += [f"![{filename}]({filename})", ""]
    lines += ["## 추적 정보", "", "원본 native 미승인 표시와 실패 이력은 변경하지 않았습니다. 별도 영수증으로 이 연구 학습만 허용했으며, 모든 학습 20,000행과 49칸 confusion을 검증했습니다.", "",
              "이 도구는 원격 원본에서 별도로 고정한 저장 예측을 읽고 평가합니다. 새 신경망 추론·Torch·pickle 읽기를 실행하지 않습니다.", ""]
    for arm, info in report["arms"].items(): lines += [f"- {arm} 가중치 SHA256: `{info['weights_sha256']}`"]
    lines += ["", "전체 수치·원본 해시는 [results.json](results.json), 전체 49칸 confusion은 해당 파일의 evaluation에 있습니다.", ""]
    return "\n".join(lines)


def publish(args):
    output = contract.canonical(Path(args.output))
    require(output == OUTPUT and not output.exists(), "fresh fixed results directory required")
    inputs = (Path(args.run_root), Path(args.data_root), Path(args.eval_root))
    require(not any(output.is_relative_to(path) for path in inputs), "output must remain outside immutable inputs")
    report, panels, pins = audit(args)
    if args.validate_only:
        return {"status": report["status"], "published": False, "full_actual_exposure_rows": 20000,
                "evaluation_counts": {name: value[0] for name, value in SPLITS.items()}}
    pins.recheck()
    parent = next(path for path in (output, *output.parents) if path.exists())
    require(shutil.disk_usage(parent).free >= 5*1024**3+512*1024**2, "5GiB free plus512MiB publication reserve required")
    source_payloads = {Path(filename).name: (filename, pins.payload(Path(filename), sha))
                       for filename, sha in list(pins.values.items())
                       if filename in {str(Path(__file__).resolve()), str(Path(contract.__file__).resolve()),
                                       str(Path(contract.frozen.__file__).resolve())}}
    pins.recheck()
    output.mkdir(parents=True)
    source_dir = output / "source"; source_dir.mkdir()
    archived_sources = {}
    for name, (filename, payload) in source_payloads.items():
        with (source_dir / name).open("xb") as stream: stream.write(payload)
        archived_sources["source/"+name] = {"original_path": filename, "sha256": pins.values[filename]}
    files = {}
    weights = {arm: value["weights_sha256"] for arm, value in report["arms"].items()}
    for split, index, *values in panels:
        name = f"{split}_{index:04d}.png"
        with (output / name).open("xb") as stream: canvas(split, index, *values, weights).save(stream, format="PNG")
        files[name] = contract.digest(output / name)
    pins.recheck()
    report.update(created_at_utc=datetime.now(timezone.utc).isoformat(), files=files,
                  archived_publication_sources=archived_sources,
                  fixed_indices={name: [0, count//2, count-1] for name, (count, _) in SPLITS.items()})
    contract.write_new(output / "results.json", report)
    with (output / "README.md").open("x") as stream: stream.write(markdown(report, files))
    pins.recheck()
    return {"status": report["status"], "published": True, "output": str(output),
            "results_sha256": contract.digest(output / "results.json"), "png_count": len(files)}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--run-root", type=Path, required=True)
    parser.add_argument("--data-root", type=Path, default=DATA)
    parser.add_argument("--eval-root", type=Path, default=EVAL)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    for name in ("caller-summary", "expected-data", "expected-registry", "expected-source", "artifact-pins"):
        parser.add_argument("--"+name+"-sha256", required=True)
    parser.add_argument("--artifact-pins-json", type=Path, required=True)
    parser.add_argument("--validate-only", action="store_true")
    print(json.dumps(publish(parser.parse_args(argv)), sort_keys=True), flush=True)
    return 0


if __name__ == "__main__": raise SystemExit(main())
