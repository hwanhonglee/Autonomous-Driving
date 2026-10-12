#!/usr/bin/env python3
"""HH_260929 - Independently verify the saved Town02 p00 route-holdout A/B receipt."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np


LABELS = ("other", "road", "sidewalk", "lane_marking",
          "traffic_light_fixture", "traffic_sign", "pole")
EXPECTED_DATA_MANIFEST_SHA256 = "f4764b9aa9d015f3d136a145f6194dd60bda8900c27dd2423de6145b9f472834"
EXPECTED_PARENT_MANIFEST_SHA256 = "e9805574a252e72f8e81fcd434156da7782b25e8e965b8078f976cf4f234892c"
EXPECTED_PARENT_CHECKPOINT_SHA256 = "ae489cd99e48f6f68da3241e8296c705847398c23b8ba69b71ec024cef4ef380"
EXPECTED_TRAINING_SOURCE_SHA256 = "7ca69282db68019c6c5a2d09145dfcb4426047cf69c3f8f9355072ac0d3cd108"
EXPECTED_RESEARCH_SCOPE_SHA256 = "c4870d042a074f559be7ed841ba0351be007d2d562d9a0b7567db2ac95b719b0"
COUNTS = {"train_a": 497, "train_b": 729, "holdout_town02_p02": 232, "dev_town01": 118}
SPLITS = {"p02": ("holdout_town02_p02", 232), "town01": ("dev_town01", 118)}


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def digest(path: Path) -> str:
    require(path.is_file() and not path.is_symlink(), f"regular file required: {path}")
    sha = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            sha.update(block)
    return sha.hexdigest()


def read_json(path: Path):
    require(path.is_file() and not path.is_symlink(), f"regular JSON required: {path}")
    return json.loads(path.read_text(encoding="utf-8"))


def confusion_of(target: np.ndarray, prediction: np.ndarray) -> np.ndarray:
    """HH_260929 - Count raw target/prediction pairs without calling the training evaluator."""
    require(target.shape == prediction.shape and target.dtype == prediction.dtype == np.uint8,
            "prediction/target ABI differs")
    require(target.ndim == 2 and target.shape == (360, 640)
            and int(target.max()) < 7 and int(prediction.max()) < 7,
            "prediction/target shape or class range differs")
    packed = target.astype(np.uint16) * 7 + prediction
    return np.bincount(packed.ravel(), minlength=49).reshape(7, 7)


def metrics_of(matrix: np.ndarray) -> dict:
    true = matrix.sum(axis=1)
    predicted = matrix.sum(axis=0)
    diagonal = np.diag(matrix)
    union = true + predicted - diagonal
    return {
        "confusion": matrix.tolist(),
        "true_class_pixels": true.tolist(),
        "iou": {name: (float(diagonal[index] / union[index]) if union[index] else None)
                for index, name in enumerate(LABELS)},
        "precision": {name: (float(diagonal[index] / predicted[index]) if predicted[index] else None)
                      for index, name in enumerate(LABELS)},
        "pixel_accuracy": float(diagonal.sum() / matrix.sum()),
    }


def compare_numbers(actual, saved, label: str) -> None:
    """HH_260929 - Check every published numeric leaf, including undefined classes."""
    if isinstance(actual, dict):
        require(isinstance(saved, dict) and set(actual) == set(saved), f"{label} keys differ")
        for key, value in actual.items():
            compare_numbers(value, saved[key], f"{label}.{key}")
    elif isinstance(actual, list):
        require(isinstance(saved, list) and len(actual) == len(saved), f"{label} length differs")
        for index, value in enumerate(actual):
            compare_numbers(value, saved[index], f"{label}[{index}]")
    elif isinstance(actual, float):
        require(isinstance(saved, (int, float)) and not isinstance(saved, bool)
                and math.isclose(actual, float(saved), rel_tol=0, abs_tol=1e-12),
                f"{label} value differs")
    else:
        require(actual == saved, f"{label} value differs")


def recalculate_gates(a: dict, b: dict) -> dict:
    """HH_260929 - Apply the five frozen A/B gates from the saved predictions."""
    delta_p02 = {name: b["p02"]["iou"][name] - a["p02"]["iou"][name] for name in LABELS}
    delta_town01 = {name: b["town01"]["iou"][name] - a["town01"]["iou"][name]
                    for name in LABELS}
    epsilon = 1e-12
    gates = {
        "p02_lane_gain_at_least_2pp": delta_p02["lane_marking"] + epsilon >= 0.02,
        "p02_road_drop_at_most_1pp": delta_p02["road"] + epsilon >= -0.01,
        "town01_road_drop_at_most_0p5pp": delta_town01["road"] + epsilon >= -0.005,
        "town01_lane_drop_at_most_0p5pp": delta_town01["lane_marking"] + epsilon >= -0.005,
        "town01_all_seven_drop_at_most_0p5pp": all(
            delta_town01[name] + epsilon >= -0.005 for name in LABELS),
    }
    return {"gates": gates, "all_gates_passed": all(gates.values()),
            "p02_iou_delta_b_minus_a": delta_p02,
            "town01_iou_delta_b_minus_a": delta_town01,
            "independent_dev_sites": 0, "automatic_model_promotion": False}


def audit(data_root: Path, run_root: Path) -> dict:
    require(data_root.is_dir() and run_root.is_dir(), "dataset and run directories required")
    manifest_path = data_root / "manifest.json"
    require(digest(manifest_path) == EXPECTED_DATA_MANIFEST_SHA256,
            "preregistered dataset manifest SHA-256 differs")
    manifest = read_json(manifest_path)
    started = read_json(run_root / "started.json")
    summary = read_json(run_root / "summary.json")
    require(manifest["schema"] == "portable_e2e.map_town02_p00_holdout_data.v1"
            and manifest["parent_manifest_sha256"] == EXPECTED_PARENT_MANIFEST_SHA256
            and manifest["labels"] == list(LABELS)
            and manifest["dataset_admitted"] is False
            and manifest["independent_dev_approved"] is False
            and manifest["exploratory_ab_train_authorized"] is True
            and manifest["provenance"]["research_scope_sha256"] == EXPECTED_RESEARCH_SCOPE_SHA256,
            "exploratory dataset provenance/approval differs")
    require(digest(run_root / "data_manifest.json") == EXPECTED_DATA_MANIFEST_SHA256
            and started["dataset_manifest_sha256"] == EXPECTED_DATA_MANIFEST_SHA256,
            "training dataset manifest copy differs")
    require(started["parent_checkpoint_sha256"] == EXPECTED_PARENT_CHECKPOINT_SHA256
            and started["seed"] == 20260922 and started["steps_per_arm"] == 2000
            and started["batch_size"] == 8
            and started["learning_rate"] == 3e-4
            and started["weight_decay"] == 1e-4
            and started["gpu"]["physical_index"] == 0
            and started["gpu"]["uuid"] == "GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5"
            and started["research_scope_sha256"] == EXPECTED_RESEARCH_SCOPE_SHA256
            and started["automatic_promotion"] is False
            and started["independent_dev_sites"] == 0,
            "training contract or GPU0 receipt differs")
    require(digest(run_root / "source/train_map_town02_p00_route_holdout.py")
            == started["training_source_sha256"] == EXPECTED_TRAINING_SOURCE_SHA256,
            "preregistered training source bytes differ")
    for relative, sha in started["support_sha256"].items():
        require(digest(run_root / "source" / relative) == sha,
                f"support source bytes differ: {relative}")
    for relative, sha in started["frozen_helper_sha256"].items():
        require(digest(run_root / "frozen_helper" / relative) == sha,
                f"frozen helper bytes differ: {relative}")
    require(set(manifest["files"]) == set(COUNTS), "split list differs")
    targets = {}
    split_histograms = {}
    for name, count in COUNTS.items():
        receipt = manifest["files"][name]
        require(receipt["file"] == f"{name}.npz" and receipt["count"] == count
                and receipt["sha256"] == digest(data_root / receipt["file"]),
                f"{name} package hash/count differs")
        with np.load(data_root / receipt["file"], allow_pickle=False) as archive:
            require(set(archive.files) == {"rgb", "target"}, f"{name} keys differ")
            rgb = archive["rgb"]
            target = archive["target"]
        require(rgb.shape == (count, 360, 640, 3) and rgb.dtype == np.uint8
                and target.shape == (count, 360, 640) and target.dtype == np.uint8
                and int(target.max()) < 7, f"{name} package array ABI differs")
        histogram = np.bincount(target.ravel(), minlength=7).tolist()
        require(histogram == receipt["class_pixels"], f"{name} label histogram differs")
        split_histograms[name] = histogram
        if name == "train_a":
            train_a_rgb, train_a_target = rgb, target
        elif name == "train_b":
            require(np.array_equal(rgb[:497], train_a_rgb)
                    and np.array_equal(target[:497], train_a_target),
                    "B altered frozen A TRAIN prefix")
            del train_a_rgb, train_a_target
        if name in ("dev_town01", "holdout_town02_p02"):
            targets[name] = target
    require(started["class_pixels"] == split_histograms["train_a"],
            "fixed A-only class weights source differs")

    computed = {}
    per_frame_defined = {}
    for arm in ("A", "B"):
        arm_dir = run_root / arm
        recorded = read_json(arm_dir / "metrics.json")
        require(recorded == summary[arm], f"{arm} summary/metrics copy differs")
        require(digest(arm_dir / "checkpoint.pt") == recorded["checkpoint_sha256"]
                and recorded["deployment_approved"] is False
                and recorded["train_count"] == COUNTS["train_a" if arm == "A" else "train_b"],
                f"{arm} checkpoint/train receipt differs")
        with (arm_dir / "history.jsonl").open(encoding="utf-8") as stream:
            history = [json.loads(line) for line in stream if line.strip()]
        require([row["step"] for row in history] == list(range(100, 2001, 100))
                and all(math.isfinite(row["loss"]) and math.isfinite(row["wall_seconds"])
                        for row in history)
                and all(first["wall_seconds"] < second["wall_seconds"]
                        for first, second in zip(history, history[1:]))
                and history[-1]["wall_seconds"] <= recorded["training_wall_seconds"] + 0.01,
                f"{arm} 2000-step log differs")
        saved_frame_iou = read_json(arm_dir / "frame_iou.json")
        computed[arm] = {}
        per_frame_defined[arm] = {}
        for split, (source_name, count) in SPLITS.items():
            with np.load(arm_dir / f"{split}_predictions.npz", allow_pickle=False) as archive:
                require(set(archive.files) == {"prediction"},
                        f"{arm}/{split} prediction keys differ")
                prediction = archive["prediction"]
            require(prediction.shape == (count, 360, 640) and prediction.dtype == np.uint8,
                    f"{arm}/{split} prediction ABI differs")
            frame_rows = saved_frame_iou[split]
            require(len(frame_rows) == count, f"{arm}/{split} frame count differs")
            matrix = np.zeros((7, 7), dtype=np.int64)
            defined = dict.fromkeys(LABELS, 0)
            for index in range(count):
                frame_matrix = confusion_of(targets[source_name][index], prediction[index])
                matrix += frame_matrix
                frame_iou = metrics_of(frame_matrix)["iou"]
                compare_numbers(frame_iou, frame_rows[index], f"{arm}/{split}/frame{index}")
                for name, value in frame_iou.items():
                    defined[name] += value is not None
            computed[arm][split] = metrics_of(matrix)
            compare_numbers(computed[arm][split], recorded[split], f"{arm}/{split}")
            require(matrix.sum() == count * 360 * 640, f"{arm}/{split} pixel count differs")
            per_frame_defined[arm][split] = defined

    parent = read_json(run_root / "a_reproduction.json")
    require(parent["pass"] is True and parent["tolerance"] == 0.005
            and set(parent["iou_absolute_difference"]) == set(LABELS)
            and all(value == 0 for value in parent["iou_absolute_difference"].values()),
            "A reproduction receipt differs")
    gates = recalculate_gates(computed["A"], computed["B"])
    compare_numbers(gates, summary["gates"], "gates")
    require(summary["status"] == "A_B_INTERNAL_ROUTE_HOLDOUT_COMPLETE_NOT_PROMOTED"
            and summary["model_promotion"] is False
            and summary["independent_dev_sites"] == 0
            and summary["hd_map_complete"] is False
            and summary["learned_closed_loop_driving"] is False
            and gates["all_gates_passed"] is False,
            "run was promoted or failed gates were hidden")
    return {"status": "INDEPENDENT_SAVED_ARTIFACT_AUDIT_PASS",
            "split_counts": COUNTS, "source_hashes_checked": 1 + len(started["support_sha256"])
            + len(started["frozen_helper_sha256"]),
            "data_file_hashes_checked": len(COUNTS),
            "checkpoints_hashed": 2,
            "evaluated_frames_per_arm": {name: count for name, (_, count) in SPLITS.items()},
            "frame_iou_rows_checked": 2 * sum(count for _, count in SPLITS.values()),
            "confusion_matrices_checked": 2 * len(SPLITS),
            "per_frame_defined_iou_counts": per_frame_defined,
            "p02_iou": {arm: computed[arm]["p02"]["iou"] for arm in computed},
            "town01_iou": {arm: computed[arm]["town01"]["iou"] for arm in computed},
            "gates": gates,
            "training_wall_seconds_per_arm": {
                arm: summary[arm]["training_wall_seconds"] for arm in ("A", "B")},
            "automatic_model_promotion": False,
            "independent_dev_sites": 0}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--run", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(audit(args.data, args.run), ensure_ascii=False, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
