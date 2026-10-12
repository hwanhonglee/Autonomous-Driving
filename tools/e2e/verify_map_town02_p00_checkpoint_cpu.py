#!/usr/bin/env python3
"""HH_260929 - Re-infer the archived map A/B checkpoints on CPU without retraining."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys


# HH_260929 - Bind independently copied inputs before loading checkpoint pickle bytes.
MANIFEST_SHA256 = "f4764b9aa9d015f3d136a145f6194dd60bda8900c27dd2423de6145b9f472834"
SUMMARY_SHA256 = "bd0d9a100ca8e8420cbaea856c4566c3985683e7040782bcd7138f83d1d488f6"
MODEL_SOURCE_SHA256 = "1b0076c8f9d34192ab5aedf6cdf96ce2c14458e2bb49986685a6811376b0750b"
CHECKPOINT_SHA256 = {
    "A": "70db106975b990e1ad4dacbf150edefd715515b351a22f6bc2f19c5d99d4b393",
    "B": "04614038b3d77a38f79b5a043c995b78ef6ee1d64537575fb2903c261ea5a746",
}
SPLITS = {"town01": ("dev_town01", 118), "p02": ("holdout_town02_p02", 232)}
# HH_260929 - This permissive consistency guard is not a model-quality or promotion gate.
MIN_PIXEL_AGREEMENT = 0.995


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def confusion_matrix(target, prediction, classes: int, np):
    """HH_260929 - Count target/prediction pairs independent of the remote evaluator."""
    require(target.shape == prediction.shape, "target/prediction shape mismatch")
    require(target.dtype == np.uint8 and prediction.dtype == np.uint8,
            "target/prediction dtype mismatch")
    require(int(target.max()) < classes and int(prediction.max()) < classes,
            "target/prediction class outside frozen vocabulary")
    code = target.astype(np.int64) * classes + prediction
    return np.bincount(code.ravel(), minlength=classes * classes).reshape(classes, classes)


def iou_from_matrix(matrix, labels, np):
    """HH_260929 - Keep absent-class IoU undefined instead of silently treating it as zero."""
    diagonal = np.diag(matrix)
    union = matrix.sum(0) + matrix.sum(1) - diagonal
    return {name: None if not union[index] else float(diagonal[index] / union[index])
            for index, name in enumerate(labels)}


def compare_predictions(reinferred, archived, target, labels, np):
    """HH_260929 - Measure CPU/GPU disagreement and its 7-class metric effect."""
    require(reinferred.shape == archived.shape == target.shape,
            "re-inference/archived/target shapes differ")
    require(reinferred.dtype == archived.dtype == target.dtype == np.uint8,
            "re-inference/archived/target dtypes differ")
    difference = reinferred != archived
    total = int(difference.size)
    changed = int(np.count_nonzero(difference))
    per_frame = np.count_nonzero(difference.reshape(len(difference), -1), axis=1)
    expected_matrix = confusion_matrix(target, archived, len(labels), np)
    actual_matrix = confusion_matrix(target, reinferred, len(labels), np)
    expected_iou = iou_from_matrix(expected_matrix, labels, np)
    actual_iou = iou_from_matrix(actual_matrix, labels, np)
    delta = {name: None if expected_iou[name] is None or actual_iou[name] is None
             else actual_iou[name] - expected_iou[name] for name in labels}
    return {
        "frames": int(len(difference)), "pixels": total, "changed_pixels": changed,
        "pixel_agreement": 1.0 - changed / total,
        "worst_frame_index": int(np.argmax(per_frame)),
        "worst_frame_changed_pixels": int(per_frame.max()),
        "archived_iou": expected_iou, "cpu_iou": actual_iou,
        "cpu_minus_archived_iou": delta,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    # HH_260929 - Force read-only model evaluation on local CPU before importing Torch.
    os.environ["CUDA_VISIBLE_DEVICES"] = ""
    import numpy as np
    import torch

    data_root, run_root, output = args.data.resolve(), args.run.resolve(), args.output.absolute()
    require(output.parent.is_dir() and not output.exists() and not output.is_symlink(),
            "fresh output file in an existing directory required")
    manifest_path, summary_path = data_root / "manifest.json", run_root / "summary.json"
    source_path = run_root / "source/map_perception.py"
    require(sha256(manifest_path) == MANIFEST_SHA256, "package manifest differs")
    require(sha256(summary_path) == SUMMARY_SHA256, "run summary differs")
    require(sha256(source_path) == MODEL_SOURCE_SHA256, "archived model code differs")
    manifest = json.loads(manifest_path.read_text())
    summary = json.loads(summary_path.read_text())
    require(summary.get("model_promotion") is False
            and summary.get("independent_dev_sites") == 0,
            "archived result is not the frozen research candidate")
    require(set(manifest["files"]) == {"train_a", "train_b", *(
        source for source, _ in SPLITS.values())}, "data split set differs")
    for arm, expected in CHECKPOINT_SHA256.items():
        require(summary[arm]["checkpoint_sha256"] == expected
                and sha256(run_root / arm / "checkpoint.pt") == expected,
                f"{arm} checkpoint differs")

    spec = importlib.util.spec_from_file_location("frozen_map_perception_cpu", source_path)
    require(spec is not None and spec.loader is not None, "frozen model import failed")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    torch.set_num_threads(2)
    results = {}
    with torch.no_grad():
        for arm in CHECKPOINT_SHA256:
            checkpoint = torch.load(run_root / arm / "checkpoint.pt", map_location="cpu")
            require(tuple(checkpoint["labels"]) == tuple(module.LABELS),
                    f"{arm} model labels differ")
            model = module.MapPerception()
            model.load_state_dict(checkpoint["model_state_dict"], strict=True)
            model.eval()
            results[arm] = {}
            for split, (data_key, frames) in SPLITS.items():
                ref = manifest["files"][data_key]
                data_path = data_root / ref["file"]
                require(ref["count"] == frames and sha256(data_path) == ref["sha256"],
                        f"{split} package differs")
                with np.load(data_path, allow_pickle=False) as package:
                    rgb, target = package["rgb"], package["target"]
                prediction_path = run_root / arm / f"{split}_predictions.npz"
                with np.load(prediction_path, allow_pickle=False) as saved:
                    archived = saved["prediction"]
                require(rgb.shape == (frames, 360, 640, 3)
                        and target.shape == archived.shape == (frames, 360, 640),
                        f"{split} ABI differs")
                inferred = np.empty_like(archived)
                for begin in range(0, frames, 4):
                    batch = torch.from_numpy(rgb[begin:begin + 4].transpose(0, 3, 1, 2).copy())
                    batch = batch.float() / 255.0
                    inferred[begin:begin + 4] = model(batch).argmax(1).numpy().astype(np.uint8)
                comparison = compare_predictions(inferred, archived, target, module.LABELS, np)
                comparison["archived_prediction_sha256"] = sha256(prediction_path)
                require(comparison["pixel_agreement"] >= MIN_PIXEL_AGREEMENT,
                        f"{arm}/{split} checkpoint and archived predictions diverge")
                results[arm][split] = comparison
                print(json.dumps({"arm": arm, "split": split,
                                  "pixel_agreement": comparison["pixel_agreement"],
                                  "changed_pixels": comparison["changed_pixels"]}), flush=True)
                del rgb, target, archived, inferred
    report = {
        "schema": "portable_e2e.map_town02_p00_checkpoint_cpu_reinference.v1",
        "status": "CPU_CHECKPOINT_TO_ARCHIVED_PREDICTION_CONSISTENT",
        "runtime_torch": torch.__version__, "torch_cuda_visible_devices": "",
        "minimum_pixel_agreement": MIN_PIXEL_AGREEMENT,
        "data_manifest_sha256": MANIFEST_SHA256, "run_summary_sha256": SUMMARY_SHA256,
        "model_source_sha256": MODEL_SOURCE_SHA256,
        "checkpoint_sha256": CHECKPOINT_SHA256,
        "results": results, "model_promotion": False,
        "independent_dev_sites": 0, "learned_closed_loop_driving": False,
    }
    with output.open("x") as stream:
        json.dump(report, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps({"status": report["status"], "output": str(output)}), flush=True)


if __name__ == "__main__":
    main()
