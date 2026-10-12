#!/usr/bin/env python3
"""HH_260929 - Independently replay the archived map A/B DEV confusion matrices."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


LABELS = ("other", "road", "sidewalk", "lane_marking", "traffic_light_fixture", "traffic_sign", "pole")
DEV_COUNTS = {"town01": 118, "town05": 232}


def digest(path: Path) -> str:
    hasher = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            hasher.update(block)
    return hasher.hexdigest()


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def replay_confusion(target: np.ndarray, prediction: np.ndarray) -> np.ndarray:
    """HH_260929 - Count every DEV pixel without using training-time metric code."""
    require(target.shape == prediction.shape and target.dtype == np.uint8
            and prediction.dtype == np.uint8, "prediction and target contract differs")
    require(int(target.max()) < len(LABELS) and int(prediction.max()) < len(LABELS),
            "semantic class index escaped contract")
    n = len(LABELS)
    return np.bincount((target.astype(np.int64) * n + prediction).ravel(),
                       minlength=n * n).reshape(n, n)


def audit(data: Path, run: Path) -> dict:
    manifest = json.loads((data / "manifest.json").read_text())
    started = json.loads((run / "started.json").read_text())
    summary = json.loads((run / "summary.json").read_text())
    require(summary["status"] == "A_B_COMPLETE_EXPLORATORY_NOT_PROMOTED"
            and summary["model_promotion"] is False
            and summary["gpu_training_executed"] is True,
            "incomplete or promoted run cannot be audited as this diagnostic")
    require(digest(data / "manifest.json") == started["data_manifest_sha256"]
            and digest(run / "data_manifest.json") == started["data_manifest_sha256"],
            "data manifest differs from frozen run")
    require(digest(run / "source" / "train_map_road_lane_ab.py")
            == started["training_source_sha256"], "trainer source changed")
    for source, expected in started["support_sha256"].items():
        require(digest(run / "source" / source) == expected,
                f"support source changed: {source}")

    checked = {}
    for town, count in DEV_COUNTS.items():
        name = "dev_" + town
        ref = manifest["files"][name]
        path = data / ref["file"]
        require(ref["count"] == count and digest(path) == ref["sha256"],
                f"{name} DEV input changed")
        with np.load(path, allow_pickle=False) as arrays:
            require(set(arrays.files) == {"rgb", "target"}, f"{name} keys changed")
            target = arrays["target"].copy()
        require(target.shape == (count, 360, 640), f"{name} shape changed")
        checked[town] = {}
        for arm in ("D0", "D1"):
            path = run / arm / f"{town}_predictions.npz"
            with np.load(path, allow_pickle=False) as arrays:
                require(set(arrays.files) == {"prediction"},
                        f"{arm}/{town} prediction keys changed")
                prediction = arrays["prediction"].copy()
            confusion = replay_confusion(target, prediction)
            metrics = json.loads((run / arm / "metrics.json").read_text())
            require(metrics == summary[arm], f"{arm} summary and metrics differ")
            require(np.array_equal(confusion, np.asarray(metrics[town]["confusion"])),
                    f"{arm}/{town} archived confusion does not replay")
            intersection = np.diag(confusion)
            union = confusion.sum(0) + confusion.sum(1) - intersection
            for index, label in enumerate(LABELS):
                expected = float(intersection[index] / union[index]) if union[index] else None
                observed = metrics[town]["iou"][label]
                require((expected is None and observed is None) or
                        (expected is not None and observed is not None
                         and abs(expected - observed) <= 1e-12),
                        f"{arm}/{town}/{label} IoU does not replay")
            require(digest(run / arm / "checkpoint.pt") == metrics["checkpoint_sha256"],
                    f"{arm} checkpoint changed")
            checked[town][arm] = metrics[town]["iou"]
    return {"status": "ARCHIVED_DEV_PIXEL_REPLAY_PASS", "counts": DEV_COUNTS,
            "iou": checked, "model_promotion": False}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--run", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(audit(args.data, args.run), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
