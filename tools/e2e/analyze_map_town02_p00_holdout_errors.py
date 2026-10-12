#!/usr/bin/env python3
"""HH_260929 - Localize frozen Town02 p00 A/B segmentation errors from saved masks."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


# HH_260929 - Freeze the prior Town05 analysis ROI definitions before reading this run's scores.
ROI_SOURCE_SHA256 = "a895e904cdee53fb20ae8d4dcc534197cdf30d2ee5ab806d71eab1de91c454ac"
DATA_MANIFEST_SHA256 = "f4764b9aa9d015f3d136a145f6194dd60bda8900c27dd2423de6145b9f472834"
RUN_SUMMARY_SHA256 = "bd0d9a100ca8e8420cbaea856c4566c3985683e7040782bcd7138f83d1d488f6"
SOURCES = {
    "town01": ("dev_town01.npz", "417c60f924abb044617e2aba5b1afd70b83f5da7f905e54513e9c220172ba4b3", 118),
    "p02": ("holdout_town02_p02.npz", "858f92ccd874c154576d3cf6a85132508801a22671baf6d0c07abe3936eb16cb", 232),
}
PREDICTIONS = {
    ("A", "town01"): "e95e9e88bad161836f3a8fdc1a128ddfb980e8c34ee64473de54226a8ad41032",
    ("B", "town01"): "b0788cd86f0602cc3729bc05f4acff8b917cf52a91737a57a31ac8c41796913d",
    ("A", "p02"): "f82317818a0e0b2a3747b4dd4370d091c96a3fddc384df4265e3b49925fad43b",
    ("B", "p02"): "0dd25174c9af31616bf5aa9414cce65ff424f4d46e5f31e31017cd2f71e8505c",
}
SUBJECTS = (("town01", "lane_marking", 3),
            ("town01", "traffic_light_fixture", 4),
            ("p02", "lane_marking", 3))
REGIONS = ("full", "far", "near_full", "near_ego")


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


def roi_slices(height: int, width: int) -> dict[str, tuple[slice, slice]]:
    """HH_260929 - Reuse pre-existing full/upper/lower/ego-center source masks exactly."""
    return {
        "full": (slice(0, height), slice(0, width)),
        "far": (slice(0, height // 2), slice(0, width)),
        "near_full": (slice(height // 2, height), slice(0, width)),
        "near_ego": (slice(height // 2, height), slice(width // 4, 3 * width // 4)),
    }


def counts(target: np.ndarray, prediction: np.ndarray, class_id: int) -> dict[str, int]:
    """HH_260929 - Count class TP/FP/FN directly from paired semantic pixels."""
    truth = target == class_id
    guessed = prediction == class_id
    tp = int(np.count_nonzero(truth & guessed))
    fp = int(np.count_nonzero(~truth & guessed))
    fn = int(np.count_nonzero(truth & ~guessed))
    return {"tp": tp, "fp": fp, "fn": fn, "gt_pixels": tp + fn,
            "predicted_pixels": tp + fp}


def iou(row: dict[str, int]) -> float | None:
    denominator = row["tp"] + row["fp"] + row["fn"]
    return row["tp"] / denominator if denominator else None


def summarize_rows(rows: list[dict[str, int]]) -> dict:
    total = {name: sum(row[name] for row in rows) for name in ("tp", "fp", "fn", "gt_pixels", "predicted_pixels")}
    total["micro_iou"] = iou(total)
    total["gt_present_frames"] = sum(row["gt_pixels"] > 0 for row in rows)
    total["zero_tp_with_gt_frames"] = sum(row["gt_pixels"] > 0 and row["tp"] == 0 for row in rows)
    return total


def paired_distribution(a_rows: list[dict], b_rows: list[dict]) -> dict:
    """HH_260929 - Count all frames and fixed temporal thirds without picking examples."""
    require(len(a_rows) == len(b_rows), "paired frame count differs")
    changes = []
    for a, b in zip(a_rows, b_rows):
        first, second = iou(a), iou(b)
        changes.append(None if first is None or second is None else second - first)

    def subset(indices):
        values = np.asarray([changes[index] for index in indices if changes[index] is not None],
                            dtype=np.float64)
        return {"frames": len(indices), "defined_pairs": len(values),
                "improved": int(np.count_nonzero(values > 1e-12)),
                "regressed": int(np.count_nonzero(values < -1e-12)),
                "tied": int(np.count_nonzero(np.abs(values) <= 1e-12)),
                "median_delta": float(np.median(values)) if len(values) else None,
                "mean_delta": float(np.mean(values)) if len(values) else None}

    result = subset(range(len(changes)))
    chunks = np.array_split(np.arange(len(changes)), 3)
    result["temporal_thirds"] = [subset(chunk.tolist()) for chunk in chunks]
    return result


def analyze_pair(target: np.ndarray, predictions: dict[str, np.ndarray], class_id: int) -> dict:
    require(target.ndim == 3 and target.shape[1:] == (360, 640)
            and target.dtype == np.uint8 and int(target.max()) < 7,
            "semantic target ABI differs")
    require(set(predictions) == {"A", "B"}
            and all(pred.shape == target.shape and pred.dtype == np.uint8
                    and int(pred.max()) < 7 for pred in predictions.values()),
            "paired prediction ABI differs")
    rois = roi_slices(360, 640)
    result = {}
    for region, (ys, xs) in rois.items():
        arm_rows = {
            arm: [counts(target[index, ys, xs], pred[index, ys, xs], class_id)
                  for index in range(len(target))]
            for arm, pred in predictions.items()
        }
        a = summarize_rows(arm_rows["A"])
        b = summarize_rows(arm_rows["B"])
        result[region] = {
            "A": a, "B": b,
            "delta_b_minus_a": {name: b[name] - a[name] for name in ("tp", "fp", "fn")},
            "micro_iou_delta": b["micro_iou"] - a["micro_iou"]
            if a["micro_iou"] is not None and b["micro_iou"] is not None else None,
            "paired_frame_iou": paired_distribution(arm_rows["A"], arm_rows["B"]),
        }
    return result


def cross_class_errors(target: np.ndarray, prediction: np.ndarray, class_id: int) -> dict:
    """HH_260929 - Reveal which other labels receive missed class pixels or source FP."""
    fp_source = np.bincount(target[prediction == class_id].ravel(), minlength=7)
    fn_destination = np.bincount(prediction[target == class_id].ravel(), minlength=7)
    fp_source[class_id] = 0
    fn_destination[class_id] = 0
    return {"fp_source_class": fp_source.tolist(),
            "fn_predicted_class": fn_destination.tolist()}


def analyze(data_root: Path, run_root: Path, roi_source: Path) -> dict:
    require(digest(roi_source) == ROI_SOURCE_SHA256, "frozen ROI source changed")
    require(digest(data_root / "manifest.json") == DATA_MANIFEST_SHA256,
            "dataset manifest changed")
    require(digest(run_root / "summary.json") == RUN_SUMMARY_SHA256,
            "run summary changed")
    summary = json.loads((run_root / "summary.json").read_text(encoding="utf-8"))
    result = {"schema": "portable_e2e.town02_p00_error_analysis.v1",
              "roi_xyxy": {"full": [0, 0, 640, 360], "far": [0, 0, 640, 180],
                           "near_full": [0, 180, 640, 360],
                           "near_ego": [160, 180, 480, 360]},
              "roi_source_sha256": ROI_SOURCE_SHA256,
              "dataset_manifest_sha256": DATA_MANIFEST_SHA256,
              "run_summary_sha256": RUN_SUMMARY_SHA256,
              "subjects": {}, "independent_dev_sites": 0,
              "model_promotion": False}
    for split in SOURCES:
        filename, expected_hash, count = SOURCES[split]
        require(digest(data_root / filename) == expected_hash,
                f"{split} target bytes changed")
        with np.load(data_root / filename, allow_pickle=False) as archive:
            require(set(archive.files) == {"rgb", "target"}, f"{split} keys changed")
            target = archive["target"]
        require(target.shape == (count, 360, 640), f"{split} target count changed")
        predictions = {}
        for arm in ("A", "B"):
            path = run_root / arm / f"{split}_predictions.npz"
            require(digest(path) == PREDICTIONS[(arm, split)],
                    f"{arm}/{split} prediction bytes changed")
            with np.load(path, allow_pickle=False) as archive:
                require(set(archive.files) == {"prediction"},
                        f"{arm}/{split} prediction keys changed")
                predictions[arm] = archive["prediction"]
        for subject_split, name, class_id in SUBJECTS:
            if split != subject_split:
                continue
            analysis = analyze_pair(target, predictions, class_id)
            require(abs(analysis["full"]["A"]["micro_iou"]
                        - summary["A"][split]["iou"][name]) < 1e-12
                    and abs(analysis["full"]["B"]["micro_iou"]
                            - summary["B"][split]["iou"][name]) < 1e-12,
                    f"{split}/{name} full-frame IoU differs from audited run")
            analysis["full_cross_class"] = {
                arm: cross_class_errors(target, pred, class_id)
                for arm, pred in predictions.items()}
            result["subjects"][f"{split}/{name}"] = analysis
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--roi-source", type=Path,
                        default=Path(__file__).with_name("analyze_map_road_lane_ab_town05_errors.py"))
    args = parser.parse_args()
    print(json.dumps(analyze(args.data, args.run, args.roi_source),
                     ensure_ascii=False, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
