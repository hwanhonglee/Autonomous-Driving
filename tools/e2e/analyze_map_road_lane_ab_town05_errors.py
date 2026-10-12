#!/usr/bin/env python3
"""HH_260929 - Describe Town05 DEV road/lane errors from frozen arrays, not causal claims."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from scripts.e2e import render_map_road_lane_ab_town05 as verified


REGIONS = ("full", "far", "near_full", "near_ego")
CSV_FIELDS = (
    "frame_index", "arm", "region", "oracle_lane_px", "pred_lane_px",
    "lane_tp", "lane_fp", "lane_fn", "lane_iou", "lane_recall", "lane_precision",
    "oracle_road_px", "pred_road_px", "road_tp", "road_fp", "road_fn",
    "road_iou", "road_recall", "road_precision", "nonroad_px", "road_fp_nonroad_rate",
    *(f"road_fp_on_{name}" for name in verified.LABELS if name != "road"),
)


def ratio(numerator: int, denominator: int) -> float | None:
    return numerator / denominator if denominator else None


def region_slices(height: int, width: int) -> dict[str, tuple[slice, slice]]:
    """HH_260929 - Match the rendered near-ego crop and separate the upper/lower FOV."""
    middle = height // 2
    return {
        "full": (slice(0, height), slice(0, width)),
        "far": (slice(0, middle), slice(0, width)),
        "near_full": (slice(middle, height), slice(0, width)),
        "near_ego": (slice(middle, height), slice(width // 4, 3 * width // 4)),
    }


def frame_row(index: int, arm: str, region: str,
              target: np.ndarray, prediction: np.ndarray) -> dict:
    lane_true = target == verified.LANE
    lane_pred = prediction == verified.LANE
    road_true = target == verified.ROAD
    road_pred = prediction == verified.ROAD
    lane_tp = int(np.count_nonzero(lane_true & lane_pred))
    lane_fp = int(np.count_nonzero(~lane_true & lane_pred))
    lane_fn = int(np.count_nonzero(lane_true & ~lane_pred))
    road_tp = int(np.count_nonzero(road_true & road_pred))
    road_fp_mask = ~road_true & road_pred
    road_fp = int(np.count_nonzero(road_fp_mask))
    road_fn = int(np.count_nonzero(road_true & ~road_pred))
    nonroad_px = target.size - int(np.count_nonzero(road_true))
    false_positive_classes = np.bincount(target[road_fp_mask].ravel(),
                                         minlength=len(verified.LABELS))
    result = {
        "frame_index": index, "arm": arm, "region": region,
        "oracle_lane_px": lane_tp + lane_fn, "pred_lane_px": lane_tp + lane_fp,
        "lane_tp": lane_tp, "lane_fp": lane_fp, "lane_fn": lane_fn,
        "lane_iou": ratio(lane_tp, lane_tp + lane_fp + lane_fn),
        "lane_recall": ratio(lane_tp, lane_tp + lane_fn),
        "lane_precision": ratio(lane_tp, lane_tp + lane_fp),
        "oracle_road_px": road_tp + road_fn, "pred_road_px": road_tp + road_fp,
        "road_tp": road_tp, "road_fp": road_fp, "road_fn": road_fn,
        "road_iou": ratio(road_tp, road_tp + road_fp + road_fn),
        "road_recall": ratio(road_tp, road_tp + road_fn),
        "road_precision": ratio(road_tp, road_tp + road_fp),
        "nonroad_px": nonroad_px,
        "road_fp_nonroad_rate": ratio(road_fp, nonroad_px),
    }
    for label, name in enumerate(verified.LABELS):
        if name != "road":
            result[f"road_fp_on_{name}"] = int(false_positive_classes[label])
    return result


def distribution(rows: list[dict], key: str) -> dict:
    values = np.asarray([row[key] for row in rows if row[key] is not None], dtype=np.float64)
    if not len(values):
        return {"defined_frames": 0, "mean": None, "p10": None,
                "median": None, "p90": None, "min": None, "max": None}
    return {"defined_frames": len(values), "mean": float(values.mean()),
            "p10": float(np.quantile(values, 0.1)),
            "median": float(np.median(values)),
            "p90": float(np.quantile(values, 0.9)),
            "min": float(values.min()), "max": float(values.max())}


def aggregate(rows: list[dict]) -> dict:
    """HH_260929 - Keep micro pixel scores distinct from frame-distribution scores."""
    lane_tp = sum(row["lane_tp"] for row in rows)
    lane_fp = sum(row["lane_fp"] for row in rows)
    lane_fn = sum(row["lane_fn"] for row in rows)
    road_tp = sum(row["road_tp"] for row in rows)
    road_fp = sum(row["road_fp"] for row in rows)
    road_fn = sum(row["road_fn"] for row in rows)
    nonroad = sum(row["nonroad_px"] for row in rows)
    return {
        "frames": len(rows),
        "lane": {
            "oracle_px": lane_tp + lane_fn, "predicted_px": lane_tp + lane_fp,
            "tp": lane_tp, "fp": lane_fp, "fn": lane_fn,
            "micro_iou": ratio(lane_tp, lane_tp + lane_fp + lane_fn),
            "micro_recall": ratio(lane_tp, lane_tp + lane_fn),
            "micro_precision": ratio(lane_tp, lane_tp + lane_fp),
            "oracle_present_frames": sum(row["oracle_lane_px"] > 0 for row in rows),
            "zero_tp_with_oracle_frames": sum(row["oracle_lane_px"] > 0
                                               and row["lane_tp"] == 0 for row in rows),
            "zero_predicted_frames": sum(row["pred_lane_px"] == 0 for row in rows),
            "frame_iou": distribution(rows, "lane_iou"),
            "frame_recall": distribution(rows, "lane_recall"),
        },
        "road": {
            "oracle_px": road_tp + road_fn, "predicted_px": road_tp + road_fp,
            "tp": road_tp, "fp": road_fp, "fn": road_fn,
            "micro_iou": ratio(road_tp, road_tp + road_fp + road_fn),
            "micro_recall": ratio(road_tp, road_tp + road_fn),
            "micro_precision": ratio(road_tp, road_tp + road_fp),
            "nonroad_px": nonroad,
            "fp_nonroad_rate": ratio(road_fp, nonroad),
            "fp_by_oracle_class": {
                name: sum(row[f"road_fp_on_{name}"] for row in rows)
                for name in verified.LABELS if name != "road"
            },
            "frame_fp_nonroad_rate": distribution(rows, "road_fp_nonroad_rate"),
        },
    }


def paired_delta(d0_rows: list[dict], d1_rows: list[dict], key: str) -> dict:
    require = verified.require
    require(len(d0_rows) == len(d1_rows), "paired frame count changed")
    values = []
    for d0, d1 in zip(d0_rows, d1_rows):
        require(d0["frame_index"] == d1["frame_index"], "paired frame order changed")
        if d0[key] is not None and d1[key] is not None:
            values.append(d1[key] - d0[key])
    difference = np.asarray(values, dtype=np.float64)
    if not len(difference):
        return {"defined_pairs": 0, "d1_gt_d0": 0, "d1_lt_d0": 0, "equal": 0,
                "mean_d1_minus_d0": None, "median_d1_minus_d0": None}
    return {"defined_pairs": len(difference),
            "d1_gt_d0": int(np.count_nonzero(difference > 1e-12)),
            "d1_lt_d0": int(np.count_nonzero(difference < -1e-12)),
            "equal": int(np.count_nonzero(np.abs(difference) <= 1e-12)),
            "mean_d1_minus_d0": float(difference.mean()),
            "median_d1_minus_d0": float(np.median(difference))}


def analyze(target: np.ndarray, predictions: dict[str, np.ndarray]) -> tuple[dict, list[dict]]:
    """HH_260929 - Compare exact aligned DEV masks; never inspect independent TEST."""
    verified.require(target.ndim == 3 and target.dtype == np.uint8
                     and target.shape[0] > 0 and int(target.max()) < len(verified.LABELS),
                     "uint8 semantic target required")
    verified.require(set(predictions) == {"D0", "D1"}
                     and all(pred.shape == target.shape and pred.dtype == np.uint8
                             and int(pred.max()) < len(verified.LABELS)
                             for pred in predictions.values()),
                     "aligned D0/D1 semantic predictions required")
    count, height, width = target.shape
    slices = region_slices(height, width)
    rows: list[dict] = []
    by_arm_region = {(arm, region): [] for arm in ("D0", "D1") for region in REGIONS}
    for index in range(count):
        for arm in ("D0", "D1"):
            for region, (ys, xs) in slices.items():
                row = frame_row(index, arm, region,
                                target[index, ys, xs], predictions[arm][index, ys, xs])
                rows.append(row)
                by_arm_region[(arm, region)].append(row)
    summary = {
        "schema": "portable_e2e.map_road_lane_ab_town05_error_analysis.v1",
        "split": "Town05_Opt DEV_ONLY", "independent_test_used": False,
        "causal_attribution": False, "frame_count": count,
        "region_xyxy": {
            "full": [0, 0, width, height], "far": [0, 0, width, height // 2],
            "near_full": [0, height // 2, width, height],
            "near_ego": [width // 4, height // 2, 3 * width // 4, height],
        },
        "arms": {arm: {region: aggregate(by_arm_region[(arm, region)])
                       for region in REGIONS} for arm in ("D0", "D1")},
        "paired_frame_deltas": {
            region: {key: paired_delta(by_arm_region[("D0", region)],
                                       by_arm_region[("D1", region)], key)
                     for key in ("lane_iou", "lane_recall", "road_fp_nonroad_rate")}
            for region in REGIONS
        },
    }
    return summary, rows


def write_outputs(output: Path, summary: dict, rows: list[dict]) -> dict:
    """HH_260929 - Preserve prior evidence by requiring a brand-new output directory."""
    verified.require(not output.exists() and not output.is_symlink(),
                     "fresh error-analysis output required")
    verified.require(output.parent.is_dir() and not output.parent.is_symlink(),
                     "existing regular output parent required")
    output.mkdir(exist_ok=False)
    summary_path = output / "summary.json"
    per_frame_path = output / "per_frame.csv"
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n",
                            encoding="utf-8")
    with per_frame_path.open("x", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    return {"summary_sha256": verified.digest(summary_path),
            "per_frame_sha256": verified.digest(per_frame_path)}


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, default=verified.DEFAULT_DATA)
    parser.add_argument("--run-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args(argv)
    verified.require(not args.output_dir.exists() and not args.output_dir.is_symlink(),
                     "fresh error-analysis output required")
    verified.require(not args.output_dir.resolve().is_relative_to(args.data.resolve())
                     and not args.output_dir.resolve().is_relative_to(args.run_root.resolve()),
                     "error-analysis output must be outside source data and run")
    rgb, target, _, data_hashes = verified.load_dev(args.data)
    del rgb
    predictions, run_hashes = verified.load_run(args.run_root, target,
                                                data_hashes["manifest"])
    summary, rows = analyze(target, predictions)
    summary["provenance"] = {"data": data_hashes, "run": run_hashes,
                             "analyzer_sha256": verified.digest(Path(__file__))}
    hashes = write_outputs(args.output_dir, summary, rows)
    print(json.dumps({"status": "DEV_ERROR_ANALYSIS_ONLY", "output": str(args.output_dir),
                      "frame_rows": len(rows), **hashes}, sort_keys=True))


if __name__ == "__main__":
    main()
