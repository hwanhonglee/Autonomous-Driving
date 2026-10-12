#!/usr/bin/env python3
"""HH_260929 - Independently replay frozen D1/photo DEV pixels and all run hashes."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


LABELS = ("other", "road", "sidewalk", "lane_marking",
          "traffic_light_fixture", "traffic_sign", "pole")
PLAN_SHA256 = "79c14b50b268fa3a51f023baf34f602faa95cd687b7d90158911ca1372609579"
DATA_SHA256 = "313b41ed4fb897f01fb4fb433c1bafb234ef9b234e62f96a462ef2188e9dccc6"
INITIAL_STATE_SHA256 = "e78098947d72df35bcee1eb8719ad0e9b0acd67ec6a1e393197d477823da2058"


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def sha256(path: Path) -> str:
    require(path.is_file() and not path.is_symlink(), f"regular file required: {path}")
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(block)
    return result.hexdigest()


def read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def check_sum_manifest(run: Path, manifest_path: Path) -> int:
    """HH_260929 - Check the exact 15-file remote copy, not a partial subset."""
    lines = manifest_path.read_text(encoding="utf-8").splitlines()
    expected = {}
    for line in lines:
        value, relative = line.split("  ", 1)
        require(relative.startswith("remote_run_v1/"), "unexpected hash path")
        key = relative.removeprefix("remote_run_v1/")
        require(key not in expected and len(value) == 64, "duplicate or invalid hash row")
        expected[key] = value
    found = {path.relative_to(run).as_posix() for path in run.rglob("*") if path.is_file()}
    require(set(expected) == found and len(expected) == 15,
            "remote run file set changed")
    for relative, value in expected.items():
        require(sha256(run / relative) == value, f"archived bytes changed: {relative}")
    return len(expected)


def independent_metrics(target: np.ndarray, prediction: np.ndarray) -> dict:
    """HH_260929 - Recompute seven-class pixel confusion without trainer imports."""
    require(target.shape == prediction.shape and target.dtype == prediction.dtype == np.uint8
            and target.ndim == 3 and target.shape[1:] == (360, 640)
            and int(target.max()) < 7 and int(prediction.max()) < 7,
            "DEV target/prediction ABI changed")
    encoded = target.astype(np.int64) * 7 + prediction.astype(np.int64)
    confusion = np.bincount(encoded.ravel(), minlength=49).reshape(7, 7)
    intersection = np.diag(confusion)
    union = confusion.sum(0) + confusion.sum(1) - intersection
    predicted = confusion.sum(0)
    return {"confusion": confusion.tolist(),
            "iou": {name: float(intersection[i] / union[i]) if union[i] else None
                    for i, name in enumerate(LABELS)},
            "precision": {name: float(intersection[i] / predicted[i]) if predicted[i] else None
                          for i, name in enumerate(LABELS)},
            "pixel_accuracy": float(intersection.sum() / confusion.sum())}


def same_metrics(recorded: dict, replayed: dict) -> None:
    require(recorded["confusion"] == replayed["confusion"], "confusion differs")
    for field in ("iou", "precision"):
        require(set(recorded[field]) == set(replayed[field]) == set(LABELS),
                "seven-class metric keys changed")
        for name in LABELS:
            a, b = recorded[field][name], replayed[field][name]
            require(a == b or a is not None and b is not None and abs(a - b) <= 1e-12,
                    f"{field} differs: {name}")
    require(abs(recorded["pixel_accuracy"] - replayed["pixel_accuracy"]) <= 1e-12,
            "pixel accuracy differs")


def replay(data: Path, baseline: Path, run: Path, sums: Path) -> dict:
    count = check_sum_manifest(run, sums)
    plan = run / "source/plan.md"
    receipt = read_json(run / "cpu_preflight.json")
    started = read_json(run / "started.json")
    summary = read_json(run / "summary.json")
    candidate = read_json(run / "metrics.json")
    baseline_metrics = read_json(baseline / "D1/metrics.json")
    baseline_summary = read_json(baseline / "summary.json")
    require(sha256(plan) == PLAN_SHA256
            and receipt == read_json(run / "source/cpu_preflight_receipt.json")
            and receipt["plan_sha256"] == PLAN_SHA256
            and started["plan_sha256"] == PLAN_SHA256
            and receipt["status"] == "CPU_PREFLIGHT_PASS"
            and started["initial_state_sha256"] == INITIAL_STATE_SHA256
            and sha256(run / "source/train_map_photometric_pilot.py") == receipt["runner_sha256"]
            and started["runner_sha256"] == receipt["runner_sha256"],
            "frozen pilot source/preflight contract changed")
    for relative, expected in receipt["baseline_file_sha256"].items():
        require(sha256(baseline / relative) == expected, "frozen D1 baseline bytes changed")
    require(baseline_summary["D1"] == baseline_metrics
            and summary["baseline"]["file_sha256"] == receipt["baseline_file_sha256"]
            and summary["candidate"] == candidate,
            "frozen baseline/candidate metadata differs")
    for relative, expected in summary["file_sha256"].items():
        require(sha256(run / relative) == expected, f"pilot file hash differs: {relative}")
    require(candidate["checkpoint_sha256"] == sha256(run / "checkpoint.pt")
            and candidate["train_count"] == 729
            and candidate["policy_total_samples"] == 16000
            and started["dev_arrays_opened_before_checkpoint"] is False
            and summary["test_used"] is False
            and summary["model_promotion"] is False,
            "pilot completion contract differs")
    history = [json.loads(line) for line in
               (run / "history.jsonl").read_text(encoding="utf-8").splitlines()]
    require([row["step"] for row in history] == list(range(100, 2001, 100))
            and history[-1]["policy_enabled_samples"] == candidate["policy_enabled_samples"],
            "two-thousand-step history incomplete")
    require(sha256(data / "manifest.json") == DATA_SHA256
            and sha256(run / "data_manifest.json") == DATA_SHA256,
            "dataset manifest differs")
    data_manifest = read_json(data / "manifest.json")
    replayed = {}
    for split, town in (("dev_town01", "town01"), ("dev_town05", "town05")):
        ref = data_manifest["files"][split]
        path = data / ref["file"]
        require(sha256(path) == ref["sha256"], f"frozen {town} DEV bytes changed")
        with np.load(path, allow_pickle=False) as archive:
            require(set(archive.files) == {"rgb", "target"}, "DEV archive keys changed")
            target = archive["target"]
        require(target.shape[0] == ref["count"], "DEV count changed")
        replayed[town] = {}
        for arm, prediction_path, metrics in (
                ("frozen_d1", baseline / f"D1/{town}_predictions.npz", baseline_metrics),
                ("photo", run / f"{town}_predictions.npz", candidate)):
            with np.load(prediction_path, allow_pickle=False) as archive:
                require(set(archive.files) == {"prediction"}, "prediction keys changed")
                prediction = archive["prediction"]
            result = independent_metrics(target, prediction)
            same_metrics(metrics[town], result)
            require(summary["baseline"][town] == metrics[town] if arm == "frozen_d1"
                    else summary["candidate"][town] == metrics[town],
                    "summary metrics differ")
            replayed[town][arm] = result
    deltas = {town: {name: (None if replayed[town]["frozen_d1"]["iou"][name] is None
                                or replayed[town]["photo"]["iou"][name] is None
                                else replayed[town]["photo"]["iou"][name]
                                - replayed[town]["frozen_d1"]["iou"][name])
                     for name in LABELS} for town in ("town01", "town05")}
    checks = {"town05_lane_plus_5pp_vs_d1": deltas["town05"]["lane_marking"] >= 0.05,
              "town05_road_drop_at_most_1pp_vs_d1": deltas["town05"]["road"] >= -0.01,
              "town01_road_drop_at_most_1pp_vs_d1": deltas["town01"]["road"] >= -0.01,
              "town01_lane_drop_at_most_1pp_vs_d1": deltas["town01"]["lane_marking"] >= -0.01}
    require(summary["gate"]["checks"] == checks
            and summary["gate"]["passed_exploratory_only"] == all(checks.values())
            and summary["gate"]["iou_delta_vs_frozen_d1"] == deltas,
            "preregistered diagnostic gate differs")
    return {"status": "INDEPENDENT_D1_AND_PHOTO_DEV_PIXEL_REPLAY_PASS",
            "run_files_sha_verified": count, "dev_confusions_replayed": 4,
            "all_seven_classes_replayed": True, "gate": checks,
            "iou_delta_vs_frozen_d1": deltas, "test_used": False,
            "model_promotion": False}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    for name in ("data", "baseline", "run", "sums"):
        parser.add_argument("--" + name, required=True, type=Path)
    args = parser.parse_args()
    print(json.dumps(replay(args.data, args.baseline, args.run, args.sums),
                     sort_keys=True, allow_nan=False))


if __name__ == "__main__":
    main()
