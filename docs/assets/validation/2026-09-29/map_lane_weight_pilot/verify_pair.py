#!/usr/bin/env python3
"""HH_260929 - Replay the first TRAIN-route holdout confusion without DEV/TEST access."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[5]
DATA = ROOT / "storage/experiments/training/2026-09-29/map_road_lane_ab_data_v2"
RUN = Path(__file__).resolve().parent / "remote_run_v1/pair_00_historical_20260922"
DATA_SHA256 = "313b41ed4fb897f01fb4fb433c1bafb234ef9b234e62f96a462ef2188e9dccc6"
D0_SHA256 = "69dae433616cf27690310af9ff1923ef3b2e67641fa935e2a2f77116eea02ace"
LABELS = ("other", "road", "sidewalk", "lane_marking",
          "traffic_light_fixture", "traffic_sign", "pole")


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


def read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def replay() -> dict:
    """HH_260929 - Recount all 114 heldout TRAIN-route frames and source hashes."""
    summary = read_json(RUN / "summary.json")
    require(summary["status"] == "PAIR_FAIL_STOP"
            and summary["pair"] == {"pair_index": 0, "fold": "historical",
                                    "holdout_start": 0, "holdout_end": 114,
                                    "seed": 20260922}
            and summary["data_manifest_sha256"] == DATA_SHA256
            and summary["model_promotion"] is False
            and summary["test_used"] is False,
            "first pair metadata changed")
    require(digest(DATA / "manifest.json") == DATA_SHA256
            and digest(DATA / "train_d0.npz") == D0_SHA256,
            "frozen TRAIN bytes changed")
    for file, expected in summary["file_sha256"].items():
        require(digest(RUN / file) == expected, f"archived pair file changed: {file}")
    with np.load(DATA / "train_d0.npz", allow_pickle=False) as archive:
        target = archive["target"][:114].copy()
    require(target.shape == (114, 360, 640)
            and target.dtype == np.uint8, "heldout TRAIN target ABI changed")
    checked = {}
    for arm in ("B", "W2"):
        with np.load(RUN / f"{arm}_holdout_predictions.npz", allow_pickle=False) as archive:
            require(set(archive.files) == {"prediction"}, "prediction keys changed")
            prediction = archive["prediction"]
        require(prediction.shape == target.shape and prediction.dtype == np.uint8
                and int(prediction.max()) < 7, "prediction ABI changed")
        confusion = np.bincount((target.astype(np.int64) * 7 + prediction).ravel(),
                                minlength=49).reshape(7, 7)
        metrics = read_json(RUN / f"{arm}_metrics.json")
        require(confusion.tolist() == metrics["confusion"],
                f"{arm} archived confusion differs")
        diagonal = np.diag(confusion)
        union = confusion.sum(0) + confusion.sum(1) - diagonal
        iou = {name: float(diagonal[index] / union[index]) if union[index] else None
               for index, name in enumerate(LABELS)}
        require(iou == metrics["iou"] == summary[("baseline_iou" if arm == "B"
                                                 else "candidate_iou")],
                f"{arm} IoU differs")
        require(digest(RUN / f"{arm}_checkpoint.pt") == metrics["checkpoint_sha256"]
                and digest(RUN / f"{arm}_holdout_predictions.npz")
                == metrics["prediction_sha256"],
                f"{arm} model or predictions differ")
        checked[arm] = {"road_iou": iou["road"],
                        "lane_iou": iou["lane_marking"],
                        "initial_state_sha256": metrics["initial_state_sha256"]}
    require(checked["B"]["initial_state_sha256"]
            == checked["W2"]["initial_state_sha256"]
            == summary["initial_state_sha256"], "paired initialization differs")
    lane_delta = checked["W2"]["lane_iou"] - checked["B"]["lane_iou"]
    road_delta = checked["W2"]["road_iou"] - checked["B"]["road_iou"]
    require(lane_delta == summary["gate"]["lane_iou_delta"]
            and road_delta == summary["gate"]["road_iou_delta"]
            and summary["gate"]["passed"] is False
            and lane_delta < 0.05 and road_delta >= -0.01,
            "first pair stop gate differs")
    return {"status": "INDEPENDENT_TRAIN_HOLDOUT_PIXEL_REPLAY_PASS",
            "heldout_frames": 114, "arm": checked,
            "lane_iou_delta": lane_delta, "road_iou_delta": road_delta,
            "next_pair_allowed": False, "dev_arrays_opened": False,
            "test_used": False, "model_promotion": False}


if __name__ == "__main__":
    print(json.dumps(replay(), sort_keys=True))
