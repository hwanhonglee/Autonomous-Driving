#!/usr/bin/env python3
"""HH_260929 - Replay frozen map predictions by image region without training."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
DATA = ROOT / "storage/experiments/training/2026-09-29/map_road_lane_ab_data_v2"
BASELINE = ROOT / "storage/experiments/training/2026-09-29/map_road_lane_ab_run_v1/D1"
CANDIDATE = ROOT / "docs/assets/validation/2026-09-29/map_photometric_pilot/remote_run_v1"
LABELS = ("other", "road", "sidewalk", "lane_marking",
          "traffic_light_fixture", "traffic_sign", "pole")
SITES = {"town01": 118, "town05": 232}
REGIONS = {
    "full": (slice(None), slice(None), slice(None)),
    "far": (slice(None), slice(0, 180), slice(None)),
    "near_full": (slice(None), slice(180, 360), slice(None)),
    "near_ego": (slice(None), slice(180, 360), slice(160, 480)),
}


def require(condition: bool, reason: str) -> None:
    if not condition:
        raise ValueError(reason)


def digest(path: Path) -> str:
    """HH_260929 - Bind every retrospective diagnostic to its actual bytes."""
    require(path.is_file() and not path.is_symlink(), f"regular file required: {path}")
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(block)
    return result.hexdigest()


def class_region(target: np.ndarray, prediction: np.ndarray, class_id: int) -> dict:
    """HH_260929 - Use pooled pixels; nested image regions are not independent samples."""
    require(target.shape == prediction.shape and target.ndim == 3,
            "same-shape N,H,W arrays required")
    truth = target == class_id
    found = prediction == class_id
    tp = int(np.count_nonzero(truth & found))
    fp = int(np.count_nonzero(~truth & found))
    fn = int(np.count_nonzero(truth & ~found))
    return {
        "oracle_pixels": tp + fn, "predicted_pixels": tp + fp,
        "true_positive": tp, "false_positive": fp, "false_negative": fn,
        "iou": (tp / (tp + fp + fn)) if tp + fp + fn else None,
        "recall": (tp / (tp + fn)) if tp + fn else None,
        "precision": (tp / (tp + fp)) if tp + fp else None,
    }


def confusion(target: np.ndarray, prediction: np.ndarray) -> list[list[int]]:
    require(target.shape == prediction.shape and target.ndim == 3,
            "same-shape N,H,W arrays required")
    require(target.dtype == prediction.dtype == np.uint8 and
            int(target.max()) < len(LABELS) and int(prediction.max()) < len(LABELS),
            "seven-class uint8 predictions required")
    return np.bincount((target.astype(np.int64) * len(LABELS) + prediction).ravel(),
                       minlength=len(LABELS) ** 2).reshape(len(LABELS), len(LABELS)).tolist()


def read_arrays(site: str) -> tuple[np.ndarray, np.ndarray, np.ndarray, dict]:
    """HH_260929 - Refuse any spatial claim if either published full-FOV replay differs."""
    count = SITES[site]
    paths = {
        "dev": DATA / f"dev_{site}.npz",
        "baseline_predictions": BASELINE / f"{site}_predictions.npz",
        "candidate_predictions": CANDIDATE / f"{site}_predictions.npz",
    }
    manifest = json.loads((DATA / "manifest.json").read_text(encoding="utf-8"))
    require(digest(paths["dev"]) == manifest["files"][f"dev_{site}"]["sha256"],
            "development array differs from frozen manifest")
    with np.load(paths["dev"], allow_pickle=False) as archive:
        require(set(archive.files) == {"rgb", "target"}, "DEV archive keys changed")
        target = archive["target"]
    require(target.shape == (count, 360, 640), "DEV shape changed")
    predictions = []
    for name in ("baseline_predictions", "candidate_predictions"):
        with np.load(paths[name], allow_pickle=False) as archive:
            require(set(archive.files) == {"prediction"}, "prediction keys changed")
            prediction = archive["prediction"]
        require(prediction.shape == target.shape, "prediction shape changed")
        predictions.append(prediction)
    baseline_metrics = json.loads((BASELINE / "metrics.json").read_text(encoding="utf-8"))
    candidate_metrics = json.loads((CANDIDATE / "metrics.json").read_text(encoding="utf-8"))
    for name, prediction, metrics in (
        ("baseline", predictions[0], baseline_metrics),
        ("candidate", predictions[1], candidate_metrics),
    ):
        require(confusion(target, prediction) == metrics[site]["confusion"],
                f"{name} {site} published confusion is not replayable")
    return target, predictions[0], predictions[1], {
        name: digest(path) for name, path in paths.items()}


def audit() -> dict:
    """HH_260929 - Measure spatial tradeoffs on already-exposed DEV, never promotion."""
    sites = {}
    for site in SITES:
        target, baseline, candidate, source_sha = read_arrays(site)
        regions = {}
        for region, selection in REGIONS.items():
            regions[region] = {}
            for class_id in (1, 3):
                label = LABELS[class_id]
                old = class_region(target[selection], baseline[selection], class_id)
                new = class_region(target[selection], candidate[selection], class_id)
                regions[region][label] = {
                    "baseline": old, "photometric_candidate": new,
                    "iou_delta_pp": None if old["iou"] is None or new["iou"] is None
                    else 100 * (new["iou"] - old["iou"]),
                }
        sites[site] = {"frames": SITES[site], "source_sha256": source_sha,
                       "regions": regions}
    return {
        "schema": "portable_e2e.map_photometric_spatial_diagnostic.v1",
        "status": "POST_HOC_REPEATED_DEV_DIAGNOSTIC_NOT_PROMOTED",
        "source_metrics_sha256": {
            "baseline": digest(BASELINE / "metrics.json"),
            "candidate": digest(CANDIDATE / "metrics.json"),
        },
        "regions_overlap": True, "independent_dev_count": 0,
        "test_used": False, "model_promotion": False, "sites": sites,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path,
                        help="fresh JSON artifact path; omit for stdout only")
    args = parser.parse_args()
    result = audit()
    payload = json.dumps(result, indent=2, sort_keys=True, allow_nan=False) + "\n"
    if args.output is None:
        print(payload, end="")
    else:
        require(not args.output.exists() and not args.output.is_symlink()
                and args.output.parent.is_dir() and not args.output.parent.is_symlink(),
                "new JSON artifact path required")
        with args.output.open("x", encoding="utf-8") as stream:
            stream.write(payload)
        print(args.output)


if __name__ == "__main__":
    main()
