#!/usr/bin/env python3
"""HH_261001 - Measure TRAIN-only raw lane support without fitting or selecting frames."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "docs/assets/validation/2026-10-01/map_unused_route_inventory/audit_v1/candidate_manifest.json"
MANIFEST_SHA256 = "3c89ab9516e032122728d01bd72b1f9e5c31b5e287253e292143aa67e06382a6"
AUTHORITY = ROOT / "docs/assets/validation/2026-10-01/map_unused_route_inventory/package_authorization.json"
AUTHORITY_SHA256 = "844bbc685eb0e49c427687d53432d622d3d8123e519e870423c460375bdfe43e"
ROUTES = ("p02", "p04")


def digest(path: Path) -> str:
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(block)
    return result.hexdigest()


def lane_neighborhood_histogram(lane: np.ndarray) -> list[int]:
    """HH_261001 - Count same-class pixels in a 3x3 window at each lane pixel."""
    if lane.ndim != 2 or lane.dtype != np.bool_:
        raise ValueError("A two-dimensional boolean lane mask is required")
    padded = np.pad(lane, 1, constant_values=False)
    neighbors = np.zeros(lane.shape, dtype=np.uint8)
    for y in range(3):
        for x in range(3):
            neighbors += padded[y:y + lane.shape[0], x:x + lane.shape[1]]
    return np.bincount(neighbors[lane], minlength=10).astype(int).tolist()


def main() -> None:
    if digest(MANIFEST) != MANIFEST_SHA256:
        raise ValueError("Immutable candidate manifest differs")
    if digest(AUTHORITY) != AUTHORITY_SHA256:
        raise ValueError("Study-specific TRAIN authorization differs")
    authorization = json.loads(AUTHORITY.read_text(encoding="utf-8"))
    if (authorization.get("candidate_manifest_sha256") != MANIFEST_SHA256
            or authorization.get("study_train_authorized") is not True
            or authorization.get("ctrack_train_eligible") is not False):
        raise ValueError("Original raw source not authorized for the geometry study")
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    if tuple(manifest["captures"]) != ROUTES:
        raise ValueError("Route set or order changed")
    result = {"schema": "portable_e2e.map_train_lane_support_readonly.v1",
              "manifest_sha256": MANIFEST_SHA256,
              "geometry_study_authorization_sha256": AUTHORITY_SHA256,
              "scope": "approved geometry-study TRAIN source only; no DEV, CTrack, or model inference",
              "meaning": "9-neighborhood describes 3x3 class support, not physical paint width",
              "routes": {}}
    for route in ROUTES:
        rows = manifest["captures"][route]["frames"]
        if len(rows) != 240:
            raise ValueError(f"Incomplete route: {route}")
        hist = np.zeros(10, dtype=np.int64)
        positive = 0
        for index, row in enumerate(rows):
            if row["capture_index"] != index or row["proposed_include"] is not True:
                raise ValueError("Frame order or complete-inclusion rule changed")
            path = ROOT / row["sensor_path"]
            if not path.is_file() or path.is_symlink() or digest(path) != row["sensor_sha256"]:
                raise ValueError(f"Raw source changed: {path}")
            with np.load(path, allow_pickle=False) as archive:
                if set(archive.files) != {"semantic", "depth", "world_from_camera"}:
                    raise ValueError("Raw sensor archive ABI changed")
                tags = archive["semantic"]
                if tags.shape != (360, 640, 4) or tags.dtype != np.uint8:
                    raise ValueError("Raw semantic image ABI changed")
                lane = tags[..., 2] == 24
                count = int(lane.sum())
                if count != row["class_pixels"][3]:
                    raise ValueError("Raw lane support differs from frozen frame manifest")
                positive += int(count > 0)
                hist += lane_neighborhood_histogram(lane)
        total = int(hist.sum())
        expected = manifest["captures"][route]["class_pixels_all240"]["lane_marking"]
        if total != expected or positive != manifest["captures"][route]["class_positive_frames_all240"]["lane_marking"]:
            raise ValueError("Complete-route lane support differs from frozen manifest")
        result["routes"][route] = {"frames": len(rows), "positive_frames": positive,
                                   "lane_pixels": total,
                                   "same_lane_3x3_neighbor_count_0_to_9": hist.tolist(),
                                   "without_complete_3x3_lanes": int(sum(hist[:9])),
                                   "without_complete_3x3_fraction": float(sum(hist[:9]) / total)}
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
