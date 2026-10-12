"""HH_260906 - Diagnose Town02 p02 sign identity with native CARLA depth and poses."""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
from pathlib import Path

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[2]
SOURCE_FILE = ROOT / "tools/e2e/audit_map_town02_p02_objects.py"
SPEC = importlib.util.spec_from_file_location("_p02_object_inventory", SOURCE_FILE)
assert SPEC is not None and SPEC.loader is not None
objects = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(objects)
OUTPUT = objects.OUTPUT / "world_geometry.json"
FOV_DEG = 90.0
FOCAL_PX = objects.WIDTH / (2 * math.tan(math.radians(FOV_DEG) / 2))
MAX_DEPTH_M = 1000.0


def world_centroid(depth_bgra: np.ndarray, camera_to_world: np.ndarray,
                   ys: np.ndarray, xs: np.ndarray) -> dict:
    """HH_260906 - Apply the archived CARLA BGRA optical-depth projection exactly."""
    objects.require(depth_bgra.shape == (objects.HEIGHT, objects.WIDTH, 4)
                    and depth_bgra.dtype == np.uint8, "native depth contract differs")
    matrix = np.asarray(camera_to_world, dtype=np.float64)
    objects.require(matrix.shape == (4, 4) and np.isfinite(matrix).all()
                    and np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-6)
                    and np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-5)
                    and abs(np.linalg.det(matrix[:3, :3]) - 1.0) <= 1e-5,
                    "camera-to-world transform is not rigid")
    sampled = depth_bgra[ys, xs].astype(np.float64)
    meters = (sampled[:, 2] + sampled[:, 1] * 256 + sampled[:, 0] * 65536)
    meters *= 1000.0 / 16777215.0
    valid = np.isfinite(meters) & (meters > 0.1) & (meters < MAX_DEPTH_M)
    invalid = int((~valid).sum())
    if not valid.any():
        return {"valid_depth_pixels": 0, "invalid_depth_pixels": invalid,
                "median_depth_m": None, "world_median_xyz": None,
                "world_xy_p95_radius_m": None}
    z = meters[valid]
    u = xs[valid]
    v = ys[valid]
    optical = np.stack([z, (u - objects.WIDTH / 2) * z / FOCAL_PX,
                        -(v - objects.HEIGHT / 2) * z / FOCAL_PX,
                        np.ones_like(z)])
    world = (matrix @ optical).T[:, :3]
    median = np.median(world, axis=0)
    radius = np.linalg.norm(world[:, :2] - median[:2], axis=1)
    return {"valid_depth_pixels": int(valid.sum()),
            "invalid_depth_pixels": invalid,
            "median_depth_m": round(float(np.median(z)), 6),
            "world_median_xyz": [round(float(x), 6) for x in median],
            "world_xy_p95_radius_m": round(float(np.percentile(radius, 95)), 6)}


def build_geometry() -> dict:
    """HH_260906 - Bind 3D diagnostics to the immutable all-frame component inventory."""
    inventory = objects.build_inventory()
    by_stem: dict[str, list[dict]] = {}
    for row in inventory["all_components"]:
        by_stem.setdefault(row["stem"], []).append(row)
    records = []
    for stem in sorted(by_stem):
        with np.load(objects.SOURCE / (stem + ".npz"), allow_pickle=False) as sensor:
            semantic = sensor["semantic"]
            depth = sensor["depth"]
            matrix = sensor["world_from_camera"]
            labels = cv2.connectedComponents((semantic[..., 2] == objects.TAG).astype(np.uint8),
                                              connectivity=8)[1]
            for row in by_stem[stem]:
                ys, xs = np.where(labels == row["label"])
                objects.require(len(xs) == row["pixels"],
                                f"component label changed: {row['id']}")
                records.append({"id": row["id"], "stem": stem, "pixels": row["pixels"],
                                "provisional_2d_track_id": row["provisional_track_id"],
                                **world_centroid(depth, matrix, ys, xs)})
    objects.require(len(records) == inventory["component_count"]
                    and sum(row["pixels"] for row in records)
                    == inventory["retained_tag8_pixels"],
                    "geometry lost source components")
    by_id = {row["id"]: row for row in records}
    anchors = ["0015_c01", "0016_c01", "0066_c01", "0082_c01", "0083_c01",
               "0088_c02", "0089_c01", "0239_c01"]
    # HH_260906 - Anchors are diagnostic examples, not a learned or automatically certified object list.
    examples = {key: by_id[key] for key in anchors}
    early_left = np.array(examples["0016_c01"]["world_median_xyz"][:2])
    front_thirty = np.array(examples["0083_c01"]["world_median_xyz"][:2])
    later_left = np.array(examples["0089_c01"]["world_median_xyz"][:2])
    pairwise = {
        "early_left_to_front_30_m": round(float(np.linalg.norm(early_left - front_thirty)), 6),
        "early_left_to_later_left_m": round(float(np.linalg.norm(early_left - later_left)), 6),
        "front_30_to_later_left_m": round(float(np.linalg.norm(front_thirty - later_left)), 6),
        "early_left_two_frame_drift_m": round(float(np.linalg.norm(
            np.array(examples["0015_c01"]["world_median_xyz"][:2]) - early_left)), 6),
        "front_30_two_frame_drift_m": round(float(np.linalg.norm(
            np.array(examples["0082_c01"]["world_median_xyz"][:2]) - front_thirty)), 6),
        "later_left_two_frame_drift_m": round(float(np.linalg.norm(
            np.array(examples["0088_c02"]["world_median_xyz"][:2]) - later_left)), 6),
    }
    return {
        "schema": "portable_e2e.town02_p02_sign_world_geometry.v1",
        "component_inventory_sha256": objects.digest(objects.OUTPUT / "component_inventory.json"),
        "source": str(objects.SOURCE.relative_to(ROOT)),
        "projection": "CARLA BGRA optical-axis depth; UE forward/right/up; native rigid world_from_camera",
        "camera_wh": [objects.WIDTH, objects.HEIGHT],
        "horizontal_fov_deg": FOV_DEG,
        "focal_px": FOCAL_PX,
        "invalid_depth_rule": "nonfinite, <=0.1m or >=1000m excluded",
        "component_count": len(records),
        "total_component_pixels": sum(row["pixels"] for row in records),
        "invalid_depth_pixels": sum(row["invalid_depth_pixels"] for row in records),
        "zero_valid_depth_components": sum(row["valid_depth_pixels"] == 0 for row in records),
        "anchors": examples,
        "anchor_distances_and_static_drift": pairwise,
        "all_component_world_centroids": records,
        "geometry_alone_certifies_rgb_sign_object": False,
        "geometry_alone_certifies_95pct_visual_review": False,
        "dataset_admitted": False,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--write", action="store_true")
    args = parser.parse_args()
    report = build_geometry()
    payload = (json.dumps(report, indent=2, ensure_ascii=False) + "\n").encode()
    if args.write:
        objects.require(not OUTPUT.exists(), "world geometry artifact already exists")
        OUTPUT.write_bytes(payload)
    else:
        objects.require(OUTPUT.read_bytes() == payload, "world geometry differs")
    print(json.dumps({key: report[key] for key in (
        "component_count", "invalid_depth_pixels", "zero_valid_depth_components",
        "anchor_distances_and_static_drift")}, ensure_ascii=False))


if __name__ == "__main__":
    main()
