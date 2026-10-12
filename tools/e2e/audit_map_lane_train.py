#!/usr/bin/env python3
"""HH_260929 - Read-only, source-bound audit of the frozen D0/D1 TRAIN pixels."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image


ROOT = Path(__file__).resolve().parents[2]
DATA = ROOT / "storage/experiments/training/2026-09-29/map_road_lane_ab_data_v2"
PARENT = ROOT / "storage/datasets/prepared/map_expanded_20260926_ai_v2/manifest.json"
TOWN02_AUDIT = ROOT / "docs/assets/validation/2026-09-27/map_town02_p02_train_scout/summary.json"
TOWN02_SOURCE = ROOT / "storage/experiments/training/2026-09-27/map_town02_p02_sign_train_scout_epic_v1/mapping"
DATA_MANIFEST_SHA256 = "313b41ed4fb897f01fb4fb433c1bafb234ef9b234e62f96a462ef2188e9dccc6"
PARENT_SHA256 = "e9805574a252e72f8e81fcd434156da7782b25e8e965b8078f976cf4f234892c"
TOWN02_AUDIT_SHA256 = "d2dd43837eeef50c9ab71106b6e67a926802fada111d735de2b859c51d1a9d49"
LABELS = ("other", "road", "sidewalk", "lane_marking", "traffic_light_fixture", "traffic_sign", "pole")
TAG_TO_INDEX = {1: 1, 2: 2, 24: 3, 7: 4, 8: 5, 6: 6}
CLASS_INDEX = {"road": 1, "lane_marking": 3}
DEPTH_EDGES_M = (0.1, 10.0, 20.0, 40.0, 80.0, 160.0, 1000.0)


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def digest(path: Path) -> str:
    """HH_260929 - Refuse links and stream each immutable source byte check."""
    require(path.is_file() and not path.is_symlink(), f"regular source required: {path}")
    sha = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            sha.update(block)
    return sha.hexdigest()


def read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def target_from_tags(tags: np.ndarray) -> np.ndarray:
    """HH_260929 - Unknown CARLA tags remain other, as in the frozen builder."""
    lut = np.zeros(256, dtype=np.uint8)
    for tag, index in TAG_TO_INDEX.items():
        lut[tag] = index
    return lut[tags]


def optical_depth_m(depth_bgra: np.ndarray, mask: np.ndarray) -> np.ndarray:
    """HH_260929 - Decode CARLA BGRA optical depth only at chosen oracle pixels."""
    sample = depth_bgra[mask].astype(np.uint32)
    return (sample[:, 2] + sample[:, 1] * 256 + sample[:, 0] * 65536) * (1000.0 / 16777215.0)


def load_train(path: Path, expected_sha: str, count: int) -> tuple[np.ndarray, np.ndarray]:
    require(digest(path) == expected_sha, f"frozen split bytes changed: {path.name}")
    with np.load(path, allow_pickle=False) as archive:
        require(set(archive.files) == {"rgb", "target"}, "TRAIN array keys changed")
        rgb, target = archive["rgb"], archive["target"]
    require(rgb.shape == (count, 360, 640, 3)
            and target.shape == (count, 360, 640)
            and rgb.dtype == target.dtype == np.uint8
            and int(target.max()) < len(LABELS), "TRAIN array shape or dtype changed")
    return rgb, target


def source_root_for_record(row: dict, historical: Path) -> Path:
    """HH_260929 - AI-v2's first 114 records predate source_collection fields."""
    value = row.get("source_collection")
    return historical if value is None else ROOT / value


def validate_sensor_pair(rgb: np.ndarray, target: np.ndarray, raw_rgb: Path,
                         raw_sensor: Path, rgb_sha: str, sensor_sha: str) -> tuple[np.ndarray, np.ndarray, float]:
    """HH_260929 - Bind packaged frame to both raw source bytes and semantic tags."""
    require(digest(raw_rgb) == rgb_sha and digest(raw_sensor) == sensor_sha,
            f"raw sensor hash changed: {raw_rgb}")
    with Image.open(raw_rgb) as image:
        require(image.mode == "RGB" and image.size == (640, 360), "raw RGB ABI changed")
        require(np.array_equal(np.asarray(image), rgb), "packaged RGB differs from source")
    with np.load(raw_sensor, allow_pickle=False) as archive:
        require(set(archive.files) == {"depth", "semantic", "world_from_camera"},
                "raw sensor keys changed")
        depth, semantic, camera = (archive[key] for key in
                                   ("depth", "semantic", "world_from_camera"))
    require(depth.shape == semantic.shape == (360, 640, 4)
            and depth.dtype == semantic.dtype == np.uint8
            and camera.shape == (4, 4) and np.isfinite(camera).all()
            and np.allclose(camera[3], [0, 0, 0, 1], atol=1e-6)
            and np.allclose(camera[:3, :3].T @ camera[:3, :3], np.eye(3), atol=1e-5)
            and abs(float(np.linalg.det(camera[:3, :3])) - 1.0) <= 1e-5,
            "raw sensor ABI or camera transform changed")
    require(np.array_equal(target_from_tags(semantic[..., 2]), target),
            "packaged target differs from raw semantic tags")
    return depth, semantic[..., 2], float(camera[2, 3])


class Totals:
    """HH_260929 - Accumulate fixed-bin TRAIN diagnostics without retaining raw frames."""

    def __init__(self) -> None:
        self.frames = 0
        self.class_pixels = np.zeros(7, dtype=np.int64)
        self.positive_frames = np.zeros(7, dtype=np.int64)
        self.grid_yx_class = np.zeros((3, 3, 7), dtype=np.int64)
        self.near_ego_class = np.zeros(7, dtype=np.int64)
        self.raw_tag_pixels = np.zeros(256, dtype=np.int64)
        self.frame_class_pixels = {key: [] for key in CLASS_INDEX}
        self.rgb_sum = {key: np.zeros(3, dtype=np.int64) for key in CLASS_INDEX}
        self.luma_bins = {key: np.zeros(4, dtype=np.int64) for key in CLASS_INDEX}
        self.saturation_bins = {key: np.zeros(3, dtype=np.int64) for key in CLASS_INDEX}
        self.depth_bins = {key: np.zeros(7, dtype=np.int64) for key in CLASS_INDEX}
        self.camera_z_m = []

    def add(self, rgb: np.ndarray, target: np.ndarray, depth: np.ndarray,
            tags: np.ndarray, camera_z_m: float) -> None:
        self.frames += 1
        counts = np.bincount(target.ravel(), minlength=7)
        self.class_pixels += counts
        self.positive_frames += counts > 0
        self.raw_tag_pixels += np.bincount(tags.ravel(), minlength=256)
        for yi, (y0, y1) in enumerate(((0, 120), (120, 240), (240, 360))):
            for xi, (x0, x1) in enumerate(((0, 213), (213, 427), (427, 640))):
                self.grid_yx_class[yi, xi] += np.bincount(
                    target[y0:y1, x0:x1].ravel(), minlength=7)
        self.near_ego_class += np.bincount(target[180:360, 160:480].ravel(), minlength=7)
        self.camera_z_m.append(camera_z_m)
        for key, index in CLASS_INDEX.items():
            self.frame_class_pixels[key].append(int(counts[index]))
            mask = target == index
            colors = rgb[mask].astype(np.int32)
            self.rgb_sum[key] += colors.sum(axis=0, dtype=np.int64)
            luma = (77 * colors[:, 0] + 150 * colors[:, 1] + 29 * colors[:, 2]) >> 8
            self.luma_bins[key] += np.bincount(np.minimum(luma // 64, 3), minlength=4)
            saturation = colors.max(axis=1) - colors.min(axis=1)
            self.saturation_bins[key] += np.bincount(
                np.digitize(saturation, (32, 64)), minlength=3)
            meters = optical_depth_m(depth, mask)
            valid = (meters > 0.1) & (meters < 1000.0)
            self.depth_bins[key][0] += int((~valid).sum())
            self.depth_bins[key][1:] += np.histogram(meters[valid], bins=DEPTH_EDGES_M)[0]

    def report(self) -> dict:
        require(self.frames > 0, "empty TRAIN group")
        classes = {}
        for key, index in CLASS_INDEX.items():
            values = np.asarray(self.frame_class_pixels[key])
            pixels = int(self.class_pixels[index])
            classes[key] = {
                "pixels": pixels,
                "frame_positive": int(self.positive_frames[index]),
                "per_frame_median_pixels": float(np.median(values)),
                "per_frame_p10_p90_pixels": [float(np.percentile(values, q)) for q in (10, 90)],
                "rgb_mean": [round(float(v / pixels), 3) for v in self.rgb_sum[key]],
                "luma_0_63_64_127_128_191_192_255": self.luma_bins[key].tolist(),
                "rgb_range_0_31_32_63_64_255": self.saturation_bins[key].tolist(),
                "depth_invalid_0p1_10_20_40_80_160_1000m": self.depth_bins[key].tolist(),
                "grid_yx_pixels": self.grid_yx_class[:, :, index].tolist(),
                "near_ego_pixels": int(self.near_ego_class[index]),
            }
            require(sum(classes[key]["depth_invalid_0p1_10_20_40_80_160_1000m"]) == pixels,
                    "depth histogram lost class pixels")
        return {
            "frames": self.frames,
            "class_pixels": self.class_pixels.tolist(),
            "class_positive_frames": self.positive_frames.tolist(),
            "class_fraction": [round(float(v / (self.frames * 360 * 640)), 8)
                               for v in self.class_pixels],
            "raw_tag_pixels_nonzero": {str(i): int(v) for i, v in enumerate(self.raw_tag_pixels) if v},
            "camera_world_z_median_min_max": [round(float(f(self.camera_z_m)), 4)
                                             for f in (np.median, np.min, np.max)],
            "road_lane": classes,
        }


def audit() -> dict:
    """HH_260929 - Consume only pinned TRAIN inputs; never open DEV arrays or TEST."""
    manifest_path = DATA / "manifest.json"
    require(digest(manifest_path) == DATA_MANIFEST_SHA256
            and digest(PARENT) == PARENT_SHA256
            and digest(TOWN02_AUDIT) == TOWN02_AUDIT_SHA256,
            "frozen TRAIN provenance document changed")
    manifest, parent, town02 = (read_json(path) for path in
                                (manifest_path, PARENT, TOWN02_AUDIT))
    require(manifest["schema"] == "portable_e2e.map_road_lane_ab_data.v1"
            and tuple(manifest["labels"]) == tuple(parent["labels"]) == LABELS
            and manifest["parent_manifest_sha256"] == PARENT_SHA256
            and manifest["town02_split"] == "TRAIN_EXPLORATORY_ROAD_LANE_ONLY"
            and manifest["town05_split"] == "DEV_ONLY"
            and manifest["split_contract"]["dev_used_for_training"] is False
            and manifest["split_contract"]["test_used"] is False,
            "TRAIN-only package contract changed")
    files = manifest["files"]
    require(files["train_d0"]["count"] == 497 and files["train_d1"]["count"] == 729
            and files["train_d0"]["sha256"] == parent["splits"]["train"]["sha256"]
            and files["dev_town01"]["sha256"] == parent["splits"]["dev"]["sha256"],
            "fixed Town01 source split changed")
    d0_rgb, d0_target = load_train(DATA / files["train_d0"]["file"],
                                   files["train_d0"]["sha256"], 497)
    d1_rgb, d1_target = load_train(DATA / files["train_d1"]["file"],
                                   files["train_d1"]["sha256"], 729)
    require(np.array_equal(d1_rgb[:497], d0_rgb)
            and np.array_equal(d1_target[:497], d0_target), "D1 prefix differs from D0")
    for name, target in (("train_d0", d0_target), ("train_d1", d1_target)):
        require(np.bincount(target.ravel(), minlength=7).tolist()
                == files[name]["class_pixels"], f"{name} class pixels changed")
    source_rows = parent["splits"]["train"]["source_records"]
    source_hashes = parent["splits"]["train"]["source_hashes"]
    require(len(source_rows) == 497 and len(source_hashes) == 994,
            "Town01 per-frame source ledger changed")
    explicit_roots = {row["source_collection"] for row in source_rows
                      if "source_collection" in row}
    all_roots = {str(Path(path).parent.parent) for path in source_hashes}
    historical_roots = all_roots - explicit_roots
    require(len(historical_roots) == 1, "historical Town01 source is ambiguous")
    historical = ROOT / historical_roots.pop()
    require(town02["status"] == "REJECTED_SMALL_SIGN_TRAIN_CANDIDATE"
            and town02["provenance"]["town"] == "Town02_Opt"
            and town02["retained_frames_0p25m"] == 232
            and town02["small_sign_gate_passed"] is False,
            "Town02 source scope changed")
    pairs = manifest["provenance"]["town02_retained_raw_pairs"]
    require(len(pairs) == 232
            and [row["stem"] for row in pairs] == town02["retained_stems"],
            "Town02 retention order changed")
    by_stem = {row["stem"]: row for row in town02["frames"]}
    groups = {key: Totals() for key in
              ("town01_all", "town01_historical_straight", "town01_p03", "town01_p01", "town02_p02")}
    for index, row in enumerate(source_rows):
        source = source_root_for_record(row, historical)
        relative = source.relative_to(ROOT)
        stem = row["stem"]
        rgb_path, sensor_path = (source / (stem + suffix) for suffix in (".png", ".npz"))
        depth, tags, camera_z = validate_sensor_pair(
            d0_rgb[index], d0_target[index], rgb_path, sensor_path,
            source_hashes[str(relative / (stem + ".png"))],
            source_hashes[str(relative / (stem + ".npz"))])
        group = ("town01_historical_straight" if index < 114 else
                 "town01_p03" if index < 301 else "town01_p01")
        for key in ("town01_all", group):
            groups[key].add(d0_rgb[index], d0_target[index], depth, tags, camera_z)
    for index, pair in enumerate(pairs, start=497):
        stem = pair["stem"]
        require(np.bincount(d1_target[index].ravel(), minlength=7).tolist()
                == by_stem[stem]["class_pixels"], f"Town02 audited class pixels changed: {stem}")
        depth, tags, camera_z = validate_sensor_pair(
            d1_rgb[index], d1_target[index],
            TOWN02_SOURCE / (stem + ".png"), TOWN02_SOURCE / (stem + ".npz"),
            pair["rgb_sha256"], pair["semantic_npz_sha256"])
        groups["town02_p02"].add(d1_rgb[index], d1_target[index], depth, tags, camera_z)
    reports = {name: item.report() for name, item in groups.items()}
    require(reports["town01_all"]["frames"] == 497
            and [reports[key]["frames"] for key in
                 ("town01_historical_straight", "town01_p03", "town01_p01", "town02_p02")]
            == [114, 187, 196, 232]
            and (np.asarray(reports["town01_all"]["class_pixels"])
                 + np.asarray(reports["town02_p02"]["class_pixels"])).tolist()
            == files["train_d1"]["class_pixels"], "TRAIN group partition changed")
    return {
        "schema": "portable_e2e.map_lane_train_audit.v1",
        "status": "READ_ONLY_TRAIN_SOURCE_REPLAY_PASS",
        "input_sha256": {"data_manifest": DATA_MANIFEST_SHA256,
                         "parent_manifest": PARENT_SHA256,
                         "town02_source_audit": TOWN02_AUDIT_SHA256,
                         "train_d0": files["train_d0"]["sha256"],
                         "train_d1": files["train_d1"]["sha256"]},
        "scope": "D0/D1 TRAIN only; no DEV arrays, TEST, model inference, or training",
        "sensor_contract": "CARLA RGB and semantic BGRA; raw depth BGRA and rigid camera pose audited but model input is RGB only",
        "pixel_regions": "3x3 grid: y [0,120),[120,240),[240,360); x [0,213),[213,427),[427,640); near_ego x [160,480), y [180,360)",
        "groups": reports,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--verify", type=Path, help="compare with an existing report; never write it")
    args = parser.parse_args()
    report = audit()
    if args.verify:
        require(read_json(args.verify) == report, "published TRAIN audit differs")
        print(json.dumps({"status": "PUBLISHED_TRAIN_AUDIT_MATCH", "report": str(args.verify)}))
    else:
        print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
