#!/usr/bin/env python3
"""HH_260929 - Compare raw C-track camera tags with an older Town05 capture.

This is a CPU-only diagnostic. It neither infers labels from RGB nor modifies
the 240-frame inputs, and its palette is only a display of raw integer IDs.
"""

from __future__ import annotations

from collections import Counter
import hashlib
import json
from pathlib import Path
import tempfile

import carla
import numpy as np
from PIL import Image, ImageDraw


ROOT = Path(__file__).resolve().parents[2]
LEFT = ROOT / "storage/experiments/training/2026-09-29/map_ctrack_candidate_v2_left_owned_01/mapping"
TOWN05 = ROOT / "storage/experiments/training/2026-09-27/map_town05_independent_dev_capture_epic_v2/mapping"
OUTPUT = ROOT / "docs/assets/validation/2026-09-29/map_ctrack_left_semantic_failure"
FRAMES = 240
SELECTED = (0, 120, 239)
GIF_FRAMES = (*range(0, FRAMES, 20), 239)
RAW_COLORS = {
    0: (231, 40, 110),   # HH_260929 - Magenta means CARLA raw ID 0, not an inferred road label.
    1: (38, 70, 160),
    2: (235, 160, 45),
    3: (105, 105, 110),
    6: (205, 80, 30),
    7: (255, 210, 20),
    8: (180, 100, 35),
    9: (54, 145, 55),
    11: (95, 175, 235),
    24: (250, 250, 250),
}
PIXEL_PROBES = {
    "visible_left_white_edge_frame120": (150, 300),
    "visible_right_white_edge_frame120": (450, 300),
    "dark_road_center_frame120": (320, 300),
}


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _raw_color(value: int) -> tuple[int, int, int]:
    return RAW_COLORS.get(value, ((value * 79) % 256, (value * 131) % 256,
                                  (value * 197) % 256))


def _colorize(tags: np.ndarray) -> Image.Image:
    rgb = np.zeros((*tags.shape, 3), dtype=np.uint8)
    for value in np.unique(tags):
        rgb[tags == value] = _raw_color(int(value))
    return Image.fromarray(rgb, "RGB")


def _scan(source: Path) -> dict:
    if not source.is_dir() or source.is_symlink():
        raise ValueError(f"regular capture directory required: {source}")
    summary_path, records_path = source / "summary.json", source / "records.json"
    if any(not path.is_file() or path.is_symlink() for path in (summary_path, records_path)):
        raise ValueError(f"source manifest is missing or symlinked: {source}")
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    records = json.loads(records_path.read_text(encoding="utf-8"))
    if summary.get("frame_count") != FRAMES or len(records) != FRAMES:
        raise ValueError(f"expected exactly {FRAMES} archived frames: {source}")
    counts = Counter()
    per_frame = []
    valid_fraction = []
    for index in range(FRAMES):
        stem = source / "frames" / f"{index:04d}"
        npz = stem.with_suffix(".npz")
        if not npz.is_file() or npz.is_symlink():
            raise ValueError(f"missing native semantic/depth frame: {npz}")
        with np.load(npz, allow_pickle=False) as values:
            depth = values["depth"]
            semantic = values["semantic"]
        if (depth.shape != (360, 640, 4) or semantic.shape != depth.shape
                or depth.dtype != np.uint8 or semantic.dtype != np.uint8):
            raise ValueError(f"invalid native BGRA frame {index}: {source}")
        tags = semantic[..., 2]
        unique, pixel_counts = np.unique(tags, return_counts=True)
        row_counts = {int(key): int(value) for key, value in zip(unique, pixel_counts)}
        counts.update(row_counts)
        encoded = depth.astype(np.float64)
        metres = (encoded[..., 2] + 256.0 * encoded[..., 1]
                  + 65536.0 * encoded[..., 0]) * (1000.0 / 16777215.0)
        fraction = float(np.count_nonzero(np.isfinite(metres)
                    & (metres > 0.1) & (metres < 40.0)) / tags.size)
        valid_fraction.append(fraction)
        per_frame.append({"frame_index": index, "raw_tag_pixels": row_counts,
                          "valid_depth_0p1_to_40m_fraction": fraction})
    road_id = int(carla.CityObjectLabel.Roads)
    lane_id = int(carla.CityObjectLabel.RoadLines)
    return {
        "source": str(source),
        "summary_sha256": _sha256(summary_path),
        "records_sha256": _sha256(records_path),
        "collector_status": summary.get("status"),
        "town": summary.get("town"),
        "frame_count": FRAMES,
        "observed_cells": summary.get("observed_cells"),
        "label_table": summary.get("label_table"),
        "total_raw_tag_pixels": dict(sorted(counts.items())),
        "road_tag_id": road_id,
        "lane_tag_id": lane_id,
        "road_tag_frames": sum(row["raw_tag_pixels"].get(road_id, 0) > 0 for row in per_frame),
        "lane_tag_frames": sum(row["raw_tag_pixels"].get(lane_id, 0) > 0 for row in per_frame),
        "valid_depth_0p1_to_40m_fraction": {
            "min": min(valid_fraction), "mean": float(np.mean(valid_fraction)),
            "max": max(valid_fraction)},
        "per_frame": per_frame,
    }


def _panel(source: Path, index: int, *, size: tuple[int, int] = (640, 360)) -> Image.Image:
    png = source / "frames" / f"{index:04d}.png"
    npz = source / "frames" / f"{index:04d}.npz"
    if not png.is_file() or png.is_symlink():
        raise ValueError(f"missing native RGB frame: {png}")
    with Image.open(png) as image:
        rgb = image.convert("RGB")
    with np.load(npz, allow_pickle=False) as values:
        tags = values["semantic"][..., 2]
    semantic = _colorize(tags)
    if rgb.size != (640, 360) or semantic.size != rgb.size:
        raise ValueError(f"RGB and semantic shape differ at frame {index}")
    resampling = getattr(Image, "Resampling", Image)
    rgb = rgb.resize(size, resampling.BILINEAR)
    semantic = semantic.resize(size, resampling.NEAREST)
    width, height = size
    panel = Image.new("RGB", (width * 2, height + 50), (250, 250, 250))
    panel.paste(rgb, (0, 25))
    panel.paste(semantic, (width, 25))
    draw = ImageDraw.Draw(panel)
    draw.text((8, 5), f"C-track left #{index:04d} | CARLA BasicAgent, NOT learned/Autoware",
              fill=(0, 0, 0))
    draw.text((8, height + 30), "Full-FOV RGB (original pixels)", fill=(0, 0, 0))
    draw.text((width + 8, height + 30),
              "Raw semantic IDs: magenta=0; road=1/lane=24 absent", fill=(0, 0, 0))
    return panel


def main() -> None:
    if OUTPUT.exists() or OUTPUT.is_symlink():
        raise ValueError("diagnostic output already exists; never overwrite evidence")
    left = _scan(LEFT)
    town05 = _scan(TOWN05)
    ids = tuple(sorted(set(left["total_raw_tag_pixels"]) | set(town05["total_raw_tag_pixels"])))
    names = ("None", "Roads", "RoadLines", "Sidewalks", "Buildings", "Vegetation",
             "Sky", "TrafficLight", "TrafficSigns", "Poles")
    installed_enum = {name: int(getattr(carla.CityObjectLabel, name))
                      for name in names if hasattr(carla.CityObjectLabel, name)}
    with np.load(LEFT / "frames/0120.npz", allow_pickle=False) as values:
        tags120 = values["semantic"][..., 2]
    with Image.open(LEFT / "frames/0120.png") as image:
        rgb120 = np.asarray(image.convert("RGB"))
    probes = {name: {"xy": [x, y], "rgb": rgb120[y, x].tolist(),
                     "raw_semantic_id": int(tags120[y, x])}
              for name, (x, y) in PIXEL_PROBES.items()}
    # HH_260929 - Publish only a complete diagnostic; retain a failed staging dir for review.
    staged = Path(tempfile.mkdtemp(prefix=".ctrack_left_semantic_staged_",
                                   dir=OUTPUT.parent))
    report = {
        "schema": "portable_e2e.ctrack_left_raw_semantic_diagnostic.v1",
        "method": "CPU-only read of all 240 native BGRA depth/semantic NPZ pairs per site",
        "capture_type": "CARLA BasicAgent; not learned control or Autoware",
        "raw_tags_only": True, "pseudo_labels_created": False,
        "dataset_admitted": False, "independent_dev_approved": False,
        "installed_carla_city_object_label": installed_enum,
        "display_palette_raw_ids": {str(i): list(_raw_color(i)) for i in ids},
        "frame120_visual_pixel_probes": probes,
        "ctrack_left": left, "town05_reference": town05,
    }
    (staged / "metrics.json").write_text(json.dumps(report, indent=2) + "\n")
    panels = [_panel(LEFT, index) for index in SELECTED]
    contact = Image.new("RGB", (1280, len(panels) * 410), "white")
    for row, panel in enumerate(panels):
        contact.paste(panel, (0, row * 410))
    contact.save(staged / "full_fov_rgb_vs_raw_semantic.png", optimize=True)
    gif_panels = [_panel(LEFT, index, size=(480, 270)).quantize(colors=128)
                  for index in GIF_FRAMES]
    gif_panels[0].save(staged / "sampled_full_fov_rgb_vs_raw_semantic.gif",
                       save_all=True, append_images=gif_panels[1:], duration=300,
                       loop=0, disposal=2, optimize=False)
    staged.rename(OUTPUT)
    print(json.dumps({"output": str(OUTPUT),
                      "ctrack_road_frames": left["road_tag_frames"],
                      "ctrack_lane_frames": left["lane_tag_frames"],
                      "town05_road_frames": town05["road_tag_frames"],
                      "town05_lane_frames": town05["lane_tag_frames"]}))


if __name__ == "__main__":
    main()
