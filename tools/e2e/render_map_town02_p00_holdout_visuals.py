#!/usr/bin/env python3
"""HH_260929 - Render pinned, full-FOV Town02 p02 and Town01 map A/B evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[2]
DATA = ROOT / "storage/experiments/training/2026-09-29/map_town02_p00_route_holdout_data_v1"
RUN = ROOT / "storage/experiments/training/2026-09-29/map_town02_p00_holdout_run_v1"
OUTPUT = ROOT / "docs/assets/validation/2026-09-29/map_town02_p00_route_holdout/visuals"
DATA_MANIFEST_SHA256 = "f4764b9aa9d015f3d136a145f6194dd60bda8900c27dd2423de6145b9f472834"
RUN_SUMMARY_SHA256 = "bd0d9a100ca8e8420cbaea856c4566c3985683e7040782bcd7138f83d1d488f6"
LABELS = ("other", "road", "sidewalk", "lane_marking", "traffic_light_fixture",
          "traffic_sign", "pole")
COLORS = np.array(((30, 30, 40), (67, 92, 123), (141, 88, 147), (246, 222, 90),
                   (246, 100, 70), (60, 220, 180), (179, 190, 199)), dtype=np.uint8)
FRAME_WH = (640, 360)
CANVAS_WH = (1280, 936)
PANEL_POSITIONS = ((0, 48), (640, 48), (0, 510), (640, 510))
# HH_260929 - Fix temporal positions before consulting per-frame model scores.
SPLITS = {
    "p02": {"dataset": "holdout_town02_p02", "count": 232, "indices": (0, 115, 231),
            "title": "Town02_Opt p02 | historically exposed internal route holdout",
            "caption": "p02 is not an independent DEV or sealed TEST site"},
    "town01": {"dataset": "dev_town01", "count": 118, "indices": (0, 58, 117),
               "title": "Town01 | known DEV route",
               "caption": "Town01 is a known protection metric, not a new independent site"},
}


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def digest(path: Path) -> str:
    require(path.is_file() and not path.is_symlink(), f"regular file required: {path}")
    hash_state = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            hash_state.update(block)
    return hash_state.hexdigest()


def read_json(path: Path) -> dict:
    require(path.is_file() and not path.is_symlink(), f"regular JSON required: {path}")
    value = json.loads(path.read_text(encoding="utf-8"))
    require(isinstance(value, dict), f"JSON object required: {path}")
    return value


def load_contract(data_root: Path, run_root: Path) -> tuple[dict, dict, dict]:
    """HH_260929 - Require exact source bytes, complete predictions, and failed promotion gate."""
    require(data_root.is_dir() and run_root.is_dir()
            and not data_root.is_symlink() and not run_root.is_symlink(),
            "regular source directories required")
    manifest_path, summary_path = data_root / "manifest.json", run_root / "summary.json"
    require(digest(manifest_path) == DATA_MANIFEST_SHA256, "data manifest bytes changed")
    require(digest(summary_path) == RUN_SUMMARY_SHA256, "A/B summary bytes changed")
    manifest, summary = read_json(manifest_path), read_json(summary_path)
    started_path = run_root / "started.json"
    started = read_json(started_path)
    require(started.get("dataset_manifest_sha256") == DATA_MANIFEST_SHA256
            and started.get("automatic_promotion") is False
            and summary.get("status") == "A_B_INTERNAL_ROUTE_HOLDOUT_COMPLETE_NOT_PROMOTED"
            and summary.get("gpu_training_executed") is True
            and summary.get("model_promotion") is False
            and summary.get("gates", {}).get("all_gates_passed") is False
            and summary.get("independent_dev_sites") == 0
            and manifest.get("image_size_wh") == list(FRAME_WH)
            and tuple(manifest.get("labels", ())) == LABELS,
            "A/B source or promotion status changed")
    provenance = {"data_manifest_sha256": digest(manifest_path),
                  "run_summary_sha256": digest(summary_path),
                  "run_started_sha256": digest(started_path),
                  "source_files_sha256": {}}
    return manifest, summary, provenance


def confusion(target: np.ndarray, prediction: np.ndarray) -> list[list[int]]:
    """HH_260929 - Recompute all 7x7 counts from native predictions in small chunks."""
    matrix = np.zeros((len(LABELS), len(LABELS)), dtype=np.int64)
    for start in range(0, len(target), 8):
        encoded = target[start:start + 8].astype(np.int64) * len(LABELS)
        encoded += prediction[start:start + 8]
        matrix += np.bincount(encoded.ravel(), minlength=len(LABELS) ** 2).reshape(
            len(LABELS), len(LABELS))
    return matrix.tolist()


def load_split(data_root: Path, run_root: Path, manifest: dict, summary: dict,
               provenance: dict, name: str) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """HH_260929 - Align each native RGB/GT frame with both archived model outputs."""
    spec = SPLITS[name]
    entry = manifest.get("files", {}).get(spec["dataset"])
    require(isinstance(entry, dict) and entry.get("count") == spec["count"]
            and entry.get("file") == f"{spec['dataset']}.npz",
            f"{name} dataset contract changed")
    dataset_path = data_root / entry["file"]
    require(digest(dataset_path) == entry["sha256"], f"{name} dataset bytes changed")
    with np.load(dataset_path, allow_pickle=False) as archive:
        require(set(archive.files) == {"rgb", "target"}, f"{name} dataset keys changed")
        rgb, target = archive["rgb"].copy(), archive["target"].copy()
    count = spec["count"]
    require(rgb.shape == (count, 360, 640, 3)
            and target.shape == (count, 360, 640)
            and rgb.dtype == target.dtype == np.uint8
            and int(target.max()) < len(LABELS),
            f"{name} native RGB/GT contract changed")
    provenance["source_files_sha256"][entry["file"]] = digest(dataset_path)
    preds = []
    for arm in ("A", "B"):
        prediction_path = run_root / arm / f"{name}_predictions.npz"
        with np.load(prediction_path, allow_pickle=False) as archive:
            require(set(archive.files) == {"prediction"},
                    f"{name} {arm} prediction keys changed")
            prediction = archive["prediction"].copy()
        require(prediction.shape == target.shape and prediction.dtype == np.uint8
                and int(prediction.max()) < len(LABELS),
                f"{name} {arm} prediction shape/class changed")
        require(confusion(target, prediction) == summary[arm][name]["confusion"],
                f"{name} {arm} pixels disagree with run metrics")
        provenance["source_files_sha256"][f"{arm}/{name}_predictions.npz"] = digest(
            prediction_path)
        preds.append(prediction)
    return rgb, target, preds[0], preds[1]


def font(size: int) -> ImageFont.ImageFont:
    path = Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")
    return ImageFont.truetype(str(path), size) if path.is_file() else ImageFont.load_default()


def render_frame(split: str, index: int, rgb: np.ndarray, target: np.ndarray,
                 a: np.ndarray, b: np.ndarray) -> Image.Image:
    """HH_260929 - Keep every RGB and segmentation pixel at its original 640x360 position."""
    require(rgb.shape == (360, 640, 3)
            and all(item.shape == (360, 640) for item in (target, a, b)),
            "complete native frame required")
    panels = (("Original RGB input", rgb), ("CARLA semantic GT | seven class colors", COLORS[target]),
              ("A | Town01 TRAIN 497 | model prediction", COLORS[a]),
              ("B | A + Town02 p00 TRAIN 232 | model prediction", COLORS[b]))
    canvas = Image.new("RGB", CANVAS_WH, (18, 27, 40))
    draw = ImageDraw.Draw(canvas)
    title_font, panel_font, small_font = font(18), font(16), font(13)
    spec = SPLITS[split]
    draw.text((12, 9), f"{spec['title']} | fixed frame {index:03d}/{spec['count'] - 1:03d}",
              fill="white", font=title_font)
    for (left, top), (caption, pixels) in zip(PANEL_POSITIONS, panels):
        draw.text((left + 10, top), caption, fill="white", font=panel_font)
        canvas.paste(Image.fromarray(pixels, mode="RGB"), (left, top + 23))
    draw.text((12, 909), f"{spec['caption']}; static frame comparison, not a live Autoware drive.",
              fill=(218, 227, 239), font=small_font)
    # HH_260929 - Use the same seven-class palette for GT, A, and B without merging classes.
    legend_top = 447
    for position, (label, color) in enumerate(zip(LABELS, COLORS)):
        column, row = position % 4, position // 4
        x, y = column * 320 + 11, legend_top + row * 21
        draw.rectangle((x, y, x + 13, y + 13), fill=tuple(int(v) for v in color))
        draw.text((x + 19, y - 2), label, fill=(225, 235, 244), font=small_font)
    return canvas


def write_gallery(data_root: Path, run_root: Path, output: Path) -> dict:
    """HH_260929 - Save immutable sampled evidence after verifying both run arms."""
    require(not output.exists() and not output.is_symlink(), "fresh output directory required")
    require(output.parent.is_dir() and not output.parent.is_symlink(),
            "existing regular gallery parent required")
    require(not output.resolve().is_relative_to(data_root.resolve())
            and not output.resolve().is_relative_to(run_root.resolve()),
            "gallery must be outside source data and run")
    manifest, summary, provenance = load_contract(data_root, run_root)
    rendered: dict[str, list[tuple[str, Image.Image]]] = {}
    for split in SPLITS:
        rgb, target, a, b = load_split(data_root, run_root, manifest, summary, provenance, split)
        rendered[split] = [
            (f"{split}_{index:03d}_full_fov_rgb_gt_a_b.png",
             render_frame(split, index, rgb[index], target[index], a[index], b[index]))
            for index in SPLITS[split]["indices"]
        ]
    output.mkdir(exist_ok=False)
    files = {}
    for split, items in rendered.items():
        for name, frame in items:
            path = output / name
            frame.save(path, format="PNG", optimize=True)
            files[name] = digest(path)
        gif_name = f"{split}_three_fixed_frames_sampled.gif"
        gif_path = output / gif_name
        frames = [frame for _, frame in items]
        frames[0].save(gif_path, format="GIF", save_all=True, append_images=frames[1:],
                       duration=1400, loop=0, optimize=False)
        files[gif_name] = digest(gif_path)
    result = {
        "schema": "portable_e2e.map_town02_p00_holdout_visuals.v1",
        "source": "archived 640x360 RGB, CARLA semantic GT, and native A/B predictions",
        "full_fov_wh": list(FRAME_WH), "canvas_wh": list(CANVAS_WH),
        "panels": ["original_rgb", "seven_class_gt", "a_prediction", "b_prediction"],
        "palette_rgb": {name: [int(v) for v in color] for name, color in zip(LABELS, COLORS)},
        "frame_selection": {split: {"indices": list(spec["indices"]),
                                    "method": "fixed first, temporal midpoint, last; chosen before frame scores"}
                            for split, spec in SPLITS.items()},
        "gif_semantics": "three fixed sampled images per split; not continuous driving footage",
        "p02_independent_test": False, "independent_dev_sites": 0,
        "all_training_gates_passed": False, "model_promotion": False,
        "source_sha256": provenance, "files_sha256": files,
    }
    (output / "manifest.json").write_text(json.dumps(result, indent=2, sort_keys=True) + "\n",
                                          encoding="utf-8")
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, default=DATA)
    parser.add_argument("--run", type=Path, default=RUN)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    args = parser.parse_args()
    result = write_gallery(args.data, args.run, args.output)
    print(json.dumps({"output": str(args.output),
                      "frames": {k: v["indices"] for k, v in result["frame_selection"].items()},
                      "files": result["files_sha256"]}, sort_keys=True))


if __name__ == "__main__":
    main()
