#!/usr/bin/env python3
"""HH_260929 - Render full-FOV Town05 road/lane A/B DEV evidence, never a TEST claim."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_DATA = ROOT / "storage/experiments/training/2026-09-29/map_road_lane_ab_data_v2"
EXPECTED_MANIFEST_SHA256 = "313b41ed4fb897f01fb4fb433c1bafb234ef9b234e62f96a462ef2188e9dccc6"
EXPECTED_DEV_SHA256 = "8e04221d5c6990cf43b03bbdfefa14004be50bf2e2069c1d346b6822eb631413"
EXPECTED_COUNT = 232
HEIGHT, WIDTH = 360, 640
LABELS = ("other", "road", "sidewalk", "lane_marking", "traffic_light_fixture",
          "traffic_sign", "pole")
ROAD = LABELS.index("road")
LANE = LABELS.index("lane_marking")
ROAD_COLOR = (25, 205, 245)
LANE_COLOR = (255, 226, 55)
CROP_XYXY = (160, 180, 480, 360)
CANVAS_SIZE = (1280, 1286)


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
    require(path.is_file() and not path.is_symlink(), f"regular JSON file required: {path}")
    value = json.loads(path.read_text(encoding="utf-8"))
    require(isinstance(value, dict), f"JSON object required: {path}")
    return value


def load_dev(data_root: Path) -> tuple[np.ndarray, np.ndarray, dict, dict]:
    """HH_260929 - Pin the exact 232-frame Town05 DEV package before viewing it."""
    require(data_root.is_dir() and not data_root.is_symlink(), "regular data directory required")
    manifest_path = data_root / "manifest.json"
    require(digest(manifest_path) == EXPECTED_MANIFEST_SHA256,
            "Town05 data manifest differs from the preregistered A/B package")
    manifest = read_json(manifest_path)
    require(manifest.get("schema") == "portable_e2e.map_road_lane_ab_data.v1"
            and manifest.get("town05_split") == "DEV_ONLY"
            and manifest.get("image_size_wh") == [WIDTH, HEIGHT]
            and tuple(manifest.get("labels", ())) == LABELS,
            "Town05 DEV package contract changed")
    ref = manifest.get("files", {}).get("dev_town05")
    require(isinstance(ref, dict) and ref.get("file") == "dev_town05.npz"
            and ref.get("count") == EXPECTED_COUNT
            and ref.get("sha256") == EXPECTED_DEV_SHA256
            and ref.get("towns") == ["Town05_Opt"],
            "Town05 DEV file metadata changed")
    dev_path = data_root / "dev_town05.npz"
    require(digest(dev_path) == EXPECTED_DEV_SHA256, "Town05 DEV bytes changed")
    with np.load(dev_path, allow_pickle=False) as archive:
        require(set(archive.files) == {"rgb", "target"}, "Town05 DEV array keys changed")
        rgb, target = archive["rgb"].copy(), archive["target"].copy()
    require(rgb.shape == (EXPECTED_COUNT, HEIGHT, WIDTH, 3)
            and target.shape == (EXPECTED_COUNT, HEIGHT, WIDTH)
            and rgb.dtype == target.dtype == np.uint8
            and int(target.max()) < len(LABELS), "Town05 DEV native array contract changed")
    require(np.bincount(target.ravel(), minlength=len(LABELS)).tolist()
            == ref.get("class_pixels"), "Town05 DEV semantic counts changed")
    return rgb, target, manifest, {"manifest": digest(manifest_path), "dev": digest(dev_path)}


def confusion_for(target: np.ndarray, prediction: np.ndarray) -> list[list[int]]:
    """HH_260929 - Bound memory while independently replaying stored DEV confusion."""
    confusion = np.zeros((len(LABELS), len(LABELS)), dtype=np.int64)
    for begin in range(0, len(target), 8):
        encoded = target[begin:begin + 8].astype(np.int64) * len(LABELS)
        encoded += prediction[begin:begin + 8]
        confusion += np.bincount(encoded.ravel(), minlength=len(LABELS) ** 2).reshape(
            len(LABELS), len(LABELS))
    return confusion.tolist()


def load_run(run_root: Path, target: np.ndarray, manifest_sha256: str
             ) -> tuple[dict[str, np.ndarray], dict]:
    """HH_260929 - Require a complete, non-promoted D0/D1 run and matching DEV metrics."""
    require(run_root.is_dir() and not run_root.is_symlink(), "regular run directory required")
    started_path = run_root / "started.json"
    summary_path = run_root / "summary.json"
    reproduction_path = run_root / "d0_reproduction.json"
    started, summary, reproduction = (read_json(path) for path in
                                      (started_path, summary_path, reproduction_path))
    require(started.get("schema") == "portable_e2e.map_road_lane_ab_run.v1"
            and started.get("data_manifest_sha256") == manifest_sha256
            and started.get("automatic_promotion") is False
            and summary.get("status") == "A_B_COMPLETE_EXPLORATORY_NOT_PROMOTED"
            and summary.get("model_promotion") is False
            and summary.get("gpu_training_executed") is True
            and summary.get("gates", {}).get("automatic_model_promotion") is False
            and summary.get("sign_dev_qualified") is False
            and summary.get("hd_map_complete") is False
            and summary.get("learned_closed_loop_driving") is False
            and reproduction.get("pass") is True,
            "complete exploratory D0/D1 run contract required")
    gate_met = summary["gates"].get("road_lane_development_target_met")
    require(type(gate_met) is bool, "road/lane DEV gate result required")
    predictions: dict[str, np.ndarray] = {}
    provenance = {"started_sha256": digest(started_path),
                  "summary_sha256": digest(summary_path),
                  "d0_reproduction_sha256": digest(reproduction_path),
                  "road_lane_development_target_met": gate_met,
                  "arms": {}}
    for arm in ("D0", "D1"):
        arm_dir = run_root / arm
        require(arm_dir.is_dir() and not arm_dir.is_symlink(), f"{arm} directory missing")
        metrics_path = arm_dir / "metrics.json"
        prediction_path = arm_dir / "town05_predictions.npz"
        metrics = read_json(metrics_path)
        require(summary.get(arm) == metrics and metrics.get("deployment_approved") is False,
                f"{arm} summary or deployment status changed")
        require(prediction_path.is_file() and not prediction_path.is_symlink(),
                f"regular {arm} prediction file required")
        with np.load(prediction_path, allow_pickle=False) as archive:
            require(set(archive.files) == {"prediction"}, f"{arm} prediction keys changed")
            prediction = archive["prediction"].copy()
        require(prediction.shape == target.shape and prediction.dtype == np.uint8
                and int(prediction.max()) < len(LABELS),
                f"{arm} native Town05 prediction contract changed")
        require(confusion_for(target, prediction) == metrics.get("town05", {}).get("confusion"),
                f"{arm} prediction differs from Town05 metrics")
        predictions[arm] = prediction
        provenance["arms"][arm] = {"metrics_sha256": digest(metrics_path),
                                   "prediction_sha256": digest(prediction_path)}
    return predictions, provenance


def overlay(rgb: np.ndarray, semantic: np.ndarray) -> np.ndarray:
    """HH_260929 - Tint only road and lane; leave all other RGB context visible."""
    require(rgb.shape == (HEIGHT, WIDTH, 3) and semantic.shape == (HEIGHT, WIDTH)
            and rgb.dtype == semantic.dtype == np.uint8, "native frame required")
    result = rgb.copy()
    for label, color, opacity in ((ROAD, ROAD_COLOR, 0.42),
                                  (LANE, LANE_COLOR, 0.86)):
        mask = semantic == label
        result[mask] = np.rint(rgb[mask] * (1 - opacity)
                               + np.asarray(color) * opacity).astype(np.uint8)
    return result


def font(size: int) -> ImageFont.ImageFont:
    path = Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")
    return ImageFont.truetype(str(path), size) if path.is_file() else ImageFont.load_default()


def render_frame(index: int, rgb: np.ndarray, target: np.ndarray,
                 d0: np.ndarray, d1: np.ndarray, gate_met: bool) -> Image.Image:
    """HH_260929 - Show four complete 640x360 views and a marked near-ego crop."""
    base = Image.fromarray(rgb)
    oracle = Image.fromarray(overlay(rgb, target))
    d0_view = Image.fromarray(overlay(rgb, d0))
    d1_view = Image.fromarray(overlay(rgb, d1))
    ImageDraw.Draw(base).rectangle((CROP_XYXY[0], CROP_XYXY[1],
                                    CROP_XYXY[2] - 1, CROP_XYXY[3] - 1),
                                   outline=(255, 255, 255), width=2)
    crop_size = (WIDTH, HEIGHT)
    panels = (("RGB input | white box = near-ego detail", base),
              ("CARLA oracle target | road/lane overlay", oracle),
              ("D0 Town01-only | unpromoted prediction", d0_view),
              ("D1 + Town02 p02 | unpromoted prediction", d1_view),
              ("D0 near-ego detail | original pixels x2",
               d0_view.crop(CROP_XYXY).resize(crop_size, Image.NEAREST)),
              ("D1 near-ego detail | original pixels x2",
               d1_view.crop(CROP_XYXY).resize(crop_size, Image.NEAREST)))
    canvas = Image.new("RGB", CANVAS_SIZE, (15, 23, 34))
    draw = ImageDraw.Draw(canvas)
    title_font, panel_font = font(19), font(16)
    gate_label = "PASS" if gate_met else "FAIL"
    draw.text((12, 8),
              f"Town05_Opt DEV frame {index:03d} | road/lane DEV gate {gate_label} | NOT PROMOTED | NOT TEST",
              fill="white", font=title_font)
    draw.rectangle((12, 43, 36, 61), fill=ROAD_COLOR)
    draw.text((44, 41), "road", fill="white", font=panel_font)
    draw.rectangle((124, 43, 148, 61), fill=LANE_COLOR)
    draw.text((156, 41), "lane marking", fill="white", font=panel_font)
    draw.text((390, 41), "Other classes remain untinted; all upper views retain the full 640x360 FOV.",
              fill=(210, 217, 228), font=panel_font)
    for position, (caption, picture) in enumerate(panels):
        left = (position % 2) * WIDTH
        top = 74 + (position // 2) * 388
        draw.text((left + 10, top + 4), caption, fill="white", font=panel_font)
        canvas.paste(picture, (left, top + 28))
    draw.text((12, 1248), "DEV visualization only; not HD-map completeness, closed-loop driving, or deployment approval.",
              fill=(210, 217, 228), font=panel_font)
    return canvas


def write_gallery(output: Path, rgb: np.ndarray, target: np.ndarray,
                  predictions: dict[str, np.ndarray], provenance: dict) -> dict:
    """HH_260929 - Use fixed temporal positions, never select frames by model quality."""
    require(not output.exists() and not output.is_symlink(), "fresh visualization output required")
    require(output.parent.is_dir() and not output.parent.is_symlink(),
            "existing regular output parent required")
    count = len(rgb)
    require(count >= 3 and target.shape == (count, HEIGHT, WIDTH)
            and all(predictions[arm].shape == target.shape for arm in ("D0", "D1")),
            "aligned native DEV arrays required")
    indices = [0, (count - 1) // 2, count - 1]
    output.mkdir(exist_ok=False)
    frames = []
    files = {}
    for index in indices:
        frame = render_frame(index, rgb[index], target[index],
                             predictions["D0"][index], predictions["D1"][index],
                             provenance["run"]["road_lane_development_target_met"])
        name = f"town05_dev_{index:03d}_road_lane_ab.png"
        path = output / name
        frame.save(path, format="PNG")
        files[name] = digest(path)
        frames.append(frame)
    gif_name = "town05_dev_road_lane_ab.gif"
    gif_path = output / gif_name
    frames[0].save(gif_path, format="GIF", save_all=True, append_images=frames[1:],
                   duration=1300, loop=0, optimize=False)
    files[gif_name] = digest(gif_path)
    manifest = {
        "schema": "portable_e2e.map_road_lane_ab_town05_visuals.v1",
        "split": "Town05_Opt DEV_ONLY", "independent_test_used": False,
        "automatic_model_promotion": False,
        "interpretation": "static semantic road/lane diagnostic; not HD-map, driving, or deployment approval",
        "selection": "fixed first, temporal midpoint, last; independent of prediction quality",
        "indices": indices, "full_fov_wh": [WIDTH, HEIGHT],
        "near_ego_crop_xyxy": list(CROP_XYXY), "near_ego_scale": 2,
        "overlay": {"road_rgb": list(ROAD_COLOR), "road_opacity": 0.42,
                    "lane_rgb": list(LANE_COLOR), "lane_opacity": 0.86},
        "provenance": provenance, "files_sha256": files,
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n",
                                          encoding="utf-8")
    return manifest


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, default=DEFAULT_DATA)
    parser.add_argument("--run-root", type=Path, required=True,
                        help="completed exploratory run containing D0/D1 predictions and metrics")
    parser.add_argument("--output", type=Path, required=True,
                        help="new directory; existing evidence is never overwritten")
    args = parser.parse_args(argv)
    require(not args.output.exists() and not args.output.is_symlink(),
            "fresh visualization output required")
    require(not args.output.resolve().is_relative_to(args.data.resolve())
            and not args.output.resolve().is_relative_to(args.run_root.resolve()),
            "visualization output must be outside source data and run")
    rgb, target, _, data_hashes = load_dev(args.data)
    predictions, run_hashes = load_run(args.run_root, target, data_hashes["manifest"])
    result = write_gallery(args.output, rgb, target, predictions,
                           {"data": data_hashes, "run": run_hashes})
    print(json.dumps({"status": "DEV_VISUALS_RENDERED_NOT_PROMOTED",
                      "output": str(args.output), "indices": result["indices"],
                      "files_sha256": result["files_sha256"]}, sort_keys=True))


if __name__ == "__main__":
    main()
