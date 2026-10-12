#!/usr/bin/env python3
"""HH_260929 - Render fixed full-FOV repeated-DEV D1/photo comparisons."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

import verify_result


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[4]
DATA = ROOT / "storage/experiments/training/2026-09-29/map_road_lane_ab_data_v2"
BASELINE = ROOT / "storage/experiments/training/2026-09-29/map_road_lane_ab_run_v1"
RUN = HERE / "remote_run_v1"
SUMS = HERE / "SHA256SUMS"
OUTPUT = HERE / "visuals"
GIF_NAME = "town05_dev_000_115_231_four_panel_near_ego.gif"
TOWN05_GIF_INDICES = (0, 115, 231)
NEAR_EGO_BOX = (160, 180, 480, 360)
PALETTE = {
    1: (24, 115, 235), 2: (145, 82, 181), 3: (255, 212, 35),
    4: (232, 65, 67), 5: (224, 70, 173), 6: (42, 180, 180),
}


def font(size: int):
    path = Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")
    return ImageFont.truetype(str(path), size) if path.is_file() else ImageFont.load_default()


def overlay(rgb: np.ndarray, labels: np.ndarray) -> Image.Image:
    """HH_260929 - Color semantic classes without cropping the camera image."""
    result = rgb.copy().astype(np.uint16)
    for label, color in PALETTE.items():
        mask = labels == label
        result[mask] = (result[mask] * 3 + np.asarray(color, dtype=np.uint16) * 2) // 5
    return Image.fromarray(result.astype(np.uint8), "RGB")


def render() -> list[Path]:
    verify_result.replay(DATA, BASELINE, RUN, SUMS)
    OUTPUT.mkdir(exist_ok=False)
    paths = []
    for town, split, index in (("town01", "dev_town01", 0),
                               ("town05", "dev_town05", 115)):
        with np.load(DATA / f"{split}.npz", allow_pickle=False) as archive:
            rgb = archive["rgb"][index].copy()
            target = archive["target"][index].copy()
        with np.load(BASELINE / f"D1/{town}_predictions.npz", allow_pickle=False) as archive:
            base = archive["prediction"][index].copy()
        with np.load(RUN / f"{town}_predictions.npz", allow_pickle=False) as archive:
            photo = archive["prediction"][index].copy()
        canvas = Image.new("RGB", (2560, 470), (245, 247, 251))
        draw = ImageDraw.Draw(canvas)
        draw.text((15, 8), f"{town.upper()} | repeated DEV frame {index:03d}",
                  fill=(20, 30, 42), font=font(27))
        draw.text((1550, 15), "EXPLORATORY ONLY | NOT INDEPENDENT | NOT HD MAP",
                  fill=(150, 35, 39), font=font(18))
        panels = (Image.fromarray(rgb), overlay(rgb, target), overlay(rgb, base),
                  overlay(rgb, photo))
        names = ("RGB", "CARLA oracle", "Frozen D1", "Photometric candidate")
        for column, (panel, name) in enumerate(zip(panels, names)):
            x = column * 640
            draw.text((x + 12, 53), name, fill=(26, 38, 53), font=font(20))
            canvas.paste(panel, (x, 80))
        draw.text((15, 446), "Full 640x360 camera view per panel | road blue, lane yellow; other classes tinted",
                  fill=(55, 67, 82), font=font(15))
        path = OUTPUT / f"{town}_dev_{index:03d}_rgb_oracle_d1_photo.png"
        with path.open("xb") as stream:
            canvas.save(stream, format="PNG")
        paths.append(path)
    return paths


def gif_frame(index: int, rgb: np.ndarray, target: np.ndarray,
              base: np.ndarray, photo: np.ndarray) -> Image.Image:
    """HH_260929 - Keep four full FOVs and add the same near-ego crop below each."""
    panels = (Image.fromarray(rgb), overlay(rgb, target), overlay(rgb, base),
              overlay(rgb, photo))
    names = ("RGB", "CARLA oracle", "Frozen D1", "Photometric candidate")
    canvas = Image.new("RGB", (2560, 800), (245, 247, 251))
    draw = ImageDraw.Draw(canvas)
    draw.text((15, 8), f"TOWN05 repeated DEV | fixed source-order frame {index:03d}",
              fill=(20, 30, 42), font=font(27))
    draw.text((1530, 15), "EXPLORATORY | NOT AUTOWARE | NOT HD MAP",
              fill=(150, 35, 39), font=font(18))
    resample = getattr(Image, "Resampling", Image)
    for column, (panel, name) in enumerate(zip(panels, names)):
        x = column * 640
        draw.text((x + 12, 53), name, fill=(26, 38, 53), font=font(20))
        canvas.paste(panel, (x, 80))
        draw.text((x + 80, 452), "Near ego: x=160..479, y=180..359",
                  fill=(26, 38, 53), font=font(15))
        crop = panel.crop(NEAR_EGO_BOX).resize((480, 270), resample.NEAREST)
        canvas.paste(crop, (x + 80, 480))
        draw.rectangle((x + 79, 479, x + 560, 750), outline=(44, 58, 74), width=2)
    draw.text((15, 770),
              "Each upper panel is the full 640x360 camera FOV; lower boxes magnify identical pixels | sampled snapshots, not continuous driving video",
              fill=(55, 67, 82), font=font(15))
    return canvas


def render_gif(path: Path = OUTPUT / GIF_NAME) -> Path:
    """HH_260929 - Save only fixed 000/115/231 repeated-DEV snapshots, never overwrite."""
    verify_result.replay(DATA, BASELINE, RUN, SUMS)
    if path.exists() or path.is_symlink():
        raise FileExistsError(f"fresh GIF path required: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    positions = list(TOWN05_GIF_INDICES)
    with np.load(DATA / "dev_town05.npz", allow_pickle=False) as archive:
        rgb = archive["rgb"][positions].copy()
        target = archive["target"][positions].copy()
    with np.load(BASELINE / "D1/town05_predictions.npz", allow_pickle=False) as archive:
        base = archive["prediction"][positions].copy()
    with np.load(RUN / "town05_predictions.npz", allow_pickle=False) as archive:
        photo = archive["prediction"][positions].copy()
    frames = [gif_frame(index, rgb[i], target[i], base[i], photo[i]).convert(
        "P", palette=Image.ADAPTIVE, colors=256)
        for i, index in enumerate(TOWN05_GIF_INDICES)]
    with path.open("xb") as stream:
        frames[0].save(stream, format="GIF", save_all=True, append_images=frames[1:],
                       duration=[1800] * len(frames), loop=0, disposal=2, optimize=False)
    with Image.open(path) as saved:
        if saved.n_frames != len(TOWN05_GIF_INDICES) or saved.size != (2560, 800):
            raise ValueError("saved GIF frame contract changed")
    return path


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--gif-only", action="store_true")
    parser.add_argument("--gif-output", type=Path, default=OUTPUT / GIF_NAME)
    args = parser.parse_args()
    if args.gif_only:
        print(render_gif(args.gif_output))
    else:
        for item in render():
            print(item)
