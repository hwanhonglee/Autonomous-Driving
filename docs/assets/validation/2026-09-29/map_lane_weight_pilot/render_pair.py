#!/usr/bin/env python3
"""HH_260929 - Render fixed first/last full-FOV TRAIN holdout oracle and paired predictions."""

from __future__ import annotations

from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

import verify_pair


PALETTE = {
    1: (24, 115, 235),
    2: (145, 82, 181),
    3: (255, 212, 35),
    4: (232, 65, 67),
    5: (224, 70, 173),
    6: (42, 180, 180),
}
OUTPUT = Path(__file__).resolve().parent / "visuals"


def font(size: int):
    """HH_260929 - Use a local readable font without fetching an asset."""
    path = Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")
    return ImageFont.truetype(str(path), size) if path.is_file() else ImageFont.load_default()


def overlay(rgb: np.ndarray, labels: np.ndarray) -> Image.Image:
    """HH_260929 - Preserve the original 640x360 FOV and tint all six named classes."""
    result = rgb.copy().astype(np.uint16)
    for index, color in PALETTE.items():
        mask = labels == index
        result[mask] = (result[mask] * 3 + np.asarray(color, dtype=np.uint16) * 2) // 5
    return Image.fromarray(result.astype(np.uint8), "RGB")


def render() -> list[Path]:
    verify_pair.replay()
    with np.load(verify_pair.DATA / "train_d0.npz", allow_pickle=False) as archive:
        rgb = archive["rgb"][[0, 113]].copy()
        target = archive["target"][[0, 113]].copy()
    predictions = {}
    for arm in ("B", "W2"):
        with np.load(verify_pair.RUN / f"{arm}_holdout_predictions.npz",
                     allow_pickle=False) as archive:
            predictions[arm] = archive["prediction"][[0, 113]].copy()
    OUTPUT.mkdir(exist_ok=False)
    paths = []
    for position, frame_index in enumerate((0, 113)):
        canvas = Image.new("RGB", (2560, 470), (245, 247, 251))
        draw = ImageDraw.Draw(canvas)
        draw.text((15, 8), f"Town01 historical route | heldout TRAIN frame {frame_index:03d}",
                  fill=(20, 30, 42), font=font(27))
        draw.text((1715, 15), "INTERNAL ONLY | NOT DEV/TEST | NOT HD MAP",
                  fill=(150, 35, 39), font=font(18))
        panels = [Image.fromarray(rgb[position]),
                  overlay(rgb[position], target[position]),
                  overlay(rgb[position], predictions["B"][position]),
                  overlay(rgb[position], predictions["W2"][position])]
        for column, (panel, label) in enumerate(zip(
                panels, ("RGB", "CARLA oracle", "B: base loss", "W2: lane x2 loss"))):
            x = column * 640
            draw.text((x + 12, 53), label, fill=(26, 38, 53), font=font(20))
            canvas.paste(panel, (x, 80))
        draw.text((15, 446), "Full 640x360 camera view in each panel | road blue, lane yellow; other classes tinted",
                  fill=(55, 67, 82), font=font(15))
        path = OUTPUT / f"heldout_{frame_index:03d}_rgb_oracle_b_w2.png"
        with path.open("xb") as stream:
            canvas.save(stream, format="PNG")
        paths.append(path)
    return paths


if __name__ == "__main__":
    for item in render():
        print(item)
