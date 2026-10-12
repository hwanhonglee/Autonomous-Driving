#!/usr/bin/env python3
"""HH_260929 - Add geometry-only, ego-centered insets to C-track raw-tag evidence."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

from PIL import Image, ImageDraw

from scripts.e2e.diagnose_ctrack_semantic_capture import LEFT, OUTPUT, SELECTED, GIF_FRAMES, _panel


ROOT = Path(__file__).resolve().parents[2]
ROUTE = ROOT / ("storage/experiments/training/2026-09-08/turn_launch_matrix_v1/"
                "c_track_left/turn_launch_013_v1/run_002/episode/route.json")
ROUTE_SHA256 = "2299e5bf2bc86789650da72336fdcfe7c11c585df0b9e0a0fe29a44730bae79b"
CONTACT_NAME = "full_fov_with_centered_route_and_actual.png"
GIF_NAME = "sampled_with_centered_route_and_actual.gif"


def _input_geometry() -> tuple[list[tuple[float, float]], list[dict]]:
    # HH_260929 - Pin the actual archived route; base_link y is negated exactly as in the collector.
    if not ROUTE.is_file() or ROUTE.is_symlink() or not (LEFT / "records.json").is_file():
        raise ValueError("required archived route or pose records are absent")
    if hashlib.sha256(ROUTE.read_bytes()).hexdigest() != ROUTE_SHA256:
        raise ValueError("archived route bytes differ from the owned capture")
    route = json.loads(ROUTE.read_text(encoding="utf-8"))
    records = json.loads((LEFT / "records.json").read_text(encoding="utf-8"))
    if route.get("town") != "C_track_1_0_7" or route.get("scenario") != "left":
        raise ValueError("route scenario differs from the owned capture")
    if len(records) != 240 or len(route.get("route", [])) != 216:
        raise ValueError("archived route or actual path is incomplete")
    planned = [(float(point["x"]), -float(point["y"])) for point in route["route"]]
    for index, row in enumerate(records):
        if (row.get("stem") != f"frames/{index:04d}" or
                not all(math.isfinite(float(row[key])) for key in
                        ("ego_x", "ego_y", "ego_yaw_deg"))):
            raise ValueError(f"invalid archived ego pose at frame {index}")
    return planned, records


def _append_inset(panel: Image.Image, index: int,
                  planned: list[tuple[float, float]], records: list[dict]) -> Image.Image:
    width, height = panel.size
    extra = 220 if width == 1280 else 180
    result = Image.new("RGB", (width, height + extra), (13, 22, 32))
    result.paste(panel, (0, 0))
    draw = ImageDraw.Draw(result)
    plot_width = 500 if width == 1280 else 380
    plot_height = extra - 24
    plot_left = (width - plot_width) // 2
    plot_top = height + 12
    plot_cx = plot_width / 2
    plot_cy = plot_height / 2
    pixels_per_m = 2.6 if width == 1280 else 2.15
    ego = records[index]
    heading = math.radians(float(ego["ego_yaw_deg"]))
    hx, hy = math.cos(heading), math.sin(heading)
    plot = Image.new("RGB", (plot_width, plot_height), (21, 36, 49))
    plot_draw = ImageDraw.Draw(plot)

    def project(point: tuple[float, float]) -> tuple[float, float]:
        dx = point[0] - float(ego["ego_x"])
        dy = point[1] - float(ego["ego_y"])
        # HH_260929 - Ego is always centered; screen up is its current heading.
        right = -hy * dx + hx * dy
        forward = hx * dx + hy * dy
        return plot_cx + right * pixels_per_m, plot_cy - forward * pixels_per_m

    plot_draw.ellipse((plot_cx - 10 * pixels_per_m, plot_cy - 10 * pixels_per_m,
                       plot_cx + 10 * pixels_per_m, plot_cy + 10 * pixels_per_m),
                      outline=(55, 69, 81), width=1)
    plot_draw.line([project(point) for point in planned], fill=(66, 194, 232), width=3)
    actual = [(float(row["ego_x"]), float(row["ego_y"])) for row in records[:index + 1]]
    if len(actual) > 1:
        plot_draw.line([project(point) for point in actual], fill=(155, 225, 76), width=3)
    plot_draw.polygon(((plot_cx, plot_cy - 9), (plot_cx - 7, plot_cy + 7),
                       (plot_cx + 7, plot_cy + 7)), fill=(255, 255, 255))
    result.paste(plot, (plot_left, plot_top))
    draw.rectangle((plot_left, plot_top, plot_left + plot_width - 1,
                    plot_top + plot_height - 1), outline=(106, 119, 130))
    draw.text((12, height + 16), "BasicAgent/CARLA route + actual pose", fill=(245, 245, 245))
    draw.text((12, height + 36), "NOT learned / Autoware / HD map", fill=(245, 245, 245))
    draw.text((12, height + 61), "Ego fixed center; heading is up", fill=(232, 232, 232))
    draw.text((12, height + 83), "Cyan = pinned route", fill=(66, 194, 232))
    draw.text((12, height + 103), "Lime = traveled actual", fill=(155, 225, 76))
    draw.text((12, height + 125), "No road/lane semantic labels", fill=(235, 125, 165))
    draw.text((plot_left + 8, plot_top + 7), f"Frame {index:03d} | 10 m ring",
              fill=(245, 245, 245))
    return result


def main() -> None:
    # HH_260929 - Supplemental figures never replace full-FOV raw diagnostic originals.
    if not OUTPUT.is_dir() or OUTPUT.is_symlink():
        raise ValueError("raw semantic diagnostic must exist first")
    contact_path = OUTPUT / CONTACT_NAME
    gif_path = OUTPUT / GIF_NAME
    if any(path.exists() or path.is_symlink() for path in (contact_path, gif_path)):
        raise ValueError("supplemental evidence already exists; refuse overwrite")
    planned, records = _input_geometry()
    rows = [_append_inset(_panel(LEFT, index), index, planned, records)
            for index in SELECTED]
    contact = Image.new("RGB", (1280, 630 * len(rows)), "white")
    for index, row in enumerate(rows):
        contact.paste(row, (0, index * 630))
    contact.save(contact_path, optimize=True)
    gif_rows = [_append_inset(_panel(LEFT, index, size=(480, 270)),
                              index, planned, records).quantize(colors=128)
                for index in GIF_FRAMES]
    gif_rows[0].save(gif_path, save_all=True, append_images=gif_rows[1:],
                     duration=300, loop=0, disposal=2, optimize=False)
    print(json.dumps({"contact": str(contact_path), "gif": str(gif_path),
                      "frames": len(gif_rows), "status": "geometry-only; no map labels"}))


if __name__ == "__main__":
    main()
