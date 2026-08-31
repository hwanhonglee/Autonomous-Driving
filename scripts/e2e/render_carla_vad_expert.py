#!/usr/bin/env python3
"""Render a six-camera and top-down preview of a CARLA expert episode."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import shutil
import subprocess
import tempfile
from typing import Any, Mapping, Sequence

from PIL import Image, ImageDraw, ImageFont

try:
    from export_carla_vad_expert import read_jsonl, state_pose, state_time
except ModuleNotFoundError:
    from scripts.e2e.export_carla_vad_expert import read_jsonl, state_pose, state_time


DISPLAY_CAMERA_GRID = (
    ("CAM_FRONT_LEFT", "CAM_FRONT", "CAM_FRONT_RIGHT"),
    ("CAM_BACK_LEFT", "CAM_BACK", "CAM_BACK_RIGHT"),
)
COMMAND_NAMES = {
    0: "LEFT",
    1: "RIGHT",
    2: "STRAIGHT",
    3: "LANE FOLLOW",
    4: "CHANGE LEFT",
    5: "CHANGE RIGHT",
}
COLORS = {
    "background": "#f3f5f7",
    "panel": "#ffffff",
    "border": "#c8d0d8",
    "route": "#697581",
    "history": "#087e8b",
    "future": "#e4572e",
    "vehicle": "#1b1f23",
    "text": "#182026",
    "muted": "#59636d",
}


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode", required=True, type=Path)
    parser.add_argument("--export", required=True, type=Path, dest="export_dir")
    parser.add_argument("--output-png", required=True, type=Path)
    parser.add_argument("--output-gif", required=True, type=Path)
    parser.add_argument("--fps", type=float, default=5.0)
    parser.add_argument("--max-frames", type=int, default=100)
    parser.add_argument("--width", type=int, default=1600)
    parser.add_argument("--height", type=int, default=900)
    args = parser.parse_args()
    if args.output_png.suffix.lower() != ".png":
        parser.error("--output-png must end in .png")
    if args.output_gif.suffix.lower() != ".gif":
        parser.error("--output-gif must end in .gif")
    if not math.isfinite(args.fps) or args.fps <= 0.0:
        parser.error("--fps must be finite and positive")
    if args.max_frames < 2:
        parser.error("--max-frames must be at least 2")
    if args.width < 960 or args.height < 540:
        parser.error("--width/--height must be at least 960x540")
    return args


def _font(size: int, bold: bool = False) -> ImageFont.ImageFont:
    names = (
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
        if bold
        else "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
    )
    try:
        return ImageFont.truetype(names, size=size)
    except OSError:
        return ImageFont.load_default()


def _route_xy(route: Mapping[str, Any]) -> list[tuple[float, float]]:
    points = route.get("route")
    if not isinstance(points, list) or len(points) < 2:
        raise ValueError("route.json must contain at least two route points")
    result = []
    for point in points:
        if not isinstance(point, Mapping):
            raise ValueError("route point must be an object")
        result.append((float(point["x"]), float(point["y"])))
    return result


def _sample_future_world(sample: Mapping[str, Any]) -> list[tuple[float, float]]:
    pose = sample["pose_map_xyz_yaw"]
    x, y, yaw = float(pose[0]), float(pose[1]), float(pose[3])
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    points = [(x, y)]
    for local_x, local_y in sample["future_expert"]["positions_xy"]:
        points.append(
            (
                x + cosine * float(local_x) - sine * float(local_y),
                y + sine * float(local_x) + cosine * float(local_y),
            )
        )
    return points


def _map_transform(
    all_xy: Sequence[tuple[float, float]], box: tuple[int, int, int, int]
):
    left, top, right, bottom = box
    xs = [point[0] for point in all_xy]
    ys = [point[1] for point in all_xy]
    minimum_x, maximum_x = min(xs), max(xs)
    minimum_y, maximum_y = min(ys), max(ys)
    span_x = max(5.0, maximum_x - minimum_x)
    span_y = max(5.0, maximum_y - minimum_y)
    padding = 24
    scale = min((right - left - 2 * padding) / span_x, (bottom - top - 2 * padding) / span_y)
    center_x = 0.5 * (minimum_x + maximum_x)
    center_y = 0.5 * (minimum_y + maximum_y)
    canvas_center_x = 0.5 * (left + right)
    canvas_center_y = 0.5 * (top + bottom)

    def transform(point: tuple[float, float]) -> tuple[int, int]:
        return (
            int(round(canvas_center_x + (point[0] - center_x) * scale)),
            int(round(canvas_center_y - (point[1] - center_y) * scale)),
        )

    return transform


def _draw_polyline(
    draw: ImageDraw.ImageDraw,
    transform,
    points: Sequence[tuple[float, float]],
    fill: str,
    width: int,
) -> None:
    if len(points) >= 2:
        draw.line([transform(point) for point in points], fill=fill, width=width, joint="curve")


def _draw_camera(
    canvas: Image.Image,
    episode: Path,
    sample: Mapping[str, Any],
    name: str,
    box: tuple[int, int, int, int],
) -> None:
    left, top, right, bottom = box
    path = episode / sample["cameras"][name]["path"]
    with Image.open(path) as source:
        camera = source.convert("RGB")
        target_width = right - left
        target_height = bottom - top
        scale = max(target_width / camera.width, target_height / camera.height)
        # ``Image.Resampling`` was introduced after Ubuntu 22.04's Pillow
        # 9.0 package.  Keep the validation renderer compatible with both the
        # platform package and newer Pillow releases.
        resampling = getattr(Image, "Resampling", Image)
        resized = camera.resize(
            (int(math.ceil(camera.width * scale)), int(math.ceil(camera.height * scale))),
            resampling.BILINEAR,
        )
        crop_left = max(0, (resized.width - target_width) // 2)
        crop_top = max(0, (resized.height - target_height) // 2)
        camera = resized.crop(
            (crop_left, crop_top, crop_left + target_width, crop_top + target_height)
        )
    canvas.paste(camera, (left, top))
    overlay = Image.new("RGBA", (right - left, 34), (0, 0, 0, 165))
    canvas.paste(overlay, (left, top), overlay)
    draw = ImageDraw.Draw(canvas)
    draw.text((left + 10, top + 7), name.replace("CAM_", ""), fill="white", font=_font(16, True))
    draw.rectangle((left, top, right - 1, bottom - 1), outline=COLORS["border"], width=1)


def render_frame(
    episode: Path,
    route: Mapping[str, Any],
    states: Sequence[Mapping[str, Any]],
    sample: Mapping[str, Any],
    width: int,
    height: int,
) -> Image.Image:
    canvas = Image.new("RGB", (width, height), COLORS["background"])
    draw = ImageDraw.Draw(canvas)
    margin = 20
    header_height = 62
    footer_height = 94
    gap = 12
    left_width = int(width * 0.61)
    content_top = margin + header_height
    content_bottom = height - margin - footer_height
    camera_box = (margin, content_top, left_width, content_bottom)
    map_box = (left_width + gap, content_top, width - margin, content_bottom)

    title = f"CARLA expert / {sample.get('town', 'unknown')} / {sample.get('weather', 'unknown')}"
    draw.text((margin, margin), title, fill=COLORS["text"], font=_font(26, True))
    command = int(sample["vad_command_0based"])
    subtitle = (
        f"frame {sample['frame']}   command {COMMAND_NAMES.get(command, str(command))}   "
        f"progress {float(sample.get('route_progress_m', 0.0)):.1f} m   "
        f"CTE {float(sample.get('route_cte_m', 0.0)):.2f} m"
    )
    draw.text((margin, margin + 34), subtitle, fill=COLORS["muted"], font=_font(16))

    camera_width = (camera_box[2] - camera_box[0] - 2 * gap) // 3
    camera_height = (camera_box[3] - camera_box[1] - gap) // 2
    for row, names in enumerate(DISPLAY_CAMERA_GRID):
        for column, name in enumerate(names):
            left = camera_box[0] + column * (camera_width + gap)
            top = camera_box[1] + row * (camera_height + gap)
            right = camera_box[2] if column == 2 else left + camera_width
            bottom = camera_box[3] if row == 1 else top + camera_height
            _draw_camera(canvas, episode, sample, name, (left, top, right, bottom))

    draw.rectangle(map_box, fill=COLORS["panel"], outline=COLORS["border"], width=1)
    route_xy = _route_xy(route)
    sample_time = float(sample["timestamp_ns"]) * 1.0e-9
    history_xy = [
        (state_pose(state)[0], state_pose(state)[1])
        for state in states
        if state_time(state) <= sample_time + 1.0e-6
    ]
    future_xy = _sample_future_world(sample)
    transform = _map_transform(route_xy + history_xy + future_xy, map_box)
    _draw_polyline(draw, transform, route_xy, COLORS["route"], 4)
    _draw_polyline(draw, transform, history_xy, COLORS["history"], 6)
    _draw_polyline(draw, transform, future_xy, COLORS["future"], 6)
    vehicle = transform(future_xy[0])
    radius = 7
    draw.ellipse(
        (vehicle[0] - radius, vehicle[1] - radius, vehicle[0] + radius, vehicle[1] + radius),
        fill=COLORS["vehicle"],
        outline="white",
        width=2,
    )
    for point in future_xy[1:]:
        screen = transform(point)
        draw.ellipse((screen[0] - 4, screen[1] - 4, screen[0] + 4, screen[1] + 4), fill=COLORS["future"])
    legend_y = map_box[1] + 14
    for color, label in (
        (COLORS["route"], "route"),
        (COLORS["history"], "expert history"),
        (COLORS["future"], "future label (3.0 s)"),
    ):
        draw.line((map_box[0] + 14, legend_y + 8, map_box[0] + 42, legend_y + 8), fill=color, width=5)
        draw.text((map_box[0] + 50, legend_y), label, fill=COLORS["text"], font=_font(15))
        legend_y += 25

    footer_top = height - margin - footer_height + 16
    draw.text(
        (margin, footer_top),
        "Expert: CARLA BasicAgent   Labels: 0.5 s intervals, anchor base_link x-forward/y-left",
        fill=COLORS["text"],
        font=_font(18, True),
    )
    draw.text(
        (margin, footer_top + 30),
        "Camera display is spatial; dataset stores explicit private Tiny and public Bench2Drive orders.",
        fill=COLORS["muted"],
        font=_font(16),
    )
    return canvas


def _select_indices(samples: Sequence[Mapping[str, Any]], maximum: int) -> list[int]:
    if len(samples) <= maximum:
        return list(range(len(samples)))
    return sorted(
        {int(round(index * (len(samples) - 1) / (maximum - 1))) for index in range(maximum)}
    )


def _overview_index(samples: Sequence[Mapping[str, Any]]) -> int:
    maneuver = [
        index for index, sample in enumerate(samples) if int(sample["vad_command_0based"]) in (0, 1)
    ]
    return maneuver[len(maneuver) // 2] if maneuver else len(samples) // 2


def render_episode(
    episode: Path,
    export_dir: Path,
    output_png: Path,
    output_gif: Path,
    fps: float = 5.0,
    maximum_frames: int = 100,
    width: int = 1600,
    height: int = 900,
) -> dict[str, Any]:
    episode = episode.expanduser().resolve()
    export_dir = export_dir.expanduser().resolve()
    samples = read_jsonl(export_dir / "samples.jsonl")
    states = read_jsonl(episode / "states.jsonl")
    route = json.loads((episode / "route.json").read_text(encoding="utf-8"))
    output_png.parent.mkdir(parents=True, exist_ok=True)
    output_gif.parent.mkdir(parents=True, exist_ok=True)
    overview = render_frame(episode, route, states, samples[_overview_index(samples)], width, height)
    overview.save(output_png)
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        raise RuntimeError("ffmpeg is required to encode the preview GIF")
    indices = _select_indices(samples, maximum_frames)
    with tempfile.TemporaryDirectory(prefix="carla_expert_preview_") as temporary:
        frame_dir = Path(temporary)
        for output_index, sample_index in enumerate(indices):
            frame = render_frame(episode, route, states, samples[sample_index], width, height)
            frame.save(frame_dir / f"frame_{output_index:05d}.png", compress_level=1)
        command = [
            ffmpeg,
            "-hide_banner",
            "-loglevel",
            "error",
            "-y",
            "-framerate",
            f"{fps:.6f}",
            "-i",
            str(frame_dir / "frame_%05d.png"),
            "-filter_complex",
            "[0:v]split[a][b];[a]palettegen=max_colors=128[p];[b][p]paletteuse=dither=bayer",
            "-loop",
            "0",
            str(output_gif),
        ]
        result = subprocess.run(command, text=True, capture_output=True, check=False)
        if result.returncode != 0:
            raise RuntimeError(f"ffmpeg GIF encoding failed: {result.stderr.strip()}")
    return {
        "sample_count": len(samples),
        "rendered_frame_count": len(indices),
        "overview_sample_index": _overview_index(samples),
        "output_png": str(output_png),
        "output_gif": str(output_gif),
    }


def main() -> int:
    args = _parse_args()
    report = render_episode(
        args.episode,
        args.export_dir,
        args.output_png,
        args.output_gif,
        fps=args.fps,
        maximum_frames=args.max_frames,
        width=args.width,
        height=args.height,
    )
    print(
        f"PASS frames={report['rendered_frame_count']}/{report['sample_count']} "
        f"png={report['output_png']} gif={report['output_gif']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
