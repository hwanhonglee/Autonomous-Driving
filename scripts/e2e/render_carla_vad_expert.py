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
    "grid": "#e2e7eb",
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


def _ego_centered_transform(
    ego_xy: tuple[float, float],
    ego_yaw: float,
    reference_xy: Sequence[tuple[float, float]],
    box: tuple[int, int, int, int],
):
    """Return a heading-up transform whose origin is always the panel center.

    ``reference_xy`` determines only the zoom.  In particular, adding a long
    route cannot move the ego marker away from the center as it did in the
    original whole-route view.
    """

    left, top, right, bottom = box
    cosine = math.cos(ego_yaw)
    sine = math.sin(ego_yaw)

    def to_local(point: tuple[float, float]) -> tuple[float, float]:
        delta_x = point[0] - ego_xy[0]
        delta_y = point[1] - ego_xy[1]
        # base_link convention: x is forward and y is left.
        return (
            cosine * delta_x + sine * delta_y,
            -sine * delta_x + cosine * delta_y,
        )

    local_reference = [to_local(point) for point in reference_xy]
    maximum_forward = max((abs(point[0]) for point in local_reference), default=0.0)
    maximum_lateral = max((abs(point[1]) for point in local_reference), default=0.0)
    # Keep a useful neighbourhood visible while allowing a 6.4 s trajectory
    # at road speed to zoom the panel out as needed.
    forward_extent_m = max(24.0, maximum_forward * 1.15)
    lateral_extent_m = max(12.0, maximum_lateral * 1.15)
    padding = 22
    half_width_px = max(1.0, 0.5 * (right - left) - padding)
    half_height_px = max(1.0, 0.5 * (bottom - top) - padding)
    scale = min(
        half_width_px / lateral_extent_m,
        half_height_px / forward_extent_m,
        14.0,
    )
    center_x = 0.5 * (left + right)
    center_y = 0.5 * (top + bottom)

    def transform(point: tuple[float, float]) -> tuple[int, int]:
        forward, lateral = to_local(point)
        return (
            int(round(center_x - lateral * scale)),
            int(round(center_y - forward * scale)),
        )

    return transform, scale


def _format_number(value: float) -> str:
    return f"{value:.3f}".rstrip("0").rstrip(".")


def _future_horizons(sample: Mapping[str, Any]) -> tuple[float, ...]:
    future = sample.get("future_expert")
    if not isinstance(future, Mapping) or "horizons_s" not in future:
        return ()
    raw_horizons = future["horizons_s"]
    if not isinstance(raw_horizons, Sequence) or isinstance(raw_horizons, (str, bytes)):
        raise ValueError("future_expert.horizons_s must be an array")
    horizons = tuple(float(value) for value in raw_horizons)
    positions = future.get("positions_xy")
    if isinstance(positions, Sequence) and not isinstance(positions, (str, bytes)):
        if len(horizons) != len(positions):
            raise ValueError("future horizons and positions must have equal length")
    if not horizons or any(not math.isfinite(value) or value <= 0.0 for value in horizons):
        raise ValueError("future horizons must be finite and positive")
    if any(second <= first for first, second in zip(horizons, horizons[1:])):
        raise ValueError("future horizons must be strictly increasing")
    return horizons


def _future_timing_summary(sample: Mapping[str, Any]) -> str:
    future = sample.get("future_expert")
    positions = future.get("positions_xy", ()) if isinstance(future, Mapping) else ()
    point_count = len(positions) if isinstance(positions, Sequence) else 0
    horizons = _future_horizons(sample)
    if not horizons:
        return f"timing not exported ({point_count} points)"

    horizon_text = f"{_format_number(horizons[0])}\u2013{_format_number(horizons[-1])} s"
    if len(horizons) < 2:
        return f"{horizon_text} ({len(horizons)} point)"
    intervals = [second - first for first, second in zip(horizons, horizons[1:])]
    mean_interval = sum(intervals) / len(intervals)
    tolerance = max(1.0e-6, mean_interval * 1.0e-3)
    if all(abs(interval - mean_interval) <= tolerance for interval in intervals):
        rate_text = _format_number(1.0 / mean_interval)
        return f"{rate_text} Hz, {horizon_text} ({len(horizons)} points)"
    return f"non-uniform, {horizon_text} ({len(horizons)} points)"


def _future_legend_label(sample: Mapping[str, Any]) -> str:
    horizons = _future_horizons(sample)
    if horizons:
        return f"future label (to {_format_number(horizons[-1])} s)"
    future = sample.get("future_expert")
    positions = future.get("positions_xy", ()) if isinstance(future, Mapping) else ()
    point_count = len(positions) if isinstance(positions, Sequence) else 0
    return f"future label ({point_count} points)"


def _route_length(route_xy: Sequence[tuple[float, float]]) -> float:
    return sum(
        math.hypot(second[0] - first[0], second[1] - first[1])
        for first, second in zip(route_xy, route_xy[1:])
    )


def _draw_polyline(
    draw: ImageDraw.ImageDraw,
    transform,
    points: Sequence[tuple[float, float]],
    fill: str,
    width: int,
) -> None:
    if len(points) >= 2:
        draw.line([transform(point) for point in points], fill=fill, width=width, joint="curve")


def _draw_ego_marker(draw: ImageDraw.ImageDraw, center: tuple[int, int]) -> None:
    """Draw a high-contrast, heading-up ego arrow around an exact center."""

    center_x, center_y = center
    draw.ellipse(
        (center_x - 19, center_y - 19, center_x + 19, center_y + 19),
        fill=COLORS["panel"],
        outline=COLORS["vehicle"],
        width=2,
    )
    outer = (
        (center_x, center_y - 17),
        (center_x - 11, center_y + 12),
        (center_x, center_y + 7),
        (center_x + 11, center_y + 12),
    )
    inner = (
        (center_x, center_y - 12),
        (center_x - 7, center_y + 8),
        (center_x, center_y + 4),
        (center_x + 7, center_y + 8),
    )
    draw.polygon(outer, fill="white")
    draw.polygon(inner, fill=COLORS["vehicle"])


def _draw_map_panel(
    canvas: Image.Image,
    box: tuple[int, int, int, int],
    route_xy: Sequence[tuple[float, float]],
    history_xy: Sequence[tuple[float, float]],
    recent_history_xy: Sequence[tuple[float, float]],
    future_xy: Sequence[tuple[float, float]],
    sample: Mapping[str, Any],
) -> None:
    """Draw an ego-centred local map plus a north-up full-route inset."""

    left, top, right, bottom = box
    panel_width = right - left
    panel_height = bottom - top
    panel = Image.new("RGB", (panel_width, panel_height), COLORS["panel"])
    draw = ImageDraw.Draw(panel)
    ego_xy = future_xy[0]
    ego_yaw = float(sample["pose_map_xyz_yaw"][3])
    transform, scale = _ego_centered_transform(
        ego_xy,
        ego_yaw,
        [ego_xy, *recent_history_xy, *future_xy[1:]],
        (0, 0, panel_width, panel_height),
    )
    center = transform(ego_xy)

    draw.line((center[0], 0, center[0], panel_height), fill=COLORS["grid"], width=1)
    draw.line((0, center[1], panel_width, center[1]), fill=COLORS["grid"], width=1)
    _draw_polyline(draw, transform, route_xy, COLORS["route"], 4)
    _draw_polyline(draw, transform, history_xy, COLORS["history"], 6)
    _draw_polyline(draw, transform, future_xy, COLORS["future"], 6)
    marker_stride = max(1, math.ceil(max(1, len(future_xy) - 1) / 16))
    marked_indices = set(range(1, len(future_xy), marker_stride))
    if len(future_xy) > 1:
        marked_indices.add(len(future_xy) - 1)
    for index in sorted(marked_indices):
        screen = transform(future_xy[index])
        draw.ellipse(
            (screen[0] - 3, screen[1] - 3, screen[0] + 3, screen[1] + 3),
            fill=COLORS["future"],
        )
    _draw_ego_marker(draw, center)
    draw.rounded_rectangle((10, 10, 202, 91), radius=7, fill="#ffffff", outline=COLORS["border"])
    legend_y = 19
    for color, label in (
        (COLORS["route"], "route"),
        (COLORS["history"], "expert history"),
        (COLORS["future"], _future_legend_label(sample)),
    ):
        draw.line((20, legend_y + 8, 47, legend_y + 8), fill=color, width=5)
        draw.text((55, legend_y), label, fill=COLORS["text"], font=_font(14))
        legend_y += 23

    inset_width = max(118, min(178, panel_width // 3))
    inset_height = max(92, min(132, panel_height // 4))
    inset = (
        panel_width - inset_width - 11,
        10,
        panel_width - 10,
        10 + inset_height,
    )
    draw.rounded_rectangle(inset, radius=7, fill="#ffffff", outline=COLORS["border"], width=2)
    route_length = _route_length(route_xy)
    route_progress = float(sample.get("route_progress_m", 0.0))
    progress_percent = 0.0 if route_length <= 0.0 else 100.0 * route_progress / route_length
    progress_percent = max(0.0, min(100.0, progress_percent))
    draw.text(
        (inset[0] + 8, inset[1] + 6),
        f"FULL ROUTE  {progress_percent:.0f}%",
        fill=COLORS["muted"],
        font=_font(12, True),
    )
    inset_inner = (inset[0] + 7, inset[1] + 25, inset[2] - 7, inset[3] - 7)
    inset_transform = _map_transform(route_xy, inset_inner)
    _draw_polyline(draw, inset_transform, route_xy, COLORS["route"], 2)
    _draw_polyline(draw, inset_transform, history_xy, COLORS["history"], 3)
    inset_vehicle = inset_transform(ego_xy)
    inset_heading = inset_transform(
        (ego_xy[0] + 3.0 * math.cos(ego_yaw), ego_xy[1] + 3.0 * math.sin(ego_yaw))
    )
    draw.line((*inset_vehicle, *inset_heading), fill=COLORS["vehicle"], width=2)
    draw.ellipse(
        (
            inset_vehicle[0] - 4,
            inset_vehicle[1] - 4,
            inset_vehicle[0] + 4,
            inset_vehicle[1] + 4,
        ),
        fill=COLORS["vehicle"],
        outline="white",
        width=1,
    )

    scale_bar_m = next(
        (distance for distance in (5, 10, 20, 50, 100) if distance * scale >= 42),
        100,
    )
    scale_bar_px = int(round(scale_bar_m * scale))
    scale_bar_px = min(scale_bar_px, max(1, panel_width - 40))
    scale_y = panel_height - 23
    draw.line((18, scale_y, 18 + scale_bar_px, scale_y), fill=COLORS["vehicle"], width=3)
    draw.line((18, scale_y - 4, 18, scale_y + 4), fill=COLORS["vehicle"], width=2)
    draw.line(
        (18 + scale_bar_px, scale_y - 4, 18 + scale_bar_px, scale_y + 4),
        fill=COLORS["vehicle"],
        width=2,
    )
    draw.text(
        (20, scale_y - 20),
        f"{scale_bar_m} m   EGO CENTERED / HEADING UP",
        fill=COLORS["muted"],
        font=_font(12, True),
    )
    canvas.paste(panel, (left, top))
    ImageDraw.Draw(canvas).rectangle(box, outline=COLORS["border"], width=1)


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

    route_xy = _route_xy(route)
    sample_time = float(sample["timestamp_ns"]) * 1.0e-9
    history_with_time = [
        (state_time(state), (state_pose(state)[0], state_pose(state)[1]))
        for state in states
        if state_time(state) <= sample_time + 1.0e-6
    ]
    history_xy = [point for _, point in history_with_time]
    recent_history_xy = [
        point for timestamp, point in history_with_time if timestamp >= sample_time - 4.0
    ]
    future_xy = _sample_future_world(sample)
    _draw_map_panel(
        canvas,
        map_box,
        route_xy,
        history_xy,
        recent_history_xy,
        future_xy,
        sample,
    )

    footer_top = height - margin - footer_height + 16
    draw.text(
        (margin, footer_top),
        f"Expert: CARLA BasicAgent   Labels: {_future_timing_summary(sample)}",
        fill=COLORS["text"],
        font=_font(18, True),
    )
    draw.text(
        (margin, footer_top + 30),
        "Anchor: base_link x-forward/y-left. Camera display is spatial; stored orders are explicit.",
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
