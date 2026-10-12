#!/usr/bin/env python3
"""Render a deterministic top-down VAD/controller replay from a compact ROS bag."""

from __future__ import annotations

import argparse
import bisect
import math
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import analyze_turn_dynamics as dynamics  # noqa: E402


COMMAND_NAMES = {
    0: "LEFT",
    1: "RIGHT",
    2: "STRAIGHT",
    3: "LANE FOLLOW",
    4: "CHANGE LEFT",
    5: "CHANGE RIGHT",
}


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, type=Path, help="rosbag2 directory")
    parser.add_argument("--route-file", required=True, type=Path)
    parser.add_argument("--output-gif", required=True, type=Path)
    parser.add_argument("--output-mp4", type=Path)
    parser.add_argument(
        "--crop",
        choices=("turn", "motion", "all"),
        default="turn",
        help="time and map extent to render (default: turn)",
    )
    parser.add_argument("--fps", type=float, default=5.0)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--padding-sec", type=float, default=1.0)
    parser.add_argument("--motion-threshold-mps", type=float, default=0.05)
    parser.add_argument("--turn-margin-m", type=float, default=5.0)
    parser.add_argument("--corridor-half-width-m", type=float, default=0.5)
    args = parser.parse_args()

    if args.output_gif.suffix.lower() != ".gif":
        parser.error("--output-gif must end in .gif")
    if args.output_mp4 is not None and args.output_mp4.suffix.lower() != ".mp4":
        parser.error("--output-mp4 must end in .mp4")
    if not math.isfinite(args.fps) or args.fps <= 0.0:
        parser.error("--fps must be finite and positive")
    if args.width < 640 or args.height < 360:
        parser.error("--width/--height must be at least 640x360")
    if args.width % 2 or args.height % 2:
        parser.error("--width/--height must be even for yuv420p MP4 output")
    for name in ("padding_sec", "motion_threshold_mps", "turn_margin_m"):
        value = getattr(args, name)
        if not math.isfinite(value) or value < 0.0:
            parser.error(f"--{name.replace('_', '-')} must be finite and non-negative")
    if not math.isfinite(args.corridor_half_width_m) or args.corridor_half_width_m <= 0.0:
        parser.error("--corridor-half-width-m must be finite and positive")
    return args


def _stamp_sec(record: dict[str, Any]) -> float:
    if "stamp_ns" not in record:
        raise ValueError("record has no message header stamp")
    return int(record["stamp_ns"]) * 1.0e-9


def _sort_by_stamp(records: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Sort by header stamp and keep the last received record for duplicate stamps."""
    ordered = sorted(records, key=lambda record: (_stamp_sec(record), int(record["bag_ns"])))
    deduplicated: list[dict[str, Any]] = []
    for record in ordered:
        if deduplicated and record["stamp_ns"] == deduplicated[-1]["stamp_ns"]:
            deduplicated[-1] = record
        else:
            deduplicated.append(record)
    return deduplicated


class StampedSeries:
    def __init__(self, records: list[dict[str, Any]]):
        self.records = _sort_by_stamp(records)
        self.times = [_stamp_sec(record) for record in self.records]

    def latest(self, time_sec: float) -> dict[str, Any] | None:
        index = bisect.bisect_right(self.times, time_sec) - 1
        return self.records[index] if index >= 0 else None


def _route_turn_range(
    route_progress: np.ndarray,
    route_command: np.ndarray,
    margin_m: float,
) -> tuple[float, float]:
    indices = np.flatnonzero(np.isin(route_command, (0, 1)))
    if not len(indices):
        raise ValueError("route has no LEFT or RIGHT command segment")
    return (
        max(float(route_progress[0]), float(route_progress[indices[0]]) - margin_m),
        min(float(route_progress[-1]), float(route_progress[indices[-1]]) + margin_m),
    )


def _odometry_arrays(
    odometry: list[dict[str, Any]],
    route_xy: np.ndarray,
    route_progress: np.ndarray,
) -> dict[str, np.ndarray]:
    records = _sort_by_stamp(odometry)
    xy = np.asarray([[record["x"], record["y"]] for record in records], dtype=float)
    cte, progress, route_yaw, _ = dynamics._project_to_polyline(
        xy, route_xy, route_progress
    )
    return {
        "time": np.asarray([_stamp_sec(record) for record in records], dtype=float),
        "x": xy[:, 0],
        "y": xy[:, 1],
        "yaw": np.unwrap(np.asarray([record["yaw"] for record in records], dtype=float)),
        "speed": np.asarray([record["speed"] for record in records], dtype=float),
        "cte": cte,
        "progress": progress,
        "route_yaw": route_yaw,
    }


def _select_interval(
    odometry: dict[str, np.ndarray],
    crop: str,
    route_progress: np.ndarray,
    route_command: np.ndarray,
    padding_sec: float,
    motion_threshold_mps: float,
    turn_margin_m: float,
) -> tuple[float, float, tuple[float, float] | None]:
    times = odometry["time"]
    if not len(times):
        raise ValueError("odometry is empty")

    turn_range: tuple[float, float] | None = None
    if crop == "all":
        selected = np.ones(len(times), dtype=bool)
    elif crop == "motion":
        selected = np.abs(odometry["speed"]) >= motion_threshold_mps
    else:
        turn_range = _route_turn_range(route_progress, route_command, turn_margin_m)
        selected = (
            np.isfinite(odometry["progress"])
            & (odometry["progress"] >= turn_range[0])
            & (odometry["progress"] <= turn_range[1])
        )
        moving = np.abs(odometry["speed"]) >= motion_threshold_mps
        if np.any(selected & moving):
            selected &= moving

    indices = np.flatnonzero(selected)
    if not len(indices):
        raise ValueError(f"no odometry samples matched --crop {crop}")
    start = max(float(times[0]), float(times[indices[0]]) - padding_sec)
    end = min(float(times[-1]), float(times[indices[-1]]) + padding_sec)
    if end <= start:
        raise ValueError("selected animation interval is empty")
    return start, end, turn_range


def _frame_times(start: float, end: float, fps: float) -> np.ndarray:
    count = int(math.floor((end - start) * fps + 1.0e-9)) + 1
    return start + np.arange(count, dtype=float) / fps


def _route_slice(
    route_xy: np.ndarray,
    route_progress: np.ndarray,
    turn_range: tuple[float, float] | None,
) -> np.ndarray:
    if turn_range is None:
        return route_xy
    mask = (route_progress >= turn_range[0]) & (route_progress <= turn_range[1])
    indices = np.flatnonzero(mask)
    if not len(indices):
        return route_xy
    begin = max(0, int(indices[0]) - 1)
    end = min(len(route_xy), int(indices[-1]) + 2)
    return route_xy[begin:end]


def _axis_limits(xy: np.ndarray, aspect: float) -> tuple[float, float, float, float]:
    minimum = np.min(xy, axis=0)
    maximum = np.max(xy, axis=0)
    center = 0.5 * (minimum + maximum)
    width = max(8.0, float(maximum[0] - minimum[0]) + 6.0)
    height = max(8.0, float(maximum[1] - minimum[1]) + 6.0)
    if width / height < aspect:
        width = height * aspect
    else:
        height = width / aspect
    return (
        float(center[0] - 0.5 * width),
        float(center[0] + 0.5 * width),
        float(center[1] - 0.5 * height),
        float(center[1] + 0.5 * height),
    )


def _corridor_polygon(route_xy: np.ndarray, half_width: float) -> np.ndarray:
    tangent = np.gradient(route_xy, axis=0)
    norm = np.linalg.norm(tangent, axis=1)
    norm[norm < 1.0e-9] = 1.0
    normal = np.column_stack((-tangent[:, 1] / norm, tangent[:, 0] / norm))
    left = route_xy + half_width * normal
    right = route_xy - half_width * normal
    return np.vstack((left, right[::-1]))


def _interpolate_odometry(odometry: dict[str, np.ndarray], time_sec: float) -> dict[str, float]:
    return {
        key: float(np.interp(time_sec, odometry["time"], odometry[key]))
        for key in ("x", "y", "yaw", "speed", "cte", "progress")
    }


def _snapshot_route_offset(
    record: dict[str, Any] | None,
    route_xy: np.ndarray,
    route_progress: np.ndarray,
) -> float | None:
    if record is None or len(record["xy"]) < 2:
        return None
    offset, _, _, _ = dynamics._project_to_polyline(record["xy"], route_xy, route_progress)
    finite = np.abs(offset[np.isfinite(offset)])
    return float(np.max(finite)) if len(finite) else None


def _actual_to_path_distance(
    actual_xy: np.ndarray, record: dict[str, Any] | None
) -> float | None:
    if record is None or len(record["xy"]) < 2:
        return None
    distance, _, _, _ = dynamics._project_to_polyline(actual_xy, record["xy"])
    return float(abs(distance[0])) if len(distance) and math.isfinite(distance[0]) else None


def _format_metric(value: float | None, unit: str) -> str:
    return "n/a" if value is None or not math.isfinite(value) else f"{value:.3f} {unit}"


def _command_at_progress(
    progress: float, route_progress: np.ndarray, route_command: np.ndarray
) -> int:
    index = int(np.searchsorted(route_progress, progress, side="right") - 1)
    return int(route_command[int(np.clip(index, 0, len(route_command) - 1))])


def _render_frames(
    frame_dir: Path,
    frame_times: np.ndarray,
    start_sec: float,
    route_xy: np.ndarray,
    route_progress: np.ndarray,
    route_command: np.ndarray,
    odometry: dict[str, np.ndarray],
    raw: StampedSeries,
    final: StampedSeries,
    predicted: StampedSeries,
    turn_range: tuple[float, float] | None,
    corridor_half_width_m: float,
    width: int,
    height: int,
) -> None:
    display_route = _route_slice(route_xy, route_progress, turn_range)
    plot_aspect = (width * 0.70) / height
    x_min, x_max, y_min, y_max = _axis_limits(display_route, plot_aspect)
    corridor = _corridor_polygon(display_route, corridor_half_width_m)
    cte_limit = max(1.25, float(np.nanmax(np.abs(odometry["cte"]))) * 1.15)
    dpi = 100

    figure = plt.figure(figsize=(width / dpi, height / dpi), dpi=dpi, facecolor="#f5f6f7")
    grid = figure.add_gridspec(
        2, 2, width_ratios=(2.35, 1.0), height_ratios=(2.0, 1.0),
        left=0.055, right=0.975, top=0.86, bottom=0.10, wspace=0.18, hspace=0.28,
    )
    map_axis = figure.add_subplot(grid[:, 0])
    metric_axis = figure.add_subplot(grid[0, 1])
    history_axis = figure.add_subplot(grid[1, 1])

    colors = {
        "route": "#2e7d32",
        "actual": "#111827",
        "raw": "#e67e22",
        "final": "#1976d2",
        "predicted": "#8e44ad",
        "ego": "#d32f2f",
    }

    for frame_index, time_sec in enumerate(frame_times, start=1):
        actual = _interpolate_odometry(odometry, float(time_sec))
        raw_record = raw.latest(float(time_sec))
        final_record = final.latest(float(time_sec))
        predicted_record = predicted.latest(float(time_sec))
        history = odometry["time"] <= time_sec

        map_axis.clear()
        metric_axis.clear()
        history_axis.clear()

        map_axis.fill(
            corridor[:, 0], corridor[:, 1], color="#c8e6c9", alpha=0.42,
            label=f"Route corridor (+/-{corridor_half_width_m:.1f} m)", zorder=0,
        )
        map_axis.plot(
            route_xy[:, 0], route_xy[:, 1], color=colors["route"], linewidth=2.6,
            label="CARLA route", zorder=1,
        )
        map_axis.plot(
            odometry["x"][history], odometry["y"][history], color=colors["actual"],
            linewidth=2.2, label="Actual trace", zorder=5,
        )
        for record, label, color, style, width_px in (
            (raw_record, "VAD selected raw", colors["raw"], "--", 2.0),
            (final_record, "Final controller input", colors["final"], "-", 2.4),
            (predicted_record, "MPC predicted", colors["predicted"], ":", 2.2),
        ):
            if record is not None and len(record["xy"]):
                map_axis.plot(
                    record["xy"][:, 0], record["xy"][:, 1], color=color,
                    linestyle=style, linewidth=width_px, label=label, zorder=4,
                )

        heading_length = 1.8
        map_axis.scatter(
            [actual["x"]], [actual["y"]], s=75, color=colors["ego"],
            edgecolor="white", linewidth=1.2, label="Ego", zorder=7,
        )
        map_axis.arrow(
            actual["x"], actual["y"],
            heading_length * math.cos(actual["yaw"]),
            heading_length * math.sin(actual["yaw"]),
            width=0.08, head_width=0.55, head_length=0.65,
            length_includes_head=True, color=colors["ego"], zorder=6,
        )
        map_axis.set_xlim(x_min, x_max)
        map_axis.set_ylim(y_min, y_max)
        map_axis.set_aspect("equal", adjustable="box")
        map_axis.set_xlabel("Map x [m]")
        map_axis.set_ylabel("Map y [m]")
        map_axis.grid(color="#b0b7bf", alpha=0.30)
        map_axis.legend(loc="best", fontsize=8, framealpha=0.94)

        command = _command_at_progress(actual["progress"], route_progress, route_command)
        elapsed = float(time_sec - start_sec)
        figure.suptitle(
            f"VAD -> route manager -> MPC -> CARLA | {COMMAND_NAMES.get(command, str(command))}",
            fontsize=15, fontweight="bold", color="#111827", y=0.965,
        )
        map_axis.set_title(
            f"Simulation +{elapsed:05.1f} s | speed {abs(actual['speed']):.2f} m/s",
            fontsize=11,
        )

        raw_offset = _snapshot_route_offset(raw_record, route_xy, route_progress)
        final_offset = _snapshot_route_offset(final_record, route_xy, route_progress)
        tracking_error = _actual_to_path_distance(
            np.asarray([[actual["x"], actual["y"]]]), final_record
        )
        metric_axis.axis("off")
        metric_axis.set_title("Current separation", loc="left", fontsize=11, fontweight="bold")
        lines = (
            ("Actual -> route CTE", f"{actual['cte']:+.3f} m", colors["actual"]),
            ("Actual -> final", _format_metric(tracking_error, "m"), colors["final"]),
            ("VAD raw max route offset", _format_metric(raw_offset, "m"), colors["raw"]),
            ("Final max route offset", _format_metric(final_offset, "m"), colors["final"]),
            ("Route progress", f"{actual['progress']:.1f} m", colors["route"]),
        )
        for index, (label, value, color) in enumerate(lines):
            y = 0.88 - index * 0.17
            metric_axis.text(0.0, y, label, transform=metric_axis.transAxes, fontsize=9, color="#4b5563")
            metric_axis.text(
                1.0, y, value, transform=metric_axis.transAxes, fontsize=11,
                fontweight="bold", color=color, ha="right",
            )

        history_axis.axhline(0.0, color="#6b7280", linewidth=0.8)
        history_axis.fill_between(
            [0.0, max(float(frame_times[-1] - start_sec), 0.2)],
            [-corridor_half_width_m, -corridor_half_width_m],
            [corridor_half_width_m, corridor_half_width_m],
            color="#c8e6c9", alpha=0.35,
        )
        history_axis.plot(
            odometry["time"][history] - start_sec,
            odometry["cte"][history], color=colors["actual"], linewidth=1.8,
        )
        history_axis.scatter([elapsed], [actual["cte"]], s=25, color=colors["ego"], zorder=3)
        history_axis.set_xlim(0.0, max(float(frame_times[-1] - start_sec), 0.2))
        history_axis.set_ylim(-cte_limit, cte_limit)
        history_axis.set_title("Actual route CTE", fontsize=10)
        history_axis.set_xlabel("Simulation time [s]", fontsize=8)
        history_axis.set_ylabel("Signed CTE [m]", fontsize=8)
        history_axis.tick_params(labelsize=8)
        history_axis.grid(alpha=0.25)

        output = frame_dir / f"frame_{frame_index:05d}.png"
        figure.savefig(output, dpi=dpi, facecolor=figure.get_facecolor())

    plt.close(figure)


def _run_ffmpeg(command: list[str]) -> None:
    result = subprocess.run(command, text=True, capture_output=True, check=False)
    if result.returncode != 0:
        detail = result.stderr.strip().splitlines()
        message = detail[-1] if detail else "unknown ffmpeg error"
        raise RuntimeError(f"ffmpeg failed: {message}")


def _encode_outputs(
    frame_dir: Path,
    fps: float,
    output_gif: Path,
    output_mp4: Path | None,
) -> None:
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        raise RuntimeError("ffmpeg is required to encode GIF/MP4 output")
    output_gif = output_gif.expanduser().resolve()
    output_gif.parent.mkdir(parents=True, exist_ok=True)
    pattern = str(frame_dir / "frame_%05d.png")
    _run_ffmpeg(
        [
            ffmpeg, "-hide_banner", "-loglevel", "error", "-y",
            "-framerate", f"{fps:g}", "-i", pattern,
            "-filter_complex",
            "[0:v]split[a][b];[a]palettegen=max_colors=128[p];[b][p]paletteuse",
            "-loop", "0", str(output_gif),
        ]
    )
    if output_mp4 is not None:
        output_mp4 = output_mp4.expanduser().resolve()
        output_mp4.parent.mkdir(parents=True, exist_ok=True)
        _run_ffmpeg(
            [
                ffmpeg, "-hide_banner", "-loglevel", "error", "-y",
                "-framerate", f"{fps:g}", "-i", pattern,
                "-c:v", "libx264", "-crf", "22", "-pix_fmt", "yuv420p",
                str(output_mp4),
            ]
        )


def _select_predicted_records(
    records: dict[str, list[dict[str, Any]]],
) -> list[dict[str, Any]]:
    predicted_topic, predicted_records = dynamics._first_topic_with_records(
        records, dynamics.PREDICTED_TOPICS
    )
    if predicted_topic is None:
        print(
            "warning: bag has no MPC predicted trajectory on "
            + ", ".join(dynamics.PREDICTED_TOPICS)
            + "; rendering without the controller prediction.",
            file=sys.stderr,
        )
    return predicted_records


def main() -> int:
    args = _parse_args()
    records, _ = dynamics._read_bag(args.bag)
    _, route_xy, route_progress, route_command = dynamics._load_route(args.route_file)
    missing = dynamics._missing_required_topics(records)
    if missing:
        raise RuntimeError("bag has no messages for required topics: " + ", ".join(missing))

    odometry = _odometry_arrays(records[dynamics.ODOMETRY_TOPIC], route_xy, route_progress)
    start, end, turn_range = _select_interval(
        odometry,
        args.crop,
        route_progress,
        route_command,
        args.padding_sec,
        args.motion_threshold_mps,
        args.turn_margin_m,
    )
    times = _frame_times(start, end, args.fps)
    raw = StampedSeries(records[dynamics.RAW_TOPIC])
    final = StampedSeries(records[dynamics.FINAL_TOPIC])
    predicted = StampedSeries(_select_predicted_records(records))

    with tempfile.TemporaryDirectory(prefix="autoware_turn_animation_") as directory:
        frame_dir = Path(directory)
        _render_frames(
            frame_dir,
            times,
            start,
            route_xy,
            route_progress,
            route_command,
            odometry,
            raw,
            final,
            predicted,
            turn_range,
            args.corridor_half_width_m,
            args.width,
            args.height,
        )
        _encode_outputs(frame_dir, args.fps, args.output_gif, args.output_mp4)

    duration = (len(times) - 1) / args.fps
    outputs = [str(args.output_gif.resolve())]
    if args.output_mp4 is not None:
        outputs.append(str(args.output_mp4.resolve()))
    print(
        f"Rendered {len(times)} frames ({duration:.1f} simulation seconds): "
        + ", ".join(outputs)
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
