#!/usr/bin/env python3

import argparse
import json
import math
import os
import tempfile
import textwrap
import warnings
from pathlib import Path

warnings.filterwarnings(
    "ignore", message="Unable to import Axes3D.*", category=UserWarning
)

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


COMMAND_NAMES = (
    "LEFT",
    "RIGHT",
    "STRAIGHT",
    "LANE_FOLLOW",
    "CHANGE_LANE_LEFT",
    "CHANGE_LANE_RIGHT",
)


def finite_number(value):
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(value)
    )


def load_json(path, label):
    source = Path(path).expanduser().resolve()
    try:
        payload = json.loads(source.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"failed to read {label} JSON {source}: {error}") from error
    if not isinstance(payload, dict):
        raise ValueError(f"{label} JSON must contain an object: {source}")
    return source, payload


def coordinates(items, label):
    points = []
    if not isinstance(items, list):
        raise ValueError(f"{label} must be a list")
    for index, item in enumerate(items):
        if not isinstance(item, dict):
            raise ValueError(f"{label}[{index}] must be an object")
        x = item.get("x")
        y = item.get("y")
        if not finite_number(x) or not finite_number(y):
            raise ValueError(f"{label}[{index}] has invalid x/y coordinates")
        points.append((float(x), float(y)))
    return points


def actual_coordinates(result):
    samples = result.get("actual_path", [])
    if samples:
        return coordinates(samples, "result.actual_path"), False

    points = []
    for key in ("initial", "final"):
        snapshot = result.get(key, {})
        position = snapshot.get("position") if isinstance(snapshot, dict) else None
        if not isinstance(position, dict):
            continue
        candidate = coordinates([position], f"result.{key}.position")[0]
        if not points or candidate != points[-1]:
            points.append(candidate)
    return points, True


def format_value(value, unit="", digits=2):
    if not finite_number(value):
        return "n/a"
    return f"{float(value):.{digits}f}{unit}"


def command_summary(commands):
    if not isinstance(commands, list) or not commands:
        return "n/a"
    labels = []
    for command in commands:
        if isinstance(command, int) and 0 <= command < len(COMMAND_NAMES):
            labels.append(COMMAND_NAMES[command])
        else:
            labels.append(str(command))
    return ", ".join(labels)


def render_route_result(route, result, output_path):
    reference = coordinates(route.get("route", []), "route.route")
    if len(reference) < 2:
        raise ValueError("route.route must contain at least two points")
    actual, used_snapshot_fallback = actual_coordinates(result)

    route_x, route_y = zip(*reference)
    actual_x = [point[0] for point in actual]
    actual_y = [point[1] for point in actual]
    success = result.get("success") is True
    actual_color = "#1769aa" if success else "#c23b32"
    status_color = "#17834c" if success else "#b42318"

    figure = plt.figure(
        figsize=(12.0, 7.0), facecolor="#f4f6f8", layout="constrained"
    )
    grid = figure.add_gridspec(1, 2, width_ratios=(3.6, 1.25))
    axis = figure.add_subplot(grid[0, 0])
    summary = figure.add_subplot(grid[0, 1])
    axis.set_facecolor("#ffffff")
    summary.set_facecolor("#f4f6f8")

    axis.plot(
        route_x,
        route_y,
        color="#16845b",
        linewidth=2.8,
        label="Reference route",
        zorder=2,
    )
    if actual:
        actual_label = (
            "Initial-to-final fallback"
            if used_snapshot_fallback
            else "Actual odometry path"
        )
        if len(actual) >= 2:
            axis.plot(
                actual_x,
                actual_y,
                color=actual_color,
                linewidth=2.2,
                linestyle="--" if used_snapshot_fallback else "-",
                label=actual_label,
                zorder=3,
            )
        else:
            axis.scatter(
                actual_x,
                actual_y,
                color=actual_color,
                s=35,
                label="Actual odometry position",
                zorder=3,
            )
        axis.scatter(
            actual_x[-1],
            actual_y[-1],
            marker="x",
            color=actual_color,
            linewidths=2.2,
            s=75,
            label="Final position",
            zorder=6,
        )

    start = reference[0]
    goal = reference[-1]
    axis.scatter(
        *start, marker="o", color="#20262e", edgecolor="white", s=75, zorder=5
    )
    axis.scatter(
        *goal,
        marker="*",
        color="#e58b00",
        edgecolor="#20262e",
        s=180,
        zorder=5,
    )
    axis.annotate(
        "Start", start, xytext=(7, 8), textcoords="offset points", fontsize=9
    )
    axis.annotate(
        "Goal", goal, xytext=(7, 8), textcoords="offset points", fontsize=9
    )

    all_x = list(route_x) + actual_x
    all_y = list(route_y) + actual_y
    x_min, x_max = min(all_x), max(all_x)
    y_min, y_max = min(all_y), max(all_y)
    span = max(x_max - x_min, y_max - y_min, 5.0)
    padding = max(0.75, span * 0.08)
    axis.set_xlim(x_min - padding, x_max + padding)
    axis.set_ylim(y_min - padding, y_max + padding)
    axis.set_aspect("equal", adjustable="box")
    axis.grid(True, color="#d8dee6", linewidth=0.7, alpha=0.8)
    axis.set_xlabel("Map X [m]")
    axis.set_ylabel("Map Y [m]")
    axis.legend(loc="best", frameon=True, framealpha=0.94, fontsize=9)

    town = str(route.get("town", "Unknown town"))
    scenario = str(route.get("scenario", "unknown scenario"))
    weather = str(route.get("weather", "unknown weather"))
    axis.set_title(
        f"{town} | {scenario} | {weather}",
        loc="left",
        fontsize=14,
        weight="bold",
    )

    metrics = result.get("metrics", {})
    final = result.get("final", {})
    route_length = route.get("route_length_m")
    if not finite_number(route_length):
        route_length = reference_distance(route)
    reason = str(result.get("reason", "no result reason"))
    commands = command_summary(metrics.get("commands_seen"))
    path_note = (
        "snapshot fallback" if used_snapshot_fallback else f"{len(actual)} samples"
    )
    maximum_correction = metrics.get("maximum_trajectory_correction_m")
    assisted = finite_number(maximum_correction) and maximum_correction > 1.0e-3
    status_label = "HYBRID PASS" if success and assisted else ("PASS" if success else "FAIL")

    summary.axis("off")
    summary.text(
        0.0,
        0.98,
        status_label,
        color=status_color,
        fontsize=22,
        weight="bold",
        va="top",
    )
    summary.text(
        0.0,
        0.90,
        textwrap.fill(reason, width=30),
        color="#343a40",
        fontsize=10,
        va="top",
        linespacing=1.35,
    )
    summary_lines = (
        ("Route length", format_value(route_length, " m")),
        ("Traveled", format_value(metrics.get("traveled_distance_m"), " m")),
        ("Remaining", format_value(final.get("remaining_distance_m"), " m")),
        ("Max |CTE|", format_value(metrics.get("maximum_absolute_cte_m"), " m")),
        (
            "Max correction",
            format_value(metrics.get("maximum_trajectory_correction_m"), " m"),
        ),
        ("Simulation", format_value(metrics.get("sim_elapsed_sec"), " s", digits=1)),
        ("Wall clock", format_value(metrics.get("wall_elapsed_sec"), " s", digits=1)),
        ("Commands", commands),
        ("Actual path", path_note),
    )
    y = 0.75
    for label, value in summary_lines:
        summary.text(0.0, y, label, color="#667085", fontsize=8.5, va="top")
        summary.text(
            0.0,
            y - 0.032,
            value,
            color="#20262e",
            fontsize=10.5,
            weight="bold",
            va="top",
            wrap=True,
        )
        y -= 0.085

    output = Path(output_path).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{output.stem}.", suffix=".png", dir=str(output.parent)
    )
    os.close(descriptor)
    try:
        figure.savefig(temporary_name, dpi=160, facecolor=figure.get_facecolor())
        os.replace(temporary_name, output)
    finally:
        plt.close(figure)
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
    return output


def reference_distance(route):
    total = 0.0
    previous = None
    for point in coordinates(route.get("route", []), "route.route"):
        if previous is not None:
            total += math.hypot(point[0] - previous[0], point[1] - previous[1])
        previous = point
    return total


def parse_args():
    parser = argparse.ArgumentParser(
        description="Render a CARLA VAD route test result as a PNG overview."
    )
    parser.add_argument("route", help="prepared CARLA route JSON")
    parser.add_argument("result", help="route_test.py result JSON")
    parser.add_argument(
        "--output",
        help="PNG output path (default: result path with a .png suffix)",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    _, route = load_json(args.route, "route")
    result_source, result = load_json(args.result, "result")
    output = args.output or str(result_source.with_suffix(".png"))
    rendered = render_route_result(route, result, output)
    print(f"Route result image: {rendered}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
