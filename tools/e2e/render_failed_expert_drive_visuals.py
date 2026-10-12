#!/usr/bin/env python3
"""HH_260928 - Render descriptive visuals from a failed CARLA expert episode.partial."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch


def read_jsonl(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text().splitlines() if line]


def arrow(ax, x: float, y: float, yaw: float, length: float, color: str) -> None:
    ax.add_patch(
        FancyArrowPatch(
            (x, y),
            (x + length * math.cos(yaw), y + length * math.sin(yaw)),
            arrowstyle="-|>",
            mutation_scale=18,
            linewidth=2.0,
            color=color,
            zorder=8,
        )
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode-partial", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    episode = args.episode_partial.resolve()
    output = args.output_dir.resolve()

    manifest = json.loads((episode / "manifest.json").read_text())
    if manifest.get("status") != "failed" or episode.name != "episode.partial":
        raise SystemExit("Expected a failed episode.partial, not an accepted dataset")

    route_data = json.loads((episode / "route.json").read_text())
    route = route_data["route"]
    states = read_jsonl(episode / "states.jsonl")
    camera = {row["frame"]: row for row in read_jsonl(episode / "camera_frames.jsonl")}
    driving = [row for row in states if row["capture_phase"] == "driving"]
    turning = [row for row in route if row["road_option"] in {"RIGHT", "LEFT"}]
    if not driving or not turning:
        raise SystemExit("Driving samples and a turn route are required")

    # HH_260928 - Choose the recorded camera nearest the middle of the turn for synchronized context.
    turn_mid_m = 0.5 * (turning[0]["distance_m"] + turning[-1]["distance_m"])
    turn_state = min(
        (row for row in driving if row["frame"] in camera),
        key=lambda row: abs(row["route_progress_m"] - turn_mid_m),
    )
    frame = turn_state["frame"]
    image_path = episode / camera[frame]["images"]["CAM_FRONT"]
    if not image_path.is_file():
        raise SystemExit(f"Missing synchronized camera image: {image_path}")

    # HH_260928 - Preserve the first rendered failure witnesses instead of silently replacing them.
    if output.exists():
        raise SystemExit(f"Refusing to overwrite existing diagnostic output: {output}")
    output.mkdir(parents=True, exist_ok=False)
    rx = [row["x"] for row in route]
    ry = [row["y"] for row in route]
    ax_ = [row["x"] for row in driving]
    ay_ = [row["y"] for row in driving]
    speed = math.hypot(turn_state["vx"], turn_state["vy"])
    goal = route_data["goal_ros_pose"]
    end = driving[-1]
    final_gap = math.hypot(end["x"] - goal["x"], end["y"] - goal["y"])

    # HH_260928 - Keep the complete camera pixels visible and center the synchronized ego in the route inset.
    fig, (cam_ax, local_ax) = plt.subplots(
        1, 2, figsize=(13.6, 5.4), gridspec_kw={"width_ratios": [1.25, 1]},
    )
    fig.subplots_adjust(left=0.035, right=0.975, bottom=0.17, top=0.84, wspace=0.15)
    fig.suptitle("Town01 J87 right turn | failed expert capture", fontsize=15, weight="bold")
    cam_ax.imshow(plt.imread(image_path))
    cam_ax.set_title(f"Recorded CAM_FRONT | frame {frame} | t={turn_state['timestamp']:.2f}s")
    cam_ax.set_axis_off()
    local_ax.plot(rx, ry, color="#2563eb", linewidth=2.1, label="catalog route")
    local_ax.plot(ax_, ay_, color="#dc2626", linewidth=1.8, label="recorded BasicAgent path")
    local_ax.scatter([turn_state["x"]], [turn_state["y"]], s=95, color="#e11d48", zorder=7)
    arrow(local_ax, turn_state["x"], turn_state["y"], turn_state["yaw"], 5.0, "#e11d48")
    local_ax.set_xlim(turn_state["x"] - 24, turn_state["x"] + 24)
    local_ax.set_ylim(turn_state["y"] - 24, turn_state["y"] + 24)
    local_ax.set_aspect("equal", adjustable="box")
    local_ax.grid(alpha=0.25)
    local_ax.set_xlabel("ROS map x (m)")
    local_ax.set_ylabel("ROS map y (m)")
    local_ax.set_title(
        f"Ego-centered route context | progress {turn_state['route_progress_m']:.1f} m"
    )
    local_ax.legend(loc="lower left", fontsize=8)
    fig.text(
        0.5, 0.035,
        f"BasicAgent expert diagnostic only  |  {speed * 3.6:.1f} km/h at frame  "
        "|  source: failed episode.partial  |  no learned E2E / Autoware control",
        ha="center", va="bottom", fontsize=9, color="#4b5563",
    )
    context_path = output / "turn_camera_and_ego_context.png"
    fig.savefig(context_path, dpi=160)
    plt.close(fig)

    # HH_260928 - Show the whole recorded path and the catalog goal without classifying a failed run as QA pass.
    fig, full_ax = plt.subplots(figsize=(9, 8), constrained_layout=True)
    fig.suptitle("Town01 J87 right turn | route versus recorded path", fontsize=15, weight="bold")
    full_ax.plot(rx, ry, color="#2563eb", linewidth=2.7, label="catalog route")
    full_ax.plot(ax_, ay_, color="#dc2626", linewidth=2.0, label="recorded BasicAgent path")
    full_ax.scatter([rx[0]], [ry[0]], marker="o", s=85, color="#16a34a", zorder=6, label="start")
    full_ax.scatter([goal["x"]], [goal["y"]], marker="*", s=165, color="#2563eb", zorder=7, label="goal")
    full_ax.scatter([end["x"]], [end["y"]], marker="x", s=125, color="#dc2626", linewidths=3, zorder=7, label="last state")
    full_ax.scatter(
        [turn_state["x"]], [turn_state["y"]], marker="o", s=95,
        facecolor="white", edgecolor="#e11d48", linewidth=2, zorder=7, label=f"camera frame {frame}",
    )
    full_ax.plot([end["x"], goal["x"]], [end["y"], goal["y"]], color="#7c3aed", linestyle=":", linewidth=1.8)
    full_ax.annotate(
        f"Final planar gap {final_gap:.3f} m\n(goal tolerance 2.500 m)",
        xy=(end["x"], end["y"]), xytext=(end["x"] + 16, end["y"] + 8),
        fontsize=9, arrowprops={"arrowstyle": "->", "color": "#4b5563"},
        bbox={"boxstyle": "round,pad=0.35", "facecolor": "white", "edgecolor": "#9ca3af"},
    )
    full_ax.set_aspect("equal", adjustable="box")
    full_ax.grid(alpha=0.25)
    full_ax.set_xlabel("ROS map x (m)")
    full_ax.set_ylabel("ROS map y (m)")
    full_ax.legend(loc="upper left", fontsize=9)
    full_ax.text(
        0.01, 0.01,
        "FAILED partial | visualization of raw actor positions, not a lane-containment verdict",
        transform=full_ax.transAxes, fontsize=8.5, color="#7f1d1d",
        bbox={"facecolor": "#fef2f2", "edgecolor": "#fecaca", "pad": 5},
    )
    trajectory_path = output / "route_vs_recorded_path.png"
    fig.savefig(trajectory_path, dpi=160)
    plt.close(fig)

    print(json.dumps({
        "camera_frame": frame,
        "camera_image": str(image_path),
        "turn_progress_m": turn_state["route_progress_m"],
        "final_planar_gap_m": final_gap,
        "visuals": [str(context_path), str(trajectory_path)],
    }, indent=2))


if __name__ == "__main__":
    main()
