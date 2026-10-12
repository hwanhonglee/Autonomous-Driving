#!/usr/bin/env python3
"""Plot the fixed QA_v3 peak curvature witness from pinned raw coordinates only."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


ROOT = Path(__file__).resolve().parents[6]
QA = Path(__file__).resolve().parent
EPISODE = ROOT / "storage/experiments/training/2026-09-27/drive_long_route_town01_right_s0002_p06_9kph_trial_v1/episode"
OUTPUT = QA / "topdown_raw_turn_peak.png"
MANIFEST = QA / "topdown_visual_manifest.json"


def sha(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> None:
    assert not OUTPUT.exists() and not MANIFEST.exists(), "fresh visual output required"
    report_path = QA / "summary.json"
    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert report["status"] == "FAIL_CLOSED" and report["dataset_admitted"] is False
    evidence = report["sources"]["raw_metadata"]
    for name in ("route.json", "states.jsonl"):
        assert sha(EPISODE / name) == evidence["episode/" + name]["sha256"], "raw input changed"
    route = json.loads((EPISODE / "route.json").read_text(encoding="utf-8"))["route"]
    states = [json.loads(line) for line in (EPISODE / "states.jsonl").open(encoding="utf-8")]
    state_by_frame = {row["frame"]: row for row in states}
    metric = report["future"]["original_nine_metrics"]["metrics"]["xy_curvature"]
    peak = metric["maximum"]
    assert metric["unique_episode_relative_100ms_violation_tick_count"] == 1
    assert peak["sample_id"] == "raw-frame:2777" and peak["point_index"] == 1
    anchor_frame = int(peak["sample_id"].split(":")[1])
    anchor = state_by_frame[anchor_frame]
    target_ns = peak["target_timestamp_ns"]
    target = min(states, key=lambda row: abs(round(row["timestamp"] * 1e9) - target_ns))
    assert target["frame"] == 2781 and abs(round(target["timestamp"] * 1e9) - target_ns) <= 5
    first = report["window"]["first_selected"]["frame"]
    last = report["window"]["last_selected"]["frame"]
    selected = [row for row in states if first <= row["frame"] <= last]
    ox, oy = anchor["x"], anchor["y"]
    relative = lambda rows: ([row["x"] - ox for row in rows], [row["y"] - oy for row in rows])
    route_window = [point for point in route if 30. <= point["distance_m"] <= 120.]
    fig, ax = plt.subplots(figsize=(8.5, 8.5), dpi=150)
    ax.plot(*relative(route), color="#b6bec8", linewidth=1.5, label="Original route (all)", zorder=1)
    ax.plot(*relative(route_window), color="#2584bd", linewidth=3,
            label="Original route, 30–120 m", zorder=2)
    ax.plot(*relative(selected), color="#238d63", linewidth=2,
            label="Measured base_link, selected interval", zorder=3)
    near = [state_by_frame[frame] for frame in (2777, 2779, 2781, 2783)]
    ax.plot(*relative(near), color="#b150b6", linewidth=3, marker="o", markersize=4,
            label="Measured 0.1 s steps at curvature peak", zorder=5)
    ax.scatter([0], [0], marker="s", s=100, color="#1c5347", edgecolor="white", linewidth=1,
               label="Ego at anchor 2777", zorder=7)
    ax.scatter([target["x"] - ox], [target["y"] - oy], marker="X", s=150,
               color="#d44136", edgecolor="white", linewidth=1, label="Failed tick, frame 2781", zorder=8)
    ax.arrow(0, 0, 4 * math.cos(anchor["yaw"]), 4 * math.sin(anchor["yaw"]),
             head_width=.6, head_length=.8, color="#1c5347", linewidth=2, length_includes_head=True, zorder=6)
    ax.set_xlim(-18, 18)
    ax.set_ylim(-18, 18)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("East of anchor base_link (m)")
    ax.set_ylabel("North of anchor base_link (m)")
    ax.set_title("Town01 RIGHT: raw path curvature witness\nOffline diagnostic, CARLA BasicAgent capture")
    ax.grid(alpha=.2)
    ax.legend(loc="upper left", fontsize=7, framealpha=.95)
    ax.text(.02, .02, "No intended-lane body-containment proof", transform=ax.transAxes,
            fontsize=8, color="#7e2830", bbox={"facecolor": "white", "edgecolor": "none", "alpha": .9})
    fig.tight_layout()
    fig.savefig(OUTPUT)
    plt.close(fig)
    MANIFEST.write_text(json.dumps({
        "schema": "carla.town01_long_right_topdown_visual.v1",
        "kind": "offline_raw_coordinate_diagnostic_not_carla_or_autoware_gui",
        "selection": "QA_v3 fixed selected first–last native frames; no failure removal",
        "anchor_frame": anchor_frame, "target_frame": target["frame"],
        "anchor_progress_m": anchor["route_progress_m"], "target_progress_m": target["route_progress_m"],
        "peak_xy_curvature_per_m": peak["value"], "original_limit_per_m": 0.2,
        "raw_summary_sha256": sha(report_path),
        "raw_route_sha256": sha(EPISODE / "route.json"),
        "raw_states_sha256": sha(EPISODE / "states.jsonl"),
        "plot_recipe_sha256": sha(Path(__file__)), "output_sha256": sha(OUTPUT),
        "scope": "Plot uses only raw route points, recorded base_link XY/yaw, and preselected frame limits. "
                 "It does not estimate lane boundaries, vehicle body containment, or learned control."},
        indent=2, allow_nan=False) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
