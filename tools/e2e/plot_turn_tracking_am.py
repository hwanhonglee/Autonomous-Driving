#!/usr/bin/env python3
"""Plot measured rear-axle turn tracking for an owned three-trial A/B."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


# HH_260906 - Keep this plot descriptive; admission uses the frozen full-future audit.
TURN_BEGIN_M = 42.25
TURN_END_M = 58.67


def read_trial(path: Path) -> tuple[dict, list[dict]]:
    route = json.loads((path / "episode" / "route.json").read_text())
    states = [json.loads(line) for line in (path / "episode" / "states.jsonl").open()]
    if route.get("coordinate_reference") != "base_link":
        raise ValueError(f"Not a base_link route: {path}")
    return route, states


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trial", action="append", nargs=2, metavar=("LABEL", "PATH"), required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if len(args.trial) != 3:
        parser.error("exactly three trials are required")

    trials = []
    for label, raw in args.trial:
        route, states = read_trial(Path(raw))
        selected = [
            state for state in states
            if state.get("capture_phase") == "driving"
            and TURN_BEGIN_M <= state.get("route_progress_m", -1) <= TURN_END_M
        ]
        if not selected:
            raise ValueError(f"No states inside the turn window: {raw}")
        trials.append((label, route, selected))
    route_xy = [(point["x"], point["y"]) for point in trials[0][1]["route"]
                if 39 <= point["distance_m"] <= 62]
    fig, axes = plt.subplots(1, 2, figsize=(15, 6), constrained_layout=True)
    ax_xy, ax_cte = axes
    ax_xy.plot([p[0] for p in route_xy], [p[1] for p in route_xy],
               color="black", linestyle="--", linewidth=2, label="Catalog route")
    colors = ("#377eb8", "#ff7f00", "#4daf4a")
    for (label, _, states), color in zip(trials, colors, strict=True):
        xs = [s["x"] for s in states]
        ys = [s["y"] for s in states]
        progress = [s["route_progress_m"] for s in states]
        cte = [abs(s["route_cte_m"]) for s in states]
        ax_xy.plot(xs, ys, color=color, linewidth=2, label=f"{label} measured base_link")
        midpoint = len(states) // 2
        ax_xy.scatter([xs[midpoint]], [ys[midpoint]], s=48, color=color,
                      edgecolor="white", zorder=5)
        ax_cte.plot(progress, cte, color=color, linewidth=2, label=label)
    ax_xy.set_title("Same Town01 right turn: route and measured rear-axle tracks")
    ax_xy.set_xlabel("ROS-map x (m)")
    ax_xy.set_ylabel("ROS-map y (m)")
    ax_xy.set_aspect("equal", adjustable="box")
    ax_xy.grid(alpha=0.25)
    ax_xy.legend(loc="best", fontsize=8)
    ax_cte.axhline(0.10, color="#555555", linestyle=":", label="AM mean guard 0.10 m")
    ax_cte.axhline(0.30, color="#aa3333", linestyle=":", label="AM max guard 0.30 m")
    ax_cte.set_title("Measured route-center distance, not body containment")
    ax_cte.set_xlabel("Route progress (m)")
    ax_cte.set_ylabel("Absolute base_link CTE (m)")
    ax_cte.set_xlim(TURN_BEGIN_M, TURN_END_M)
    ax_cte.set_ylim(bottom=0)
    ax_cte.grid(alpha=0.25)
    ax_cte.legend(loc="best", fontsize=8)
    fig.suptitle("BasicAgent expert-only A/B — no learned driving or training admission", fontsize=14)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=160)
    plt.close(fig)


if __name__ == "__main__":
    main()
