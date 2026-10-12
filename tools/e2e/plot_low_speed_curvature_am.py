#!/usr/bin/env python3
"""HH_260906 - Plot original low-speed curvature witnesses without changing admission."""

from __future__ import annotations

import argparse
from bisect import bisect_left
import gzip
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def read_witnesses(trial: Path, future: Path) -> list[dict]:
    """HH_260906 - Deduplicate overlapping horizons by measured 100 ms target tick."""
    states = [json.loads(line) for line in (trial / "episode/states.jsonl").open()]
    if any(a["timestamp"] >= b["timestamp"] for a, b in zip(states, states[1:])):
        raise ValueError(f"native state times are not strictly increasing: {trial}")
    stamps = [state["timestamp"] for state in states]
    by_tick: dict[int, list[tuple[dict, int]]] = {}
    with gzip.open(future, "rt") as stream:
        for line in stream:
            anchor = json.loads(line)
            diagnostic = anchor.get("diagnostic")
            if diagnostic is None:
                continue
            for step in diagnostic["steps"]:
                if step["metrics"]["xy_curvature"][1]:
                    by_tick.setdefault(step["episode_relative_100ms_tick"], []).append(
                        (step, anchor["frame"])
                    )
    output = []
    for tick, entries in sorted(by_tick.items()):
        step, _ = max(entries, key=lambda item: item[0]["metrics"]["xy_curvature"][0])
        target = step["target_timestamp_ns"] / 1e9
        index = bisect_left(stamps, target)
        index = min((max(index - 1, 0), min(index, len(states) - 1)),
                    key=lambda value: abs(stamps[value] - target))
        if index < 2 or abs(stamps[index] - target) > 1e-6:
            raise ValueError(f"future target is not on the original native timeline: {trial} tick {tick}")
        now, before = states[index], states[index - 2]
        distance = math.hypot(now["x"] - before["x"], now["y"] - before["y"])
        if distance <= 0:
            raise ValueError(f"curvature witness has no native displacement: {trial} tick {tick}")
        yaw_delta = math.atan2(math.sin(now["yaw"] - before["yaw"]),
                               math.cos(now["yaw"] - before["yaw"]))
        output.append({
            "tick": tick,
            "distance_m": distance,
            "xy_curvature": step["metrics"]["xy_curvature"][0],
            "body_yaw_change_per_distance": abs(yaw_delta) / distance,
            "region": "launch" if now["route_progress_m"] < 1.0 else
                      "near goal" if step["future_region"] == "near_goal" else "turn",
            "only_first_future_point": len(entries) == 1 and entries[0][0]["index"] == 0,
        })
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", action="append", nargs=3,
                        metavar=("LABEL", "TRIAL", "FUTURE"), required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if len(args.case) != 3 or [entry[0] for entry in args.case] != ["AK", "AL", "AM"]:
        parser.error("exactly AK, AL and AM are required in that order")

    cases = [(label, read_witnesses(Path(trial), Path(future)))
             for label, trial, future in args.case]
    colors = {"AK": "#377eb8", "AL": "#ff7f00", "AM": "#4daf4a"}
    fig, axes = plt.subplots(1, 2, figsize=(13, 5), constrained_layout=True)
    for ax, region in zip(axes, ("launch", "near goal")):
        for label, witnesses in cases:
            selected = [row for row in witnesses if row["region"] == region]
            for only_first in (False, True):
                subset = [row for row in selected if row["only_first_future_point"] == only_first]
                if not subset:
                    continue
                ax.scatter([row["distance_m"] for row in subset],
                           [row["xy_curvature"] for row in subset],
                           color=colors[label], marker="o" if only_first else "x", s=50,
                           label=f"{label} {'first future point only' if only_first else 'repeated windows'}")
        if region == "near goal":
            am = [row for label, witnesses in cases if label == "AM"
                  for row in witnesses if row["region"] == region
                  and row["body_yaw_change_per_distance"] > 0]
            ax.scatter([row["distance_m"] for row in am],
                       [row["body_yaw_change_per_distance"] for row in am],
                       color="#333333", marker="+", s=40, label="AM body-yaw change / distance")
        ax.axhline(0.2, color="#aa3333", linestyle="--", linewidth=1,
                   label="unchanged XY curvature limit 0.2/m")
        ax.set_xscale("log")
        ax.set_yscale("log")
        ax.set_xlabel("Measured base_link displacement in 100 ms (m)")
        ax.set_ylabel("Absolute heading change / displacement (1/m)")
        ax.set_title(f"{region.title()} — unique failed 100 ms target ticks")
        ax.grid(alpha=0.25, which="both")
        ax.legend(loc="best", fontsize=7)
    fig.suptitle("Original CARLA BasicAgent trajectories; diagnostic only, no data admission")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=160)
    plt.close(fig)


if __name__ == "__main__":
    main()
