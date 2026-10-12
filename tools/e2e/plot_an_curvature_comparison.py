#!/usr/bin/env python3
"""HH_260906 - Compare measured AM/AN curvature witnesses without changing the frozen audit."""
from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
from matplotlib import pyplot as plt

from scripts.e2e.plot_low_speed_curvature_am import read_witnesses


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--am-trial", type=Path, required=True)
    parser.add_argument("--am-future", type=Path, required=True)
    parser.add_argument("--an-trial", type=Path, required=True)
    parser.add_argument("--an-future", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    trials = {
        "AM": read_witnesses(args.am_trial, args.am_future),
        "AN": read_witnesses(args.an_trial, args.an_future),
    }
    colors = {"AM": "#dc8c26", "AN": "#227dc4"}
    fig, axes = plt.subplots(1, 2, figsize=(13, 5), constrained_layout=True)
    for ax, region in zip(axes, ("launch", "near goal")):
        for label, rows in trials.items():
            selected = [row for row in rows if row["region"] == region]
            for first_only, marker in ((False, "x"), (True, "o")):
                subset = [row for row in selected if row["only_first_future_point"] == first_only]
                if subset:
                    ax.scatter([row["tick"] for row in subset],
                               [row["xy_curvature"] for row in subset],
                               s=66, alpha=.85, color=colors[label], marker=marker,
                               label=f"{label}: {'first point only' if first_only else 'repeated windows'}")
        am_count = sum(row["region"] == region for row in trials["AM"])
        an_count = sum(row["region"] == region for row in trials["AN"])
        ax.axhline(.2, color="#a33", linestyle="--", label="unchanged 0.2/m limit")
        ax.set_yscale("log")
        ax.set_ylim(.16, 160)
        ax.set_xlabel("Unique 100 ms episode tick")
        ax.set_ylabel("Absolute XY curvature (1/m)")
        ax.set_title(f"{region.title()}: AM {am_count} vs AN {an_count} failed ticks")
        ax.grid(alpha=.25, which="both")
        ax.legend(fontsize=8, loc="best")
    fig.suptitle("Same Town01 BasicAgent A/B: original 6.4 s future geometry; no data admission")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=160)
    plt.close(fig)


if __name__ == "__main__":
    main()
