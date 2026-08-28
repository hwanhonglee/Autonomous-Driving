#!/usr/bin/env python3
"""Summarize repeated CARLA controller trials and screened alternatives."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import statistics

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402


REPEATED_GROUPS = (
    ("baseline", "Stock 0.24/0.27", "baseline_full_repeat"),
    ("delay009_tau015", "MPC 0.09/0.15", "mpc_delay009_tau015_repeat"),
    ("delay012_tau015", "MPC 0.12/0.15", "mpc_delay012_tau015_repeat"),
)
SCREEN_TRIALS = (
    "vad_soft_screen01",
    "vad_soft_temporal_screen01",
    "vad_soft_temporal_speed_screen01",
    "vad_hard_speed_screen01",
    "smart_mpc_nominal_ilqr_screen02",
)


def nested(payload, *keys, default=None):
    value = payload
    for key in keys:
        if not isinstance(value, dict) or key not in value:
            return default
        value = value[key]
    return value


def read_trial(path: Path) -> dict[str, object]:
    result = json.loads((path / "result.json").read_text(encoding="utf-8"))
    diagnosis = json.loads((path / "diagnosis.json").read_text(encoding="utf-8"))
    metrics = diagnosis["metrics"]
    return {
        "name": path.name,
        "path": str(path.resolve()),
        "success": bool(result["success"]),
        "reason": result["reason"],
        "actual_to_route_max_cte_m": nested(
            metrics, "tracking", "actual_to_route_cte_m", "max_abs"
        ),
        "actual_to_final_max_cte_m": nested(
            metrics, "tracking", "actual_to_final_cte_m", "max_abs"
        ),
        "inward_corner_cut_m": nested(
            metrics, "peak_corner_cut", "actual_to_route_signed_cte_m"
        ),
        "steer_command_peak_rad": nested(
            metrics, "mpc_diagnostic", "final_steer_command_rad", "max_abs",
            default=nested(
                metrics, "steering_tracking", "command_peak_abs_rad"
            ),
        ),
        "steer_command_p95_rad": nested(
            metrics, "mpc_diagnostic", "final_steer_command_rad", "p95_abs"
        ),
        "final_path_curvature_p95_per_m": nested(
            metrics, "final_path", "snapshot_peak_curvature_per_m", "p95_abs"
        ),
        "trajectory_age_p95_sec": nested(
            metrics, "tracking", "final_trajectory_age_sec", "p95_abs"
        ),
        "maximum_trajectory_correction_m": nested(
            result, "metrics", "maximum_trajectory_correction_m"
        ),
        "sim_elapsed_sec": nested(result, "metrics", "sim_elapsed_sec"),
    }


def numeric_summary(values: list[float]) -> dict[str, float]:
    return {
        "mean": statistics.fmean(values),
        "median": statistics.median(values),
        "min": min(values),
        "max": max(values),
    }


def summarize_group(key: str, label: str, trials: list[dict[str, object]]) -> dict:
    metric_keys = (
        "actual_to_route_max_cte_m",
        "actual_to_final_max_cte_m",
        "inward_corner_cut_m",
        "steer_command_peak_rad",
        "steer_command_p95_rad",
        "final_path_curvature_p95_per_m",
    )
    summary = {}
    for metric in metric_keys:
        values = [float(trial[metric]) for trial in trials if trial[metric] is not None]
        if values:
            summary[metric] = numeric_summary(values)
    return {
        "key": key,
        "label": label,
        "runs": len(trials),
        "passes": sum(bool(trial["success"]) for trial in trials),
        "trials": trials,
        "metrics": summary,
    }


def improvement_percent(baseline: float, candidate: float) -> float:
    return 100.0 * (baseline - candidate) / baseline


def build_summary(root: Path) -> dict:
    groups = []
    for key, label, prefix in REPEATED_GROUPS:
        trials = [read_trial(root / f"{prefix}0{repeat}") for repeat in (1, 2, 3)]
        groups.append(summarize_group(key, label, trials))

    screens = [read_trial(root / name) for name in SCREEN_TRIALS]
    baseline = groups[0]["metrics"]
    comparisons = {}
    for group in groups[1:]:
        comparisons[group["key"]] = {
            metric: improvement_percent(
                baseline[metric]["median"], group["metrics"][metric]["median"]
            )
            for metric in (
                "actual_to_route_max_cte_m",
                "actual_to_final_max_cte_m",
            )
        }

    return {
        "schema_version": 1,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "method": {
            "route": "Town01 fixed right turn, ClearNoon",
            "planner": "fast VAD, default lookahead 2.0 m and hard 0.5 m corridor",
            "acceptance": "three independent full-stack runs; strict route success retained",
            "excluded_from_statistics": {
                "mpc_delay009_tau015_visual01": (
                    "camera PNG compression reduced CARLA real-time factor"
                )
            },
        },
        "repeated_groups": groups,
        "median_improvement_percent_vs_baseline": comparisons,
        "screen_trials": screens,
        "decision": {
            "accepted_simulation_candidate": "delay009_tau015",
            "reason": (
                "3/3 strict route passes with lower median route and controller tracking "
                "error than stock"
            ),
            "hold_candidate": "delay012_tau015",
            "hold_reason": (
                "lowest tracking errors, but only 2/3 strict passes; one run stopped "
                "1.8 mm outside the direct 1.0 m goal tolerance"
            ),
            "vad_postprocessing": "rejected for this route after closed-loop screening",
            "smart_mpc": (
                "rejected: 1.03 m controller tracking error and stall 3.27 m before goal"
            ),
            "source_defaults_changed": False,
        },
    }


def render(summary: dict, output: Path) -> None:
    groups = summary["repeated_groups"]
    colors = ("#69727d", "#14866d", "#d28b18")
    panels = (
        ("actual_to_route_max_cte_m", "Inward route overshoot", "m"),
        ("actual_to_final_max_cte_m", "Controller tracking error", "m"),
        ("steer_command_p95_rad", "Steering command p95", "rad"),
    )
    fig, axes = plt.subplots(1, 3, figsize=(13.2, 4.8), constrained_layout=True)
    for axis, (metric, title, unit) in zip(axes, panels):
        for index, (group, color) in enumerate(zip(groups, colors)):
            values = np.asarray(
                [trial[metric] for trial in group["trials"]], dtype=float
            )
            axis.bar(
                index,
                float(np.median(values)),
                width=0.58,
                color=color,
                alpha=0.82,
                zorder=2,
            )
            offsets = np.linspace(-0.13, 0.13, len(values))
            for offset, value, trial in zip(offsets, values, group["trials"]):
                marker = "o" if trial["success"] else "X"
                axis.scatter(
                    index + offset,
                    value,
                    marker=marker,
                    s=52,
                    color="#15191e" if trial["success"] else "#b3261e",
                    zorder=3,
                )
        axis.set_title(title, fontsize=12, weight="bold")
        axis.set_ylabel(unit)
        axis.set_xticks(range(len(groups)), [group["label"] for group in groups])
        axis.grid(axis="y", color="#d9dde2", linewidth=0.8, zorder=1)
        axis.spines[["top", "right"]].set_visible(False)
    fig.suptitle(
        "CARLA right-turn closed-loop comparison | bars: median, dots: runs, X: strict fail",
        fontsize=14,
        weight="bold",
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=170, facecolor="white")
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("optimization_dir", type=Path)
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--output-png", type=Path)
    args = parser.parse_args()
    root = args.optimization_dir.expanduser().resolve()
    output_json = args.output_json or root / "optimization_summary.json"
    output_png = args.output_png or root / "optimization_comparison.png"
    summary = build_summary(root)
    output_json.write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    render(summary, output_png)
    print(f"summary: {output_json}")
    print(f"comparison: {output_png}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
