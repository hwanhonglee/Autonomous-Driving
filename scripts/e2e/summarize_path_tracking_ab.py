#!/usr/bin/env python3
"""Summarize two repeated path-tracking trial groups and render an A/B plot."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import statistics
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402


METRICS = (
    (
        "route_cte_max_m",
        ("metrics", "tracking", "actual_to_route_cte_m", "max_abs"),
        "Route CTE max",
        "m",
    ),
    (
        "route_cte_p95_m",
        ("metrics", "tracking", "actual_to_route_cte_m", "p95_abs"),
        "Route CTE p95",
        "m",
    ),
    (
        "final_cte_p95_m",
        ("metrics", "tracking", "actual_to_final_cte_m", "p95_abs"),
        "Actual-to-final CTE p95",
        "m",
    ),
    (
        "path_curvature_p95_per_m",
        (
            "metrics",
            "final_path",
            "snapshot_peak_curvature_per_m",
            "p95_abs",
        ),
        "Final-path curvature p95",
        "1/m",
    ),
    (
        "steer_command_p95_rad",
        ("metrics", "mpc_diagnostic", "final_steer_command_rad", "p95_abs"),
        "Steering command p95",
        "rad",
    ),
    (
        "final_yaw_error_p95_rad",
        (
            "metrics",
            "tracking",
            "actual_to_final_yaw_error_rad",
            "p95_abs",
        ),
        "Actual-to-final yaw p95",
        "rad",
    ),
)


def nested(payload: dict[str, Any], keys: tuple[str, ...]) -> Any:
    value: Any = payload
    for key in keys:
        value = value[key]
    return value


def read_trial(path: Path) -> dict[str, Any]:
    result = json.loads((path / "result.json").read_text(encoding="utf-8"))
    diagnosis = json.loads((path / "diagnosis.json").read_text(encoding="utf-8"))
    trial = {
        "name": path.name,
        "path": str(path.resolve()),
        "success": bool(result["success"]),
        "reason": result["reason"],
    }
    for key, source, _title, _unit in METRICS:
        trial[key] = float(nested(diagnosis, source))
    return trial


def read_group(root: Path, prefix: str, label: str) -> dict[str, Any]:
    paths = sorted(path for path in root.glob(f"{prefix}*") if path.is_dir())
    if not paths:
        raise RuntimeError(f"no trial directories match {prefix!r} in {root}")
    trials = [read_trial(path) for path in paths]
    metrics = {}
    for key, _source, _title, _unit in METRICS:
        values = [trial[key] for trial in trials]
        metrics[key] = {
            "mean": statistics.fmean(values),
            "median": statistics.median(values),
            "min": min(values),
            "max": max(values),
            "values": values,
        }
    return {
        "label": label,
        "prefix": prefix,
        "runs": len(trials),
        "passes": sum(trial["success"] for trial in trials),
        "trials": trials,
        "metrics": metrics,
    }


def build_summary(
    root: Path,
    baseline_prefix: str,
    baseline_label: str,
    candidate_prefix: str,
    candidate_label: str,
) -> dict[str, Any]:
    baseline = read_group(root, baseline_prefix, baseline_label)
    candidate = read_group(root, candidate_prefix, candidate_label)
    changes = {}
    gates = {
        "all_candidate_runs_pass": candidate["passes"] == candidate["runs"],
    }
    for key, _source, _title, _unit in METRICS:
        baseline_mean = baseline["metrics"][key]["mean"]
        candidate_mean = candidate["metrics"][key]["mean"]
        change = 100.0 * (candidate_mean - baseline_mean) / baseline_mean
        changes[key] = change
        gates[f"{key}_non_worsening"] = candidate_mean <= baseline_mean
    accepted = all(gates.values())
    return {
        "schema_version": 1,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "root": str(root),
        "groups": {"baseline": baseline, "candidate": candidate},
        "candidate_mean_change_percent": changes,
        "acceptance_gates": gates,
        "decision": "ACCEPT" if accepted else "HOLD",
        "decision_rule": "all runs pass and every reported lower-is-better metric is non-worsening",
    }


def render(summary: dict[str, Any], output: Path, title: str) -> None:
    baseline = summary["groups"]["baseline"]
    candidate = summary["groups"]["candidate"]
    changes = summary["candidate_mean_change_percent"]
    colors = ("#56616d", "#167c80")
    fig, axes = plt.subplots(2, 3, figsize=(14.4, 8.0), constrained_layout=True)

    for axis, (key, _source, metric_title, unit) in zip(axes.flat, METRICS):
        groups = (baseline, candidate)
        for index, (group, color) in enumerate(zip(groups, colors)):
            values = np.asarray(group["metrics"][key]["values"], dtype=float)
            axis.bar(
                index,
                float(np.mean(values)),
                color=color,
                width=0.58,
                alpha=0.88,
                zorder=2,
            )
            offsets = np.linspace(-0.12, 0.12, len(values))
            axis.scatter(
                index + offsets,
                values,
                color="#111820",
                edgecolors="white",
                linewidths=0.8,
                s=48,
                zorder=3,
            )
        change = changes[key]
        change_color = "#177245" if change <= 0.0 else "#b33a2b"
        axis.set_title(metric_title, fontsize=11.5, weight="bold")
        axis.text(
            0.98,
            0.96,
            f"candidate {change:+.1f}%",
            color=change_color,
            ha="right",
            va="top",
            transform=axis.transAxes,
            fontsize=10,
            weight="bold",
        )
        axis.set_ylabel(unit)
        axis.set_xticks((0, 1), (baseline["label"], candidate["label"]))
        axis.grid(axis="y", color="#d9dde2", linewidth=0.8, zorder=1)
        axis.spines[["top", "right"]].set_visible(False)
        axis.set_ylim(bottom=0.0)

    decision = summary["decision"]
    fig.suptitle(
        f"{title} | decision: {decision} | bars: mean, dots: independent runs",
        fontsize=15,
        weight="bold",
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=170, facecolor="white")
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trial_root", type=Path)
    parser.add_argument("--baseline-prefix", default="right_baseline_")
    parser.add_argument("--baseline-label", default="Recommended")
    parser.add_argument("--candidate-prefix", default="right_outlier_gate_")
    parser.add_argument("--candidate-label", default="Outlier gate")
    parser.add_argument("--title", default="CARLA right-turn path tracking A/B")
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--output-png", type=Path)
    args = parser.parse_args()

    root = args.trial_root.expanduser().resolve()
    output_json = args.output_json or root / "decision.json"
    output_png = args.output_png or root / "decision.png"
    summary = build_summary(
        root,
        args.baseline_prefix,
        args.baseline_label,
        args.candidate_prefix,
        args.candidate_label,
    )
    output_json.parent.mkdir(parents=True, exist_ok=True)
    output_json.write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    render(summary, output_png, args.title)
    print(f"decision: {summary['decision']}")
    print(f"summary:  {output_json}")
    print(f"plot:     {output_png}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
