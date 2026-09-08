#!/usr/bin/env python3
"""HH_260906 - Plot measured common objectives and separate composite regret without model loading or promotion."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from portable_e2e.contract import _loads_json
from scripts.e2e import render_portable_no_accel_learning as legacy
from scripts.e2e import summarize_portable_candidate_regret as campaign

ROOT = Path(__file__).resolve().parents[2]
ARMS, SEEDS = tuple(campaign.ARMS), tuple(campaign.base.SEEDS)
COMMON_METRICS = ("regression_loss", "selected_ade_m")
AUXILIARY = "candidate_regret_loss"
VAL_METRICS = legacy.VAL_METRICS
require, read, checked_sha, finite = legacy.require, legacy.read, legacy.checked_sha, legacy.finite


def history(payload, arm):
    # HH_260906 - Reuse all 1540-step/6155-exposure checks, but never plot the different A/B total objectives.
    require(arm in ARMS, "unreviewed plotting arm")
    rows, counted_bins = legacy.history(payload)
    enabled = arm == ARMS[1]
    metrics = (*COMMON_METRICS, AUXILIARY) if enabled else COMMON_METRICS
    for row in rows:
        require((AUXILIARY in row) == enabled, "auxiliary history presence differs from its fixed arm")
        for name in metrics:
            finite(row.get(name))
    bins = []
    for counted in counted_bins:
        group = rows[counted["first_step"] - 1:counted["last_step"]]
        values = {name: math.fsum(row[name] * row["batch_domain_sample_counts"]["carla"] for row in group)
                  / counted["sample_exposures"] for name in metrics}
        bins.append({k: v for k, v in counted.items() if k not in legacy.TRAIN_METRICS} | values)
    return rows, bins


def validate_summary(report):
    require(report.get("schema") == campaign.SCHEMA and report.get("status") == "COMPLETE_NOT_PROMOTED"
        and all(report.get(k) is False for k in ("automatic_promotion", "vehicle_control_approved", "test_evaluated",
            "test_used_for_training_or_selection", "training_data_approved_by_this_report")), "complete denied train/val-only summary required")
    require([p.get("seed") for p in report.get("pairs", [])] == list(SEEDS), "all three paired seeds required")
    require(report.get("completed_stage_count") == 18 and len(report.get("stages", [])) == 18
        and all(s.get("status") == "COMPLETE" for s in report["stages"]), "all eighteen stages required")
    for pair in report["pairs"]:
        require(pair.get("candidate_screen") in ("PASS", "FAIL") and pair.get("absolute_quality") in ("PASS", "FAIL"), "pair verdict missing")
        for label, arm in zip(("baseline", "candidate"), ARMS):
            run = pair[label]
            require(run["arm"] == arm and run["training_dataset_size"] == 1147 and run["validation_sample_count"] == 337,
                "model arm or denominator differs")
            for name in VAL_METRICS: finite(run["metrics"][name])
            geometry = run["geometry"]
            require(type(geometry["selected_pass_count"]) is int and 0 <= geometry["selected_pass_count"] <= 337
                and geometry["sample_count"] == 337, "invalid geometry count")


def draw(output, report, measurements):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    colors = ("#2468a2", "#bf721a", "#28875b")
    fig, axes = plt.subplots(3, 1, figsize=(16, 12))
    for index, item in enumerate(measurements):
        bins, candidate = item["bins"], item["arm"] == ARMS[1]
        x = [(b["first_step"] + b["last_step"]) / 2 for b in bins]
        label = f"{item['seed']}: {'B auxiliary 0.1' if candidate else 'A original 0.0'}"
        for axis, name in zip(axes[:2], COMMON_METRICS):
            axis.plot(x, [b[name] for b in bins], color=colors[index // 2], linestyle="--" if candidate else "-", marker=".", label=label)
        if candidate:
            axes[2].plot(x, [b[AUXILIARY] for b in bins], color=colors[index // 2], marker=".", label=str(item["seed"]))
    titles = ("Common regression loss (same formula in both arms)", "Selected training ADE (m)",
              "B only: unweighted expected composite-cost regret (not ADE regret)")
    for axis, title in zip(axes, titles):
        axis.set(title=title, xlabel="Optimizer step", ylabel="Sample-weighted mean", xlim=(1, 1540))
        axis.axvspan(1500.5, 1540.5, color="gray", alpha=.12); axis.grid(alpha=.2)
    axes[2].legend(loc="upper right", ncol=3, fontsize=9)
    fig.legend(*axes[0].get_legend_handles_labels(), loc="upper center", bbox_to_anchor=(.5, .957), ncol=3, fontsize=9)
    fig.suptitle("Measured training: all 6 fresh fits / 1,540 steps each", fontsize=16)
    fig.text(.5, .047, "All batches included: 100-step weighted bins; final shaded bin = 40 steps / 160 exposures. No intermediate validation.", ha="center", fontsize=10)
    fig.text(.5, .024, "Each fit = 5 full epochs + 105 batches / 6,155 exposures. Different A/B total losses are deliberately not compared.", ha="center", fontsize=10)
    fig.subplots_adjust(top=.875, bottom=.12, left=.085, right=.975, hspace=.42)
    fig.savefig(output / "01_common_learning_and_auxiliary.png", dpi=120); plt.close(fig)
    fig, axes = plt.subplots(2, 2, figsize=(16, 10))
    for axis, name in zip(axes.flat, (*VAL_METRICS, "geometry.selected_pass_count")):
        peak = 0.
        for index, pair in enumerate(report["pairs"]):
            for offset, label, color in ((-.18, "baseline", "#2468a2"), (.18, "candidate", "#bf721a")):
                run = pair[label]
                value = run["geometry"]["selected_pass_count"] if name.startswith("geometry") else run["metrics"][name]
                axis.bar(index + offset, value, .34, color=color, label="A original 0.0" if label == "baseline" else "B auxiliary 0.1")
                axis.text(index + offset, value, f"{value}/337" if name.startswith("geometry") else f"{value:.3f}", ha="center", va="bottom", fontsize=9)
                peak = max(peak, value)
        ticks = [f"{p['seed']}\nRelative {p['candidate_screen']} / Absolute {p['absolute_quality']}" for p in report["pairs"]]
        axis.set_xticks(range(3), ticks, fontsize=8)
        axis.set(ylabel=name, ylim=(0, max(.01, peak) * 1.20)); axis.grid(axis="y", alpha=.2)
    handles, labels = axes.flat[0].get_legend_handles_labels()
    fig.legend(handles[:2], labels[:2], loc="upper center", bbox_to_anchor=(.5, .94), ncol=2)
    fig.suptitle("Final checkpoint val337 only | all three pairs | no learned closed-loop evidence", fontsize=15)
    fig.text(.5, .035, f"Relative screen: {report['candidate_screen']} | Absolute quality: {report['absolute_quality']} | No automatic promotion", ha="center")
    fig.subplots_adjust(top=.84, bottom=.13, left=.08, right=.975, hspace=.3, wspace=.23)
    fig.savefig(output / "02_paired_validation.png", dpi=120); plt.close(fig)


def render(root, summary_path, output):
    root, summary_path, output = (Path(p).absolute() for p in (root, summary_path, output))
    require(not output.exists() and all(not p.is_symlink() for p in (output, *output.parents))
        and not any(output.resolve().is_relative_to(p) for p in (root.resolve(), summary_path.parent.resolve(), (ROOT / "datasets").resolve())),
        "fresh output must be outside inputs and datasets")
    summary_bytes = read(summary_path)
    report = _loads_json(summary_bytes.decode(), "candidate regret summary")
    require(campaign.canonical(report) == campaign.canonical(campaign.summarize_campaign(root)), "summary differs from freshly verified campaign")
    validate_summary(report)
    pins = {}
    for entry in report["input_manifest"]:
        name, relative = entry["path"], Path(entry["path"])
        require(not relative.is_absolute() and ".." not in relative.parts and name not in pins, "unsafe/duplicate input path")
        require(checked_sha(root / name) == entry["sha256"], "summary input SHA mismatch")
        pins[name] = entry["sha256"]
    modules = (legacy, campaign, campaign.shared, campaign.base, campaign.expansion)
    files = {Path(__file__).resolve(), *(Path(m.__file__).resolve() for m in modules)}
    sources = {p.relative_to(ROOT).as_posix(): campaign.sha_file(p) for p in files}
    measurements = []
    for pair in report["pairs"]:
        for label, arm in zip(("baseline", "candidate"), ARMS):
            prefix = f"seed_{pair['seed']}/{arm}"
            name = prefix + "/training/metrics.jsonl"
            payload = read(root / name); rows, bins = history(payload, arm)
            pins[name] = campaign.base._sha(payload)
            training = _loads_json(read(root / prefix / "training/run.json").decode(), "training report")
            evaluation = _loads_json(read(root / prefix / "evaluation/metrics.json").decode(), "evaluation report")
            require(campaign.canonical(rows[-1]) == campaign.canonical(training.get("last_metrics")), "history final row differs from training report")
            require(all(evaluation.get("metrics", {}).get(n) == pair[label]["metrics"][n] for n in VAL_METRICS), "raw validation value differs")
            require(campaign.auxiliary_diagnostics(training, evaluation, arm) == pair[label]["auxiliary_diagnostics"], "raw auxiliary evidence differs")
            measurements.append({"seed": pair["seed"], "arm": arm, "bins": bins, "optimizer_steps": 1540,
                "sample_exposures": 6155, "complete_epochs": 5, "partial_epoch_batches": 105, "partial_epoch_exposures": 420})
    output.mkdir(parents=True, exist_ok=False); draw(output, report, measurements)
    require(read(summary_path) == summary_bytes and all(checked_sha(root / n) == v for n, v in pins.items()), "plot inputs changed during rendering")
    require(all(campaign.sha_file(ROOT / n) == v for n, v in sources.items()), "plot source changed during rendering")
    proof = {"schema": "portable_e2e.candidate_regret_plots.v1", "summary_sha256": campaign.base._sha(summary_bytes),
        "input_layout": "single_campaign_root_v1", "inputs": [{"path": n, "sha256": v} for n, v in sorted(pins.items())],
        "source_sha256": sources, "training_bins": measurements, "common_training_metrics": list(COMMON_METRICS),
        "candidate_only_auxiliary_metric": AUXILIARY, "raw_total_loss_compared": False,
        "training_data_approved": False, "model_loaded": False, "model_training": False, "test_evaluated": False,
        "note": "HH_260906 - All batches and pair failures retained; composite-cost regret is not selected-minus-oracle ADE. No intermediate validation."}
    with (output / "plot_inputs.json").open("x") as stream:
        stream.write(json.dumps(proof, indent=2, allow_nan=False) + "\n")
    with (output / "SHA256SUMS").open("x") as stream:
        for path in sorted(output.iterdir()):
            if path.name != "SHA256SUMS": stream.write(f"{campaign.sha_file(path)}  {path.name}\n")
    return proof


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("campaign_root", type=Path); parser.add_argument("--summary", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True); args = parser.parse_args(argv)
    render(args.campaign_root, args.summary, args.output_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
