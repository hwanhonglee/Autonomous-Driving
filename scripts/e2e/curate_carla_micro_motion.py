#!/usr/bin/env python3
"""HH_260906 - Render measured micro-motion sidecars without replacing raw labels or granting admission."""

from __future__ import annotations

import argparse
from collections import defaultdict
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import json
import math
from pathlib import Path

from scripts.e2e import analyze_carla_micro_motion as analysis

ROOT = Path(__file__).resolve().parents[2]
SERIES = ("native_20hz", "derived_10hz_offset0", "derived_10hz_offset1")
REFERENCES = ("actor_api_reference", "recorded_rear_legacy_planar")
BINS = ("lt0p1", "0p1_to_0p5", "0p5_to_1", "ge1")
RULES = ("left", "right", "trapezoid")
require = analysis.require


def checked_inputs(root, expected_summary_sha, expected_checksums_sha):
    """HH_260906 - Bind complete sidecar outputs, including large streams and the exact executed source."""
    root = Path(root)
    names = {"summary.json", "execution_status.json", "executed_source.py", "native_intervals.jsonl", "future_directions.jsonl"}
    require(root.is_dir() and not root.is_symlink(), "sidecar input must be a real directory")
    require({p.name for p in root.iterdir()} == names | {"SHA256SUMS"}, "sidecar file inventory differs")
    require(not any(p.is_symlink() for p in root.iterdir()), "sidecar symlink rejected")
    require(analysis.sha(root / "SHA256SUMS") == expected_checksums_sha, "reviewed checksum-manifest SHA differs")
    entries = [line.split("  ") for line in (root / "SHA256SUMS").read_text().splitlines()]
    require(len(entries) == len(names) and all(len(e) == 2 for e in entries)
        and {e[1] for e in entries} == names, "sidecar checksum inventory differs")
    pins = {}
    for expected, name in entries:
        actual = analysis.sha(root / name)
        require(actual == expected, "sidecar artifact checksum differs")
        pins[name] = {"sha256": actual, "size_bytes": (root / name).stat().st_size}
    require(pins["summary.json"]["sha256"] == expected_summary_sha, "reviewed summary SHA differs")
    summary = analysis.load_json((root / "summary.json").read_bytes())
    status = analysis.load_json((root / "execution_status.json").read_bytes())
    require(summary["schema"] == analysis.SCHEMA and summary["status"] == "MEASURED_NOT_ADMITTED"
        and status["status"] == "COMPLETE", "completed unadmitted sidecar required")
    require(summary["source_sha256"] == pins["executed_source.py"]["sha256"]
        == analysis.sha(Path(analysis.__file__)), "reviewed executed analyzer source differs")
    require(status["source_sha256"] == summary["source_sha256"], "execution status source differs")
    require(summary["input_and_source_postcheck_pass"] is True and status["input_and_source_postcheck_pass"] is True,
        "sidecar input/source postcheck missing")
    require(summary["input_pins"] == status["input_pins"], "sidecar original input bindings differ")
    require({name: value["sha256"] for name, value in summary["input_pins"].items()} == analysis.PINS,
        "sidecar does not bind the frozen original input artifacts")
    require(all(summary["scope"][key] is False for key in
        ("training_data_approved", "dataset_admission", "labels_modified", "model_loaded", "training", "live_simulator_access", "thresholds_changed")),
        "sidecar scope or denial differs")
    pins["SHA256SUMS"] = {"sha256": analysis.sha(root / "SHA256SUMS"), "size_bytes": (root / "SHA256SUMS").stat().st_size}
    return summary, pins


def collect_plot_values(root, summary):
    """HH_260906 - Pool actual rows, never average percentiles, and keep all failures in direction denominators."""
    cases = analysis.expected_cases()
    residuals = defaultdict(list)
    trace = defaultdict(list)
    intervals = defaultdict(int)
    native_identities = set()
    for row in analysis.rows(root / "native_intervals.jsonl"):
        case, series = row["case_id"], row["series"]
        require(case in cases and series in SERIES, "unknown interval case/series")
        identity = (case, series, analysis.integer(row["first_frame"]), analysis.integer(row["last_frame"]))
        require(identity not in native_identities, "duplicate native interval identity")
        native_identities.add(identity)
        intervals[case + "/" + series] += 1
        for reference in REFERENCES:
            for rule in RULES:
                value = analysis.number(row["references"][reference]["residual_norm_m"][rule])
                require(value >= 0, "negative residual norm")
                residuals[case, series, reference, rule].append(value)
        if series == "native_20hz" and case in (cases[1], cases[6]):
            trace[case].append(row)
    require(dict(intervals) == summary["native_interval_counts"], "native plot denominator differs")
    differences = defaultdict(list)
    availability = defaultdict(int)
    failed_by_index = defaultdict(int)
    direction_count = 0
    failed_anchors = defaultdict(set)
    anchor_next_index = defaultdict(int)
    anchor_timestamps = {}
    direction_identities = set()
    for row in analysis.rows(root / "future_directions.jsonl"):
        case, index, bucket = row["case_id"], analysis.integer(row["index"]), row["xy_speed_bin_100ms"]
        require(case in cases and index < 64 and bucket in BINS and type(row["original_curvature_failed"]) is bool,
            "direction case/index/bin/verdict differs")
        anchor_frame = analysis.integer(row["anchor_frame"])
        anchor = (case, anchor_frame)
        identity = (*anchor, index)
        require(identity not in direction_identities, "duplicate future index identity")
        direction_identities.add(identity)
        require(index == anchor_next_index[anchor], "future anchor indices are not an exact ordered 0..63 sequence")
        anchor_next_index[anchor] += 1
        stamp = analysis.integer(row["anchor_timestamp_ns"])
        require(anchor not in anchor_timestamps or anchor_timestamps[anchor] == stamp, "future anchor timestamp changed")
        anchor_timestamps[anchor] = stamp
        require(analysis.integer(row["target_timestamp_ns"]) == stamp + (index + 1)*100_000_000,
            "future index timestamp grid differs")
        direction_count += 1
        if row["original_curvature_failed"]:
            failed_anchors[case].add(row["anchor_frame"])
            failed_by_index[str(index)] += 1
        for count in (1, 2, 5):
            availability[str(count) + "/" + row["windows"][str(count)]["status"]] += 1
        for count in (2, 5):
            value = row["comparisons"][f"direction_{count}00ms_vs_100ms_abs_rad"]
            if value is not None:
                value = analysis.number(value)
                require(0 <= value <= math.pi, "direction difference outside wrapped range")
                differences[str(count), bucket, "all_assessed"].append(value)
                if row["original_curvature_failed"]:
                    differences[str(count), bucket, "original_failed"].append(value)
    require(direction_count == summary["future_index_row_count"] == 7216 * 64,
        "all future indices required for publication")
    require(len(anchor_next_index) == 7216 and all(count == 64 for count in anchor_next_index.values()),
        "every future anchor must contain exactly all 64 indices")
    require(all(sum(key[0] == case for key in anchor_next_index) == count
        for case, count in zip(cases, analysis.FULL_COUNTS)), "per-case future anchor count differs")
    require({case: len(failed_anchors[case]) for case in cases}
        == summary["original_curvature_violating_anchors_by_case"], "original failed anchors differ")
    compact = {"native_residual_pooled": {"/".join(key): analysis.describe(values) for key, values in residuals.items()},
        "direction_difference_pooled_radians": {"/".join(key): analysis.describe(values) for key, values in differences.items()},
        "window_availability": dict(availability), "original_failed_point_windows_by_index": dict(failed_by_index),
        "native_interval_counts": dict(intervals), "future_index_rows": direction_count,
        "original_failed_anchors_by_case": {case: len(failed_anchors[case]) for case in cases},
        "scope": "All pooled values are measured overlapping rows, not independent physical events or new acceptance gates."}
    return compact, trace


def draw(compact, traces, output):
    """HH_260906 - Draw real numerical evidence using existing plotting packages; no synthetic driving imagery."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    cases = analysis.expected_cases()
    labels = [f"{i+1}: .{pedal:02d}" for i, pedal in enumerate(analysis.ORDER)]
    fig, axes = plt.subplots(2, 3, figsize=(16, 9), sharex=True)
    for r, reference in enumerate(REFERENCES):
        for c, series in enumerate(SERIES):
            ax = axes[r, c]
            for j, rule in enumerate(RULES):
                values = [compact["native_residual_pooled"]["/".join((case, series, reference, rule))]["p95"] * 1000 for case in cases]
                ax.bar(np.arange(8) + (j-1)*.24, values, .24, label=rule)
            ax.set_xticks(np.arange(8), labels, rotation=35)
            ax.set_ylabel("P95 XY displacement residual [mm]")
            ax.set_title(("API reference" if r == 0 else "Legacy rear reference") + " / " + series.replace("derived_", ""))
            ax.grid(axis="y", alpha=.25)
            if r == c == 0: ax.legend()
    fig.suptitle("All eight recordings: position delta vs native-velocity integration", fontsize=16)
    fig.text(.5, .012, "All phases and boundaries included. Residuals are descriptive; API reference is not proven COM. Not a model-driving test.", ha="center", fontsize=10)
    fig.tight_layout(rect=(0,.04,1,.96)); fig.savefig(output / "01_position_velocity_residuals.png", dpi=130); plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(16, 6))
    pretty_bins = ("<0.1", "0.1-0.5", "0.5-1", ">=1")
    for ax, count in zip(axes, (2,5)):
        for j, subset in enumerate(("all_assessed", "original_failed")):
            for b, bucket in enumerate(BINS):
                stat = compact["direction_difference_pooled_radians"].get("/".join((str(count), bucket, subset)), {"count":0,"p95":None})
                x = b + (j-.5)*.34
                if stat["count"]:
                    value = math.degrees(stat["p95"])
                    ax.bar(x, value, .32, color=("tab:blue", "tab:orange")[j], label=subset if b == 0 else None)
                    ax.annotate(f"n={stat['count']}", (x,value), xytext=(0,4), textcoords="offset points", ha="center", fontsize=8, rotation=90)
                else:
                    ax.text(x, 2, "unavailable\nn=0", ha="center", va="bottom", fontsize=8, rotation=90)
        ax.set_title(f"Same endpoint: {count}00 ms vs 100 ms direction")
        ax.set_xticks(range(4), pretty_bins); ax.set_xlabel("100 ms XY interval-average speed [m/s]")
        ax.set_ylabel("P95 wrapped absolute direction difference [deg]"); ax.set_ylim(0,215)
        ax.grid(axis="y",alpha=.25); ax.legend()
    fig.suptitle("All 64 indices: interval sensitivity, not a replacement curvature gate", fontsize=16)
    fig.text(.5,.012,"Short-prefix and tiny-displacement comparisons remain unavailable. Counts overlap in time; original FAIL flags are retained.",ha="center",fontsize=10)
    fig.tight_layout(rect=(0,.05,1,.94)); fig.savefig(output / "02_direction_interval_sensitivity.png",dpi=130); plt.close(fig)

    fig, axes = plt.subplots(2,2,figsize=(16,8))
    for row_index, case in enumerate((cases[1],cases[6])):
        records=traces[case]; first=records[0]["first_timestamp_ns"]
        times=[(r["target_timestamp_ns"]-first)/1e9 for r in records]
        stop=next((t for t,r in zip(times,records) if r["native_phases"][-1]=="stationary_tail"),None)
        require(stop is not None,"trace requires recorded tail boundary")
        for c,(lo,hi,title) in enumerate(((0,8,"first 8 s, warmup retained"),(stop-8,times[-1],"last 8 s of driving and full tail"))):
            ax=axes[row_index,c]
            for reference in REFERENCES:
                values=[r["references"][reference]["residual_norm_m"]["trapezoid"]*1000 for r in records]
                ax.plot(times,values,label=reference,linewidth=1)
            ax.axvline(stop,color="black",linestyle="--",label="tail begins")
            ax.set_xlim(lo,hi); ax.set_yscale("symlog",linthresh=.001)
            ax.set_title(f"{case} / {title}"); ax.set_xlabel("Seconds from first native observation")
            ax.set_ylabel("20 Hz trapezoid XY residual [mm]"); ax.grid(alpha=.25)
            if row_index == c == 0: ax.legend(fontsize=8)
    fig.suptitle("Both .13 repetitions: recorded launch/stop residuals, still unadmitted",fontsize=16)
    fig.text(.5,.012,"Fixed profile pair, not a selected best interval; complete all-case streams and summary are retained separately.",ha="center",fontsize=10)
    fig.tight_layout(rect=(0,.05,1,.95)); fig.savefig(output / "03_both_013_repetitions_trace.png",dpi=130); plt.close(fig)


def publish(root, output, expected_summary_sha, expected_checksums_sha):
    root, output = Path(root), Path(output)
    require(not output.exists() and not output.is_symlink() and not output.resolve().is_relative_to(root.resolve())
        and not output.resolve().is_relative_to((ROOT / "datasets").resolve()), "fresh publication outside data required")
    summary,pins=checked_inputs(root,expected_summary_sha,expected_checksums_sha)
    sources={str(Path(module.__file__).resolve().relative_to(ROOT)):analysis.sha(Path(module.__file__)) for module in (analysis,)}
    sources[Path(__file__).resolve().relative_to(ROOT).as_posix()]=analysis.sha(Path(__file__))
    compact,traces=collect_plot_values(root,summary)
    output.mkdir(parents=True,exist_ok=False)
    (output/"summary.json").write_bytes((root/"summary.json").read_bytes())
    analysis.write_json(output/"plot_metrics.json",compact)
    draw(compact,traces,output)
    require(checked_inputs(root,expected_summary_sha,expected_checksums_sha)[1]==pins,"sidecar inputs changed during rendering")
    require(all(analysis.sha(ROOT/name)==value for name,value in sources.items()),"plotting source changed during rendering")
    provenance={"note":"HH_260906 - Preserve measured sidecar provenance; publication does not run inference or approve data.",
        "created_utc":datetime.now(timezone.utc).isoformat(),"input_artifacts":pins,"source_sha256":sources,
        "plot_versions":{name:importlib.metadata.version(name) for name in ("matplotlib","numpy","pillow")},
        "training_data_approved":False,"model_inference":False,"new_capture":False}
    analysis.write_json(output/"provenance.json",provenance)
    (output/"SHA256SUMS").write_text("".join(f"{analysis.sha(p)}  {p.name}\n" for p in sorted(output.iterdir()) if p.is_file()))
    return provenance


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument("--input-root",type=Path,required=True)
    parser.add_argument("--output-dir",type=Path,required=True)
    parser.add_argument("--expected-summary-sha256",required=True)
    parser.add_argument("--expected-checksums-sha256",required=True)
    args=parser.parse_args(argv)
    publish(args.input_root,args.output_dir,args.expected_summary_sha256,args.expected_checksums_sha256)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
