#!/usr/bin/env python3
"""HH_260906 - Publish fixed, authenticated Town07 numeric evidence without changing the original observations."""

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import platform
import re


HERE = Path(__file__).resolve().parent
ROOT = next(path for path in HERE.parents if (path / ".git").exists())
SOURCE = ROOT / "artifacts/training/2026-09-09/town07_goal_stop_conditioning_v2"
PINS = {
    "summary.json": "6835bb5bb22a786253b8395b8635ba713e77383339ee00903d2e4e0e5e08e399",
    "unique_ticks.jsonl": "2dc549f48efa4c0e674721ce5324e0c516808b9f6826b7d093611d1983090014",
    "SHA256SUMS": "d395fa34728ed9607bb94a937d9f36407b9d4569934eba55bf45054151d5f505",
}
ANALYZER_SHA = "32f8d6a6f0ee09b5ee044045a5c636d6e500334b7112c8bd9534193e1d65acc6"
PHASES = ("driving/launch_low", "driving/normal_pid", "driving/coast_low", "stationary_tail/setup_or_tail")
EXPECTED = {"run_001": (671, 145, 576, 60, (2, 2, 55, 1)),
            "run_002": (666, 154, 695, 59, (3, 3, 52, 1))}
OUTPUTS = ("summary.json", "unique_ticks.jsonl", "original_SHA256SUMS",
           "01_joined_motion.png", "02_heading_conditioning.png", "provenance.json", "SHA256SUMS")


def require(condition, message):
    if not condition:
        raise ValueError(message)


def sha(path):
    require(path.is_file() and not path.is_symlink(), "expected a regular nonsymlink file")
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def public_bytes(data):
    # HH_260906 - Generic rejection patterns intentionally contain no private account or endpoint literal.
    require(not re.search(rb"/(?:home|root|Users)/|\b(?:\d{1,3}\.){3}\d{1,3}\b|-----BEGIN [A-Z ]*PRIVATE KEY", data),
            "private path, endpoint, or key marker in publication input")


def phase(row):
    return row["native_target_phase"] + "/" + row["native_target_pilot_state"]


def inputs():
    for name, expected in PINS.items():
        require(sha(SOURCE / name) == expected, "original conditioning artifact SHA mismatch")
    source_bytes = {name: (SOURCE / name).read_bytes() for name in PINS}
    for data in source_bytes.values():
        public_bytes(data)
    summary = json.loads(source_bytes["summary.json"])
    rows = [json.loads(line) for line in source_bytes["unique_ticks.jsonl"].splitlines()]
    require(summary["schema"] == "carla_expert.town07_goal_stop_conditioning.v1"
            and summary["status"] == "DIAGNOSED_NOT_ADMITTED", "unreviewed diagnostic status")
    require(summary["scope"]["training_data_approved"] is False
            and summary["scope"]["source_data_or_labels_modified"] is False, "diagnostic scope changed")
    require(len(rows) == 119 and len({(r["trial"], r["tick"]) for r in rows}) == 119,
            "all 119 unique intervals must be retained")
    require([t["trial"] for t in summary["trials"]] == list(EXPECTED), "two-trial order changed")
    for trial in summary["trials"]:
        name = trial["trial"]
        count, failures, repeated, unique, phases = EXPECTED[name]
        subset = [row for row in rows if row["trial"] == name]
        require(trial["anchor_disposition_counts"] == {"full_64_point_anchor": count,
                "tail_label_context_only": 65, "violating_anchor_count": failures}, "full denominator changed")
        require(len(subset) == trial["unique_100ms_tick_count"] == unique
                and sum(r["repeated_violating_window_point_count"] for r in subset)
                == trial["violating_window_point_count"] == repeated, "failure multiplicity changed")
        require(Counter(phase(row) for row in subset) == dict(zip(PHASES, phases)), "phase denominator changed")
        require(all(r["recomputed_original_violation"] is True and r["xy_curvature_limit_rad_per_m"] == .2
                and abs(r["nearest_native_minus_target_ns"]) <= 74 for r in subset), "original failure or association changed")
    require(len(summary["input_sha256"]) == 12, "expected twelve historical inputs")
    for name, expected in summary["input_sha256"].items():
        relative = Path(name)
        require(not relative.is_absolute() and ".." not in relative.parts, "input reference is not repository relative")
        require(sha(ROOT / relative) == expected, "historical input bytes changed")
    require(summary["analyzer"]["sha256"] == ANALYZER_SHA
            and sha(ROOT / summary["analyzer"]["path"]) == ANALYZER_SHA, "analyzer source changed")
    return summary, rows, source_bytes


def phase_background(axis, subset):
    colors = ("#e7efff", "#ffeccc", "#e5f3eb", "#f5dfe8")
    start = 0
    for name, color in zip(PHASES, colors):
        count = sum(phase(row) == name for row in subset)
        axis.axvspan(start - .5, start + count - .5, color=color, zorder=0)
        start += count
    axis.set_xlim(-.7, len(subset) - .3)
    axis.grid(alpha=.2)
    axis.set_xlabel("Unique violating interval rank (grouped; NOT a continuous time axis)", fontsize=8)


def render(rows):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Patch

    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 9, "axes.titleweight": "bold"})
    motion, axes = plt.subplots(2, 3, figsize=(16, 8))
    heading, angles = plt.subplots(2, 1, figsize=(14, 9))
    plotted = {}
    for index, (trial, expected) in enumerate(EXPECTED.items()):
        subset = sorted((r for r in rows if r["trial"] == trial),
                        key=lambda r: (PHASES.index(phase(r)), r["native_to_frame"]))
        x = list(range(len(subset)))
        plotted[trial] = [{"rank": i, "tick": r["tick"], "native_to_frame": r["native_to_frame"],
                          "phase": phase(r)} for i, r in enumerate(subset)]
        for axis in (*axes[index], angles[index]):
            phase_background(axis, subset)
        suffix = f"{trial}: launch {expected[4][0]} / early PID {expected[4][1]} / coast {expected[4][2]} / tail 1"
        axes[index, 0].scatter(x, [r["raw_body_planar_endpoint_speeds_mps"][-1] for r in subset],
                               s=19, label="Native body endpoint speed", color="#215b96")
        axes[index, 0].scatter(x, [r["xy_interval_speed_mps"] for r in subset],
                               s=25, marker="x", label="100 ms XY interval-average speed", color="#d26721")
        axes[index, 0].set(ylabel="Speed (m/s)", title=suffix)
        axes[index, 0].legend(fontsize=8)
        axes[index, 1].scatter(x, [1000 * r["xy_displacement_m"] for r in subset], s=20, color="#215b96")
        axes[index, 1].set(ylabel="Reconstructed 100 ms XY displacement (mm)", title="All unique failure intervals")
        axes[index, 2].scatter(x, [r["api_controls_start_middle_end"][-1]["steer"] for r in subset],
                               s=20, label="API-reported endpoint steer", color="#794ea6")
        axes[index, 2].axhline(0, color="#555555", linewidth=.7)
        axes[index, 2].set(ylabel="Normalized steer (not tire force)", title="Control observation; not actuation proof")
        traces = (
            ("xy_heading_change_rad", "Observed XY direction change", "#bb3333", "o", False),
            ("allowed_heading_change_at_observed_distance_rad", "Original 0.2 rad/m × observed displacement", "#222222", "_", False),
            ("conditional_rounding_heading_change_bound_rad", "Conditional binary32 rounding bound (not measured error)", "#be8410", "x", False),
            ("body_yaw_change_over_interval_rad", "Absolute body yaw change (different observable)", "#246ca0", "+", True),
        )
        zeros = 0
        for key, label, color, marker, absolute in traces:
            values = [abs(r[key]) if absolute else r[key] for r in subset]
            if absolute:
                zeros = values.count(0.)
            # HH_260906 - A symlog axis retains exact zero; no positive floor is substituted for the observation.
            angles[index].scatter(x, values, label=label, color=color, marker=marker, s=25)
        angles[index].set_yscale("symlog", linthresh=1e-7)
        angles[index].set(ylabel="Absolute angular change / conditional bound (rad)",
                          title=suffix + f"; exact-zero body yaw changes retained: {zeros}")
        angles[index].legend(loc="upper left", ncol=2, fontsize=8)
    phase_handles = [Patch(facecolor=c, label=l) for c, l in zip(
        ("#e7efff", "#ffeccc", "#e5f3eb", "#f5dfe8"), ("Launch", "Early normal PID", "Low-speed coast", "First tail interval"))]
    for figure in (motion, heading):
        figure.legend(handles=phase_handles, loc="lower center", ncol=4, bbox_to_anchor=(.5, .04))
        figure.text(.5, .015, "ACTUAL NUMERIC DIAGNOSTIC — not camera footage, learned driving, or an admission decision.",
                    ha="center", fontsize=10, color="#873030")
    motion.suptitle("Town07: all 119 unique violating intervals from 1,337 full windows / 85,568 future points", fontsize=14)
    heading.suptitle("Observed direction changes vs unchanged allowance and a conditional rounding calculation", fontsize=14)
    motion.tight_layout(rect=(0, .09, 1, .95))
    heading.tight_layout(rect=(0, .09, 1, .95))
    motion.savefig(HERE / "01_joined_motion.png", dpi=150)
    heading.savefig(HERE / "02_heading_conditioning.png", dpi=150)
    plt.close(motion)
    plt.close(heading)
    return {"matplotlib_version": matplotlib.__version__, "python_version": platform.python_version(),
            "per_plot_unique_interval_count": 119, "plot_order": plotted,
            "zero_substitution": False, "axis_rank_is_continuous_time": False}


def main():
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--check-only", action="store_true")
    args = parser.parse_args()
    before = {name: sha(HERE / name) for name in ("render_conditioning.py", "README.md")}
    summary, rows, original = inputs()
    if args.check_only:
        print(json.dumps({"status": "AUTHENTICATED", "trials": 2, "unique_intervals": 119,
                          "full_windows": 1337, "future_points": 85568}))
        return
    require(all(not (HERE / name).exists() for name in OUTPUTS), "fresh publication required; no overwrites")
    rendered = render(rows)
    for name, output_name in (("summary.json", "summary.json"), ("unique_ticks.jsonl", "unique_ticks.jsonl"),
                              ("SHA256SUMS", "original_SHA256SUMS")):
        with (HERE / output_name).open("xb") as stream:
            stream.write(original[name])
    inputs()
    require(before == {name: sha(HERE / name) for name in before}, "publisher or README changed during rendering")
    provenance = {"schema": "carla_expert.town07_conditioning_publication.v1", "status": "PUBLISHED_NOT_ADMITTED",
        "source_artifacts": {str((SOURCE / name).relative_to(ROOT)): value for name, value in PINS.items()},
        "historical_input_sha256": summary["input_sha256"], "analyzer": summary["analyzer"],
        "publication_source_sha256": before, "rendering": rendered,
        "exact_byte_copies": ["summary.json", "unique_ticks.jsonl", "original_SHA256SUMS"],
        "pre_post_input_and_source_checks": True, "new_capture": False, "model_loaded": False,
        "training_data_approved": False, "original_data_or_labels_modified": False,
        "runtime_gates_modified": False, "historical_execution_commit_inferred": False,
        "limitations": summary["limitations"]}
    data = (json.dumps(provenance, indent=2, sort_keys=True, allow_nan=False) + "\n").encode()
    public_bytes(data)
    with (HERE / "provenance.json").open("xb") as stream:
        stream.write(data)
    names = sorted((*before, *OUTPUTS[:-1]))
    with (HERE / "SHA256SUMS").open("x") as stream:
        stream.write("".join(f"{sha(HERE / name)}  {name}\n" for name in names))
    print(json.dumps({"status": "PUBLISHED_NOT_ADMITTED", "files": len(names) + 1,
                      "unique_intervals": 119, "original_inputs_unchanged": True}))


if __name__ == "__main__":
    main()
