#!/usr/bin/env python3
"""HH_260906 - Explain fixed raw XY-curvature failures without changing labels, gates or admission."""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import struct

from scripts.e2e import audit_carla_raw_pre_admission as raw

K = raw.targets.PHYSICAL_MAXIMUM_CURVATURE_RAD_PER_M
T = raw.targets.HEADING_MINIMUM_STEP_M


def wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def float32_spacing(value):
    # HH_260906 - Coordinate-scale ULP is sensitivity information, not an asserted simulator error bound.
    bits = struct.unpack("<I", struct.pack("<f", abs(value)))[0]
    low = struct.unpack("<f", struct.pack("<I", bits))[0]
    high = struct.unpack("<f", struct.pack("<I", bits + 1))[0]
    return high - low


def geometry_steps(ego, futures, wheelbase=2.85):
    """HH_260906 - Compare the exact rear-point audit with explicit diagnostic alternatives, never replacement labels."""
    previous = ego
    heading = 0.0
    proxy_heading = 0.0
    skipped = 0.0
    previous_assessed_distance = None
    records = []
    for i, future in enumerate(futures):
        dx, dy = future["x"] - previous["x"], future["y"] - previous["y"]
        distance = math.hypot(dx, dy)
        angle = wrap(math.atan2(dy, dx) - ego["yaw"])
        delta = abs(wrap(angle - heading)) if distance > T else None
        curvature = delta / distance if delta is not None else None
        yaw_delta = abs(wrap(future["yaw"] - previous["yaw"]))
        lateral_yaw_shift = wheelbase / 2 * (math.cos(future["yaw"]) - math.cos(previous["yaw"]))
        proxy_dx = dx + lateral_yaw_shift
        proxy_dy = dy + wheelbase / 2 * (math.sin(future["yaw"]) - math.sin(previous["yaw"]))
        proxy_distance = math.hypot(proxy_dx, proxy_dy)
        proxy_angle = wrap(math.atan2(proxy_dy, proxy_dx) - ego["yaw"])
        proxy_curvature = abs(wrap(proxy_angle - proxy_heading)) / proxy_distance if proxy_distance > T else None
        epsilon = 4 * math.hypot(float32_spacing(future["x"]), float32_spacing(future["y"]))
        angular_sensitivity = math.asin(min(1., 2 * epsilon / distance)) if distance > T else None
        if angular_sensitivity is not None and previous_assessed_distance is not None:
            angular_sensitivity += math.asin(min(1., 2 * epsilon / previous_assessed_distance))
        fail = curvature is not None and curvature > K + raw.targets.ROUNDING_TOLERANCE
        records.append({"point_index": i, "distance_m": distance, "xy_speed_mps": distance / .1,
            "heading_change_rad": delta, "allowed_heading_change_rad": K * distance,
            "curvature_rad_per_m": curvature, "curvature_failed": fail,
            "preceding_unassessed_distance_m": skipped,
            "skip_threshold_budget_can_explain_failure": bool(fail and delta <= K * (distance + skipped)
                + raw.targets.ROUNDING_TOLERANCE * distance),
            "body_yaw_change_rad": yaw_delta,
            "body_yaw_change_over_xy_distance": yaw_delta / distance if distance > T else None,
            "four_float32_ulp_position_sensitivity_m": epsilon,
            "four_ulp_sensitivity_can_cover_excess_angle": bool(fail and delta <= K * distance + angular_sensitivity),
            "zero_pitch_actor_center_proxy_curvature": proxy_curvature,
            "zero_pitch_actor_center_proxy_failed": proxy_curvature is not None and proxy_curvature > K + raw.targets.ROUNDING_TOLERANCE,
            "pose_step_world_xy_m": [dx, dy], "planar_yaw_reference_offset_delta_xy_m": [lateral_yaw_shift, proxy_dy - dy],
            "instantaneous_speed_label_mps": math.hypot(future["vx"], future["vy"]),
            "end_speed_vs_interval_xy_speed_error_mps": math.hypot(future["vx"], future["vy"]) - distance / .1})
        if distance > T:
            heading, skipped, previous_assessed_distance = angle, 0.0, distance
        else:
            skipped += distance
        if proxy_distance > T:
            proxy_heading = proxy_angle
        previous = future
    return records


def ending_trace(states):
    # HH_260906 - The complete last three driving seconds and tail are displayed, not a selected good interval.
    last_driving = next(row for row in reversed(states) if row["capture_phase"] == "driving")
    selected = [row for row in states if row["timestamp"] >= last_driving["timestamp"] - 3.0]
    reference = selected[0]
    yaw = reference["yaw"]
    out, previous = [], None
    for row in selected:
        dx, dy = row["x"] - reference["x"], row["y"] - reference["y"]
        cx = dx + 1.425 * (math.cos(row["yaw"]) - math.cos(reference["yaw"]))
        cy = dy + 1.425 * (math.sin(row["yaw"]) - math.sin(reference["yaw"]))
        out.append({"frame": row["frame"], "time_from_goal_stop_s": row["timestamp"] - last_driving["timestamp"],
            "capture_phase": row["capture_phase"], "rear_lateral_displacement_mm": (-math.sin(yaw) * dx + math.cos(yaw) * dy) * 1000,
            "zero_pitch_center_proxy_lateral_displacement_mm": (-math.sin(yaw) * cx + math.cos(yaw) * cy) * 1000,
            "yaw_change_from_reference_deg": math.degrees(wrap(row["yaw"] - reference["yaw"])),
            "reported_yaw_rate_radps": row["yaw_rate"],
            "pose_finite_difference_yaw_rate_radps": None if previous is None else wrap(row["yaw"] - previous["yaw"]) / (row["timestamp"] - previous["timestamp"]),
            "vx_mps": row["vx"], "vy_mps": row["vy"], "measured_steering_rad": row["steering_tire_angle_rad"],
            "reported_steer": row["current_control"]["steer"], "requested_steer": row["next_control"]["steer"]})
        previous = row
    return out


def analyze(diagnostic_root, raw_parent, output, expected_summary_sha):
    diagnostic_root, raw_parent, output = map(Path, (diagnostic_root, raw_parent, output))
    raw.require(not output.exists() and not output.is_symlink() and not any(output.resolve().is_relative_to(p.resolve()) for p in (diagnostic_root, raw_parent)), "fresh output required outside inputs")
    summary_bytes = raw.control_audit.checked_bytes(diagnostic_root, "summary.json", {})
    raw.require(raw.digest(summary_bytes) == expected_summary_sha, "raw diagnostic summary SHA mismatch")
    summary = raw.scalar._loads(summary_bytes.decode())
    raw.require(summary["schema"] == raw.SCHEMA and summary["trial_count"] == 2, "original two-trial diagnostic required")
    inputs = {"summary.json": expected_summary_sha}
    for line in (diagnostic_root / "SHA256SUMS").read_text().splitlines():
        expected, name = line.split("  ")
        raw.require(raw.scalar.sha(diagnostic_root / name) == expected, "raw diagnostic file SHA mismatch")
        inputs[name] = expected
    source_pins = {str(Path(__file__).relative_to(raw.ROOT)): raw.scalar.sha(Path(__file__)), **raw.source_identity()["files"]}
    witnesses, results, traces = [], [], {}
    data = {}
    for trial in summary["trials"]:
        name = trial["trial_id"]
        raw.require(name in ("run_001", "run_002"), "unexpected original trial name")
        root = raw_parent / name
        for item in trial["source_manifest"]:
            raw.require(raw.scalar.sha(root / item["path"]) == item["sha256"], "original metadata changed since full pixel/future diagnostic")
        states = raw.read_metadata(root, trial["original_episode_directory"] + "/states.jsonl", {}, lines=True)
        timeline = raw.measured_timeline(states)
        data[name] = (trial, timeline, [t for t, _ in timeline])
        traces[name] = ending_trace(states)
    reproduced = {name: Counter() for name in data}
    for line in (diagnostic_root / "future_anchor_audit.jsonl").open():
        record = raw.scalar._loads(line)
        name = record["trial"]
        if "diagnostic" not in record:
            continue
        trial, timeline, stamps = data[name]
        _, ego = raw.adapter._causal_state(timeline, record["anchor_timestamp_ns"])
        steps = record["diagnostic"]["steps"]
        future = [raw.future_state(timeline, stamps, step["target_timestamp_ns"])[0] for step in steps]
        raw.require(all(item is not None for item in future), "old diagnostic future cannot be reproduced")
        comparisons = geometry_steps(ego, future)
        for source, measured in zip(steps, comparisons):
            value, failed = source["metrics"]["xy_curvature"]
            raw.require((value is None and measured["curvature_rad_per_m"] is None) or value is not None
                and math.isclose(value, measured["curvature_rad_per_m"], rel_tol=1e-6, abs_tol=1e-6), "curvature formula reproduction differs")
            raw.require(failed == measured["curvature_failed"], "curvature verdict cannot be silently changed")
            reproduced[name]["all_valid_steps"] += 1
            reproduced[name]["assessed_curvature_steps"] += value is not None
            if failed:
                witnesses.append({"trial": name, "anchor_frame": record["frame"], "anchor_phase": record["capture_phase"],
                    "target_timestamp_ns": source["target_timestamp_ns"], "target_tick": source["episode_relative_100ms_tick"],
                    "future_region": source["future_region"], "existing_lateral_acceleration": source["metrics"]["xy_lateral_acceleration"][0],
                    "raw_anchor_velocity_slip_rad": math.atan2(ego["vy"], ego["vx"]), **measured})
    for name, (trial, timeline, _) in data.items():
        rows = [r for r in witnesses if r["trial"] == name]
        old = trial["measured_future"]["full_64_point_anchors"]["metrics"]["xy_curvature"]
        raw.require(len(rows) == old["violating_step_count"] and len({r["anchor_frame"] for r in rows}) == old["violating_anchor_count"], "original failure denominator changed")
        results.append({"trial": name, "full_64_anchor_count": trial["measured_future"]["full_64_point_anchors"]["sample_count"],
            "full_64_anchor_counts_by_phase": trial["measured_future"]["full_anchor_counts_by_phase"],
            **reproduced[name], "curvature_violating_steps": len(rows), "curvature_violating_anchors": old["violating_anchor_count"],
            "unique_100ms_violation_ticks": len({r["target_tick"] for r in rows}),
            "skip_threshold_budget_could_explain_count": sum(r["skip_threshold_budget_can_explain_failure"] for r in rows),
            "four_ulp_sensitivity_could_cover_count": sum(r["four_ulp_sensitivity_can_cover_excess_angle"] for r in rows),
            "zero_pitch_center_proxy_still_failed_count": sum(r["zero_pitch_actor_center_proxy_failed"] for r in rows),
            "body_yaw_proxy_within_curvature_bound_count": sum(r["body_yaw_change_over_xy_distance"] <= K for r in rows),
            "xy_speed_range_mps": [min(r["xy_speed_mps"] for r in rows), max(r["xy_speed_mps"] for r in rows)],
            "worst_curvature_witness": max(rows, key=lambda r: r["curvature_rad_per_m"]),
            "all_original_failures_retained": True, "actor_center_pitch_roll_were_recorded": False})
    for name, expected in inputs.items():
        raw.require(raw.scalar.sha(diagnostic_root / name) == expected, "diagnostic input changed during analysis")
    for name, (trial, _, _) in data.items():
        for item in trial["source_manifest"]:
            raw.require(raw.scalar.sha(raw_parent / name / item["path"]) == item["sha256"], "raw source changed during analysis")
    raw.require(all(raw.scalar.sha(raw.ROOT / path) == pin for path, pin in source_pins.items()), "analysis implementation changed")
    report = {"schema": "carla_expert.low_speed_xy_explanation.v1", "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "status": "DIAGNOSED_NOT_ADMITTED", "source_pins": source_pins, "prior_diagnostic_pins": inputs, "trials": results,
        "formulas": {"curvature": "abs(wrap(displacement_heading_i - previous_assessed_heading)) / distance_i",
            "decoder_heading_bound": "abs(delta_heading_i) <= min(0.2, 2.8/max(entry_speed,speed_i)^2) * speed_i * 0.1",
            "lateral_crossover_speed_mps": math.sqrt(2.8 / K),
            "skip_threshold": "Existing audit ignores heading update when distance <=0.0001m; hypothetical cumulative skipped-distance budget is checked without changing any verdict.",
            "center_proxy": "world rear XY + 1.425*[cos(yaw),sin(yaw)] assumes pitch=0 and is NOT observed actor-center truth or replacement labels.",
            "four_ulp_sensitivity": "Assume endpoint XY perturbation radius4*hypot(float32_spacing(x),float32_spacing(y)); angular sensitivity asin(2eps/d_i)+asin(2eps/d_previous). Not a measured noise bound or correction."},
        "scope": {"labels_changed": False, "gates_changed": False, "masks_changed": False, "model_loaded": False,
            "training_data_approved": False, "test_payload_read": False, "camera_pixels_redecoded": False},
        "limitations": ["The old raw capture lacks actor-center transform/pitch/roll and immutable-frame kinematic brackets; exact source attribution is impossible retrospectively.",
            "Low lateral acceleration and small body-yaw changes do not override the separate XY curvature constraint.",
            "Velocity endpoint labels and integrated XY interval speeds are distinct even for continuous physically smooth motion."]}
    output.mkdir(parents=True, exist_ok=False)
    for name, value in (("summary.json", report), ("curvature_witnesses.json", witnesses), ("ending_native_traces.json", traces)):
        (output / name).write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")
    render(report, witnesses, traces, output)
    (output / "SHA256SUMS").write_text("".join(f"{raw.scalar.sha(p)}  {p.name}\n" for p in sorted(output.iterdir())))
    return report


def render(report, witnesses, traces, output):
    # HH_260906 - Plot only actual witnessed violations and complete declared terminal traces.
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(12.8, 6.4))
    for name, color in (("run_001", "#be3b3b"), ("run_002", "#2678ab")):
        selected = [r for r in witnesses if r["trial"] == name]
        axes[0].scatter([r["xy_speed_mps"] for r in selected], [r["curvature_rad_per_m"] for r in selected], s=13, alpha=.25, c=color, label=name)
        axes[1].scatter([r["distance_m"] * 1000 for r in selected], [math.degrees(r["heading_change_rad"]) for r in selected], s=13, alpha=.25, c=color, label=name)
    axes[0].axhline(K, c="black", linestyle="--", label="Unchanged curvature cap 0.2/m")
    axes[0].set(xlabel="Observed XY segment speed (m/s)", ylabel="Existing XY curvature (1/m)", yscale="log")
    distances = sorted(r["distance_m"] for r in witnesses)
    axes[1].plot([d * 1000 for d in distances], [math.degrees(K * d) for d in distances], c="black", linestyle="--", label="Decoder heading budget")
    axes[1].set(xlabel="Observed 100 ms segment length (mm)", ylabel="Wrapped heading change (degree)", yscale="log")
    for ax in axes:
        ax.legend(fontsize=8); ax.grid(alpha=.2)
    fig.suptitle("Original raw low-speed XY failures | all overlapping violations retained", fontsize=13)
    fig.text(.5, .02, "Position-derived heading is not vehicle yaw. No lateral-acceleration violation does not cancel the separate curvature limit.", ha="center", fontsize=8)
    fig.tight_layout(rect=(0, .05, 1, .94)); fig.savefig(output / "01_low_speed_curvature_and_heading_budget.png", dpi=150); plt.close(fig)
    fig, axes = plt.subplots(3, 2, figsize=(12.8, 9.6), sharex="col")
    for col, (name, rows) in enumerate(traces.items()):
        times = [r["time_from_goal_stop_s"] for r in rows]
        axes[0, col].plot(times, [r["rear_lateral_displacement_mm"] for r in rows], label="Recorded rear-point XY")
        axes[0, col].plot(times, [r["zero_pitch_center_proxy_lateral_displacement_mm"] for r in rows], label="Zero-pitch center PROXY", alpha=.7)
        axes[0, col].set_title(name + " | terminal native 20 Hz trace")
        axes[1, col].plot(times, [r["yaw_change_from_reference_deg"] for r in rows], label="Recorded body yaw change")
        axes[2, col].plot(times, [r["reported_yaw_rate_radps"] for r in rows], label="Reported angular velocity")
        axes[2, col].plot(times, [r["pose_finite_difference_yaw_rate_radps"] for r in rows], label="Pose yaw finite difference", alpha=.7)
        for ax in axes[:, col]:
            ax.axvline(0, c="gray", linestyle="--"); ax.grid(alpha=.2); ax.legend(fontsize=7)
        axes[0, col].set_ylabel("Lateral displacement (mm)"); axes[1, col].set_ylabel("Yaw change (degree)")
        axes[2, col].set_ylabel("Yaw rate (rad/s)"); axes[2, col].set_xlabel("Seconds relative to final driving observation")
    fig.suptitle("Measured terminal reference-point / yaw consistency | no resampling or label correction", fontsize=12)
    fig.text(.5, .015, "Entire final 3 driving seconds + tail. Old capture lacks true actor-center pose/pitch/roll; center curve is only a zero-pitch sensitivity proxy.", ha="center", fontsize=8)
    fig.tight_layout(rect=(0, .035, 1, .95)); fig.savefig(output / "02_terminal_rear_point_yaw_consistency.png", dpi=150); plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("diagnostic_root", type=Path)
    parser.add_argument("raw_parent", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--expected-summary-sha256", required=True)
    args = parser.parse_args(argv)
    analyze(args.diagnostic_root, args.raw_parent, args.output_dir, args.expected_summary_sha256)


if __name__ == "__main__":
    main()
