#!/usr/bin/env python3
"""HH_261001 - Render measured native pilot traces without inventing driving footage."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
RUN = ROOT / "storage/experiments/validation/2026-10-01/town07_low_speed_10_30_native_pilot_v1"
OUTPUT = ROOT / "docs/assets/validation/2026-10-01/town07_low_speed_10_30_native_pilot_v1/results_v1"


def measured_summary(rows):
    """HH_261001 - Preserve all rows and include entry and closing intervals in window rates."""
    if not rows or len(rows) > 1300:
        raise ValueError("bounded native pilot rows required")
    speeds = []
    for row in rows:
        speed = math.hypot(*row["world_velocity_carla"][:2])
        if not math.isfinite(speed) or abs(speed - row["speed_mps"]) > 1e-12:
            raise ValueError("native planar speed reconstruction differs")
        speeds.append(speed)
    rates = []
    window_rates = []
    for index, (previous, current) in enumerate(zip(rows, rows[1:]), start=1):
        dt = current["timestamp"] - previous["timestamp"]
        if current["frame"] != previous["frame"] + 1 or abs(dt - .05) > 1e-6:
            raise ValueError("native frame or timestamp cadence differs")
        rate = (speeds[index] - speeds[index - 1]) / dt
        rates.append(rate)
        if 10 <= current["route_progress_m"] <= 30 or 10 <= previous["route_progress_m"] <= 30:
            window_rates.append(rate)
    selected = [row for row in rows if 10 <= row["route_progress_m"] <= 30]
    if not selected or not window_rates or rows[-1]["route_progress_m"] <= 30:
        raise ValueError("entire fixed window and its closing sample required")
    return dict(native_state_count=len(rows), selected_state_count=len(selected),
        selected_span_seconds=selected[-1]["timestamp"] - selected[0]["timestamp"],
        selected_progress_first_last_m=[selected[0]["route_progress_m"], selected[-1]["route_progress_m"]],
        selected_speed_min_mps=min(row["speed_mps"] for row in selected),
        selected_speed_max_mps=max(row["speed_mps"] for row in selected),
        selected_max_abs_scalar_rate_mps2=max(map(abs, window_rates)),
        whole_run_max_abs_scalar_rate_mps2=max(map(abs, rates)),
        selected_max_abs_route_cte_m=max(abs(row["route_cte_m"]) for row in selected),
        observed_collision_count=rows[-1]["collision_count"],
        observed_lane_invasion_count=rows[-1]["lane_invasion_count"],
        training_data_approved=False, model_inferences=0,
        whole_body_lane_clearance_proven=False, camera_count=0,
        plot_is_actual_camera_footage=False)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    args = parser.parse_args(argv)
    if args.output.exists() or args.output.is_symlink():
        raise ValueError("fresh create-only rendering output required")
    if args.output.resolve() == RUN.resolve() or RUN.resolve() in args.output.resolve().parents:
        raise ValueError("rendering must not write into retained native run")
    outcome = json.loads((RUN / "pilot_result.json").read_text())
    if (outcome["status"] != "PASS_FEASIBILITY_ONLY"
            or outcome["independent_raw_replay"]["status"] != "PASS"
            or outcome["source_hashes_match"] is not True
            or outcome["original_owner"]["cleanup_errors"]
            or outcome["original_owner"]["ports_released"] is not True):
        raise ValueError("completed audited native pilot receipt required")
    state_path = RUN / "episode.partial/states.jsonl"
    if state_path.is_symlink() or state_path.stat().st_size > 8 * 1024**2:
        raise ValueError("bounded regular state input required")
    payload = state_path.read_bytes()
    digest = hashlib.sha256(payload).hexdigest()
    if digest != outcome["independent_raw_replay"]["state_sha256"]:
        raise ValueError("native state hash differs from audited receipt")
    rows = [json.loads(line) for line in payload.splitlines()]
    summary = measured_summary(rows)
    if summary["selected_state_count"] != outcome["assessment"]["selected_count"]:
        raise ValueError("fixed window recount differs")
    summary.update(source_state_sha256=digest,
        renderer_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    relative = [row["timestamp"] - rows[0]["timestamp"] for row in rows]
    selected = [(time, row) for time, row in zip(relative, rows) if 10 <= row["route_progress_m"] <= 30]
    window_start, window_end = selected[0][0], selected[-1][0]
    rates = [(b["speed_mps"] - a["speed_mps"]) / (b["timestamp"] - a["timestamp"])
             for a, b in zip(rows, rows[1:])]
    figure, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    axes[0].plot(relative, [row["speed_mps"] for row in rows], label="measured world planar speed")
    axes[0].axhline(4 / 3.6, color="black", linestyle=":", label="expert target 4 km/h")
    axes[0].set_ylabel("speed (m/s)")
    axes[0].legend(loc="lower right", fontsize=9)
    axes[1].plot(relative[1:], rates)
    axes[1].axhline(2.9, color="gray", linestyle=":")
    axes[1].axhline(-2.9, color="gray", linestyle=":")
    axes[1].set_ylabel("native delta speed / dt (m/s2)")
    axes[2].plot(relative, [row["route_progress_m"] for row in rows])
    axes[2].set(ylabel="measured route progress (m)", xlabel="simulation seconds from first recorded native state")
    for axis in axes:
        axis.axvspan(window_start, window_end, color="green", alpha=.12)
        axis.grid(alpha=.25)
    axes[0].set_title("Town07 BasicAgent native pilot: green = entire 10-30 m window; no model or cameras")
    figure.tight_layout()
    args.output.mkdir(parents=True, exist_ok=False)
    figure.savefig(args.output / "native_pilot_speed_rate_progress.png", dpi=150)
    plt.close(figure)
    with (args.output / "measured_summary.json").open("x") as stream:
        json.dump(summary, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
