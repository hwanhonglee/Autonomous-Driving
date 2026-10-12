#!/usr/bin/env python3
"""HH_260906 - Align sealed AN/AO native controls with unchanged 100 ms curvature witnesses."""
from __future__ import annotations

import argparse
from bisect import bisect_left
import gzip
import hashlib
import json
import math
from pathlib import Path


ROUTE_SHA256 = "3f94016e8e86da050f86fb90bf4deaff668ecf59e38357f53018036dbfdf005a"
WINDOWS = {"launch": (48, 62), "turn": (290, 300), "near_goal": (505, 558)}
REQUIRED_TICKS = tuple(sorted(set(range(53, 61)) | {295, 509, 516, 544, 545, 551, 552, 556}))
PLOT_TICKS = tuple(sorted(set(REQUIRED_TICKS) | {
    tick for begin, end in WINDOWS.values() for tick in range(begin, end + 1)}))


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def wrapped_delta(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def read_case(arm: str, trial: Path, future: Path) -> dict:
    """HH_260906 - Refuse an unowned run, stale route, or incomplete full-future audit."""
    owner = json.loads((trial / "owner_result.json").read_text())
    plan = json.loads((trial / "owner_plan.json").read_text())
    episode = trial / "episode"
    manifest = json.loads((episode / "manifest.json").read_text())
    if (owner.get("exit_code") != 0 or not owner.get("source_bytes_unchanged_and_archived")
            or len(owner.get("source_checks", {})) != (15 if arm == "AN" else 16)
            or not all(owner.get("source_checks", {}).values())
            or len(owner.get("external_source_checks", {})) != 3
            or not all(owner.get("external_source_checks", {}).values())
            or manifest.get("status") != "complete"
            or manifest.get("cleanup", {}).get("completed") is not True
            or plan.get("route_sha256") != ROUTE_SHA256
            or sha256(episode / "route.json") != ROUTE_SHA256):
        raise ValueError(f"{arm} requires sealed complete same-route evidence")
    states = [json.loads(line) for line in (episode / "states.jsonl").open()]
    stamps = [row["timestamp"] for row in states]
    if len(states) < 4 or any(a >= b for a, b in zip(stamps, stamps[1:])):
        raise ValueError(f"{arm} native timestamps are not strictly ordered")
    all_steps: dict[int, list[dict]] = {}
    with gzip.open(future, "rt") as stream:
        for line in stream:
            row = json.loads(line)
            if row.get("disposition") != "full_64_point_anchor":
                continue
            diagnostic = row.get("diagnostic")
            if diagnostic is None or len(diagnostic["steps"]) != 64:
                raise ValueError(f"{arm} has an incomplete full-future anchor")
            for step in diagnostic["steps"]:
                if step["metrics"]["xy_curvature"][0] is not None:
                    all_steps.setdefault(step["episode_relative_100ms_tick"], []).append(step)
    if not all_steps:
        raise ValueError(f"{arm} has no audited curvature steps")
    return {"arm": arm, "trial": trial, "future": future, "owner": owner,
            "states": states, "stamps": stamps, "steps": all_steps,
            "input_sha256": {
                "states.jsonl": sha256(episode / "states.jsonl"),
                "future.jsonl.gz": sha256(future),
                "owner_result.json": sha256(trial / "owner_result.json"),
                "route.json": sha256(episode / "route.json"),
            }}


def native_at(case: dict, target_ns: int) -> dict:
    """HH_260906 - Match the frozen 100 ms target to its exact 20 Hz physical frame."""
    states, stamps = case["states"], case["stamps"]
    target = target_ns / 1e9
    near = bisect_left(stamps, target)
    index = min((max(near - 1, 0), min(near, len(states) - 1)),
                key=lambda i: abs(stamps[i] - target))
    if index < 2 or abs(stamps[index] - target) > 1e-6:
        raise ValueError(f"{case['arm']} future/native timestamp mismatch at {target_ns}")
    row, before = states[index], states[index - 2]
    dt = row["timestamp"] - before["timestamp"]
    if abs(dt - 0.1) > 1e-4:
        raise ValueError(f"{case['arm']} native 100 ms bracket changed")
    dx, dy = row["x"] - before["x"], row["y"] - before["y"]
    distance = math.hypot(dx, dy)
    yaw_delta = wrapped_delta(row["yaw"] - before["yaw"])
    return {
        "frame": row["frame"], "previous_100ms_frame": before["frame"],
        "native_target_time_error_ns": round((row["timestamp"] - target) * 1e9),
        "speed_mps": math.hypot(row["vx"], row["vy"]),
        "route_progress_m": row["route_progress_m"],
        "route_remaining_m": row["goal_stop"]["remaining_route_arc_m"],
        "route_cte_m": row["route_cte_m"],
        "body_yaw_rad": row["yaw"], "body_yaw_delta_100ms_rad": yaw_delta,
        "body_yaw_change_per_m": abs(yaw_delta) / distance if distance > 0 else None,
        "steering_tire_angle_rad": row["steering_tire_angle_rad"],
        "current_control": {name: row["current_control"][name] for name in ("steer", "throttle", "brake")},
        "next_control": {name: row["next_control"][name] for name in ("steer", "throttle", "brake")},
        "delta_xy_100ms_m": [dx, dy], "distance_100ms_m": distance,
        "an_applied": bool(row["goal_stop"].get("an_normal_steering", {}).get("applied_to_normal_command")),
        "ao_applied": bool(row["goal_stop"].get("ao_launch_steering", {}).get("applied_to_normal_command")),
        "capture_phase": row["capture_phase"],
    }


def tick_summary(case: dict, tick: int) -> dict | None:
    """HH_260906 - Keep all overlapping windows and distinguish first-only from repeated failure."""
    entries = case["steps"].get(tick)
    if not entries:
        return None
    positive = [step for step in entries if step["metrics"]["xy_curvature"][1]]
    selected = max(positive or entries, key=lambda step: step["metrics"]["xy_curvature"][0])
    return {
        "tick": tick,
        "max_frozen_xy_curvature_per_m": max(step["metrics"]["xy_curvature"][0] for step in entries),
        "violating": bool(positive), "violating_window_count": len(positive),
        "first_future_point_only": len(positive) == 1 and positive[0]["index"] == 0 if positive else False,
        "frozen_target_timestamp_ns": selected["target_timestamp_ns"],
        "frozen_selected_point_index": selected["index"],
        "frozen_future_region": selected["future_region"],
        "native": native_at(case, selected["target_timestamp_ns"]),
    }


def native_context(case: dict, begin_tick: int, end_tick: int) -> list[dict]:
    """HH_260906 - Retain every native 20 Hz state in each prespecified mechanism window."""
    origin = case["stamps"][0]
    output = []
    for index, row in enumerate(case["states"]):
        tick = (row["timestamp"] - origin) / 0.1
        if begin_tick <= tick <= end_tick and index >= 2:
            before = case["states"][index - 2]
            output.append({"tick": tick, "frame": row["frame"],
                           "speed_mps": math.hypot(row["vx"], row["vy"]),
                           "current_steer": row["current_control"]["steer"],
                           "next_steer": row["next_control"]["steer"],
                           "tire_angle_rad": row["steering_tire_angle_rad"],
                           "body_yaw_rad": row["yaw"],
                           "body_yaw_delta_100ms_rad": wrapped_delta(row["yaw"] - before["yaw"]),
                           "progress_m": row["route_progress_m"], "cte_m": row["route_cte_m"],
                           "ao_applied": bool(row["goal_stop"].get("ao_launch_steering", {}).get("applied_to_normal_command"))})
    return output


def build_evidence(cases: list[dict]) -> dict:
    result = {"schema": "carla.drive_ao_mechanism_audit.v1",
              "notice": "Read-only control/geometry correlation; no causal proof, label change, or training admission.",
              "route_sha256": ROUTE_SHA256,
              "frozen_curvature_limit_per_m": 0.2,
              "method": {
                  "native_sampling": "all saved 20 Hz states in fixed windows",
                  "future_sampling": "all original 64-step/6.4 s windows; deduplicate by episode-relative 100 ms target tick",
                  "native_alignment_tolerance_ns": 1000,
                  "body_yaw_proxy": "wrapped native yaw delta over two 50 ms frames divided by native XY displacement; not decoder XY curvature",
                  "window_ticks": WINDOWS,
                  "cross_run_alignment": "same relative episode tick, not identical simulator frame or proof of matched world state",
              }, "cases": {}}
    for case in cases:
        arm = case["arm"]
        all_ticks = [tick_summary(case, tick) for tick in sorted(case["steps"])]
        witnessed = [entry for entry in all_ticks if entry is not None and entry["violating"]]
        result["cases"][arm] = {
            "trial": str(case["trial"].resolve()), "future": str(case["future"].resolve()),
            "input_sha256": case["input_sha256"],
            "owner_source_bytes_unchanged_and_archived": case["owner"]["source_bytes_unchanged_and_archived"],
            "witness_count": len(witnessed), "witnesses": witnessed,
            "selected_ticks": {str(tick): tick_summary(case, tick) for tick in PLOT_TICKS},
            "contexts_20hz": {name: native_context(case, *limits) for name, limits in WINDOWS.items()},
            "normal_steering_interventions": {
                "AN": sum(bool(row["goal_stop"].get("an_normal_steering", {}).get("applied_to_normal_command"))
                          for row in case["states"]),
                "AO": sum(bool(row["goal_stop"].get("ao_launch_steering", {}).get("applied_to_normal_command"))
                          for row in case["states"]),
            },
        }
    ao_rows = cases[1]["states"]
    applied = [i for i, row in enumerate(ao_rows) if row["goal_stop"].get("ao_launch_steering", {}).get("applied_to_normal_command")]
    if not applied or not ao_rows[applied[0]]["goal_stop"]["ao_launch_steering"]["includes_first_handoff_command"]:
        raise ValueError("AO launch handoff evidence is absent")
    last = applied[-1]
    result["ao_release"] = {
        "first_applied_frame": ao_rows[applied[0]]["frame"],
        "last_applied_frame": ao_rows[last]["frame"],
        "last_applied_measured_speed_mps": math.hypot(ao_rows[last]["vx"], ao_rows[last]["vy"]),
        "last_applied_next_steer": ao_rows[last]["next_control"]["steer"],
        "first_nonapplied_frame": ao_rows[last + 1]["frame"],
        "first_nonapplied_measured_speed_mps": math.hypot(ao_rows[last + 1]["vx"], ao_rows[last + 1]["vy"]),
        "first_nonapplied_next_steer": ao_rows[last + 1]["next_control"]["steer"],
    }
    return result


def render_figure(result: dict, output: Path) -> None:
    """HH_260906 - Show commands, wheels, yaw and original curvature on one relative clock."""
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib import pyplot as plt

    colors = {"AN": "#3178c6", "AO": "#e68624"}
    fig, axes = plt.subplots(4, 3, figsize=(18, 12), constrained_layout=True)
    for column, (region, bounds) in enumerate(WINDOWS.items()):
        yaw_axis = axes[2, column].twinx()
        for arm in ("AN", "AO"):
            case = result["cases"][arm]
            context = case["contexts_20hz"][region]
            x = [row["tick"] for row in context]
            color = colors[arm]
            axes[0, column].plot(x, [row["speed_mps"] for row in context], color=color, label=arm)
            axes[1, column].plot(x, [row["current_steer"] for row in context], color=color, label=arm)
            axes[2, column].plot(x, [row["tire_angle_rad"] for row in context], color=color, label=arm)
            yaw_axis.plot(x, [row["body_yaw_delta_100ms_rad"] * 1000 for row in context],
                          color=color, alpha=.45, linestyle=":")
            plotted = [(int(tick), entry) for tick, entry in case["selected_ticks"].items()
                       if entry is not None and bounds[0] <= int(tick) <= bounds[1]]
            plotted.sort()
            axes[3, column].plot([tick for tick, _ in plotted],
                                 [entry["max_frozen_xy_curvature_per_m"] for _, entry in plotted],
                                 color=color, alpha=.5, label=arm)
            for tick, entry in plotted:
                if entry["violating"]:
                    marker = "x" if entry["first_future_point_only"] else "o"
                    axes[3, column].scatter([tick], [entry["max_frozen_xy_curvature_per_m"]],
                                             color=color, marker=marker, s=45, zorder=5)
        yaw_axis.set_ylabel("body Δyaw/100 ms (mrad)", fontsize=8)
        axes[0, column].set_title({"launch": "Launch / zero-steer release", "turn": "Turn / tick 295",
                                   "near_goal": "Near goal / low displacement"}[region])
        for row in range(4):
            axes[row, column].set_xlim(*bounds)
            axes[row, column].grid(alpha=.23)
        axes[3, column].axhline(.2, color="#b52727", linestyle="--", linewidth=1)
        axes[3, column].set_yscale("log")
        axes[3, column].set_xlabel("episode-relative 100 ms tick")
    for row, label in enumerate(("speed (m/s)", "current steer (normalized)",
                                 "measured tire angle (rad)", "frozen XY curvature (1/m)")):
        axes[row, 0].set_ylabel(label)
    axes[0, 0].legend(loc="upper left")
    axes[3, 0].text(.02, .02, "circle: repeated windows; x: first-point only\nred: unchanged 0.2/m gate",
                    transform=axes[3, 0].transAxes, fontsize=8, va="bottom")
    fig.suptitle("Owned CARLA BasicAgent AN/AO: native 20 Hz state vs original 100 ms full-future audit\n"
                 "Relative ticks align time, not identical world state. Dotted yaw uses a separate right axis.")
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=150)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("an_trial", "ao_trial", "an_future", "ao_future", "output_json", "output_png"):
        parser.add_argument("--" + name.replace("_", "-"), type=Path, required=True)
    args = parser.parse_args()
    if args.output_json.exists() or args.output_png.exists():
        parser.error("diagnostic outputs already exist; refusing overwrite")
    cases = [read_case("AN", args.an_trial, args.an_future), read_case("AO", args.ao_trial, args.ao_future)]
    result = build_evidence(cases)
    result["script_sha256"] = sha256(Path(__file__))
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    with args.output_json.open("x") as stream:
        json.dump(result, stream, indent=2)
    render_figure(result, args.output_png)


if __name__ == "__main__":
    main()
