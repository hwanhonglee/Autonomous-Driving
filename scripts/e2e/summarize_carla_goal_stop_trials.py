#!/usr/bin/env python3
"""HH_260906 - Independently summarize every finalized expert trial without importing CARLA or torch."""

from __future__ import annotations

import argparse
import ast
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[2]
PHASES = ("stationary_warmup", "driving", "stationary_tail")
CAMERAS = ("CAM_FRONT", "CAM_BACK", "CAM_FRONT_LEFT", "CAM_BACK_LEFT", "CAM_FRONT_RIGHT", "CAM_BACK_RIGHT")
TOLERANCE = 1.0e-4


class EvidenceError(ValueError):
    """HH_260906 - Reject incomplete or contradictory evidence rather than silently omit a trial."""


def require(value, message):
    if not value:
        raise EvidenceError(message)


def number(value):
    require(not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value),
            "measurement must be a finite number")
    return float(value)


def integer(value):
    require(isinstance(value, int) and not isinstance(value, bool) and value >= 0, "invalid nonnegative integer")
    return value


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _pairs(items):
    value = {}
    for key, item in items:
        require(key not in value, "duplicate JSON key")
        value[key] = item
    return value


def _loads(value):
    return json.loads(value, object_pairs_hook=_pairs,
        parse_constant=lambda _: (_ for _ in ()).throw(EvidenceError("nonfinite JSON constant")))


def read_file(root, relative, ledger, *, jsonl=False):
    path = root / relative
    require(path.is_file() and not path.is_symlink() and path.resolve().is_relative_to(root.resolve()),
            f"required finalized evidence is missing or unsafe: {relative}")
    require(path.stat().st_size <= 256 * 1024 * 1024, "evidence file exceeds bounded reader size")
    contents = path.read_bytes()
    ledger.append({"path": relative, "sha256": hashlib.sha256(contents).hexdigest(), "size_bytes": len(contents)})
    text = contents.decode("utf-8")
    return [_loads(line) for line in text.splitlines() if line.strip()] if jsonl else _loads(text)


def source_bounds():
    """HH_260906 - Read literal scalar limits from source AST; never import model weights or torch."""
    model = ROOT / "portable_e2e/model.py"
    runtime = ROOT / "portable_e2e/runtime_contract.py"
    model_tree = ast.parse(model.read_text())
    decoder = next(ast.literal_eval(node.value) for node in model_tree.body if isinstance(node, ast.Assign)
        and any(isinstance(target, ast.Name) and target.id == "PHYSICAL_MAXIMUM_ACCELERATION_MPS2" for target in node.targets))
    gate = next(node for node in ast.parse(runtime.read_text()).body if isinstance(node, ast.ClassDef) and node.name == "RuntimeGateConfig")
    values = {node.target.id: ast.literal_eval(node.value) for node in gate.body if isinstance(node, ast.AnnAssign)
              and isinstance(node.target, ast.Name) and node.target.id in ("maximum_acceleration_mps2", "maximum_deceleration_mps2")}
    require(decoder == 2.9 and values == {"maximum_acceleration_mps2": 3.0, "maximum_deceleration_mps2": 6.0},
            "reviewed scalar limits changed; this summary cannot silently relax them")
    return {"physical_decoder": {"maximum_acceleration_mps2": decoder, "maximum_deceleration_mps2": decoder},
            "runtime_speed_rate_gate": values,
            "source_sha256": {"portable_e2e/model.py": sha(model), "portable_e2e/runtime_contract.py": sha(runtime)}}


def project_route(points, x, y, progress, step):
    """HH_260906 - Recompute monotonic local route projection without using collector helper results."""
    end, candidates = min(points[-1][2], progress + step), []
    for first, second in zip(points, points[1:]):
        if second[2] <= first[2] or second[2] < progress or first[2] > end:
            continue
        dx, dy = second[0] - first[0], second[1] - first[1]
        distance2 = dx * dx + dy * dy
        if distance2 <= 1.0e-12:
            continue
        ratio = ((x - first[0]) * dx + (y - first[1]) * dy) / distance2
        ratio = max(max(0.0, (progress - first[2]) / (second[2] - first[2])),
                    min(min(1.0, (end - first[2]) / (second[2] - first[2])), ratio))
        candidates.append((math.hypot(x - first[0] - ratio * dx, y - first[1] - ratio * dy),
                           first[2] + ratio * (second[2] - first[2])))
    require(candidates, "no valid local route projection")
    error, arc = min(candidates)
    return arc, error


def control(value):
    require(isinstance(value, dict), "missing applied or next control")
    result = {name: number(value[name]) for name in ("throttle", "brake", "steer")}
    require(0 <= result["throttle"] <= 1.000001 and 0 <= result["brake"] <= 1.000001
            and abs(result["steer"]) <= 1.000001, "control outside normalized domain")
    return result


def intervals(rows, expected_dt, bounds):
    """HH_260906 - Retain phase-boundary prefixes and applied/next controls at every violating interval."""
    values = []
    for previous, current in zip(rows, rows[1:]):
        dt = current["timestamp"] - previous["timestamp"]
        require(dt > 0, "timestamps must increase")
        rate = (current["speed_mps"] - previous["speed_mps"]) / dt
        values.append({"from_frame": previous["frame"], "to_frame": current["frame"], "dt_s": dt,
            "from_phase": previous["phase"], "to_phase": current["phase"],
            "time_from_capture_start_s": current["elapsed_s"], "from_speed_mps": previous["speed_mps"],
            "to_speed_mps": current["speed_mps"], "speed_rate_mps2": rate,
            "from_applied_control": previous["current_control"], "from_next_control": previous["next_control"],
            "to_applied_control": current["current_control"], "to_next_control": current["next_control"],
            "to_goal_remaining_m": current["remaining_route_m"], "to_goal_error_m": current["goal_error_m"],
            "to_command_target_speed_mps": current["target_speed_mps"], "to_control_source": current["control_source"]})
    result = {"interval_count": len(values), "cadence_violation_count": sum(abs(row["dt_s"] - expected_dt) > TOLERANCE for row in values),
              "by_phase": {}}
    for phase in ("all", *PHASES):
        selected = values if phase == "all" else [row for row in values if row["to_phase"] == phase]
        rates = [row["speed_rate_mps2"] for row in selected]
        row = {"interval_count": len(selected), "minimum_mps2": min(rates) if rates else None,
               "maximum_mps2": max(rates) if rates else None, "phase_boundary_intervals": [row for row in selected if row["from_phase"] != row["to_phase"]]}
        for name in ("physical_decoder", "runtime_speed_rate_gate"):
            limit = bounds[name]
            bad = [row for row in selected if row["speed_rate_mps2"] > limit["maximum_acceleration_mps2"] + 1.0e-9
                   or row["speed_rate_mps2"] < -limit["maximum_deceleration_mps2"] - 1.0e-9]
            row[name] = {"violation_count": len(bad), "violation_intervals": bad,
                         "assessed_and_clear": bool(selected) and not bad}
        result["by_phase"][phase] = row
    return result, values


def analyze_native(states, cameras, route, config, bounds):
    """HH_260906 - Recompute scalar QA and measured endpoint dwell independently of stored completion flags."""
    require(states, "finalized collection contains no state data")
    points = [(number(point["x"]), number(point["y"]), number(point["distance_m"])) for point in route["route"]]
    require(len(points) >= 2 and points[0][2] == 0 and points[-1][2] > 0
            and all(b[2] >= a[2] and (b[2] > a[2] or math.dist(a[:2], b[:2]) <= 1.0e-9)
                    for a, b in zip(points, points[1:])), "invalid route arc")
    final = points[-1]
    tangent = (final[0] - points[-2][0], final[1] - points[-2][1])
    tangent_length = math.hypot(*tangent)
    require(tangent_length > 0, "missing terminal route tangent")
    limits = {key: number(config[key]) for key in ("goal_tolerance_m", "stopped_speed_mps", "hold_seconds",
        "minimum_tail_seconds", "maximum_projection_step_m", "maximum_projection_error_m")}
    require(limits["goal_tolerance_m"] == 1.0 and limits["stopped_speed_mps"] == 0.1 and limits["hold_seconds"] == 2.0
            and limits["minimum_tail_seconds"] >= 6.5 and limits["maximum_projection_step_m"] == 1.0
            and limits["maximum_projection_error_m"] == 3.0, "goal quality thresholds changed")
    rows, progress, previous_phase, previous_frame, previous_time = [], 0.0, 0, None, None
    first_time = number(states[0]["timestamp"])
    for state in states:
        frame, timestamp = integer(state["frame"]), number(state["timestamp"])
        phase = state["capture_phase"]
        require(phase in PHASES and PHASES.index(phase) >= previous_phase, "invalid capture phase order")
        require(previous_frame is None or frame > previous_frame, "state frames must increase")
        require(previous_time is None or timestamp > previous_time, "state timestamps must increase")
        previous_frame, previous_time, previous_phase = frame, timestamp, PHASES.index(phase)
        x, y = number(state["x"]), number(state["y"])
        progress, cte = project_route(points, x, y, progress, limits["maximum_projection_step_m"])
        remaining = final[2] - progress
        goal_error = math.hypot(x - final[0], y - final[1])
        overshoot = ((x - final[0]) * tangent[0] + (y - final[1]) * tangent[1]) / tangent_length
        speed = math.hypot(number(state["vx"]), number(state["vy"]))
        goal_stop = state.get("goal_stop", {})
        in_goal = 1.0e-6 < remaining <= limits["goal_tolerance_m"] and goal_error <= limits["goal_tolerance_m"] and overshoot <= 0
        event_counts = {}
        for event in ("collision", "lane_invasion"):
            require(isinstance(state.get(event), list), "missing native event ledger")
            event_counts[event] = len(state[event])
        rows.append({"frame": frame, "timestamp": timestamp, "elapsed_s": timestamp - first_time, "phase": phase,
            "speed_mps": speed, "target_speed_mps": number(goal_stop["target_speed_mps"]),
            "remaining_route_m": remaining, "goal_error_m": goal_error, "terminal_overshoot_m": overshoot,
            "recomputed_route_cte_m": cte, "recorded_route_projection_error_m": abs(progress - number(state["route_progress_m"])),
            "stopped_in_goal": in_goal and speed <= limits["stopped_speed_mps"],
            "current_control": control(state["current_control"]), "next_control": control(state["next_control"]),
            "control_source": goal_stop.get("control_source"), **event_counts})
    expected_frames = [row["frame"] for row in rows[::2]]
    camera_frames = [integer(row["frame"]) for row in cameras]
    by_frame = {row["frame"]: row for row in rows}
    require(len(set(camera_frames)) == len(camera_frames) and all(frame in by_frame for frame in camera_frames), "duplicate or unowned camera frame")
    camera_rows = []
    camera_metadata_ok = True
    for record in cameras:
        row = by_frame[record["frame"]]
        camera_metadata_ok &= abs(number(record["timestamp"]) - row["timestamp"]) <= TOLERANCE and record["capture_phase"] == row["phase"]
        camera_metadata_ok &= tuple(record.get("camera_order", [])) == CAMERAS and set(record.get("images", {})) == set(CAMERAS)
        timestamps = record.get("source_timestamps", {})
        camera_metadata_ok &= set(timestamps) == set(CAMERAS)
        camera_metadata_ok &= all(abs(number(value) - row["timestamp"]) <= 0.020 for value in timestamps.values())
        camera_rows.append(row)
    native, native_intervals = intervals(rows, 0.05, bounds)
    camera, camera_intervals = intervals(camera_rows, 0.1, bounds)
    driving = [row for row in rows if row["phase"] == "driving"]
    tail = [row for row in rows if row["phase"] == "stationary_tail"]
    dwell_start = None
    for row in driving:
        dwell_start = row["timestamp"] if row["stopped_in_goal"] and dwell_start is None else dwell_start if row["stopped_in_goal"] else None
    dwell_seconds = driving[-1]["timestamp"] - dwell_start if driving and dwell_start is not None else 0.0
    goal_reached = bool(driving and driving[-1]["stopped_in_goal"] and dwell_seconds >= limits["hold_seconds"] - TOLERANCE)
    tail_duration = tail[-1]["timestamp"] - driving[-1]["timestamp"] if tail and driving else 0.0
    flags = {"native_frame_sequence_complete": all(b["frame"] == a["frame"] + 1 for a, b in zip(rows, rows[1:])),
        "native_20hz_cadence": native["cadence_violation_count"] == 0,
        "camera_exact_10hz_full_capture_coverage": camera_frames == expected_frames and camera_metadata_ok,
        "camera_10hz_cadence": camera["cadence_violation_count"] == 0,
        "native_decoder_speed_rate_clear": native["by_phase"]["all"]["physical_decoder"]["assessed_and_clear"],
        "camera_decoder_speed_rate_clear": camera["by_phase"]["all"]["physical_decoder"]["assessed_and_clear"],
        "measured_stop_and_two_second_dwell": goal_reached,
        "full_stopped_goal_tail": bool(tail) and len(tail) >= math.ceil(limits["minimum_tail_seconds"] * 20)
            and tail_duration >= limits["minimum_tail_seconds"] - TOLERANCE and all(row["stopped_in_goal"] for row in tail),
        "route_projection_matches_recording": max(row["recorded_route_projection_error_m"] for row in rows) <= TOLERANCE,
        "route_cross_track_within_limit": all(row["recomputed_route_cte_m"] <= limits["maximum_projection_error_m"] for row in rows),
        "maximum_measured_speed_within_30kph": max(row["speed_mps"] for row in rows) <= 30 / 3.6 + 1.0e-6,
        "no_recorded_collision_or_lane_invasion": all(row["collision"] == row["lane_invasion"] == 0 for row in rows)}
    result = {"flags": flags, "failed_flags": [name for name, passed in flags.items() if not passed],
        "raw_scalar_quality_clear": all(flags.values()), "goal_reached_independently": goal_reached,
        "goal_dwell_seconds": dwell_seconds, "tail_elapsed_from_driving_end_seconds": tail_duration,
        "final_driving": driving[-1] if driving else None, "maximum_measured_speed_kmh": max(row["speed_mps"] for row in rows) * 3.6,
        "maximum_recorded_projection_discrepancy_m": max(row["recorded_route_projection_error_m"] for row in rows),
        "phase_counts": {phase: {"native_states": sum(row["phase"] == phase for row in rows),
            "camera_anchors": sum(row["phase"] == phase for row in camera_rows)} for phase in PHASES},
        "event_counts": {name: sum(row[name] for row in rows) for name in ("collision", "lane_invasion")},
        "control_ranges": {origin: {name: {"minimum": min(row[origin][name] for row in rows),
            "maximum": max(row[origin][name] for row in rows)} for name in ("throttle", "brake", "steer")}
            for origin in ("current_control", "next_control")},
        "speed_rate_qa": {"native_20hz": native, "camera_10hz": camera}}
    return result, {"native_states": rows, "native_intervals": native_intervals, "camera_intervals": camera_intervals}


def summarize_trial(root, bounds):
    root = Path(root)
    require(root.is_dir() and not root.is_symlink(), "unsafe trial directory")
    ledger = []
    owner = read_file(root, "owner_result.json", ledger)
    require(owner.get("learned_model_control") is False and owner.get("vehicle_control_approved") is False, "owner scope is not expert-only")
    exit_code = integer(owner["exit_code"])
    stopped = read_file(root, "lifecycle/stopped.json", ledger)
    require(stopped.get("status") == "PASS" and stopped.get("mode") == "stopped" and stopped.get("stage") == "stopped"
            and stopped.get("read_only") is True and stopped.get("port_released") is True
            and stopped.get("owner_process_state") is None, "owned simulator has no valid stopped proof")
    pid = integer(stopped["owner_pid"])
    require(pid > 1 and stopped.get("owner_pgid") == pid and stopped.get("generation_id") == f"expert_{pid}"
            and stopped.get("host") == "127.0.0.1", "stopped proof ownership mismatch")
    plan = read_file(root, "owner_plan.json", ledger) if (root / "owner_plan.json").exists() else None
    started = read_file(root, "owner_started.json", ledger) if (root / "owner_started.json").exists() else None
    require(started is not None or plan is not None, "no owned startup identity")
    identity = started or plan
    require(identity["port"] == stopped["port"] and identity["map"] == stopped["expected_map"], "stopped proof endpoint mismatch")
    if started:
        require(started.get("server_pid") == pid and started.get("server_pgid") == pid, "started and stopped owners differ")
    existing = [name for name in ("episode", "episode.partial") if (root / name).exists()]
    require(len(existing) <= 1, "both successful and partial datasets exist")
    result = {"trial_id": root.name, "owner_exit_code": exit_code, "owner_stopped_proof": True,
        "owner_plan_recorded": plan is not None, "map": identity["map"], "quality": identity.get("quality"),
        "status": "FAILED_NO_CAPTURE", "raw_data_available": False, "raw_quality_candidate": False,
        "source_manifest": ledger, "learned_model_control": False, "vehicle_control_approved": False}
    timeline = None
    if existing:
        directory = existing[0]
        manifest = read_file(root, f"{directory}/manifest.json", ledger)
        require(manifest.get("status") in ("complete", "failed"), "capture manifest is not finalized")
        route = read_file(root, f"{directory}/route.json", ledger)
        states = read_file(root, f"{directory}/states.jsonl", ledger, jsonl=True)
        cameras = read_file(root, f"{directory}/camera_frames.jsonl", ledger, jsonl=True)
        route_sha = next(item["sha256"] for item in ledger if item["path"] == f"{directory}/route.json")
        provenance = manifest["provenance"]
        require(route_sha == provenance["route_sha256"] == identity["route_sha256"], "route evidence hash mismatch")
        require(route["town"] == identity["map"], "route and simulator map differ")
        if plan:
            require(plan["route_sha256"] == route_sha and plan["map"] == identity["map"] and plan["port"] == identity["port"], "planned input identity mismatch")
            for source, key in (("scripts/e2e/collect_carla_vad_expert.py", "collector_sha256"),
                                ("scripts/e2e/carla_goal_stop_profile.py", "goal_stop_helper_sha256")):
                require(plan["source_sha256"][source] == provenance[key], "planned versus collector source hash mismatch")
        capture = manifest["capture_contract"]
        config = capture["goal_stop_profile"]
        require(capture["physics_hz"] == 20 and capture["camera_hz"] == 10 and capture["camera_interval_ticks"] == 2,
                "unsupported recorded capture cadence")
        require(capture.get("client_map_loading_allowed") is False, "capture allowed unowned map loading")
        require(all(config["bounds"][name] == bounds[name] for name in bounds), "recorded physical bound provenance mismatch")
        if states:
            native, timeline = analyze_native(states, cameras, route, config, bounds)
        else:
            # HH_260906 - A finalized startup failure remains in the denominator even with no captured states.
            require(exit_code != 0 and manifest["status"] == "failed" and not cameras,
                    "empty capture is inconsistent with declared success or camera evidence")
            native = {"flags": {"nonempty_native_states": False}, "goal_reached_independently": False,
                "goal_dwell_seconds": 0.0, "event_counts": {"collision": 0, "lane_invasion": 0},
                "phase_counts": {phase: {"native_states": 0, "camera_anchors": 0} for phase in PHASES},
                "maximum_measured_speed_kmh": None, "final_driving": None, "speed_rate_qa": {}}
        declared = manifest.get("result", {})
        native["flags"]["manifest_state_and_camera_counts_match"] = declared.get("state_count") == len(states) and declared.get("camera_anchor_count") == len(cameras)
        native["flags"]["manifest_events_match"] = (declared.get("collision_event_count") == native["event_counts"]["collision"]
            and declared.get("lane_invasion_event_count") == native["event_counts"]["lane_invasion"])
        native["flags"]["capture_cleanup_completed"] = manifest.get("cleanup", {}).get("completed") is True
        native["flags"]["collector_goal_claim_matches_measured_goal"] = declared.get("goal_reached", False) == native["goal_reached_independently"]
        native["failed_flags"] = [name for name, passed in native["flags"].items() if not passed]
        native["raw_scalar_quality_clear"] = all(native["flags"].values())
        candidate = bool(exit_code == 0 and manifest["status"] == "complete" and directory == "episode"
                         and native["raw_scalar_quality_clear"])
        result.update({"status": "RAW_QA_CANDIDATE_ONLY" if candidate else "FAILED_RAW_QA_OR_CAPTURE",
            "raw_data_available": True, "raw_quality_candidate": candidate, "capture_status": manifest["status"],
            "profile_id": config["profile_id"], "goal_stop_profile": {key: value for key, value in config.items()
                if key not in ("bounds", "terminal_plan", "effective_control")},
            "nominal_target_speed_kmh": number(capture["target_speed_kmh"]),
            "collector_reported_goal_reached": declared.get("goal_reached"),
            "collector_reported_quality_status": declared.get("goal_stop_quality", {}).get("status"),
            "termination_reason": declared.get("termination_reason"),
            "collector_source_sha256": provenance["collector_sha256"], "goal_stop_helper_sha256": provenance["goal_stop_helper_sha256"],
            "independent_qa": native})
    else:
        require(exit_code != 0, "owner claims success without capture evidence")
    for item in ledger:
        require(sha(root / item["path"]) == item["sha256"], "finalized input changed during analysis")
    return result, timeline


def render_trial(trial, timeline, output):
    # HH_260906 - Plot recorded measurements, not camera screenshots or a learned-policy demonstration.
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    rows = timeline["native_states"]
    figure, axes = plt.subplots(3, 1, figsize=(12, 9))
    times = [row["elapsed_s"] for row in rows]
    axes[0].plot(times, [row["speed_mps"] * 3.6 for row in rows], label="Measured speed", color="#21618c")
    axes[0].plot(times, [row["target_speed_mps"] * 3.6 for row in rows], label="Command target (not nominal cruise)", color="#d17b25")
    axes[0].set_ylabel("Speed (km/h)")
    for cadence, color, label in (("native_intervals", "#21618c", "Actual 20 Hz"), ("camera_intervals", "#c0392b", "Camera-aligned 10 Hz")):
        values = timeline[cadence]
        axes[1].plot([row["time_from_capture_start_s"] for row in values], [row["speed_rate_mps2"] for row in values], label=label, color=color, alpha=0.85)
    axes[1].axhline(2.9, linestyle="--", color="black", label="Unchanged decoder +/-2.9")
    axes[1].axhline(-2.9, linestyle="--", color="black")
    axes[1].set_ylabel("Actual speed rate (m/s²)")
    for key, label in (("remaining_route_m", "Recomputed remaining route arc"), ("goal_error_m", "Measured planar goal error")):
        axes[2].plot(times, [row[key] for row in rows], label=label)
    axes[2].axhline(1.0, linestyle="--", color="black", label="Goal tolerance 1 m")
    zoom_start = max(times[0], times[-1] - 15)
    axes[2].set_xlim(zoom_start, times[-1] + 0.2)
    visible_goal_values = [row[key] for row in rows if row["elapsed_s"] >= zoom_start
                          for key in ("remaining_route_m", "goal_error_m")]
    axes[2].set_ylim(-0.15, max(1.25, max(visible_goal_values) * 1.08))
    axes[2].set_ylabel("Goal distance (m)")
    axes[2].set_xlabel("Seconds from first recorded state; goal panel shows last 15 s")
    for axis in axes:
        for previous, current in zip(rows, rows[1:]):
            if previous["phase"] != current["phase"]:
                axis.axvline(current["elapsed_s"], color="gray", linestyle=":", alpha=0.65)
        axis.legend(loc="best", fontsize=9)
        axis.grid(alpha=0.18)
    qa = trial["independent_qa"]
    figure.suptitle(f"{trial['trial_id']} | expert collection | goal={'YES' if qa['goal_reached_independently'] else 'NO'} | "
        f"raw scalar QA={'CLEAR' if qa['raw_scalar_quality_clear'] else 'FAIL'}\n"
        f"Nominal {trial['nominal_target_speed_kmh']:g} km/h; measured maximum {qa['maximum_measured_speed_kmh']:.3f} km/h", fontweight="bold")
    figure.text(0.5, 0.013, "Actual recorded traces; no camera footage, learned model control, test-set use, full decoder or 30 km/h autonomy PASS claim.", ha="center", fontsize=9)
    figure.tight_layout(rect=(0, 0.04, 1, 0.93))
    figure.savefig(output, dpi=150)
    plt.close(figure)


def summarize_trials(trials_root, output):
    trials_root, output = Path(trials_root), Path(output)
    require(trials_root.is_dir() and not trials_root.is_symlink(), "trial collection root is unsafe")
    require(not output.exists() and not output.is_symlink(), "summary output already exists")
    require(not output.resolve().is_relative_to(trials_root.resolve()), "summary must not modify the original trial tree")
    roots = sorted(path for path in trials_root.iterdir() if re.fullmatch(r"run_[0-9]+", path.name))
    require(roots, "no trial directories discovered")
    require(all((root / "owner_result.json").is_file() for root in roots), "INCOMPLETE: every discovered trial requires owner_result; active trials cannot be skipped")
    bounds = source_bounds()
    trials, timelines = [], {}
    for root in roots:
        result, timeline = summarize_trial(root, bounds)
        trials.append(result)
        if timeline is not None:
            timelines[root.name] = timeline
    # HH_260906 - Recheck every included trial after the whole batch has been read, not only per-trial.
    for root, trial in zip(roots, trials):
        for item in trial["source_manifest"]:
            require(sha(root / item["path"]) == item["sha256"], "trial source changed during batch analysis")
    report = {"schema": "portable_e2e.goal_stop_trial_summary.v1", "status": "FINALIZED_TRIALS_REVIEWED_NOT_PROMOTED",
        "created_at_utc": datetime.now(timezone.utc).isoformat(), "summarizer_source_sha256": sha(Path(__file__)),
        "total_discovered_and_included_trials": len(trials), "failed_trial_count": sum(not trial["raw_quality_candidate"] for trial in trials),
        "raw_quality_candidate_count": sum(trial["raw_quality_candidate"] for trial in trials), "trials": trials,
        "scalar_bounds": bounds, "scope": {"all_discovered_trials_in_denominator": True, "original_inputs_modified": False,
            "camera_jsonl_metadata_read": True, "camera_pixels_opened": False, "model_loaded": False,
            "learned_model_control": False, "live_simulator_access": False, "test_dataset_used": False, "automatic_promotion": False},
        "limitations": ["Raw scalar QA is not full XY/decoder feasibility or learned driving validation.",
            "Nominal speed settings are separate from maximum measured speed; no 30 km/h pass follows from a nominal setting.",
            "Phase boundaries retain preceding states; low-speed spikes are not removed.",
            "Stored source bytes are hashed without assuming historical runs had the latest wrapper provenance.",
            "Camera metadata coverage is audited; JPEG bytes, camera geometry and image content are not validated here."]}
    output.mkdir(parents=True, exist_ok=False)
    (output / "summary.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    for trial in trials:
        if trial["trial_id"] in timelines:
            render_trial(trial, timelines[trial["trial_id"]], output / f"{trial['trial_id']}_measured_timeline.png")
    (output / "SHA256SUMS").write_text("".join(f"{sha(path)}  {path.name}\n" for path in sorted(output.iterdir())))
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trials_root", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        report = summarize_trials(args.trials_root, args.output_dir)
    except (EvidenceError, OSError, ValueError, KeyError, TypeError, ImportError, StopIteration) as error:
        print(f"GOAL_STOP_SUMMARY_ERROR: {error}", file=sys.stderr)
        return 2
    print(json.dumps({key: report[key] for key in ("status", "total_discovered_and_included_trials", "failed_trial_count", "raw_quality_candidate_count")}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
