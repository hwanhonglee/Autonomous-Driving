#!/usr/bin/env python3
"""HH_260906 - Audit every frozen Town07 v3 development attempt independently of worker PASS labels."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys

if __package__:
    from . import summarize_carla_goal_stop_trials as base
else:
    import summarize_carla_goal_stop_trials as base

ROUTE_SHA = "8285e70a790d5e8ae75803db1aa538122b56e6fcfc3586e60eaa947d330417c9"
REVIEWED_SOURCES = {
    "scripts/e2e/carla_goal_stop_profile.py": "2ef5d93665d7500e2f33a650e8abd9f26d2f44f903fd569780aa49db58d68dcf",
    "scripts/e2e/collect_carla_vad_expert.py": "9f78a394b4c005a0d9b458ad697c7446cda4e576d6a3a5bce770b177d0d30e8f",
}
SOURCE_NAMES = {
    "scripts/e2e/run_owned_carla_expert_trial.sh", "scripts/e2e/run_carla_map.sh",
    "scripts/e2e/process_group_cleanup.sh", "scripts/e2e/workspace_runtime_lock.sh",
    "scripts/e2e/probe_carla_server.py", "scripts/e2e/env.sh", *REVIEWED_SOURCES,
    "portable_e2e/model.py", "portable_e2e/runtime_contract.py",
}
FROZEN = {
    "profile_id": "comfortable_v3", "desired_deceleration_mps2": 0.6,
    "target_acceleration_limit_mps2": 1.0, "normal_brake_cap": 0.1, "normal_throttle_cap": 0.4,
    "stop_buffer_m": 0.75, "goal_tolerance_m": 1.0, "stopped_speed_mps": 0.1,
    "hold_seconds": 2.0, "minimum_tail_seconds": 6.5, "maximum_projection_step_m": 1.0,
    "maximum_projection_error_m": 3.0, "nominal_cruise_speed_mps": 8.0,
    "maximum_actual_speed_mps": 30 / 3.6, "launch_throttle": 0.15,
    "launch_handoff_speed_mps": 0.5, "maximum_launch_seconds": 8.0,
    "post_handoff_throttle_ramp_per_second": 0.05,
    "approach_reference_speed_mps": 3.0396464855480536, "approach_reference_distance_m": 35.0,
    "coast_reference_seconds": 6.93, "minimum_coast_entry_speed_mps": 2.8,
    "maximum_coast_entry_speed_mps": 3.2, "maximum_coast_seconds": 45.0,
    "cruise_minimum_speed_mps": 7.8, "cruise_maximum_speed_mps": 8.2,
    "minimum_cruise_seconds": 5.0, "route_sha256": ROUTE_SHA,
    "route_length_m": 210.5975914062836, "maximum_attempts_per_revision": 2,
    "development_only": True, "training_data_approved": False,
}
EPS = 1.0e-6
REPOSITORY = Path(__file__).resolve().parents[2]


def require(value, message):
    base.require(value, message)


def checked_bytes(root, relative, ledger):
    # HH_260906 - Pin exact bytes and reject path aliases before interpreting any evidence.
    path = root / relative
    require(not Path(relative).is_absolute() and ".." not in Path(relative).parts, "unsafe relative source path")
    require(path.is_file() and not any(p.is_symlink() for p in [path, *path.parents] if p != root.parent),
            f"missing or symlinked evidence: {relative}")
    require(path.resolve().is_relative_to(root.resolve()) and path.stat().st_size <= 256 * 1024 * 1024,
            "unsafe or oversized evidence")
    contents = path.read_bytes()
    ledger[relative] = {"sha256": hashlib.sha256(contents).hexdigest(), "size_bytes": len(contents)}
    return contents


def read_json(root, relative, ledger, lines=False):
    text = checked_bytes(root, relative, ledger).decode("utf-8")
    return [base._loads(line) for line in text.splitlines() if line.strip()] if lines else base._loads(text)


def same(actual, expected):
    if expected is None or isinstance(expected, (str, bool)):
        return actual == expected and (not isinstance(expected, bool) or isinstance(actual, bool))
    return (not isinstance(actual, bool) and isinstance(actual, (int, float))
            and math.isfinite(actual) and abs(actual - expected) <= EPS)


def recorded_commit_bytes(commit, relative):
    # HH_260906 - Unrelated dirty analysis files do not invalidate exact archived execution bytes.
    require(re.fullmatch(r"[0-9a-f]{40}", commit) and relative in SOURCE_NAMES, "unsafe recorded source identity")
    environment = dict(os.environ, GIT_NO_LAZY_FETCH="1", GIT_ALLOW_PROTOCOL="", GIT_TERMINAL_PROMPT="0")
    result = subprocess.run(["git", "-c", "protocol.allow=never", "show", f"{commit}:{relative}"], cwd=REPOSITORY,
                            env=environment, capture_output=True, timeout=10, check=False)
    require(result.returncode == 0, "recorded execution commit is unavailable locally; no fetch attempted")
    return result.stdout


def analyze_protocol(states, rows):
    """HH_260906 - Reconstruct clocks, envelopes and normal-control bounds without importing the governor."""
    require(len(states) == len(rows), "independent state alignment mismatch")
    flags = {key: True for key in ("engagement_boundary", "governor_state_and_timers", "target_envelope_and_slew",
        "normal_pedal_protocol", "requested_to_applied_control_link", "emergency_observation_before_abort",
        "actual_planar_and_vx_maximum", "continuous_cruise", "launch_handoff_observed", "coast_entry_observed")}
    failures, transitions, speed_violations, control_mismatches = [], [], [], []
    lookback_counts = {"one_row": 0, "two_rows": 0, "neither_of_previous_two": 0}
    start = handoff = coast = last_active = cruise_start = None
    longest_cruise, previous_target, hold_ticks = 0.0, 0.0, 0
    was_stopped, latched_failure, emergency_count = False, None, 0
    previous_pending = False
    warmup = [i for i, r in enumerate(rows) if r["phase"] == "stationary_warmup"]
    expected_engagement = warmup[-1] if warmup and any(r["phase"] == "driving" for r in rows) else None

    def check(name, ok, frame, detail):
        if not ok:
            flags[name] = False
            failures.append({"frame": frame, "check": name, "detail": detail})

    for index, (state, row) in enumerate(zip(states, rows)):
        gp = state.get("goal_stop", {})
        t, speed, frame = row["timestamp"], row["speed_mps"], row["frame"]
        vx = base.number(state["vx"])
        engagement = gp.get("next_control_starts_driving") is True
        check("engagement_boundary", engagement == (index == expected_engagement), frame, "unexpected engagement row")
        active = row["phase"] == "driving" or engagement
        if index:
            matched_lookback = next((offset for offset in (1, 2) if index >= offset
                and all(same(row["current_control"][key], rows[index - offset]["next_control"][key])
                        for key in ("throttle", "brake", "steer"))), None)
            lookback_counts[{1: "one_row", 2: "two_rows", None: "neither_of_previous_two"}[matched_lookback]] += 1
            if matched_lookback != 1:
                control_mismatches.append({"frame": frame, "timestamp": t, "previous_frame": rows[index - 1]["frame"],
                    "current_observed_control": row["current_control"], "previous_requested_control": rows[index - 1]["next_control"],
                    "earlier_requested_control": rows[index - 2]["next_control"] if index >= 2 else None,
                    "nearest_matching_lookback_rows": matched_lookback})
            for key in ("throttle", "brake", "steer"):
                check("requested_to_applied_control_link", same(row["current_control"][key], rows[index - 1]["next_control"][key]),
                      frame, f"previous next_control != current_control.{key}")
        if previous_pending:
            check("emergency_observation_before_abort", row["phase"] == "driving"
                  and gp.get("termination_reason") == "comfortable_v3_emergency_override", frame, "hazard was not observed then aborted")
        if speed > 30 / 3.6 or vx > 30 / 3.6:
            speed_violations.append({"frame": frame, "timestamp": t, "planar_speed_mps": speed, "vx_mps": vx})
            latched_failure = latched_failure or "comfortable_v3_actual_speed_exceeded"
        if active:
            if start is None:
                start = t
            last_active = t
        elapsed = 0.0 if start is None else last_active - start
        transition = None
        if active and handoff is None:
            if speed >= 0.5:
                handoff, transition = t, "launch_to_normal_pid"
            elif elapsed >= 8.0:
                latched_failure = latched_failure or "comfortable_v3_launch_timeout"
        release_distance = 0.75 + 6.93 * speed
        if active and handoff is not None and coast is None and row["remaining_route_m"] <= release_distance:
            if not 2.8 <= speed <= 3.2:
                latched_failure = latched_failure or "comfortable_v3_coast_entry_speed_outside_band"
            elif latched_failure is None:
                coast, transition = t, "normal_pid_to_zero_pedal_coast"
        coast_elapsed = None if coast is None else last_active - coast
        if row["phase"] == "driving":
            hold_ticks = hold_ticks + 1 if row["stopped_in_goal"] and was_stopped else 0
            was_stopped = row["stopped_in_goal"]
        complete = hold_ticks >= 40
        if active and coast_elapsed is not None and coast_elapsed >= 45.0 and not complete:
            latched_failure = latched_failure or "comfortable_v3_coast_timeout"
        envelope = min(8.0, math.sqrt(3.0396464855480536 ** 2 + 1.2 * max(row["remaining_route_m"] - 35.0, 0.0)))
        target = 0.0 if not active or coast is not None or complete else min(envelope, previous_target + 0.05)
        handoff_elapsed = None if handoff is None else last_active - handoff
        cap = 0.15 if handoff_elapsed is None else min(0.4, 0.15 + 0.05 * (handoff_elapsed + 0.05))
        expected_state = ("complete" if complete else "coast_low" if coast is not None else "launch_low" if handoff is None
                          else "normal_pid") if active else "setup_or_tail"
        expected_fields = {"pilot_state": expected_state, "pilot_transition": transition, "measured_timestamp": t,
            "measured_longitudinal_speed_mps": vx, "driving_elapsed_sec": elapsed, "handoff_elapsed_sec": handoff_elapsed,
            "coast_elapsed_sec": coast_elapsed, "coast_release_distance_m": release_distance,
            "coast_entry_latched": coast is not None, "normal_throttle_cap": cap, "normal_brake_cap": 0.1,
            "complete": complete, "hold_ticks": hold_ticks, "hold_duration_sec": hold_ticks * 0.05,
            "training_data_approved": False}
        for key, value in expected_fields.items():
            check("governor_state_and_timers", same(gp.get(key), value), frame, key)
        for key, value in (("target_speed_mps", target), ("distance_speed_envelope_mps", envelope)):
            check("target_envelope_and_slew", same(gp.get(key), value), frame, key)
        previous_target = target
        if transition:
            transitions.append({"transition": transition, "frame": frame, "timestamp": t, "elapsed_driving_sec": elapsed,
                "speed_mps": speed, "remaining_route_m": row["remaining_route_m"], "release_distance_m": release_distance})
        abort = latched_failure
        if row["phase"] == "driving":
            if not abort and row["recomputed_route_cte_m"] > 3.0:
                abort = "comfortable_goal_route_projection_failure"
            if not abort and (row["remaining_route_m"] <= 1e-6 or (row["remaining_route_m"] <= 1.0
                    and row["goal_error_m"] <= 1.0 and row["terminal_overshoot_m"] > 0)):
                abort = "comfortable_goal_endpoint_overshoot"
            if not abort and complete:
                abort = "comfortable_goal_measured_stop_and_dwell"
            if not abort and gp.get("basic_agent_done") is True and not (row["remaining_route_m"] <= 1.0 and row["goal_error_m"] <= 1.0):
                abort = "basic_agent_done_before_comfortable_goal_window"
            check("governor_state_and_timers", gp.get("termination_reason") == abort, frame, "termination_reason")
        else:
            abort = None
        count = base.integer(gp.get("emergency_override_count"))
        check("emergency_observation_before_abort", count in (emergency_count, emergency_count + 1), frame, "emergency count changed incorrectly")
        new_emergency = count == emergency_count + 1
        if new_emergency:
            latched_failure = latched_failure or "comfortable_v3_emergency_override"
        pending = abort is None and latched_failure == "comfortable_v3_emergency_override"
        check("emergency_observation_before_abort", gp.get("emergency_failure_pending_next_tick") is pending
              and gp.get("next_control_is_unchanged_emergency_override") is pending, frame, "emergency pending marker")
        check("governor_state_and_timers", gp.get("pilot_failure_reason") == latched_failure, frame, "latched failure")
        command = row["next_control"]
        if pending:
            pedal_ok = same(command["throttle"], 0.0) and same(command["brake"], 0.5)
            check("emergency_observation_before_abort", pedal_ok and "unchanged_emergency_override_requested" in str(gp.get("control_source")),
                  frame, "original emergency pedals/source missing")
        elif not active or abort:
            pedal_ok = same(command["throttle"], 0.0) and same(command["brake"], 1.0) and same(command["steer"], 0.0)
        elif coast is not None:
            pedal_ok = same(command["throttle"], 0.0) and same(command["brake"], 0.0)
        elif handoff is None:
            pedal_ok = same(command["throttle"], 0.15) and same(command["brake"], 0.0)
        else:
            pedal_ok = command["throttle"] <= cap + EPS and command["brake"] <= 0.1 + EPS and min(command["throttle"], command["brake"]) <= EPS
        check("normal_pedal_protocol", pedal_ok, frame, "command violates phase or normal pedal cap")
        previous_pending, emergency_count = pending, count
        in_cruise = row["phase"] == "driving" and 7.8 <= speed <= 8.2
        if index and abs(t - rows[index - 1]["timestamp"] - 0.05) > 1e-4:
            cruise_start = None
        if in_cruise:
            cruise_start = t if cruise_start is None else cruise_start
            longest_cruise = max(longest_cruise, t - cruise_start)
        else:
            cruise_start = None
    flags["emergency_observation_before_abort"] &= not previous_pending
    flags["actual_planar_and_vx_maximum"] = not speed_violations
    flags["continuous_cruise"] = longest_cruise + 1e-9 >= 5.0
    flags["launch_handoff_observed"], flags["coast_entry_observed"] = handoff is not None, coast is not None
    return {"flags": flags, "failed_flags": [k for k, v in flags.items() if not v], "all_checks_pass": all(flags.values()),
        "mismatch_count": len(failures), "mismatches": failures, "transitions": transitions,
        "speed_violations": speed_violations, "longest_continuous_cruise_seconds": longest_cruise,
        "emergency_override_count": emergency_count, "latched_failure": latched_failure,
        "coast_elapsed_at_driving_end_seconds": None if coast is None else last_active - coast,
        "control_observation_alignment": {"adjacent_transition_count": max(0, len(rows) - 1),
            "one_row_mismatch_count": len(control_mismatches), "nearest_matching_lookback_histogram": lookback_counts,
            "mismatches": control_mismatches, "absolute_comparison_tolerance": EPS,
            "interpretation": "Nearest equality is a diagnostic association, not proof of physical actuation delay. No command timestamps, state rows, or prior labels were realigned. The strict one-row continuity check remains failed when any discrepancy exists."},
        "normal_steering_nonzero_samples": sum(abs(r["next_control"]["steer"]) > EPS for r in rows if r["phase"] == "driving"),
        "limits_of_control_proof": "PID pre-governor steering, hazard perception and frame-bound observed controls are not separately logged. Frozen source and observed/requested comparisons do not independently prove actuator timing, perception or lateral-controller correctness."}


def audit_trial(root):
    """HH_260906 - Require finalized ownership and ten exact source archives, including archived scalar bounds."""
    root, ledger = Path(root), {}
    plan = read_json(root, "owner_plan.json", ledger)
    owner = read_json(root, "owner_result.json", ledger)
    require(plan.get("source_bytes_archived") is True and plan.get("bounds_source_bytes_archived") is True,
            "v3 requires complete source and bounds archives")
    require(set(plan["source_sha256"]) == SOURCE_NAMES, "v3 requires exactly ten expected archived sources")
    require(isinstance(plan.get("source_worktree_status"), str) and re.fullmatch(r"[0-9a-f]{40}", plan["source_head_commit"]),
            "recorded source identity or worktree status missing")
    require(owner.get("source_bytes_unchanged_and_archived") is True and set(owner["source_checks"]) == SOURCE_NAMES
            and all(v is True for v in owner["source_checks"].values()), "source-after proof failed")
    for name, expected in plan["source_sha256"].items():
        contents = checked_bytes(root, f"provenance/{name}", ledger)
        require(hashlib.sha256(contents).hexdigest() == expected, "archived source SHA mismatch")
        require(contents == recorded_commit_bytes(plan["source_head_commit"], name), "archived execution source differs from recorded Git commit")
        if name in REVIEWED_SOURCES:
            require(expected == REVIEWED_SOURCES[name], "pilot source revision was not reviewed by this auditor")
    require(plan["route_sha256"] == ROUTE_SHA and plan["map"] == "Town07" and plan["capture_mode"] == "expert", "pilot route/mode mismatch")
    started = read_json(root, "owner_started.json", ledger)
    previous_check_time = None
    for stage in ("ready", "after_capture", "stopped"):
        evidence = read_json(root, f"lifecycle/{stage}.json", ledger)
        require(evidence.get("status") == "PASS" and evidence.get("read_only") is True and evidence.get("stage") == stage
                and evidence["port"] == plan["port"] and evidence["expected_map"] == "Town07", "lifecycle proof mismatch")
        require(evidence.get("owner_pid") == started["server_pid"] and evidence.get("owner_pgid") == started["server_pgid"]
                and evidence.get("generation_id") == f"expert_{started['server_pid']}" and evidence.get("host") == plan["host"],
                "lifecycle process generation changed")
        require(evidence.get("mode") == ("stopped" if stage == "stopped" else "running"), "lifecycle mode mismatch")
        require((evidence.get("port_released") is True and evidence.get("owner_process_state") is None) if stage == "stopped"
                else evidence.get("active_map_basename") == "Town07", "lifecycle map or cleanup mismatch")
        check_time = datetime.fromisoformat(evidence["checked_at"])
        require(check_time.tzinfo is not None and (previous_check_time is None or check_time > previous_check_time), "lifecycle timestamps out of order")
        previous_check_time = check_time
    dirs = [name for name in ("episode", "episode.partial") if (root / name).exists()]
    require(len(dirs) <= 1, "both final and partial episode exist")
    # HH_260906 - The base audit independently projects XY, recomputes goal dwell and retains every phase boundary.
    result, timeline = base.summarize_trial(root, None)
    for entry in result["source_manifest"]:
        checked_bytes(root, entry["path"], ledger)
    protocol = None
    if dirs:
        prefix = dirs[0]
        manifest = read_json(root, f"{prefix}/manifest.json", ledger)
        config = manifest["capture_contract"]["goal_stop_profile"]
        require(all(config.get(k) == v and (not isinstance(v, bool) or isinstance(config.get(k), bool)) for k, v in FROZEN.items()),
                "recorded v3 profile differs from the frozen pilot")
        require(manifest["capture_contract"]["target_speed_kmh"] == 28.8 and manifest["result"].get("training_data_approved") is False,
                "nominal cruise or development boundary mismatch")
        states = read_json(root, f"{prefix}/states.jsonl", ledger, lines=True)
        cameras = read_json(root, f"{prefix}/camera_frames.jsonl", ledger, lines=True)
        if timeline is not None:
            protocol = analyze_protocol(states, timeline["native_states"])
        image_names = []
        for camera in cameras:
            for relative in camera["images"].values():
                p = root / prefix / relative
                require(not Path(relative).is_absolute() and ".." not in Path(relative).parts and p.is_file()
                        and not any(q.is_symlink() for q in (p, *p.parents)) and p.resolve().is_relative_to((root / prefix).resolve()),
                        "camera image reference missing or unsafe")
                image_names.append(relative)
        require(len(image_names) == len(set(image_names)), "camera references reuse an image path")
        result["camera_file_reference_count"] = len(image_names)
    candidate = bool(result["raw_quality_candidate"] and protocol and protocol["all_checks_pass"])
    result.update({"pilot_protocol": protocol, "status": "DEVELOPMENT_SCREEN_CLEAR_NOT_ADMITTED" if candidate else "DEVELOPMENT_SCREEN_FAIL",
        "development_screen_clear": candidate, "training_data_approved": False, "source_head_commit": plan["source_head_commit"],
        "recorded_source_worktree_status": plan["source_worktree_status"],
        "archived_execution_sources_match_recorded_commit": True,
        "whole_worktree_was_clean": plan["source_worktree_status"] == "",
        "source_manifest": [{"path": p, **v} for p, v in sorted(ledger.items())]})
    return result


def audit_campaign(root, output):
    root, output = Path(root), Path(output)
    require(root.is_dir() and not root.is_symlink(), "unsafe pilot root")
    require(not output.exists() and not output.is_symlink() and not output.resolve().is_relative_to(root.resolve()), "new output must be outside the raw pilot tree")
    ledger = {}
    pilot_plan = read_json(root, "pilot_plan.json", ledger)
    require(pilot_plan.get("profile") == "comfortable_v3" and pilot_plan.get("maximum_attempts_per_revision") == 2
            and pilot_plan.get("automatic_retry") is False and pilot_plan.get("training_data_approved") is False
            and pilot_plan.get("route_sha256") == ROUTE_SHA, "prospective pilot plan mismatch")
    parent = root / "town07_straight_calibration"
    trials = sorted(p for p in parent.iterdir() if p.name.startswith("run_"))
    require([p.name for p in trials] in (["run_001"], ["run_001", "run_002"]), "include all one or two same-revision pilot attempts")
    require(all((p / "owner_result.json").is_file() for p in trials), "INCOMPLETE: active attempts cannot be omitted")
    results = [audit_trial(p) for p in trials]
    require(len({r["source_head_commit"] for r in results}) == 1, "attempts use different source revisions")
    require(results[0]["source_head_commit"].startswith(pilot_plan["source_commit_short"]), "pilot declaration source differs from execution")
    for p, r in zip(trials, results):
        owner_plan = read_json(p, "owner_plan.json", {})
        require(datetime.fromisoformat(pilot_plan["declared_at_utc"].replace("Z", "+00:00"))
                <= datetime.fromisoformat(owner_plan["planned_at_utc"]), "pilot plan was declared after execution plan")
        for entry in r["source_manifest"]:
            require(base.sha(p / entry["path"]) == entry["sha256"], "input changed during campaign audit")
        if "bounds_source_proof" in r:
            base.recheck_bounds_source_archive(p, r["bounds_source_proof"])
    report = {"schema": "portable_e2e.comfortable_v3_independent_audit.v1", "status": "REVIEWED_NOT_ADMITTED",
        "created_at_utc": datetime.now(timezone.utc).isoformat(), "trial_count": len(results),
        "development_screen_clear_count": sum(r["development_screen_clear"] for r in results),
        "base_raw_quality_candidate_count": sum(r["raw_quality_candidate"] for r in results),
        "development_screen_fail_count": sum(not r["development_screen_clear"] for r in results), "trials": results,
        "pilot_plan_source": ledger["pilot_plan.json"],
        "auditor_source_sha256": base.sha(Path(__file__)), "base_auditor_source_sha256": base.sha(Path(base.__file__)),
        "scope": {"all_discovered_attempts_included": True, "dataset_admission": False, "automatic_promotion": False,
            "camera_metadata_and_file_existence_checked": True, "camera_pixels_or_full_image_hashes_checked": False,
            "full_future_xy_feasibility_checked": False, "live_simulator_access": False, "model_loaded": False,
            "learned_model_control": False, "test_payload_used": False, "original_inputs_modified": False},
        "interpretation": "Same route/condition repeats are not independent sites. A clear development screen is not robust qualification, full future-XY feasibility, image-content QA, data admission or learned driving."}
    require(base.sha(root / "pilot_plan.json") == ledger["pilot_plan.json"]["sha256"], "pilot plan changed during audit")
    output.mkdir(parents=True, exist_ok=False)
    (output / "summary.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    (output / "SHA256SUMS").write_text(f"{base.sha(output / 'summary.json')}  summary.json\n")
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("pilot_root", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        report = audit_campaign(args.pilot_root, args.output_dir)
    except (base.EvidenceError, OSError, ValueError, KeyError, TypeError, StopIteration, subprocess.TimeoutExpired) as error:
        print(f"COMFORTABLE_V3_AUDIT_ERROR: {error}", file=sys.stderr)
        return 2
    print(json.dumps({k: report[k] for k in ("status", "trial_count", "development_screen_clear_count", "development_screen_fail_count")}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
