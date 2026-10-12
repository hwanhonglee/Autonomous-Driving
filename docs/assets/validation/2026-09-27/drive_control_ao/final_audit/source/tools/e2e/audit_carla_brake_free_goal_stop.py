#!/usr/bin/env python3
"""HH_260906 - Audit the explicit brake-free V4 revision without changing V3 pins or admitting data."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import sys

if __package__:
    from . import audit_carla_acknowledged_control as ack
    from . import audit_carla_comfortable_v3_trial as pilot
    from . import summarize_carla_goal_stop_trials as base
else:
    import audit_carla_acknowledged_control as ack
    import audit_carla_comfortable_v3_trial as pilot
    import summarize_carla_goal_stop_trials as base

ROOT = Path(__file__).resolve().parents[2]
COMMIT = "9d3dcab9d425df1ba673f1e932686049bd17a011"
REVIEWED_SOURCES = {
    "scripts/e2e/collect_carla_vad_expert.py": "53d213bdef56895a27969cc6dd4eac638ba4fa6697badeaaa3367bd5741f01d0",
    "scripts/e2e/carla_goal_stop_profile.py": "6e70b4362b805b73be66bc727afc79a8684dfc12f2f5d69998fbfffa9eef3dab",
}
ROUTE_SHA = "8285e70a790d5e8ae75803db1aa538122b56e6fcfc3586e60eaa947d330417c9"
require, same = base.require, pilot.same
# HH_260906 - This separate literal contract and reconstruction deliberately retain every V3 limit except normal brake zero.
FROZEN = {
    "profile_id": "comfortable_v4", "desired_deceleration_mps2": 0.6,
    "target_acceleration_limit_mps2": 1.0, "normal_brake_cap": 0.0, "normal_throttle_cap": 0.4,
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
                  and gp.get("termination_reason") == "comfortable_v4_emergency_override", frame, "hazard was not observed then aborted")
        if speed > 30 / 3.6 or vx > 30 / 3.6:
            speed_violations.append({"frame": frame, "timestamp": t, "planar_speed_mps": speed, "vx_mps": vx})
            latched_failure = latched_failure or "comfortable_v4_actual_speed_exceeded"
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
                latched_failure = latched_failure or "comfortable_v4_launch_timeout"
        release_distance = 0.75 + 6.93 * speed
        if active and handoff is not None and coast is None and row["remaining_route_m"] <= release_distance:
            if not 2.8 <= speed <= 3.2:
                latched_failure = latched_failure or "comfortable_v4_coast_entry_speed_outside_band"
            elif latched_failure is None:
                coast, transition = t, "normal_pid_to_zero_pedal_coast"
        coast_elapsed = None if coast is None else last_active - coast
        if row["phase"] == "driving":
            hold_ticks = hold_ticks + 1 if row["stopped_in_goal"] and was_stopped else 0
            was_stopped = row["stopped_in_goal"]
        complete = hold_ticks >= 40
        if active and coast_elapsed is not None and coast_elapsed >= 45.0 and not complete:
            latched_failure = latched_failure or "comfortable_v4_coast_timeout"
        envelope = min(8.0, math.sqrt(3.0396464855480536 ** 2 + 1.2 * max(row["remaining_route_m"] - 35.0, 0.0)))
        target = 0.0 if not active or coast is not None or complete else min(envelope, previous_target + 0.05)
        handoff_elapsed = None if handoff is None else last_active - handoff
        cap = 0.15 if handoff_elapsed is None else min(0.4, 0.15 + 0.05 * (handoff_elapsed + 0.05))
        expected_state = ("complete" if complete else "coast_low" if coast is not None else "launch_low" if handoff is None
                          else "normal_pid") if active else "setup_or_tail"
        expected_fields = {"pilot_state": expected_state, "pilot_transition": transition, "measured_timestamp": t,
            "measured_longitudinal_speed_mps": vx, "driving_elapsed_sec": elapsed, "handoff_elapsed_sec": handoff_elapsed,
            "coast_elapsed_sec": coast_elapsed, "coast_release_distance_m": release_distance,
            "coast_entry_latched": coast is not None, "normal_throttle_cap": cap, "normal_brake_cap": 0.0,
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
            latched_failure = latched_failure or "comfortable_v4_emergency_override"
        pending = abort is None and latched_failure == "comfortable_v4_emergency_override"
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
            pedal_ok = command["throttle"] <= cap + EPS and command["brake"] == 0.0 and min(command["throttle"], command["brake"]) <= EPS
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

def verify_owner_sources(root, ledger):
    """HH_260906 - Verify the new explicit revision without overriding the original async auditor's frozen pins."""
    read = lambda name: pilot.read_json(root, name, ledger)
    plan, owner, started = read("owner_plan.json"), read("owner_result.json"), read("owner_started.json")
    base.require(plan.get("source_bytes_archived") is True and plan.get("bounds_source_bytes_archived") is True
                 and set(plan["source_sha256"]) == pilot.SOURCE_NAMES, "ack trial requires exactly ten archived sources")
    base.require(owner.get("source_bytes_unchanged_and_archived") is True and set(owner["source_checks"]) == pilot.SOURCE_NAMES
                 and all(value is True for value in owner["source_checks"].values()), "owned source postcheck failed")
    base.require(plan.get("host") == "127.0.0.1" and plan.get("map") == started.get("map") == "Town07"
                 and plan.get("capture_mode") == "expert" and plan.get("quality") == started.get("quality") == "Low"
                 and plan.get("route_sha256") == pilot.ROUTE_SHA, "ack trial is outside the reviewed Low Town07 expert protocol")
    base.require(owner.get("learned_model_control") is False and owner.get("vehicle_control_approved") is False,
                 "owner scope must remain expert-only")
    argv = plan["collector_argv"]
    base.require(argv.count("--control-transport") == 1 and argv[argv.index("--control-transport") + 1] == "acknowledged_batch",
                 "acknowledged transport was not explicitly selected")
    base.require({path.relative_to(root / "provenance").as_posix() for path in (root / "provenance").rglob("*")
                  if path.is_file() or path.is_symlink()} == pilot.SOURCE_NAMES, "unexpected or missing source archive path")
    for name, expected in plan["source_sha256"].items():
        raw = pilot.checked_bytes(root, "provenance/" + name, ledger)
        base.require(hashlib.sha256(raw).hexdigest() == expected, "source archive SHA mismatch")
        base.require(raw == pilot.recorded_commit_bytes(plan["source_head_commit"], name), "source archive differs from recorded commit")
        base.require(raw == pilot.recorded_commit_bytes(COMMIT, name), "execution bytes differ from reviewed V4 commit")
        if name in REVIEWED_SOURCES:
            base.require(expected == REVIEWED_SOURCES[name], "unreviewed acknowledged protocol source revision")
    pid = base.integer(started["server_pid"])
    base.require(pid > 1 and started["server_pgid"] == pid and started["port"] == plan["port"], "owned startup identity mismatch")
    log = pilot.checked_bytes(root, "server.log", ledger)
    previous = None
    for stage in ("ready", "after_capture", "stopped"):
        evidence = read("lifecycle/" + stage + ".json")
        base.require(evidence.get("status") == "PASS" and evidence.get("stage") == stage and evidence.get("read_only") is True
                     and evidence.get("owner_pid") == evidence.get("owner_pgid") == pid
                     and evidence.get("generation_id") == f"expert_{pid}" and evidence.get("host") == plan["host"]
                     and evidence.get("port") == plan["port"] and evidence.get("expected_map") == "Town07", "owned lifecycle mismatch")
        base.require(evidence.get("mode") == ("stopped" if stage == "stopped" else "running"), "lifecycle mode mismatch")
        base.require((evidence.get("port_released") is True and evidence.get("owner_process_state") is None) if stage == "stopped"
                     else evidence.get("active_map_basename") == "Town07", "running map or stopped proof mismatch")
        prefix = evidence["server_log"]
        base.require(type(prefix["size_bytes"]) is int and 0 < prefix["size_bytes"] <= len(log)
                     and hashlib.sha256(log[:prefix["size_bytes"]]).hexdigest() == prefix["sha256"], "lifecycle log prefix SHA mismatch")
        current = datetime.fromisoformat(evidence["checked_at"])
        base.require(current.tzinfo is not None and (previous is None or current > previous), "lifecycle timestamps are out of order")
        previous = current
    return plan, owner


def strict_equal(actual, expected):
    """HH_260906 - Reject bool-as-number and extra nested fields in prospective contracts."""
    if isinstance(expected, dict):
        return isinstance(actual, dict) and set(actual) == set(expected) and all(strict_equal(actual[k], v) for k, v in expected.items())
    if isinstance(expected, list):
        return isinstance(actual, list) and len(actual) == len(expected) and all(strict_equal(a, e) for a, e in zip(actual, expected))
    if type(expected) in (int, float):
        return type(actual) in (int, float) and math.isfinite(actual) and actual == expected
    return type(actual) is type(expected) and actual == expected


def utc(value):
    require(isinstance(value, str), "timestamp must be a UTC string")
    result = datetime.fromisoformat(value.replace("Z", "+00:00"))
    require(result.tzinfo is not None and result.utcoffset().total_seconds() == 0, "timestamp is not UTC")
    return result


def validate_plan_document(plan):
    """HH_260906 - V4 has its own explicitly frozen plan; no V3 campaign or source pin is monkeypatched."""
    expected = {
        "source_commit": COMMIT, "collector_sha256": REVIEWED_SOURCES["scripts/e2e/collect_carla_vad_expert.py"],
        "goal_helper_sha256": REVIEWED_SOURCES["scripts/e2e/carla_goal_stop_profile.py"],
        "profile": "comfortable_v4", "control_transport": "acknowledged_batch",
        "changed_numerical_configuration": {"normal_brake_cap": {"from": .1, "to": 0}},
        "route_sha256": ROUTE_SHA, "route_length_m": 210.5975914062836, "map": "Town07", "scenario": "straight",
        "vehicle": "vehicle.toyota.prius", "weather": "ClearNoon", "seed": 0, "quality": "Low",
        "nominal_target_speed_kmh": 28.8, "maximum_actual_speed_kmh": 30, "physics_hz": 20, "camera_hz": 10,
        "maximum_total_sim_seconds": 180, "wall_timeout_seconds": 900, "finish_before_utc": "2026-09-08T01:00:00Z",
        "maximum_attempts_per_revision": 2, "automatic_retry": False, "first_output": "town07_straight_calibration/run_001",
        "baseline_outputs": ["../acknowledged_control_v1/town07_straight_calibration/run_001",
                             "../acknowledged_control_v1/town07_straight_calibration/run_002"],
        "normal_lateral_controller_unchanged": True, "emergency_control_unchanged": True, "quality_limits_unchanged": True,
        "training_data_approved": False, "learned_model_control": False, "remote_training_started": False}
    extra = {"comment", "declared_at_utc", "expected_risk", "notice"}
    require(isinstance(plan, dict) and set(plan) == set(expected) | extra, "unknown or missing V4 plan field")
    require(all(strict_equal(plan[k], v) for k, v in expected.items()), "prospective V4 plan differs from frozen revision")
    require(all(isinstance(plan[k], str) and plan[k] for k in extra), "empty prospective explanation")
    require(utc(plan["declared_at_utc"]) < utc(plan["finish_before_utc"]), "plan declared after deadline")


def validate_cli(owner, root):
    """HH_260906 - Bind every option, including route, rig and lateral PID, without importing the live collector."""
    argv = owner["collector_argv"]
    expected = {"--host": "127.0.0.1", "--port": str(owner["port"]), "--physics-hz": 20., "--capture-hz": 10.,
        "--target-speed-kmh": 28.8, "--max-duration-sec": 180., "--stationary-warmup-sec": 3.5,
        "--stationary-tail-sec": 6.5, "--spawn-z-offset-m": .5, "--weather": "ClearNoon", "--seed": 0.,
        "--goal-stop-profile": "comfortable_v4", "--goal-tolerance-m": 1., "--control-transport": "acknowledged_batch",
        "--mapping": "autoware_e2e_vad_launch/config/sensor_mapping_vad_fast_reliable.yaml",
        "--calibration": "src/launcher/autoware_launch/sensor_kit/carla_sensor_kit_launch/carla_sensor_kit_description/config/sensor_kit_calibration.yaml",
        "--basic-agent-base-min-distance-m": 3., "--basic-agent-distance-ratio": .5,
        "--basic-agent-lateral-kp": 1.95, "--basic-agent-lateral-ki": .05, "--basic-agent-lateral-kd": .2,
        "--basic-agent-max-steering": .8, "--basic-agent-lane-offset-m": 0.}
    require(isinstance(argv, list) and len(argv) == 2 + 2 * len(expected) and all(isinstance(x, str) for x in argv), "extra or missing V4 CLI argument")
    require(Path(argv[0]).resolve() == (root / "episode").resolve() and argv[1] == owner["route_path"], "output or route CLI mismatch")
    options = argv[2::2]
    require(len(set(options)) == len(options) and set(options) == set(expected), "duplicate, abbreviated or unknown CLI option")
    for key, value in zip(options, argv[3::2]):
        require(strict_equal(float(value), expected[key]) if type(expected[key]) is float else value == expected[key], "frozen CLI value differs: " + key)


def validate_profile(manifest):
    config = manifest["capture_contract"]["goal_stop_profile"]
    require(all(strict_equal(config.get(k), v) for k, v in FROZEN.items()), "V4 profile differs from explicit frozen configuration")
    require(manifest["capture_contract"]["target_speed_kmh"] == 28.8 and manifest["coordinate_contract"]["wheelbase_m"] == 2.85
            and manifest["result"].get("training_data_approved") is False, "V4 geometry, target or admission boundary changed")
    effective = config.get("effective_control")
    if effective is not None:
        require(effective["normal_brake_cap"] == 0 and effective["normal_throttle_cap"] == .4
                and effective["basic_agent_emergency_brake_modified"] is False and effective["lateral_control_modified"] is False
                and effective["emergency_return_control_modified"] is False
                and effective["unchanged_basic_agent_emergency_brake"] == .5, "normal cap or emergency/lateral boundary changed")
    transport = manifest["capture_contract"]["control_transport"]
    expected = {"schema": "carla.acknowledged_control_transport.v1", "mode": "acknowledged_batch",
        "single_actor_single_response": True, "implicit_tick": False, "async_fallback_allowed": False,
        "receipt_journal": "control_receipts.jsonl", "receipt_persisted_before_next_tick": True,
        "comparison_absolute_tolerance": 1e-6, "automatic_gear_equality_required": False,
        "label_rewrite_or_lookback_pass_allowed": False, "physical_actuation_proven": False,
        "native_acceleration_definition_changed": False,
        "kinematics_source": "tick-exact immutable WorldSnapshot.find(owned_actor_id)"}
    require(all(strict_equal(transport.get(k), v) for k, v in expected.items()), "V4 ACK contract changed")
    return transport


def audit_trial(root):
    """HH_260906 - Failed/no-tail/empty captures remain auditable failures, never filtered successful subsets."""
    root, ledger = Path(root), {}
    require((root / "owner_result.json").is_file(), "INCOMPLETE: V4 owner result missing")
    plan, owner = verify_owner_sources(root, ledger)
    validate_cli(plan, root)
    result, timeline = base.summarize_trial(root, None)
    for entry in result["source_manifest"]:
        pilot.checked_bytes(root, entry["path"], ledger)
    directories = [name for name in ("episode", "episode.partial") if (root / name).exists()]
    require(len(directories) <= 1, "both final and partial V4 episodes exist")
    images, states, receipts, cameras, manifest, transport = [], [], [], [], None, {}
    if directories:
        prefix = directories[0] + "/"
        manifest = pilot.read_json(root, prefix + "manifest.json", ledger)
        transport = validate_profile(manifest)
        states = pilot.read_json(root, prefix + "states.jsonl", ledger, lines=True)
        cameras = pilot.read_json(root, prefix + "camera_frames.jsonl", ledger, lines=True)
        if (root / prefix / "control_receipts.jsonl").exists():
            receipts = pilot.read_json(root, prefix + "control_receipts.jsonl", ledger, lines=True)
        else:
            require(not states and owner["exit_code"] != 0 and manifest["status"] == "failed", "missing receipt journal for observed capture")
    measured = ack.analyze_transport(states, receipts, transport.get("bootstrap_observation"))
    measured["camera_alignment"] = ack.analyze_camera_alignment(states, cameras)
    measured["flags"]["complete_camera_frame_alignment"] = measured["camera_alignment"]["status"] == "PASS"
    measured["failed_flags"] = [k for k, v in measured["flags"].items() if not v]
    measured["status"] = "FAIL" if measured["failed_flags"] else "PASS"
    declared = manifest["result"].get("control_transport", {}) if manifest else {}
    if receipts or states or declared:
        require(declared.get("receipt_journal_sha256") == ledger[prefix + "control_receipts.jsonl"]["sha256"], "receipt manifest SHA mismatch")
        require(declared.get("receipt_journal_error") is None and declared.get("physical_actuation_proven") is False, "receipt finalization or physical claim invalid")
        for field, key in (("command_receipt_count", "receipt_count"), ("acknowledged_command_count", "acknowledged_count"),
                           ("failed_command_count", "failed_receipt_count"), ("control_alignment_failure_count", "alignment_failure_count")):
            require(type(declared.get(field)) is int and declared[field] == measured[key], "receipt denominator differs from manifest")
    protocol = analyze_protocol(states, timeline["native_states"]) if timeline else None
    for camera in cameras:
        for name in base.CAMERAS:
            path = prefix + camera["images"][name]
            raw = pilot.checked_bytes(root, path, {})
            images.append({"path": path, "sha256": hashlib.sha256(raw).hexdigest(), "size_bytes": len(raw)})
    require(len({x["path"] for x in images}) == len(images), "duplicate image references")
    images.sort(key=lambda x: x["path"])
    normal = [row for row in states if row["capture_phase"] == "driving" and not row["goal_stop"].get("termination_reason")
              and not row["goal_stop"].get("emergency_failure_pending_next_tick")]
    result.update({"status": "AUDITED_NOT_ADMITTED", "transport_protocol": measured, "pilot_protocol": protocol,
        "normal_driving_nonzero_brake_count": sum(row["next_control"]["brake"] != 0 for row in normal),
        "normal_driving_command_count": len(normal), "source_head_commit": plan["source_head_commit"],
        "reviewed_execution_commit": COMMIT, "all_ten_archives_match_owner_and_reviewed_commits": True,
        "whole_owner_head_identical_to_reviewed_commit": plan["source_head_commit"] == COMMIT,
        "archived_source_sha256": plan["source_sha256"], "comparison_configuration": ack.comparison_configuration(manifest) if manifest and "runtime" in manifest else None,
        "image_byte_integrity": {"file_count": len(images), "total_size_bytes": sum(x["size_bytes"] for x in images),
            "canonical_sorted_file_ledger_sha256": hashlib.sha256(json.dumps(images, sort_keys=True, separators=(",", ":")).encode()).hexdigest(),
            "pixels_decoded_or_visually_approved": False}, "source_manifest": [{"path": name, **value} for name, value in sorted(ledger.items())],
        "training_data_approved": False, "dataset_admission": False, "full_future_xy_admission": False,
        "physical_actuation_proven": False, "failed_before_first_observation": not states})
    for item in result["source_manifest"] + images:
        require(base.sha(root / item["path"]) == item["sha256"], "V4 input changed during audit")
    if "bounds_source_proof" in result:
        base.recheck_bounds_source_archive(root, result["bounds_source_proof"])
    return result, images, timeline


def verify_second_review(review, first, first_plan, second_plan, completed, planned, ledger):
    """HH_260906 - Recompute the pre-second review rather than trust its first-trial PASS claim."""
    qa, transport = first["independent_qa"], first["transport_protocol"]
    expected = {"previous_output": "town07_straight_calibration/run_001", "next_output": "town07_straight_calibration/run_002",
        "reviewed_execution_source_commit": COMMIT, "previous_owner_head": first_plan["source_head_commit"],
        "next_expected_owner_head": second_plan["source_head_commit"], "owner_exit_code": first["owner_exit_code"],
        "source_checks_passed": 10, "lifecycle_ready_after_stopped": "PASS", "independent_transport_status": transport["status"],
        "acknowledged_receipts": transport["acknowledged_count"], "control_mismatch_count": transport["control_mismatch_count"],
        "native_scalar_quality": "PASS" if qa["raw_scalar_quality_clear"] else "FAIL",
        "collision_count": qa["event_counts"]["collision"], "lane_invasion_count": qa["event_counts"]["lane_invasion"],
        "normal_driving_nonzero_brake_count": first["normal_driving_nonzero_brake_count"],
        "source_and_parameters_changed": False, "maximum_attempts_per_revision": 2, "automatic_retry": False,
        "training_data_approved": False, "goal_distance_m": qa["final_driving"]["goal_error_m"]}
    for cadence, label in (("native_20hz", "native"), ("camera_10hz", "camera")):
        values = qa["speed_rate_qa"][cadence]["by_phase"]["all"]
        for field in ("minimum", "maximum"):
            expected[f"{field}_{label}_speed_rate_mps2"] = values[field + "_mps2"]
    for kind in ("states", "receipts"):
        suffix = "states.jsonl" if kind == "states" else "control_receipts.jsonl"
        matches = [v["sha256"] for k, v in ledger.items() if k.startswith("town07_straight_calibration/run_001/") and k.endswith("/" + suffix)]
        require(len(matches) == 1, "first review source journal ambiguity")
        expected[f"previous_{kind}_sha256"] = matches[0]
    extra = {"comment", "reviewed_at_utc", "head_change_notice", "reason"}
    require(isinstance(review, dict) and set(review) == set(expected) | extra, "unexpected second review fields")
    require(all(strict_equal(review[k], v) for k, v in expected.items()), "second review contradicts independent first evidence")
    require(all(isinstance(review[k], str) and review[k] for k in extra), "empty second review explanation")
    require(completed <= utc(review["reviewed_at_utc"]) < planned, "second review does not precede second planning")


def validate_campaign_plan(campaign, baseline_root, audited_results):
    """HH_260906 - Retain one or two discovered attempts, with strict recorded prospective time and source bindings."""
    campaign, baseline_root, ledger = Path(campaign), Path(baseline_root), {}
    plan = pilot.read_json(campaign, "pilot_plan.json", ledger)
    validate_plan_document(plan)
    roots = sorted(path for path in (campaign / "town07_straight_calibration").iterdir() if path.name.startswith("run_"))
    names = [path.name for path in roots]
    require(names in (["run_001"], ["run_001", "run_002"]), "noncontiguous or excess V4 attempt denominator")
    require([row["trial_id"] for row in audited_results] == names, "audited subset does not match all attempted V4 runs")
    for name, relative in zip(("run_001", "run_002"), plan["baseline_outputs"]):
        require((campaign / relative).resolve() == (baseline_root / name).resolve(), "prospective baseline root mismatch")
    declared, deadline = utc(plan["declared_at_utc"]), utc(plan["finish_before_utc"])
    owner_plans, completed_times, planned_times, rows = [], [], [], []
    for root, audited in zip(roots, audited_results):
        require(root.is_dir() and not root.is_symlink(), "unsafe V4 attempt root")
        relative = root.relative_to(campaign).as_posix()
        read = lambda name: pilot.read_json(campaign, relative + "/" + name, ledger)
        owner_plan, started, owner = read("owner_plan.json"), read("owner_started.json"), read("owner_result.json")
        require(owner_plan["collector_wall_timeout_sec"] == 900 and owner_plan["finish_before_utc"] == plan["finish_before_utc"], "attempt time budget differs from declaration")
        require(audited["all_ten_archives_match_owner_and_reviewed_commits"] is True
                and audited["source_head_commit"] == owner_plan["source_head_commit"]
                and audited["archived_source_sha256"] == owner_plan["source_sha256"], "audit and owner source identities disagree")
        for source, digest in owner_plan["source_sha256"].items():
            archived = pilot.checked_bytes(campaign, relative + "/provenance/" + source, ledger)
            require(hashlib.sha256(archived).hexdigest() == digest and archived == pilot.recorded_commit_bytes(COMMIT, source)
                    and archived == pilot.recorded_commit_bytes(owner_plan["source_head_commit"], source), "prospective execution-byte identity mismatch")
        planned, start, completed = utc(owner_plan["planned_at_utc"]), utc(started["started_at_utc"]), utc(owner["completed_at_utc"])
        checks = [utc(read("lifecycle/" + stage + ".json")["checked_at"]) for stage in ("ready", "after_capture", "stopped")]
        # HH_260906 - The owned wrapper writes owner_started only after the successful ready probe, not at process spawn.
        require(declared <= planned < checks[0] <= start < checks[1] < checks[2] <= completed <= deadline, "prospective execution timestamps contradict declaration or deadline")
        for directory in ("episode", "episode.partial"):
            if (root / directory / "manifest.json").exists():
                require(start <= utc(read(directory + "/manifest.json")["created_at"]) <= checks[1], "capture creation is outside recorded ready/capture interval")
        for entry in audited["source_manifest"]:
            pilot.checked_bytes(campaign, relative + "/" + entry["path"], ledger)
        owner_plans.append(owner_plan)
        completed_times.append(completed)
        planned_times.append(planned)
        rows.append({"trial_id": root.name, "finalized_at_audit": True, "owner_exit_code": owner["exit_code"],
            "owner_head": owner_plan["source_head_commit"], "reviewed_execution_commit": COMMIT,
            "all_ten_execution_files_equal_reviewed_commit": True})
    review = None
    if len(roots) == 2:
        review = pilot.read_json(campaign, "run_002_preflight_review.json", ledger)
        verify_second_review(review, audited_results[0], *owner_plans, completed_times[0], planned_times[1], ledger)
    return {"status": "RECORDED_PROSPECTIVE_PLAN_SOURCE_AND_TIME_BINDINGS_VERIFIED", "plan": plan,
        "plan_sha256": ledger["pilot_plan.json"]["sha256"], "discovered_attempts": rows, "attempt_count": len(rows),
        "maximum_attempt_count": 2, "all_attempts_finalized": True, "second_review_record": review,
        "second_review_record_verified": review is not None, "historical_file_creation_time_proven": False,
        "notice": "Recorded declarations and ordered execution/review timestamps are verified. This is not immutable historical filesystem creation proof. Publication-only owner HEAD differences are allowed solely when all ten execution archives exactly match both owner and reviewed commits.",
        "source_manifest": [{"path": name, **value} for name, value in sorted(ledger.items())]}


def audit_campaign(campaign, baseline_root):
    """HH_260906 - Independently include both ACK V3 baselines and every bounded V4 attempt."""
    campaign, baseline_root = Path(campaign), Path(baseline_root)
    roots = sorted(path for path in (campaign / "town07_straight_calibration").iterdir() if path.name.startswith("run_"))
    require([p.name for p in roots] in (["run_001"], ["run_001", "run_002"]), "V4 attempted-run denominator invalid")
    require(sorted(p.name for p in baseline_root.iterdir() if p.name.startswith("run_")) == ["run_001", "run_002"], "both ACK V3 baselines are required")
    audits = [audit_trial(root) for root in roots]
    baselines = [ack.audit_trial(baseline_root / name) for name in ("run_001", "run_002")]
    results = [item[0] for item in audits]
    prospective = validate_campaign_plan(campaign, baseline_root, results)
    configurations = [r["comparison_configuration"] for r in results] + [r["comparison_configuration"] for r, _ in baselines]
    require(configurations[0] is not None and all(value == configurations[0] for value in configurations), "camera/world/BasicAgent comparison conditions changed")
    profile_differences = []
    for v4 in results:
        for v3, _ in baselines:
            differences = {key: {"baseline": v3["goal_stop_profile"].get(key), "candidate": v4["goal_stop_profile"].get(key)}
                for key in set(v3["goal_stop_profile"]) | set(v4["goal_stop_profile"])
                if not strict_equal(v3["goal_stop_profile"].get(key), v4["goal_stop_profile"].get(key))}
            require(differences == {"profile_id": {"baseline": "comfortable_v3", "candidate": "comfortable_v4"},
                                   "normal_brake_cap": {"baseline": .1, "candidate": 0}}, "comparison changes more than the declared brake cap and profile ID")
            profile_differences.append({"v3": v3["trial_id"], "v4": v4["trial_id"], "differences": differences})
    report = {"schema": "portable_e2e.brake_free_goal_stop_campaign_audit.v1", "status": "AUDITED_NOT_ADMITTED",
        "created_at_utc": datetime.now(timezone.utc).isoformat(), "reviewed_execution_commit": COMMIT,
        "candidate_attempt_count": len(results), "baseline_attempt_count": 2,
        "candidate_trials": results, "baseline_trials": [r for r, _ in baselines], "prospective_plan_proof": prospective,
        "same_physical_camera_and_basic_agent_configuration": True, "declared_profile_differences": profile_differences,
        "scope": {"all_discovered_attempts_included": True, "same_initial_condition_repetitions_not_independent_routes": True,
            "training_data_approved": False, "dataset_admission": False, "automatic_promotion": False,
            "full_future_xy_admission": False, "camera_pixels_visually_approved": False,
            "physical_actuation_proven": False, "model_loaded": False, "live_simulator_access": False, "test_payload_used": False},
        "interpretation": "This is a bounded control-configuration comparison, not learned autonomous driving. All normal braking was removed while setup, verified stopped-tail and emergency/abort braking remain. API-reported controls and server acceptance do not prove physical pedal or torque timing. Scalar and protocol PASS do not establish full XY feasibility, visual quality, road generalization or data admission.",
        "audit_source_sha256": {Path(path).resolve().relative_to(ROOT).as_posix(): base.sha(Path(path))
            for path in (__file__, ack.__file__, pilot.__file__, base.__file__)}}
    bindings = [(campaign, prospective["source_manifest"])]
    bindings += [(root, r["source_manifest"] + images) for root, (r, images, _) in zip(roots, audits)]
    bindings += [(baseline_root / r["trial_id"], r["source_manifest"] + images) for r, images in baselines]
    for raw_root, entries in bindings:
        for entry in entries:
            require(base.sha(raw_root / entry["path"]) == entry["sha256"], "campaign evidence changed after audit")
    require(sorted(p.name for p in roots[0].parent.iterdir() if p.name.startswith("run_")) == [p.name for p in roots], "V4 denominator changed during audit")
    return report, audits


def sanitize(value):
    """HH_260906 - Retain raw hashes while removing account-specific paths and endpoints from public metadata."""
    if isinstance(value, str):
        value = re.sub(r"/home/[^/\s]+/autoware_e2e(?=/|$)", "${REPO_ROOT}", value)
        value = re.sub(r"/home/[^/\s]+", "${USER_HOME}", value)
        return re.sub(r"\b(?:\d{1,3}\.){3}\d{1,3}\b", "${HOST}", value)
    if isinstance(value, list):
        return [sanitize(item) for item in value]
    if isinstance(value, dict):
        return {key: sanitize(item) for key, item in value.items()}
    return value


def new_output(output, roots):
    output = Path(output)
    require(not output.exists() and not output.is_symlink() and not any(p.is_symlink() for p in output.parents), "publication output exists or is aliased")
    require(all(not output.resolve().is_relative_to(Path(root).resolve()) for root in roots), "publication output is inside raw evidence")
    return output


def checked_visual_payloads(visual_root, raw_root):
    """HH_260906 - Copy only exact already-rendered actual-camera bytes after raw-source and decode checks."""
    from PIL import Image
    visual_root, raw_root, ledger = Path(visual_root), Path(raw_root), {}
    visual = pilot.read_json(visual_root, "visual_provenance.json", ledger)
    require(visual.get("schema") == "carla_expert.raw_trial_visual_diagnostic.v1", "unknown raw visual schema")
    prefix = visual["source_episode_name"]
    require(prefix in ("episode", "episode.partial"), "unsafe visual episode name")
    require(set(visual["source_metadata_sha256"]) == {"manifest.json", "route.json", "states.jsonl", "camera_frames.jsonl"}, "visual source metadata set differs")
    for name, digest in visual["source_metadata_sha256"].items():
        require(hashlib.sha256(pilot.checked_bytes(raw_root, prefix + "/" + name, {})).hexdigest() == digest, "visual belongs to different raw capture")
    for name, digest in visual["displayed_image_sha256"].items():
        require(hashlib.sha256(pilot.checked_bytes(raw_root, prefix + "/" + name, {})).hexdigest() == digest, "visual source image changed")
    names = {"01_start.png", "02_measured_cruise.png", "03_coast_entry.png", "04_goal_dwell.png",
             "05_final_observation.png", "06_maximum_observed_deceleration.png", "whole_recording_accelerated.gif", "visual_provenance.json"}
    require({p.name for p in visual_root.iterdir()} == names, "incomplete or unexpected visual output set")
    payloads, dimensions = {}, {}
    for name in sorted(names):
        raw = pilot.checked_bytes(visual_root, name, ledger)
        if name.endswith((".png", ".gif")):
            with Image.open(visual_root / name) as im:
                require(im.size == (1600, 900), "visual dimensions differ from recorded renderer")
                count = getattr(im, "n_frames", 1)
                for index in range(count):
                    im.seek(index)
                    im.load()
                dimensions[name] = {"width": 1600, "height": 900, "frames": count}
                if name.endswith(".gif"):
                    require(count == len(visual["rendered_indices"]), "GIF frame count differs from render provenance")
        payloads[name] = raw
    return payloads, {"original_visual_provenance": visual, "source_manifest": [{"path": k, **v} for k, v in sorted(ledger.items())],
        "decoded_outputs": dimensions, "publication_pixels_changed": False,
        "notice": "Exact existing 1600x900 rendered camera/route diagnostics, not live Autoware screenshots. GIF is accelerated and sampled at its declared stride; it is not camera FPS measurement."}


def render_comparison(report, campaign, baseline_root, output):
    """HH_260906 - Preserve every native interval, including setup, tail, phase boundaries and failed baseline spikes."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    figure, axes = plt.subplots(2, 2, figsize=(12.8, 7.2), squeeze=False)
    sources = [("V3 ACK / " + r["trial_id"], Path(baseline_root) / r["trial_id"], r) for r in report["baseline_trials"]]
    sources += [("V4 zero normal brake / " + r["trial_id"], Path(campaign) / "town07_straight_calibration" / r["trial_id"], r)
                for r in report["candidate_trials"]]
    for axis, (label, root, result) in zip(axes.flat, sources):
        _, timeline = base.summarize_trial(root, None)
        if timeline:
            for key, color, title in (("native_intervals", "#226699", "20 Hz"), ("camera_intervals", "#df7733", "Camera-aligned 10 Hz")):
                values = timeline[key]
                axis.plot([v["time_from_capture_start_s"] for v in values], [v["speed_rate_mps2"] for v in values], color=color, lw=1, label=title)
        axis.axhline(2.9, color="#111111", ls="--", lw=.8, label="Decoder +/-2.9")
        axis.axhline(-2.9, color="#111111", ls="--", lw=.8)
        axis.set_title(label + " | scalar " + ("CLEAR" if result["raw_quality_candidate"] else "FAIL"), fontsize=11)
        axis.set_xlabel("Seconds from first recorded state")
        axis.set_ylabel("Measured speed rate (m/s²)")
        axis.grid(alpha=.15)
        axis.legend(fontsize=8)
    for axis in list(axes.flat)[len(sources):]:
        axis.set_visible(False)
    figure.suptitle("All bounded attempts retained: two braking baselines and zero-normal-brake candidates", fontsize=13)
    figure.text(.5, .014, "Actual scalar traces only. Different panel Y scales retain baseline spikes. Not full XY, image quality, dataset admission or learned autonomy.", ha="center", fontsize=9)
    figure.tight_layout(rect=(0, .035, 1, .95))
    figure.savefig(output, dpi=150)
    plt.close(figure)


def publish(campaign, baseline_root, output, visual_root=None):
    """HH_260906 - Create a new category only; failed private evidence and prior publications remain untouched."""
    campaign, baseline_root = Path(campaign), Path(baseline_root)
    output = new_output(output, [campaign, baseline_root] + ([] if visual_root is None else [visual_root]))
    report, audits = audit_campaign(campaign, baseline_root)
    visuals = {}
    if visual_root is not None:
        for trial in report["candidate_trials"]:
            name = trial["trial_id"]
            visuals[name] = checked_visual_payloads(Path(visual_root) / name, campaign / "town07_straight_calibration" / name)
    report["publication_notice"] = "Redacted independently recomputed metadata view with raw-source SHA; no raw states, labels or image corpus copied into Git. Original private evidence remains unchanged."
    output.mkdir(parents=True, exist_ok=False)
    def write(name, value):
        with (output / name).open("x", encoding="utf-8") as stream:
            stream.write(value)
    write("summary.json", json.dumps(sanitize(report), indent=2, allow_nan=False) + "\n")
    for trial, images, _ in audits:
        write(trial["trial_id"] + "_image_hashes.json", json.dumps(images, indent=2, allow_nan=False) + "\n")
    render_comparison(report, campaign, baseline_root, output / "01_all_four_speed_rate_traces.png")
    visual_metadata = {}
    for name, (payloads, proof) in visuals.items():
        directory = output / name
        directory.mkdir()
        for relative, raw in payloads.items():
            with (directory / relative).open("xb") as stream:
                stream.write(raw)
        visual_metadata[name] = proof
    columns = []
    for arm, rows in (("V3 ACK", report["baseline_trials"]), ("V4 brake=0", report["candidate_trials"])):
        for row in rows:
            qa = row["independent_qa"]
            rate = qa["speed_rate_qa"]["native_20hz"]["by_phase"]["all"]
            columns.append(f"| {arm} {row['trial_id']} | {row['owner_exit_code']} | {'PASS' if qa['raw_scalar_quality_clear'] else 'FAIL'} | {rate['minimum_mps2']:.6f} / {rate['maximum_mps2']:.6f} | {qa['final_driving']['goal_error_m']:.6f} |")
    footage = "\n\n".join(f"### V4 {name}\n\n![실제 원시 카메라와 경로의 가속 재생]({name}/whole_recording_accelerated.gif)\n\n[순항 PNG]({name}/02_measured_cruise.png) · [정지 유지 PNG]({name}/04_goal_dwell.png) · [원시 영상 출처]({name}/visual_provenance.json)" for name in visuals)
    write("README.md", "<!-- HH_260906 - Keep all four actual outcomes and distinguish scalar improvement from data admission. -->\n"
        "# 정상 브레이크 제거 V4: 기존 실패 2건과 반복 시험 비교\n\n"
        "Town07 직진의 같은 초기 조건에서 정상 브레이크 상한만 0.1→0으로 변경했습니다. 정상 조향, 비상 제동, 정지 확인 후 꼬리 구간의 전제동은 유지했습니다. 기존 V3 ACK 두 건과 실행된 V4 전부를 포함하며 실패 구간을 자르지 않았습니다. **학습 모델 주행·데이터셋 승인·실차 검증은 아닙니다.**\n\n"
        "| 조건 | 종료 코드 | 전체 속도 변화율·목표정지 QA | 20Hz 최솟값 / 최댓값 (m/s²) | 마지막 주행 목표 오차 (m) |\n|---|---:|---|---:|---:|\n"
        + "\n".join(columns) + "\n\n![모든 시도의 실제 속도 변화율](01_all_four_speed_rate_traces.png)\n\n"
        "그래프는 준비·주행·정지 꼬리와 모든 경계 샘플을 포함합니다. 패널별 Y축 범위가 다르며 기존 실패의 큰 감속도 그대로 표시했습니다. 물리20Hz, 카메라10Hz, 명목28.8km/h, 실제 상한30km/h, 목표1m·0.1m/s 이하2초 유지, decoder ±2.9 및 runtime +3/−6m/s² 기준은 변경하지 않았습니다.\n\n"
        "모든10개 실행 소스의 보관 바이트는 사전 검토 커밋과 실행 당시 HEAD 양쪽에 일치합니다. 두 번째 HEAD는 공개 문서 커밋 때문에 달라졌으므로 전체 커밋이 같다고 주장하지 않습니다. 사전 계획·첫 결과·두 번째 검토·실행의 기록된 시간과 원본 SHA를 검증했으며, 역사적 파일 생성 시점의 불변 증명으로 확대하지 않습니다.\n\n"
        "제어 API 보고 값과 서버 ACK는 물리 페달·토크 적용 시점의 증명이 아닙니다. 같은 경로 반복은 독립 경로 검증이 아니며, 전체 future XY/영상 품질의 후속 검사와 데이터 승인은 별도입니다. Low의 체크무늬 노면을 포함한 실제 영상은 미화하지 않았습니다.\n\n"
        + footage + "\n\n영상은 원본 카메라를 사용한1600×900 진단 합성 화면이며 Autoware/RViz 라이브 화면이 아닙니다. GIF는 명시된 간격의 가속 재생으로 카메라 FPS 측정값이 아닙니다.\n\n"
        "[독립 재계산 결과](summary.json) · [원본/공개 해시 출처](provenance.json) · [전체 공개 SHA256](SHA256SUMS)\n\n"
        "원본: `artifacts/training/2026-09-08/brake_free_goal_stop_v4`. 이전 V3: `artifacts/training/2026-09-08/acknowledged_control_v1`. 원본·학습/검증/테스트 데이터셋은 변경하지 않았습니다.\n\n"
        "재현: `python3 scripts/e2e/audit_carla_brake_free_goal_stop.py --campaign <V4 원본> --baseline-root <V3 town07_straight_calibration> --visual-root <V4 시각화 원본> --output <새 폴더>`. 기존 출력은 덮어쓰지 않습니다.\n")
    files = sorted(path for path in output.rglob("*") if path.is_file())
    provenance = {"schema": "portable_e2e.brake_free_goal_stop_publication.v1", "audit_source_sha256": report["audit_source_sha256"],
        "raw_campaign": campaign.resolve().relative_to(ROOT).as_posix(), "raw_baseline": baseline_root.resolve().relative_to(ROOT).as_posix(),
        "raw_visual_provenance": visual_metadata, "public_payload_sha256": {p.relative_to(output).as_posix(): base.sha(p) for p in files},
        "notice": "Summary is an independently recomputed redacted view; rendered PNG/GIF/provenance bytes are copied exactly. Raw source and image-ledger hashes remain available; original states and full camera corpus are private."}
    write("provenance.json", json.dumps(sanitize(provenance), indent=2, allow_nan=False) + "\n")
    write("SHA256SUMS", "".join(f"{base.sha(p)}  {p.relative_to(output).as_posix()}\n" for p in sorted(output.rglob("*")) if p.is_file()))
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--campaign", type=Path, required=True)
    parser.add_argument("--baseline-root", type=Path, required=True)
    parser.add_argument("--visual-root", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        report = publish(args.campaign, args.baseline_root, args.output, args.visual_root)
    except (base.EvidenceError, OSError, ValueError, KeyError, TypeError) as error:
        print(f"BRAKE_FREE_AUDIT_ERROR: {error}", file=sys.stderr)
        return 2
    print(json.dumps({"status": report["status"], "v4_attempts": report["candidate_attempt_count"], "v3_attempts": 2, "dataset_admission": False}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

