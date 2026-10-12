#!/usr/bin/env python3
"""HH_260906 - Independently audit all eight fixed-order full-route launch cases; retain failures and never admit data."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import io
import json
import math
import os
from pathlib import Path
import re
import subprocess

if __package__:
    from . import audit_carla_turn_low_goal_stop as turn
    from . import audit_carla_agent_initialization as initialization_audit
    from . import audit_carla_acknowledged_control as ack
    from . import audit_carla_brake_free_goal_stop as v4
    from . import audit_carla_comfortable_v3_trial as pilot
    from . import summarize_carla_goal_stop_trials as base
else:
    import audit_carla_turn_low_goal_stop as turn
    import audit_carla_agent_initialization as initialization_audit
    import audit_carla_acknowledged_control as ack
    import audit_carla_brake_free_goal_stop as v4
    import audit_carla_comfortable_v3_trial as pilot
    import summarize_carla_goal_stop_trials as base

ROOT = Path(__file__).resolve().parents[2]
COMMIT = "aa1abbbbef03accf14479fd8caec1e3492a548c5"
ORIGINAL_PLAN_SHA = "075051366ec76e1a96c83fa2f7f678acaa72bd122e89cd02ef37714892f19267"
ORIGINAL_DEADLINE = "2026-09-08T01:00:00Z"
RESUMED_DEADLINE = "2026-09-08T08:00:00Z"
SOURCES = turn.SOURCES
MAP, ROUTE_SHA = turn.MAP, turn.ROUTE_SHA
require, strict_equal, utc, same = base.require, v4.strict_equal, v4.utc, pilot.same
recorded_bytes = turn.recorded_bytes
EPS = 1e-6
# HH_260906 - These literal named revisions share every later control parameter; lower launch pedals do not move the .15 handoff origin.
PEDALS = {"turn_launch_012_v1": .12, "turn_launch_013_v1": .13, "turn_launch_014_v1": .14, "turn_launch_015_v1": .15}
ORDER = ("turn_launch_015_v1", "turn_launch_013_v1", "turn_launch_014_v1", "turn_launch_012_v1",
         "turn_launch_012_v1", "turn_launch_014_v1", "turn_launch_013_v1", "turn_launch_015_v1")
REVIEWED_SOURCES = {
    "scripts/e2e/collect_carla_vad_expert.py": "341e3479742a532d474ec151fe32b594d67046ad3a512291265c0bf4fb8cddad",
    "scripts/e2e/carla_goal_stop_profile.py": "b8a3bb8168e7b316a8dd30bc2a2bf711463f609b4deef73970dc0288a4ee5fb9",
    "scripts/e2e/carla_wall_timing.py": "2037b681ec75bfa85f1e34a7510766d6cd06dee20b0478773732f7fe0fafbf0a",
}
REVIEW_SOURCES = {
    "scripts/e2e/summarize_carla_goal_stop_trials.py": "890984ed44df9dacc5112a2c2d099652b8fd0490e41c6856aea9c216a4029032",
    "scripts/e2e/audit_carla_acknowledged_control.py": "9d6fccb6874a5cdfc85e8bf5d9229435e4ba6fe6850eb63a9e83488f39bcc5bd",
    "scripts/e2e/audit_carla_agent_initialization.py": "9e29bd0b82707cb850a5fe7e23f988c2b909dd9056fe59bb188d963588461c25",
}
PLAN_FIXED = {k: v for k, v in initialization_audit.PLAN_FIXED.items()
              if k not in ("first_output", "optional_second_output", "review_required_before_second_attempt")}
PLAN_FIXED.update(schema="portable_e2e.turn_launch_matrix_plan.v1", source_commit=COMMIT,
    profile="named_turn_launch_matrix_v1", maximum_total_attempts=8, common_post_handoff_initial_throttle=.15,
    post_handoff_ramp_per_second=.05, full_route_capture=True, launch_only_eight_second_probe=False,
    maximum_launch_handoff_seconds=8)
CONTINUATION_POLICY = {
    "automatic_retry": False, "timestamped_review_before_each_next_case": True, "all_declared_case_outcomes_retained": True,
    "stop_remaining_for_collision_lane_invasion_reverse_or_speed_bound": True,
    "stop_remaining_for_source_ack_initialization_or_lifecycle_failure": True,
    "scalar_acceleration_failure_or_launch_timeout_may_continue_after_review": True,
    "no_automatic_winner_selection": True, "no_training_admission": True,
}


def frozen_configuration(profile):
    """HH_260906 - Return a new literal configuration without mutating old profile constants."""
    require(isinstance(profile, str) and profile in PEDALS, "unknown named launch profile")
    return {**turn.FROZEN, "profile_id": profile, "launch_throttle": PEDALS[profile], "post_handoff_initial_throttle": .15}


def validated_profile(owner):
    """HH_260906 - Resolve only the full explicit named profile from an unambiguous CLI."""
    argv = owner["collector_argv"]
    require(isinstance(argv, list) and all(isinstance(v, str) for v in argv) and argv.count("--goal-stop-profile") == 1,
            "unique matrix profile CLI required")
    index = argv.index("--goal-stop-profile")
    require(index >= 2 and index + 1 < len(argv) and argv[index + 1] in PEDALS, "unsupported named launch profile")
    return argv[index + 1]


def validate_jpeg_payload(raw):
    """HH_260906 - Decode exact raw JPEG bytes solely for format/rig integrity, not semantic image approval."""
    from PIL import Image
    with Image.open(io.BytesIO(raw)) as decoded:
        require(decoded.format == "JPEG" and decoded.size == (640, 360) and decoded.mode == "RGB",
                "matrix JPEG format/rig dimensions differ")
        decoded.load()


def validate_cli(owner, root):
    """HH_260906 - Reuse only the unchanged option ABI after explicitly checking the new profile; historical source pins are never substituted."""
    profile = validated_profile(owner)
    argv = list(owner["collector_argv"])
    index = argv.index("--goal-stop-profile")
    argv[index + 1] = "turn_low_v1"
    initialization_audit.validate_cli({**owner, "collector_argv": argv}, root)
    return profile



def analyze_protocol(states, rows, profile):
    """HH_260906 - Reconstruct clocks, envelopes and normal-control bounds without importing the governor."""
    require(profile in PEDALS, "unknown matrix profile")
    launch_pedal = PEDALS[profile]
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
        check("governor_state_and_timers", gp.get("profile_id") == profile, row["frame"], "literal profile ID")
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
                  and gp.get("termination_reason") == f"{profile}_emergency_override", frame, "hazard was not observed then aborted")
        if speed > 4.3 or vx > 4.3:
            speed_violations.append({"frame": frame, "timestamp": t, "planar_speed_mps": speed, "vx_mps": vx})
            latched_failure = latched_failure or f"{profile}_actual_speed_exceeded"
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
                latched_failure = latched_failure or f"{profile}_launch_timeout"
        release_distance = 0.75 + 6.93 * speed
        if active and handoff is not None and coast is None and row["remaining_route_m"] <= release_distance:
            if not 2.8 <= speed <= 3.2:
                latched_failure = latched_failure or f"{profile}_coast_entry_speed_outside_band"
            elif latched_failure is None:
                coast, transition = t, "normal_pid_to_zero_pedal_coast"
        coast_elapsed = None if coast is None else last_active - coast
        if row["phase"] == "driving":
            hold_ticks = hold_ticks + 1 if row["stopped_in_goal"] and was_stopped else 0
            was_stopped = row["stopped_in_goal"]
        complete = hold_ticks >= 40
        if active and coast_elapsed is not None and coast_elapsed >= 45.0 and not complete:
            latched_failure = latched_failure or f"{profile}_coast_timeout"
        envelope = min(4.0, math.sqrt(3.0396464855480536 ** 2 + 1.2 * max(row["remaining_route_m"] - 35.0, 0.0)))
        target = 0.0 if not active or coast is not None or complete else min(envelope, previous_target + 0.05)
        handoff_elapsed = None if handoff is None else last_active - handoff
        cap = launch_pedal if handoff_elapsed is None else min(0.4, 0.15 + 0.05 * (handoff_elapsed + 0.05))
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
            latched_failure = latched_failure or f"{profile}_emergency_override"
        pending = abort is None and latched_failure == f"{profile}_emergency_override"
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
            pedal_ok = same(command["throttle"], launch_pedal) and same(command["brake"], 0.0)
        else:
            pedal_ok = command["throttle"] <= cap + EPS and command["brake"] == 0.0 and min(command["throttle"], command["brake"]) <= EPS
        check("normal_pedal_protocol", pedal_ok, frame, "command violates phase or normal pedal cap")
        previous_pending, emergency_count = pending, count
        in_cruise = row["phase"] == "driving" and 3.8 <= speed <= 4.2
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


def validate_profile(manifest, route):
    """HH_260906 - Low-speed stability and downstream alignment metadata must never become a 30 km/h or native-pose claim."""
    capture, result = manifest["capture_contract"], manifest["result"]
    config = capture["goal_stop_profile"]
    expected = frozen_configuration(config.get("profile_id"))
    require(all(strict_equal(config.get(k), v) for k, v in expected.items()), "turn profile differs from literal frozen configuration")
    require(capture["target_speed_kmh"] == 14.4 and manifest["coordinate_contract"]["wheelbase_m"] == 2.85
            and result.get("training_data_approved") is False and result.get("development_only") is True
            and result.get("qualification_30_kph") == config.get("qualification_30_kph") == "NOT_CLAIMED", "turn speed/admission claim differs")
    require(route["town"] == MAP and route["scenario"] == "left" and route["weather"] == "ClearNoon"
            and route.get("coordinate_alignment") is None and route["route_length_m"] == turn.FROZEN["route_length_m"], "turn route or native alignment differs")
    require(strict_equal(config.get("downstream_map_alignment_metadata_only"), {
        "carla_to_autoware_map_translation_m": [0., 0., -15.], "applied_to_native_spawn_goal_state_or_sensor_tf": False,
        "notice": "Separate downstream Autoware map alignment, not a sensor extrinsic or learned-model TF change."}), "turn alignment metadata altered")
    require(config.get("future_dataset_split_if_separately_admitted") == "train"
            and "historical route-shape checks" in config.get("route_metadata_qualification_notice", ""), "turn historical route/split notice missing")
    effective = config.get("effective_control")
    if effective is not None:
        require(effective["normal_brake_cap"] == 0 and effective["normal_throttle_cap"] == .4
                and effective["basic_agent_emergency_brake_modified"] is False and effective["lateral_control_modified"] is False
                and effective["emergency_return_control_modified"] is False
                and effective["unchanged_basic_agent_emergency_brake"] == .5, "turn emergency or normal control contract changed")
    transport = capture["control_transport"]
    expected = {"schema": "carla.acknowledged_control_transport.v1", "mode": "acknowledged_batch",
        "single_actor_single_response": True, "implicit_tick": False, "async_fallback_allowed": False,
        "receipt_journal": "control_receipts.jsonl", "receipt_persisted_before_next_tick": True,
        "comparison_absolute_tolerance": 1e-6, "automatic_gear_equality_required": False,
        "label_rewrite_or_lookback_pass_allowed": False, "physical_actuation_proven": False,
        "native_acceleration_definition_changed": False, "kinematics_source": "tick-exact immutable WorldSnapshot.find(owned_actor_id)"}
    require(all(strict_equal(transport.get(k), v) for k, v in expected.items()), "turn acknowledged-control contract changed")
    return transport


def verify_owner_sources(root, ledger, *, finish_before_utc=ORIGINAL_DEADLINE):
    """HH_260906 - Bind the exact new source revision and Epic C-track lifecycle, never the current worktree or V4 pins."""
    read = lambda name: pilot.read_json(root, name, ledger)
    plan, owner, started = read("owner_plan.json"), read("owner_result.json"), read("owner_started.json")
    require(plan.get("source_bytes_archived") is plan.get("bounds_source_bytes_archived") is True
            and plan.get("wall_timing_enabled") is plan.get("wall_timing_source_bytes_archived") is True
            and plan.get("wall_timing_schema") == "carla.expert_wall_timing.v1", "turn eleven-source archive contract missing")
    require(set(plan["source_sha256"]) == SOURCES and set(owner["source_checks"]) == SOURCES
            and owner.get("source_bytes_unchanged_and_archived") is True
            and all(v is True for v in owner["source_checks"].values()), "turn source postcheck failed")
    require({p.relative_to(root / "provenance").as_posix() for p in (root / "provenance").rglob("*")
             if p.is_file() or p.is_symlink()} == SOURCES, "missing or unexpected turn source archive")
    for name, expected in plan["source_sha256"].items():
        raw = pilot.checked_bytes(root, "provenance/" + name, ledger)
        require(hashlib.sha256(raw).hexdigest() == expected and raw == recorded_bytes(COMMIT, name)
                and raw == recorded_bytes(plan["source_head_commit"], name), "turn reviewed/owner/archive source mismatch")
        if name in REVIEWED_SOURCES:
            require(expected == REVIEWED_SOURCES[name], "unreviewed turn source SHA")
    require(plan.get("host") == "127.0.0.1" and plan.get("map") == started.get("map") == MAP
            and plan.get("quality") == started.get("quality") == "Epic"
            and plan.get("capture_mode") == owner.get("capture_mode") == "expert"
            and plan.get("worker_path") == "scripts/e2e/collect_carla_vad_expert.py"
            and plan.get("route_sha256") == ROUTE_SHA, "turn endpoint/map/quality/input contract differs")
    require(all(item.get(k) is False for item in (plan, owner) for k in ("learned_model_control", "vehicle_control_approved")),
            "turn remains expert-only and not approved")
    require(plan.get("server_extra_options") == ["-RenderOffScreen", "-nosound"]
            and plan.get("collector_wall_timeout_sec") == 900 and plan.get("finish_before_utc") == finish_before_utc,
            "turn timing/render contract differs")
    validate_cli(plan, root)
    pid = base.integer(started["server_pid"])
    require(pid > 1 and started["server_pgid"] == pid and started["port"] == plan["port"], "turn owned process identity mismatch")
    log, times = pilot.checked_bytes(root, "server.log", ledger), []
    for stage in ("ready", "after_capture", "stopped"):
        evidence = read("lifecycle/" + stage + ".json")
        require(evidence.get("status") == "PASS" and evidence.get("stage") == stage and evidence.get("read_only") is True
                and evidence.get("owner_pid") == evidence.get("owner_pgid") == pid
                and evidence.get("generation_id") == f"expert_{pid}" and evidence.get("host") == plan["host"]
                and evidence.get("port") == plan["port"] and evidence.get("expected_map") == MAP
                and evidence.get("mode") == ("stopped" if stage == "stopped" else "running"), "turn lifecycle contract differs")
        require((evidence.get("port_released") is True and evidence.get("owner_process_state") is None) if stage == "stopped"
                else evidence.get("active_map_basename") == MAP, "turn running map or stopped proof differs")
        prefix = evidence["server_log"]
        require(type(prefix["size_bytes"]) is int and 0 < prefix["size_bytes"] <= len(log)
                and hashlib.sha256(log[:prefix["size_bytes"]]).hexdigest() == prefix["sha256"], "turn lifecycle log prefix mismatch")
        times.append(utc(evidence["checked_at"]))
    require(utc(plan["planned_at_utc"]) < times[0] <= utc(started["started_at_utc"]) < times[1] < times[2]
            <= utc(owner["completed_at_utc"]) <= utc(plan["finish_before_utc"]), "turn lifecycle times invalid")
    return plan, owner, started, times


def audit_trial(root, *, finish_before_utc=ORIGINAL_DEADLINE):
    """HH_260906 - Preserve failed, aborted and no-tail trials; protocol success does not waive raw scalar or future XY quality."""
    root, ledger = Path(root).resolve(), {}
    require((root / "owner_result.json").is_file(), "INCOMPLETE: turn owner result missing")
    plan, owner, started, times = verify_owner_sources(root, ledger, finish_before_utc=finish_before_utc)
    result, timeline = base.summarize_trial(root, None)
    for entry in result["source_manifest"]:
        pilot.checked_bytes(root, entry["path"], ledger)
    directories = [name for name in ("episode", "episode.partial") if (root / name).exists()]
    require(len(directories) <= 1, "ambiguous turn final/partial evidence")
    states, cameras, receipts, images, transport, manifest = [], [], [], [], {}, None
    if directories:
        prefix = directories[0] + "/"
        read = lambda name, lines=False: pilot.read_json(root, prefix + name, ledger, lines=lines)
        manifest, route, states, cameras = read("manifest.json"), read("route.json"), read("states.jsonl", True), read("camera_frames.jsonl", True)
        transport = validate_profile(manifest, route)
        require(manifest["capture_contract"]["goal_stop_profile"]["profile_id"] == validated_profile(plan),
                "matrix manifest profile differs from the executed CLI")
        require(utc(started["started_at_utc"]) <= utc(manifest["created_at"]) <= times[1], "turn manifest created outside owned interval")
        if "runtime" in manifest:
            require(manifest["runtime"]["town"].split("/")[-1] == MAP and manifest["runtime"]["weather"] == "ClearNoon"
                    and manifest["runtime"]["vehicle_type"] == "vehicle.toyota.prius", "turn measured runtime identity changed")
        for key, digest in {"route_sha256": ROUTE_SHA,
            "mapping_sha256": "9aaff2befed7ad12376b2e04bbdd51bd1808a3bafe39d87a6f6b241dbcca3136",
            "calibration_sha256": "5022cd1de5b48e9c824b6f2f8c59991fa665eeaf7a7bafd084c88acdb65e4bea"}.items():
            require(manifest["provenance"].get(key) == digest, "turn rig/route provenance differs")
        if (root / prefix / "control_receipts.jsonl").exists():
            receipts = read("control_receipts.jsonl", True)
        else:
            require(not states and owner["exit_code"] != 0, "observed turn capture has no receipt journal")
        # HH_260906 - Preserve timing evidence hashes here; its independent interval audit is a separate diagnostic.
        for name in ("wall_timing.jsonl", "wall_timing_summary.json", "wall_timing_recovery.jsonl"):
            if (root / prefix / name).exists():
                pilot.checked_bytes(root, prefix + name, ledger)
    measured = ack.analyze_transport(states, receipts, transport.get("bootstrap_observation"))
    measured["camera_alignment"] = ack.analyze_camera_alignment(states, cameras)
    measured["flags"]["complete_camera_frame_alignment"] = measured["camera_alignment"]["status"] == "PASS"
    measured["failed_flags"] = [k for k, v in measured["flags"].items() if not v]
    measured["status"] = "FAIL" if measured["failed_flags"] else "PASS"
    declared = manifest["result"].get("control_transport", {}) if manifest else {}
    if receipts or states or declared:
        require(declared.get("receipt_journal_sha256") == ledger[prefix + "control_receipts.jsonl"]["sha256"]
                and declared.get("receipt_journal_error") is None and declared.get("physical_actuation_proven") is False,
                "turn receipt finalization differs")
        for field, key in (("command_receipt_count", "receipt_count"), ("acknowledged_command_count", "acknowledged_count"),
            ("failed_command_count", "failed_receipt_count"), ("control_alignment_failure_count", "alignment_failure_count")):
            require(type(declared.get(field)) is int and declared[field] == measured[key], "turn receipt denominator differs")
    profile = validated_profile(plan)
    protocol = analyze_protocol(states, timeline["native_states"], profile) if timeline else analyze_protocol([], [], profile)
    initialization = initialization_audit.analyze_initialization(manifest["capture_contract"].get("agent_initialization") if manifest else None,
        states, receipts, transport.get("bootstrap_observation"))
    derived = {}
    if timeline:
        bounds = manifest["capture_contract"]["goal_stop_profile"]["bounds"]
        for offset in (0, 1):
            derived[str(offset)] = base.intervals(timeline["native_states"][offset::2], .1, bounds)[0]
    for camera in cameras:
        for name in base.CAMERAS:
            path = prefix + camera["images"][name]
            raw = pilot.checked_bytes(root, path, {})
            validate_jpeg_payload(raw)
            images.append({"path": path, "sha256": hashlib.sha256(raw).hexdigest(), "size_bytes": len(raw)})
    require(len({x["path"] for x in images}) == len(images), "duplicate turn image references")
    images.sort(key=lambda x: x["path"])
    normal = [s for s in states if s["capture_phase"] == "driving" and not s["goal_stop"].get("termination_reason")
              and not s["goal_stop"].get("emergency_failure_pending_next_tick")]
    result.update({"status": "AUDITED_NOT_ADMITTED", "transport_protocol": measured, "pilot_protocol": protocol,
        "matrix_profile": profile, "initialization_protocol": initialization, "alternate_10_hz_offsets_diagnostic_only": derived,
        "normal_driving_nonzero_brake_count": sum(s["next_control"]["brake"] != 0 for s in normal),
        "normal_driving_command_count": len(normal), "source_head_commit": plan["source_head_commit"],
        "reviewed_execution_commit": COMMIT, "all_eleven_archives_match_owner_and_reviewed_commits": True,
        "archived_source_sha256": plan["source_sha256"], "comparison_configuration": ack.comparison_configuration(manifest) if manifest and "runtime" in manifest else None,
        "image_byte_integrity": {"file_count": len(images), "total_size_bytes": sum(x["size_bytes"] for x in images),
            "canonical_sorted_file_ledger_sha256": hashlib.sha256(json.dumps(images, sort_keys=True, separators=(",", ":")).encode()).hexdigest(),
            "all_images_decoded": True, "content_or_visual_quality_approved": False}, "source_manifest": [{"path": n, **v} for n, v in sorted(ledger.items())],
        "qualification_30_kph": "NOT_CLAIMED", "low_speed_stability_band_mps": [3.8, 4.2],
        "training_data_approved": False, "dataset_admission": False, "full_future_xy_admission": False,
        "physical_actuation_proven": False, "failed_before_first_observation": not states,
        "wall_timing_independently_reconstructed": False, "campaign_preregistration_checked_by_trial_only": False})
    for entry in result["source_manifest"] + images:
        require(base.sha(root / entry["path"]) == entry["sha256"], "turn raw input changed during audit")
    if "bounds_source_proof" in result:
        base.recheck_bounds_source_archive(root, result["bounds_source_proof"])
    return result, images, timeline


def expected_cases():
    """HH_260906 - Enumerate all eight predeclared outputs independently, including cases that never run."""
    counts, cases = {}, []
    for sequence, profile in enumerate(ORDER, 1):
        counts[profile] = counts.get(profile, 0) + 1
        replicate = counts[profile]
        cases.append({"sequence": sequence, "case_id": f"{sequence:02d}_{profile}_r{replicate}",
            "profile": profile, "replicate": replicate, "output": f"c_track_left/{profile}/run_{replicate:03d}"})
    return cases


def validate_plan_document(plan):
    """HH_260906 - Bind the fixed matrix, unchanged source/rig and whole-episode controls; no model or worker is imported."""
    extras = {"comment", "declared_at_utc", "source_hashes", "common_capture_flags", "historical_lateral_configuration",
        "comparison_notice", "expected_risks", "historical_before_bootstrap_reference", "initialization_contract",
        "review_measurements", "cases", "continuation_policy", "initialization_reference"}
    require(isinstance(plan, dict) and set(plan) == set(PLAN_FIXED) | extras, "matrix prospective fields differ")
    require(all(strict_equal(plan[k], value) for k, value in PLAN_FIXED.items()), "matrix prospective fixed settings differ")
    require(strict_equal(plan["continuation_policy"], CONTINUATION_POLICY)
            and strict_equal(plan["initialization_contract"], initialization_audit.INIT_CONTRACT), "matrix continuation/initialization contract differs")
    require(utc(plan["declared_at_utc"]) < utc(ORIGINAL_DEADLINE), "original prospective declaration too late")
    require(set(plan["source_hashes"]) == SOURCES, "matrix exact eleven source pins missing")
    for name, digest in plan["source_hashes"].items():
        require(hashlib.sha256(recorded_bytes(COMMIT, name)).hexdigest() == digest, "matrix source differs from frozen commit")
    require(isinstance(plan["cases"], list) and len(plan["cases"]) == 8, "all eight prospective cases required")
    common = plan["common_capture_flags"]
    require(isinstance(common, list) and all(isinstance(v, str) for v in common) and "--goal-stop-profile" not in common,
            "common flags must leave only the named profile to each case")
    for actual, expected in zip(plan["cases"], expected_cases()):
        require(set(actual) == set(expected) | {"capture_flags", "frozen_configuration"}
                and all(strict_equal(actual[k], v) for k, v in expected.items()), "matrix order, replicate or output differs")
        require(strict_equal(actual["frozen_configuration"], frozen_configuration(expected["profile"]))
                and actual["capture_flags"] == common + ["--goal-stop-profile", expected["profile"]],
                "matrix launch or common handoff parameters differ")
        validate_cli({"port": 2100, "route_path": "route.json", "collector_argv": [str(ROOT / "episode"), "route.json",
            "--host", "127.0.0.1", "--port", "2100", *actual["capture_flags"]]}, ROOT)
    ledger = {}
    old_lateral = pilot.read_json(ROOT, plan["historical_lateral_baseline_path"], ledger)
    require(ledger[plan["historical_lateral_baseline_path"]]["sha256"] == plan["historical_lateral_baseline_sha256"]
            and strict_equal(old_lateral["capture_contract"]["basic_agent_control"], plan["historical_lateral_configuration"]),
            "historical compact lateral settings differ")
    for key, root, commit, fixed in (
        ("historical_before_bootstrap_reference", initialization_audit.HISTORICAL_ROOT, turn.COMMIT, initialization_audit.HISTORICAL_SHA),
        ("initialization_reference", "artifacts/training/2026-09-08/agent_initialization_v1", initialization_audit.COMMIT, {
            "pilot_plan.json": "bfdaa5c1c16cbb60e8cc9e6eb4a8a07439e3db799e9ab8447ef94d013c5815b8",
            "c_track_left/run_001/episode.partial/states.jsonl": "7e763dc564292afb0e35500cd78fbd7f236a213e6240bf763c92fbaedec83e9a",
            "c_track_left/run_001/episode.partial/control_receipts.jsonl": "573e6f72e83a1dd74d129b051792d65568a34ed6e081dc84d929ff8b8a5db88d",
            "c_track_left/run_001/episode.partial/manifest.json": "5618ab37ee4802f878ad3b1b3ed9f73b5f4f76eb7eaeb904de5277f7b02d9eb4"})):
        reference = plan[key]
        require(reference["campaign"] == root and reference["source_commit"] == commit and reference["source_sha256"] == fixed,
                "matrix historical context identity differs")
        for name, digest in fixed.items():
            require(hashlib.sha256(pilot.checked_bytes(ROOT, root + "/" + name, ledger)).hexdigest() == digest,
                    "matrix historical input changed")
    return ledger


def validate_resume_document(document, plan, campaign, ledger):
    """HH_260906 - Permit only the explicit afternoon deadline extension; never modify the original expired plan or finished outcomes."""
    fixed = {"schema": "portable_e2e.turn_launch_resume_authorization.v1", "original_plan_sha256": ORIGINAL_PLAN_SHA,
        "original_finish_before_utc": ORIGINAL_DEADLINE, "resumed_finish_before_utc": RESUMED_DEADLINE,
        "allowed_case_sequences": [3, 4, 5, 6, 7, 8], "frozen_source_commit": COMMIT,
        "only_changed_execution_bound": "finish_before_utc for the six not-yet-started cases",
        "original_order_and_two_replicates_per_profile_unchanged": True, "new_attempts_added": 0,
        "automatic_retry": False, "precase_review_required": True, "no_training_admission": True,
        "old_cases_reclassified": False}
    extras = {"comment", "declared_at_utc", "user_request", "completed_cases", "source_hashes", "interruption_notice"}
    require(isinstance(document, dict) and set(document) == set(fixed) | extras
            and all(strict_equal(document[k], v) for k, v in fixed.items()), "resume scope or deadline changed")
    require(all(isinstance(document[k], str) and document[k] for k in ("comment", "user_request", "interruption_notice")),
            "resume authorization or interruption disclosure missing")
    require(document["source_hashes"] == plan["source_hashes"], "resume may not change execution sources")
    expected = expected_cases()[:2]
    require(set(document["completed_cases"]) == {case["case_id"] for case in expected}, "resume does not bind exactly the completed first two cases")
    declared = utc(document["declared_at_utc"])
    require(utc(ORIGINAL_DEADLINE) < declared < utc(RESUMED_DEADLINE), "resume must be after expired boundary and before new deadline")
    for case in expected:
        name = case["output"] + "/owner_result.json"
        owner = pilot.read_json(campaign, name, ledger)
        proof = document["completed_cases"][case["case_id"]]
        require(set(proof) == {"output", "owner_result_sha256"} and proof["output"] == case["output"]
                and proof["owner_result_sha256"] == ledger[name]["sha256"], "resume completed outcome SHA differs")
        require(utc(owner["completed_at_utc"]) < utc(ORIGINAL_DEADLINE) < declared, "resume cannot reclassify a late or unfinished previous case")
    return document


def resolve_review_sources(pins):
    """HH_260906 - Resolve the recorded review implementation by exact bytes, never by assuming the present worktree executed historically."""
    require(pins == REVIEW_SOURCES, "unexpected preliminary review implementation pins")
    proof = {}
    for name, digest in pins.items():
        if (ROOT / name).is_file() and base.sha(ROOT / name) == digest:
            proof[name] = {"sha256": digest, "resolution": "current_hash_match", "historical_execution_proven": False}
            continue
        env = dict(os.environ, GIT_NO_LAZY_FETCH="1", GIT_ALLOW_PROTOCOL="", GIT_TERMINAL_PROMPT="0")
        commits = subprocess.run(["git", "-c", "protocol.allow=never", "log", "--all", "-100", "--format=%H", "--", name],
            cwd=ROOT, env=env, capture_output=True, timeout=10, check=False)
        require(commits.returncode == 0, "review source history unavailable; no fetch attempted")
        for commit in commits.stdout.decode().splitlines():
            require(re.fullmatch(r"[a-f0-9]{40}", commit) is not None, "invalid historical source commit")
            blob = subprocess.run(["git", "-c", "protocol.allow=never", "show", commit + ":" + name], cwd=ROOT,
                env=env, capture_output=True, timeout=10, check=False)
            if blob.returncode == 0 and hashlib.sha256(blob.stdout).hexdigest() == digest:
                proof[name] = {"sha256": digest, "resolution": "offline_git_blob", "commit": commit, "historical_execution_proven": False}
                break
        require(name in proof, "exact preliminary review source unavailable locally; no fetch attempted")
    return proof


def continuation_observations(trial, states):
    """HH_260906 - Record raw reverse-related evidence without inventing a retroactive reverse-speed threshold."""
    minimum_vx = min((base.number(s["vx"]) for s in states), default=None)
    controls = [ack.full_control(s[key]) for s in states for key in ("current_control", "next_control")]
    reverse = sum(c["reverse"] for c in controls)
    manual = sum(c["manual_gear_shift"] for c in controls)
    events = trial.get("independent_qa", {}).get("event_counts", {"collision": None, "lane_invasion": None})
    established_failure = bool(not states or events["collision"] is None or events["lane_invasion"] is None
        or events["collision"] or events["lane_invasion"] or reverse or manual
        or trial["transport_protocol"]["status"] != "PASS" or trial["initialization_protocol"]["status"] != "PASS"
        or not trial["pilot_protocol"]["flags"]["actual_planar_and_vx_maximum"])
    return {"minimum_recorded_vx_mps": minimum_vx, "reverse_control_observation_count": reverse,
        "manual_gear_control_observation_count": manual, "collision_events": events["collision"],
        "lane_invasion_events": events["lane_invasion"], "established_continuation_stop_condition": established_failure,
        "numerical_reverse_threshold_preregistered": False,
        "notice": "The original plan names reverse but gives no numeric vx threshold. Tiny negative vx alone is not a proven reverse event. A reviewer's later -0.1 m/s sanity judgment is not retroactive preregistration or an admission gate."}


def validate_review(review, previous, previous_case, next_case, previous_root, next_owner_plan, plan_sha, resume_sha=None):
    """HH_260906 - Recompute every declared review measurement and raw binding; a preliminary review is not the final matrix audit."""
    qa = previous["independent_qa"]
    native = qa["speed_rate_qa"]["native_20hz"]["by_phase"]["all"]
    camera = qa["speed_rate_qa"]["camera_10hz"]["by_phase"]["all"]
    prefix = "episode.partial" if (previous_root / "episode.partial").is_dir() else "episode"
    names = {"owner_plan.json", "owner_result.json", "lifecycle/stopped.json"} | {
        prefix + "/" + n for n in ("manifest.json", "states.jsonl", "control_receipts.jsonl", "camera_frames.jsonl")}
    fixed = {"schema": "portable_e2e.turn_launch_case_continuation_review.v1", "completed_case_id": previous_case["case_id"],
        "completed_case_sequence": previous_case["sequence"], "next_case_id": next_case["case_id"],
        "campaign_plan_sha256": plan_sha, "exact_eleven_source_archive_postcheck": True, "owned_lifecycle": True,
        "ack_status": previous["transport_protocol"]["status"], "initialization_status": previous["initialization_protocol"]["status"],
        "camera_alignment_status": previous["transport_protocol"]["camera_alignment"]["status"],
        "native_states": previous["initialization_protocol"]["raw_state_count"],
        "camera_anchors": sum(v["camera_anchors"] for v in qa["phase_counts"].values()),
        "ack_receipts": previous["transport_protocol"]["receipt_count"], "scalar_failed_flags": qa["failed_flags"],
        "maximum_native_acceleration_mps2": native["maximum_mps2"], "minimum_native_acceleration_mps2": native["minimum_mps2"],
        "maximum_camera_acceleration_mps2": camera["maximum_mps2"], "maximum_speed_kmh": qa["maximum_measured_speed_kmh"],
        "goal_error_m": qa["final_driving"]["goal_error_m"] if qa["final_driving"] else None,
        "initialization_measurements": previous["initialization_protocol"]["measurements"],
        "full_matrix_governor_audit_complete": False, "jpeg_payload_sha_audit_complete": False,
        "training_data_approved": False, "automatic_retry": False}
    extras = {"comment", "reviewed_at_utc", "review_source_sha256", "raw_pins", "next_predeclared_case_may_start", "reason"}
    if resume_sha is not None:
        fixed["resume_authorization_sha256"] = resume_sha
    if "reverse_diagnostic" in review:
        extras.add("reverse_diagnostic")
        diagnostic, measured = review["reverse_diagnostic"], previous["continuation_observations"]
        expected_reverse = {"minimum_native_vx_mps": measured["minimum_recorded_vx_mps"],
            "reverse_control_count": measured["reverse_control_observation_count"],
            "manual_gear_control_count": measured["manual_gear_control_observation_count"]}
        require(set(diagnostic) == set(expected_reverse) | {"notice"}
                and all(strict_equal(diagnostic[k], v) for k, v in expected_reverse.items())
                and isinstance(diagnostic["notice"], str) and diagnostic["notice"], "review reverse diagnostic contradicts raw controls")
    require(isinstance(review, dict) and set(review) == set(fixed) | extras
            and all(strict_equal(review[k], value) for k, value in fixed.items()), "preliminary review contradicts recomputed evidence")
    require(all(isinstance(review[k], str) and review[k] for k in ("comment", "reason")), "review rationale missing")
    require(set(review["raw_pins"]) == names and all(base.sha(previous_root / n) == review["raw_pins"][n] for n in names),
            "review raw input hashes differ")
    source_proof = resolve_review_sources(review["review_source_sha256"])
    owner = pilot.read_json(previous_root, "owner_result.json", {})
    require(utc(owner["completed_at_utc"]) <= utc(review["reviewed_at_utc"])
            < utc(next_owner_plan["planned_at_utc"]), "review does not intervene before next case planning")
    require(review["next_predeclared_case_may_start"] is True
            and not previous["continuation_observations"]["established_continuation_stop_condition"],
            "next case started despite a recorded stop condition or denied review")
    return {"review": review, "source_identity_proof": source_proof, "independent_measurement_binding": "PASS",
            "historical_review_was_full_matrix_audit": False}


def handoff_diagnostic(trial, timeline):
    """HH_260906 - Retain launch, handoff-prefix and later interval denominators without selecting only favourable phases."""
    transitions = trial["pilot_protocol"]["transitions"]
    handoffs = [row for row in transitions if row["transition"] == "launch_to_normal_pid"]
    require(len(handoffs) <= 1, "multiple launch handoffs")
    rows, intervals = timeline["native_states"], timeline["native_intervals"]
    handoff = handoffs[0] if handoffs else None
    handoff_elapsed = next((r["elapsed_s"] for r in rows if handoff and r["frame"] == handoff["frame"]), None)
    def summarize(values):
        return {"interval_count": len(values), "minimum_mps2": min((v["speed_rate_mps2"] for v in values), default=None),
            "maximum_mps2": max((v["speed_rate_mps2"] for v in values), default=None),
            "decoder_violations": [v for v in values if abs(v["speed_rate_mps2"]) > 2.9 + 1e-9],
            "runtime_violations": [v for v in values if v["speed_rate_mps2"] > 3 + 1e-9 or v["speed_rate_mps2"] < -6 - 1e-9]}
    launch = [v for v in intervals if v["to_phase"] == "driving" and (handoff is None or v["to_frame"] <= handoff["frame"])]
    later = [v for v in intervals if handoff and v["to_phase"] == "driving" and v["to_frame"] > handoff["frame"]]
    prefix = [] if handoff is None else [v for v in intervals if handoff_elapsed - .05 - EPS <= v["time_from_capture_start_s"] <= handoff_elapsed + .5 + EPS]
    index = next((i for i, row in enumerate(rows) if handoff and row["frame"] == handoff["frame"]), None)
    return {"all_native_intervals": summarize(intervals), "launch_through_handoff_observation": summarize(launch),
        "later_driving_intervals": summarize(later), "handoff_minus_one_tick_plus_half_second_diagnostic": summarize(prefix),
        "handoff": handoff, "handoff_boundary_rows": [] if index is None else rows[max(0, index - 1):index + 2],
        "observed_command_transition_is_physical_timing_proof": False,
        "all_native_interval_count": len(intervals), "phase_partitions_replace_full_quality": False}


def audit_campaign(campaign):
    """HH_260906 - Audit every declared case, chronological review and explicit resumed boundary; unrun cases stay visible."""
    campaign, ledger = Path(campaign).resolve(), {}
    source_before = {Path(p).resolve().relative_to(ROOT).as_posix(): base.sha(Path(p)) for p in
        (__file__, turn.__file__, initialization_audit.__file__, ack.__file__, v4.__file__, pilot.__file__, base.__file__)}
    plan = pilot.read_json(campaign, "pilot_plan.json", ledger)
    require(ledger["pilot_plan.json"]["sha256"] == ORIGINAL_PLAN_SHA, "original approved matrix plan bytes changed")
    historical = validate_plan_document(plan)
    resume = None
    if (campaign / "resume_authorization_v1.json").exists():
        resume = validate_resume_document(pilot.read_json(campaign, "resume_authorization_v1.json", ledger), plan, campaign, ledger)
    cases = expected_cases()
    parent = campaign / "c_track_left"
    known = {case["output"] for case in cases}
    found = {p.relative_to(campaign).as_posix() for p in parent.glob("*/*")} if parent.exists() else set()
    require(found <= known and all((campaign / name).is_dir() and not (campaign / name).is_symlink() for name in found),
            "unknown, excess, symlink or non-directory matrix attempts")
    if parent.exists():
        require(all(p.is_dir() and not p.is_symlink() and p.name in PEDALS for p in parent.iterdir()), "unexpected matrix profile directory")
    indices = [case["sequence"] for case in cases if case["output"] in found]
    require(indices == list(range(1, len(indices) + 1)), "matrix omitted, reordered or repeated a prospective case")
    results, audits, reviews, owner_plans = [], [], [], {}
    for case in cases:
        root = campaign / case["output"]
        if case["output"] not in found:
            results.append({**case, "status": "NOT_RUN", "dataset_admission": False})
            continue
        expected_deadline = ORIGINAL_DEADLINE if case["sequence"] <= 2 else RESUMED_DEADLINE
        require(case["sequence"] <= 2 or resume is not None, "later case requires explicit resumed authorization")
        if not (root / "owner_plan.json").exists():
            require(case["sequence"] == len(indices), "next case exists after incomplete predecessor planning")
            results.append({**case, "status": "INCOMPLETE", "dataset_admission": False})
            continue
        owner_plan = pilot.read_json(campaign, case["output"] + "/owner_plan.json", ledger)
        owner_plans[case["sequence"]] = owner_plan
        prospective_case = plan["cases"][case["sequence"] - 1]
        require(owner_plan["source_sha256"] == plan["source_hashes"] and owner_plan["finish_before_utc"] == expected_deadline
                and Path(owner_plan["route_path"]).resolve() == (ROOT / plan["route_path"]).resolve(), "matrix owner source/route/deadline differs")
        require(owner_plan["collector_argv"] == [str(root / "episode"), owner_plan["route_path"], "--host", "127.0.0.1",
            "--port", str(owner_plan["port"]), *prospective_case["capture_flags"]], "executed case CLI differs from original plan")
        require(utc(plan["declared_at_utc"]) < utc(owner_plan["planned_at_utc"]) < utc(expected_deadline), "case planning outside prospective interval")
        if case["sequence"] > 2:
            require(utc(resume["declared_at_utc"]) < utc(owner_plan["planned_at_utc"]), "resumed case was planned before authorization")
        if not (root / "owner_result.json").exists():
            require(case["sequence"] == len(indices), "next case started before predecessor was finalized")
            results.append({**case, "status": "INCOMPLETE", "dataset_admission": False})
            continue
        result = audit_trial(root, finish_before_utc=expected_deadline)
        trial, images, timeline = result
        prefix = "episode.partial" if (root / "episode.partial").exists() else "episode"
        states = pilot.read_json(root, prefix + "/states.jsonl", {}, lines=True) if (root / prefix / "states.jsonl").exists() else []
        trial["continuation_observations"] = continuation_observations(trial, states)
        trial["handoff_diagnostic"] = handoff_diagnostic(trial, timeline) if timeline else None
        results.append({**case, "status": "FINALIZED", "owner_exit_code": trial["owner_exit_code"],
            "independent_scalar_quality_clear": trial.get("independent_qa", {}).get("raw_scalar_quality_clear", False),
            "pilot_protocol_clear": trial["pilot_protocol"]["all_checks_pass"], "audit": trial, "dataset_admission": False})
        audits.append((root, result))
    for index in range(1, len(indices)):
        previous, next_case = cases[index - 1], cases[index]
        require(results[index - 1]["status"] == "FINALIZED" and index + 1 in owner_plans, "started matrix case lacks finalized preceding evidence")
        name = f"reviews/after_case_{index:02d}.json"
        review = pilot.read_json(campaign, name, ledger)
        resume_sha = ledger["resume_authorization_v1.json"]["sha256"] if index >= 2 and resume else None
        if resume_sha is not None:
            require(utc(review["reviewed_at_utc"]) >= utc(resume["declared_at_utc"]), "resumed continuation review predates authorization")
        reviews.append(validate_review(review, results[index - 1]["audit"], previous, next_case, campaign / previous["output"],
            owner_plans[index + 1], ORIGINAL_PLAN_SHA, resume_sha))
    complete = all(result["status"] == "FINALIZED" for result in results)
    report = {"schema": "portable_e2e.turn_launch_matrix_audit.v1", "status": "AUDITED_NOT_ADMITTED" if complete else "INCOMPLETE",
        "created_at_utc": datetime.now(timezone.utc).isoformat(), "reviewed_execution_commit": COMMIT,
        "prospective_plan": plan, "prospective_plan_sha256": ORIGINAL_PLAN_SHA, "resume_authorization": resume,
        "planned_cases": 8, "discovered_cases": len(indices), "finalized_cases": len(audits),
        "not_run_cases": sum(r["status"] == "NOT_RUN" for r in results), "all_planned_cases_retained": True,
        "cases": results, "continuation_reviews": reviews, "qualification_30_kph": "NOT_CLAIMED",
        "training_data_approved": False, "dataset_admission": False, "automatic_winner_selection": False,
        "full_future_xy_admission": False, "physical_actuation_proven": False,
        "historical_file_creation_time_proven": False, "repetitions_are_independent_routes": False,
        "wall_timing_independently_reconstructed": False, "pixels_decoded_not_content_approved": True,
        "source_manifest": [{"path": n, **v} for n, v in sorted(ledger.items())],
        "historical_source_manifest": [{"path": n, **v} for n, v in sorted(historical.items())],
        "audit_source_sha256": source_before, "auditor_sources_unchanged_during_audit": True,
        "interpretation": "Eight fixed-order full-route C-track 14.4 km/h expert cases, two per literal launch pedal. Every later ramp begins at .15. All native intervals, launch and handoff prefixes, raw camera frames and JPEG byte/decode checks are retained. Scalar failures are not replaced by 10 Hz diagnostics. Original two cases keep the overnight deadline; only six unstarted cases use the separately authorized afternoon deadline. No randomization, independently new routes, model control, causal proof, winner promotion, full XY/content approval or dataset admission."}
    for entry in report["source_manifest"]:
        require(base.sha(campaign / entry["path"]) == entry["sha256"], "matrix prospective/review source changed during audit")
    for entry in report["historical_source_manifest"]:
        require(base.sha(ROOT / entry["path"]) == entry["sha256"], "matrix historical input changed during audit")
    for root, (trial, images, _) in audits:
        for entry in trial["source_manifest"] + images:
            require(base.sha(root / entry["path"]) == entry["sha256"], "matrix raw/source/image changed during campaign audit")
    for name, digest in source_before.items():
        require(base.sha(ROOT / name) == digest, "matrix auditor implementation changed during audit")
    return report, audits


def main(argv=None):
    """HH_260906 - Write only a new private diagnostic folder, preserving every incomplete/failed original."""
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("campaign", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args(argv)
    output = v4.new_output(args.output_dir, [args.campaign])
    report, audits = audit_campaign(args.campaign)
    output.mkdir(parents=True, exist_ok=False)
    (output / "audit.json").write_text(json.dumps(v4.sanitize(report), indent=2, allow_nan=False) + "\n")
    for root, (_, images, _) in audits:
        (output / (root.parent.name + "_" + root.name + "_image_hashes.json")).write_text(json.dumps(images, indent=2) + "\n")
    print(json.dumps({"status": report["status"], "finalized": report["finalized_cases"], "not_run": report["not_run_cases"], "dataset_admission": False}))
    return 0 if report["status"] != "INCOMPLETE" else 2


if __name__ == "__main__":
    raise SystemExit(main())
