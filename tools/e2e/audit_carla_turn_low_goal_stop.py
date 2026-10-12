#!/usr/bin/env python3
"""HH_260906 - Independently audit the exact low-speed C-track turn without weakening historical V4 pins or admitting data."""

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

if __package__:
    from . import audit_carla_acknowledged_control as ack
    from . import audit_carla_brake_free_goal_stop as v4
    from . import audit_carla_comfortable_v3_trial as pilot
    from . import summarize_carla_goal_stop_trials as base
else:
    import audit_carla_acknowledged_control as ack
    import audit_carla_brake_free_goal_stop as v4
    import audit_carla_comfortable_v3_trial as pilot
    import summarize_carla_goal_stop_trials as base

ROOT = Path(__file__).resolve().parents[2]
COMMIT = "c4f40fc7c3eb86399392f9e582be8eefb4cd15bb"
SOURCES = pilot.SOURCE_NAMES | {"scripts/e2e/carla_wall_timing.py"}
REVIEWED_SOURCES = {
    "scripts/e2e/collect_carla_vad_expert.py": "8afc9254a67bab26962c43a451a94ed6b4663dc8ace1d57f9dc1ab1eded4c2ef",
    "scripts/e2e/carla_goal_stop_profile.py": "7ef2846bf8261766e6b41117fb8d901051e2330e4aac771130e5203740c41ca8",
    "scripts/e2e/carla_wall_timing.py": "2037b681ec75bfa85f1e34a7510766d6cd06dee20b0478773732f7fe0fafbf0a",
}
ROUTE_SHA = "2299e5bf2bc86789650da72336fdcfe7c11c585df0b9e0a0fe29a44730bae79b"
MAP = "C_track_1_0_7"
require, same, strict_equal, utc = base.require, pilot.same, v4.strict_equal, v4.utc
EPS = 1.0e-6
# HH_260906 - Literal independent contract; do not import, monkeypatch or silently substitute a historical V4 configuration.
FROZEN = {
    "profile_id": "turn_low_v1", "desired_deceleration_mps2": 0.6, "target_acceleration_limit_mps2": 1.0,
    "stop_buffer_m": 0.75, "goal_tolerance_m": 1.0, "stopped_speed_mps": 0.1, "hold_seconds": 2.0,
    "normal_brake_cap": 0.0, "normal_throttle_cap": 0.4, "minimum_tail_seconds": 6.5,
    "maximum_projection_step_m": 1.0, "maximum_projection_error_m": 3.0,
    "nominal_cruise_speed_mps": 4.0, "maximum_actual_speed_mps": 4.3, "launch_throttle": 0.15,
    "launch_handoff_speed_mps": 0.5, "maximum_launch_seconds": 8.0, "post_handoff_throttle_ramp_per_second": 0.05,
    "approach_reference_speed_mps": 3.0396464855480536, "approach_reference_distance_m": 35.0,
    "coast_reference_seconds": 6.93, "minimum_coast_entry_speed_mps": 2.8, "maximum_coast_entry_speed_mps": 3.2,
    "maximum_coast_seconds": 45.0, "cruise_minimum_speed_mps": 3.8, "cruise_maximum_speed_mps": 4.2,
    "minimum_cruise_seconds": 5.0, "route_sha256": ROUTE_SHA, "route_length_m": 206.31622010469437,
    "maximum_attempts_per_revision": 2, "development_only": True, "training_data_approved": False,
    "empirical_reference_notice": "Single-condition coast reference; not a robust stopping-distance guarantee.",
}
PLAN_FIXED = {
    "schema": "portable_e2e.turn_low_plan.v1", "source_commit": COMMIT, "profile": "turn_low_v1",
    "control_transport": "acknowledged_batch", "wall_timing": True, "map": MAP, "scenario": "left",
    "route_sha256": ROUTE_SHA, "route_length_m": 206.31622010469437,
    "route_path": "datasets/raw/carla/common10_v1/2026-09-04/30kph/c_track_1_0_7/turn/route_catalog_v5/routes/c_track_1_0_7/left/c_track_1_0_7_left_s0000_p01.json",
    "vehicle": "vehicle.toyota.prius", "weather": "ClearNoon", "quality": "Epic", "seed": 0,
    "physics_hz": 20, "camera_hz": 10, "nominal_target_speed_kmh": 14.4, "maximum_actual_speed_mps": 4.3,
    "low_speed_stability_band_mps": [3.8, 4.2], "minimum_stability_seconds": 5,
    "qualification_30_kph": "NOT_CLAIMED", "maximum_total_sim_seconds": 180, "wall_timeout_seconds": 900,
    "finish_before_utc": "2026-09-08T01:00:00Z", "maximum_attempts_per_revision": 2, "automatic_retry": False,
    "first_output": "c_track_left/run_001", "optional_second_output": "c_track_left/run_002",
    "review_required_before_second_attempt": True, "normal_brake_cap": 0, "emergency_control_unchanged": True,
    "physical_quality_limits_unchanged": True, "goal_tolerance_m": 1, "stopped_speed_mps": .1,
    "goal_dwell_seconds": 2, "stationary_tail_seconds": 6.5, "training_data_approved": False,
    "learned_model_control": False, "split_if_later_separately_admitted": "train", "native_carla_z_translation_applied_m": 0,
    "downstream_autoware_alignment_metadata_only_m": [0, 0, -15],
    "mapping_sha256": "9aaff2befed7ad12376b2e04bbdd51bd1808a3bafe39d87a6f6b241dbcca3136",
    "calibration_sha256": "5022cd1de5b48e9c824b6f2f8c59991fa665eeaf7a7bafd084c88acdb65e4bea",
    "historical_lateral_baseline_path": "datasets/raw/carla/common10_v1/2026-09-05/30kph/c_track_1_0_7/turn/qualified/b1_compact_lookahead/ClearNoon/seed_0000/run_001/episode/manifest.json",
    "historical_lateral_baseline_sha256": "bed0681f4ba6e494a18af3f9c8061cb9f77870fc2f241c9ce3c5f4abf051fc5a",
}

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
                  and gp.get("termination_reason") == "turn_low_v1_emergency_override", frame, "hazard was not observed then aborted")
        if speed > 4.3 or vx > 4.3:
            speed_violations.append({"frame": frame, "timestamp": t, "planar_speed_mps": speed, "vx_mps": vx})
            latched_failure = latched_failure or "turn_low_v1_actual_speed_exceeded"
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
                latched_failure = latched_failure or "turn_low_v1_launch_timeout"
        release_distance = 0.75 + 6.93 * speed
        if active and handoff is not None and coast is None and row["remaining_route_m"] <= release_distance:
            if not 2.8 <= speed <= 3.2:
                latched_failure = latched_failure or "turn_low_v1_coast_entry_speed_outside_band"
            elif latched_failure is None:
                coast, transition = t, "normal_pid_to_zero_pedal_coast"
        coast_elapsed = None if coast is None else last_active - coast
        if row["phase"] == "driving":
            hold_ticks = hold_ticks + 1 if row["stopped_in_goal"] and was_stopped else 0
            was_stopped = row["stopped_in_goal"]
        complete = hold_ticks >= 40
        if active and coast_elapsed is not None and coast_elapsed >= 45.0 and not complete:
            latched_failure = latched_failure or "turn_low_v1_coast_timeout"
        envelope = min(4.0, math.sqrt(3.0396464855480536 ** 2 + 1.2 * max(row["remaining_route_m"] - 35.0, 0.0)))
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
            latched_failure = latched_failure or "turn_low_v1_emergency_override"
        pending = abort is None and latched_failure == "turn_low_v1_emergency_override"
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


def recorded_bytes(commit, relative):
    """HH_260906 - Read only locally available exact Git blobs, with an explicit eleven-source allowlist and no fetch."""
    require(isinstance(commit, str) and re.fullmatch(r"[a-f0-9]{40}", commit) is not None and relative in SOURCES,
            "unsafe turn source identity")
    result = subprocess.run(["git", "-c", "protocol.allow=never", "show", commit + ":" + relative], cwd=ROOT,
        env=dict(os.environ, GIT_NO_LAZY_FETCH="1", GIT_ALLOW_PROTOCOL="", GIT_TERMINAL_PROMPT="0"),
        capture_output=True, timeout=10, check=False)
    require(result.returncode == 0, "reviewed source history unavailable locally; no fetch attempted")
    return result.stdout


def validate_cli(owner, root):
    """HH_260906 - Bind the new 14.4 km/h and historical C-track compact lookahead explicitly, not the Town07 CLI."""
    expected = {"--host": "127.0.0.1", "--port": str(owner["port"]), "--physics-hz": 20., "--capture-hz": 10.,
        "--target-speed-kmh": 14.4, "--max-duration-sec": 180., "--stationary-warmup-sec": 3.5,
        "--stationary-tail-sec": 6.5, "--spawn-z-offset-m": .5, "--weather": "ClearNoon", "--seed": 0.,
        "--goal-stop-profile": "turn_low_v1", "--goal-tolerance-m": 1., "--control-transport": "acknowledged_batch",
        "--mapping": "autoware_e2e_vad_launch/config/sensor_mapping_vad_fast_reliable.yaml",
        "--calibration": "src/launcher/autoware_launch/sensor_kit/carla_sensor_kit_launch/carla_sensor_kit_description/config/sensor_kit_calibration.yaml",
        "--basic-agent-base-min-distance-m": 2., "--basic-agent-distance-ratio": .2,
        "--basic-agent-lateral-kp": 1.95, "--basic-agent-lateral-ki": .05, "--basic-agent-lateral-kd": .2,
        "--basic-agent-max-steering": .8, "--basic-agent-lane-offset-m": 0.}
    original = owner["collector_argv"]
    require(isinstance(original, list) and all(isinstance(v, str) for v in original)
            and original.count("--wall-timing") == 1, "explicit single timing flag required")
    argv = [v for v in original if v != "--wall-timing"]
    require(len(argv) == 2 + 2 * len(expected), "extra or missing turn CLI argument")
    require(Path(argv[0]).resolve() == (root / "episode").resolve() and argv[1] == owner["route_path"], "output or route CLI mismatch")
    options = argv[2::2]
    require(len(set(options)) == len(options) and set(options) == set(expected), "duplicate, abbreviated or unknown turn CLI option")
    for key, value in zip(options, argv[3::2]):
        require(strict_equal(float(value), expected[key]) if type(expected[key]) is float else value == expected[key],
                "frozen turn CLI differs: " + key)


def verify_owner_sources(root, ledger):
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
            and plan.get("collector_wall_timeout_sec") == 900 and plan.get("finish_before_utc") == "2026-09-08T01:00:00Z",
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


def validate_profile(manifest, route):
    """HH_260906 - Low-speed stability and downstream alignment metadata must never become a 30 km/h or native-pose claim."""
    capture, result = manifest["capture_contract"], manifest["result"]
    config = capture["goal_stop_profile"]
    require(all(strict_equal(config.get(k), v) for k, v in FROZEN.items()), "turn profile differs from literal frozen configuration")
    require(capture["target_speed_kmh"] == 14.4 and manifest["coordinate_contract"]["wheelbase_m"] == 2.85
            and result.get("training_data_approved") is False and result.get("development_only") is True
            and result.get("qualification_30_kph") == config.get("qualification_30_kph") == "NOT_CLAIMED", "turn speed/admission claim differs")
    require(route["town"] == MAP and route["scenario"] == "left" and route["weather"] == "ClearNoon"
            and route.get("coordinate_alignment") is None and route["route_length_m"] == FROZEN["route_length_m"], "turn route or native alignment differs")
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


def audit_trial(root):
    """HH_260906 - Preserve failed, aborted and no-tail trials; protocol success does not waive raw scalar or future XY quality."""
    root, ledger = Path(root).resolve(), {}
    require((root / "owner_result.json").is_file(), "INCOMPLETE: turn owner result missing")
    plan, owner, started, times = verify_owner_sources(root, ledger)
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
    protocol = analyze_protocol(states, timeline["native_states"]) if timeline else analyze_protocol([], [])
    for camera in cameras:
        for name in base.CAMERAS:
            path = prefix + camera["images"][name]
            raw = pilot.checked_bytes(root, path, {})
            images.append({"path": path, "sha256": hashlib.sha256(raw).hexdigest(), "size_bytes": len(raw)})
    require(len({x["path"] for x in images}) == len(images), "duplicate turn image references")
    images.sort(key=lambda x: x["path"])
    normal = [s for s in states if s["capture_phase"] == "driving" and not s["goal_stop"].get("termination_reason")
              and not s["goal_stop"].get("emergency_failure_pending_next_tick")]
    result.update({"status": "AUDITED_NOT_ADMITTED", "transport_protocol": measured, "pilot_protocol": protocol,
        "normal_driving_nonzero_brake_count": sum(s["next_control"]["brake"] != 0 for s in normal),
        "normal_driving_command_count": len(normal), "source_head_commit": plan["source_head_commit"],
        "reviewed_execution_commit": COMMIT, "all_eleven_archives_match_owner_and_reviewed_commits": True,
        "archived_source_sha256": plan["source_sha256"], "comparison_configuration": ack.comparison_configuration(manifest) if manifest and "runtime" in manifest else None,
        "image_byte_integrity": {"file_count": len(images), "total_size_bytes": sum(x["size_bytes"] for x in images),
            "canonical_sorted_file_ledger_sha256": hashlib.sha256(json.dumps(images, sort_keys=True, separators=(",", ":")).encode()).hexdigest(),
            "pixels_decoded_or_visually_approved": False}, "source_manifest": [{"path": n, **v} for n, v in sorted(ledger.items())],
        "qualification_30_kph": "NOT_CLAIMED", "low_speed_stability_band_mps": [3.8, 4.2],
        "training_data_approved": False, "dataset_admission": False, "full_future_xy_admission": False,
        "physical_actuation_proven": False, "failed_before_first_observation": not states,
        "wall_timing_independently_reconstructed": False, "campaign_preregistration_checked_by_trial_only": False})
    for entry in result["source_manifest"] + images:
        require(base.sha(root / entry["path"]) == entry["sha256"], "turn raw input changed during audit")
    if "bounds_source_proof" in result:
        base.recheck_bounds_source_archive(root, result["bounds_source_proof"])
    return result, images, timeline


def validate_plan_document(plan):
    """HH_260906 - Bind all declared settings, source files and historical lateral provenance before interpreting any result."""
    extras = {"comment", "declared_at_utc", "source_hashes", "capture_flags", "historical_lateral_configuration", "comparison_notice", "expected_risks"}
    require(isinstance(plan, dict) and set(plan) == set(PLAN_FIXED) | extras, "unknown or missing low-turn prospective field")
    require(all(strict_equal(plan[k], value) for k, value in PLAN_FIXED.items()), "low-turn prospective contract changed")
    require(all(isinstance(plan[k], str) and plan[k] for k in ("comment", "comparison_notice")), "prospective explanation missing")
    require(isinstance(plan["expected_risks"], list) and len(plan["expected_risks"]) == 3
            and all(isinstance(v, str) and v for v in plan["expected_risks"]), "prospective risk disclosure missing")
    require(utc(plan["declared_at_utc"]) < utc(plan["finish_before_utc"]), "prospective declaration after deadline")
    require(isinstance(plan["source_hashes"], dict) and set(plan["source_hashes"]) == SOURCES, "prospective exact eleven sources missing")
    for name, digest in plan["source_hashes"].items():
        require(hashlib.sha256(recorded_bytes(COMMIT, name)).hexdigest() == digest, "prospective source differs from frozen commit")
    require(isinstance(plan["capture_flags"], list), "prospective CLI flags missing")
    validate_cli({"port": 2100, "route_path": "route.json", "collector_argv": [str(ROOT / "episode"), "route.json",
        "--host", "127.0.0.1", "--port", "2100", *plan["capture_flags"]]}, ROOT)
    # HH_260906 - The old train manifest is read only for lateral settings; no test payloads or old dataset bytes are changed.
    ledger = {}
    original = pilot.read_json(ROOT, plan["historical_lateral_baseline_path"], ledger)
    require(ledger[plan["historical_lateral_baseline_path"]]["sha256"] == plan["historical_lateral_baseline_sha256"], "historical lateral source SHA differs")
    control = original["capture_contract"]["basic_agent_control"]
    require(strict_equal(control, plan["historical_lateral_configuration"]), "historical lateral configuration differs from source")
    require(control["effective_opt_dict"]["base_min_distance"] == control["waypoint_purge_lookahead"]["base_min_distance_m"] == 2.
            and control["effective_opt_dict"]["distance_ratio"] == control["waypoint_purge_lookahead"]["distance_ratio_s"] == .2,
            "historical lateral configuration is not the declared compact C-track baseline")
    return ledger


def validate_second_review(review, first, first_root, second_root, first_owner, second_plan):
    """HH_260906 - Any optional repetition needs an intervening source-bound review, never a selective automatic retry."""
    state_entries = [e for e in first["source_manifest"] if e["path"].endswith("/states.jsonl")]
    require(len(state_entries) == 1, "second review has no unique preceding native source")
    fixed = {"schema": "portable_e2e.turn_low_second_review.v1", "source_commit": COMMIT,
        "previous_output": "c_track_left/run_001", "next_output": "c_track_left/run_002",
        "previous_states_sha256": state_entries[0]["sha256"], "previous_owner_exit_code": first_owner["exit_code"],
        "previous_transport_status": first["transport_protocol"]["status"],
        "previous_native_scalar_quality_clear": first["independent_qa"]["raw_scalar_quality_clear"],
        "previous_pilot_protocol_all_checks_pass": first["pilot_protocol"]["all_checks_pass"],
        "source_and_parameters_changed": False, "automatic_retry": False, "training_data_approved": False,
        "qualification_30_kph": "NOT_CLAIMED"}
    require(isinstance(review, dict) and set(review) == set(fixed) | {"reviewed_at_utc", "comment", "reason"}
            and all(strict_equal(review[k], value) for k, value in fixed.items()), "optional second review contradicts independent first evidence")
    require(all(isinstance(review[k], str) and review[k] for k in ("comment", "reason")), "second review rationale missing")
    require(utc(first_owner["completed_at_utc"]) <= utc(review["reviewed_at_utc"]) < utc(second_plan["planned_at_utc"]),
            "second review does not intervene before second planning")


def audit_campaign(campaign):
    """HH_260906 - Retain every discovered attempt and distinguish maximum two from a requirement to repeat failed data."""
    campaign, ledger = Path(campaign).resolve(), {}
    plan = pilot.read_json(campaign, "pilot_plan.json", ledger)
    historical_sources = validate_plan_document(plan)
    parent = campaign / "c_track_left"
    roots = sorted(p for p in parent.iterdir() if p.name.startswith("run_")) if parent.exists() else []
    require([p.name for p in roots] in ([], ["run_001"], ["run_001", "run_002"]), "excess, omitted or noncontiguous low-turn attempts")
    results, audits, owners, plans = [], [], [], []
    for root in roots:
        require(root.is_dir() and not root.is_symlink(), "unsafe low-turn attempt directory")
        if not (root / "owner_result.json").exists():
            results.append({"trial_id": root.name, "status": "INCOMPLETE", "dataset_admission": False})
            continue
        result = audit_trial(root)
        audited = result[0]
        relative = root.relative_to(campaign).as_posix()
        owner = pilot.read_json(campaign, relative + "/owner_result.json", ledger)
        owner_plan = pilot.read_json(campaign, relative + "/owner_plan.json", ledger)
        require(utc(plan["declared_at_utc"]) <= utc(owner_plan["planned_at_utc"]), "turn was planned before prospective declaration")
        require(owner_plan["source_sha256"] == plan["source_hashes"]
                and Path(owner_plan["route_path"]).resolve() == (ROOT / plan["route_path"]).resolve(), "turn source or route differs from declaration")
        require(owner_plan["collector_argv"] == [str(root / "episode"), owner_plan["route_path"], "--host", "127.0.0.1",
                "--port", str(owner_plan["port"]), *plan["capture_flags"]], "turn exact argv differs from prospective plan")
        results.append(audited); audits.append((root, result)); owners.append(owner); plans.append(owner_plan)
    review = None
    if len(roots) == 2:
        require(len(audits) >= 1 and results[0]["status"] != "INCOMPLETE", "second attempt started before first was finalized")
        review = pilot.read_json(campaign, "run_002_preflight_review.json", ledger)
        second_plan = pilot.read_json(campaign, "c_track_left/run_002/owner_plan.json", ledger)
        validate_second_review(review, results[0], roots[0], roots[1], owners[0], second_plan)
    complete = bool(roots) and len(audits) == len(roots)
    report = {"schema": "portable_e2e.turn_low_goal_stop_campaign_audit.v1", "status": "AUDITED_NOT_ADMITTED" if complete else "INCOMPLETE",
        "created_at_utc": datetime.now(timezone.utc).isoformat(), "reviewed_execution_commit": COMMIT,
        "prospective_plan": plan, "prospective_plan_sha256": ledger["pilot_plan.json"]["sha256"],
        "discovered_attempts": len(roots), "finalized_attempts": len(audits), "maximum_attempts": 2,
        "all_discovered_attempts_retained": True, "optional_second_review": review, "trials": results,
        "historical_file_creation_time_proven": False, "qualification_30_kph": "NOT_CLAIMED", "training_data_approved": False,
        "dataset_admission": False, "full_future_xy_admission": False, "automatic_promotion": False,
        "same_initial_condition_repetitions_not_independent_routes": True,
        "identical_control_between_maps_comparison": False, "wall_timing_independently_reconstructed": False,
        "source_manifest": [{"path": name, **value} for name, value in sorted(ledger.items())],
        "historical_lateral_source_manifest": [{"path": name, **value} for name, value in sorted(historical_sources.items())],
        "audit_source_sha256": {Path(p).resolve().relative_to(ROOT).as_posix(): base.sha(Path(p)) for p in (__file__, ack.__file__, v4.__file__, pilot.__file__, base.__file__)},
        "interpretation": "A separate 14.4 km/h C-track left development diagnostic with historical C-track compact lookahead, not an identical-control Town07 comparison. All raw failures remain. Low-speed stability is measured 3.8..4.2 m/s for 5 seconds; the inherited base 30 km/h ceiling flag is only a loose upper bound, never 30 km/h cruise qualification. Exact native 20 Hz acceleration, stop/goal geometry and ACK evidence are independently recomputed. Camera byte hashes are not pixel approval. Timing files are preserved but not independently reconstructed here. No model, training, test payload, native coordinate rewrite or data admission."}
    for entry in report["source_manifest"]:
        require(base.sha(campaign / entry["path"]) == entry["sha256"], "prospective source changed during audit")
    for entry in report["historical_lateral_source_manifest"]:
        require(base.sha(ROOT / entry["path"]) == entry["sha256"], "historical lateral source changed during audit")
    for root, (result, images, _) in audits:
        for entry in result["source_manifest"] + images:
            require(base.sha(root / entry["path"]) == entry["sha256"], "turn source changed during campaign audit")
    return report, audits


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("campaign", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--visual-root", type=Path, help="publish actual camera evidence after independent raw audit")
    args = parser.parse_args(argv)
    if args.visual_root is not None:
        report = publish(args.campaign, args.output_dir, args.visual_root)
        print(json.dumps({"status": report["status"], "attempts": report["discovered_attempts"], "dataset_admission": False}))
        return 0
    output = v4.new_output(args.output_dir, [args.campaign])
    report, audits = audit_campaign(args.campaign)
    output.mkdir(parents=True, exist_ok=False)
    (output / "audit.json").write_text(json.dumps(v4.sanitize(report), indent=2, allow_nan=False) + "\n")
    for root, (_, images, _) in audits:
        (output / (root.name + "_image_hashes.json")).write_text(json.dumps(images, indent=2) + "\n")
    print(json.dumps({"status": report["status"], "attempts": report["discovered_attempts"], "dataset_admission": False}))
    return 0 if report["status"] != "INCOMPLETE" else 2


def checked_turn_visuals(visual_root, raw_root):
    """HH_260906 - Verify exact existing low-speed visual bytes; no missing 30-class cruise image is invented."""
    from PIL import Image
    visual_root, raw_root, ledger = Path(visual_root), Path(raw_root), {}
    metadata = pilot.read_json(visual_root, "visual_provenance.json", ledger)
    require(metadata["schema"] == "carla_expert.raw_trial_visual_diagnostic.v1" and metadata["original_owner_exit_code"] == 1
            and metadata["training_data_approved"] is False and metadata["learned_model_control"] is False,
            "actual turn visual status or scope differs")
    prefix = metadata["source_episode_name"] + "/"
    require(prefix == "episode.partial/", "first failed turn evidence must retain its partial directory")
    require(set(metadata["source_metadata_sha256"]) == {"manifest.json", "route.json", "states.jsonl", "camera_frames.jsonl"}, "turn visual raw metadata set differs")
    for name, digest in {**metadata["source_metadata_sha256"], **metadata["displayed_image_sha256"]}.items():
        raw = pilot.checked_bytes(raw_root, prefix + name, {})
        require(hashlib.sha256(raw).hexdigest() == digest, "turn visual belongs to changed or different raw evidence")
    expected_png = {"01_start", "03_coast_entry", "04_goal_dwell", "05_final_observation", "06_maximum_observed_deceleration"}
    require(set(metadata["png_indices"]) == expected_png, "unexpected low-speed renderer frame selection")
    expected = {name + ".png" for name in expected_png} | {"whole_recording_accelerated.gif", "visual_provenance.json"}
    require({p.name for p in visual_root.iterdir()} == expected and metadata["camera_stride"] == 5 and metadata["playback_fps"] == 10,
            "turn visual output set or playback changed")
    count = base.integer(metadata["camera_anchor_count"])
    indices = list(range(0, count, 5))
    if indices[-1] != count - 1:
        indices.append(count - 1)
    require(metadata["rendered_indices"] == indices, "turn visual did not retain whole recording at declared stride")
    payloads, dimensions = {}, {}
    for name in sorted(expected):
        payloads[name] = pilot.checked_bytes(visual_root, name, ledger)
        if name.endswith((".png", ".gif")):
            with Image.open(visual_root / name) as im:
                require(im.size == (1600, 900), "turn visual dimensions differ")
                frames = getattr(im, "n_frames", 1)
                for index in range(frames):
                    im.seek(index); im.load()
                if name.endswith(".gif"):
                    require(frames == len(indices), "turn GIF frame count differs")
                dimensions[name] = {"width": 1600, "height": 900, "frames": frames}
    return payloads, {"original_visual_metadata": metadata, "decoded_outputs": dimensions,
        "source_manifest": [{"path": k, **v} for k, v in sorted(ledger.items())],
        "missing_02_measured_cruise_notice": "The unchanged generic renderer selects that frame only at >=7.8 m/s; it correctly produces no 30-class cruise image in this 4 m/s turn diagnostic."}


def render_actual_left_midpoint(root, output):
    """HH_260906 - Choose one camera anchor from the catalog LEFT interval midpoint, never by attractive outcome or appearance."""
    from scripts.e2e import render_carla_raw_trial as renderer
    data = renderer.load_trial(root)
    points = data["route"]["route"]
    left = [(i, p) for i, p in enumerate(points) if p.get("road_option") == "LEFT"]
    require(left and [i for i, _ in left] == list(range(left[0][0], left[-1][0] + 1)), "catalog LEFT interval missing or disjoint")
    start, end = float(left[0][1]["distance_m"]), float(left[-1][1]["distance_m"])
    midpoint = (start + end) / 2
    selected = min(range(len(data["cameras"])), key=lambda i: abs(data["states"][data["frames"][data["cameras"][i]["frame"]]]["route_progress_m"] - midpoint))
    canvas = renderer.render_frame(data, selected, 5, 10)
    canvas.save(output)
    state = data["states"][data["frames"][data["cameras"][selected]["frame"]]]
    for name, digest in {**data["source_metadata_sha256"], **data["displayed_image_sha256"]}.items():
        require(base.sha(data["episode"] / name) == digest, "left midpoint source changed during rendering")
    return {"schema": "carla_expert.catalog_left_midpoint_visual.v1", "selection": "Nearest recorded camera anchor to the single contiguous catalog LEFT tag interval midpoint; no outcome/image based selection.",
        "catalog_left_first_index": left[0][0], "catalog_left_last_index": left[-1][0], "catalog_arc_interval_m": [start, end],
        "catalog_arc_midpoint_m": midpoint, "selected_camera_index": selected, "selected_frame": state["frame"],
        "selected_actual_progress_m": state["route_progress_m"], "selected_actual_speed_kmh": math.hypot(state["vx"], state["vy"]) * 3.6,
        "source_episode_name": data["episode"].name, "source_metadata_sha256": data["source_metadata_sha256"],
        "displayed_image_sha256": data["displayed_image_sha256"], "renderer_sha256": base.sha(Path(renderer.__file__)),
        "layout_sha256": base.sha(Path(renderer.view.__file__)), "rendered_png_sha256": base.sha(output),
        "learned_model_control": False, "training_data_approved": False}


def render_numerical_failure(result, timeline, root, output):
    """HH_260906 - Plot all raw phases and native violations with an explicit startup inset, without smoothing or dropping samples."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    rows = timeline["native_states"]
    prefix = "episode.partial" if (root / "episode.partial").exists() else "episode"
    states = pilot.read_json(root, prefix + "/states.jsonl", {}, lines=True)
    route = pilot.read_json(root, prefix + "/route.json", {})
    figure, axes = plt.subplots(2, 2, figsize=(12.8, 7.2))
    times = [r["elapsed_s"] for r in rows]
    axes[0, 0].plot(times, [r["speed_mps"] * 3.6 for r in rows], label="Measured speed")
    axes[0, 0].plot(times, [r["target_speed_mps"] * 3.6 for r in rows], label="Command target", alpha=.8)
    axes[0, 0].axhline(4.3 * 3.6, color="black", ls="--", lw=.8, label="Actual 4.3 m/s cap")
    axes[0, 0].set(xlabel="Seconds from first state", ylabel="Speed (km/h)", title="Entire recording: setup, driving and stopped tail")
    axes[0, 1].plot([p["x"] for p in route["route"]], [p["y"] for p in route["route"]], label="Original catalog route", lw=2)
    axes[0, 1].plot([s["x"] for s in states], [s["y"] for s in states], label="Measured rear-axle trace", lw=1)
    axes[0, 1].scatter([states[0]["x"], states[-1]["x"]], [states[0]["y"], states[-1]["y"]], marker="x", color="black")
    axes[0, 1].set(xlabel="Native ROS X (m)", ylabel="Native ROS Y (m)", title="Full C-track LEFT route; unchanged native coordinates")
    axes[0, 1].axis("equal")
    for key, title, color in (("native_intervals", "Native 20 Hz", "#206080"), ("camera_intervals", "Camera-aligned 10 Hz", "#d17b25")):
        intervals = timeline[key]
        for axis in axes[1]:
            axis.plot([v["time_from_capture_start_s"] for v in intervals], [v["speed_rate_mps2"] for v in intervals], label=title, color=color, lw=1)
    violation = max(timeline["native_intervals"], key=lambda v: v["speed_rate_mps2"])
    for axis in axes[1]:
        axis.axhline(2.9, color="black", ls="--", lw=.8, label="Decoder +/-2.9")
        axis.axhline(-2.9, color="black", ls="--", lw=.8)
        axis.set(xlabel="Seconds from first state", ylabel="Measured scalar speed rate (m/s²)")
    axes[1, 0].set_title(f"All {len(timeline['native_intervals']):,} native intervals: startup violation retained")
    axes[1, 1].axhline(3.0, color="#a00000", ls=":", lw=1, label="Runtime acceleration +3.0")
    axes[1, 1].set_xlim(max(0, violation["time_from_capture_start_s"] - 2), violation["time_from_capture_start_s"] + 2)
    axes[1, 1].set_title(f"Launch failure: +{violation['speed_rate_mps2']:.6f} m/s²")
    for axis in axes.flat:
        axis.legend(fontsize=7); axis.grid(alpha=.18)
    figure.suptitle("C-track 14.4 km/h development | route/goal/ACK complete | native scalar FAIL", fontsize=14)
    figure.text(.5, .009, "Actual records, no filtering. 10 Hz passes but cannot replace native 20 Hz failure. Not 30 km/h, full XY, image or training admission.", ha="center", fontsize=8)
    figure.tight_layout(rect=(0, .025, 1, .95))
    figure.savefig(output, dpi=150)
    plt.close(figure)


def publish(campaign, output, visual_root):
    """HH_260906 - Publish one recorded failure create-only, retaining the optional unperformed second attempt as unexhausted."""
    campaign, visual_root = Path(campaign).resolve(), Path(visual_root).resolve()
    output = v4.new_output(output, [campaign, visual_root])
    report, audits = audit_campaign(campaign)
    require(report["status"] == "AUDITED_NOT_ADMITTED" and report["discovered_attempts"] == 1, "this category freezes the first low-turn failure only")
    root, (trial, images, timeline) = audits[0]
    require(trial["owner_exit_code"] == 1 and not trial["independent_qa"]["raw_scalar_quality_clear"], "first low-turn category must retain the actual failure")
    # HH_260906 - This human-readable category freezes the first actual journals, not any later same-source repetition.
    publication_journals = {"episode.partial/states.jsonl": "62b2d3d0a967a38810424071492d8643a5b66f3b38578531bdb446572ad3502f",
        "episode.partial/control_receipts.jsonl": "4a7c7c89f79bd436e0d593a24a453669a1b37be5f83d9e950c5ed883f3a52e00"}
    require(all(base.sha(root / path) == digest for path, digest in publication_journals.items()), "publication is restricted to the exact first raw turn journals")
    payloads, visual_proof = checked_turn_visuals(visual_root / "run_001", root)
    raw_route = pilot.checked_bytes(root, "episode.partial/route.json", {})
    require(hashlib.sha256(raw_route).hexdigest() == ROUTE_SHA and re.search(rb"/home/|password|PRIVATE KEY", raw_route, re.I) is None,
            "exact published route has changed or contains private text")
    output.mkdir(parents=True, exist_ok=False)
    def write(name, value):
        with (output / name).open("x", encoding="utf-8") as stream:
            stream.write(value)
    report["optional_second_attempt"] = {"status": "NOT_RUN", "maximum_two_attempt_quota": "NOT_EXHAUSTED", "mandatory_repeat": False}
    report["publication_notice"] = "Redacted independent metadata view; original private source bytes remain unchanged and are bound by SHA256. No full camera corpus or training dataset copied into Git."
    write("audit.json", json.dumps(v4.sanitize(report), indent=2, allow_nan=False) + "\n")
    write("run_001_image_hashes.json", json.dumps(images, indent=2) + "\n")
    (output / "original_c_track_left_route.json").write_bytes(raw_route)
    camera_dir = output / "actual_camera_route_evidence"
    camera_dir.mkdir()
    for name, payload in payloads.items():
        if name.endswith(".json"):
            write("actual_camera_route_evidence/" + name, json.dumps(v4.sanitize(json.loads(payload)), indent=2) + "\n")
        else:
            (camera_dir / name).write_bytes(payload)
    extra = render_actual_left_midpoint(root, camera_dir / "07_catalog_left_midpoint.png")
    write("actual_camera_route_evidence/07_catalog_left_midpoint_provenance.json", json.dumps(v4.sanitize(extra), indent=2) + "\n")
    render_numerical_failure(trial, timeline, root, output / "01_full_route_and_native_launch_failure.png")
    proof = {"schema": "portable_e2e.turn_low_publication.v1", "audit_sources": report["audit_source_sha256"],
        "original_route_sha256": ROUTE_SHA, "original_route_exact_bytes": True, "existing_png_gif_exact_bytes": True,
        "metadata_json_policy": "Public metadata views may be redacted/reformatted; original raw hashes remain in the source manifests.",
        "actual_visual_proof": visual_proof, "additional_left_midpoint_proof": extra,
        "source_raw_trial": root.relative_to(ROOT).as_posix(), "source_visual_root": visual_root.relative_to(ROOT).as_posix(),
        "training_data_approved": False}
    write("publication_manifest.json", json.dumps(v4.sanitize(proof), indent=2) + "\n")
    write("README.md", """<!-- HH_260906 - Preserve the first low-speed turn failure with exact raw provenance and no admission claim. -->
# C-track 저속 좌회전: 경로·정차 완료, 출발 가속도 실패

실제 CARLA BasicAgent 전문가 주행입니다. 학습 모델이나 Autoware 자율주행 화면이 아닙니다. 목표 14.4 km/h의 별도 개발 시험이며 30 km/h 검증·학습 데이터 승인은 하지 않았습니다.

- 실행 **1회 / 허용 최대 2회**. 첫 실행 실패를 그대로 보존했습니다. 두 번째는 **미실행(NOT_RUN)**이며 횟수 한도가 소진된 것도, 반드시 재시도해야 하는 것도 아닙니다.
- native 1,907개 상태, 954개 10Hz 카메라 묶음(5,724개 이미지), ACK 1,912개 모두 확인; 불일치·충돌·차선 침범 0개.
- 실제 최대 14.4086 km/h, 저속 안정 구간 39.45초. 종점 오차 0.9386m, 2초 정지 유지와 130개 정차 후속 상태 확인.
- **20Hz 출발 한 구간 +3.654933 m/s²**가 물리 디코더 +2.9 및 runtime +3.0 기준을 초과했습니다. 10Hz 최대 +2.450543만 보고 통과시키지 않습니다. 원본 구간을 제거·평활화하지 않았습니다.
- 기존 C-track lateral 설정 2.0m/0.2s를 사용했습니다. Town07의 3.0m/0.5s와 동일 제어 조건의 맵 간 A/B가 아닙니다. 원본 경로의 예전 30km/h PASS는 새 시험의 승인 근거가 아닙니다.

[전체 경로·속도·출발 위반 확대](01_full_route_and_native_launch_failure.png) · [좌회전 중앙 실제 6카메라/차량 중심 경로](actual_camera_route_evidence/07_catalog_left_midpoint.png) · [전체 구간 GIF](actual_camera_route_evidence/whole_recording_accelerated.gif) · [정차 장면](actual_camera_route_evidence/04_goal_dwell.png)

GIF는 10Hz 카메라에서 5개 간격으로 고른 10fps **가속 재생**입니다. 센서 FPS나 GUI 끊김 측정이 아닙니다. 1600×900 카메라 합성 화면은 전체 FOV를 보존하고 차량을 경로 패널 중앙에 표시합니다. 07 장면은 원본 LEFT 태그 구간의 중간 진행거리와 가장 가까운 카메라로 고정 선정했습니다. 기존 renderer의 02는 7.8m/s 이상을 조건으로 하므로 이 저속 기록에는 생성하지 않았습니다.

[정확한 원본 경로 JSON](original_c_track_left_route.json) · [독립 계측과 소스·사전 계획 검증](audit.json) · [전체 이미지 SHA 목록](run_001_image_hashes.json) · [발행 출처 증거](publication_manifest.json) · [파일 검증값](SHA256SUMS)

원본 C-track 경로 끝에는 같은 위치·진행거리의 점 두 개가 있습니다. 독립 계측기의 접선 선택만 직전의 다른 위치를 찾도록 수정했습니다. 원본 경로 점·배열·파일과 실제 차량 높이는 변경하지 않았습니다. downstream Autoware z−15 정렬은 native CARLA spawn/goal/상태/센서 TF에 적용하지 않았습니다.

원본 이미지와 실패 기록은 private artifacts에 유지합니다. 이 폴더의 PNG/GIF 기존 렌더본과 경로는 원본 바이트이며 JSON은 계정 경로를 가린 메타데이터 뷰입니다. 원본 해시를 보존합니다. 카메라 바이트 무결성은 전체 픽셀·미래 XY·학습 데이터 품질 승인과 다릅니다.
""")
    for path, digest in report["audit_source_sha256"].items():
        require(base.sha(ROOT / path) == digest, "auditor source changed while publishing")
    for entry in trial["source_manifest"] + images:
        require(base.sha(root / entry["path"]) == entry["sha256"], "raw turn source changed while publishing")
    for entry in visual_proof["source_manifest"]:
        require(base.sha(visual_root / "run_001" / entry["path"]) == entry["sha256"], "visual source changed while publishing")
    files = sorted(p for p in output.rglob("*") if p.is_file())
    write("SHA256SUMS", "".join(base.sha(p) + "  " + p.relative_to(output).as_posix() + "\n" for p in files))
    return report


if __name__ == "__main__":
    raise SystemExit(main())
