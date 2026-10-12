#!/usr/bin/env python3
"""HH_260906 - Audit fresh v6 ABBA evidence with original raw QA; never admit or rewrite targets."""
from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict, replace
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from scripts.e2e import audit_carla_acknowledged_control as ack
from scripts.e2e import audit_carla_raw_pre_admission as raw
from scripts.e2e import carla_goal_stop_profile as profiles
from scripts.e2e import summarize_carla_goal_stop_trials as scalar

SCHEMA = "carla.low_speed_steering_abba_summary.v1"
PROFILE = "comfortable_v6_low_speed_steering"
PLAN_SHA256 = "f0d4ee95bb1ab9f46117e42107a2b396950e16cf9f2b918bd4f6d46718fe131a"
MAX_EVIDENCE_BYTES = 256 * 1024 * 1024
CASES = (("01_native_A1", "native"), ("02_zero_B1", "zero"), ("03_zero_B2", "zero"), ("04_native_A2", "native"))
EXECUTION_SOURCES = {"scripts/e2e/" + name for name in ("run_owned_carla_expert_trial.sh", "run_carla_map.sh",
    "process_group_cleanup.sh", "workspace_runtime_lock.sh", "probe_carla_server.py", "env.sh",
    "collect_carla_vad_expert.py", "carla_goal_stop_profile.py")} | {"portable_e2e/model.py", "portable_e2e/runtime_contract.py"}
INPUT_PATHS = {"docs/assets/validation/2026-09-01/town07/autoware_vad/straight/autoware_vad_route.json",
    "autoware_e2e_vad_launch/config/sensor_mapping_vad_fast_reliable.yaml",
    "src/launcher/autoware_launch/sensor_kit/carla_sensor_kit_launch/carla_sensor_kit_description/config/sensor_kit_calibration.yaml"}
AUDIT_MATH_PATHS = {"scripts/e2e/" + name for name in ("audit_carla_raw_pre_admission.py", "audit_carla_acknowledged_control.py",
    "audit_portable_target_feasibility.py", "summarize_carla_goal_stop_trials.py", "prepare_carla_common10_dataset.py")} | {
    "portable_e2e/model.py", "portable_e2e/runtime_contract.py"}
PHYSICAL_FIELDS = ("capture_phase", "x", "y", "z", "yaw", "vx", "vy", "ax", "ay", "yaw_rate",
    "actor_snapshot_transform_carla", "world_velocity_carla", "world_acceleration_carla", "world_angular_velocity_carla_deg_s",
    "front_left_wheel_steer_angle_carla_deg", "front_right_wheel_steer_angle_carla_deg",
    "front_left_wheel_steer_angle_ros_rad", "front_right_wheel_steer_angle_ros_rad", "steering_tire_angle_rad",
    "speed_limit_carla_kmh", "speed_limit_mps", "command", "route_progress_m", "route_cte_m", "current_control")
require, number = scalar.require, scalar.number


def safe(path):
    require(".." not in Path(path).parts, "parent traversal is not evidence")
    path = Path(path).absolute()
    require(not any(p.is_symlink() for p in (path, *path.parents)), "symlink evidence path")
    return path


def encoded(value):
    return json.dumps(value, sort_keys=True, allow_nan=False, separators=(",", ":")).encode()


def equal(first, second):
    return encoded(first) == encoded(second)


def configuration_equal(first, second):
    # HH_260906 - The pinned prospective JSON spells whole-valued physical quantities as integers; booleans stay strict.
    if type(first) is bool or type(second) is bool:
        return type(first) is type(second) and first == second
    if isinstance(first, dict) and isinstance(second, dict):
        return set(first) == set(second) and all(configuration_equal(first[key], second[key]) for key in first)
    if type(first) in (int, float) and type(second) in (int, float):
        return number(first) == number(second)
    return equal(first, second)


def checked(path, ledger, expected=None, *, jsonl=False, binary=False):
    path = safe(path)
    require(path.is_file() and path.stat().st_size <= MAX_EVIDENCE_BYTES, "bounded regular evidence file required")
    payload = path.read_bytes()
    require(len(payload) <= MAX_EVIDENCE_BYTES, "evidence payload exceeds byte bound")
    digest = hashlib.sha256(payload).hexdigest()
    require(expected is None or digest == expected, f"input SHA-256 mismatch: {path}")
    require(str(path) not in ledger or ledger[str(path)] == digest, "evidence changed between repeated reads")
    ledger[str(path)] = digest
    if binary:
        return payload
    def load(value):
        return json.loads(value, object_pairs_hook=scalar._pairs,
            parse_float=lambda value: number(float(value)),
            parse_constant=lambda value: (_ for _ in ()).throw(scalar.EvidenceError("nonfinite JSON constant")))
    return [load(line) for line in payload.splitlines() if line.strip()] if jsonl else load(payload)


def instant(value):
    stamp = datetime.fromisoformat(value.replace("Z", "+00:00"))
    require(stamp.tzinfo is not None, "UTC-aware evidence time required")
    return stamp


def validate_plan(plan):
    require(plan.get("schema") == "carla.low_speed_steering_abba_plan.v1" and plan["maximum_attempts_per_revision"] == 4 and
        plan["automatic_retry"] is False and all(case["maximum_attempts"] == 1 for case in plan["cases"]), "prospective schema/attempt budget differs")
    require([(case["id"], case["arm"]) for case in plan["cases"]] == list(CASES), "fixed four-case ABBA order required")
    require(set(plan["source_sha256"]) == EXECUTION_SOURCES and set(plan["input_sha256"]) == INPUT_PATHS,
        "execution or fixed-input closure differs")
    require(set(plan["audit_math_sha256"]) == AUDIT_MATH_PATHS, "prospective audit math closure differs")
    for pins in (plan["source_sha256"], plan["input_sha256"], plan["audit_math_sha256"]):
        require(all(isinstance(value, str) and len(value) == 64 and set(value) <= set("0123456789abcdef") for value in pins.values()), "invalid source/input pin")
    require(set(plan["protocol_configuration_by_arm"]) == {"native", "zero"}, "both frozen protocol configurations required")
    for arm in ("native", "zero"):
        expected = asdict(replace(profiles.LowSpeedSteeringDevelopmentGoalStopConfig(), low_speed_steering_mode=arm))
        require(configuration_equal(plan["protocol_configuration_by_arm"][arm], expected), "v6 protocol configuration differs")
    require(plan.get("training_data_approved") is False and plan.get("learned_model_control") is False and
        plan.get("vehicle_control_approved") is False, "prospective scope must explicitly deny admission and learned/vehicle control")
    instant(plan["declared_at_utc"])


def blank_control():
    return dict(throttle=0., steer=0., brake=1., hand_brake=False, reverse=False, gear=0, manual_gear_shift=False)


def steering_protocol(states, native_rows, manifest, receipts, arm):
    """HH_260906 - Independently replay the intermediate policy, then the pre-existing guard and actual command receipt."""
    require(len(states) == len(native_rows) and all(a["frame"] == b["frame"] and a["timestamp"] == b["timestamp"]
        for a, b in zip(states, native_rows)), "policy/native row count/frame/time mismatch")
    failures, decisions, switches, records = [], Counter(), [], []
    commands = {receipt["sequence"]: receipt for receipt in receipts}
    initial_rows = [row for row in states if row["goal_stop"].get("next_control_starts_driving") is True]
    driving = [row for row in states if row["capture_phase"] == "driving"]
    if driving:
        require(len(initial_rows) == 1, "driving requires exactly one initial command source")
    hold_ticks, previous_stopped, hold_source, previous_eligible = 0, False, None, None
    event_totals = {name: sum(len(row[name]) for row in states) for name in ("collision", "lane_invasion")}
    previous_events = {name: 0 for name in event_totals}
    def check(condition, frame, name):
        if not condition:
            failures.append({"frame": frame, "check": name})
    for row, measured in zip(states, native_rows):
        gp, frame = row["goal_stop"], row["frame"]
        request = gp["low_speed_steering"]
        initial = gp.get("next_control_starts_driving") is True
        source_driving = row["capture_phase"] == "driving" or initial
        driving_command = initial or (row["capture_phase"] == "driving" and gp["termination_reason"] is None)
        speed = math.hypot(number(row["vx"]), number(row["vy"]))
        failure = gp.get("pilot_failure_reason")
        emergency = gp["emergency_override_count"] > 0 or failure == PROFILE + "_emergency_override"
        eligible = driving_command and source_driving and not emergency and failure is None and speed <= .75
        overridden = eligible and arm == "zero"
        reason = ("emergency_unchanged" if emergency else "governor_failure_unchanged" if failure is not None
            else "not_driving_command_unchanged" if not (driving_command and source_driving)
            else "native_mode_unchanged" if arm == "native" else "above_threshold_unchanged" if not eligible else "zero_steering_requested")
        expected = {"schema": "carla.low_speed_steering_request.v1", "mode": arm,
            "source_timestamp": row["timestamp"], "source_measured_speed_mps": speed,
            "governor_driving_enabled": source_driving, "driving_command": driving_command,
            "threshold_mps": .75, "emergency_override_present": emergency, "governor_failure_reason": failure,
            "eligible": eligible, "override_requested": overridden, "decision": reason,
            "control_stage": "after_original_agent_proposal_checks_before_existing_stopped_brake_guard",
            "physical_application_proven": False, "training_data_approved": False, "research_only": True}
        check(all(key in request and equal(request[key], value) for key, value in expected.items()), frame, "policy_source_decision_and_scope")
        check(gp["profile_id"] == PROFILE and gp["training_data_approved"] is False and
            gp["measured_timestamp"] == row["timestamp"] and gp["measured_speed_mps"] == speed and
            gp["low_speed_steering_driving_enabled"] is source_driving, frame, "governor_source_binding")
        monitor = gp["low_speed_steering_event_monitor"]
        counts = monitor["observed_event_counts"]
        check(monitor["source_timestamp"] == row["timestamp"] and monitor["asynchronous_event_delivery"] is True and
            monitor["zero_latency_abort_proven"] is False and set(counts) == set(event_totals) and all(
                type(counts[name]) is int and previous_events[name] <= counts[name] <= event_totals[name] for name in event_totals),
            frame, "bounded_asynchronous_event_monitor")
        if any(counts.values()):
            check(failure is not None and not overridden, frame, "observed_event_stops_new_intervention")
        previous_events = counts
        before = dict(ack.full_control(request["input_control"]))
        returned = dict(ack.full_control(request["returned_control"]))
        wanted = dict(before, steer=0.) if overridden else before
        check(equal(returned, wanted) and request["input_steer"] == before["steer"] and
            request["returned_steer"] == returned["steer"] and
            request["steering_value_changed"] is (before["steer"] != returned["steer"]), frame, "only_requested_steer_changed")
        guarded = dict(returned)
        guard_speed = math.sqrt(sum(number(v)**2 for v in row["world_velocity_carla"])) if initial else speed
        if not emergency and guard_speed < .3 and (guarded["brake"] > .05 or guarded["hand_brake"]):
            guarded["steer"] = 0.
        alignment = row["control_transport"]
        next_receipt = commands[alignment["next_command_receipt_sequence"]]
        # HH_260906 - Keep the final abort/driving-end replacement explicit; never pretend it was the policy output.
        replacement = next_receipt["reason"] in ("alignment_failure_abort", "driving_end_brake")
        final = blank_control() if replacement else guarded
        # HH_260906 - These are serialized requests from one object, not tolerant physical readback comparisons.
        check(equal(final, row["next_control"]) and equal(final, next_receipt["requested_control"]), frame, "guarded_final_command_and_ACK")
        if replacement:
            check((next_receipt["reason"] == "alignment_failure_abort" and alignment["status"] == "FAIL") or
                (next_receipt["reason"] == "driving_end_brake" and driving and frame == driving[-1]["frame"]), frame, "final_replacement_reason")
        if row["capture_phase"] == "driving":
            hold_ticks = hold_ticks + 1 if measured["stopped_in_goal"] and previous_stopped else 0
            previous_stopped = measured["stopped_in_goal"]
            if hold_source is None and gp["coast_entry_latched"] is True and measured["stopped_in_goal"] and failure is None:
                hold_source = row["timestamp"]
                check(gp["pilot_transition"] == "zero_pedal_coast_to_service_brake_hold" and row["current_control"]["brake"] == 0., frame, "causal_hold_entry")
        check(gp["hold_ticks"] == hold_ticks and gp["required_hold_ticks"] == 40 and
            abs(gp["hold_duration_sec"] - hold_ticks*.05) <= 1e-9 and gp["complete"] is (hold_ticks >= 40), frame, "unchanged_measured_dwell")
        check(gp["normal_hold_brake"] == 1. and gp["hold_brake_latched"] is (hold_source is not None) and
            gp["hold_brake_source_timestamp"] == hold_source, frame, "unchanged_hold_brake_latch")
        if driving_command and not emergency and failure is None:
            if hold_source is not None:
                check(before["throttle"] == 0. and before["brake"] == 1., frame, "service_brake_hold_pedals")
            elif gp["coast_entry_latched"]:
                check(before["throttle"] == before["brake"] == 0., frame, "zero_pedal_coast")
            elif gp["handoff_elapsed_sec"] is None:
                check(abs(before["throttle"]-.15) <= 1e-6 and before["brake"] == 0., frame, "unchanged_launch_pedals")
            else:
                check(before["throttle"] <= gp["normal_throttle_cap"] + 1e-6 and before["brake"] == 0., frame, "unchanged_normal_pedal_caps")
        decisions[reason] += 1
        if source_driving and previous_eligible is not eligible:
            switches.append({"frame": frame, "timestamp": row["timestamp"], "eligible": eligible, "speed_mps": speed})
            previous_eligible = eligible
        records.append({"frame": frame, "timestamp": row["timestamp"], "capture_phase": row["capture_phase"],
            "initial_engagement": initial, "request": request, "post_existing_guard_control": guarded,
            "final_replacement_reason": next_receipt["reason"] if replacement else None,
            "final_next_control": row["next_control"], "receipt_sequence": next_receipt["sequence"]})
    return {"status": "PASS" if states and not failures else "FAIL", "failures": failures,
        "state_receipt_count": len(records), "decision_counts": dict(decisions), "eligibility_switches": switches,
        "hold_source_timestamp": hold_source, "initial_engagement_source_count": len(initial_rows),
        "requests": records, "physical_application_proven": False, "training_data_approved": False}


def initialization_checks(states, manifest):
    init = manifest["capture_contract"]["agent_initialization"]
    require(init["mode"] == "after_bootstrap" and init["existing_bootstrap_tick_reused"] is True and
        init["additional_world_ticks"] == 0 and init["pid_history_overwritten"] is False and
        init["first_proposed_steering_delta_limit"] == .1 and init["comparison_absolute_tolerance"] == 1e-6,
        "initialization policy changed")
    bootstrap = manifest["capture_contract"]["control_transport"]["bootstrap_observation"]
    construction = init["construction"]
    require(construction["status"] == "PASS", "constructor did not pass")
    for when in ("before", "after"):
        observed = construction[when]
        require(observed["status"] == "PASS" and observed["frame_before"] == observed["frame_after"] == bootstrap["frame"] and
            observed["timestamp"] == bootstrap["timestamp"] and observed["actor_snapshot_transform_carla"] == bootstrap["actor_snapshot_transform_carla"] and
            not ack.mismatched_controls(observed["current_control"], blank_control(), requested_pair=True), "constructor witness differs from bootstrap")
    require(construction["after"]["pid_past_steering"] == 0., "constructor PID history is not zero")
    sources = [row for row in states if row["goal_stop"].get("next_control_starts_driving")]
    if not sources:
        require(not any(row["capture_phase"] == "driving" for row in states), "missing initial source")
        return {"status": "NOT_REACHED", "additional_world_ticks": 0}
    require(len(sources) == 1, "duplicate initial source")
    source, engagement = sources[0], init["engagement"]
    proposed = ack.full_control(engagement["first_proposed_control"])
    before = engagement["before"]
    require(engagement["status"] == "PASS" and engagement["first_command_acknowledged"] is True and
        before["frame_before"] == before["frame_after"] == engagement["frame_after_agent_step"] == source["frame"] and
        before["timestamp"] == source["timestamp"] and before["pid_past_steering"] == 0. and
        before["actor_snapshot_transform_carla"] == source["actor_snapshot_transform_carla"] and
        not ack.mismatched_controls(before["current_control"], source["current_control"], requested_pair=True), "initial source/frame/pose/control differs")
    delta = proposed["steer"] - before["pid_past_steering"]
    require(engagement["first_proposed_steering_delta"] == delta and abs(delta) <= .1+1e-6 and
        equal(proposed, source["goal_stop"]["low_speed_steering"]["input_control"]) and
        equal(engagement["first_sent_control"], source["next_control"]) and
        engagement["first_command_receipt_sequence"] == source["control_transport"]["next_command_receipt_sequence"], "original proposed-control check or first sent command differs")
    return {"status": "PASS", "initial_source_frame": source["frame"], "initial_source_timestamp": source["timestamp"],
        "first_proposed_steering_delta": delta, "additional_world_ticks": 0, "pid_history_overwritten": False}


def summarize_one(case_root, case, plan):
    """HH_260906 - Callable after each owned run; absent/in-progress/failed cases never become successes."""
    validate_plan(plan)
    root, ledger = safe(case_root), {}
    base = dict(case_id=case["id"], arm=case["arm"], training_data_approved=False, research_only=True,
        learned_model_control=False, vehicle_control_approved=False, native_state_count=None, camera_anchor_count=None)
    if not root.exists():
        return {**base, "status": "NOT_RUN"}, [], [], []
    if not (root/"owner_result.json").exists():
        return {**base, "status": "NOT_FINALIZED", "notice": "No finalized owner receipt; no raw payload audited."}, [], [], []
    owner_plan = checked(root/"owner_plan.json", ledger)
    owner = checked(root/"owner_result.json", ledger)
    require(owner_plan["source_sha256"] == plan["source_sha256"] and owner_plan["source_bytes_archived"] is True and
        owner_plan["bounds_source_bytes_archived"] is True, "prospective/execution source closure differs")
    require(owner["source_bytes_unchanged_and_archived"] is True and owner["source_checks"] == {name: True for name in plan["source_sha256"]}, "owner source-after proof failed")
    require(instant(owner_plan["planned_at_utc"]) >= instant(plan["declared_at_utc"]), "run preceded prospective declaration")
    started = checked(root/"owner_started.json", ledger)
    require(instant(started["started_at_utc"]) >= instant(owner_plan["planned_at_utc"]) and
        instant(owner["completed_at_utc"]) >= instant(started["started_at_utc"]), "owner timestamps are reversed")
    lifecycle = {}
    for stage in ("ready", "after_capture"):
        observed = checked(root/"lifecycle"/(stage+".json"), ledger)
        require(observed["status"] == "PASS" and observed["stage"] == stage and observed["mode"] == "running" and
            observed["read_only"] is True and observed["owner_pid"] == observed["owner_pgid"] == started["server_pid"] == started["server_pgid"] and
            observed["generation_id"] == f"expert_{started['server_pid']}" and observed["expected_map"] == observed["active_map_basename"] == "Town07" and
            observed["host"] == "127.0.0.1" and observed["port"] == 2100, "lifecycle owner/map/frame witness differs")
        require(instant(owner_plan["planned_at_utc"]) <= instant(observed["checked_at"]) <= instant(owner["completed_at_utc"]), "lifecycle time outside owned execution")
        lifecycle[stage] = observed
    command = case["command"]
    require(command[:2] == ["bash", "scripts/e2e/run_owned_carla_expert_trial.sh"] and
        safe(ROOT/command[2]) == root and command.count("--") == 1, "planned owned command/root differs")
    options = command[command.index("--")+1:]
    expected_argv = [str(root/"episode"), str(safe(ROOT/command[3])), "--host", "127.0.0.1", "--port", "2100", *options]
    require(owner_plan["collector_argv"] == expected_argv and owner_plan["capture_mode"] == "expert" and
        owner_plan["map"] == "Town07" and owner_plan["quality"] == "Epic" and owner_plan["port"] == 2100 and
        owner_plan["host"] == "127.0.0.1", "owned route/argv/endpoint differs")
    for name, pin in plan["source_sha256"].items():
        checked(ROOT/name, ledger, pin, binary=True)
        checked(root/"provenance"/name, ledger, pin, binary=True)
    for name, pin in plan["input_sha256"].items():
        checked(ROOT/name, ledger, pin, binary=True)
    for name, pin in plan["audit_math_sha256"].items():
        checked(ROOT/name, ledger, pin, binary=True)
    original, native = scalar.summarize_trial(root, None)
    for item in original["source_manifest"]:
        checked(root/item["path"], ledger, item["sha256"], binary=True)
    result = {**base, "status": "FAILED_CAPTURE_NOT_ADMITTED", "scalar": original,
        "owner_exit_code": owner["exit_code"], "source_provenance_checked": True, "metadata_sha256": ledger}
    images, futures, states = [], [], []
    if not original["raw_data_available"]:
        return result, images, futures, states
    directory = "episode" if (root/"episode").exists() else "episode.partial"
    episode = root/directory
    manifest = checked(episode/"manifest.json", ledger)
    route = checked(episode/"route.json", ledger)
    states = checked(episode/"states.jsonl", ledger, jsonl=True)
    cameras = checked(episode/"camera_frames.jsonl", ledger, jsonl=True)
    receipts = checked(episode/"control_receipts.jsonl", ledger, jsonl=True)
    configuration = plan["protocol_configuration_by_arm"][case["arm"]]
    require(configuration_equal({key: manifest["capture_contract"]["goal_stop_profile"].get(key) for key in configuration}, configuration), "recorded v6 configuration differs")
    require(manifest["result"]["training_data_approved"] is False and manifest["result"]["development_only"] is True,
        "raw v6 diagnostic denials changed")
    require(manifest["capture_contract"]["stopped_brake_steering_suppressed"] is True and
        manifest["capture_contract"]["control_transport"]["mode"] == "acknowledged_batch", "inherited control guards changed")
    for field, prefix in (("source_route", "route"), ("mapping_file", "mapping"), ("calibration_file", "calibration")):
        path = safe(manifest["provenance"][field])
        relative = str(path.relative_to(ROOT))
        require(relative in plan["input_sha256"] and manifest["provenance"][prefix+"_sha256"] == plan["input_sha256"][relative], "captured fixed input differs")
    settings = manifest.get("runtime", {}).get("capture_world_settings")
    expected_settings = dict(synchronous_mode=True, fixed_delta_seconds=.05, no_rendering_mode=False,
        substepping=True, max_substep_delta_time=.01, max_substeps=10)
    require(equal(settings, expected_settings), "recorded six-field physics settings differ")
    result.update(native_state_count=len(states), camera_anchor_count=len(cameras), original_episode_directory=directory,
        recorded_world_settings={key: manifest["runtime"].get(key) for key in ("original_world_settings", "capture_world_settings")},
        world_restoration_claim="Collector cleanup receipt only; no independent eleven-field restoration proof.")
    if states:
        bootstrap = manifest["capture_contract"]["control_transport"]["bootstrap_observation"]
        require(lifecycle["ready"]["snapshot_frame"] < bootstrap["frame"] < states[0]["frame"] and
            lifecycle["after_capture"]["snapshot_frame"] >= states[-1]["frame"] and
            lifecycle["after_capture"]["snapshot_elapsed_seconds"] >= states[-1]["timestamp"], "lifecycle snapshots do not bracket captured frames/times")
        timeline = raw.measured_timeline(states)
        failures = []
        def stage(name, function):
            try:
                result[name] = function()
            except (ValueError, KeyError, TypeError, OSError) as error:
                result[name] = {"status": "FAILED_VALIDATION", "error": f"{type(error).__name__}: {error}"}
                failures.append(name)
        stage("initialization", lambda: initialization_checks(states, manifest))
        stage("transport", lambda: ack.analyze_transport(states, receipts,
            manifest["capture_contract"]["control_transport"]["bootstrap_observation"], wheelbase=2.85))
        stage("steering_policy", lambda: steering_protocol(states, native["native_states"], manifest, receipts, case["arm"]))
        try:
            pixels, images, anchors = raw.camera_audit(episode, manifest, cameras, timeline)
            result["camera_pixels"] = pixels
            future, futures = raw.measured_future_audit(timeline, cameras, route, anchors)
            result["measured_future"] = future
        except (ValueError, KeyError, TypeError, OSError) as error:
            result["future_or_camera_error"] = f"{type(error).__name__}: {error}"
            failures.append("camera_or_future")
        for row in images:
            checked(episode/row["path"], ledger, row["sha256"], binary=True)
        execution_clear = (owner["exit_code"] == 0 and original["raw_quality_candidate"] and not failures and
            all(result[name].get("status") == "PASS" for name in ("initialization", "transport", "steering_policy")) and
            result.get("camera_pixels", {}).get("all_pixel_and_metadata_checks_clear") is True)
        future = result.get("measured_future", {})
        full = future.get("full_64_point_anchors", {})
        result["all_original_future_metrics_clear"] = (bool(full.get("sample_count")) and all(
            metric["violating_step_count"] == 0 for metric in full.get("metrics", {}).values()) and
            set(future.get("disposition_counts", {})) <= {"full_64_point_anchor", "tail_label_context_only"})
        result["execution_and_scalar_pixel_protocol_clear"] = bool(execution_clear)
        result["status"] = "COMPLETE_DIAGNOSTIC_NOT_ADMITTED" if execution_clear else "FAILED_CAPTURE_OR_QA_NOT_ADMITTED"
    for name, pin in tuple(ledger.items()):
        checked(name, {}, pin, binary=True)
    return result, images, futures, states


def compare_physical(first, second, *, warmup_only=False):
    if not first or not second:
        return {"status": "UNAVAILABLE", "comparison_performed": False}
    a = [row for row in first if row["capture_phase"] == "stationary_warmup"] if warmup_only else first
    b = [row for row in second if row["capture_phase"] == "stationary_warmup"] if warmup_only else second
    fields = PHYSICAL_FIELDS if warmup_only else (*PHYSICAL_FIELDS, "next_control")
    mismatches, maximum = [], {}
    for index, (left, right) in enumerate(zip(a, b)):
        changed = [key for key in fields if not equal(left[key], right[key])]
        if changed:
            mismatches.append({"native_index": index, "first_frame": left["frame"], "second_frame": right["frame"], "fields": changed})
        for key in fields:
            if type(left[key]) in (int, float) and type(right[key]) in (int, float):
                maximum[key] = max(maximum.get(key, 0.), abs(left[key]-right[key]))
    return {"status": "EXACT_MATCH" if len(a) == len(b) and not mismatches else "DIFFERENT",
        "comparison_performed": True, "first_state_count": len(a), "second_state_count": len(b),
        "compared_native_count": min(len(a), len(b)), "mismatched_native_count": len(mismatches),
        "first_mismatch": mismatches[0] if mismatches else None, "mismatches": mismatches,
        "maximum_absolute_scalar_difference": maximum, "exact_fields": list(fields),
        "excluded": ["actor/server IDs", "absolute frame/timestamp", "v6 policy/arm metadata", "next_control at warmup-only treatment boundary"]}


def curvature_incidence(states, futures):
    # HH_260906 - Preserve overlapping window incidences and every unique endpoint; group by observed state, not inferred cause.
    groups = {}
    transitions = [{"frame": row["frame"], "timestamp": row["timestamp"], "transition": row["goal_stop"]["pilot_transition"]}
        for row in states if row["goal_stop"].get("pilot_transition")]
    for anchor in futures:
        for step in anchor.get("diagnostic", {}).get("steps", []):
            if step["metrics"]["xy_curvature"][1]:
                tick = step["episode_relative_100ms_tick"]
                groups.setdefault(tick, []).append({"anchor_frame": anchor["frame"], "point_index": step["index"],
                    "target_timestamp_ns": step["target_timestamp_ns"], "curvature_rad_per_m": step["metrics"]["xy_curvature"][0]})
    witnesses, phases = [], {}
    for tick, occurrences in sorted(groups.items()):
        index = tick*2
        require(2 <= index < len(states), "curvature tick has no native endpoint")
        endpoint, start = states[index], states[index-2]
        end_ns = round(endpoint["timestamp"]*1e9)
        require(all(abs(item["target_timestamp_ns"]-end_ns) <= 500 for item in occurrences), "curvature endpoint timestamp binding differs")
        pilot = endpoint["goal_stop"]["pilot_state"]
        counts = phases.setdefault(pilot, {"unique_endpoint_count": 0, "overlapping_window_step_count": 0})
        counts["unique_endpoint_count"] += 1
        counts["overlapping_window_step_count"] += len(occurrences)
        witnesses.append({"episode_relative_100ms_tick": tick, "native_endpoint_frame": endpoint["frame"],
            "native_endpoint_timestamp": endpoint["timestamp"], "capture_phase": endpoint["capture_phase"], "pilot_state": pilot,
            "endpoint_planar_speed_mps": math.hypot(endpoint["vx"], endpoint["vy"]),
            "native_100ms_displacement_m": math.hypot(endpoint["x"]-start["x"], endpoint["y"]-start["y"]),
            "native_100ms_yaw_change_rad": math.atan2(math.sin(endpoint["yaw"]-start["yaw"]), math.cos(endpoint["yaw"]-start["yaw"])),
            "current_control": endpoint["current_control"], "overlapping_window_step_count": len(occurrences),
            "first_point_failure_count": sum(row["point_index"] == 0 for row in occurrences),
            "transition_sources_inside_native_interval": [row for row in transitions if start["timestamp"] <= row["timestamp"] < endpoint["timestamp"]],
            "original_failure_retained": True, "occurrences": occurrences})
    return {"unique_endpoint_count": len(witnesses), "overlapping_window_step_count": sum(map(len, groups.values())),
        "by_observed_pilot_state": phases, "transition_sources": transitions, "witnesses": witnesses,
        "interpretation": "Grouping and boundary overlap are observational, not a physical-cause assignment. All original flags remain failures."}


def comparison_verdict(trials):
    rows = {}
    for trial in trials:
        full = trial.get("measured_future", {}).get("full_64_point_anchors", {})
        metric = full.get("metrics", {}).get("xy_curvature", {})
        count, denominator = metric.get("violating_anchor_count"), full.get("sample_count", 0)
        rows[trial["case_id"]] = {"full_anchor_count": denominator, "curvature_violating_anchor_count": count,
            "curvature_violating_anchor_fraction": count/denominator if count is not None and denominator else None,
            "curvature_overlapping_failing_step_count": metric.get("violating_step_count"),
            "curvature_unique_100ms_endpoint_count": metric.get("unique_episode_relative_100ms_violation_tick_count"),
            "maximum_assessed_curvature_rad_per_m": (metric.get("maximum") or {}).get("value"),
            "all_metric_violating_anchor_counts": {name: item["violating_anchor_count"] for name, item in full.get("metrics", {}).items()}}
    complete = all(t["status"] == "COMPLETE_DIAGNOSTIC_NOT_ADMITTED" and rows[t["case_id"]]["full_anchor_count"] for t in trials)
    cross = []
    if any(row["full_anchor_count"] for row in rows.values()):
        for b in (CASES[1][0], CASES[2][0]):
            for a in (CASES[0][0], CASES[3][0]):
                first, second = rows[a], rows[b]
                if not first["full_anchor_count"] or not second["full_anchor_count"]:
                    continue
                no_other = all(second["all_metric_violating_anchor_counts"][key] <= value and
                    second["all_metric_violating_anchor_counts"][key]/second["full_anchor_count"] <= value/first["full_anchor_count"]
                    for key, value in first["all_metric_violating_anchor_counts"].items() if key != "xy_curvature")
                count_delta = second["curvature_violating_anchor_count"] - first["curvature_violating_anchor_count"]
                rate_delta = second["curvature_violating_anchor_fraction"] - first["curvature_violating_anchor_fraction"]
                maximum_delta = second["maximum_assessed_curvature_rad_per_m"] - first["maximum_assessed_curvature_rad_per_m"]
                cross.append({"native_case": a, "zero_case": b, "zero_minus_native_failing_anchor_count": count_delta,
                    "zero_minus_native_failing_anchor_fraction": rate_delta, "other_original_label_gates_not_regressed": no_other,
                    "zero_minus_native_maximum_curvature_rad_per_m": maximum_delta,
                    "worst_observed_curvature_increased": maximum_delta > 0,
                    "favorable_count_and_rate": count_delta < 0 and rate_delta < 0 and no_other})
    favorable = complete and all(row["favorable_count_and_rate"] for row in cross)
    return {"status": "INCOMPLETE_OR_FAILED_NO_COMPARATIVE_VERDICT" if not complete else
            "FAVORABLE_CONDITION_SPECIFIC_SIGNAL_NOT_ADMITTED" if favorable else "MIXED_OR_NO_CONSISTENT_IMPROVEMENT_NOT_ADMITTED",
        "per_case_counts_and_rates": rows, "observed_cross_arm_comparisons": cross,
        "required_cross_arm_comparison_count": 4, "available_cross_arm_comparison_count": len(cross),
        "interpretation": "A favorable count/rate signal can coexist with a worse maximum. Remaining original failures are unresolved and never admitted.",
        "training_data_approved": False, "criterion": "Both zero repeats improve count AND rate against BOTH native repeats, without other original gate regressions; no cherry-picking."}


def execution_attempts(path, expected, plan, campaign, ledger):
    require(path is not None and expected is not None, "attempt companion path and SHA must be supplied together")
    document = checked(path, ledger, expected)
    require(document["schema"] == "carla.low_speed_steering_execution_attempts.v1" and document["plan_sha256"] == PLAN_SHA256 and
        document["training_data_approved"] is False and document["learned_model_control"] is False, "attempt companion scope/plan differs")
    instant(document["recorded_at_utc"])
    require(len(document["records"]) == 4, "all four planned attempt records required")
    for record, case in zip(document["records"], plan["cases"]):
        require(record["case_id"] == case["id"] and record["arm"] == case["arm"] and
            record["command"] == case["command"] and type(record["attempt_count"]) is int and record["attempt_count"] == 1,
            "attempt identity/command/count differs")
        if record["status"] == "PRELAUNCH_BLOCKED_NOT_CAPTURED":
            require(not (campaign/case["id"]).exists() and record["exit_code"] != 0 and
                all(record[key] is False for key in ("output_directory_created", "owner_started", "collector_started", "automatic_retry")) and
                isinstance(record["observed_output"], str) and bool(record["observed_output"]), "prelaunch block contradicts captured evidence")
        else:
            require(record["status"] == "CAPTURE_COMPLETED" and
                safe(ROOT/record["owner_result_path"]) == campaign/case["id"]/"owner_result.json" and
                (campaign/case["id"]/"owner_result.json").is_file(), "completed attempt lacks its owned result")
    return document


def run(campaign_root, plan_path, plan_sha256, output_dir=None, *, execution_attempts_path=None, execution_attempts_sha256=None):
    campaign, plan_path, ledger = safe(campaign_root), safe(plan_path), {}
    require(plan_sha256 == PLAN_SHA256, "only the frozen prospective v6 plan is supported")
    plan = checked(plan_path, ledger, plan_sha256)
    validate_plan(plan)
    attempts = None
    if execution_attempts_path is not None or execution_attempts_sha256 is not None:
        attempts = execution_attempts(execution_attempts_path, execution_attempts_sha256, plan, campaign, ledger)
    require(attempts is not None or not (campaign/"execution_attempts.json").exists(), "existing execution-attempt companion must be hash-bound, not ignored")
    for category in ("source_sha256", "input_sha256", "audit_math_sha256"):
        for name, pin in plan[category].items():
            checked(ROOT/name, ledger, pin, binary=True)
    output = None if output_dir is None else safe(output_dir)
    if output is not None:
        require(not output.exists() and output.parent.is_dir(), "fresh output with existing parent required")
        for case_id, _ in CASES:
            case_root = campaign/case_id
            require(output != case_root and case_root not in output.parents and output not in case_root.parents,
                "output must be disjoint from raw cases")
    sources = raw.source_identity()
    sources["files"][str(Path(__file__).resolve().relative_to(ROOT))] = scalar.sha(Path(__file__))
    sources["files"]["scripts/e2e/carla_goal_stop_profile.py"] = scalar.sha(ROOT/"scripts/e2e/carla_goal_stop_profile.py")
    trials, images, futures, policy_witnesses, physical = [], [], [], [], {}
    for case in plan["cases"]:
        try:
            report, pictures, windows, states = summarize_one(campaign/case["id"], case, plan)
        except (ValueError, KeyError, TypeError, OSError) as error:
            report, pictures, windows, states = ({"case_id": case["id"], "arm": case["arm"], "status": "FAILED_VALIDATION",
                "error": f"{type(error).__name__}: {error}", "training_data_approved": False,
                "native_state_count": None, "camera_anchor_count": None}, [], [], [])
        trials.append(report)
        if attempts is not None:
            attempt = attempts["records"][len(trials)-1]
            report["execution_attempt"] = attempt
            if attempt["status"] == "PRELAUNCH_BLOCKED_NOT_CAPTURED":
                require(report["status"] == "NOT_RUN", "prelaunch block has unexpected raw evidence")
                report["status"] = "PRELAUNCH_BLOCKED_NOT_CAPTURED"
        if states:
            report["curvature_incidence"] = curvature_incidence(states, windows)
        physical[case["id"]] = states
        policy = report.get("steering_policy", {})
        requests = policy.pop("requests", [])
        if policy:
            policy.update(witness_file="steering_policy_witnesses.jsonl", witness_case_id=case["id"], witness_count=len(requests))
            require("state_receipt_count" not in policy or policy["state_receipt_count"] == len(requests), "policy witness count differs")
        policy_witnesses.extend({"case_id": case["id"], "arm": case["arm"], **row} for row in requests)
        images.extend({"case_id": case["id"], "arm": case["arm"], **row} for row in pictures)
        futures.extend({"case_id": case["id"], "arm": case["arm"], **row} for row in windows)
    pairs = {"native_repeat_A1_A2": compare_physical(physical[CASES[0][0]], physical[CASES[3][0]]),
        "zero_repeat_B1_B2": compare_physical(physical[CASES[1][0]], physical[CASES[2][0]]),
        "before_treatment_A1_B1": compare_physical(physical[CASES[0][0]], physical[CASES[1][0]], warmup_only=True),
        "before_treatment_B2_A2": compare_physical(physical[CASES[2][0]], physical[CASES[3][0]], warmup_only=True)}
    pairs["prospective_primary_comparison"] = comparison_verdict(trials)
    summary = {"schema": SCHEMA, "status": "DIAGNOSED_NOT_ADMITTED", "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "plan_sha256": plan_sha256, "plan_path": str(plan_path), "source_sha256": sources,
        "execution_attempts_sha256": execution_attempts_sha256, "execution_attempts_evidence": attempts,
        "planned_case_count": 4, "attempted_case_count": len(attempts["records"]) if attempts is not None else sum(row["status"] != "NOT_RUN" for row in trials),
        "training_data_approved": False, "research_only": True, "dataset_export": False, "target_retiming": False,
        "learned_model_control": False, "vehicle_control_approved": False, "cases": trials,
        "case_status_counts": dict(Counter(row["status"] for row in trials)),
        "all_four_completed_diagnostics": all(row["status"] == "COMPLETE_DIAGNOSTIC_NOT_ADMITTED" for row in trials),
        "audited_native_state_count": sum(row["native_state_count"] or 0 for row in trials),
        "audited_camera_anchor_count": sum(row["camera_anchor_count"] or 0 for row in trials),
        "audited_original_jpeg_count": len(images), "future_anchor_record_count": len(futures),
        "steering_policy_witness_count": len(policy_witnesses), "steering_policy_witness_file": "steering_policy_witnesses.jsonl",
        "limitations": ["Every original full future gate is unchanged; improvement does not grant training admission.",
            "Four matched runs in one condition do not establish broad robustness or a unique physical cause.",
            "Output steering changes only; PID history is not reset and threshold switching is retained.",
            "A/B physical common-prefix comparison excludes the last warmup next command, where treatment first occurs.",
            "Exact physical repeat checks exclude clocks/IDs; no numeric tolerance, retiming or trajectory normalization is used.",
            "Inherited six-field settings and cleanup receipts are not independent eleven-field restoration or substep torque proof."]}
    for name, pin in sources["files"].items():
        checked(ROOT/name, {}, pin, binary=True)
    checked(plan_path, {}, plan_sha256, binary=True)
    for name, pin in ledger.items():
        checked(name, {}, pin, binary=True)
    for trial in trials:
        for name, pin in trial.get("metadata_sha256", {}).items():
            checked(name, {}, pin, binary=True)
    if output is not None:
        output.mkdir()
        for name, value in (("summary.json", summary), ("paired_comparisons.json", pairs)):
            with (output/name).open("xb") as stream:
                stream.write(json.dumps(value, indent=2, allow_nan=False).encode()+b"\n")
        for name, rows in (("image_inventory.jsonl", images), ("future_windows.jsonl", futures), ("steering_policy_witnesses.jsonl", policy_witnesses)):
            with (output/name).open("xb") as stream:
                for row in rows:
                    stream.write(encoded(row)+b"\n")
        with (output/"SHA256SUMS").open("x") as stream:
            stream.write("".join(f"{scalar.sha(path)}  {path.name}\n" for path in sorted(output.iterdir()) if path.is_file() and path.name != "SHA256SUMS"))
    return summary, pairs


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--campaign-root", required=True, type=Path)
    parser.add_argument("--plan", required=True, type=Path)
    parser.add_argument("--plan-sha256", required=True)
    parser.add_argument("--execution-attempts", type=Path)
    parser.add_argument("--execution-attempts-sha256")
    choice = parser.add_mutually_exclusive_group(required=True)
    choice.add_argument("--output-dir", type=Path)
    choice.add_argument("--verify-only", action="store_true")
    args = parser.parse_args(argv)
    try:
        summary, _ = run(args.campaign_root, args.plan, args.plan_sha256, args.output_dir,
            execution_attempts_path=args.execution_attempts, execution_attempts_sha256=args.execution_attempts_sha256)
    except (ValueError, KeyError, TypeError, OSError) as error:
        parser.exit(1, f"v6 summary failed: {error}\n")
    print(json.dumps({key: summary[key] for key in ("status", "case_status_counts", "audited_native_state_count", "audited_camera_anchor_count")}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
