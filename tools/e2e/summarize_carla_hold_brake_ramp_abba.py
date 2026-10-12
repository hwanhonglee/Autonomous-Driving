#!/usr/bin/env python3
"""HH_260906 - Audit the fresh v7 hold-brake command ramp without changing original raw QA or admission."""
from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict, replace
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import struct
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from scripts.e2e import summarize_carla_low_speed_steering_abba as common

SCHEMA = "carla.hold_brake_ramp_abba_summary.v1"
PROFILE = "comfortable_v7_hold_brake_ramp"
CASES = (("01_immediate_A1", "immediate"), ("02_ramp_B1", "ramp"), ("03_ramp_B2", "ramp"), ("04_immediate_A2", "immediate"))
PLAN_SHA256 = "a7708081681efb7dddf9fc52e547a0950b500dcb2c36f5be9f1deaa2402d0a48"
ORCHESTRATION_PATHS = {"scripts/e2e/probe_carla_port_readiness.py", *{
    "artifacts/training/2026-09-14/hold_brake_ramp_abba_v7/"+name for name in ("run_case.py", "render_evidence.py")}}
COMMON_SOURCE_SHA256 = "53b42cf4e1be70f64ead8c6d143aed4dabc46e5f486407f0bacb9f70e6410da1"
safe, checked, equal, configuration_equal, encoded, instant = common.safe, common.checked, common.equal, common.configuration_equal, common.encoded, common.instant
raw, ack, scalar, profiles = common.raw, common.ack, common.scalar, common.profiles
require, number = scalar.require, scalar.number


def required_sha(value):
    require(isinstance(value, str) and len(value) == 64 and set(value) <= set("0123456789abcdef"), "mandatory SHA-256 reference missing or invalid")
    return value


def float32(value):
    value = number(value)
    try:
        result = struct.unpack("!f", struct.pack("!f", value))[0]
    except OverflowError as error:
        raise scalar.EvidenceError("normal command cannot be represented as float32") from error
    require(math.isfinite(result), "normal command is nonfinite after float32 storage")
    return result


def requested_hold_brake(mode, source_timestamp, hold_source_timestamp):
    """HH_260906 - Use measured command-source time with the prospective +dt convention; never round to a tick count."""
    require(mode in ("immediate", "ramp"), "unknown hold-brake arm")
    source, start = number(source_timestamp), number(hold_source_timestamp)
    require(source >= start, "normal command precedes the measured hold latch")
    return 1. if mode == "immediate" else min(1., (source-start+.05)/.5)


def validate_plan(plan):
    require(plan["schema"] == "carla.hold_brake_ramp_abba_plan.v1" and
        [(case["id"], case["arm"]) for case in plan["cases"]] == list(CASES), "fixed prospective v7 ABBA plan required")
    require(plan["maximum_attempts_per_revision"] == 4 and plan["automatic_retry"] is False and
        all(type(case["maximum_attempts"]) is int and case["maximum_attempts"] == 1 for case in plan["cases"]), "attempt budget/retry policy differs")
    require(set(plan["source_sha256"]) == common.EXECUTION_SOURCES and set(plan["input_sha256"]) == common.INPUT_PATHS and
        set(plan["audit_math_sha256"]) == common.AUDIT_MATH_PATHS, "source/input/audit closure differs")
    require(set(plan["orchestration_sha256"]) == ORCHESTRATION_PATHS, "orchestration closure differs")
    for category in ("source_sha256", "input_sha256", "audit_math_sha256", "orchestration_sha256"):
        require(all(isinstance(pin, str) and len(pin) == 64 and set(pin) <= set("0123456789abcdef") for pin in plan[category].values()), "invalid evidence pin")
    require(set(plan["protocol_configuration_by_arm"]) == {"immediate", "ramp"}, "both v7 protocol configurations required")
    for arm in ("immediate", "ramp"):
        expected = asdict(replace(profiles.HoldBrakeRampDevelopmentGoalStopConfig(), hold_brake_mode=arm))
        require(configuration_equal(plan["protocol_configuration_by_arm"][arm], expected), "v7 protocol configuration differs")
    require(all(plan[key] is False for key in ("automatic_retry", "training_data_approved", "learned_model_control",
        "vehicle_control_approved", "dataset_export", "target_retiming", "gate_changes")), "prospective scope/denials differ")
    instant(plan["declared_at_utc"])


def before_hold_comparison(first, second):
    """HH_260906 - Compare the full physical prefix through the first hold source, but not its already-treated next command."""
    def prefix(rows):
        index = next((i for i, row in enumerate(rows) if row["goal_stop"].get("pilot_transition") == "zero_pedal_coast_to_service_brake_hold"), None)
        return None if index is None else rows[:index+1]
    first, second = prefix(first), prefix(second)
    if first is None or second is None:
        return {"status": "UNAVAILABLE", "comparison_performed": False}
    mismatches = []
    for index, (a, b) in enumerate(zip(first, second)):
        fields = [key for key in common.PHYSICAL_FIELDS if not equal(a[key], b[key])]
        if index < min(len(first), len(second))-1 and not equal(a["next_control"], b["next_control"]):
            fields.append("next_control_before_hold_source")
        if fields:
            mismatches.append({"native_index": index, "first_frame": a["frame"], "second_frame": b["frame"], "fields": fields})
    return {"status": "EXACT_MATCH" if len(first) == len(second) and not mismatches else "DIFFERENT",
        "comparison_performed": True, "first_prefix_state_count": len(first), "second_prefix_state_count": len(second),
        "compared_native_count": min(len(first), len(second)), "mismatched_native_count": len(mismatches),
        "first_mismatch": mismatches[0] if mismatches else None, "mismatches": mismatches,
        "physical_fields": list(common.PHYSICAL_FIELDS),
        "hold_source_next_controls": [first[-1]["next_control"], second[-1]["next_control"]],
        "excluded": ["absolute clocks/frame IDs", "arm/policy metadata", "only the hold-source next_control where brake treatment first occurs"],
        "normalization_or_retiming_performed": False}


def verify_owner(root, case, plan, ledger):
    owner_plan = checked(root/"owner_plan.json", ledger)
    owner = checked(root/"owner_result.json", ledger)
    started = checked(root/"owner_started.json", ledger)
    require(owner_plan["source_sha256"] == plan["source_sha256"] and owner_plan["source_bytes_archived"] is True and
        owner_plan["bounds_source_bytes_archived"] is True and owner["source_bytes_unchanged_and_archived"] is True and
        owner["source_checks"] == {name: True for name in plan["source_sha256"]}, "execution source/after proof differs")
    require(instant(plan["declared_at_utc"]) <= instant(owner_plan["planned_at_utc"]) <= instant(started["started_at_utc"]) <=
        instant(owner["completed_at_utc"]), "execution preceded declaration or reversed its clock")
    command = case["command"]
    require(command[:2] == ["bash", "scripts/e2e/run_owned_carla_expert_trial.sh"] and command.count("--") == 1 and
        safe(ROOT/command[2]) == root, "case command/root differs")
    argv = [str(root/"episode"), str(safe(ROOT/command[3])), "--host", "127.0.0.1", "--port", "2100", *command[command.index("--")+1:]]
    require(owner_plan["collector_argv"] == argv and owner_plan["capture_mode"] == "expert" and owner_plan["host"] == "127.0.0.1" and
        owner_plan["port"] == 2100 and owner_plan["map"] == "Town07" and owner_plan["quality"] == "Epic", "owned command/condition differs")
    lifecycle = {}
    for stage in ("ready", "after_capture"):
        receipt = checked(root/"lifecycle"/(stage+".json"), ledger)
        require(receipt["status"] == "PASS" and receipt["stage"] == stage and receipt["mode"] == "running" and receipt["read_only"] is True and
            receipt["owner_pid"] == receipt["owner_pgid"] == started["server_pid"] == started["server_pgid"] and
            receipt["generation_id"] == f"expert_{started['server_pid']}" and receipt["expected_map"] == receipt["active_map_basename"] == "Town07" and
            receipt["host"] == "127.0.0.1" and receipt["port"] == 2100 and
            instant(owner_plan["planned_at_utc"]) <= instant(receipt["checked_at"]) <= instant(owner["completed_at_utc"]), "lifecycle identity/time differs")
        lifecycle[stage] = receipt
    for name, pin in plan["source_sha256"].items():
        checked(ROOT/name, ledger, pin, binary=True)
        checked(root/"provenance"/name, ledger, pin, binary=True)
    for category in ("input_sha256", "audit_math_sha256"):
        for name, pin in plan[category].items():
            checked(ROOT/name, ledger, pin, binary=True)
    return owner, lifecycle


def hold_and_steering_protocol(states, native_rows, manifest, receipts, arm):
    """HH_260906 - Replay measured latch time, pre-emergency normal pedals, unchanged emergency authority, output steering, guard and serialized ACK."""
    require(arm in ("immediate", "ramp"), "unknown brake arm")
    require(len(states) == len(native_rows) and all(a["frame"] == b["frame"] and a["timestamp"] == b["timestamp"]
        for a, b in zip(states, native_rows)), "policy/native row count/frame/time mismatch")
    commands = {receipt["sequence"]: receipt for receipt in receipts}
    require(len(commands) == len(receipts), "duplicate command receipt sequence")
    initial_rows = [row for row in states if row["goal_stop"].get("next_control_starts_driving") is True]
    driving = [row for row in states if row["capture_phase"] == "driving"]
    require(not driving or len(initial_rows) == 1, "driving requires exactly one initial command source")
    failures, witnesses, decisions, ramp = [], [], Counter(), []
    hold_source, hold_ticks, previous_stopped, previous_failure = None, 0, False, None
    event_totals = {name: sum(len(row[name]) for row in states) for name in ("collision", "lane_invasion")}
    previous_events = {name: 0 for name in event_totals}
    emergency_brake = number(manifest["capture_contract"]["goal_stop_profile"]["effective_control"]["unchanged_basic_agent_emergency_brake"])
    require(0. <= emergency_brake <= 1., "invalid original emergency brake")
    def check(condition, frame, name):
        if not condition:
            failures.append({"frame": frame, "check": name})
    for row, measured in zip(states, native_rows):
        gp, frame, stamp = row["goal_stop"], row["frame"], row["timestamp"]
        request, normal = gp["low_speed_steering"], gp["hold_brake_control"]
        initial = gp.get("next_control_starts_driving") is True
        source_driving = row["capture_phase"] == "driving" or initial
        driving_command = initial or (row["capture_phase"] == "driving" and gp["termination_reason"] is None)
        speed = math.hypot(number(row["vx"]), number(row["vy"]))
        failure = gp.get("pilot_failure_reason")
        emergency = gp["emergency_override_count"] > 0 or failure == PROFILE+"_emergency_override"
        eligible = driving_command and source_driving and not emergency and failure is None and speed <= .75
        reason = ("emergency_unchanged" if emergency else "governor_failure_unchanged" if failure is not None
            else "not_driving_command_unchanged" if not (driving_command and source_driving)
            else "above_threshold_unchanged" if not eligible else "zero_steering_requested")
        if row["capture_phase"] == "driving":
            hold_ticks = hold_ticks+1 if measured["stopped_in_goal"] and previous_stopped else 0
            previous_stopped = measured["stopped_in_goal"]
            # HH_260906 - Update can latch before this same observation's event monitor or later agent emergency fails the case.
            latch_stage_clear = previous_failure is None and failure in (None, PROFILE+"_observed_event_abort", PROFILE+"_emergency_override")
            if hold_source is None and gp["coast_entry_latched"] is True and measured["stopped_in_goal"] and latch_stage_clear:
                hold_source = stamp
                check(gp["pilot_transition"] == "zero_pedal_coast_to_service_brake_hold" and row["current_control"]["brake"] == 0., frame, "causal_hold_entry")
        check(gp["hold_ticks"] == hold_ticks and gp["required_hold_ticks"] == 40 and
            abs(gp["hold_duration_sec"]-hold_ticks*.05) <= 1e-9 and gp["complete"] is (hold_ticks >= 40), frame, "unchanged_measured_dwell")
        check(gp["normal_hold_brake"] == 1. and gp["hold_brake_latched"] is (hold_source is not None) and
            gp["hold_brake_source_timestamp"] == hold_source, frame, "unchanged_hold_latch")
        check(gp["profile_id"] == PROFILE and gp["training_data_approved"] is False and gp["measured_timestamp"] == stamp and
            gp["measured_speed_mps"] == speed and gp["low_speed_steering_driving_enabled"] is source_driving, frame, "governor_source_binding")
        expected = dict(schema="carla.low_speed_steering_request.v1", mode="zero", source_timestamp=stamp,
            source_measured_speed_mps=speed, governor_driving_enabled=source_driving, driving_command=driving_command,
            threshold_mps=.75, emergency_override_present=emergency, governor_failure_reason=failure,
            eligible=eligible, override_requested=eligible, decision=reason,
            control_stage="after_original_agent_proposal_checks_before_existing_stopped_brake_guard",
            physical_application_proven=False, training_data_approved=False, research_only=True)
        check(all(key in request and equal(request[key], value) for key, value in expected.items()), frame, "steering_source_decision_scope")
        expected_normal = dict(schema="carla.hold_brake_control_request.v1", mode=arm, source_timestamp=stamp,
            hold_source_timestamp=hold_source, hold_latched=hold_source is not None, ramp_seconds=.5, native_dt_seconds=.05,
            normal_control_executed=driving_command, driving_command=driving_command, superseded_by_emergency=emergency,
            control_stage="LocalPlanner_normal_control_before_BasicAgent_emergency_and_output_steering",
            physical_application_proven=False, training_data_approved=False, research_only=True)
        check(all(key in normal and equal(normal[key], value) for key, value in expected_normal.items()), frame, "normal_source_execution_scope")
        mathematical, normal_returned = None, None
        if driving_command:
            original_normal = dict(ack.full_control(normal["input_control"]))
            normal_returned = dict(ack.full_control(normal["returned_control"]))
            expected_returned = dict(original_normal)
            elapsed = None if hold_source is None else stamp-hold_source
            if hold_source is not None:
                mathematical = requested_hold_brake(arm, stamp, hold_source)
                expected_returned.update(throttle=0., brake=float32(mathematical))
            elif gp["coast_entry_latched"]:
                expected_returned.update(throttle=0., brake=0.)
            elif gp["handoff_elapsed_sec"] is None:
                expected_returned.update(throttle=float32(.15), brake=0.)
            else:
                expected_returned.update(throttle=float32(min(original_normal["throttle"], number(gp["normal_throttle_cap"]))), brake=0.)
            check(equal(normal_returned, expected_returned), frame, "original_normal_pedals_and_only_pedals")
            check(equal(normal["hold_elapsed_seconds"], elapsed) and
                equal(normal["next_interval_end_elapsed_seconds"], None if elapsed is None else elapsed+.05) and
                equal(normal["mathematical_hold_brake"], mathematical), frame, "independent_measured_clock_plus_dt_formula")
        else:
            check(all(normal[key] is None for key in ("input_control", "returned_control", "hold_elapsed_seconds",
                "next_interval_end_elapsed_seconds", "mathematical_hold_brake")), frame, "no_stale_normal_receipt_for_non_driving_command")
        before = dict(ack.full_control(request["input_control"]))
        returned = dict(ack.full_control(request["returned_control"]))
        expected_before = dict(normal_returned) if normal_returned is not None else common.blank_control()
        if driving_command and emergency:
            expected_before.update(throttle=0., brake=float32(emergency_brake), hand_brake=False)
        check(equal(before, expected_before), frame, "original_emergency_supersession_or_normal_output")
        expected_returned = dict(before, steer=0.) if eligible else before
        check(equal(returned, expected_returned) and request["input_steer"] == before["steer"] and
            request["returned_steer"] == returned["steer"] and request["steering_value_changed"] is (before["steer"] != returned["steer"]),
            frame, "only_requested_steer_changed")
        monitor = gp["low_speed_steering_event_monitor"]
        counts = monitor["observed_event_counts"]
        check(monitor["source_timestamp"] == stamp and monitor["asynchronous_event_delivery"] is True and
            monitor["zero_latency_abort_proven"] is False and set(counts) == set(event_totals) and all(
                type(counts[name]) is int and previous_events[name] <= counts[name] <= event_totals[name] for name in event_totals),
            frame, "bounded_asynchronous_event_monitor")
        if any(counts.values()):
            check(failure is not None and not eligible, frame, "observed_event_stops_new_intervention")
        previous_events = counts
        guarded = dict(returned)
        guard_speed = math.sqrt(sum(number(v)**2 for v in row["world_velocity_carla"])) if initial else speed
        if not emergency and guard_speed < .3 and (guarded["brake"] > .05 or guarded["hand_brake"]):
            guarded["steer"] = 0.
        alignment = row["control_transport"]
        following = commands[alignment["next_command_receipt_sequence"]]
        replacement = following["reason"] in ("alignment_failure_abort", "driving_end_brake")
        final = common.blank_control() if replacement else guarded
        check(equal(final, row["next_control"]) and equal(final, following["requested_control"]), frame, "guarded_final_command_and_ACK")
        if replacement:
            check((following["reason"] == "alignment_failure_abort" and alignment["status"] == "FAIL") or
                (following["reason"] == "driving_end_brake" and driving and frame == driving[-1]["frame"]), frame, "final_replacement_reason")
        if mathematical is not None:
            ramp.append(dict(frame=frame, source_timestamp=stamp, source_elapsed_seconds=stamp-hold_source,
                mathematical_requested_brake=mathematical, stored_normal_brake=normal_returned["brake"],
                post_emergency_brake=before["brake"], final_next_brake=final["brake"], emergency=emergency,
                measured_source_speed_mps=speed, original_current_brake=row["current_control"]["brake"]))
        decisions[reason] += 1
        witnesses.append(dict(frame=frame, timestamp=stamp, capture_phase=row["capture_phase"], initial_engagement=initial,
            normal_request=normal, steering_request=request, independently_requested_hold_brake=mathematical,
            post_existing_guard_control=guarded, final_replacement_reason=following["reason"] if replacement else None,
            final_next_control=row["next_control"], receipt_sequence=following["sequence"]))
        previous_failure = failure
    return dict(status="PASS" if states and not failures else "FAIL", failures=failures, state_receipt_count=len(witnesses),
        requests=witnesses, decision_counts=dict(decisions), hold_source_timestamp=hold_source,
        initial_engagement_source_count=len(initial_rows), normal_hold_command_count=len(ramp),
        normal_hold_commands=ramp, mathematical_first_full_command=next((row for row in ramp if row["mathematical_requested_brake"] == 1.), None),
        stored_first_full_command=next((row for row in ramp if row["stored_normal_brake"] == 1.), None),
        physical_application_proven=False, training_data_approved=False,
        interpretation="Measured source time plus 50 ms defines the normal request, not physical substep onset. Float32 storage is separately reconstructed; no tick rounding or label change.")


def summarize_one(case_root, case, plan):
    """HH_260906 - Independently callable between owned cases; failed captures retain their original rows and failures."""
    validate_plan(plan)
    root, ledger = safe(case_root), {}
    checked(ROOT/"scripts/e2e/summarize_carla_low_speed_steering_abba.py", ledger, COMMON_SOURCE_SHA256, binary=True)
    base = dict(case_id=case["id"], arm=case["arm"], training_data_approved=False, research_only=True,
        learned_model_control=False, vehicle_control_approved=False, native_state_count=None, camera_anchor_count=None)
    if not root.exists():
        return {**base, "status": "NOT_RUN"}, [], [], []
    if not (root/"owner_result.json").exists():
        return {**base, "status": "NOT_FINALIZED", "notice": "No finalized owner receipt; no raw payload audited."}, [], [], []
    owner, lifecycle = verify_owner(root, case, plan, ledger)
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
    profile = manifest["capture_contract"]["goal_stop_profile"]
    require(configuration_equal({key: profile.get(key) for key in configuration}, configuration), "recorded v7 configuration differs")
    require(manifest["result"]["training_data_approved"] is False and manifest["result"]["development_only"] is True,
        "raw v7 diagnostic denials changed")
    experiment = profile["hold_brake_experiment"]
    require(experiment["emergency_ramp_allowed"] is False and experiment["hold_dwell_or_tail_criteria_changed"] is False and
        experiment["physical_actuation_timing_proven"] is False and experiment["historical_v6_completion_changed"] is False and
        experiment["ramp_first_requested_brake"] == .1 and experiment["ramp_nominal_first_full_brake_command_elapsed_seconds"] == .45 and
        experiment["ramp_nominal_first_full_brake_readback_elapsed_seconds"] == .5 and experiment["immediate_requested_brake"] == 1., "v7 intervention scope differs")
    require(manifest["capture_contract"]["stopped_brake_steering_suppressed"] is True and
        manifest["capture_contract"]["control_transport"]["mode"] == "acknowledged_batch", "inherited control guards changed")
    for field, prefix in (("source_route", "route"), ("mapping_file", "mapping"), ("calibration_file", "calibration")):
        relative = str(safe(manifest["provenance"][field]).relative_to(ROOT))
        require(relative in plan["input_sha256"] and manifest["provenance"][prefix+"_sha256"] == plan["input_sha256"][relative], "captured fixed input differs")
    require(manifest["provenance"]["goal_stop_helper_sha256"] == plan["source_sha256"]["scripts/e2e/carla_goal_stop_profile.py"], "captured v7 helper differs")
    settings = manifest.get("runtime", {}).get("capture_world_settings")
    require(equal(settings, dict(synchronous_mode=True, fixed_delta_seconds=.05, no_rendering_mode=False,
        substepping=True, max_substep_delta_time=.01, max_substeps=10)), "recorded six-field physics settings differ")
    result.update(native_state_count=len(states), camera_anchor_count=len(cameras), original_episode_directory=directory,
        recorded_world_settings={key: manifest["runtime"].get(key) for key in ("original_world_settings", "capture_world_settings")},
        world_restoration_claim="Collector cleanup receipt only; no independent eleven-field restoration proof.")
    if states:
        bootstrap = manifest["capture_contract"]["control_transport"]["bootstrap_observation"]
        require(lifecycle["ready"]["snapshot_frame"] < bootstrap["frame"] < states[0]["frame"] and
            lifecycle["after_capture"]["snapshot_frame"] >= states[-1]["frame"] and
            lifecycle["after_capture"]["snapshot_elapsed_seconds"] >= states[-1]["timestamp"], "lifecycle does not bracket native frames/times")
        timeline, failures = raw.measured_timeline(states), []
        def stage(name, function):
            try:
                result[name] = function()
            except (ValueError, KeyError, TypeError, OSError) as error:
                result[name] = {"status": "FAILED_VALIDATION", "error": f"{type(error).__name__}: {error}"}
                failures.append(name)
        stage("initialization", lambda: common.initialization_checks(states, manifest))
        stage("transport", lambda: ack.analyze_transport(states, receipts, bootstrap, wheelbase=2.85))
        stage("hold_and_steering_policy", lambda: hold_and_steering_protocol(states, native["native_states"], manifest, receipts, case["arm"]))
        try:
            pixels, images, anchors = raw.camera_audit(episode, manifest, cameras, timeline)
            result["camera_pixels"] = pixels
            result["measured_future"], futures = raw.measured_future_audit(timeline, cameras, route, anchors)
        except (ValueError, KeyError, TypeError, OSError) as error:
            result["future_or_camera_error"] = f"{type(error).__name__}: {error}"
            failures.append("camera_or_future")
        for row in images:
            checked(episode/row["path"], ledger, row["sha256"], binary=True)
        clear = (owner["exit_code"] == 0 and original["raw_quality_candidate"] and not failures and
            all(result[name].get("status") == "PASS" for name in ("initialization", "transport", "hold_and_steering_policy")) and
            result.get("camera_pixels", {}).get("all_pixel_and_metadata_checks_clear") is True)
        future = result.get("measured_future", {})
        full = future.get("full_64_point_anchors", {})
        result["all_original_future_metrics_clear"] = (bool(full.get("sample_count")) and bool(full.get("metrics")) and all(
            metric["violating_step_count"] == 0 for metric in full["metrics"].values()) and
            set(future.get("disposition_counts", {})) <= {"full_64_point_anchor", "tail_label_context_only"})
        result["execution_and_scalar_pixel_protocol_clear"] = bool(clear)
        result["status"] = "COMPLETE_DIAGNOSTIC_NOT_ADMITTED" if clear else "FAILED_CAPTURE_OR_QA_NOT_ADMITTED"
    for name, pin in tuple(ledger.items()):
        checked(name, {}, pin, binary=True)
    return result, images, futures, states


def comparison_verdict(trials, pairs):
    require([(row["case_id"], row["arm"]) for row in trials] == list(CASES), "comparison requires all four planned denominators")
    rows = {}
    for trial in trials:
        full = trial.get("measured_future", {}).get("full_64_point_anchors", {})
        metric = full.get("metrics", {}).get("xy_curvature", {})
        count, denominator = metric.get("violating_anchor_count"), full.get("sample_count", 0)
        rows[trial["case_id"]] = dict(full_anchor_count=denominator, curvature_violating_anchor_count=count,
            curvature_violating_anchor_fraction=count/denominator if count is not None and denominator else None,
            curvature_overlapping_failing_step_count=metric.get("violating_step_count"),
            curvature_unique_100ms_endpoint_count=metric.get("unique_episode_relative_100ms_violation_tick_count"),
            maximum_assessed_curvature_rad_per_m=(metric.get("maximum") or {}).get("value"),
            all_metric_violating_anchor_counts={name: item["violating_anchor_count"] for name, item in full.get("metrics", {}).items()})
    complete = all(row["status"] == "COMPLETE_DIAGNOSTIC_NOT_ADMITTED" and rows[row["case_id"]]["full_anchor_count"] for row in trials)
    matched = all(pair.get("status") == "EXACT_MATCH" for pair in pairs.values())
    cross = []
    for b in (CASES[1][0], CASES[2][0]):
        for a in (CASES[0][0], CASES[3][0]):
            first, second = rows[a], rows[b]
            if not first["full_anchor_count"] or not second["full_anchor_count"]:
                continue
            no_other = all(second["all_metric_violating_anchor_counts"][name] <= count and
                second["all_metric_violating_anchor_counts"][name]/second["full_anchor_count"] <= count/first["full_anchor_count"]
                for name, count in first["all_metric_violating_anchor_counts"].items() if name != "xy_curvature")
            count_delta = second["curvature_violating_anchor_count"]-first["curvature_violating_anchor_count"]
            rate_delta = second["curvature_violating_anchor_fraction"]-first["curvature_violating_anchor_fraction"]
            max_delta = second["maximum_assessed_curvature_rad_per_m"]-first["maximum_assessed_curvature_rad_per_m"]
            cross.append(dict(immediate_case=a, ramp_case=b, ramp_minus_immediate_failing_anchor_count=count_delta,
                ramp_minus_immediate_failing_anchor_fraction=rate_delta, other_original_gates_not_regressed=no_other,
                ramp_minus_immediate_maximum_curvature_rad_per_m=max_delta, worst_observed_curvature_increased=max_delta > 0,
                favorable_count_and_rate=count_delta < 0 and rate_delta < 0 and no_other))
    favorable = complete and len(cross) == 4 and all(row["favorable_count_and_rate"] for row in cross)
    return dict(status="INCOMPLETE_OR_FAILED_NO_COMPARATIVE_VERDICT" if not complete else
        "FAVORABLE_CONDITION_SPECIFIC_SIGNAL_NOT_ADMITTED" if favorable else "MIXED_OR_NO_CONSISTENT_IMPROVEMENT_NOT_ADMITTED",
        per_case_counts_and_rates=rows, observed_cross_arm_comparisons=cross, required_cross_arm_comparison_count=4,
        available_cross_arm_comparison_count=len(cross), exact_repeats_and_pretreatment_prefixes=matched,
        training_data_approved=False, criterion="Both ramp repeats improve full original curvature count AND rate against BOTH immediate repeats, without other original gate count/rate regressions and with all operational/scalar/camera/control checks clear. Exact prefix/repeat checks are separately reported secondary evidence.",
        interpretation="Report worse maxima independently even if count/rate improve. Every remaining original failure is unresolved; matched results do not identify a unique physical cause or grant admission.")


def validate_readiness(report, journal, expected_source):
    """HH_260906 - Replay only the stored observation policy; never contact or reserve a socket during analysis."""
    require(report["schema"] == "carla.local_port_readiness.v1" and report["bind_host"] == "127.0.0.1" and
        report["ports"] == [2100, 2101, 2102] and report["protocols"] == ["TCP", "UDP"] and
        report["observation_budget_seconds"] == 60. and report["minimum_sample_spacing_seconds"] == 2. and
        report["required_consecutive_clear_checks"] == 3, "readiness policy differs")
    require(report["source_sha256_before"] == report["source_sha256_after"] == expected_source, "readiness source differs")
    require(all(report[key] is False for key in ("simulator_contacted", "simulator_launched", "wrapper_invoked",
        "port_reserved_after_probe", "reuse_socket_options_used", "automatic_trial_retry", "foreign_processes_signaled", "installations_performed")), "readiness scope differs")
    samples = report["samples"]
    require(journal and journal[0]["kind"] == "started" and journal[-1]["kind"] == "finished" and
        journal[0]["source_sha256"] == expected_source and journal[0]["started_at_utc"] == report["started_at_utc"] and
        journal[-1]["status"] == report["status"], "readiness journal boundary differs")
    bound_samples = [row for row in journal if row["kind"] == "bind_sample"]
    require(len(bound_samples) == len(samples), "readiness journal sample count differs")
    expected_bindings = {(p, protocol) for p in (2100, 2101, 2102) for protocol in ("TCP", "UDP")}
    consecutive, previous_time = 0, None
    for index, (sample, bound) in enumerate(zip(samples, bound_samples)):
        elapsed = number(sample["elapsed_seconds"])
        require(type(sample["index"]) is int and sample["index"] == index and 0. <= elapsed < 60. and
            (previous_time is None or elapsed-previous_time >= 2.), "readiness sample clock/index differs")
        require(len(sample["bindings"]) == 6 and {(row["port"], row["protocol"]) for row in sample["bindings"]} == expected_bindings and
            all(row["host"] == "127.0.0.1" and row["status"] in ("CLEAR", "BLOCKED") for row in sample["bindings"]), "readiness six-bind witness differs")
        clear = all(row["status"] == "CLEAR" for row in sample["bindings"])
        consecutive = consecutive+1 if clear else 0
        require(sample["all_six_clear"] is clear and type(sample["consecutive_clear_checks"]) is int and
            sample["consecutive_clear_checks"] == consecutive, "readiness clear streak differs")
        require(equal({key: value for key, value in bound.items() if key not in ("kind", "ss")},
            {key: value for key, value in sample.items() if key != "ss"}) and bound["ss"] == [], "readiness journal bind payload differs")
        snapshots = [{key: value for key, value in row.items() if key not in ("kind", "sample_index")}
            for row in journal if row["kind"] == "ss_snapshot" and row["sample_index"] == index]
        require(equal(snapshots, sample["ss"]), "readiness ss journal differs")
        require(not snapshots if clear else [row["protocol"] for row in snapshots] == ["TCP", "UDP"], "readiness failed-sample diagnostic coverage differs")
        previous_time = elapsed
    require(report["final_consecutive_clear_checks"] == consecutive and
        report["ready_observed"] is (report["status"] == "READY_OBSERVED_NOT_RESERVED"), "readiness final status differs")
    if report["ready_observed"]:
        require(consecutive >= 3 and report["error"] is None and number(report["elapsed_seconds"]) <= 60., "readiness success lacks bounded clear streak")
    require(previous_time is None or number(report["elapsed_seconds"]) >= previous_time, "readiness total duration precedes final sample")
    require(instant(report["started_at_utc"]) <= instant(report["completed_at_utc"]), "readiness UTC reversed")
    return dict(status=report["status"], ready_observed=report["ready_observed"], sample_count=len(samples),
        final_consecutive_clear_checks=consecutive, elapsed_seconds=report["elapsed_seconds"], port_reserved=False)


def execution_record(campaign, case, plan, index, ledger):
    directory = campaign/"execution"/case["id"]
    if not directory.exists():
        require(not (campaign/case["id"]).exists(), "raw case exists without durable execution declaration")
        return dict(case_id=case["id"], arm=case["arm"], status="NOT_RUN", wrapper_attempt_count=0)
    declaration = checked(directory/"declaration.json", ledger)
    result = checked(directory/"result.json", ledger)
    require(declaration["schema"] == "carla.v7_case_execution_declaration.v1" and declaration["case_id"] == result["case_id"] == case["id"] and
        declaration["arm"] == case["arm"] and declaration["plan_sha256"] == PLAN_SHA256 and declaration["command"] == case["command"] and
        declaration["maximum_wrapper_attempts"] == 1 and declaration["automatic_retry"] is False and
        declaration["training_data_approved"] is False, "durable declaration differs")
    prior = declaration["prior_case_continuation"]
    require([row["case_id"] for row in prior] == [row["id"] for row in plan["cases"][:index]] and
        all(row["status"] == "COMPLETE_DIAGNOSTIC_NOT_ADMITTED" and row["execution_and_scalar_pixel_protocol_clear"] is True for row in prior), "prior continuation record differs")
    require(result["schema"] == "carla.v7_case_execution_result.v1" and result["automatic_retry"] is False and
        result["training_data_approved"] is False and type(result["wrapper_attempt_count"]) is int and
        instant(plan["declared_at_utc"]) <= instant(declaration["declared_at_utc"]) <= instant(result["completed_at_utc"]), "durable result scope/clock differs")
    exceptional = ("READINESS_INVOCATION_FAILED_NOT_ATTEMPTED", "PIN_MISMATCH_NOT_ATTEMPTED", "WRAPPER_INVOCATION_OR_WAIT_FAILED_STATUS_UNCONFIRMED")
    def retained(readiness_summary):
        return dict(case_id=case["id"], arm=case["arm"], **{key: value for key, value in result.items() if key not in ("case_id", "arm")},
            readiness=readiness_summary, execution_result_path=str((directory/"result.json").relative_to(ROOT)),
            execution_result_sha256=ledger[str(directory/"result.json")], declaration_sha256=ledger[str(directory/"declaration.json")])
    if result["status"] in exceptional:
        attempted = result["status"] == "WRAPPER_INVOCATION_OR_WAIT_FAILED_STATUS_UNCONFIRMED"
        require(result["wrapper_attempt_count"] == int(attempted) and isinstance(result["error"], str) and bool(result["error"]), "exceptional result lacks explicit uncertainty")
        require(attempted or (not (directory/"wrapper_started.json").exists() and not (campaign/case["id"]).exists()), "not-attempted exception contradicts execution")
        if attempted:
            started = checked(directory/"wrapper_started.json", ledger)
            require(started["case_id"] == case["id"] and started["command"] == case["command"] and
                started["wrapper_attempt_count"] == 1 and started["automatic_retry"] is False, "unconfirmed wrapper intent differs")
        return retained(dict(status="NOT_AUTHENTICATED_AFTER_ORCHESTRATION_EXCEPTION", ready_observed=None,
            notice="Exception receipt retained; no exit/cleanup success or raw capture completeness is inferred."))
    require(type(result["exit_code"]) is int, "durable wrapper exit code must be an integer")
    if result["status"] == "READINESS_BLOCKED_NOT_ATTEMPTED" and result.get("readiness_sha256") is None:
        require(not (directory/"readiness.json").exists() and not (directory/"wrapper_started.json").exists() and
            not (campaign/case["id"]).exists() and result["wrapper_attempt_count"] == 0 and result["exit_code"] != 0, "missing readiness result contradicts execution")
        return retained(dict(status="NO_FINAL_READINESS_REPORT", ready_observed=False))
    readiness = checked(directory/"readiness.json", ledger, required_sha(result["readiness_sha256"]))
    require(readiness["sample_journal"] == "readiness.json.samples.jsonl", "readiness journal path differs")
    journal = checked(directory/readiness["sample_journal"], ledger, required_sha(readiness["sample_journal_sha256"]), jsonl=True)
    readiness_summary = validate_readiness(readiness, journal, plan["orchestration_sha256"]["scripts/e2e/probe_carla_port_readiness.py"])
    require(instant(declaration["declared_at_utc"]) <= instant(readiness["started_at_utc"]) <= instant(readiness["completed_at_utc"]) <=
        instant(result["completed_at_utc"]), "readiness outside durable execution")
    if result["status"] == "READINESS_BLOCKED_NOT_ATTEMPTED":
        require(result["wrapper_attempt_count"] == 0 and result["exit_code"] != 0 and not readiness["ready_observed"] and
            not (directory/"wrapper_started.json").exists() and not (campaign/case["id"]).exists(), "blocked readiness contradicts execution")
    else:
        require(result["status"] in ("WRAPPER_COMPLETED", "WRAPPER_FAILED_WITH_OUTPUT", "PRELAUNCH_BLOCKED_NOT_CAPTURED") and
            result["wrapper_attempt_count"] == 1 and readiness["ready_observed"] is True, "wrapper result/readiness differs")
        started = checked(directory/"wrapper_started.json", ledger)
        checked(directory/"wrapper.log", ledger, required_sha(result["wrapper_log_sha256"]), binary=True)
        require(started["schema"] == "carla.v7_wrapper_invocation.v1" and started["case_id"] == case["id"] and
            started["command"] == case["command"] and started["readiness_sha256"] == result["readiness_sha256"] and
            started["wrapper_attempt_count"] == 1 and started["automatic_retry"] is False and
            instant(readiness["completed_at_utc"]) <= instant(started["started_at_utc"]) <= instant(result["completed_at_utc"]), "wrapper intent differs")
        exists = (campaign/case["id"]).exists()
        if result["status"] == "PRELAUNCH_BLOCKED_NOT_CAPTURED":
            require(not exists and result["exit_code"] != 0 and result["owner_result_sha256"] is None, "prelaunch block contradicts raw evidence")
        elif result["status"] == "WRAPPER_COMPLETED":
            owner = checked(campaign/case["id"]/"owner_result.json", ledger, required_sha(result["owner_result_sha256"]))
            require(result["exit_code"] == owner["exit_code"] == 0 and instant(owner["completed_at_utc"]) <= instant(result["completed_at_utc"]), "completed wrapper lacks matching owner success")
        else:
            require(exists and result["exit_code"] != 0, "failed wrapper result differs")
            if result["owner_result_sha256"] is not None:
                checked(campaign/case["id"]/"owner_result.json", ledger, required_sha(result["owner_result_sha256"]))
    return retained(readiness_summary)


def attempts_companion(path, expected, plan, records, ledger):
    document = checked(path, ledger, expected)
    require(document["schema"] == "carla.hold_brake_ramp_execution_attempts.v1" and document["plan_sha256"] == PLAN_SHA256 and
        document["training_data_approved"] is False and document["learned_model_control"] is False and len(document["records"]) == 4, "attempt companion scope differs")
    instant(document["recorded_at_utc"])
    for item, case, actual in zip(document["records"], plan["cases"], records):
        require(item["case_id"] == case["id"] and item["arm"] == case["arm"] and item["command"] == case["command"] and
            type(item["attempt_count"]) is int and item["attempt_count"] == actual["wrapper_attempt_count"] and
            item["status"] == ("CAPTURE_COMPLETED" if actual["status"] == "WRAPPER_COMPLETED" else actual["status"]), "attempt companion identity/count/status differs")
        if actual["status"] != "NOT_RUN":
            require(item["execution_result_path"] == actual["execution_result_path"] and
                item["execution_result_sha256"] == actual["execution_result_sha256"], "attempt companion durable result differs")
    return document


def hold_motion(states):
    """HH_260906 - Keep signed pose/velocity disagreement visible; this is neither a physical-cause test nor a gate waiver."""
    index = next((i for i, row in enumerate(states) if row["goal_stop"].get("pilot_transition") == "zero_pedal_coast_to_service_brake_hold"), None)
    if index is None:
        return {"status": "NOT_REACHED"}
    selected = states[index:]
    intervals = []
    for left, right in zip(selected, selected[1:]):
        dt = number(right["timestamp"])-number(left["timestamp"])
        require(abs(dt-.05) <= 1e-4, "hold native interval differs")
        actor_delta = [right["actor_snapshot_transform_carla"][key]-left["actor_snapshot_transform_carla"][key] for key in ("x", "y")]
        actor_delta[1] *= -1.
        integral = [dt*.5*(left["world_velocity_carla"][i]+right["world_velocity_carla"][i]) * (1. if i == 0 else -1.) for i in range(2)]
        base_delta = [right[key]-left[key] for key in ("x", "y")]
        intervals.append(dict(source_frame=left["frame"], destination_frame=right["frame"], source_timestamp=left["timestamp"],
            destination_timestamp=right["timestamp"], source_elapsed_seconds=left["timestamp"]-selected[0]["timestamp"],
            actor_xy_delta_ros_m=actor_delta, actor_signed_trapezoid_velocity_integral_ros_m=integral,
            actor_pose_minus_velocity_residual_norm_m=math.hypot(*(a-b for a, b in zip(actor_delta, integral))),
            virtual_base_xy_delta_m=base_delta, virtual_base_displacement_m=math.hypot(*base_delta),
            virtual_base_forward_displacement_m=base_delta[0]*math.cos(left["yaw"])+base_delta[1]*math.sin(left["yaw"]),
            measured_yaw_change_rad=math.atan2(math.sin(right["yaw"]-left["yaw"]), math.cos(right["yaw"]-left["yaw"])),
            source_next_control=left["next_control"], destination_current_control=right["current_control"],
            destination_planar_speed_mps=math.hypot(right["vx"], right["vy"])))
    return dict(status="OBSERVED_NOT_PHYSICALLY_APPROVED", hold_source_frame=selected[0]["frame"], hold_source_timestamp=selected[0]["timestamp"],
        native_source_through_end_count=len(selected), observed_duration_seconds=selected[-1]["timestamp"]-selected[0]["timestamp"],
        interval_count=len(intervals), first_interval=intervals[0] if intervals else None,
        first_second_intervals=[row for row in intervals if row["source_elapsed_seconds"] < 1.],
        maximum_actor_pose_velocity_residual_norm_m=max((row["actor_pose_minus_velocity_residual_norm_m"] for row in intervals), default=None),
        backward_virtual_base_interval_count=sum(row["virtual_base_forward_displacement_m"] < 0. for row in intervals),
        phase_counts=dict(Counter(row["capture_phase"] for row in selected)),
        interpretation="All native intervals through original dwell and tail are included. Actor signed vector integration is trapezoidal observed-state analysis, not substep ground truth; virtual base and actor positions are distinct.")


def compare_post_hold_motion(first, second):
    def select(rows):
        index = next((i for i, row in enumerate(rows) if row["goal_stop"].get("pilot_transition") == "zero_pedal_coast_to_service_brake_hold"), None)
        return [] if index is None else rows[index:]
    first, second = select(first), select(second)
    if not first or not second:
        return {"status": "UNAVAILABLE", "comparison_performed": False}
    fields = [key for key in common.PHYSICAL_FIELDS if key != "current_control"]
    changed = [{"relative_native_index": i, "first_frame": a["frame"], "second_frame": b["frame"],
        "fields": [key for key in fields if not equal(a[key], b[key])]}
        for i, (a, b) in enumerate(zip(first, second)) if any(not equal(a[key], b[key]) for key in fields)]
    return dict(status="EXACT_MATCH" if len(first) == len(second) and not changed else "DIFFERENT", comparison_performed=True,
        first_state_count=len(first), second_state_count=len(second), compared_native_count=min(len(first), len(second)),
        mismatched_native_count=len(changed), first_mismatch=changed[0] if changed else None, mismatches=changed, exact_fields=fields,
        excluded=["all current/next control requests and readbacks", "absolute clocks/frame IDs", "arm/policy metadata"],
        interpretation="Motion-only observed equality does not imply the intended brake requests were equal, or prove a unique cause/physical actuator timing.")


def run(campaign_root, plan_path, plan_sha256, output_dir=None, *, attempts_path=None, attempts_sha256=None):
    campaign, plan_path, ledger = safe(campaign_root), safe(plan_path), {}
    require(plan_sha256 == PLAN_SHA256, "only the frozen prospective v7 plan is supported")
    plan = checked(plan_path, ledger, plan_sha256)
    validate_plan(plan)
    require((attempts_path is None) == (attempts_sha256 is None), "attempt companion path and SHA must be supplied together")
    require(attempts_path is not None or not (campaign/"execution_attempts.json").exists(), "existing attempt companion cannot be ignored")
    for category in ("source_sha256", "input_sha256", "audit_math_sha256", "orchestration_sha256"):
        for name, pin in plan[category].items():
            checked(ROOT/name, ledger, pin, binary=True)
    checked(ROOT/"scripts/e2e/summarize_carla_low_speed_steering_abba.py", ledger, COMMON_SOURCE_SHA256, binary=True)
    output = None if output_dir is None else safe(output_dir)
    if output is not None:
        require(not output.exists() and output.parent.is_dir(), "fresh output with existing parent required")
        for target in [campaign/case_id for case_id, _ in CASES]+[campaign/"execution"]:
            require(output != target and target not in output.parents and output not in target.parents, "output must be disjoint from raw and execution evidence")
    sources = raw.source_identity()
    for name in (str(Path(__file__).resolve().relative_to(ROOT)), "scripts/e2e/summarize_carla_low_speed_steering_abba.py", "scripts/e2e/carla_goal_stop_profile.py"):
        sources["files"][name] = scalar.sha(ROOT/name)
    execution = [execution_record(campaign, case, plan, index, ledger) for index, case in enumerate(plan["cases"])]
    stopped = False
    for record in execution:
        require(not stopped or record["status"] == "NOT_RUN", "execution continued after blocked/failed/not-run case")
        stopped |= record["status"] != "WRAPPER_COMPLETED"
    attempts = None if attempts_path is None else attempts_companion(attempts_path, attempts_sha256, plan, execution, ledger)
    trials, images, futures, witnesses, physical = [], [], [], [], {}
    for case, record in zip(plan["cases"], execution):
        try:
            if record["status"] in ("READINESS_INVOCATION_FAILED_NOT_ATTEMPTED", "PIN_MISMATCH_NOT_ATTEMPTED", "WRAPPER_INVOCATION_OR_WAIT_FAILED_STATUS_UNCONFIRMED"):
                result, pictures, windows, states = (dict(case_id=case["id"], arm=case["arm"], status=record["status"],
                    training_data_approved=False, native_state_count=None, camera_anchor_count=None,
                    notice="Durable orchestration exception only; any raw bytes remain unaudited and cleanup is not inferred."), [], [], [])
            else:
                result, pictures, windows, states = summarize_one(campaign/case["id"], case, plan)
        except (ValueError, KeyError, TypeError, OSError) as error:
            result, pictures, windows, states = (dict(case_id=case["id"], arm=case["arm"], status="FAILED_VALIDATION",
                error=f"{type(error).__name__}: {error}", training_data_approved=False, native_state_count=None, camera_anchor_count=None), [], [], [])
        result["durable_execution"] = record
        if record["status"] in ("PRELAUNCH_BLOCKED_NOT_CAPTURED", "READINESS_BLOCKED_NOT_ATTEMPTED"):
            require(result["status"] == "NOT_RUN", "blocked execution contradicts raw case")
            result["status"] = record["status"]
        elif record["status"] != "WRAPPER_COMPLETED" and result["status"] == "COMPLETE_DIAGNOSTIC_NOT_ADMITTED":
            raise scalar.EvidenceError("raw complete claim contradicts durable wrapper failure")
        if states:
            result["curvature_incidence"] = common.curvature_incidence(states, windows)
            result["hold_motion"] = hold_motion(states)
        policy = result.get("hold_and_steering_policy", {})
        requests = policy.pop("requests", [])
        if policy:
            require("state_receipt_count" not in policy or policy["state_receipt_count"] == len(requests), "control policy witness count differs")
            policy.update(witness_file="control_policy_witnesses.jsonl", witness_case_id=case["id"], witness_count=len(requests))
        for request, state in zip(requests, states):
            request["measured_physical_state"] = {key: state[key] for key in common.PHYSICAL_FIELDS}
        witnesses.extend(dict(case_id=case["id"], arm=case["arm"], **row) for row in requests)
        images.extend(dict(case_id=case["id"], arm=case["arm"], **row) for row in pictures)
        futures.extend(dict(case_id=case["id"], arm=case["arm"], **row) for row in windows)
        trials.append(result)
        physical[case["id"]] = states
    pairs = dict(immediate_repeat_A1_A2=common.compare_physical(physical[CASES[0][0]], physical[CASES[3][0]]),
        ramp_repeat_B1_B2=common.compare_physical(physical[CASES[1][0]], physical[CASES[2][0]]),
        before_hold_A1_B1=before_hold_comparison(physical[CASES[0][0]], physical[CASES[1][0]]),
        before_hold_B2_A2=before_hold_comparison(physical[CASES[2][0]], physical[CASES[3][0]]))
    verdict = comparison_verdict(trials, pairs)
    pairs["prospective_primary_comparison"] = verdict
    pairs["post_hold_motion_only_A1_B1"] = compare_post_hold_motion(physical[CASES[0][0]], physical[CASES[1][0]])
    pairs["post_hold_motion_only_B2_A2"] = compare_post_hold_motion(physical[CASES[2][0]], physical[CASES[3][0]])
    summary = dict(schema=SCHEMA, status="DIAGNOSED_NOT_ADMITTED", generated_at_utc=datetime.now(timezone.utc).isoformat(),
        plan_sha256=plan_sha256, plan_path=str(plan_path), source_sha256=sources,
        execution_attempts_sha256=attempts_sha256, execution_attempts_evidence=attempts,
        planned_case_count=4, wrapper_attempted_case_count=sum(row["wrapper_attempt_count"] for row in execution),
        readiness_observed_case_count=sum(row["status"] != "NOT_RUN" for row in execution),
        training_data_approved=False, research_only=True, dataset_export=False, target_retiming=False, gate_changes=False,
        learned_model_control=False, vehicle_control_approved=False, cases=trials,
        case_status_counts=dict(Counter(row["status"] for row in trials)),
        all_four_completed_diagnostics=all(row["status"] == "COMPLETE_DIAGNOSTIC_NOT_ADMITTED" for row in trials),
        audited_native_state_count=sum(row["native_state_count"] or 0 for row in trials),
        audited_camera_anchor_count=sum(row["camera_anchor_count"] or 0 for row in trials),
        audited_original_jpeg_count=len(images), future_anchor_record_count=len(futures),
        control_policy_witness_count=len(witnesses), control_policy_witness_file="control_policy_witnesses.jsonl",
        primary_comparison_status=verdict["status"], metadata_sha256=ledger,
        limitations=["All original full-future gates and failed rows are retained; no training or vehicle approval.",
            "Normal hold-brake source-clock requests are not physical substep onset measurements. Original emergency authority and zero-steering policy remain separate.",
            "Both arms are fresh v7; this never completes or retries the historical incomplete v6 campaign.",
            "Matched repeats and prefixes exclude only absolute clocks/IDs and the first treated next command at the hold source, without retiming or normalization.",
            "A favorable count/rate can coexist with a worse maximum. One fixed Town07 route/seed/weather cannot establish broad robustness or unique physical cause.",
            "Readiness is observed, not reserved; the wrapper's admission remains authoritative.",
            "Recorded six-field settings and cleanup receipts are not an independent eleven-field restoration proof."])
    for name, pin in sources["files"].items():
        checked(ROOT/name, {}, pin, binary=True)
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
        for name, rows in (("image_inventory.jsonl", images), ("future_windows.jsonl", futures), ("control_policy_witnesses.jsonl", witnesses)):
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
    parser.add_argument("--attempts", type=Path)
    parser.add_argument("--attempts-sha256")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--output-dir", type=Path)
    group.add_argument("--verify-only", action="store_true")
    args = parser.parse_args(argv)
    try:
        summary, _ = run(args.campaign_root, args.plan, args.plan_sha256, args.output_dir,
            attempts_path=args.attempts, attempts_sha256=args.attempts_sha256)
    except (ValueError, KeyError, TypeError, OSError) as error:
        parser.exit(1, f"v7 summary failed: {error}\n")
    print(json.dumps({key: summary[key] for key in ("status", "case_status_counts", "audited_native_state_count",
        "audited_camera_anchor_count", "primary_comparison_status")}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
