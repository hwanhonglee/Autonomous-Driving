#!/usr/bin/env python3
"""HH_260906 - Audit the fresh fixed-cadence two-case terminal feedback experiment without changing labels, control or admission."""
from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict, replace
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from scripts.e2e import summarize_carla_hold_brake_ramp_abba as prior
from scripts.e2e import carla_physics_substeps as substeps
from scripts.e2e import summarize_carla_hold_substep_abba as numerical
common = prior.common

SCHEMA = "carla.terminal_approach_summary.v1"
PROFILE = "comfortable_v9_terminal_feedback"
CASES = (("01_coast_A", "coast"), ("02_feedback_B", "feedback"))
PHYSICS = {"coast": "reference_10ms", "feedback": "reference_10ms"}
EXECUTION_SOURCES = prior.common.EXECUTION_SOURCES | {"scripts/e2e/carla_physics_substeps.py"}
ANALYSIS_PINS = {"scripts/e2e/summarize_carla_hold_substep_abba.py": "a5655f6764e87ceb7fda67a0341dab8b3c95ea7fe36db6a7857eba06cc33dcb7", "scripts/e2e/summarize_carla_hold_brake_ramp_abba.py": "adca365e601239a4f6b43230e827f5f72caa272b40a946e6c74fdba8437cacef", "scripts/e2e/summarize_carla_low_speed_steering_abba.py": "53b42cf4e1be70f64ead8c6d143aed4dabc46e5f486407f0bacb9f70e6410da1"}
SELF_PATH = "scripts/e2e/summarize_carla_terminal_approach.py"
CAMPAIGN_PATH = "artifacts/training/2026-09-15/terminal_approach_v9"
ORCHESTRATION_PATHS = {"scripts/e2e/probe_carla_port_readiness.py", *{
    "artifacts/training/2026-09-15/terminal_approach_v9/"+name for name in ("run_case.py", "render_evidence.py")}}
COMMON_SOURCE_SHA256 = "53b42cf4e1be70f64ead8c6d143aed4dabc46e5f486407f0bacb9f70e6410da1"
AUDIT_MATH_PINS = {
    "scripts/e2e/audit_carla_raw_pre_admission.py": "0a7983857aea0e2b6588bb27712865ce1f70c27aa06a187fca4d0fc1a72dbc00",
    "scripts/e2e/audit_carla_acknowledged_control.py": "9d6fccb6874a5cdfc85e8bf5d9229435e4ba6fe6850eb63a9e83488f39bcc5bd",
    "scripts/e2e/audit_portable_target_feasibility.py": "8b56a4990f51639ec5ff9bca12f7453051772d1984e4db768613384b25f00894",
    "scripts/e2e/summarize_carla_goal_stop_trials.py": "890984ed44df9dacc5112a2c2d099652b8fd0490e41c6856aea9c216a4029032",
    "scripts/e2e/prepare_carla_common10_dataset.py": "c63c7e5b7e0c06336c885f2d6120e3e8a8a74daf372ddf5df167eac2b4b4b666",
    "portable_e2e/model.py": "fbbedef10b97add830e0d887400107b2bcd6dfb5f0c5c4901e399d2e4717fd6e",
    "portable_e2e/runtime_contract.py": "38e993278ef84b149efc90931423cd90b1562d86c9eb1585260d50e03b2ae0d3",
}
safe, checked, equal, configuration_equal, encoded, instant = common.safe, common.checked, common.equal, common.configuration_equal, common.encoded, common.instant
raw, ack, scalar, profiles = common.raw, common.ack, common.scalar, common.profiles
require, number = scalar.require, scalar.number


def required_sha(value):
    require(isinstance(value, str) and len(value) == 64 and set(value) <= set("0123456789abcdef"), "mandatory SHA-256 reference missing or invalid")
    return value


def check_analysis_pins(plan, ledger):
    require(set(plan["analysis_sha256"]) == {*ANALYSIS_PINS, SELF_PATH}, "analysis closure differs")
    for name, pin in plan["analysis_sha256"].items():
        require(name not in ANALYSIS_PINS or pin == ANALYSIS_PINS[name], "frozen pure audit dependency differs")
        checked(ROOT/name, ledger, required_sha(pin), binary=True)


def validate_substeps(manifest, arm):
    """HH_260906 - Both prospective arms retain the exact same eleven-field reference setting."""
    require(arm in PHYSICS, "unknown terminal approach arm")
    return numerical.validate_substeps(manifest, "reference")


def validate_plan(plan):
    require(plan["schema"] == "carla.terminal_approach_plan.v1" and
        [(case["id"],case["arm"]) for case in plan["cases"]] == list(CASES), "fixed two-case terminal approach plan required")
    require(type(plan["maximum_attempts_per_revision"]) is int and plan["maximum_attempts_per_revision"] == 2 and
        all(type(case["maximum_attempts"]) is int and case["maximum_attempts"] == 1 for case in plan["cases"]),
        "two cases with one attempt each required")
    require(plan["physics_profile_by_arm"] == PHYSICS, "both arms require the fixed reference 10ms setting")
    require(set(plan["orchestration_sha256"]) == ORCHESTRATION_PATHS, "fixed readiness/runner/renderer closure required")
    expected_bounds = dict(control_transport="acknowledged_batch", agent_initialization="after_bootstrap",
        target_speed_kmh=28.8, physics_hz=20, camera_hz=10, stationary_warmup_seconds=3.5,
        required_measured_stop_dwell_seconds=2., stationary_tail_seconds=6.5, quality="Epic", map="Town07",
        seed=0, weather="ClearNoon", wheelbase_m=2.85, fixed_max_substeps=10,
        explicit_settings_readback_required=True, vehicle_type="vehicle.toyota.prius", scenario="straight",
        spawn_z_offset_m=.5, maximum_total_simulation_seconds=180, collector_wall_timeout_seconds=900)
    require(all(configuration_equal(plan.get(key), value) for key,value in expected_bounds.items()), "fixed experiment bounds differ")
    require(equal(plan["required_original_substeps"], dict(substepping=True,max_substep_delta_time=.01,max_substeps=10)) and
        equal(plan["requested_max_substep_delta_time_by_arm"], dict(coast=.01,feedback=.01)), "declared numerical settings differ")
    require(set(plan["protocol_configuration_by_arm"]) == {"coast", "feedback"}, "both terminal modes required")
    for arm in PHYSICS:
        expected = asdict(replace(profiles.TerminalApproachDevelopmentGoalStopConfig(), terminal_approach_mode=arm))
        require(configuration_equal(plan["protocol_configuration_by_arm"][arm], expected), "fixed terminal control configuration differs")
    require(set(plan["source_sha256"]) == EXECUTION_SOURCES and set(plan["input_sha256"]) == common.INPUT_PATHS and
        plan["audit_math_sha256"] == AUDIT_MATH_PINS, "native execution/input/frozen original-math closure differs")
    for category in ("source_sha256","input_sha256","audit_math_sha256","orchestration_sha256","analysis_sha256"):
        require(isinstance(plan[category], dict) and plan[category], "empty pin category")
        for name,pin in plan[category].items():
            require(not Path(name).is_absolute() and ".." not in Path(name).parts, "unsafe source pin")
            required_sha(pin)
    require(set(plan["analysis_sha256"]) == {*ANALYSIS_PINS, SELF_PATH} and
        all(plan["analysis_sha256"][name] == pin for name,pin in ANALYSIS_PINS.items()), "analysis dependency contract differs")
    normalized_commands=[]
    for case in plan["cases"]:
        command=case["command"]
        require(isinstance(command,list) and all(isinstance(v,str) for v in command) and
            command[:2] == ["bash","scripts/e2e/run_owned_carla_expert_trial.sh"] and command.count("--")==1,
            "owned wrapper command required")
        flags = command[command.index("--")+1:]
        require(len(flags)%2==0 and len(set(flags[::2]))==len(flags[::2]) and
            all(key.startswith("--") for key in flags[::2]), "collector flags must be unique option/value pairs")
        fixed_flags={"--goal-stop-profile":PROFILE,"--hold-brake-mode":"immediate",
            "--low-speed-steering-mode":"zero","--physics-substep-profile":PHYSICS[case["arm"]],
            "--terminal-approach-mode":case["arm"],
            "--physics-hz":"20","--capture-hz":"10","--target-speed-kmh":"28.8",
            "--control-transport":"acknowledged_batch","--agent-initialization":"after_bootstrap",
            "--max-duration-sec":"180","--stationary-warmup-sec":"3.5","--stationary-tail-sec":"6.5",
            "--spawn-z-offset-m":"0.5","--vehicle-type":"vehicle.toyota.prius","--wheelbase-m":"2.85",
            "--weather":"ClearNoon","--seed":"0","--goal-tolerance-m":"1.0",
            "--mapping":"autoware_e2e_vad_launch/config/sensor_mapping_vad_fast_reliable.yaml",
            "--calibration":"src/launcher/autoware_launch/sensor_kit/carla_sensor_kit_launch/carla_sensor_kit_description/config/sensor_kit_calibration.yaml",
            "--basic-agent-base-min-distance-m":"3.0","--basic-agent-distance-ratio":"0.5",
            "--basic-agent-lateral-kp":"1.95","--basic-agent-lateral-ki":"0.05","--basic-agent-lateral-kd":"0.2",
            "--basic-agent-max-steering":"0.8","--basic-agent-lane-offset-m":"0.0"}
        require(dict(zip(flags[::2],flags[1::2])) == fixed_flags, "fixed collector flags differ or contain extra factors")
        require(command[2] == CAMPAIGN_PATH+"/"+case["id"] and command[3] in common.INPUT_PATHS and
            command[4:command.index("--")] == ["--port","2100","--quality","Epic","--wall-timeout-sec","900","--capture-mode","expert"],
            "fixed case output/owned admission flags differ")
        normalized=list(command)
        normalized[2]="DECLARED_CASE_OUTPUT"
        normalized[normalized.index("--terminal-approach-mode")+1]="DECLARED_TERMINAL_ARM"
        normalized_commands.append(normalized)
    require(all(command==normalized_commands[0] for command in normalized_commands),
        "commands change more than case output and terminal approach mode")
    require(all(plan[key] is False for key in ("automatic_retry","training_data_approved","learned_model_control",
        "vehicle_control_approved","dataset_export","target_retiming","gate_changes")), "scope or no-retry denials differ")
    instant(plan["declared_at_utc"])


def original_metrics_clear(future):
    full=future.get("full_64_point_anchors",{})
    return bool(full.get("sample_count")) and bool(full.get("metrics")) and all(
        metric["violating_step_count"]==0 for metric in full["metrics"].values()) and set(
        future.get("disposition_counts",{})) <= {"full_64_point_anchor","tail_label_context_only"}


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
    intent = checked(root.parent/"execution"/case["id"]/"wrapper_started.json", ledger)
    require(intent["schema"] == "carla.v9_wrapper_invocation.v1" and intent["case_id"] == case["id"] and
        intent["command"] == command and instant(intent["started_at_utc"]) <= instant(owner_plan["planned_at_utc"]),
        "wrapper/readiness evidence follows owner launch")
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


def native_residuals(states):
    """HH_260906 - Reuse only the frozen agnostic signed native-interval computation."""
    return numerical.native_residuals(states)


def feedback_law(remaining_arc, speed, goal_window, previous_throttle, interval):
    """HH_260906 - Independent prospective reference/pedal math; these are requests, not measured speed limits."""
    remaining_arc,speed,previous_throttle,interval = map(number,(remaining_arc,speed,previous_throttle,interval))
    require(type(goal_window) is bool and speed >= 0. and 0. <= previous_throttle <= .15 and interval > 0.,
        "invalid feedback source or prior request")
    target = 0. if goal_window else min(.35,max(.12,.25*max(remaining_arc-.75,0.)))
    error = max(target-speed,0.)
    desired = 0. if goal_window else min(.15,.08*min(1.,error/.1)+.5*error)
    mathematical = min(desired,previous_throttle+.10*interval)
    return dict(target_speed_mps=target, positive_speed_error_mps=error,
        raw_throttle=desired, mathematical_throttle=mathematical)


def replay_terminal_decision(row, arm, hold_source, previous_failure, previous_emergency, memory, check):
    """HH_260906 - Reconstruct the latch, measured-source request memory and pre/post-emergency snapshots independently."""
    gp,stamp,frame=row["goal_stop"],number(row["timestamp"]),row["frame"]
    speed=math.hypot(number(row["vx"]),number(row["vy"]))
    remaining=number(gp["remaining_route_arc_m"])
    initial=gp.get("next_control_starts_driving") is True
    driving=row["capture_phase"]=="driving" or initial
    command=initial or (row["capture_phase"]=="driving" and gp["termination_reason"] is None)
    failure=gp.get("pilot_failure_reason")
    emergency=gp["emergency_override_count"]>0 or failure==PROFILE+"_emergency_override"
    post_update_failure=previous_failure is None and failure in (PROFILE+"_observed_event_abort",PROFILE+"_emergency_override")
    update_failure=None if post_update_failure else failure
    if driving and memory["handoff"] is None and speed>=.5:
        memory["handoff"]=stamp
    release=.75+6.93*speed
    if driving and memory["handoff"] is not None and memory["coast"] is None and remaining<=release and 2.8<=speed<=3.2 and update_failure is None:
        memory["coast"]=stamp
        check(gp["pilot_transition"]=="normal_pid_to_zero_pedal_coast",frame,"independent_coast_source_transition")
    check(gp["coast_entry_latched"] is (memory["coast"] is not None),frame,"independent_coast_latch")
    if (arm=="feedback" and driving and memory["coast"] is not None and hold_source is None and
        update_failure is None and previous_emergency==0 and memory["feedback"] is None and speed<=.75 and not gp["complete"]):
        memory["feedback"]=stamp
        check(gp["pilot_transition"]=="zero_pedal_coast_to_terminal_feedback",frame,"independent_feedback_latch_transition")
    eligible=(arm=="feedback" and driving and memory["feedback"] is not None and hold_source is None and
        update_failure is None and previous_emergency==0 and not gp["complete"])
    interval=.05 if memory["request_timestamp"] is None else stamp-memory["request_timestamp"]
    math_values=feedback_law(remaining,speed,gp["goal_window"],memory["throttle"],interval) if eligible else dict(
        target_speed_mps=0.,positive_speed_error_mps=0.,raw_throttle=0.,mathematical_throttle=0.)
    decision=("governor_failure_unchanged" if update_failure is not None else "coast_mode_unchanged" if arm=="coast"
        else "not_driving_command_unchanged" if not driving else "original_hold_priority" if hold_source is not None
        else "feedback_not_latched" if not eligible else "goal_window_zero_pedal" if gp["goal_window"] else "bounded_terminal_feedback")
    expected=dict(schema="carla.terminal_approach_decision.v1",mode=arm,source_timestamp=stamp,
        source_measured_speed_mps=speed,source_remaining_arc_m=remaining,
        source_planar_goal_distance_m=number(gp["planar_goal_distance_m"]),source_terminal_overshoot_m=number(gp["terminal_overshoot_m"]),
        goal_window=gp["goal_window"],upstream_of_endpoint=gp["upstream_of_endpoint"],driving_enabled=driving,
        coast_latched=memory["coast"] is not None,coast_source_timestamp=memory["coast"],
        feedback_latched=memory["feedback"] is not None,feedback_source_timestamp=memory["feedback"],
        hold_latched=hold_source is not None,eligible=eligible,**math_values,
        previous_requested_throttle=memory["throttle"],previous_request_timestamp=memory["request_timestamp"],
        rate_interval_seconds=interval,rate_limit_per_second=.10,next_native_dt_seconds=.05,
        decision=decision,governor_failure_reason=update_failure,physical_application_proven=False,training_data_approved=False,research_only=True)
    final=dict(expected)
    if post_update_failure:
        final.update(eligible=False,target_speed_mps=0.,positive_speed_error_mps=0.,raw_throttle=0.,mathematical_throttle=0.,
            decision="governor_failure_unchanged",governor_failure_reason=failure)
    check(equal(gp["terminal_approach"],final),frame,"independent_terminal_decision_latch_law_and_request_memory")
    normal=gp["terminal_approach_control"]
    normal_expected=dict(schema="carla.terminal_approach_control_request.v1",mode=arm,source_timestamp=stamp,
        normal_control_executed=command,driving_command=command,superseded_by_emergency=emergency,
        superseded_by_governor_failure=failure is not None,
        control_stage="LocalPlanner_normal_control_before_BasicAgent_emergency_and_output_steering",
        physical_application_proven=False,training_data_approved=False,research_only=True)
    check(all(key in normal and equal(normal[key],value) for key,value in normal_expected.items()),frame,"terminal_normal_source_scope")
    if command:
        check(equal(normal["decision_at_normal_control"],expected),frame,"terminal_pre_emergency_decision_snapshot")
        hold=gp["hold_brake_control"]
        check(equal(normal["input_control"],hold["input_control"]) and equal(normal["returned_control"],hold["returned_control"]),
            frame,"terminal_normal_controls_match_original_hold_receipt")
        if eligible:
            memory["throttle"],memory["request_timestamp"]=math_values["mathematical_throttle"],stamp
    else:
        check(all(normal[key] is None for key in ("input_control","returned_control","decision_at_normal_control")),frame,"no_stale_terminal_normal_receipt")
    steering=gp["low_speed_steering"]
    check(equal(normal["output_steering_stage_input_control"],steering["input_control"]) and
        equal(normal["output_steering_stage_returned_control"],steering["returned_control"]),frame,"terminal_output_stage_controls")
    return expected


def terminal_control_protocol(states, native_rows, manifest, receipts, arm):
    """HH_260906 - Replay measured latch time, pre-emergency normal pedals, unchanged emergency authority, output steering, guard and serialized ACK."""
    require(arm in ("coast", "feedback"), "unknown terminal arm")
    require(len(states) == len(native_rows) and all(a["frame"] == b["frame"] and a["timestamp"] == b["timestamp"]
        for a, b in zip(states, native_rows)), "policy/native row count/frame/time mismatch")
    commands = {receipt["sequence"]: receipt for receipt in receipts}
    require(len(commands) == len(receipts), "duplicate command receipt sequence")
    initial_rows = [row for row in states if row["goal_stop"].get("next_control_starts_driving") is True]
    driving = [row for row in states if row["capture_phase"] == "driving"]
    require(not driving or len(initial_rows) == 1, "driving requires exactly one initial command source")
    failures, witnesses, decisions, ramp = [], [], Counter(), []
    hold_source, hold_ticks, previous_stopped, previous_failure = None, 0, False, None
    previous_emergency=0
    memory=dict(coast=None,handoff=None,feedback=None,request_timestamp=None,throttle=0.)
    event_totals = {name: sum(len(row[name]) for row in states) for name in ("collision", "lane_invasion")}
    previous_events = {name: 0 for name in event_totals}
    emergency_brake = number(manifest["capture_contract"]["goal_stop_profile"]["effective_control"]["unchanged_basic_agent_emergency_brake"])
    require(0. <= emergency_brake <= 1., "invalid original emergency brake")
    def check(condition, frame, name):
        if not condition:
            failures.append({"frame": frame, "check": name})
    for row, measured in zip(states, native_rows):
        gp, frame, stamp = row["goal_stop"], row["frame"], row["timestamp"]
        # HH_260906 - Bind the control law to the original independent pose/route projection, not self-consistent forged receipts.
        measured_geometry={"remaining_route_arc_m":number(measured["remaining_route_m"]),
            "planar_goal_distance_m":number(measured["goal_error_m"]),
            "terminal_overshoot_m":number(measured["terminal_overshoot_m"])}
        check(all(equal(gp.get(key),value) for key,value in measured_geometry.items()) and
            gp["goal_window"] is (measured_geometry["remaining_route_arc_m"]<=1. and measured_geometry["planar_goal_distance_m"]<=1.) and
            gp["upstream_of_endpoint"] is (measured_geometry["remaining_route_arc_m"]>1e-6),frame,"independent_pose_route_geometry_binding")
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
                check(gp["pilot_transition"] == "zero_pedal_coast_to_service_brake_hold" , frame, "causal_hold_entry")
        check(gp["hold_ticks"] == hold_ticks and gp["required_hold_ticks"] == 40 and
            abs(gp["hold_duration_sec"]-hold_ticks*.05) <= 1e-9 and gp["complete"] is (hold_ticks >= 40), frame, "unchanged_measured_dwell")
        check(gp["normal_hold_brake"] == 1. and gp["hold_brake_latched"] is (hold_source is not None) and
            gp["hold_brake_source_timestamp"] == hold_source, frame, "unchanged_hold_latch")
        check(gp["profile_id"] == PROFILE and gp["training_data_approved"] is False and gp["measured_timestamp"] == stamp and
            gp["measured_speed_mps"] == speed and gp["low_speed_steering_driving_enabled"] is source_driving, frame, "governor_source_binding")
        terminal = replay_terminal_decision(row,arm,hold_source,previous_failure,previous_emergency,memory,check)
        expected = dict(schema="carla.low_speed_steering_request.v1", mode="zero", source_timestamp=stamp,
            source_measured_speed_mps=speed, governor_driving_enabled=source_driving, driving_command=driving_command,
            threshold_mps=.75, emergency_override_present=emergency, governor_failure_reason=failure,
            eligible=eligible, override_requested=eligible, decision=reason,
            control_stage="after_original_agent_proposal_checks_before_existing_stopped_brake_guard",
            physical_application_proven=False, training_data_approved=False, research_only=True)
        check(all(key in request and equal(request[key], value) for key, value in expected.items()), frame, "steering_source_decision_scope")
        expected_normal = dict(schema="carla.hold_brake_control_request.v1", mode="immediate", source_timestamp=stamp,
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
                mathematical = 1.
                expected_returned.update(throttle=0., brake=prior.float32(mathematical))
            elif gp["coast_entry_latched"]:
                expected_returned.update(throttle=prior.float32(terminal["mathematical_throttle"]), brake=0.)
            elif gp["handoff_elapsed_sec"] is None:
                expected_returned.update(throttle=prior.float32(.15), brake=0.)
            else:
                expected_returned.update(throttle=prior.float32(min(original_normal["throttle"], number(gp["normal_throttle_cap"]))), brake=0.)
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
            expected_before.update(throttle=0., brake=prior.float32(emergency_brake), hand_brake=False)
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
            terminal_decision=gp["terminal_approach"],terminal_normal_request=gp["terminal_approach_control"],
            independently_replayed_pre_output_terminal_decision=terminal,
            post_existing_guard_control=guarded, final_replacement_reason=following["reason"] if replacement else None,
            final_next_control=row["next_control"], receipt_sequence=following["sequence"]))
        previous_failure = failure
        previous_emergency = gp["emergency_override_count"]
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
    check_analysis_pins(plan, ledger)
    require(case in plan["cases"] and root == safe(ROOT/case["command"][2]), "case/root is not the explicit planned target")
    checked(ROOT/"scripts/e2e/summarize_carla_low_speed_steering_abba.py", ledger, COMMON_SOURCE_SHA256, binary=True)
    base = dict(case_id=case["id"], arm=case["arm"], training_data_approved=False, research_only=True,
        learned_model_control=False, vehicle_control_approved=False, native_state_count=None, camera_anchor_count=None)
    if not root.exists():
        return {**base, "status": "NOT_RUN"}, [], [], []
    if not (root/"owner_result.json").exists():
        return {**base, "status": "NOT_FINALIZED", "notice": "No finalized owner receipt; no raw payload audited."}, [], [], []
    recorded_plan=checked(root.parent/"plan.json",ledger)
    require(equal(recorded_plan,plan), "continuation plan differs from immutable on-disk declaration")
    durable=execution_record(root.parent,case,plan,plan["cases"].index(case),ledger,ledger[str(root.parent/"plan.json")])
    owner, lifecycle = verify_owner(root, case, plan, ledger)
    original, native = scalar.summarize_trial(root, None)
    for item in original["source_manifest"]:
        checked(root/item["path"], ledger, item["sha256"], binary=True)
    result = {**base, "status": "FAILED_CAPTURE_NOT_ADMITTED", "scalar": original,
        "owner_exit_code": owner["exit_code"], "source_provenance_checked": True, "metadata_sha256": ledger}
    images, futures, states = [], [], []
    if not original["raw_data_available"]:
        for name,pin in tuple(ledger.items()):
            checked(name, {}, pin, binary=True)
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
    require(configuration_equal({key: profile.get(key) for key in configuration}, configuration), "recorded unchanged immediate/zero configuration differs")
    require(manifest["result"]["training_data_approved"] is False and manifest["result"]["development_only"] is True,
        "raw diagnostic denials changed")
    experiment = profile["hold_brake_experiment"]
    require(experiment["emergency_ramp_allowed"] is False and experiment["hold_dwell_or_tail_criteria_changed"] is False and
        experiment["physical_actuation_timing_proven"] is False and experiment["historical_v6_completion_changed"] is False and
        experiment["ramp_first_requested_brake"] == .1 and experiment["ramp_nominal_first_full_brake_command_elapsed_seconds"] == .45 and
        experiment["ramp_nominal_first_full_brake_readback_elapsed_seconds"] == .5 and experiment["immediate_requested_brake"] == 1., "v7 intervention scope differs")
    terminal = profile["terminal_approach_experiment"]
    require(terminal["mode"] == case["arm"] and terminal["physics_profile"] == "reference_10ms" and
        terminal["normal_control_stage"] == "LocalPlanner normal pedals before the unchanged BasicAgent emergency decision" and
        all(terminal[key] is False for key in ("goal_dwell_tail_or_timeout_criteria_changed",
            "original_future_or_native_gate_changes", "emergency_control_modified",
            "physical_actuation_timing_proven", "training_data_approved")), "terminal intervention scope differs")
    require(manifest["capture_contract"]["stopped_brake_steering_suppressed"] is True and
        manifest["capture_contract"]["control_transport"]["mode"] == "acknowledged_batch", "inherited control guards changed")
    for field, prefix in (("source_route", "route"), ("mapping_file", "mapping"), ("calibration_file", "calibration")):
        relative = str(safe(manifest["provenance"][field]).relative_to(ROOT))
        require(relative in plan["input_sha256"] and manifest["provenance"][prefix+"_sha256"] == plan["input_sha256"][relative], "captured fixed input differs")
    require(manifest["provenance"]["goal_stop_helper_sha256"] == plan["source_sha256"]["scripts/e2e/carla_goal_stop_profile.py"], "captured control helper differs")
    require(manifest["provenance"]["physics_substeps_helper_sha256"] ==
        plan["source_sha256"]["scripts/e2e/carla_physics_substeps.py"], "captured substep helper differs")

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
        stage("physics_substeps", lambda: validate_substeps(manifest, case["arm"]))
        stage("initialization", lambda: common.initialization_checks(states, manifest))
        stage("transport", lambda: ack.analyze_transport(states, receipts, bootstrap, wheelbase=2.85))
        stage("hold_and_steering_policy", lambda: terminal_control_protocol(states, native["native_states"], manifest, receipts, case["arm"]))
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
            durable["status"] == "WRAPPER_COMPLETED" and
            all(result[name].get("status") == "PASS" for name in ("physics_substeps", "initialization", "transport", "hold_and_steering_policy")) and
            result.get("camera_pixels", {}).get("all_pixel_and_metadata_checks_clear") is True)
        future = result.get("measured_future", {})
        result["all_original_future_metrics_clear"] = original_metrics_clear(future)
        result["execution_and_scalar_pixel_protocol_clear"] = bool(clear)
        result["status"] = "COMPLETE_DIAGNOSTIC_NOT_ADMITTED" if clear else "FAILED_CAPTURE_OR_QA_NOT_ADMITTED"
    for name, pin in tuple(ledger.items()):
        checked(name, {}, pin, binary=True)
    return result, images, futures, states


def comparison_verdict(trials, pairs):
    """HH_260906 - Completion and every original failure remain visible; one pair is not repeatability evidence."""
    require([(row["case_id"], row["arm"]) for row in trials] == list(CASES), "both planned denominators required")
    rows = {}
    for trial in trials:
        full = trial.get("measured_future", {}).get("full_64_point_anchors", {})
        denominator = full.get("sample_count", 0)
        qa = trial.get("scalar", {}).get("independent_qa", {})
        rows[trial["case_id"]] = dict(status=trial["status"], full_anchor_count=denominator,
            all_original_future_metrics_clear=trial.get("all_original_future_metrics_clear", False),
            original_scalar_flags=qa.get("flags"), terminal_motion=trial.get("terminal_motion"),
            original_speed_rate_qa={cadence: {key:value for key,value in item["by_phase"]["all"].items()
                if key != "phase_boundary_intervals"} for cadence,item in qa.get("speed_rate_qa", {}).items()},
            metrics={name: dict(violating_anchor_count=metric["violating_anchor_count"],
                violating_anchor_fraction=metric["violating_anchor_count"]/denominator if denominator else None,
                violating_step_count=metric["violating_step_count"],
                unique_100ms_endpoint_count=metric.get("unique_episode_relative_100ms_violation_tick_count"),
                maximum=(metric.get("maximum") or {}).get("value"))
                for name,metric in full.get("metrics", {}).items()})
    a,b = (rows[case_id] for case_id,_ in CASES)
    deltas = {}
    for name in a["metrics"].keys() & b["metrics"].keys():
        first,second = a["metrics"][name],b["metrics"][name]
        deltas[name] = {key: second[key]-first[key] if first[key] is not None and second[key] is not None else None
            for key in ("violating_anchor_count","violating_anchor_fraction","violating_step_count","maximum")}
    complete = all(row["status"] == "COMPLETE_DIAGNOSTIC_NOT_ADMITTED" and rows[row["case_id"]]["full_anchor_count"] for row in trials)
    return dict(status="BOTH_COMPLETED_CONDITION_SPECIFIC_NOT_ADMITTED" if complete else
        "INCOMPLETE_OR_FAILED_NO_COMPARATIVE_VERDICT", per_case_counts_and_rates=rows,
        observed_feedback_minus_coast_metric_deltas=deltas,
        feedback_all_original_future_metrics_clear=b["all_original_future_metrics_clear"],
        pre_feedback_prefix=pairs.get("pre_feedback_prefix"),
        planned_case_count=2, repeatability_assessed=False, training_data_approved=False,
        interpretation="Measured task completion is primary. All scalar/native/camera/control gates are unchanged. Report every original future failure and count/rate/maximum separately; smaller curvature alone is not success. One route/seed/weather pair does not establish repeatability or universal robustness.")


def validate_readiness(report, journal, expected_source):
    return prior.validate_readiness(report, journal, expected_source)


def execution_record(campaign, case, plan, index, ledger, plan_sha256):
    directory = campaign/"execution"/case["id"]
    if not directory.exists():
        require(not (campaign/case["id"]).exists(), "raw case exists without durable execution declaration")
        return dict(case_id=case["id"], arm=case["arm"], status="NOT_RUN", wrapper_attempt_count=0)
    declaration = checked(directory/"declaration.json", ledger)
    result = checked(directory/"result.json", ledger)
    require(declaration["schema"] == "carla.v9_case_execution_declaration.v1" and declaration["case_id"] == result["case_id"] == case["id"] and
        declaration["arm"] == case["arm"] and declaration["plan_sha256"] == plan_sha256 and declaration["command"] == case["command"] and
        declaration["maximum_wrapper_attempts"] == 1 and declaration["automatic_retry"] is False and
        declaration["training_data_approved"] is False, "durable declaration differs")
    prior = declaration["prior_case_continuation"]
    require([row["case_id"] for row in prior] == [row["id"] for row in plan["cases"][:index]] and
        all(row["status"] == "COMPLETE_DIAGNOSTIC_NOT_ADMITTED" and row["execution_and_scalar_pixel_protocol_clear"] is True for row in prior), "prior continuation record differs")
    require(result["schema"] == "carla.v9_case_execution_result.v1" and result["automatic_retry"] is False and
        result["training_data_approved"] is False and type(result["wrapper_attempt_count"]) is int and
        instant(plan["declared_at_utc"]) <= instant(declaration["declared_at_utc"]) <= instant(result["completed_at_utc"]), "durable result scope/clock differs")
    exceptional = ("READINESS_INVOCATION_FAILED_NOT_ATTEMPTED", "PIN_MISMATCH_NOT_ATTEMPTED", "WRAPPER_INVOCATION_OR_WAIT_FAILED_STATUS_UNCONFIRMED")
    def retained(readiness_summary):
        return dict(case_id=case["id"], arm=case["arm"], **{key: value for key, value in result.items() if key not in ("case_id", "arm")},
            readiness=readiness_summary, execution_result_path=str((directory/"result.json").relative_to(ROOT)),
            declared_at_utc=declaration["declared_at_utc"], prior_case_continuation=prior,
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
        require(started["schema"] == "carla.v9_wrapper_invocation.v1" and started["case_id"] == case["id"] and
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


def enforce_attempt_order(execution):
    stopped=False
    previous=None
    for record in execution:
        require(not stopped or record["status"]=="NOT_RUN", "execution continued after failed/blocked/not-run case")
        if record["status"]!="NOT_RUN":
            if previous is not None:
                require(instant(previous["completed_at_utc"]) <= instant(record["declared_at_utc"]),
                    "next case was declared before previous completion")
            previous=record
        stopped |= record["status"]!="WRAPPER_COMPLETED"


def verify_continuation_results(execution,trials):
    for index,record in enumerate(execution):
        if record["status"]=="NOT_RUN":
            continue
        declared=record["prior_case_continuation"]
        require(len(declared)==index, "continuation denominator differs")
        for receipt,actual in zip(declared,trials[:index]):
            require(all(receipt[key]==actual.get(key) for key in ("case_id","status",
                "execution_and_scalar_pixel_protocol_clear","all_original_future_metrics_clear")),
                "declared prior continuation disagrees with actual independent audit")


def hold_motion(states):
    return prior.hold_motion(states)


def compare_post_hold_motion(first, second):
    return prior.compare_post_hold_motion(first, second)


def before_feedback_comparison(first, second):
    """HH_260906 - Compare actual paired observations through the first treated source, excluding only its next command."""
    index = next((i for i,row in enumerate(second)
        if row["goal_stop"]["terminal_approach_control"]["normal_control_executed"] and
        row["goal_stop"]["terminal_approach_control"]["decision_at_normal_control"]["eligible"] and
        row["goal_stop"]["terminal_approach_control"]["returned_control"]["throttle"]>0. and
        row["next_control"]["throttle"]>0.), None)
    if index is None or len(first) <= index:
        return dict(status="UNAVAILABLE", comparison_performed=False)
    mismatches=[]
    for i,(a,b) in enumerate(zip(first[:index+1],second[:index+1])):
        fields=[key for key in common.PHYSICAL_FIELDS if not equal(a[key],b[key])]
        if i < index and not equal(a["next_control"],b["next_control"]):
            fields.append("next_control_before_feedback_source")
        if fields:
            mismatches.append(dict(native_index=i,fields=fields,first_frame=a["frame"],second_frame=b["frame"]))
    return dict(status="DIFFERENT" if mismatches else "EXACT_MATCH", comparison_performed=True,
        compared_native_count=index+1, mismatched_native_count=len(mismatches), mismatches=mismatches,
        physical_fields=list(common.PHYSICAL_FIELDS), feedback_source_next_controls=[first[index]["next_control"],second[index]["next_control"]],
        excluded=["absolute timestamps/frame IDs", "mode/policy metadata", "only the first feedback-source next_control"],
        normalization_or_retiming_performed=False)


def terminal_motion(states):
    """HH_260906 - Count real normal requests and following observed travel without claiming actuator causality."""
    active=[]
    for index,row in enumerate(states):
        gp=row["goal_stop"]
        control=gp["terminal_approach_control"]
        if control["normal_control_executed"] and control["decision_at_normal_control"]["eligible"]:
            active.append(index)
    intervals=[(states[i],states[i+1]) for i in active if i+1<len(states)]
    positive=[i for i in active if states[i]["goal_stop"]["terminal_approach_control"]["returned_control"]["throttle"]>0.]
    retained=[i for i in positive if states[i]["next_control"]["throttle"]>0.]
    first=states[active[0]] if active else None
    driving=[row for row in states if row["capture_phase"]=="driving"]
    final=driving[-1] if driving else None
    return dict(status="OBSERVED" if active else "NOT_ACTIVATED", eligible_normal_request_count=len(active),
        positive_throttle_request_count=len(positive), following_observed_interval_count=len(intervals),
        positive_throttle_retained_in_next_control_count=len(retained),
        positive_throttle_superseded_before_next_control_count=len(positive)-len(retained),
        following_observed_virtual_base_travel_m=sum(math.hypot(b["x"]-a["x"],b["y"]-a["y"]) for a,b in intervals),
        first_source_frame=first["frame"] if first else None, first_source_timestamp=first["timestamp"] if first else None,
        first_source_remaining_arc_m=first["goal_stop"]["remaining_route_arc_m"] if first else None,
        final_driving_remaining_arc_m=final["goal_stop"]["remaining_route_arc_m"] if final else None,
        final_driving_speed_mps=math.hypot(final["vx"],final["vy"]) if final else None,
        measured_goal_complete=final["goal_stop"]["complete"] if final else False,
        hold_latched=any(row["goal_stop"]["hold_brake_latched"] for row in states),
        normal_request_is_physical_actuation=False, training_data_approved=False,
        interpretation="Distance is measured on actual following native intervals, not attributed to throttle or substituted for raw labels.")


def run(campaign_root, plan_path, plan_sha256, output_dir=None):
    campaign, plan_path, ledger = safe(campaign_root), safe(plan_path), {}
    required_sha(plan_sha256)
    plan = checked(plan_path, ledger, plan_sha256)
    validate_plan(plan)
    require(campaign == safe(ROOT/CAMPAIGN_PATH) and plan_path == campaign/"plan.json", "explicit campaign/plan path differs")
    check_analysis_pins(plan, ledger)
    for category in ("source_sha256", "input_sha256", "audit_math_sha256", "orchestration_sha256", "analysis_sha256"):
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
    execution = [execution_record(campaign, case, plan, index, ledger, plan_sha256) for index, case in enumerate(plan["cases"])]
    enforce_attempt_order(execution)
    trials, images, futures, witnesses, residuals, physical = [], [], [], [], [], {}
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
            result["terminal_motion"] = terminal_motion(states)
            values = native_residuals(states)
            result["native_residual_interval_count"] = len(values)
            residuals.extend(dict(case_id=case["id"], arm=case["arm"], **row) for row in values)
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
    verify_continuation_results(execution,trials)
    first,second=(physical[case_id] for case_id,_ in CASES)
    pairs = dict(pre_feedback_prefix=before_feedback_comparison(first,second),
        cross_arm_warmup=common.compare_physical(first,second,warmup_only=True),
        complete_physical_trace=common.compare_physical(first,second))
    verdict = comparison_verdict(trials, pairs)
    pairs["prospective_primary_comparison"] = verdict
    pairs["post_hold_motion_only_A_B"] = compare_post_hold_motion(first,second)
    summary = dict(schema=SCHEMA, status="DIAGNOSED_NOT_ADMITTED", generated_at_utc=datetime.now(timezone.utc).isoformat(),
        plan_sha256=plan_sha256, plan_path=str(plan_path), source_sha256=sources,
        execution_attempts_evidence=execution, physics_profile_by_arm=PHYSICS,
        planned_case_count=2, wrapper_attempted_case_count=sum(row["wrapper_attempt_count"] for row in execution),
        readiness_observed_case_count=sum(row.get("readiness",{}).get("sample_count",0)>0 for row in execution),
        readiness_invocation_case_count=sum(row["status"] != "NOT_RUN" for row in execution),
        training_data_approved=False, research_only=True, dataset_export=False, target_retiming=False, gate_changes=False,
        learned_model_control=False, vehicle_control_approved=False, cases=trials,
        case_status_counts=dict(Counter(row["status"] for row in trials)),
        both_completed_diagnostics=all(row["status"] == "COMPLETE_DIAGNOSTIC_NOT_ADMITTED" for row in trials),
        audited_native_state_count=sum(row["native_state_count"] or 0 for row in trials),
        audited_camera_anchor_count=sum(row["camera_anchor_count"] or 0 for row in trials),
        audited_original_jpeg_count=len(images), future_anchor_record_count=len(futures),
        control_policy_witness_count=len(witnesses), control_policy_witness_file="control_policy_witnesses.jsonl",
        native_residual_interval_count=len(residuals), native_residual_file="native_residuals.jsonl",
        primary_comparison_status=verdict["status"], metadata_sha256=ledger,
        limitations=["All original full-future gates and failed rows are retained; no training or vehicle approval.",
            "Maximum substep settings do not prove actual internal timesteps or physical accuracy. Normal immediate hold, emergency authority and zero steering remain unchanged.",
            "Both arms are fresh v9 trials with fixed reference_10ms, immediate hold and zero low-speed steering; only bounded terminal throttle feedback differs.",
            "The pre-feedback prefix comparison excludes only the first treated source's next command. Absolute clocks/IDs are excluded without retiming.",
            "Count/rate decreases can coexist with a worse maximum. One fixed Town07 route/seed/weather pair cannot establish repeatability, broad robustness or unique physical cause.",
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
        for name, rows in (("image_inventory.jsonl", images), ("future_windows.jsonl", futures), ("control_policy_witnesses.jsonl", witnesses), ("native_residuals.jsonl", residuals)):
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
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--output-dir", type=Path)
    group.add_argument("--verify-only", action="store_true")
    args = parser.parse_args(argv)
    try:
        summary, _ = run(args.campaign_root, args.plan, args.plan_sha256, args.output_dir)
    except (ValueError, KeyError, TypeError, OSError) as error:
        parser.exit(1, f"terminal approach summary failed: {error}\n")
    print(json.dumps({key: summary[key] for key in ("status", "case_status_counts", "audited_native_state_count",
        "audited_camera_anchor_count", "primary_comparison_status")}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
