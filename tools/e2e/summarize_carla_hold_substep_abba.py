#!/usr/bin/env python3
"""HH_260906 - Audit the fresh fixed-cadence numerical substep ABBA without changing labels, control or admission."""
from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from scripts.e2e import summarize_carla_hold_brake_ramp_abba as prior
from scripts.e2e import carla_physics_substeps as substeps
common = prior.common

SCHEMA = "carla.hold_substep_abba_summary.v1"
PROFILE = "comfortable_v7_hold_brake_ramp"
CASES = (("01_reference_A1", "reference"), ("02_fine_B1", "fine"), ("03_fine_B2", "fine"), ("04_reference_A2", "reference"))
PHYSICS = {"reference": "reference_10ms", "fine": "fine_5ms"}
EXECUTION_SOURCES = prior.common.EXECUTION_SOURCES | {"scripts/e2e/carla_physics_substeps.py"}
ANALYSIS_PINS = {"scripts/e2e/summarize_carla_hold_brake_ramp_abba.py": "adca365e601239a4f6b43230e827f5f72caa272b40a946e6c74fdba8437cacef", "scripts/e2e/summarize_carla_low_speed_steering_abba.py": "53b42cf4e1be70f64ead8c6d143aed4dabc46e5f486407f0bacb9f70e6410da1"}
SELF_PATH = "scripts/e2e/summarize_carla_hold_substep_abba.py"
CAMPAIGN_PATH = "artifacts/training/2026-09-15/native_boundary_and_substeps/substep_abba"
ORCHESTRATION_PATHS = {"scripts/e2e/probe_carla_port_readiness.py", *{
    "artifacts/training/2026-09-15/native_boundary_and_substeps/substep_abba/"+name for name in ("run_case.py", "render_evidence.py")}}
COMMON_SOURCE_SHA256 = "53b42cf4e1be70f64ead8c6d143aed4dabc46e5f486407f0bacb9f70e6410da1"
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
    """HH_260906 - Verify the eleven-field before/requested/after receipt; after means application, not restoration."""
    require(arm in PHYSICS, "unknown numerical arm")
    record = manifest["capture_contract"]["physics_substeps"]
    profile = PHYSICS[arm]
    expected = substeps.new_record(profile)
    require(record["status"] == "PASS" and "error" not in record, "substep setup failed or was not reached")
    for key, value in expected.items():
        if key not in ("status", "before", "requested", "after", "notice"):
            require(equal(record.get(key), value), "substep scope/profile differs: "+key)
    fields = {*substeps.BOOL_FIELDS, *substeps.FLOAT_FIELDS, "max_substeps"}
    before = record["before"]
    require(isinstance(before, dict) and set(before) == fields, "eleven original settings required")
    for key in substeps.BOOL_FIELDS:
        require(type(before[key]) is bool, "setting boolean type differs")
    for key in substeps.FLOAT_FIELDS:
        if key == "fixed_delta_seconds" and before[key] is None:
            continue
        require(type(before[key]) is float and number(before[key]) >= 0., "setting float type/value differs")
    require(type(before["max_substeps"]) is int and before["max_substeps"] == 10 and
        before["substepping"] is True and before["max_substep_delta_time"] == .01 and
        before["no_rendering_mode"] is False, "original true/.01/10/rendering reference differs")
    wanted = dict(before, synchronous_mode=True, fixed_delta_seconds=.05,
        max_substep_delta_time=substeps.PROFILES[profile])
    require(substeps.equal_settings(record["requested"], wanted) and
        substeps.equal_settings(record["after"], wanted), "eleven-field requested/readback differs")
    runtime = manifest["runtime"]["capture_world_settings"]
    six = {key: wanted[key] for key in ("synchronous_mode", "fixed_delta_seconds", "no_rendering_mode",
        "substepping", "max_substep_delta_time", "max_substeps")}
    require(equal(runtime, six), "native runtime six-field snapshot differs")
    original = manifest["runtime"]["original_world_settings"]
    require(equal(original, {key: before[key] for key in six}), "original runtime/eleven-field receipt differs")
    return dict(status="PASS", profile=profile, before=before, requested=record["requested"], after=record["after"],
        measured_internal_substeps=False, independent_restoration_proven=False,
        interpretation="A maximum numerical setting with readback, not measured internal integration intervals or physical accuracy.")


def validate_plan(plan):
    require(plan["schema"] == "carla.hold_substep_abba_plan.v1" and
        [(case["id"],case["arm"]) for case in plan["cases"]] == list(CASES), "fixed numerical ABBA plan required")
    require(type(plan["maximum_attempts_per_revision"]) is int and plan["maximum_attempts_per_revision"] == 4 and
        all(type(case["maximum_attempts"]) is int and case["maximum_attempts"] == 1 for case in plan["cases"]),
        "four cases with one attempt each required")
    require(plan["physics_profile_by_arm"] == PHYSICS, "only the explicit 10ms/5ms comparison is supported")
    require(set(plan["orchestration_sha256"]) == ORCHESTRATION_PATHS, "fixed readiness/runner/renderer closure required")
    expected_bounds = dict(control_transport="acknowledged_batch", agent_initialization="after_bootstrap",
        target_speed_kmh=28.8, physics_hz=20, camera_hz=10, stationary_warmup_seconds=3.5,
        required_measured_stop_dwell_seconds=2., stationary_tail_seconds=6.5, quality="Epic", map="Town07",
        seed=0, weather="ClearNoon", wheelbase_m=2.85, fixed_max_substeps=10,
        explicit_settings_readback_required=True, vehicle_type="vehicle.toyota.prius", scenario="straight",
        spawn_z_offset_m=.5, maximum_total_simulation_seconds=180, collector_wall_timeout_seconds=900)
    require(all(configuration_equal(plan.get(key), value) for key,value in expected_bounds.items()), "fixed experiment bounds differ")
    require(equal(plan["required_original_substeps"], dict(substepping=True,max_substep_delta_time=.01,max_substeps=10)) and
        equal(plan["requested_max_substep_delta_time_by_arm"], dict(reference=.01,fine=.005)), "declared numerical settings differ")
    require(configuration_equal(plan["protocol_configuration"], asdict(profiles.HoldBrakeRampDevelopmentGoalStopConfig())),
        "one unchanged v7 immediate/zero control configuration required")
    require(set(plan["source_sha256"]) == EXECUTION_SOURCES and set(plan["input_sha256"]) == common.INPUT_PATHS and
        set(plan["audit_math_sha256"]) == common.AUDIT_MATH_PATHS, "native execution/input/original-math closure differs")
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
        for key,value in {"--goal-stop-profile":PROFILE,"--hold-brake-mode":"immediate",
            "--low-speed-steering-mode":"zero","--physics-substep-profile":PHYSICS[case["arm"]],
            "--physics-hz":"20","--capture-hz":"10","--target-speed-kmh":"28.8",
            "--control-transport":"acknowledged_batch","--agent-initialization":"after_bootstrap"}.items():
            require(flags.count(key)==1 and flags.index(key)+1<len(flags) and flags[flags.index(key)+1]==value,
                "fixed collector flag differs: "+key)
        require(command[2] == CAMPAIGN_PATH+"/"+case["id"] and command[3] in common.INPUT_PATHS and
            command[4:command.index("--")] == ["--port","2100","--quality","Epic","--wall-timeout-sec","900","--capture-mode","expert"],
            "fixed case output/owned admission flags differ")
        normalized=list(command)
        normalized[2]="DECLARED_CASE_OUTPUT"
        normalized[normalized.index("--physics-substep-profile")+1]="DECLARED_NUMERICAL_ARM"
        normalized_commands.append(normalized)
    require(all(command==normalized_commands[0] for command in normalized_commands),
        "commands change more than case output and numerical substep profile")
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
    require(intent["schema"] == "carla.v8_wrapper_invocation.v1" and intent["case_id"] == case["id"] and
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
    """HH_260906 - Analyze every actual native interval; never substitute integrated positions for raw labels."""
    output=[]
    def actor(row):
        pose=row["actor_snapshot_transform_carla"]
        return [number(pose["x"]),-number(pose["y"])]
    def velocity(row):
        return [number(row["world_velocity_carla"][0]),-number(row["world_velocity_carla"][1])]
    def base_velocity(row):
        c,s=math.cos(number(row["yaw"])),math.sin(number(row["yaw"]))
        vx,vy=number(row["vx"]),number(row["vy"])
        return [c*vx-s*vy,s*vx+c*vy]
    for index,(a,b) in enumerate(zip(states,states[1:])):
        dt=number(b["timestamp"])-number(a["timestamp"])
        require(0.<dt and abs(dt-.05)<=1e-4 and b["frame"]==a["frame"]+1, "native residual interval gap")
        dp_actor=[y-x for x,y in zip(actor(a),actor(b))]
        dp_base=[number(b[k])-number(a[k]) for k in ("x","y")]
        actor_i=[dt*.5*(x+y) for x,y in zip(velocity(a),velocity(b))]
        base_i=[dt*.5*(x+y) for x,y in zip(base_velocity(a),base_velocity(b))]
        actor_e=[x-y for x,y in zip(dp_actor,actor_i)]
        base_e=[x-y for x,y in zip(dp_base,base_i)]
        yaw_delta=math.atan2(math.sin(b["yaw"]-a["yaw"]),math.cos(b["yaw"]-a["yaw"]))
        output.append(dict(native_source_index=index, source_frame=a["frame"], destination_frame=b["frame"],
            source_timestamp=a["timestamp"],destination_timestamp=b["timestamp"],dt_seconds=dt,
            source_phase=a["capture_phase"],destination_phase=b["capture_phase"],
            source_pilot_state=a["goal_stop"]["pilot_state"],destination_pilot_state=b["goal_stop"]["pilot_state"],
            actor_delta_ros_xy_m=dp_actor,virtual_base_delta_ros_xy_m=dp_base,
            actor_signed_velocity_integral_ros_xy_m=actor_i,virtual_base_signed_velocity_integral_ros_xy_m=base_i,
            actor_pose_minus_velocity_residual_ros_xy_m=actor_e,virtual_base_pose_minus_velocity_residual_ros_xy_m=base_e,
            actor_residual_norm_m=math.hypot(*actor_e),virtual_base_residual_norm_m=math.hypot(*base_e),
            pose_offset_delta_ros_xy_m=[x-y for x,y in zip(dp_base,dp_actor)],
            applied_planar_velocity_shift_integral_ros_xy_m=[x-y for x,y in zip(base_i,actor_i)],
            measured_yaw_change_rad=yaw_delta,
            signed_yaw_rate_integral_rad=dt*.5*(number(a["yaw_rate"])+number(b["yaw_rate"])),
            source_next_control=a["next_control"],destination_current_control=b["current_control"],
            physical_substep_truth_proven=False,training_data_approved=False))
    return output


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
    configuration = plan["protocol_configuration"]
    profile = manifest["capture_contract"]["goal_stop_profile"]
    require(configuration_equal({key: profile.get(key) for key in configuration}, configuration), "recorded unchanged immediate/zero configuration differs")
    require(manifest["result"]["training_data_approved"] is False and manifest["result"]["development_only"] is True,
        "raw diagnostic denials changed")
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
        stage("hold_and_steering_policy", lambda: prior.hold_and_steering_protocol(states, native["native_states"], manifest, receipts, "immediate"))
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
            cross.append(dict(reference_case=a, fine_case=b, fine_minus_reference_failing_anchor_count=count_delta,
                fine_minus_reference_failing_anchor_fraction=rate_delta, other_original_gates_not_regressed=no_other,
                fine_minus_reference_maximum_curvature_rad_per_m=max_delta, worst_observed_curvature_increased=max_delta > 0,
                favorable_count_and_rate=count_delta < 0 and rate_delta < 0 and no_other))
    favorable = complete and len(cross) == 4 and all(row["favorable_count_and_rate"] for row in cross)
    return dict(status="INCOMPLETE_OR_FAILED_NO_COMPARATIVE_VERDICT" if not complete else
        "FAVORABLE_CONDITION_SPECIFIC_SIGNAL_NOT_ADMITTED" if favorable else "MIXED_OR_NO_CONSISTENT_IMPROVEMENT_NOT_ADMITTED",
        per_case_counts_and_rates=rows, observed_cross_arm_comparisons=cross, required_cross_arm_comparison_count=4,
        available_cross_arm_comparison_count=len(cross), exact_repeats_and_cross_arm_observations=matched,
        training_data_approved=False, criterion="Both fine-substep repeats improve full original curvature count AND rate against BOTH reference repeats, without other original gate count/rate regressions and with all operational/scalar/camera/control checks clear. Repeat/cross-arm state equality is separately reported secondary evidence; the numerical treatment begins before all recorded states.",
        interpretation="Report worse maxima independently even if count/rate improve. Every remaining original failure is unresolved; matched results do not identify a unique physical cause or grant admission.")


def validate_readiness(report, journal, expected_source):
    return prior.validate_readiness(report, journal, expected_source)


def execution_record(campaign, case, plan, index, ledger, plan_sha256):
    directory = campaign/"execution"/case["id"]
    if not directory.exists():
        require(not (campaign/case["id"]).exists(), "raw case exists without durable execution declaration")
        return dict(case_id=case["id"], arm=case["arm"], status="NOT_RUN", wrapper_attempt_count=0)
    declaration = checked(directory/"declaration.json", ledger)
    result = checked(directory/"result.json", ledger)
    require(declaration["schema"] == "carla.v8_case_execution_declaration.v1" and declaration["case_id"] == result["case_id"] == case["id"] and
        declaration["arm"] == case["arm"] and declaration["plan_sha256"] == plan_sha256 and declaration["command"] == case["command"] and
        declaration["maximum_wrapper_attempts"] == 1 and declaration["automatic_retry"] is False and
        declaration["training_data_approved"] is False, "durable declaration differs")
    prior = declaration["prior_case_continuation"]
    require([row["case_id"] for row in prior] == [row["id"] for row in plan["cases"][:index]] and
        all(row["status"] == "COMPLETE_DIAGNOSTIC_NOT_ADMITTED" and row["execution_and_scalar_pixel_protocol_clear"] is True for row in prior), "prior continuation record differs")
    require(result["schema"] == "carla.v8_case_execution_result.v1" and result["automatic_retry"] is False and
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
        require(started["schema"] == "carla.v8_wrapper_invocation.v1" and started["case_id"] == case["id"] and
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


def run(campaign_root, plan_path, plan_sha256, output_dir=None):
    campaign, plan_path, ledger = safe(campaign_root), safe(plan_path), {}
    required_sha(plan_sha256)
    plan = checked(plan_path, ledger, plan_sha256)
    validate_plan(plan)
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
    pairs = dict(reference_repeat_A1_A2=common.compare_physical(physical[CASES[0][0]], physical[CASES[3][0]]),
        fine_repeat_B1_B2=common.compare_physical(physical[CASES[1][0]], physical[CASES[2][0]]),
        cross_arm_warmup_A1_B1=common.compare_physical(physical[CASES[0][0]], physical[CASES[1][0]], warmup_only=True),
        cross_arm_warmup_B2_A2=common.compare_physical(physical[CASES[2][0]], physical[CASES[3][0]], warmup_only=True))
    verdict = comparison_verdict(trials, pairs)
    pairs["prospective_primary_comparison"] = verdict
    pairs["post_hold_motion_only_A1_B1"] = compare_post_hold_motion(physical[CASES[0][0]], physical[CASES[1][0]])
    pairs["post_hold_motion_only_B2_A2"] = compare_post_hold_motion(physical[CASES[2][0]], physical[CASES[3][0]])
    summary = dict(schema=SCHEMA, status="DIAGNOSED_NOT_ADMITTED", generated_at_utc=datetime.now(timezone.utc).isoformat(),
        plan_sha256=plan_sha256, plan_path=str(plan_path), source_sha256=sources,
        execution_attempts_evidence=execution, physics_profile_by_arm=PHYSICS,
        planned_case_count=4, wrapper_attempted_case_count=sum(row["wrapper_attempt_count"] for row in execution),
        readiness_observed_case_count=sum(row.get("readiness",{}).get("sample_count",0)>0 for row in execution),
        readiness_invocation_case_count=sum(row["status"] != "NOT_RUN" for row in execution),
        training_data_approved=False, research_only=True, dataset_export=False, target_retiming=False, gate_changes=False,
        learned_model_control=False, vehicle_control_approved=False, cases=trials,
        case_status_counts=dict(Counter(row["status"] for row in trials)),
        all_four_completed_diagnostics=all(row["status"] == "COMPLETE_DIAGNOSTIC_NOT_ADMITTED" for row in trials),
        audited_native_state_count=sum(row["native_state_count"] or 0 for row in trials),
        audited_camera_anchor_count=sum(row["camera_anchor_count"] or 0 for row in trials),
        audited_original_jpeg_count=len(images), future_anchor_record_count=len(futures),
        control_policy_witness_count=len(witnesses), control_policy_witness_file="control_policy_witnesses.jsonl",
        native_residual_interval_count=len(residuals), native_residual_file="native_residuals.jsonl",
        primary_comparison_status=verdict["status"], metadata_sha256=ledger,
        limitations=["All original full-future gates and failed rows are retained; no training or vehicle approval.",
            "Maximum substep settings do not prove actual internal timesteps or physical accuracy. Normal immediate hold, emergency authority and zero steering remain unchanged.",
            "Both arms are fresh numerical-substep trials with unchanged v7 immediate/zero control; no previous campaign is retried.",
            "Cross-arm warmup is already under the numerical treatment; it is not an untreated prefix. Absolute clocks/IDs are excluded, without retiming.",
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
        parser.exit(1, f"substep summary failed: {error}\n")
    print(json.dumps({key: summary[key] for key in ("status", "case_status_counts", "audited_native_state_count",
        "audited_camera_anchor_count", "primary_comparison_status")}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
