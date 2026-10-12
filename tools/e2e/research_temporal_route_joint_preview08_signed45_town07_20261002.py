"""HH_261002 - Prospectively compare only0.8s versus0.3s causal learned speed tracking in all three signed45 arms."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import math
from pathlib import Path
import re
from types import SimpleNamespace

from . import research_temporal_route_joint_thread4_signed45_town07_20261002 as previous
from . import collect_temporal_route_joint_preview08_signed45_20261002 as live

ROOT, base, parent = previous.ROOT, previous.base, previous.parent
MODULE = "scripts.e2e.research_temporal_route_joint_preview08_signed45_town07_20261002"
SCHEMA = "portable_e2e.actual_route_joint_thread4_preview08_signed45_three_arms.20261002.v1"
FROZEN_OWNER_SHA = "8130063784148eda4a3ec985e42918ca54aba8cec948761844e87bcb82915ce4"
BASELINE = ROOT/"storage/experiments/validation/2026-10-02/temporal_route_joint_thread4_signed45_batch_audit_v1/batch_outcome.json"
BASELINE_SHA = "314bd92fdb2b0069c068fc0f973fbc37f08aff207e44578590a9dc31200e5258"
require, digest, read_json = base.require, base.digest, base.read_json
_fixed, _sources, _load, _diagnostic, _summary = previous.fixed_contract, previous.runtime_sources, previous.load_model, previous.diagnostic_report, previous.batch_summary
_disk = previous.disk_check
_active, _baseline = False, None


def batch_paths(run_id):
    require(isinstance(run_id, str) and re.fullmatch(r"v[1-9][0-9]*", run_id), "fresh preview research version required")
    output = ROOT/("storage/experiments/validation/2026-10-02/temporal_route_joint_preview08_signed45_town07_"+run_id)
    return output, output.with_name(output.name+"_plan.json")


def baseline_proof(pins):
    """HH_261002 - Verify all prior outcomes and complete source hashes without duplicating their JPEG trees."""
    require(digest(BASELINE) == BASELINE_SHA, "complete original0.3s actual report changed")
    result = read_json(BASELINE)
    require(result["final_endpoint_sha256"] == pins and result["prospective_trials"] == 3
        and result["joint_motion_body_policy_wall_trial_pass_count"] == 0
        and result["all_failed_tail_retained"] is True and result["all_three_complete_thread_readbacks_passed"] is True
        and result["all_actual_native_states"] == 2700 and result["all_actual_camera_bundles"] == 1350
        and result["all_actual_ACKs"] == 2712 and result["actual_JPEG_assets"] == 8100
        and [row["trial_index"] for row in result["trials"]] == [0, 1, 2]
        and result["cpu_thread_numerical_preset"] == previous.numerical_preset(), "same complete three-arm CPU4 baseline required")
    sources = result["source_sha256"]
    require(isinstance(sources, dict) and sources, "complete prior output/source proof required")
    for name, sha in sources.items():
        path = Path(name)
        require(path.is_absolute() and path.resolve() == path and path.is_relative_to(ROOT) and not path.is_symlink()
            and isinstance(sha, str) and re.fullmatch(r"[0-9a-f]{64}", sha) and digest(path) == sha,
            "original0.3s actual source/artifact changed: "+name)
    acceleration = []
    for index in range(3):
        output = Path(result["batch"])/("trial_%02d"%index)
        episodes = [path for path in (output/"episode",output/"episode.partial") if path.is_dir()]
        require(len(episodes) == 1 and str(episodes[0]/"states.jsonl") in sources,
                "exact original three-arm native acceleration source required")
        acceleration.append(dict(trial_index=index, **acceleration_report(episodes[0])))
    result["prospective_preview_acceleration_guard_applied_to_all_original_baseline_arms"] = acceleration
    return result


def calibration_contract():
    return dict(schema=live.PREVIEW_SCHEMA, original_speed_preview_s=.3, configured_speed_preview_s=.8,
        sampling="min(len(speed)-1,max(0,round(preview_s/timestep_s)-1)) on the unchanged selected64 knots",
        either_preview_target_at_or_below_original_STOP_threshold_preserves_original_tracker=True,
        any_available_non_GO_selected_intent_preserves_original_tracker=True, original_STOP_threshold_mps=.05,
        actual_runtime_intent="explicit_cruise_GO_not_trained_STOP", unchanged_PI_target_cap_mps=2.4,
        unchanged_PI_maximum_throttle=.5, no_new_throttle_floor_or_cap=True,
        possible_startup_acceleration_or_native_speed_overshoot_retained_as_failure=True,
        additional_prospective_full_driving_actor_world_XY_speed_rate_guard_mps2=dict(minimum=-6.,maximum=3.),
        additional_guard_conjoined_with_old_motion_body_policy_wall_not_original_threshold_replacement=True,
        world_XY_acceleration_norm_and_abs_speed_rate2p9_reported_as_distinct_observations=True,
        raw_NN_decoder_selected_trajectory_actual_v0_teacher_and_all_gate_thresholds_unchanged=True,
        controller_calibration_not_more_trained_weights_or_model_promotion=True,
        unchanged_policy_deadline_seconds=.1, unchanged_observed_bundle_to_ACK_deadline_seconds=.1,
        unchanged90m_three_arm_threshold=True, all_normal_controller_configuration_readbacks_required=True,
        baseline_report_path=str(BASELINE), baseline_report_sha256=BASELINE_SHA,
        full_three_prior_failures_retained=True, full_MAIN9_completion_claimed=False)


def fixed_contract(run_id, longitudinal_pi):
    result = _fixed(run_id, longitudinal_pi)
    result.update(schema=SCHEMA, experiment="Same final6000/CPU4 three-arm signed45 with only causal speed preview0.8 calibration",
        causal_selected_speed_preview_calibration=calibration_contract(),
        original_decoder_gate_selected_tracker_math_unchanged_except_disclosed_PI_headroom=False,
        learned_selected_decoded_path_actual_v0_PI_and_handoff_math_unchanged=False,
        actual_same_2p40_PI_45seconds_and_all_original_safety_thresholds=True,
        tracker_sampling_changed_only_by_declared_preview_with_original_STOP_guard=True,
        actual_same_V3_baseline_physical_setup=False,
        baseline_actual_output_source_sha256=dict(_baseline["source_sha256"]),
        baseline_actual_native_acceleration_under_same_additive_guard=_baseline[
            "prospective_preview_acceleration_guard_applied_to_all_original_baseline_arms"],
        prior_output_hashes_verified_before_and_after_not_duplicated_as_camera_assets=True,
        sealed_baseline_report_archive_bytes_per_trial=BASELINE.stat().st_size,
        additional_three_trial_baseline_report_archive_reserve_bytes=3*BASELINE.stat().st_size)
    return result


def runtime_sources():
    require(digest(Path(previous.__file__)) == FROZEN_OWNER_SHA, "frozen thread4 owner changed")
    live.verify_frozen()
    pins = _sources()
    for path in base.imported_source_closure((Path(__file__).resolve(),)):
        name, sha = str(path.relative_to(ROOT)), digest(path)
        require(name not in pins or pins[name] == sha, "preview import closure conflicts with a frozen source")
        pins[name] = sha
    return pins


def load_model(weights, metadata, **pins):
    model = _load(weights, metadata, **pins)
    model._research_speed_preview_calibration = live.PREVIEW_SCHEMA
    return model


def disk_check(free=None):
    result = _disk(free)
    extra = 3*BASELINE.stat().st_size
    require(result["free_bytes"] >= result["required_bytes"]+extra, "complete prior0.3 report archive reserve unavailable")
    return dict(result, baseline_report_archive_reserve_bytes=extra, required_bytes=result["required_bytes"]+extra)


def configuration_report(episode):
    """HH_261002 - Match all normal controller witnesses and every fresh tracker to the untouched raw command ledger."""
    records = parent.jsonl(episode/"speed_preview_configuration.jsonl")
    timing = parent.jsonl(episode/"wall_timing.jsonl")
    commands = parent.jsonl(episode/"learned_control.jsonl")
    normal_timing = [r for r in timing if r.get("command_reason") in
        ("initial_drive_control", "next_driving_control") and r.get("command_frame") is not None]
    expected = [(r["command_frame"], r["command_reason"]) for r in normal_timing]
    actual = [(r["command_frame"], r["command_reason"]) for r in records]
    require(expected and actual == expected and len(actual) == len(set(actual)), "complete normal preview readback denominator differs")
    controller = records[0]["actual_policy_controller"]
    live.runtime.ControllerConfig(**controller).validate()
    require(controller["speed_preview_s"] == .8 and len(controller) == 9, "actual measured nine-field controller differs")
    require(all(controller[name] == value for name,value in dict(speed_cap_mps=2.5, minimum_lookahead_m=1.5,
        lookahead_time_s=.8, throttle_gain=.7, brake_gain=.7, maximum_throttle=.5).items()),
        "preview calibration changed an original non-geometric controller parameter")
    # HH_261002 - Startup decisions have no command_frame field; bind them to the actual transport ACK, not fabricated frames.
    command_map = {r["ack_receipt_sequence"]: r for r in commands}
    require(len(command_map) == len(commands) == len(expected), "raw command/witness chain differs")
    fresh_count, guarded_count = 0, 0
    for row, wall in zip(records, normal_timing):
        raw = command_map[wall["ACK_sequence"]]
        require(raw["command_reason"] == wall["command_reason"], "actual ACK command reason differs")
        require(row["schema"] == live.PREVIEW_SCHEMA and row["actual_policy_controller"] == controller
            and wall["proposal_started_monotonic_ns"] <= row["proposal_started_monotonic_ns"]
            <= row["proposal_finished_monotonic_ns"] <= wall["proposal_finished_monotonic_ns"]
            <= wall["ACK_started_monotonic_ns"] <= wall["ACK_finished_monotonic_ns"], "controller witness timing/configuration differs")
        require(row["control_source"] == raw.get("control_source")
            and row["learned_control_proposed"] == bool(raw.get("learned_control_proposed"))
            and row["held_between_camera_frames"] == bool(raw.get("held_between_camera_frames"))
            and row["inference_frame"] == raw.get("inference_frame")
            and row["actual_returned_control"] == raw.get("control"), "preview witness differs from original returned raw decision")
        fresh = raw.get("held_between_camera_frames") is False
        receipt = row["tracker_receipt"]
        if receipt is not None:
            require(fresh and raw.get("selected") is not None, "held/startup proposal reintegrated a tracker")
            selected = SimpleNamespace(**raw["selected"])
            expected_receipt = live.preview_selection(selected, live.runtime.ControllerConfig(**controller))
            require({k: v for k,v in receipt.items() if k != "original_tracker_control"} == expected_receipt,
                    "selected causal knot/STOP guard witness differs")
            require(receipt["original_tracker_control"]["desired_speed_mps"] == expected_receipt["effective_selected_target_mps"],
                    "recorded tracker changed its selected target")
            if raw.get("learned_control_proposed"):
                require(raw["controller"] == controller and raw["control"]["desired_speed_mps"] == expected_receipt["effective_selected_target_mps"],
                        "accepted raw control/configuration differs")
            fresh_count += 1; guarded_count += int(expected_receipt["original_STOP_tracker_preserved"])
        elif fresh and raw.get("learned_control_proposed"):
            require(False, "accepted fresh proposal lacks exact selected-knot preview witness")
    require(fresh_count > 0, "no actual fresh preview control witness")
    return dict(passed=True, status="PASS_COMPLETE_PREVIEW_CONFIGURATION", normal_proposal_denominator=len(expected),
        controller_readback_count=len(records), fresh_tracker_count=fresh_count, original_STOP_or_non_GO_fallback_count=guarded_count,
        actual_controller=controller, actual_geometry_not_nominal_manifest_substitution=True,
        source_sha256={str(episode/name):digest(episode/name) for name in
            ("speed_preview_configuration.jsonl", "wall_timing.jsonl", "learned_control.jsonl")})


def acceleration_report(episode):
    # HH_261002 - Add a prospectively declared full driving-rate screen; scalar acceleration and2.9 observations are distinct.
    rows = parent.jsonl(episode/"states.jsonl")
    require(len(rows) == 900 and all(b["frame"] == a["frame"]+1
        and abs(b["timestamp"]-a["timestamp"]-.05) <= 1.e-6 for a,b in zip(rows,rows[1:])),
        "complete900 native20Hz acceleration chain required")
    first = next((i for i,r in enumerate(rows) if r.get("capture_phase") == "driving"),None)
    driving = rows[first:] if first is not None else []
    require(driving and all(r.get("capture_phase") == "driving" for r in driving),
            "all native rows after first driving tick must remain in the acceleration denominator")
    scalar = [math.hypot(*r["world_acceleration_carla"][:2]) for r in driving]
    intervals = rows[max(0,first-1):]
    rates = [(math.hypot(*b["world_velocity_carla"][:2])-math.hypot(*a["world_velocity_carla"][:2]))/(b["timestamp"]-a["timestamp"])
        for a,b in zip(intervals, intervals[1:])]
    require(len(rates) == len(driving)-(first == 0), "complete destination-driving native rate denominator required")
    require(scalar and rates and all(math.isfinite(v) for v in scalar+rates), "complete finite native acceleration diagnostics required")
    violated = [dict(native_frame=b["frame"],timestamp=b["timestamp"],actor_world_XY_speed_rate_mps2=rate)
        for b,rate in zip(intervals[1:],rates) if rate < -6. or rate > 3.]
    return dict(passed=not violated, full_native_count=len(rows), full_driving_native_count=len(driving),
        driving_speed_rate_interval_denominator=len(rates), warmup_to_first_driving_interval_included=first>0,
        maximum_guard_mps2=3., minimum_guard_mps2=-6.,
        actor_world_XY_acceleration_norm_maximum_mps2=max(scalar),
        actor_world_XY_speed_rate_minimum_mps2=min(rates), actor_world_XY_speed_rate_maximum_mps2=max(rates),
        actual_acceleration_above3_observation_count=sum(v>3 for v in scalar),
        native_speed_rate_outside_minus6_plus3_count=len(violated), native_speed_rate_violations=violated,
        native_speed_rate_abs_above2p9_observation_count=sum(abs(v)>2.9 for v in rates),
        additive_screen_not_new_feature_or_dataset_admission=True, no_original_safety_threshold_changed=True)


def diagnostic_report(output, plan, owner):
    result = _diagnostic(output, plan, owner)
    episode = output/"episode" if (output/"episode").is_dir() else output/"episode.partial"
    try:
        proof = configuration_report(episode); acceleration = acceleration_report(episode)
        baseline_proof(_baseline["final_endpoint_sha256"]); parent.verify_derived_source_closure(output, plan)
    except Exception as error:
        proof = dict(passed=False, status="FAIL_PREVIEW_CONFIGURATION", error=type(error).__name__+": "+str(error))
        acceleration = dict(passed=False,status="UNPROVEN", error=proof["error"])
    result.update(schema=SCHEMA+".diagnostic_report", actual_preview_configuration=proof,
        causal_selected_speed_preview_calibration=calibration_contract(), native_acceleration_diagnostic=acceleration,
        actual_same_V3_baseline_physical_setup=False)
    result["learned_driving_screen_passed"] = bool(result.get("learned_driving_screen_passed")) and proof["passed"] and acceleration["passed"]
    result["status"] = "PASS_BASIC_PREVIEW08_SIGNED_PRECURSOR" if result["learned_driving_screen_passed"] else "FAIL_BASIC_PREVIEW08_SIGNED_PRECURSOR"
    return result


def batch_summary(run_id, expected):
    result = _summary(run_id, expected)
    root, _ = batch_paths(run_id)
    proved = [bool(read_json(root/("trial_%02d"%i)/"diagnostic_report.json").get("actual_preview_configuration",{}).get("passed",False)) for i in range(3)]
    accelerations = [bool(read_json(root/("trial_%02d"%i)/"diagnostic_report.json").get("native_acceleration_diagnostic",{}).get("passed",False)) for i in range(3)]
    result.update(schema=SCHEMA+".batch_summary", causal_selected_speed_preview_calibration=calibration_contract(),
        all_three_actual_preview_readbacks_passed=all(proved), all_three_additive_native_acceleration_screens_passed=all(accelerations),
        actual_same_V3_baseline_physical_setup=False)
    for field in ("all_trials_motion_body_policy_AND_wall100ms_passed", "all_trials_basic_driving_passed"):
        if field in result: result[field] = bool(result[field]) and all(proved) and all(accelerations)
    return result


@contextmanager
def activated(pins):
    global _active, _baseline
    require(not _active and not previous._active, "preview owner cannot nest")
    proof = baseline_proof(pins)
    require(digest(Path(previous.__file__)) == FROZEN_OWNER_SHA, "frozen CPU4 owner changed")
    values = dict(batch_paths=batch_paths, MODULE=MODULE, SCHEMA=SCHEMA, live=live, fixed_contract=fixed_contract,
        runtime_sources=runtime_sources, load_model=load_model, diagnostic_report=diagnostic_report, batch_summary=batch_summary,
        disk_check=disk_check)
    saved = {name:getattr(previous,name) for name in values}; assets = base.ASSETS
    _active, _baseline = True, proof
    try:
        for name,value in values.items(): setattr(previous,name,value)
        base.ASSETS = dict(base.ASSETS) | {BASELINE:BASELINE_SHA}
        with previous.activated(pins): yield
    finally:
        for name,value in reversed(tuple(saved.items())): setattr(previous,name,value)
        base.ASSETS = assets; _active, _baseline = False, None


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    for name in ("weights", "metadata", "checkpoint", "declaration"):
        parser.add_argument("--expected-"+name+"-sha256", required=True)
    args,remainder = parser.parse_known_args(argv)
    pins = {name+"_sha256":getattr(args,"expected_"+name+"_sha256") for name in ("weights", "metadata", "checkpoint", "declaration")}
    with activated(pins): return parent.main(remainder)


if __name__ == "__main__":
    raise SystemExit(main())
