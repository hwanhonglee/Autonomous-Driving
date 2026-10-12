"""HH_261002 - Independently verify causal preview08, complete PI/ACK chronology and all unchanged signed45 gates."""

from __future__ import annotations

import argparse
import ast
from contextlib import contextmanager
import copy
import json
import math
from pathlib import Path

from . import audit_temporal_route_joint_thread4_signed45_batch_v2_20261002 as previous

thread = previous.previous
joint, signed = thread.previous, thread.previous.previous
ROOT, CARLA_ROOT, frozen, PROFILES = thread.ROOT, thread.CARLA_ROOT, thread.frozen, thread.PROFILES
require, digest, read_json, rows, add_pin, source_closure = thread.require, thread.digest, thread.read_json, thread.rows, thread.add_pin, thread.source_closure
strict_equal, endpoint_pins, measured_rear_errors = thread.strict_equal, thread.endpoint_pins, thread.measured_rear_errors
SCHEMA = "portable_e2e.actual_route_joint_preview08_signed45_archive_audit.20261002.v1"
RECEIPT_SCHEMA = SCHEMA + ".caller_pins"
OWNER_SCHEMA = "portable_e2e.actual_route_joint_thread4_preview08_signed45_three_arms.20261002.v2"
COLLECTOR_SCHEMA = "portable_e2e.actual_route_joint_thread4_preview08_signed45.20261002.v1"
PREVIEW_SCHEMA = "portable_e2e.causal_selected_speed_preview08_calibration.20261002.v1"
BASELINE_SHA = "314bd92fdb2b0069c068fc0f973fbc37f08aff207e44578590a9dc31200e5258"
BASELINE = ROOT / "storage/experiments/validation/2026-10-02/temporal_route_joint_thread4_signed45_batch_audit_v1/batch_outcome.json"
BASELINE_BATCH = ROOT / "storage/experiments/validation/2026-10-02/temporal_route_joint_thread4_signed45_town07_v1"
OWNER_SOURCE = ROOT / "tools/e2e/research_temporal_route_joint_preview08_signed45_town07_20261002.py"
COMPATIBILITY_SCHEMA = "portable_e2e.preview08_exact_baseline_reader_compatibility.20261002.v2"
FAILURE = ROOT / "storage/experiments/validation/2026-10-02/temporal_route_joint_preview08_v1_failed_declaration_context.json"
FAILURE_SHA = "30b1aac21720b6b104f61f33497cbde853df9e27d322f527901caf756b1a67a3"
NEW_FROZEN = dict(thread.NEW_FROZEN, **{
    "tools/e2e/audit_temporal_route_joint_thread4_signed45_batch_20261002.py": previous.V1_SHA,
    "tools/e2e/audit_temporal_route_joint_thread4_signed45_batch_v2_20261002.py": "1c4c5ca766d195b9163fc46e9257ea559ea00ad0abcd7932f7591b95f59aabc9",
    "tools/e2e/research_temporal_route_joint_preview08_signed45_town07_20261002.py": "e2627b08eaf61f70e854ee1c9163a46adb6ab4869e5ca7eddfb15a6f40025090",
    "tools/e2e/research_temporal_route_joint_preview08_signed45_v2_town07_20261002.py": "f46594c8c065a2da658fb7e743ecfdb97fae48ceaf201a666feef0f8a1d80894",
    "tools/e2e/collect_temporal_route_joint_preview08_signed45_20261002.py": "90a3788fa4be92136cb73b2f3413b4ddbbdd2c0a89f55f5998256e1abbba1498",
    "learning/portable_e2e/simulation_research.py": "4a66e2d3b522272822ddb43c43edeed1ebc6a2ee6c20bd0cda6ce2b3ea6c9252",
    "learning/portable_e2e/longitudinal_pi_research_20261001.py": "bc15f390398c6b1c2e678b5c90896d4ac4612ea349451b4cdb530d3f0946665b",
    "learning/portable_e2e/longitudinal_pi_headroom_research_20261001.py": "fd984bdd6473e4e350a62712451be02768e6a730ad8b6dd033c79ca3af9a1ec5",
})
FROZEN_HELPERS = dict(thread.FROZEN_HELPERS, **NEW_FROZEN)
_active, _baseline = False, None
_joint_contract = joint.joint_baseline_contract


def verify_frozen():
    thread.verify_frozen()
    for name, sha in FROZEN_HELPERS.items():
        require(digest(ROOT/name) == sha, "unknown preview08 audit dependency: " + name)


def number(value):
    require(type(value) in (int, float) and math.isfinite(value), "finite non-Boolean preview measurement required")
    return float(value)


def equivalent(first, second):
    # HH_261002 - This is arithmetic reconstruction tolerance, never a relaxed physical PASS threshold.
    if isinstance(first, dict) or isinstance(second, dict):
        return isinstance(first, dict) and isinstance(second, dict) and set(first) == set(second) and all(equivalent(first[k], second[k]) for k in first)
    if isinstance(first, (list, tuple)) or isinstance(second, (list, tuple)):
        return isinstance(first, (list, tuple)) and isinstance(second, (list, tuple)) and len(first) == len(second) and all(equivalent(a,b) for a,b in zip(first,second))
    if type(first) is bool or type(second) is bool: return type(first) is type(second) and first == second
    if type(first) in (int, float) and type(second) in (int, float):
        return math.isfinite(first) and math.isfinite(second) and abs(first-second) <= 1.e-12
    return first == second


def calibration_contract():
    # HH_261002 - Parse only the pinned owner's literal declaration; never import Torch, model or physical owner.
    tree = ast.parse(OWNER_SOURCE.read_bytes())
    functions = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "calibration_contract"]
    require(len(functions) == 1, "exact preview calibration declaration required")
    from types import SimpleNamespace
    namespace = dict(live=SimpleNamespace(PREVIEW_SCHEMA=PREVIEW_SCHEMA), BASELINE=BASELINE, BASELINE_SHA=BASELINE_SHA)
    exec(compile(ast.fix_missing_locations(ast.Module(body=functions, type_ignores=[])), str(Path(__file__))+":literal_preview_contract", "exec"), namespace)
    return namespace["calibration_contract"]()


def preview_signed_contract():
    """HH_261002 - Remove only one now-false sampling-unchanged assertion in addition to the two old learning assertions."""
    tree = ast.parse(Path(joint.__file__).read_bytes())
    function = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "joint_baseline_contract")
    original = copy.deepcopy(function)
    sets = [n for n in function.body if isinstance(n, ast.Assign) and any(isinstance(t,ast.Name) and t.id == "names" for t in n.targets)]
    require(len(sets) == 1 and isinstance(sets[0].value, ast.Set), "exact old-learning assertion set required")
    extra = "learned_selected_decoded_path_actual_v0_PI_and_handoff_math_unchanged"
    require(extra not in [n.value for n in sets[0].value.elts], "sampling intervention already changed frozen contract")
    sets[0].value.elts.append(ast.Constant(value=extra))
    parity = copy.deepcopy(function)
    next(n for n in parity.body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id == "names" for t in n.targets)).value.elts.pop()
    require(ast.dump(parity,include_attributes=False) == ast.dump(original,include_attributes=False), "every other original physical/recovery contract statement must remain identical")
    namespace = dict(vars(joint))
    exec(compile(ast.fix_missing_locations(ast.Module(body=[function],type_ignores=[])), str(Path(__file__))+":same_contract_explicit_sampling_revision", "exec"), namespace)
    return namespace["joint_baseline_contract"]()


def acceleration_census(states):
    require(len(states) == 900 and all(type(s.get("frame")) is int for s in states)
        and all(b["frame"] == a["frame"]+1 and abs(number(b["timestamp"])-number(a["timestamp"])-.05) <= 1.e-6 for a,b in zip(states,states[1:])), "complete900 native20Hz rate denominator required")
    first = next((i for i,s in enumerate(states) if s.get("capture_phase") == "driving"), None)
    driving = states[first:] if first is not None else []
    require(driving and all(s.get("capture_phase") == "driving" for s in driving), "no driving tail may be omitted from rate census")
    intervals = states[max(0,first-1):]
    rates = [(math.hypot(*(number(v) for v in b["world_velocity_carla"][:2]))-math.hypot(*(number(v) for v in a["world_velocity_carla"][:2])))/(number(b["timestamp"])-number(a["timestamp"])) for a,b in zip(intervals,intervals[1:])]
    norms = [math.hypot(*(number(v) for v in s["world_acceleration_carla"][:2])) for s in driving]
    require(len(rates) == len(driving)-(first == 0) and rates and all(math.isfinite(v) for v in rates+norms), "complete finite destination-driving intervals required")
    violations = [dict(native_frame=b["frame"],timestamp=b["timestamp"],actor_world_XY_speed_rate_mps2=rate) for b,rate in zip(intervals[1:],rates) if rate < -6. or rate > 3.]
    return dict(passed=not violations,full_native_count=900,full_driving_native_count=len(driving),driving_speed_rate_interval_denominator=len(rates),warmup_to_first_driving_interval_included=first>0,
        maximum_guard_mps2=3.,minimum_guard_mps2=-6.,actor_world_XY_acceleration_norm_maximum_mps2=max(norms),actor_world_XY_speed_rate_minimum_mps2=min(rates),actor_world_XY_speed_rate_maximum_mps2=max(rates),
        actual_acceleration_above3_observation_count=sum(v>3 for v in norms),native_speed_rate_outside_minus6_plus3_count=len(violations),native_speed_rate_violations=violations,
        native_speed_rate_abs_above2p9_observation_count=sum(abs(v)>2.9 for v in rates),additive_screen_not_new_feature_or_dataset_admission=True,no_original_safety_threshold_changed=True)


def baseline_proof(caller, endpoint, pins):
    identity = caller.get("baseline_report")
    require(identity == dict(path=str(BASELINE),sha256=BASELINE_SHA), "exact caller-pinned complete original0.3s baseline required")
    # HH_261002 - Only the exact caller-pinned baseline may exceed the unchanged generic4MiB JSON limit.
    add_pin(pins,BASELINE,BASELINE_SHA)
    require(BASELINE.stat().st_size <= 32*1024**2, "bounded exact baseline report required")
    old = json.loads(BASELINE.read_text())
    require(old["schema"] == thread.SCHEMA and old["batch"] == str(BASELINE_BATCH) and old["final_endpoint_sha256"] == endpoint
        and old["prospective_trials"] == 3 and old["all_failed_tail_retained"] is True and old["all_three_complete_thread_readbacks_passed"] is True
        and old["all_actual_native_states"] == 2700 and old["all_actual_camera_bundles"] == 1350 and old["all_actual_ACKs"] == 2712 and old["actual_JPEG_assets"] == 8100
        and [r["signed_recovery_profile"] for r in old["trials"]] == list(PROFILES) and strict_equal(old["cpu_thread_numerical_preset"],thread.pure_preset()[0]), "same final6000/allthree actualCPU4 baseline required")
    for name, sha in old["source_sha256"].items(): add_pin(pins,name,sha)
    rates = []
    for i in range(3):
        folder = BASELINE_BATCH/("trial_%02d"%i)
        episodes = [p for p in (folder/"episode",folder/"episode.partial") if p.is_dir()]
        require(len(episodes) == 1 and str(episodes[0]/"states.jsonl") in pins, "exact baseline native state source required")
        rates.append(dict(trial_index=i,**acceleration_census(rows(episodes[0]/"states.jsonl"))))
    return old, rates


def verify_contract(plan, trial):
    thread._verify(plan,trial); thread.benchmark_proof(plan)
    fixed = dict(learned_selected_decoded_path_actual_v0_PI_and_handoff_math_unchanged=False,
        original_decoder_gate_selected_tracker_math_unchanged_except_disclosed_PI_headroom=False,
        actual_same_2p40_PI_45seconds_and_all_original_safety_thresholds=True,
        tracker_sampling_changed_only_by_declared_preview_with_original_STOP_guard=True,actual_same_V3_baseline_physical_setup=False,
        prior_output_hashes_verified_before_and_after_not_duplicated_as_camera_assets=True)
    require(all(strict_equal(plan.get(k),v) for k,v in fixed.items()) and strict_equal(plan.get("causal_selected_speed_preview_calibration"),calibration_contract()), "explicit preview-only intervention contract differs")
    require(_baseline is not None and strict_equal(plan.get("baseline_complete_source_output_census"),source_census(_baseline[0]))
        and equivalent(plan.get("baseline_actual_native_acceleration_under_same_additive_guard"),_baseline[1]), "all prior source pins/rate outcomes must be retained")
    require("baseline_actual_output_source_sha256" not in plan and plan.get("compatibility_revision_schema") == COMPATIBILITY_SCHEMA
        and plan.get("frozen_v1_owner_source_sha256") == NEW_FROZEN["tools/e2e/research_temporal_route_joint_preview08_signed45_town07_20261002.py"]
        and plan.get("failed_v1_declaration_context_path") == str(FAILURE) and plan.get("failed_v1_declaration_context_sha256") == FAILURE_SHA
        and plan.get("exact_baseline_only_JSON_limit_bytes") == 32*1024**2 and plan.get("general_JSON_reader_limit_unchanged_bytes") == 4*1024**2
        and plan.get("collector_and_physical_preview_calibration_unchanged") is True and plan["asset_sha256"].get(str(FAILURE)) == FAILURE_SHA,
        "explicit bounded baseline compatibility/retained failed invocation required")
    size = BASELINE.stat().st_size
    require(plan.get("sealed_baseline_report_archive_bytes_per_trial") == size and plan.get("additional_three_trial_baseline_report_archive_reserve_bytes") == 3*size
        and plan["asset_sha256"].get(str(BASELINE)) == BASELINE_SHA, "all three declared baseline report copies/reserve required")
    require(all(plan["source_sha256"].get(name) == sha for name,sha in NEW_FROZEN.items() if "audit_" not in Path(name).name), "exact prospective preview/PI/tracker source closure required")


def source_census(report):
    import hashlib
    sources = report["source_sha256"]
    return dict(pin_count=len(sources),canonical_mapping_sha256=hashlib.sha256(json.dumps(sources,sort_keys=True,separators=(",",":"),allow_nan=False).encode()).hexdigest(),
        exact_report_path=str(BASELINE),exact_report_sha256=BASELINE_SHA,all_original_pins_live_verified_before_and_after=True,
        full_mapping_archived_in_exact_report_asset=True,original_all_three_failures_retained=True,no_camera_source_pin_dropped=True)


def preview_targets(selected, config):
    speeds, xy = selected["speed_mps"], selected["xy_base_m"]
    require(len(speeds) == len(xy) == 64 and type(selected["timestep_s"]) in (int,float) and selected["timestep_s"] == .1
        and all(number(v) >= 0 for v in speeds), "finite64 selected trajectory ABI required")
    old_index, new_index = max(0,round(.3/.1)-1), max(0,round(.8/.1)-1)
    old, new = min(config["speed_cap_mps"],speeds[old_index]), min(config["speed_cap_mps"],speeds[new_index])
    intent = selected.get("intent",0)
    guard = "non_GO_selected_intent" if not (type(intent) is int and intent == 0) else "either_selected_preview_target_at_or_below_original_STOP_threshold" if min(old,new) <= .05 else None
    return dict(schema=PREVIEW_SCHEMA,configured_speed_preview_s=.8,effective_speed_preview_s=.3 if guard else .8,
        original_speed_preview_s=.3,original_preview_index=old_index,proposed_preview_index=new_index,
        original_selected_target_mps=old,proposed_selected_target_mps=new,effective_selected_target_mps=old if guard else new,
        original_STOP_threshold_mps=.05,original_STOP_tracker_preserved=guard is not None,fallback_reason=guard,selected_intent_available="intent" in selected,
        explicit_cruise_GO_not_trained_STOP=True,selected_trajectory_changed=False,target_is_only_a_causal_selected_NN_speed_knot=True,
        actual_measured_speed_clipped=False,controller_calibration_not_model_improvement=True)


def tracker_math(selected, current, config, desired):
    # HH_261002 - Independently reconstruct the frozen pure-pursuit/normal tracker using the actual nine-field geometry.
    lookahead = max(config["minimum_lookahead_m"],max(0.,current)*config["lookahead_time_s"])
    points = [(x,y) for x,y in selected["xy_base_m"] if x > 0 and math.hypot(x,y) > 1.e-6]
    target = next((p for p in points if math.hypot(*p) >= lookahead),points[-1] if points else None)
    if target is None:
        require(max(selected["speed_mps"]) <= .1, "moving plan lacks forward pursuit point")
        steer = 0.
    else:
        curvature = 2*target[1]/(target[0]**2+target[1]**2)
        steer = max(-1.,min(1.,-math.atan(config["wheelbase_m"]*curvature)/config["maximum_steer_angle_rad"]))
    error = desired-max(0.,current)
    throttle, brake = min(config["maximum_throttle"],max(0.,config["throttle_gain"]*error)), min(1.,max(0.,-config["brake_gain"]*error))
    if desired <= .05: throttle, brake = 0.,max(.3,brake)
    return dict(throttle=throttle,brake=brake,steer=steer,reverse=False,hand_brake=False,desired_speed_mps=desired,pursuit_xy_base_m=target,steering_convention="CARLA_right_positive_from_base_link_y_left")


def pi_math(original, current, integral):
    # HH_261002 - Execute the declared original PI arithmetic once per fresh attempt; held ACKs never integrate.
    desired = min(original["desired_speed_mps"],2.4)
    if desired <= .05: return dict(original),0.,dict(stop_branch_preserved=True,integral_before=integral,integral_after=0.,integral_reset_reason="model_stop_target",feedforward=0.,throttle_floor_added=False)
    error = desired-max(0.,current); unbounded = integral+error*.1; candidate = max(-1.,min(1.,unbounded))
    unsaturated = .7*error+.2*candidate
    held = (unsaturated > .5 and error > 0.) or (unsaturated < -1. and error < 0.)
    if held: candidate = integral
    request = .7*error+.2*candidate
    control = dict(original,throttle=min(.5,max(0.,request)),brake=min(1.,max(0.,-request)))
    audit = dict(desired_speed_mps=desired,observed_speed_mps=current,error_mps=error,timestep_s=.1,integral_before=integral,integral_unbounded=unbounded,
        integral_after=candidate,integral_bound_applied=abs(unbounded)>1.,integral_held_for_saturation=held,signed_request=request,signed_unsaturated_candidate=unsaturated,
        stop_branch_preserved=False,integral_reset_reason=None,feedforward=0.,throttle_floor_added=False,throttle_saturated=request>.5,brake_saturated=request < -1.)
    return control,candidate,audit


def preview_census(records, timing, commands, receipts, states):
    normal = [r for r in timing if r.get("command_reason") in ("initial_drive_control","next_driving_control") and r.get("command_frame") is not None]
    keys = [(r["command_frame"],r["command_reason"]) for r in normal]
    require(keys and keys == [(r["command_frame"],r["command_reason"]) for r in records] and len(keys) == len(set(keys)), "all normal preview witnesses required")
    by_ACK = {r["ack_receipt_sequence"]:r for r in commands}; ACKs = {r["sequence"]:r for r in receipts}; observed = {s["frame"]:s for s in states}
    require(len(by_ACK) == len(commands) == len(normal) and len(ACKs) == len(receipts), "complete unique command/ACK denominator required")
    config = records[0]["actual_policy_controller"]
    fixed = dict(speed_cap_mps=2.5,minimum_lookahead_m=1.5,lookahead_time_s=.8,speed_preview_s=.8,throttle_gain=.7,brake_gain=.7,maximum_throttle=.5)
    require(set(config) == set(fixed)|{"wheelbase_m","maximum_steer_angle_rad"} and all(strict_equal(config.get(k),v) for k,v in fixed.items())
        and number(config["wheelbase_m"]) > 0 and 0 < number(config["maximum_steer_angle_rad"]) < math.pi/2, "actual original geometry/nine-field preview-only controller required")
    integral, fresh_count, guarded, pi_count, held_count, over_envelope_inputs = 0.,0,0,0,0,0
    last_fresh = None
    for row, wall in zip(records,normal):
        raw = by_ACK[wall["ACK_sequence"]]; ack = ACKs[wall["ACK_sequence"]]
        require(row["schema"] == PREVIEW_SCHEMA and row["actual_policy_controller"] == config and raw["command_reason"] == wall["command_reason"]
            and wall["proposal_started_monotonic_ns"] <= row["proposal_started_monotonic_ns"] <= row["proposal_finished_monotonic_ns"] <= wall["proposal_finished_monotonic_ns"] <= wall["ACK_started_monotonic_ns"] <= wall["ACK_finished_monotonic_ns"], "preview timing/configuration/ACK chain differs")
        require(row["control_source"] == raw.get("control_source") and row["learned_control_proposed"] is bool(raw.get("learned_control_proposed"))
            and row["held_between_camera_frames"] is bool(raw.get("held_between_camera_frames")) and row["inference_frame"] == raw.get("inference_frame")
            and equivalent(row["actual_returned_control"],raw.get("control")), "readback must bind the actual untouched raw decision")
        if raw.get("control") is not None:
            for name in ("throttle","brake","steer","reverse","hand_brake"):
                a,b = raw["control"][name],ack["requested_control"][name]
                require(type(a) is type(b) and a == b if type(a) is bool or type(b) is bool else abs(number(a)-number(b)) <= 1.e-6, "actual five actuator fields differ from ACK")
        receipt = row["tracker_receipt"]
        if receipt is None:
            require(not (raw.get("held_between_camera_frames") is False and raw.get("learned_control_proposed") is True), "accepted fresh decision missing tracker witness")
            if raw.get("held_between_camera_frames") is True and raw.get("learned_control_proposed") is True:
                require(last_fresh is not None and equivalent(raw.get("control"),last_fresh["control"]) and equivalent(raw.get("longitudinal_pi_research"),last_fresh.get("longitudinal_pi_research")), "held command recomputed PI or rewrote prior selected control")
                held_count += 1
            if raw.get("control_source") == "emergency_brake": integral = 0.
            continue
        require(raw.get("held_between_camera_frames") is False and raw.get("selected") is not None, "only fresh attempts may invoke tracker/PI")
        selected = raw["selected"]; prediction = raw["decoded_output"]
        require(selected["candidate_index"] == 0 and len(prediction["candidate_xy"]) == len(prediction["candidate_speed"]) == 1
            and equivalent(selected["xy_base_m"],prediction["candidate_xy"][0]) and equivalent(selected["speed_mps"],prediction["candidate_speed"][0]),
            "selected trajectory must equal the unchanged decoded sole NN candidate with its original candidate axis")
        witness = preview_targets(selected,config)
        require(equivalent({k:v for k,v in receipt.items() if k != "original_tracker_control"},witness), "independent exact selected speed knot/STOP guard differs")
        current = number(raw["measured_current_speed_mps"])
        # HH_261002 - Runtime norm is float32; native JSON is double. This arithmetic tolerance never changes either physical speed gate.
        require(current >= 0 and raw["inference_frame"] in observed and abs(current-math.hypot(number(observed[raw["inference_frame"]]["vx"]),number(observed[raw["inference_frame"]]["vy"]))) <= 1.e-6, "original actual current base speed cannot be clipped or relabelled")
        # HH_261002 - Preserve any real over-envelope input as failed evidence; the inherited full-native2.5 gate decides physical PASS.
        over_envelope_inputs += int(current > 2.5)
        original = tracker_math(selected,current,config,witness["effective_selected_target_mps"])
        require(equivalent(receipt["original_tracker_control"],original), "original full tracker output differs from independent geometry/speed arithmetic")
        control, after, pi = pi_math(original,current,integral)
        actual_pi = raw.get("longitudinal_pi_research",{})
        require(equivalent(actual_pi.get("original_normal_tracker_control"),original) and equivalent(actual_pi.get("proposed_normal_tracker_control"),control)
            and all(equivalent(actual_pi.get(k),v) for k,v in pi.items()) and actual_pi.get("controller_target_cap_mps") == 2.4
            and actual_pi.get("original_model_target_mps") == original["desired_speed_mps"] and actual_pi.get("governed_controller_target_mps") == min(original["desired_speed_mps"],2.4)
            and actual_pi.get("one_fresh_camera_update_one_PI_integration") is True and actual_pi.get("observed_speed_clipped") is False, "one original headroom PI update/complete causal integral chronology differs")
        if raw.get("learned_control_proposed") is True:
            require(equivalent(raw["control"],control) and raw["controller"] == config, "applied raw control or controller differs from independent tracker+PI")
            integral = after; last_fresh = raw
        else: integral = 0.
        fresh_count += 1; pi_count += 1; guarded += int(witness["original_STOP_tracker_preserved"])
    require(fresh_count > 0, "no actual fresh preview attempt is NOT_PROVEN, not vacuous PASS")
    return dict(passed=True,normal_proposal_denominator=len(normal),controller_readback_count=len(records),fresh_tracker_count=fresh_count,
        original_STOP_or_non_GO_fallback_count=guarded,actual_controller=config,actual_geometry_not_nominal_manifest_substitution=True,
        independent_PI_update_count=pi_count,held_ACK_without_new_PI_count=held_count,complete_PI_integral_chronology_reconstructed=True,
        all_selected_NN_knots_original_tracker_and_five_ACK_fields_reconstructed=True,actual_speed_not_clipped=True,
        native_double_to_model_float32_norm_reconstruction_atol_mps=1.e-6,physical_speed_gate_tolerance_changed=False,
        fresh_tracker_inputs_above_physical2p5_envelope_retained=over_envelope_inputs)


def extra_trial_evidence(manifest, states, commands, receipts, route, plan, diagnostic, pins, episode):
    result = thread._extra(manifest,states,commands,receipts,route,plan,diagnostic,pins,episode)
    result["independent_cpu_thread_preset"] = thread.independent_thread_readback(episode,manifest,diagnostic,pins)
    path = episode/"speed_preview_configuration.jsonl"; add_pin(pins,path)
    proof = preview_census(rows(path),rows(episode/"wall_timing.jsonl"),commands,receipts,states)
    reported = diagnostic.get("actual_preview_configuration",{})
    require(reported.get("passed") is proof["passed"] and all(equivalent(reported.get(k),proof[k]) for k in ("normal_proposal_denominator","controller_readback_count","fresh_tracker_count","original_STOP_or_non_GO_fallback_count","actual_controller")), "independent complete preview census differs from owner")
    acceleration = acceleration_census(states)
    require(equivalent(diagnostic.get("native_acceleration_diagnostic"),acceleration), "independent full900/prefix native-rate census differs from owner")
    contract = manifest["capture_contract"].get("causal_selected_speed_preview_calibration")
    require(strict_equal(contract,dict(schema=PREVIEW_SCHEMA,original_speed_preview_s=.3,configured_speed_preview_s=.8,either_preview_STOP_or_non_GO_preserves_entire_original_tracker=True,
        readback_journal="speed_preview_configuration.jsonl",unchanged_PI_target_cap_mps=2.4,unchanged_policy_deadline_seconds=.1,unchanged_observed_bundle_to_ACK_deadline_seconds=.1,
        NN_raw_decoded_selected_and_measured_speed_unchanged=True,model_promoted=False,vehicle_control_approved=False)), "actual preview manifest differs")
    result.update(independent_preview_configuration=proof,independent_additive_native_acceleration=acceleration)
    return result


def transformed_trial_reader():
    """HH_261002 - Keep every original raw/source/body/wall statement; add only preview and acceleration conjuncts and exact metadata selector."""
    legacy = signed.previous
    tree = ast.parse(Path(legacy.__file__).read_bytes())
    function = next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name == "load_trial")
    original = copy.deepcopy(function)
    anchors = [i for i,n in enumerate(function.body) if isinstance(n,ast.Expr) and isinstance(n.value,ast.Call) and any(isinstance(a,ast.Constant) and a.value == "independent driving/wall result differs from caller-pinned report" for a in n.value.args)]
    require(len(anchors) == 1, "exact frozen raw driving comparison anchor required")
    injected = ast.parse("""extra = extra_trial_evidence(manifest,states,commands,receipts,read_json(episode/'route.json'),plan,diagnostic,pins,episode)
basic_driving_before_new_recovery = driving
driving = driving and extra['independent_actual_spawn']['actual_signed_spawn_identity_passed'] and extra['independent_manual1_initialization']['manual_first_gear_initialization_passed'] and extra['independent_signed_recovery']['case_screen_passed'] and extra['independent_cpu_thread_preset']['passed'] and extra['independent_preview_configuration']['passed'] and extra['independent_additive_native_acceleration']['passed']
""").body
    function.body[anchors[0]:anchors[0]] = injected
    selectors = [n for n in ast.walk(function) if isinstance(n,ast.If) and ast.dump(n.test,include_attributes=False) == ast.dump(ast.parse('section=="asset_sha256" and source.name=="metadata.json" and value==endpoint["metadata"]',mode="eval").body,include_attributes=False)]
    require(len(selectors) == 1, "unique original metadata selector required")
    old_selector = copy.deepcopy(selectors[0].test)
    selectors[0].test = ast.BoolOp(op=ast.And(),values=[selectors[0].test,ast.parse("source == EXACT_METADATA",mode="eval").body])
    returned = function.body[-1]; require(isinstance(returned,ast.Return), "original return required")
    revision = next(k for k in returned.value.keywords if k.arg == "training_data_revision"); old_revision = copy.deepcopy(revision.value)
    revision.value = ast.parse("metadata['current_training_purpose']",mode="eval").body
    added = [ast.keyword(arg="signed_recovery_profile",value=ast.parse("plan['signed_recovery_profile']",mode="eval").body),ast.keyword(arg="basic_driving_before_new_recovery_passed",value=ast.Name(id="basic_driving_before_new_recovery",ctx=ast.Load())),ast.keyword(arg=None,value=ast.Name(id="extra",ctx=ast.Load()))]
    returned.value.keywords.extend(added)
    parity = copy.deepcopy(function); del parity.body[anchors[0]:anchors[0]+len(injected)]; del parity.body[-1].value.keywords[-len(added):]
    next(k for k in parity.body[-1].value.keywords if k.arg == "training_data_revision").value = old_revision
    next(n for n in ast.walk(parity) if isinstance(n,ast.If) and isinstance(n.test,ast.BoolOp) and any(isinstance(v,ast.Compare) and ast.dump(v,include_attributes=False) == ast.dump(ast.parse("source == EXACT_METADATA",mode="eval").body,include_attributes=False) for v in n.test.values)).test = old_selector
    require(ast.dump(parity,include_attributes=False) == ast.dump(original,include_attributes=False), "every original hash/physical/control gate statement must remain exact")
    namespace = dict(vars(legacy),verify_contract=verify_contract,extra_trial_evidence=extra_trial_evidence,EXACT_METADATA=previous.EXACT_METADATA,audit_post_ACK_recording=signed.writer.audit_post_ACK_recording)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[function],type_ignores=[])),str(Path(__file__))+":unchanged_full_reader_plus_preview_witness","exec"),namespace)
    return namespace["load_trial"]


@contextmanager
def activated(endpoint, baseline):
    global _active, _baseline
    require(not _active and not previous._active and not thread._active and joint.joint_baseline_contract is _joint_contract, "preview audit cannot nest or replace unknown contract")
    verify_frozen()
    values = dict(SCHEMA=SCHEMA,RECEIPT_SCHEMA=RECEIPT_SCHEMA,OWNER_SCHEMA=OWNER_SCHEMA,COLLECTOR_SCHEMA=COLLECTOR_SCHEMA,
        FROZEN_HELPERS=FROZEN_HELPERS,NEW_FROZEN=NEW_FROZEN,verify_contract=verify_contract,transformed_trial_reader=transformed_trial_reader)
    saved = {k:getattr(thread,k) for k in values}; _active,_baseline = True,baseline
    try:
        for k,v in values.items(): setattr(thread,k,v)
        joint.joint_baseline_contract = preview_signed_contract
        with thread.activated(endpoint): yield
    finally:
        joint.joint_baseline_contract = _joint_contract
        for k,v in saved.items(): setattr(thread,k,v)
        _active,_baseline = False,None


def build_report(batch,batch_sha,caller_path,caller_sha):
    pins = {}; add_pin(pins,caller_path,caller_sha); caller = read_json(caller_path)
    endpoint = endpoint_pins(caller.get("final_endpoint")); baseline = baseline_proof(caller,endpoint,pins)
    with activated(endpoint,baseline): report = thread._build(batch,batch_sha,caller_path,caller_sha)
    report["source_sha256"].update(pins)
    for path in source_closure(Path(__file__),ROOT/"tools/workspace_python.py",ROOT/"tools/canonical_workspace.py"): add_pin(report["source_sha256"],path)
    add_pin(report["source_sha256"],joint.RUN/"declaration.json",endpoint["declaration_sha256"])
    report.pop("original_frozen97_check_enforced_by_pinned_numeric_loader",None)
    fixed = dict(optimizer_updates=6000,known_total_optimizer_updates=12200,trainable_state_tensor_count=54,frozen_state_tensor_count=47,original_backbone_weights_byte_unchanged=False,
        semantic_outputs_preserved=False,visual_encoder_frozen=True,semantic_head_parameters_frozen=True,original47_bytefixed_check_enforced_by_pinned_numeric_loader=True,
        all101_tensor_values_independently_reloaded_by_this_audit=False,no_torch_pickle_loaded=True,numeric_NPZ_values_not_reloaded_by_this_offline_auditor=True,
        model_weights_unchanged=True,teacher_startup_and_PI2p4_parameters_unchanged=True,CPU_thread_numerical_preset_changed=False,learned_tracker_speed_preview_changed=True,
        model_change_is_learned_weights_only_not_teacher_controller_or_threshold_change=False,final_endpoint_sha256=endpoint,cpu_thread_numerical_preset=thread.pure_preset()[0],
        causal_selected_speed_preview_calibration=calibration_contract(),all_three_complete_thread_readbacks_passed=all(r["independent_cpu_thread_preset"]["passed"] for r in report["trials"]),
        all_three_actual_preview_readbacks_passed=all(r["independent_preview_configuration"]["passed"] for r in report["trials"]),all_three_additive_native_acceleration_screens_passed=all(r["independent_additive_native_acceleration"]["passed"] for r in report["trials"]))
    report.update(fixed)
    metrics = ("joint_motion_body_policy_wall100ms_passed","native_actor_progress_m","post_handoff_base_norm_maximum_mps","post_handoff_actor_CTE_p95_m","post_handoff_actor_CTE_maximum_m","delivery_to_ACK_ms","fresh_policy100ms_passed","original_body_screen_passed")
    report["parent_comparison"] = dict(baseline_report_path=str(BASELINE),baseline_report_sha256=BASELINE_SHA,baseline_original_joint_pass_count=baseline[0]["joint_motion_body_policy_wall_trial_pass_count"],
        baseline_under_additive_rate_guard_joint_pass_count=sum(a["joint_motion_body_policy_wall100ms_passed"] and g["passed"] for a,g in zip(baseline[0]["trials"],baseline[1])),
        candidate_joint_pass_count=report["joint_motion_body_policy_wall_trial_pass_count"],same_fixed_final6000_weights_profiles_seeds_teacher_PI_threads_and_old_thresholds=True,
        difference_is_causal_speed_preview_not_new_weights_or_changed_threads=True,additional_rate_guard_applied_to_both_all_three_arms=True,original_baseline_results_rewritten=False,
        all_three_parent_and_candidate_attempts_retained=True,no_best_attempt_selection=True,per_arm=[dict(trial_index=i,case_id=PROFILES[i]["case_id"],baseline={k:a[k] for k in metrics},preview08={k:b[k] for k in metrics},baseline_additive_native_rate=g,candidate_additive_native_rate=b["independent_additive_native_acceleration"]) for i,(a,b,g) in enumerate(zip(baseline[0]["trials"],report["trials"],baseline[1]))])
    frozen.assert_pins(report["source_sha256"]); return report


def markdown(report):
    def n(v): return "미측정" if v is None else "%.3f"%v
    lines = ["# 같은 주행 모델: 속도 예측을 0.3초 → 0.8초 앞에서 따라본 결과","","<!-- HH_261002 - Report all actual arms, complete native-rate failures and the unchanged learned model. -->","",
        "전체 기준: 이전 %d/3 → 새 비교 %d/3. 좌·우 복귀는 %d/2이며 중앙 조건은 별도입니다."%(report["parent_comparison"]["baseline_original_joint_pass_count"],report["joint_motion_body_policy_wall_trial_pass_count"],report["signed_recovery_pass_count"]),"",
        "| 조건 | 이전 → 새 전체 기준 | 전진 거리 | 속도 최대 | 수신→ACK 최대 | 새 가감속 검사 |","|---|---|---:|---:|---:|---|"]
    for row in report["parent_comparison"]["per_arm"]:
        a,b = row["baseline"],row["preview08"]
        lines.append("| %s | %s → %s | %s → %s m | %s → %s m/s | %s → %s ms | %s (%d위반) |"%(row["case_id"],"PASS" if a["joint_motion_body_policy_wall100ms_passed"] else "FAIL","PASS" if b["joint_motion_body_policy_wall100ms_passed"] else "FAIL",n(a["native_actor_progress_m"]),n(b["native_actor_progress_m"]),n(a["post_handoff_base_norm_maximum_mps"]),n(b["post_handoff_base_norm_maximum_mps"]),n(a["delivery_to_ACK_ms"].get("maximum")),n(b["delivery_to_ACK_ms"].get("maximum")),"PASS" if row["candidate_additive_native_rate"]["passed"] else "FAIL",row["candidate_additive_native_rate"]["native_speed_rate_outside_minus6_plus3_count"]))
    lines += ["","바뀐 것은 제어기가 이미 예측된 속도의 어느 시점을 따르는지(0.3→0.8초)뿐입니다. 가중치·모델 궤적·실제 v0·CPU4/1·PI2.4·출발/seed·45초·90m·속도2.5·차체/10cm·100ms는 그대로입니다. 두 시점 중 어느 속도든 0.05m/s 이하이거나 명시적 non-GO이면 원래 0.3초 tracker를 통째로 사용합니다. STOP 학습 완료나 새 모델 개선을 뜻하지 않습니다.","",
        "추가로 원자료 20Hz의 actor XY 속도 변화율 +3/−6m/s²를 전체 driving 구간(직전 warmup 경계 포함)에 확인합니다. 이전 세 실행에도 같은 추가 검사를 적용하되 기존 결과는 수정하지 않습니다. 가속도 벡터 크기와 절대 변화율 2.9는 서로 다른 별도 관측이며 데이터 학습 승인으로 바꾸지 않습니다.","",
        "전체 3조건·2,700상태·1,350 실제 6카메라 묶음·8,100JPEG·2,712ACK와 실패 꼬리를 남깁니다. 매 normal 설정, 모든 fresh 선택 속도·원 tracker·PI 적분·실제 ACK를 독립 재계산했고 held 명령은 PI를 다시 적분하지 않았습니다. 센서 노출부터 완전한10Hz는 미측정입니다.","",
        "정식 메인 0/9 · 하위 0/30. 제한된 Town07 저속 직진 비교이며 회전·종점 정지·30km/h·다른 맵·실차 승인이 아닙니다.","","[전체 결과 JSON](batch_outcome.json) · [원자료/소스SHA](source_evidence.json)",""]
    return "\n".join(lines)


def publish(output,*args):
    output = Path(output); require(not output.exists() and not output.is_symlink() and output.absolute() == output.resolve(), "fresh canonical create-only preview audit output required")
    require(not any(p.is_symlink() for p in output.parents) and not output.is_relative_to(Path(args[0]).resolve()), "audit output cannot modify immutable source batch or follow symlink ancestors")
    report = build_report(*args); frozen.assert_pins(report["source_sha256"]); output.mkdir(parents=True,exist_ok=False)
    for name,value in (("batch_outcome.json",report),("source_evidence.json",report["source_sha256"])):
        with (output/name).open("x") as stream: json.dump(value,stream,ensure_ascii=False,indent=2,allow_nan=False)
    with (output/"README.md").open("x") as stream: stream.write(markdown(report))
    frozen.assert_pins(report["source_sha256"]); return dict(output=str(output),result_sha256=digest(output/"batch_outcome.json"),all_trials_joint_passed=report["all_trials_motion_body_policy_AND_wall100ms_passed"],simulator_or_GPU_used=False)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    for name in ("batch","caller-receipts","output"): parser.add_argument("--"+name,type=Path,required=True)
    for name in ("batch-sha256","caller-receipts-sha256"): parser.add_argument("--"+name,required=True)
    parser.add_argument("--validate-only",action="store_true"); args = parser.parse_args(argv)
    inputs = (args.batch,args.batch_sha256,args.caller_receipts,args.caller_receipts_sha256)
    result = build_report(*inputs) if args.validate_only else publish(args.output,*inputs)
    print(json.dumps(result,ensure_ascii=False,allow_nan=False)); return 0


if __name__ == "__main__": raise SystemExit(main())
