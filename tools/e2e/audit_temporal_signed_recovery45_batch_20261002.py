"""HH_261002 - Independently authenticate two actual signed recoveries and one center comparator, retaining all attempts."""

from __future__ import annotations

import argparse
import ast
from contextlib import contextmanager
import copy
import json
import math
from pathlib import Path

from . import audit_temporal_route_recovery_fast45_batch_20261001 as previous
from . import audit_temporal_route_recovery_fast45_batch_v2_20261001 as writer

ROOT, CARLA_ROOT, frozen, transport = previous.ROOT, previous.CARLA_ROOT, previous.frozen, previous.transport
require, digest, read_json, rows, add_pin, source_closure = (previous.require, previous.digest, previous.read_json,
    previous.rows, previous.add_pin, previous.source_closure)
SCHEMA = "portable_e2e.actual_signed_recovery45_all_attempt_archive_audit.20261002.v1"
RECEIPT_SCHEMA = SCHEMA + ".caller_pins"
OWNER_SCHEMA = "portable_e2e.actual_signed_recovery45_three_arms.20261002.v3"
COLLECTOR_SCHEMA = "portable_e2e.actual_signed_recovery45.20261002.v3"
CHECKER_SHA = "815ef4c5806d1914eadfdd12eba1a42c392ceee58521b6f786d3c83349372695"
PROFILES = (
    dict(case_id="negative_outward", seed=2026102100, signed_lateral_offset_ros_m=-.20, signed_yaw_offset_ros_deg=-1.5),
    dict(case_id="positive_outward", seed=2026102101, signed_lateral_offset_ros_m=.20, signed_yaw_offset_ros_deg=1.5),
    dict(case_id="centered_setup_comparator", seed=2026102102, signed_lateral_offset_ros_m=0., signed_yaw_offset_ros_deg=0.),
)
NEW_FROZEN = {
    "tools/e2e/audit_temporal_route_recovery_fast45_batch_20261001.py": writer.V1_SHA,
    "tools/e2e/audit_temporal_route_recovery_fast45_batch_v2_20261001.py": "e3f7af13e648a1d7bb42f2d7086c5489032376a7cee7a4001b3a959a5015acf2",
    "tools/e2e/research_temporal_signed_recovery45_town07_20261002.py": "429af0b7d49889419caba872d25e15655bad3fabab8d3ba1ce871ed14e8510e3",
    "tools/e2e/collect_temporal_signed_recovery45_20261002.py": "eee6de2fa1402910eb725bd60724ac1d0753730f12ef47a1492f1e1d964a2fae",
    "tools/e2e/research_temporal_signed_recovery45_v2_town07_20261002.py": "ac029ad1b840a96c7105ce1cf9aad2531bbc177112ff8a0c689fa24a1804b7bd",
    "tools/e2e/collect_temporal_signed_recovery45_v2_20261002.py": "d3086074d4b5b66554582b442df89d0a91a6fbb9bf7bcc1e8191863c6797d19e",
    "tools/e2e/research_temporal_signed_recovery45_v3_town07_20261002.py": "b2f126b8f8d4c515805dcca10288a079cf509987f051a4f88446fc9027b948db",
    "tools/e2e/collect_temporal_signed_recovery45_v3_20261002.py": "d693e11422f7b60f5b16c687bab9df093b72712f1918ab736d8114727df1af33",
    "tools/e2e/run_owned_town07_signed_recovery_v3_20261001.py": "01f6157bd61e32fe525166b4698fa70f8b82156843ecc1bb93edb538fd706a9a",
    "tools/e2e/town07_signed_recovery_task_20261001.py": "fddd47336e5a914f89db60dc7dccfa1dd613c3917125b213754de86a83958ecb",
}
FROZEN_HELPERS = dict(previous.FROZEN_HELPERS, **NEW_FROZEN)
_verify_contract, _load_trial, _build_report = previous.verify_contract, previous.load_trial, previous.build_report
_active = False


def number(value):
    require(type(value) in (int, float) and math.isfinite(value), "finite non-Boolean measurement required")
    return float(value)


def strict_equal(first, second):
    if isinstance(first, dict) or isinstance(second, dict):
        return isinstance(first, dict) and isinstance(second, dict) and set(first) == set(second) \
            and all(strict_equal(first[k], second[k]) for k in first)
    if isinstance(first, list) or isinstance(second, list):
        return isinstance(first, list) and isinstance(second, list) and len(first) == len(second) \
            and all(strict_equal(a,b) for a,b in zip(first,second))
    if type(first) is bool or type(second) is bool: return type(first) is type(second) and first == second
    return first == second


def verify_frozen():
    writer.verify_frozen_sources()
    for name, value in FROZEN_HELPERS.items():
        require(digest(ROOT/name) == value, "unknown frozen signed-recovery audit dependency: " + name)
    module = ast.parse((ROOT/"tools/e2e/collect_temporal_signed_recovery45_v2_20261002.py").read_bytes())
    declarations = [n for n in module.body if isinstance(n,ast.Assign)
        and any(isinstance(t,ast.Name) and t.id == "PROFILES" for t in n.targets)]
    require(len(declarations) == 1, "exact frozen three-profile declaration required")
    # HH_261002 - Parse the finite constructor literals without importing the physical/model collector.
    actual = []
    for node in declarations[0].value.elts:
        require(isinstance(node,ast.Call) and isinstance(node.func,ast.Name) and node.func.id == "dict"
            and not node.args and all(k.arg for k in node.keywords), "unexpected physical profile AST")
        actual.append({k.arg:ast.literal_eval(k.value) for k in node.keywords})
    require(tuple(actual) == PROFILES, "all three physical profiles and seeds must be exact, not repeated/selected")
    require(all(type(r["seed"]) is int and 0 <= r["seed"] < 2**32 for r in PROFILES), "actual NumPy uint32 seed bound required")


def verify_contract(plan, trial):
    _verify_contract(plan,trial)
    require(type(trial) is int and 0 <= trial < 3 and strict_equal(plan.get("signed_recovery_profile"),PROFILES[trial])
        and type(plan.get("seed")) is int and plan.get("seed") == PROFILES[trial]["seed"]
        and plan.get("agent_initialization") == "after_bootstrap",
        "exact signed arm and actual after-bootstrap initialization required")
    argv=plan["capture_argv"]
    require(isinstance(argv,list) and argv.count("--seed") == argv.count("--agent-initialization") == 1
        and argv[argv.index("--seed")+1] == str(PROFILES[trial]["seed"])
        and argv[argv.index("--agent-initialization")+1] == "after_bootstrap", "declared actual CLI seed/initialization differs")
    fixed=dict(current_fixed_final3000_V3_weights_unchanged=True,no_new_training_or_checkpoint_selection=True,
        actual_spawn_before_actor_and_sensors=True,captured_pixel_pose_transform=False,
        learned_selected_decoded_path_actual_v0_PI_and_handoff_math_unchanged=True,
        normal_teacher_hooks_are_before_unchanged_BasicAgent_hazard_override=True,
        no_teacher_recovery_counted_as_learned=True,initialization_controls_or_snapshots_proxied=False,
        full_MAIN9_completion_claimed=False,full_route_goal_stop_approved=False,
        startup_recovery_was_not_training_scope=True,known_same_TRAIN_corridor_and_signed_geometry=True,
        unseen_seed_is_not_independent_test=True,unsigned32_seed_prevalidated=True,
        failed_v1_seeds_and_output_retained=True,original_exception_not_masked_by_no_spawn_finalizer=True,
        validator_profile_namespace_explicit_before_collector_activation=True,
        failed_v2_declaration_and_preflight_retained=True,v2_preflight_failure_was_before_actual_simulator_execution=True)
    require(all(strict_equal(plan.get(k),v) for k,v in fixed.items()), "signed recovery scope/output/teacher denials differ")
    require(plan.get("actual_seed_interval_inclusive") == [0,2**32-1], "prospective actual seed interval differs")
    expected=dict(first_effect_reference="First native rear-base observation matched to a prior actual learned ACK",
        required_noncentered_case=trial != 2, minimum_initial_abs_rear_CTE_m=.10,
        minimum_initial_abs_yaw_error_deg=.5, convergence_abs_rear_CTE_m=.10,convergence_abs_yaw_error_deg=.5,
        convergence_deadline_seconds=10.,minimum_continuous_converged_seconds=2.,
        convergence_deadline_interpretation="Enter both bands within10s of first learned effect, then remain continuously inside for2s; confirmation may occur by12s.",
        required_initial_error_signs_match_declared_spawn=True,all_after_first_learned_effect_rows_retained=True,
        independent_map_or_initialization_geometry_claim=False)
    require(strict_equal(plan.get("recovery_screen"),expected), "exact10s entry/continuous2s confirmation recovery contract required")
    require(strict_equal(plan.get("initialization_gear_contract"),dict(actual_manual_gear_shift=True,actual_gear=1,
        parent_function_source_sha256=CHECKER_SHA,checker_revision="carla.actual_signed_teacher_recovery.20261001.v3",
        original_frame_actor_finite_PID_full_brake_guards_preserved=True)), "actual strict manual1 checker contract differs")
    require(all(plan["source_sha256"].get(name) == value for name,value in NEW_FROZEN.items()
        if "audit_" not in Path(name).name), "new actual source identities absent from declared/archive closure")


def independently_perturb(center, spec, wheelbase):
    require(wheelbase == 2.85 and set(center) == {"x","y","z","roll","pitch","yaw"}, "exact rear-reference spawn geometry required")
    for value in center.values(): number(value)
    theta,pitch,half=math.radians(center["yaw"]),math.radians(center["pitch"]),wheelbase/2
    rear=dict(center)
    rear["x"] -= half*math.cos(theta)*math.cos(pitch)
    rear["y"] -= half*math.sin(theta)*math.cos(pitch)
    rear["z"] -= half*math.sin(pitch)
    rear["x"] += spec["signed_lateral_offset_ros_m"]*math.sin(theta)
    rear["y"] -= spec["signed_lateral_offset_ros_m"]*math.cos(theta)
    rear["yaw"] -= spec["signed_yaw_offset_ros_deg"]
    changed=dict(rear); angle=math.radians(rear["yaw"])
    changed["x"] += half*math.cos(angle)*math.cos(pitch)
    changed["y"] += half*math.sin(angle)*math.cos(pitch)
    changed["z"] += half*math.sin(pitch)
    return changed,rear


def independent_spawn(manifest, plan):
    contract=manifest["capture_contract"]; spawn=contract["actual_signed_spawn"]; spec=plan["signed_recovery_profile"]
    wheelbase=manifest["coordinate_contract"]["wheelbase_m"]
    changed,rear=independently_perturb(spawn["nominal_actor_center_carla"],spec,wheelbase)
    require(spawn.get("schema") == COLLECTOR_SCHEMA+".spawn" and all(strict_equal(spawn.get(k),v) for k,v in spec.items())
        and strict_equal(spawn["perturbed_rear_reference_carla"],rear)
        and strict_equal(spawn["perturbed_actor_center_carla"],changed)
        and strict_equal(spawn["actual_requested_spawn_carla"],dict(changed,z=changed["z"]+.5))
        and spawn.get("applied_once_before_sensors_and_ticks") is True
        and spawn.get("pixels_or_observed_poses_post_transformed") is False
        and type(manifest["result"].get("actual_spawn_perturbation_call_count")) is int
        and manifest["result"]["actual_spawn_perturbation_call_count"] == 1, "actual once-only signed spawn differs")
    expected=dict(actual_manual_gear_shift=True,actual_gear=1,checker_function_source_sha256=CHECKER_SHA,
        original_frame_actor_PID_finite_and_full_brake_guards_preserved=True,observed_controls_or_snapshots_proxied=False)
    require(strict_equal(contract.get("signed_recovery_initialization_gear_contract"),expected), "original observed strict manual1 witness identity differs")
    startup=contract["signed_recovery_startup_teacher"]
    preset=dict(schema=COLLECTOR_SCHEMA+".startup",source_manual_controller_revision="carla.actual_signed_teacher_recovery.20261001.v3",
        launch_throttle=.11,post_handoff_native_time_pedal_slew_per_second=.025,normal_throttle_cap=.20,
        zero_normal_steering_until_speed_mps=.75,normal_manual_gear_shift=True,normal_gear=1,
        actual_teacher_target_cap_mps=2.4,original_BasicAgent_hazard_return_unchanged=True,
        startup_teacher_is_not_learned_recovery=True,observed_states_or_camera_pixels_changed=False)
    require(all(strict_equal(startup.get(k),v) for k,v in preset.items()), "disclosed manual1 startup teacher preset differs")
    return dict(status="PASS",actual_signed_spawn_identity_passed=True,actual_once_pre_sensor_spawn=True,
        geometry_reference="Rear-base ROS left/CCW; CARLA lateral/yaw signs inverted before actor creation.",
        startup_teacher_recovery_is_not_learned_recovery=True,nominal_actor_center=spawn["nominal_actor_center_carla"],
        independent_requested_actor_center_spawn=spawn["actual_requested_spawn_carla"])


def independent_initialization(manifest, states, receipts):
    """HH_261002 - Independently link original real manual1 constructor/engagement witnesses; never proxy observed flags."""
    contract=manifest["capture_contract"]; metadata=contract["agent_initialization"]
    fixed=dict(schema="carla.basic_agent_initialization.v1",mode="after_bootstrap",explicit_opt_in=True,
        required_control_transport="acknowledged_batch",existing_bootstrap_tick_reused=True,additional_world_ticks=0,
        constructor_requires_verified_bootstrap=True,pid_history_overwritten=False,observed_zero_steering_required=True,
        first_proposed_steering_delta_limit=.1,comparison_absolute_tolerance=1e-6,physical_cause_proven=False,training_data_approved=False)
    require(set(metadata) == set(fixed)|{"construction","engagement"}
        and all(strict_equal(metadata[k],v) for k,v in fixed.items()), "exact unchanged constructor witness contract required")
    construction, engagement=metadata["construction"],metadata["engagement"]
    if construction.get("status") != "PASS" or engagement.get("status") != "PASS" or engagement.get("first_command_acknowledged") is not True:
        return dict(status="FAIL",manual_first_gear_initialization_passed=False,original_construction=construction,
            original_engagement=engagement,physical_actuation_timing_proven=False)
    bootstrap=contract["control_transport"]["bootstrap_observation"]
    boot=[r for r in receipts if r.get("reason") == "pre_capture_bootstrap"]
    initial=[r for r in receipts if r.get("reason") == "initial_drive_control"]
    require(len(boot) == len(initial) == 1, "unique original bootstrap/initial-drive ACK required")
    br,ir=boot[0],initial[0]; actor=br["actor_id"]
    require(type(actor) is int and br["status"] == "ACKNOWLEDGED" and br["server_accepted"] is True
        and br["response_actor_id"] == actor and br["do_tick"] is False
        and br["before_frame"] == br["after_ack_frame"] == bootstrap["frame"]-1
        and bootstrap["alignment"]["status"] == "PASS"
        and bootstrap["alignment"]["expected_receipt_sequence"] == br["sequence"], "real bootstrap ACK/snapshot chain differs")

    def witness(value, reference, *, pid=False):
        require(value.get("status") == "PASS" and all(type(value.get(k)) is int and value[k] == reference["frame"]
            for k in ("expected_frame","frame_before","frame_after")) and value.get("actor_id") == actor
            and number(value["timestamp"]) == number(reference["timestamp"]), "no-advance initialization frame/actor/timestamp differs")
        pose=value["actor_snapshot_transform_carla"]
        require(set(pose) == {"x","y","z","roll","pitch","yaw"}
            and all(number(v) == number(reference["actor_snapshot_transform_carla"][k]) for k,v in pose.items()), "actual initialization snapshot pose differs")
        control=transport.full_control(value["current_control"])
        require(strict_equal(control,transport.full_control(reference["current_control"]))
            and control["throttle"] == control["steer"] == 0. and control["brake"] == 1.
            and control["manual_gear_shift"] is True and control["gear"] == 1
            and control["hand_brake"] is False and control["reverse"] is False,
            "initialization must observe true pure fullbrake manual1, not modified witness flags")
        require(value.get("raw_state_reference") == "CARLA actor snapshot API reference point; physical COM identity is unverified.", "reference-point caveat missing")
        if pid: require(number(value["pid_past_steering"]) == 0., "actual observed PID history must remain zero")

    witness(construction["before"],bootstrap); witness(construction["after"],bootstrap,pid=True)
    warmup=[s for s in states if s.get("capture_phase") == "stationary_warmup"]
    require(len(warmup) == 24 and states[:24] == warmup
        and [s["frame"] for s in warmup] == list(range(bootstrap["frame"]+1,bootstrap["frame"]+25)), "all existing24 warmup ticks required, no hidden additional tick")
    last=warmup[-1]; witness(engagement["before"],last,pid=True)
    proposed=transport.full_control(engagement["first_proposed_control"])
    sent=transport.full_control(engagement["first_sent_control"])
    delta=number(proposed["steer"])-number(engagement["before"]["pid_past_steering"])
    require(type(engagement["frame_after_agent_step"]) is int and engagement["frame_after_agent_step"] == last["frame"]
        and number(engagement["first_proposed_steering_delta"]) == delta and abs(delta) <= .1+1e-6, "first actual proposed steering/frame differs")
    expected_sent=dict(proposed)
    emergency=last.get("goal_stop",{}).get("next_control_is_unchanged_emergency_override") is True
    if not emergency and math.sqrt(sum(number(v)**2 for v in last["world_velocity_carla"])) < .3 \
            and (proposed["brake"] > .05 or proposed["hand_brake"]): expected_sent["steer"]=0.
    require(strict_equal(sent,expected_sent), "proposed/sent change is not original stopped steering guard")
    require(type(engagement["first_command_receipt_sequence"]) is int and ir["sequence"] == engagement["first_command_receipt_sequence"]
        and ir["status"] == "ACKNOWLEDGED" and ir["server_accepted"] is True
        and ir["actor_id"] == ir["response_actor_id"] == actor
        and ir["expected_before_frame"] == ir["before_frame"] == ir["after_ack_frame"] == last["frame"]
        and ir["do_tick"] is False and ir["physical_actuation_proven"] is False
        and strict_equal(sent,transport.full_control(ir["requested_control"]))
        and strict_equal(sent,transport.full_control(last["next_control"]))
        and last["control_transport"]["next_command_receipt_sequence"] == ir["sequence"], "initial sent control/accepted ACK differs")
    first=states[24]
    require(first.get("capture_phase") == "driving" and first["frame"] == last["frame"]+1
        and first["control_transport"]["expected_receipt_sequence"] == ir["sequence"]
        and first["control_transport"]["status"] == "PASS"
        and not transport.mismatched_controls(first["current_control"],sent), "first subsequent actual observation is not prior-ACK-bound")
    return dict(status="PASS",manual_first_gear_initialization_passed=True,construction_frame=bootstrap["frame"],
        engagement_frame=last["frame"],first_driving_observation_frame=first["frame"],first_command_receipt_sequence=ir["sequence"],
        original_construction=construction,original_engagement=engagement,all_original_warmup_ticks_retained=24,
        observed_flags_or_snapshots_proxied=False,physical_actuation_timing_proven=False,
        notice="Exact real API snapshot/accepted ACK chain; not physical instantaneous pedal-application timing or MAIN9 approval.")


def measured_rear_errors(states, route):
    """HH_261002 - Sequential independent offline projection uses only actual current/prior rear poses and archived route."""
    points=route["route"]; require(len(points) >= 2, "archived route segments required")
    distances=[number(p["distance_m"]) for p in points]
    require(all(b >= a for a,b in zip(distances,distances[1:])) and distances[-1] > 0., "monotonic archived route required")
    progress=0.; result=[]
    for row in states:
        x,y,yaw,timestamp=(number(row[k]) for k in ("x","y","yaw","timestamp"))
        best=None
        for index,(p,q) in enumerate(zip(points,points[1:])):
            if distances[index+1] < max(0.,progress-3.) or distances[index] > min(distances[-1],progress+80.): continue
            dx,dy=number(q["x"])-number(p["x"]),number(q["y"])-number(p["y"])
            squared=dx*dx+dy*dy
            ratio=max(0.,min(1.,((x-number(p["x"]))*dx+(y-number(p["y"]))*dy)/squared)) if squared > 1.e-9 else 0.
            px,py=number(p["x"])+ratio*dx,number(p["y"])+ratio*dy
            arc=distances[index]+ratio*(distances[index+1]-distances[index])
            candidate=(math.hypot(x-px,y-py),-arc,index,arc)
            if best is None or candidate < best: best=candidate
        require(best is not None, "no independently causal route segment available")
        progress=max(progress,best[3]); p,q=points[best[2]:best[2]+2]
        dx,dy=number(q["x"])-number(p["x"]),number(q["y"])-number(p["y"]); length=math.hypot(dx,dy)
        require(length > 1.e-6, "independent signed projection requires nondegenerate route")
        signed=(dx*(y-number(p["y"]))-dy*(x-number(p["x"])))/length
        yaw_error=math.degrees(math.atan2(math.sin(yaw-math.atan2(dy,dx)),math.cos(yaw-math.atan2(dy,dx))))
        result.append(dict(frame=row["frame"],timestamp=timestamp,signed_rear_CTE_m=signed,
            signed_rear_yaw_error_deg=yaw_error,independent_rear_arc_m=progress,independent_segment_index=best[2]))
    return result


def independent_recovery(states, commands, receipts, route, spec):
    """HH_261002 - First observed prior learned ACK starts the full interval; no teacher or outgoing same-frame effect is counted."""
    require(spec in PROFILES, "exact actual arm required")
    learned={r["ack_receipt_sequence"] for r in commands if r.get("application",{}).get("learned_control_applied") is True}
    prior=[]
    for state in states:
        receipt=previous.prior_ack(state,receipts)
        # HH_261002 - The frozen helper independently checks the latest accepted prior command; retain its exact sequence.
        if isinstance(receipt,tuple): receipt=receipt[0]
        prior.append(receipt["sequence"])
    start=next((i for i,sequence in enumerate(prior) if sequence in learned),None)
    if start is None:
        return dict(status="NOT_PROVEN_NO_LEARNED_EFFECT",recovery_screen_passed=False,case_screen_passed=False,
            first_learned_effect=None,convergence=None,post_effect_native_count=0)
    projected=measured_rear_errors(states,route)
    effects=states[start:]; errors=projected[start:]; is_signed=spec["signed_lateral_offset_ros_m"] != 0.
    first=errors[0]; first_cte,first_yaw=first["signed_rear_CTE_m"],first["signed_rear_yaw_error_deg"]
    present=(abs(first_cte) >= .10 and abs(first_yaw) >= .5) if is_signed else True
    correct=(first_cte*spec["signed_lateral_offset_ros_m"] > 0 and first_yaw*spec["signed_yaw_offset_ros_deg"] > 0) if is_signed else True
    complete=all(row.get("capture_phase") == "driving" and row.get("control_transport",{}).get("status") == "PASS"
        and prior[start+i] in learned for i,row in enumerate(effects))
    inside=None; convergence=None; metrics=[]
    for state,error in zip(effects,errors):
        elapsed=error["timestamp"]-first["timestamp"]
        metrics.append(dict(frame=state["frame"],elapsed_simulation_seconds=elapsed,signed_rear_CTE_m=error["signed_rear_CTE_m"],
            signed_rear_yaw_error_deg=error["signed_rear_yaw_error_deg"],expected_prior_ACK_sequence=prior[start+len(metrics)]))
        if abs(error["signed_rear_CTE_m"]) <= .10 and abs(error["signed_rear_yaw_error_deg"]) <= .5:
            if inside is None: inside=elapsed
            if convergence is None and inside <= 10. and elapsed-inside >= 2.-1.e-6:
                convergence=dict(first_converged_simulation_seconds=inside,continuous_converged_seconds=elapsed-inside,
                    confirmation_native_frame=state["frame"])
        else: inside=None
    passed=bool(present and correct and complete and convergence)
    status="PASS_BOUNDED_SIGNED_RECOVERY" if passed else "FAIL_BOUNDED_SIGNED_RECOVERY"
    if not present or not correct: status="NOT_PROVEN_INITIAL_PERTURBATION_REMOVED_OR_WRONG_SIGN"
    if not is_signed: status="PASS_CENTER_SETUP_COMPARATOR" if passed else "FAIL_CENTER_SETUP_COMPARATOR"
    return dict(status=status,noncentered_recovery_case=is_signed,case_screen_passed=passed,recovery_screen_passed=passed and is_signed,
        centered_setup_screen_passed=passed and not is_signed,
        first_learned_effect=dict(native_frame=effects[0]["frame"],timestamp_simulation_seconds=first["timestamp"],
            expected_prior_ACK_sequence=prior[start],signed_rear_CTE_m=first_cte,signed_rear_yaw_error_deg=first_yaw),
        initial_actual_perturbation_still_present=present,initial_signed_errors_match_declared_signs=correct,
        startup_teacher_recovery_is_not_learned_recovery=True,convergence=convergence,
        complete_post_effect_learned_interval=complete,post_effect_native_count=len(effects),
        rear_projection_is_offline_evaluation_not_runtime_input=True,full_post_effect_metric_rows=metrics)


def extra_trial_evidence(manifest, states, commands, receipts, route, plan, diagnostic, pins, episode):
    # HH_261002 - Additional raw teacher journal is pinned but is never substituted for actual learned ACK ownership.
    add_pin(pins,episode/"signed_startup_normal_teacher.jsonl")
    spawn=independent_spawn(manifest,plan)
    init=independent_initialization(manifest,states,receipts)
    recovery=independent_recovery(states,commands,receipts,route,plan["signed_recovery_profile"])
    require(diagnostic.get("schema") == OWNER_SCHEMA+".diagnostic_report"
        and diagnostic.get("actual_signed_spawn_identity_passed") == spawn["actual_signed_spawn_identity_passed"], "original spawn diagnostic differs from independently reconstructed geometry")
    original=diagnostic["bounded_signed_recovery"]
    require(set(original) == set(recovery), "original signed recovery metric fields differ")
    # HH_261002 - Compare every raw-derived metric, not only the reported success bit; arithmetic tolerance is not a gate tolerance.
    def equivalent(a,b):
        if isinstance(a,dict) or isinstance(b,dict): return isinstance(a,dict) and isinstance(b,dict) and set(a)==set(b) and all(equivalent(a[k],b[k]) for k in a)
        if isinstance(a,list) or isinstance(b,list): return isinstance(a,list) and isinstance(b,list) and len(a)==len(b) and all(equivalent(x,y) for x,y in zip(a,b))
        if type(a) is bool or type(b) is bool: return type(a) is type(b) and a==b
        if type(a) in (float,int) and type(b) in (float,int): return math.isfinite(a) and math.isfinite(b) and abs(a-b) <= 1.e-9
        return a==b
    require(equivalent(original,recovery), "independent full signed recovery/first prior ACK metrics differ from original report")
    return dict(independent_actual_spawn=spawn,independent_manual1_initialization=init,independent_signed_recovery=recovery)


def transformed_trial_reader():
    """HH_261002 - Preserve every original authenticator statement; insert only new independent setup/recovery evidence and conjunctive gates."""
    tree=ast.parse(Path(previous.__file__).read_bytes())
    function=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name == "load_trial")
    original=copy.deepcopy(function)
    anchor=[i for i,n in enumerate(function.body) if isinstance(n,ast.Expr) and isinstance(n.value,ast.Call)
        and any(isinstance(a,ast.Constant) and a.value == "independent driving/wall result differs from caller-pinned report" for a in n.value.args)]
    require(len(anchor) == 1, "frozen trial-reader conjunctive-result anchor changed")
    injected=ast.parse("""extra = extra_trial_evidence(manifest, states, commands, receipts, read_json(episode/'route.json'), plan, diagnostic, pins, episode)
basic_driving_before_new_recovery = driving
driving = driving and extra['independent_actual_spawn']['actual_signed_spawn_identity_passed'] and extra['independent_manual1_initialization']['manual_first_gear_initialization_passed'] and extra['independent_signed_recovery']['case_screen_passed']
""").body
    function.body[anchor[0]:anchor[0]]=injected
    returned=function.body[-1]
    require(isinstance(returned,ast.Return) and isinstance(returned.value,ast.Call) and isinstance(returned.value.func,ast.Name)
        and returned.value.func.id == "dict", "original full trial return AST differs")
    added=[ast.keyword(arg="signed_recovery_profile",value=ast.parse("plan['signed_recovery_profile']",mode="eval").body),
        ast.keyword(arg="basic_driving_before_new_recovery_passed",value=ast.Name(id="basic_driving_before_new_recovery",ctx=ast.Load())),
        ast.keyword(arg=None,value=ast.Name(id="extra",ctx=ast.Load()))]
    returned.value.keywords.extend(added)
    parity=copy.deepcopy(function); del parity.body[anchor[0]:anchor[0]+len(injected)]; del parity.body[-1].value.keywords[-len(added):]
    require(ast.dump(parity,include_attributes=False) == ast.dump(original,include_attributes=False), "all frozen trial authenticator statements must remain identical")
    namespace=dict(vars(previous)); namespace.update(verify_contract=verify_contract,extra_trial_evidence=extra_trial_evidence,
        audit_post_ACK_recording=writer.audit_post_ACK_recording)
    module=ast.fix_missing_locations(ast.Module(body=[function],type_ignores=[]))
    exec(compile(module,str(Path(__file__)) + ":original_authenticator_plus_signed_evidence","exec"),namespace)
    return namespace["load_trial"]


@contextmanager
def activated():
    """HH_261002 - Offline only, fully restored schema/source/reader bindings; captured sources and receipts are never rewritten."""
    global _active
    require(not _active and not writer._active, "signed audit cannot nest with another scope")
    verify_frozen()
    require(previous.verify_contract is _verify_contract and previous.load_trial is _load_trial,
        "unknown original audit function binding cannot be replaced")
    names=dict(SCHEMA=SCHEMA,RECEIPT_SCHEMA=RECEIPT_SCHEMA,OWNER_SCHEMA=OWNER_SCHEMA,FROZEN_HELPERS=FROZEN_HELPERS,
        verify_contract=verify_contract,load_trial=transformed_trial_reader())
    saved={name:getattr(previous,name) for name in names}; _active=True
    try:
        for name,value in names.items(): setattr(previous,name,value)
        with writer.activated(): yield
    finally:
        for name,value in saved.items(): setattr(previous,name,value)
        _active=False


def build_report(*args):
    with activated(): result=_build_report(*args)
    require(result["schema"] == SCHEMA and [r["signed_recovery_profile"] for r in result["trials"]] == list(PROFILES), "exact distinct three-arm independent report required")
    for path in source_closure(Path(__file__),ROOT/"tools/workspace_python.py",ROOT/"tools/canonical_workspace.py"): add_pin(result["source_sha256"],path)
    caller=read_json(args[2])
    history=configuration_attempt_history(caller,result["source_sha256"])
    cases=[r["independent_signed_recovery"] for r in result["trials"]]
    result.update(signed_recovery_case_count=2,centered_setup_comparator_count=1,
        signed_recovery_pass_count=sum(r["recovery_screen_passed"] for r in cases[:2]),
        all_two_signed_recovery_cases_passed=all(r["recovery_screen_passed"] for r in cases[:2]),
        centered_setup_comparator_passed=cases[2]["case_screen_passed"],
        no_teacher_recovery_counted_as_learned=True,all_three_are_not_same_seed_repetitions=True,
        recovery_entry_deadline_seconds=10.,recovery_confirmation_continuous_seconds=2.,
        confirmation_may_occur_by12_seconds=True,first_effect_is_latest_prior_actual_learned_ACK=True,
        exact_writer_camera_order=list(writer.WRITER_CAMERA_ORDER),writer_order_audit_revision=writer.AUDIT_REVISION_SCHEMA,
        actual_manual_first_gear_initialization_independently_checked=True,
        previous_configuration_attempts=history,configuration_attempt_history_supplied=bool(history),
        previous_configuration_failure_is_not_model_driving_failure=True,
        independent_map_validation=False,full_route_goal_stop_approved=False,full_MAIN9_completion_claimed=False)
    frozen.assert_pins(result["source_sha256"])
    return result


def configuration_attempt_history(caller,pins):
    """HH_261002 - Optional caller-pinned pre-capture failures remain a separate appendix, never fabricated900/450 driving results."""
    records=caller.get("configuration_attempts",[])
    require(isinstance(records,list), "configuration history must be a caller-pinned list")
    result=[]
    for item in records:
        require(isinstance(item,dict) and item.get("run_id") in ("v1","v2"), "only the two known aborted configuration versions are allowed")
        run=item["run_id"]
        batch=ROOT/("storage/experiments/validation/2026-10-02/temporal_signed_recovery45_town07_"+run)
        plan=batch.with_name(batch.name+"_plan.json");add_pin(pins,plan,item["batch_declaration_sha256"])
        declaration=read_json(plan)
        expected_schema="portable_e2e.actual_signed_recovery45_three_arms.20261002."+run+".batch"
        require(declaration["schema"] == expected_schema and declaration["output"] == str(batch), "aborted configuration declaration identity differs")
        if run == "v1":
            artifact=batch/"trial_00/owner_result.json"
            add_pin(pins,artifact,item["trial0_owner_result_sha256"]);owner=read_json(artifact)
            require(owner.get("status") in ("OWNER_FAILURE","CAPTURE_RETURNED_NONZERO_RETAINED")
                and owner.get("learned_driving_screen_passed") is False
                and not any((batch/("trial_%02d"%i)).exists() for i in (1,2)),
                "invalid-seed v1 must remain one failed owner and two unexecuted arms")
            raw=[]
            for alias in ("episode","episode.partial"):
                for name in ("states.jsonl","camera_frames.jsonl","learned_control.jsonl"):
                    path=batch/"trial_00"/alias/name
                    if path.exists():
                        add_pin(pins,path); require(not rows(path), "v1 is not a zero-observation pre-capture failure");raw.append(str(path))
            result.append(dict(run_id=run,status="FAILED_BEFORE_CAPTURE_INVALID_UINT32_SEED",batch_declaration_sha256=item["batch_declaration_sha256"],
                original_owner_result_sha256=item["trial0_owner_result_sha256"],failed_owner_trial_count=1,
                unexecuted_trial_indices=[1,2],actual_native_and_camera_observations=0,empty_raw_journals=raw,model_performance_evaluated=False))
        else:
            require(not any((batch/("trial_%02d"%i)).exists() for i in range(3)), "v2 was not an unexecuted preflight-only configuration")
            result.append(dict(run_id=run,status="DECLARED_BUT_NO_PHYSICAL_TRIAL_OUTPUT",batch_declaration_sha256=item["batch_declaration_sha256"],
                unexecuted_trial_indices=[0,1,2],model_performance_evaluated=False,
                preflight_error_detail_not_independently_verified_without_receipt=True))
    require(len({r["run_id"] for r in result}) == len(result), "duplicate previous configuration attempts")
    return result


def markdown(report):
    def fmt(value): return "미측정" if value is None else "%.3f"%value
    lines=["# Town07 실제 복귀 — 좌·우 2조건 + 중앙 비교 1조건","",
        "<!-- HH_261002 - Independently distinguish actual learned recovery from teacher startup and full-feature completion. -->","",
        "좌·우 실제 복귀 %d/2 · 중앙 출발 비교 %s · 주행/차체/정책/관측→ACK 모두 통과 %d/3." % (report["signed_recovery_pass_count"],
            "PASS" if report["centered_setup_comparator_passed"] else "FAIL",report["joint_motion_body_policy_wall_trial_pass_count"]),"",
        "| 조건 | 첫 실제 모델 효과: 부호 CTE / 방향 오차 | 밴드 진입 / 2초 확인 | 전진 | native base 속도 최대 | 관측→ACK 최대 | 전체 기준 |",
        "|---|---:|---:|---:|---:|---:|---|"]
    for row in report["trials"]:
        recovery=row["independent_signed_recovery"]; first=recovery.get("first_learned_effect") or {}; convergence=recovery.get("convergence") or {}
        entry=convergence.get("first_converged_simulation_seconds")
        confirmation=entry+convergence["continuous_converged_seconds"] if entry is not None else None
        recovery_label="복귀 PASS" if recovery.get("recovery_screen_passed") else "중앙 비교 PASS" if recovery.get("centered_setup_screen_passed") else recovery["status"]
        lines.append("| %s | %s m / %s° | %s / %s s | %s m | %s m/s | %s ms | %s |" % (row["signed_recovery_profile"]["case_id"],
            fmt(first.get("signed_rear_CTE_m")),fmt(first.get("signed_rear_yaw_error_deg")),fmt(entry),fmt(confirmation),
            fmt(row["native_actor_progress_m"]),fmt(row.get("post_handoff_base_norm_maximum_mps")),fmt(row["delivery_to_ACK_ms"].get("maximum")),
            "PASS" if row["joint_motion_body_policy_wall100ms_passed"] else "FAIL ("+recovery_label+")"))
    lines += ["","앞선 구성 오류는 주행 성능과 구분합니다. v1은 seed 범위 오류로 관측 전 실패했고 v2는 활성화 전 검증 경로 오류로 실제 주행을 실행하지 못했습니다. 기존 선언·실패 자료는 보존하며 새 v3 결과로 덮어쓰지 않습니다. 이 보고서에 이전 자료의 caller SHA가 제공되지 않았다면 해당 오류의 원자료 독립 검산까지 완료했다고 주장하지 않습니다.","",
        "실제 센서 생성 전 차량 위치 ±0.20 m·방향 ±1.5°를 옮깁니다. 중앙 조건은 비교용이며 복귀 2조건의 분모에 넣지 않습니다. 출발은 공개된 manual1 전문가 설정이며, 첫 **이전 모델 ACK가 실제 다음 상태에 반영된 때**부터만 모델 복귀를 계산합니다.","",
        "좌·우 조건은 첫 모델 효과 때 실제 오차가 각각 0.10 m·0.5° 이상이고 선언된 부호여야 합니다. 출발 전문가가 먼저 복귀시켰다면 미증명입니다. 모델 효과 이후 10초 안에 0.10 m·0.5° 밴드로 들어가고 연속 2초 유지해야 합니다(확인은 최대 12초). 거절·정지·비상제동 이후 상태도 전체 평가에 남깁니다.","",
        "전체 45초 × 3조건, 2,700 native 관측·1,350 실제 6카메라 묶음·8,100 JPEG·2,712 ACK를 고정 분모로 독립 확인합니다. 차체 900개/연속 sweep 899개와 차선 10 cm 여유, 90 m 이상 전진, 안전·속도·정책 100 ms 기준은 이전 시험과 같습니다. 원래 실패 기록은 덮어쓰지 않습니다.","",
        "정책 100 ms와 관측 bundle delivery→ACK 벽시계 100 ms는 각각 검증합니다. 노출/callback 진입부터의 완전한 센서→ACK 10 Hz는 아직 미측정입니다. 수락된 fresh 제안 0개는 미증명/FAIL이며 거절 원자료도 별도로 보존합니다.","",
        "현재 모델은 final3000의 고정 97개 backbone + 4개 학습 회복 branch이며 PI 목표 2.40 m/s는 명시적 제어 보정입니다. rolling 4.0–34.1초 복귀를 학습했으며 초기 출발 학습·다른 맵 독립성·완전한 경로 도착/정지·회전·실차 성능은 주장하지 않습니다. 정식 기능 승인 9개 중 0개 / 하위 30개 중 0개이며 이 시험을 통과해도 MAIN9 개발은 계속됩니다.","",
        "[전체 독립 확인 JSON](batch_outcome.json) · [원자료·소스 SHA256](source_evidence.json)",""]
    return "\n".join(lines)


def publish(output,*args):
    output=Path(output); require(not output.exists() and not output.is_symlink(), "fresh create-only signed audit output required")
    result=build_report(*args); frozen.assert_pins(result["source_sha256"]); output.mkdir(parents=True,exist_ok=False)
    for name,value in (("batch_outcome.json",result),("source_evidence.json",result["source_sha256"])):
        with (output/name).open("x") as stream: json.dump(value,stream,ensure_ascii=False,indent=2,allow_nan=False)
    with (output/"README.md").open("x") as stream: stream.write(markdown(result))
    frozen.assert_pins(result["source_sha256"])
    return dict(output=str(output),result_sha256=digest(output/"batch_outcome.json"),
        all_trials_joint_passed=result["all_trials_motion_body_policy_AND_wall100ms_passed"],simulator_or_GPU_used=False)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    for name in ("batch","caller-receipts","output"): parser.add_argument("--"+name,type=Path,required=True)
    for name in ("batch-sha256","caller-receipts-sha256"): parser.add_argument("--"+name,required=True)
    parser.add_argument("--validate-only",action="store_true"); args=parser.parse_args(argv)
    inputs=(args.batch,args.batch_sha256,args.caller_receipts,args.caller_receipts_sha256)
    result=build_report(*inputs) if args.validate_only else publish(args.output,*inputs)
    print(json.dumps(result,ensure_ascii=False,allow_nan=False)); return 0


if __name__ == "__main__": raise SystemExit(main())
