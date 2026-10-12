"""HH_261001 - Qualify prospectively fixed V4 rolling-recovery TRAIN or reserved HOLDOUT without changing native denials."""

from __future__ import annotations

import argparse
from bisect import bisect_left
from collections import Counter
import gzip
import json
import math
from pathlib import Path
import shutil

from scripts.e2e import prepare_carla_common10_dataset as adapter
from scripts.e2e import qualify_carla_moving_common10 as independent
from scripts.e2e import town07_signed_recovery_task_20261001 as task
from scripts.e2e import run_owned_town07_signed_recovery_v4_20261001 as cohort
from scripts.e2e import audit_carla_raw_pre_admission as raw
from scripts.e2e import audit_carla_turn_launch_raw_geometry as futures_math
from scripts.e2e import audit_town07_low_speed_moving_pending_20261001 as native_math
from scripts.e2e import audit_carla_acknowledged_control as ack
from scripts.e2e import research_temporal_v2_owned_camera_town07_20261001 as source_closure
from portable_e2e import contract, dataset


ROOT=task.ROOT
SCHEMA="carla.actual_signed_recovery_common10_qualification.20261001.v4"
RESULT_SCHEMA="carla.actual_signed_recovery_common10_conversion.20261001.v4"
ADAPTER_ID="carla_actual_signed_recovery_to_common10_20261001_v4"
COUNT,NATIVE,CAMERA=302,965,483
ROLLING_SCOPE=dict(rolling_recovery_only=True,startup_anchors_used_for_training=False,
    raw_startup_retained_as_context_only=True,fixed_driving_relative_anchor_offsets=[80,682],
    fixed_anchor_driving_seconds=[4.0,34.1])
SOURCE_PATHS=("tools/e2e/qualify_town07_signed_recovery_common10_20261001.py",
    "tools/e2e/prepare_town07_signed_recovery_common10_20261001.py",
    "tools/e2e/prepare_carla_common10_dataset.py","tools/e2e/qualify_carla_moving_common10.py",
    "tools/e2e/audit_carla_raw_pre_admission.py","tools/e2e/audit_carla_turn_launch_raw_geometry.py",
    "tools/e2e/audit_town07_low_speed_moving_pending_20261001.py","tools/e2e/audit_carla_acknowledged_control.py",
    "tools/e2e/audit_portable_target_feasibility.py","learning/portable_e2e/contract.py",
    "learning/portable_e2e/dataset.py","learning/portable_e2e/torch_dataset.py",
    "learning/portable_e2e/temporal_dataset_v2.py","learning/portable_e2e/config/common_10hz_v1.contract.json")
require=independent.require


def safe(path,*,external=False):
    value=Path(path).absolute()
    require(".." not in value.parts and value==value.resolve() and not any(p.is_symlink() for p in (value,*value.parents)),"canonical nonsymlink input required")
    require(external or value.is_relative_to(ROOT),"research path escaped workspace")
    return value


def sha(path):return adapter._sha256(safe(path,external=True))
def document(path):return adapter._json(safe(path))


def identities(group):
    require(group in ("train","holdout"),"explicit TRAIN or reserved HOLDOUT scope required")
    return tuple(row["case_id"] for row in cohort.CASES if (row["split"]=="train")== (group=="train"))


def prepared_path(run_id,group):
    require(run_id=="v4","failed v1/v2/v3 or arbitrary recovery cohort cannot be admitted")
    task.paths(run_id)
    identities(group)
    return ROOT/("storage/datasets/prepared/town07-signed-recovery-"+group+"-20261001-"+run_id)


def dataset_id(run_id,group):return prepared_path(run_id,group).name


def scope_manifest(manifest):
    # HH_261001 - Preserve BOTH original explicit-denial guards, including every old trainingFalse restriction.
    adapter._require_native_admission_not_denied(manifest)
    contract._require_admission_not_denied(manifest,"native signed recovery",include_native_metadata=True)
    require(manifest.get("status")=="complete" and manifest.get("cleanup",{}).get("completed") is True,"complete raw capture/cleanup required")
    result=manifest.get("result",{})
    require(result.get("task_complete") is True and result.get("goal_reached") is False,
        "completed prospective recovery task with truthful false catalog goal required")
    for value in (manifest,result,result.get("moving_task",{})):
        require(value.get("admission_status")=="PENDING_QA" and value.get("training_admission_state")=="PENDING_QA_NOT_AUTHORIZED"
            and "training_data_approved" not in value and value.get("vehicle_control_approved") is False,
            "fresh unapproved native PENDING semantics changed")
    require(result.get("collision_event_count")==result.get("lane_invasion_event_count")==0,"raw collision/lane events prevent conversion")


def teacher_audit(commands,receipts,states):
    # HH_261001 - Replay unchanged V3 control under the new V4 data scope on actual actor velocity, never base-vx proxy.
    expected=[r for r in receipts if r["reason"] in ("initial_drive_control","next_driving_control")]
    actual=[r for r in commands if r["reason"] in ("initial_drive_control","next_driving_control")]
    require([r["ack_receipt"] for r in actual]==expected,"teacher command/ACK denominator differs")
    by_frame={r["frame"]:r for r in states}
    # HH_261001 - Instantiate the explicit NEW controller, never rely on an implicit mutable module alias.
    policy=cohort.v3.FirstGearRecoveryLaunchControl();proposal=0;emergencies=[]
    for row in actual:
        receipt=row["ack_receipt"];frame=receipt["before_frame"];normal=row["normal_policy"]
        require(row["learned_control"] is False and row["teacher_control"] is True
            and not ack.mismatched_controls(row["final_BasicAgent_or_brake_proposal"],receipt["requested_control"]),"actual teacher proposal/ACK differs")
        state=by_frame[frame]
        if state.get("moving_task",{}).get("termination_reason") is not None:
            require(normal is None and receipt["requested_control"]["throttle"]==0. and receipt["requested_control"]["brake"]==1.,"future-support closure must retain full-brake cleanup")
            continue
        target=2.4 if proposal==0 else min(2.4,.075*proposal);proposal+=1
        ramp=row["target_ramp"]
        require(ramp["source_frame"]==frame and math.isclose(ramp["actual_teacher_target_mps"],target,abs_tol=1e-12)
            and ramp["teacher_target_cap_mps"]==2.4 and ramp["actual_observed_speed_clipped"] is False,"actual teacher target ramp differs")
        if normal is None:
            emergencies.append(frame)
            require(receipt["requested_control"]["throttle"]==0. and receipt["requested_control"]["brake"]>0.,"missing normal proposal is not genuine hazard brake")
            continue
        speed=math.hypot(*state["world_velocity_carla"][:2])
        require(normal["source_frame"]==frame and abs(normal["source_timestamp"]-state["timestamp"])<=1e-9
            and abs(normal["measured_planar_speed_mps"]-speed)<=1e-9,"normal teacher state is not current actual actor speed")
        raw_control=normal["raw_normal_planner_proposal"]
        capped=dict(raw_control,throttle=min(.20,raw_control["throttle"]),brake=0.)
        control=native_math.RecordedFloatControl(capped)
        policy.apply(control,speed=speed,timestamp=state["timestamp"],frame=frame,normal_proposal=raw_control)
        require(policy.last==normal,"prospectively declared NEW normal teacher controller replay differs")
        if ack.mismatched_controls(normal["profile_normal_control"],receipt["requested_control"]):
            emergencies.append(frame)
            require(receipt["requested_control"]["throttle"]==0. and receipt["requested_control"]["brake"]>0.,"BasicAgent hazard override was transformed")
    return dict(status="PASS",actual_driving_ACK_count=len(actual),independent_prospective_new_normal_teacher_replay=True,
                normal_launch_controls_inherited_v2=False,actual_actor_speed_reference=True,hazard_override_frames=emergencies)


def verify_teacher_metadata(manifest):
    """HH_261001 - Fail before pixel/window QA if old teacher metadata disguises the new prospective launch."""
    require(manifest.get("control_source")=="Disclosed_BasicAgent_DATA_V4_moving_recovery_with_unchanged_manual_first_gear_V3_controller",
            "actual disclosed V4 rolling teacher source differs")
    expected=dict(normal_launch_controls_inherited_v2=False,
        normal_launch_controller_revision=cohort.v3.TASK_SCHEMA+".preselected_first_gear_normal_control",
        launch_throttle=.11,launch_handoff_mps=.5,maximum_launch_seconds=8.,post_handoff_throttle_ramp_per_second=.025,
        post_handoff_cap=.20,normal_brake_cap=0.,low_speed_zero_steering_mps=.75,inherited_BasicAgent_emergency_unchanged=True,
        measured_speed_clipped=False,measured_acceleration_clipped=False,native_acceleration_qa_limit_mps2=3.,
        launch_clock="actual world snapshot time",launch_speed_source="actual CARLA planar actor velocity",
        prospective_calibration_not_acceleration_admission_claim=True,gear=1,manual_gear_shift=True,
        bootstrap_and_stationary_blank_full_brake_first_gear=True,
        normal_only_gear_intervention_before_unchanged_BasicAgent_hazard_override=True,
        final_hazard_return_modified_after_override=False,inherited_v2_launch_calibration=True,
        actual_measured_states_unchanged=True)
    value=manifest.get("capture_contract",{}).get("teacher_controller",{})
    require(all(value.get(key)==item and type(value.get(key)) is type(item) for key,item in expected.items()),
            "new teacher launch/hazard/unchanged actual-state metadata differs")
    expected_gear=dict(actual_manual_gear_shift=True,actual_gear=1,
        parent_function_source_sha256=cohort.v3.PARENT_CHECKER_SOURCE_SHA,
        all_original_frame_actor_finite_PID_and_full_brake_guards_preserved=True,
        observed_control_or_snapshot_proxy_used=False)
    require(value.get("initialization_gear_contract")==expected_gear,
            "explicit new manual-first initialization gear contract differs")
    # HH_261001 - This is a prospective rolling data interval, not a claim that startup defects were fixed or fitted.
    scope=dict(schema=cohort.TASK_SCHEMA,selected_driving_native_frames=[80,682],
        selected_driving_relative_time_ns=[4_000_000_000,34_100_000_000],all_raw_startup_retained=True,
        startup_training_claimed=False,per_case_adaptive_or_quality_crop=False,
        controller_and_initialization_revision_unchanged_from_manual_first_gear_V3=True,
        expected_native_states=NATIVE,expected_camera_bundles=CAMERA)
    require(manifest.get("capture_contract",{}).get("moving_recovery_training_scope")==scope,
            "explicit prospective rolling-only scope/raw startup retention differs")


def initialization_audit(manifest,states,receipts):
    """HH_261001 - Independently bind NEW manual-first constructor/engagement to actual snapshots and native ACKs.

    The historical automatic-only auditor is NOT called or relabelled PASS.
    Frame, actor, pose, service brake, zero PID, first proposal and subsequent
    application remain exact; only the prospectively declared gear predicate
    is manualTrue/firstgear1. No control/snapshot proxy or source rewrite occurs.
    """
    capture=manifest["capture_contract"];metadata=capture["agent_initialization"]
    fixed=dict(schema="carla.basic_agent_initialization.v1",mode="after_bootstrap",explicit_opt_in=True,
        required_control_transport="acknowledged_batch",existing_bootstrap_tick_reused=True,additional_world_ticks=0,
        constructor_requires_verified_bootstrap=True,pid_history_overwritten=False,observed_zero_steering_required=True,
        first_proposed_steering_delta_limit=.1,comparison_absolute_tolerance=1e-6,physical_cause_proven=False)
    require(all(metadata.get(k)==v and type(metadata.get(k)) is type(v) for k,v in fixed.items()),
            "original initialization frame/PID/timing contract changed")
    initial=[r for r in receipts if r["reason"]=="initial_drive_control"]
    bootstrap_receipts=[r for r in receipts if r["reason"]=="pre_capture_bootstrap"]
    require(len(initial)==len(bootstrap_receipts)==1,"unique initial/bootstrap ACK witnesses required")
    bootstrap=capture["control_transport"]["bootstrap_observation"];br=bootstrap_receipts[0];actor=br["actor_id"]
    require(br["status"]=="ACKNOWLEDGED" and br["server_accepted"] is True and br["response_actor_id"]==actor
        and br["do_tick"] is False and br["before_frame"]==br["after_ack_frame"]==bootstrap["frame"]-1
        and bootstrap["alignment"]["status"]=="PASS" and bootstrap["alignment"]["expected_receipt_sequence"]==br["sequence"],
        "actual bootstrap ACK/application differs")
    def witness(value,reference,*,pid):
        require(value["status"]=="PASS" and all(type(value[k]) is int and value[k]==reference["frame"]
            for k in ("expected_frame","frame_before","frame_after")) and value["timestamp"]==reference["timestamp"]
            and type(value["actor_id"]) is int and value["actor_id"]==actor,"initialization native identity/timestamp differs")
        pose=value["actor_snapshot_transform_carla"]
        require(set(pose)=={"x","y","z","roll","pitch","yaw"} and all(type(v) in (int,float) and math.isfinite(v) for v in pose.values())
            and pose==reference["actor_snapshot_transform_carla"],"initialization actual snapshot pose differs")
        control=ack.full_control(value["current_control"])
        require(control==ack.full_control(reference["current_control"]) and control["throttle"]==control["steer"]==0.
            and control["brake"]==1. and control["hand_brake"] is False and control["reverse"] is False
            and control["manual_gear_shift"] is True and type(control["gear"]) is int and control["gear"]==1,
            "actual constructor/engagement full-brake manual-first control differs")
        require(value["raw_state_reference"]=="CARLA actor snapshot API reference point; physical COM identity is unverified.",
                "unverified physical-reference caveat changed")
        if pid:require(type(value["pid_past_steering"]) in (int,float) and value["pid_past_steering"]==0.,"actual zero PID history required")
    construction,engagement=metadata["construction"],metadata["engagement"]
    require(construction["status"]==engagement["status"]=="PASS" and engagement["first_command_acknowledged"] is True,
            "construction or engagement failed")
    witness(construction["before"],bootstrap,pid=False);witness(construction["after"],bootstrap,pid=True)
    warmup=[s for s in states if s["capture_phase"]=="stationary_warmup"]
    require(len(warmup)==24 and states[:24]==warmup and [s["frame"] for s in warmup]==list(range(bootstrap["frame"]+1,bootstrap["frame"]+25)),
            "declared exact24 warmup native ticks not retained")
    last=warmup[-1];witness(engagement["before"],last,pid=True)
    proposed=ack.full_control(engagement["first_proposed_control"]);sent=ack.full_control(engagement["first_sent_control"])
    delta=proposed["steer"]-engagement["before"]["pid_past_steering"]
    require(type(engagement["frame_after_agent_step"]) is int and engagement["frame_after_agent_step"]==last["frame"]
        and engagement["first_proposed_steering_delta"]==delta and abs(delta)<=.1+1e-6,"first BasicAgent step/frame/PID delta differs")
    expected=dict(proposed);speed=math.sqrt(sum(v*v for v in last["world_velocity_carla"]))
    if last.get("goal_stop",{}).get("next_control_is_unchanged_emergency_override") is not True and speed<.3 and (proposed["brake"]>.05 or proposed["hand_brake"]):
        expected["steer"]=0.
    require(sent==expected,"first proposal changed beyond original emergency/stopped-steering guard")
    receipt=initial[0]
    require(type(engagement["first_command_receipt_sequence"]) is int and engagement["first_command_receipt_sequence"]==receipt["sequence"]
        and receipt["status"]=="ACKNOWLEDGED" and receipt["server_accepted"] is True and receipt["actor_id"]==receipt["response_actor_id"]==actor
        and receipt["expected_before_frame"]==receipt["before_frame"]==receipt["after_ack_frame"]==last["frame"]
        and receipt["do_tick"] is False and receipt["physical_actuation_proven"] is False
        and sent==ack.full_control(receipt["requested_control"])==ack.full_control(last["next_control"])
        and last["control_transport"]["next_command_receipt_sequence"]==receipt["sequence"]
        and ("goal_stop" not in last or last["goal_stop"].get("next_control_starts_driving") is True),
        "first proposal/sent/native/ACK chain differs")
    first=states[24]
    require(first["capture_phase"]=="driving" and first["frame"]==last["frame"]+1
        and first["control_transport"]["expected_receipt_sequence"]==receipt["sequence"] and first["control_transport"]["status"]=="PASS"
        and not ack.mismatched_controls(first["current_control"],sent),"first subsequent native application is not actual receipt-bound")
    return dict(schema=cohort.TASK_SCHEMA+".initialization_independent_audit",status="PASS",constructor_frame=bootstrap["frame"],
        engagement_frame=last["frame"],first_actual_driving_frame=first["frame"],actual_manual_first_gear_preselected_before_throttle=True,
        historical_automatic_only_auditor_relabelled=False,observed_control_or_snapshot_proxy_used=False,
        exact_native_warmup_count=24,PID_history_overwritten=False,physical_actuation_proven=False)


def measured_v0_first_edge_checks(future):
    """HH_261001 - Supplement frozen legacy-vx diagnostics with unchanged bounds at actual hypot(vx,vy)."""
    # HH_261001 - Labels, native states and old diagnostics are never rewritten; only the decoder start is reassessed.
    if future.get("valid_future_points",0)<1 or "discrete_bound_diagnostic" not in future:
        return ["measured_v0_first_edge_unavailable"]
    targets=raw.targets;velocity=future["current_recorded_velocity_base_mps"]
    v0=math.hypot(*velocity[:2]);speed=future["diagnostic_endpoint_planar_speed_mps"][0]
    xy=future["diagnostic_future_xy_m"][0];yaw=future["diagnostic_body_yaw_delta_rad"][0]
    dt=(future["diagnostic_target_timestamp_ns"][0]-future["anchor_timestamp_ns"])/1e9
    require(dt==.1 and all(math.isfinite(v) for v in (v0,speed,*xy,yaw)),"measured start/first target is not finite original100ms label")
    geo_speed=math.hypot(*xy)/dt;acceleration=(speed-v0)/dt;geo_acceleration=(geo_speed-v0)/dt
    maximum=targets.PHYSICAL_MAXIMUM_ACCELERATION_MPS2;tolerance=targets.ROUNDING_TOLERANCE
    tests={"speed_acceleration":acceleration>maximum+tolerance,"speed_deceleration":acceleration < -maximum-tolerance,
           "xy_acceleration":geo_acceleration>maximum+tolerance,"xy_deceleration":geo_acceleration < -maximum-tolerance}
    curvature=future["discrete_bound_diagnostic"]["steps"][0]["metrics"]["xy_curvature"][0]
    tests["xy_lateral_acceleration"]=curvature is not None and curvature*max(v0,geo_speed)**2>targets.PHYSICAL_MAXIMUM_LATERAL_ACCELERATION_MPS2+tolerance
    if speed*dt>targets.HEADING_MINIMUM_STEP_M:
        limit=min(targets.PHYSICAL_MAXIMUM_CURVATURE_RAD_PER_M,targets.PHYSICAL_MAXIMUM_LATERAL_ACCELERATION_MPS2/max(v0,speed,1e-6)**2)*speed*dt
        tests["yaw_label_heading_envelope"]=abs(math.atan2(math.sin(yaw),math.cos(yaw)))>limit+tolerance
    return ["measured_v0_"+name for name,failed in tests.items() if failed]


def load_raw_journals(episode,names):
    """HH_261001 - Return unchanged native JSON rows under the frozen list-only independent ACK API."""
    return [list(adapter._jsonl(episode/name)) for name in names]


def window_checks(selected,cameras,anchors,states,geometry,futures):
    # HH_261001 - All declared windows are checked; failures never remove an anchor from this fixed denominator.
    native=native_math.native_checks(states,geometry)
    stamps=[r["timestamp_ns"] for r in native];indices={r["frame"]:i for i,r in enumerate(native)}
    protected=set();windows=[]
    for anchor in selected:
        index=anchor["camera_index"];first=index-9;history=cameras[max(0,first):index+1]
        lo=indices[history[0]["frame"]]-1;hi=bisect_left(stamps,anchor["timestamp_ns"]+6_400_000_000)
        failures=[]
        if first<0 or lo<0 or len(history)!=10 or not all(r["capture_phase"]=="driving" for r in history):failures.append("real_ten_camera_history")
        if not all(b["frame"]-a["frame"]==2 for a,b in zip(history,history[1:])):failures.append("history_frame_grid")
        future=futures[index]
        if future.get("valid_future_points")!=64 or future.get("future_crosses_stationary_tail") is True or hi>=len(native):failures.append("unaltered64future_before_tail")
        lo=max(0,lo);hi=min(hi,len(native)-1);covered=set(range(lo,hi+1));protected.update(covered)
        for check in native_math.NATIVE_CHECKS:
            # HH_261001 - The V4 prior50ms history guard must genuinely be driving, never warmup or synthesized context.
            clear=all(native[i]["checks"][check] for i in covered)
            if not clear:failures.append(check)
        if not all(r["same_recorded_frame_and_timestamp"] for r in anchors[max(0,first):index+1]):failures.append("camera_native_history_binding")
        if not native_math.nine_clear(future):failures.append("original_all_nine_future_bounds")
        if not all(native_math.nine_clear(r,first_only=True) for r in futures[max(0,first):index]):failures.append("original_all_nine_history_edge_bounds")
        failures.extend(measured_v0_first_edge_checks(future))
        if any(measured_v0_first_edge_checks(r) for r in futures[max(0,first):index]):failures.append("measured_v0_history_first_edge_bounds")
        for row in native[lo:hi+1]:
            if row["speed_mps"]>2.5:failures.append("unchanged_low_speed_base_v0_support");break
        windows.append(dict(**anchor,protected_native_indices=[lo,hi],history_camera_indices=[first,index],
                            necessary_checks_clear=not failures,failure_reasons=failures))
    return windows,native_math.scoped_failures(native,protected)


def case_evidence(run_id,case_id,plan_sha,pin):
    """HH_261001 - Replay only the complete new V4 cohort through its explicitly restored task scope."""
    require(run_id=="v4","old v1/v2/v3 subsets cannot become the new rolling recovery dataset")
    with cohort.activated():return _case_evidence(run_id,case_id,plan_sha,pin)


def inspect_case(run_id,case_id,plan_sha):
    """HH_261001 - Inspect one captured case without writing or asserting any admission authority."""
    pins={}
    def pin(path,expected=None):
        path=safe(path,external=True);value=sha(path)
        require(expected is None or value==expected,"debug source/artifact pin differs: "+str(path))
        require(str(path) not in pins or pins[str(path)]==value,"debug input changed between reads")
        pins[str(path)]=value
    result=case_evidence(run_id,case_id,plan_sha,pin)
    for path,value in pins.items():require(sha(path)==value,"case debug source/artifact changed during read")
    return dict(schema=SCHEMA+".case_debug",run_id=run_id,case_id=case_id,
        status="CASE_NECESSARY_CHECKS_CLEAR_NOT_ADMITTED" if result["all_fixed_anchor_necessary_checks_clear"] else "CASE_NATIVE_OR_FUTURE_CHECKS_FAILED_NOT_ADMITTED",
        raw_native_states=result["raw_native_states"],raw_camera_anchors=result["raw_camera_anchors"],raw_image_count=result["raw_image_count"],
        selected_camera_anchors=result["selected_camera_anchors"],selected_span_ns=result["selected_span_ns"],
        all_fixed_anchor_necessary_checks_clear=result["all_fixed_anchor_necessary_checks_clear"],
        failed_fixed_window_count=sum(not row["necessary_checks_clear"] for row in result["all_fixed_anchor_window_checks"]),
        window_check_counts=result["window_check_counts"],protected_native_failures=result["protected_native_failures"],
        outside_protected_native_failures=result["outside_protected_native_failures"],
        failed_window_diagnostics=[row for row in result["all_fixed_anchor_window_checks"] if not row["necessary_checks_clear"]],
        native_manifest_sha256=result["native_manifest_sha256"],case_plan_sha256=plan_sha,
        full_all_eleven_cohort_checked=False,research_training_authorized=False,research_holdout_evaluation_authorized=False,
        training_data_approved=False,vehicle_control_approved=False,raw_native_denials_overridden=False,
        admission_receipt_written=False,dataset_conversion_performed=False,**ROLLING_SCOPE,
        notice="Debug-only one-case QA; fixed302 denominator retained, no failed-window removal or group admission.")


def _case_evidence(run_id,case_id,plan_sha,pin):
    case,plan_path=task.paths(run_id,case_id);episode=case/"episode"
    pin(plan_path,plan_sha);pin(case/"declaration.json",plan_sha)
    plan=document(plan_path)
    fixed=task.fixed_contract(run_id,case_id)
    require(all(plan[k]==v for k,v in fixed.items()),"prospective recovery case fields changed")
    manifest=document(episode/"manifest.json");scope_manifest(manifest);verify_teacher_metadata(manifest)
    names=("manifest.json","route.json","states.jsonl","camera_frames.jsonl","control_receipts.jsonl",
           "actual_teacher_control.jsonl","actual_camera_pixels.jsonl","active_map_opendrive.xodr","moving_task_plan.json")
    for name in names:pin(episode/name,plan_sha if name=="moving_task_plan.json" else None)
    for name in ("owner_result.json","diagnostic_report.json","derived_body/summary.json","derived_body/all_native_bodies_and_sweeps.jsonl.gz"):pin(case/name)
    owner,diagnostic,body=[document(case/n) for n in ("owner_result.json","diagnostic_report.json","derived_body/summary.json")]
    require(owner["declaration_sha256"]==plan_sha and not owner["cleanup_errors"] and owner["source_hashes_match"] is True
        and owner["external_source_hashes_match"] is True and diagnostic["actual_teacher_capture_passed"] is True
        and diagnostic["training_data_approved"] is False and diagnostic["future_labels_qualified"] is False,
        "owned real teacher capture/closure QA did not pass")
    require(body["all_native_body_and_swept_clearance_passed"] is True and body["full_body_denominator"]==NATIVE
        and body["full_sweep_denominator"]==NATIVE-1 and body["ten_centimeter_inset_preserved"] is True,"whole capture body/sweep QA failed")
    for name,expected in body["input_sha256"].items():pin(Path(name),expected)
    for name,expected in plan["source_sha256"].items():pin(ROOT/name,expected);pin(case/"source"/name,expected)
    external_root=Path("/home/a/carla-autoware-universe/CARLA_0.9.15")
    for section in ("external_source_sha256","asset_sha256"):
        for name,expected in plan[section].items():
            source=Path(name);pin(source,expected)
            root=ROOT if source.is_relative_to(ROOT) else external_root
            pin(case/("source" if root==ROOT else "source_external")/source.relative_to(root),expected)
    states,cameras,receipts,commands=load_raw_journals(episode,names[2:6])
    require(len(states)==NATIVE and len(cameras)==CAMERA,"full965native/483camera denominator changed")
    route=document(episode/"route.json");require(sha(episode/"route.json")==task.ROUTE_SHA,"unchanged causal route bytes required")
    replay=task.reconstruct(plan,states,cameras,route["route"])
    require(replay.result(tail_states=130,goal_reached=False)==manifest["result"]["moving_task"] and replay.failure is None
        and len(replay.selected)==COUNT,"all302 fixed selected anchors must replay exactly")
    selected=[dict(row,camera_index=next(i for i,c in enumerate(cameras) if c["frame"]==row["frame"])) for row in replay.selected]
    with gzip.open(case/"derived_body/all_native_bodies_and_sweeps.jsonl.gz","rt") as stream:geometry=[json.loads(line) for line in stream]
    require(len(geometry)==len(states) and all(g["frame"]==s["frame"] for g,s in zip(geometry,states)),"body rows do not cover exact raw native frames")
    timeline=raw.measured_timeline(states)
    pixels,image_rows,anchors=raw.camera_audit(episode,manifest,cameras,timeline)
    require(pixels["all_pixel_and_metadata_checks_clear"] is True and len(image_rows)==CAMERA*6,"full six-camera pixel/rig QA failed")
    for row in image_rows:pin(episode/row["path"],row["sha256"])
    transport=ack.analyze_transport(states,receipts,manifest["capture_contract"]["control_transport"]["bootstrap_observation"],wheelbase=2.85)
    require(transport["status"]=="PASS" and ack.analyze_camera_alignment(states,cameras)["status"]=="PASS","independent native ACK/camera binding failed")
    initialization=initialization_audit(manifest,states,receipts)
    controls=teacher_audit(commands,receipts,states)
    summary,futures=futures_math.geometry_futures(timeline,cameras,route,anchors,case_id)
    windows,defects=window_checks(selected,cameras,anchors,states,geometry,futures)
    require(len(windows)==COUNT,"fixed selected302 denominator changed; no quality filtering permitted")
    clear=not any(r["failure_reasons"] for r in windows) and not defects["protected"]
    require(all(cameras[row["camera_index"]]["capture_phase"]=="driving" for row in selected),"non-driving selected anchor")
    span=selected[-1]["timestamp_ns"]-selected[0]["timestamp_ns"]
    require(span>=30_000_000_000,"frozen Common10 minimum30s selected span must not be relaxed")
    native_result=manifest["result"]
    return dict(case_id=case_id,native_episode=str(episode),case_plan_sha256=plan_sha,native_manifest_sha256=sha(episode/"manifest.json"),
        native_state_sha256=sha(episode/"states.jsonl"),native_camera_sha256=sha(episode/"camera_frames.jsonl"),
        source_route_sha256=sha(episode/"route.json"),dataset_split_intent=plan["split"],seed=plan["seed"],
        signed_lateral_offset_ros_m=plan["signed_lateral_offset_ros_m"],signed_yaw_offset_ros_deg=plan["signed_yaw_offset_ros_deg"],
        selected_camera_frames=[r["frame"] for r in selected],selected_anchors=selected,selected_camera_anchors=COUNT,
        raw_camera_anchors=CAMERA,raw_native_states=NATIVE,raw_image_count=CAMERA*6,selected_span_ns=span,
        route_identity=dict(route_id="town07_signed_recovery_"+case_id,route_sha256=task.ROUTE_SHA,
                            route_group_independence="NOT_CERTIFIED: same existing TRAIN corridor; HOLDOUT is unseen seed/initialization only"),
        future_quality_summary=summary,window_check_counts=dict(Counter(reason for row in windows for reason in row["failure_reasons"])),
        all_fixed_anchor_necessary_checks_clear=clear,whole_body_and_snapshot_sweep_clearance_passed=True,
        all_fixed_anchor_window_checks=windows,protected_native_failures=defects["protected"],
        control_replay=controls,initialization_independent_audit=initialization,outside_protected_native_failures=defects["outside_protected"],
        whole_recording_goal_stop_quality=native_result.get("whole_recording_goal_stop_quality_diagnostic"),
        original_goal_reached=False,original_native_pending_metadata_unchanged=True)


def evidence(run_id,batch_sha,group):
    require(run_id=="v4","only prospectively declared new V4 rolling data may be qualified")
    _,batch_path=task.paths(run_id)
    require(sha(batch_path)==independent.hash_value(batch_sha),"externally pinned all-eleven batch declaration differs")
    batch=document(batch_path)
    require(batch["schema"]==cohort.SCHEMA and batch["run_id"]=="v4"
        and [(r["case_id"],r["split"]) for r in batch["cases"]]==[(r["case_id"],r["split"]) for r in cohort.CASES] and batch["train_case_count"]==9
        and batch["holdout_case_count"]==2 and batch["automatic_retry"] is False
        and batch["holdout_training_or_checkpoint_selection_allowed"] is False,"eleven prospective split/case identities differ")
    inputs={}
    def pin(path,expected=None):
        path=safe(path,external=True);value=sha(path)
        require(expected is None or value==expected,"pinned source/artifact changed: "+str(path))
        require(str(path) not in inputs or inputs[str(path)]==value,"evidence changed between reads")
        inputs[str(path)]=value
    pin(batch_path,batch_sha)
    for row in batch["cases"]:
        _,expected_plan=task.paths(run_id,row["case_id"])
        require(safe(row["declaration_path"])==expected_plan,"batch case declaration escaped exact new V4 path")
        pin(expected_plan,row["declaration_sha256"])
    # HH_261001 - The whole prospective eleven-case cohort must pass; no favorable split or case subset is admitted.
    all_cases=[case_evidence(run_id,row["case_id"],row["declaration_sha256"],pin) for row in batch["cases"]]
    require(all(case["all_fixed_anchor_necessary_checks_clear"] is True for case in all_cases),
        "a declared group window/protected native check failed; no quality-conditioned case or anchor exclusion allowed")
    cases=[case for case in all_cases if case["case_id"] in identities(group)]
    required=set(ROOT/name for name in SOURCE_PATHS)
    required.update(source_closure.imported_source_closure((Path(__file__).resolve(),ROOT/SOURCE_PATHS[1])))
    source={str(path.relative_to(ROOT)):sha(path) for path in sorted(required)}
    for name,value in inputs.items():require(sha(name)==value,"input changed during qualification")
    return dict(schema=SCHEMA,status="QUALIFIED_RESEARCH_TRAIN" if group=="train" else "QUALIFIED_RESERVED_HOLDOUT_EVALUATION_ONLY",
        run_id=run_id,group=group,batch_declaration_sha256=batch_sha,prepared_output=str(prepared_path(run_id,group)),dataset_id=dataset_id(run_id,group),
        cases=cases,expected_case_ids=list(identities(group)),episode_count=len(cases),selected_camera_anchors=COUNT*len(cases),
        all_eleven_fixed_cohort_cases_qualified=True,full_cohort_case_count=len(all_cases),
        full_cohort_fixed_selected_anchors=COUNT*len(all_cases),
        full_cohort_case_provenance=[{key:case[key] for key in ("case_id","native_episode","case_plan_sha256","native_manifest_sha256",
            "native_state_sha256","native_camera_sha256","source_route_sha256","dataset_split_intent","seed",
            "signed_lateral_offset_ros_m","signed_yaw_offset_ros_deg","selected_camera_anchors","selected_span_ns",
            "all_fixed_anchor_necessary_checks_clear")} for case in all_cases],
        required_true_future_points=64,required_true_future_seconds=6.4,original_Common10_minimum_selected_span_seconds=30,
        source_sha256=source,input_sha256=inputs,research_training_authorized=group=="train",**ROLLING_SCOPE,
        research_holdout_evaluation_authorized=group=="holdout",holdout_training_or_checkpoint_selection_allowed=False,
        raw_native_denials_overridden=False,original_pending_metadata_unchanged=True,window_filtering_performed=False,
        thresholds_relaxed=False,labels_or_pixels_transformed=False,synthetic_history_created=False,training_run_performed=False,
        vehicle_control_approved=False,model_promoted=False,model_or_vehicle_release_approved=False,
        authority="New byte-pinned technical research qualification within requested model training; no external human approval asserted.",
        notice="Overlapping fixed windows, not independent maneuvers; real partial prepared boundary history must be masked. HOLDOUT never fits or selects a checkpoint.")


def recheck(qualification):
    require(sha(qualification["receipt_path"])==qualification["receipt_sha256"],"authenticated qualification receipt changed")
    for name,value in qualification["document"]["input_sha256"].items():require(sha(name)==value,"qualification input changed")
    for name,value in qualification["document"]["source_sha256"].items():require(sha(ROOT/name)==value,"qualification/converter source changed")


def entry_for(spec,qualification):
    entries=[c for c in qualification["document"]["cases"] if safe(c["native_episode"])==safe(spec.path)]
    require(len(entries)==1,"native episode not in authenticated fixed group")
    entry=entries[0];expected="train" if entry["dataset_split_intent"]=="train" else "val"
    require(spec.split==expected,"HOLDOUT cannot become TRAIN")
    return entry


def authorize(args):
    require(getattr(args,"execute_reviewed_recovery_conversion",False) is True,"explicit reviewed recovery conversion flag required")
    path=safe(args.moving_qualification);expected=independent.hash_value(args.moving_qualification_sha256)
    require(sha(path)==expected,"caller qualification SHA differs")
    value=document(path)
    require(value==evidence(value["run_id"],value["batch_declaration_sha256"],value["group"]),"receipt does not rederive all actual research evidence")
    require(args.dataset_id==value["dataset_id"] and safe(args.output)==safe(value["prepared_output"])
        and args.image_mode=="copy" and args.validate_mode=="planning","fixed fresh copy/planning research output required")
    require(len(args.episode)==len(value["cases"]) and [safe(s.path) for s in args.episode]==[safe(c["native_episode"]) for c in value["cases"]],"fixed ordered whole group required, no best-case selection")
    qualification=dict(receipt_path=str(path),receipt_sha256=expected,document=value,native_objects=[])
    for spec in args.episode:entry_for(spec,qualification)
    copied=sum((Path(case["native_episode"])/f"images/{name}/{frame:08d}.jpg").stat().st_size
        for case in value["cases"] for frame in case["selected_camera_frames"] for name in contract.CAMERA_ORDER)
    require(shutil.disk_usage(ROOT).free>=5*1024**3+copied+512*1024**2,
        "selected independent copies plus fixed5GiB/512MiB reserve required before output")
    recheck(qualification)
    return qualification


def verify_native_scope(spec,manifest,qualification):
    scope_manifest(manifest);entry=entry_for(spec,qualification)
    require(entry["native_manifest_sha256"]==sha(spec.path/"manifest.json"),"native manifest identity differs")


def native_route_id(spec,manifest,route,qualification):
    verify_native_scope(spec,manifest,qualification);entry=entry_for(spec,qualification)
    require(sha(spec.path/"route.json")==entry["route_identity"]["route_sha256"],"native causal route changed")
    return entry["route_identity"]["route_id"]


def signed_route_state(route,state):
    # HH_261001 - Diagnostic current pose only; this HD/corridor truth is never inserted into model features.
    closest=None
    for index,(a,b) in enumerate(zip(route["route"],route["route"][1:])):
        dx,dy=b["x"]-a["x"],b["y"]-a["y"];length=math.hypot(dx,dy)
        if length<=1e-9:continue
        ratio=max(0.,min(1.,((state["x"]-a["x"])*dx+(state["y"]-a["y"])*dy)/length**2))
        distance=math.hypot(state["x"]-a["x"]-ratio*dx,state["y"]-a["y"]-ratio*dy)
        heading=math.atan2(dy,dx)
        # HH_261001 - Match frozen canonical navigation's distance/heading/index tie rule; no future route state is read.
        key=(round(distance**2,12),-math.cos(state["yaw"]-heading),index)
        if closest is None or key<closest[0]:
            closest=(key,(dx*(state["y"]-a["y"])-dy*(state["x"]-a["x"]))/length,heading)
    require(closest is not None,"current signed diagnostic lacks a real route segment")
    delta=state["yaw"]-closest[2]
    return closest[1],math.atan2(math.sin(delta),math.cos(delta))


def validate_prepared(prepared,native,qualification,native_report):
    """HH_261001 - Independently reconstruct all copied pixels, causal inputs and original64future labels per declared case."""
    from scripts.e2e import audit_portable_target_feasibility as targets
    from scripts.e2e import train_temporal_v2_low_speed_joint_20261001 as joint_math
    from portable_e2e import temporal_dataset_v2 as temporal
    recheck(qualification)
    root=safe(prepared);value=qualification["document"];group=value["group"]
    split="train" if group=="train" else "val"
    loaded=dataset.load_training_examples(root,split=split,mode="planning",check_image_hashes=True)
    expected_count=COUNT*len(value["cases"])
    require(len(loaded.examples)==native_report["sample_count"]==expected_count
        and native_report["episode_count"]==len(value["cases"]),"whole fixed TRAIN/HOLDOUT prepared denominator differs")
    natives=qualification["native_objects"]
    require(len(natives)==len(value["cases"]) and len({n.episode_id for n in natives})==len(natives),"per-case native reader census incomplete")
    by_id={n.episode_id:n for n in natives};manifest=document(root/"dataset.json")
    require(len(manifest["episodes"])==len(natives),"prepared episode list omitted a case")
    all_audits=[];signed=[];case_records=[];contract_data=contract.load_contract()
    for reference in manifest["episodes"]:
        path=root/reference["manifest"];episode=document(path);item=by_id[episode["episode_id"]]
        entry=entry_for(item.spec,qualification);folder=path.parent
        require(episode["split"]==split and episode["source_provenance"]["adapter_id"]==ADAPTER_ID,"prepared episode adapter or reserved split differs")
        collection=document(folder/"collection_config.json")
        require(collection["adapter_id"]==ADAPTER_ID and collection["native_capture_contract"]==item.manifest["capture_contract"]
            and collection["native_runtime"]==item.manifest["runtime"] and collection["native_result"]==item.manifest["result"],"native metadata was normalized or rewritten")
        for name,expected in (("native_manifest.json",entry["native_manifest_sha256"]),("native_route.json",entry["source_route_sha256"]),
                              ("moving_qualification.json",qualification["receipt_sha256"])):
            require(sha(folder/name)==expected,"immutable native/receipt copy differs")
        source=document(folder/"source_manifest.json")
        require(len(source["camera_frames"])==CAMERA*6,"unselected original context provenance omitted")
        for image in source["camera_frames"]:
            require(image["source_payload_sha256"]==value["input_sha256"][str(item.spec.path/image["source_uri"])],"raw context/source pixels differ")
        samples=adapter._jsonl(folder/"samples.jsonl");require(len(samples)==COUNT,"selected case anchor count changed")
        timeline=adapter._state_timeline(item);state_by_stamp=dict(timeline);stamps=list(state_by_stamp)
        tail=next(t for t,s in timeline if s["capture_phase"]=="stationary_tail")
        polyline=[]
        for point in item.route["route"]:
            xy=[float(point["x"]),float(point["y"])]
            if not polyline or math.dist(polyline[-1],xy)>1e-9:polyline.append(xy)
        require(document(folder/"route_geometry.json")==dict(route_id=item.route_id,frame_id="map",polyline_m=polyline),"causal route geometry changed")
        by_frame={r["frame"]:r for r in item.camera_frames};copied=[]
        for index,(sample,anchor) in enumerate(zip(samples,entry["selected_anchors"])):
            require(sample["sequence_index"]==index and sample["anchor_timestamp_ns"]==anchor["timestamp_ns"],"selected original anchor timestamps/order changed")
            independent.independently_expected(item,sample,state_by_stamp,stamps,polyline,episode["route_geometry_sha256"],contract_data)
            for image,camera_name in zip(sample["camera_bundle"],contract.CAMERA_ORDER):
                old=item.spec.path/by_frame[anchor["frame"]]["images"][camera_name];new=folder/image["path"]
                require(image["name"]==camera_name and sha(old)==sha(new)==image["sha256"]
                    and old.stat().st_ino!=new.stat().st_ino and new.stat().st_nlink==1,"prepared pixels are not independent identical copies")
                copied.append(new)
            audit=targets.audit_sample(sample,route_length_m=item.route["route_length_m"],episode_start_ns=episode["start_timestamp_ns"],
                                       episode_end_ns=episode["end_timestamp_ns"],tail_start_ns=tail)
            audit["episode_id"]=item.episode_id;all_audits.append(audit)
            state=state_by_stamp[sample["ego"]["timestamp_ns"]];cte,yaw=signed_route_state(item.route,state)
            signed.append(dict(case_id=entry["case_id"],episode_id=item.episode_id,sample_id=sample["sample_id"],token=sample["sample_id"],
                frame=anchor["frame"],timestamp_ns=anchor["timestamp_ns"],actual_measured_rear_base_signed_CTE_m=cte,
                actual_measured_rear_base_CTE_m=abs(cte),actual_yaw_route_heading_error_rad=yaw,
                declared_signed_lateral_offset_ros_m=entry["signed_lateral_offset_ros_m"],declared_signed_yaw_offset_ros_deg=entry["signed_yaw_offset_ros_deg"],
                source_native_manifest_sha256=entry["native_manifest_sha256"],diagnostic_only_not_model_input=True))
        require(len(copied)==COUNT*6 and set(copied)=={p for p in (folder/"images").rglob("*") if p.is_file()},"selected pixel copy denominator differs")
        require(episode["capture_accounting"]["ineligible_anchor_reasons"]=={"moving_task_fixed_window_context_only":CAMERA-COUNT},"raw context membership changed")
        case_records.append(dict(case_id=entry["case_id"],episode_id=item.episode_id,manifest=reference["manifest"],
            native_episode=entry["native_episode"],native_manifest_sha256=entry["native_manifest_sha256"],
            source_route_sha256=entry["source_route_sha256"],case_plan_sha256=entry["case_plan_sha256"],
            qualification_receipt_sha256=qualification["receipt_sha256"],source_camera_sha256=entry["native_camera_sha256"],
            source_state_sha256=entry["native_state_sha256"],selected_examples=COUNT,split=split,
            signed_lateral_offset_ros_m=entry["signed_lateral_offset_ros_m"],signed_yaw_offset_ros_deg=entry["signed_yaw_offset_ros_deg"]))
    measured=targets.summarize_samples(all_audits,64)
    require(measured["valid_step_count"]==expected_count*64 and measured["invalid_step_count"]==0
        and all(m["violating_step_count"]==0 for m in measured["metrics"].values()),"unchanged nine prepared future bounds failed")
    # HH_261001 - Current speed support is measured hypot(vx,vy), never clipped, and no future distance becomes navigation.
    for example in loaded.examples:
        v0=math.hypot(example.features[1],example.features[2])
        require(0<v0<=2.5 and example.valid_point_count==64 and all(0<v<=2.5 for v in example.target_speed_mps),"actual current/future speed outside original low-speed support")
    meta=temporal.load_common10_temporal_metadata(root,split=split,expected_tokens=tuple(e.token for e in loaded.examples))
    view=temporal.TemporalCommon10TorchDataset(loaded.examples,raw_metadata=meta,split=split,verify_image_sha256=True)
    supports=[];measured_speeds=[]
    for example in view.examples:
        route_xy,route_mask=view._route(example)
        support=float(joint_math.causal_route_support(route_xy[None],route_mask[None]))
        require(support>=joint_math.PathTimeDecoderV2().config.go_active_distance_m,
                "causal navigation lacks original18m GO support; expert futures cannot supply support")
        supports.append(support);measured_speeds.append(math.hypot(example.features[1],example.features[2]))
    boundaries={}
    for index,example in enumerate(view.examples):
        if example.episode_id not in boundaries:
            require(example.sequence_index==0,"episode boundary must start at selected sequence0")
            masks=[view[index+i]["time_valid"].tolist() for i in range(9)]
            require(sum(masks[0])==1 and all(masks[8]),"prepared partial history must remain truly masked")
            boundaries[example.episode_id]=masks
    full_camera=sum(all(view._past_index(e,offset) is not None for offset in temporal.TIME_OFFSETS_NS) for e in view.examples)
    full_ego=sum(bool(view._ego_history(e)[1].all()) for e in view.examples)
    require(full_camera==(COUNT-8)*len(natives) and full_ego==(COUNT-9)*len(natives),"prepared honest history counts differ")
    signed_path=root/"signed_current_state_audit.jsonl"
    adapter._write_jsonl(signed_path,signed)
    recheck(qualification)
    return dict(schema=RESULT_SCHEMA,status="QUALIFIED_RESEARCH_TRAIN_COMMON10" if group=="train" else "QUALIFIED_HOLDOUT_COMMON10_EVALUATION_ONLY",
        dataset_manifest_sha256=sha(root/"dataset.json"),validation_report_sha256=sha(root/"validation_report.json"),
        qualification_receipt_sha256=qualification["receipt_sha256"],
        train_loader_fingerprint_sha256=loaded.fingerprint_sha256 if split=="train" else None,
        val_loader_fingerprint_sha256=loaded.fingerprint_sha256 if split=="val" else None,
        temporal_dataset_fingerprint_sha256=view.fingerprint_sha256,episode_count=len(natives),
        all_eleven_fixed_cohort_cases_qualified=value["all_eleven_fixed_cohort_cases_qualified"],
        full_cohort_case_count=value["full_cohort_case_count"],
        full_cohort_fixed_selected_anchors=value["full_cohort_fixed_selected_anchors"],
        full_cohort_case_provenance=value["full_cohort_case_provenance"],batch_declaration_sha256=value["batch_declaration_sha256"],
        train_examples=expected_count if split=="train" else 0,val_examples=expected_count if split=="val" else 0,test_examples=0,
        research_training_authorized=group=="train",research_holdout_evaluation_authorized=group=="holdout",**ROLLING_SCOPE,
        holdout_training_or_checkpoint_selection_allowed=False,exact_native_ego_navigation_and_target_reconstruction=True,
        true_future_points=expected_count*64,original_Common10_minimum_selected_span_seconds=30,
        per_episode_native_qualification=case_records,signed_current_state_audit_sha256=sha(signed_path),
        signed_current_state_audit_schema="diagnostic_only_actual_signed_current_state.20261001.v1",
        per_episode_first_nine_temporal_history_masks=boundaries,full_five_camera_history_examples=full_camera,
        full_ten_ego_history_examples=full_ego,unselected_raw_context_preserved_in_source_manifest=True,
        raw_native_denials_overridden=False,original_pending_metadata_unchanged=True,synthetic_history_created=False,
        labels_clipped=False,labels_or_pixels_transformed=False,future_labels_used_as_model_input=False,
        actual_hypot_current_speed_no_clipping=True,causal_navigation_support_not_future_distance=True,
        causal_navigation_support_m=dict(minimum=min(supports),maximum=max(supports)),
        actual_current_planar_speed_mps=dict(minimum=min(measured_speeds),maximum=max(measured_speeds)),
        unchanged_target_audit=measured,vehicle_control_approved=False,model_promoted=False,model_or_vehicle_release_approved=False,
        notice="Fixed recovery302 overlapping windows per episode; partial leading prepared histories masked, same corridor not independent-map approval.")


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument("--run-id",default="v4",choices=("v4",))
    parser.add_argument("--batch-plan-sha256")
    parser.add_argument("--group",choices=("train","holdout"))
    parser.add_argument("--output",type=Path)
    parser.add_argument("--inspect-case",choices=tuple(row["case_id"] for row in cohort.CASES))
    parser.add_argument("--case-plan-sha256")
    args=parser.parse_args(argv)
    if args.inspect_case is not None:
        require(args.case_plan_sha256 is not None and args.batch_plan_sha256 is None and args.group is None and args.output is None,
                "debug case requires only case SHA; it never writes or grants group authority")
        print(json.dumps(inspect_case(args.run_id,args.inspect_case,independent.hash_value(args.case_plan_sha256)),allow_nan=False))
        return 0
    require(args.case_plan_sha256 is None and args.batch_plan_sha256 is not None and args.group is not None and args.output is not None,
            "qualification requires paired batch/group/fresh output, not debug-case arguments")
    output=safe(args.output)
    batch,_=task.paths(args.run_id)
    require(not output.exists() and not output.is_relative_to(batch) and not output.is_relative_to(prepared_path(args.run_id,args.group)),"fresh external receipt output required")
    value=evidence(args.run_id,args.batch_plan_sha256,args.group)
    output.parent.mkdir(parents=True,exist_ok=True)
    with output.open("x") as stream:json.dump(value,stream,indent=2,allow_nan=False);stream.write("\n")
    print(json.dumps(dict(status=value["status"],receipt_path=str(output),receipt_sha256=sha(output),selected_anchors=value["selected_camera_anchors"])))
    return 0


if __name__=="__main__":raise SystemExit(main())
