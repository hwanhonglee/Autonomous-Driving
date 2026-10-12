"""HH_261001 - Prospectively own three 45-second final3000 headroom/CPU-finite trials; never launch from contract mode."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import gzip
import json
import math
from pathlib import Path
import re
import shutil
import sys

import numpy as np

from portable_e2e.temporal_cpu_finite_validation_v2_20261001 import upgrade_cpu_finite_model
from . import collect_temporal_v2_headroom_fast45_research_20261001 as live
from . import research_temporal_v2_owned_camera_town07_20261001 as base
from . import audit_simulation_research_body_lane_20261001 as geometry
from .run_owned_ah_town07_research_v2_20261001 import BUDGET_ERROR


ROOT, CARLA_ROOT, ROUTE = base.ROOT, base.CARLA_ROOT, base.ROUTE
ASSETS, CHECKPOINT_SHA, independent_audit = base.ASSETS, base.CHECKPOINT_SHA, base.independent_audit
MODULE = "scripts.e2e.research_temporal_v2_headroom_fast45_town07_20261001"
SCHEMA = "portable_e2e.temporal_v2_headroom_fast45_three_trials.20261001.v1"
FROZEN_REVISIONS = dict(base.FROZEN_GENERIC, **{
    "learning/portable_e2e/temporal_owned_camera_history_v2_20261001.py": "bf5f24322b684aa437bb3efe9944bfd86de5c18da7b08094e53d7a25269c2f08",
    "tools/e2e/collect_temporal_v2_owned_camera_research_20261001.py": "dc31fb87df83fc908324cc0f55d28ab0c3a5d26f7f9fdf185d9bd690a61b50ad",
    "learning/portable_e2e/longitudinal_pi_headroom_research_20261001.py": "fd984bdd6473e4e350a62712451be02768e6a730ad8b6dd033c79ca3af9a1ec5",
    "learning/portable_e2e/temporal_headroom_camera_research_v2_20261001.py": "7c19f3ee11f584f62b2eeb17e76e6c426c90efca5312506d5b1688203129a124",
    "tools/e2e/collect_temporal_v2_headroom_research_20261001.py": "5f4ccabf2e96bfdf2e50ad300a196433d43aa30730bb3aa7f90fec6666654ba8",
    "learning/portable_e2e/temporal_cpu_finite_validation_v2_20261001.py": "ecc6f7af5e8ee8dcbbc3a656c50e7de474f7a7bfa1e1eca630af40faa59e16c5",
    "tools/e2e/audit_simulation_research_body_lane_20261001.py": "6e932dcb39fa40e664218d3d1dddf20dd524f54e99ca55d98793c76cce8c03ac",
    "tools/e2e/audit_future_actor_lane_containment.py": "75d73449c8ffb17e27d1ddb419bd064a58ab052043b2a7b0ada69de8a65fa57d",
    "tools/e2e/research_temporal_v2_owned_camera_town07_20261001.py": "fbeb39844f07a4e1928665253ded08038fa0cc27f705d2c6fad2728d934d800b",
})
_ORIGINAL = {name:getattr(base,name) for name in (
    "paths", "capture_argv", "fixed_contract", "runtime_sources", "load_npz", "disk_check")}
_trial = None
_active = False
require, digest, read_json, write_new = base.require, base.digest, base.read_json, base.write_new
jsonl, directory_bytes = base.jsonl, base.directory_bytes


def batch_paths(run_id):
    require(isinstance(run_id,str) and re.fullmatch(r"v[1-9][0-9]*",run_id), "safe explicit batch version required")
    output=ROOT/("storage/experiments/validation/2026-10-01/temporal_v2_headroom_fast45_town07_"+run_id)
    return output,output.with_name(output.name+"_plan.json")


def paths(run_id):
    require(type(_trial) is int and 0<=_trial<3,"explicit prospective trial index required")
    output,_=batch_paths(run_id)
    return output/("trial_%02d"%_trial),output.with_name(output.name+("_trial_%02d_plan.json"%_trial))


def capture_argv(run_id,longitudinal_pi,declaration_sha=base.PLAN_TOKEN):
    require(longitudinal_pi is True,"headroom trials require explicit PI")
    argv=_ORIGINAL["capture_argv"](run_id,True,declaration_sha)
    index=argv.index("--max-duration-sec")+1
    require(argv[index]=="20","frozen base capture argument changed")
    argv[index]="45"
    return argv


def fixed_contract(run_id,longitudinal_pi):
    require(longitudinal_pi is True,"explicit PI required")
    result=_ORIGINAL["fixed_contract"](run_id,True)
    result.update(schema=SCHEMA,trial_index=_trial,prospective_trial_count=3,
        all_three_trials_required=True,automatic_retry=False,outcome_selected_retry=False,
        total_capture_seconds=45,controller_target_cap_mps=2.40,
        explicit_controller_calibration_intervention=True,
        original_9_kmh_target_tracking_claimed=False,model_only_performance_claimed=False,
        original_decoder_gate_tracker_math_unchanged=False,
        original_NN_decoder_selected_trajectory_math_unchanged=True,
        tracker_intervention="only PI target=min(original normal-tracker target,2.40); same gains/state/update/steer/stop",
        CPU_finite_guard_revision="CPUfloat32 NumPy predicate; both complete original validations retained",
        all_original_shape_dtype_mask_pose_cadence_state_and_input_SHA_guards_preserved=True,
        reserved_complete_three_trial_batch_bytes=3*base.ARM_RESERVE,
        capture_argv=capture_argv(run_id,True),
        worker_command=[sys.executable,"-B",str(ROOT/"tools/workspace_python.py"),"-m",MODULE,"--",
            "--mode","worker","--run-id",run_id,"--trial",str(_trial),"--trial-plan-sha256",base.PLAN_TOKEN])
    result["precursor_motion_screen"]["net_learned_progress_minimum_m"]=90.
    result["precursor_motion_screen"]["original_model_stop_target_fresh_proposals_maximum"]=0
    result["precursor_motion_screen"]["original_model_stop_target_threshold_mps"]=.05
    result["precursor_motion_screen"]["applied_original_model_stop_target_fresh_and_held_commands_maximum"]=0
    result["precursor_motion_screen"]["post_handoff_base_planar_speed_maximum_mps"]=2.5
    result["precursor_motion_screen"]["post_handoff_actor_world_planar_speed_maximum_mps"]=2.7
    result["sensor_and_ACK_screen"].update(native_state_count=900,camera_bundle_count=450)
    result["body_clearance_screen"]=dict(all_native_boxes=900,all_snapshot_linear_sweeps=899,
        lane_inset_m=.1,frozen_geometry_thresholds_unchanged=True,
        outgoing_source_annotation_is_not_actuator_proof=True,physical_continuous_time_proven=False)
    result["screen_interpretation"]=dict(basic_driving="motion+body+source+ACK+native/camera chain+unchanged100ms policy",
        observed_bundle_to_ACK_wall_100ms="separate all-fresh-denominator outcome, never hidden by driving success",
        full_sensor_callback_to_ACK="unmeasured; never approved",full_feature_RTE01=False)
    result["execute_exit_status_interpretation"]="operational ownership+bounded capture only; not a driving or wall-latency PASS"
    return result


def runtime_sources():
    pins=_ORIGINAL["runtime_sources"]()
    sources=base.imported_source_closure((Path(__file__).resolve(),))
    pins.update({str(path.relative_to(ROOT)):digest(path) for path in sorted(sources)})
    require(all(pins.get(name)==sha for name,sha in FROZEN_REVISIONS.items()),"frozen reviewed revision changed")
    return pins


def load_model(*args,**kwargs):
    return upgrade_cpu_finite_model(_ORIGINAL["load_npz"](*args,**kwargs))


def disk_check(free=None):
    value=shutil.disk_usage(ROOT).free if free is None else free
    required=base.FLOOR+3*base.ARM_RESERVE+base.SHARED_RESERVE
    require(type(value) is int and value>=required,"projected full three-trial disk reserve insufficient")
    return dict(free_bytes=value,required_bytes=required,prospective_trials=3,
        episode_quota_bytes=base.EPISODE_QUOTA,sampled_exact_input_quota_bytes=base.SAMPLE_QUOTA)


def classify_bounded_capture(manifest,state_count,camera_count):
    capture=manifest.get("capture_contract",{})
    return manifest.get("status")=="failed" and manifest.get("error")==BUDGET_ERROR \
        and state_count==900 and camera_count==450 and capture.get("physics_hz")==20. \
        and capture.get("camera_hz")==10. and capture.get("maximum_total_duration_sec")==45.


@contextmanager
def scoped_trial(trial):
    """HH_261001 - Reuse the frozen owner lifecycle only through fully restored explicit version bindings."""
    global _trial,_active
    require(not _active and type(trial) is int and 0<=trial<3,"non-nested explicit trial scope required")
    previous_trial=_trial
    changes=dict(paths=paths,capture_argv=capture_argv,fixed_contract=fixed_contract,
        runtime_sources=runtime_sources,load_npz=load_model,disk_check=disk_check,
        live=live,SCHEMA=SCHEMA,MODULE=MODULE,diagnostic_report=diagnostic_report,
        classify_bounded_capture=classify_bounded_capture)
    saved={name:getattr(base,name) for name in changes}
    original_validation=live.frozen.validate_capture
    _active=True;_trial=trial
    try:
        for name,value in changes.items():setattr(base,name,value)
        live.frozen.validate_capture=live.validate_capture
        yield
    finally:
        live.frozen.validate_capture=original_validation
        for name,value in saved.items():setattr(base,name,value)
        _trial=previous_trial;_active=False


def verify_derived_source_closure(output,plan):
    """HH_261001 - Close the offline late-read interval against all declared live and archived code/API/assets."""
    for name,sha in plan["source_sha256"].items():
        source=ROOT/name
        require(source.resolve().is_relative_to(ROOT) and digest(source)==sha==digest(output/"source"/name),
            "post-body local runtime/archive source changed: "+name)
    for section in ("external_source_sha256","asset_sha256"):
        for name,sha in plan[section].items():
            source=Path(name)
            if source.is_relative_to(ROOT):archive=output/"source"/source.relative_to(ROOT)
            else:
                require(source.is_relative_to(CARLA_ROOT),"post-body source escaped declared roots")
                archive=output/"source_external"/source.relative_to(CARLA_ROOT)
            require(digest(source)==sha==digest(archive),"post-body external/API/asset archive changed: "+name)


def body_report(output,plan):
    """HH_261001 - Audit all archived native boxes/sweeps after cleanup; failed capture tails remain in the geometry."""
    episode=output/"episode.partial"
    if not episode.is_dir():episode=output/"episode"
    names=("manifest.json","route.json","states.jsonl","camera_frames.jsonl","control_receipts.jsonl",
           "learned_control.jsonl","wall_timing.jsonl","active_map_opendrive.xodr")
    pins={str(episode/name):digest(episode/name) for name in names}
    for name,sha in plan["source_sha256"].items():
        require(digest(ROOT/name)==sha==digest(output/"source"/name),"body audit runtime/archive source changed")
    manifest=read_json(episode/"manifest.json")
    states,receipts=jsonl(episode/"states.jsonl"),jsonl(episode/"control_receipts.jsonl")
    require(len(states)==900,"body screen requires all900 native observations")
    actors={row["actor_id"] for row in receipts};require(len(actors)==1,"one exact ACK actor required")
    require(digest(episode/"route.json")==ASSETS[ROUTE],"body archived route differs from predeclared asset")
    require(digest(episode/"active_map_opendrive.xodr")==ASSETS[base.XODR],"body archived live XODR differs")
    require(sys.version_info[:2]==(3,10),"pinned offline CARLA map API requires Python3.10")
    saved_path=list(sys.path)
    try:
        sys.path.insert(0,str(base.EGG))
        import carla
        require(str(carla.__file__).startswith(str(base.EGG)+"/"),"foreign offline CARLA API")
        rows,census=geometry.body_census(manifest,read_json(episode/"route.json"),states,
            (episode/"active_map_opendrive.xodr").read_bytes(),next(iter(actors)))
    finally:sys.path[:]=saved_path
    require(len(rows)==900,"all native body rows required")
    passed=all(value["status"]=="PASS" and value["assessed_count"]==(899 if key.startswith("previous_sweep") else 900)
               for key,value in census.items()) and len(census)==4
    require(all(digest(name)==sha for name,sha in pins.items()),"raw body inputs changed during audit")
    verify_derived_source_closure(output,plan)
    derived=output/"derived_body";derived.mkdir(exist_ok=False)
    with gzip.open(derived/"all900_native_boxes_and899_sweeps.jsonl.gz","xt") as stream:
        for row in rows:stream.write(json.dumps(row,allow_nan=False)+"\n")
    report=dict(status="PASS" if passed else "FAIL",body_and_swept_clearance_passed=passed,
        status_scope="all900 boxes/all899 bounded snapshot-linear sweeps with frozen10cm inset",
        body_geometry=census,input_sha256=pins,physical_continuous_time_proven=False,
        full_declared_source_external_API_asset_closure_rechecked_after_body=True,
        outgoing_source_annotation_is_not_actuator_proof=True)
    write_new(derived/"summary.json",report)
    return report

def diagnostic_core(output,plan,owner):
    """HH_261001 - Separate capture/ownership/sensor/ACK/wall metrics from unapproved route/driving features."""
    episode=output/"episode.partial"
    if not episode.is_dir(): episode=output/"episode"
    manifest=read_json(episode/"manifest.json") if (episode/"manifest.json").is_file() else {}
    states,cameras,commands=(jsonl(episode/name) for name in ("states.jsonl","camera_frames.jsonl","learned_control.jsonl"))
    timing=jsonl(episode/"wall_timing.jsonl")
    summary=live.frozen.wall_timing_summary(timing)
    fresh=[row for row in timing if row["learned_control_proposed"] and row["held_between_camera_frames"] is False]
    bundles=[row["observed_bundle"] for row in fresh if row["observed_bundle"]]
    unique={row["native_frame"]:row for row in bundles}
    observed=sorted(unique.values(),key=lambda row:row["native_frame"])
    wall_span=(observed[-1]["exact_bundle_delivery_monotonic_ns"]-observed[0]["exact_bundle_delivery_monotonic_ns"])/1.e9 if len(observed)>1 else None
    simulated_span=(observed[-1]["source_image_timestamp_simulation_ns"]-observed[0]["source_image_timestamp_simulation_ns"])/1.e9 if len(observed)>1 else None
    values=[row["exact_bundle_delivery_to_ACK_seconds"] for row in fresh if row["exact_bundle_delivery_to_ACK_seconds"] is not None]
    ownership=not owner["cleanup_errors"] and owner["source_hashes_match"] and owner["external_source_hashes_match"]
    native_chain=bool(states) and all(b["frame"]-a["frame"]==1 and abs(b["timestamp"]-a["timestamp"]-.05)<=1.e-6 for a,b in zip(states,states[1:]))
    camera_chain=bool(cameras) and all(b["frame"]-a["frame"]==2 and abs(b["timestamp"]-a["timestamp"]-.1)<=1.e-6
                                     for a,b in zip(cameras,cameras[1:]))
    native_by_frame={row["frame"]:row for row in states}
    exact_camera_match=bool(cameras) and all(row["frame"] in native_by_frame
        and abs(row["timestamp"]-native_by_frame[row["frame"]]["timestamp"])<=1.e-6
        and set(row.get("source_timestamps",{}))==set(live.frozen.collector.MODEL_CAMERA_ORDER)
        and all(abs(value-row["timestamp"])<=1.e-6 for value in row["source_timestamps"].values()) for row in cameras)
    receipts=jsonl(episode/"control_receipts.jsonl")
    try:
        wheelbase=manifest["coordinate_contract"]["wheelbase_m"]
        require(wheelbase==plan["recorded_base_link_wheelbase_m"],"recorded wheelbase differs from predeclared command")
        transport_audit=independent_audit.analyze_transport(states,receipts,
            manifest.get("capture_contract",{}).get("control_transport",{}).get("bootstrap_observation"),wheelbase=wheelbase)
    except Exception as error:
        transport_audit=dict(status="FAIL",error=type(error).__name__+": "+str(error))
    try: camera_audit=independent_audit.analyze_camera_alignment(states,cameras)
    except Exception as error: camera_audit=dict(status="FAIL",error=type(error).__name__+": "+str(error))
    independent_pass=transport_audit["status"]==camera_audit["status"]=="PASS"
    bounded=(classify_bounded_capture(manifest,len(states),len(cameras)) or manifest.get("status")=="complete") \
        and len(states)==900 and len(cameras)==450
    learned_ACK_sequences={row["ack_receipt_sequence"] for row in commands
                           if row.get("application",{}).get("learned_control_applied",False)}
    expected_fresh=[row for row in commands if row.get("learned_control_proposed",False)
                    and row.get("held_between_camera_frames") is False]
    fresh_timing_complete=bool(expected_fresh) and len(fresh)==len(expected_fresh) and len(values)==len(fresh)
    for command in expected_fresh:
        matching=[row for row in fresh if row["ACK_sequence"]==command["ack_receipt_sequence"]
                  and row["inference_frame"]==command["inference_frame"] and row["command_frame"]==command["command_frame"]]
        fresh_timing_complete &= len(matching)==1 and matching[0]["ACK_server_accepted"] is True \
            and matching[0]["source_checkpoint_sha256"]==CHECKPOINT_SHA \
            and matching[0]["stream_source_sha256"]==owner["declaration_sha256"]
    latency_pass=ownership and bounded and native_chain and camera_chain and exact_camera_match and independent_pass \
        and fresh_timing_complete and float(np.percentile(values,95))<=.1 and max(values)<=.1
    first_effect=next((index for index,row in enumerate(states)
        if row.get("control_transport",{}).get("expected_receipt_sequence") in learned_ACK_sequences),None)
    # HH_261001 - Retain every subsequent driving tick, including missing/nonlearned/emergency ticks, not selected successes.
    effects=states[first_effect:] if first_effect is not None else []
    complete_effect_interval=bool(effects) and all(row.get("capture_phase")=="driving"
        and row.get("control_transport",{}).get("status")=="PASS"
        and row["control_transport"]["expected_receipt_sequence"] in learned_ACK_sequences for row in effects)
    effect_sequences={row.get("control_transport",{}).get("expected_receipt_sequence") for row in effects}-{None}
    rejected=sum(row.get("control_source")=="emergency_brake" for row in commands)
    collector_progress=effects[-1]["route_progress_m"]-effects[0]["route_progress_m"] if len(effects)>1 else None
    progress=None;cte=[];projection_error=None
    try:
        archived_route=output/"source"/ROUTE.relative_to(ROOT)
        require(digest(archived_route)==ASSETS[ROUTE],"archived route geometry pin differs")
        projector=live.frozen.collector.RouteProjector(read_json(archived_route))
        native_projections=[];previous=0.
        for row in effects:
            pose=row["actor_snapshot_transform_carla"]
            projection=projector.project(float(pose["x"]),-float(pose["y"]),previous)
            previous=projection.progress_m
            native_projections.append(projection)
        progress=native_projections[-1].progress_m-native_projections[0].progress_m if len(native_projections)>1 else None
        cte=[abs(item.cross_track_error_m) for item in native_projections]
    except Exception as error:
        projection_error=type(error).__name__+": "+str(error)
    dwell=longest_dwell=0.
    for row in effects:
        velocity=row["world_velocity_carla"]
        if float(np.hypot(velocity[0],velocity[1]))<=.05: dwell+=.05
        else: dwell=0.
        longest_dwell=max(longest_dwell,dwell)
    collisions=sum(bool(row.get("collision")) for row in states)
    invasions=sum(bool(row.get("lane_invasion")) for row in states)
    motion_core=ownership and bounded and independent_pass and complete_effect_interval \
        and progress is not None and progress>=90. and longest_dwell<1. \
        and float(np.percentile(cte,95))<=.35 and max(cte)<=.75 and collisions==invasions==rejected==0
    return dict(schema=SCHEMA+".diagnostic_report",ownership_passed=ownership,bounded_capture_complete=bounded,
        original_owner=owner,original_capture_status=manifest.get("status"),original_capture_error=manifest.get("error"),
        source_checkpoint_sha256=CHECKPOINT_SHA,checkpoint_selection=plan["checkpoint_selection"],candidate_count=1,
        native_state_count=len(states),camera_bundle_count=len(cameras),native_state_chain_passed=native_chain,
        camera_frame_chain_passed=camera_chain,exact_camera_native_timestamp_match=exact_camera_match,
        actual_ACK_receipt_count=len(receipts),
        independent_transport_audit=transport_audit,independent_camera_alignment_audit=camera_audit,
        failed_ACK_receipt_count=sum(row.get("status")!="ACKNOWLEDGED" for row in receipts),
        observed_next_native_control_PASS_count=sum(row.get("control_transport",{}).get("status")=="PASS" for row in states),
        learned_proposal_count=sum(row.get("learned_control_proposed",False) for row in commands),
        learned_ACK_count=sum(row.get("application",{}).get("learned_control_applied",False) for row in commands),
        rejected_learned_or_emergency_commands=rejected,
        learned_ACK_matched_native_effect_count=sum(row.get("control_transport",{}).get("status")=="PASS"
            and row["control_transport"]["expected_receipt_sequence"] in learned_ACK_sequences for row in effects) if independent_pass else 0,
        post_handoff_native_effect_interval_count=len(effects),
        complete_post_handoff_native_effect_interval=complete_effect_interval,
        learned_ACKs_without_observed_native_effect_sequences=sorted(learned_ACK_sequences-effect_sequences),
        learned_effect_progress_m=progress,learned_effect_CTE_p95_m=float(np.percentile(cte,95)) if cte else None,
        learned_effect_progress_reference="native CARLA actor API reference independently projected on archived ROS route; not verified COM",
        native_progress_projection_error=projection_error,
        collector_rear_base_route_progress_delta_m=collector_progress,
        learned_effect_CTE_absolute_maximum_m=max(cte) if cte else None,
        learned_effect_maximum_stationary_dwell_s=longest_dwell,
        stationary_speed_reference="native actor snapshot world XY norm; model base-v0 unchanged",
        native_observations_with_collision_events=collisions,native_observations_with_lane_invasion_events=invasions,
        predeclared_motion_core_screen_passed_before_body_swept_clearance=motion_core,
        wall_timing=summary,complete_fresh_learned_wall_receipt_denominator=fresh_timing_complete,
        predeclared_observed_bundle_to_ACK_wall_latency_screen_passed=latency_pass,
        observed_learned_camera_wall_span_seconds=wall_span,matching_simulation_span_seconds=simulated_span,
        measured_simulated_seconds_per_wall_second=simulated_span/wall_span if wall_span and simulated_span else None,
        sensor_callback_entry_observed=False,exposure_to_ACK_wall_latency_proven=False,ten_hz_end_to_end_proven=False,
        exact_normalized_tensor_archive_retained=False,episode_output_bytes=directory_bytes(episode),
        precursor_motion_screen="core computed from ACK-matched native effects; independent body/swept clearance still required",
        learned_driving_screen_passed=False,full_route_or_stop_approved=False,full_RTE01_approved=False,
        vehicle_control_approved=False,model_promoted=False,training_data_approved=False)


def diagnostic_report(output,plan,owner):
    """HH_261001 - Driving/100ms policy and observed-bundle-to-ACK wall latency are separate prospective outcomes."""
    try:
        result=diagnostic_core(output,plan,owner)
    except Exception as error:
        return dict(schema=SCHEMA+".diagnostic_report",status="FAIL_DIAGNOSTIC",
            error=type(error).__name__+": "+str(error),original_owner=owner,
            ownership_passed=False,bounded_capture_complete=False,learned_driving_screen_passed=False,
            full_RTE01_approved=False,vehicle_control_approved=False,model_promoted=False)
    episode=output/"episode.partial"
    if not episode.is_dir():episode=output/"episode"
    commands=jsonl(episode/"learned_control.jsonl")
    fresh=[row for row in commands if row.get("learned_control_proposed")
        and row.get("held_between_camera_frames") is False]
    applied=[row for row in commands if row.get("application",{}).get("learned_control_applied") is True]
    applied_stop=[row for row in applied if isinstance(row.get("control",{}).get("desired_speed_mps"),(int,float))
        and row["control"]["desired_speed_mps"]<=.05]
    stop_fresh=sum(row.get("held_between_camera_frames") is False for row in applied_stop)
    stop_held=sum(row.get("held_between_camera_frames") is True for row in applied_stop)
    states=jsonl(episode/"states.jsonl")
    learned_sequences={row["ack_receipt_sequence"] for row in applied}
    first_effect=next((index for index,row in enumerate(states)
        if row.get("control_transport",{}).get("expected_receipt_sequence") in learned_sequences),None)
    post=states[first_effect:] if first_effect is not None else []
    base_speeds=[math.hypot(row["vx"],row["vy"]) for row in post]
    actor_speeds=[math.hypot(*row["world_velocity_carla"][:2]) for row in post]
    speed_envelopes=bool(post) and all(math.isfinite(v) and v<=2.5 for v in base_speeds) \
        and all(math.isfinite(v) and v<=2.7 for v in actor_speeds)
    times=[row.get("total_seconds") for row in fresh]
    hard_policy=bool(fresh) and all(isinstance(value,(int,float)) and np.isfinite(value) and 0<=value<=.1 for value in times)
    controller_receipts=bool(fresh)
    original_stop_target_count=0
    for row in fresh:
        audit=row.get("longitudinal_pi_research",{})
        target=row.get("control",{}).get("desired_speed_mps")
        original_stop_target_count+=isinstance(target,(int,float)) and target<=.05
        controller_receipts &= isinstance(target,(int,float)) and audit.get("controller_target_cap_mps")==2.4 \
            and audit.get("original_model_target_mps")==target \
            and audit.get("governed_controller_target_mps")==min(target,2.4) \
            and audit.get("explicit_controller_calibration_intervention") is True \
            and audit.get("observed_speed_clipped") is False \
            and audit.get("observed_speed_mps")==row.get("measured_current_speed_mps") \
            and audit.get("one_fresh_camera_update_one_PI_integration") is True \
            and audit.get("proposed_normal_tracker_control")==row.get("control")
    body=dict(status="NOT_PROVEN",body_and_swept_clearance_passed=False)
    if result["ownership_passed"] and result["bounded_capture_complete"]:
        try:body=body_report(output,plan)
        except Exception as error:body=dict(status="FAIL",body_and_swept_clearance_passed=False,
            error=type(error).__name__+": "+str(error))
    driving=result["predeclared_motion_core_screen_passed_before_body_swept_clearance"] \
        and result["native_state_chain_passed"] and result["camera_frame_chain_passed"] \
        and result["exact_camera_native_timestamp_match"] and hard_policy and controller_receipts \
        and original_stop_target_count==0 and len(applied_stop)==0 and speed_envelopes \
        and body["body_and_swept_clearance_passed"]
    result.update(trial_index=_trial,status="PASS_BASIC_DRIVING_PRECURSOR" if driving else "FAIL_BASIC_DRIVING_PRECURSOR",
        prospective_trial_count=3,controller_target_cap_mps=2.4,explicit_controller_calibration_intervention=True,
        model_only_performance_claimed=False,original_9_kmh_target_tracking_claimed=False,
        original_NN_decoder_selected_trajectory_math_unchanged=True,
        CPU_finite_guard_revision="CPUfloat32 NumPy; original dualfullvalidation retained",
        predeclared_fresh_policy100ms_screen_passed=hard_policy,
        explicit_controller_intervention_receipt_denominator_passed=controller_receipts,
        original_model_stop_target_fresh_proposals=original_stop_target_count,
        applied_original_model_stop_target_fresh_commands=stop_fresh,
        applied_original_model_stop_target_held_commands=stop_held,
        applied_original_model_stop_target_total_commands=len(applied_stop),
        original_stop_branch_preserved_but_cruise_screen_rejects_any_stop_target=True,
        full_post_handoff_speed_envelopes_passed=speed_envelopes,
        full_post_handoff_base_planar_speed_maximum_mps=max(base_speeds) if base_speeds else None,
        full_post_handoff_actor_world_planar_speed_maximum_mps=max(actor_speeds) if actor_speeds else None,
        full_post_handoff_speed_denominator=len(post),measured_speeds_clipped=False,
        learned_driving_screen_passed=driving,independent_body_clearance=body,
        full_sensor_callback_to_ACK_100ms_approved=False,
        full_route_or_stop_approved=False,full_RTE01_approved=False,
        wall_100ms_failure_is_separate_and_retained=not result["predeclared_observed_bundle_to_ACK_wall_latency_screen_passed"])
    return result


def declare_batch(run_id):
    output,path=batch_paths(run_id)
    require(not output.exists() and not output.is_symlink() and not path.exists() and not path.is_symlink(),
            "fresh prospective three-trial batch required")
    disk_check()
    trial_paths=[]
    for trial in range(3):
        with scoped_trial(trial):
            plan=base.build_plan(run_id,True)
            _,trial_path=paths(run_id)
            trial_path.parent.mkdir(parents=True,exist_ok=True)
            write_new(trial_path,plan)
            trial_paths.append(dict(trial_index=trial,declaration_path=str(trial_path),declaration_sha256=digest(trial_path)))
    plan=dict(schema=SCHEMA+".batch",run_id=run_id,output=str(output),
        declared_at_utc=datetime.now(timezone.utc).isoformat(),prospective_trial_count=3,
        all_three_trials_required=True,attempts_per_trial=1,automatic_retry=False,outcome_selected_retry=False,
        trials=trial_paths,net_progress_minimum_m=90.,capture_seconds_per_trial=45,
        controller_target_cap_mps=2.4,explicit_controller_calibration_intervention=True,
        original_model_speed_envelope_mps=2.5,hard_policy_deadline_seconds=.1,
        wall_bundle_delivery_to_ACK_100ms_separate=True,fullsensor_callback_to_ACK_unmeasured=True,
        full_RTE01_approved=False,vehicle_control_approved=False,model_promoted=False,
        failed_attempts_retained=True)
    write_new(path,plan)
    return dict(batch_declaration_path=str(path),batch_declaration_sha256=digest(path),trials=trial_paths,
        simulator_started=False)


def verify_batch(run_id,expected):
    output,path=batch_paths(run_id)
    require(isinstance(expected,str) and re.fullmatch(r"[0-9a-f]{64}",expected) and digest(path)==expected,
        "exact prospective batch declaration required")
    plan=read_json(path)
    require(plan["schema"]==SCHEMA+".batch" and plan["run_id"]==run_id and plan["output"]==str(output)
        and plan["prospective_trial_count"]==3 and plan["all_three_trials_required"] is True
        and plan["automatic_retry"] is False and plan["outcome_selected_retry"] is False,
        "fixed all-three prospective declaration differs")
    required=dict(attempts_per_trial=1,net_progress_minimum_m=90.,capture_seconds_per_trial=45,
        controller_target_cap_mps=2.4,explicit_controller_calibration_intervention=True,
        original_model_speed_envelope_mps=2.5,hard_policy_deadline_seconds=.1,
        wall_bundle_delivery_to_ACK_100ms_separate=True,fullsensor_callback_to_ACK_unmeasured=True,
        full_RTE01_approved=False,vehicle_control_approved=False,model_promoted=False,failed_attempts_retained=True)
    require(all(plan.get(key)==value for key,value in required.items()),"fixed batch thresholds or outcome semantics differ")
    require(set(plan)==set(required)|{"schema","run_id","output","declared_at_utc","prospective_trial_count",
        "all_three_trials_required","automatic_retry","outcome_selected_retry","trials"},"batch declaration shape differs")
    stamp=datetime.fromisoformat(plan["declared_at_utc"])
    require(stamp.tzinfo is not None and stamp.utcoffset().total_seconds()==0 and stamp<=datetime.now(timezone.utc),
        "prior UTC batch declaration required")
    require(len(plan["trials"])==3 and [row["trial_index"] for row in plan["trials"]]==[0,1,2],
        "all three ordered prospective trials required")
    for trial in range(3):
        with scoped_trial(trial):
            _,trial_path=paths(run_id); row=plan["trials"][trial]
            require(row["declaration_path"]==str(trial_path),"trial declaration escaped fixed namespace")
            base.verify_plan(run_id,row["declaration_sha256"])
    return plan


def batch_summary(run_id,expected):
    plan=verify_batch(run_id,expected)
    results=[]
    for trial in range(3):
        with scoped_trial(trial):output,_=paths(run_id)
        report_path=output/"diagnostic_report.json"
        report=read_json(report_path) if report_path.is_file() else {}
        owner_path=output/"owner_result.json"
        owner=read_json(owner_path) if owner_path.is_file() else {}
        results.append(dict(trial_index=trial,status=report.get("status","NOT_EXECUTED_OR_NO_DIAGNOSTIC"),
            owner_status=owner.get("status","NOT_EXECUTED"),
            learned_driving_screen_passed=report.get("learned_driving_screen_passed",False),
            observed_bundle_to_ACK_wall100ms_passed=report.get("predeclared_observed_bundle_to_ACK_wall_latency_screen_passed",False),
            learned_effect_progress_m=report.get("learned_effect_progress_m"),
            diagnostic_sha256=digest(report_path) if report else None,
            original_owner_error=owner.get("error"),cleanup_errors=owner.get("cleanup_errors")))
    return dict(schema=SCHEMA+".batch_summary",declaration_sha256=expected,all_three_trials=results,
        all_trials_basic_driving_passed=all(row["learned_driving_screen_passed"] for row in results),
        all_trials_observed_bundle_to_ACK_wall100ms_passed=all(row["observed_bundle_to_ACK_wall100ms_passed"] for row in results),
        all_trials_motion_body_policy_AND_wall100ms_passed=all(row["learned_driving_screen_passed"]
            and row["observed_bundle_to_ACK_wall100ms_passed"] for row in results),
        fullsensor_callback_to_ACK_measured=False,full_RTE01_approved=False,
        failed_or_missing_trials_not_removed=True,vehicle_control_approved=False,model_promoted=False)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument("--mode",choices=("contract","declare","preflight","execute","worker","summary"),default="contract")
    parser.add_argument("--run-id",default="v1")
    parser.add_argument("--trial",type=int,choices=(0,1,2),default=0)
    parser.add_argument("--plan-sha256")
    parser.add_argument("--trial-plan-sha256")
    args=parser.parse_args(argv)
    if args.mode=="declare":result=declare_batch(args.run_id)
    elif args.mode=="contract":
        with scoped_trial(args.trial):result=fixed_contract(args.run_id,True)
    elif args.mode=="summary":result=batch_summary(args.run_id,args.plan_sha256)
    elif args.mode=="worker":
        with scoped_trial(args.trial):return base.worker(args.run_id,args.trial_plan_sha256)
    else:
        plan=verify_batch(args.run_id,args.plan_sha256)
        trial_sha=plan["trials"][args.trial]["declaration_sha256"]
        with scoped_trial(args.trial):
            if args.mode=="execute":return base.execute(args.run_id,trial_sha)
            _,result=base.preflight(args.run_id,trial_sha)
    print(json.dumps(result,indent=2,allow_nan=False))
    return 0


if __name__=="__main__":
    raise SystemExit(main())
