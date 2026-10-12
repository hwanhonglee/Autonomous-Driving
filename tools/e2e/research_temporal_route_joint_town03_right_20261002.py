"""HH_261002 - Own one original-navigation known-DEV Town03 learned right-turn event; not goal/feature approval."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import gzip
import json
import math
from pathlib import Path
import re
import shutil
import sys

import numpy as np

from . import research_temporal_route_joint_signed45_town07_20261002 as numeric
from . import research_temporal_route_joint_thread4_signed45_town07_20261002 as threads
from . import research_temporal_route_joint_preview05_signed45_town07_20261002 as preview
from . import collect_temporal_route_joint_town03_right_20261002 as live
from . import audit_simulation_research_body_lane_20261001 as geometry

base, ROOT, CARLA_ROOT = numeric.base, numeric.ROOT, numeric.base.CARLA_ROOT
require,digest,read_json,write_new,jsonl = base.require,base.digest,base.read_json,base.write_new,base.jsonl
MODULE = "scripts.e2e.research_temporal_route_joint_town03_right_20261002"
SCHEMA = "portable_e2e.owned_route_joint_town03_known_dev_right.20261002.v1"
ROUTE = ROOT/"storage/datasets/raw/carla/common10_v1/2026-09-05/30kph/town03/turn/qualified_routes/town03_right_s0000_p00.aligned.json"
XODR = CARLA_ROOT/"CarlaUE4/Content/Carla/Maps/OpenDrive/Town03.xodr"
UMAP = CARLA_ROOT/"CarlaUE4/Content/Carla/Maps/Town03.umap"
XODR_SHA = "abf177bcd9b66cfefb7ac4919f6456815f1384f4229f1343c8a6a90bbb7713c9"
UMAP_SHA = "3409354bb605409012f4153fa7ee1191a4205ad71303eb5873e6ef7a9cea78e4"
EPISODE_QUOTA, ARM_RESERVE = 2*1024**3, (2*1024**3)+128*1024**2
_active,_pins,_benchmark_assets=False,None,None
_original_worker,_original_environment=base.worker,base.environment
_original_fixed=base.fixed_contract


def paths(run_id):
    require(isinstance(run_id,str) and re.fullmatch(r"v[1-9][0-9]*",run_id),"fresh Town03 version required")
    output=ROOT/("storage/experiments/validation/2026-10-02/temporal_route_joint_town03_known_dev_right_"+run_id)
    return output,output.with_name(output.name+"_plan.json")


def capture_argv(run_id,longitudinal_pi,declaration_sha=base.PLAN_TOKEN):
    require(longitudinal_pi is True and _pins is not None,"externally pinned final6000/PI required")
    output,_=paths(run_id)
    return ["--research-weights",str(numeric.WEIGHTS),"--research-metadata",str(numeric.METADATA),
        "--research-weights-sha256",_pins["weights_sha256"],"--research-metadata-sha256",_pins["metadata_sha256"],
        "--research-source-sha256",declaration_sha,"--research-longitudinal-pi",str(output/"episode"),str(ROUTE),
        "--host","127.0.0.1","--port","2440","--control-transport","acknowledged_batch",
        "--agent-initialization","after_bootstrap","--stationary-warmup-sec","1.2","--stationary-tail-sec","0",
        "--max-duration-sec","110","--target-speed-kmh","9","--seed",str(live.SEED),"--weather","ClearNoon",
        "--mapping",str(base.MAPPING),"--calibration",str(base.CALIBRATION),"--spawn-z-offset-m","0.5",
        "--wheelbase-m","2.85","--expected-live-xodr-sha256",XODR_SHA,"--goal-stop-profile","disabled"]


def fixed_contract(run_id,longitudinal_pi):
    require(longitudinal_pi is True and _pins is not None,"fixed private final6000 scope required")
    result=_original_fixed(run_id,True)
    # HH_261002 - Reuse generic safety fields, never Town07 signed45 geometry/result assumptions.
    for name in ("parent_checkpoint_updates","one_episode_specialized_research"):
        result.pop(name,None)
    output,plan=paths(run_id)
    result.update(schema=SCHEMA,output=str(output),declaration_path=str(plan),town="Town03",scenario="right",seed=live.SEED,
        dataset_split="val",known_DEV_site_reused=True,independent_validation=False,training_data_approved=False,
        experiment="One actual learned right turn on complete original known-DEV213-point navigation",
        total_capture_seconds=110,stationary_tail_seconds=0,maximum_native_state_count=2200,maximum_camera_bundle_count=1100,
        maximum_driving_native_ticks=2176,maximum_warmup_native_ticks=24,
        source_checkpoint_sha256=_pins["checkpoint_sha256"],optimizer_updates=6000,known_total_optimizer_updates=12200,
        checkpoint_selection="fixed completed final6000; no endpoint/model selection during study",
        new_fixed_endpoint_export_sha256=dict(weights=_pins["weights_sha256"],metadata=_pins["metadata_sha256"]),
        actual_training_declaration_sha256=_pins["declaration_sha256"],
        parent_numeric_export_sha256=dict(weights=numeric.PARENT_NPZ_SHA,metadata=numeric.PARENT_METADATA_SHA),
        backbone_parent_numeric_export_sha256=dict(weights=numeric.BACKBONE_NPZ_SHA,metadata=numeric.BACKBONE_METADATA_SHA),
        trainable_state_tensor_count=54,frozen_state_tensor_count=47,original_backbone_weights_byte_unchanged=False,
        semantic_head_parameters_frozen=True,semantic_outputs_preserved=False,
        controller_target_cap_mps=2.4,speed_preview_s=.5,lateral_lookahead_time_s=.8,
        explicit_controller_calibration_intervention=True,model_only_performance_claimed=False,
        original_decoder_gate_tracker_math_unchanged=False,
        original_NN_decoder_selected_trajectory_math_unchanged=True,
        tracker_changed_only_by_disclosed_preview_and_PI_headroom=True,
        cpu_thread_numerical_preset=threads.numerical_preset(),
        full_original_navigation_points=213,original_navigation_sha256=live.ROUTE_SHA,
        original_catalog_route_length_m=live.ROUTE_LENGTH,navigation_truncated_for_easy_goal=False,
        old_GO_minimum_actual_navigation_support_m=18.,GO_support_threshold_unchanged=True,
        oracle_future_inputs=False,runtime_route_as_output_replacement=False,
        event=dict(schema=live.SCHEMA+".event",entry_original_route_arc_m=live.ENTRY,exit_original_route_arc_m=live.EXIT,
            expected_route_signed_turn_degrees=live.EXPECTED_TURN_DEG,measured_unwrapped_right_turn_maximum_deg=-80.,
            post_exit_actual_rear_XY_travel_minimum_m=10.,post_exit_continuous_band_seconds=3.,
            current_unclamped_route_arc_minimum_m=live.EXIT+10.,
            actual_rear_CTE_band_maximum_m=.35,actual_heading_to_current_route_band_maximum_deg=5.,
            prior_learned_ACK_current_effect_required_at_every_native_row=True,
            outgoing_original_safety_rejection_cannot_trigger_success=True,
            observer_after_original_native_camera_proposal_ACK_and_JPEG_recording=True,
            event_completion_is_not_goal_reached=True,closing_brake_is_not_learned_STOP=True),
        full_failed_tail_until_total110s_unless_original_capture_safety_abort=True,
        dynamic_success_interval_not_chosen_posthoc=True,attempts=1,automatic_retry=False,
        output_episode_quota_bytes=EPISODE_QUOTA,reserved_attempt_bytes=ARM_RESERVE,
        event_is_not_full_route_stop_RTE03_or_MAIN9_approval=True,full_RTE03_approved=False,
        original_model_and_gate_math_unchanged=True,controller_and_thread_presets_explicit_not_new_training=True,
        server_command=["bash","tools/e2e/run_carla_map.sh","Town03","--port","2440","--quality","Epic",
            "--startup-timeout-sec","120","--","-RenderOffScreen","-nosound"],
        capture_argv=capture_argv(run_id,True),owner_wall_timeout_s=600.,
        worker_command=[sys.executable,"-B",str(ROOT/"tools/workspace_python.py"),"-m",MODULE,"--",
            "--mode","worker","--run-id",run_id,"--plan-sha256",base.PLAN_TOKEN]
            +[value for name in ("weights","metadata","checkpoint","declaration")
                for value in ("--expected-"+name+"-sha256",_pins[name+"_sha256"])])
    result["sensor_and_ACK_screen"].update(native_state_count="actual whole event or110s bound; no discarded suffix",
        camera_bundle_count="every even native capture ordinal",maximum_native_state_count=2200,maximum_camera_bundle_count=1100)
    result["precursor_motion_screen"]["net_learned_progress_minimum_m"]=90.
    result["body_clearance_screen"]=dict(all_native_boxes="all actualN",all_snapshot_linear_sweeps="all actualN-1",lane_inset_m=.1)
    result["additive_native_speed_rate_guard_mps2"]=dict(minimum=-6.,maximum=3.,warmup_to_first_driving_interval_included=True)
    result["execute_exit_status_interpretation"]="operational ownership+complete retained interval; see joint event/safety/wall outcome"
    return result


def runtime_sources():
    require(_pins is not None,"exact final6000 source scope required")
    pins=numeric.runtime_sources();live.verify_frozen()
    for path in base.imported_source_closure((Path(__file__).resolve(),)):
        name=str(path.relative_to(ROOT));sha=digest(path)
        require(name not in pins or pins[name]==sha,"new Town03 source conflicts with exact frozen dependency")
        pins[name]=sha
    return pins


def external_sources():
    require(digest(UMAP)==UMAP_SHA,"exact packaged Town03 map bytes changed")
    return {str(path):digest(path) for path in base.AGENT_FILES+(CARLA_ROOT/"CarlaUE4.sh",
        CARLA_ROOT/"CarlaUE4/Config/DefaultEngine.ini",UMAP)}


def load_model(weights,metadata,**pins):
    threads.configure_threads();model=numeric.load_model(weights,metadata,**pins)
    model._research_cpu_thread_numerical_revision=live.THREAD_SCHEMA
    model._research_speed_preview_calibration=live.PREVIEW_SCHEMA
    threads.live.thread_readback()
    return model


def private_worker(run_id,expected):
    require(_active and numeric._owner_plan_sha is None,"exclusive new Town03 worker required")
    plan=base.verify_plan(run_id,expected);base.assert_owned_worker(plan,expected)
    numeric._owner_plan_sha=expected
    try:return _original_worker(run_id,expected)
    finally:numeric._owner_plan_sha=None


def environment():
    result=_original_environment();result.update(OMP_NUM_THREADS="4",MKL_NUM_THREADS="4",OPENBLAS_NUM_THREADS="4")
    return result


def disk_check(free=None):
    value=shutil.disk_usage(ROOT).free if free is None else free
    extra=sum(path.stat().st_size for path in _benchmark_assets)
    required=base.FLOOR+ARM_RESERVE+base.SHARED_RESERVE+extra
    require(type(value) is int and value>=required,"complete new110s/source/close disk reserve required")
    return dict(free_bytes=value,required_bytes=required,episode_quota_bytes=EPISODE_QUOTA,
        sampled_exact_input_quota_bytes=base.SAMPLE_QUOTA,benchmark_archive_bytes=extra)


def verify_closure(output,plan):
    numeric.parent.verify_derived_source_closure(output,plan)


def complete_interval(manifest,states,cameras,event):
    if not states:return False
    native=[r["frame"] for r in states]
    chain=native==list(range(native[0],native[0]+len(native))) and len(native)<=2200 \
        and [r["frame"] for r in cameras]==native[::2]
    warmup=[r for r in states if r["capture_phase"]=="stationary_warmup"]
    driving=[r for r in states if r["capture_phase"]=="driving"]
    schedule=manifest.get("capture_contract",{}).get("capture_phase_schedule",{})
    phase= len(warmup)==24 and len(warmup)+len(driving)==len(states) and len(driving)<=2176 \
        and schedule.get("stationary_tail",{}).get("scheduled_ticks")==0
    event_close=manifest.get("status")=="complete" and event["completed"] is True \
        and manifest.get("result",{}).get("goal_reached") is False \
        and manifest["result"].get("known_dev_right_turn_event",{}).get("completed") is True
    budget=manifest.get("status")=="failed" and manifest.get("error")=="BasicAgent did not finish within the driving-phase tick budget before the maximum capture duration" \
        and len(states)==2200 and len(cameras)==1100 and event["completed"] is False
    return bool(chain and phase and (event_close or budget))


def independent_event(route,states,commands):
    observer=live.RightTurnObserver(route)
    learned={r["ack_receipt_sequence"] for r in commands if r.get("application",{}).get("learned_control_applied") is True}
    by_frame={r["command_frame"]:r for r in commands if "command_frame" in r}
    require(len(by_frame)==sum("command_frame" in r for r in commands),"unique outgoing normal decision frame required")
    for state in states:
        if state["capture_phase"]!="driving":continue
        sequence=state.get("control_transport",{}).get("expected_receipt_sequence")
        learned_effect=sequence in learned and state.get("control_transport",{}).get("status")=="PASS"
        outgoing=by_frame.get(state["frame"],{})
        outgoing_safe=bool(outgoing) and outgoing.get("control_source")!="emergency_brake"
        if observer.observe(state,learned_effect=learned_effect,outgoing_safe=outgoing_safe):
            require(state is states[-1],"captured rows after declared event are not allowed to disappear")
    return observer.summary(),observer.records


def body_report(output,plan,episode,states,receipts,first_effect_frame):
    count=len(states);require(count>1,"all actual body rows required")
    require(digest(episode/"route.json")==live.ROUTE_SHA and digest(episode/"active_map_opendrive.xodr")==XODR_SHA,
        "exact original navigation/live Town03 map required")
    actors={r["actor_id"] for r in receipts};require(len(actors)==1,"one ACK actor required")
    saved=list(sys.path)
    try:
        sys.path.insert(0,str(base.EGG));import carla
        require(str(carla.__file__).startswith(str(base.EGG)+"/"),"foreign offline CARLA API")
        rows,census=geometry.body_census(read_json(episode/"manifest.json"),read_json(episode/"route.json"),states,
            (episode/"active_map_opendrive.xodr").read_bytes(),next(iter(actors)))
    finally:sys.path[:]=saved
    passed=len(rows)==count and len(census)==4 and all(r["status"]=="PASS"
        and r["assessed_count"]==(count-1 if k.startswith("previous_sweep") else count) for k,r in census.items())
    verify_closure(output,plan)
    target=output/"derived_body";target.mkdir(exist_ok=False)
    with gzip.open(target/"all_native_boxes_and_sweeps.jsonl.gz","xt") as stream:
        for row in rows:stream.write(json.dumps(row,allow_nan=False)+"\n")
    result=dict(body_and_swept_clearance_passed=passed,body_geometry=census,all_native_boxes=count,
        all_snapshot_linear_sweeps=count-1,lane_inset_m=.1,physical_continuous_time_proven=False)
    post=[r for r in rows if first_effect_frame is not None and r["frame"]>=first_effect_frame]
    cte=[abs(r["independent_actor_route_distance_m"]) for r in post]
    result.update(post_handoff_actor_CTE_p95_m=float(np.percentile(cte,95)) if cte else None,
        post_handoff_actor_CTE_maximum_m=max(cte) if cte else None,
        post_handoff_actor_progress_m=post[-1]["independent_actor_arc_m"]-post[0]["independent_actor_arc_m"] if len(post)>1 else None,
        post_handoff_actor_geometry_denominator=len(post),actor_and_rear_projection_references_not_interchanged=True)
    write_new(target/"summary.json",result);return result


def recording_report(output,episode,cameras,timing):
    records=jsonl(episode/"recording_after_ack.jsonl")
    by_frame={r["frame"]:r for r in records};by_ack={r["ACK_sequence"]:r for r in timing}
    require(len(records)==len(by_frame)==len(cameras) and len(by_ack)==len(timing),"complete unique camera/ACK receipts required")
    pictures={}
    for camera in cameras:
        order=live.collector.MODEL_CAMERA_ORDER
        require(tuple(camera["camera_order"])==order and set(camera["images"])==set(order),"exact producer camera order required")
        row=by_frame[camera["frame"]];ack=by_ack[row["ACK_receipt_sequence"]]
        require(row["enqueued_camera_count"]==row["written_camera_count"]==6 and row["failure"] is None
            and row["all_queued_images_written"] is True and row["acknowledged_before_recording"] is True
            and ack["ACK_server_accepted"] is True and ack["command_frame"]==camera["frame"]
            and ack["ACK_finished_monotonic_ns"]<=row["recording_started_monotonic_ns"]<=row["recording_finished_monotonic_ns"],
            "all exact six-image saves must follow their actual ACK")
        relatives=[Path(camera["images"][name]) for name in order]
        require(all(not p.is_absolute() and ".." not in p.parts and len(p.parts)==3 and p.parts[:2]==("images",name)
            and p.suffix==".jpg" for name,p in zip(order,relatives)),"unsafe exact image lexical path")
        require(row["written_files"] in ([str(output/alias/p) for p in relatives] for alias in ("episode.partial","episode")),
            "literal original partial/final recording path/order differs")
        for relative in relatives:
            path=episode/relative
            require(path.resolve()==path and not any(p.is_symlink() for p in (path,*path.parents)) and str(path) not in pictures,
                "image escaped or duplicated")
            pictures[str(path)]=digest(path)
    require(len(pictures)==6*len(cameras),"entire distinct JPEG denominator required")
    return dict(passed=True,bundles=len(cameras),JPEG_count=len(pictures),JPEG_sha256=pictures,JPEG_decoded=False)


def native_rate_report(states):
    # HH_261002 - Preserve the0.5 additive scalar SPEED-rate guard, including the last warmup→first driving interval.
    require(states and all(b["frame"]==a["frame"]+1 and abs(b["timestamp"]-a["timestamp"]-.05)<=1.e-6
        for a,b in zip(states,states[1:])),"whole consecutive20Hz native rate census required")
    first=next((i for i,s in enumerate(states) if s["capture_phase"]=="driving"),None)
    driving=states[first:] if first is not None else []
    require(driving and all(s["capture_phase"]=="driving" for s in driving),"every native row after driving start required")
    prefix=states[max(0,first-1):]
    rates=[(math.hypot(*b["world_velocity_carla"][:2])-math.hypot(*a["world_velocity_carla"][:2]))/(b["timestamp"]-a["timestamp"])
        for a,b in zip(prefix,prefix[1:])]
    require(len(rates)==len(driving)-(first==0),"full destination-driving speed-rate denominator differs")
    passed=bool(rates) and all(math.isfinite(v) and -6<=v<=3 for v in rates)
    return dict(passed=passed,rates=rates,driving_native_count=len(driving),interval_count=len(rates),
        first_warmup_to_driving_boundary_included=first>0,minimum_guard_mps2=-6.,maximum_guard_mps2=3.)


def actor_motion_report(body,post_count):
    cte=body.get("post_handoff_actor_CTE_p95_m");progress=body.get("post_handoff_actor_progress_m")
    return cte is not None and cte<=.35 and body["post_handoff_actor_CTE_maximum_m"]<=.75 \
        and progress is not None and progress>=90 and body["post_handoff_actor_geometry_denominator"]==post_count


def diagnostic_report(output,plan,owner):
    raw={}
    result=dict(schema=SCHEMA+".diagnostic_report",ownership_passed=False,bounded_capture_complete=False,
        learned_driving_screen_passed=False,joint_turn_motion_body_policy_wall100ms_passed=False,
        full_MAIN9_completion_claimed=False,full_RTE03_approved=False,training_data_approved=False,
        full_route_or_stop_approved=False,vehicle_control_approved=False,model_promoted=False,original_owner=owner,
        all_failed_tail_retained=False,all_available_raw_files_retained_and_hashed=False)
    try:
        episode=output/"episode" if (output/"episode").is_dir() else output/"episode.partial"
        raw={str(path):digest(path) for path in episode.rglob("*") if path.is_file()}
        result.update(raw_source_sha256=raw,all_available_raw_files_retained_and_hashed=bool(raw))
        manifest=read_json(episode/"manifest.json")
        states,cameras,commands,receipts,timing=[jsonl(episode/name) for name in
            ("states.jsonl","camera_frames.jsonl","learned_control.jsonl","control_receipts.jsonl","wall_timing.jsonl")]
        result.update(actual_native_state_count=len(states),actual_camera_bundle_count=len(cameras),actual_ACK_receipt_count=len(receipts),
            original_capture_status=manifest.get("status"),original_capture_error=manifest.get("error"))
        ownership=not owner["cleanup_errors"] and owner["source_hashes_match"] and owner["external_source_hashes_match"]
        applied=[r for r in commands if r.get("application",{}).get("learned_control_applied") is True]
        sequences={r["ack_receipt_sequence"] for r in applied}
        first=next((i for i,s in enumerate(states) if s.get("control_transport",{}).get("expected_receipt_sequence") in sequences),None)
        # HH_261002 - Original hard-abort captures still receive ALL recorded body/swept checks, never only successful intervals.
        body=dict(body_and_swept_clearance_passed=False,status="NOT_PROVEN",recorded_native_count=len(states))
        if ownership and len(states)>1:
            try:body=body_report(output,plan,episode,states,receipts,states[first]["frame"] if first is not None else None)
            except Exception as error:body.update(status="FAIL",error=type(error).__name__+": "+str(error))
        result.update(ownership_passed=ownership,independent_body=body)
        route=read_json(output/"source"/ROUTE.relative_to(ROOT));event,observations=independent_event(route,states,commands)
        bounded=complete_interval(manifest,states,cameras,event)
        result.update(bounded_capture_complete=bounded,all_failed_tail_retained=bounded,
            incomplete_or_original_hard_abort_not_claimed_complete=not bounded)
        chain=bool(states) and all(abs(b["timestamp"]-a["timestamp"]-.05)<=1.e-6 for a,b in zip(states,states[1:]))
        require([r["sequence"] for r in receipts]==list(range(1,len(receipts)+1)),"complete ordered ACK receipt chain required")
        ack=base.independent_audit.analyze_transport(states,receipts,
            manifest["capture_contract"]["control_transport"]["bootstrap_observation"],wheelbase=manifest["coordinate_contract"]["wheelbase_m"])
        camera=base.independent_audit.analyze_camera_alignment(states,cameras)
        require(manifest["coordinate_contract"]["wheelbase_m"]==2.85 and
            manifest["capture_contract"]["learned_research"]["stream_identity"]["split"]=="val","actual known-DEV coordinate/identity differs")
        recording=recording_report(output,episode,cameras,timing)
        configuration=preview.configuration_report(episode);thread_readback=threads.readback_report(episode)
        post=states[first:] if first is not None else []
        full_effect=bool(post) and all(s["capture_phase"]=="driving" and s["control_transport"]["status"]=="PASS"
            and s["control_transport"]["expected_receipt_sequence"] in sequences for s in post)
        fresh=[r for r in commands if r.get("learned_control_proposed") is True and r.get("held_between_camera_frames") is False]
        fresh_wall=[r for r in timing if r.get("learned_control_proposed") is True and r.get("held_between_camera_frames") is False]
        wall_by_ack={r["ACK_sequence"]:r for r in fresh_wall};values=[]
        wall_complete=bool(fresh) and len(wall_by_ack)==len(fresh_wall)==len(fresh)
        for command in fresh:
            row=wall_by_ack.get(command["ack_receipt_sequence"],{})
            wall_complete &= row.get("ACK_server_accepted") is True and row.get("inference_frame")==command["inference_frame"] \
                and row.get("command_frame")==command["command_frame"] and row.get("stream_source_sha256")==owner["declaration_sha256"] \
                and row.get("source_checkpoint_sha256")==plan["source_checkpoint_sha256"]
            values.append(row.get("exact_bundle_delivery_to_ACK_seconds"))
        wall=wall_complete and all(type(v) in (int,float) and math.isfinite(v) and 0<=v<=.1 for v in values)
        policy=bool(fresh) and all(type(r.get("total_seconds")) in (int,float) and 0<=r["total_seconds"]<=.1 for r in fresh)
        PI=bool(fresh)
        for decision in fresh:
            audit=decision.get("longitudinal_pi_research",{});target=decision.get("control",{}).get("desired_speed_mps")
            PI &= type(target) in (int,float) and math.isfinite(target) and audit.get("controller_target_cap_mps")==2.4 \
                and audit.get("original_model_target_mps")==target and audit.get("governed_controller_target_mps")==min(target,2.4) \
                and audit.get("explicit_controller_calibration_intervention") is True and audit.get("observed_speed_clipped") is False \
                and audit.get("observed_speed_mps")==decision.get("measured_current_speed_mps") \
                and audit.get("one_fresh_camera_update_one_PI_integration") is True \
                and audit.get("proposed_normal_tracker_control")==decision.get("control")
        zero=sum(r.get("control",{}).get("desired_speed_mps",math.inf)<=.05 for r in applied)
        emergency=sum(r.get("control_source")=="emergency_brake" for r in commands)
        base_v=[math.hypot(s["vx"],s["vy"]) for s in post];world_v=[math.hypot(*s["world_velocity_carla"][:2]) for s in post]
        speed=bool(post) and all(math.isfinite(v) and v<=2.5 for v in base_v) and all(math.isfinite(v) and v<=2.7 for v in world_v)
        native_rates=native_rate_report(states);rates=native_rates["rates"];acceleration=native_rates["passed"]
        cte=[abs(r["actual_rear_CTE_m"]) for r in observations if first is not None and r["frame"]>=states[first]["frame"]]
        progress=observations[-1]["actual_rear_progress_m"]-next(r["actual_rear_progress_m"] for r in observations
            if first is not None and r["frame"]>=states[first]["frame"]) if cte else None
        dwell=longest=0.
        for s in post:
            dwell=dwell+.05 if math.hypot(*s["world_velocity_carla"][:2])<=.05 else 0.;longest=max(longest,dwell)
        actor_motion=actor_motion_report(body,len(post))
        motion=ownership and bounded and chain and ack["status"]==camera["status"]=="PASS" and full_effect \
            and progress is not None and progress>=90 and cte and float(np.percentile(cte,95))<=.35 and max(cte)<=.75 \
            and longest<1 and emergency==zero==0 and speed and acceleration and policy and PI and actor_motion and configuration["passed"] \
            and thread_readback["passed"] and body["body_and_swept_clearance_passed"] \
            and not any(s.get("collision") or s.get("lane_invasion") for s in states)
        verify_closure(output,plan);require(all(digest(name)==sha for name,sha in raw.items()),"raw inputs changed during independent audit")
        result.update(ownership_passed=ownership,bounded_capture_complete=bounded,actual_native_state_count=len(states),
            actual_camera_bundle_count=len(cameras),actual_ACK_receipt_count=len(receipts),independent_event=event,
            independent_ACK=ack,independent_camera=camera,recording=recording,actual_preview_configuration=configuration,
            actual_thread_settings=thread_readback,independent_body=body,complete_post_handoff_interval=full_effect,
            actual_base_speed_maximum_mps=max(base_v) if base_v else None,actual_actor_speed_maximum_mps=max(world_v) if world_v else None,
            learned_progress_m=progress,CTE_p95_m=float(np.percentile(cte,95)) if cte else None,CTE_maximum_m=max(cte) if cte else None,
            stationary_dwell_maximum_s=longest,zero_target_applied_fresh_and_held_count=zero,emergency_command_count=emergency,
            native_speed_rate_minimum_mps2=min(rates) if rates else None,native_speed_rate_maximum_mps2=max(rates) if rates else None,
            additive_native_speed_rate_passed=acceleration,fresh_policy100ms_passed=policy,
            native_speed_rate_census=native_rates,
            complete_fresh_PI_receipt_denominator_passed=PI,original_actor_motion_thresholds_passed=actor_motion,
            complete_fresh_wall_denominator=wall_complete,observed_bundle_to_ACK_wall100ms_passed=wall,
            observed_bundle_to_ACK_maximum_ms=max(values)*1000 if wall_complete and all(v is not None for v in values) else None,
            learned_driving_screen_passed=bool(motion),joint_turn_motion_body_policy_wall100ms_passed=bool(motion and event["completed"] and wall),
            all_failed_tail_retained=bounded,raw_source_sha256=raw,full_sensor_callback_to_ACK_unmeasured=True)
    except Exception as error:
        result["diagnostic_error"]=type(error).__name__+": "+str(error)
    try:
        raw_unchanged=bool(raw) and all(digest(name)==sha for name,sha in raw.items())
    except Exception as error:
        raw_unchanged=False;result["raw_postcheck_error"]=type(error).__name__+": "+str(error)
    result["all_available_raw_files_retained_and_hashed"]=raw_unchanged
    result["raw_before_after_unchanged"]=raw_unchanged
    if not raw_unchanged:
        result["learned_driving_screen_passed"]=False;result["joint_turn_motion_body_policy_wall100ms_passed"]=False
    result["status"]="PASS_KNOWN_DEV_RIGHT_TURN_PRECURSOR" if result["joint_turn_motion_body_policy_wall100ms_passed"] else "FAIL_KNOWN_DEV_RIGHT_TURN_PRECURSOR"
    return result


@contextmanager
def activated(pins):
    global _active,_pins,_benchmark_assets
    require(not _active and not numeric._active,"new Town03 owner cannot nest")
    numeric.verify_exports(pins);assets=threads.evidence_assets(pins)
    # HH_261002 - Live navigation/map are only Town03; explicitly archived Town07 benchmark evidence is not current navigation.
    assets=dict(assets)|{ROUTE:live.ROUTE_SHA,XODR:XODR_SHA,base.EGG:base.ASSETS[base.EGG],
        base.MAPPING:base.ASSETS[base.MAPPING],base.CALIBRATION:base.ASSETS[base.CALIBRATION],
        numeric.WEIGHTS:pins["weights_sha256"],numeric.METADATA:pins["metadata_sha256"],
        numeric.WARM_PARENT/"model_state.npz":numeric.PARENT_NPZ_SHA,numeric.WARM_PARENT/"metadata.json":numeric.PARENT_METADATA_SHA,
        numeric.BACKBONE_PARENT/"model_state.npz":numeric.BACKBONE_NPZ_SHA,numeric.BACKBONE_PARENT/"metadata.json":numeric.BACKBONE_METADATA_SHA}
    values=dict(paths=paths,capture_argv=capture_argv,fixed_contract=fixed_contract,runtime_sources=runtime_sources,
        external_sources=external_sources,load_npz=load_model,disk_check=disk_check,live=live,SCHEMA=SCHEMA,MODULE=MODULE,
        diagnostic_report=diagnostic_report,worker=private_worker,environment=environment,ROUTE=ROUTE,XODR=XODR,
        WEIGHTS=numeric.WEIGHTS,METADATA=numeric.METADATA,WEIGHTS_SHA=pins["weights_sha256"],METADATA_SHA=pins["metadata_sha256"],
        CHECKPOINT_SHA=pins["checkpoint_sha256"],ASSETS=assets,EPISODE_QUOTA=EPISODE_QUOTA,ARM_RESERVE=ARM_RESERVE)
    saved={name:getattr(base,name) for name in values};old_exports=numeric._exports;old_validate=live.frozen.validate_capture
    old_numeric_module=numeric.MODULE
    _active=True;_pins=dict(pins);_benchmark_assets=assets
    numeric._exports=dict(pins);numeric.MODULE=MODULE
    try:
        for name,value in values.items():setattr(base,name,value)
        live.frozen.validate_capture=live.validate_capture
        yield
    finally:
        live.frozen.validate_capture=old_validate;numeric._exports=old_exports;numeric.MODULE=old_numeric_module
        for name,value in reversed(tuple(saved.items())):setattr(base,name,value)
        _active=False;_pins=None;_benchmark_assets=None


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    for name in ("weights","metadata","checkpoint","declaration"):
        parser.add_argument("--expected-"+name+"-sha256",required=True)
    args,remainder=parser.parse_known_args(argv)
    pins={name+"_sha256":getattr(args,"expected_"+name+"_sha256") for name in ("weights","metadata","checkpoint","declaration")}
    with activated(pins):return base.main(remainder)


if __name__=="__main__":raise SystemExit(main())
