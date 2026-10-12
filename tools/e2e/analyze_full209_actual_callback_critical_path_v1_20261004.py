"""HH_261004 - Reconstruct every actual full209 source-to-plan path and native/quota boundary read-only, preserving original FAIL."""

from __future__ import annotations

import argparse
from collections import defaultdict
import hashlib
import json
import math
from pathlib import Path
import statistics

from .benchmark_scandir_episode_quota_v1_20261004 import digest, require, safe_path, tree_members, write_new_json, read_json

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve()
TEST = ROOT / "learning/tests/test_full209_actual_callback_critical_path_v1_20261004.py"
RUN = ROOT / "storage/experiments/validation/2026-10-04/temporal_route_joint_town03_known_dev_straight_full209_v1"
EPISODE = RUN / "episode"
CALLER = ROOT / "storage/experiments/plans/2026-10-04/town03_known_dev_straight_full209_v1/ROOT_actual_publication_caller.json"
CALLER_SHA = "021cec4b62777dbceaa0c053c01b2e7afa03b92d85ad7e4c758f540c416bf209"
OUTPUT = ROOT / "storage/experiments/diagnostics/2026-10-04/full209_actual_callback_critical_path_readonly_v1"
SCHEMA = "portable_e2e.full209_actual_callback_critical_path_readonly.20261004.v1"
LIVE_SCHEMA = "portable_e2e.actual_route_joint_preview05_town03_known_dev_straight_full209.20261004.v1"
ROLES = ("CAM_FRONT", "CAM_BACK", "CAM_FRONT_LEFT", "CAM_BACK_LEFT", "CAM_FRONT_RIGHT", "CAM_BACK_RIGHT")
NATIVE_STAGES = ("world_tick_started", "world_tick_returned", "snapshot_bound", "actor_state_ready", "agent_branch_started", "agent_branch_finished", "native_state_finished", "exact_bundle_started", "exact_bundle_finished")
POLICY_STAGES = ("input_ready", "raw_nn_finished", "decoded_plan_ready")
FILES = ("callback_timing_receipts.json", "quota_walk_timing_receipts.json", "states.jsonl", "camera_frames.jsonl", "wall_timing.jsonl", "learned_control.jsonl", "manifest.json", "catalog_goal_cleanup_receipt.json", "route.json")


def json_file(path, limit=64*1024**2):
    path=safe_path(path)
    require(0<path.stat().st_size<=limit,"bounded original diagnostic ABI required")
    def pairs(items):
        value={}
        for key,item in items:
            require(key not in value,"duplicate actual JSON key forbidden")
            value[key]=item
        return value
    return json.loads(path.read_text(),object_pairs_hook=pairs,parse_constant=lambda value:require(False,"nonfinite actual JSON forbidden"))


def json_lines(path):
    path=safe_path(path)
    require(path.stat().st_size<=64*1024**2,"bounded original actual JSONL required")
    def pairs(items):
        result={}
        for key,value in items:
            require(key not in result,"duplicate actual JSONL key forbidden")
            result[key]=value
        return result
    return [json.loads(line,object_pairs_hook=pairs,parse_constant=lambda value:require(False,"nonfinite actual JSONL forbidden")) for line in path.read_text().splitlines()]


def snapshot():
    require(digest(CALLER)==CALLER_SHA,"original complete full209 manifest SHA differs")
    caller=read_json(CALLER)
    require(caller.get("run_root")==str(RUN),"exact original full209 run required")
    inventory=tree_members(RUN);manifest=caller["run_files_sha256"]
    require(set(inventory["files"])==set(manifest),"whole original run membership differs")
    for name,expected in manifest.items():
        relative=Path(name)
        require(not relative.is_absolute() and relative.parts and all(part not in (".","..") for part in relative.parts)
                and relative.as_posix()==name,"canonical complete manifest member required")
        require(digest(RUN/name)==expected,"whole original manifest member SHA differs: "+name)
    source_names=("tools/e2e/benchmark_scandir_episode_quota_v1_20261004.py",
        "tools/e2e/collect_temporal_route_joint_town03_straight_full209_v1_20261004.py",
        "tools/e2e/research_temporal_route_joint_town03_straight_full209_v1_20261004.py",
        "tools/e2e/collect_temporal_route_joint_town03_straight_callback45_v3_20261004.py")
    sources={name:digest(ROOT/name) for name in source_names}
    for name in source_names[1:]:
        require(sources[name]==manifest["source/"+name],"frozen actual runtime source changed")
    sources.update({str(path.relative_to(ROOT)):digest(path) for path in (HERE,TEST)})
    require(digest(CALLER)==CALLER_SHA and tree_members(RUN)==inventory,"original inputs changed during full integrity check")
    return dict(original_manifest_sha256=CALLER_SHA,whole_run_file_count=len(manifest),full_manifest_verified=True,
        metadata_inventory_sha256=hashlib.sha256(json.dumps(inventory,sort_keys=True).encode()).hexdigest(),
        raw_sha256={str(path.relative_to(RUN)):manifest[str(path.relative_to(RUN))] for path in [RUN/"diagnostic_report.json"]+[EPISODE/name for name in FILES]},
        source_sha256=sources)


def stats(values):
    require(values and all(type(value) in (int,float) and math.isfinite(value) for value in values),"complete finite metric denominator required")
    values=sorted(values);location=.95*(len(values)-1);index=int(location)
    return dict(count=len(values),minimum=min(values),median=statistics.median(values),
        p95_linear=values[index]+(values[min(index+1,len(values)-1)]-values[index])*(location-index),maximum=max(values))


def source_join(value,cameras,commands,walls):
    """HH_261004 - Join every consumed native object, actor, stamp and BGRA SHA; never choose a favorable repeated callback."""
    require(value.get("schema")==LIVE_SCHEMA+".callback_receipts" and value.get("errors")==[] and value.get("inflight_callbacks")==0
        and value.get("all_callbacks_including_bootstrap_or_unconsumed_preserved") is True,"complete drained actual callback ledger required")
    actors={row["role"]:row["actor_id"] for row in value["camera_bindings"]}
    require(tuple(actors)==ROLES and len(set(actors.values()))==6,"exact six original camera actors required")
    callbacks=value["callbacks"];by_identity={}
    require(sorted(row["receipt_sequence"] for row in callbacks)==list(range(1,len(callbacks)+1)),"all native callback receipt sequences required")
    for row in callbacks:
        identity=(row["role"],row["native_frame"],row["image_object_id"])
        require(identity not in by_identity and row["actor_id"]==actors[row["role"]]
            and row["source_byte_count"]==640*360*4 and row["original_queue_put_calls"]==1
            and row["original_queue_put_returned"] is True and row["original_queue_put_error"] is None and row["receipt_error"] is None
            and row["source_pixels_changed"] is False and row["source_timestamp_changed"] is False
            and row["callback_enter_monotonic_ns"]<=row["queue_put_started_monotonic_ns"]<=row["queue_put_finished_monotonic_ns"]<=row["source_SHA_finished_monotonic_ns"],
            "duplicate/forged original callback identity or receipt chronology")
        by_identity[identity]=row
    consumed,by_stamp={},{}
    require(len(value["bundles"])==len(cameras),"every actual camera selection bundle required")
    for bundle,camera in zip(value["bundles"],cameras):
        frame=bundle["native_frame"];stamp=int(round(camera["timestamp"]*1e9))
        require(frame==camera["frame"] and frame not in consumed and stamp not in by_stamp
            and tuple(camera["camera_order"])==ROLES and tuple(row["role"] for row in bundle["cameras"])==ROLES,"exact selected bundle membership/order required")
        selected=[]
        for source in bundle["cameras"]:
            row=by_identity[(source["role"],frame,source["image_object_id"])]
            require(source["native_frame"]==frame and source["source_image_timestamp_simulation_ns"]==row["source_image_timestamp_simulation_ns"]==stamp
                and int(round(camera["source_timestamps"][source["role"]]*1e9))==stamp
                and row["callback_enter_monotonic_ns"]<=bundle["bundle_binding_finished_monotonic_ns"],"selected original object/stamp differs")
            selected.append(row)
        consumed[frame]=selected;by_stamp[stamp]=selected
    groups=defaultdict(dict)
    for mark in value["policy_stage_marks"]:
        group=groups[mark["frame_timestamp_simulation_ns"]]
        require(mark["stage"] in POLICY_STAGES and mark["stage"] not in group
            and mark["original_policy_step_statements_changed"] is False,"unique actual unchanged policy statements required")
        group[mark["stage"]]=mark
    fresh=[row for row in commands if row.get("learned_control_proposed") is True and row.get("held_between_camera_frames") is False]
    require(len(groups)==len(fresh) and len({row["inference_frame"] for row in fresh})==len(fresh),"whole actual fresh NN denominator required")
    wall_by_ACK={row["ACK_sequence"]:row for row in walls};require(len(wall_by_ACK)==len(walls),"unique original ACK sequences required")
    rows=[];SHA_checks=0
    for command in fresh:
        frame=command["inference_frame"];selected=consumed[frame];stamp=selected[0]["source_image_timestamp_simulation_ns"]
        group=groups[stamp];require(set(group)==set(POLICY_STAGES) and len({mark["policy_object_id"] for mark in group.values()})==1,"three same-policy actual markers required")
        inputs,NN,plan=(group[stage]["stage_finished_monotonic_ns"] for stage in POLICY_STAGES)
        ack=wall_by_ACK[command["ack_receipt_sequence"]];first=min(row["callback_enter_monotonic_ns"] for row in selected);last=max(row["callback_enter_monotonic_ns"] for row in selected)
        require(command["command_frame"]==ack["command_frame"]==ack["inference_frame"]==frame
            and ack["ACK_server_accepted"] is True and ack["learned_control_proposed"] is True and ack["held_between_camera_frames"] is False
            and ack["control_source"]=="learned_trajectory_tracker"
            and ack["proposal_started_monotonic_ns"]<=inputs<=NN<=plan<=ack["proposal_finished_monotonic_ns"]<=ack["ACK_started_monotonic_ns"]<=ack["ACK_finished_monotonic_ns"]
            and last<=ack["observed_bundle"]["exact_bundle_delivery_monotonic_ns"]<=inputs,"whole actual source/NN/decoded/ACK chronology differs")
        history=command["owned_camera_history"]["source_camera_frames"]
        require(len(history)==5 and history[-1]["timestamp_ns"]==stamp and all(a["timestamp_ns"]<b["timestamp_ns"] for a,b in zip(history,history[1:])),"genuine five-slot consumed causal history required")
        for source in history:
            prior=by_stamp[source["timestamp_ns"]]
            require(source["timestamp_ns"]<=stamp and len(source["camera_sha256"])==6,"past consumed source must be causal and complete")
            for row,sha in zip(prior,source["camera_sha256"]):
                require(row["source_BGRA_encoding_prefixed_sha256"]==sha,"exact consumed historical BGRA SHA differs")
                SHA_checks+=1
        rows.append(dict(frame=frame,ACK_sequence=ack["ACK_sequence"],branch=command["navigation_terminal_research"]["branch"],
            first_callback_ns=first,last_callback_ns=last,input_ready_ns=inputs,raw_NN_finished_ns=NN,decoded_plan_ready_ns=plan,
            ACK_finished_ns=ack["ACK_finished_monotonic_ns"],selected_callback_receipt_sequences=[row["receipt_sequence"] for row in selected],
            first_callback_plan_ms=(plan-first)/1e6,first_callback_ACK_ms=(ack["ACK_finished_monotonic_ns"]-first)/1e6))
    return rows,dict(all_callbacks=len(callbacks),selected_callbacks=6*len(cameras),unselected_callbacks=len(callbacks)-6*len(cameras),
        fresh_NN_count=len(fresh),exact_past_source_SHA_checks=SHA_checks,earliest_or_latest_repeated_callback_selected_by_latency=False)


def native_paths(value,states,cameras,route):
    """HH_261004 - Recompute the genuine goal exception from original geometry, not the failed owner's parser result."""
    groups=defaultdict(dict);camera_frames={row["frame"] for row in cameras}
    for mark in value["native_stage_marks"]:
        group=groups[mark["tick_index"]]
        require(mark["stage"] in NATIVE_STAGES and mark["stage"] not in group,"unique complete native stage ABI required")
        group[mark["stage"]]=mark
    require(set(groups)==set(range(len(states))) and len(value["native_stage_marks"])==7*len(states)+2*len(cameras),"whole native7N+2C marker denominator required")
    points=route["route"];total=sum(math.hypot(b["x"]-a["x"],b["y"]-a["y"]) for a,b in zip(points,points[1:]));goal=points[-1]
    statuses=[max(0.,total-row["route_progress_m"])<=2.5 and math.hypot(row["x"]-goal["x"],row["y"]-goal["y"])<=2.5 for row in states]
    require(statuses[-1] and not any(statuses[:-1]),"original two goal predicates first became true only at actual last native state")
    cleanup=states[-1]["catalog_goal_cleanup_receipt"]
    require(cleanup["command_frame"]==states[-1]["frame"] and cleanup["original_catalog_two_predicates_passed"] is True
        and cleanup["status"]=="ACKNOWLEDGED" and cleanup["learned_STOP_proven"] is False,"authenticated genuine terminal cleanup required, not learned STOP")
    rows=[];by_frame={}
    for index,state in enumerate(states):
        group=groups[index];frame=state["frame"];phase=state["capture_phase"];camera=frame in camera_frames
        names=NATIVE_STAGES if camera else NATIVE_STAGES[:7]
        require(set(group)==set(names) and group["world_tick_started"]["native_frame"] is None
            and all(mark["capture_phase"]==phase and mark["original_native_order_or_math_changed"] is False for mark in group.values())
            and all(mark["native_frame"]==frame for stage,mark in group.items() if stage!="world_tick_started"),"exact actual native frame/phase/stages required")
        clock=lambda name:group[name]["stage_finished_monotonic_ns"]
        require(all(clock(a)<=clock(b) for a,b in zip(NATIVE_STAGES[:7],NATIVE_STAGES[1:7])),"actual native statement clock order differs")
        invoked=phase=="driving" and index!=len(states)-1
        require(group["agent_branch_started"]["agent_run_step_invoked"] is invoked and group["agent_branch_finished"]["agent_run_step_invoked"] is invoked,"actual warmup/normal/final-goal branch differs")
        row=dict(ordinal=index,frame=frame,capture_phase=phase,agent_run_step_invoked=invoked)
        segments=("world_tick","snapshot_binding","snapshot_to_actor_state","actor_state_to_agent","agent_or_blank_brake","agent_to_state_finished")
        for name,a,b in zip(segments,NATIVE_STAGES[:6],NATIVE_STAGES[1:7]):row[name+"_ms"]=(clock(b)-clock(a))/1e6
        if camera:
            require(clock("native_state_finished")<=clock("exact_bundle_started")<=clock("exact_bundle_finished"),"native camera provider clock order differs")
            row["outer_exact_provider_ms"]=(clock("exact_bundle_finished")-clock("exact_bundle_started"))/1e6
        rows.append(row);by_frame[frame]=group
    return rows,by_frame


def critical_paths(rows,native_groups,quota,cameras,walls):
    require(quota["schema"]==LIVE_SCHEMA+".quota_walk_receipts" and len(quota["rows"])==len(cameras),"every actual quota walk required")
    q_by_frame={};quota_rows=[];wall_by_ACK={row["ACK_sequence"]:row for row in walls}
    for index,(q,camera) in enumerate(zip(quota["rows"],cameras)):
        require(q["sequence"]==index+1 and q["native_frame"]==camera["frame"] and q["status"]=="PASS" and q["complete_walk"] is True
            and q["timestamp_simulation_ns"]==int(round(camera["timestamp"]*1e9)) and q["capture_phase"]==camera["capture_phase"]
            and q["returned_regular_file_bytes"]==q["regular_file_bytes"] and q["source_payload_read"] is False
            and q["cached_or_incremental_byte_count_used"] is False and q["walk_started_monotonic_ns"]<=q["walk_finished_monotonic_ns"],"actual full-tree quota correspondence/count/clock differs")
        q_by_frame[q["native_frame"]]=q
        quota_rows.append(dict(ordinal=index,frame=q["native_frame"],capture_phase=q["capture_phase"],
            regular_files=q["regular_files"],regular_file_bytes=q["regular_file_bytes"],walk_ms=(q["walk_finished_monotonic_ns"]-q["walk_started_monotonic_ns"])/1e6))
    fresh=[]
    for original in rows:
        f=original["frame"];q=q_by_frame[f];n=native_groups[f];ack=wall_by_ACK[original["ACK_sequence"]];bundle=ack["observed_bundle"]
        clocks=dict(first=original["first_callback_ns"],last=original["last_callback_ns"],outer=n["exact_bundle_started"]["stage_finished_monotonic_ns"],
            quota_start=q["walk_started_monotonic_ns"],quota_end=q["walk_finished_monotonic_ns"],inner=bundle["exact_bundle_wait_started_monotonic_ns"],
            delivery=bundle["exact_bundle_delivery_monotonic_ns"],build=bundle["observed_frame_build_finished_monotonic_ns"],
            outer_end=n["exact_bundle_finished"]["stage_finished_monotonic_ns"],proposal=ack["proposal_started_monotonic_ns"],
            input=original["input_ready_ns"],NN=original["raw_NN_finished_ns"],plan=original["decoded_plan_ready_ns"],
            proposal_end=ack["proposal_finished_monotonic_ns"],ACK_start=ack["ACK_started_monotonic_ns"],ACK=original["ACK_finished_ns"])
        ordered=("outer","quota_start","quota_end","inner","delivery","build","outer_end","proposal","input","NN","plan","proposal_end","ACK_start","ACK")
        require(all(clocks[a]<=clocks[b] for a,b in zip(ordered,ordered[1:])) and clocks["first"]<=clocks["last"]<=clocks["delivery"],"complete actual outer-quota-inner-input-NN-decode-ACK chronology required")
        row=dict(original,quota_regular_files=q["regular_files"],quota_regular_file_bytes=q["regular_file_bytes"],clocks_monotonic_ns=clocks)
        segments=dict(camera_arrival_skew=("first","last"),first_callback_delivery=("first","delivery"),last_callback_delivery=("last","delivery"),
            outer_quota_start=("outer","quota_start"),quota_walk=("quota_start","quota_end"),quota_disk_dispatch=("quota_end","inner"),
            inner_queue_wait=("inner","delivery"),delivery_observed_build=("delivery","build"),observed_build_outer_end=("build","outer_end"),
            outer_end_proposal=("outer_end","proposal"),proposal_input=("proposal","input"),delivery_input=("delivery","input"),
            NN=("input","NN"),NN_decoded_plan=("NN","plan"),decoded_plan_proposal_end=("plan","proposal_end"),
            proposal_end_ACK_start=("proposal_end","ACK_start"),ACK_send=("ACK_start","ACK"),decoded_plan_ACK=("plan","ACK"))
        for name,(a,b) in segments.items():row[name+"_ms"]=(clocks[b]-clocks[a])/1e6
        row["quota_overlap_after_first_callback_ms"]=max(0,min(clocks["quota_end"],clocks["ACK"])-max(clocks["quota_start"],clocks["first"]))/1e6
        row["plan100ms_failed"]=row["first_callback_plan_ms"]>100.;row["ACK100ms_failed"]=row["first_callback_ACK_ms"]>100.
        fresh.append(row)
    return fresh,quota_rows


def metrics(rows):
    return {name:stats([row[name] for row in rows if name in row]) for name in rows[0] if name.endswith("_ms")}


def analyze():
    before=snapshot();original=json_file(RUN/"diagnostic_report.json")
    require(original.get("status")=="FAIL_KNOWN_DEV_FULL209_RESEARCH","original full209 FAIL must remain unchanged")
    value,quota=json_file(EPISODE/FILES[0]),json_file(EPISODE/FILES[1]);states,cameras,walls,commands=[json_lines(EPISODE/name) for name in FILES[2:6]]
    require(len(states)==1836 and len(cameras)==918 and len(walls)==1840,"exact complete original full209 denominator required")
    joined,census=source_join(value,cameras,commands,walls);require(len(joined)==883,"all883 actual fresh plans required")
    native,groups=native_paths(value,states,cameras,json_file(EPISODE/"route.json"));fresh,walks=critical_paths(joined,groups,quota,cameras,walls)
    branches={}
    for branch in ("ordinary_exact_delegate","authenticated_navigation_terminal_V2"):
        selected=[row for row in fresh if row["branch"]==branch]
        branches[branch]=dict(count=len(selected),plan100ms_failure_count=sum(row["plan100ms_failed"] for row in selected),
            ACK100ms_failure_count=sum(row["ACK100ms_failed"] for row in selected),metrics=metrics(selected))
    after=snapshot();require(before==after,"original inputs/sources changed during readonly diagnosis")
    return dict(schema=SCHEMA,provenance="HH_261004 - Independently retain all source joins and timing rows while preserving the failed actual owner's outcome.",
        original_status=original["status"],original_FAIL_not_regraded=True,input_source_seal_before=before,input_source_seal_after=after,
        input_source_unchanged=True,source_object_actor_stamp_and_past_SHA_join=census,all883_fresh_critical_path_rows=fresh,
        all1836_native_rows=native,all918_quota_rows=walks,branch_metrics=branches,all_fresh_metrics=metrics(fresh),all_native_metrics=metrics(native),
        all_camera_native_metrics=metrics([row for row in native if "outer_exact_provider_ms" in row]),all_quota_metrics=metrics(walks),
        native7N_plus2C_count=len(value["native_stage_marks"]),independent_native_order_and_actual_final_goal_branch_verified=True,
        final_native_frame=states[-1]["frame"],final_camera_frame=cameras[-1]["frame"],final_agent_run_step_invoked=False,
        plan100ms_failed_frames=[row["frame"] for row in fresh if row["plan100ms_failed"]],ACK100ms_failed_frames=[row["frame"] for row in fresh if row["ACK100ms_failed"]],
        worst_first_callback_ACK_rows=sorted(fresh,key=lambda row:row["first_callback_ACK_ms"],reverse=True)[:10],
        JPEG_pixels_used_for_analysis=False,NN_forward_executed=False,runtime_activated=False,model_promoted=False,formal_function_approval=False,
        internal_terminal_witness_decoder_GC_or_scheduling_cause_proven=False,individual_actor_RPC_duration_proven=False,
        actor_boundary_source_scope="acknowledged_snapshot_state plus FL/FR get_wheel_steer_angle and get_speed_limit before actor_state_ready",
        hardware_exposure_to_ACK_proven=False,normalization_candidate_activated=False)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument("--execute-readonly",action="store_true",required=True)
    parser.parse_args(argv);require(not OUTPUT.exists(),"fresh create-only readonly diagnosis output required")
    report=analyze();safe_path(OUTPUT,missing_tail=True);OUTPUT.mkdir(parents=True,exist_ok=False)
    write_new_json(OUTPUT/"report.json",report)
    summary={name:report[name] for name in ("original_status","original_FAIL_not_regraded","input_source_unchanged","source_object_actor_stamp_and_past_SHA_join", "branch_metrics","native7N_plus2C_count","plan100ms_failed_frames","ACK100ms_failed_frames")}
    summary.update(report_sha256=digest(OUTPUT/"report.json"),reproducer_sha256=digest(HERE),output=str(OUTPUT))
    write_new_json(OUTPUT/"summary.json",summary)
    print(json.dumps(summary,indent=2,sort_keys=True,allow_nan=False));return 0


if __name__=="__main__":raise SystemExit(main())
