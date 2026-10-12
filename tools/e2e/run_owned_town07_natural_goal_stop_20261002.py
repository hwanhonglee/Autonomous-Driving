"""HH_261002 - Prospectively own ONE full-route normal goal-stop teacher pilot; no learned feature claim."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import sys

from scripts.e2e import run_owned_town07_signed_recovery_20261001 as previous
from scripts.e2e import collect_town07_natural_goal_stop_20261002 as worker_module
from scripts.e2e import town07_natural_goal_stop_task_20261002 as task
from scripts.e2e import qualify_town07_signed_recovery_common10_20261001 as initialization

ROOT, lifecycle, ASSETS = previous.ROOT, previous.lifecycle, previous.ASSETS
require,digest,read_json,write_new,jsonl = previous.require,previous.digest,previous.read_json,previous.write_new,previous.jsonl
MODULE="scripts.e2e.run_owned_town07_natural_goal_stop_20261002"
SCHEMA=task.SCHEMA+".owned_pilot"
CASE_IDS=("pilot_00",)
_parent_sources=previous.runtime_sources
_active=False


def capture_argv(run_id,case_id,plan_sha=lifecycle.PLAN_TOKEN):
    output,plan=task.paths(run_id,case_id)
    return ["--natural-goal-run-id",run_id,str(output/"episode"),str(task.ROUTE),"--host","127.0.0.1","--port","2440",
        "--control-transport","acknowledged_batch","--agent-initialization","after_bootstrap",
        "--stationary-warmup-sec","1.2","--stationary-tail-sec","6.5","--max-duration-sec","140",
        "--target-speed-kmh","8.64","--seed",str(task.SEED),"--weather","ClearNoon",
        "--mapping",str(previous.MAPPING),"--calibration",str(previous.CALIBRATION),"--spawn-z-offset-m",".5",
        "--wheelbase-m","2.85","--expected-live-xodr-sha256",ASSETS[previous.XODR],"--goal-stop-profile","comfortable_v1",
        "--goal-tolerance-m","1","--basic-agent-base-min-distance-m","2","--basic-agent-distance-ratio",".2",
        "--moving-task-plan",str(plan),"--moving-task-plan-sha256",plan_sha]


def execution_contract(run_id,case_id):
    output,_=task.paths(run_id,case_id)
    return dict(owner_output=str(output),server_command=["bash","tools/e2e/run_carla_map.sh","Town07","--port","2440",
        "--quality","Epic","--startup-timeout-sec","120","--","-RenderOffScreen","-nosound"],
        worker_command=[sys.executable,"-B",str(ROOT/"tools/workspace_python.py"),"-m",MODULE,"--","--mode","worker",
            "--run-id",run_id,"--case-id",case_id,"--case-plan-sha256",lifecycle.PLAN_TOKEN],owner_wall_timeout_s=600.)


def runtime_sources():
    worker_module.manual.verify_frozen_dependencies()
    pins=_parent_sources()
    for path in lifecycle.imported_source_closure((Path(__file__).resolve(),)):
        pins[str(path.relative_to(ROOT))]=digest(path)
    require(all(pins.get(name)==value for name,value in lifecycle.FROZEN_GENERIC.items()),"frozen private lifecycle dependency changed")
    return dict(sorted(pins.items()))


def disk_check(free=None):
    free=shutil.disk_usage(ROOT).free if free is None else free
    required=task.FLOOR+task.QUOTA+2*task.RESERVE
    require(type(free) is int and free>=required,"one full pilot plus floor/closing/archive reserve required")
    return dict(free_bytes=free,required_bytes=required,prospective_cases=1,reserved_case_bytes=task.QUOTA)


def diagnostic_report(output,plan,owner):
    episode=output/"episode"
    if not episode.is_dir():episode=output/"episode.partial"
    report=dict(schema=SCHEMA+".teacher_outcome",case_id="pilot_00",status="FAIL_RETAINED",original_owner=owner,
        ownership_passed=False,bounded_capture_complete=False,actual_teacher_capture_passed=False,
        learned_control=False,training_data_approved=False,future_labels_qualified=False,model_promoted=False,
        vehicle_control_approved=False,full_RTE04_approved=False,main_functions_approved=0,subfunctions_approved=0,
        teacher_completion_is_not_learned_goal_stop=True,attempt_count=1,automatic_retry=False)
    try:
        manifest=read_json(episode/"manifest.json")
        states,cameras,receipts=(jsonl(episode/name) for name in ("states.jsonl","camera_frames.jsonl","control_receipts.jsonl"))
        ownership=not owner["cleanup_errors"] and owner["source_hashes_match"] and owner["external_source_hashes_match"]
        report.update(ownership_passed=bool(ownership),native_status=manifest.get("status"),native_error=manifest.get("error"),
            native_state_count=len(states),camera_bundle_count=len(cameras),ACK_receipt_count=len(receipts))
        require(states and cameras and receipts,"retained complete raw denominators required")
        capture=manifest["capture_contract"]
        require(capture["natural_goal_stop_teacher"]==plan and capture["goal_stop_profile"]["profile_id"]==task.PROFILE,
            "exact prospectively declared natural-goal teacher identity required")
        transport=lifecycle.independent_audit.analyze_transport(states,receipts,capture["control_transport"]["bootstrap_observation"],wheelbase=2.85)
        camera=lifecycle.independent_audit.analyze_camera_alignment(states,cameras)
        native_chain=all(b["frame"]==a["frame"]+1 and abs(b["timestamp"]-a["timestamp"]-.05)<=1e-6 for a,b in zip(states,states[1:]))
        camera_chain=all(b["frame"]==a["frame"]+2 and abs(b["timestamp"]-a["timestamp"]-.1)<=1e-6 for a,b in zip(cameras,cameras[1:]))
        pixels=previous.pixel_audit(episode,cameras)
        route=read_json(episode/"route.json")
        replay=task.reconstruct(plan,states,cameras,route["route"])
        tail=sum(row["capture_phase"]=="stationary_tail" for row in states)
        moving=replay.result(tail_states=tail,goal_reached=manifest.get("result",{}).get("goal_reached",False))
        require(moving==manifest["result"]["moving_task"],"event-defined membership changed")
        initialized=initialization.initialization_audit(manifest,states,receipts)
        config=task.goal_configuration(type("Args",(),task.FIXED_ARGS)())
        quality=worker_module.base.measured_stop_quality(states,[r["frame"] for r in cameras],config,
            capture["goal_stop_profile"]["bounds"],manifest.get("result",{}).get("goal_reached",False))
        require(quality==manifest["result"]["goal_stop_quality"],"original measured normal-stop quality changed")
        body=previous.body_report(output,plan,episode,states,receipts,manifest) if ownership else dict(status="NOT_PROVEN",all_native_body_and_swept_clearance_passed=False)
        completion=manifest["result"]
        bounded=manifest.get("status")=="complete" and moving["task_complete"] and tail==130 \
            and completion["termination_reason"]=="comfortable_goal_measured_stop_and_dwell" and quality["status"]=="PASS"
        passed=ownership and bounded and native_chain and camera_chain and transport["status"]==camera["status"]==initialized["status"]=="PASS" \
            and body["all_native_body_and_swept_clearance_passed"]
        report.update(status="PASS_ACTUAL_TEACHER_GOAL_CAPTURE_PENDING_FUTURE_QA" if passed else "FAIL_RETAINED",
            actual_teacher_capture_passed=bool(passed),bounded_capture_complete=bool(bounded),
            independent_transport_audit=transport,independent_camera_alignment_audit=camera,
            independently_observed_manual_first_gear=initialized,full_capture_body_clearance=body,
            original_measured_stop_quality=quality,event_defined_membership=moving,actual_camera_source_pixel_audit=pixels,
            native_chain_passed=native_chain,camera_chain_passed=camera_chain,
            future64_labels_not_yet_qualified=True,undefined_near_standstill_curvature_not_relabelled_PASS=True,
            rear_base_catalog_goal_not_bumper_stop_line=True)
        previous.closure_check(output,plan)
    except Exception as error:
        report.update(status="FAIL_RETAINED",actual_teacher_capture_passed=False,error=type(error).__name__+": "+str(error))
    return report


@contextmanager
def activated():
    global _active
    require(not _active and not previous._active,"natural goal owner activation cannot nest")
    names=dict(task=task,worker_module=worker_module,MODULE=MODULE,SCHEMA=SCHEMA,CASE_IDS=CASE_IDS,
        capture_argv=capture_argv,execution_contract=execution_contract,runtime_sources=runtime_sources,
        disk_check=disk_check,diagnostic_report=diagnostic_report)
    saved={k:getattr(previous,k) for k in names}
    _active=True
    try:
        for k,v in names.items():setattr(previous,k,v)
        yield
    finally:
        for k,v in saved.items():setattr(previous,k,v)
        _active=False


def declare(run_id):
    output,path=task.paths(run_id)
    require(not output.exists() and not path.exists(),"fresh one-pilot create-only declaration required")
    disk=disk_check()
    plan=previous.build_plan(run_id,"pilot_00")
    output.mkdir(parents=True,exist_ok=False)
    _,case_path=task.paths(run_id,"pilot_00")
    write_new(case_path,plan)
    batch=dict(schema=SCHEMA,run_id=run_id,declared_at_utc=datetime.now(timezone.utc).isoformat(),attempts=1,
        automatic_retry=False,cases=[dict(case_id="pilot_00",declaration_path=str(case_path),declaration_sha256=digest(case_path))],
        preflight_disk=disk,training_data_approved=False,learned_control=False,failed_attempt_retained=True)
    write_new(path,batch)
    return dict(batch_plan=str(path),batch_plan_sha256=digest(path),case_plan=str(case_path),case_plan_sha256=digest(case_path),simulator_started=False)


def verify_batch(run_id,expected):
    _,path=task.paths(run_id)
    require(isinstance(expected,str) and digest(path)==expected,"one prior pilot batch SHA required")
    batch=read_json(path)
    require(batch["schema"]==SCHEMA and batch["run_id"]==run_id and batch["attempts"]==1 and batch["automatic_retry"] is False
        and len(batch["cases"])==1 and batch["cases"][0]["case_id"]=="pilot_00","single prospective pilot denominator required")
    spec=batch["cases"][0]
    require(spec["declaration_path"]==str(task.paths(run_id,"pilot_00")[1]),"fixed pilot plan path required")
    previous.verify_case(run_id,"pilot_00",spec["declaration_sha256"])
    return spec


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument("--mode",choices=("contract","declare","preflight","execute","worker","summary"),default="contract")
    parser.add_argument("--run-id",choices=("v1",),default="v1")
    parser.add_argument("--case-id",choices=CASE_IDS,default="pilot_00")
    parser.add_argument("--plan-sha256");parser.add_argument("--case-plan-sha256")
    args=parser.parse_args(argv)
    with activated():
        if args.mode=="contract":result=task.fixed_contract(args.run_id,"pilot_00")
        elif args.mode=="declare":result=declare(args.run_id)
        elif args.mode=="worker":return previous.worker(args.run_id,"pilot_00",args.case_plan_sha256)
        else:
            spec=verify_batch(args.run_id,args.plan_sha256)
            if args.mode=="summary":
                output,_=task.paths(args.run_id,"pilot_00")
                report=output/"diagnostic_report.json"
                result=read_json(report) if report.is_file() else dict(status="NOT_EXECUTED",attempts=1,learned_control=False,training_data_approved=False)
            else:
                with previous.scoped_case("pilot_00"):
                    if args.mode=="execute":return lifecycle.execute(args.run_id,spec["declaration_sha256"])
                    _,result=previous.preflight(args.run_id,spec["declaration_sha256"])
    print(json.dumps(result,indent=2,allow_nan=False))
    return 0


if __name__=="__main__":raise SystemExit(main())
