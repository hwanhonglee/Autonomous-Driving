"""HH_261002 - Own one prospective gentle goal-stop teacher pilot; complete failure-preserving diagnostics."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import sys

from scripts.e2e import run_owned_town07_signed_recovery_20261001 as previous
from scripts.e2e import run_owned_town07_natural_goal_stop_20261002 as baseline
from scripts.e2e import collect_town07_natural_goal_stop_v2_20261002 as worker_module
from scripts.e2e import town07_natural_goal_stop_task_v2_20261002 as task
from scripts.e2e import qualify_town07_signed_recovery_common10_20261001 as initialization

ROOT, lifecycle, ASSETS = previous.ROOT, previous.lifecycle, previous.ASSETS
require,digest,read_json,write_new,jsonl = previous.require,previous.digest,previous.read_json,previous.write_new,previous.jsonl
MODULE = "scripts.e2e.run_owned_town07_natural_goal_stop_v2_20261002"
SCHEMA = task.SCHEMA+".owned_pilot"
CASE_IDS = ("pilot_00",)
_parent_sources = baseline.runtime_sources
_active = False
QUALITY_FAILURE = "CollectionError: comfortable goal-stop measured quality failed; preserve partial evidence"


def capture_argv(run_id,case_id,plan_sha=lifecycle.PLAN_TOKEN):
    output,plan = task.paths(run_id,case_id)
    return ["--natural-goal-run-id",run_id,str(output/"episode"),str(task.ROUTE),"--host","127.0.0.1","--port","2440",
        "--control-transport","acknowledged_batch","--agent-initialization","after_bootstrap",
        "--stationary-warmup-sec","1.2","--stationary-tail-sec","6.5","--max-duration-sec","140",
        "--target-speed-kmh","8.64","--seed",str(task.SEED),"--weather","ClearNoon",
        "--mapping",str(previous.MAPPING),"--calibration",str(previous.CALIBRATION),"--spawn-z-offset-m",".5",
        "--wheelbase-m","2.85","--expected-live-xodr-sha256",ASSETS[previous.XODR],"--goal-stop-profile","comfortable_v1",
        "--goal-tolerance-m","1","--basic-agent-base-min-distance-m","2","--basic-agent-distance-ratio",".2",
        "--moving-task-plan",str(plan),"--moving-task-plan-sha256",plan_sha]


def execution_contract(run_id,case_id):
    output,_ = task.paths(run_id,case_id)
    return dict(owner_output=str(output),server_command=["bash","tools/e2e/run_carla_map.sh","Town07","--port","2440",
        "--quality","Epic","--startup-timeout-sec","120","--","-RenderOffScreen","-nosound"],
        worker_command=[sys.executable,"-B",str(ROOT/"tools/workspace_python.py"),"-m",MODULE,"--","--mode","worker",
            "--run-id",run_id,"--case-id",case_id,"--case-plan-sha256",lifecycle.PLAN_TOKEN],owner_wall_timeout_s=600.)


def runtime_sources():
    task.verify_frozen()
    worker_module.manual.verify_frozen_dependencies()
    pins = _parent_sources()
    for path in lifecycle.imported_source_closure((Path(__file__).resolve(),)):
        pins[str(path.relative_to(ROOT))] = digest(path)
    require(all(pins.get(name) == value for name,value in lifecycle.FROZEN_GENERIC.items()), "frozen private lifecycle changed")
    require(all(pins.get(name) == value for name,value in task.FROZEN_V1.items()), "frozen goal v1 closure changed")
    return dict(sorted(pins.items()))


def disk_check(free=None):
    free = shutil.disk_usage(ROOT).free if free is None else free
    required = task.FLOOR+task.QUOTA+2*task.RESERVE
    require(type(free) is int and free >= required, "one full v2 pilot plus floor/closing/archive reserve required")
    return dict(free_bytes=free,required_bytes=required,prospective_cases=1,reserved_case_bytes=task.QUOTA)


def compare_membership(stored,replayed,manifest,owner,quality,quality_matches):
    """HH_261002 - Normalize only the post-capture failure marker for comparison; never repair raw or grant PASS."""
    if stored == replayed:
        return dict(status="PASS_EXACT",normalization_applied=False,raw_source_result=stored,replayed_result=replayed,
            source_capture_failure_preserved=manifest.get("status") != "complete")
    expected_failure = manifest.get("status") == "failed" and manifest.get("error") == QUALITY_FAILURE \
        and type(owner.get("worker_exit_code")) is int and owner["worker_exit_code"] != 0 \
        and quality.get("status") == "FAIL" and quality_matches is True \
        and stored.get("task_complete") is False and stored.get("failure_reason") == "moving_task_capture_or_cleanup_failed" \
        and replayed.get("task_complete") is True and replayed.get("failure_reason") is None
    require(expected_failure, "event-defined membership differs beyond an authenticated post-capture quality failure")
    comparison = dict(stored,task_complete=replayed["task_complete"],failure_reason=replayed["failure_reason"])
    require(comparison == replayed, "only terminal task_complete/failure_reason comparison markers may differ")
    return dict(status="PASS_COMPARISON_ONLY_SOURCE_CAPTURE_FAILED",normalization_applied=True,
        normalized_comparison_fields=["task_complete","failure_reason"],raw_source_result=stored,replayed_result=replayed,
        source_capture_failure_preserved=True,source_failure_reason=stored["failure_reason"],
        original_native_quality_passed=False,training_authorization=False)


def capture_passed(manifest,owner,checks,tail):
    # HH_261002 - Neither marker normalization nor a real goal HOLD can erase failed native quality/worker status.
    required = ("capture_identity","transport","camera","native_chain","camera_chain","pixels","membership",
        "initialization","quality_matches","body","source_closure")
    return bool(manifest.get("status") == "complete" and type(owner.get("worker_exit_code")) is int and owner["worker_exit_code"] == 0
        and checks.get("ownership") is True and all(checks.get(name) is True for name in required)
        and checks.get("quality_passed") is True and checks.get("raw_task_complete") is True and tail == 130
        and manifest.get("result",{}).get("termination_reason") == "comfortable_goal_measured_stop_and_dwell")


def diagnostic_report(output,plan,owner):
    episode = output/"episode"
    if not episode.is_dir():episode = output/"episode.partial"
    report = dict(schema=SCHEMA+".teacher_outcome",case_id="pilot_00",status="FAIL_RETAINED",original_owner=owner,
        ownership_passed=False,bounded_capture_complete=False,actual_teacher_capture_passed=False,
        learned_control=False,training_data_approved=False,future_labels_qualified=False,model_promoted=False,
        vehicle_control_approved=False,full_RTE04_approved=False,main_functions_approved=0,subfunctions_approved=0,
        formal_feature_denominators=dict(main=9,sub=30),teacher_completion_is_not_learned_goal_stop=True,
        attempt_count=1,automatic_retry=False,diagnostic_errors=[],all_available_raw_checks_attempted=True)
    results,checks = {},{}

    def attempt(name,operation):
        try:
            results[name] = operation()
            return results[name]
        except Exception as error:
            results[name] = dict(status="NOT_PROVEN",error=type(error).__name__+": "+str(error))
            report["diagnostic_errors"].append(dict(check=name,error=results[name]["error"]))
            return None

    manifest = attempt("manifest",lambda:read_json(episode/"manifest.json"))
    states = attempt("states",lambda:jsonl(episode/"states.jsonl"))
    cameras = attempt("cameras",lambda:jsonl(episode/"camera_frames.jsonl"))
    receipts = attempt("receipts",lambda:jsonl(episode/"control_receipts.jsonl"))
    route = attempt("route",lambda:read_json(episode/"route.json"))
    checks["ownership"] = not owner.get("cleanup_errors",["missing"]) and owner.get("source_hashes_match") is True \
        and owner.get("external_source_hashes_match") is True
    report.update(ownership_passed=bool(checks["ownership"]),native_state_count=len(states) if states is not None else None,
        camera_bundle_count=len(cameras) if cameras is not None else None,ACK_receipt_count=len(receipts) if receipts is not None else None)
    if manifest is not None:
        report.update(native_status=manifest.get("status"),native_error=manifest.get("error"))
    if manifest is not None and states and cameras and receipts:
        capture = manifest.get("capture_contract",{})
        def identity():
            require(capture["natural_goal_stop_teacher"] == plan and capture["goal_stop_profile"]["profile_id"] == task.PROFILE,
                "exact prior v2 plan and governor identity required")
            metadata = capture["teacher_controller"]
            require(metadata["normal_brake_cap"] == .01 and metadata["normal_throttle_cap"] == .20
                and metadata["prospective_desired_deceleration_mps2"] == .2, "actual v2 teacher metadata differs")
            return True
        checks["capture_identity"] = attempt("capture_identity",identity) is True
        transport = attempt("transport",lambda:lifecycle.independent_audit.analyze_transport(states,receipts,
            capture["control_transport"]["bootstrap_observation"],wheelbase=2.85))
        camera = attempt("camera",lambda:lifecycle.independent_audit.analyze_camera_alignment(states,cameras))
        checks["transport"] = transport is not None and transport.get("status") == "PASS"
        checks["camera"] = camera is not None and camera.get("status") == "PASS"
        checks["native_chain"] = attempt("native_chain",lambda:all(b["frame"] == a["frame"]+1
            and abs(b["timestamp"]-a["timestamp"]-.05) <= 1e-6 for a,b in zip(states,states[1:]))) is True
        checks["camera_chain"] = attempt("camera_chain",lambda:all(b["frame"] == a["frame"]+2
            and abs(b["timestamp"]-a["timestamp"]-.1) <= 1e-6 for a,b in zip(cameras,cameras[1:]))) is True
        checks["pixels"] = attempt("pixels",lambda:previous.pixel_audit(episode,cameras)) is not None
        initialized = attempt("initialization",lambda:initialization.initialization_audit(manifest,states,receipts))
        checks["initialization"] = initialized is not None and initialized.get("status") == "PASS"
        tail = sum(row.get("capture_phase") == "stationary_tail" for row in states)
        quality = attempt("quality",lambda:worker_module.base.measured_stop_quality(states,[r["frame"] for r in cameras],
            task.goal_configuration(type("Args",(),task.FIXED_ARGS)()),capture["goal_stop_profile"]["bounds"],
            manifest.get("result",{}).get("goal_reached",False)))
        checks["quality_matches"] = quality is not None and quality == manifest.get("result",{}).get("goal_stop_quality")
        checks["quality_passed"] = checks["quality_matches"] and quality.get("status") == "PASS"
        checks["raw_task_complete"] = manifest.get("result",{}).get("moving_task",{}).get("task_complete") is True
        def membership():
            require(route is not None,"actual route unavailable")
            replay = task.reconstruct(plan,states,cameras,route["route"])
            moving = replay.result(tail_states=tail,goal_reached=manifest.get("result",{}).get("goal_reached",False))
            return compare_membership(manifest["result"]["moving_task"],moving,manifest,owner,quality or {},checks["quality_matches"])
        checks["membership"] = attempt("membership",membership) is not None
        # HH_261002 - Membership/quality failure never skips the full native body/sweep denominator.
        body = attempt("body",lambda:previous.body_report(output,plan,episode,states,receipts,manifest)) if checks["ownership"] \
            else dict(status="NOT_PROVEN",all_native_body_and_swept_clearance_passed=False)
        if not checks["ownership"]:results["body"] = body
        checks["body"] = body is not None and body.get("all_native_body_and_swept_clearance_passed") is True
        checks["source_closure"] = attempt("source_closure",lambda:previous.closure_check(output,plan)) is not None
        # closure_check returns no value on success; absence of its exception is the actual proof.
        checks["source_closure"] = "source_closure" in results and not any(e["check"] == "source_closure" for e in report["diagnostic_errors"])
        passed = capture_passed(manifest,owner,checks,tail)
        report.update(status="PASS_ACTUAL_TEACHER_GOAL_CAPTURE_PENDING_FUTURE_QA" if passed else "FAIL_RETAINED",
            actual_teacher_capture_passed=passed,bounded_capture_complete=passed,stationary_tail_state_count=tail)
    report.update(checks=checks,independent_raw_audits={name:value for name,value in results.items()
        if name not in ("manifest","states","cameras","receipts","route")},
        future64_labels_not_yet_qualified=True,undefined_near_standstill_curvature_not_relabelled_PASS=True,
        rear_base_catalog_goal_not_bumper_stop_line=True)
    return report


@contextmanager
def activated():
    global _active
    require(not _active and not previous._active,"v2 goal owner activation cannot nest")
    names = dict(task=task,worker_module=worker_module,MODULE=MODULE,SCHEMA=SCHEMA,CASE_IDS=CASE_IDS,
        capture_argv=capture_argv,execution_contract=execution_contract,runtime_sources=runtime_sources,
        disk_check=disk_check,diagnostic_report=diagnostic_report)
    saved = {name:getattr(previous,name) for name in names}
    _active = True
    try:
        for name,value in names.items():setattr(previous,name,value)
        yield
    finally:
        for name,value in saved.items():setattr(previous,name,value)
        _active = False


def declare(run_id):
    output,path = task.paths(run_id)
    require(not output.exists() and not path.exists(),"fresh one-pilot v2 create-only declaration required")
    disk = disk_check()
    plan = previous.build_plan(run_id,"pilot_00")
    output.mkdir(parents=True,exist_ok=False)
    _,case_path = task.paths(run_id,"pilot_00")
    write_new(case_path,plan)
    batch = dict(schema=SCHEMA,run_id=run_id,declared_at_utc=datetime.now(timezone.utc).isoformat(),attempts=1,
        automatic_retry=False,cases=[dict(case_id="pilot_00",declaration_path=str(case_path),declaration_sha256=digest(case_path))],
        preflight_disk=disk,training_data_approved=False,learned_control=False,failed_attempt_retained=True)
    write_new(path,batch)
    return dict(batch_plan=str(path),batch_plan_sha256=digest(path),case_plan=str(case_path),case_plan_sha256=digest(case_path),simulator_started=False)


def verify_batch(run_id,expected):
    _,path = task.paths(run_id)
    require(isinstance(expected,str) and digest(path) == expected,"one prior v2 batch SHA required")
    batch = read_json(path)
    require(batch["schema"] == SCHEMA and batch["run_id"] == run_id and batch["attempts"] == 1 and batch["automatic_retry"] is False
        and len(batch["cases"]) == 1 and batch["cases"][0]["case_id"] == "pilot_00","single prospective pilot denominator required")
    spec = batch["cases"][0]
    require(spec["declaration_path"] == str(task.paths(run_id,"pilot_00")[1]),"fixed v2 pilot plan path required")
    previous.verify_case(run_id,"pilot_00",spec["declaration_sha256"])
    return spec


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument("--mode",choices=("contract","declare","preflight","execute","worker","summary"),default="contract")
    parser.add_argument("--run-id",choices=("v2",),default="v2")
    parser.add_argument("--case-id",choices=CASE_IDS,default="pilot_00")
    parser.add_argument("--plan-sha256");parser.add_argument("--case-plan-sha256")
    args = parser.parse_args(argv)
    with activated():
        if args.mode == "contract":result = task.fixed_contract(args.run_id,"pilot_00")
        elif args.mode == "declare":result = declare(args.run_id)
        elif args.mode == "worker":return previous.worker(args.run_id,"pilot_00",args.case_plan_sha256)
        else:
            spec = verify_batch(args.run_id,args.plan_sha256)
            if args.mode == "summary":
                output,_ = task.paths(args.run_id,"pilot_00")
                report = output/"diagnostic_report.json"
                result = read_json(report) if report.is_file() else dict(status="NOT_EXECUTED",attempts=1,learned_control=False,training_data_approved=False)
            else:
                with previous.scoped_case("pilot_00"):
                    if args.mode == "execute":return lifecycle.execute(args.run_id,spec["declaration_sha256"])
                    _,result = previous.preflight(args.run_id,spec["declaration_sha256"])
    print(json.dumps(result,indent=2,allow_nan=False))
    return 0


if __name__ == "__main__":raise SystemExit(main())
