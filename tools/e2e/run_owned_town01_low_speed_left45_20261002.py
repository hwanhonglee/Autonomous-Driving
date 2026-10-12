"""HH_261002 - One fresh owned Town01 LEFT teacher pilot; no actual execution or data admission is implicit."""
from __future__ import annotations

import argparse
import ast
import copy
from contextlib import contextmanager
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import shutil
import sys

from scripts.e2e import run_owned_town07_signed_recovery_20261001 as parent
from scripts.e2e import town01_low_speed_left45_task_20261002 as task
from scripts.e2e import collect_town01_low_speed_left45_20261002 as worker_module
from scripts.e2e import qualify_town07_signed_recovery_common10_20261001 as manual_witness

ROOT, CARLA_ROOT, EGG, PORT = parent.ROOT, parent.CARLA_ROOT, parent.EGG, parent.PORT
lifecycle = parent.lifecycle
MAPPING, CALIBRATION = parent.MAPPING, parent.CALIBRATION
XODR = ROOT/"storage/experiments/training/2026-09-27/drive_long_route_town01_right_s0002_p06_9kph_steer045_trial_v1/episode/active_map_opendrive.xodr"
NOMINAL = ROOT/"storage/experiments/diagnostics/2026-10-02/town01_train_turn_nominal_census_v2/summary.json"
ASSETS = {MAPPING: parent.ASSETS[MAPPING], CALIBRATION: parent.ASSETS[CALIBRATION], task.ROUTE: task.ROUTE_SHA,
    XODR: "97a7f6ac67812567e5c8ee0599cd823b23f80f30f3f97c502212e38b72e2b709",
    NOMINAL: "94943ff013c17b3c80017d1ac4538dad0cdddc9c62b5c6063722688b8d598475"}
MODULE = "scripts.e2e.run_owned_town01_low_speed_left45_20261002"
SCHEMA = task.SCHEMA+".owned"
CASE_IDS = ("pilot_00",)
PARENT_SHA = "d452e3901e42ced1af894e1ef329b6142ed10860de02cc0a4a2512f0022f500b"
require, digest, read_json, write_new, jsonl = parent.require, parent.digest, parent.read_json, parent.write_new, parent.jsonl
geometry = parent.geometry
subprocess = parent.subprocess
re, gzip = parent.re, parent.gzip
_case, _active = "pilot_00", False


def paths(run_id): return task.paths(run_id, "pilot_00")


def runtime_sources():
    worker_module.verify_frozen()
    require(digest(parent.__file__) == PARENT_SHA, "immutable owner provider changed")
    sources = lifecycle.imported_source_closure((Path(__file__).resolve(), ROOT/"tools/workspace_python.py", ROOT/"tools/canonical_workspace.py"))
    sources.update((worker_module.signals.SOURCE, ROOT/"tools/e2e/run_carla_map.sh", ROOT/"tools/e2e/env.sh",
        lifecycle.DEFAULT_CONTRACT_PATH, lifecycle.INHERITED_FREEZE))
    pins = {str(path.relative_to(ROOT)):digest(path) for path in sorted(sources)}
    require(digest(lifecycle.INHERITED_FREEZE)==lifecycle.INHERITED_FREEZE_SHA, "inherited freeze changed")
    inherited = read_json(lifecycle.INHERITED_FREEZE)["source_sha256"]
    require(all(name not in inherited or sha==inherited[name] for name,sha in pins.items()), "inherited scientific source changed")
    require(all(pins.get(name)==sha for name,sha in lifecycle.FROZEN_GENERIC.items()), "frozen lifecycle source missing/changed")
    require(pins.get("tools/e2e/town01_train_left45_green_fixture_20261002.py") == "b9a198f1c37398a424e096490d5de51df68caf87c9dde97759b7693a592efaab", "frozen Town01 signal adapter changed")
    return dict(sorted(pins.items()))


def disk_check(free=None):
    free = shutil.disk_usage(ROOT).free if free is None else free
    required = task.FLOOR+task.QUOTA+2*task.RESERVE
    require(type(free) is int and free >= required, "one full3GiB pilot plus5GiB floor and two closing reserves required")
    return dict(free_bytes=free, required_bytes=required, prospective_cases=1, reserved_case_bytes=task.QUOTA)


def execution_contract(run_id, case_id):
    output,_ = task.paths(run_id,case_id)
    return dict(owner_output=str(output), server_command=["bash", "tools/e2e/run_carla_map.sh", "Town01", "--port", "2440",
        "--quality", "Epic", "--startup-timeout-sec", "120", "--", "-RenderOffScreen", "-nosound"],
        worker_command=[sys.executable,"-B",str(ROOT/"tools/workspace_python.py"),"-m",MODULE,"--","--mode","worker",
            "--run-id",run_id,"--case-id",case_id,"--case-plan-sha256",lifecycle.PLAN_TOKEN], owner_wall_timeout_s=600.)


def adapted_originals():
    # HH_261002 - Use unchanged declaration/ownership/pixel/body algorithms under exact fresh asset/provider globals, never mutate parent globals.
    require(digest(parent.__file__)==PARENT_SHA,"owner source changed before private compilation")
    names=("capture_argv","build_plan","verify_case","preflight","closure_check","pixel_audit","body_report","diagnostic_report")
    tree=ast.parse(Path(parent.__file__).read_bytes()); nodes=[]
    for name in names:
        found=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name==name]
        require(len(found)==1,"unique original owner function required: "+name)
        node=copy.deepcopy(found[0])
        if name=="capture_argv":
            values=[n for n in ast.walk(node) if isinstance(n,ast.Constant) and n.value=="50"]
            require(len(values)==1,"unique inherited driving budget literal required"); values[0].value="60"
        if name=="diagnostic_report":
            counts={903:0,452:0}
            for value in ast.walk(node):
                if isinstance(value,ast.Constant) and type(value.value) is int and value.value in counts:
                    old=value.value; counts[old]+=1; value.value=task.NATIVE_COUNT if old==903 else task.CAMERA_COUNT
            require(counts=={903:2,452:2},"only exact native/camera census denominators may change")
        nodes.append(node)
    namespace=dict(globals(), diagnostic_report=None)
    exec(compile(ast.fix_missing_locations(ast.Module(body=nodes,type_ignores=[])),__file__,"exec"),namespace)
    return namespace


_originals=adapted_originals()
for _name in ("capture_argv","build_plan","verify_case","preflight","closure_check","pixel_audit","body_report"):
    globals()[_name]=_originals[_name]


def motion_quality(states):
    require(len(states)>1,"all native rates require actual states")
    origin=next(row["frame"] for row in states if row["capture_phase"]=="driving")
    violations, physical, rates=[],[],[]
    for a,b in zip(states,states[1:]):
        dt=b["timestamp"]-a["timestamp"]
        require(math.isfinite(dt) and abs(dt-.05)<=1e-6,"actual native cadence differs")
        va,vb=math.hypot(a["vx"],a["vy"]),math.hypot(b["vx"],b["vy"])
        require(all(math.isfinite(x) for x in (va,vb)),"nonfinite measured rear-base speed")
        rate=(vb-va)/dt; rates.append(rate)
        reasons=[]
        if rate>3.+1e-6 or rate < -6.-1e-6: reasons.append("all_native_scalar_dvdt_outside_minus6_plus3")
        if max(va,vb)>2.5: reasons.append("actual_native_rear_base_speed_exceeds2p5")
        protected=origin+61<=a["frame"] and b["frame"]<=origin+900
        if abs(rate)>2.9+1e-6:
            physical.append(dict(before_frame=a["frame"],after_frame=b["frame"],scalar_dvdt_mps2=rate,
                protected_history_or_true_future=protected,capture_phase=b["capture_phase"]))
            if protected: reasons.append("protected_native_scalar_dvdt_exceeds_physical2p9")
        if reasons: violations.append(dict(before_frame=a["frame"],after_frame=b["frame"],scalar_dvdt_mps2=rate,
            before_speed_mps=va,after_speed_mps=vb,capture_phase=b["capture_phase"],reasons=reasons))
    return dict(status="PASS_SCALAR_ONLY" if not violations else "FAIL",all_native_interval_count=len(rates),
        scalar_dvdt_min_mps2=min(rates),scalar_dvdt_max_mps2=max(rates),violations=violations,
        all_native_abs2p9_diagnostic_violations=physical,
        scalar_reference="Unchanged actual rear-base hypot(vx,vy) finite difference; NOT actor API acceleration norm.",
        no_startup_or_tail_runtime_exemption=True,true_future64_qualification_not_performed=True)


def turn_screen(states,route):
    points=route["route"]; block=route["physical_turn_preflight"]["selected_block"]
    end=block["end_index"]
    exit_arc=sum(math.hypot(b["x"]-a["x"],b["y"]-a["y"]) for a,b in zip(points[:end+1],points[1:end+1]))
    driving=[row for row in states if row["capture_phase"]=="driving"]
    require(bool(driving),"actual turn requires driving states")
    before_end=[row for row in driving if row["frame"]<=driving[0]["frame"]+task.LAST_FRAME]
    yaw=0.; previous=driving[0]["yaw"]; earliest=None
    for row in before_end:
        yaw+=math.atan2(math.sin(row["yaw"]-previous),math.cos(row["yaw"]-previous));previous=row["yaw"]
        progress=task.measured_progress(points,row["x"],row["y"])
        if earliest is None and math.degrees(yaw)>=60. and progress>=exit_arc+10.:
            earliest=dict(frame=row["frame"],timestamp=row["timestamp"],signed_actual_yaw_change_deg=math.degrees(yaw),
                actual_measured_route_progress_m=progress)
    selected=[row for row in before_end if row["frame"]>=driving[0]["frame"]+task.FIRST_FRAME and (row["frame"]-driving[0]["frame"])%2==0]
    length=sum(math.hypot(b["x"]-a["x"],b["y"]-a["y"]) for a,b in zip(points,points[1:]))
    support=bool(selected) and all(length-task.measured_progress(points,row["x"],row["y"])>=18. for row in selected)
    return dict(status="PASS" if earliest is not None and support and len(selected)==task.ANCHORS else "FAIL",
        earliest_measured_exit_plus10_and_60degree_witness=earliest,original_encoded_exit_index=end,
        exact_route_XY_exit_arc_m=exit_arc,minimum_causal_navigation_support_m=18.,all_fixed_anchors_route_support_passed=support,
        selected_native_denominator=len(selected),not_traffic_rule_or_learned_turn_approval=True)


def teacher_command_audit(commands,receipts,states):
    # HH_261002 - Independently replay the unchanged normal manual law; tail/closing ACKs are real context, never STOP labels.
    ack=manual_witness.ack
    require([row["ack_receipt"] for row in commands]==receipts,"full teacher/ACK command denominator differs")
    by_frame={row["frame"]:row for row in states}
    policy=worker_module.manual.FirstGearRecoveryLaunchControl(); proposals=0; hazards=[]
    expected_normal_frames=[states[23]["frame"]]+[row["frame"] for row in states if row["capture_phase"]=="driving"][:-1]
    actual_normal_frames=[]
    coast_reasons={"driving_end_brake","next_stationary_tail_control"}
    for row in commands:
        receipt=row["ack_receipt"];frame=receipt["before_frame"];reason=row["reason"]
        require(row["learned_control"] is False and row["teacher_control"] is True
            and not ack.mismatched_controls(row["final_BasicAgent_or_brake_proposal"],receipt["requested_control"]),"actual teacher proposal/ACK differs")
        normal=row["normal_policy"]
        if reason in ("initial_drive_control","next_driving_control") and frame in expected_normal_frames:
            target=2.4 if proposals==0 else min(2.4,.075*proposals); proposals+=1
            ramp=row["target_ramp"]
            require(ramp["source_frame"]==frame and math.isclose(ramp["actual_teacher_target_mps"],target,abs_tol=1e-12)
                and ramp["teacher_target_cap_mps"]==2.4 and ramp["actual_observed_speed_clipped"] is False,"actual normal target ramp differs")
            actual_normal_frames.append(frame)
            if normal is None:
                require(receipt["requested_control"]["throttle"]==0. and receipt["requested_control"]["brake"]>0.,"missing normal request is not unchanged hazard brake")
                hazards.append(frame);continue
            state=by_frame[frame];speed=math.hypot(*state["world_velocity_carla"][:2])
            require(normal["source_frame"]==frame and normal["source_timestamp"]==state["timestamp"]
                and normal["measured_planar_speed_mps"]==speed,"normal source is not same-frame actual actor speed")
            raw=normal["raw_normal_planner_proposal"]
            control=manual_witness.native_math.RecordedFloatControl(dict(raw,throttle=min(.20,raw["throttle"]),brake=0.))
            policy.apply(control,speed=speed,timestamp=state["timestamp"],frame=frame,normal_proposal=raw)
            policy.last["schema"]=task.SCHEMA+".actual_normal_teacher_control"
            require(policy.last==normal,"actual first-gear normal teacher replay differs")
            if ack.mismatched_controls(normal["profile_normal_control"],receipt["requested_control"]):
                require(receipt["requested_control"]["throttle"]==0. and receipt["requested_control"]["brake"]>0.,"normal result changed beyond downstream original hazard")
                hazards.append(frame)
        if reason in coast_reasons or (reason=="next_driving_control" and frame==states[924]["frame"]):
            control=ack.full_control(receipt["requested_control"])
            require(control==dict(throttle=0.,brake=0.,steer=0.,gear=1,manual_gear_shift=True,reverse=False,hand_brake=False),
                "diagnostic coast ACK is not exact zero-pedal manual1")
    require(actual_normal_frames==expected_normal_frames and proposals==901,"warmup-engagement plus900 real normal proposals required")
    tail=[row for row in states if row["capture_phase"]=="stationary_tail"]
    require(len(tail)==130 and all(ack.full_control(row["current_control"])==dict(throttle=0.,brake=0.,steer=0.,gear=1,
        manual_gear_shift=True,reverse=False,hand_brake=False) for row in tail),"all130 actual coast observations required")
    closing=[row for row in receipts if row["reason"]=="left45_post_capture_cleanup_full_brake"]
    require(len(closing)==1 and closing[0]==receipts[-1] and closing[0]["before_frame"]==states[-1]["frame"]
        and closing[0]["requested_control"]["throttle"]==closing[0]["requested_control"]["steer"]==0.
        and closing[0]["requested_control"]["brake"]==1.,"separate post-last-pose cleanup full brake required")
    return dict(status="PASS",all_actual_command_count=len(commands),normal_proposal_count=proposals,
        original_hazard_override_frames=hazards,diagnostic_coast_observations=130,
        post_capture_cleanup_receipt_sequence=closing[0]["sequence"],no_recorded_stopped_pose_fabricated=True)


def diagnostic_report(output,plan,owner):
    result=_originals["diagnostic_report"](output,plan,owner)
    episode=output/"episode" if (output/"episode").is_dir() else output/"episode.partial"
    try:
        manifest=read_json(episode/"manifest.json"); states=jsonl(episode/"states.jsonl"); receipts=jsonl(episode/"control_receipts.jsonl")
        quality=motion_quality(states); turn=turn_screen(states,read_json(episode/"route.json"))
        commands=teacher_command_audit(jsonl(episode/"actual_teacher_control.jsonl"),receipts,states)
        init=manual_witness.initialization_audit(manifest,states,receipts)
        fixture=read_json(episode/"owned_green_signal_fixture.json")
        green=fixture==manifest["capture_contract"]["owned_green_signal_fixture"] \
            and fixture["state"]=="RESTORED" and fixture["original_state_freeze_and_duration_restored"] is True
        require(fixture["fixture_adapter_schema"]==worker_module.signals.SCHEMA,"exact Town01 Green fixture schema required")
        complete=result["bounded_capture_complete"]
        passed=result["actual_teacher_capture_passed"] and complete and quality["status"]=="PASS_SCALAR_ONLY" \
            and turn["status"]==init["status"]==commands["status"]=="PASS" and green \
            and type(owner.get("worker_exit_code")) is int and owner["worker_exit_code"]==0
        result.update(status="PASS_ACTUAL_TEACHER_PENDING_TRUE_FUTURE_QA" if passed else "FAIL_RETAINED",
            actual_teacher_capture_passed=bool(passed),all_native_motion_quality=quality,actual_left_turn_screen=turn,
            actual_manual_first_initialization=init,controlled_signal_fixture_passed=green,
            independent_actual_teacher_commands=commands,
            diagnostic_coast_tail_not_stationary_or_STOP_TRAIN=True)
        closure_check(output,plan)
    except Exception as error:
        result.update(status="FAIL_RETAINED",actual_teacher_capture_passed=False,additional_audit_error=type(error).__name__+": "+str(error))
    return result


@contextmanager
def activated():
    global _active
    require(not _active,"one LEFT45 owner activation cannot nest"); runtime_sources()
    changes=dict(paths=paths,preflight=preflight,diagnostic_report=diagnostic_report,SCHEMA=SCHEMA,
        CHECKPOINT_SHA=None,ASSETS=ASSETS,MODULE=MODULE)
    saved={name:getattr(lifecycle,name) for name in changes}; _active=True
    try:
        for name,value in changes.items(): setattr(lifecycle,name,value)
        yield
    finally:
        for name,value in saved.items(): setattr(lifecycle,name,value)
        _active=False


def declare(run_id):
    batch,path=task.paths(run_id); output,case_path=task.paths(run_id,"pilot_00")
    require(not batch.exists() and not path.exists(),"fresh one-attempt pilot declaration required")
    disk=disk_check(); plan=build_plan(run_id,"pilot_00")
    nominal=read_json(NOMINAL); item=next(row for row in nominal["results"] if row["case_id"]=="town01_left_s0002_p03")
    require(item["status"]=="NOMINAL_CLEAR_NOT_DRIVING_PROOF" and item["source_route_sha256"]==task.ROUTE_SHA,
        "original whole-route nominal clearance witness changed")
    batch.mkdir(parents=True,exist_ok=False); write_new(case_path,plan)
    receipt=dict(schema=SCHEMA,run_id=run_id,case_id="pilot_00",case_plan_sha256=digest(case_path),case_plan_path=str(case_path),
        attempts=1,automatic_retry=False,preflight_disk=disk,nominal_clearance_not_actual_training_approval=True,
        training_data_approved=False,vehicle_control_approved=False,learned_control=False)
    write_new(path,receipt)
    return dict(receipt,batch_plan_path=str(path),batch_plan_sha256=digest(path),simulator_started=False)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument("--mode",choices=("contract","declare","preflight","execute","worker","summary"),default="contract")
    parser.add_argument("--run-id",default="v1"); parser.add_argument("--case-id",choices=CASE_IDS,default="pilot_00")
    parser.add_argument("--case-plan-sha256"); args=parser.parse_args(argv)
    if args.mode=="contract": result=dict(task.fixed_contract(args.run_id,args.case_id),**execution_contract(args.run_id,args.case_id))
    elif args.mode=="declare": result=declare(args.run_id)
    elif args.mode=="summary": result=read_json(paths(args.run_id)[0]/"diagnostic_report.json")
    elif args.mode=="worker":
        plan=verify_case(args.run_id,args.case_id,args.case_plan_sha256)
        lifecycle.assert_owned_worker(dict(plan,output=plan["owner_output"]),args.case_plan_sha256)
        parsed=worker_module.parse_args(capture_argv(args.run_id,args.case_id,args.case_plan_sha256))
        with worker_module.scope(parsed): print(worker_module.base.run(parsed))
        return 0
    else:
        with activated():
            if args.mode=="execute":
                operational=lifecycle.execute(args.run_id,args.case_plan_sha256)
                report=read_json(paths(args.run_id)[0]/"diagnostic_report.json")
                return 0 if operational==0 and report["actual_teacher_capture_passed"] else 2
            _,result=preflight(args.run_id,args.case_plan_sha256)
    print(json.dumps(result,indent=2,allow_nan=False)); return 0


if __name__=="__main__": raise SystemExit(main())
