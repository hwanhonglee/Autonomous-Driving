"""HH_261002 - Own one non-training manual-first coast diagnostic using the frozen private lifecycle."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import gzip
import json
import math
from pathlib import Path
import shutil
import subprocess
import struct
import sys

from scripts.e2e import research_temporal_v2_owned_camera_town07_20261001 as private
from scripts.e2e import calibrate_town07_manual_first_coast_20261002 as worker_module
from scripts.e2e import audit_simulation_research_body_lane_20261001 as geometry
from scripts.e2e import audit_carla_acknowledged_control as ack_math

ROOT,CARLA_ROOT,EGG,ROUTE,XODR=private.ROOT,private.CARLA_ROOT,private.EGG,private.ROUTE,private.XODR
SCHEMA=worker_module.SCHEMA+".owned_measurement"
MODULE="scripts.e2e.run_owned_town07_manual_first_coast_20261002"
ASSETS={path:private.ASSETS[path] for path in (ROUTE,XODR,EGG)}
require,digest,read_json,write_new,jsonl=private.require,private.digest,private.read_json,private.write_new,private.jsonl
_active=False


def paths(run_id):
    require(run_id == "v1","one exact calibration version required")
    output=ROOT/"storage/experiments/validation/2026-10-02/town07_manual_first_zero_pedal_coast_v1"
    require(output==output.resolve(),"canonical output parents without symlinks required")
    return output,output.with_name(output.name+"_plan.json")


def fixed_contract(run_id):
    output,_=paths(run_id)
    return dict(schema=SCHEMA,run_id=run_id,protocol=worker_module.protocol(),owner_output=str(output),
        route_file=str(ROUTE),route_sha256=ASSETS[ROUTE],xodr_sha256=ASSETS[XODR],python_api=str(EGG),
        server_command=["bash","tools/e2e/run_carla_map.sh","Town07","--port","2440","--quality","Epic",
            "--startup-timeout-sec","120","--","-RenderOffScreen","-nosound"],
        worker_command=[sys.executable,"-B",str(ROOT/"tools/workspace_python.py"),"-m",MODULE,"--","--mode","worker",
            "--run-id",run_id,"--plan-sha256",private.PLAN_TOKEN],owner_wall_timeout_s=300.,
        CLI_exit_zero="all declared measurement/source/body/quality checks passed; never TRAIN or vehicle approval",
        no_cameras_or_model_loading=True,training_data_approved=False,vehicle_control_approved=False)


def runtime_sources():
    # HH_261002 - Reuse deterministic import-only pins and explicitly freeze every reused legacy overlap.
    sources=private.imported_source_closure((Path(__file__).resolve(),ROOT/"tools/workspace_python.py",ROOT/"tools/canonical_workspace.py"))
    sources.update((ROOT/"tools/e2e/run_carla_map.sh",ROOT/"tools/e2e/env.sh",private.DEFAULT_CONTRACT_PATH,private.INHERITED_FREEZE))
    sources.update((ROOT/"learning/tests/test_town07_manual_first_coast_20261002.py",
        ROOT/"docs/assets/validation/2026-10-02/town07_manual_first_coast_v1/PRIOR_EVIDENCE.md"))
    pins={str(path.relative_to(ROOT)):digest(path) for path in sorted(sources)}
    require(digest(private.INHERITED_FREEZE) == private.INHERITED_FREEZE_SHA,"original source freeze changed")
    old=read_json(private.INHERITED_FREEZE)["source_sha256"]
    require(all(value == old[name] for name,value in pins.items() if name in old),"a frozen inherited source changed")
    require(all(pins.get(name)==value for name,value in private.FROZEN_GENERIC.items()),"frozen private dependencies changed")
    require(pins.get("tools/e2e/run_owned_town07_signed_recovery_v3_20261001.py") ==
        "01f6157bd61e32fe525166b4698fa70f8b82156843ecc1bb93edb538fd706a9a","frozen manual-first checker source changed")
    return pins


def disk_check():
    free=shutil.disk_usage(ROOT).free
    required=worker_module.FLOOR+worker_module.QUOTA+2*worker_module.RESERVE
    require(free >= required,"calibration quota plus floor and closing/archive reserve required")
    return dict(free_bytes=free,required_bytes=required)


def build_plan(run_id):
    return dict(fixed_contract(run_id),declared_at_utc=datetime.now(timezone.utc).isoformat(),
        source_sha256=runtime_sources(),external_source_sha256=private.external_sources(),
        asset_sha256={str(path):value for path,value in ASSETS.items()})


def verify_plan(run_id,expected):
    _,path=paths(run_id)
    require(isinstance(expected,str) and digest(path)==expected,"exact prior calibration declaration SHA required")
    plan=read_json(path); fixed=fixed_contract(run_id)
    require(set(plan)==set(fixed)|{"declared_at_utc","source_sha256","external_source_sha256","asset_sha256"}
        and all(plan[key]==value for key,value in fixed.items()),"fixed calibration declaration changed")
    stamp=datetime.fromisoformat(plan["declared_at_utc"])
    require(stamp.tzinfo is not None and stamp.utcoffset().total_seconds()==0 and stamp<=datetime.now(timezone.utc),"prior UTC declaration required")
    require(plan["source_sha256"]==runtime_sources() and plan["external_source_sha256"]==private.external_sources()
        and plan["asset_sha256"]=={str(path):value for path,value in ASSETS.items()},"declared source/API/asset closure changed")
    require(all(digest(name)==value for name,value in plan["asset_sha256"].items()),"declared asset bytes changed")
    return plan


def preflight(run_id,expected):
    plan=verify_plan(run_id,expected); output,_=paths(run_id)
    require(sys.version_info[:2]==(3,10),"pinned CARLA Python3.10 required")
    require(not output.exists() and not output.is_symlink(),"create-only calibration output required")
    private.lifecycle._assert_no_other_carla(); private.ports_free(private.PORT)
    disk=disk_check()
    result=subprocess.run([sys.executable,"-B","-c","import carla,agents,yaml; print(carla.__file__); print(agents.__file__)"],
        cwd=ROOT,env=private.environment(),text=True,capture_output=True,timeout=30,check=True)
    lines=result.stdout.strip().splitlines()
    require(len(lines)==2 and lines[0].startswith(str(EGG)+"/")
        and lines[1]==str(CARLA_ROOT/"PythonAPI/carla/agents/__init__.py"),"foreign API/agent source")
    return plan,dict(status="PREFLIGHT_PASS_NO_SIMULATOR",declaration_sha256=expected,disk=disk,
        simulator_started=False,model_loaded=False,camera_capture=False,training_data_approved=False)


def closure_check(output,plan):
    for name,value in plan["source_sha256"].items():
        require(digest(ROOT/name)==value==digest(output/"source"/name),"live/archive source changed: "+name)
    for section in ("external_source_sha256","asset_sha256"):
        for name,value in plan[section].items():
            path=Path(name); local=path.is_relative_to(ROOT)
            require(local or path.is_relative_to(CARLA_ROOT),"declared asset outside fixed roots")
            archived=output/("source" if local else "source_external")/path.relative_to(ROOT if local else CARLA_ROOT)
            require(digest(path)==value==digest(archived),"live/archive API or asset changed: "+name)


def validate_raw(rows,cleanup,receipts,result):
    """HH_261002 - Independently bind every actual observation to its same command ACK and raw snapshot math."""
    combined=rows+cleanup
    require(bool(rows) and bool(cleanup),"normal and separate cleanup observations required")
    require([row["sequence"] for row in receipts]==list(range(1,len(receipts)+1)),"ACK sequence gaps/duplicates")
    actors={row["actor_id"] for row in receipts}; require(actors=={result["actor_id"]},"ACK actor identity differs")
    for receipt in receipts:
        require(receipt.get("server_accepted") is True and receipt.get("status")=="ACKNOWLEDGED"
            and type(receipt["actor_id"]) is int and receipt["actor_id"]>0
            and type(receipt["response_actor_id"]) is int and receipt["response_actor_id"]==receipt["actor_id"]
            and receipt.get("do_tick") is False and receipt["after_ack_frame"]==receipt["before_frame"]
            and receipt["expected_before_frame"]==receipt["before_frame"],"failed/no-tick ACK cannot prove measurement")
    used=[]
    for row in combined:
        seq=row["receipt_sequence"]; require(type(seq) is int and 1<=seq<=len(receipts),"native ACK sequence invalid")
        receipt=receipts[seq-1]; used.append(seq)
        require(row["frame"]==receipt["before_frame"]+1 and receipt["reason"]==row["capture_phase"],"native frame/stage not bound to actual ACK")
        alignment=row["control_alignment"]
        require(alignment["status"]=="PASS" and alignment["frame_binding_pass"] is True
            and alignment["expected_receipt_sequence"]==seq and alignment["expected_command_before_frame"]==receipt["before_frame"]
            and alignment["observed_frame"]==alignment["control_read_frame_before"]==alignment["control_read_frame_after"]==row["frame"]
            and alignment["mismatched_fields"]==[],"native readback frame witness differs")
        require(not ack_math.mismatched_controls(row["current_control"],receipt["requested_control"])
            and not ack_math.mismatched_controls(row["requested_control"],receipt["requested_control"],requested_pair=True),"actual control differs from matching ACK")
        worker_module.measured_control_is_manual_first(row["current_control"],coast=row["capture_phase"]=="coast")
        state=ack_math.recompute_snapshot_state(row,2.85)
        require(all(abs(row[key]-state[key])<=1e-9 for key in state),"native rear-base state differs from actual immutable snapshot")
        require(abs(row["actor_world_planar_speed_mps"]-math.hypot(*row["world_velocity_carla"][:2]))<=1e-12,"actor world speed diagnostic differs")
    require(used==list(range(1,len(combined)+1)) and len(receipts)==len(combined),"normal/cleanup ACK denominator differs")
    worker_module.quality(combined)
    require(rows[0]["capture_phase"]=="bootstrap" and sum(row["capture_phase"]=="bootstrap" for row in rows)==1,
        "one actual bootstrap required")
    require([row["capture_phase"] for row in rows[1:25]]==["stationary_warmup"]*24,"full24 warmup ticks required")
    require(all(row["current_control"]["throttle"]==0. and row["current_control"]["brake"]==1. for row in rows[:25]),
        "bootstrap and every warmup must be actual full brake without throttle")
    witnesses=result["initialization_witnesses"]
    require(len(witnesses)==2,"both actual manual-first fullbrake initialization witnesses required")
    for witness,row in zip(witnesses,(rows[0],rows[24])):
        require(witness["status"]=="PASS" and witness["actor_id"]==result["actor_id"]
            and witness["expected_frame"]==witness["frame_before"]==witness["frame_after"]==row["frame"]
            and witness["timestamp"]==row["timestamp"] and witness["current_control"]==row["current_control"]
            and witness["actor_snapshot_transform_carla"]==row["actor_snapshot_transform_carla"]
            and row["current_control"]["throttle"]==0. and row["current_control"]["brake"]==1.
            and "pid_past_steering" not in witness,"actual manual1 initialization witness differs; no PID constructor claim")
    coast=[row for row in rows if row["capture_phase"]=="coast"]
    require(len(coast)==900 and rows[-900:]==coast,"all900 coast ticks must form the untrimmed fixed tail")
    preparation=rows[25:-900]
    require(bool(preparation) and all(row["capture_phase"]=="open_loop_launch" for row in preparation),"fixed launch phase membership differs")
    launch=worker_module.OpenLoopPreparation()
    for before,row in zip(rows[24:-900],preparation):
        throttle=launch.throttle(math.hypot(before["vx"],before["vy"]),before["timestamp"])
        actual_float32=struct.unpack("f",struct.pack("f",throttle))[0]
        require(row["requested_control"]["throttle"]==actual_float32 and row["requested_control"]["brake"]==0.,
            "declared open-loop preparation law differs from original float32 command")
    require(all(math.hypot(row["vx"],row["vy"])<1.5 for row in preparation[:-1])
        and math.hypot(preparation[-1]["vx"],preparation[-1]["vy"])>=1.5,"coast did not follow the first actual 1.5m/s crossing")
    require(result["entry"]["frame"]==preparation[-1]["frame"] and result["entry"]["timestamp"]==preparation[-1]["timestamp"]
        and result["entry"]["measured_rear_base_speed_mps"]==math.hypot(preparation[-1]["vx"],preparation[-1]["vy"]),"raw entry speed was rewritten")
    snapshots=result["physics_snapshots"]
    require(set(snapshots)=={"initial","coast_entry","coast_end"},"all three actual physics snapshots required")
    for name,row in (("initial",rows[24]),("coast_entry",preparation[-1]),("coast_end",coast[-1])):
        observed=snapshots[name]
        require(observed["frame"]==row["frame"] and observed["timestamp"]==row["timestamp"]
            and observed["actual_control"]==row["current_control"] and observed["internal_engine_or_wheel_RPM_measured"] is False,
            "actual physics snapshot frame/control witness differs")
        require(observed["physics"]["schema"]=="carla.vehicle_physics_snapshot.v1"
            and observed["physics"]["values"]["mass"]==1375. and len(observed["physics"]["values"]["wheels"])==4,
            "actual Prius physics snapshot is missing or inconsistent")
    require(all(row["capture_phase"]=="cleanup_full_brake" and row["current_control"]["brake"]==1. for row in cleanup)
        and cleanup[0]["frame"]==coast[-1]["frame"]+1,"cleanup brake entered before fixed coast ended")
    require(all(not any(row["events_seen"]) for row in combined),"actual collision/invasion event retained")
    require(all(math.hypot(row["vx"],row["vy"])<=2.5 and row["actor_world_planar_speed_mps"]<=2.5
        and row["route_cte_m"]<=.75 for row in rows),"native speed/CTE operational bound failed")
    require(result["native_motion_quality"]==worker_module.quality(rows),"stored whole preparation/coast quality differs")
    require(result.get("original_world_settings_restored") is True and result.get("original_weather_restored") is True,
        "original world settings/weather restoration not proven")
    return dict(status="PASS_RAW_BINDING_ONLY",normal_native_count=len(rows),coast_native_count=900,
        cleanup_native_count=len(cleanup),ACK_count=len(receipts),all_preparation_retained=True,training_data_approved=False)


def diagnostic_report(output,plan,owner):
    report=dict(schema=SCHEMA+".outcome",status="FAIL_RETAINED",ownership_passed=False,bounded_capture_complete=False,
        measurement_passed=False,learned_control=False,training_data_approved=False,vehicle_control_approved=False,
        full_RTE04_approved=False,main_functions_approved=0,subfunctions_approved=0,errors=[])
    folder=output/"calibration"; inputs={}
    def attempt(name,fn):
        try: report[name]=fn(); return report[name]
        except Exception as error: report["errors"].append(dict(check=name,error=type(error).__name__+": "+str(error))); return None
    ownership=not owner.get("cleanup_errors",["missing"]) and owner.get("source_hashes_match") is True and owner.get("external_source_hashes_match") is True
    report["ownership_passed"]=ownership
    result=attempt("raw_result",lambda:read_json(folder/"result.json"))
    manifest=attempt("raw_manifest",lambda:read_json(folder/"manifest.json"))
    rows=attempt("normal_states",lambda:jsonl(folder/"states.jsonl"))
    cleanup=attempt("cleanup_states",lambda:jsonl(folder/"cleanup_states.jsonl"))
    receipts=attempt("control_receipts",lambda:jsonl(folder/"control_receipts.jsonl"))
    if result is not None and manifest is not None:
        def raw_hashes():
            require(manifest["capture_contract"]["protocol"]==plan["protocol"]==result["protocol"],"raw protocol differs")
            require(set(manifest["raw_file_sha256"])=={"result.json","states.jsonl","cleanup_states.jsonl","control_receipts.jsonl","route.json","active_map_opendrive.xodr"},
                "all six original calibration artifacts must have original writer hashes")
            for name,value in manifest["raw_file_sha256"].items(): require(Path(name).name==name and digest(folder/name)==value,"raw artifact hash differs")
            inputs.update({str(folder/name):digest(folder/name) for name in ("manifest.json","result.json","states.jsonl","cleanup_states.jsonl","control_receipts.jsonl","route.json","active_map_opendrive.xodr")})
            return dict(status="PASS",source_capture_status_preserved=result["status"],input_sha256=inputs)
        attempt("raw_hash_check",raw_hashes)
    if all(value is not None for value in (rows,cleanup,receipts,result)):
        attempt("native_and_ACK_audit",lambda:validate_raw(rows,cleanup,receipts,result))
        attempt("native_motion_quality",lambda:worker_module.quality(rows))
        report["bounded_capture_complete"]=bool(result.get("measurement_completed") is True and sum(row["capture_phase"]=="coast" for row in rows)==900)
    if rows and cleanup and manifest is not None and ownership:
        def body_audit():
            require(digest(folder/"route.json")==plan["route_sha256"] and digest(folder/"active_map_opendrive.xodr")==plan["xodr_sha256"],"raw geometry source differs")
            saved=list(sys.path)
            try:
                sys.path.insert(0,str(EGG)); import carla
                require(str(carla.__file__).startswith(str(EGG)+"/"),"foreign offline body API")
                body,census=geometry.body_census(manifest,read_json(folder/"route.json"),rows+cleanup,
                    (folder/"active_map_opendrive.xodr").read_bytes(),result["actor_id"])
            finally: sys.path[:]=saved
            count=len(rows)+len(cleanup)
            expected={kind+"_"+region:count-(1 if kind=="previous_sweep" else 0)
                for kind in ("sampled_body","previous_sweep") for region in ("raw_lane","inset_10cm_lane")}
            require(len(body)==count and set(census)==set(expected)
                and all(census[name]["assessed_count"]==value for name,value in expected.items()),
                "full original body/10cm/sweep denominator differs")
            require(all(digest(name)==value for name,value in inputs.items()),"raw changed during body audit")
            closure_check(output,plan)
            derived=output/"derived_body"; derived.mkdir(exist_ok=False)
            with gzip.open(derived/"all_native_bodies_and_sweeps.jsonl.gz","xt") as stream:
                for row in body: stream.write(json.dumps(row,allow_nan=False)+"\n")
            outcome=dict(status="PASS" if all(row["status"]=="PASS" for row in census.values()) else "FAIL",
                full_native_count=len(rows)+len(cleanup),full_sweep_count=len(rows)+len(cleanup)-1,
                preparation_coast_cleanup_all_included=True,census=census,input_sha256=inputs,ten_centimeter_inset_preserved=True,
                physical_continuous_time_proven=False,training_data_approved=False)
            write_new(derived/"summary.json",outcome); return outcome
        attempt("whole_body_and_sweep",body_audit)
    attempt("full_source_closure",lambda:(closure_check(output,plan) or dict(status="PASS")))
    report["measurement_passed"]=bool(ownership and not report["errors"] and result is not None
        and result["status"]=="PASS_MEASURED_CALIBRATION_NOT_TRAIN" and type(owner.get("worker_exit_code")) is int and owner["worker_exit_code"]==0
        and report["bounded_capture_complete"] is True
        and report.get("native_motion_quality",{}).get("status")=="PASS_SCALAR_ONLY"
        and report.get("whole_body_and_sweep",{}).get("status")=="PASS")
    report["status"]="PASS_MEASURED_CALIBRATION_NOT_TRAIN" if report["measurement_passed"] else "FAIL_RETAINED"
    # HH_261002 - Keep large raw arrays in their existing files; the human outcome contains counts, pins and witnesses.
    for key in ("normal_states","cleanup_states","control_receipts"):
        if key in report: report[key+"_count"]=len(report.pop(key))
    return report


@contextmanager
def activated():
    global _active
    require(not _active,"non-nested coast owner required")
    changes=dict(paths=paths,preflight=preflight,diagnostic_report=diagnostic_report,SCHEMA=SCHEMA,
        CHECKPOINT_SHA=None,ASSETS=ASSETS,MODULE=MODULE)
    saved={name:getattr(private,name) for name in changes}; _active=True
    try:
        for name,value in changes.items():setattr(private,name,value)
        yield
    finally:
        for name,value in saved.items():setattr(private,name,value)
        _active=False


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument("--mode",choices=("contract","declare","preflight","execute","worker"),default="contract")
    parser.add_argument("--run-id",default="v1"); parser.add_argument("--plan-sha256")
    args=parser.parse_args(argv)
    if args.mode=="contract":print(json.dumps(fixed_contract(args.run_id),indent=2));return 0
    if args.mode=="declare":
        output,path=paths(args.run_id); require(not output.exists() and not path.exists() and not path.is_symlink(),"fresh version required")
        disk_check(); plan=build_plan(args.run_id); path.parent.mkdir(parents=True,exist_ok=True);write_new(path,plan)
        print(json.dumps(dict(declaration_path=str(path),declaration_sha256=digest(path),simulator_started=False)));return 0
    if args.mode=="preflight":_,result=preflight(args.run_id,args.plan_sha256);print(json.dumps(result,indent=2));return 0
    if args.mode=="execute":
        with activated():private.execute(args.run_id,args.plan_sha256)
        output,_=paths(args.run_id)
        return 0 if read_json(output/"diagnostic_report.json")["measurement_passed"] is True else 2
    plan=verify_plan(args.run_id,args.plan_sha256); output,_=paths(args.run_id)
    owned=dict(plan,output=plan["owner_output"])
    result=worker_module.run(output/"calibration",plan,lambda:private.assert_owned_worker(owned,args.plan_sha256))
    return 0 if result["status"]=="PASS_MEASURED_CALIBRATION_NOT_TRAIN" else 2


if __name__=="__main__":raise SystemExit(main())
