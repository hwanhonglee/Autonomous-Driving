"""HH_261002 - Prepared GOOD6000 versus fixed LEFT20k rolling TRAIN-route diagnosis; ROOT alone executes.

Both plans precede either capture. All failures remain denominators. This is not
independent validation, full-route completion, STOP performance or promotion.
"""
from __future__ import annotations

import argparse
import ast
import copy
from contextlib import contextmanager
from datetime import datetime, timezone
import gzip
import json
import math
import re
from pathlib import Path
import shutil
import sys
from types import SimpleNamespace

import torch

from portable_e2e import temporal_left_joint_numeric_loader_20261002 as loader
from . import temporal_left_joint_runtime_contract_20261002 as runtime_contract
from . import research_temporal_route_joint_signed45_town07_20261002 as numeric
from . import research_temporal_route_joint_thread4_signed45_town07_20261002 as threads
from . import run_owned_town01_low_speed_left45_v2_20261002 as teacher_owner
from . import research_temporal_route_joint_preview05_signed45_town07_20261002 as preview_owner
from . import collect_temporal_left_joint_left45_20261002 as live

ROOT, base = live.ROOT, teacher_owner.lifecycle
CARLA_ROOT, EGG, PORT = base.CARLA_ROOT, base.EGG, base.PORT
SCHEMA, MODULE = live.OWNER_SCHEMA, "scripts.e2e.research_temporal_left_joint_left45_20261002"
ARMS = live.ARMS
MAPPING, CALIBRATION, XODR = teacher_owner.MAPPING, teacher_owner.CALIBRATION, teacher_owner.XODR
geometry, subprocess = teacher_owner.geometry, teacher_owner.subprocess
require,digest,read_json,write_new,jsonl=base.require,base.digest,base.read_json,base.write_new,base.jsonl
_scope,_active,_owner_plan_sha=None,False,None
task,ASSETS,_originals=None,None,None
_case="pilot_00"
_external_sources=base.external_sources
_environment=base.environment
_batch_sha=None
parent_loader_schema=numeric.live.LOADER_SCHEMA
parent_owner_schema=numeric.OWNER_SCHEMA
CURRENT_PARENT=numeric.RUN/"exports/step_006000"


def root_path():return ROOT/"storage/experiments/validation/2026-10-02/temporal_left_joint_left45_v1"
def batch_path():return root_path()/"batch_plan.json"


def paths(run_id,case_id=None):
    require(run_id=="v1" and case_id in (None,"pilot_00") and _scope is not None,"fresh fixed LEFT comparison arm required")
    return root_path()/_scope.arm,root_path()/(_scope.arm+"_plan.json")


def authority(path,sha):
    path=loader.canonical(path)
    require(path.is_relative_to(ROOT/"storage/experiments/plans/2026-10-02"),"separate ROOT completed20k authority required")
    value=loader.read_json(path,sha)
    require(value.get("schema")==loader.AUTHORITY_SCHEMA and value.get("mode")=="RESEARCH_ONLY"
        and value.get("fixed_endpoint_completed") is True and value.get("parent")==loader.PARENT
        and value.get("parent_history_sha256")==loader.PARENT_HISTORY_SHA
        and value.get("new_LEFT_conversion_sha256")==loader.LEFT_CONVERSION_SHA
        and value.get("model_promoted") is False and value.get("vehicle_control_approved") is False,
        "actual fixed20k completion/GOODparent evidence must precede either physical plan")
    run,parent=loader.canonical(value["run"]),loader.canonical(value["parent_run"])
    require(run.is_relative_to(ROOT/"storage/experiments/training/2026-10-02") and parent==numeric.RUN
        and run!=parent and set(value["endpoint"])==set(loader.PARENT)
        and all(loader.is_sha(v) for v in value["endpoint"].values()),"exact separate actual final20k/four pins required")
    d=loader.read_json(run/"exports/step_020000/metadata.json",value["endpoint"]["metadata_sha256"]);loader.verify_scope(d)
    require(d["source_sha256"]==value["source_sha256"] and len(value["source_sha256"])==30
        and d["npz_sha256"]==value["endpoint"]["weights_sha256"]
        and d["checkpoint_sha256"]==value["endpoint"]["checkpoint_sha256"]
        and d["declaration_sha256"]==value["endpoint"]["declaration_sha256"],"actual source30/final20k metadata differs")
    return path,value,run


def evidence_assets():
    value,run=_scope.authority,_scope.run
    result=dict(teacher_owner.ASSETS)
    result[_scope.authority_path]=_scope.authority_sha
    for relative,key in (("summary.json","summary_sha256"),("fit_result.json","fit_result_sha256"),
        ("declaration.json","declaration_sha256"),("evaluation_opened_after_final.json","evaluation_boundary_sha256"),
        ("history.jsonl","history_sha256")):
        require(loader.is_sha(value[key]),"all actual completion/journal pins required");result[run/relative]=value[key]
    for folder,pins,step in ((run,value["endpoint"],20000),(numeric.RUN,loader.PARENT,6000)):
        result.update({folder/f"exports/step_{step:06d}/model_state.npz":pins["weights_sha256"],
            folder/f"exports/step_{step:06d}/metadata.json":pins["metadata_sha256"],
            folder/f"checkpoints/step_{step:06d}.pt":pins["checkpoint_sha256"],folder/"declaration.json":pins["declaration_sha256"]})
    result[numeric.RUN/"history.jsonl"]=loader.PARENT_HISTORY_SHA
    parent=read_json(CURRENT_PARENT/"metadata.json");warm=read_json(numeric.WARM_PARENT/"metadata.json")
    for folder,d in ((numeric.WARM_PARENT,parent),(numeric.BACKBONE_PARENT,warm)):
        result[folder/"model_state.npz"]=d["parent_npz_sha256"];result[folder/"metadata.json"]=d["parent_metadata_sha256"]
    for name,pin in value["source_sha256"].items():
        relative=Path(name);require(not relative.is_absolute() and ".." not in relative.parts and str(relative)==name,"canonical training-source key required")
        result[run/"source"/relative]=pin
    require(all(digest(path)==pin for path,pin in result.items()),"source/model/actual journal/route asset changed")
    return result


def runtime_sources():
    sources=base.imported_source_closure((Path(__file__).resolve(),ROOT/"tools/workspace_python.py",ROOT/"tools/canonical_workspace.py"))
    sources.update((ROOT/"tools/e2e/run_carla_map.sh",ROOT/"tools/e2e/env.sh",base.DEFAULT_CONTRACT_PATH,base.INHERITED_FREEZE))
    if _scope is not None:sources.update(ROOT/name for name in _scope.authority["source_sha256"])
    pins={str(path.relative_to(ROOT)):digest(path) for path in sorted(sources)}
    require(digest(base.INHERITED_FREEZE)==base.INHERITED_FREEZE_SHA,"inherited freeze changed")
    inherited=read_json(base.INHERITED_FREEZE)["source_sha256"]
    require(all(name not in inherited or pin==inherited[name] for name,pin in pins.items())
        and all(pins.get(name)==pin for name,pin in base.FROZEN_GENERIC.items()),"frozen numerical/ownership dependency changed")
    if _scope is not None:require(all(pins.get(name)==pin for name,pin in _scope.authority["source_sha256"].items()),"all30 actual fit sources required")
    return pins


def external_sources():
    result=_external_sources();old=CARLA_ROOT/"CarlaUE4/Content/Carla/Maps/Town07.umap"
    require(str(old) in result,"original external map asset guard changed");result.pop(str(old))
    actual=CARLA_ROOT/"CarlaUE4/Content/Carla/Maps/Town01.umap"
    result[str(actual)]=digest(actual);return result


def environment():
    # HH_261002 - This isolated worker inherits the externally pinned BOTH-plan identity and explicit4/1 CPU preset.
    result=_environment();result.update(OMP_NUM_THREADS="4",MKL_NUM_THREADS="4",OPENBLAS_NUM_THREADS="4")
    if _batch_sha is not None:result["LEFT45_COMPARISON_BATCH_SHA256"]=_batch_sha
    return result


def decorate_task_contract(result):
    value=dict(result,arm=_scope.arm,arm_order=list(ARMS),completed_run_authority_sha256=_scope.authority_sha,
        selected_four_endpoint_sha256=dict(_scope.pins),source_checkpoint_sha256=_scope.pins["checkpoint_sha256"],
        fixed_endpoint_updates=6000 if _scope.arm=="parent6000" else 20000,
        known_total_optimizer_updates=12200 if _scope.arm=="parent6000" else 32200,
        actual_training_source_count=26 if _scope.arm=="parent6000" else 30,
        original_export_false_flags_preserved=True,semantic_outputs_preserved=False,stop_intent_trained=False,
        no_fit_or_new_TRAIN_admission_during_physical_comparison=True)
    return value


def execution_contract(run_id,case_id):
    output,_=paths(run_id,case_id)
    return dict(owner_output=str(output),server_command=["bash","tools/e2e/run_carla_map.sh","Town01","--port","2440",
        "--quality","Epic","--startup-timeout-sec","120","--","-RenderOffScreen","-nosound"],
        worker_command=[sys.executable,"-B",str(ROOT/"tools/workspace_python.py"),"-m",MODULE,"--","--mode","worker",
            "--run-id",run_id,"--arm",_scope.arm,"--case-plan-sha256",base.PLAN_TOKEN,
            "--completed-run-authority",str(_scope.authority_path),"--completed-run-authority-sha256",_scope.authority_sha],
        owner_wall_timeout_s=600.)


def disk_check(free=None):
    value=shutil.disk_usage(ROOT).free if free is None else free
    asset_bytes=sum(path.stat().st_size for path in ASSETS)
    required=base.FLOOR+2*(base.EPISODE_QUOTA+asset_bytes+base.CLOSING_RESERVE)+base.SHARED_RESERVE
    require(type(value) is int and value>=required,"BOTH entire case quotas/source archives plus unchanged5GiB floor required")
    return dict(free_bytes=value,required_bytes=required,predeclared_cases=2,episode_quota_bytes=base.EPISODE_QUOTA,
        per_case_source_asset_archive_bytes=asset_bytes)


def capture_argv(run_id,case_id,plan_sha=base.PLAN_TOKEN):
    # HH_261002 - Original LEFT/manual45+130 source CLI plus separately pinned neural identities; no fresh route or teacher labels.
    args=_originals["original_capture_argv"](run_id,case_id,plan_sha)
    args.extend(["--research-weights",str(_scope.weights),"--research-metadata",str(_scope.metadata),
        "--research-weights-sha256",_scope.pins["weights_sha256"],"--research-metadata-sha256",_scope.pins["metadata_sha256"],
        "--research-source-sha256",plan_sha,"--research-longitudinal-pi"])
    return args


def adapted_originals():
    require(digest(teacher_owner.parent.__file__)==teacher_owner.PARENT_SHA,"frozen owner source changed")
    # HH_261002 - Original functions retain their lifecycle API names, but bind the exact NEW Town01 asset query, never Town07 globals.
    lifecycle=SimpleNamespace(**dict(vars(base),external_sources=external_sources,environment=environment))
    namespace=dict(globals(),lifecycle=lifecycle,task=task,worker_module=SimpleNamespace(base=live.collector,parse_args=lambda argv:live.parse_args(argv,sys.modules[__name__])))
    definitions=[];tree=ast.parse(Path(teacher_owner.parent.__file__).read_bytes())
    for name in ("capture_argv","build_plan","verify_case","preflight","closure_check","pixel_audit","body_report"):
        node=copy.deepcopy(next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name==name))
        if name=="capture_argv":
            hits=[n for n in ast.walk(node) if isinstance(n,ast.Constant) and n.value=="50"]
            require(len(hits)==1,"exact original ceiling literal required");hits[0].value="60"
        if name=="body_report":
            hits=[n for n in ast.walk(node) if isinstance(n,ast.Constant) and n.value=="actual_teacher_control.jsonl"]
            require(len(hits)==1,"exact frozen raw control-journal pin required");hits[0].value="control_receipts.jsonl"
        definitions.append(node)
    exec(compile(ast.fix_missing_locations(ast.Module(body=definitions,type_ignores=[])),__file__,"exec"),namespace)
    namespace["original_capture_argv"]=namespace["capture_argv"];namespace["capture_argv"]=capture_argv
    return namespace


def load_model(plan_sha=None):
    pins=_scope.pins;threads.configure_threads()
    purpose="owned_carla_research_only" if plan_sha is not None else "offline_cpu_numeric_validation_only"
    if _scope.arm=="LEFT20000":
        binding=runtime_contract.owner_binding(pins,_scope.authority_sha,owner_id=MODULE,purpose=purpose,owner_plan_sha256=plan_sha)
        model=loader.load_left_joint_research_npz(_scope.weights,_scope.metadata,weights_sha256=pins["weights_sha256"],metadata_sha256=pins["metadata_sha256"],
            current_parent_weights=CURRENT_PARENT/"model_state.npz",current_parent_metadata=CURRENT_PARENT/"metadata.json",
            warm_parent_weights=numeric.WARM_PARENT/"model_state.npz",warm_parent_metadata=numeric.WARM_PARENT/"metadata.json",
            backbone_parent_weights=numeric.BACKBONE_PARENT/"model_state.npz",backbone_parent_metadata=numeric.BACKBONE_PARENT/"metadata.json",
            completed_run_authority=_scope.authority_path,completed_run_authority_sha256=_scope.authority_sha,research_owner_binding=binding,source_root=ROOT)
    else:
        binding=dict(schema=numeric.OWNER_SCHEMA,mode="RESEARCH_ONLY",owner_id=MODULE,purpose=purpose,**pins,
            fixed_endpoint_updates=6000,simulator_control_authorized=plan_sha is not None,owner_plan_sha256=plan_sha,
            simulation_only=True,model_promoted=False,vehicle_control_approved=False)
        model=numeric.load_route_joint_research_npz(_scope.weights,_scope.metadata,weights_sha256=pins["weights_sha256"],metadata_sha256=pins["metadata_sha256"],
            parent_weights=numeric.WARM_PARENT/"model_state.npz",parent_metadata=numeric.WARM_PARENT/"metadata.json",
            backbone_parent_weights=numeric.BACKBONE_PARENT/"model_state.npz",backbone_parent_metadata=numeric.BACKBONE_PARENT/"metadata.json",
            research_owner_binding=binding,source_root=ROOT)
    model._research_cpu_thread_numerical_revision=live.THREAD_SCHEMA;model._research_speed_preview_calibration=live.PREVIEW_SCHEMA
    threads.live.thread_readback()
    return model


def projection(points,x,y):
    # HH_261002 - Full connected XY route projection with NO monotonic progress clamp; regressions are real evidence.
    arc=0.;candidates=[]
    for a,b in zip(points,points[1:]):
        dx,dy=b["x"]-a["x"],b["y"]-a["y"];length=math.hypot(dx,dy)
        if length==0.:continue
        u=max(0.,min(1.,((x-a["x"])*dx+(y-a["y"])*dy)/(length*length)))
        ex,ey=x-(a["x"]+u*dx),y-(a["y"]+u*dy)
        candidates.append((ex*ex+ey*ey,arc+u*length,(dx*ey-dy*ex)/length,math.atan2(dy,dx)))
        arc+=length
    require(bool(candidates) and all(math.isfinite(v) for row in candidates for v in row),"finite full original route projection required")
    return min(candidates,key=lambda row:(row[0],row[1]))[1:]


def percentile(values,fraction):
    require(bool(values),"nonempty actual metric denominator required")
    data=sorted(values);position=(len(data)-1)*fraction;lo=int(position);hi=min(lo+1,len(data)-1)
    return data[lo]+(data[hi]-data[lo])*(position-lo)


def left_motion_report(states,route,decisions,receipts):
    points=route["route"];block=route["physical_turn_preflight"]["selected_block"]
    start,end=block["start_index"],block["end_index"]
    require(type(start) is int and type(end) is int and 0<=start<end<len(points),"exact encoded LEFT block required")
    arcs=[0.]
    for a,b in zip(points,points[1:]):arcs.append(arcs[-1]+math.hypot(b["x"]-a["x"],b["y"]-a["y"]))
    headings=[math.atan2(b["y"]-a["y"],b["x"]-a["x"]) for a,b in zip(points,points[1:])]
    planned=sum(math.atan2(math.sin(b-a),math.cos(b-a)) for a,b in zip(headings[start:end],headings[start+1:end+1]))
    planned_deg=math.degrees(planned);require(planned_deg>=60.,"prospective original route must encode a genuine LEFT")
    by_sequence={row["ack_receipt_sequence"]:row for row in decisions}
    ACKs={r["sequence"]:r for r in receipts}
    require(len(by_sequence)==len(decisions) and len(ACKs)==len(receipts),"unique raw decision/ACK denominator required")
    drive=[row for row in states if row["capture_phase"]=="driving"]
    rows=[];first_effect=None;entry_yaw=None;last_yaw=None;unwrapped=0.;stable=None;complete=None
    postturn_travel=0.;postturn_previous=None
    for i,state in enumerate(drive):
        progress,cte,heading=projection(points,state["x"],state["y"])
        yaw=state["yaw"]
        if progress>=arcs[end] and postturn_previous is None:postturn_previous=(state["x"],state["y"])
        elif postturn_previous is not None:
            postturn_travel+=math.hypot(state["x"]-postturn_previous[0],state["y"]-postturn_previous[1])
            postturn_previous=(state["x"],state["y"])
        if last_yaw is not None:unwrapped+=math.atan2(math.sin(yaw-last_yaw),math.cos(yaw-last_yaw))
        last_yaw=yaw
        sequence=state["control_transport"]["expected_receipt_sequence"]
        decision=by_sequence.get(sequence);receipt=ACKs.get(sequence,{})
        learned=bool(decision and decision.get("application",{}).get("learned_control_applied") is True
            and decision.get("learned_control_proposed") is True and decision.get("control_source")=="learned_trajectory_tracker"
            and state["control_transport"].get("status")=="PASS" and receipt.get("server_accepted") is True
            and receipt.get("before_frame")==decision.get("command_frame")==state["frame"]-1)
        if first_effect is None and learned:first_effect=dict(frame=state["frame"],timestamp=state["timestamp"],progress_m=progress)
        if entry_yaw is None and progress>=arcs[start]:entry_yaw=unwrapped
        turn_deg=None if entry_yaw is None else math.degrees(unwrapped-entry_yaw)
        error=abs(math.degrees(math.atan2(math.sin(yaw-heading),math.cos(yaw-heading))))
        band=learned and progress>=arcs[end] and abs(cte)<=.35 and error<=5.
        stable=state["timestamp"] if band and stable is None else stable if band else None
        row=dict(frame=state["frame"],timestamp=state["timestamp"],prior_ACK_sequence=sequence,
            actual_prior_learned_ACK_effect=learned,unclamped_rear_XY_progress_m=progress,CTE_m=cte,heading_error_deg=error,
            signed_actual_turn_deg=turn_deg,in_stable_postturn_band=band,actual_postturn_XY_travel_m=postturn_travel)
        rows.append(row)
        if complete is None and first_effect is not None and first_effect["progress_m"]<arcs[start] and stable is not None \
                and state["timestamp"]-stable>=3.-1e-6 and progress>=arcs[end]+10. and postturn_travel>=10. and turn_deg is not None \
                and turn_deg>=60. and abs(turn_deg-planned_deg)<=15.:
            complete=dict(row,stable_seconds=state["timestamp"]-stable)
    post=[] if first_effect is None else [row for row in rows if row["frame"]>=first_effect["frame"]]
    progress=None if not post else post[-1]["unclamped_rear_XY_progress_m"]-post[0]["unclamped_rear_XY_progress_m"]
    ctes=[abs(r["CTE_m"]) for r in post]
    stop=[d for d in decisions if d.get("application",{}).get("learned_control_applied") is True
        and d.get("control",{}).get("desired_speed_mps",1.)<=.05]
    core=bool(post) and all(r["actual_prior_learned_ACK_effect"] for r in post) and not stop \
        and progress>=90. and max(ctes)<=1. and percentile(ctes,.95)<=.35
    return dict(passed=bool(complete is not None and core),actual_turn_event=complete,first_actual_learned_effect=first_effect,
        original_XY_turn_entry_m=arcs[start],original_XY_turn_exit_m=arcs[end],prospective_route_heading_change_deg=planned_deg,
        learned_post_handoff_progress_m=progress,post_handoff_native_denominator=len(post),
        rejected_or_nonlearned_post_handoff_count=sum(not r["actual_prior_learned_ACK_effect"] for r in post),
        unexpected_zero_target_command_count=len(stop),maximum_CTE_m=max(ctes) if ctes else None,
        CTE_p95_m=percentile(ctes,.95) if ctes else None,all_driving_rows=rows)


def native_rates(states):
    require(len(states)>1,"actual native speed rate denominator required");rates=[];rear=[];actor=[]
    for row in states:
        rear.append(math.hypot(row["vx"],row["vy"]));actor.append(math.hypot(*row["world_velocity_carla"][:2]))
    for i,(a,b) in enumerate(zip(states,states[1:])):
        dt=b["timestamp"]-a["timestamp"]
        require(b["frame"]==a["frame"]+1 and math.isfinite(dt) and abs(dt-.05)<=1e-6,"full real native interval cadence required")
        rates.append((actor[i+1]-actor[i])/dt)
    require(all(math.isfinite(v) for v in (*rates,*rear,*actor)),"finite actual native rates/speeds required")
    return dict(passed=min(rates)>=-6.-1e-6 and max(rates)<=3.+1e-6 and max(rear)<=2.5 and max(actor)<=2.7,
        all_native_interval_count=len(rates),actor_scalar_dvdt_min_mps2=min(rates),actor_scalar_dvdt_max_mps2=max(rates),
        actual_rear_speed_max_mps=max(rear),actual_actor_speed_max_mps=max(actor),
        observed_abs2p9_violation_count=sum(abs(v)>2.9+1e-6 for v in rates),
        includes_last_warmup_to_first_drive_and_all_coast_intervals=True,actor_API_acceleration_is_not_scalar_dvdt=True)


def timing_report(decisions,wall,receipts):
    expected=[d for d in decisions if d.get("learned_control_proposed") is True and d.get("held_between_camera_frames") is False]
    by_sequence={r["sequence"]:r for r in receipts};timings={r["ACK_sequence"]:r for r in wall}
    require(len(by_sequence)==len(receipts) and len(timings)==len(wall),"complete unique actual timing/ACK rows required")
    policy=[];latency=[];PI=bool(expected)
    for d in expected:
        seq=d["ack_receipt_sequence"];r=by_sequence[seq];t=timings[seq]
        require(r["server_accepted"] is True and t["ACK_server_accepted"] is True
            and t["command_frame"]==d["command_frame"] and t["source_checkpoint_sha256"]==_scope.pins["checkpoint_sha256"]
            and t["stream_source_sha256"]==_owner_plan_sha_or_report and t["learned_control_proposed"] is True,
            "every actual fresh learned proposal needs its matching accepted original ACK timing")
        require(t.get("inference_frame")==d.get("inference_frame"),"exact fresh inference frame differs")
        p=d.get("total_seconds");v=t["exact_bundle_delivery_to_ACK_seconds"]
        require(type(p) in (float,int) and type(v) in (float,int) and math.isfinite(p) and math.isfinite(v) and p>=0. and v>=0.,"finite actual full policy/wall times required")
        policy.append(p);latency.append(v)
        audit=d.get("longitudinal_pi_research",{});target=d.get("control",{}).get("desired_speed_mps")
        PI &= type(target) in (int,float) and math.isfinite(target) and audit.get("controller_target_cap_mps")==2.4 \
            and audit.get("original_model_target_mps")==target and audit.get("governed_controller_target_mps")==min(target,2.4) \
            and audit.get("explicit_controller_calibration_intervention") is True and audit.get("observed_speed_clipped") is False \
            and audit.get("observed_speed_mps")==d.get("measured_current_speed_mps") \
            and audit.get("one_fresh_camera_update_one_PI_integration") is True and audit.get("proposed_normal_tracker_control")==d.get("control")
    return dict(policy_passed=bool(policy) and max(policy)<=.1,wall_passed=bool(latency) and max(latency)<=.1,
        PI_target_cap_and_one_fresh_update_passed=bool(PI),accepted_fresh_denominator=len(expected),
        policy_max_s=max(policy) if policy else None,wall_max_s=max(latency) if latency else None,
        rejected_proposal_count=sum(d.get("learned_control_proposed") is False and d.get("control_source")=="emergency_brake" for d in decisions),
        sensor_callback_entry_observed=False,exposure_to_ACK_wall_latency_proven=False)


_owner_plan_sha_or_report=None
def diagnostic_report(output,plan,owner):
    global _owner_plan_sha_or_report
    result=dict(schema=SCHEMA+".diagnostic_report",arm=plan["arm"],status="FAIL_RETAINED",original_owner=owner,
        joint_motion_body_policy_wall100ms_passed=False,model_promoted=False,vehicle_control_approved=False,
        independent_validation=False,TRAIN_seen_route=True,all_failed_tail_retained=False,
        ownership_passed=False,bounded_capture_complete=False)
    episode=output/"episode" if (output/"episode").is_dir() else output/"episode.partial"
    result["raw_source_sha256"]={str(p):digest(p) for p in episode.rglob("*") if p.is_file()}
    try:
        states,cameras,receipts=[jsonl(episode/name) for name in ("states.jsonl","camera_frames.jsonl","control_receipts.jsonl")]
        manifest=read_json(episode/"manifest.json");decisions=jsonl(episode/"learned_control.jsonl") if (episode/"learned_control.jsonl").is_file() else []
        ownership=not owner["cleanup_errors"] and owner["source_hashes_match"] is True and owner["external_source_hashes_match"] is True
        result.update(native_state_count=len(states),camera_bundle_count=len(cameras),ACK_count=len(receipts),ownership_passed=ownership)
        if ownership and states:
            result["full_capture_body_clearance"]=_originals["body_report"](output,plan,episode,states,receipts,manifest)
        # HH_261002 - The source-authenticated LEFT context reader preserves all original ACK math and validates the extra unobserved closing brake.
        audit=teacher_owner.closing.provider()
        transport=audit.analyze_transport(states,receipts,manifest["capture_contract"]["control_transport"]["bootstrap_observation"],wheelbase=manifest["coordinate_contract"]["wheelbase_m"])
        camera=audit.analyze_camera_alignment(states,cameras)
        pixels=_originals["pixel_audit"](episode,cameras)
        motion=left_motion_report(states,read_json(episode/"route.json"),decisions,receipts);rates=native_rates(states)
        _owner_plan_sha_or_report=digest(output/"declaration.json")
        timing=timing_report(decisions,jsonl(episode/"wall_timing.jsonl"),receipts)
        configuration=preview_owner.configuration_report(episode);thread_settings=threads.readback_report(episode)
        recording=read_json(episode/"actual_recording_binding.json");fixture=read_json(episode/"owned_green_signal_fixture.json")
        complete=manifest.get("status")=="complete" and (len(states),len(cameras),len(receipts))==(1055,528,1060) \
            and {phase:sum(s["capture_phase"]==phase for s in states) for phase in ("stationary_warmup","driving","stationary_tail")}==live.source_contract()["phase_native_state_counts"]
        recorded=recording["schema"]==live.SCHEMA+".actual_recording_binding" and recording.get("warmup_session_guard_passed") is True \
            and recording["actual_recording_bundle_count"]==recording["actual_after_ACK_receipt_count"]==528 and recording["pending_source_camera_count"]==0
        green=fixture["state"]=="RESTORED" and fixture["original_state_freeze_and_duration_restored"] is True \
            and fixture==manifest["capture_contract"]["owned_green_signal_fixture"]
        body=result.get("full_capture_body_clearance",{}).get("all_native_body_and_swept_clearance_passed") is True
        passed=ownership and complete and body and transport["status"]==camera["status"]==pixels["status"]=="PASS" \
            and motion["passed"] and rates["passed"] and timing["policy_passed"] and timing["wall_passed"] \
            and timing["PI_target_cap_and_one_fresh_update_passed"] and configuration["passed"] and thread_settings["passed"] and recorded and green \
            and not any(s.get("collision") or s.get("lane_invasion") for s in states)
        result.update(status="PASS_ROLLING_TRAIN_LEFT_DIAGNOSTIC_NOT_APPROVED" if passed else "FAIL_RETAINED",
            joint_motion_body_policy_wall100ms_passed=bool(passed),bounded_capture_complete=complete,all_failed_tail_retained=complete,
            independent_transport_audit=transport,independent_camera_alignment_audit=camera,actual_camera_source_pixel_audit=pixels,
            actual_LEFT_motion=motion,all_native_motion=rates,actual_policy_and_bundle_ACK_timing=timing,
            actual_preview_configuration=configuration,actual_threads=thread_settings,actual_recording_binding=recording,controlled_green_restored=green)
        closure_check(output,plan)
        require(all(digest(p)==pin for p,pin in result["raw_source_sha256"].items()),"raw source mutated during independent audit")
    except Exception as error:
        result.update(status="FAIL_RETAINED",joint_motion_body_policy_wall100ms_passed=False,
            diagnostic_error=type(error).__name__+": "+str(error))
    finally:_owner_plan_sha_or_report=None
    return result


@contextmanager
def activated(path,sha,arm):
    global _scope,_active,task,ASSETS,_originals
    require(not _active and _scope is None and arm in ARMS,"exclusive prospective comparison arm required")
    source,value,run=authority(path,sha);ep=loader.PARENT if arm=="parent6000" else value["endpoint"]
    folder=CURRENT_PARENT if arm=="parent6000" else run/"exports/step_020000"
    _scope=SimpleNamespace(arm=arm,pins=ep,authority_path=source,authority_sha=sha,authority=value,run=run,weights=folder/"model_state.npz",metadata=folder/"metadata.json")
    old=task,ASSETS,_originals;_active=True
    try:
        task=live.task_namespace(sys.modules[__name__]);ASSETS=evidence_assets();_originals=adapted_originals()
        yield
    finally:
        task,ASSETS,_originals=old;_scope,_active=None,False


def verify_case(run_id,case_id,expected):return _originals["verify_case"](run_id,case_id,expected)
def closure_check(output,plan):return _originals["closure_check"](output,plan)
def preflight(run_id,expected):
    plan,info=_originals["preflight"](run_id,expected);load_model();return plan,info


def worker(run_id,expected):
    global _owner_plan_sha
    require(_owner_plan_sha is None,"private worker cannot nest")
    plan=verify_case(run_id,"pilot_00",expected);base.assert_owned_worker(dict(plan,output=plan["owner_output"]),expected)
    _owner_plan_sha=expected
    try:
        args=live.parse_args(capture_argv(run_id,"pilot_00",expected),sys.modules[__name__]);model=load_model(expected)
        from .defer_exact_jpeg_until_ACK_research_20261001 import deferred_recording
        with live.activated(model,args,sys.modules[__name__]),base.episode_quota(args),deferred_recording(live.collector,args.output.with_name("episode.partial")):
            result=live.collector.run(args)
        print(result);return 0
    finally:_owner_plan_sha=None


def verify_batch(expected):
    require(loader.is_sha(expected) and digest(batch_path())==expected,"BOTH complete externally pinned declarations required")
    batch=read_json(batch_path())
    require(batch.get("schema")==SCHEMA+".batch" and batch.get("arm_order")==list(ARMS)
        and batch.get("physical_case_count")==2 and batch.get("completed_run_authority_sha256")==_scope.authority_sha
        and set(batch.get("case_sha256",{}))==set(ARMS),"comparison batch identity differs")
    for arm,pin in batch["case_sha256"].items():
        require(loader.is_sha(pin) and digest(root_path()/(arm+"_plan.json"))==pin,"one prospective model plan changed")
    return batch


def declare(path,sha):
    require(not root_path().exists(),"fresh BOTH-arm declaration required");plans={}
    # HH_261002 - Caller is outside an arm scope at the CLI; build BOTH complete declarations before writing either.
    for arm in ARMS:
        with activated(path,sha,arm):disk_check();plans[arm]=_originals["build_plan"]("v1","pilot_00")
    root_path().mkdir(parents=True,exist_ok=False)
    for arm,plan in plans.items():write_new(root_path()/(arm+"_plan.json"),plan)
    value=dict(schema=SCHEMA+".batch",arm_order=list(ARMS),physical_case_count=2,
        completed_run_authority_sha256=sha,case_sha256={arm:digest(root_path()/(arm+"_plan.json")) for arm in ARMS},
        automatic_retry=False,outcome_selected_retry=False,simulator_started=False,model_promoted=False,vehicle_control_approved=False)
    write_new(batch_path(),value);return dict(batch_plan_path=str(batch_path()),batch_plan_sha256=digest(batch_path()),simulator_started=False)


def main(argv=None):
    global _batch_sha
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument("--mode",choices=("contract","declare","preflight","execute","worker","summary"),default="contract")
    parser.add_argument("--run-id",choices=("v1",),default="v1");parser.add_argument("--arm",choices=ARMS)
    parser.add_argument("--completed-run-authority",type=Path);parser.add_argument("--completed-run-authority-sha256")
    parser.add_argument("--batch-plan-sha256");parser.add_argument("--case-plan-sha256");args=parser.parse_args(argv)
    if args.mode=="contract":print(json.dumps(live.source_contract(),indent=2));return 0
    require(args.completed_run_authority is not None and loader.is_sha(args.completed_run_authority_sha256),"separate actual ROOT completion authority required")
    if args.mode=="declare":
        require(args.arm is None,"both arms predeclared together")
        # HH_261002 - No physical caller or output is created until both fixed plans are valid.
        result=declare(args.completed_run_authority,args.completed_run_authority_sha256)
        print(json.dumps(result));return 0
    require(args.arm in ARMS,"explicit predeclared arm required")
    with activated(args.completed_run_authority,args.completed_run_authority_sha256,args.arm):
        expected=args.case_plan_sha256
        if args.mode=="worker":
            import os
            batch=verify_batch(os.environ.get("LEFT45_COMPARISON_BATCH_SHA256"))
            require(expected==batch["case_sha256"][args.arm],"private worker differs from predeclared arm")
            return worker("v1",expected)
        batch=verify_batch(args.batch_plan_sha256)
        expected=batch["case_sha256"][args.arm]
        if args.mode=="summary":print(json.dumps(read_json(paths("v1")[0]/"diagnostic_report.json")));return 0
        if args.arm=="LEFT20000" and args.mode=="execute":
            prior=read_json(root_path()/"parent6000/owner_result.json")
            require(prior["declaration_sha256"]==batch["case_sha256"]["parent6000"] and prior["cleanup_errors"]==[]
                and prior["source_hashes_match"] is True and prior["external_source_hashes_match"] is True,
                "GOODparent attempt must finish/close first regardless of FAIL/PASS")
        if args.mode=="preflight":print(json.dumps(preflight("v1",expected)[1]));return 0
        changes=dict(paths=paths,preflight=preflight,diagnostic_report=diagnostic_report,SCHEMA=SCHEMA,
            CHECKPOINT_SHA=_scope.pins["checkpoint_sha256"],ASSETS=ASSETS,MODULE=MODULE,external_sources=external_sources,environment=environment)
        saved={name:getattr(base,name) for name in changes}
        _batch_sha=args.batch_plan_sha256
        try:
            for name,value in changes.items():setattr(base,name,value)
            return base.execute("v1",expected)
        finally:
            for name,value in saved.items():setattr(base,name,value)
            _batch_sha=None


if __name__=="__main__":raise SystemExit(main())
