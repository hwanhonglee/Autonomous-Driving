"""HH_261002 - Synthetic causal coast/HOLD, frozen AST, source and failure guards only; never physical capture or TRAIN approval."""

import ast
from copy import deepcopy
from pathlib import Path
import struct
import sys
from types import ModuleType, SimpleNamespace

import pytest

from scripts.e2e import town07_natural_goal_stop_brakefree_v6_20261002 as task
from scripts.e2e import collect_town07_natural_goal_stop_brakefree_v6_20261002 as collector
from scripts.e2e import run_owned_town07_natural_goal_stop_brakefree_v6_20261002 as owner


def configuration():return task.goal_configuration(SimpleNamespace(**task.FIXED_ARGS))


def governor():return task.CoastToGoalGovernor(configuration(),1.5,20.)


def measured(g,frame,remaining=20.,speed=1.5,*,count_hold=True,planar=None,overshoot=None):
    return g.update(remaining,remaining if planar is None else planar,speed,-remaining if overshoot is None else overshoot,
        count_hold=count_hold,source_frame=frame,source_timestamp=frame*.05,source_state=dict(vx=speed,vy=0.))


def control(**kwargs):
    values=dict(throttle=.2,brake=0.,steer=.1,hand_brake=False,reverse=False,manual_gear_shift=False,gear=0)
    values.update(kwargs)
    c=collector.ReconstructedFloat32Control()
    for name,value in values.items():setattr(c,name,value)
    return c


def normal(policy,record,**kwargs):
    c=control(**kwargs)
    return policy.apply(c,goal_record=record,speed=record["measured_speed_mps"],timestamp=record["source_timestamp"],
        frame=record["source_frame"],target_speed_mps=record["target_speed_mps"]*3.6/3.6,
        normal_proposal=collector.base.control_dict(c))


def test_exact_single_prospective_protocol_no_startup_or_native_admission_override():
    cfg=configuration();plan=task.fixed_contract("v6","pilot_00")
    assert cfg.normal_brake_cap==0. and cfg.normal_throttle_cap==.20
    assert cfg.stopped_speed_mps==.05 and cfg.hold_seconds==2. and cfg.minimum_tail_seconds==6.5
    assert plan["fixed_arguments"]["target_speed_kmh"]==5.4 and plan["nominal_teacher_cruise_mps"]==1.5
    assert plan["fixed_arguments"]["max_duration_sec"]==200. and plan["seed"]==task.FIXED_ARGS["seed"]==task.SEED==2026102400
    assert plan["route_length_m"]==210.5975914062836 and plan["route_sha256"]==task.ROUTE_SHA
    assert plan["attempts"]==1 and plan["automatic_retry"] is False
    assert plan["selected_first_driving_relative_native_frame"]==80 and plan["stationary_tail_native_ticks"]==130
    assert plan["required_future_support_ns"]==6_400_000_000 and plan["minimum_selected_span_ns"]==30_000_000_000
    assert plan["startup_anchors_selected"] is False and plan["stationary_tail_is_training_anchor"] is False
    assert plan["formal_feature_denominators"]==dict(main=9,sub=30)
    assert "training_data_approved" not in task.pending_scope()
    contract=task.mission_contract()
    assert contract["cruise_and_coast_normal_brake"]==0.
    assert contract["service_hold_brake"]==1. and contract["actual_coast_entry_speed_band_mps"]==[1.48,1.58]
    assert contract["coast_release_remaining_arc_m"]==11.5 and contract["maximum_coast_to_completion_seconds"]==45.
    assert contract["training_data_approved"] is False and contract["vehicle_control_approved"] is False
    assert owner.execution_contract("v6","pilot_00")["owner_wall_timeout_s"]==600.


@pytest.mark.parametrize("run,case",[("v1","pilot_00"),("v2","pilot_00"),("v3","pilot_00"),("v4","pilot_00"),("v5","pilot_00"),("v6","train_00"),("../v3","pilot_00")])
def test_exact_fresh_namespace(run,case):
    with pytest.raises(Exception):task.paths(run,case)


def test_frozen_target_ramp_identical_until_one_declared_coast_latch():
    new=governor();old=task.goal.GoalStopGovernor(configuration(),1.5,20.)
    for frame in range(1,31):
        current=measured(new,frame)
        original=old.update(20.,20.,1.5,-20.)
        assert all(current[key]==value for key,value in original.items())
        assert current["terminal_mission_phase"]=="GO"
    record=measured(new,31,remaining=11.5)
    assert record["terminal_mission_phase"]=="BRAKE_TO_STOP" and record["target_speed_mps"]==0.
    assert record["coast_source_measurements"]["frame"]==31 and record["pilot_failure_reason"] is None
    record=measured(new,32,remaining=11.7,speed=1.52)
    assert record["terminal_mission_phase"]=="BRAKE_TO_STOP" and record["target_speed_mps"]==0.
    assert record["coast_source_measurements"]["frame"]==31 and record["pilot_transition"] is None


@pytest.mark.parametrize("speed",[1.479999,1.580001,0.,2.4])
def test_bad_actual_entry_aborts_via_original_failure_path_not_forced_speed(speed):
    g=governor();record=measured(g,1,remaining=11.5,speed=speed)
    assert record["coast_source_measurements"] is None and record["pilot_failure_reason"]=="brakefree_coast_entry_outside_predeclared_actual_speed_band"
    assert record["target_speed_mps"]==0. and record["measured_speed_mps"]==speed
    assert task.goal.goal_stop_termination_reason(record,False,0.,configuration())==record["pilot_failure_reason"]


def test_first_low_speed_crossing_not_hold_completion_and_original40_intervals_preserved():
    g=governor();measured(g,1,remaining=11.5)
    record=measured(g,2,remaining=.75,speed=.05)
    assert record["terminal_mission_phase"]=="HOLD" and record["hold_ticks"]==0 and record["complete"] is False
    assert record["low_speed_service_hold_source"]["frame"]==2
    # HH_261002 - A measured speed rebound resets true dwell but cannot restart the latched throttle.
    record=measured(g,3,remaining=.75,speed=.050001)
    assert record["hold_ticks"]==0 and record["complete"] is False and record["target_speed_mps"]==0.
    for frame in range(4,44):record=measured(g,frame,remaining=.75,speed=.02)
    assert record["hold_ticks"]==39 and record["complete"] is False
    record=measured(g,44,remaining=.75,speed=.02)
    assert record["hold_ticks"]==40 and record["hold_duration_sec"]==2. and record["complete"] is True
    assert task.goal.goal_stop_termination_reason(record,True,0.,configuration())=="comfortable_goal_measured_stop_and_dwell"


@pytest.mark.parametrize("remaining,planar,overshoot,speed",[(1.000001,1.000001,-1.,.01),(.75,1.000001,-.75,.01),
    (0.,0.,0.,.01),(.75,.75,.01,.01),(.75,.75,-.75,.050001)])
def test_service_hold_never_precedes_all_actual_goal_and_speed_conditions(remaining,planar,overshoot,speed):
    g=governor();measured(g,1,remaining=11.5)
    record=measured(g,2,remaining=remaining,planar=planar,overshoot=overshoot,speed=speed)
    assert record["low_speed_service_hold_source"] is None and record["terminal_mission_phase"]=="BRAKE_TO_STOP"


def test_45second_incomplete_coast_timeout_is_preserved_failure_not_auto_retry():
    g=governor();measured(g,1,remaining=11.5)
    for frame in range(2,902):record=measured(g,frame,remaining=5.,speed=.2)
    assert record["coast_elapsed_s"]==pytest.approx(45.) and record["pilot_failure_reason"]=="brakefree_coast_goal_completion_timeout"
    assert not record["complete"] and task.goal.goal_stop_termination_reason(record,False,0.,configuration())==record["pilot_failure_reason"]


@pytest.mark.parametrize("mutation",["bool_vx","nan_vx","different_v0","bool_frame","bool_time","future_gap","duplicate_twice"])
def test_actual_source_numerics_and_native_sequence_never_rewritten(mutation):
    g=governor();measured(g,1,count_hold=False)
    values=dict(remaining_arc_m=20.,planar_distance_m=20.,speed_mps=1.5,terminal_overshoot=-20.,count_hold=True,
        source_frame=2,source_timestamp=.1,source_state=dict(vx=1.5,vy=0.))
    if mutation=="bool_vx":values["source_state"]["vx"]=True
    elif mutation=="nan_vx":values["source_state"]["vx"]=float("nan")
    elif mutation=="different_v0":values["source_state"]["vx"]=1.6
    elif mutation=="bool_frame":values["source_frame"]=True
    elif mutation=="bool_time":values["source_timestamp"]=True
    elif mutation=="future_gap":values["source_frame"]=3
    else:
        measured(g,1,count_hold=False);values.update(source_frame=1,source_timestamp=.05,count_hold=False)
    with pytest.raises(Exception):g.update(**values)


def test_only_one_authentic_no_tick_engagement_and_no_setup_coast():
    g=governor();record=measured(g,1,remaining=.75,speed=0.,count_hold=False)
    assert g.coast is None and g.service_hold is None and record["complete"] is False
    measured(g,1,remaining=.75,speed=0.,count_hold=False)
    with pytest.raises(Exception):measured(g,1,remaining=.75,speed=0.,count_hold=False)


def test_normal_manual1_launch_then_exact_zero_pedals_and_low_speed_service_hold():
    g=governor();policy=collector.BrakeFreeLaunchCoastHoldControl()
    record=measured(g,1,speed=0.)
    c=normal(policy,record)
    assert c.throttle==collector.native_float32(.11) and c.brake==0. and c.manual_gear_shift is True and c.gear==1
    record=measured(g,2,remaining=11.5,speed=1.5);c=normal(policy,record)
    assert c.throttle==c.brake==0. and policy.last["terminal_mission_phase"]=="BRAKE_TO_STOP"
    record=measured(g,3,remaining=.75,speed=.04);c=normal(policy,record)
    assert c.throttle==0. and c.brake==1. and policy.last["terminal_mission_phase"]=="HOLD"
    assert policy.last["actual_governor_observation"]["low_speed_service_hold_source"]["frame"]==3
    assert policy.last["downstream_BasicAgent_hazard_override_untouched"] is True


@pytest.mark.parametrize("mutation",["positive_brake","target_stale","frame_future","time_future","v0_changed","no_hold_source"])
def test_normal_hook_denies_positive_brake_or_noncausal_governor(mutation):
    g=governor();measured(g,1,remaining=11.5);record=measured(g,2,remaining=.75,speed=.04)
    kwargs={}
    if mutation=="positive_brake":kwargs["brake"]=.01
    elif mutation=="target_stale":record["target_speed_mps"]=.1
    elif mutation=="frame_future":record["source_frame"]+=1
    elif mutation=="time_future":record["source_timestamp"]+=.05
    elif mutation=="v0_changed":record["measured_speed_mps"]+=.01
    else:record["low_speed_service_hold_source"]=None
    c=control(**kwargs)
    with pytest.raises(Exception):collector.BrakeFreeLaunchCoastHoldControl().apply(c,goal_record=record,speed=.04,
        timestamp=.1,frame=2,target_speed_mps=0.,normal_proposal=collector.base.control_dict(c))


def test_kph_roundoff_allowed_but_physical_latch_and_target_changes_not_relaxed():
    g=governor();record=measured(g,1,speed=0.)
    c=normal(collector.BrakeFreeLaunchCoastHoldControl(),record)
    assert c.brake==0. and record["target_speed_mps"]==.07500000000000001
    c=control()
    with pytest.raises(Exception):collector.BrakeFreeLaunchCoastHoldControl().apply(c,goal_record=record,speed=0.,timestamp=.05,
        frame=1,target_speed_mps=record["target_speed_mps"]+1e-10,normal_proposal=collector.base.control_dict(c))


def test_full_bootstrap_and_tail_rates_retain_2p9_and_original3_minus6_no_vacuous_pass():
    bootstrap=dict(frame=1,timestamp=.05,vx=0.,vy=0.)
    manifest=dict(capture_contract=dict(control_transport=dict(bootstrap_observation=bootstrap)))
    states=[dict(frame=2,timestamp=.1,vx=.1475,vy=0.,capture_phase="stationary_warmup"),
        dict(frame=3,timestamp=.15,vx=.1475,vy=0.,capture_phase="driving"),
        dict(frame=4,timestamp=.2,vx=.0,vy=0.,capture_phase="stationary_tail")]
    report=task.full_native_quality(manifest,states)
    assert report["status"]=="FAIL" and report["native_interval_denominator"]==3 and report["failed_interval_count"]==2
    assert report["failed_intervals"][0]["failed_bounds"]==["physical_decoder_2_9"]
    assert report["failed_intervals"][1]["phase"]=="stationary_tail"
    assert report["bootstrap_native_count"]==1 and report["full_captured_warmup_driving_HOLD_tail_included"] is True
    assert "NOT actor-world" in report["scalar_speed_rate_reference"] and report["training_data_approved"] is False
    assert task.full_native_quality(manifest,[])["status"]=="FAIL"


def test_task_and_owner_scope_fully_restored_after_errors_and_nesting():
    for module,parent,names in ((task,task.previous,("SCHEMA","PROFILE","SEED","QUOTA","FIXED_ARGS","paths","pending_scope","goal_configuration","fixed_contract","MovingTask")),
        (owner,owner.previous,("task","worker_module","MODULE","SCHEMA","CASE_IDS","ASSETS","capture_argv","execution_contract","runtime_sources","disk_check","diagnostic_report"))):
        saved={name:getattr(parent,name) for name in names}
        with pytest.raises(RuntimeError):
            with module.activated():
                with pytest.raises(Exception):
                    with module.activated():pass
                raise RuntimeError("synthetic")
        assert all(getattr(parent,name) is value for name,value in saved.items()) and not module._active


def test_real_public_parse_exact_run200_seconds_manual_after_bootstrap_and_restore(monkeypatch):
    monkeypatch.setattr(task,"configuration_from_args",lambda args:dict(run=args.natural_goal_run_id))
    saved=task.configuration_from_args;helper=collector.base.configuration_from_args
    args=collector.parse_args(owner.capture_argv("v6","pilot_00"))
    assert args.natural_goal_run_id=="v6" and args.target_speed_kmh==5.4 and args.seed==2026102400
    assert args.max_duration_sec==200. and args.agent_initialization=="after_bootstrap"
    assert args.stationary_tail_sec==6.5 and args.stationary_warmup_sec==1.2
    assert task.configuration_from_args is saved and collector.base.configuration_from_args is helper
    assert owner.worker_module is collector and owner.MODULE.endswith("brakefree_v6_20261002")


def test_exact_outer_seed_fixed_args_public_contract_and_inherited_build_plan_consistency(monkeypatch,capsys):
    old_seed=task.previous.SEED
    assert owner.main(["--mode","contract"])==0
    import json
    public=json.loads(capsys.readouterr().out)
    assert public["seed"]==public["fixed_arguments"]["seed"]==task.SEED==2026102400
    assert task.previous.SEED==old_seed
    monkeypatch.setattr(owner,"runtime_sources",lambda:{"new/source":"a"*64})
    monkeypatch.setattr(owner.lifecycle,"external_sources",lambda:{"external/source":"b"*64})
    with owner.activated():
        plan=owner.previous.build_plan("v6","pilot_00")
    assert plan["seed"]==plan["fixed_arguments"]["seed"]==task.SEED
    assert owner.capture_argv("v6","pilot_00")[owner.capture_argv("v6","pilot_00").index("--seed")+1]==str(plan["seed"])
    assert plan["source_sha256"]=={"new/source":"a"*64} and plan["owner_wall_timeout_s"]==600.
    assert task.previous.SEED==old_seed and owner._active is False and task._active is False


def test_real_guarded_collector_scope_and_hazard_identity_restore(monkeypatch,tmp_path):
    agents,navigation,module=(ModuleType(name) for name in ("agents","agents.navigation","agents.navigation.basic_agent"))
    agents.navigation,navigation.basic_agent=navigation,module
    for name,value in (("agents",agents),("agents.navigation",navigation),("agents.navigation.basic_agent",module)):monkeypatch.setitem(sys.modules,name,value)
    native=SimpleNamespace(frame=100,timestamp=SimpleNamespace(elapsed_seconds=5.))
    c=control(brake=1.,throttle=0.)
    planner=SimpleNamespace(run_step=lambda:c)
    class OriginalAgent:
        def __init__(self):self._vehicle=SimpleNamespace(get_world=lambda:SimpleNamespace(get_snapshot=lambda:native))
        def get_local_planner(self):return planner
        def add_emergency_stop(self,value):return value
    module.BasicAgent=OriginalAgent
    monkeypatch.setattr(task,"configuration_from_args",lambda args:{"fixed":"synthetic"})
    monkeypatch.setattr(task,"verify_frozen",lambda:None)
    monkeypatch.setattr(collector.manual,"verify_frozen_dependencies",lambda:None)
    monkeypatch.setattr(collector.base,"collect_episode",lambda *a,**kw:"captured")
    monkeypatch.setattr(collector.base,"load_route",lambda p:dict(route=[dict(x=0.,y=0.,distance_m=0.,vad_command=0),dict(x=210.,y=0.,distance_m=210.,vad_command=0)]))
    calls=[]
    def install(planner,brake,throttle):
        calls.append((brake,throttle));return dict(normal_brake_cap=brake,normal_throttle_cap=throttle)
    monkeypatch.setattr(collector.base,"install_normal_brake_cap",install)
    monkeypatch.setattr(task.goal,"install_normal_brake_cap",install)
    manifest=dict(capture_contract={})
    names=("collect_episode","AcknowledgedControlTransport","moving_capture_module","acknowledged_snapshot_state","_save_jpeg",
        "_blank_brake_control","checked_agent_initialization_observation","configuration_from_args","GoalStopGovernor")
    saved={name:getattr(collector.base,name) for name in names}
    with pytest.raises(RuntimeError):
        with collector.scope(SimpleNamespace()):
            assert collector.base.collect_episode(None,None,None,tmp_path,None,None,manifest)=="captured"
            teacher=module.BasicAgent();assert teacher.add_emergency_stop(c) is c
            assert calls==[(0.,.20)]
            assert manifest["capture_contract"]["teacher_controller"]["causal_brakefree_terminal_mission"]==task.mission_contract()
            assert manifest["control_source"].endswith("low_speed_hold_v6")
            assert collector.base.checked_agent_initialization_observation is collector.manual.checked_first_gear_initialization_observation
            assert collector.runtime_provider(object()).__file__==task.__file__
            with pytest.raises(Exception):
                with collector.scope(SimpleNamespace()):pass
            raise RuntimeError("synthetic")
    assert all(getattr(collector.base,name) is value for name,value in saved.items()) and module.BasicAgent is OriginalAgent
    assert collector._active is False
    import json
    rows=[json.loads(line) for line in (tmp_path/"actual_hazard_overrides.jsonl").read_text().splitlines()]
    assert len(rows)==1 and rows[0]["returned_control"]["brake"]==1. and rows[0]["original_return_unchanged"] is True


def test_exact_frozen_declaring_verifying_functions_and_worker_ownership_order():
    for name in ("declare","verify_batch"):
        revised=owner.inherited_api(name)
        assert revised.__code__.co_consts==getattr(owner.baseline,name).__code__.co_consts
        assert revised.__globals__["task"] is task and revised.__globals__["MODULE"]==owner.MODULE
    source=Path(owner.previous.__file__).read_text()
    node=next(n for n in ast.parse(source).body if isinstance(n,ast.FunctionDef) and n.name=="worker")
    segment=ast.get_source_segment(source,node)
    assert segment.index("assert_owned_worker")<segment.index("with worker_module.scope")<segment.index("base.run")
    task.verify_frozen()
    closure={str(path.relative_to(task.ROOT)) for path in owner.lifecycle.imported_source_closure((Path(owner.__file__).resolve(),))}
    assert set(task.FROZEN).issubset(closure)
    assert {"tools/e2e/"+name for name in ("town07_natural_goal_stop_brakefree_v6_20261002.py",
        "collect_town07_natural_goal_stop_brakefree_v6_20261002.py","run_owned_town07_natural_goal_stop_brakefree_v6_20261002.py")}.issubset(closure)
    assert set(task.COAST_EVIDENCE)=={"declaration.json","calibration/result.json","calibration/states.jsonl","calibration/control_receipts.jsonl","diagnostic_report.json"}


@pytest.mark.parametrize("missing",["new_causal_mission","full_native_original_bounds","quality_passed","body","source_closure"])
def test_new_actual_quality_and_mission_conjunction_cannot_trust_success_markers(missing):
    checks=dict.fromkeys(("ownership","capture_identity","transport","camera","native_chain","camera_chain","pixels","membership",
        "initialization","quality_matches","body","source_closure","raw_task_complete","quality_passed","new_causal_mission","full_native_original_bounds"),True)
    manifest=dict(status="complete",result=dict(termination_reason="comfortable_goal_measured_stop_and_dwell"))
    assert owner.capture_passed(manifest,dict(worker_exit_code=0),checks,130)
    checks[missing]=False
    assert not owner.capture_passed(manifest,dict(worker_exit_code=0),checks,130)
    assert not owner.capture_passed(manifest,dict(worker_exit_code=False),dict(checks,**{missing:True}),130)


def test_body_denominator_includes_only_genuine_bootstrap_prefix(monkeypatch):
    bootstrap=dict(frame=99,timestamp=4.95,vx=0.,vy=0.)
    states=[dict(frame=100,timestamp=5.,capture_phase="driving"),dict(frame=101,timestamp=5.05,capture_phase="stationary_tail")]
    manifest=dict(capture_contract=dict(control_transport=dict(bootstrap_observation=bootstrap)))
    observed=[]
    def body(*args):
        observed.extend(args[3]);return dict(full_body_denominator=len(args[3]),full_sweep_denominator=len(args[3])-1)
    monkeypatch.setattr(owner.previous,"body_report",body)
    result=owner.body_with_bootstrap(None,None,None,states,[],manifest)
    assert result["full_body_denominator"]==3 and result["full_sweep_denominator"]==2
    assert observed[0]==dict(bootstrap,capture_phase="pre_capture_bootstrap") and observed[1:]==states
    assert result["bootstrap_is_not_TRAIN_or_camera"] is True


def mission_fixture():
    # HH_261002 - Synthetic journal chronology, not physical motion: warmup engagement reuses one frame; final HOLD state skips normal planner.
    route=dict(route=[dict(x=0.,y=0.,distance_m=0.,vad_command=0),dict(x=12.,y=0.,distance_m=12.,vad_command=0)])
    g=governor();states=[];updates=[];targets=[];observations={}
    for frame in range(1,185):
        phase="stationary_warmup" if frame<=2 else "driving" if frame<=54 else "stationary_tail"
        x=0. if frame<=2 else min(11.25,(frame-2)*.9375)
        speed=0. if frame<=2 or phase=="stationary_tail" else 1.5 if frame<14 else .02
        state=dict(frame=frame,timestamp=frame*.05,capture_phase=phase,x=x,y=0.,vx=speed,vy=0.,
            current_control=dict(manual_gear_shift=True,gear=1),route_progress_m=x,route_cte_m=0.)
        record=g.update(12.-x,12.-x,speed,x-12.,count_hold=phase=="driving",source_frame=frame,
            source_timestamp=state["timestamp"],source_state=state)
        updates.append(dict(schema=task.SCHEMA+".actual_governor_update",measured_state_frame=frame,count_hold=phase=="driving",
            original_governor_update=deepcopy(record),governor_math_changed=True,measured_speed_clipped=False))
        if phase!="driving":g.target_speed_mps=0.;record["target_speed_mps"]=0.
        observations[frame]=deepcopy(record)
        state["goal_stop"]=deepcopy(record);states.append(state)
        if frame==2:
            fresh=g.update(12.,12.,0.,-12.,count_hold=False,source_frame=frame,source_timestamp=state["timestamp"],source_state=state)
            updates.append(dict(schema=task.SCHEMA+".actual_governor_update",measured_state_frame=frame,count_hold=False,
                original_governor_update=deepcopy(fresh),governor_math_changed=True,measured_speed_clipped=False))
            observations[frame]=deepcopy(fresh);state["goal_stop"].update(fresh);state["goal_stop"]["next_control_starts_driving"]=True
        if frame==2 or phase=="driving":
            target=observations[frame]["target_speed_mps"]
            targets.append(dict(source_frame=frame,source_timestamp=state["timestamp"],target_speed_mps=target*3.6/3.6,
                requested_target_kph=target*3.6,target_changed_by_wrapper=False))
    # HH_261002 - First stopped frame14 then exactly40 continuous intervals through54; tail remains context only.
    assert states[53]["goal_stop"]["hold_ticks"]==40 and states[53]["goal_stop"]["complete"]
    assignments={r["source_frame"]:r for r in targets}
    policy=collector.BrakeFreeLaunchCoastHoldControl();commands=[];receipts=[]
    for frame in range(1,185):
        is_normal=frame==2 or 3<=frame<54
        s=states[frame-1]
        if is_normal:
            request=normal(policy,observations[frame]);saved=deepcopy(policy.last)
        else:request=control(throttle=0.,brake=1.,manual_gear_shift=True,gear=1);saved=None
        ack=dict(sequence=len(receipts),before_frame=frame,requested_control=collector.base.control_dict(request))
        receipts.append(ack)
        commands.append(dict(ack_receipt=deepcopy(ack),learned_control=False,teacher_control=True,normal_policy=saved,
            actual_governor_target_assignment=deepcopy(assignments.get(frame)),final_BasicAgent_or_brake_proposal=collector.base.control_dict(request)))
    return {},states,receipts,commands,updates,targets,route,[]


def test_all_native_mission_replay_initial_engagement_and_final_completion_denominators():
    inputs=mission_fixture();report=collector.audit_mission(*inputs)
    driving=sum(s["capture_phase"]=="driving" for s in inputs[1])
    assert report["recorded_native_count"]==184 and report["all_native_updates"]==185
    assert report["normal_policy_count"]==driving==52
    assert report["one_no_tick_engagement_update"] is True and report["future64_QA_or_TRAIN_authorization"] is False
    assert inputs[3][1]["normal_policy"]["source_frame"]==2
    assert inputs[3][53]["normal_policy"] is None and inputs[3][53]["final_BasicAgent_or_brake_proposal"]["brake"]==1.
    assert inputs[5][-1]["source_frame"]==54 and inputs[5][-1]["target_speed_mps"]==0.
    assert report["actual_service_hold_entry"]["frame"]==14


@pytest.mark.parametrize("mutation",["missing_update","missing_final_target","stale_normal_target","bad_final_ACK","hazard","bool_gear","false_completion",
    "changed_hold_source","missing_normal","future_final_target_timestamp","nonzero_coast_throttle"])
def test_all_native_mission_replay_rejects_corruption_without_cropping(mutation):
    inputs=list(mission_fixture())
    if mutation=="missing_update":inputs[4].pop(20)
    elif mutation=="missing_final_target":inputs[5].pop()
    elif mutation=="stale_normal_target":inputs[3][20]["actual_governor_target_assignment"]["source_frame"]-=1
    elif mutation=="bad_final_ACK":inputs[2][20]["requested_control"]["throttle"]+=.01
    elif mutation=="hazard":inputs[-1]=[dict(original_return_unchanged=True)]
    elif mutation=="bool_gear":inputs[1][20]["current_control"]["gear"]=True
    elif mutation=="false_completion":inputs[1][53]["goal_stop"]["complete"]=False
    elif mutation=="changed_hold_source":inputs[3][14]["normal_policy"]["actual_governor_observation"]["low_speed_service_hold_source"]["frame"]-=1
    elif mutation=="missing_normal":inputs[3][20]["normal_policy"]=None
    elif mutation=="future_final_target_timestamp":inputs[5][-1]["source_timestamp"]+=.05
    else:inputs[3][12]["normal_policy"]["profile_normal_control"]["throttle"]+=.01
    with pytest.raises(Exception):collector.audit_mission(*inputs)


@pytest.mark.parametrize("native_bounds_pass,mission_pass",[(True,True),(False,True),(True,False)])
def test_compiled_owner_diagnostic_preserves_all_checks_and_requires_both_new_proofs(monkeypatch,tmp_path,native_bounds_pass,mission_pass):
    plan=task.fixed_contract("v6","pilot_00");quality=dict(status="PASS")
    capture=dict(natural_goal_stop_teacher=plan,goal_stop_profile=dict(profile_id=task.PROFILE,bounds={},effective_control=dict(phase_hook_rebound_after_original_outer_cap=True)),
        control_transport=dict(bootstrap_observation=dict(frame=0,timestamp=0.,vx=0.,vy=0.)),
        teacher_controller=dict(normal_brake_cap=0.,normal_throttle_cap=.20,causal_brakefree_terminal_mission=task.mission_contract(),phase_hook_after_original_outer_cap=True))
    result=dict(task_complete=True,failure_reason=None)
    manifest=dict(status="complete",capture_contract=capture,result=dict(moving_task=result,goal_stop_quality=quality,
        goal_reached=True,termination_reason="comfortable_goal_measured_stop_and_dwell"))
    states=[dict(frame=i,timestamp=i*.05,capture_phase="driving" if i<4 else "stationary_tail") for i in range(134)]
    cameras=[dict(frame=i,timestamp=i*.05) for i in range(0,134,2)];receipts=[dict(sequence=i) for i in range(139)]
    monkeypatch.setattr(owner.baseline,"read_json",lambda p:manifest if p.name=="manifest.json" else dict(route=[]))
    monkeypatch.setattr(owner.baseline,"jsonl",lambda p:{"states.jsonl":states,"camera_frames.jsonl":cameras,"control_receipts.jsonl":receipts}[p.name])
    monkeypatch.setattr(task,"verify_frozen",lambda:None)
    monkeypatch.setattr(owner.lifecycle.independent_audit,"analyze_transport",lambda *a,**k:dict(status="PASS"))
    monkeypatch.setattr(owner.lifecycle.independent_audit,"analyze_camera_alignment",lambda *a,**k:dict(status="PASS"))
    monkeypatch.setattr(owner.previous,"pixel_audit",lambda *a:dict(source_pixels_passed=True))
    monkeypatch.setattr(owner.baseline.initialization,"initialization_audit",lambda *a:dict(status="PASS"))
    calls=[]
    def body(output,plan,episode,actual_states,actual_receipts,manifest):
        calls.append(("body",len(actual_states),len(actual_receipts)))
        return dict(all_native_body_and_swept_clearance_passed=True)
    monkeypatch.setattr(owner.previous,"body_report",body)
    monkeypatch.setattr(owner.previous,"closure_check",lambda *a:calls.append(("closure",)))
    monkeypatch.setattr(owner.worker_module.base,"measured_stop_quality",lambda *a:quality)
    monkeypatch.setattr(task,"reconstruct",lambda *a:SimpleNamespace(result=lambda **k:result))
    monkeypatch.setattr(task,"full_native_quality",lambda *a:dict(status="PASS_SCALAR_ONLY" if native_bounds_pass else "FAIL"))
    monkeypatch.setattr(owner,"mission_audit",lambda *a:dict(status="PASS_CAUSAL_MISSION_AND_NORMAL_ACK_ONLY" if mission_pass else "FAIL"))
    current_owner=dict(worker_exit_code=0,cleanup_errors=[],source_hashes_match=True,external_source_hashes_match=True)
    report=owner.diagnostic_report(tmp_path,plan,current_owner)
    assert ("body",135,139) in calls and ("closure",) in calls
    assert report["native_state_count"]==134 and report["ACK_receipt_count"]==139 and report["stationary_tail_state_count"]==130
    assert report["checks"]["full_native_original_bounds"] is native_bounds_pass
    assert report["checks"]["new_causal_mission"] is mission_pass
    passed=native_bounds_pass and mission_pass
    assert report["actual_teacher_capture_passed"] is passed and report["bounded_capture_complete"] is passed
    assert report["status"]==("PASS_ACTUAL_TEACHER_GOAL_CAPTURE_PENDING_FUTURE_QA" if passed else "FAIL_RETAINED")
    assert report["training_data_approved"] is False and report["vehicle_control_approved"] is False and report["future_labels_qualified"] is False


def test_v6_only_changes_fresh_pending_marker_not_frozen_v3_or_denial_guard():
    # HH_261002 - The preflight-only v3 declaration and original denial remain permanent; a NEW pending marker is not TRAIN permission.
    from portable_e2e import contract
    from scripts.e2e import town07_natural_goal_stop_brakefree_v3_20261002 as v3
    original=v3.CoastToGoalGovernor(v3.goal_configuration(SimpleNamespace(**v3.FIXED_ARGS)),1.5,20.)
    new=governor()
    for frame in range(1,5):
        args=dict(remaining_arc_m=20.,planar_distance_m=20.,speed_mps=1.5,terminal_overshoot=-20.,count_hold=True,
            source_frame=frame,source_timestamp=frame*.05,source_state=dict(vx=1.5,vy=0.))
        old=original.update(**args);fresh=new.update(**args)
        assert old["training_data_approved"] is False and "training_data_approved" not in fresh
        assert fresh["training_admission_state"]=="PENDING_QA_NOT_AUTHORIZED"
        stripped_old={k:v for k,v in old.items() if k not in ("profile_id","training_data_approved")}
        stripped_new={k:v for k,v in fresh.items() if k not in ("profile_id","training_admission_state")}
        assert stripped_old==stripped_new
    denied=dict(result=dict(goal_stop_quality=dict(driving_final_stop=deepcopy(old))))
    pending=dict(result=dict(goal_stop_quality=dict(driving_final_stop=deepcopy(fresh))))
    with pytest.raises(contract.ContractError):contract._require_admission_not_denied(denied,"frozen v3",include_native_metadata=True)
    contract._require_admission_not_denied(pending,"prospective v6",include_native_metadata=True)
    assert denied["result"]["goal_stop_quality"]["driving_final_stop"]["training_data_approved"] is False
    pending["result"]["training_data_approved"]=False
    with pytest.raises(contract.ContractError):contract._require_admission_not_denied(pending,"actual native denial",include_native_metadata=True)
    assert task.mission_contract()["training_data_approved"] is False and owner.SCHEMA.endswith(".v6.owned_pilot")


def test_v6_code_source_closure_includes_all_frozen_v3_bytes_and_physics_laws_identical():
    from scripts.e2e import collect_town07_natural_goal_stop_brakefree_v3_20261002 as v3worker
    old=collector.BrakeFreeLaunchCoastHoldControl();new=v3worker.BrakeFreeLaunchCoastHoldControl()
    from scripts.e2e import town07_natural_goal_stop_brakefree_v3_20261002 as v3task
    g3=v3task.CoastToGoalGovernor(v3task.goal_configuration(SimpleNamespace(**v3task.FIXED_ARGS)),1.5,20.)
    g5=governor()
    for frame,(remaining,speed) in enumerate(((20.,0.),(20.,.5),(11.5,1.5),(.75,.04),(.75,.02)),1):
        kwargs=dict(remaining_arc_m=remaining,planar_distance_m=remaining,speed_mps=speed,terminal_overshoot=-remaining,
            count_hold=True,source_frame=frame,source_timestamp=frame*.05,source_state=dict(vx=speed,vy=0.))
        record3=g3.update(**kwargs);record5=g5.update(**kwargs)
        c3=control();c5=control()
        values=dict(speed=speed,timestamp=frame*.05,frame=frame,target_speed_mps=record5["target_speed_mps"],normal_proposal=collector.base.control_dict(c5))
        old.apply(c5,goal_record=record5,**values);new.apply(c3,goal_record=record3,**values)
        assert collector.base.control_dict(c5)==collector.base.control_dict(c3)
    closure={str(p.relative_to(task.ROOT)) for p in owner.lifecycle.imported_source_closure((Path(owner.__file__).resolve(),))}
    required={"tools/e2e/"+name for name in ("town07_natural_goal_stop_brakefree_v3_20261002.py",
        "collect_town07_natural_goal_stop_brakefree_v3_20261002.py","run_owned_town07_natural_goal_stop_brakefree_v3_20261002.py")}
    assert required.issubset(closure) and required.issubset(task.FROZEN)
    assert task.FIXED_ARGS==v3task.FIXED_ARGS and task.SEED==v3task.SEED


def quota_observation(moving, output_bytes):
    # HH_261002 - An actual byte count is never relabeled; only the new prospective bound is selected.
    return moving.observe(frame=100,timestamp=5.,phase="driving",progress=0.,base_stop_reason=None,event_count=0,
        native_actor_speed_mps=1.5,rear_base_CTE_m=0.,observed_output_bytes=output_bytes,
        goal_record=dict(profile_id=task.PROFILE,complete=False),observed_free_disk_bytes=10*1024**3)


@pytest.mark.parametrize("byte_count,failed",[(1024**3+1,False),(3*1024**3-64*1024**2,False),
    (3*1024**3-64*1024**2+1,True),(3*1024**3,True)])
def test_actual_native_quota_bound_reaches_three_gib_without_lowering_reserves(byte_count,failed):
    from scripts.e2e import town07_natural_goal_stop_brakefree_v4_20261002 as v4
    assert v4.QUOTA==1024**3 and task.QUOTA==3*1024**3
    before=task.previous.QUOTA
    plan=task.fixed_contract("v6","pilot_00")
    assert plan["episode_quota_bytes"]==task.QUOTA and plan["free_disk_floor_bytes"]==5*1024**3
    assert plan["closing_reserve_bytes"]==64*1024**2
    moving=task.MovingTask(plan);row,reason=quota_observation(moving,byte_count)
    assert (moving.failure=="natural_goal_episode_quota") is failed
    assert row["observed_output_bytes"]==byte_count and (reason is not None) is failed
    assert task.previous.QUOTA==before
    old=v4.MovingTask(v4.fixed_contract("v4","pilot_00"))
    _,oldreason=old.observe(frame=100,timestamp=5.,phase="driving",progress=0.,base_stop_reason=None,event_count=0,
        native_actor_speed_mps=1.5,rear_base_CTE_m=0.,observed_output_bytes=byte_count,
        goal_record=dict(profile_id=v4.PROFILE,complete=False),observed_free_disk_bytes=10*1024**3)
    assert oldreason=="natural_goal_episode_quota"


def test_quota_scoped_to_original_native_observe_and_restored_after_exception():
    original=task.previous.QUOTA
    with pytest.raises(RuntimeError):
        with task.activated():
            assert task.previous.QUOTA==task.QUOTA==3*1024**3
            assert task.previous.fixed_contract("v6","pilot_00")["episode_quota_bytes"]==3*1024**3
            raise RuntimeError("synthetic")
    assert task.previous.QUOTA==original==1024**3


def test_same_native_disk_floor_and_two_closing_reserves_checked_before_declaration(monkeypatch):
    from scripts.e2e import run_owned_town07_natural_goal_stop_brakefree_v4_20261002 as v4
    required=5*1024**3+3*1024**3+2*64*1024**2
    assert owner.disk_check(required)==dict(free_bytes=required,required_bytes=required,prospective_cases=1,reserved_case_bytes=3*1024**3)
    assert v4.disk_check(required)["reserved_case_bytes"]==1024**3
    for invalid in (required-1,True,False,required*.999):
        with pytest.raises(Exception):owner.disk_check(invalid)
    monkeypatch.setattr(owner,"runtime_sources",lambda:{"source":"a"*64})
    monkeypatch.setattr(owner.lifecycle,"external_sources",lambda:{"external":"b"*64})
    with owner.activated():
        plan=owner.previous.build_plan("v6","pilot_00")
        assert plan["episode_quota_bytes"]==3*1024**3
        assert owner.previous.disk_check(required)["reserved_case_bytes"]==3*1024**3
    assert owner.previous.task is not task


def test_v6_only_quota_and_namespace_change_v4_governor_and_normal_control_bitwise_identical():
    from scripts.e2e import town07_natural_goal_stop_brakefree_v4_20261002 as v4
    from scripts.e2e import collect_town07_natural_goal_stop_brakefree_v4_20261002 as worker4
    old=v4.CoastToGoalGovernor(v4.goal_configuration(SimpleNamespace(**v4.FIXED_ARGS)),1.5,20.)
    new=governor();oldpolicy=worker4.BrakeFreeLaunchCoastHoldControl();newpolicy=collector.BrakeFreeLaunchCoastHoldControl()
    for frame,(remaining,speed) in enumerate(((20.,0.),(20.,.5),(11.5,1.5),(.75,.04),(.75,.02)),1):
        args=dict(remaining_arc_m=remaining,planar_distance_m=remaining,speed_mps=speed,terminal_overshoot=-remaining,
            count_hold=True,source_frame=frame,source_timestamp=frame*.05,source_state=dict(vx=speed,vy=0.))
        oldrow=old.update(**args);newrow=new.update(**args)
        assert {k:v for k,v in oldrow.items() if k!="profile_id"}=={k:v for k,v in newrow.items() if k!="profile_id"}
        c4,c5=control(),control();kwargs=dict(speed=speed,timestamp=frame*.05,frame=frame,
            target_speed_mps=newrow["target_speed_mps"],normal_proposal=collector.base.control_dict(c5))
        oldpolicy.apply(c4,goal_record=oldrow,**kwargs);newpolicy.apply(c5,goal_record=newrow,**kwargs)
        assert collector.base.control_dict(c4)==collector.base.control_dict(c5)
    assert task.FIXED_ARGS==v4.FIXED_ARGS and task.SEED==v4.SEED==2026102400
    assert task.FLOOR==v4.FLOOR and task.RESERVE==v4.RESERVE
    closure={str(p.relative_to(task.ROOT)) for p in owner.lifecycle.imported_source_closure((Path(owner.__file__).resolve(),))}
    required={"tools/e2e/"+name for name in ("town07_natural_goal_stop_brakefree_v4_20261002.py",
        "collect_town07_natural_goal_stop_brakefree_v4_20261002.py","run_owned_town07_natural_goal_stop_brakefree_v4_20261002.py")}
    assert required.issubset(task.FROZEN) and required.issubset(closure)


def construct_with_real_original_caps(monkeypatch,tmp_path):
    # HH_261002 - Execute the frozen real construct_agent AST, including its SECOND cap, using synthetic CARLA interfaces only.
    agents,navigation,module=(ModuleType(name) for name in ("agents","agents.navigation","agents.navigation.basic_agent"))
    agents.navigation,navigation.basic_agent=navigation,module
    for name,value in (("agents",agents),("agents.navigation",navigation),("agents.navigation.basic_agent",module)):monkeypatch.setitem(sys.modules,name,value)
    native=SimpleNamespace(frame=100,timestamp=SimpleNamespace(elapsed_seconds=5.))
    raw_state=dict(frame=100,timestamp=5.,vx=1.5,vy=0.,x=0.,y=0.)
    vehicle=SimpleNamespace(get_world=lambda:SimpleNamespace(get_snapshot=lambda:native),get_velocity=lambda:SimpleNamespace(x=1.5,y=0.))
    class OriginalAgent:
        def __init__(self,ego,*args,**kwargs):
            self._vehicle=ego;self._max_brake=1.
            self.planner=SimpleNamespace(_vehicle_controller=SimpleNamespace(max_brake=.10,max_throt=.20),run_step=lambda:control(throttle=.20,brake=.10))
        def set_global_plan(self,plan):pass
        def get_local_planner(self):return self.planner
        def set_target_speed(self,speed):pass
        def run_step(self):return self.planner.run_step()
        def add_emergency_stop(self,value):value.throttle=0.;value.brake=1.;return value
    module.BasicAgent=OriginalAgent
    monkeypatch.setattr(task,"configuration_from_args",lambda args:{"fixed":"synthetic"})
    monkeypatch.setattr(task,"verify_frozen",lambda:None)
    monkeypatch.setattr(collector.manual,"verify_frozen_dependencies",lambda:None)
    monkeypatch.setattr(collector.base,"collect_episode",lambda *a,**kw:"captured")
    monkeypatch.setattr(collector.base,"load_route",lambda p:dict(route=[dict(x=0.,y=0.,distance_m=0.,vad_command=0),dict(x=210.,y=0.,distance_m=210.,vad_command=0)]))
    monkeypatch.setattr(collector.base,"acknowledged_snapshot_state",lambda *a,**k:(dict(raw_state),dict(raw_state)))
    original_cap=task.goal.install_normal_brake_cap
    manifest=dict(capture_contract=dict(goal_stop_profile={}))
    context=collector.scope(SimpleNamespace());context.__enter__()
    collector.base.collect_episode(None,None,None,tmp_path,None,None,manifest)
    collector.base.acknowledged_snapshot_state(native)
    cfg=configuration()
    g=collector.base.GoalStopGovernor(cfg,1.5,20.)
    source=Path(collector.base.__file__).read_text();tree=ast.parse(source)
    construct=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=="construct_agent")
    namespace=dict(Any=object,BasicAgent=module.BasicAgent,ego=vehicle,args=SimpleNamespace(target_speed_kmh=5.4),
        basic_agent_options={},carla_map=None,planner=None,plan=[],goal_stop_config=cfg,goal_stop_governor=g,
        DevelopmentGoalStopGovernor=task.goal.DevelopmentGoalStopGovernor,
        install_development_control=lambda *a:pytest.fail("not the declared original normal path"),
        install_normal_brake_cap=collector.base.install_normal_brake_cap,manifest=manifest)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[construct],type_ignores=[])),"synthetic exact construct_agent","exec"),namespace)
    built,planner=namespace["construct_agent"]()
    return context,built,planner,g,native,raw_state,manifest,original_cap


def test_real_secondcap_is_before_phase_hook_actual_GO_COAST_HOLD_final_sevenfields(monkeypatch,tmp_path):
    context,built,planner,g,native,state,manifest,original=construct_with_real_original_caps(monkeypatch,tmp_path)
    try:
        assert planner._vehicle_controller.max_brake==0. and planner._vehicle_controller.max_throt==.20
        binding=planner._hh_brakefree_phase_cap_binding
        assert binding["after_original_outer_cap"] is True
        assert manifest["capture_contract"]["teacher_controller"]["phase_hook_after_original_outer_cap"] is True
        assert manifest["capture_contract"]["goal_stop_profile"]["effective_control"]["phase_hook_rebound_after_original_outer_cap"] is True
        for frame,(remaining,speed,phase,brake) in enumerate(((20.,1.5,"GO",0.),(11.5,1.5,"BRAKE_TO_STOP",0.),(.75,.04,"HOLD",1.)),100):
            native.frame=frame;native.timestamp.elapsed_seconds=frame*.05;state.update(frame=frame,timestamp=frame*.05,vx=speed)
            collector.base.acknowledged_snapshot_state(native)
            record=g.update(remaining,remaining,speed,-remaining,count_hold=True)
            built.set_target_speed(record["target_speed_mps"]*3.6)
            actual=built.run_step();observed=collector.base.control_dict(actual)
            assert record["terminal_mission_phase"]==phase and observed["brake"]==brake
            assert observed["manual_gear_shift"] is True and observed["gear"]==1
            if phase!="GO":assert observed["throttle"]==0.
            # HH_261002 - This is a synthetic ACK witness, not a CARLA physical receipt; ALL seven request fields remain exact.
            ACK=dict(requested_control=deepcopy(observed))
            assert ACK["requested_control"]==observed and set(observed)=={"throttle","steer","brake","hand_brake","reverse","gear","manual_gear_shift"}
        hazard=built.add_emergency_stop(actual)
        assert hazard is actual and hazard.brake==1. and hazard.throttle==0.
        with pytest.raises(Exception):collector.base.install_normal_brake_cap(planner,0.,.20)
    finally:context.__exit__(None,None,None)
    assert collector.base.install_normal_brake_cap is original


def test_original_wrong_secondcap_order_still_erases_HOLD_and_fails_unmodified_mission_audit(monkeypatch,tmp_path):
    # HH_261002 - Keep the rejected v5 behavior as an explicit negative test; never forgive its genuine applied ACK.
    context,built,planner,g,native,state,manifest,original=construct_with_real_original_caps(monkeypatch,tmp_path)
    try:
        g.update(11.5,11.5,1.5,-11.5,count_hold=True)
        native.frame=101;native.timestamp.elapsed_seconds=5.05;state.update(frame=101,timestamp=5.05,vx=.04)
        collector.base.acknowledged_snapshot_state(native);record=g.update(.75,.75,.04,-.75,count_hold=True)
        built.set_target_speed(record["target_speed_mps"]*3.6)
        # HH_261002 - This extra synthetic OLD-order cap is not the new protocol; it reproduces the causal defect exactly.
        original(planner,0.,.20)
        actual=built.run_step()
        assert record["terminal_mission_phase"]=="HOLD" and actual.brake==0.
    finally:context.__exit__(None,None,None)
    inputs=list(mission_fixture());hold=next(r for r in inputs[3] if r["normal_policy"] and r["normal_policy"]["terminal_mission_phase"]=="HOLD")
    sequence=hold["ack_receipt"]["sequence"];hold["final_BasicAgent_or_brake_proposal"]["brake"]=0.
    hold["ack_receipt"]["requested_control"]["brake"]=0.;inputs[2][sequence]["requested_control"]["brake"]=0.
    with pytest.raises(Exception,match="normal return was superseded or altered"):collector.audit_mission(*inputs)


def test_exact_original_installer_identity_and_scope_restoration_after_exception(monkeypatch,tmp_path):
    original=collector.base.install_normal_brake_cap
    monkeypatch.setattr(collector.base,"install_normal_brake_cap",lambda *a,**k:{})
    with pytest.raises(Exception,match="function identity"):
        with collector.scope(SimpleNamespace()):pass
    assert collector._active is False
    monkeypatch.setattr(collector.base,"install_normal_brake_cap",original)
    context,built,planner,g,native,state,manifest,original=construct_with_real_original_caps(monkeypatch,tmp_path)
    try:
        with pytest.raises(Exception):
            with collector.scope(SimpleNamespace()):pass
    finally:context.__exit__(RuntimeError,RuntimeError("synthetic"),None)
    assert collector.base.install_normal_brake_cap is original and collector._active is False


def test_v6_import_closure_keeps_all_three_frozen_v5_protocol_sources():
    # HH_261002 - Preserve the failed v5 implementation as immutable lineage, not an implicitly approved replacement.
    closure={str(p.relative_to(task.ROOT)) for p in owner.lifecycle.imported_source_closure((Path(owner.__file__).resolve(),))}
    required={"tools/e2e/"+name for name in ("town07_natural_goal_stop_brakefree_v5_20261002.py",
        "collect_town07_natural_goal_stop_brakefree_v5_20261002.py","run_owned_town07_natural_goal_stop_brakefree_v5_20261002.py")}
    assert required.issubset(closure) and required.issubset(task.FROZEN)
    task.verify_frozen()
    contract=task.fixed_contract("v6","pilot_00")
    assert contract["seed"]==task.FIXED_ARGS["seed"]==task.SEED==2026102400
    assert contract["episode_quota_bytes"]==3*1024**3
    assert contract["causal_brakefree_terminal_mission"]["normal_phase_hook_after_both_original_normal_caps"] is True
    assert contract["causal_brakefree_terminal_mission"]["original_two_normal_cap_calls_unchanged"] is True
