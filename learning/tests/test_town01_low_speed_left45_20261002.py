"""HH_261002 - Synthetic exact-membership, phase-control, private-source and no-admission tests; no CARLA/NN execution."""
import ast
import copy
import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace
from types import ModuleType
from unittest import mock

import pytest

from scripts.e2e import town01_low_speed_left45_task_20261002 as task
from scripts.e2e import collect_town01_low_speed_left45_20261002 as worker
from scripts.e2e import run_owned_town01_low_speed_left45_20261002 as owner


def complete(drift=False, omitted=None, event=None, speed=2.4):
    value=task.MovingTask(task.fixed_contract("v1","pilot_00")); states=[]; cameras=[]
    for frame in range(1,1056):
        phase="stationary_warmup" if frame<=24 else "driving" if frame<=925 else "stationary_tail"
        stamp=frame*(.05000000074505806 if drift else .05)
        row,_=value.observe(frame=frame,timestamp=stamp,phase=phase,progress=frame*.1,base_stop_reason=None,
            event_count=int(frame==event),observed_free_disk_bytes=10*1024**3,
            native_actor_speed_mps=speed,rear_base_CTE_m=0.,observed_output_bytes=0)
        states.append(dict(frame=frame,timestamp=stamp,capture_phase=phase,moving_task=row))
        if frame%2 and frame!=omitted:
            camera=dict(frame=frame,timestamp=stamp,capture_phase=phase,camera_order=list(task.original.CAMERAS),
                source_timestamps={name:stamp for name in task.original.CAMERAS},images={name:"real.jpg" for name in task.original.CAMERAS})
            value.saved_camera(camera); cameras.append(camera)
    return value,states,cameras


@pytest.mark.parametrize("drift",[False,True])
def test_exact347_real_history_support1055_all_tail_not_stop(drift):
    value,states,cameras=complete(drift)
    result=value.result(tail_states=130)
    assert result["task_complete"] and len(value.selected)==347
    assert len(states)==1055 and len(cameras)==528
    assert value.first["frame"]==105 and value.last["frame"]==797 and value.support["frame"]==925
    assert [r["frame"] for r in value.history_support["camera_observations"]]==list(range(87,106,2))
    assert value.history_support["preceding_native_guard"]["frame"]==86
    assert value.history_support["preceding_native_guard"]["phase"]=="driving"
    assert value.history_support["preceding_guard_may_be_stationary_warmup"] is False
    assert result["selected_driving_relative_time_ns"]==[4_000_000_000,38_600_000_000]
    assert value.last["frame"]-value.first["frame"]==692
    assert all(not row["moving_task_selected"] for row in cameras if row["frame"]<105 or row["frame"]>797)
    assert result["training_admission_state"]=="PENDING_QA_NOT_AUTHORIZED" and "training_data_approved" not in result


@pytest.mark.parametrize("omitted",[87,105,797])
def test_missing_actual_camera_never_quality_filtered_or_completed(omitted):
    value,_,_=complete(omitted=omitted)
    assert value.result(tail_states=130)["task_complete"] is False


@pytest.mark.parametrize("event",[1,500,1049])
def test_full_startup_drive_tail_events_remain_failure(event):
    value,_,_=complete(event=event)
    assert value.failure=="recovery_collision_or_lane_event"
    assert not value.result(tail_states=130)["task_complete"]


def test_incomplete_tail_or_higher_actual_speed_denies():
    value,_,_=complete()
    assert not value.result(tail_states=129)["task_complete"]
    bad,_,_=complete(speed=2.8)
    assert bad.failure=="recovery_native_actor_speed_limit"


@pytest.mark.parametrize("run,case",[("v2","pilot_00"),("v1","train_04"),("../v1","pilot_00")])
def test_unknown_run_or_case_not_allowed(run,case):
    with pytest.raises(ValueError): task.fixed_contract(run,case)


def test_exact_public_seed_and_byte_disk_bounds():
    result=task.fixed_contract("v1","pilot_00")
    assert result["seed"]==result["fixed_arguments"]["seed"]==2026102500<2**32
    assert result["episode_quota_bytes"]==3*1024**3 and result["expected_JPEG_count"]==3168
    assert result["true_future_point_count"]==22208 and result["future_qualification_required"] is True
    assert result["normal_stop_or_full_route_success_claim"] is False
    assert result["diagnostic_tail_policy"]["stationary_or_STOP_supervision_claimed"] is False
    required=task.FLOOR+task.QUOTA+2*task.RESERVE
    assert owner.disk_check(required)["required_bytes"]==required
    with pytest.raises(RuntimeError): owner.disk_check(required-1)


def blank(_):
    return SimpleNamespace(throttle=0.,brake=1.,steer=0.,manual_gear_shift=False,gear=0,reverse=False,hand_brake=False)


@pytest.mark.parametrize("phase,record,expected",[
    ("stationary_warmup",None,1.),("driving",dict(future_support_complete=False,termination_reason=None),1.),
    ("driving",dict(future_support_complete=True,termination_reason="recovery_future_support_complete"),0.),
    ("stationary_tail",dict(future_support_complete=True,termination_reason=None),0.),
    ("stationary_tail",dict(future_support_complete=True,termination_reason="recovery_collision_or_lane_event"),1.)])
def test_current_phase_blank_exact_no_low_speed_hack(phase,record,expected):
    with mock.patch.object(worker,"_session",dict(original_blank=blank,states=[])):
        control=worker.phase_control(None,phase,record)
        assert control.throttle==control.steer==0. and control.brake==expected
        assert control.manual_gear_shift is True and control.gear==1


def test_outside_tick_end_uses_appended_support_state_and_cleanup_full_brake():
    last=dict(capture_phase="driving",moving_task=dict(future_support_complete=True,termination_reason="recovery_future_support_complete"))
    with mock.patch.object(worker,"_session",dict(original_blank=blank,states=[last])):
        assert worker.phase_blank(None).brake==0.
        assert worker.manual_full_brake(None).brake==1.


def test_original_capture_ast_changes_only_fixture_current_blank_and_aftercapture_cleanup():
    original=worker.definition(worker.base,"collect_episode")
    changed=worker.capture_ast()
    block=next(n for n in ast.walk(changed) if isinstance(n,ast.Try) and any(isinstance(s,ast.Assign)
        and any(isinstance(t,ast.Name) and t.id=="blueprint_library" for t in s.targets) for s in n.body))
    fixture=next(i for i,n in enumerate(block.body) if isinstance(n,ast.Expr) and isinstance(n.value,ast.Call)
        and isinstance(n.value.func,ast.Name) and n.value.func.id=="install_green_fixture")
    assert isinstance(block.body[fixture+1],ast.Assign)
    del block.body[fixture]
    closing=block.finalbody.pop(0)
    assert "left45_post_capture_cleanup_full_brake" in ast.unparse(closing)
    count=0
    for item in ast.walk(changed):
        if isinstance(item,ast.Call) and isinstance(item.func,ast.Name) and item.func.id=="phase_control":
            item.func.id="_blank_brake_control"; item.args=item.args[:1];count+=1
    assert count==1 and ast.dump(changed)==ast.dump(original)


def test_private_teacher_provider_namespace_not_stale_parent():
    assert worker.parse_args.__globals__["task"] is task
    assert worker._teacher["runtime_provider"].__globals__["task"] is task
    assert worker._teacher["scope"].__wrapped__.__globals__["task"] is task
    assert worker._teacher["scope"].__wrapped__.__globals__["v2"].CombinedControl is worker.Launch
    assert owner._originals["diagnostic_report"].__globals__["task"] is task
    assert owner.capture_argv.__globals__["worker_module"] is worker


def states_for_quality(speeds,phase="driving",frame0=25):
    return [dict(frame=frame0+i,timestamp=i*.05,capture_phase=phase,vx=speed,vy=0.) for i,speed in enumerate(speeds)]


def test_all_native_runtime_bad_prep_or_tail_cannot_be_exempted():
    for phase in ("stationary_warmup","driving","stationary_tail"):
        states=states_for_quality([1.,.5],phase=phase)
        states[0]["capture_phase"]="driving"
        result=owner.motion_quality(states)
        assert result["status"]=="FAIL" and result["violations"][0]["scalar_dvdt_mps2"]==-10.


def test_protected_2p95_rejects_distinct_runtime_3_bound_and_keeps_tail_diagnostic():
    states=states_for_quality([0.,0.],frame0=25)
    states += [dict(frame=86,timestamp=3.05,capture_phase="driving",vx=1.,vy=0.),
        dict(frame=87,timestamp=3.10,capture_phase="driving",vx=1.1475,vy=0.)]
    # HH_261002 - Test the protected pair alone with an independent preceding driving origin, without fabricating a cadence hole.
    origin=[dict(frame=25+i,timestamp=i*.05,capture_phase="driving",vx=1.,vy=0.) for i in range(63)]
    origin[-1]["vx"]=1.1475
    result=owner.motion_quality(origin)
    assert result["status"]=="FAIL" and result["violations"][-1]["reasons"]==["protected_native_scalar_dvdt_exceeds_physical2p9"]


def simple_turn(yaw_gain=math.pi/2,progress_gain=100.):
    route=dict(route=[dict(x=0.,y=0.),dict(x=20.,y=0.),dict(x=20.,y=30.),dict(x=20.,y=230.)],
        physical_turn_preflight=dict(selected_block=dict(end_index=2)))
    rows=[]
    for i in range(901):
        progress=progress_gain*i/772
        rows.append(dict(frame=25+i,timestamp=i*.05,capture_phase="driving",x=20. if progress>20 else progress,
            y=max(0.,progress-20),yaw=yaw_gain*min(1.,i/500)))
    return rows,route


@pytest.mark.parametrize("yaw,progress,passed",[(math.pi/2,100.,True),(math.pi/4,100.,False),(math.pi/2,40.,False),(-math.pi/2,100.,False)])
def test_actual_left_exit_plus10_before_lastanchor_requires_real_signed_yaw(yaw,progress,passed):
    rows,route=simple_turn(yaw,progress)
    result=owner.turn_screen(rows,route)
    assert (result["status"]=="PASS")==passed
    assert result["selected_native_denominator"]==347


def test_owner_600wall_public_server_town01_original_no_weights():
    contract=owner.execution_contract("v1","pilot_00")
    assert contract["owner_wall_timeout_s"]==600.
    assert "Town01" in contract["server_command"] and "Town07" not in contract["server_command"]
    assert not any("weights" in str(path) for path in owner.ASSETS)
    argv=owner.capture_argv("v1","pilot_00","a"*64)
    assert argv[argv.index("--max-duration-sec")+1]=="60"
    assert argv[argv.index("--seed")+1]=="2026102500"
    assert argv[argv.index("--expected-live-xodr-sha256")+1]==owner.ASSETS[owner.XODR]


def test_source_synthetic_plan_configuration_restores_and_no_false_training_admission():
    contract=task.fixed_contract("v1","pilot_00")
    assert "training_data_approved" not in json.dumps(contract)
    assert task.MovingTask.__module__==task.__name__
    assert task.configuration_from_args.__globals__ is task.__dict__
    assert task.reconstruct.__globals__ is task.__dict__
    assert owner.preflight.__globals__["runtime_sources"] is owner.runtime_sources


def command_fixture():
    states=[];commands=[];receipts=[]
    zero=dict(throttle=0.,brake=0.,steer=0.,gear=1,manual_gear_shift=True,reverse=False,hand_brake=False)
    braking=dict(zero,brake=1.)
    policy=worker.manual.FirstGearRecoveryLaunchControl();normal_index=0
    def emit(frame,reason,control,normal=None,ramp=None):
        receipt=dict(before_frame=frame,sequence=len(receipts)+1,reason=reason,requested_control=copy.deepcopy(control))
        receipts.append(receipt)
        commands.append(dict(reason=reason,ack_receipt=receipt,normal_policy=copy.deepcopy(normal),target_ramp=ramp,
            final_BasicAgent_or_brake_proposal=copy.deepcopy(control),learned_control=False,teacher_control=True))
    for frame in range(1,1056):
        phase="stationary_warmup" if frame<=24 else "driving" if frame<=925 else "stationary_tail"
        speed=0. if frame<=24 else 2.4
        states.append(dict(frame=frame,timestamp=frame*.05,capture_phase=phase,world_velocity_carla=[speed,0.,0.],
            current_control=copy.deepcopy(zero if phase=="stationary_tail" else braking)))
        if 24<=frame<=924:
            raw=dict(throttle=.2,brake=0.,steer=.02,gear=0,manual_gear_shift=False,reverse=False,hand_brake=False)
            control=owner.manual_witness.native_math.RecordedFloatControl(raw)
            policy.apply(control,speed=speed,timestamp=frame*.05,frame=frame,normal_proposal=raw)
            policy.last["schema"]=task.SCHEMA+".actual_normal_teacher_control"
            target=2.4 if normal_index==0 else min(2.4,.075*normal_index);normal_index+=1
            ramp=dict(source_frame=frame,actual_teacher_target_mps=target,teacher_target_cap_mps=2.4,actual_observed_speed_clipped=False)
            emit(frame,"initial_drive_control" if frame==24 else "next_driving_control",worker.base.control_dict(control),policy.last,ramp)
        elif frame==925:
            emit(frame,"next_driving_control",zero);emit(frame,"driving_end_brake",zero)
        else:
            emit(frame,"next_stationary_tail_control" if phase=="stationary_tail" else "next_stationary_warmup_control",
                zero if phase=="stationary_tail" else braking)
    emit(1055,"left45_post_capture_cleanup_full_brake",braking)
    return commands,receipts,states


def test_complete_independent_normal_replay_context_and_post_pose_cleanup():
    commands,receipts,states=command_fixture()
    result=owner.teacher_command_audit(commands,receipts,states)
    assert result["status"]=="PASS" and result["normal_proposal_count"]==901
    assert result["diagnostic_coast_observations"]==130 and not result["original_hazard_override_frames"]


@pytest.mark.parametrize("mutation",["normal_source","normal_replay","normal_target","tail_current","tail_requested","closing","missing_command"])
def test_complete_teacher_audit_rejects_changed_or_missing_real_sources(mutation):
    commands,receipts,states=command_fixture()
    normal=next(row for row in commands if row["normal_policy"] is not None)
    if mutation=="normal_source": normal["normal_policy"]["source_frame"]+=1
    elif mutation=="normal_replay": normal["normal_policy"]["handoff_latched"]=True
    elif mutation=="normal_target": normal["target_ramp"]["actual_teacher_target_mps"]=0.
    elif mutation=="tail_current": states[-1]["current_control"]["brake"]=1.
    elif mutation=="tail_requested":
        index=next(i for i,row in enumerate(commands) if row["reason"]=="next_stationary_tail_control")
        commands[index]["ack_receipt"]["requested_control"]["brake"]=1.
        commands[index]["final_BasicAgent_or_brake_proposal"]["brake"]=1.
    elif mutation=="closing": commands[-1]["ack_receipt"]["before_frame"]-=1
    else: commands.pop(1)
    with pytest.raises((RuntimeError,ValueError)): owner.teacher_command_audit(commands,receipts,states)


def test_runtime_owner_activation_restores_all_original_bindings_after_exception():
    names=("paths","preflight","diagnostic_report","SCHEMA","CHECKPOINT_SHA","ASSETS","MODULE")
    before={name:getattr(owner.lifecycle,name) for name in names}
    with mock.patch.object(owner,"runtime_sources",return_value={}):
        with pytest.raises(LookupError):
            with owner.activated():
                assert owner.lifecycle.paths is owner.paths and owner.lifecycle.MODULE==owner.MODULE
                assert owner.lifecycle.diagnostic_report is owner.diagnostic_report
                with pytest.raises(RuntimeError):
                    with owner.activated(): pass
                raise LookupError("synthetic worker abort")
    assert not owner._active and all(getattr(owner.lifecycle,name) is value for name,value in before.items())


def test_real_private_teacher_scope_provider_snapshot_blank_manual_and_restore():
    names=("collect_episode","AcknowledgedControlTransport","moving_capture_module","apply_spawn_z_offset",
        "acknowledged_snapshot_state","_save_jpeg","_blank_brake_control","checked_agent_initialization_observation")
    before={name:getattr(worker.base,name) for name in names}
    agents=ModuleType("agents");navigation=ModuleType("agents.navigation");basic=ModuleType("agents.navigation.basic_agent")
    class FakeBasicAgent: pass
    basic.BasicAgent=FakeBasicAgent;navigation.basic_agent=basic;agents.navigation=navigation
    plan=task.fixed_contract("v1","pilot_00")
    synthetic_route=dict(route=[dict(x=0.,y=0.,distance_m=0.,vad_command=3),dict(x=100.,y=0.,distance_m=100.,vad_command=3)])
    with mock.patch.dict(sys.modules,{"agents":agents,"agents.navigation":navigation,"agents.navigation.basic_agent":basic}), \
            mock.patch.object(worker,"assert_owned_capture"),mock.patch.object(task,"configuration_from_args",return_value=plan), \
            mock.patch.object(worker.base,"load_route",return_value=synthetic_route):
        with pytest.raises(LookupError):
            with worker.scope(SimpleNamespace(wheelbase_m=2.85)):
                provider=worker.base.moving_capture_module(None)
                assert provider.__file__==task.__file__ and provider.reconstruct is task.reconstruct
                assert worker.base._blank_brake_control is worker.phase_blank
                assert worker.base.checked_agent_initialization_observation is worker.manual.checked_first_gear_initialization_observation
                assert issubclass(basic.BasicAgent,FakeBasicAgent)
                with pytest.raises(ValueError):
                    with worker.scope(None): pass
                raise LookupError("synthetic transport error")
    assert not worker._active and worker._session is None
    assert basic.BasicAgent is FakeBasicAgent and all(getattr(worker.base,name) is value for name,value in before.items())


def test_native_quota_above_three_gib_and_low_disk_preserved_as_denials():
    for free,size,reason in [(10*1024**3,task.QUOTA,"recovery_episode_disk_quota"),
            (task.FLOOR,0,"recovery_disk_floor")]:
        value=task.MovingTask(task.fixed_contract("v1","pilot_00"))
        value.observe(frame=1,timestamp=.05,phase="stationary_warmup",progress=0.,base_stop_reason=None,event_count=0,
            observed_free_disk_bytes=free,native_actor_speed_mps=0.,rear_base_CTE_m=0.,observed_output_bytes=size)
        assert value.failure==reason
