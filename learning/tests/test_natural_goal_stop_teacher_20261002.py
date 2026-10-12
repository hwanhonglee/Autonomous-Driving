"""HH_261002 - Synthetic protocol/control tests only; no CARLA, GPU, training or actual capture."""

import ast
from contextlib import contextmanager
from datetime import datetime,timezone
from pathlib import Path
from types import SimpleNamespace
import sys
import struct

import pytest

from scripts.e2e import town07_natural_goal_stop_task_20261002 as task
from scripts.e2e import collect_town07_natural_goal_stop_20261002 as collector
from scripts.e2e import run_owned_town07_natural_goal_stop_20261002 as owner


def control(**kw):
    result=dict(throttle=.2,brake=0.,steer=.1,hand_brake=False,reverse=False,manual_gear_shift=False,gear=0)
    result.update(kw)
    return SimpleNamespace(**result)


def apply(policy, *, speed=0.,timestamp=1.,frame=1,target=2.4,**kw):
    c=control(**kw)
    return policy.apply(c,speed=speed,timestamp=timestamp,frame=frame,target_speed_mps=target,
        normal_proposal=collector.base.control_dict(c))


def observation(instance,frame,*,phase="driving",complete=False,reason=None,**kw):
    values=dict(frame=frame,timestamp=frame*.05,phase=phase,progress=frame*.1,base_stop_reason=reason,event_count=0,
        native_actor_speed_mps=0. if phase=="stationary_tail" or complete else 2.4,rear_base_CTE_m=0.,observed_output_bytes=0,
        goal_record=dict(profile_id=task.PROFILE,complete=complete,phase="complete" if complete else "cruise"),
        observed_free_disk_bytes=task.FLOOR+task.RESERVE)
    values.update(kw)
    return instance.observe(**values)


def camera(frame,phase="driving"):
    return dict(frame=frame,timestamp=frame*.05,capture_phase=phase,camera_order=task.original.CAMERAS,
        images={n:n+".jpg" for n in task.original.CAMERAS},source_timestamps={n:frame*.05 for n in task.original.CAMERAS})


def test_fixed_one_pilot_scope_is_not_old_rolling_recovery_or_feature_claim():
    plan=task.fixed_contract("v1","pilot_00")
    assert plan["seed"]==2026102200 and 0<=plan["seed"]<2**32
    assert plan["attempts"]==1 and plan["automatic_retry"] is False
    assert plan["fixed_arguments"]["max_duration_sec"]==140.
    assert plan["fixed_arguments"]["target_speed_kmh"]==8.64
    assert plan["learned_goal_stop_success_claim"] is False and plan["full_RTE04_approved"] is False
    assert plan["main_functions_approved"]==plan["subfunctions_approved"]==0
    assert plan["formal_feature_denominators"]==dict(main=9,sub=30)
    assert plan["startup_anchors_selected"] is False and plan["stationary_tail_is_training_anchor"] is False
    assert plan["stationary_tail_can_be_true_future_context"] is True
    assert "training_data_approved" not in task.pending_scope()


@pytest.mark.parametrize("run,case",[("v2","pilot_00"),("v1","train_04"),("../v1","pilot_00")])
def test_no_previous_run_path_or_cohort_alias(run,case):
    with pytest.raises(Exception):task.paths(run,case)


def test_new_strict_stop_config_original_governor_math_and_tail_budget():
    args=SimpleNamespace(**task.FIXED_ARGS)
    cfg=task.goal_configuration(args)
    assert cfg.stopped_speed_mps==.05 and cfg.normal_brake_cap==.1 and cfg.normal_throttle_cap==.2
    assert cfg.stop_buffer_m==.75 and cfg.goal_tolerance_m==1. and cfg.hold_seconds==2.
    governor=task.goal.GoalStopGovernor(cfg,2.4,20.)
    for _ in range(40):record=governor.update(.75,.75,.04,-.75)
    assert record["complete"] is False and record["hold_ticks"]==39
    record=governor.update(.75,.75,.04,-.75)
    assert record["complete"] is True and record["hold_duration_sec"]==2.
    governor.update(.75,.75,.050001,-.75)
    assert governor.complete is False
    schedule=collector.base.capture_phase_schedule(20.,140.,1.2,6.5)
    assert schedule["maximum_total_ticks"]==2800 and schedule["stationary_tail"]["scheduled_ticks"]==130
    assert schedule["driving"]["maximum_ticks"]==2646


def test_launch_first_gear_then_disclosed_slew_not_speed_label_clipping():
    policy=collector.NaturalGoalLaunchControl()
    c=apply(policy)
    assert c.throttle==.11 and c.brake==0 and c.manual_gear_shift is True and c.gear==1 and c.steer==0
    c=apply(policy,speed=.5,timestamp=2.,frame=2)
    assert c.throttle==pytest.approx(.11125)
    c=apply(policy,speed=2.,timestamp=6.,frame=3)
    assert c.throttle==.2 and c.steer==.1
    assert policy.last["measured_planar_speed_mps"]==2.
    assert policy.last["governor_target_reset_to_cruise"] is False


@pytest.mark.parametrize("brake,target",[(.001,2.4),(.1,2.4),(.1,0.),(0.,0.)])
def test_goal_normal_brake_or_zero_target_never_overwritten_with_launch_throttle(brake,target):
    policy=collector.NaturalGoalLaunchControl()
    c=apply(policy,brake=brake,target=target,throttle=0.)
    assert c.brake==brake and c.throttle==0.
    assert policy.last["normal_braking_preserved"] is True
    assert policy.started_at is None


class NativeFloat32Control(SimpleNamespace):
    """HH_261002 - Simulate actual VehicleControl pedal/steer float32 assignment, not Python decimal fields."""
    def __setattr__(self,name,value):
        if name in ("throttle","brake","steer"):value=collector.native_float32(value)
        super().__setattr__(name,value)


@pytest.mark.parametrize("brake",[.10,0.])
def test_actual_float32_cap_readbacks_accepted_without_rewriting_observed_values(brake):
    p=collector.NaturalGoalLaunchControl()
    c=NativeFloat32Control()
    for k,v in vars(control(brake=brake)).items():setattr(c,k,v)
    before=collector.base.control_dict(c)
    assert c.throttle>.20
    if brake:assert c.brake>.10
    p.apply(c,speed=1.,timestamp=1.,frame=1,target_speed_mps=8.64/3.6,normal_proposal=before)
    assert c.brake==before["brake"]
    assert p.last["capped_normal_proposal"]==before
    assert p.last["actual_governor_target_speed_mps"]==8.64/3.6


@pytest.mark.parametrize("field,cap",[("brake",.1),("throttle",.2)])
def test_next_float32_above_exact_requested_cap_still_rejected(field,cap):
    bits=struct.unpack("!I",struct.pack("!f",cap))[0]
    next_float=struct.unpack("!f",struct.pack("!I",bits+1))[0]
    with pytest.raises(Exception):apply(collector.NaturalGoalLaunchControl(),**{field:next_float})


def test_zero_steering_is_startup_only_not_later_goal_stop_rewrite():
    p=collector.NaturalGoalLaunchControl()
    apply(p,speed=.76)
    c=apply(p,speed=.01,timestamp=2.,frame=2,target=0.,brake=.1,throttle=0.,steer=.08)
    assert c.steer==.08 and p.last["startup_only_zero_steering"] is False


@pytest.mark.parametrize("bad",[dict(speed=-1),dict(speed=float("nan")),dict(timestamp=float("inf")),dict(target=2.401),dict(frame=True)])
def test_launch_invalid_actual_inputs_fail_closed(bad):
    with pytest.raises(Exception):apply(collector.NaturalGoalLaunchControl(),**bad)


def test_launch_timeout_and_clock_reversal_not_automatic_retry():
    p=collector.NaturalGoalLaunchControl();apply(p)
    with pytest.raises(Exception):apply(p,timestamp=.99,frame=2)
    with pytest.raises(Exception):apply(p,timestamp=9.,frame=3)


def test_all_event_defined_moving_and_hold_anchors_kept_with_actual_tail_context():
    instance=task.MovingTask(task.fixed_contract("v1","pilot_00"))
    for f in range(801):
        observation(instance,f,complete=f==800,reason="comfortable_goal_measured_stop_and_dwell" if f==800 else None)
        if f%2==0:instance.saved_camera(camera(f))
    assert instance.first["frame"]==80 and instance.last["frame"]==800 and len(instance.selected)==361
    assert instance.support is None and instance.failure is None
    for f in range(801,931):
        observation(instance,f,phase="stationary_tail",complete=True)
        if f%2==0:
            c=camera(f,"stationary_tail");instance.saved_camera(c);assert c["moving_task_selected"] is False
    result=instance.result(tail_states=130,goal_reached=True)
    assert result["task_complete"] is True and result["selected_camera_span_ns"]==36_000_000_000
    assert instance.support["timestamp_ns"]>=instance.last["timestamp_ns"]+6_400_000_000


@pytest.mark.parametrize("reason",["basic_agent_done_before_comfortable_goal_window","moving_task_future_support_complete","comfortable_goal_endpoint_overshoot"])
def test_other_teacher_endings_never_counted_as_natural_goal_hold(reason):
    instance=task.MovingTask(task.fixed_contract("v1","pilot_00"))
    _,returned=observation(instance,0,reason=reason)
    assert instance.failure is not None and returned==instance.failure
    assert instance.result(tail_states=130,goal_reached=False)["task_complete"] is False


def test_tail_without_hold_and_short_capture_not_claimed_complete():
    instance=task.MovingTask(task.fixed_contract("v1","pilot_00"));observation(instance,0,phase="stationary_tail")
    assert instance.failure=="natural_goal_tail_without_real_driving_hold"
    instance=task.MovingTask(task.fixed_contract("v1","pilot_00"))
    for f in range(101):
        observation(instance,f,complete=f==100,reason="comfortable_goal_measured_stop_and_dwell" if f==100 else None)
        if f%2==0:instance.saved_camera(camera(f))
    for f in range(101,231):observation(instance,f,phase="stationary_tail",complete=True)
    assert instance.result(tail_states=130,goal_reached=True)["task_complete"] is False


@pytest.mark.parametrize("changed,reason",[(dict(event_count=1),"natural_goal_collision_or_lane_event"),
    (dict(native_actor_speed_mps=2.7001),"natural_goal_native_speed_guard"),
    (dict(rear_base_CTE_m=1.001),"natural_goal_rear_CTE_guard"),
    (dict(observed_output_bytes=task.QUOTA-task.RESERVE+1),"natural_goal_episode_quota"),
    (dict(observed_free_disk_bytes=task.FLOOR+task.RESERVE-1),"natural_goal_disk_floor")])
def test_real_guards_retained_and_no_label_quality_filter(changed,reason):
    i=task.MovingTask(task.fixed_contract("v1","pilot_00"));observation(i,0,**changed)
    assert i.failure==reason


def test_parsing_scope_restores_original_helpers_on_exception():
    names=("moving_capture_module","configuration_from_args")
    saved={n:getattr(collector.base,n) for n in names}
    with pytest.raises(RuntimeError):
        with collector.parsing_scope():
            assert collector.base.configuration_from_args is task.goal_configuration
            raise RuntimeError("synthetic")
    assert all(getattr(collector.base,n) is v for n,v in saved.items())


def test_real_base_parse_arguments_with_exact_new_provider(monkeypatch):
    original=task.configuration_from_args
    monkeypatch.setattr(task,"configuration_from_args",lambda args:dict(run=args.natural_goal_run_id))
    argv=owner.capture_argv("v1","pilot_00")
    args=collector.parse_args(argv)
    assert args.natural_goal_run_id=="v1" and args.target_speed_kmh==8.64 and args.goal_stop_profile=="comfortable_v1"
    assert args.agent_initialization=="after_bootstrap" and args.seed==task.SEED
    assert task.configuration_from_args is not original


def test_owner_restores_every_original_alias_and_rejects_nesting():
    names=("task","worker_module","MODULE","SCHEMA","CASE_IDS","capture_argv","execution_contract","runtime_sources","disk_check","diagnostic_report")
    saved={n:getattr(owner.previous,n) for n in names}
    with pytest.raises(RuntimeError):
        with owner.activated():
            assert owner.previous.task is task
            with pytest.raises(Exception):
                with owner.activated():pass
            raise RuntimeError("synthetic")
    assert all(getattr(owner.previous,n) is v for n,v in saved.items()) and owner._active is False


def test_minimum_one_pilot_reserve_is_not_deleted_or_overwritten():
    needed=task.FLOOR+task.QUOTA+2*task.RESERVE
    assert owner.disk_check(needed)["prospective_cases"]==1
    with pytest.raises(Exception):owner.disk_check(needed-1)


def test_private_worker_assertion_precedes_scope_and_capture_and_no_model_import():
    source=Path(owner.previous.__file__).read_text()
    function=next(n for n in ast.parse(source).body if isinstance(n,ast.FunctionDef) and n.name=="worker")
    segment=ast.get_source_segment(source,function)
    assert segment.index("assert_owned_worker")<segment.index("with worker_module.scope")<segment.index("base.run")
    source=Path(collector.__file__).read_text()
    assert "class TargetRamp" not in source and "def run_step(self" not in source
    assert "FirstGearRecoveryLaunchControl(" not in source
    assert "checked_agent_initialization_observation=manual.checked_first_gear_initialization_observation" in source
    assert "control.brake = 0" not in source and "simulator_control_authorized=True" not in source
