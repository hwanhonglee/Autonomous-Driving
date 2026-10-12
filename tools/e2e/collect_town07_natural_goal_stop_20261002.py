"""HH_261002 - Actual goal-stop teacher, normal brakes and governor targets preserved; private owned worker only."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import hashlib
import json
import math
import struct
import sys
from types import SimpleNamespace

from scripts.e2e import collect_carla_vad_expert as base
from scripts.e2e import run_owned_town07_signed_recovery_v3_20261001 as manual
from scripts.e2e import town07_natural_goal_stop_task_20261002 as task

_active = False


def native_float32(value):
    """HH_261002 - Compare exact CARLA field representation; never replace observed control or requested cap."""
    return struct.unpack("!f",struct.pack("!f",value))[0]


NORMAL_BRAKE_READBACK_BOUND=native_float32(.10)
NORMAL_THROTTLE_READBACK_BOUND=native_float32(.20)


def runtime_provider(guarded_task):
    return SimpleNamespace(**{name:getattr(task,name) for name in ("configuration_from_args","pending_scope","reconstruct",
        "validate_original_physics","validate_applied_physics","measured_progress","canonical")},
        MovingTask=guarded_task,__file__=task.__file__)


@contextmanager
def parsing_scope():
    names=dict(moving_capture_module=lambda args:task,configuration_from_args=task.goal_configuration)
    saved={name:getattr(base,name) for name in names}
    try:
        for name,value in names.items():setattr(base,name,value)
        yield
    finally:
        for name,value in saved.items():setattr(base,name,value)


def parse_args(argv=None):
    parser=argparse.ArgumentParser(add_help=False,allow_abbrev=False)
    parser.add_argument("--natural-goal-run-id",required=True,choices=("v1",))
    ours,remaining=parser.parse_known_args(argv)
    original=task.configuration_from_args
    def attach(parsed):
        parsed.natural_goal_run_id=ours.natural_goal_run_id
        return original(parsed)
    task.configuration_from_args=attach
    try:
        with parsing_scope():args=base.parse_args(remaining)
    finally:task.configuration_from_args=original
    args.natural_goal_run_id=ours.natural_goal_run_id
    task.configuration_from_args(args)
    return args


class NaturalGoalLaunchControl:
    """HH_261002 - Startup-only .11 launch; never erase normal braking or replace the measured goal target."""
    def __init__(self):
        self.started_at=self.previous_time=self.handoff_at=None
        self.startup_steering_completed=False
        self.last=None

    def apply(self,control,*,speed,timestamp,frame,target_speed_mps,normal_proposal):
        task.require(all(type(v) in (int,float) and math.isfinite(v) for v in (speed,timestamp,target_speed_mps))
            and speed>=0 and 0<=target_speed_mps<=2.4+1e-12 and type(frame) is int and frame>=0,
            "finite actual speed/time/governor target required")
        task.require(self.previous_time is None or timestamp>=self.previous_time,"native launch clock reversed")
        self.previous_time=timestamp
        before=base.control_dict(control)
        task.require(0<=float(control.brake)<=NORMAL_BRAKE_READBACK_BOUND
            and 0<=float(control.throttle)<=NORMAL_THROTTLE_READBACK_BOUND,
            "original normal brake/throttle caps must precede the new launch hook")
        braking=float(control.brake)>0 or bool(control.hand_brake) or bool(control.reverse) or target_speed_mps<=0
        if not braking and self.started_at is None:self.started_at=timestamp
        if self.handoff_at is None and speed>=.5:self.handoff_at=timestamp
        if self.started_at is not None and self.handoff_at is None and timestamp-self.started_at>=8.:
            raise ValueError("natural-goal launch did not reach actual .5m/s within8s")
        cap=.11 if self.handoff_at is None else min(.20,.11+.025*(timestamp-self.handoff_at+.05))
        if not braking:
            control.throttle=.11 if self.handoff_at is None else min(float(control.throttle),cap)
        if speed>.75:self.startup_steering_completed=True
        startup_zero=not self.startup_steering_completed and speed<=.75
        if startup_zero:control.steer=0.
        control.manual_gear_shift,control.gear=True,1
        self.last=dict(schema=task.SCHEMA+".normal_control",source_frame=frame,source_timestamp=timestamp,
            measured_planar_speed_mps=speed,actual_governor_target_speed_mps=target_speed_mps,
            raw_normal_planner_proposal=dict(normal_proposal),capped_normal_proposal=before,
            profile_normal_control=base.control_dict(control),normal_braking_preserved=braking,
            launch_started_at=self.started_at,handoff_at=self.handoff_at,normal_throttle_cap=cap,
            startup_only_zero_steering=startup_zero,startup_steering_completed=self.startup_steering_completed,
            insertion="LocalPlanner normal proposal before unchanged BasicAgent hazard override",
            final_hazard_return_modified_after_override=False,actual_measured_speed_clipped=False,
            governor_target_reset_to_cruise=False,learned_control=False)
        return control


@contextmanager
def scope(args):
    global _active
    task.require(not _active,"natural-goal worker scope cannot nest")
    from agents.navigation import basic_agent as agent_module
    plan=task.configuration_from_args(args)
    manual.verify_frozen_dependencies()
    original_agent=agent_module.BasicAgent
    names=("collect_episode","AcknowledgedControlTransport","moving_capture_module","acknowledged_snapshot_state",
        "_save_jpeg","_blank_brake_control","checked_agent_initialization_observation","configuration_from_args","GoalStopGovernor")
    saved={name:getattr(base,name) for name in names}
    session=dict(output=None,manifest=None,latest_state=None,latest_raw=None,latest_frame=None,goal=None,target=None,policy=None)
    projector=base.RouteProjector(base.load_route(task.ROUTE))
    pixels={}

    def journal(name,value):
        if session["output"] is not None:
            with (session["output"]/name).open("a") as stream:stream.write(json.dumps(value,allow_nan=False)+"\n")

    def save_image(image,path,quality):
        task.require(len(image.raw_data)==image.width*image.height*4,"exact original BGRA payload required")
        original_sha=hashlib.sha256(image.raw_data).hexdigest()
        saved["_save_jpeg"](image,path,quality)
        name=path.parent.name
        task.require(name in base.MODEL_CAMERA_ORDER,"one declared camera required")
        records=pixels.setdefault(int(image.frame),{})
        task.require(name not in records,"duplicate source camera")
        row=dict(camera=name,frame=int(image.frame),timestamp=float(image.timestamp),width=int(image.width),height=int(image.height),
            encoding="original_carla_bgra8",original_BGRA_sha256=original_sha,saved_JPEG_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            saved_JPEG_relative_path=str(path.relative_to(session["output"])),JPEG_encoder_unchanged=True,pose_or_image_post_transformed=False)
        records[name]=row;journal("actual_camera_pixels.jsonl",row)

    def snapshot(*p,**kw):
        state,raw=saved["acknowledged_snapshot_state"](*p,**kw)
        session.update(latest_state=state,latest_raw=raw,latest_frame=int(p[0].frame))
        return state,raw

    class RecordedGovernor(saved["GoalStopGovernor"]):
        def update(self,*p,**kw):
            result=super().update(*p,**kw)
            session["goal"]=result
            journal("actual_goal_governor.jsonl",dict(schema=task.SCHEMA+".actual_governor_update",
                measured_state_frame=session["latest_frame"],count_hold=kw.get("count_hold",True),
                original_governor_update=dict(result),governor_math_changed=False,measured_speed_clipped=False))
            return result

    class GuardedTask(task.MovingTask):
        def observe(self,**kw):
            state,raw=session["latest_state"],session["latest_raw"]
            task.require(state is not None and raw is not None and session["goal"] is not None,"actual native and governor observations required")
            total=sum(p.stat().st_size for p in session["output"].rglob("*") if p.is_file())
            return super().observe(**kw,native_actor_speed_mps=math.hypot(*raw["world_velocity_carla"][:2]),
                rear_base_CTE_m=projector.project(state["x"],state["y"],kw["progress"]).cross_track_error_m,
                observed_output_bytes=total,goal_record=dict(session["goal"]))
        def saved_camera(self,camera):
            records=pixels.pop(camera["frame"],{})
            task.require(set(records)==set(base.MODEL_CAMERA_ORDER) and all(records[n]["timestamp"]==camera["source_timestamps"][n] for n in records),
                "all six original source pixel/timestamp identities required")
            camera["original_BGRA_source_sha256"]={n:r["original_BGRA_sha256"] for n,r in records.items()}
            camera["saved_JPEG_sha256"]={n:r["saved_JPEG_sha256"] for n,r in records.items()}
            return super().saved_camera(camera)

    class Teacher(original_agent):
        def __init__(self,*p,**kw):
            super().__init__(*p,**kw)
            planner=self.get_local_planner()
            original,observed=planner.run_step,{"value":None}
            def raw(*a,**k):
                result=original(*a,**k);observed["value"]=base.control_dict(result);return result
            planner.run_step=raw
            metadata=base.install_normal_brake_cap(planner,.10,.20)
            normal,policy=planner.run_step,NaturalGoalLaunchControl()
            session["policy"]=policy
            def step(*a,**k):
                result=normal(*a,**k);velocity=self._vehicle.get_velocity();native=self._vehicle.get_world().get_snapshot()
                target=session["target"]
                task.require(target is not None and target["source_frame"]==native.frame,"actual current governor target assignment required")
                return policy.apply(result,speed=math.hypot(velocity.x,velocity.y),timestamp=native.timestamp.elapsed_seconds,
                    frame=native.frame,target_speed_mps=target["target_speed_mps"],normal_proposal=observed["value"])
            planner.run_step=step
            session["manifest"]["capture_contract"]["teacher_controller"]=dict(metadata,
                normal_launch_controller_revision=task.SCHEMA+".normal_control",gear=1,manual_gear_shift=True,
                bootstrap_and_stationary_blank_full_brake_first_gear=True,governor_target_reset_to_cruise=False,
                normal_brakes_permitted=True,inherited_BasicAgent_emergency_unchanged=True,
                normal_cap_readback_comparison="exact float32 roundtrip of original .10 brake/.20 throttle requests; no observed value clipping",
                initialization_gear_contract=dict(actual_manual_gear_shift=True,actual_gear=1,
                    parent_function_source_sha256=manual.PARENT_CHECKER_SOURCE_SHA,
                    all_original_frame_actor_finite_PID_and_full_brake_guards_preserved=True,observed_control_or_snapshot_proxy_used=False))
        def set_target_speed(self,speed):
            # HH_261002 - Observe the collector assignment, never replace it with a rolling cruise ramp.
            result=super().set_target_speed(speed)
            native=self._vehicle.get_world().get_snapshot()
            session["target"]=dict(source_frame=int(native.frame),source_timestamp=float(native.timestamp.elapsed_seconds),
                target_speed_mps=float(speed)/3.6,requested_target_kph=float(speed),target_changed_by_wrapper=False)
            journal("actual_goal_target_assignments.jsonl",dict(session["target"]))
            return result

    class Transport(saved["AcknowledgedControlTransport"]):
        def send(self,control,reason,expected_before_frame=None):
            receipt=super().send(control,reason,expected_before_frame)
            policy=session["policy"]
            normal=policy.last if policy and policy.last and policy.last["source_frame"]==receipt["before_frame"] else None
            journal("actual_teacher_control.jsonl",dict(schema=task.SCHEMA+".actual_teacher_command",reason=reason,normal_policy=normal,
                actual_governor_target_assignment=session["target"],final_BasicAgent_or_brake_proposal=base.control_dict(control),
                ack_receipt=receipt,learned_control=False,teacher_control=True,training_data_approved=False))
            return receipt

    def collect(*p,**kw):
        _,_,_,partial,_,_,manifest=p
        session.update(output=partial,manifest=manifest)
        manifest.update(task.pending_scope(),control_source="Disclosed_BasicAgent_natural_goal_stop_normal_brakes_manual1_v1")
        manifest["capture_contract"]["natural_goal_stop_teacher"]=plan
        return saved["collect_episode"](*p,**kw)

    def full_brake(carla):
        control=saved["_blank_brake_control"](carla)
        task.require(control.throttle==0. and control.brake==1. and control.steer==0. and not control.hand_brake and not control.reverse,
            "unchanged pure full-service brake required")
        control.manual_gear_shift,control.gear=True,1
        return control

    changes=dict(collect_episode=collect,AcknowledgedControlTransport=Transport,moving_capture_module=lambda parsed:runtime_provider(GuardedTask),
        acknowledged_snapshot_state=snapshot,_save_jpeg=save_image,_blank_brake_control=full_brake,
        checked_agent_initialization_observation=manual.checked_first_gear_initialization_observation,
        configuration_from_args=task.goal_configuration,GoalStopGovernor=RecordedGovernor)
    _active=True
    try:
        for name,value in changes.items():setattr(base,name,value)
        agent_module.BasicAgent=Teacher
        yield
    finally:
        for name,value in saved.items():setattr(base,name,value)
        agent_module.BasicAgent=original_agent
        _active=False


def main(argv=None):
    print("Private natural-goal worker: use run_owned_town07_natural_goal_stop_20261002 --mode execute",file=sys.stderr)
    return 1


if __name__=="__main__":raise SystemExit(main())
