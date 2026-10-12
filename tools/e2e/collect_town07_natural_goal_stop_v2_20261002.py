"""HH_261002 - Explicit v2 provider/cap overlay; frozen v1 snapshot, pixel, governor and hazard machinery unchanged."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import sys

from scripts.e2e import collect_town07_natural_goal_stop_20261002 as previous
from scripts.e2e import town07_natural_goal_stop_task_v2_20261002 as task

base, manual = previous.base, previous.manual
native_float32 = previous.native_float32
NORMAL_BRAKE_READBACK_BOUND = native_float32(task.NORMAL_BRAKE_CAP)
NORMAL_THROTTLE_READBACK_BOUND = previous.NORMAL_THROTTLE_READBACK_BOUND
_active = False


class NaturalGoalLaunchControl(previous.NaturalGoalLaunchControl):
    """HH_261002 - Strict actual float32 .01 readback guard before unchanged startup/brake-preservation logic."""
    def apply(self, control, **kwargs):
        task.require(0 <= float(control.brake) <= NORMAL_BRAKE_READBACK_BOUND,
            "exact float32 roundtrip of NEW .01 normal brake cap required; no observed pedal rewrite")
        result = super().apply(control,**kwargs)
        self.last.update(schema=task.SCHEMA+".normal_control", declared_normal_brake_cap=.01,
            actual_brake_readback_bound=NORMAL_BRAKE_READBACK_BOUND,
            native_float32_globals_modified=False)
        return result


@contextmanager
def parsing_scope():
    names = dict(moving_capture_module=lambda args:task, configuration_from_args=task.goal_configuration)
    saved = {name:getattr(base,name) for name in names}
    try:
        for name,value in names.items():setattr(base,name,value)
        yield
    finally:
        for name,value in saved.items():setattr(base,name,value)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(add_help=False,allow_abbrev=False)
    parser.add_argument("--natural-goal-run-id",required=True,choices=("v2",))
    ours,remaining = parser.parse_known_args(argv)
    original = task.configuration_from_args
    def attach(parsed):
        parsed.natural_goal_run_id = ours.natural_goal_run_id
        return original(parsed)
    task.configuration_from_args = attach
    try:
        with parsing_scope():args = base.parse_args(remaining)
    finally:task.configuration_from_args = original
    args.natural_goal_run_id = ours.natural_goal_run_id
    task.configuration_from_args(args)
    return args


def install_v2_cap(original_installer, planner, requested_cap, throttle_cap=None):
    # HH_261002 - Both frozen Teacher's .10 request and original collector's v2 .01 configuration use the NEW .01 cap.
    task.require(requested_cap in (.10,.01) and throttle_cap == .20,
        "only known frozen normal-planner cap calls may use the v2 provider")
    return original_installer(planner,task.NORMAL_BRAKE_CAP,throttle_cap)


@contextmanager
def scope(args):
    global _active
    task.require(not _active and not previous._active, "v2 goal collector cannot nest or attach to another scope")
    task.verify_frozen()
    from agents.navigation import basic_agent as agent_module
    saved = dict(task=previous.task, policy=previous.NaturalGoalLaunchControl,
        installer=base.install_normal_brake_cap, collect=base.collect_episode)
    session = dict(manifest=None)

    def cap(planner, requested_cap, throttle_cap=None):
        return install_v2_cap(saved["installer"],planner,requested_cap,throttle_cap)

    def collect(*args,**kwargs):
        manifest = args[6]
        session["manifest"] = manifest
        # HH_261002 - The frozen collect overlay calls this before any agent/native collection starts.
        manifest["control_source"] = "Disclosed_BasicAgent_natural_goal_stop_gentle_normal_brakes_manual1_v2"
        return saved["collect"](*args,**kwargs)

    _active = True
    try:
        with task.activated():
            previous.task, previous.NaturalGoalLaunchControl = task, NaturalGoalLaunchControl
            base.install_normal_brake_cap,base.collect_episode = cap,collect
            with previous.scope(args):
                frozen_teacher = agent_module.BasicAgent

                class Teacher(frozen_teacher):
                    def __init__(self,*args,**kwargs):
                        super().__init__(*args,**kwargs)
                        task.require(session["manifest"] is not None, "actual capture manifest must precede teacher construction")
                        metadata = session["manifest"]["capture_contract"]["teacher_controller"]
                        task.require(metadata["normal_brake_cap"] == .01 and metadata["normal_throttle_cap"] == .20,
                            "actual installed v2 planner cap differs")
                        metadata.update(normal_cap_readback_comparison=
                            "exact float32 roundtrip of NEW .01 brake/.20 throttle requests; no observed value clipping",
                            normal_launch_controller_revision=task.SCHEMA+".normal_control",
                            prospective_desired_deceleration_mps2=.2, governor_math_changed=False,
                            individual_factor_causal_effect_claim=False, native_float32_globals_modified=False)

                agent_module.BasicAgent = Teacher
                try:yield
                finally:agent_module.BasicAgent = frozen_teacher
    finally:
        previous.task, previous.NaturalGoalLaunchControl = saved["task"],saved["policy"]
        base.install_normal_brake_cap,base.collect_episode = saved["installer"],saved["collect"]
        _active = False


def main(argv=None):
    print("Private v2 natural-goal worker: use run_owned_town07_natural_goal_stop_v2_20261002 --mode execute",file=sys.stderr)
    return 1


if __name__ == "__main__":raise SystemExit(main())
