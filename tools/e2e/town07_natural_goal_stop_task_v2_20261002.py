"""HH_261002 - Prospective two-factor gentle goal-stop pilot; original event membership and governor math."""

from __future__ import annotations

from contextlib import contextmanager
from dataclasses import replace

from scripts.e2e import town07_natural_goal_stop_task_20261002 as previous

ROOT, ROUTE, ROUTE_SHA = previous.ROOT, previous.ROUTE, previous.ROUTE_SHA
FLOOR, RESERVE, QUOTA, SEED = previous.FLOOR, previous.RESERVE, previous.QUOTA, previous.SEED
require, sha, read, encoded, canonical = previous.require, previous.sha, previous.read, previous.encoded, previous.canonical
original, goal = previous.original, previous.goal
measured_progress = previous.measured_progress
validate_original_physics, validate_applied_physics = previous.validate_original_physics, previous.validate_applied_physics
SCHEMA = "carla.actual_natural_goal_stop_teacher.20261002.v2"
PROFILE = "natural_goal_stop_manual1_2p40_brake0p01_decel0p20_hold0p05.20261002.v2"
FIXED_ARGS = dict(previous.FIXED_ARGS)
NORMAL_BRAKE_CAP = .01
DESIRED_DECELERATION_MPS2 = .2
FROZEN_V1 = {
    "tools/e2e/town07_natural_goal_stop_task_20261002.py": "a636058d890c6c28d47bec9de8c134ccaff3594c724ed67860ed8fd16324f35b",
    "tools/e2e/collect_town07_natural_goal_stop_20261002.py": "8eee92c1c83535316ff19189823faa898b10a3344eb6e43eb5844a94f184ce72",
    "tools/e2e/run_owned_town07_natural_goal_stop_20261002.py": "f05fdf610dbd8ec9ec410beb858af1c588b120f5a13b5cc39943d47d0a7eb1ce",
}
_fixed_contract = previous.fixed_contract
_configuration_from_args = previous.configuration_from_args
_reconstruct = previous.reconstruct
_active = False


def verify_frozen():
    for name, expected in FROZEN_V1.items():
        require(sha(ROOT/name) == expected, "frozen natural-goal v1 source changed: "+name)


def paths(run_id, case_id=None):
    require(run_id == "v2" and case_id in (None, "pilot_00"), "one NEW v2 pilot only; no v1 output reuse")
    batch = ROOT/"storage/experiments/validation/2026-10-02/town07_natural_goal_stop_teacher_v2"
    return (batch, batch.with_name(batch.name+"_plan.json")) if case_id is None else (batch/case_id, batch/(case_id+"_plan.json"))


def goal_configuration(args):
    require(args.goal_stop_profile == "comfortable_v1" and args.target_speed_kmh == 8.64
        and args.physics_hz == 20. and args.capture_hz == 10. and args.goal_tolerance_m == 1.
        and args.stationary_tail_sec == 6.5, "fixed v2 natural-goal configuration required")
    return replace(goal.ComfortableGoalStopConfig(), profile_id=PROFILE, stopped_speed_mps=.05,
        normal_throttle_cap=.20, normal_brake_cap=NORMAL_BRAKE_CAP,
        desired_deceleration_mps2=DESIRED_DECELERATION_MPS2)


@contextmanager
def activated():
    """HH_261002 - Rebind only the frozen task provider API, never native float or governor arithmetic."""
    global _active
    require(not _active, "v2 task activation cannot nest")
    names = dict(SCHEMA=SCHEMA, PROFILE=PROFILE, FIXED_ARGS=FIXED_ARGS, paths=paths,
        pending_scope=pending_scope, goal_configuration=goal_configuration, fixed_contract=fixed_contract, MovingTask=MovingTask)
    saved = {name:getattr(previous,name) for name in names}
    _active = True
    try:
        for name, value in names.items():setattr(previous,name,value)
        yield
    finally:
        for name, value in saved.items():setattr(previous,name,value)
        _active = False


@contextmanager
def _bound():
    if _active:
        yield
    else:
        with activated():yield


# HH_261002 - Keep the original pending function outside the temporary provider alias to avoid recursion.
_pending_scope = previous.pending_scope


def pending_scope():
    # HH_261002 - Native PENDING_QA is neither conversion nor training authorization.
    result = _pending_scope()
    result["qualification_notice"] = "NEW two-factor .01 normal-brake/.2 target-deceleration teacher pilot; actual full QA and separate admission required."
    return result


def fixed_contract(run_id, case_id):
    with _bound():result = _fixed_contract(run_id,case_id)
    result["controller"]["normal_brake_cap"] = NORMAL_BRAKE_CAP
    result["prospective_two_factor_revision"] = dict(
        normal_brake_cap=dict(baseline=.10, prospective=.01),
        desired_deceleration_mps2=dict(baseline=1.5, prospective=.2),
        governor_math_changed=False, individual_factor_causal_effect_claim=False,
        same_route_seed_weather_and_all_quality_bounds=True,
        earlier_target_envelope_is_not_measured_brake_capability=True)
    result["frozen_v1_source_sha256"] = dict(FROZEN_V1)
    result["teacher"] = "Original BasicAgent hazards and original GoalStopGovernor math; NEW .01 brake cap/.2 desired deceleration with unchanged manual1/.11 launch."
    return result


def configuration_from_args(args):
    with _bound():return _configuration_from_args(args)


class MovingTask(previous.MovingTask):
    """HH_261002 - Delegate every native event, selection and guard unchanged under the explicit v2 provider."""
    def __init__(self, plan):
        with _bound():super().__init__(plan)

    def observe(self, **kwargs):
        with _bound():return super().observe(**kwargs)

    def result(self, **kwargs):
        with _bound():return super().result(**kwargs)


def reconstruct(plan, states, cameras, route_points):
    with _bound():return _reconstruct(plan,states,cameras,route_points)
