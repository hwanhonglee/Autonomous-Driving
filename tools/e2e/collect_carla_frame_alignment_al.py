"""HH_260906 - Rear-axle waypoint/lateral-control A/B on the fixed AK Town01 trial."""
from __future__ import annotations

from contextlib import contextmanager
from dataclasses import replace
import hashlib
import inspect
import math
import os
from pathlib import Path
import sys

from . import collect_carla_goal_stop_ak as ak
from . import collect_carla_vad_expert as base


# HH_260906 - Fail closed if CARLA changes the two getter call sites this proxy is designed for.
UPSTREAM_SOURCE_SHA256 = {
    "local_planner.py": "7a5db9112b0d107038765abfb87b2d7a418b98e4d2c0aa92a4aade53c95b319f",
    "controller.py": "f95f427a5ad5ba50a96f6971df72a5f3ec078c8457b0ba2a12558396e8b5c402",
}
ARM = "rear_axle"
PROFILE_ID = "comfortable_v2_al_rear_axle"
WHEELBASE_M = base.WHEELBASE_M
REAR_SHIFT_M = WHEELBASE_M / 2.0


def validate_args(args):
    """HH_260906 - Preserve AK B's complete protocol and use its exact 2.85 m label wheelbase."""
    if getattr(args, "al_arm", None) != ARM or float(args.wheelbase_m) != WHEELBASE_M:
        raise ValueError("AL requires rear_axle arm and the exact 2.85 m labelled wheelbase")
    args.ak_arm = "coast_27_hold"
    parent = ak.validate_args(args)
    return replace(parent, profile_id=PROFILE_ID,
                   policy_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())


def parse_args(argv=None):
    """HH_260906 - Require an explicit AL arm before any simulator access."""
    import argparse

    parser = argparse.ArgumentParser(add_help=False, allow_abbrev=False)
    parser.add_argument("--al-arm", choices=(ARM,), required=True)
    own, rest = parser.parse_known_args(argv)
    args = base.parse_args(rest)
    args.al_arm = own.al_arm
    validate_args(args)
    return args


def verify_upstream_sources(planner):
    """HH_260906 - Confirm the exact installed CARLA implementation used by this scoped intervention."""
    from agents.navigation.controller import PIDLateralController
    from agents.navigation.local_planner import LocalPlanner

    controller = planner._vehicle_controller
    if type(planner) is not LocalPlanner or type(controller._lat_controller) is not PIDLateralController:
        raise ValueError("AL requires unmodified CARLA LocalPlanner and PIDLateralController")
    carla_root = os.environ.get("CARLA_ROOT")
    if not carla_root:
        raise ValueError("AL requires an explicit CARLA_ROOT")
    origin = Path(carla_root).resolve(strict=True) / "PythonAPI/carla/agents/navigation"
    proven = {}
    for filename, control_class in (("local_planner.py", LocalPlanner),
                                    ("controller.py", PIDLateralController)):
        source = inspect.getsourcefile(control_class)
        expected_path = origin / filename
        if source is None or Path(source).resolve(strict=True) != expected_path.resolve(strict=True):
            raise ValueError("AL CARLA PythonAPI source differs from owned CARLA_ROOT")
        digest = hashlib.sha256(expected_path.read_bytes()).hexdigest()
        if digest != UPSTREAM_SOURCE_SHA256[filename]:
            raise ValueError("AL CARLA PythonAPI source bytes differ from the pinned getter contract")
        proven[filename] = digest
    return proven


class RearAxleControlProxy:
    """HH_260906 - Expose only the LocalPlanner getters needed by the pinned CARLA source."""

    __slots__ = ("_actor", "_rear_shift_m")

    def __init__(self, actor, rear_shift_m=REAR_SHIFT_M):
        if not math.isfinite(rear_shift_m) or rear_shift_m != REAR_SHIFT_M:
            raise ValueError("AL rear-axle shift must match the labelled 2.85 m wheelbase")
        self._actor = actor
        self._rear_shift_m = rear_shift_m

    def get_transform(self):
        original = self._actor.get_transform()
        location = original.location
        forward = original.get_forward_vector()
        values = (location.x, location.y, location.z, forward.x, forward.y, forward.z)
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("AL rear-axle proxy requires a finite actor transform")
        shifted = type(location)(
            x=float(location.x) - self._rear_shift_m * float(forward.x),
            y=float(location.y) - self._rear_shift_m * float(forward.y),
            z=float(location.z) - self._rear_shift_m * float(forward.z),
        )
        return type(original)(shifted, original.rotation)

    def get_location(self):
        return self.get_transform().location

    def get_velocity(self):
        return self._actor.get_velocity()

    def get_speed_limit(self):
        return self._actor.get_speed_limit()

    def get_world(self):
        return self._actor.get_world()


def aligned_installer(planner, cap, throttle_cap=None):
    """HH_260906 - Shift only purge and lateral PID for one normal control call, then restore references."""
    governor = ak._active_governor
    if governor is None or governor.config.profile_id != PROFILE_ID or governor.config.ak_arm != "coast_27_hold":
        raise ValueError("AL requires the measured AK B governor")
    controller = planner._vehicle_controller
    actor = planner._vehicle
    lateral = controller._lat_controller
    longitudinal = controller._lon_controller
    if controller._vehicle is not actor or lateral._vehicle is not actor or longitudinal._vehicle is not actor:
        raise ValueError("AL refuses unexpected BasicAgent actor references")
    upstream = verify_upstream_sources(planner)
    proxy = RearAxleControlProxy(actor)
    metadata = ak.ak_installer(planner, cap, throttle_cap)
    normal_step = planner.run_step
    active = False

    def frame_aligned_step(*args, **kwargs):
        nonlocal active
        if active or planner._vehicle is not actor or lateral._vehicle is not actor:
            raise ValueError("AL normal planner call is reentrant or actor references changed")
        if controller._vehicle is not actor or longitudinal._vehicle is not actor:
            raise ValueError("AL longitudinal controller lost its original actor")
        active = True
        planner._vehicle = proxy
        lateral._vehicle = proxy
        try:
            return normal_step(*args, **kwargs)
        finally:
            lateral._vehicle = actor
            planner._vehicle = actor
            active = False

    planner.run_step = frame_aligned_step
    metadata.update(
        al_arm=ARM, control_reference="rear_axle_proxy_for_waypoint_purge_and_lateral_PID_only",
        rear_axle_shift_m=REAR_SHIFT_M, route_and_stop_goal_reference_unchanged=True,
        basic_agent_actor_reference_unchanged=True, longitudinal_PID_actor_reference_unchanged=True,
        emergency_override_remains_after_normal_planner=True,
        upstream_carla_pythonapi_sha256=upstream,
        policy_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        development_only=True, training_data_approved=False,
    )
    return metadata


@contextmanager
def configuration_scope():
    """HH_260906 - Nest the frozen AK hooks and restore every AL override on failure."""
    with ak.configuration_scope():
        if (base.configuration_from_args is not ak.validate_args
                or base.GoalStopGovernor is not ak.AKGovernor
                or base.install_normal_brake_cap is not ak.ak_installer):
            raise ValueError("unexpected AK provider before AL installation")
        base.configuration_from_args = validate_args
        base.install_normal_brake_cap = aligned_installer
        try:
            yield
        finally:
            base.configuration_from_args = ak.validate_args
            base.install_normal_brake_cap = ak.ak_installer


def main(argv=None):
    try:
        args = parse_args(argv)
        with configuration_scope():
            output = base.run(args)
    except (base.CollectionError, OSError, ValueError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    print(f"capture={output}; AL development only, NOT admitted")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
