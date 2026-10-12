"""HH_260906 - Isolate a goal-window normal-steering A/B from the fixed AM drive."""
from __future__ import annotations

import argparse
from contextlib import contextmanager
from dataclasses import replace
import hashlib
import inspect
import math
import os
from pathlib import Path
import sys

from . import collect_carla_frame_alignment_al as al
from . import collect_carla_goal_stop_ak as ak
from . import collect_carla_vad_expert as base


# HH_260906 - A is the sealed AM capture; B differs only after a measured near-goal normal command.
ARM = "goal_window_020_zero"
MAX_NORMAL_STEERING_SPEED_MPS = 0.20
AM_LATERAL_KP = 1.70
AN_BASIC_AGENT_SHA256 = "7eb47c0ac30f36e2e71266e918c27fc02c8ff7b74d8f75259c313022e50be2ff"
UPSTREAM_SOURCE_SHA256 = {**al.UPSTREAM_SOURCE_SHA256, "basic_agent.py": AN_BASIC_AGENT_SHA256}
ORIGINAL_SUPPRESS = base.suppress_stopped_brake_steering


def validate_args(args):
    """HH_260906 - Retain every AM input and reject a changed steering threshold or controller gain."""
    if getattr(args, "an_arm", None) != ARM or float(args.basic_agent_lateral_kp) != AM_LATERAL_KP:
        raise ValueError("AN requires the fixed goal-window arm and AM Kp=1.70")
    args.al_arm = al.ARM
    parent = al.validate_args(args)
    return replace(parent, policy_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())


def parse_args(argv=None):
    """HH_260906 - Require the new development arm explicitly before simulator access."""
    parser = argparse.ArgumentParser(add_help=False, allow_abbrev=False)
    parser.add_argument("--an-arm", choices=(ARM,), required=True)
    own, rest = parser.parse_known_args(argv)
    args = base.parse_args(rest)
    args.an_arm = own.an_arm
    validate_args(args)
    return args


def verify_basic_agent_source():
    """HH_260906 - Bind the post-emergency output stage to BasicAgent's positive-brake implementation."""
    from agents.navigation.basic_agent import BasicAgent

    carla_root = os.environ.get("CARLA_ROOT")
    if not carla_root:
        raise ValueError("AN requires an explicit CARLA_ROOT")
    source = (Path(carla_root).resolve(strict=True) /
              "PythonAPI/carla/agents/navigation/basic_agent.py")
    if source.is_symlink() or not source.is_file():
        raise ValueError("AN BasicAgent source is not a regular file")
    imported_source = inspect.getsourcefile(BasicAgent)
    if imported_source is None or Path(imported_source).resolve(strict=True) != source.resolve(strict=True):
        raise ValueError("AN imported BasicAgent differs from the owned CARLA_ROOT source")
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    if digest != AN_BASIC_AGENT_SHA256:
        raise ValueError("AN BasicAgent emergency-brake source differs from the pinned implementation")
    return digest


class ANGovernor(ak.AKGovernor):
    """HH_260906 - Keep the exact AK stop governor and expose only one same-frame steering guard."""

    def __init__(self, config, cruise_speed_mps, physics_hz):
        super().__init__(config, cruise_speed_mps, physics_hz)
        self.an_record = None

    def update(self, remaining_arc_m, planar_distance_m, speed_mps, terminal_overshoot,
               *, count_hold=True):
        record = super().update(remaining_arc_m, planar_distance_m, speed_mps,
                                terminal_overshoot, count_hold=count_hold)
        eligible = (count_hold and record["goal_window"] and record["upstream_of_endpoint"]
                    and terminal_overshoot <= 0.0 and speed_mps <= MAX_NORMAL_STEERING_SPEED_MPS
                    and not self.hold_latched)
        record["an_normal_steering"] = {
            "arm": ARM, "measured_guard_eligible": eligible,
            "measurement_is_driving": bool(count_hold),
            "remaining_route_arc_m": remaining_arc_m,
            "planar_goal_distance_m": planar_distance_m,
            "terminal_overshoot_m": terminal_overshoot,
            "measured_speed_mps": speed_mps,
            "speed_threshold_mps": MAX_NORMAL_STEERING_SPEED_MPS,
            "applied_to_normal_command": False,
            "pre_intervention_steer": None,
            "post_intervention_steer": None,
            "emergency_or_hold_unchanged": False,
        }
        self.an_record = record
        return record


def an_suppress_stopped_brake_steering(control, speed_mps, enabled,
                                       speed_threshold_mps=0.3):
    """HH_260906 - Zero eligible normal output after BasicAgent hazard selection, before the old guard."""
    governor = ak._active_governor
    if type(governor) is not ANGovernor or governor.an_record is None:
        raise ValueError("AN needs a same-frame measured governor before output steering")
    record = governor.an_record["an_normal_steering"]
    if not math.isfinite(speed_mps) or not math.isclose(
            speed_mps, record["measured_speed_mps"], rel_tol=0.0, abs_tol=1e-9):
        raise ValueError("AN output speed does not match its measured source frame")
    brake = float(control.brake)
    if not math.isfinite(brake):
        raise ValueError("AN refuses nonfinite BasicAgent brake")
    # HH_260906 - The pinned BasicAgent hazard branch sets brake=0.5; AK hold sets brake=1.
    normal = brake == 0.0 and not bool(control.hand_brake)
    if record["measured_guard_eligible"] and normal:
        before = float(control.steer)
        if not math.isfinite(before):
            raise ValueError("AN refuses nonfinite BasicAgent steering")
        control.steer = 0.0
        record.update(applied_to_normal_command=True, pre_intervention_steer=before,
                      post_intervention_steer=0.0)
    elif brake > 0.0 or bool(control.hand_brake):
        record["emergency_or_hold_unchanged"] = True
    old_guard_applied = ORIGINAL_SUPPRESS(control, speed_mps, enabled, speed_threshold_mps)
    return bool(record["applied_to_normal_command"] or old_guard_applied)


def an_installer(planner, cap, throttle_cap=None):
    """HH_260906 - Preserve AL's exact normal planner/PID and add output-stage provenance only."""
    governor = ak._active_governor
    if type(governor) is not ANGovernor:
        raise ValueError("AN requires its measured governor")
    metadata = al.aligned_installer(planner, cap, throttle_cap)
    metadata.update(
        an_arm=ARM,
        an_intervention="final_normal_steering_zero_inside_measured_goal_window_at_or_below_0.20_mps",
        an_stage="after_BasicAgent_hazard_decision_before_existing_stopped_brake_guard",
        an_emergency_and_hold_controls_unchanged=True,
        an_basic_agent_source_sha256=AN_BASIC_AGENT_SHA256,
        an_policy_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        training_data_approved=False,
    )
    return metadata


@contextmanager
def configuration_scope():
    """HH_260906 - Nest AL/AK hooks and restore the one post-Agent control hook on every exit."""
    with al.configuration_scope():
        if (base.configuration_from_args is not al.validate_args
                or base.GoalStopGovernor is not ak.AKGovernor
                or base.install_normal_brake_cap is not al.aligned_installer
                or base.suppress_stopped_brake_steering is not ORIGINAL_SUPPRESS):
            raise ValueError("unexpected provider before AN installation")
        base.configuration_from_args = validate_args
        base.GoalStopGovernor = ANGovernor
        base.install_normal_brake_cap = an_installer
        base.suppress_stopped_brake_steering = an_suppress_stopped_brake_steering
        try:
            yield
        finally:
            base.suppress_stopped_brake_steering = ORIGINAL_SUPPRESS
            base.install_normal_brake_cap = al.aligned_installer
            base.GoalStopGovernor = ak.AKGovernor
            base.configuration_from_args = al.validate_args


def main(argv=None):
    try:
        args = parse_args(argv)
        verify_basic_agent_source()
        with configuration_scope():
            output = base.run(args)
    except (base.CollectionError, OSError, ValueError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    print(f"capture={output}; AN development only, NOT admitted")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
