"""HH_260906 - Add one measured launch steering guard to the fixed AN BasicAgent trial."""
from __future__ import annotations

import argparse
from contextlib import contextmanager
from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path
import sys

from . import collect_carla_low_speed_steering_an as an
from . import collect_carla_goal_stop_ak as ak
from . import collect_carla_vad_expert as base


# HH_260906 - The route SHA is already pinned by AK/AJ; bind the matching measured route length too.
ARM = "start_zero_below_020"
ROUTE_LENGTH_M = 76.16005405260421
MAX_LAUNCH_PROGRESS_M = 1.0
MAX_LAUNCH_SPEED_MPS = 0.20
UPSTREAM_SOURCE_SHA256 = an.UPSTREAM_SOURCE_SHA256


def validate_args(args):
    """HH_260906 - Preserve AN's exact route, controls and training denial before simulator access."""
    if getattr(args, "ao_arm", None) != ARM:
        raise ValueError("AO requires its one fixed launch arm")
    route = json.loads(args.route_file.read_text())
    if (route.get("coordinate_reference") != "base_link"
            or route.get("route_length_m") != ROUTE_LENGTH_M):
        raise ValueError("AO route length/reference differs from the predeclared source")
    args.an_arm = an.ARM
    parent = an.validate_args(args)
    return replace(parent, policy_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())


def parse_args(argv=None):
    """HH_260906 - Require the AO arm explicitly and let AN/AL/AK lock every old input."""
    parser = argparse.ArgumentParser(add_help=False, allow_abbrev=False)
    parser.add_argument("--ao-arm", choices=(ARM,), required=True)
    own, rest = parser.parse_known_args(argv)
    args = base.parse_args(rest)
    args.ao_arm = own.ao_arm
    validate_args(args)
    return args


def ao_suppress_stopped_brake_steering(control, speed_mps, enabled,
                                       speed_threshold_mps=0.3):
    """HH_260906 - Suppress only non-braking launch output after BasicAgent's hazard branch."""
    governor = ak._active_governor
    if type(governor) is not an.ANGovernor or governor.an_record is None:
        raise ValueError("AO needs AN's same-frame measured governor")
    source = governor.an_record
    record = source["an_normal_steering"]
    if not math.isfinite(speed_mps) or not math.isclose(
            speed_mps, record["measured_speed_mps"], rel_tol=0.0, abs_tol=1e-9):
        raise ValueError("AO output speed differs from the measured source frame")
    brake = float(control.brake)
    if not math.isfinite(brake):
        raise ValueError("AO refuses nonfinite BasicAgent brake")
    within_start = (record["remaining_route_arc_m"] > ROUTE_LENGTH_M - MAX_LAUNCH_PROGRESS_M
                    and record["remaining_route_arc_m"] <= ROUTE_LENGTH_M + 1e-6
                    and not source["goal_window"])
    eligible = within_start and speed_mps < MAX_LAUNCH_SPEED_MPS
    normal = brake == 0.0 and not bool(control.hand_brake)
    applied = eligible and normal
    before = float(control.steer) if applied else None
    if applied:
        if not math.isfinite(before):
            raise ValueError("AO refuses nonfinite BasicAgent steering")
        control.steer = 0.0
    # HH_260906 - AN keeps its independent near-goal guard and the old emergency/stopped-brake guard.
    source["ao_launch_steering"] = {
        "arm": ARM, "within_first_route_meter": within_start,
        "measured_speed_mps": speed_mps, "speed_threshold_exclusive_mps": MAX_LAUNCH_SPEED_MPS,
        "remaining_route_arc_m": record["remaining_route_arc_m"],
        "route_length_m": ROUTE_LENGTH_M, "normal_zero_brake_output": normal,
        "applied_to_normal_command": applied,
        "pre_intervention_steer": before,
        "post_intervention_steer": 0.0 if applied else None,
        "emergency_or_hold_unchanged": brake > 0.0 or bool(control.hand_brake),
        "includes_first_handoff_command": not record["measurement_is_driving"] and normal and eligible,
        "training_data_approved": False,
    }
    an_applied = an.an_suppress_stopped_brake_steering(
        control, speed_mps, enabled, speed_threshold_mps)
    return bool(applied or an_applied)


def ao_installer(planner, cap, throttle_cap=None):
    """HH_260906 - Leave AN's planner/PID and goal control installed, adding provenance only."""
    metadata = an.an_installer(planner, cap, throttle_cap)
    metadata.update(
        ao_arm=ARM, ao_route_length_m=ROUTE_LENGTH_M,
        ao_launch_progress_max_exclusive_m=MAX_LAUNCH_PROGRESS_M,
        ao_launch_speed_max_exclusive_mps=MAX_LAUNCH_SPEED_MPS,
        ao_stage="after_BasicAgent_hazard_decision_before_AN_goal_and_original_brake_guard",
        ao_initial_handoff_included=True, ao_emergency_and_hold_controls_unchanged=True,
        ao_policy_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        training_data_approved=False,
    )
    return metadata


@contextmanager
def configuration_scope():
    """HH_260906 - Nest the complete AN source chain and restore every AO-only hook."""
    with an.configuration_scope():
        if (base.configuration_from_args is not an.validate_args
                or base.GoalStopGovernor is not an.ANGovernor
                or base.install_normal_brake_cap is not an.an_installer
                or base.suppress_stopped_brake_steering is not an.an_suppress_stopped_brake_steering):
            raise ValueError("unexpected AN provider before AO installation")
        base.configuration_from_args = validate_args
        base.install_normal_brake_cap = ao_installer
        base.suppress_stopped_brake_steering = ao_suppress_stopped_brake_steering
        try:
            yield
        finally:
            base.suppress_stopped_brake_steering = an.an_suppress_stopped_brake_steering
            base.install_normal_brake_cap = an.an_installer
            base.configuration_from_args = an.validate_args


def main(argv=None):
    try:
        args = parse_args(argv)
        an.verify_basic_agent_source()
        with configuration_scope():
            output = base.run(args)
    except (base.CollectionError, OSError, ValueError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    print(f"capture={output}; AO development only, NOT admitted")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
