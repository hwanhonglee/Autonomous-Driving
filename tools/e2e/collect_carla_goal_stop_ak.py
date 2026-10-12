"""HH_260906 - Fixed Town01 coast-distance/low-speed hold A/B, development evidence only."""
from __future__ import annotations

import argparse
from contextlib import contextmanager
from dataclasses import asdict, dataclass
import hashlib
import math
from pathlib import Path
import sys

from . import collect_carla_pedal_caps_aj as aj
from . import collect_carla_vad_expert as base
from .carla_goal_stop_profile import GoalStopGovernor


# HH_260906 - The AJ terminal slope predicts a <=0.1 m/s entry before the endpoint with this earlier coast.
COAST_BUFFER_M = 2.7
ARMS = ("coast_27", "coast_27_hold")
ORIGINAL_CONFIGURATION = base.configuration_from_args
ORIGINAL_GOVERNOR = base.GoalStopGovernor
ORIGINAL_INSTALLER = base.install_normal_brake_cap
_active_governor: AKGovernor | None = None


@dataclass(frozen=True)
class AKConfig(aj.PedalCapsConfig):
    # HH_260906 - Keep the experiment denied until separate source-bound admission and full future QA.
    ak_arm: str = "coast_27"
    hold_brake: float = 1.0


def validate_args(args):
    """HH_260906 - Reuse AJ's exact route/rig/clock lock and change only the named terminal intervention."""
    if args.ak_arm not in ARMS:
        raise ValueError("unknown AK arm")
    args.pedal_arm = "staged_coast_buffer"
    previous = aj.validate_args(args)
    values = asdict(previous)
    values.update(profile_id="comfortable_v2_ak_" + args.ak_arm,
                  stop_buffer_m=COAST_BUFFER_M,
                  ak_arm=args.ak_arm,
                  policy_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    return AKConfig(**values)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(add_help=False, allow_abbrev=False)
    parser.add_argument("--ak-arm", choices=ARMS, required=True)
    own, rest = parser.parse_known_args(argv)
    args = base.parse_args(rest)
    args.ak_arm = own.ak_arm
    validate_args(args)
    return args


class AKGovernor(GoalStopGovernor):
    """HH_260906 - Latch hold only after an actual in-window, upstream <=0.1 m/s observation."""

    def __init__(self, config, cruise_speed_mps, physics_hz):
        global _active_governor
        if type(config) is not AKConfig or config.ak_arm not in ARMS or _active_governor is not None:
            raise ValueError("AK requires one exact scoped governor")
        super().__init__(config, cruise_speed_mps, physics_hz)
        self.hold_latched = False
        self.hold_source_tick = None
        self.observation_tick = 0
        _active_governor = self

    def update(self, remaining_arc_m, planar_distance_m, speed_mps, terminal_overshoot,
               *, count_hold=True):
        record = super().update(remaining_arc_m, planar_distance_m, speed_mps,
                                terminal_overshoot, count_hold=count_hold)
        self.observation_tick += 1
        if (self.config.ak_arm == "coast_27_hold" and count_hold and not self.hold_latched
                and record["goal_window"] and record["upstream_of_endpoint"]
                and terminal_overshoot <= 0 and speed_mps <= self.config.stopped_speed_mps):
            self.hold_latched = True
            self.hold_source_tick = self.observation_tick
        record.update(ak_arm=self.config.ak_arm, hold_brake_latched=self.hold_latched,
                      hold_brake_source_observation_tick=self.hold_source_tick,
                      hold_brake_trigger="measured <=0.1 m/s, inside unchanged 1 m goal window, upstream"
                      if self.hold_latched else None,
                      development_only=True, training_data_approved=False)
        return record


def ak_installer(planner, cap, throttle_cap=None):
    """HH_260906 - Change only normal controls after measured latch; BasicAgent emergency remains downstream."""
    if _active_governor is None:
        raise ValueError("AK normal control requires its measured governor")
    metadata = aj.staged_installer(planner, cap, throttle_cap)
    normal = planner.run_step

    def step(*args, **kwargs):
        control = normal(*args, **kwargs)
        if _active_governor.hold_latched:
            control.throttle = 0.0
            control.brake = _active_governor.config.hold_brake
        return control

    planner.run_step = step
    metadata.update(ak_arm=_active_governor.config.ak_arm,
                    hold_brake_source="following normal command after measured <=0.1 m/s upstream goal-window entry",
                    hold_brake_after_emergency=False,
                    policy_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    return metadata


@contextmanager
def configuration_scope():
    """HH_260906 - Restore every process-local worker hook and latch even after a failed capture."""
    global _active_governor
    if (base.configuration_from_args is not ORIGINAL_CONFIGURATION
            or base.GoalStopGovernor is not ORIGINAL_GOVERNOR
            or base.install_normal_brake_cap is not ORIGINAL_INSTALLER):
        raise ValueError("unexpected collector provider")
    _active_governor = None
    base.configuration_from_args = validate_args
    base.GoalStopGovernor = AKGovernor
    base.install_normal_brake_cap = ak_installer
    try:
        yield
    finally:
        base.configuration_from_args = ORIGINAL_CONFIGURATION
        base.GoalStopGovernor = ORIGINAL_GOVERNOR
        base.install_normal_brake_cap = ORIGINAL_INSTALLER
        _active_governor = None


def main(argv=None):
    try:
        args = parse_args(argv)
        with configuration_scope():
            output = base.run(args)
    except (base.CollectionError, OSError, ValueError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    print(f"capture={output}; AK development only, NOT admitted")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
