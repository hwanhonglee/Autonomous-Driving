"""HH_261001 - Verify measured launch/ramp logic, preserved braking/steering and scoped expert hooks."""

from types import SimpleNamespace

import pytest

from scripts.e2e.collect_town07_low_speed_start_20261001 import LaunchControl, scope, base, ORIGINAL_CONFIGURATION
from scripts.e2e.run_owned_town07_low_speed_start_20261001 import command, OUTPUT
from scripts.e2e.collect_town07_low_speed_start_20261001 import parse_args


def control(throttle=.2, brake=0., steer=.04):
    return SimpleNamespace(throttle=throttle, brake=brake, steer=steer, hand_brake=False,
                           reverse=False, gear=1, manual_gear_shift=False)


def test_launch_is_measured_then_latched_ramp_preserves_normal_brake_and_steering():
    policy = LaunchControl()
    first = policy.apply(control(), speed=0., timestamp=10., frame=1, normal_proposal={})
    assert first.throttle == .13 and first.brake == 0. and first.steer == .04
    next_control = policy.apply(control(brake=.1), speed=.5, timestamp=11., frame=21, normal_proposal={})
    assert next_control.throttle == pytest.approx(.1325)
    assert next_control.brake == .1 and next_control.steer == .04
    reduced = policy.apply(control(throttle=0., brake=.15), speed=.01, timestamp=12., frame=41, normal_proposal={})
    assert reduced.throttle == 0. and reduced.brake == .15
    assert policy.handoff_at == 11.


def test_timeout_and_backwards_clock_fail_without_invented_handoff():
    policy = LaunchControl()
    policy.apply(control(), speed=0., timestamp=10., frame=1, normal_proposal={})
    with pytest.raises(ValueError, match="8 seconds"):
        policy.apply(control(), speed=.49, timestamp=18., frame=161, normal_proposal={})
    with pytest.raises(ValueError, match="backwards"):
        policy.apply(control(), speed=.51, timestamp=17., frame=141, normal_proposal={})


def test_fixed_command_parses_and_process_hooks_restore_after_exception():
    argv = command()
    args = parse_args(argv[argv.index(str(OUTPUT / "episode")):])
    original_transport = base.AcknowledgedControlTransport
    with pytest.raises(RuntimeError):
        with scope(args):
            assert base.configuration_from_args is not ORIGINAL_CONFIGURATION
            raise RuntimeError("test failure")
    assert base.configuration_from_args is ORIGINAL_CONFIGURATION
    assert base.AcknowledgedControlTransport is original_transport
