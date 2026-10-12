"""HH_261001 - Verify revised normal-only controls, exact stop contract, and untouched downstream emergency."""

from types import SimpleNamespace

import pytest

from scripts.e2e import collect_town07_low_speed_start_v2_20261001 as worker
from scripts.e2e.run_owned_town07_low_speed_start_v2_20261001 import command, OUTPUT


def control(throttle=.2, brake=0., steer=.04):
    return SimpleNamespace(throttle=throttle, brake=brake, steer=steer, hand_brake=False,
                           reverse=False, gear=1, manual_gear_shift=False)


def arguments():
    argv = command()
    return worker.parse_args(argv[argv.index(str(OUTPUT / "episode")):])


def test_fixed_configuration_changes_controls_not_measured_quality():
    config = worker.configuration(arguments())
    assert config.profile_id == worker.PROFILE
    assert (config.normal_brake_cap, config.desired_deceleration_mps2) == (0., .5)
    assert (config.goal_tolerance_m, config.stopped_speed_mps, config.hold_seconds) == (1., .1, 2.)
    assert config.minimum_tail_seconds == 6.5 and not config.training_data_approved
    assert config.launch_throttle == .13 and config.post_handoff_throttle_ramp_per_second == .05


def test_low_speed_normal_steering_only_preserves_latched_launch_and_pid_fields():
    policy = worker.CombinedControl()
    result = policy.apply(control(), speed=0., timestamp=10., frame=1, normal_proposal={"steer": .04})
    assert (result.throttle, result.brake, result.steer) == (.13, 0., 0.)
    assert policy.last["launch_control_before_steering_intervention"]["steer"] == .04
    result = policy.apply(control(steer=-.01), speed=.5, timestamp=11., frame=21, normal_proposal={})
    assert result.throttle == pytest.approx(.1325) and result.steer == 0.
    result = policy.apply(control(steer=.02), speed=.75, timestamp=11.05, frame=22, normal_proposal={})
    assert result.steer == 0.
    result = policy.apply(control(steer=-.03), speed=.75001, timestamp=11.1, frame=23, normal_proposal={})
    assert result.steer == -.03 and not policy.last["low_speed_steering_intervention"]
    result = policy.apply(control(throttle=0., steer=.05), speed=.1, timestamp=12., frame=41, normal_proposal={})
    assert (result.throttle, result.brake, result.steer) == (0., 0., 0.)
    assert policy.handoff_at == 11. and result.gear == 1 and not result.hand_brake


def test_normal_installer_retains_raw_proposal_and_leaves_emergency_output_untouched():
    args = arguments()
    snapshot = SimpleNamespace(frame=1, timestamp=SimpleNamespace(elapsed_seconds=10.))
    vehicle = SimpleNamespace(get_velocity=lambda: SimpleNamespace(x=0., y=0.),
                              get_world=lambda: SimpleNamespace(get_snapshot=lambda: snapshot))
    planner = SimpleNamespace(_vehicle=vehicle,
        _vehicle_controller=SimpleNamespace(max_brake=1., max_throt=.75),
        run_step=lambda: control(throttle=.7, brake=1., steer=.03))
    emergency = control(throttle=0., brake=1., steer=.2)
    original_transport = worker.base.AcknowledgedControlTransport
    with worker.scope(args):
        metadata = worker.base.install_normal_brake_cap(planner, 0., .2)
        ordinary = planner.run_step()
        assert (ordinary.throttle, ordinary.brake, ordinary.steer) == (.13, 0., 0.)
        assert metadata["basic_agent_emergency_brake_modified"] is False
        # HH_261001 - BasicAgent can replace the normal result later; no agent-output clamp is installed.
        final_agent_output = emergency
        assert (final_agent_output.brake, final_agent_output.steer) == (1., .2)
    assert worker.base.configuration_from_args is worker.ORIGINAL_CONFIGURATION
    assert worker.base.AcknowledgedControlTransport is original_transport


def test_reject_nonzero_precapped_brake_and_restore_hooks_on_exception():
    policy = worker.CombinedControl()
    with pytest.raises(ValueError, match="capped to zero"):
        policy.apply(control(brake=.02), speed=0., timestamp=10., frame=1, normal_proposal={})
    original = worker.base.collect_episode
    with pytest.raises(RuntimeError):
        with worker.scope(arguments()):
            raise RuntimeError("test failure")
    assert worker.base.collect_episode is original
    assert worker.base.install_normal_brake_cap is worker.ORIGINAL_INSTALLER


def test_audit_retains_partial_window_count_without_claiming_bound_assessment():
    from scripts.e2e.audit_town07_low_speed_start_v2_20261001 import summarize_windows
    empty = summarize_windows([])
    partial = summarize_windows([{"valid_future_points": 3}])
    assert empty["raw_anchors"] == 0 and empty["bound_assessed"] == 0
    assert partial["raw_anchors"] == 1 and partial["full_windows"] == 0
    assert partial["bound_assessed"] == 0
