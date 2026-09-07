"""HH_260906 - Test fixed launch-pedal comparisons without changing historical traces or admission bounds."""

from dataclasses import asdict, replace
import math
from types import SimpleNamespace

import pytest

from scripts.e2e import carla_goal_stop_profile as profiles
from scripts.e2e import collect_carla_vad_expert as collector


def arguments(profile="turn_launch_015_v1", **changes):
    result = dict(goal_stop_profile=profile, agent_initialization="after_bootstrap",
        control_transport="acknowledged_batch", target_speed_kmh=14.4, stationary_warmup_sec=3.5,
        stationary_tail_sec=6.5, physics_hz=20., capture_hz=10., goal_tolerance_m=1.,
        vehicle_type="vehicle.toyota.prius", max_duration_sec=180., weather="ClearNoon")
    result.update(changes)
    return SimpleNamespace(**result)


def update(governor, timestamp, speed, remaining=150., driving=True):
    return governor.update(remaining, remaining, speed, -remaining, timestamp=timestamp,
                          longitudinal_speed_mps=speed, driving=driving)


@pytest.mark.parametrize("profile,pedal", list(profiles.TURN_LAUNCH_PEDALS.items()))
def test_named_cases_change_only_declared_launch_and_new_common_ramp_field(profile, pedal):
    config = profiles.configuration_from_args(arguments(profile))
    old, new = asdict(profiles.TurnLowDevelopmentGoalStopConfig()), asdict(config)
    assert set(new) - set(old) == {"post_handoff_initial_throttle"}
    assert {key for key in old if old[key] != new[key]} <= {"profile_id", "launch_throttle"}
    assert new["launch_throttle"] == pedal and new["post_handoff_initial_throttle"] == .15
    assert new["maximum_attempts_per_revision"] == 2 and new["maximum_launch_seconds"] == 8.
    assert new["training_data_approved"] is False and new["development_only"] is True
    assert profiles.is_low_turn_config(config)


@pytest.mark.parametrize("profile", list(profiles.TURN_LAUNCH_PEDALS))
@pytest.mark.parametrize("change", [dict(agent_initialization="before_bootstrap"),
    dict(control_transport="legacy_async"), dict(target_speed_kmh=28.8), dict(physics_hz=10),
    dict(capture_hz=5), dict(stationary_warmup_sec=0), dict(stationary_tail_sec=0)])
def test_unreviewed_setup_rejected_before_simulator(profile, change):
    with pytest.raises(ValueError):
        profiles.configuration_from_args(arguments(profile, **change))


@pytest.mark.parametrize("change", [dict(launch_throttle=.11), dict(post_handoff_initial_throttle=.14),
    dict(post_handoff_throttle_ramp_per_second=.06), dict(maximum_launch_seconds=9.),
    dict(normal_brake_cap=.1), dict(normal_throttle_cap=.5), dict(maximum_actual_speed_mps=4.4),
    dict(minimum_cruise_seconds=0.), dict(goal_tolerance_m=True), dict(training_data_approved=0)])
def test_governor_rejects_substituted_fields_or_types(change):
    config = replace(profiles.turn_launch_configuration("turn_launch_013_v1"), **change)
    with pytest.raises(ValueError):
        profiles.DevelopmentGoalStopGovernor(config, 4., 20.)


def test_unknown_revision_and_arbitrary_subclass_rejected():
    with pytest.raises(ValueError):
        profiles.turn_launch_configuration("turn_launch_011_v1")
    class Foreign(profiles.TurnLaunchDevelopmentGoalStopConfig):
        pass
    with pytest.raises(ValueError):
        profiles.DevelopmentGoalStopGovernor(Foreign(), 4., 20.)


@pytest.mark.parametrize("profile,pedal", list(profiles.TURN_LAUNCH_PEDALS.items()))
def test_launch_pedal_and_post_handoff_ramp_are_separate(profile, pedal):
    governor = profiles.DevelopmentGoalStopGovernor(profiles.turn_launch_configuration(profile), 4., 20.)
    update(governor, 0., .1)
    command = SimpleNamespace(throttle=.7, brake=.3, steer=.37)
    assert governor.normal_control(command) is command
    assert (command.throttle, command.brake, command.steer) == (pedal, 0., .37)
    row = update(governor, .05, .5)
    assert row["normal_throttle_cap"] == pytest.approx(.1525)
    governor.normal_control(command := SimpleNamespace(throttle=.7, brake=.3, steer=-.21))
    assert (command.throttle, command.brake, command.steer) == pytest.approx((.1525, 0., -.21))
    row = update(governor, .10, .6)
    assert row["normal_throttle_cap"] == pytest.approx(.155)


def test_point_fifteen_case_preserves_original_low_profile_numerical_trace():
    # HH_260906 - Fixed identical measured inputs isolate implementation parity, not actual simulator repeatability.
    old = profiles.DevelopmentGoalStopGovernor(profiles.TurnLowDevelopmentGoalStopConfig(), 4., 20.)
    new = profiles.DevelopmentGoalStopGovernor(profiles.turn_launch_configuration("turn_launch_015_v1"), 4., 20.)
    for index in range(200):
        speed = .2 if index < 10 else 4.
        a, b = update(old, index * .05, speed), update(new, index * .05, speed)
        a.pop("profile_id"); b.pop("profile_id")
        assert a == b
        command_a = SimpleNamespace(throttle=.35, brake=.1, steer=.3)
        command_b = SimpleNamespace(throttle=.35, brake=.1, steer=.3)
        assert vars(old.normal_control(command_a)) == vars(new.normal_control(command_b))


@pytest.mark.parametrize("profile", list(profiles.TURN_LAUNCH_PEDALS))
def test_handoff_timeout_speed_and_emergency_authority_remain_strict(profile):
    governor = profiles.DevelopmentGoalStopGovernor(profiles.turn_launch_configuration(profile), 4., 20.)
    update(governor, 0., .1)
    for index in range(1, 160):
        assert update(governor, index * .05, .49)["pilot_failure_reason"] is None
    assert update(governor, 8., .49)["pilot_failure_reason"] == profile + "_launch_timeout"
    governor = profiles.DevelopmentGoalStopGovernor(profiles.turn_launch_configuration(profile), 4., 20.)
    assert update(governor, 0., math.nextafter(4.3, math.inf))["pilot_failure_reason"] == profile + "_actual_speed_exceeded"
    governor = profiles.DevelopmentGoalStopGovernor(profiles.turn_launch_configuration(profile), 4., 20.)
    update(governor, 0., .1)
    planner = SimpleNamespace(_vehicle_controller=SimpleNamespace(max_brake=.3, max_throt=.75))
    planner.run_step = lambda: SimpleNamespace(throttle=.7, brake=.3, steer=.37)
    def emergency(command):
        command.brake = .5
        return command
    agent = SimpleNamespace(get_local_planner=lambda: planner, add_emergency_stop=emergency)
    metadata = profiles.install_development_control(agent, governor)
    command = planner.run_step()
    assert agent.add_emergency_stop(command) is command and command.brake == .5 and command.steer == .37
    assert governor.failure_reason == profile + "_emergency_override"
    assert metadata["emergency_return_control_modified"] is False


@pytest.mark.parametrize("profile", list(profiles.TURN_LAUNCH_PEDALS))
def test_cli_requires_both_ack_and_explicit_initialization(profile):
    argv = ["out", "route", "--goal-stop-profile", profile, "--target-speed-kmh", "14.4",
        "--stationary-warmup-sec", "3.5", "--stationary-tail-sec", "6.5", "--goal-tolerance-m", "1.0",
        "--control-transport", "acknowledged_batch"]
    with pytest.raises(SystemExit):
        collector.parse_args(argv)
    assert collector.parse_args(argv + ["--agent-initialization", "after_bootstrap"]).goal_stop_profile == profile


def test_historical_default_and_low_configuration_are_unchanged():
    assert collector.parse_args(["out", "route"]).goal_stop_profile == "disabled"
    assert collector.parse_args(["out", "route"]).agent_initialization == "before_bootstrap"
    assert profiles.configuration_from_args(arguments("turn_low_v1", agent_initialization="before_bootstrap")) == profiles.TurnLowDevelopmentGoalStopConfig()
