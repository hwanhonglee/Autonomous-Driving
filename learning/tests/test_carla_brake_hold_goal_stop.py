"""HH_260906 - Verify the opt-in next-command service brake latch without weakening historical stopping checks."""

from dataclasses import asdict, replace
from types import SimpleNamespace

import pytest

from scripts.e2e import carla_goal_stop_profile as profiles


def args(**changes):
    values = dict(goal_stop_profile="comfortable_v5_brake_hold", control_transport="acknowledged_batch",
                  agent_initialization="after_bootstrap", target_speed_kmh=28.8, physics_hz=20., capture_hz=10.,
                  goal_tolerance_m=1., stationary_warmup_sec=3.5, stationary_tail_sec=6.5,
                  max_duration_sec=180., vehicle_type="vehicle.toyota.prius", weather="ClearNoon")
    return SimpleNamespace(**dict(values, **changes))


def governor():
    return profiles.DevelopmentGoalStopGovernor(profiles.BrakeHoldDevelopmentGoalStopConfig(), 8., 20.)


def observed(gov, timestamp, *, remaining=.8, distance=.8, speed=.02, overshoot=-.8):
    return gov.update(remaining, distance, speed, overshoot, timestamp=timestamp,
                      longitudinal_speed_mps=speed, driving=True)


def coast_entry(gov):
    # HH_260906 - Unit inputs model phase transitions, not a substitute for an actual bounded-route CARLA capture.
    observed(gov, 0., remaining=180., distance=180., speed=0., overshoot=-180.)
    observed(gov, .05, remaining=179., distance=179., speed=.5, overshoot=-179.)
    return observed(gov, .1, remaining=20., distance=20., speed=3., overshoot=-20.)


def control():
    return SimpleNamespace(throttle=.4, brake=.1, steer=.27, hand_brake=False,
                           reverse=False, manual_gear_shift=False)


def test_only_new_declared_control_revision_changes_and_denials_remain():
    previous = asdict(profiles.BrakeFreeDevelopmentGoalStopConfig())
    new = asdict(profiles.configuration_from_args(args()))
    assert {key: value for key, value in new.items() if value != previous.get(key)} == {
        "profile_id": "comfortable_v5_brake_hold", "normal_hold_brake": 1., "maximum_attempts_per_revision": 1}
    assert new["training_data_approved"] is False and new["development_only"] is True
    assert new["hold_seconds"] == 2. and new["minimum_tail_seconds"] == 6.5


@pytest.mark.parametrize("change", [dict(control_transport="legacy_async"),
    dict(agent_initialization="before_bootstrap"), dict(target_speed_kmh=30.),
    dict(stationary_warmup_sec=0.), dict(stationary_tail_sec=6.), dict(physics_hz=10.),
    dict(goal_tolerance_m=2.), dict(max_duration_sec=181.)])
def test_incompatible_settings_rejected(change):
    with pytest.raises(ValueError): profiles.configuration_from_args(args(**change))


@pytest.mark.parametrize("change", [dict(normal_hold_brake=.5), dict(normal_hold_brake=True),
    dict(hold_seconds=1.), dict(training_data_approved=True), dict(maximum_attempts_per_revision=2)])
def test_mutated_frozen_config_rejected(change):
    with pytest.raises(ValueError):
        profiles.DevelopmentGoalStopGovernor(replace(profiles.BrakeHoldDevelopmentGoalStopConfig(), **change), 8., 20.)


def test_next_control_latches_brake_after_observation_without_counting_first_sample_as_dwell():
    gov = governor()
    coast_entry(gov)
    old = gov.normal_control(control())
    assert old.throttle == old.brake == 0.
    entry = observed(gov, .15)
    assert entry["pilot_transition"] == "zero_pedal_coast_to_service_brake_hold"
    assert entry["hold_brake_source_timestamp"] == .15 and entry["hold_ticks"] == 0
    assert entry["complete"] is False and old.brake == 0.
    following = gov.normal_control(control())
    assert (following.throttle, following.brake, following.steer) == (0., 1., .27)
    for count in range(1, 40):
        row = observed(gov, .15 + .05 * count)
        assert row["hold_ticks"] == count and row["complete"] is False
        assert row["pilot_transition"] is None and row["hold_brake_latched"] is True
    row = observed(gov, 2.15)
    assert row["hold_ticks"] == 40 and row["hold_duration_sec"] == 2. and row["complete"] is True
    assert gov.normal_control(control()).brake == 1.


@pytest.mark.parametrize("change", [dict(remaining=1.01), dict(distance=1.01),
    dict(speed=.100001), dict(remaining=0.), dict(overshoot=.001)])
def test_brake_entry_requires_actual_speed_and_both_near_goal_upstream_conditions(change):
    gov = governor(); coast_entry(gov)
    row = observed(gov, .15, **change)
    assert row["hold_brake_latched"] is False and gov.normal_control(control()).brake == 0.


def test_latch_does_not_hide_subsequent_motion_or_shorten_dwell():
    gov = governor(); coast_entry(gov); observed(gov, .15)
    assert observed(gov, .2)["hold_ticks"] == 1
    moved = observed(gov, .25, speed=.101)
    assert moved["hold_ticks"] == 0 and moved["complete"] is False
    assert moved["hold_brake_latched"] is True and gov.normal_control(control()).brake == 1.
    assert observed(gov, .3)["hold_ticks"] == 0
    assert observed(gov, .35)["hold_ticks"] == 1


def test_emergency_failure_cannot_arm_new_normal_hold():
    gov = governor(); coast_entry(gov); gov.fail("retained_emergency")
    row = observed(gov, .15)
    assert row["pilot_failure_reason"] == "retained_emergency" and row["hold_brake_latched"] is False


def test_previous_brake_free_profile_keeps_its_original_zero_pedal_dwell():
    gov = profiles.DevelopmentGoalStopGovernor(profiles.BrakeFreeDevelopmentGoalStopConfig(), 8., 20.)
    coast_entry(gov); row = observed(gov, .15)
    assert "hold_brake_latched" not in row and gov.normal_control(control()).brake == 0.


def test_warmup_observation_cannot_latch_normal_service_control():
    gov = governor()
    row = gov.update(.8, .8, .02, -.8, timestamp=0., longitudinal_speed_mps=.02, driving=False)
    assert row["hold_brake_latched"] is False

