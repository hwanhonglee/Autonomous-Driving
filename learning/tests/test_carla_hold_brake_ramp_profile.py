"""HH_260906 - Test the isolated v7 normal hold ramp without CARLA, inference or admission changes."""

from dataclasses import asdict, replace
import json
import math
import struct
from types import SimpleNamespace

import pytest

from scripts.e2e import carla_goal_stop_profile as p


PROFILE = "comfortable_v7_hold_brake_ramp"


def arguments(**changes):
    values = dict(goal_stop_profile=PROFILE, low_speed_steering_mode="zero", hold_brake_mode="ramp",
                  control_transport="acknowledged_batch", agent_initialization="after_bootstrap",
                  target_speed_kmh=28.8, physics_hz=20., capture_hz=10., goal_tolerance_m=1.,
                  stationary_warmup_sec=3.5, stationary_tail_sec=6.5, max_duration_sec=180.,
                  vehicle_type="vehicle.toyota.prius", weather="ClearNoon")
    return SimpleNamespace(**dict(values, **changes))


def governor(mode="ramp"):
    return p.DevelopmentGoalStopGovernor(p.configuration_from_args(arguments(hold_brake_mode=mode)), 8., 20.)


def control():
    return SimpleNamespace(throttle=.4, brake=.1, steer=.27, hand_brake=False, reverse=False,
                           gear=0, manual_gear_shift=False)


def observe(gov, stamp, speed=.09, remaining=.8, driving=True):
    return gov.update(remaining, remaining, speed, -remaining, timestamp=stamp,
                      longitudinal_speed_mps=speed, driving=driving, count_hold=driving)


def enter_hold(gov):
    observe(gov, 0., 0., 180.)
    observe(gov, .05, .5, 179.)
    observe(gov, .1, 3., 20.)
    return observe(gov, .15)


def request(gov, record, *, driving=True, proposed=None):
    result = gov.normal_control(control() if proposed is None else proposed)
    p.annotate_development_control(record, gov, None)
    assert p.apply_low_speed_steering(result, gov, record, driving_command=driving) is result
    return result


@pytest.mark.parametrize("mode", ["immediate", "ramp"])
def test_only_declared_hold_mode_revision_changes_v6_zero_configuration(mode):
    old = asdict(replace(p.LowSpeedSteeringDevelopmentGoalStopConfig(), low_speed_steering_mode="zero"))
    new = asdict(p.configuration_from_args(arguments(hold_brake_mode=mode)))
    assert {k:v for k,v in new.items() if old.get(k) != v} == {
        "profile_id": PROFILE, "hold_brake_mode": mode, "hold_brake_ramp_seconds": .5}
    assert new["maximum_attempts_per_revision"] == 4
    assert new["hold_seconds"] == 2. and new["minimum_tail_seconds"] == 6.5
    assert new["low_speed_steering_threshold_mps"] == .75 and new["normal_hold_brake"] == 1.
    assert new["research_only"] is True and new["training_data_approved"] is False


@pytest.mark.parametrize("mode", [None, "", "RAMP", "other", False, 0])
def test_hold_mode_must_be_explicit(mode):
    with pytest.raises(ValueError, match="explicit hold_brake_mode"):
        p.configuration_from_args(arguments(hold_brake_mode=mode))


@pytest.mark.parametrize("mode", [None, "native", "ZERO", False, 0])
def test_v7_requires_explicit_zero_steering(mode):
    with pytest.raises(ValueError, match="low_speed_steering_mode=zero"):
        p.configuration_from_args(arguments(low_speed_steering_mode=mode))


@pytest.mark.parametrize("profile", ["disabled", "comfortable_v1", "comfortable_v2", "comfortable_v3",
    "comfortable_v4", "comfortable_v5_brake_hold", "comfortable_v6_low_speed_steering", "turn_low_v1", *p.TURN_LAUNCH_PEDALS])
def test_hold_option_is_forbidden_for_all_older_profiles(profile):
    args = SimpleNamespace(goal_stop_profile=profile, hold_brake_mode="ramp",
                           low_speed_steering_mode="zero" if profile == "comfortable_v6_low_speed_steering" else None)
    with pytest.raises(ValueError, match="hold_brake_mode is valid only"):
        p.configuration_from_args(args)


@pytest.mark.parametrize("changes", [dict(control_transport="legacy_async"), dict(agent_initialization="before_bootstrap"),
    dict(target_speed_kmh=30.), dict(physics_hz=10.), dict(capture_hz=5.), dict(goal_tolerance_m=2.),
    dict(stationary_warmup_sec=0.), dict(stationary_tail_sec=6.), dict(max_duration_sec=181.),
    dict(vehicle_type="vehicle.other")])
def test_original_capture_preflight_bounds_unchanged(changes):
    with pytest.raises(ValueError): p.configuration_from_args(arguments(**changes))


@pytest.mark.parametrize("changes", [dict(hold_brake_mode="other"), dict(hold_brake_ramp_seconds=.6),
    dict(low_speed_steering_mode="native"), dict(low_speed_steering_threshold_mps=.8),
    dict(normal_hold_brake=.5), dict(hold_seconds=1.), dict(stopped_speed_mps=.2),
    dict(training_data_approved=True), dict(maximum_attempts_per_revision=5),
    dict(maximum_attempts_per_revision=4.), dict(research_only=False), dict(normal_hold_brake=True)])
def test_exact_frozen_config_cannot_smuggle_other_factors(changes):
    with pytest.raises(ValueError):
        p.DevelopmentGoalStopGovernor(replace(p.HoldBrakeRampDevelopmentGoalStopConfig(), **changes), 8., 20.)


@pytest.mark.parametrize("mode", ["immediate", "ramp"])
def test_requested_curve_uses_actual_elapsed_plus_dt_and_preserves_measured_dwell(mode):
    gov = governor(mode)
    row = enter_hold(gov)
    assert row["hold_ticks"] == 0 and row["hold_brake_source_timestamp"] == .15
    values = []
    for index in range(41):
        if index:
            row = observe(gov, .15 + index*.05000000074505806)
        result = request(gov, row)
        receipt = row["hold_brake_control"]
        elapsed = row["measured_timestamp"] - .15
        expected = 1. if mode == "immediate" else min(1., (elapsed+.05)/.5)
        values.append(result.brake)
        assert result.throttle == result.steer == 0. and result.brake == expected
        assert receipt["mathematical_hold_brake"] == expected
        assert receipt["hold_elapsed_seconds"] == elapsed
        assert receipt["next_interval_end_elapsed_seconds"] == elapsed+.05
        assert receipt["hold_source_timestamp"] == .15 and receipt["source_timestamp"] == row["measured_timestamp"]
        assert receipt["returned_control"]["steer"] == .27
        assert receipt["returned_control"] == row["low_speed_steering"]["input_control"]
        assert receipt["control_stage"] == "LocalPlanner_normal_control_before_BasicAgent_emergency_and_output_steering"
        assert receipt["normal_control_executed"] is True and receipt["superseded_by_emergency"] is False
        assert receipt["physical_application_proven"] is False and receipt["training_data_approved"] is False
        assert row["hold_ticks"] == index and row["complete"] is (index >= 40)
        json.dumps(row, allow_nan=False)
    assert values[0] == (.1 if mode == "ramp" else 1.) and values[9] == 1.
    assert row["hold_duration_sec"] == 2.


def test_elapsed_is_not_rounded_to_a_tick_and_moving_resets_dwell_not_ramp_origin():
    gov = governor()
    row = enter_hold(gov); request(gov, row)
    row = observe(gov, .2000003, speed=.10001)
    result = request(gov, row)
    expected = (.2000003-.15+.05)/.5
    assert result.brake == expected and expected != .2
    assert row["hold_ticks"] == 0 and row["complete"] is False
    assert row["hold_brake_source_timestamp"] == .15
    row = observe(gov, .2500003)
    assert row["hold_ticks"] == 0
    assert request(gov, row).brake == (.2500003-.15+.05)/.5


def test_receipt_preserves_math_and_actual_float32_output_separately():
    class Float32Control(SimpleNamespace):
        def __setattr__(self, name, value):
            if name in ("throttle", "brake", "steer"):
                value = struct.unpack("f", struct.pack("f", value))[0]
            super().__setattr__(name, value)
    gov = governor()
    row = enter_hold(gov)
    result = request(gov, row, proposed=Float32Control(**vars(control())))
    receipt = row["hold_brake_control"]
    assert receipt["mathematical_hold_brake"] == .1
    assert receipt["returned_control"]["brake"] == result.brake == 0.10000000149011612
    assert receipt["returned_control"]["brake"] != receipt["mathematical_hold_brake"]


@pytest.mark.parametrize("driving", [True, False])
def test_each_update_clears_old_normal_receipt_for_early_emergency_setup_or_tail(driving):
    gov = governor()
    row = enter_hold(gov); request(gov, row)
    assert gov.last_hold_brake_control is not None
    row = observe(gov, .2, driving=driving)
    assert gov.last_hold_brake_control is None
    gov.emergency_override_count = 1
    gov.fail(PROFILE + "_emergency_override")
    proposed = control(); proposed.brake = .5
    before = vars(proposed).copy()
    p.annotate_development_control(row, gov, None)
    p.apply_low_speed_steering(proposed, gov, row, driving_command=driving)
    assert vars(proposed) == before
    receipt = row["hold_brake_control"]
    assert receipt["normal_control_executed"] is False and receipt["superseded_by_emergency"] is True
    assert receipt["input_control"] is receipt["returned_control"] is receipt["mathematical_hold_brake"] is None


def test_normal_ramp_output_is_retained_but_original_emergency_return_wins():
    gov = governor()
    row = enter_hold(gov)
    planner = SimpleNamespace(_vehicle_controller=SimpleNamespace(max_brake=.3, max_throt=.75), run_step=control)
    emergency_result = []
    def emergency(proposed):
        assert proposed.brake == .1
        proposed.throttle, proposed.brake, proposed.steer = 0., .5, -.33
        emergency_result.append(proposed)
        return proposed
    agent = SimpleNamespace(get_local_planner=lambda: planner, add_emergency_stop=emergency)
    metadata = p.install_development_control(agent, gov)
    result = agent.add_emergency_stop(planner.run_step())
    p.annotate_development_control(row, gov, None)
    p.apply_low_speed_steering(result, gov, row, driving_command=True)
    assert result is emergency_result[0] and (result.brake, result.steer) == (.5, -.33)
    assert row["hold_brake_control"]["returned_control"]["brake"] == .1
    assert row["hold_brake_control"]["superseded_by_emergency"] is True
    assert row["low_speed_steering"]["input_control"]["brake"] == .5
    assert metadata["emergency_return_control_modified"] is False


def test_prehold_controls_and_v7_immediate_match_v6_zero_without_legacy_new_fields():
    old = p.DevelopmentGoalStopGovernor(replace(p.LowSpeedSteeringDevelopmentGoalStopConfig(), low_speed_steering_mode="zero"), 8., 20.)
    new = governor("immediate")
    rows = [(0., 0., 180.), (.05, .5, 179.), (.1, 3., 20.)]
    rows += [(.15+index*.05, .09, .8) for index in range(41)]
    for stamp, speed, remaining in rows:
        a, b = observe(old, stamp, speed, remaining), observe(new, stamp, speed, remaining)
        ca, cb = request(old, a), request(new, b)
        assert vars(ca) == vars(cb)
        assert "hold_brake_control" not in a and not hasattr(old, "last_hold_brake_control")
        normalized = {k:v for k,v in b.items() if k != "hold_brake_control"}
        normalized["profile_id"] = a["profile_id"]
        assert json.dumps(normalized, sort_keys=True) == json.dumps(a, sort_keys=True)
        if stamp < .15:
            assert b["hold_brake_control"]["normal_control_executed"] is True
            assert b["hold_brake_control"]["mathematical_hold_brake"] is None

