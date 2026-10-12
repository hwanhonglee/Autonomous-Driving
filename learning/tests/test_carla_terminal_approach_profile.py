"""HH_260906 - Exercise frozen terminal feedback with synthetic observations, never CARLA or admission edits."""

from dataclasses import asdict, replace
import json
import struct
from types import SimpleNamespace

import pytest

from scripts.e2e import carla_goal_stop_profile as p


PROFILE = "comfortable_v9_terminal_feedback"


def arguments(**changes):
    values = dict(goal_stop_profile=PROFILE, terminal_approach_mode="feedback",
                  low_speed_steering_mode="zero", hold_brake_mode="immediate",
                  physics_substep_profile="reference_10ms", control_transport="acknowledged_batch",
                  agent_initialization="after_bootstrap", target_speed_kmh=28.8, physics_hz=20.,
                  capture_hz=10., goal_tolerance_m=1., stationary_warmup_sec=3.5,
                  stationary_tail_sec=6.5, max_duration_sec=180., vehicle_type="vehicle.toyota.prius")
    return SimpleNamespace(**dict(values, **changes))


def governor(mode="feedback"):
    return p.DevelopmentGoalStopGovernor(p.configuration_from_args(arguments(terminal_approach_mode=mode)), 8., 20.)


def control():
    return SimpleNamespace(throttle=.4, brake=.1, steer=.27, hand_brake=False,
                           reverse=False, gear=0, manual_gear_shift=False)


def observe(gov, stamp, speed=.2, remaining=2., driving=True, **changes):
    values = dict(remaining_arc_m=remaining, planar_distance_m=remaining, speed_mps=speed,
                  terminal_overshoot=-remaining, timestamp=stamp, longitudinal_speed_mps=speed,
                  driving=driving, count_hold=driving)
    values.update(changes)
    return gov.update(**values)


def prime(gov, speed=.2, remaining=2.):
    observe(gov, 0., 0., 180.)
    observe(gov, .05, .5, 179.)
    observe(gov, .1, 3., 20.)
    return observe(gov, .15, speed, remaining)


def request(gov, row, proposed=None, driving=True):
    result = gov.normal_control(control() if proposed is None else proposed)
    p.annotate_development_control(row, gov, None)
    assert p.apply_low_speed_steering(result, gov, row, driving_command=driving) is result
    return result


@pytest.mark.parametrize("mode", ["coast", "feedback"])
def test_frozen_configuration_changes_only_declared_research_fields(mode):
    new = asdict(p.configuration_from_args(arguments(terminal_approach_mode=mode)))
    old = asdict(p.HoldBrakeRampDevelopmentGoalStopConfig())
    inherited_changes = {k: new[k] for k in old if new[k] != old[k]}
    assert inherited_changes == {"profile_id": PROFILE, "maximum_attempts_per_revision": 2}
    assert new["terminal_approach_mode"] == mode
    assert new["terminal_feedforward_taper_mps"] == .1
    assert (new["hold_seconds"], new["minimum_tail_seconds"], new["maximum_coast_seconds"]) == (2., 6.5, 45.)
    assert new["normal_brake_cap"] == 0. and new["normal_hold_brake"] == 1.
    assert new["training_data_approved"] is False and new["research_only"] is True


@pytest.mark.parametrize("mode", [None, "", "FEEDBACK", "other", False, 0])
def test_terminal_mode_is_explicit(mode):
    with pytest.raises(ValueError, match="explicit terminal_approach_mode"):
        p.configuration_from_args(arguments(terminal_approach_mode=mode))


@pytest.mark.parametrize("changes", [dict(hold_brake_mode=None), dict(hold_brake_mode="ramp"),
    dict(low_speed_steering_mode=None), dict(low_speed_steering_mode="native"),
    dict(physics_substep_profile=None), dict(physics_substep_profile="inherited"),
    dict(physics_substep_profile="fine_5ms"), dict(control_transport="legacy_async"),
    dict(agent_initialization="before_bootstrap"), dict(target_speed_kmh=30.),
    dict(physics_hz=40.), dict(capture_hz=20.), dict(goal_tolerance_m=2.),
    dict(stationary_warmup_sec=0.), dict(stationary_tail_sec=6.), dict(max_duration_sec=181.)])
def test_only_fixed_other_factors_and_old_capture_bounds_allowed(changes):
    with pytest.raises(ValueError):
        p.configuration_from_args(arguments(**changes))


@pytest.mark.parametrize("profile", ["disabled", "comfortable_v1", "comfortable_v2", "comfortable_v3",
    "comfortable_v4", "comfortable_v5_brake_hold", "comfortable_v6_low_speed_steering",
    "comfortable_v7_hold_brake_ramp", "turn_low_v1", *p.TURN_LAUNCH_PEDALS])
def test_terminal_option_rejected_for_every_old_profile(profile):
    args = arguments(goal_stop_profile=profile,
                     low_speed_steering_mode="zero" if profile in ("comfortable_v6_low_speed_steering", "comfortable_v7_hold_brake_ramp") else None,
                     hold_brake_mode="immediate" if profile == "comfortable_v7_hold_brake_ramp" else None)
    with pytest.raises(ValueError, match="terminal_approach_mode is valid only"):
        p.configuration_from_args(args)


@pytest.mark.parametrize("changes", [dict(terminal_approach_mode="other"), dict(hold_brake_mode="ramp"),
    dict(low_speed_steering_mode="native"), dict(terminal_activation_speed_mps=.8),
    dict(terminal_maximum_target_speed_mps=.4), dict(terminal_feedforward_taper_mps=.2),
    dict(terminal_maximum_throttle=.16), dict(terminal_throttle_increase_per_second=.2),
    dict(hold_seconds=1.), dict(maximum_coast_seconds=46.), dict(stopped_speed_mps=.2),
    dict(maximum_attempts_per_revision=2.), dict(normal_hold_brake=True), dict(training_data_approved=True)])
def test_exact_types_and_frozen_values_cannot_smuggle_another_factor(changes):
    with pytest.raises(ValueError):
        p.DevelopmentGoalStopGovernor(replace(p.TerminalApproachDevelopmentGoalStopConfig(), **changes), 8., 20.)


@pytest.mark.parametrize("field,value", [("speed_mps", True), ("remaining_arc_m", float("nan")),
    ("timestamp", float("inf")), ("longitudinal_speed_mps", False), ("terminal_overshoot", float("nan")),
    ("driving", 1), ("count_hold", 0)])
def test_v9_source_observation_requires_finite_numbers_and_strict_flags(field, value):
    with pytest.raises(ValueError):
        observe(governor(), 0., **{field: value})


def test_coast_arm_controls_and_all_historical_fields_match_v7_immediate():
    old = p.DevelopmentGoalStopGovernor(p.HoldBrakeRampDevelopmentGoalStopConfig(), 8., 20.)
    new = governor("coast")
    samples = [(0., 0., 180.), (.05, .5, 179.), (.1, 3., 20.), (.15, .2, 2.)]
    samples += [(.2+i*.05, .09, .8) for i in range(41)]
    for stamp, speed, remaining in samples:
        a, b = observe(old, stamp, speed, remaining), observe(new, stamp, speed, remaining)
        ca, cb = request(old, a), request(new, b)
        assert vars(ca) == vars(cb)
        normalized = {k: v for k, v in b.items() if k not in ("terminal_approach", "terminal_approach_control")}
        normalized["profile_id"] = a["profile_id"]
        assert normalized == a
        assert b["terminal_approach"]["feedback_latched"] is False


def test_feedback_latches_once_on_observed_threshold_and_preserves_coast_clock():
    gov = governor()
    row = prime(gov, speed=.75001)
    assert row["terminal_approach"]["feedback_latched"] is False
    assert request(gov, row).throttle == 0.
    row = observe(gov, .2, speed=.75)
    assert row["pilot_transition"] == "zero_pedal_coast_to_terminal_feedback"
    assert row["phase"] == "terminal_feedback"
    assert row["terminal_approach"]["feedback_source_timestamp"] == .2
    assert request(gov, row).throttle == 0.
    row = observe(gov, .25, speed=.8)
    assert row["pilot_transition"] is None
    assert row["terminal_approach"]["feedback_source_timestamp"] == .2
    assert row["coast_elapsed_sec"] == .25-.1
    assert request(gov, row).throttle == 0.


@pytest.mark.parametrize("speed,remaining", [(0., 2.), (.1, 2.), (.35, 3.), (.5, 3.), (.12, 1.01), (.119999, 1.01)])
def test_continuous_law_exact_reference_zero_at_target_and_first_slew(speed, remaining):
    gov = governor(); row = prime(gov, speed, remaining)
    result = request(gov, row)
    target = min(.35, max(.12, .25*max(remaining-.75, 0.)))
    error = max(target-speed, 0.)
    desired = min(.15, .08*min(1., error/.1)+.5*error)
    decision = row["terminal_approach"]
    assert decision["target_speed_mps"] == target
    assert decision["positive_speed_error_mps"] == error
    assert decision["raw_throttle"] == desired
    assert result.throttle == decision["mathematical_throttle"] == min(desired, .1*.05)
    assert result.brake == result.steer == 0.
    if speed >= target:
        assert result.throttle == desired == 0.
    assert row["hold_ticks"] == 0 and row["complete"] is False
    json.dumps(row, allow_nan=False)


def test_slew_uses_measured_source_deltas_not_control_call_count_and_reductions_are_immediate():
    gov = governor(); row = prime(gov)
    first = request(gov, row).throttle
    assert request(gov, row).throttle == first
    stamp = .2000003
    row = observe(gov, stamp)
    second = request(gov, row).throttle
    assert second == first+.1*(stamp-.15)
    assert row["terminal_approach"]["previous_request_timestamp"] == .15
    assert request(gov, row).throttle == second
    row = observe(gov, .2500003, speed=.34)
    assert row["terminal_approach"]["target_speed_mps"] < .34
    assert row["terminal_approach"]["raw_throttle"] == 0. and request(gov, row).throttle == 0.
    row = observe(gov, .3000003)
    assert request(gov, row).throttle == pytest.approx(.005)


def test_goal_window_zero_pedals_then_original_hold_and_full_measured_dwell():
    gov = governor(); row = prime(gov); request(gov, row)
    row = observe(gov, .2, .11, .8)
    assert row["terminal_approach"]["decision"] == "goal_window_zero_pedal"
    assert row["terminal_approach"]["target_speed_mps"] == 0.
    result = request(gov, row)
    assert result.throttle == result.brake == 0. and row["hold_brake_latched"] is False
    for i in range(41):
        row = observe(gov, .25+i*.05, .09, .8)
        result = request(gov, row)
        assert result.throttle == result.steer == 0. and result.brake == 1.
        assert row["terminal_approach"]["decision"] == "original_hold_priority"
        assert row["hold_ticks"] == i and row["complete"] is (i == 40)
        assert row["hold_brake_control"]["mathematical_hold_brake"] == 1.
        assert row["hold_brake_source_timestamp"] == .25
        if i == 0:
            assert row["pilot_transition"] == "zero_pedal_coast_to_service_brake_hold"
    assert row["hold_duration_sec"] == 2.


def test_brake_hold_has_priority_if_low_speed_latch_and_goal_entry_share_observation():
    gov = governor(); row = prime(gov, .09, .8)
    assert row["pilot_transition"] == "zero_pedal_coast_to_service_brake_hold"
    assert row["terminal_approach"]["feedback_latched"] is False
    assert request(gov, row).brake == 1.


def test_first_request_after_unexecuted_observations_has_only_one_native_interval_budget():
    gov = governor(); row = prime(gov)
    row = observe(gov, .2)
    assert row["terminal_approach"]["previous_request_timestamp"] is None
    assert request(gov, row).throttle == .1*.05


def test_feedback_does_not_change_timeout_or_rewrite_prior_observation_on_new_failure():
    gov = governor(); row = prime(gov); request(gov, row)
    for tick in range(4, 903):
        previous = json.dumps(row, sort_keys=True)
        old_row = row
        row = observe(gov, tick*.05, .2, 2.)
        assert json.dumps(old_row, sort_keys=True) == previous
        request(gov, row)
    assert row["pilot_failure_reason"] == PROFILE+"_coast_timeout"
    assert row["coast_elapsed_sec"] == 45.
    assert row["terminal_approach"]["eligible"] is False
    assert row["terminal_approach_control"]["returned_control"]["throttle"] == 0.
    assert row["complete"] is False and row["hold_brake_latched"] is False


def test_same_row_emergency_invalidates_live_decision_but_preserves_upstream_snapshot_and_output():
    gov = governor(); row = prime(gov)
    planner = SimpleNamespace(_vehicle_controller=SimpleNamespace(max_brake=.3, max_throt=.75), run_step=control)
    def emergency(value):
        value.throttle, value.brake, value.steer = 0., .5, -.33
        return value
    agent = SimpleNamespace(get_local_planner=lambda: planner, add_emergency_stop=emergency)
    metadata = p.install_development_control(agent, gov)
    result = agent.add_emergency_stop(planner.run_step())
    p.annotate_development_control(row, gov, None)
    p.apply_low_speed_steering(result, gov, row, driving_command=True)
    decision = row["terminal_approach"]
    receipt = row["terminal_approach_control"]
    assert decision["eligible"] is False and decision["mathematical_throttle"] == 0.
    assert receipt["decision_at_normal_control"]["eligible"] is True
    assert receipt["decision_at_normal_control"]["mathematical_throttle"] == .005000000000000001
    assert receipt["returned_control"]["throttle"] == .005000000000000001
    assert receipt["superseded_by_emergency"] is receipt["superseded_by_governor_failure"] is True
    assert (result.throttle, result.brake, result.steer) == (0., .5, -.33)
    assert receipt["output_steering_stage_returned_control"]["brake"] == .5
    assert metadata["emergency_return_control_modified"] is False
    assert metadata["terminal_approach_target_is_actual_speed_cap"] is False


@pytest.mark.parametrize("driving", [True, False])
def test_new_observation_clears_normal_cache_and_early_failure_cannot_reuse_feedback(driving):
    gov = governor(); row = prime(gov); request(gov, row)
    row = observe(gov, .2, driving=driving)
    assert gov.last_terminal_approach_control is None
    gov.fail("event_failure")
    result = control()
    before = vars(result).copy()
    p.apply_low_speed_steering(result, gov, row, driving_command=driving)
    assert vars(result) == before
    receipt = row["terminal_approach_control"]
    assert receipt["normal_control_executed"] is False
    assert receipt["input_control"] is receipt["returned_control"] is receipt["decision_at_normal_control"] is None
    assert row["terminal_approach"]["eligible"] is False
    assert gov.normal_control(control()).throttle == 0.


def test_float32_serialized_controls_are_not_misreported_as_mathematical_request():
    class Float32Control(SimpleNamespace):
        def __setattr__(self, name, value):
            if name in ("throttle", "brake", "steer"):
                value = struct.unpack("f", struct.pack("f", value))[0]
            super().__setattr__(name, value)
    gov = governor(); row = prime(gov)
    result = request(gov, row, Float32Control(**vars(control())))
    receipt = row["terminal_approach_control"]
    mathematical = receipt["decision_at_normal_control"]["mathematical_throttle"]
    assert mathematical == .1*.05
    assert result.throttle == receipt["returned_control"]["throttle"] != mathematical
    assert gov.terminal_previous_requested_throttle == mathematical
    assert receipt["returned_control"] == row["low_speed_steering"]["input_control"]
    assert receipt["physical_application_proven"] is receipt["training_data_approved"] is False


def test_stale_record_and_stale_cached_control_fail_closed():
    gov = governor(); old = prime(gov); request(gov, old)
    cached = gov.last_terminal_approach_control
    row = observe(gov, .2)
    with pytest.raises(ValueError, match="current measured"):
        p.apply_low_speed_steering(control(), gov, old, driving_command=True)
    gov.last_terminal_approach_control = cached
    with pytest.raises(ValueError, match="another measured observation"):
        p.apply_low_speed_steering(control(), gov, row, driving_command=True)
