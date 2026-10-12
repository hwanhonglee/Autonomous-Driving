# HH_260906 - Locate the repository independently of the relocated test directory.
"""HH_260906 - CPU-only v6 request tests: no CARLA imports, altered labels or admission claims."""

from dataclasses import asdict, replace
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts.e2e import carla_goal_stop_profile as profiles


ROOT = next(p for p in Path(__file__).resolve().parents if (p / ".git").exists())
PROFILE = "comfortable_v6_low_speed_steering"


def arguments(**changes):
    values = dict(goal_stop_profile=PROFILE, low_speed_steering_mode="native",
                  control_transport="acknowledged_batch", agent_initialization="after_bootstrap",
                  target_speed_kmh=28.8, physics_hz=20., capture_hz=10., goal_tolerance_m=1.,
                  stationary_warmup_sec=3.5, stationary_tail_sec=6.5, max_duration_sec=180.,
                  vehicle_type="vehicle.toyota.prius", weather="ClearNoon")
    return SimpleNamespace(**dict(values, **changes))


def governor(mode="zero"):
    return profiles.DevelopmentGoalStopGovernor(
        profiles.configuration_from_args(arguments(low_speed_steering_mode=mode)), 8., 20.)


def observed(gov, timestamp=0., *, speed=.02, remaining=100., driving=True):
    return gov.update(remaining, remaining, speed, -remaining, timestamp=timestamp,
                      longitudinal_speed_mps=speed, driving=driving)


def control(steer=.27):
    return SimpleNamespace(throttle=.4, brake=.1, steer=steer, hand_brake=False,
                           reverse=False, manual_gear_shift=False, gear=1)


@pytest.mark.parametrize("mode", ["native", "zero"])
def test_config_changes_only_declared_revision_mode_and_abba_budget(mode):
    previous = asdict(profiles.BrakeHoldDevelopmentGoalStopConfig())
    current = asdict(profiles.configuration_from_args(arguments(low_speed_steering_mode=mode)))
    assert {key: value for key, value in current.items() if previous.get(key) != value} == {
        "profile_id": PROFILE, "low_speed_steering_mode": mode,
        "low_speed_steering_threshold_mps": .75, "maximum_attempts_per_revision": 4,
        "research_only": True}
    assert current["normal_hold_brake"] == 1.
    assert current["hold_seconds"] == 2. and current["minimum_tail_seconds"] == 6.5
    assert current["training_data_approved"] is False and current["development_only"] is True


@pytest.mark.parametrize("mode", [None, "", "ZERO", "invalid", 0, False])
def test_v6_requires_explicit_valid_mode(mode):
    with pytest.raises(ValueError, match="explicit low_speed_steering_mode"):
        profiles.configuration_from_args(arguments(low_speed_steering_mode=mode))


@pytest.mark.parametrize("profile", ["disabled", "comfortable_v1", "comfortable_v2", "comfortable_v3",
    "comfortable_v4", "comfortable_v5_brake_hold", "turn_low_v1", *profiles.TURN_LAUNCH_PEDALS])
@pytest.mark.parametrize("mode", ["native", "zero"])
def test_mode_rejected_for_every_old_profile_before_other_arguments(profile, mode):
    with pytest.raises(ValueError, match="valid only"):
        profiles.configuration_from_args(SimpleNamespace(goal_stop_profile=profile, low_speed_steering_mode=mode))


def test_absent_or_none_mode_preserves_disabled_defaults():
    assert profiles.configuration_from_args(SimpleNamespace()) is None
    assert profiles.configuration_from_args(SimpleNamespace(low_speed_steering_mode=None)) is None


@pytest.mark.parametrize("changes", [dict(control_transport="legacy_async"),
    dict(agent_initialization="before_bootstrap"), dict(target_speed_kmh=30.),
    dict(stationary_warmup_sec=0.), dict(stationary_tail_sec=6.4), dict(physics_hz=10.),
    dict(capture_hz=20.), dict(goal_tolerance_m=2.), dict(max_duration_sec=181.),
    dict(vehicle_type="vehicle.other")])
def test_original_physical_and_collection_preflight_limits_remain(changes):
    with pytest.raises(ValueError):
        profiles.configuration_from_args(arguments(**changes))


@pytest.mark.parametrize("changes", [dict(low_speed_steering_threshold_mps=.8),
    dict(low_speed_steering_threshold_mps=0), dict(low_speed_steering_mode="other"),
    dict(normal_hold_brake=.5), dict(normal_hold_brake=True), dict(hold_seconds=1.),
    dict(minimum_tail_seconds=6.), dict(maximum_attempts_per_revision=5),
    dict(maximum_attempts_per_revision=4.), dict(training_data_approved=True),
    dict(research_only=False), dict(profile_id="comfortable_v5_brake_hold")])
def test_frozen_config_and_exact_field_types_cannot_be_weakened(changes):
    with pytest.raises(ValueError):
        profiles.DevelopmentGoalStopGovernor(
            replace(profiles.LowSpeedSteeringDevelopmentGoalStopConfig(), **changes), 8., 20.)


def test_exact_original_town07_route_and_weather_guards_remain():
    path = ROOT / "docs/assets/validation/2026-09-01/town07/autoware_vad/straight/autoware_vad_route.json"
    route = json.loads(path.read_text())
    profiles.validate_development_route(arguments(), route, path)
    with pytest.raises(ValueError):
        profiles.validate_development_route(arguments(weather="WetNoon"), route, path)
    with pytest.raises(ValueError):
        profiles.validate_development_route(arguments(), dict(route, town="Town03"), path)


@pytest.mark.parametrize("mode", ["native", "zero"])
@pytest.mark.parametrize("speed", [0., .1, .6342, .75, math.nextafter(.75, math.inf), 2.])
def test_inclusive_cutoff_changes_only_requested_steer(mode, speed):
    gov = governor(mode)
    record = observed(gov, speed=speed)
    original_record = dict(record)
    proposed = control()
    original_control = vars(proposed).copy()
    state = dict(gov.last_record)
    assert profiles.apply_low_speed_steering(proposed, gov, record, driving_command=True) is proposed
    expected = 0. if mode == "zero" and speed <= .75 else .27
    assert vars(proposed) == dict(original_control, steer=expected)
    assert gov.last_record == state
    assert {key: value for key, value in record.items() if key != "low_speed_steering"} == original_record
    receipt = record["low_speed_steering"]
    assert receipt["input_steer"] == .27 and receipt["returned_steer"] == expected
    assert receipt["input_control"] == original_control
    assert receipt["returned_control"] == dict(original_control, steer=expected)
    assert receipt["source_timestamp"] == 0. and receipt["source_measured_speed_mps"] == speed
    assert receipt["override_requested"] is (mode == "zero" and speed <= .75)
    assert receipt["physical_application_proven"] is False
    assert receipt["training_data_approved"] is False and receipt["research_only"] is True
    assert receipt["control_stage"] == "after_original_agent_proposal_checks_before_existing_stopped_brake_guard"


@pytest.mark.parametrize("steer", [-.8, 0., .8])
def test_zero_request_records_no_change_when_already_zero(steer):
    gov = governor()
    record = observed(gov)
    proposed = control(steer)
    profiles.apply_low_speed_steering(proposed, gov, record, driving_command=True)
    assert proposed.steer == 0.
    assert record["low_speed_steering"]["override_requested"] is True
    assert record["low_speed_steering"]["steering_value_changed"] is (steer != 0.)


@pytest.mark.parametrize("driving,command", [(False, False), (False, True), (True, False)])
def test_setup_tail_and_non_driving_commands_unchanged(driving, command):
    gov = governor()
    record = observed(gov, driving=driving)
    proposed = control()
    before = vars(proposed).copy()
    profiles.apply_low_speed_steering(proposed, gov, record, driving_command=command)
    assert vars(proposed) == before
    assert record["low_speed_steering"]["decision"] == "not_driving_command_unchanged"


@pytest.mark.parametrize("failure,emergencies", [("retained_failure", 0),
    (PROFILE + "_emergency_override", 1), (None, 1)])
def test_emergency_and_any_retained_governor_failure_unchanged(failure, emergencies):
    gov = governor()
    record = observed(gov)
    gov.failure_reason, gov.emergency_override_count = failure, emergencies
    proposed = control()
    before = vars(proposed).copy()
    profiles.apply_low_speed_steering(proposed, gov, record, driving_command=True)
    assert vars(proposed) == before
    assert record["low_speed_steering"]["override_requested"] is False


@pytest.mark.parametrize("change", [dict(measured_timestamp=.05), dict(measured_speed_mps=.03),
    dict(profile_id="old"), dict(low_speed_steering_driving_enabled=False)])
def test_receipt_rejects_stale_or_foreign_observation(change):
    gov = governor()
    record = observed(gov)
    record.update(change)
    proposed = control()
    with pytest.raises(ValueError, match="current measured governor state"):
        profiles.apply_low_speed_steering(proposed, gov, record, driving_command=True)
    assert proposed.steer == .27 and "low_speed_steering" not in record


@pytest.mark.parametrize("steer", [math.nan, math.inf, -math.inf])
def test_nonfinite_original_steering_never_becomes_finite_fabricated_control(steer):
    gov = governor()
    record = observed(gov)
    with pytest.raises(ValueError, match="finite"):
        profiles.apply_low_speed_steering(control(steer), gov, record, driving_command=True)
    assert "low_speed_steering" not in record


def test_missing_state_record_or_nonboolean_command_is_rejected():
    gov = governor()
    with pytest.raises(ValueError):
        profiles.apply_low_speed_steering(control(), gov, {}, driving_command=True)
    record = observed(gov)
    for missing_record, command in [(None, True), (record, 1)]:
        with pytest.raises(ValueError):
            profiles.apply_low_speed_steering(control(), gov, missing_record, driving_command=command)


def test_historical_profile_helper_is_noop_without_new_records_or_requirements():
    gov = profiles.DevelopmentGoalStopGovernor(profiles.BrakeHoldDevelopmentGoalStopConfig(), 8., 20.)
    record = observed(gov)
    before = dict(record)
    proposed = control()
    assert profiles.apply_low_speed_steering(proposed, gov, record, driving_command=True) is proposed
    assert record == before and proposed.steer == .27
    assert "low_speed_steering_driving_enabled" not in record
    assert profiles.apply_low_speed_steering(proposed, None, None, driving_command=False) is proposed


@pytest.mark.parametrize("mode", ["native", "zero"])
def test_v5_and_v6_pedals_hold_dwell_and_measured_records_match(mode):
    old = profiles.DevelopmentGoalStopGovernor(profiles.BrakeHoldDevelopmentGoalStopConfig(), 8., 20.)
    new = governor(mode)
    inputs = [(0., 180., 0.), (.05, 179., .5), (.1, 20., 3.)]
    inputs += [(.15 + index * .05, .8, .02) for index in range(41)]
    for stamp, remaining, speed in inputs:
        old_record = observed(old, stamp, speed=speed, remaining=remaining)
        new_record = observed(new, stamp, speed=speed, remaining=remaining)
        normalized = {key: value for key, value in new_record.items()
                      if key != "low_speed_steering_driving_enabled"}
        normalized["profile_id"] = old_record["profile_id"]
        assert json.dumps(normalized, sort_keys=True) == json.dumps(old_record, sort_keys=True)
        assert vars(new.normal_control(control())) == vars(old.normal_control(control()))
    assert new_record["hold_ticks"] == 40 and new_record["hold_duration_sec"] == 2.
    assert new_record["complete"] is True and new.normal_control(control()).brake == 1.


@pytest.mark.parametrize("emergency", [False, True])
def test_native_agent_proposal_and_internal_pid_updates_precede_optional_request(emergency):
    gov = governor()
    record = observed(gov)
    controller = SimpleNamespace(max_brake=.3, max_throt=.75, past_steering=.2, calls=0)
    planner = SimpleNamespace(_vehicle_controller=controller)

    def native_step():
        controller.calls += 1
        controller.past_steering += .1
        return control(controller.past_steering)

    def original_emergency(proposed):
        proposed.throttle, proposed.brake, proposed.steer = 0., .5, -.33
        return proposed

    planner.run_step = native_step
    agent = SimpleNamespace(get_local_planner=lambda: planner, add_emergency_stop=original_emergency)

    def agent_step():
        result = planner.run_step()
        return agent.add_emergency_stop(result) if emergency else result

    agent.run_step = agent_step
    metadata = profiles.install_development_control(agent, gov)
    assert agent.run_step is agent_step
    assert metadata["basic_agent_and_pid_internal_updates_modified"] is False
    proposed = agent.run_step()
    assert controller.calls == 1 and controller.past_steering == pytest.approx(.3)
    if not emergency:
        # HH_260906 - The collector's unchanged proposal check sees the original .1 PID step, not clamped zero.
        assert proposed.steer - .2 == pytest.approx(.1)
    original = vars(proposed).copy()
    profiles.annotate_development_control(record, gov, None)
    assert profiles.apply_low_speed_steering(proposed, gov, record, driving_command=True) is proposed
    assert controller.past_steering == pytest.approx(.3) and controller.calls == 1
    assert vars(proposed) == (original if emergency else dict(original, steer=0.))
    if emergency:
        assert record["next_control_is_unchanged_emergency_override"] is True


def test_full_control_projection_preserves_all_flags_and_gear_and_is_finite_json():
    gov = governor()
    record = observed(gov)
    proposed = control(-.63)
    proposed.hand_brake, proposed.reverse, proposed.manual_gear_shift, proposed.gear = True, True, True, -1
    original = vars(proposed).copy()
    profiles.apply_low_speed_steering(proposed, gov, record, driving_command=True)
    receipt = record["low_speed_steering"]
    assert receipt["input_control"] == original
    assert receipt["returned_control"] == dict(original, steer=0.)
    assert vars(proposed) == dict(original, steer=0.)
    json.dumps(receipt, allow_nan=False)


def test_control_projection_defaults_match_the_collector_exactly():
    assert profiles.low_speed_steering_control_projection(SimpleNamespace()) == {
        "throttle": 0., "steer": 0., "brake": 0., "hand_brake": False,
        "reverse": False, "gear": 0, "manual_gear_shift": False}
    assert list(profiles.low_speed_steering_control_projection(control())) == [
        "throttle", "steer", "brake", "hand_brake", "reverse", "gear", "manual_gear_shift"]


@pytest.mark.parametrize("changes", [dict(throttle=math.nan), dict(brake=math.inf),
    dict(steer=-math.inf), dict(throttle=True), dict(brake="0"), dict(throttle=1.01),
    dict(brake=-.01), dict(steer=1.01), dict(hand_brake=1), dict(reverse=0),
    dict(manual_gear_shift=None), dict(gear=1.), dict(gear=True)])
def test_malformed_full_projection_is_rejected_before_any_steering_change(changes):
    gov = governor()
    record = observed(gov)
    proposed = control()
    vars(proposed).update(changes)
    before = vars(proposed).copy()
    with pytest.raises(ValueError):
        profiles.apply_low_speed_steering(proposed, gov, record, driving_command=True)
    assert vars(proposed) == before and "low_speed_steering" not in record
