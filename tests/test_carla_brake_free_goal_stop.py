"""HH_260906 - Test the brake-only development revision without weakening existing quality or emergency authority."""

from dataclasses import asdict, replace
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts/e2e"))
import carla_goal_stop_profile as profiles
from collect_carla_vad_expert import parse_args


def arguments(**changes):
    values = dict(goal_stop_profile="comfortable_v4", control_transport="acknowledged_batch",
                  target_speed_kmh=28.8, stationary_warmup_sec=3.5, stationary_tail_sec=6.5,
                  physics_hz=20.0, capture_hz=10.0, goal_tolerance_m=1.0,
                  vehicle_type="vehicle.toyota.prius", max_duration_sec=180.0, weather="ClearNoon")
    values.update(changes)
    return SimpleNamespace(**values)


def tick(governor, timestamp, speed=0.5, remaining=100.0, **changes):
    kwargs = dict(timestamp=timestamp, longitudinal_speed_mps=speed, driving=True)
    kwargs.update(changes)
    return governor.update(remaining, remaining, speed, -remaining, **kwargs)


def test_only_profile_identity_and_normal_brake_cap_change():
    old = asdict(profiles.DevelopmentGoalStopConfig())
    new = asdict(profiles.configuration_from_args(arguments()))
    assert {key: value for key, value in new.items() if old[key] != value} == {
        "profile_id": "comfortable_v4", "normal_brake_cap": 0.0}
    assert new["development_only"] and not new["training_data_approved"]
    assert new["maximum_attempts_per_revision"] == 2


@pytest.mark.parametrize("changes", [
    {"control_transport": "legacy_async"}, {"target_speed_kmh": 30.0},
    {"physics_hz": 10.0}, {"capture_hz": 5.0}, {"stationary_warmup_sec": 0.0},
    {"stationary_tail_sec": 6.4}, {"max_duration_sec": 181.0},
    {"goal_tolerance_m": 1.1}, {"vehicle_type": "vehicle.other"},
])
def test_unfrozen_arguments_rejected(changes):
    with pytest.raises(ValueError):
        profiles.configuration_from_args(arguments(**changes))


def test_cli_requires_ack_and_keeps_default_disabled():
    flags = ["out", "route", "--goal-stop-profile", "comfortable_v4",
             "--target-speed-kmh", "28.8", "--stationary-warmup-sec", "3.5",
             "--stationary-tail-sec", "6.5", "--goal-tolerance-m", "1.0"]
    with pytest.raises(SystemExit):
        parse_args(flags)
    assert parse_args(flags + ["--control-transport", "acknowledged_batch"]).goal_stop_profile == "comfortable_v4"
    assert parse_args(["out", "route"]).goal_stop_profile == "disabled"


@pytest.mark.parametrize("mutation", ["bytes", "town", "weather"])
def test_same_exact_route_weather_guard(tmp_path, mutation):
    path = ROOT / "docs/assets/validation/2026-09-01/town07/autoware_vad/straight/autoware_vad_route.json"
    route = json.loads(path.read_text())
    args = arguments()
    profiles.validate_development_route(args, route, path)
    if mutation == "bytes":
        changed = tmp_path / "route.json"
        changed.write_bytes(path.read_bytes() + b"\n")
        path = changed
    elif mutation == "town":
        route["town"] = "Town03"
    else:
        args.weather = "WetNoon"
    with pytest.raises(ValueError, match="comfortable_v4"):
        profiles.validate_development_route(args, route, path)


@pytest.mark.parametrize("config", [
    replace(profiles.BrakeFreeDevelopmentGoalStopConfig(), maximum_actual_speed_mps=10.0),
    replace(profiles.DevelopmentGoalStopConfig(), normal_brake_cap=0.0),
    replace(profiles.BrakeFreeDevelopmentGoalStopConfig(), normal_brake_cap=0.1),
])
def test_named_config_cannot_smuggle_arbitrary_parameters(config):
    with pytest.raises(ValueError):
        profiles.DevelopmentGoalStopGovernor(config, 8.0, 20.0)


def test_zero_normal_brake_does_not_clip_later_emergency_return():
    governor = profiles.DevelopmentGoalStopGovernor(profiles.BrakeFreeDevelopmentGoalStopConfig(), 8.0, 20.0)
    tick(governor, 0.0)
    controller = SimpleNamespace(max_brake=0.3, max_throt=0.75)
    planner = SimpleNamespace(_vehicle_controller=controller)
    planner.run_step = lambda: SimpleNamespace(throttle=0.0, brake=0.3, steer=0.37)
    returned = []
    def emergency(control):
        assert control.brake == 0.0
        control.brake = 0.5
        returned.append(control)
        return control
    agent = SimpleNamespace(get_local_planner=lambda: planner, add_emergency_stop=emergency)
    metadata = profiles.install_development_control(agent, governor)
    result = agent.add_emergency_stop(planner.run_step())
    assert result is returned[0] and (result.brake, result.steer) == (0.5, 0.37)
    assert controller.max_brake == 0.0
    record = dict(governor.last_record)
    profiles.annotate_development_control(record, governor, None)
    assert record["emergency_failure_pending_next_tick"]
    assert record["pilot_failure_reason"] == "comfortable_v4_emergency_override"
    assert metadata["emergency_return_control_modified"] is False
    observed = tick(governor, .05, .4)
    reason = profiles.goal_stop_termination_reason(observed, False, 0.0, governor.config)
    assert reason == "comfortable_v4_emergency_override"


def test_zero_normal_brake_does_not_rescue_fast_coast_entry():
    governor = profiles.DevelopmentGoalStopGovernor(profiles.BrakeFreeDevelopmentGoalStopConfig(), 8.0, 20.0)
    tick(governor, 0.0)
    record = tick(governor, .05, 4.0, remaining=25.0)
    assert record["pilot_failure_reason"] == "comfortable_v4_coast_entry_speed_outside_band"
    assert not record["coast_entry_latched"]
    assert profiles.goal_stop_termination_reason(record, False, 0.0, governor.config) == record["pilot_failure_reason"]


def test_old_v3_deterministic_trace_exactly_matches_committed_pre_revision(tmp_path):
    # HH_260906 - Compare executable old source, not a rewritten expected implementation.
    source = subprocess.check_output(["git", "show", "cb46f359834f959db819f7cd689427f230031760:scripts/e2e/carla_goal_stop_profile.py"], cwd=ROOT)
    path = tmp_path / "original_profile.py"
    path.write_bytes(source)
    spec = importlib.util.spec_from_file_location("hh_original_profile_v3", path)
    original = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = original
    try:
        spec.loader.exec_module(original)
        old = original.DevelopmentGoalStopGovernor(original.DevelopmentGoalStopConfig(), 8.0, 20.0)
        new = profiles.DevelopmentGoalStopGovernor(profiles.DevelopmentGoalStopConfig(), 8.0, 20.0)
        assert asdict(old.config) == asdict(new.config)
        for index in range(350):
            speed = min(8.0, index * .03)
            remaining = 200.0 - index * .3
            assert tick(old, index * .05, speed, remaining) == tick(new, index * .05, speed, remaining)
            for throttle, brake in ((.5, 0.), (0., .02), (0., .3)):
                args = dict(throttle=throttle, brake=brake, steer=.1)
                assert vars(old.normal_control(SimpleNamespace(**args))) == vars(new.normal_control(SimpleNamespace(**args)))
    finally:
        sys.modules.pop(spec.name, None)
