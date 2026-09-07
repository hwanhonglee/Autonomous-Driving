"""HH_260906 - Test the isolated C-track low-speed turn scope with no simulator or dataset mutation."""

from dataclasses import asdict, replace
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts/e2e"))
import carla_goal_stop_profile as profiles
import collect_carla_vad_expert as collector


def arguments(**changes):
    values = dict(goal_stop_profile="turn_low_v1", control_transport="acknowledged_batch", target_speed_kmh=14.4,
        stationary_warmup_sec=3.5, stationary_tail_sec=6.5, physics_hz=20., capture_hz=10., goal_tolerance_m=1.,
        vehicle_type="vehicle.toyota.prius", max_duration_sec=180., weather="ClearNoon")
    values.update(changes)
    return SimpleNamespace(**values)


def tick(governor, timestamp, speed=.5, remaining=100., **changes):
    values = dict(timestamp=timestamp, longitudinal_speed_mps=speed, driving=True)
    values.update(changes)
    return governor.update(remaining, remaining, speed, -remaining, **values)


def test_exact_new_configuration_only_changes_named_low_speed_route_fields():
    old = asdict(profiles.BrakeFreeDevelopmentGoalStopConfig())
    new = asdict(profiles.configuration_from_args(arguments()))
    assert {k for k in new if new[k] != old[k]} == {"profile_id", "nominal_cruise_speed_mps", "maximum_actual_speed_mps",
        "cruise_minimum_speed_mps", "cruise_maximum_speed_mps", "route_sha256", "route_length_m"}
    assert new["normal_brake_cap"] == 0 and new["normal_throttle_cap"] == .4
    assert new["goal_tolerance_m"] == 1 and new["hold_seconds"] == 2 and new["minimum_tail_seconds"] == 6.5
    assert new["route_sha256"] == "2299e5bf2bc86789650da72336fdcfe7c11c585df0b9e0a0fe29a44730bae79b"


@pytest.mark.parametrize("changes", [dict(target_speed_kmh=28.8), dict(target_speed_kmh=15.48),
    dict(control_transport="legacy_async"), dict(physics_hz=10), dict(capture_hz=5), dict(goal_tolerance_m=1.1),
    dict(stationary_warmup_sec=0), dict(stationary_tail_sec=6.4), dict(max_duration_sec=181), dict(vehicle_type="vehicle.other")])
def test_unreviewed_arguments_fail(changes):
    with pytest.raises(ValueError):
        profiles.configuration_from_args(arguments(**changes))


def test_cli_opt_in_and_old_default_remain_distinct():
    args = ["out", "route", "--goal-stop-profile", "turn_low_v1", "--target-speed-kmh", "14.4",
        "--stationary-warmup-sec", "3.5", "--stationary-tail-sec", "6.5", "--goal-tolerance-m", "1.0"]
    with pytest.raises(SystemExit):
        collector.parse_args(args)
    assert collector.parse_args(args + ["--control-transport", "acknowledged_batch"]).goal_stop_profile == "turn_low_v1"
    assert collector.parse_args(["out", "route"]).goal_stop_profile == "disabled"


@pytest.fixture
def guarded_route(tmp_path, monkeypatch):
    # HH_260906 - Synthetic source pins are test-local; production exposes no alternate-route override.
    route = dict(schema_version=1, coordinate_reference="base_link", town="C_track_1_0_7", scenario="left",
        weather="ClearNoon", route_length_m=206.31622010469437,
        start_carla_transform=dict(x=0., y=0., z=14.4), goal_carla_transform=dict(x=206.31622010469437, y=0., z=13.9),
        route=[dict(x=0., y=0., z=13.9, distance_m=0., vad_command=3),
               dict(x=206.31622010469437, y=0., z=13.9, distance_m=206.31622010469437, vad_command=3)],
        historical_preflight=dict(status="PASS", target_speed_kmh=30.))
    path = tmp_path / "source_route.json"
    path.write_text(json.dumps(route))
    config = replace(profiles.TurnLowDevelopmentGoalStopConfig(), route_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    monkeypatch.setattr(profiles, "configuration_from_args", lambda args: config)
    return route, path


def test_exact_original_route_without_downstream_alignment_is_accepted(guarded_route):
    route, path = guarded_route
    profiles.validate_development_route(arguments(), route, path)


@pytest.mark.parametrize("change", ["town", "scenario", "weather", "length", "alignment", "bytes"])
def test_route_source_geometry_weather_and_alignment_guard(guarded_route, change):
    route, path = guarded_route
    if change == "town": route["town"] = "Town03"
    elif change == "scenario": route["scenario"] = "right"
    elif change == "weather": route["weather"] = "WetNoon"
    elif change == "length": route["route_length_m"] = 207.
    elif change == "alignment": route["coordinate_alignment"] = {"z_m": -15.}
    else: path.write_bytes(path.read_bytes() + b"\n")
    with pytest.raises(ValueError):
        profiles.validate_development_route(arguments(), route, path)


@pytest.mark.parametrize("change", [dict(normal_brake_cap=.1), dict(maximum_actual_speed_mps=4.4),
    dict(goal_tolerance_m=True), dict(route_sha256="0" * 64), dict(minimum_cruise_seconds=0.)])
def test_named_configuration_and_field_types_cannot_smuggle_bounds(change):
    config = replace(profiles.TurnLowDevelopmentGoalStopConfig(), **change)
    with pytest.raises(ValueError):
        profiles.DevelopmentGoalStopGovernor(config, 4., 20.)


def test_arbitrary_subclass_is_not_an_enabled_profile():
    class Foreign(profiles.TurnLowDevelopmentGoalStopConfig):
        pass
    with pytest.raises(ValueError):
        profiles.DevelopmentGoalStopGovernor(Foreign(), 4., 20.)


def test_new_actual_speed_bound_is_strict_without_relaxing_old_physical_limits():
    governor = profiles.DevelopmentGoalStopGovernor(profiles.TurnLowDevelopmentGoalStopConfig(), 4., 20.)
    assert tick(governor, 0, 4.3)["pilot_failure_reason"] is None
    row = tick(governor, .05, math.nextafter(4.3, math.inf))
    assert row["pilot_failure_reason"] == "turn_low_v1_actual_speed_exceeded"


def test_low_speed_target_keeps_launch_slew_and_zero_normal_brake():
    governor = profiles.DevelopmentGoalStopGovernor(profiles.TurnLowDevelopmentGoalStopConfig(), 4., 20.)
    previous = 0.
    for index in range(100):
        row = tick(governor, index * .05, .2 if index < 10 else 4.)
        assert 0 <= row["target_speed_mps"] <= 4
        assert row["target_speed_mps"] <= previous + .05 + 1e-12
        previous = row["target_speed_mps"]
        command = SimpleNamespace(throttle=.7, brake=.2, steer=.31)
        governor.normal_control(command)
        assert command.brake == 0 and command.steer == .31 and command.throttle <= row["normal_throttle_cap"]


def test_coast_entry_failure_has_no_brake_rescue_or_endpoint_relaxation():
    governor = profiles.DevelopmentGoalStopGovernor(profiles.TurnLowDevelopmentGoalStopConfig(), 4., 20.)
    tick(governor, 0)
    row = tick(governor, .05, 3.3, remaining=23.)
    assert row["pilot_failure_reason"] == "turn_low_v1_coast_entry_speed_outside_band"
    assert not row["coast_entry_latched"]
    assert profiles.goal_stop_termination_reason(row, False, 0., governor.config) == row["pilot_failure_reason"]


def test_emergency_control_identity_and_following_observation_are_preserved():
    governor = profiles.DevelopmentGoalStopGovernor(profiles.TurnLowDevelopmentGoalStopConfig(), 4., 20.)
    tick(governor, 0)
    planner = SimpleNamespace(_vehicle_controller=SimpleNamespace(max_brake=.3, max_throt=.75))
    planner.run_step = lambda: SimpleNamespace(throttle=0., brake=.3, steer=.37)
    def emergency(command):
        assert command.brake == 0
        command.brake = .5
        return command
    agent = SimpleNamespace(get_local_planner=lambda: planner, add_emergency_stop=emergency)
    metadata = profiles.install_development_control(agent, governor)
    original = planner.run_step()
    assert agent.add_emergency_stop(original) is original and original.brake == .5 and original.steer == .37
    row = tick(governor, .05)
    assert row["pilot_failure_reason"] == "turn_low_v1_emergency_override"
    assert metadata["emergency_return_control_modified"] is False


def low_speed_quality_records():
    # HH_260906 - This scalar-only synthetic fixture never asserts actual simulator positions or future XY feasibility.
    speeds = [index * .05 for index in range(80)] + [4.] * 101 + [4. - index * .05 for index in range(1, 81)] + [0.] * 41
    governor = profiles.GoalStopGovernor(profiles.ComfortableGoalStopConfig(), 4., 20.)
    records = []
    for index, speed in enumerate(speeds + [0.] * 130):
        phase = "driving" if index < len(speeds) else "stationary_tail"
        status = governor.update(.75, .75, speed, -.75, count_hold=phase == "driving")
        records.append(dict(frame=index, timestamp=index * .05, vx=speed, vy=0., route_cte_m=0.,
                            capture_phase=phase, goal_stop=status))
    return records


def scalar_quality(records):
    bounds = dict(physical_decoder=dict(maximum_acceleration_mps2=2.9, maximum_deceleration_mps2=2.9),
                  runtime_speed_rate_gate=dict(maximum_acceleration_mps2=3., maximum_deceleration_mps2=6.))
    return profiles.measured_stop_quality(records, [r["frame"] for r in records[::2]],
                                         profiles.TurnLowDevelopmentGoalStopConfig(), bounds, True)


def test_successful_low_speed_scalar_check_never_claims_30_or_training_admission():
    report = scalar_quality(low_speed_quality_records())
    assert report["status"] == "PASS"
    detail = report["development_pilot"]
    assert detail["cruise_requirement_met"] and detail["longest_continuous_cruise_seconds"] >= 5.
    assert detail["cruise_band_mps"] == [3.8, 4.2] and detail["actual_speed_limit_mps"] == 4.3
    assert detail["qualification_30_kph"] == "NOT_CLAIMED" and not detail["training_data_approved"]
    assert detail["old_30_kph_cruise_criterion"] == dict(band_mps=[7.8, 8.2], seconds=5., claimed_met=False)
    assert detail["future_dataset_split_if_separately_admitted"] == "train"
    assert report["bounds"]["physical_decoder"]["maximum_deceleration_mps2"] == 2.9
    assert report["bounds"]["runtime_speed_rate_gate"]["maximum_deceleration_mps2"] == 6.


@pytest.mark.parametrize("failure", ["speed", "cruise", "emergency", "missing_tail"])
def test_failed_low_speed_scalar_check_retains_strict_original_quality(failure):
    records = low_speed_quality_records()
    if failure == "speed": records[90]["vx"] = math.nextafter(4.3, math.inf)
    elif failure == "cruise": records[130]["vx"] = 3.79
    elif failure == "emergency": records[90]["goal_stop"]["pilot_failure_reason"] = "turn_low_v1_emergency_override"
    else: records.pop()
    report = scalar_quality(records)
    assert report["status"] == "FAIL" and not report["development_pilot"]["training_data_approved"]
    assert report["development_pilot"]["qualification_30_kph"] == "NOT_CLAIMED"


def test_failed_capture_retains_original_height_historical_notice_and_low_speed_scope(tmp_path, monkeypatch, guarded_route):
    route, path = guarded_route
    output = tmp_path / "episode"
    def failed(_args, copied_route, _specs, _partial, _states, _cameras, manifest):
        # HH_260906 - Replace only the simulator operation; ordinary parsing, source copying and failed evidence persistence still execute.
        assert copied_route == route
        assert copied_route["start_carla_transform"]["z"] == 14.4
        profile = manifest["capture_contract"]["goal_stop_profile"]
        assert "historical route-shape" in profile["route_metadata_qualification_notice"]
        assert profile["qualification_30_kph"] == "NOT_CLAIMED"
        alignment = profile["downstream_map_alignment_metadata_only"]
        assert alignment["carla_to_autoware_map_translation_m"] == [0., 0., -15.]
        assert not alignment["applied_to_native_spawn_goal_state_or_sensor_tf"]
        raise collector.CollectionError("synthetic retained turn failure")
    monkeypatch.setattr(collector, "collect_episode", failed)
    flags = [str(output), str(path), "--goal-stop-profile", "turn_low_v1", "--target-speed-kmh", "14.4",
             "--stationary-warmup-sec", "3.5", "--stationary-tail-sec", "6.5", "--goal-tolerance-m", "1.0",
             "--control-transport", "acknowledged_batch"]
    with pytest.raises(collector.CollectionError, match="synthetic retained turn failure"):
        collector.run(collector.parse_args(flags))
    partial = Path(str(output) + ".partial")
    manifest = json.loads((partial / "manifest.json").read_text())
    assert manifest["status"] == "failed" and not output.exists()
    assert manifest["result"]["qualification_30_kph"] == "NOT_CLAIMED"
    assert not manifest["result"]["training_data_approved"] and manifest["result"]["development_only"]
    assert (partial / "route.json").read_bytes() == path.read_bytes()
    assert (partial / "control_receipts.jsonl").exists()


def test_old_v4_trace_matches_exact_pre_turn_source(tmp_path):
    # HH_260906 - Compare executable historical source, including full control outputs, without reproducing its logic.
    source = subprocess.check_output(["git", "show", "3a06925:scripts/e2e/carla_goal_stop_profile.py"], cwd=ROOT)
    path = tmp_path / "old_profile.py"
    path.write_bytes(source)
    spec = importlib.util.spec_from_file_location("hh_pre_turn_profile", path)
    original = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = original
    try:
        spec.loader.exec_module(original)
        old = original.DevelopmentGoalStopGovernor(original.BrakeFreeDevelopmentGoalStopConfig(), 8., 20.)
        new = profiles.DevelopmentGoalStopGovernor(profiles.BrakeFreeDevelopmentGoalStopConfig(), 8., 20.)
        assert asdict(old.config) == asdict(new.config)
        for i in range(350):
            speed, remaining = min(8., i * .03), 200. - i * .3
            assert tick(old, i * .05, speed, remaining) == tick(new, i * .05, speed, remaining)
            for throttle, brake in ((.6, 0.), (0., .02), (0., .3)):
                args = dict(throttle=throttle, brake=brake, steer=.1)
                assert vars(old.normal_control(SimpleNamespace(**args))) == vars(new.normal_control(SimpleNamespace(**args)))
    finally:
        sys.modules.pop(spec.name, None)
