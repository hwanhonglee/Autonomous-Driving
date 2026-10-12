"""HH_260906 - Prove AO changes only a measured, normal launch steering command."""
from __future__ import annotations

from dataclasses import asdict
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts.e2e import collect_carla_launch_steering_ao as ao
from scripts.e2e import prepare_carla_common10_dataset as adapter


@pytest.fixture
def fixed_route(tmp_path, monkeypatch):
    route = tmp_path / "route.json"
    route.write_text(json.dumps({"town": "Town01", "coordinate_reference": "base_link",
                                 "route_length_m": ao.ROUTE_LENGTH_M}))
    monkeypatch.setattr(ao.ak.aj, "ROUTE_SHA", hashlib.sha256(route.read_bytes()).hexdigest())
    return route


def arguments(tmp_path, route):
    return [str(tmp_path / "episode"), str(route), "--ao-arm", ao.ARM,
            "--goal-stop-profile", "comfortable_v2", "--target-speed-kmh", "9",
            "--physics-hz", "20", "--capture-hz", "10", "--max-duration-sec", "180",
            "--stationary-warmup-sec", "3.5", "--stationary-tail-sec", "6.5",
            "--goal-tolerance-m", "1", "--agent-initialization", "after_bootstrap",
            "--control-transport", "acknowledged_batch", "--weather", "ClearNoon",
            "--seed", "2026092601", "--spawn-z-offset-m", "0",
            "--basic-agent-base-min-distance-m", "2", "--basic-agent-distance-ratio", ".2",
            "--basic-agent-lateral-kp", "1.70", "--wheelbase-m", "2.85"]


def control(*, steer=.0681, throttle=.12, brake=0., hand_brake=False):
    return SimpleNamespace(steer=steer, throttle=throttle, brake=brake,
                           hand_brake=hand_brake, reverse=False)


def test_fixed_an_input_and_training_denial(tmp_path, fixed_route):
    args = ao.parse_args(arguments(tmp_path, fixed_route))
    config = ao.validate_args(args)
    assert args.an_arm == ao.an.ARM and args.al_arm == ao.an.al.ARM
    assert config.normal_brake_cap == 0 and config.normal_throttle_cap == .2
    assert config.training_data_approved is False and config.development_only is True
    assert config.policy_source_sha256 == hashlib.sha256(Path(ao.__file__).read_bytes()).hexdigest()
    with pytest.raises(adapter.AdapterError):
        adapter._require_native_admission_not_denied({
            "capture_contract": {"goal_stop_profile": asdict(config)}})


@pytest.mark.parametrize("extra", [
    ["--basic-agent-lateral-kp", "1.95"], ["--target-speed-kmh", "10"],
    ["--goal-tolerance-m", "2"], ["--weather", "WetNoon"],
])
def test_changed_comparator_input_rejected_before_simulator(tmp_path, fixed_route, extra):
    with pytest.raises((ValueError, SystemExit)):
        ao.parse_args(arguments(tmp_path, fixed_route) + extra)


def test_route_length_or_reference_change_rejected_before_simulator(tmp_path, fixed_route, monkeypatch):
    for key, changed in (("route_length_m", 76.), ("coordinate_reference", "map")):
        route = json.loads(fixed_route.read_text())
        route[key] = changed
        fixed_route.write_text(json.dumps(route))
        monkeypatch.setattr(ao.ak.aj, "ROUTE_SHA",
                            hashlib.sha256(fixed_route.read_bytes()).hexdigest())
        with pytest.raises(ValueError, match="route length/reference"):
            ao.parse_args(arguments(tmp_path, fixed_route))
        route[key] = ao.ROUTE_LENGTH_M if key == "route_length_m" else "base_link"
        fixed_route.write_text(json.dumps(route))


@pytest.mark.parametrize("remaining,speed,driving,expected", [
    (ao.ROUTE_LENGTH_M, 0., False, True),  # initial BasicAgent handoff in stationary_warmup
    (ao.ROUTE_LENGTH_M, .098, True, True),
    (ao.ROUTE_LENGTH_M, .20, True, False),
    (ao.ROUTE_LENGTH_M - 1., .098, True, False),
    (ao.ROUTE_LENGTH_M - 5., .098, True, False),
])
def test_launch_window_is_strict_and_includes_first_handoff(
        tmp_path, fixed_route, remaining, speed, driving, expected):
    config = ao.validate_args(ao.parse_args(arguments(tmp_path, fixed_route)))
    with ao.configuration_scope():
        governor = ao.an.ANGovernor(config, 2.5, 20.)
        row = governor.update(remaining, 58., speed, -29.85, count_hold=driving)
        candidate = control()
        applied = ao.ao_suppress_stopped_brake_steering(candidate, speed, True)
        assert applied is expected
        assert candidate.steer == (0. if expected else .0681)
        assert (candidate.throttle, candidate.brake) == (.12, 0.)
        evidence = row["ao_launch_steering"]
        assert evidence["applied_to_normal_command"] is expected
        assert evidence["includes_first_handoff_command"] is (expected and not driving)


@pytest.mark.parametrize("brake,hand_brake,enabled", [
    (.5, False, True), (1., False, True), (0., True, True), (.5, False, False),
])
def test_emergency_and_hold_output_matches_an_exactly(
        tmp_path, fixed_route, brake, hand_brake, enabled):
    config = ao.validate_args(ao.parse_args(arguments(tmp_path, fixed_route)))
    with ao.configuration_scope():
        governor = ao.an.ANGovernor(config, 2.5, 20.)
        row = governor.update(ao.ROUTE_LENGTH_M, 58., .09, -29.85, count_hold=True)
        candidate = control(brake=brake, hand_brake=hand_brake)
        baseline = control(brake=brake, hand_brake=hand_brake)
        ao.ao_suppress_stopped_brake_steering(candidate, .09, enabled)
        ao.an.an_suppress_stopped_brake_steering(baseline, .09, enabled)
        assert vars(candidate) == vars(baseline)
        assert row["ao_launch_steering"]["applied_to_normal_command"] is False
        assert row["ao_launch_steering"]["emergency_or_hold_unchanged"] is True


def test_an_goal_window_retained_and_ao_does_not_apply(tmp_path, fixed_route):
    config = ao.validate_args(ao.parse_args(arguments(tmp_path, fixed_route)))
    with ao.configuration_scope():
        governor = ao.an.ANGovernor(config, 2.5, 20.)
        row = governor.update(.9, .9, .19, -.3)
        candidate = control(steer=.34)
        assert ao.ao_suppress_stopped_brake_steering(candidate, .19, True)
        assert candidate.steer == 0.
        assert row["ao_launch_steering"]["applied_to_normal_command"] is False
        assert row["an_normal_steering"]["applied_to_normal_command"] is True


def test_stale_speed_rejected_and_all_hooks_restore(tmp_path, fixed_route):
    config = ao.validate_args(ao.parse_args(arguments(tmp_path, fixed_route)))
    with pytest.raises(RuntimeError, match="injected"):
        with ao.configuration_scope():
            assert ao.base.GoalStopGovernor is ao.an.ANGovernor
            assert ao.base.install_normal_brake_cap is ao.ao_installer
            assert ao.base.suppress_stopped_brake_steering is ao.ao_suppress_stopped_brake_steering
            governor = ao.an.ANGovernor(config, 2.5, 20.)
            governor.update(ao.ROUTE_LENGTH_M, 58., .09, -29.85, count_hold=False)
            with pytest.raises(ValueError, match="source frame"):
                ao.ao_suppress_stopped_brake_steering(control(), .08, True)
            raise RuntimeError("injected")
    assert ao.base.GoalStopGovernor is ao.ak.ORIGINAL_GOVERNOR
    assert ao.base.install_normal_brake_cap is ao.ak.ORIGINAL_INSTALLER
    assert ao.base.suppress_stopped_brake_steering is ao.an.ORIGINAL_SUPPRESS
