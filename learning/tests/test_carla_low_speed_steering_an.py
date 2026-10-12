"""HH_260906 - Prove AN changes only one measured normal steering output without CARLA."""
from __future__ import annotations

from dataclasses import asdict
import hashlib
from pathlib import Path
import sys
import types
from types import SimpleNamespace

import pytest

from scripts.e2e import collect_carla_low_speed_steering_an as an
from scripts.e2e import prepare_carla_common10_dataset as adapter


@pytest.fixture
def fixed_route(tmp_path, monkeypatch):
    route = tmp_path / "route.json"
    route.write_text('{"town":"Town01"}')
    monkeypatch.setattr(an.ak.aj, "ROUTE_SHA", hashlib.sha256(route.read_bytes()).hexdigest())
    return route


def arguments(tmp_path, route):
    return [str(tmp_path / "episode"), str(route), "--an-arm", an.ARM,
            "--goal-stop-profile", "comfortable_v2", "--target-speed-kmh", "9",
            "--physics-hz", "20", "--capture-hz", "10", "--max-duration-sec", "180",
            "--stationary-warmup-sec", "3.5", "--stationary-tail-sec", "6.5",
            "--goal-tolerance-m", "1", "--agent-initialization", "after_bootstrap",
            "--control-transport", "acknowledged_batch", "--weather", "ClearNoon",
            "--seed", "2026092601", "--spawn-z-offset-m", "0",
            "--basic-agent-base-min-distance-m", "2", "--basic-agent-distance-ratio", ".2",
            "--basic-agent-lateral-kp", "1.70", "--wheelbase-m", "2.85"]


def make_control(*, steer=.34, brake=0., throttle=.12, hand_brake=False):
    return SimpleNamespace(steer=steer, brake=brake, throttle=throttle,
                           hand_brake=hand_brake, reverse=False)


def test_am_inputs_and_training_denial_are_fixed(tmp_path, fixed_route):
    args = an.parse_args(arguments(tmp_path, fixed_route))
    config = an.validate_args(args)
    assert args.al_arm == an.al.ARM and args.ak_arm == "coast_27_hold"
    assert config.profile_id == an.al.PROFILE_ID
    assert config.stop_buffer_m == 2.7 and config.goal_tolerance_m == 1
    assert config.stopped_speed_mps == .1 and config.hold_seconds == 2
    assert config.normal_brake_cap == 0 and config.normal_throttle_cap == .2
    assert config.training_data_approved is False and config.development_only is True
    assert config.policy_source_sha256 == hashlib.sha256(Path(an.__file__).read_bytes()).hexdigest()
    with pytest.raises(adapter.AdapterError):
        adapter._require_native_admission_not_denied({
            "capture_contract": {"goal_stop_profile": asdict(config)}})


@pytest.mark.parametrize("extra", [
    ["--basic-agent-lateral-kp", "1.95"], ["--target-speed-kmh", "10"],
    ["--goal-tolerance-m", "2"], ["--weather", "WetNoon"],
])
def test_changed_comparator_inputs_rejected_before_simulator(tmp_path, fixed_route, extra):
    with pytest.raises((ValueError, SystemExit)):
        an.parse_args(arguments(tmp_path, fixed_route) + extra)


@pytest.mark.parametrize("observation,eligible", [
    (dict(remaining=.9, planar=.9, speed=.19, overshoot=-.3, driving=True), True),
    (dict(remaining=1.01, planar=.9, speed=.19, overshoot=-.3, driving=True), False),
    (dict(remaining=.9, planar=1.01, speed=.19, overshoot=-.3, driving=True), False),
    (dict(remaining=.9, planar=.9, speed=.201, overshoot=-.3, driving=True), False),
    (dict(remaining=.9, planar=.9, speed=.19, overshoot=.01, driving=True), False),
    (dict(remaining=.9, planar=.9, speed=.19, overshoot=-.3, driving=False), False),
])
def test_guard_changes_only_inside_measured_goal_window(tmp_path, fixed_route, observation, eligible):
    config = an.validate_args(an.parse_args(arguments(tmp_path, fixed_route)))
    with an.configuration_scope():
        governor = an.ANGovernor(config, 2.5, 20.)
        row = governor.update(observation["remaining"], observation["planar"],
                              observation["speed"], observation["overshoot"],
                              count_hold=observation["driving"])
        assert row["an_normal_steering"]["measured_guard_eligible"] is eligible
        control = make_control()
        applied = an.an_suppress_stopped_brake_steering(control, observation["speed"], True)
        assert applied is eligible
        assert control.steer == (0.0 if eligible else .34)
        assert (control.throttle, control.brake) == (.12, 0.)
        assert row["an_normal_steering"]["applied_to_normal_command"] is eligible


@pytest.mark.parametrize("brake,enabled", [(.5, True), (.5, False), (1., True)])
def test_emergency_and_hold_match_original_output_guard(tmp_path, fixed_route, brake, enabled):
    config = an.validate_args(an.parse_args(arguments(tmp_path, fixed_route)))
    with an.configuration_scope():
        governor = an.ANGovernor(config, 2.5, 20.)
        row = governor.update(.9, .9, .19, -.3)
        assert row["an_normal_steering"]["measured_guard_eligible"]
        candidate, baseline = make_control(brake=brake), make_control(brake=brake)
        an.an_suppress_stopped_brake_steering(candidate, .19, enabled)
        an.ORIGINAL_SUPPRESS(baseline, .19, enabled)
        assert vars(candidate) == vars(baseline)
        assert row["an_normal_steering"]["applied_to_normal_command"] is False
        assert row["an_normal_steering"]["emergency_or_hold_unchanged"] is True


def test_an_observation_cannot_be_reused_with_another_speed(tmp_path, fixed_route):
    config = an.validate_args(an.parse_args(arguments(tmp_path, fixed_route)))
    with an.configuration_scope():
        governor = an.ANGovernor(config, 2.5, 20.)
        governor.update(.9, .9, .19, -.3)
        with pytest.raises(ValueError, match="source frame"):
            an.an_suppress_stopped_brake_steering(make_control(), .18, True)


def test_latched_hold_does_not_reopen_steering_intervention(tmp_path, fixed_route):
    config = an.validate_args(an.parse_args(arguments(tmp_path, fixed_route)))
    with an.configuration_scope():
        governor = an.ANGovernor(config, 2.5, 20.)
        row = governor.update(.5, .5, .09, -.2)
        assert governor.hold_latched and not row["an_normal_steering"]["measured_guard_eligible"]
        control = make_control(brake=1.)
        an.an_suppress_stopped_brake_steering(control, .09, True)
        assert row["an_normal_steering"]["applied_to_normal_command"] is False
        assert (control.throttle, control.brake, control.steer) == (.12, 1., 0.)


def test_nested_hooks_restore_even_after_exception():
    with pytest.raises(RuntimeError, match="injected"):
        with an.configuration_scope():
            assert an.base.GoalStopGovernor is an.ANGovernor
            assert an.base.install_normal_brake_cap is an.an_installer
            assert an.base.suppress_stopped_brake_steering is an.an_suppress_stopped_brake_steering
            raise RuntimeError("injected")
    assert an.base.GoalStopGovernor is an.ak.ORIGINAL_GOVERNOR
    assert an.base.install_normal_brake_cap is an.ak.ORIGINAL_INSTALLER
    assert an.base.suppress_stopped_brake_steering is an.ORIGINAL_SUPPRESS


def test_basic_agent_source_is_exactly_pinned(tmp_path, monkeypatch):
    source = tmp_path / "PythonAPI/carla/agents/navigation/basic_agent.py"
    source.parent.mkdir(parents=True)
    source.write_bytes(b"verified BasicAgent fixture")
    monkeypatch.setenv("CARLA_ROOT", str(tmp_path))
    monkeypatch.setattr(an, "AN_BASIC_AGENT_SHA256", hashlib.sha256(source.read_bytes()).hexdigest())
    # HH_260906 - Mirror the installed PythonAPI module while injecting only the resolved fixture source.
    agents = types.ModuleType("agents")
    navigation = types.ModuleType("agents.navigation")
    basic_agent = types.ModuleType("agents.navigation.basic_agent")
    basic_agent.BasicAgent = type("BasicAgent", (), {})
    monkeypatch.setitem(sys.modules, "agents", agents)
    monkeypatch.setitem(sys.modules, "agents.navigation", navigation)
    monkeypatch.setitem(sys.modules, "agents.navigation.basic_agent", basic_agent)
    monkeypatch.setattr(an.inspect, "getsourcefile", lambda _: str(source))
    assert an.verify_basic_agent_source() == an.AN_BASIC_AGENT_SHA256
    source.write_bytes(b"changed BasicAgent fixture")
    with pytest.raises(ValueError, match="differs"):
        an.verify_basic_agent_source()
