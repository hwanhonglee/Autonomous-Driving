"""HH_260906 - Verify the separate AK controller without running or admitting CARLA data."""
from dataclasses import asdict
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts.e2e import collect_carla_goal_stop_ak as ak
from scripts.e2e import prepare_carla_common10_dataset as adapter


@pytest.fixture(autouse=True)
def fixed_route(tmp_path, monkeypatch):
    route = tmp_path / "route.json"
    route.write_text('{"town":"Town01"}')
    monkeypatch.setattr(ak.aj, "ROUTE_SHA", ak.hashlib.sha256(route.read_bytes()).hexdigest())
    return route


def arguments(tmp_path, route, arm):
    return [str(tmp_path / "episode"), str(route), "--ak-arm", arm,
            "--goal-stop-profile", "comfortable_v2", "--target-speed-kmh", "9",
            "--physics-hz", "20", "--capture-hz", "10", "--max-duration-sec", "180",
            "--stationary-warmup-sec", "3.5", "--stationary-tail-sec", "6.5",
            "--goal-tolerance-m", "1", "--agent-initialization", "after_bootstrap",
            "--control-transport", "acknowledged_batch", "--weather", "ClearNoon",
            "--seed", "2026092601", "--spawn-z-offset-m", "0",
            "--basic-agent-base-min-distance-m", "2", "--basic-agent-distance-ratio", ".2"]


@pytest.mark.parametrize("arm", ak.ARMS)
def test_exact_protocol_and_admission_remain_denied(tmp_path, fixed_route, arm):
    args = ak.parse_args(arguments(tmp_path, fixed_route, arm))
    config = ak.validate_args(args)
    assert config.ak_arm == arm and config.stop_buffer_m == 2.7
    assert config.normal_throttle_cap == .2 and config.normal_brake_cap == 0
    assert config.launch_only_cap == .13 and config.desired_deceleration_mps2 == .12
    assert config.goal_tolerance_m == 1 and config.stopped_speed_mps == .1
    assert config.hold_seconds == 2 and config.training_data_approved is False
    assert config.development_only is True
    with pytest.raises(adapter.AdapterError):
        adapter._require_native_admission_not_denied({
            "capture_contract": {"goal_stop_profile": asdict(config)}})


@pytest.mark.parametrize("flag,value", [("--target-speed-kmh", "10"),
    ("--goal-tolerance-m", "2"), ("--weather", "WetNoon")])
def test_protocol_change_fails_before_simulator(tmp_path, fixed_route, flag, value):
    with pytest.raises((ValueError, SystemExit)):
        ak.parse_args(arguments(tmp_path, fixed_route, "coast_27_hold") + [flag, value])


def test_hold_requires_measured_upstream_goal_window_and_next_normal_command():
    velocity = SimpleNamespace(x=0.09, y=0.)
    planner = SimpleNamespace(
        _vehicle_controller=SimpleNamespace(max_brake=.5, max_throt=.75),
        _vehicle=SimpleNamespace(get_velocity=lambda: velocity),
        run_step=lambda: SimpleNamespace(throttle=.2, brake=0., steer=.25))
    with ak.configuration_scope():
        governor = ak.AKGovernor(ak.AKConfig(ak_arm="coast_27_hold"), 2.5, 20.)
        ak.ak_installer(planner, 0., .2)
        before = governor.update(.5, .5, .09, -.5, count_hold=False)
        assert not before["hold_brake_latched"]
        assert planner.run_step().brake == 0.
        outside = governor.update(1.1, 1.1, .09, -1.1)
        assert not outside["hold_brake_latched"]
        fast = governor.update(.5, .5, .11, -.5)
        assert not fast["hold_brake_latched"]
        entered = governor.update(.5, .5, .09, -.5)
        assert entered["hold_brake_latched"] and entered["hold_ticks"] == 0
        control = planner.run_step()
        assert (control.throttle, control.brake, control.steer) == (0., 1., .25)
        assert governor.required_hold_ticks == 40
        assert ak._active_governor is governor
    assert ak._active_governor is None
    assert ak.base.GoalStopGovernor is ak.ORIGINAL_GOVERNOR
    assert ak.base.install_normal_brake_cap is ak.ORIGINAL_INSTALLER


def test_coast_comparator_never_requests_hold():
    with ak.configuration_scope():
        governor = ak.AKGovernor(ak.AKConfig(ak_arm="coast_27"), 2.5, 20.)
        observation = governor.update(.4, .4, .09, -.4)
        assert observation["phase"] == "holding"
        assert observation["hold_brake_latched"] is False


def test_scope_restores_hooks_on_failure():
    with pytest.raises(RuntimeError):
        with ak.configuration_scope():
            raise RuntimeError("fixture")
    assert ak.base.configuration_from_args is ak.ORIGINAL_CONFIGURATION
    assert ak.base.GoalStopGovernor is ak.ORIGINAL_GOVERNOR
