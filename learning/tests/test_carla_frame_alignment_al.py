"""HH_260906 - Verify AL's fixed frame intervention without starting CARLA or admitting data."""
from __future__ import annotations

from dataclasses import asdict
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts.e2e import collect_carla_frame_alignment_al as al
from scripts.e2e import prepare_carla_common10_dataset as adapter


@pytest.fixture(autouse=True)
def fixed_route(tmp_path, monkeypatch):
    route = tmp_path / "route.json"
    route.write_text('{"town":"Town01"}')
    monkeypatch.setattr(al.ak.aj, "ROUTE_SHA", al.hashlib.sha256(route.read_bytes()).hexdigest())
    return route


def arguments(tmp_path, route):
    return [str(tmp_path / "episode"), str(route), "--al-arm", "rear_axle",
            "--goal-stop-profile", "comfortable_v2", "--target-speed-kmh", "9",
            "--physics-hz", "20", "--capture-hz", "10", "--max-duration-sec", "180",
            "--stationary-warmup-sec", "3.5", "--stationary-tail-sec", "6.5",
            "--goal-tolerance-m", "1", "--agent-initialization", "after_bootstrap",
            "--control-transport", "acknowledged_batch", "--weather", "ClearNoon",
            "--seed", "2026092601", "--spawn-z-offset-m", "0",
            "--basic-agent-base-min-distance-m", "2", "--basic-agent-distance-ratio", ".2",
            "--wheelbase-m", "2.85"]


def test_fixed_ak_b_protocol_stays_development_only(tmp_path, fixed_route):
    args = al.parse_args(arguments(tmp_path, fixed_route))
    config = al.validate_args(args)
    assert args.ak_arm == "coast_27_hold"
    assert config.profile_id == al.PROFILE_ID
    assert config.stop_buffer_m == 2.7
    assert config.normal_throttle_cap == .2 and config.normal_brake_cap == 0
    assert config.launch_only_cap == .13 and config.desired_deceleration_mps2 == .12
    assert config.goal_tolerance_m == 1 and config.hold_seconds == 2
    assert config.training_data_approved is False and config.development_only is True
    assert config.policy_source_sha256 == al.hashlib.sha256(Path(al.__file__).read_bytes()).hexdigest()
    with pytest.raises(adapter.AdapterError):
        adapter._require_native_admission_not_denied({
            "capture_contract": {"goal_stop_profile": asdict(config)}})


@pytest.mark.parametrize("flag,value", [("--target-speed-kmh", "10"),
    ("--wheelbase-m", "2.9"), ("--goal-tolerance-m", "2"), ("--weather", "WetNoon")])
def test_protocol_change_is_rejected_before_simulator(tmp_path, fixed_route, flag, value):
    with pytest.raises((ValueError, SystemExit)):
        al.parse_args(arguments(tmp_path, fixed_route) + [flag, value])


class Location:
    def __init__(self, x=0., y=0., z=0.):
        self.x, self.y, self.z = x, y, z

    def distance(self, other):
        return math.dist((self.x, self.y, self.z), (other.x, other.y, other.z))


class Transform:
    def __init__(self, location, rotation):
        self.location, self.rotation = location, rotation

    def get_forward_vector(self):
        yaw = math.radians(self.rotation.yaw)
        pitch = math.radians(self.rotation.pitch)
        return SimpleNamespace(x=math.cos(pitch) * math.cos(yaw),
                               y=math.cos(pitch) * math.sin(yaw), z=math.sin(pitch))


class Actor:
    def __init__(self, *, yaw=0., pitch=0., speed=.6):
        self.transform = Transform(Location(5., 2., 3.), SimpleNamespace(yaw=yaw, pitch=pitch))
        self.velocity = SimpleNamespace(x=speed, y=0., z=0.)

    def get_transform(self):
        return self.transform

    def get_location(self):
        return self.transform.location

    def get_velocity(self):
        return self.velocity

    def get_speed_limit(self):
        return 30.

    def get_world(self):
        return "unchanged-world"


def test_rear_axle_transform_matches_label_frame_without_mutating_actor():
    actor = Actor(yaw=90., pitch=0.)
    proxy = al.RearAxleControlProxy(actor)
    shift = proxy.get_transform()
    assert shift.location.x == pytest.approx(5.)
    assert shift.location.y == pytest.approx(2. - 1.425)
    assert shift.location.z == pytest.approx(3.)
    assert proxy.get_location().y == pytest.approx(shift.location.y)
    assert actor.get_location().y == 2.
    assert proxy.get_velocity() is actor.velocity
    assert proxy.get_speed_limit() == 30.
    assert proxy.get_world() == "unchanged-world"
    pitched = al.RearAxleControlProxy(Actor(yaw=0., pitch=30.)).get_transform()
    assert pitched.location.x == pytest.approx(5. - 1.425 * math.cos(math.radians(30)))
    assert pitched.location.z == pytest.approx(3. - 1.425 * .5)


def test_nonfinite_actor_transform_rejected():
    actor = Actor()
    actor.transform.location.x = float("nan")
    with pytest.raises(ValueError, match="finite actor transform"):
        al.RearAxleControlProxy(actor).get_location()


def fake_planner(actor, *, raise_during_step=False):
    controller = SimpleNamespace(
        _vehicle=actor, _lat_controller=SimpleNamespace(_vehicle=actor),
        _lon_controller=SimpleNamespace(_vehicle=actor), max_brake=.5, max_throt=.75)
    planner = SimpleNamespace(_vehicle=actor, _vehicle_controller=controller)
    observations = []

    def original_step():
        observations.append({
            "purge_x": planner._vehicle.get_location().x,
            "lateral_x": controller._lat_controller._vehicle.get_transform().location.x,
            "longitudinal_actor_original": controller._lon_controller._vehicle is actor,
            "vehicle_pid_actor_original": controller._vehicle is actor,
        })
        if raise_during_step:
            raise RuntimeError("injected normal planner failure")
        return SimpleNamespace(throttle=.22, brake=0., steer=.3)

    planner.run_step = original_step
    return planner, observations


def test_only_purge_and_lateral_use_proxy_and_emergency_remains_downstream(
        tmp_path, fixed_route, monkeypatch):
    monkeypatch.setattr(al, "verify_upstream_sources", lambda _planner: dict(al.UPSTREAM_SOURCE_SHA256))
    actor = Actor()
    planner, observations = fake_planner(actor)
    args = al.parse_args(arguments(tmp_path, fixed_route))
    config = al.validate_args(args)
    with al.configuration_scope():
        al.ak.AKGovernor(config, 2.5, 20.)
        metadata = al.aligned_installer(planner, 0., .2)
        normal = planner.run_step()
        assert observations[-1] == {
            "purge_x": pytest.approx(5. - 1.425),
            "lateral_x": pytest.approx(5. - 1.425),
            "longitudinal_actor_original": True,
            "vehicle_pid_actor_original": True,
        }
        assert (normal.throttle, normal.brake, normal.steer) == (.2, 0., .3)
        assert planner._vehicle is actor
        assert planner._vehicle_controller._lat_controller._vehicle is actor
        # HH_260906 - BasicAgent's original hazard branch runs after the normal planner call.
        emergency = planner.run_step()
        emergency.throttle = 0.
        emergency.brake = .5
        assert (emergency.throttle, emergency.brake, emergency.steer) == (0., .5, .3)
        assert metadata["basic_agent_emergency_brake_modified"] is False
        assert metadata["emergency_override_remains_after_normal_planner"] is True
        assert metadata["training_data_approved"] is False
    assert al.base.GoalStopGovernor is al.ak.ORIGINAL_GOVERNOR
    assert al.base.install_normal_brake_cap is al.ak.ORIGINAL_INSTALLER


def test_actor_references_restore_after_normal_failure(tmp_path, fixed_route, monkeypatch):
    monkeypatch.setattr(al, "verify_upstream_sources", lambda _planner: dict(al.UPSTREAM_SOURCE_SHA256))
    actor = Actor()
    planner, _ = fake_planner(actor, raise_during_step=True)
    config = al.validate_args(al.parse_args(arguments(tmp_path, fixed_route)))
    with al.configuration_scope():
        al.ak.AKGovernor(config, 2.5, 20.)
        al.aligned_installer(planner, 0., .2)
        with pytest.raises(RuntimeError, match="injected normal planner failure"):
            planner.run_step()
        assert planner._vehicle is actor
        assert planner._vehicle_controller._lat_controller._vehicle is actor
        assert planner._vehicle_controller._lon_controller._vehicle is actor
    assert al.base.configuration_from_args is al.ak.ORIGINAL_CONFIGURATION


def test_unexpected_controller_actor_reference_is_rejected(tmp_path, fixed_route, monkeypatch):
    monkeypatch.setattr(al, "verify_upstream_sources", lambda _planner: dict(al.UPSTREAM_SOURCE_SHA256))
    actor = Actor()
    planner, _ = fake_planner(actor)
    planner._vehicle_controller._lon_controller._vehicle = Actor()
    config = al.validate_args(al.parse_args(arguments(tmp_path, fixed_route)))
    with al.configuration_scope():
        al.ak.AKGovernor(config, 2.5, 20.)
        with pytest.raises(ValueError, match="actor references"):
            al.aligned_installer(planner, 0., .2)


def test_nested_scope_restores_frozen_ak_hooks_even_when_body_fails():
    with pytest.raises(RuntimeError, match="fixture"):
        with al.configuration_scope():
            assert al.base.configuration_from_args is al.validate_args
            assert al.base.GoalStopGovernor is al.ak.AKGovernor
            raise RuntimeError("fixture")
    assert al.base.configuration_from_args is al.ak.ORIGINAL_CONFIGURATION
    assert al.base.GoalStopGovernor is al.ak.ORIGINAL_GOVERNOR
    assert al.base.install_normal_brake_cap is al.ak.ORIGINAL_INSTALLER
