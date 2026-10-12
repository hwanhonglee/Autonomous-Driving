"""HH_260906 - Test the new coast schedule/preflight using the frozen stationary worker's in-memory API fixture."""

import json
import math
from pathlib import Path
import signal
import sys
from types import SimpleNamespace as NS

import pytest

from scripts.e2e import probe_carla_low_speed_coast as probe
from test_probe_carla_stationary_motion import fake_world, ROUTE  # noqa: F401


def args_for(tmp_path, *extra):
    return [str(tmp_path / "coast"), str(ROUTE), *extra]


def test_strict_fixed_arguments_and_source_closure(tmp_path):
    args = probe.parse_args(args_for(tmp_path))
    sources, inputs = probe.fingerprints(args)
    assert args.host == "127.0.0.1" and args.allow_map_load is False and args.port == 2100
    assert len(sources) == 8 and set(sources) == {"scripts/e2e/" + name for name in probe.SOURCE_NAMES}
    assert inputs["route_file"] == probe.ROUTE_SHA
    assert (probe.SETTLE_TICKS, probe.MAX_LAUNCH_TICKS, probe.COAST_TICKS, probe.MAXIMUM_TICKS) == (70, 160, 900, 1130)
    assert not args.output.exists()


@pytest.mark.parametrize("extra", [["--host", "remote"], ["--port", "80"], ["--arm", "service"],
    ["--allow-map-load"], ["--coast-ticks", "10"], ["--target-speed-kmh", "1"], ["--physics-hz", "10"],
    ["--launch-throttle", ".14"], ["--brake-tail"], ["--po", "2100"]])
def test_cli_rejects_unreviewed_controls_timing_and_remote_scope(tmp_path, extra):
    with pytest.raises(SystemExit):
        probe.parse_args(args_for(tmp_path, *extra))


@pytest.mark.parametrize("phase,throttle,brake", [("settle", 0., 1.), ("launch", .13, 0.), ("coast", 0., 0.)])
def test_phase_controls_are_exact_and_have_no_parking_or_manual_gears(phase, throttle, brake):
    control = probe.phase_control(NS(VehicleControl=lambda **kw: NS(**kw)), phase)
    assert vars(control) == dict(throttle=throttle, brake=brake, steer=0., hand_brake=False,
                                 reverse=False, manual_gear_shift=False, gear=0)


def test_first_threshold_crossing_and_full_coast_duration_are_not_retimed():
    make = lambda phase, count, speed: {"phase": phase, "phase_tick": count, "planar_speed_mps": speed}
    assert probe.following_phase(make("settle", 69, 0.)) == "settle"
    assert probe.following_phase(make("settle", 70, 0.)) == "launch"
    assert probe.following_phase(make("launch", 1, .499999)) == "launch"
    assert probe.following_phase(make("launch", 1, .5)) == "coast"
    assert probe.following_phase(make("launch", 160, .5)) == "coast"
    with pytest.raises(probe.capture.CollectionError, match="launch_timeout"):
        probe.following_phase(make("launch", 160, .499999))
    assert probe.following_phase(make("coast", 899, 0.)) == "coast"
    assert probe.following_phase(make("coast", 900, 0.)) == "COMPLETE"


@pytest.mark.parametrize("key,value,expected", [("planar_speed_mps", .751, "maximum_planar_speed_exceeded"),
    ("vx", -.101, "reverse_body_speed_exceeded"), ("net_planar_motion_m", 10.001, "maximum_planar_motion_exceeded"),
    ("route_cte_m", 1.001, "maximum_route_cte_exceeded")])
def test_operational_limits_remain_strict(key, value, expected):
    row = dict(phase="coast", phase_tick=1, planar_speed_mps=.75, vx=-.1, net_planar_motion_m=10., route_cte_m=1.)
    assert probe.safety_reason(row, 0) is None
    row[key] = value
    assert probe.safety_reason(row, 0) == expected
    assert probe.safety_reason(row, 1) == "collision"


@pytest.fixture
def coast_world(fake_world, monkeypatch):
    """HH_260906 - Extend the existing fixture with bounded map/ray RPCs and a deterministic five-tick launch."""
    config, args, world = fake_world
    args = probe.parse_args([str(args.output), str(ROUTE)])
    route = probe.capture.load_route(ROUTE)
    carla = sys.modules["carla"]
    carla.Location = lambda **kw: NS(**kw)
    carla.LaneType = NS(Driving="Driving")
    make_pose = lambda values: probe.capture._carla_transform(carla, values)
    config.update(launch_ticks=0, coast_ticks=0, coast_failure=None, ray_queries=[], tick_frame_offset=0)
    def waypoint(location, project_to_road, lane_type):
        assert project_to_road is False and lane_type == "Driving"
        if config["coast_failure"] == "lane_missing": return None
        point = min(route["route"], key=lambda p: math.hypot(p["x"] - location.x, -p["y"] - location.y))
        pose = make_pose(dict(x=point["x"], y=-point["y"], z=point["z"], roll=0., pitch=0., yaw=-math.degrees(point["yaw"])))
        if config["coast_failure"] == "lane_opposite": pose.rotation.yaw += 180.
        if config["coast_failure"] == "lane_nan": pose.rotation.yaw = float("nan")
        width = float("inf") if config["coast_failure"] == "width_inf" else 4.
        return NS(transform=pose, lane_width=width, road_id=point["road_id"], lane_id=point["lane_id"])
    spawn = make_pose(route["start_carla_transform"])
    world.get_map = lambda: NS(name="Carla/Maps/C_track_1_0_7", get_spawn_points=lambda: [spawn] * 72, get_waypoint=waypoint)
    def ray(start, end):
        assert config["tick"] == 0 and not config["actors"] and config["settings"].synchronous_mode
        config["ray_queries"].append((start, end))
        if config["coast_failure"] == "ray_error": raise RuntimeError("fixture ray error")
        if config["coast_failure"] == "ray_hit": return [NS(location=NS(x=start.x+1., y=start.y, z=start.z), label="fixture.Barrier")]
        if config["coast_failure"] == "ray_nan": return [NS(location=NS(x=float("nan"), y=start.y, z=start.z), label="fixture.Invalid")]
        if config["coast_failure"] == "frame_advanced" and len(config["ray_queries"]) == 6: config["tick"] += 1
        return []
    world.cast_ray = ray
    old_spawn, old_tick = world.try_spawn_actor, world.tick
    def spawn_actor(blueprint, transform):
        ego = old_spawn(blueprint, transform)
        original_velocity = ego.get_velocity
        def velocity():
            if config["tick"] <= 70: return original_velocity()
            if ego.control.throttle == .13:
                speed = min(.5, config["launch_ticks"] * .1)
                if config["coast_failure"] == "launch_timeout": speed = .1
                if config["coast_failure"] == "max_launch": speed = .5 if config["launch_ticks"] == 160 else .1
                if config["coast_failure"] == "overspeed": speed = .76
            else:
                speed = 0.
            yaw = math.radians(route["start_carla_transform"]["yaw"])
            return NS(x=speed*math.cos(yaw), y=speed*math.sin(yaw), z=0.)
        ego.get_velocity = velocity
        return ego
    def tick(timeout):
        control = config["controls"][-1]
        if control.throttle == .13: config["launch_ticks"] += 1
        elif control.brake == 0.: config["coast_ticks"] += 1
        return old_tick(timeout)
    world.try_spawn_actor, world.tick = spawn_actor, tick
    return config, args, world


def load_result(args, partial=False):
    output = args.output.with_name(args.output.name + ".partial") if partial else args.output
    load_rows = lambda name: [json.loads(line) for line in (output / name).read_text().splitlines()]
    return json.loads((output / "manifest.json").read_text()), load_rows("native_states.jsonl"), load_rows("control_receipts.jsonl"), load_rows("camera_frames.jsonl")


@pytest.mark.parametrize("mode,launch_count", [(None, 5), ("max_launch", 160)])
def test_full_mock_run_keeps900_coast_ticks_even_with_zero_coast_speed(coast_world, mode, launch_count):
    config, args, world = coast_world
    config["coast_failure"] = mode
    old_handlers = {number: signal.getsignal(number) for number in (signal.SIGINT, signal.SIGTERM)}
    probe.run(args)
    manifest, states, receipts, cameras = load_result(args)
    total = 70 + launch_count + 900
    assert manifest["status"] == "COMPLETE" and len(states) == len(receipts) == config["tick"] == total
    assert manifest["phase_counts"] == dict(settle=70, launch=launch_count, coast=900)
    assert len(cameras) == total//2 and len(config["saved"]) == total//2*6
    assert [row["tick"] for row in cameras] == list(range(2, total+1, 2))
    assert states[69]["phase"] == "settle" and states[70]["phase"] == "launch"
    assert states[69+launch_count]["phase"] == "launch" and states[70+launch_count]["phase"] == "coast"
    assert states[69+launch_count]["planar_speed_mps"] >= .5
    assert all(row["planar_speed_mps"] < .5 for row in states[70:69+launch_count])
    assert all(row["phase_tick"] == index+1 for index,row in enumerate(states[70+launch_count:]))
    assert all(row["requested_control"]["brake"] == row["requested_control"]["throttle"] == 0. for row in states[70+launch_count:])
    assert [(t["from_phase"], t["to_phase"], t["effective_tick"]) for t in manifest["phase_transitions"]] == [
        ("settle", "launch", 71), ("launch", "coast", 71+launch_count)]
    assert manifest["launch_handoff"] == manifest["phase_transitions"][-1]
    assert manifest["preflight"]["status"] == "PASS" and len(config["ray_queries"]) == 6
    assert manifest["preflight"]["frame_before"] == manifest["preflight"]["frame_after"]
    assert manifest["source_sha256_before"] == manifest["source_sha256_after"]
    assert not config["actors"] and vars(world.get_settings()) == vars(config["initial"])
    assert config["weather"] is config["original_weather"] and manifest["cleanup_errors"] == []
    assert manifest["weather_requested"] == manifest["weather_actual"]
    assert all(signal.getsignal(number) == handler for number,handler in old_handlers.items())


@pytest.mark.parametrize("mode,ticks", [("launch_timeout", 230), ("overspeed", 71),
    ("lane_missing", 0), ("lane_opposite", 0), ("lane_nan", 0), ("width_inf", 0),
    ("ray_hit", 0), ("ray_error", 0), ("ray_nan", 0)])
def test_new_failures_retain_partial_and_restore_without_tail_brake(coast_world, mode, ticks):
    config, args, world = coast_world
    config["coast_failure"] = mode
    with pytest.raises((probe.capture.CollectionError, RuntimeError)):
        probe.run(args)
    manifest, states, receipts, cameras = load_result(args, True)
    assert manifest["status"] == "FAIL" and len(states) == config["tick"] == ticks
    assert not args.output.exists() and not config["actors"]
    assert vars(world.get_settings()) == vars(config["initial"]) and config["weather"] is config["original_weather"]
    if ticks:
        assert states[-1]["measurement_complete"] and states[-1]["phase"] == "launch"
        assert states[-1]["requested_control"]["throttle"] == .13 and config["coast_ticks"] == 0
    else:
        assert manifest["preflight"]["status"] == "FAIL" and manifest["world_settings_before"] == manifest["world_settings_restored"]
        assert manifest["weather_before"] == manifest["weather_restored"]
    if mode == "ray_hit":
        assert manifest["preflight"]["obstacle_rays"][0]["hits"][0]["label"] == "fixture.Barrier"
    if mode == "ray_error":
        assert manifest["preflight"]["obstacle_rays"][0]["status"] == "PENDING"
    if mode in ("lane_missing", "lane_opposite", "lane_nan", "width_inf"):
        assert len(manifest["preflight"]["corridor_samples"]) == 1
        assert manifest["preflight"]["corridor_samples"][0]["status"] == "PENDING"
    if mode == "lane_nan":
        assert manifest["preflight"]["corridor_samples"][0]["waypoint_transform_carla"]["yaw"] == {"nonfinite_numeric": "nan"}
    if mode == "width_inf":
        assert manifest["preflight"]["corridor_samples"][0]["lane_width_m"] == {"nonfinite_numeric": "inf"}
    if mode == "ray_nan":
        assert manifest["preflight"]["obstacle_rays"][0]["hits"][0]["location_carla"][0] == {"nonfinite_numeric": "nan"}
    json.loads((args.output.with_name(args.output.name + ".partial") / "manifest.json").read_text(),
               parse_constant=lambda token: pytest.fail(f"nonstandard JSON token {token}"))


@pytest.mark.parametrize("failure,ticks", [("speed", 70), ("travel", 70), ("nonfinite", 10), ("control", 8),
    ("cadence", 10), ("camera_nan", 9), ("camera_frame", 9), ("collision", 10), ("signal", 8),
    ("encoding", 2), ("settings", 0), ("extra_settings", 0), ("ack", 7)])
def test_existing_protocol_and_settle_failures_remain_closed(coast_world, failure, ticks):
    config, args, world = coast_world
    config["failure"] = failure
    with pytest.raises(probe.capture.CollectionError):
        probe.run(args)
    manifest, states, receipts, _cameras = load_result(args, True)
    assert manifest["status"] == "FAIL" and len(states) == config["tick"] == ticks
    assert not config["actors"] and vars(world.get_settings()) == vars(config["initial"])


def test_spawn_nan_is_retained_as_explicit_failure_before_any_actor(coast_world):
    config, args, world = coast_world
    world.get_map().get_spawn_points()[71].location.x = float("nan")
    with pytest.raises(probe.capture.CollectionError, match="live spawn"):
        probe.run(args)
    manifest, states, receipts, _cameras = load_result(args, True)
    assert manifest["preflight"]["live_spawn_transform_carla"]["x"] == {"nonfinite_numeric": "nan"}
    assert manifest["status"] == "FAIL" and states == receipts == [] and config["actors"] == []
    json.loads((args.output.with_name(args.output.name + ".partial") / "manifest.json").read_text(),
               parse_constant=lambda token: pytest.fail(f"nonstandard JSON token {token}"))


def test_unexpected_world_advance_during_preflight_is_not_hidden(coast_world):
    config, args, world = coast_world
    config["coast_failure"] = "frame_advanced"
    with pytest.raises(probe.capture.CollectionError):
        probe.run(args)
    manifest, states, receipts, _cameras = load_result(args, True)
    assert manifest["status"] == "FAIL" and not states and not receipts and not config["actors"]
    assert manifest["preflight"]["frame_before"] != manifest["preflight"]["frame_after"]


def test_new_worker_source_tamper_denies_completion(coast_world, monkeypatch):
    config, args, _world = coast_world
    original, calls = probe.fingerprints, []
    def changed(arguments):
        sources, inputs = original(arguments)
        calls.append(True)
        if len(calls) == 2: sources["scripts/e2e/probe_carla_low_speed_coast.py"] = "0" * 64
        return sources, inputs
    monkeypatch.setattr(probe, "fingerprints", changed)
    with pytest.raises(probe.capture.CollectionError, match="final checks"):
        probe.run(args)
    manifest, _states, _receipts, _cameras = load_result(args, True)
    assert manifest["status"] == "FAIL" and not config["actors"]
    assert any("source or input bytes changed" in error for error in manifest["cleanup_errors"])
