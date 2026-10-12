# HH_260906 - Locate the repository independently of the relocated test directory.
"""HH_260906 - Test the bounded stationary worker with an in-memory API, never a CARLA server."""

import copy
import fnmatch
import hashlib
import json
from pathlib import Path
import signal
import sys
from types import SimpleNamespace as NS

import pytest

from scripts.e2e import probe_carla_stationary_motion as probe


ROUTE = next(p for p in Path(__file__).resolve().parents if (p / ".git").exists()) / "docs/assets/validation/2026-09-08/portable_e2e_learning_cycle_v1/18_c_track_low_speed_turn/original_c_track_left_route.json"


def arguments(tmp_path, *extra):
    return [str(tmp_path / "probe"), str(ROUTE), "--arm", "service", *extra]


def test_fixed_arguments_and_complete_local_sources(tmp_path):
    args = probe.parse_args(arguments(tmp_path))
    sources, inputs = probe.fingerprints(args)
    assert args.host == "127.0.0.1" and args.port == 2100 and args.allow_map_load is False
    assert probe.TOTAL_TICKS == 470 and probe.SETTLE_TICKS == 70 and probe.HOLD_TICKS == 400
    assert len(sources) == 7 and all(key.startswith("scripts/e2e/") for key in sources)
    assert all(len(value) == 64 for value in sources.values()) and inputs["route_file"] == probe.ROUTE_SHA
    assert not args.output.exists()


@pytest.mark.parametrize("extra", [["--host", "remote"], ["--port", "80"], ["--port", "65534"],
    ["--arm", "drive"], ["--allow-map-load"], ["--physics-hz", "10"], ["--physics-substep-profile", "fine_5ms"],
    ["--goal-stop-profile", "turn_launch_013_v1"], ["--target-speed-kmh", "1"], ["--ar", "parking"]])
def test_strict_args_reject_motion_and_setting_changes(tmp_path, extra):
    with pytest.raises(SystemExit):
        probe.parse_args(arguments(tmp_path, *extra))


@pytest.mark.parametrize("name", ["route_file", "mapping", "calibration"])
def test_fixed_inputs_are_rechecked_for_programmatic_run(tmp_path, name):
    args = probe.parse_args(arguments(tmp_path))
    changed = tmp_path / "changed"
    changed.write_bytes(b"HH_260906 - changed fixture")
    setattr(args, name, changed)
    with pytest.raises(probe.capture.CollectionError, match="unchanged fixed"):
        probe.fingerprints(args)


@pytest.mark.parametrize("kind", ["output", "partial", "output_link", "partial_link", "parent_link"])
def test_create_only_paths_fail_without_simulator(tmp_path, kind):
    output = tmp_path / "probe"
    if kind == "parent_link":
        (tmp_path / "redirect").symlink_to(tmp_path, target_is_directory=True)
        output = tmp_path / "redirect" / "probe"
    else:
        path = output if kind.startswith("output") else tmp_path / "probe.partial"
        path.symlink_to(tmp_path / "missing") if kind.endswith("link") else path.mkdir()
    with pytest.raises(probe.capture.CollectionError, match="fresh"):
        probe.fresh_paths(output)


def settings():
    return NS(synchronous_mode=True, fixed_delta_seconds=.05, no_rendering_mode=False,
              substepping=True, max_substep_delta_time=.01, max_substeps=10, deterministic_ragdolls=True,
              spectator_as_ego=True, max_culling_distance=0., tile_stream_distance=3000., actor_active_distance=2000.)


@pytest.mark.parametrize("key,value", [("synchronous_mode", False), ("synchronous_mode", 1),
    ("fixed_delta_seconds", .1), ("fixed_delta_seconds", float("nan")), ("fixed_delta_seconds", True),
    ("no_rendering_mode", True), ("substepping", False), ("max_substep_delta_time", .005),
    ("max_substep_delta_time", float("inf")), ("max_substeps", 20), ("max_substeps", 10.)])
def test_reference_readback_is_exact(key, value):
    actual = settings()
    probe.validate_settings(actual)
    setattr(actual, key, value)
    with pytest.raises(probe.capture.CollectionError, match="reference10ms"):
        probe.validate_settings(actual)


def test_controls_differ_only_in_handbrake():
    carla = NS(VehicleControl=lambda **kwargs: NS(**kwargs))
    service, parking = (vars(probe.fixed_control(carla, arm)) for arm in ("service", "parking"))
    assert [key for key in service if service[key] != parking[key]] == ["hand_brake"]
    assert service == dict(throttle=0., steer=0., brake=1., hand_brake=False, reverse=False, manual_gear_shift=False, gear=0)
    with pytest.raises(probe.capture.CollectionError):
        probe.fixed_control(carla, "other")


class Weather:
    """HH_260906 - Match API scalar fields plus inherited recursive preset objects, not an unrealistic string."""
    def __init__(self, level=0.):
        for name in probe.WEATHER_FIELDS:
            setattr(self, name, level)


Weather.ClearNoon = Weather(1.)
Weather.WetNoon = Weather(2.)


def test_weather_scalar_snapshot_never_recurses_into_class_presets():
    original = Weather()
    assert original.ClearNoon.ClearNoon is Weather.ClearNoon
    with pytest.raises(probe.calibration.CalibrationError, match="nesting depth"):
        probe.calibration.serialize_physics_value(original)
    assert probe.weather_snapshot(original) == dict.fromkeys(probe.WEATHER_FIELDS, 0.)
    assert probe.weather_snapshot(Weather.ClearNoon) == dict.fromkeys(probe.WEATHER_FIELDS, 1.)


@pytest.mark.parametrize("value", [None, True, False, float("nan"), float("inf"), "0", Weather()])
def test_weather_snapshot_rejects_invalid_scalar_fields(value):
    weather = Weather()
    weather.fog_density = value
    with pytest.raises(probe.capture.CollectionError, match="fog_density.*finite numeric"):
        probe.weather_snapshot(weather)


def test_weather_snapshot_allows_actual_negative_sun_angle_but_not_missing_field():
    weather = Weather()
    weather.sun_altitude_angle = -10.
    assert probe.weather_snapshot(weather)["sun_altitude_angle"] == -10.
    del weather.wind_intensity
    with pytest.raises(probe.capture.CollectionError, match="wind_intensity"):
        probe.weather_snapshot(weather)


@pytest.fixture
def fake_world(tmp_path, monkeypatch):
    """HH_260906 - The fake performs real worker ACK journaling and fixture image writes with no server dependency."""
    initial = settings()
    initial.synchronous_mode, initial.fixed_delta_seconds = False, None
    original_weather = Weather()
    config = dict(tick=0, actors=[], saved=[], controls=[], destroyed=[], stopped=[], failure=None,
                  settings=copy.copy(initial), initial=initial, weather=original_weather,
                  original_weather=original_weather, ack_fsyncs=0, fsyncs=0)
    specs = probe.capture.load_camera_specs(probe.capture.DEFAULT_MAPPING, probe.capture.DEFAULT_CALIBRATION, 2.85)
    def pose(values):
        return NS(location=NS(**{key: values.get(key, 0.) for key in ("x", "y", "z")}),
                  rotation=NS(**{key: values.get(key, 0.) for key in ("roll", "pitch", "yaw")}))
    class Actors(list):
        def filter(self, pattern): return [actor for actor in self if fnmatch.fnmatch(actor.type_id, pattern)]
    class Blueprint:
        def __init__(self, name): self.id, self.attributes = name, {}
        def set_attribute(self, name, value): self.attributes[name] = value
        def has_attribute(self, name): return name == "color"
        def get_attribute(self, name): return NS(recommended_values=["fixed-blue"])
    class Actor:
        def __init__(self, blueprint, transform):
            self.id, self.type_id = 10 + len(config["actors"]), blueprint.id
            self.pose, self.attributes, self.callback = transform, blueprint.attributes, None
            self.control = probe.fixed_control(NS(VehicleControl=lambda **kw: NS(**kw)), "service")
            config["actors"].append(self)
        def listen(self, callback): self.callback = callback
        def stop(self): config["stopped"].append(self.id)
        def destroy(self):
            config["destroyed"].append(self.id)
            config["actors"].remove(self)
            return not (config["failure"] == "destroy" and self.id == 17)
        def get_transform(self):
            value = copy.deepcopy(self.pose)
            if config["failure"] == "pre_tick_zero" and config["tick"] == 0:
                return pose({})
            if config["failure"] == "travel" and config["tick"] >= 70: value.location.x += .51
            return value
        def get_velocity(self):
            x = .11 if config["failure"] == "speed" and config["tick"] >= 70 else 0.
            if config["failure"] == "nonfinite" and config["tick"] == 10: x = float("nan")
            return NS(x=x, y=0., z=0.)
        def get_acceleration(self): return NS(x=0., y=0., z=0.)
        def get_angular_velocity(self): return NS(x=0., y=0., z=0.)
        def get_control(self):
            value = copy.copy(self.control)
            if config["failure"] == "control" and config["tick"] == 8: value.hand_brake = not value.hand_brake
            return value
        def get_wheel_steer_angle(self, which): return 0.
        def get_physics_control(self): return {"fixture": True}
    def snapshot():
        timestamp = config["tick"] * .05
        if config["failure"] == "cadence" and config["tick"] == 10: timestamp += .01
        return NS(frame=1000 + config["tick"], timestamp=NS(elapsed_seconds=timestamp),
                  find=lambda identity: next((actor for actor in config["actors"] if actor.id == identity), None))
    def tick(timeout):
        assert timeout == 4. and config["ack_fsyncs"] == config["tick"] + 1
        config["tick"] += 1
        frame = 1000 + config["tick"]
        if config["failure"] == "signal" and config["tick"] == 8:
            signal.getsignal(signal.SIGTERM)(signal.SIGTERM, None)
        for index, sensor in enumerate(actor for actor in config["actors"] if actor.type_id == "sensor.camera.rgb"):
            spec = specs[index]
            image = NS(frame=frame, timestamp=config["tick"] * .05, width=spec.width, height=spec.height)
            if config["failure"] == "camera_nan" and config["tick"] == 9: image.timestamp = float("nan")
            if config["failure"] == "camera_frame" and config["tick"] == 9: image.frame += 1
            sensor.callback(image)
        if config["failure"] == "collision" and config["tick"] == 10:
            next(actor for actor in config["actors"] if actor.type_id == "sensor.other.collision").callback(
                NS(frame=frame, timestamp=config["tick"] * .05, other_actor=NS(id=99, type_id="static.fixture", attributes={}),
                   normal_impulse=NS(x=1., y=0., z=0.)))
        return frame
    def apply_settings(value):
        config["settings"] = copy.copy(value)
        if config["failure"] == "settings" and value.synchronous_mode: config["settings"].max_substep_delta_time = .005
        if config["failure"] == "extra_settings" and value.synchronous_mode: config["settings"].actor_active_distance += 1.
        if config["failure"] == "restore_settings" and not value.synchronous_mode: config["settings"].spectator_as_ego = False
    world = NS(get_map=lambda: NS(name="Carla/Maps/C_track_1_0_7"),
        get_settings=lambda: copy.copy(config["settings"]), apply_settings=apply_settings,
        get_weather=lambda: config["weather"], set_weather=lambda value: config.update(weather=value),
        get_actors=lambda: Actors(config["actors"]), get_blueprint_library=lambda: NS(find=Blueprint),
        try_spawn_actor=lambda blueprint, transform: Actor(blueprint, transform),
        spawn_actor=lambda blueprint, transform, **_: Actor(blueprint, transform), get_snapshot=snapshot, tick=tick)
    def batch(commands, do_tick):
        assert do_tick is False and len(commands) == 1
        identity, control = commands[0]
        config["controls"].append(copy.copy(control))
        next(actor for actor in config["actors"] if actor.id == identity).control = copy.copy(control)
        error = "fixture ACK failure" if config["failure"] == "ack" and config["tick"] == 7 else ""
        return [NS(actor_id=identity, error=error, has_error=lambda: bool(error))]
    client = NS(set_timeout=lambda _: None, get_world=lambda: world, apply_batch_sync=batch,
                get_client_version=lambda: "fixture", get_server_version=lambda: "fixture")
    carla = NS(Client=lambda *_: client, WeatherParameters=Weather, AttachmentType=NS(Rigid="Rigid"),
        VehicleControl=lambda **kw: NS(**kw), VehicleWheelLocation=NS(FL_Wheel=0, FR_Wheel=1), Transform=lambda: pose({}),
        command=NS(ApplyVehicleControl=lambda identity, control: (identity, control)))
    def save_image(image, path, quality):
        if config["failure"] == "encoding" and len(config["saved"]) == 2:
            raise probe.capture.CollectionError("fixture encoding failure")
        assert quality == 95
        path.write_bytes(f"fixture {image.frame} {path.name}".encode())
        config["saved"].append(path)
    def fsync(_descriptor):
        config["fsyncs"] += 1
        config["ack_fsyncs"] = len(config["controls"])
    monkeypatch.setitem(sys.modules, "carla", carla)
    monkeypatch.setattr(probe.calibration, "require_inherited_workspace_lock", lambda: {"verified_exclusive": True})
    monkeypatch.setattr(probe.calibration, "physics_snapshot", lambda value: {"schema": "fixture", "values": value})
    monkeypatch.setattr(probe.capture, "_carla_transform", lambda _carla, values: pose(values))
    monkeypatch.setattr(probe.capture, "_save_jpeg", save_image)
    monkeypatch.setattr(probe.os, "fsync", fsync)
    return config, probe.parse_args(arguments(tmp_path)), world


@pytest.mark.parametrize("arm", ["service", "parking"])
def test_complete_mock_probe_preserves_all_ticks_cameras_ack_and_restores(fake_world, arm):
    config, args, world = fake_world
    args.arm = arm
    old_handlers = {number: signal.getsignal(number) for number in (signal.SIGINT, signal.SIGTERM)}
    output = probe.run(args)
    manifest = json.loads((output / "manifest.json").read_text())
    states = [json.loads(line) for line in (output / "native_states.jsonl").read_text().splitlines()]
    frames = [json.loads(line) for line in (output / "camera_frames.jsonl").read_text().splitlines()]
    receipts = [json.loads(line) for line in (output / "control_receipts.jsonl").read_text().splitlines()]
    assert len(states) == config["tick"] == len(receipts) == config["fsyncs"] == 470
    assert len(frames) == manifest["saved_camera_anchor_count"] == 235 and len(config["saved"]) == 1410
    assert [row["tick"] for row in frames] == list(range(2, 471, 2))
    assert sum(row["phase"] == "settle" for row in states) == 70 and sum(row["phase"] == "hold" for row in states) == 400
    assert all(row["measurement_complete"] and row["control_alignment"]["status"] == "PASS" for row in states)
    assert all(row["before_frame"] == row["after_ack_frame"] == 999 + row["sequence"] for row in receipts)
    assert all(control.hand_brake is (arm == "parking") and control.brake == 1. and control.throttle == control.steer == 0.
               for control in config["controls"])
    assert manifest["status"] == "COMPLETE" and manifest["source_sha256_before"] == manifest["source_sha256_after"]
    assert manifest["input_sha256_before"] == manifest["input_sha256_after"] and not manifest["training_data_approved"]
    assert manifest["vehicle_approved"] is False and set(manifest["physics_snapshots"]) == {"spawn", "settled"}
    assert manifest["vehicle_control_approved"] is False
    assert len(manifest["world_settings"]) == 11 and manifest["world_settings_before"] == manifest["world_settings_restored"]
    assert not config["actors"] and len(config["destroyed"]) == 8 and len(config["stopped"]) == 7
    assert vars(world.get_settings()) == vars(config["initial"]) and config["weather"] is config["original_weather"]
    assert all(signal.getsignal(number) == handler for number, handler in old_handlers.items())
    for name, expected in manifest["artifact_sha256"].items():
        assert hashlib.sha256((output / name).read_bytes()).hexdigest() == expected
    observations = [json.loads(line) for line in (output / "native_observations.jsonl").read_text().splitlines()]
    assert len(observations) == 470 and all(row["stage"] == "OBSERVED_BEFORE_CAMERA" and not row["measurement_complete"] for row in observations)


@pytest.mark.parametrize("failure,ticks", [("speed", 70), ("travel", 70), ("nonfinite", 10), ("control", 8),
    ("cadence", 10), ("camera_nan", 9), ("camera_frame", 9), ("collision", 10), ("signal", 8),
    ("encoding", 2), ("destroy", 470), ("settings", 0), ("extra_settings", 0), ("ack", 7)])
def test_failure_retains_offending_and_partial_data_with_owned_cleanup(fake_world, failure, ticks):
    config, args, world = fake_world
    config["failure"] = failure
    with pytest.raises(probe.capture.CollectionError):
        probe.run(args)
    partial = args.output.with_name(args.output.name + ".partial")
    manifest = json.loads((partial / "manifest.json").read_text())
    states = [json.loads(line) for line in (partial / "native_states.jsonl").read_text().splitlines()]
    assert manifest["status"] == "FAIL" and manifest["state_count"] == len(states) == ticks == config["tick"]
    assert not args.output.exists() and not config["actors"]
    assert vars(world.get_settings()) == vars(config["initial"]) and config["weather"] is config["original_weather"]
    if failure == "encoding":
        assert manifest["partial_camera_anchor_count"] == 1 and len(config["saved"]) == 2
        assert len((partial / "native_observations.jsonl").read_text().splitlines()) == 2
    if failure in ("speed", "travel"):
        assert states[-1]["tick"] == 70 and manifest["saved_camera_anchor_count"] == 35
    if failure == "ack":
        receipts = [json.loads(line) for line in (partial / "control_receipts.jsonl").read_text().splitlines()]
        assert len(receipts) == 8 and receipts[-1]["status"] == "FAILED" and config["fsyncs"] == 8


def test_restoration_readback_failure_denies_completion(fake_world):
    config, args, _world = fake_world
    config["failure"] = "restore_settings"
    with pytest.raises(probe.capture.CollectionError, match="final checks"):
        probe.run(args)
    manifest = json.loads((args.output.with_name(args.output.name + ".partial") / "manifest.json").read_text())
    assert manifest["status"] == "FAIL" and any("restoration readback" in error for error in manifest["cleanup_errors"])
    assert not config["actors"]


def test_journal_close_failure_never_prevents_other_cleanup():
    errors, closed = [], []
    def broken(): raise OSError("fixture close failure")
    probe.close_journals((("bad", NS(close=broken)), ("good", NS(close=lambda: closed.append(True)))), errors)
    assert errors == ["close bad: fixture close failure"] and closed == [True]


def test_source_tamper_denies_completion_after_owned_cleanup(fake_world, monkeypatch):
    config, args, _world = fake_world
    original, calls = probe.fingerprints, []
    def changed(arguments):
        sources, inputs = original(arguments)
        calls.append(True)
        if len(calls) == 2:
            sources["scripts/e2e/probe_carla_stationary_motion.py"] = "0" * 64
        return sources, inputs
    monkeypatch.setattr(probe, "fingerprints", changed)
    with pytest.raises(probe.capture.CollectionError, match="final checks"):
        probe.run(args)
    manifest = json.loads((args.output.with_name(args.output.name + ".partial") / "manifest.json").read_text())
    assert not config["actors"] and manifest["status"] == "FAIL"
    assert any("bytes changed" in error for error in manifest["cleanup_errors"])


def test_uninitialized_pre_tick_api_pose_is_retained_but_never_motion_origin(fake_world):
    config, args, _world = fake_world
    config["failure"] = "pre_tick_zero"
    output = probe.run(args)
    manifest = json.loads((output / "manifest.json").read_text())
    states = [json.loads(line) for line in (output / "native_states.jsonl").read_text().splitlines()]
    assert manifest["status"] == "COMPLETE" and len(states) == 470
    assert set(manifest["physics_snapshots"]["spawn"]["actor_transform_carla"].values()) == {0.}
    assert manifest["physics_snapshots"]["spawn"]["used_as_motion_origin"] is False
    assert manifest["motion_origin_source"] == "REQUESTED_SPAWN_TRANSFORM_NOT_PRETICK_ACTOR_API"
    assert manifest["motion_origin_transform_carla"] == manifest["requested_spawn_transform_carla"]
    assert abs(manifest["motion_origin_transform_carla"]["x"]) > 100.
    assert all(row["net_planar_motion_m"] == 0. for row in states)
    assert manifest["physics_snapshots"]["settled"]["actor_transform_carla"] == manifest["requested_spawn_transform_carla"]


@pytest.mark.parametrize("kind", ["vehicle.toyota.prius", "sensor.camera.rgb", "walker.pedestrian.0001", "controller.ai.walker"])
def test_foreign_actors_prevent_all_mutations(fake_world, kind):
    config, args, _world = fake_world
    foreign = NS(id=77, type_id=kind, attributes={})
    config["actors"].append(foreign)
    with pytest.raises(probe.capture.CollectionError):
        probe.run(args)
    assert config["actors"] == [foreign] and config["tick"] == 0 and not args.output.exists()
    assert not args.output.with_name(args.output.name + ".partial").exists()
