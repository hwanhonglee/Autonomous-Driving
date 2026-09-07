"""HH_260906 - Validate stationary-only probe admission and command barriers without creating a simulator."""

from pathlib import Path
from types import SimpleNamespace
import copy
import fnmatch
import hashlib
import json
import os
import signal
import sys

import pytest

from scripts.e2e import probe_carla_stationary_camera_quality as probe


ROUTE = Path(__file__).resolve().parents[1] / "docs/assets/validation/2026-09-01/town07/autoware_vad/straight/autoware_vad_route.json"


def test_fixed_probe_parses_without_carla_or_an_output_directory(tmp_path):
    args = probe.parse_args([str(tmp_path / "new"), str(ROUTE)])
    assert args.host == "127.0.0.1" and args.port == 2100 and args.allow_map_load is False
    assert probe.RECORD_TICKS == (100, 120, 140, 160)
    assert not (tmp_path / "new").exists()


@pytest.mark.parametrize("option", [["--host", "remote"], ["--port", "80"], ["--allow-map-load"], ["--target-speed-kmh", "30"], ["--helpful"]])
def test_probe_rejects_remote_world_motion_and_ambiguous_options(tmp_path, option):
    with pytest.raises(SystemExit):
        probe.parse_args([str(tmp_path / "new"), str(ROUTE), *option])
    assert not (tmp_path / "new").exists()


def test_probe_rejects_a_changed_route(tmp_path):
    changed = tmp_path / "route.json"
    changed.write_bytes(ROUTE.read_bytes() + b"\n")
    with pytest.raises(SystemExit):
        probe.parse_args([str(tmp_path / "new"), str(changed)])


@pytest.mark.parametrize("name", ["mapping", "calibration"])
def test_probe_refuses_rig_drift_before_a_world_is_opened(tmp_path, name):
    """HH_260906 - Low/Epic comparisons must retain identical frozen camera geometry and extrinsics."""
    changed = tmp_path / "changed.yaml"
    changed.write_text("not the fixed rig")
    with pytest.raises(SystemExit):
        probe.parse_args([str(tmp_path / "new"), str(ROUTE), f"--{name}", str(changed)])


def test_lock_must_be_an_inherited_descriptor_for_the_exact_workspace_file(monkeypatch, tmp_path):
    expected, other = tmp_path / "expected.lock", tmp_path / "other.lock"
    expected.touch()
    other.touch()
    monkeypatch.delenv("AUTOWARE_E2E_WORKSPACE_RUNTIME_LOCK_FD", raising=False)
    with pytest.raises(probe.capture.CollectionError): probe.require_owner_lock(expected)
    with other.open("a") as stream:
        monkeypatch.setenv("AUTOWARE_E2E_WORKSPACE_RUNTIME_LOCK_FD", str(stream.fileno()))
        with pytest.raises(probe.capture.CollectionError, match="does not belong"): probe.require_owner_lock(expected)
    with expected.open("a") as stream:
        monkeypatch.setenv("AUTOWARE_E2E_WORKSPACE_RUNTIME_LOCK_FD", str(stream.fileno()))
        probe.require_owner_lock(expected)


@pytest.mark.parametrize("count,error,actor", [(1, False, 42), (0, False, 42), (2, False, 42), (1, True, 42), (1, False, 99)])
def test_acknowledgment_is_exact_and_never_sends_throttle_or_a_tick(count, error, actor):
    commands = []
    def batch(items, do_tick):
        commands.append((items, do_tick))
        return [SimpleNamespace(has_error=lambda: error, actor_id=actor)] * count
    carla = SimpleNamespace(VehicleControl=lambda **kwargs: kwargs,
                            command=SimpleNamespace(ApplyVehicleControl=lambda identifier, control: (identifier, control)))
    if count == 1 and not error and actor == 42:
        probe.submit_brake(SimpleNamespace(apply_batch_sync=batch), carla, 42)
    else:
        with pytest.raises(probe.capture.CollectionError): probe.submit_brake(SimpleNamespace(apply_batch_sync=batch), carla, 42)
    assert commands == [([(42, {"throttle": 0.0, "steer": 0.0, "brake": 1.0})], False)]


@pytest.fixture
def fake_world(tmp_path, monkeypatch):
    """HH_260906 - Execute the complete160-tick worker against an in-memory CARLA-shaped world, not a simulator."""
    settings = SimpleNamespace(synchronous_mode=False, fixed_delta_seconds=None, no_rendering_mode=False,
                               substepping=True, max_substep_delta_time=.01, max_substeps=10)
    config = {"failure": None, "tick": 0, "actors": [], "brakes": [], "destroyed": [], "stopped": [], "saved": [],
              "settings": copy.copy(settings), "initial_settings": copy.copy(settings), "weather": "initial-weather",
              "applied_settings": [], "weather_updates": [], "snapshots": 0}
    specs = probe.capture.load_camera_specs(probe.capture.DEFAULT_MAPPING, probe.capture.DEFAULT_CALIBRATION, 2.85)
    class ActorList(list):
        def filter(self, pattern):
            return [actor for actor in self if fnmatch.fnmatch(actor.type_id, pattern)]
    class Blueprint:
        def __init__(self, name): self.id, self.attributes = name, {}
        def set_attribute(self, name, value): self.attributes[name] = value
    def transform(_carla, values):
        return SimpleNamespace(location=SimpleNamespace(**{name: values.get(name, 0.) for name in ("x", "y", "z")}),
            rotation=SimpleNamespace(**{name: values.get(name, 0.) for name in ("roll", "pitch", "yaw")}))
    class Actor:
        def __init__(self, blueprint, pose):
            self.id = 10 + len(config["actors"])
            self.type_id, self.attributes, self.pose, self.callback = blueprint.id, dict(blueprint.attributes), pose, None
            self.control = SimpleNamespace(throttle=0., steer=0., brake=0., gear=0, hand_brake=False, reverse=False, manual_gear_shift=False)
            config["actors"].append(self)
        def listen(self, callback): self.callback = callback
        def stop(self): config["stopped"].append(self.id)
        def destroy(self):
            config["destroyed"].append(self.id)
            if config["failure"] == "destroy_false" and self.type_id.startswith("sensor.") and self.id == 16: return False
            config["actors"].remove(self)
            return True
        def get_transform(self): return self.pose
        def get_velocity(self):
            value = .11 if config["failure"] == "moved" and config["tick"] == 71 else (
                float("nan") if config["failure"] == "nonfinite" and config["tick"] == 10 else 0.)
            return SimpleNamespace(x=value, y=0., z=0.)
        def get_control(self):
            value = copy.copy(self.control)
            if config["failure"] == "reported_throttle" and config["tick"] == 5: value.throttle = .1
            return value
    def apply_settings(value):
        config["applied_settings"].append(copy.copy(value))
        config["settings"] = copy.copy(value)
        if config["failure"] == "wrong_settings" and value.synchronous_mode: config["settings"].fixed_delta_seconds = .1
    def set_weather(value):
        config["weather_updates"].append(value)
        config["weather"] = value
    def tick(timeout):
        assert timeout == 4.
        config["tick"] += 1
        frame = 1000 + config["tick"]
        for index, sensor in enumerate(actor for actor in config["actors"] if actor.type_id.startswith("sensor.")):
            spec = specs[index]
            image = SimpleNamespace(frame=frame, timestamp=config["tick"] * .05, width=spec.width, height=spec.height)
            if config["failure"] == "unsaved_camera_timestamp" and config["tick"] == 80: image.timestamp += .01
            if config["failure"] == "unsaved_camera_geometry" and config["tick"] == 80: image.width += 1
            if config["failure"] == "skipped_camera_frame" and config["tick"] == 80: image.frame += 1
            sensor.callback(image)
        return frame
    def snapshot():
        config["snapshots"] += 1
        frame, timestamp = 1000 + config["tick"], config["tick"] * .05
        if config["failure"] == "bad_cadence" and config["tick"] == 30: timestamp += .05
        return SimpleNamespace(frame=frame, timestamp=SimpleNamespace(elapsed_seconds=timestamp),
                               find=lambda identity: next((actor for actor in config["actors"] if actor.id == identity), None))
    world = SimpleNamespace(get_map=lambda: SimpleNamespace(name="Carla/Maps/Town07"), get_settings=lambda: copy.copy(config["settings"]),
        get_weather=lambda: config["weather"], apply_settings=apply_settings, set_weather=set_weather,
        get_actors=lambda: ActorList(config["actors"]), get_blueprint_library=lambda: SimpleNamespace(find=Blueprint),
        try_spawn_actor=lambda blueprint, pose: Actor(blueprint, pose), spawn_actor=lambda blueprint, pose, **_: Actor(blueprint, pose),
        tick=tick, get_snapshot=snapshot)
    def batch(commands, do_tick):
        assert do_tick is False and len(commands) == 1
        identity, control = commands[0]
        config["brakes"].append(copy.copy(control))
        next(actor for actor in config["actors"] if actor.id == identity).control = copy.copy(control)
        return [SimpleNamespace(actor_id=identity, error="", has_error=lambda: False)]
    client = SimpleNamespace(set_timeout=lambda value: None, get_world=lambda: world, apply_batch_sync=batch)
    carla = SimpleNamespace(Client=lambda *_: client, WeatherParameters=SimpleNamespace(ClearNoon="ClearNoon"),
        AttachmentType=SimpleNamespace(Rigid="Rigid"),
        VehicleControl=lambda **kwargs: SimpleNamespace(gear=0, hand_brake=False, reverse=False, manual_gear_shift=False, **kwargs),
        command=SimpleNamespace(ApplyVehicleControl=lambda identity, control: (identity, control)))
    def save_image(image, path, quality):
        if config["failure"] == "partial_encoding" and len(config["saved"]) == 2:
            raise probe.capture.CollectionError("fake image encoding failed")
        assert quality == 95
        path.write_bytes(f"HH_260906 - Fake integration payload only, frame{image.frame}, file{path.name}".encode())
        config["saved"].append(path)
    monkeypatch.setitem(sys.modules, "carla", carla)
    monkeypatch.setattr(probe, "require_owner_lock", lambda: None)
    monkeypatch.setattr(probe.capture, "_carla_transform", transform)
    monkeypatch.setattr(probe.capture, "_save_jpeg", save_image)
    args = probe.parse_args([str(tmp_path / "probe"), str(ROUTE)])
    return config, args, world


def test_full_fake_world_records_exact160_ticks_and24_images_then_restores(fake_world):
    config, args, world = fake_world
    handlers = {number: signal.getsignal(number) for number in (signal.SIGINT, signal.SIGTERM)}
    result = probe.run(args)
    assert result == args.output and config["tick"] == config["snapshots"] == len(config["brakes"]) == 160
    assert len(config["saved"]) == 24 and config["actors"] == [] and len(config["destroyed"]) == 7 and len(config["stopped"]) == 6
    assert all(command.throttle == command.steer == 0. and command.brake == 1. for command in config["brakes"])
    assert vars(world.get_settings()) == vars(config["initial_settings"]) and config["weather"] == "initial-weather"
    assert all(signal.getsignal(number) == handler for number, handler in handlers.items())
    manifest = json.loads((result / "manifest.json").read_text())
    assert manifest["status"] == "COMPLETE" and manifest["state_count"] == 160 and manifest["cleanup_errors"] == []
    assert manifest["saved_camera_anchor_count"] == 4 and manifest["partial_camera_anchor_count"] == 0
    frames = [json.loads(line) for line in (result / "camera_frames.jsonl").read_text().splitlines()]
    assert [frame["tick"] for frame in frames] == [100, 120, 140, 160]
    for frame in frames:
        assert frame["complete"] is True and set(frame["images"]) == set(probe.capture.MODEL_CAMERA_ORDER)
        for image in frame["images"].values():
            assert hashlib.sha256((result / image["path"]).read_bytes()).hexdigest() == image["sha256"]


@pytest.mark.parametrize("failure, ticks", [("moved", 71), ("nonfinite", 10), ("bad_cadence", 30),
    ("reported_throttle", 5), ("unsaved_camera_timestamp", 80), ("unsaved_camera_geometry", 80),
    ("skipped_camera_frame", 80), ("partial_encoding", 100), ("destroy_false", 160), ("wrong_settings", 0)])
def test_fake_world_failures_preserve_partial_evidence_and_restore_owned_state(fake_world, failure, ticks):
    config, args, world = fake_world
    config["failure"] = failure
    with pytest.raises(probe.capture.CollectionError):
        probe.run(args)
    assert config["tick"] == ticks and not args.output.exists()
    partial = args.output.with_name(args.output.name + ".partial")
    manifest = json.loads((partial / "manifest.json").read_text())
    assert manifest["status"] == "FAIL" and manifest["state_count"] == ticks
    assert vars(world.get_settings()) == vars(config["initial_settings"]) and config["weather"] == "initial-weather"
    if failure == "destroy_false":
        assert manifest["cleanup_errors"] and len(manifest["destroy_rpc_confirmed_actor_ids"]) == 6
    else:
        assert config["actors"] == []
    if failure == "partial_encoding":
        assert len(list((partial / "images").glob("*.jpg"))) == 2
        assert manifest["partial_camera_anchor_count"] == 1 and manifest["saved_camera_anchor_count"] == 0
        entry = json.loads((partial / "camera_frames.jsonl").read_text().splitlines()[0])
        assert entry["complete"] is False and len(entry["images"]) == 2


@pytest.mark.parametrize("kind", ["output", "partial", "dangling_output", "dangling_partial"])
def test_existing_and_dangling_outputs_are_refused_without_simulator_access(tmp_path, monkeypatch, kind):
    output = tmp_path / "out"
    target = output.with_name("out.partial") if "partial" in kind else output
    if "dangling" in kind: target.symlink_to(tmp_path / "absent")
    else: target.mkdir()
    monkeypatch.setattr(probe, "require_owner_lock", lambda: None)
    args = probe.parse_args([str(output), str(ROUTE)])
    with pytest.raises(probe.capture.CollectionError, match="already exists"):
        probe.run(args)
    assert not (tmp_path / "absent").exists()


@pytest.mark.parametrize("actor_type", ["vehicle.other", "sensor.camera.rgb", "walker.pedestrian.0001", "controller.ai.walker"])
def test_foreign_actors_prevent_any_settings_change_or_owned_spawn(fake_world, actor_type):
    config, args, _ = fake_world
    foreign = SimpleNamespace(id=999, type_id=actor_type, attributes={})
    config["actors"].append(foreign)
    with pytest.raises(probe.capture.CollectionError):
        probe.run(args)
    assert config["actors"] == [foreign] and config["applied_settings"] == [] and config["destroyed"] == []
