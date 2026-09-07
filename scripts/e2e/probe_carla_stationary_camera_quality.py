#!/usr/bin/env python3
"""HH_260906 - Compare real stationary camera appearance in an owned Town07 world without driving or dataset export."""

from __future__ import annotations

import argparse
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import signal

if __package__:
    from . import collect_carla_vad_expert as capture
else:
    import collect_carla_vad_expert as capture


ROUTE_SHA = "8285e70a790d5e8ae75803db1aa538122b56e6fcfc3586e60eaa947d330417c9"
MAPPING_SHA = "9aaff2befed7ad12376b2e04bbdd51bd1808a3bafe39d87a6f6b241dbcca3136"
CALIBRATION_SHA = "5022cd1de5b48e9c824b6f2f8c59991fa665eeaf7a7bafd084c88acdb65e4bea"
RECORD_TICKS = (100, 120, 140, 160)


def parse_args(argv=None):
    """HH_260906 - Restrict the diagnostic before opening a client or creating artifacts."""
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("output", type=Path)
    parser.add_argument("route_file", type=Path)
    parser.add_argument("--host", default="127.0.0.1", choices=("127.0.0.1",))
    parser.add_argument("--port", type=int, default=2100)
    parser.add_argument("--mapping", type=Path, default=capture.DEFAULT_MAPPING)
    parser.add_argument("--calibration", type=Path, default=capture.DEFAULT_CALIBRATION)
    args = parser.parse_args(argv)
    args.allow_map_load = False
    if not 1024 <= args.port <= 65533:
        parser.error("port must be in [1024,65533]")
    if not args.route_file.is_file() or hashlib.sha256(args.route_file.read_bytes()).hexdigest() != ROUTE_SHA:
        parser.error("stationary visual audit requires the fixed Town07 route")
    for name, path, expected in (("mapping", args.mapping, MAPPING_SHA), ("calibration", args.calibration, CALIBRATION_SHA)):
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            parser.error(f"stationary visual audit requires the fixed six-camera {name}")
    return args


def require_owner_lock(expected_path=None):
    """HH_260906 - Require the inherited workspace lock instead of attaching to an arbitrary simulator."""
    value = os.environ.get("AUTOWARE_E2E_WORKSPACE_RUNTIME_LOCK_FD", "")
    if not value.isdigit():
        raise capture.CollectionError("owned workspace lock descriptor is required")
    descriptor = int(value)
    expected = expected_path or Path(__file__).resolve().parents[2] / "data/locks/autoware_e2e_runtime.lock"
    if Path(os.readlink(f"/proc/self/fd/{descriptor}")).resolve(strict=True) != expected.resolve(strict=True):
        raise capture.CollectionError("lock descriptor does not belong to this workspace")
    fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)


def submit_brake(client, carla, actor_id):
    """HH_260906 - Acknowledge one fixed full-brake request without ticking or any throttle command."""
    brake = carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0)
    responses = client.apply_batch_sync([carla.command.ApplyVehicleControl(actor_id, brake)], False)
    if len(responses) != 1 or responses[0].has_error() or responses[0].actor_id != actor_id:
        raise capture.CollectionError("stationary brake command was not acknowledged for the owned vehicle")


def exclusive_world(world):
    """HH_260906 - Existing sensors, walkers and controllers also make the world foreign to this probe."""
    capture._preflight_exclusive(world)
    for pattern in ("walker.pedestrian.*", "controller.ai.walker"):
        if list(world.get_actors().filter(pattern)):
            raise capture.CollectionError("stationary audit refuses foreign pedestrian actors")


def validate_settings(settings):
    """HH_260906 - Verify applied rendering and20Hz substep capacity, not merely requested settings."""
    delta = settings.fixed_delta_seconds
    capacity = settings.max_substep_delta_time * settings.max_substeps
    if (not settings.synchronous_mode or delta is None or not math.isfinite(delta) or abs(delta - .05) > 1e-9
            or settings.no_rendering_mode or not settings.substepping or not math.isfinite(capacity)
            or settings.max_substep_delta_time <= 0 or settings.max_substeps < 1 or capacity < .05):
        raise capture.CollectionError("actual stationary probe settings are not rendered synchronous20Hz with valid substeps")


def destroy_owned(actors, errors):
    """HH_260906 - Check each owned RPC destroy result without adding an unrecorded physics tick."""
    confirmed = []
    for actor in reversed(actors):
        if str(getattr(actor, "type_id", "")).startswith("sensor."):
            try:
                actor.stop()
            except Exception as error:
                errors.append(f"stop owned sensor{actor.id}: {error}")
        try:
            if actor.destroy() is not True:
                errors.append(f"destroy owned actor{actor.id} did not confirm success")
            else:
                confirmed.append(int(actor.id))
        except Exception as error:
            errors.append(f"destroy owned actor{actor.id}: {error}")
    return confirmed


def run(args):
    require_owner_lock()
    # HH_260906 - Refuse dangling links before resolve can silently redirect this create-only output.
    requested_partial = args.output.with_name(args.output.name + ".partial")
    if args.output.exists() or args.output.is_symlink() or requested_partial.exists() or requested_partial.is_symlink():
        raise capture.CollectionError("stationary audit output already exists")
    output = args.output.resolve()
    partial = output.with_name(output.name + ".partial")
    if output.exists() or output.is_symlink() or partial.exists() or partial.is_symlink():
        raise capture.CollectionError("stationary audit output already exists")
    route = capture.load_route(args.route_file)
    specs = capture.load_camera_specs(args.mapping.resolve(), args.calibration.resolve(), 2.85)
    # HH_260906 - Import only the existing local CARLA API after strict local preflight.
    import carla
    client = carla.Client(args.host, args.port)
    client.set_timeout(4.0)
    world = client.get_world()
    if world.get_map().name.rsplit("/", 1)[-1] != "Town07":
        raise capture.CollectionError("owned stationary audit map mismatch")
    original_settings, original_weather = world.get_settings(), world.get_weather()
    exclusive_world(world)
    partial.mkdir(parents=True, exist_ok=False)
    route_bytes = args.route_file.read_bytes()
    (partial / "route.json").write_bytes(route_bytes)
    actors, states, frames, cleanup_errors = [], [], [], []
    previous_handlers, interrupted = {}, False
    def stop(signum, _frame):
        nonlocal interrupted
        interrupted = True
    for signum in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[signum] = signal.getsignal(signum)
        signal.signal(signum, stop)
    report = {"schema": "carla.stationary_camera_quality.v1", "status": "RUNNING",
              "purpose": "Stationary appearance diagnostic, not expert driving, training data or model evaluation.",
              "learned_model_control": False, "training_data_approved": False,
              "physics_hz": 20.0, "camera_sensor_tick_seconds": 0.0,
              "camera_bundle_observed_every_native_tick": True, "scheduled_ticks": 160,
              "saved_camera_ticks": list(RECORD_TICKS), "all_commands": {"throttle": 0.0, "brake": 1.0, "steer": 0.0},
              "control_timing_notice": "Batch response acknowledges the requested control; get_control is API-reported state, not substep physical-application-time proof.",
              "vehicle_type": "vehicle.toyota.prius", "weather": "ClearNoon", "route_sha256": ROUTE_SHA,
              "source_sha256": {"worker": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                                "collector_helpers": hashlib.sha256(Path(capture.__file__).read_bytes()).hexdigest(),
                                "mapping": hashlib.sha256(args.mapping.read_bytes()).hexdigest(),
                                "calibration": hashlib.sha256(args.calibration.read_bytes()).hexdigest()},
              "camera_specs": [capture.asdict(spec) for spec in specs]}
    try:
        settings = world.get_settings()
        settings.synchronous_mode, settings.fixed_delta_seconds, settings.no_rendering_mode = True, .05, False
        world.apply_settings(settings)
        world.set_weather(carla.WeatherParameters.ClearNoon)
        actual_settings = world.get_settings()
        report["world_settings"] = capture._settings_dict(actual_settings)
        validate_settings(actual_settings)
        blueprint_library = world.get_blueprint_library()
        blueprint = blueprint_library.find("vehicle.toyota.prius")
        blueprint.set_attribute("role_name", "autoware_e2e_stationary_visual_audit")
        transform = capture.apply_spawn_z_offset(capture.shift_transform_local_x(route["start_carla_transform"], 1.425), .5)
        ego = world.try_spawn_actor(blueprint, capture._carla_transform(carla, transform))
        if ego is None:
            raise capture.CollectionError("cannot spawn the owned stationary vehicle")
        actors.append(ego)
        queues = {spec.name: queue.Queue() for spec in specs}
        for spec in specs:
            sensor_blueprint = blueprint_library.find(spec.carla_type)
            capture._configure_camera_blueprint(sensor_blueprint, spec)
            sensor = world.spawn_actor(sensor_blueprint, capture._carla_transform(carla, spec.carla_extrinsic),
                                       attach_to=ego, attachment_type=carla.AttachmentType.Rigid)
            actors.append(sensor)
            sensor.listen(queues[spec.name].put)
        for tick in range(1, 161):
            if interrupted:
                raise capture.CollectionError("stationary audit interrupted")
            submit_brake(client, carla, ego.id)
            frame = int(world.tick(4.0))
            snapshot = world.get_snapshot()
            actor = snapshot.find(ego.id)
            if int(snapshot.frame) != frame or actor is None:
                raise capture.CollectionError("stationary immutable actor snapshot mismatch")
            position, velocity = actor.get_transform(), actor.get_velocity()
            speed = math.hypot(velocity.x, velocity.y)
            states.append({"tick": tick, "frame": frame, "timestamp": snapshot.timestamp.elapsed_seconds,
                           "actor_transform": capture._transform_dict(position), "planar_speed_mps": speed,
                           "reported_control": capture.control_dict(ego.get_control())})
            timestamp = float(snapshot.timestamp.elapsed_seconds)
            if not math.isfinite(timestamp) or not math.isfinite(speed):
                raise capture.CollectionError("nonfinite stationary state measurement")
            if len(states) > 1 and (frame - states[-2]["frame"] != 1 or abs(timestamp - states[-2]["timestamp"] - .05) > 1e-4):
                raise capture.CollectionError("stationary state cadence/frame stride mismatch")
            control = states[-1]["reported_control"]
            if (any(not math.isfinite(control[name]) or abs(control[name] - expected) > 1e-6
                    for name, expected in (("throttle", 0.), ("brake", 1.), ("steer", 0.)))
                    or any(control[name] is not False for name in ("hand_brake", "reverse", "manual_gear_shift"))):
                raise capture.CollectionError("API-reported stationary control differs from the fixed brake request")
            if tick > 70 and speed > .1:
                raise capture.CollectionError("stationary visual audit moved after settling")
            # HH_260906 - Drain and validate every native camera bundle; retain only the four declared visual samples.
            bundle = capture.exact_camera_bundle(queues, frame, 5.0)
            for spec in specs:
                image = bundle[spec.name]
                capture._validate_image_geometry(image, spec)
                if not math.isfinite(float(image.timestamp)) or abs(image.timestamp - timestamp) > 1e-6:
                    raise capture.CollectionError("stationary camera/state timestamp mismatch")
            if tick in RECORD_TICKS:
                entry = {"tick": tick, "frame": frame, "timestamp": snapshot.timestamp.elapsed_seconds,
                         "complete": False, "images": {}}
                # HH_260906 - Preserve indexed partial images if a later camera encoding or write fails.
                frames.append(entry)
                for spec in specs:
                    image = bundle[spec.name]
                    relative = Path("images") / f"tick_{tick:03d}_{spec.name}.jpg"
                    (partial / relative).parent.mkdir(exist_ok=True)
                    capture._save_jpeg(image, partial / relative, 95)
                    entry["images"][spec.name] = {"path": relative.as_posix(), "timestamp": image.timestamp,
                                                 "sha256": hashlib.sha256((partial / relative).read_bytes()).hexdigest()}
                entry["complete"] = True
        report["status"] = "COMPLETE"
    except BaseException as error:
        report.update(status="FAIL", error=str(error))
        raise
    finally:
        report["owned_actor_ids"] = [int(actor.id) for actor in actors]
        report["destroy_rpc_confirmed_actor_ids"] = destroy_owned(actors, cleanup_errors)
        for label, restore in (("weather", lambda: world.set_weather(original_weather)),
                               ("settings", lambda: world.apply_settings(original_settings))):
            try:
                restore()
            except Exception as error:
                cleanup_errors.append(f"restore {label}: {error}")
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
        report["cleanup_errors"] = cleanup_errors
        report["state_count"] = len(states)
        report["saved_camera_anchor_count"] = sum(entry["complete"] for entry in frames)
        report["partial_camera_anchor_count"] = sum(not entry["complete"] for entry in frames)
        if cleanup_errors:
            report["status"] = "FAIL"
        capture._write_json(partial / "manifest.json", report)
        for name, rows in (("states.jsonl", states), ("camera_frames.jsonl", frames)):
            (partial / name).write_text("".join(json.dumps(row) + "\n" for row in rows))
    if report["status"] != "COMPLETE":
        raise capture.CollectionError("stationary audit cleanup failed")
    partial.rename(output)
    return output


def main():
    print(run(parse_args()))


if __name__ == "__main__":
    main()
