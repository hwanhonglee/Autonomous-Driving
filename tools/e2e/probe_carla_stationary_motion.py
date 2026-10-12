#!/usr/bin/env python3
"""HH_260906 - Observe fixed service/parking brake micro-motion; never admit data or drive a model."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import signal

if __package__:
    from . import collect_carla_vad_expert as capture
    from . import calibrate_carla_low_speed_response as calibration
    from . import probe_carla_stationary_camera_quality as camera_probe
    from . import carla_physics_substeps as substeps
else:
    import collect_carla_vad_expert as capture
    import calibrate_carla_low_speed_response as calibration
    import probe_carla_stationary_camera_quality as camera_probe
    import carla_physics_substeps as substeps


ROUTE_SHA = "2299e5bf2bc86789650da72336fdcfe7c11c585df0b9e0a0fe29a44730bae79b"
MAPPING_SHA, CALIBRATION_SHA = camera_probe.MAPPING_SHA, camera_probe.CALIBRATION_SHA
SETTLE_TICKS, HOLD_TICKS = 70, 400
TOTAL_TICKS = SETTLE_TICKS + HOLD_TICKS
SOURCE_NAMES = ("probe_carla_stationary_motion.py", "collect_carla_vad_expert.py",
                "calibrate_carla_low_speed_response.py", "probe_carla_stationary_camera_quality.py",
                "carla_goal_stop_profile.py", "carla_low_speed_response_matrix.py", "carla_physics_substeps.py")
WEATHER_FIELDS = ("cloudiness", "dust_storm", "fog_density", "fog_distance", "fog_falloff", "mie_scattering_scale",
                  "precipitation", "precipitation_deposits", "rayleigh_scattering_scale", "scattering_intensity",
                  "sun_altitude_angle", "sun_azimuth_angle", "wetness", "wind_intensity")


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def weather_snapshot(weather):
    """HH_260906 - Read only fourteen numeric API fields; class-level weather presets recursively contain weather objects."""
    result = {}
    for name in WEATHER_FIELDS:
        value = getattr(weather, name, None)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise capture.CollectionError(f"WeatherParameters {name} must be a finite numeric value")
        result[name] = float(value)
    return result


def fingerprints(args):
    """HH_260906 - Bind this worker and its complete repository-local import closure, plus all fixed inputs."""
    folder = Path(__file__).resolve().parent
    sources = {"scripts/e2e/" + name: sha256(folder / name) for name in SOURCE_NAMES}
    inputs = {name: sha256(getattr(args, name)) for name in ("route_file", "mapping", "calibration")}
    if inputs != {"route_file": ROUTE_SHA, "mapping": MAPPING_SHA, "calibration": CALIBRATION_SHA}:
        raise capture.CollectionError("stationary motion probe requires unchanged fixed route and six-camera rig")
    return sources, inputs


def fresh_paths(output):
    """HH_260906 - Reject existing outputs and symlinked ancestors before creating a partial artifact."""
    output = output.absolute()
    partial = output.with_name(output.name + ".partial")
    for path in (output, partial):
        if path.exists() or path.is_symlink() or any(parent.is_symlink() for parent in path.parents):
            raise capture.CollectionError("stationary motion output must be fresh and have no symlink ancestors")
    return output, partial


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("output", type=Path)
    parser.add_argument("route_file", type=Path)
    parser.add_argument("--host", choices=("127.0.0.1",), default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2100)
    parser.add_argument("--arm", choices=("service", "parking"), required=True)
    parser.add_argument("--mapping", type=Path, default=capture.DEFAULT_MAPPING)
    parser.add_argument("--calibration", type=Path, default=capture.DEFAULT_CALIBRATION)
    args = parser.parse_args(argv)
    args.allow_map_load = False
    try:
        if not 1024 <= args.port <= 65533:
            raise capture.CollectionError("port must be in [1024,65533]")
        fingerprints(args)
        fresh_paths(args.output)
    except (OSError, capture.CollectionError) as error:
        parser.error(str(error))
    return args


def fixed_control(carla, arm):
    """HH_260906 - The parking arm changes only hand_brake; automatic gears remain observed, not forced."""
    if arm not in ("service", "parking"):
        raise capture.CollectionError("unknown stationary brake arm")
    return carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0, hand_brake=arm == "parking",
                                reverse=False, manual_gear_shift=False, gear=0)


def validate_settings(settings):
    """HH_260906 - Require exact read-back reference settings, not merely sufficient substep capacity."""
    values = (settings.fixed_delta_seconds, settings.max_substep_delta_time)
    if (any(isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value)
            for value in values) or abs(values[0] - .05) > 1e-9 or abs(values[1] - .01) > 1e-9
            or settings.synchronous_mode is not True or settings.no_rendering_mode is not False
            or settings.substepping is not True or type(settings.max_substeps) is not int or settings.max_substeps != 10):
        raise capture.CollectionError("actual settings must be rendered synchronous20Hz/reference10ms/10 substeps")


def checked_frame(world):
    return capture.checked_agent_initialization_frame(world.get_snapshot().frame)


def physics_observation(world, ego, expected_frame):
    """HH_260906 - Bracket read-only physics/pose RPCs; wheel positions are not claimed as ground contacts or COM."""
    before = checked_frame(world)
    pose = capture._transform_dict(ego.get_transform())
    result = calibration.physics_snapshot(ego.get_physics_control())
    after = checked_frame(world)
    if before != expected_frame or after != before or any(not math.isfinite(value) for value in pose.values()):
        raise capture.CollectionError("physics snapshot frame bracket or pose failed")
    return {**result, "frame_before": before, "frame_after": after, "actor_transform_carla": pose,
            "pose_source": "Frame-bracketed actor API, not an independently verified physical reference point."}


def safety_reason(row, collision_count):
    if collision_count:
        return "collision"
    if row["tick"] >= SETTLE_TICKS and row["planar_speed_mps"] > .1:
        return "post_settle_speed_exceeded"
    if row["tick"] >= SETTLE_TICKS and row["net_planar_motion_m"] > .5:
        return "post_settle_net_motion_exceeded"
    return None


def close_journals(streams, errors):
    # HH_260906 - Journal I/O failure must never prevent owned actor/settings cleanup.
    for name, stream in streams:
        if stream is not None:
            try:
                stream.close()
            except Exception as error:
                errors.append(f"close {name}: {error}")


def run(args):
    owner = calibration.require_inherited_workspace_lock()
    source_before, input_before = fingerprints(args)
    output, partial = fresh_paths(args.output)
    if args.host != "127.0.0.1" or not 1024 <= args.port <= 65533 or args.allow_map_load is not False:
        raise capture.CollectionError("stationary probe requires the fixed local owned-world scope")
    route = capture.load_route(args.route_file)
    specs = capture.load_camera_specs(args.mapping, args.calibration, 2.85)
    if tuple(spec.name for spec in specs) != capture.MODEL_CAMERA_ORDER or any(spec.sensor_tick != 0.0 for spec in specs):
        raise capture.CollectionError("stationary probe requires six native-tick cameras in fixed model order")
    import carla
    client = carla.Client(args.host, args.port)
    client.set_timeout(4.0)
    world = client.get_world()
    if world.get_map().name.rsplit("/", 1)[-1] != route["town"]:
        raise capture.CollectionError("owned stationary motion map mismatch; map loading is prohibited")
    calibration.exclusive_world(world)
    original_settings, original_weather = world.get_settings(), world.get_weather()
    control = fixed_control(carla, args.arm)
    partial.mkdir(parents=True, exist_ok=False)
    actors, states, frames, cleanup_errors = [], [], [], []
    events = capture.EventRecorder()
    handlers, interrupted = {}, []
    report = {"schema": "carla.stationary_motion_probe.v1", "status": "RUNNING", "arm": args.arm,
        "started_at": capture.utc_now(), "training_data_approved": False, "vehicle_approved": False,
        "vehicle_control_approved": False, "learned_model_control": False, "data_admission_performed": False,
        "purpose": "Command-held stationary micro-motion diagnostic; not driving, task labels, or physical cause proof.",
        "scheduled_ticks": TOTAL_TICKS, "settle_ticks": SETTLE_TICKS, "hold_ticks": HOLD_TICKS,
        "physics_hz": 20.0, "camera_hz": 10.0, "camera_bundle_observed_every_native_tick": True,
        "saved_camera_rule": "even native ticks 2 through 470, including settle", "camera_specs": [capture.asdict(s) for s in specs],
        "vehicle_type": "vehicle.toyota.prius", "wheelbase_m": 2.85, "spawn_z_offset_m": .5,
        "weather": "ClearNoon", "color_seed": 0, "town": route["town"], "route_sha256": ROUTE_SHA,
        "owner_lock": owner, "requested_control": capture.control_dict(control), "physics_snapshots": {},
        "source_sha256_before": source_before, "input_sha256_before": input_before,
        "control_timing_notice": "Batch ACK proves acceptance; observed control/wheel angles do not prove substep actuation time.",
        "reference_notice": "Unchanged virtual base transform; physical COM/rear-axle identity remains unverified."}
    receipt_stream, observation_stream = None, None
    try:
        (partial / "route.json").write_bytes(args.route_file.read_bytes())
        (partial / "mapping.yaml").write_bytes(args.mapping.read_bytes())
        (partial / "calibration.yaml").write_bytes(args.calibration.read_bytes())
        receipt_stream = (partial / "control_receipts.jsonl").open("x", encoding="utf-8")
        observation_stream = (partial / "native_observations.jsonl").open("x", encoding="utf-8")
        def persist_receipt(receipt):
            receipt_stream.write(json.dumps(receipt, allow_nan=False) + "\n")
            receipt_stream.flush()
            os.fsync(receipt_stream.fileno())
        for number in (signal.SIGINT, signal.SIGTERM):
            handlers[number] = signal.getsignal(number)
            signal.signal(number, lambda signum, _frame: interrupted.append(signum))
        report["world_settings_before"] = substeps.settings_snapshot(original_settings)
        report["weather_before"] = weather_snapshot(original_weather)
        settings = world.get_settings()
        if not substeps.equal_settings(substeps.settings_snapshot(settings), report["world_settings_before"]):
            raise capture.CollectionError("world settings changed before stationary configuration")
        settings.synchronous_mode, settings.fixed_delta_seconds, settings.no_rendering_mode = True, .05, False
        settings.substepping, settings.max_substep_delta_time, settings.max_substeps = True, .01, 10
        requested = dict(report["world_settings_before"], synchronous_mode=True, fixed_delta_seconds=.05,
                         no_rendering_mode=False, substepping=True, max_substep_delta_time=.01, max_substeps=10)
        report["world_settings_requested"] = requested
        if not substeps.equal_settings(substeps.settings_snapshot(settings), requested):
            raise capture.CollectionError("stationary requested settings changed an undeclared field")
        world.apply_settings(settings)
        world.set_weather(carla.WeatherParameters.ClearNoon)
        report["world_settings"] = substeps.settings_snapshot(world.get_settings())
        if not substeps.equal_settings(report["world_settings"], requested):
            raise capture.CollectionError("stationary settings readback mismatch")
        validate_settings(world.get_settings())
        report["weather_actual"] = weather_snapshot(world.get_weather())
        if report["weather_actual"] != weather_snapshot(carla.WeatherParameters.ClearNoon):
            raise capture.CollectionError("ClearNoon weather readback mismatch")
        report["client_version"], report["server_version"] = client.get_client_version(), client.get_server_version()
        library = world.get_blueprint_library()
        blueprint = library.find("vehicle.toyota.prius")
        blueprint.set_attribute("role_name", "autoware_e2e_stationary_motion_probe")
        if blueprint.has_attribute("color"):
            colors = blueprint.get_attribute("color").recommended_values
            if colors:
                blueprint.set_attribute("color", colors[0])
                report["vehicle_color"] = colors[0]
        transform = capture.apply_spawn_z_offset(capture.shift_transform_local_x(route["start_carla_transform"], 1.425), .5)
        # HH_260906 - Pre-tick actor API poses may be uninitialized zeros; use the exact requested spawn, without a hidden tick/reset.
        origin = dict(transform)
        report.update(requested_spawn_transform_carla=dict(transform), motion_origin_transform_carla=origin,
                      motion_origin_source="REQUESTED_SPAWN_TRANSFORM_NOT_PRETICK_ACTOR_API")
        ego = world.try_spawn_actor(blueprint, capture._carla_transform(carla, transform))
        if ego is None:
            raise capture.CollectionError("cannot spawn owned stationary vehicle")
        actors.append(ego)
        previous_frame = checked_frame(world)
        report["physics_snapshots"]["spawn"] = physics_observation(world, ego, previous_frame)
        report["physics_snapshots"]["spawn"].update(used_as_motion_origin=False,
            pre_tick_pose_notice="Preserved pre-first-tick actor API pose may be uninitialized; it is not the motion baseline.")
        collision = world.spawn_actor(library.find("sensor.other.collision"), carla.Transform(), attach_to=ego)
        actors.append(collision)
        collision.listen(events.on_collision)
        queues = {spec.name: queue.Queue() for spec in specs}
        for spec in specs:
            blueprint = library.find(spec.carla_type)
            capture._configure_camera_blueprint(blueprint, spec)
            sensor = world.spawn_actor(blueprint, capture._carla_transform(carla, spec.carla_extrinsic),
                                       attach_to=ego, attachment_type=carla.AttachmentType.Rigid)
            actors.append(sensor)
            sensor.listen(queues[spec.name].put)
        transport = capture.AcknowledgedControlTransport(client, carla, ego, lambda: checked_frame(world), persist_receipt)
        for tick in range(1, TOTAL_TICKS + 1):
            if interrupted:
                raise capture.CollectionInterrupted("stationary motion probe interrupted")
            phase = "settle" if tick <= SETTLE_TICKS else "hold"
            receipt = transport.send(control, phase, expected_before_frame=previous_frame)
            frame = int(world.tick(4.0))
            snapshot = world.get_snapshot()
            row = {"tick": tick, "phase": phase, "frame": frame, "measurement_complete": False,
                   "requested_control": receipt["requested_control"], "collision": []}
            states.append(row)
            if int(snapshot.frame) != frame or frame != previous_frame + 1:
                raise capture.CollectionError("immutable snapshot/native frame stride mismatch")
            timestamp = float(snapshot.timestamp.elapsed_seconds)
            if not math.isfinite(timestamp) or (len(states) > 1 and abs(timestamp - states[-2]["timestamp"] - .05) > 1e-4):
                raise capture.CollectionError("stationary native timestamp cadence failed")
            row.update(timestamp=timestamp, timestamp_ns=round(timestamp * 1e9))
            state, raw = capture.acknowledged_snapshot_state(snapshot, ego, 2.85)
            row.update(state, **raw)
            before = checked_frame(world)
            observed = ego.get_control()
            row["reported_control"] = transport.checked_control(observed)
            row.update(capture.front_steering_measurement(float(ego.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel)),
                                                        float(ego.get_wheel_steer_angle(carla.VehicleWheelLocation.FR_Wheel))))
            after = checked_frame(world)
            row.update(observed_aux_frame_before=before, observed_aux_frame_after=after,
                       control_alignment=transport.compare_observation(observed, receipt, frame, before, after))
            pose = raw["actor_snapshot_transform_carla"]
            row.update(planar_speed_mps=math.hypot(*raw["world_velocity_carla"][:2]),
                       net_planar_motion_m=math.hypot(pose["x"] - origin["x"], pose["y"] - origin["y"]))
            # HH_260906 - Flush raw observations before camera I/O; this is not a completed camera-bound measurement.
            observation_stream.write(json.dumps(dict(row, stage="OBSERVED_BEFORE_CAMERA"), allow_nan=False) + "\n")
            observation_stream.flush()
            # HH_260906 - Drain every native bundle, even unsaved odd ticks and safety-offending measurements.
            bundle = capture.exact_camera_bundle(queues, frame, 5.0)
            for spec in specs:
                image = bundle[spec.name]
                capture._validate_image_geometry(image, spec)
                if not math.isfinite(float(image.timestamp)) or abs(image.timestamp - timestamp) > 1e-6:
                    raise capture.CollectionError("camera/native timestamp mismatch")
            if tick % 2 == 0:
                entry = {"tick": tick, "frame": frame, "timestamp": timestamp, "timestamp_ns": row["timestamp_ns"],
                         "complete": False, "images": {}}
                frames.append(entry)
                for spec in specs:
                    relative = Path("images") / f"tick_{tick:03d}_{spec.name}.jpg"
                    (partial / relative).parent.mkdir(exist_ok=True)
                    capture._save_jpeg(bundle[spec.name], partial / relative, 95)
                    entry["images"][spec.name] = {"path": relative.as_posix(), "timestamp": timestamp, "sha256": sha256(partial / relative)}
                entry["complete"] = True
            if row["control_alignment"]["status"] != "PASS":
                raise capture.CollectionError("stationary control/wheel observation frame alignment failed")
            row["measurement_complete"] = True
            collisions, _ = events.snapshot()
            reason = safety_reason(row, sum(map(len, collisions.values())))
            if reason:
                report["safety_failure"] = reason
                raise capture.CollectionError(reason)
            if tick == SETTLE_TICKS:
                report["physics_snapshots"]["settled"] = physics_observation(world, ego, frame)
            previous_frame = frame
        if interrupted:
            raise capture.CollectionInterrupted("stationary motion probe interrupted")
        validate_settings(world.get_settings())
        if not substeps.equal_settings(substeps.settings_snapshot(world.get_settings()), requested):
            raise capture.CollectionError("world settings drifted during stationary capture")
        report["status"] = "COMPLETE"
    except BaseException as error:
        report.update(status="FAIL", error=f"{type(error).__name__}: {error}")
        raise
    finally:
        close_journals((("control receipts", receipt_stream), ("native observations", observation_stream)), cleanup_errors)
        report["owned_actor_ids"] = [int(actor.id) for actor in actors]
        report["destroy_rpc_confirmed_actor_ids"] = camera_probe.destroy_owned(actors, cleanup_errors)
        collisions, _ = events.snapshot()
        report["collision_events"] = [event for bucket in collisions.values() for event in bucket]
        for row in states:
            row["collision"] = collisions.get(row["frame"], [])
        if report["collision_events"]:
            report.update(status="FAIL", safety_failure="collision")
        for label, restore in (("weather", lambda: world.set_weather(original_weather)),
                               ("settings", lambda: world.apply_settings(original_settings))):
            try:
                restore()
            except Exception as error:
                cleanup_errors.append(f"restore {label}: {error}")
        try:
            report["world_settings_restored"] = substeps.settings_snapshot(world.get_settings())
            report["weather_restored"] = weather_snapshot(world.get_weather())
            if (not substeps.equal_settings(report["world_settings_restored"], substeps.settings_snapshot(original_settings))
                    or report["weather_restored"] != weather_snapshot(original_weather)):
                raise capture.CollectionError("restored settings/weather readback mismatch")
        except Exception as error:
            cleanup_errors.append(f"restoration readback: {error}")
        for number, handler in handlers.items():
            signal.signal(number, handler)
        try:
            source_after, input_after = fingerprints(args)
            report.update(source_sha256_after=source_after, input_sha256_after=input_after)
            if source_after != source_before or input_after != input_before:
                raise capture.CollectionError("source or input bytes changed during the probe")
        except Exception as error:
            cleanup_errors.append(f"post-run fingerprint: {error}")
        if cleanup_errors:
            report["status"] = "FAIL"
        report.update(cleanup_errors=cleanup_errors, finished_at=capture.utc_now(), state_count=len(states),
                      saved_camera_anchor_count=sum(entry["complete"] for entry in frames),
                      partial_camera_anchor_count=sum(not entry["complete"] for entry in frames),
                      received_signals=interrupted)
        for name, rows in (("native_states.jsonl", states), ("camera_frames.jsonl", frames)):
            capture._write_jsonl(partial / name, rows)
        report["artifact_sha256"] = {name: sha256(partial / name) for name in
            ("native_states.jsonl", "native_observations.jsonl", "camera_frames.jsonl", "control_receipts.jsonl",
             "route.json", "mapping.yaml", "calibration.yaml") if (partial / name).is_file()}
        capture._write_json(partial / "manifest.json", report)
    if report["status"] != "COMPLETE":
        raise capture.CollectionError("stationary probe failed final checks; partial evidence retained")
    if output.exists() or output.is_symlink():
        raise capture.CollectionError("output appeared during capture; partial evidence retained")
    partial.rename(output)
    return output


def main():
    print(run(parse_args()))


if __name__ == "__main__":
    main()
