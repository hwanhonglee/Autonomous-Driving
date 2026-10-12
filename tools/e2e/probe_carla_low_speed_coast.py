#!/usr/bin/env python3
"""HH_260906 - Measure fixed low-speed launch/coast transitions without label changes or data admission."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import queue
import signal

if __package__:
    from . import probe_carla_stationary_motion as stationary
else:
    import probe_carla_stationary_motion as stationary

capture, calibration = stationary.capture, stationary.calibration
camera_probe, substeps = stationary.camera_probe, stationary.substeps
sha256, weather_snapshot = stationary.sha256, stationary.weather_snapshot
fresh_paths, checked_frame = stationary.fresh_paths, stationary.checked_frame
physics_observation, close_journals = stationary.physics_observation, stationary.close_journals
validate_settings = stationary.validate_settings
ROUTE_SHA, MAPPING_SHA, CALIBRATION_SHA = stationary.ROUTE_SHA, stationary.MAPPING_SHA, stationary.CALIBRATION_SHA
SOURCE_NAMES = ("probe_carla_low_speed_coast.py",) + stationary.SOURCE_NAMES
SETTLE_TICKS, MAX_LAUNCH_TICKS, COAST_TICKS = 70, 160, 900
MAXIMUM_TICKS = SETTLE_TICKS + MAX_LAUNCH_TICKS + COAST_TICKS
OPERATIONAL_LIMITS = {"max_speed_mps": .75, "minimum_body_vx_mps": -.1,
                      "max_net_motion_m": 10., "max_route_cte_m": 1.,
                      "settled_max_speed_mps": .1, "settled_max_net_motion_m": .5}


def fingerprints(args):
    # HH_260906 - Reuse all frozen input/source pins and additionally bind this sole new executable.
    sources, inputs = stationary.fingerprints(args)
    sources["scripts/e2e/probe_carla_low_speed_coast.py"] = sha256(Path(__file__))
    return sources, inputs


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("output", type=Path)
    parser.add_argument("route_file", type=Path)
    parser.add_argument("--host", choices=("127.0.0.1",), default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2100)
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


def phase_control(carla, phase):
    # HH_260906 - Only launch applies throttle; no tail brake, manual gear or steering correction exists.
    if phase not in ("settle", "launch", "coast"):
        raise capture.CollectionError("unknown low-speed coast phase")
    return carla.VehicleControl(throttle=.13 if phase == "launch" else 0., steer=0.,
                               brake=1. if phase == "settle" else 0., hand_brake=False,
                               reverse=False, manual_gear_shift=False, gear=0)


def following_phase(row):
    """HH_260906 - Handoff on the first observed threshold crossing; retain all900 coast ticks without dwell truncation."""
    phase, count = row["phase"], row["phase_tick"]
    if phase == "settle" and count == SETTLE_TICKS:
        return "launch"
    if phase == "launch":
        if row["planar_speed_mps"] >= .5:
            return "coast"
        if count == MAX_LAUNCH_TICKS:
            raise capture.CollectionError("launch_timeout_below_0.5_mps")
    if phase == "coast" and count == COAST_TICKS:
        return "COMPLETE"
    return phase


def safety_reason(row, collision_count):
    """HH_260906 - These are operational aborts, not modified trajectory admission or curvature gates."""
    if collision_count:
        return "collision"
    if row["planar_speed_mps"] > .75:
        return "maximum_planar_speed_exceeded"
    if row["vx"] < -.1:
        return "reverse_body_speed_exceeded"
    if row["net_planar_motion_m"] > 10.:
        return "maximum_planar_motion_exceeded"
    if row["route_cte_m"] > 1.:
        return "maximum_route_cte_exceeded"
    if row["phase"] == "settle" and row["phase_tick"] == SETTLE_TICKS:
        if row["planar_speed_mps"] > .1:
            return "settled_speed_exceeded"
        if row["net_planar_motion_m"] > .5:
            return "settled_motion_exceeded"
    return None


def retained_numeric(value):
    # HH_260906 - Failed API scalars remain explicit evidence without emitting nonstandard JSON NaN/Infinity tokens.
    return value if math.isfinite(value) else {"nonfinite_numeric": repr(value)}


def corridor_preflight(world, carla, route, record):
    """HH_260906 - Retain bounded lane/ray observations, not a swept-volume or complete free-space proof."""
    record.update(status="RUNNING", corridor_samples=[], obstacle_rays=[],
                  frame_before=checked_frame(world),
                  notice="Lane samples and six static-geometry rays only; not swept-volume/free-space proof.")
    calibration.exclusive_world(world)
    road_map = world.get_map()
    spawn_points = road_map.get_spawn_points()
    index = route["start_spawn_index"]
    if type(index) is not int or not 0 <= index < len(spawn_points):
        raise capture.CollectionError("fixed route spawn index is unavailable")
    observed_spawn = capture._transform_dict(spawn_points[index])
    record.update(spawn_index=index, live_spawn_transform_carla={key: retained_numeric(value) for key, value in observed_spawn.items()},
                  foreign_dynamic_actors_absent=True)
    expected = route["start_carla_transform"]
    if any(not math.isfinite(value) for value in observed_spawn.values()) or any(
            abs(observed_spawn[key] - expected[key]) > .02 for key in ("x", "y", "z")) or abs(
            math.remainder(observed_spawn["yaw"] - expected["yaw"], 360.)) > .1:
        raise capture.CollectionError("live spawn differs from the fixed route")
    points = [point for point in route["route"] if point["distance_m"] <= 14.]
    if not points or points[-1]["distance_m"] < 13.:
        raise capture.CollectionError("fixed route lacks the short forward corridor")
    yaw = math.radians(expected["yaw"])
    samples = record["corridor_samples"]
    for point in points:
        dx, dy = point["x"] - expected["x"], -point["y"] - expected["y"]
        lateral = abs(-math.sin(yaw) * dx + math.cos(yaw) * dy)
        if lateral > .2:
            raise capture.CollectionError("short route corridor deviates from zero-steer launch direction")
        location = carla.Location(x=point["x"], y=-point["y"], z=point["z"])
        sample = {"distance_m": point["distance_m"], "requested_location_carla": dict(x=point["x"], y=-point["y"], z=point["z"]),
                  "zero_steer_lateral_offset_m": lateral, "status": "PENDING"}
        samples.append(sample)
        waypoint = road_map.get_waypoint(location, project_to_road=False, lane_type=carla.LaneType.Driving)
        if waypoint is None:
            raise capture.CollectionError("fixed short corridor is not a driving lane")
        actual = capture._transform_dict(waypoint.transform)
        width = float(waypoint.lane_width)
        sample.update(waypoint_transform_carla={key: retained_numeric(value) for key, value in actual.items()},
                      road_id=waypoint.road_id, lane_id=waypoint.lane_id, lane_width_m=retained_numeric(width))
        if (any(not math.isfinite(value) for value in actual.values()) or not math.isfinite(width) or width <= 0.
                or abs(math.remainder(actual["yaw"] - expected["yaw"], 360.)) > 5.
                or math.hypot(actual["x"] - point["x"], actual["y"] + point["y"]) > .2):
            raise capture.CollectionError("live corridor lane geometry differs from the fixed route")
        sample["status"] = "PASS"
    # HH_260906 - Extend14m ahead of the route base for the10m actor travel bound and bumper margin; never filter hit labels.
    for lateral in (0., -.75, .75):
        for height in (.5, 1.25):
            start = {"x": expected["x"] - math.sin(yaw) * lateral,
                     "y": expected["y"] + math.cos(yaw) * lateral, "z": points[0]["z"] + height}
            end = {"x": start["x"] + math.cos(yaw) * 14., "y": start["y"] + math.sin(yaw) * 14.,
                   "z": points[-1]["z"] + height}
            ray = {"lateral_offset_m": lateral, "height_above_road_m": height,
                   "start_carla": start, "end_carla": end, "hits": [], "status": "PENDING"}
            record["obstacle_rays"].append(ray)
            for hit in world.cast_ray(carla.Location(**start), carla.Location(**end)):
                position = capture._vector_tuple(hit.location)
                ray["hits"].append({"location_carla": [retained_numeric(value) for value in position], "label": str(hit.label)})
                if not all(math.isfinite(value) for value in position):
                    raise capture.CollectionError("preflight ray returned a nonfinite hit")
            ray["status"] = "BLOCKED" if ray["hits"] else "CLEAR_SAMPLED_RAY"
            if ray["hits"]:
                raise capture.CollectionError("static geometry intersects a preflight ray")
    record["frame_after"] = checked_frame(world)
    if record["frame_after"] != record["frame_before"]:
        raise capture.CollectionError("world advanced during no-tick corridor preflight")
    record["status"] = "PASS"
    return record


def run(args):
    owner = calibration.require_inherited_workspace_lock()
    source_before, input_before = fingerprints(args)
    output, partial = fresh_paths(args.output)
    if args.host != "127.0.0.1" or not 1024 <= args.port <= 65533 or args.allow_map_load is not False:
        raise capture.CollectionError("low-speed coast probe requires the fixed local owned-world scope")
    route = capture.load_route(args.route_file)
    specs = capture.load_camera_specs(args.mapping, args.calibration, 2.85)
    if tuple(spec.name for spec in specs) != capture.MODEL_CAMERA_ORDER or any(spec.sensor_tick != 0.0 for spec in specs):
        raise capture.CollectionError("low-speed coast probe requires six native-tick cameras in fixed model order")
    import carla
    client = carla.Client(args.host, args.port)
    client.set_timeout(4.0)
    world = client.get_world()
    if world.get_map().name.rsplit("/", 1)[-1] != route["town"]:
        raise capture.CollectionError("owned low-speed coast motion map mismatch; map loading is prohibited")
    calibration.exclusive_world(world)
    original_settings, original_weather = world.get_settings(), world.get_weather()
    partial.mkdir(parents=True, exist_ok=False)
    actors, states, frames, cleanup_errors = [], [], [], []
    events = capture.EventRecorder()
    handlers, interrupted = {}, []
    report = {"schema": "carla.low_speed_coast_probe.v1", "status": "RUNNING",
        "started_at": capture.utc_now(), "training_data_approved": False, "vehicle_approved": False,
        "vehicle_control_approved": False, "learned_model_control": False, "data_admission_performed": False,
        "purpose": "Fixed low-speed launch then full zero-pedal coast diagnostic; no training labels or physical cause proof.",
        "maximum_native_ticks": MAXIMUM_TICKS, "settle_ticks": SETTLE_TICKS, "max_launch_ticks": MAX_LAUNCH_TICKS,
        "coast_ticks": COAST_TICKS, "launch_throttle": .13, "target_speed_mps": .5,
        "operational_limits": OPERATIONAL_LIMITS, "phase_transitions": [],
        "physics_hz": 20.0, "camera_hz": 10.0, "camera_bundle_observed_every_native_tick": True,
        "saved_camera_rule": "every even global native tick, including settle and launch", "camera_specs": [capture.asdict(s) for s in specs],
        "vehicle_type": "vehicle.toyota.prius", "wheelbase_m": 2.85, "spawn_z_offset_m": .5,
        "weather": "ClearNoon", "color_seed": 0, "town": route["town"], "route_sha256": ROUTE_SHA,
        "owner_lock": owner, "controls_by_phase": {phase: capture.control_dict(phase_control(carla, phase))
            for phase in ("settle", "launch", "coast")}, "physics_snapshots": {},
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
            raise capture.CollectionError("world settings changed before low-speed coast configuration")
        settings.synchronous_mode, settings.fixed_delta_seconds, settings.no_rendering_mode = True, .05, False
        settings.substepping, settings.max_substep_delta_time, settings.max_substeps = True, .01, 10
        requested = dict(report["world_settings_before"], synchronous_mode=True, fixed_delta_seconds=.05,
                         no_rendering_mode=False, substepping=True, max_substep_delta_time=.01, max_substeps=10)
        report["world_settings_requested"] = requested
        if not substeps.equal_settings(substeps.settings_snapshot(settings), requested):
            raise capture.CollectionError("low-speed coast requested settings changed an undeclared field")
        world.apply_settings(settings)
        report["weather_requested"] = weather_snapshot(carla.WeatherParameters.ClearNoon)
        world.set_weather(carla.WeatherParameters.ClearNoon)
        report["world_settings"] = substeps.settings_snapshot(world.get_settings())
        if not substeps.equal_settings(report["world_settings"], requested):
            raise capture.CollectionError("low-speed coast settings readback mismatch")
        validate_settings(world.get_settings())
        report["weather_actual"] = weather_snapshot(world.get_weather())
        if report["weather_actual"] != report["weather_requested"]:
            raise capture.CollectionError("ClearNoon weather readback mismatch")
        # HH_260906 - Frame-bracket read-only corridor checks after synchronous settings, but before any actor or physics tick.
        report["preflight"] = {}
        corridor_preflight(world, carla, route, report["preflight"])
        report["client_version"], report["server_version"] = client.get_client_version(), client.get_server_version()
        library = world.get_blueprint_library()
        blueprint = library.find("vehicle.toyota.prius")
        blueprint.set_attribute("role_name", "autoware_e2e_low_speed_coast_probe")
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
            raise capture.CollectionError("cannot spawn owned low-speed coast vehicle")
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
        phase, phase_tick, progress = "settle", 0, 0.0
        for tick in range(1, MAXIMUM_TICKS + 1):
            if interrupted:
                raise capture.CollectionInterrupted("low-speed coast motion probe interrupted")
            phase_tick += 1
            receipt = transport.send(phase_control(carla, phase), phase, expected_before_frame=previous_frame)
            frame = int(world.tick(4.0))
            snapshot = world.get_snapshot()
            row = {"tick": tick, "phase": phase, "phase_tick": phase_tick, "frame": frame, "measurement_complete": False,
                   "requested_control": receipt["requested_control"], "collision": []}
            states.append(row)
            if int(snapshot.frame) != frame or frame != previous_frame + 1:
                raise capture.CollectionError("immutable snapshot/native frame stride mismatch")
            timestamp = float(snapshot.timestamp.elapsed_seconds)
            if not math.isfinite(timestamp) or (len(states) > 1 and abs(timestamp - states[-2]["timestamp"] - .05) > 1e-4):
                raise capture.CollectionError("low-speed coast native timestamp cadence failed")
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
            progress, cte, segment = calibration.bounded_route_projection(route["route"], row["x"], row["y"], progress, 2.0)
            row.update(route_progress_m=progress, route_cte_m=cte, route_segment_index=segment)
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
                raise capture.CollectionError("low-speed coast control/wheel observation frame alignment failed")
            row["measurement_complete"] = True
            collisions, _ = events.snapshot()
            reason = safety_reason(row, sum(map(len, collisions.values())))
            if reason:
                report["safety_failure"] = reason
                raise capture.CollectionError(reason)
            previous_frame = frame
            next_phase = following_phase(row)
            if next_phase == "COMPLETE":
                report["completion_reason"] = "full_900_tick_zero_pedal_coast_observed"
                break
            if next_phase != phase:
                if phase == "settle":
                    report["physics_snapshots"]["settled"] = physics_observation(world, ego, frame)
                report["phase_transitions"].append({"from_phase": phase, "to_phase": next_phase,
                    "source_tick": tick, "source_phase_tick": phase_tick, "source_frame": frame,
                    "source_timestamp": timestamp, "source_planar_speed_mps": row["planar_speed_mps"],
                    "effective_tick": tick + 1, "next_requested_control": capture.control_dict(phase_control(carla, next_phase)),
                    "physical_transition_time_proven": False})
                if phase == "launch":
                    report["launch_handoff"] = dict(report["phase_transitions"][-1])
                phase, phase_tick = next_phase, 0
        if phase != "coast" or phase_tick != COAST_TICKS:
            raise capture.CollectionError("coast schedule did not complete its fixed duration")
        if interrupted:
            raise capture.CollectionInterrupted("low-speed coast motion probe interrupted")
        validate_settings(world.get_settings())
        if not substeps.equal_settings(substeps.settings_snapshot(world.get_settings()), requested):
            raise capture.CollectionError("world settings drifted during low-speed coast capture")
        report["status"] = "COMPLETE"
    except BaseException as error:
        report.update(status="FAIL", error=f"{type(error).__name__}: {error}")
        if report.get("preflight", {}).get("status") == "RUNNING":
            report["preflight"].update(status="FAIL", error=str(error))
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
                      received_signals=interrupted,
                      phase_counts={phase: sum(row["phase"] == phase for row in states) for phase in ("settle", "launch", "coast")})
        for name, rows in (("native_states.jsonl", states), ("camera_frames.jsonl", frames)):
            capture._write_jsonl(partial / name, rows)
        report["artifact_sha256"] = {name: sha256(partial / name) for name in
            ("native_states.jsonl", "native_observations.jsonl", "camera_frames.jsonl", "control_receipts.jsonl",
             "route.json", "mapping.yaml", "calibration.yaml") if (partial / name).is_file()}
        capture._write_json(partial / "manifest.json", report)
    if report["status"] != "COMPLETE":
        raise capture.CollectionError("low-speed coast probe failed final checks; partial evidence retained")
    if output.exists() or output.is_symlink():
        raise capture.CollectionError("output appeared during capture; partial evidence retained")
    partial.rename(output)
    return output


def main():
    print(run(parse_args()))


if __name__ == "__main__":
    main()
