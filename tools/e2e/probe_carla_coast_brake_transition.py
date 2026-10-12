#!/usr/bin/env python3
"""HH_260906 - Compare continued coast with service-brake application after a fixed low-speed coast."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import queue
import signal

if __package__:
    from . import probe_carla_low_speed_coast as coast
else:
    import probe_carla_low_speed_coast as coast

capture, calibration = coast.capture, coast.calibration
camera_probe, substeps = coast.camera_probe, coast.substeps
sha256, weather_snapshot = coast.sha256, coast.weather_snapshot
fresh_paths, checked_frame = coast.fresh_paths, coast.checked_frame
physics_observation, close_journals = coast.physics_observation, coast.close_journals
validate_settings, corridor_preflight = coast.validate_settings, coast.corridor_preflight
safety_reason = coast.safety_reason
ROUTE_SHA, MAPPING_SHA, CALIBRATION_SHA = coast.ROUTE_SHA, coast.MAPPING_SHA, coast.CALIBRATION_SHA
SOURCE_NAMES = ("probe_carla_coast_brake_transition.py",) + coast.SOURCE_NAMES
SETTLE_TICKS, MAX_LAUNCH_TICKS, COAST_TICKS, TAIL_TICKS = 70, 160, 600, 300
MAXIMUM_TICKS = SETTLE_TICKS + MAX_LAUNCH_TICKS + COAST_TICKS + TAIL_TICKS
OPERATIONAL_LIMITS = coast.OPERATIONAL_LIMITS


def fingerprints(args):
    # HH_260906 - Include the complete frozen helper closure plus this new transition worker; never rewrite old sources.
    sources, inputs = coast.fingerprints(args)
    sources["scripts/e2e/probe_carla_coast_brake_transition.py"] = sha256(Path(__file__))
    return sources, inputs


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("output", type=Path)
    parser.add_argument("route_file", type=Path)
    parser.add_argument("--arm", choices=("coast", "service"), required=True)
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


def phase_control(carla, phase, arm):
    """HH_260906 - Both arms share settle/launch/coast; only tail service brake differs."""
    if phase not in ("settle", "launch", "coast", "tail") or arm not in ("coast", "service"):
        raise capture.CollectionError("unknown coast/brake transition phase or arm")
    return carla.VehicleControl(throttle=.13 if phase == "launch" else 0., steer=0.,
                               brake=1. if phase == "settle" or (phase == "tail" and arm == "service") else 0.,
                               hand_brake=False, reverse=False, manual_gear_shift=False, gear=0)


def following_phase(row):
    """HH_260906 - Check the same coast endpoint in both arms; tail control begins only on the next acknowledged tick."""
    phase, count = row["phase"], row["phase_tick"]
    if phase == "settle" and count == SETTLE_TICKS:
        return "launch"
    if phase == "launch":
        if row["planar_speed_mps"] >= .5:
            return "coast"
        if count == MAX_LAUNCH_TICKS:
            raise capture.CollectionError("launch_timeout_below_0.5_mps")
    if phase == "coast" and count == COAST_TICKS:
        speed = row["planar_speed_mps"]
        if isinstance(speed, bool) or not isinstance(speed, (int, float)) or not math.isfinite(speed) or speed > .1:
            raise capture.CollectionError("coast_endpoint_speed_exceeds_0.1_mps_before_tail")
        return "tail"
    if phase == "tail" and count == TAIL_TICKS:
        return "COMPLETE"
    return phase


def run(args):
    owner = calibration.require_inherited_workspace_lock()
    source_before, input_before = fingerprints(args)
    output, partial = fresh_paths(args.output)
    if (args.host != "127.0.0.1" or not 1024 <= args.port <= 65533 or args.allow_map_load is not False
            or args.arm not in ("coast", "service")):
        raise capture.CollectionError("coast/brake transition probe requires the fixed local owned-world scope")
    route = capture.load_route(args.route_file)
    specs = capture.load_camera_specs(args.mapping, args.calibration, 2.85)
    if tuple(spec.name for spec in specs) != capture.MODEL_CAMERA_ORDER or any(spec.sensor_tick != 0.0 for spec in specs):
        raise capture.CollectionError("coast/brake transition probe requires six native-tick cameras in fixed model order")
    import carla
    client = carla.Client(args.host, args.port)
    client.set_timeout(4.0)
    world = client.get_world()
    if world.get_map().name.rsplit("/", 1)[-1] != route["town"]:
        raise capture.CollectionError("owned coast/brake transition motion map mismatch; map loading is prohibited")
    calibration.exclusive_world(world)
    original_settings, original_weather = world.get_settings(), world.get_weather()
    partial.mkdir(parents=True, exist_ok=False)
    actors, states, frames, cleanup_errors = [], [], [], []
    events = capture.EventRecorder()
    handlers, interrupted = {}, []
    report = {"schema": "carla.coast_brake_transition_probe.v1", "status": "RUNNING", "arm": args.arm,
        "started_at": capture.utc_now(), "training_data_approved": False, "vehicle_approved": False,
        "vehicle_control_approved": False, "learned_model_control": False, "data_admission_performed": False,
        "purpose": "Matched coast-to-tail brake transition diagnostic; no training labels, admission, or physical cause proof.",
        "maximum_native_ticks": MAXIMUM_TICKS, "settle_ticks": SETTLE_TICKS, "max_launch_ticks": MAX_LAUNCH_TICKS,
        "coast_ticks": COAST_TICKS, "tail_ticks": TAIL_TICKS, "tail_entry_max_speed_mps": .1,
        "launch_throttle": .13, "target_speed_mps": .5,
        "operational_limits": OPERATIONAL_LIMITS, "phase_transitions": [],
        "physics_hz": 20.0, "camera_hz": 10.0, "camera_bundle_observed_every_native_tick": True,
        "saved_camera_rule": "every even global native tick, including settle and launch", "camera_specs": [capture.asdict(s) for s in specs],
        "vehicle_type": "vehicle.toyota.prius", "wheelbase_m": 2.85, "spawn_z_offset_m": .5,
        "weather": "ClearNoon", "color_seed": 0, "town": route["town"], "route_sha256": ROUTE_SHA,
        "owner_lock": owner, "controls_by_phase": {phase: capture.control_dict(phase_control(carla, phase, args.arm))
            for phase in ("settle", "launch", "coast", "tail")}, "physics_snapshots": {},
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
            raise capture.CollectionError("world settings changed before coast/brake transition configuration")
        settings.synchronous_mode, settings.fixed_delta_seconds, settings.no_rendering_mode = True, .05, False
        settings.substepping, settings.max_substep_delta_time, settings.max_substeps = True, .01, 10
        requested = dict(report["world_settings_before"], synchronous_mode=True, fixed_delta_seconds=.05,
                         no_rendering_mode=False, substepping=True, max_substep_delta_time=.01, max_substeps=10)
        report["world_settings_requested"] = requested
        if not substeps.equal_settings(substeps.settings_snapshot(settings), requested):
            raise capture.CollectionError("coast/brake transition requested settings changed an undeclared field")
        world.apply_settings(settings)
        report["weather_requested"] = weather_snapshot(carla.WeatherParameters.ClearNoon)
        world.set_weather(carla.WeatherParameters.ClearNoon)
        report["world_settings"] = substeps.settings_snapshot(world.get_settings())
        if not substeps.equal_settings(report["world_settings"], requested):
            raise capture.CollectionError("coast/brake transition settings readback mismatch")
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
        blueprint.set_attribute("role_name", "autoware_e2e_coast_brake_transition_probe")
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
            raise capture.CollectionError("cannot spawn owned coast/brake transition vehicle")
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
                raise capture.CollectionInterrupted("coast/brake transition motion probe interrupted")
            phase_tick += 1
            receipt = transport.send(phase_control(carla, phase, args.arm), phase, expected_before_frame=previous_frame)
            frame = int(world.tick(4.0))
            snapshot = world.get_snapshot()
            row = {"tick": tick, "phase": phase, "phase_tick": phase_tick, "frame": frame, "measurement_complete": False,
                   "requested_control": receipt["requested_control"], "collision": []}
            states.append(row)
            if int(snapshot.frame) != frame or frame != previous_frame + 1:
                raise capture.CollectionError("immutable snapshot/native frame stride mismatch")
            timestamp = float(snapshot.timestamp.elapsed_seconds)
            if not math.isfinite(timestamp) or (len(states) > 1 and abs(timestamp - states[-2]["timestamp"] - .05) > 1e-4):
                raise capture.CollectionError("coast/brake transition native timestamp cadence failed")
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
                raise capture.CollectionError("coast/brake transition control/wheel observation frame alignment failed")
            row["measurement_complete"] = True
            collisions, _ = events.snapshot()
            reason = safety_reason(row, sum(map(len, collisions.values())))
            if reason:
                report["safety_failure"] = reason
                raise capture.CollectionError(reason)
            previous_frame = frame
            if phase == "coast" and phase_tick == COAST_TICKS:
                report["tail_entry_check"] = {"source_tick": tick, "source_frame": frame, "source_timestamp": timestamp,
                    "source_planar_speed_mps": row["planar_speed_mps"], "maximum_speed_mps": .1,
                    "status": "PASS" if row["planar_speed_mps"] <= .1 else "FAIL", "applies_to_both_arms": True}
            next_phase = following_phase(row)
            if next_phase == "COMPLETE":
                report["completion_reason"] = "full_600_tick_coast_and_300_tick_tail_observed"
                break
            if next_phase != phase:
                if phase == "settle":
                    report["physics_snapshots"]["settled"] = physics_observation(world, ego, frame)
                report["phase_transitions"].append({"from_phase": phase, "to_phase": next_phase,
                    "source_tick": tick, "source_phase_tick": phase_tick, "source_frame": frame,
                    "source_timestamp": timestamp, "source_planar_speed_mps": row["planar_speed_mps"],
                    "effective_tick": tick + 1, "next_requested_control": capture.control_dict(phase_control(carla, next_phase, args.arm)),
                    "physical_transition_time_proven": False})
                if phase == "launch":
                    report["launch_handoff"] = dict(report["phase_transitions"][-1])
                if phase == "coast":
                    report["tail_handoff"] = dict(report["phase_transitions"][-1])
                phase, phase_tick = next_phase, 0
        if phase != "tail" or phase_tick != TAIL_TICKS:
            raise capture.CollectionError("coast/brake schedule did not complete its fixed tail duration")
        if interrupted:
            raise capture.CollectionInterrupted("coast/brake transition motion probe interrupted")
        validate_settings(world.get_settings())
        if not substeps.equal_settings(substeps.settings_snapshot(world.get_settings()), requested):
            raise capture.CollectionError("world settings drifted during coast/brake transition capture")
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
                      phase_counts={phase: sum(row["phase"] == phase for row in states) for phase in ("settle", "launch", "coast", "tail")})
        for name, rows in (("native_states.jsonl", states), ("camera_frames.jsonl", frames)):
            capture._write_jsonl(partial / name, rows)
        report["artifact_sha256"] = {name: sha256(partial / name) for name in
            ("native_states.jsonl", "native_observations.jsonl", "camera_frames.jsonl", "control_receipts.jsonl",
             "route.json", "mapping.yaml", "calibration.yaml") if (partial / name).is_file()}
        capture._write_json(partial / "manifest.json", report)
    if report["status"] != "COMPLETE":
        raise capture.CollectionError("coast/brake transition probe failed final checks; partial evidence retained")
    if output.exists() or output.is_symlink():
        raise capture.CollectionError("output appeared during capture; partial evidence retained")
    partial.rename(output)
    return output


def main():
    print(run(parse_args()))


if __name__ == "__main__":
    main()

