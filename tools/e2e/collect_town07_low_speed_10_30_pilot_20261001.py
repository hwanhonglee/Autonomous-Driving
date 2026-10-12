"""HH_261001 - Native-only BasicAgent 4 km/h Town07 feasibility worker; no cameras or labels."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import signal
import time

from scripts.e2e import collect_carla_vad_expert as base
from scripts.e2e import carla_moving_capture as moving
from scripts.e2e import town07_low_speed_10_30_pilot_contract_20261001 as protocol


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--plan-sha256", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    if args.output.resolve() != (protocol.RUN / "episode").resolve():
        raise ValueError("worker output must equal fresh declared native pilot path")
    plan = json.loads(protocol.PLAN.read_text())
    protocol.verify_plan(plan, args.plan_sha256)
    return args, plan


def run(args, plan):
    import carla
    from agents.navigation.basic_agent import BasicAgent
    from agents.navigation.global_route_planner import GlobalRoutePlanner
    from scripts.e2e.collect_town07_throttle_isolation_20261001 import traffic_light_hazard_observation

    if args.output.exists() or args.output.is_symlink():
        raise ValueError("native pilot output must be create-only")
    partial = args.output.with_name("episode.partial")
    if partial.exists() or partial.is_symlink():
        raise ValueError("native pilot partial output must be create-only")
    if protocol.free_bytes() < protocol.FLOOR_BYTES + protocol.STARTUP_RESERVE_BYTES:
        raise ValueError("native pilot requires the 5 GiB floor plus startup reserve")
    partial.mkdir(parents=True, exist_ok=False)
    manifest = dict(schema="carla.town07_low_speed_10_30_native_pilot_run.v1",
        status="STARTED", declared_plan_sha256=args.plan_sha256,
        camera_bundles=0, model_inferences=0, training_data_approved=False,
        vehicle_control_approved=False, asynchronous_event_delivery=True,
        zero_latency_event_abort_proven=False)
    base._write_json(partial / "started.json", manifest)
    client = world = ego = transport = original_settings = original_weather = None
    actors, rows, cleanup_errors, prior_handlers = [], [], [], {}
    failure = assessment = None

    def interrupted(signum, _frame):
        raise base.CollectionInterrupted("native pilot interrupted by signal " + str(signum))

    def persist(name, record):
        with (partial / name).open("a") as stream:
            stream.write(json.dumps(record, sort_keys=True, allow_nan=False) + "\n")

    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            prior_handlers[signum] = signal.signal(signum, interrupted)
        route = base.load_route(protocol.ROUTE)
        if route["town"] != "Town07":
            raise ValueError("pinned route town differs")
        client = carla.Client("127.0.0.1", 2440)
        client.set_timeout(20.0)
        world = client.get_world()
        base._preflight_exclusive(world)
        base._preflight_no_walkers(world)
        carla_map = world.get_map()
        if carla_map.name.rsplit("/", 1)[-1] != "Town07":
            raise ValueError("owned server did not cold-start Town07")
        map_check, verified_xodr = base.verify_live_opendrive_sha256(carla_map, protocol.XODR_SHA256)
        manifest["live_opendrive_preflight"] = map_check
        original_settings, original_weather = world.get_settings(), world.get_weather()
        if (original_settings.substepping is not True
                or original_settings.max_substep_delta_time != .01
                or original_settings.max_substeps != 10
                or original_settings.no_rendering_mode is not False):
            raise ValueError("inherited rendered 20 Hz physics baseline differs")
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = protocol.TICK_SECONDS
        world.apply_settings(settings)
        applied = world.get_settings()
        if (not applied.synchronous_mode or applied.fixed_delta_seconds != protocol.TICK_SECONDS
                or applied.substepping is not True or applied.max_substep_delta_time != .01
                or applied.max_substeps != 10 or applied.no_rendering_mode is not False):
            raise ValueError("native pilot world-setting readback differs")
        manifest["physics"] = dict(before=base._settings_dict(original_settings),
            applied=base._settings_dict(applied), native_hz=20)
        world.set_weather(carla.WeatherParameters.ClearNoon)
        if hasattr(world, "set_pedestrians_seed"):
            world.set_pedestrians_seed(2026100101)
        blueprints = world.get_blueprint_library()
        blueprint = blueprints.find("vehicle.toyota.prius")
        if blueprint.has_attribute("role_name"):
            blueprint.set_attribute("role_name", "autoware_e2e_low_speed_pilot")
        rear_start = base.shift_transform_local_x(route["start_carla_transform"], 2.85 / 2.)
        ego = world.try_spawn_actor(blueprint,
            base._carla_transform(carla, base.apply_spawn_z_offset(rear_start, .5)))
        if ego is None:
            raise base.CollectionError("pinned Prius spawn failed")
        actors.append(ego)
        manifest["actor_body_geometry"] = base.actor_body_geometry(ego)
        events = base.EventRecorder()
        for sensor_type, callback in (("sensor.other.collision", events.on_collision),
                                      ("sensor.other.lane_invasion", events.on_lane_invasion)):
            sensor = world.spawn_actor(blueprints.find(sensor_type), carla.Transform(),
                attach_to=ego, attachment_type=carla.AttachmentType.Rigid)
            sensor.listen(callback)
            actors.append(sensor)
        map_record = base.archive_map_opendrive(carla_map, partial, verified_payload=verified_xodr)
        if map_record["sha256"] != protocol.XODR_SHA256:
            raise base.CollectionError("archived live OpenDRIVE differs")
        manifest["active_map_opendrive"] = map_record
        planner = GlobalRoutePlanner(carla_map, float(route.get("sampling_resolution_m", 1.)))
        rear_goal = base.shift_transform_local_x(route["goal_carla_transform"], 2.85 / 2.)
        global_plan = planner.trace_route(base._carla_location(carla, rear_start),
                                          base._carla_location(carla, rear_goal))
        if len(global_plan) < 2:
            raise base.CollectionError("pinned BasicAgent route plan is empty")
        projector = base.RouteProjector(route)
        transport = base.AcknowledgedControlTransport(client, carla, ego,
            lambda: world.get_snapshot().frame,
            lambda receipt: persist("control_receipts.jsonl", receipt))
        bootstrap = transport.send(base._blank_brake_control(carla), "bootstrap_full_brake")
        bootstrap_frame = int(world.tick(20.0))
        if world.get_snapshot().frame != bootstrap_frame:
            raise base.CollectionError("bootstrap native frame/snapshot differs")
        bootstrap_alignment = transport.compare_observation(ego.get_control(), bootstrap,
            bootstrap_frame, world.get_snapshot().frame, world.get_snapshot().frame)
        if bootstrap_alignment["status"] != "PASS":
            raise base.CollectionError("bootstrap ACK/readback differs")
        options_args = argparse.Namespace(basic_agent_base_min_distance_m=2.,
            basic_agent_distance_ratio=.2, basic_agent_lateral_kp=1.95,
            basic_agent_lateral_ki=.05, basic_agent_lateral_kd=.2,
            basic_agent_max_steering=.8, basic_agent_lane_offset_m=0.)
        options, options_receipt = base.basic_agent_control_configuration(
            options_args, float(route.get("sampling_resolution_m", 1.)))
        agent = BasicAgent(ego, target_speed=protocol.TARGET_KMH, opt_dict=options,
            map_inst=carla_map, grp_inst=planner)
        agent.set_global_plan(global_plan)
        normal_cap = base.install_normal_brake_cap(agent.get_local_planner(), 0., .20)
        manifest["expert_control"] = dict(options=options_receipt, normal_cap=normal_cap,
            BasicAgent_target_kmh=protocol.TARGET_KMH, emergency_unchanged=True)
        # HH_261001 - Match V3's 70 held-brake native ticks after its one unrecorded bootstrap tick.
        transport.send(base._blank_brake_control(carla), "stationary_warmup_start", bootstrap_frame)
        previous_frame, previous_progress = bootstrap_frame, 0.
        warmup_first_xy = None
        for warmup_index in range(protocol.WARMUP_TICKS):
            if protocol.free_bytes() < protocol.FLOOR_BYTES + protocol.CLOSING_RESERVE_BYTES:
                raise base.CollectionError("native disk floor before warmup tick")
            expected = transport.receipts[-1]
            frame = int(world.tick(20.0))
            snapshot = world.get_snapshot()
            if snapshot.frame != frame or frame != previous_frame + 1:
                raise base.CollectionError("warmup native frame/snapshot differs")
            state, raw = base.acknowledged_snapshot_state(snapshot, ego, 2.85)
            projection = projector.project(state["x"], state["y"], previous_progress)
            previous_progress = projection.progress_m
            raw_progress = moving.measured_progress(route["route"], state["x"], state["y"])
            collisions, lane_events = events.snapshot()
            collision_count, lane_count = sum(map(len, collisions.values())), sum(map(len, lane_events.values()))
            light = traffic_light_hazard_observation(world, agent, raw["world_velocity_carla"], frame)
            observed_control = ego.get_control()
            alignment = transport.compare_observation(observed_control, expected, frame,
                world.get_snapshot().frame, world.get_snapshot().frame)
            speed = math.hypot(*raw["world_velocity_carla"][:2])
            free = protocol.free_bytes()
            if warmup_first_xy is None:
                warmup_first_xy = (state["x"], state["y"])
            drift = math.hypot(state["x"] - warmup_first_xy[0], state["y"] - warmup_first_xy[1])
            record = dict(schema="carla.town07_low_speed_10_30_native_state.v1",
                phase="stationary_warmup", warmup_index=warmup_index,
                frame=frame, timestamp=float(snapshot.timestamp.elapsed_seconds),
                route_progress_m=raw_progress, route_cte_m=projection.cross_track_error_m,
                speed_mps=speed, current_control=base.control_dict(observed_control),
                control_alignment=alignment, collision_count=collision_count,
                lane_invasion_count=lane_count, traffic_light_hazard=light,
                observed_free_disk_bytes=free, **state, **raw)
            rows.append(record)
            persist("states.jsonl", record)
            if (alignment["status"] != "PASS" or collision_count or lane_count or light["hazard"]
                    or not math.isfinite(speed) or speed > .05
                    or abs(raw_progress - protocol.WARMUP_REFERENCE_PROGRESS_M) > protocol.WARMUP_PROGRESS_TOLERANCE_M
                    or drift > protocol.WARMUP_MAX_PLANAR_DISPLACEMENT_M
                    or free < protocol.FLOOR_BYTES + protocol.CLOSING_RESERVE_BYTES):
                raise base.CollectionError("settled 70-tick warmup failed at frame " + str(frame))
            previous_frame = frame
            transport.send(base._blank_brake_control(carla), "stationary_warmup_hold", frame)
        initial = agent.run_step()
        if float(initial.brake) > 0.05:
            raise base.CollectionError("BasicAgent emergency at initial proposal")
        transport.send(initial, "pilot_initial_control", previous_frame)
        for tick in range(1, protocol.MAX_TICKS + 1):
            if protocol.free_bytes() < protocol.FLOOR_BYTES + protocol.CLOSING_RESERVE_BYTES:
                raise base.CollectionError("native disk floor before world tick")
            expected = transport.receipts[-1]
            frame = int(world.tick(20.0))
            snapshot = world.get_snapshot()
            if snapshot.frame != frame:
                raise base.CollectionError("native frame/snapshot differs")
            state, raw = base.acknowledged_snapshot_state(snapshot, ego, 2.85)
            projection = projector.project(state["x"], state["y"], previous_progress)
            previous_progress = projection.progress_m
            raw_progress = moving.measured_progress(route["route"], state["x"], state["y"])
            collisions, lane_events = events.snapshot()
            collision_count = sum(map(len, collisions.values()))
            lane_count = sum(map(len, lane_events.values()))
            light = traffic_light_hazard_observation(world, agent, raw["world_velocity_carla"], frame)
            observed_control = ego.get_control()
            alignment = transport.compare_observation(observed_control, expected, frame,
                world.get_snapshot().frame, world.get_snapshot().frame)
            speed = math.hypot(*raw["world_velocity_carla"][:2])
            free = protocol.free_bytes()
            record = dict(schema="carla.town07_low_speed_10_30_native_state.v1",
                phase="driving", warmup_index=None,
                frame=frame, timestamp=float(snapshot.timestamp.elapsed_seconds),
                route_progress_m=raw_progress,
                route_cte_m=projection.cross_track_error_m,
                speed_mps=speed, current_control=base.control_dict(observed_control),
                control_alignment=alignment, collision_count=collision_count,
                lane_invasion_count=lane_count, traffic_light_hazard=light,
                observed_free_disk_bytes=free, **state, **raw)
            rows.append(record)
            persist("states.jsonl", record)
            prior = rows[-2]
            rate = (speed - prior["speed_mps"]) / (record["timestamp"] - prior["timestamp"])
            if (frame != previous_frame + 1 or alignment["status"] != "PASS"
                    or collision_count or lane_count or light["hazard"]
                    or not math.isfinite(speed) or speed > 2.9
                    or not math.isfinite(rate) or abs(rate) > protocol.GLOBAL_EMERGENCY_RATE_MPS2
                    or raw_progress + 1e-8 < prior["route_progress_m"]
                    or abs(projection.cross_track_error_m) > 1.
                    or free < protocol.FLOOR_BYTES + protocol.CLOSING_RESERVE_BYTES):
                raise base.CollectionError("pilot safety gate failed at frame " + str(frame))
            previous_frame = frame
            if raw_progress > protocol.WINDOW_HI_M:
                transport.send(base._blank_brake_control(carla), "post_window_cleanup_full_brake", frame)
                assessment = protocol.assess_native_window(rows)
                break
            if tick == protocol.MAX_TICKS or agent.done():
                raise base.CollectionError("pilot failed to close fixed 10–30 m window")
            proposal = agent.run_step()
            if float(proposal.brake) > 0.05:
                raise base.CollectionError("BasicAgent emergency before fixed 30 m closure")
            transport.send(proposal, "pilot_next_control", frame)
        if assessment is None:
            raise base.CollectionError("native pilot did not produce fixed assessment")
        time.sleep(.2)
        late_collisions, late_lanes = events.snapshot()
        manifest["late_event_counts"] = dict(collision=sum(map(len, late_collisions.values())),
            lane_invasion=sum(map(len, late_lanes.values())))
        if any(manifest["late_event_counts"].values()):
            raise base.CollectionError("late asynchronous collision/lane event")
    except BaseException as error:
        failure = f"{type(error).__name__}: {error}"
        try:
            if ego is not None and world is not None and transport is not None:
                transport.send(base._blank_brake_control(carla), "failure_cleanup_full_brake",
                    world.get_snapshot().frame)
        except Exception as cleanup_error:
            cleanup_errors.append("failure brake: " + str(cleanup_error))
    finally:
        for signum, handler in prior_handlers.items():
            signal.signal(signum, handler)
        for actor in reversed(actors):
            base._destroy_actor(actor, cleanup_errors)
        if world is not None and original_weather is not None:
            try:
                world.set_weather(original_weather)
            except Exception as error:
                cleanup_errors.append("restore weather: " + str(error))
        if world is not None and original_settings is not None:
            try:
                world.apply_settings(original_settings)
            except Exception as error:
                cleanup_errors.append("restore settings: " + str(error))
        manifest.update(status="FAILED_RETAINED" if failure or cleanup_errors or not assessment
            or assessment["status"] != "PASS_FEASIBILITY_ONLY" else "PASS_FEASIBILITY_ONLY",
            failure=failure, cleanup_errors=cleanup_errors, native_states=len(rows),
            assessment=assessment, camera_bundles=0, no_camera_files=True)
        base._write_json(partial / "manifest.json", manifest)
    return 0 if manifest["status"] == "PASS_FEASIBILITY_ONLY" else 1


def main(argv=None):
    args, plan = parse_args(argv)
    return run(args, plan)


if __name__ == "__main__":
    raise SystemExit(main())
