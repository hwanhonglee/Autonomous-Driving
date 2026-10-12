"""HH_261001 - V3 native-only CARLA worker with relative settled-start checks."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import signal
import sys
import time

from scripts.e2e import collect_carla_vad_expert as base
from scripts.e2e import town07_throttle_isolation_contract_v3_20261001 as protocol


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--trial", type=int, required=True)
    parser.add_argument("--plan-sha256", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    spec = protocol.trial(args.trial)
    if args.output.resolve() != (Path(spec["output"]) / "episode").resolve():
        raise ValueError("worker output differs from the fixed trial location")
    plan = json.loads(protocol.PLAN.read_text())
    protocol.verify_plan(plan, expected_sha256=args.plan_sha256)
    return args, spec, plan


def scripted_control(carla, throttle):
    if throttle not in protocol.THROTTLES.values():
        raise ValueError("undeclared intervention throttle")
    return carla.VehicleControl(throttle=throttle, steer=0.0, brake=0.0,
                                hand_brake=False, reverse=False)


def traffic_light_hazard_observation(world, agent, world_velocity, frame):
    """HH_261001 - Observe the source-pinned BasicAgent red-light guard without advancing its planner."""
    if int(world.get_snapshot().frame) != frame:
        raise base.CollectionError("traffic-light observation started on a different frame")
    speed = math.sqrt(sum(float(component) ** 2 for component in world_velocity))
    if not math.isfinite(speed):
        raise base.CollectionError("nonfinite world speed in traffic-light observation")
    threshold = agent._base_tlight_threshold + agent._speed_ratio * speed
    affected, light = agent._affected_by_traffic_light(agent._lights_list, threshold)
    if int(world.get_snapshot().frame) != frame:
        raise base.CollectionError("traffic-light observation crossed CARLA frames")
    return dict(frame=frame, hazard=bool(affected), light_actor_id=int(light.id) if light else None,
        light_state=str(light.state) if light else None, threshold_m=threshold,
        source="pinned BasicAgent._affected_by_traffic_light", planner_advanced=False)


def summarize(rows, spec):
    """HH_261001 - Recount only native post-branch states, not setup or cleanup braking."""
    branch = [row for row in rows if row["phase"] == "intervention"]
    if len(branch) != protocol.INTERVENTION_TICKS or [row["intervention_tick"] for row in branch] != list(range(1, 61)):
        raise ValueError("incomplete fixed intervention window")
    preceding = next(row for row in reversed(rows) if row["phase"] == "branch")
    interval = [preceding, *branch]
    rates = []
    for left, right in zip(interval, interval[1:]):
        dt = right["timestamp"] - left["timestamp"]
        if not math.isclose(dt, protocol.TICK_SECONDS, abs_tol=1e-6, rel_tol=0):
            raise ValueError("native timestamp cadence differs")
        rates.append((right["vx"] - left["vx"]) / dt)
    first_stop = next((row["timestamp"] - preceding["timestamp"] for row in branch if abs(row["vx"]) <= 0.05), None)
    return dict(schema="carla.town07_throttle_isolation_summary.v3", trial_id=spec["trial_id"],
        arm=spec["arm"], site_progress_m=spec["site_progress_m"],
        branch_frame=preceding["frame"], branch_progress_m=preceding["route_progress_m"],
        branch_speed_mps=preceding["vx"], observed_ticks=len(branch),
        signed_rate_below_minus_8_mps2_count=sum(rate < -8. for rate in rates),
        minimum_signed_rate_mps2=min(rates),
        net_route_progress_m=branch[-1]["route_progress_m"] - preceding["route_progress_m"],
        first_speed_at_or_below_0p05_s=first_stop,
        final_speed_mps=branch[-1]["vx"], camera_bundles=0, model_inferences=0,
        training_data_approved=False, vehicle_control_approved=False,
        physical_actuation_independently_proven=False)


def run(args, spec, plan):
    import carla
    from agents.navigation.basic_agent import BasicAgent
    from agents.navigation.global_route_planner import GlobalRoutePlanner

    if args.output.exists() or args.output.is_symlink():
        raise ValueError("fresh worker output required")
    partial = args.output.with_name("episode.partial")
    if partial.exists() or partial.is_symlink():
        raise ValueError("fresh partial worker output required")
    if protocol.free_bytes() < protocol.FLOOR_BYTES + protocol.TRIAL_RESERVE_BYTES:
        raise ValueError("native trial lacks declared startup disk reserve")
    partial.mkdir(parents=True, exist_ok=False)
    rows = []
    manifest = dict(schema="carla.town07_native_throttle_isolation_run.v3", status="STARTED",
        fixed_plan_sha256=args.plan_sha256, trial=spec, native_states=0, camera_bundles=0,
        model_inferences=0, training_data_approved=False, vehicle_control_approved=False,
        asset_sha256=plan["asset_sha256"], asynchronous_event_delivery=True,
        zero_latency_event_abort_proven=False)
    base._write_json(partial / "started.json", manifest)
    client = world = ego = transport = original_settings = original_weather = None
    actors = []
    cleanup_errors = []
    prior_handlers = {}
    failure = None
    summary = None

    def interrupted(signum, _frame):
        raise base.CollectionInterrupted("native trial interrupted by signal " + str(signum))

    def persist_receipt(receipt):
        with (partial / "control_receipts.jsonl").open("a") as stream:
            stream.write(json.dumps(receipt, sort_keys=True, allow_nan=False) + "\n")

    def persist_state(record):
        rows.append(record)
        with (partial / "states.jsonl").open("a") as stream:
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
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = protocol.TICK_SECONDS
        world.apply_settings(settings)
        manifest["physics"] = dict(original=base._settings_dict(original_settings),
            applied=base._settings_dict(world.get_settings()), substep_profile="inherited")
        if not world.get_settings().synchronous_mode or world.get_settings().fixed_delta_seconds != protocol.TICK_SECONDS:
            raise ValueError("native 20 Hz world settings differ")
        world.set_weather(carla.WeatherParameters.ClearNoon)
        if hasattr(world, "set_pedestrians_seed"):
            world.set_pedestrians_seed(2026100101)
        blueprints = world.get_blueprint_library()
        vehicle_blueprint = blueprints.find("vehicle.toyota.prius")
        if vehicle_blueprint.has_attribute("role_name"):
            vehicle_blueprint.set_attribute("role_name", "autoware_e2e_throttle_isolation")
        center_start = base.shift_transform_local_x(route["start_carla_transform"], 2.85 / 2.)
        actor_spawn = base.apply_spawn_z_offset(center_start, .5)
        ego = world.try_spawn_actor(vehicle_blueprint, base._carla_transform(carla, actor_spawn))
        if ego is None:
            raise base.CollectionError("pinned Town07 Prius spawn failed")
        actors.append(ego)
        manifest["actor_body_geometry"] = base.actor_body_geometry(ego)
        events = base.EventRecorder()
        for sensor_type, callback in (("sensor.other.collision", events.on_collision),
                                      ("sensor.other.lane_invasion", events.on_lane_invasion)):
            sensor = world.spawn_actor(blueprints.find(sensor_type), carla.Transform(), attach_to=ego,
                                       attachment_type=carla.AttachmentType.Rigid)
            sensor.listen(callback)
            actors.append(sensor)
        map_record = base.archive_map_opendrive(carla_map, partial, verified_payload=verified_xodr)
        if map_record.get("sha256") != protocol.XODR_SHA256:
            raise base.CollectionError("archived active map differs")
        manifest["active_map_opendrive"] = map_record
        planner = GlobalRoutePlanner(carla_map, float(route.get("sampling_resolution_m", 1.)))
        goal_center = base.shift_transform_local_x(route["goal_carla_transform"], 2.85 / 2.)
        global_plan = planner.trace_route(base._carla_location(carla, center_start),
                                          base._carla_location(carla, goal_center))
        if len(global_plan) < 2:
            raise base.CollectionError("pinned BasicAgent route plan is empty")
        projector = base.RouteProjector(route)
        transport = base.AcknowledgedControlTransport(client, carla, ego,
                    lambda: world.get_snapshot().frame, persist_receipt)
        bootstrap = transport.send(base._blank_brake_control(carla), "bootstrap_full_brake")
        bootstrap_frame = int(world.tick(20.0))
        bootstrap_snapshot = world.get_snapshot()
        if bootstrap_snapshot.frame != bootstrap_frame:
            raise base.CollectionError("bootstrap snapshot/frame mismatch")
        bootstrap_state, bootstrap_raw = base.acknowledged_snapshot_state(bootstrap_snapshot, ego, 2.85)
        bootstrap_alignment = transport.compare_observation(ego.get_control(), bootstrap, bootstrap_frame,
            world.get_snapshot().frame, world.get_snapshot().frame)
        if bootstrap_alignment["status"] != "PASS":
            raise base.CollectionError("bootstrap ACK/readback differs")
        manifest["bootstrap_observation"] = dict(frame=bootstrap_frame,
            timestamp=float(bootstrap_snapshot.timestamp.elapsed_seconds),
            current_control=base.control_dict(ego.get_control()), alignment=bootstrap_alignment,
            **bootstrap_state, **bootstrap_raw)

        # HH_261001 - Only BasicAgent's normal LocalPlanner control is capped; emergency returns remain unchanged.
        control_args = argparse.Namespace(basic_agent_base_min_distance_m=2., basic_agent_distance_ratio=.2,
            basic_agent_lateral_kp=1.95, basic_agent_lateral_ki=.05, basic_agent_lateral_kd=.2,
            basic_agent_max_steering=.8, basic_agent_lane_offset_m=0.)
        options, options_receipt = base.basic_agent_control_configuration(control_args,
            float(route.get("sampling_resolution_m", 1.)))
        agent = BasicAgent(ego, target_speed=9., opt_dict=options,
            map_inst=carla_map, grp_inst=planner)
        agent.set_global_plan(global_plan)
        cap = base.install_normal_brake_cap(agent.get_local_planner(), 0., .20)
        manifest["prefix_controller"] = dict(options=options_receipt, normal_cap=cap,
            emergency_unchanged=True, branch_does_not_call_model=True)
        previous_frame = bootstrap_frame
        bootstrap_progress_m = projector.project(bootstrap_state["x"], bootstrap_state["y"], 0.).progress_m
        previous_progress = bootstrap_progress_m
        manifest["bootstrap_observation"]["route_progress_m"] = bootstrap_progress_m
        manifest["stationary_warmup"] = dict(required_ticks=protocol.WARMUP_TICKS,
            required_seconds=protocol.WARMUP_TICKS*protocol.TICK_SECONDS,
            command="ACKed full service brake each native tick", observed_ticks=0)
        transport.send(base._blank_brake_control(carla), "stationary_warmup_start", bootstrap_frame)
        for warmup_tick in range(1, protocol.WARMUP_TICKS + 1):
            if protocol.free_bytes() < protocol.FLOOR_BYTES + protocol.CLOSING_RESERVE_BYTES:
                raise base.CollectionError("native disk floor guard before warmup tick")
            expected = transport.receipts[-1]
            frame = int(world.tick(20.0))
            snapshot = world.get_snapshot()
            if snapshot.frame != frame:
                raise base.CollectionError("warmup frame/snapshot mismatch")
            state, raw = base.acknowledged_snapshot_state(snapshot, ego, 2.85)
            projection = projector.project(state["x"], state["y"], previous_progress)
            previous_progress = projection.progress_m
            collision_events, lane_events = events.snapshot()
            collision_count = sum(map(len, collision_events.values()))
            lane_count = sum(map(len, lane_events.values()))
            traffic_light = traffic_light_hazard_observation(world, agent,
                raw["world_velocity_carla"], frame)
            before_control_frame = world.get_snapshot().frame
            observed_control = ego.get_control()
            alignment = transport.compare_observation(observed_control, expected, frame,
                before_control_frame, world.get_snapshot().frame)
            failures = protocol.safety_failures(frame=frame, previous_frame=previous_frame,
                vx=state["vx"], cte_m=projection.cross_track_error_m,
                collision_count=collision_count, lane_event_count=lane_count,
                traffic_light_hazard=traffic_light["hazard"],
                alignment_ok=alignment["status"] == "PASS", free_bytes=protocol.free_bytes())
            displacement = math.hypot(state["x"]-bootstrap_state["x"],
                state["y"]-bootstrap_state["y"])
            if warmup_tick == protocol.WARMUP_TICKS:
                failures += protocol.warmup_final_failures(vx=state["vx"],
                    bootstrap_progress_m=bootstrap_progress_m,
                    route_progress_m=projection.progress_m,
                    planar_displacement_m=displacement)
            row = dict(schema="carla.town07_throttle_isolation_native_state.v3",
                phase="stationary_warmup", warmup_tick=warmup_tick,
                frame=frame, timestamp=float(snapshot.timestamp.elapsed_seconds),
                intervention_tick=None, route_progress_m=projection.progress_m,
                route_cte_m=projection.cross_track_error_m,
                bootstrap_route_progress_m=bootstrap_progress_m,
                route_progress_delta_from_bootstrap_m=projection.progress_m - bootstrap_progress_m,
                bootstrap_planar_displacement_m=displacement,
                current_control=base.control_dict(observed_control), control_alignment=alignment,
                collision_count=collision_count, lane_invasion_count=lane_count,
                traffic_light_hazard=traffic_light,
                observed_free_disk_bytes=protocol.free_bytes(), safety_failures=failures,
                **state, **raw)
            persist_state(row)
            manifest["stationary_warmup"]["observed_ticks"] = warmup_tick
            previous_frame = frame
            if failures:
                raise base.CollectionError("fixed warmup safety abort: " + ",".join(failures))
            if warmup_tick < protocol.WARMUP_TICKS:
                transport.send(base._blank_brake_control(carla), "stationary_warmup_hold", frame)
        # HH_261001 - Drain late callbacks before the first driving command; asynchronous detection remains bounded, not instantaneous.
        time.sleep(0.2)
        warmup_collision, warmup_lane = events.snapshot()
        warmup_counts = dict(collision=sum(map(len, warmup_collision.values())),
            lane_invasion=sum(map(len, warmup_lane.values())))
        manifest["stationary_warmup"]["final_observed_event_counts"] = warmup_counts
        if any(warmup_counts.values()):
            raise base.CollectionError("asynchronous event observed during stationary warmup")
        command = agent.run_step()
        if float(command.brake) > 0.05:
            raise base.CollectionError("BasicAgent requested emergency brake before prefix")
        transport.send(command, "prefix_first_control", previous_frame)
        branch_frame = None

        for tick in range(1, protocol.PREFIX_MAX_TICKS + protocol.INTERVENTION_TICKS + 1):
            if protocol.free_bytes() < protocol.FLOOR_BYTES + protocol.CLOSING_RESERVE_BYTES:
                raise base.CollectionError("native disk floor guard before world tick")
            expected = transport.receipts[-1]
            frame = int(world.tick(20.0))
            snapshot = world.get_snapshot()
            if snapshot.frame != frame:
                raise base.CollectionError("native frame/snapshot mismatch")
            state, raw = base.acknowledged_snapshot_state(snapshot, ego, 2.85)
            projection = projector.project(state["x"], state["y"], previous_progress)
            previous_progress = projection.progress_m
            c, lane = events.snapshot()
            collision_count, lane_count = sum(map(len, c.values())), sum(map(len, lane.values()))
            traffic_light = traffic_light_hazard_observation(world, agent,
                raw["world_velocity_carla"], frame)
            before_control_frame = world.get_snapshot().frame
            observed_control = ego.get_control()
            alignment = transport.compare_observation(observed_control, expected, frame,
                before_control_frame, world.get_snapshot().frame)
            failures = protocol.safety_failures(frame=frame, previous_frame=previous_frame,
                vx=state["vx"], cte_m=projection.cross_track_error_m,
                collision_count=collision_count, lane_event_count=lane_count,
                traffic_light_hazard=traffic_light["hazard"],
                alignment_ok=alignment["status"] == "PASS", free_bytes=protocol.free_bytes())
            previous_frame = frame
            if branch_frame is None:
                phase = "prefix"
                branch = protocol.branch_status(projection.progress_m, state["vx"], spec["site_progress_m"])
                if branch == "START":
                    phase = "branch"
            else:
                phase = "intervention"
                branch = "ALREADY_STARTED"
            row = dict(schema="carla.town07_throttle_isolation_native_state.v3", phase=phase,
                frame=frame, timestamp=float(snapshot.timestamp.elapsed_seconds),
                intervention_tick=frame - branch_frame if branch_frame is not None else None,
                route_progress_m=projection.progress_m, route_cte_m=projection.cross_track_error_m,
                branch_status=branch, current_control=base.control_dict(observed_control),
                control_alignment=alignment, collision_count=collision_count,
                lane_invasion_count=lane_count, traffic_light_hazard=traffic_light,
                observed_free_disk_bytes=protocol.free_bytes(),
                safety_failures=failures, **state, **raw)
            persist_state(row)
            if failures:
                raise base.CollectionError("fixed safety abort: " + ",".join(failures))
            if branch.startswith("FAIL_"):
                raise base.CollectionError(branch)
            if phase == "branch":
                # HH_261001 - A fresh BasicAgent call checks an unexpected emergency before the scripted branch.
                proposal = agent.run_step()
                if float(proposal.brake) > 0.05:
                    raise base.CollectionError("BasicAgent emergency at intervention branch")
                branch_frame = frame
                transport.send(scripted_control(carla, spec["throttle"]), "intervention_start", frame)
            elif phase == "prefix":
                if tick >= protocol.PREFIX_MAX_TICKS or agent.done():
                    raise base.CollectionError("fixed prefix budget or BasicAgent route end before branch")
                proposal = agent.run_step()
                if float(proposal.brake) > 0.05:
                    raise base.CollectionError("BasicAgent emergency during matched prefix")
                transport.send(proposal, "prefix_control", frame)
            elif row["intervention_tick"] < protocol.INTERVENTION_TICKS:
                transport.send(scripted_control(carla, spec["throttle"]), "intervention_hold", frame)
            else:
                # HH_261001 - Cleanup full brake occurs after the counted 60 ticks and is never a training target.
                transport.send(base._blank_brake_control(carla), "post_intervention_cleanup_brake", frame)
                summary = summarize(rows, spec)
                break
        if summary is None:
            raise base.CollectionError("fixed 60-tick intervention did not complete")
        # HH_261001 - Allow delivered asynchronous events to invalidate a nominally finished 60-tick window.
        time.sleep(0.2)
        late_collision, late_lane = events.snapshot()
        final_counts = dict(collision=sum(map(len, late_collision.values())),
            lane_invasion=sum(map(len, late_lane.values())))
        manifest["final_observed_event_counts"] = final_counts
        if any(final_counts.values()):
            raise base.CollectionError("asynchronous event observed after intervention window")
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
        manifest.update(status="FAILED_RETAINED" if failure or cleanup_errors else "FINISHED_DIAGNOSTIC_ONLY",
            native_states=len(rows), failure=failure, cleanup_errors=cleanup_errors,
            summary=summary, no_camera_files=True)
        base._write_json(partial / "manifest.json", manifest)
    return 0 if not failure and not cleanup_errors else 1


def main(argv=None):
    args, spec, plan = parse_args(argv)
    return run(args, spec, plan)


if __name__ == "__main__":
    raise SystemExit(main())
