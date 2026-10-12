#!/usr/bin/env python3
"""HH_261001 - Change only the calibration PID target to1.58m/s; retain actual1.45–1.55m/s entry for ten ticks."""

from contextlib import contextmanager
import inspect
import math
from pathlib import Path
from types import SimpleNamespace

from scripts.e2e import calibrate_town12_j15824_brake_pulses_v3_20261001 as inherited


scout, collector, lift = inherited.scout, inherited.collector, inherited.lift
SETTLE_TICKS, DT = inherited.SETTLE_TICKS, inherited.DT
append, checked_geometry = inherited.append, inherited.checked_geometry
scalar_guards, destroy_owned = inherited.scalar_guards, inherited.destroy_owned
PulsePolicy = inherited.PulsePolicy
ORIGINAL_PROTOCOL = inherited.protocol
POLICY_ID = "town12_fresh_actor_tiny_brake_pulses_centered_target_v4"


def protocol():
    return {**ORIGINAL_PROTOCOL(), "policy_id": POLICY_ID,
        "original_pid_target_speed_mps": 1.58, "original_pid_target_speed_kmh": 5.688,
        "revision": "Only actual original PID target1.55->1.58m/s changes; exact measured[1.45,1.55]/10ticks and all other gates remain.",
        "reason": "V3 final5s actual1.43157–1.51202m/s, mean1.471413, with stable alternating throttle. Target shift is prospective, not proof.",
        "single_factor_revision_from_v3": True, "entry_gate_relaxed": False}


def run_level(directory, level, client, world, carla_map, carla, planner_class, route, receipt, owner_identity):
    from agents.navigation.basic_agent import BasicAgent
    from scripts.e2e.prepare_carla_route import serialize_route

    record = {"brake_level": level, "status": "RUNNING", "training_data_admitted": False}
    actors, previous, bounds = [], None, collector.source_motion_bounds()
    rows, decisions = directory / "native_frames.jsonl", directory / "normal_decisions.jsonl"
    transport = ego = None
    events = collector.EventRecorder()
    try:
        scout.assert_owner(*owner_identity)
        clearance = {**lift.spawn_geometry(route, lift.nominal_body()), "ground_samples": [], "clearance_rays": []}
        try:
            lift.physical_spawn_checks(world, carla, route, clearance)
        finally:
            scout.write_new(directory / "spawn_clearance.json", clearance)
        blueprint = world.get_blueprint_library().find("vehicle.toyota.prius")
        blueprint.set_attribute("role_name", "hero")
        ego = world.try_spawn_actor(blueprint, collector._carla_transform(carla, clearance["actor_spawn"]))
        scout.require(ego is not None, "single verified-pose Prius spawn attempt failed")
        actors.append(ego)
        record.update(actor_id=int(ego.id), actor_body_geometry=collector.actor_body_geometry(ego),
                      vehicle_physics_control_repr=str(ego.get_physics_control()))
        for kind, callback in (("sensor.other.collision", events.on_collision),
                               ("sensor.other.lane_invasion", events.on_lane_invasion)):
            sensor = world.spawn_actor(world.get_blueprint_library().find(kind), carla.Transform(), attach_to=ego)
            actors.append(sensor)
            sensor.listen(callback)
        transport = collector.AcknowledgedControlTransport(client, carla, ego,
            lambda: int(world.get_snapshot().frame), lambda row: append(directory / "control_receipts.jsonl", row))

        def advance(control, decision, *, enforce=True):
            nonlocal previous
            scout.assert_owner(*owner_identity)
            before = int(world.get_snapshot().frame)
            append(decisions, {**decision, "frame": before, "final_control": collector.control_dict(control)})
            ack = transport.send(control, decision["stage"], expected_before_frame=before)
            frame = world.tick(15.)
            snapshot = world.get_snapshot()
            scout.require(frame == before + 1 == int(snapshot.frame), "native tick was skipped or duplicated")
            state, raw = collector.acknowledged_snapshot_state(snapshot, ego, collector.WHEELBASE_M)
            current = ego.get_control()
            alignment = transport.compare_observation(current, ack, frame, frame, int(world.get_snapshot().frame))
            row = {"frame": frame, "timestamp": float(snapshot.timestamp.elapsed_seconds), **state, **raw,
                "speed_mps": math.hypot(state["vx"], state["vy"]),
                "applied_policy": decision, "applied_control": collector.control_dict(current),
                "control_alignment": alignment, "training_data_admitted": False}
            try:
                row.update(checked_geometry(world, carla_map, carla, ego, receipt, route, snapshot))
            except Exception as error:
                append(rows, {**row, "geometry_failure": f"{type(error).__name__}: {error}"})
                raise
            row["speed_rate_mps2"] = None
            if previous is not None:
                elapsed = row["timestamp"] - previous["timestamp"]
                scout.require(abs(elapsed - DT) < 1e-5, "raw snapshot cadence differs from 20 Hz")
                rate = (row["speed_mps"] - previous["speed_mps"]) / elapsed
                row.update(speed_rate_mps2=rate, measured_actor_center_dz_m=
                    raw["actor_snapshot_transform_carla"]["z"] - previous["actor_snapshot_transform_carla"]["z"])
            row["guard_failures"] = scalar_guards(row)
            row["events_seen"] = events.snapshot()
            append(rows, row)
            previous = row
            if enforce:
                scout.require(not row["guard_failures"], "; ".join(row["guard_failures"]))
                scout.require(not any(row["events_seen"]), "collision/lane event observed")
            return row

        for _ in range(SETTLE_TICKS):
            observed = advance(collector._blank_brake_control(carla), {"stage": "stationary_settle"})
        scout.require(observed["speed_mps"] <= .1, "fresh actor did not settle physically")
        planner = planner_class(carla_map, 1.)
        goal = collector.shift_transform_local_x(route["goal_carla_transform"], collector.WHEELBASE_M / 2)
        endpoint = collector._carla_transform(carla, goal)
        plan = planner.trace_route(ego.get_location(), endpoint.location)
        points = serialize_route(plan, endpoint)
        lift.base.validate_plan(points, route["route"])
        args = SimpleNamespace(target_speed_kmh=5.688, physics_hz=20.,
            basic_agent_base_min_distance_m=1.5, basic_agent_distance_ratio=.5)
        options, control_contract = collector.basic_agent_control_configuration(args, 1.)
        agent = BasicAgent(ego, target_speed=5.688, opt_dict=options, map_inst=carla_map, grp_inst=planner)
        agent.set_global_plan(plan)
        scout.require(agent._target_speed == 5.688 and agent.get_local_planner()._target_speed == 5.688,
                      "actual agent and local PID targets differ from 1.58 m/s")
        record["effective_target_speed_kmh"] = 5.688
        scout.require(not agent._ignore_traffic_lights and not agent._ignore_vehicles, "original hazard handling disabled")
        local = agent.get_local_planner()
        caps = collector.install_normal_brake_cap(local, 0., .20)
        original_normal = local.run_step
        policy = PulsePolicy(level)

        def normal(*args, **kwargs):
            control = original_normal(*args, **kwargs)
            snapshot = world.get_snapshot()
            velocity = snapshot.find(ego.id).get_velocity()
            return policy.apply(control, speed=math.hypot(velocity.x, velocity.y),
                timestamp=float(snapshot.timestamp.elapsed_seconds), frame=int(snapshot.frame))

        local.run_step = normal
        record.update(control_contract=control_contract, normal_caps=caps,
            original_basic_agent_source=str(Path(inspect.getsourcefile(BasicAgent)).resolve()),
            runtime_center_polyline_sha256=scout.route_digest(points), source_motion_bounds=bounds)
        pulse_rows, pulse_origin = [], None
        for _ in range(440):
            command = agent.run_step()
            decision = policy.last
            scout.require(decision is not None and decision["source_frame"] == world.get_snapshot().frame,
                          "normal planner decision missing or stale")
            expected = decision["normal_control_before_original_emergency"]
            emergency = collector.control_dict(command) != expected
            if decision["stage"] == "measured_brake_pulse" and pulse_origin is None:
                pulse_origin = previous
            observed = advance(command, {**decision, "original_agent_overrode_normal_control": emergency})
            scout.require(not emergency, "original BasicAgent emergency preserved; calibration terminated")
            if decision["stage"] == "measured_brake_pulse":
                pulse_rows.append(observed)
            completion = policy.completion(observed["speed_mps"])
            if completion:
                break
        else:
            raise scout.ScoutError("bounded calibration ticks exhausted")
        rates = [row["speed_rate_mps2"] for row in pulse_rows]
        record.update(status="PASS_MEASURED_CALIBRATION", completion=completion, entry=policy.entry,
            pulse_observed_ticks=len(pulse_rows), final_pulse_speed_mps=observed["speed_mps"],
            minimum_pulse_speed_rate_mps2=min(rates), maximum_pulse_speed_rate_mps2=max(rates),
            mean_pulse_speed_rate_mps2=(observed["speed_mps"] - policy.entry["speed_mps"]) /
                (observed["timestamp"] - policy.entry["timestamp"]),
            pulse_route_progress_m=observed["route_projection"]["progress_m"] - pulse_origin["route_projection"]["progress_m"],
            pulse_actor_center_dz_m=observed["actor_snapshot_transform_carla"]["z"] - pulse_origin["actor_snapshot_transform_carla"]["z"],
            pulse_decoder_rate_violations=sum(abs(rate) > bounds["physical_decoder"]["maximum_acceleration_mps2"]
                                            for rate in rates), stopping_distance_proven=False)
    except Exception as error:
        record.update(status="FAIL", error=f"{type(error).__name__}: {error}")
    finally:
        if ego is not None and transport is not None:
            try:
                for _ in range(60):
                    stopped = advance(collector._blank_brake_control(carla),
                        {"stage": "termination_service_brake_nontraining"}, enforce=False)
                    if stopped["speed_mps"] <= .1:
                        break
                record["cleanup_actual_speed_mps"] = stopped["speed_mps"]
            except Exception as error:
                record["cleanup_control_error"] = f"{type(error).__name__}: {error}"
        record["actor_cleanup"] = destroy_owned(world, actors)
        record["events"] = events.snapshot()
        if record["actor_cleanup"]["status"] != "PASS" or record.get("cleanup_control_error") or any(record["events"]):
            record["status"] = "FAIL"
        record["raw_files_sha256"] = {path.name: scout.digest(path) for path in directory.glob("*.jsonl")}
        scout.write_new(directory / "result.json", record)
    return record


@contextmanager
def configured_worker():
    old = inherited.run_level, inherited.protocol
    try:
        inherited.run_level, inherited.protocol = run_level, protocol
        yield
    finally:
        inherited.run_level, inherited.protocol = old


def main(argv=None):
    with configured_worker():
        return inherited.main(argv)


if __name__ == "__main__":
    raise SystemExit(main())

