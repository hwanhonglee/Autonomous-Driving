#!/usr/bin/env python3
"""HH_261001 - Four fresh owned vehicles measure tiny brake pulses; never produce TRAIN labels."""

from __future__ import annotations

import argparse
import hashlib
import inspect
import json
import math
from pathlib import Path
from types import SimpleNamespace

from scripts.e2e import capture_town12_j15824_spawn_lift_20261001 as lift
from scripts.e2e.collect_town07_low_speed_start_20261001 import LaunchControl


scout, collector = lift.scout, lift.collector
ROOT = lift.ROOT
LEVELS = (0., .0001, .001, .005)
PULSE_TICKS = 40
SETTLE_TICKS = 40
DT = .05
POLICY_ID = "town12_fresh_actor_tiny_brake_pulses_v1"


def protocol():
    return {"policy_id": POLICY_ID, "training_data_admitted": False, "future_labels_created": False,
        "six_camera_capture": False, "physics_hz": 20., "brake_levels": list(LEVELS),
        "fresh_owned_actor_per_level": True, "spawn_attempts_per_level": 1,
        "settle_ticks": SETTLE_TICKS, "settle_brake": 1., "target_entry_speed_mps": 2.4,
        "entry_band_mps": [2.35, 2.45], "consecutive_entry_band_ticks": 10,
        "launch_throttle": .13, "launch_handoff_mps": .5, "post_handoff_ramp_per_second": .05,
        "normal_throttle_cap": .20, "launch_normal_brake": 0., "maximum_launch_seconds": 20.,
        "pulse_ticks": PULSE_TICKS, "pulse_throttle": 0., "early_pulse_speed_floor_mps": .5,
        "minimum_all_body_corner_stop_distance_m": 40., "maximum_route_progress_m": 30.,
        "maximum_speed_mps": 2.55, "required_road_section_lane": [588, 0, 1],
        "minimum_instantaneous_body_lane_margin_m": .10,
        "base_min_distance_m": 1.5, "distance_ratio_s": .5,
        "original_basic_agent_red_emergency_unchanged": True, "traffic_light_states_written": False,
        "termination": "Full service brake is explicitly non-training cleanup, after pulse/abort; all its ticks retained.",
        "limitations": ["Fixed level order and a narrow entry-speed band are not randomized equal-state A/B.",
            "Two-second pulses do not measure a complete stopping distance or prove signal stopping safety.",
            "Upstream launch-site slope is not the later signal approach or route endpoint slope.",
            "Instantaneous body-corner guards are not swept-body lane qualification."]}


def append(path, value):
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(value, sort_keys=True, allow_nan=False) + "\n")
        stream.flush()


class PulsePolicy:
    """HH_261001 - The original planner still supplies steering; intervention precedes its agent emergency return."""
    def __init__(self, level):
        scout.require(level in LEVELS, "undeclared brake level")
        self.level, self.launch = level, LaunchControl()
        self.started_at = self.previous_time = None
        self.stable_ticks = self.pulse_ticks = 0
        self.entry = self.last = None

    def apply(self, control, *, speed, timestamp, frame):
        scout.require(math.isfinite(speed) and 0. <= speed <= 2.55 and math.isfinite(timestamp),
                      "invalid or excessive measured calibration speed/time")
        if self.previous_time is not None:
            scout.require(abs(timestamp - self.previous_time - DT) < 1e-5, "non-native calibration policy cadence")
        self.previous_time = timestamp
        if self.started_at is None:
            self.started_at = timestamp
        scout.require(float(control.brake) == 0., "normal planner brake was not capped before calibration")
        raw = collector.control_dict(control)
        if self.entry is None:
            scout.require(timestamp - self.started_at < 20., "entry-speed stabilization timed out")
            self.stable_ticks = self.stable_ticks + 1 if 2.35 <= speed <= 2.45 else 0
            if self.stable_ticks >= 10:
                self.entry = {"frame": int(frame), "timestamp": timestamp, "speed_mps": speed}
        if self.entry is None:
            control = self.launch.apply(control, speed=speed, timestamp=timestamp, frame=frame,
                                        normal_proposal=raw)
            stage = "gentle_launch_and_stabilize"
        else:
            scout.require(self.pulse_ticks < PULSE_TICKS and speed > .5,
                          "pulse called after its bounded completion")
            control.throttle, control.brake = 0., self.level
            self.pulse_ticks += 1
            stage = "measured_brake_pulse"
        self.last = {"stage": stage, "source_frame": int(frame), "source_timestamp": timestamp,
            "measured_speed_mps": speed, "brake_level": self.level, "pulse_command_number": self.pulse_ticks,
            "entry": self.entry, "normal_planner_before_intervention": raw,
            "normal_control_before_original_emergency": collector.control_dict(control),
            "launch": self.launch.last, "training_data_admitted": False}
        return control

    def completion(self, measured_speed):
        if self.entry is None:
            return None
        if measured_speed <= .5:
            return "EARLY_MEASURED_SPEED_FLOOR"
        return "FIXED_PULSE_DURATION" if self.pulse_ticks == PULSE_TICKS else None


def scalar_guards(row):
    failures = []
    if not math.isfinite(row["speed_mps"]) or row["speed_mps"] > 2.55:
        failures.append("measured speed exceeds 2.55 m/s")
    if not math.isfinite(row["front_bumper_stop_distance_m"]) or row["front_bumper_stop_distance_m"] < 40.:
        failures.append("40 m upstream signal exclusion boundary reached")
    if not math.isfinite(row["route_projection"]["progress_m"]) or row["route_projection"]["progress_m"] > 30.:
        failures.append("30 m calibration extent reached")
    if row["signal_state"] not in {"Green", "Yellow", "Red"}:
        failures.append("bound signal state unavailable")
    if row["control_alignment"]["status"] != "PASS":
        failures.append("native ACK control alignment failed")
    if len(row["body_corners"]) != 8 or any(point.get("lane") != [588, 0, 1]
           or not math.isfinite(point.get("lane_margin_m", -1.)) or point.get("lane_margin_m", -1.) < .10
           for point in row["body_corners"]):
        failures.append("instantaneous body corner violates fixed lane/10 cm inset")
    return failures


def checked_geometry(world, carla_map, carla, ego, receipt, route, snapshot):
    scout.require(int(world.id) == receipt["world_id"], "world identity changed")
    signal = receipt["controlling_signal"]
    light = world.get_actor(int(signal["actor_id"]))
    scout.require(light is not None and not light.is_dormant and str(light.get_opendrive_id()) == scout.SIGNAL_ID,
                  "controlling light identity changed")
    transform = snapshot.find(ego.id).get_transform()
    points = ego.bounding_box.get_world_vertices(transform)
    stop = signal["selected_stop_waypoint"]
    sx, sy, _ = stop["xyz_carla_m"]
    angle = math.radians(stop["yaw_carla_deg"])
    distance = min((sx - p.x) * math.cos(angle) + (sy - p.y) * math.sin(angle) for p in points)
    body = []
    for point in points:
        waypoint = carla_map.get_waypoint(point, project_to_road=False, lane_type=carla.LaneType.Driving)
        row = {"xyz_carla_m": [float(point.x), float(point.y), float(point.z)], "lane": None}
        if waypoint is not None:
            center = waypoint.transform.location
            yaw = math.radians(waypoint.transform.rotation.yaw)
            lateral = -(point.x - center.x) * math.sin(yaw) + (point.y - center.y) * math.cos(yaw)
            row.update(lane=[int(waypoint.road_id), int(waypoint.section_id), int(waypoint.lane_id)],
                lane_margin_m=float(waypoint.lane_width) / 2 - abs(lateral),
                waypoint_center_z_m=float(center.z), waypoint_pitch_deg=float(waypoint.transform.rotation.pitch))
        body.append(row)
    scout.require(world.get_snapshot().frame == snapshot.frame, "geometry crossed native frames")
    return {"signal_state": str(light.get_state()).rsplit(".", 1)[-1],
        "signal_actor_id": int(light.id), "signal_opendrive_id": scout.SIGNAL_ID,
        "front_bumper_stop_distance_m": distance, "body_corners": body,
        "route_projection": scout.project(route["route"], transform.location.x, transform.location.y)}


def destroy_owned(world, actors):
    """HH_261001 - Never enumerate foreign actors for destruction; verify every explicitly created actor is gone."""
    errors, ids = [], [int(actor.id) for actor in actors]
    for actor in reversed(actors):
        try:
            if actor.type_id.startswith("sensor."):
                actor.stop()
            if actor.destroy() is not True:
                errors.append(f"destroy returned false for owned actor {actor.id}")
        except Exception as error:
            errors.append(f"owned actor {actor.id}: {error}")
    remaining, frame = [], None
    if ids:
        try:
            # HH_261001 - GetActor(id) may reuse Episode's description cache after deletion; fresh native state is authoritative.
            frame = world.tick(15.)
            snapshot = world.get_snapshot()
            scout.require(int(snapshot.frame) == frame, "post-destroy native snapshot frame differs")
            remaining = [identity for identity in ids if snapshot.find(identity) is not None]
        except Exception as error:
            errors.append(f"could not verify owned actor disappearance: {error}")
            remaining = list(ids)
    return {"created_actor_ids": ids, "remaining_owned_actor_ids": remaining, "errors": errors,
            "post_destroy_frame": frame, "verification": "fresh immutable native snapshot actor membership",
            "status": "PASS" if not remaining and not errors else "FAIL"}


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
        args = SimpleNamespace(target_speed_kmh=8.64, physics_hz=20.,
            basic_agent_base_min_distance_m=1.5, basic_agent_distance_ratio=.5)
        options, control_contract = collector.basic_agent_control_configuration(args, 1.)
        agent = BasicAgent(ego, target_speed=8.64, opt_dict=options, map_inst=carla_map, grp_inst=planner)
        agent.set_global_plan(plan)
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


def execute(output, scout_directory, owner_pid, owner_pgid):
    scout.assert_owner(owner_pid, owner_pgid)
    receipt = lift.base.validate_scout_receipt(scout_directory)
    carla, planner_class = scout.source_api()
    from agents.navigation.basic_agent import BasicAgent
    scout.require(Path(inspect.getsourcefile(BasicAgent)).resolve() == scout.SOURCE_PLANNER.with_name("basic_agent.py"),
                  "foreign BasicAgent source")
    client = carla.Client("127.0.0.1", 2100)
    client.set_timeout(30.)
    world = client.get_world()
    carla_map = world.get_map()
    scout.require(int(world.id) == receipt["world_id"] and carla_map.name in scout.LIVE_NAMES,
                  "same-run Town12 scout world identity changed")
    scout.require(hashlib.sha256(carla_map.to_opendrive().encode()).hexdigest() == scout.XODR_SHA256,
                  "live XODR changed")
    route = collector.load_route(scout_directory / "route.json")
    original, weather = world.get_settings(), world.get_weather()
    scout.require(not original.synchronous_mode, "owned asynchronous startup expected")
    result = {"schema": "town12.actuator_calibration.v1", "protocol": protocol(), "levels": [],
        "training_data_admitted": False, "world_id": int(world.id), "xodr_sha256": scout.XODR_SHA256,
        "route_polyline_sha256": scout.ROUTE_SHA256, "original_settings": collector._settings_dict(original)}
    try:
        settings = world.get_settings()
        settings.synchronous_mode, settings.fixed_delta_seconds = True, DT
        world.apply_settings(settings)
        effective = world.get_settings()
        scout.require(effective.synchronous_mode and math.isclose(effective.fixed_delta_seconds, DT, abs_tol=1e-9),
                      "native synchronous 20 Hz settings did not apply")
        world.set_weather(carla.WeatherParameters.ClearNoon)
        result["effective_settings"] = collector._settings_dict(effective)
        for index, level in enumerate(LEVELS):
            directory = output / f"level_{index:02d}"
            directory.mkdir()
            measured = run_level(directory, level, client, world, carla_map, carla, planner_class,
                                 route, receipt, (owner_pid, owner_pgid))
            result["levels"].append(measured)
            scout.require(measured["status"] == "PASS_MEASURED_CALIBRATION", "bounded level failed; no automatic retry")
        result["status"] = "PASS"
    except Exception as error:
        result.update(status="FAIL", error=f"{type(error).__name__}: {error}")
    finally:
        try:
            world.set_weather(weather)
            world.apply_settings(original)
            result["settings_restored"] = collector._settings_dict(world.get_settings()) == collector._settings_dict(original)
            scout.require(result["settings_restored"], "owned runtime settings restoration failed")
        except Exception as error:
            result.update(status="FAIL", restoration_error=f"{type(error).__name__}: {error}")
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--scout", type=Path)
    parser.add_argument("--owner-pid", type=int)
    parser.add_argument("--owner-pgid", type=int)
    args = parser.parse_args(argv)
    if not args.execute:
        print(json.dumps({"status": "STATIC_CALIBRATION_CONTRACT", "simulator_contacted": False, **protocol()}, sort_keys=True))
        return 0
    scout.require(all(value is not None for value in (args.output, args.scout, args.owner_pid, args.owner_pgid)),
                  "explicit owner, output and same-run scout required")
    scout.require(args.output.is_dir() and not args.output.is_symlink() and not list(args.output.iterdir()),
                  "fresh owned calibration directory required")
    try:
        result = execute(args.output, args.scout, args.owner_pid, args.owner_pgid)
    except Exception as error:
        result = {"status": "FAIL", "error": f"{type(error).__name__}: {error}", "training_data_admitted": False}
    scout.write_new(args.output / "calibration_result.json", result)
    print(json.dumps({"status": result["status"], "levels": len(result.get("levels", [])),
                      "error": result.get("error"), "training_data_admitted": False}, sort_keys=True))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
