#!/usr/bin/env python3
"""HH_261001 - One bounded SI PI/coast/hold measurement; never produce or admit TRAIN data."""

import argparse
from collections import deque
import hashlib
import inspect
import json
import math
from pathlib import Path
import shutil
from types import SimpleNamespace

from scripts.e2e import calibrate_town12_j15824_brake_pulses_v4_20261001 as previous


scout, collector, lift = previous.scout, previous.collector, previous.lift
append, checked_geometry = previous.append, previous.checked_geometry
scalar_guards, destroy_owned = previous.scalar_guards, previous.destroy_owned
DT, SETTLE_TICKS = .05, 40
MINIMUM_FREE_BYTES = 5 * 1024 ** 3
POLICY_ID = "town12_nontraining_persistent_pi_coast_hold_v1"
PI_STAGE, COAST_STAGE, HOLD_STAGE = "si_pi_launch_and_stabilize", "zero_pedal_coast", "measured_low_speed_hold"


def protocol():
    return {"policy_id": POLICY_ID, "training_data_admitted": False, "future_labels_created": False,
        "six_camera_capture": False, "candidate_trials": 1, "fresh_owned_actor_per_trial": True,
        "minimum_free_disk_bytes": MINIMUM_FREE_BYTES,
        "spawn_attempts_per_trial": 1, "physics_hz": 20., "settle_ticks": SETTLE_TICKS,
        "settle_brake": 1., "target_speed_mps": 1.5, "target_speed_kmh": 5.4,
        "measured_speed": "unfiltered snapshot actor planar velocity norm; virtual-base raw labels remain separate",
        "entry_band_mps": [1.45, 1.55], "consecutive_entry_band_ticks": 10,
        "maximum_launch_seconds": 20., "maximum_coast_seconds": 10.,
        "kp_throttle_per_mps": .12, "ki_throttle_per_m": .04,
        "integral_contribution_bounds": [0., .20], "normal_throttle_bounds": [0., .20],
        "initial_throttle": 0., "throttle_rise_per_second": .02, "throttle_fall_per_second": .05,
        "anti_windup": "Candidate bounded persistent integral is rejected only when error pushes farther beyond current saturation-plus-slew bounds; output recomputed with retained integral.",
        "normal_moving_brake": 0., "coast_throttle": 0., "coast_brake": 0.,
        "hold_entry_raw_speed_maximum_mps": .05, "consecutive_hold_entry_ticks": 2,
        "hold_brake": 1., "hold_observed_ticks": 40, "hold_observed_speed_maximum_mps": .05,
        "minimum_all_body_corner_stop_distance_m": 40., "maximum_route_progress_m": 30.,
        "maximum_speed_mps": 2.55, "required_road_section_lane": [588, 0, 1],
        "minimum_instantaneous_body_lane_margin_m": .10,
        "base_min_distance_m": 1.5, "distance_ratio_s": .5,
        "original_basic_agent_red_emergency_unchanged": True, "traffic_light_states_written": False,
        "normal_coast_hold_and_original_emergency_bypass_throttle_slew": True,
        "emergency_response": "Apply original emergency, record its ACK and native response, then terminate; never resume or reset PI.",
        "replacement_scope": "Combined longitudinal controller replacement including nominal1.50 instead of empirical1.58 target compensation; not a single-Kp causal A/B.",
        "quality_policy": "Preserve complete bounded measurement despite native quality failures; fail final candidate result if any original decoder or runtime native rate limit fails. Operational guards abort immediately after preserving the violating row.",
        "termination": "Separate full-service-brake NON-TRAIN cleanup retains every observed row, including failures.",
        "smoother_launch_claimed": False, "stopping_distance_proven": False,
        "limitations": ["One upstream uphill site, not Town12 signal approach, downhill endpoint or Town07.",
            "Instantaneous body-corner checks are not swept-body qualification.",
            "No six-camera data, future labels, training admission, full-route execution or automatic retry.",
            "Native speed-rate PASS alone is not full decoder, runtime or trajectory quality PASS."]}


def clip(value, lower, upper):
    return min(upper, max(lower, value))


def require_free_space(path):
    """HH_261001 - Preserve the explicit 5 GiB floor without deleting any existing evidence."""
    free = shutil.disk_usage(path).free
    scout.require(free >= MINIMUM_FREE_BYTES, "free disk space is below the fixed 5 GiB floor")
    return free


class PersistentPI:
    """HH_261001 - Integral state is a pedal contribution; anti-windup includes the current slew bounds."""
    def __init__(self):
        self.integral = self.throttle = 0.
        self.last = None

    def step(self, speed, dt):
        scout.require(math.isfinite(speed) and 0. <= speed <= 2.55 and dt in (0., DT),
                      "finite raw speed and native PI interval required")
        error = 1.5 - speed
        old_integral, old_throttle = self.integral, self.throttle
        candidate = clip(old_integral + .04 * error * dt, 0., .20)
        lower, upper = max(0., old_throttle - .05 * dt), min(.20, old_throttle + .02 * dt)
        candidate_raw = .12 * error + candidate
        rejected = (candidate_raw > upper and error > 0.) or (candidate_raw < lower and error < 0.)
        self.integral = old_integral if rejected else candidate
        raw = .12 * error + self.integral
        self.throttle = clip(raw, lower, upper)
        self.last = {"dt_seconds": dt, "raw_speed_mps": speed, "target_speed_mps": 1.5,
            "error_mps": error, "proportional_contribution": .12 * error,
            "integral_before": old_integral, "integral_candidate": candidate,
            "candidate_unclipped_throttle": candidate_raw, "integral_update_rejected": rejected,
            "integral_after": self.integral, "previous_throttle": old_throttle,
            "allowed_throttle_lower": lower, "allowed_throttle_upper": upper,
            "unclipped_throttle_after_anti_windup": raw, "requested_throttle": self.throttle}
        return self.throttle


class StartStopPolicy:
    """HH_261001 - Causal native observations alone authorize coast and hold; original emergency runs afterward."""
    def __init__(self):
        self.pi = PersistentPI()
        self.started_at = self.previous_time = self.previous_frame = None
        self.entry = self.hold_entry = self.last = None
        self.entry_witnesses, self.low_witnesses = deque(maxlen=10), deque(maxlen=2)
        self.hold_commands = self.hold_observations = 0

    def apply(self, control, *, speed, timestamp, frame):
        scout.require(math.isfinite(speed) and 0. <= speed <= 2.55 and math.isfinite(timestamp),
                      "invalid actual calibration speed/time")
        scout.require(isinstance(frame, int) and not isinstance(frame, bool), "integer native frame required")
        first = self.previous_time is None
        if not first:
            scout.require(abs(timestamp - self.previous_time - DT) < 1e-5
                          and frame == self.previous_frame + 1, "non-native policy frame/time cadence")
        scout.require(float(control.brake) == 0., "normal planner brake must be capped before replacement")
        scout.require(self.hold_observations < 40, "policy called after bounded hold completion")
        raw = collector.control_dict(control)
        if first:
            self.started_at = timestamp
        self.previous_time, self.previous_frame = timestamp, frame
        witness = {"frame": frame, "timestamp": timestamp, "raw_actor_planar_speed_mps": speed}
        if self.entry is None:
            scout.require(timestamp - self.started_at < 20. - 1e-6, "20-second PI entry stabilization timed out")
            if 1.45 <= speed <= 1.55:
                self.entry_witnesses.append(witness)
            else:
                self.entry_witnesses.clear()
            if len(self.entry_witnesses) == 10:
                self.entry = {**witness, "preceding_ten_native_observations": list(self.entry_witnesses)}
        if self.entry is None:
            control.throttle = self.pi.step(speed, 0. if first else DT)
            control.brake, stage = 0., PI_STAGE
        else:
            if self.hold_entry is None:
                if speed <= .05:
                    self.low_witnesses.append(witness)
                else:
                    self.low_witnesses.clear()
                elapsed = timestamp - self.entry["timestamp"]
                if len(self.low_witnesses) == 2:
                    scout.require(elapsed <= 10. + 1e-6, "hold entry exceeded 10-second coast budget")
                    self.hold_entry = {**witness, "preceding_two_native_observations": list(self.low_witnesses)}
                else:
                    scout.require(elapsed < 10. - 1e-6, "10-second zero-pedal coast timed out")
            control.throttle = 0.
            if self.hold_entry is None:
                control.brake, stage = 0., COAST_STAGE
            else:
                scout.require(speed <= .05, "actual speed exceeded fixed hold maximum")
                scout.require(self.hold_commands < 40, "bounded hold command count exceeded")
                self.hold_commands += 1
                control.brake, stage = 1., HOLD_STAGE
        self.last = {"stage": stage, "source_frame": frame, "source_timestamp": timestamp,
            "raw_actor_planar_speed_mps": speed, "normal_planner_before_intervention": raw,
            "normal_control_before_original_emergency": collector.control_dict(control),
            "pi": dict(self.pi.last) if self.pi.last is not None else None,
            "pi_updated_this_tick": stage == PI_STAGE, "entry": self.entry,
            "hold_entry": self.hold_entry, "hold_command_number": self.hold_commands,
            "training_data_admitted": False}
        return control

    def observe(self, *, speed, frame):
        scout.require(self.last is not None and frame == self.last["source_frame"] + 1,
                      "hold observation not bound to its native command")
        if self.last["stage"] == HOLD_STAGE:
            scout.require(math.isfinite(speed) and 0. <= speed <= .05,
                          "observed hold speed exceeds .05 m/s")
            scout.require(self.hold_observations + 1 == self.hold_commands,
                          "duplicate or missing hold observation")
            self.hold_observations += 1
        return self.hold_observations == 40


def native_quality(rows, bounds):
    """HH_261001 - Original native bounds are diagnostic gates, never raw-value clipping or full trajectory approval."""
    normal = [row for row in rows if row["applied_policy"]["stage"] in (PI_STAGE, COAST_STAGE, HOLD_STAGE)]
    assessed = [row for row in normal if row.get("speed_rate_mps2") is not None]
    witnesses = []
    for row in assessed:
        rate = row["speed_rate_mps2"]
        failed = [name for name in ("physical_decoder", "runtime_speed_rate_gate")
                  if not math.isfinite(rate) or rate > bounds[name]["maximum_acceleration_mps2"]
                  or rate < -bounds[name]["maximum_deceleration_mps2"]]
        if failed:
            witnesses.append({"frame": row["frame"], "stage": row["applied_policy"]["stage"],
                              "speed_rate_mps2": rate, "failed_bounds": failed})
    return {"status": "PASS" if assessed and not witnesses else "FAIL", "assessed_native_intervals": len(assessed),
        "minimum_native_rate_mps2": min((row["speed_rate_mps2"] for row in assessed), default=None),
        "maximum_native_rate_mps2": max((row["speed_rate_mps2"] for row in assessed), default=None),
        "failed_native_intervals": witnesses, "full_trajectory_quality_proven": False,
        "cleanup_included": False, "training_data_admitted": False}


def checked_agent_command(agent, policy, frame):
    """HH_261001 - Inspect, but never modify, the original agent's final emergency-capable return."""
    command = agent.run_step()
    decision = policy.last
    scout.require(decision is not None and decision["source_frame"] == frame,
                  "normal decision missing or stale")
    emergency = collector.control_dict(command) != decision["normal_control_before_original_emergency"]
    return command, {**decision, "original_agent_overrode_normal_control": emergency}, emergency


def run_trial(directory, client, world, carla_map, carla, planner_class, route, receipt, owner_identity):
    from agents.navigation.basic_agent import BasicAgent
    from scripts.e2e.prepare_carla_route import serialize_route

    record = {"status": "RUNNING", "measurement_completed": False, "training_data_admitted": False}
    actors, previous_row, observed_rows = [], None, []
    bounds = collector.source_motion_bounds()
    rows, decisions = directory / "native_frames.jsonl", directory / "normal_decisions.jsonl"
    transport = ego = policy = None
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
            nonlocal previous_row
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
            velocity = snapshot.find(ego.id).get_velocity()
            row = {"frame": frame, "timestamp": float(snapshot.timestamp.elapsed_seconds), **state, **raw,
                "speed_mps": math.hypot(state["vx"], state["vy"]),
                "raw_actor_planar_speed_mps": math.hypot(velocity.x, velocity.y),
                "applied_policy": decision, "applied_control": collector.control_dict(current),
                "control_alignment": alignment, "training_data_admitted": False}
            try:
                row.update(checked_geometry(world, carla_map, carla, ego, receipt, route, snapshot))
            except Exception as error:
                append(rows, {**row, "geometry_failure": f"{type(error).__name__}: {error}"})
                raise
            row["speed_rate_mps2"] = None
            if previous_row is not None:
                elapsed = row["timestamp"] - previous_row["timestamp"]
                scout.require(abs(elapsed - DT) < 1e-5, "raw snapshot cadence differs from 20 Hz")
                row.update(speed_rate_mps2=(row["speed_mps"] - previous_row["speed_mps"]) / elapsed,
                    measured_actor_center_dz_m=raw["actor_snapshot_transform_carla"]["z"]
                    - previous_row["actor_snapshot_transform_carla"]["z"])
            row["guard_failures"], row["events_seen"] = scalar_guards(row), events.snapshot()
            row["free_disk_bytes"] = shutil.disk_usage(directory).free
            if row["free_disk_bytes"] < MINIMUM_FREE_BYTES:
                row["guard_failures"].append("free disk space is below the fixed 5 GiB floor")
            append(rows, row)
            observed_rows.append(row)
            previous_row = row
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
        args = SimpleNamespace(target_speed_kmh=5.4, physics_hz=20.,
            basic_agent_base_min_distance_m=1.5, basic_agent_distance_ratio=.5)
        options, control_contract = collector.basic_agent_control_configuration(args, 1.)
        agent = BasicAgent(ego, target_speed=5.4, opt_dict=options, map_inst=carla_map, grp_inst=planner)
        agent.set_global_plan(plan)
        scout.require(agent._target_speed == 5.4 and agent.get_local_planner()._target_speed == 5.4,
                      "agent nominal target differs from 1.50 m/s")
        scout.require(not agent._ignore_traffic_lights and not agent._ignore_vehicles, "original hazard handling disabled")
        local = agent.get_local_planner()
        caps = collector.install_normal_brake_cap(local, 0., .20)
        original_normal, policy = local.run_step, StartStopPolicy()

        def normal(*args, **kwargs):
            control = original_normal(*args, **kwargs)
            snapshot = world.get_snapshot()
            velocity = snapshot.find(ego.id).get_velocity()
            frame, timestamp = int(snapshot.frame), float(snapshot.timestamp.elapsed_seconds)
            speed = math.hypot(velocity.x, velocity.y)
            try:
                return policy.apply(control, speed=speed, timestamp=timestamp, frame=frame)
            except Exception as error:
                append(directory / "policy_aborts.jsonl", {"source_frame": frame, "source_timestamp": timestamp,
                    "raw_actor_planar_speed_mps": speed, "original_normal_proposal": collector.control_dict(control),
                    "error": f"{type(error).__name__}: {error}", "new_command_sent": False,
                    "last_ack_observation_frame": previous_row["frame"] if previous_row else None,
                    "training_data_admitted": False})
                raise

        local.run_step = normal
        record.update(control_contract=control_contract, normal_caps=caps, effective_target_speed_kmh=5.4,
            original_basic_agent_source=str(Path(inspect.getsourcefile(BasicAgent)).resolve()),
            runtime_center_polyline_sha256=scout.route_digest(points), source_motion_bounds=bounds)
        for _ in range(640):
            command, decision, emergency = checked_agent_command(agent, policy, int(world.get_snapshot().frame))
            observed = advance(command, decision)
            scout.require(not emergency, "original BasicAgent emergency applied and recorded; calibration terminated")
            if policy.observe(speed=observed["raw_actor_planar_speed_mps"], frame=observed["frame"]):
                record.update(measurement_completed=True, entry=policy.entry, hold_entry=policy.hold_entry,
                    hold_observed_ticks=policy.hold_observations, status="PASS_MEASURED_CALIBRATION")
                break
        else:
            raise scout.ScoutError("bounded 640 native control ticks exhausted")
    except Exception as error:
        record.update(status="FAIL", error=f"{type(error).__name__}: {error}")
    finally:
        record["native_motion_quality"] = native_quality(observed_rows, bounds)
        if record["status"] != "FAIL" and record["native_motion_quality"]["status"] != "PASS":
            record.update(status="FAIL", error="Original native decoder/runtime motion rate limits failed; complete raw trial retained")
        if policy is not None:
            record.update(entry=policy.entry, hold_entry=policy.hold_entry,
                          hold_observed_ticks=policy.hold_observations, final_pi_integral=policy.pi.integral)
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
        record["actor_cleanup"], record["events"] = destroy_owned(world, actors), events.snapshot()
        if record["actor_cleanup"]["status"] != "PASS" or record.get("cleanup_control_error") or any(record["events"]):
            record["status"] = "FAIL"
        record["raw_files_sha256"] = {path.name: scout.digest(path) for path in directory.glob("*.jsonl")}
        scout.write_new(directory / "result.json", record)
    return record


def execute(output, scout_directory, owner_pid, owner_pgid):
    require_free_space(output)
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
    result = {"schema": "town12.pi_coast_hold_calibration.v1", "protocol": protocol(), "levels": [],
        "training_data_admitted": False, "world_id": int(world.id), "xodr_sha256": scout.XODR_SHA256,
        "route_polyline_sha256": scout.ROUTE_SHA256, "original_settings": collector._settings_dict(original)}
    try:
        settings = world.get_settings()
        settings.synchronous_mode, settings.fixed_delta_seconds = True, DT
        world.apply_settings(settings)
        effective = world.get_settings()
        scout.require(effective.synchronous_mode and math.isclose(effective.fixed_delta_seconds, DT, abs_tol=1e-9),
                      "native synchronous 20 Hz settings did not apply")
        scout.require(effective.substepping and math.isclose(effective.max_substep_delta_time, .01, abs_tol=1e-9)
                      and effective.max_substeps == 10, "previous maximum 10 ms substep settings changed")
        world.set_weather(carla.WeatherParameters.ClearNoon)
        result["effective_settings"] = collector._settings_dict(effective)
        directory = output / "candidate_00"
        directory.mkdir()
        measured = run_trial(directory, client, world, carla_map, carla, planner_class,
                             route, receipt, (owner_pid, owner_pgid))
        result["levels"].append(measured)
        scout.require(measured["status"] == "PASS_MEASURED_CALIBRATION",
                      "bounded candidate failed: " + str(measured.get("error")))
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
    print(json.dumps({"status": result["status"], "trials": len(result.get("levels", [])),
                      "error": result.get("error"), "training_data_admitted": False}, sort_keys=True))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
