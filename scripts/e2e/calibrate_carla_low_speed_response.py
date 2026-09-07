#!/usr/bin/env python3
"""HH_260906 - Measure raw CARLA pedal responses without ROS, cameras, or training-data promotion."""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import fcntl
import math
import os
from pathlib import Path
import shutil
import signal
import sys
from typing import Any, Mapping, Sequence

if __package__:
    from . import collect_carla_vad_expert as capture
    from .carla_goal_stop_profile import bounded_route_projection, source_motion_bounds
    from .carla_low_speed_response_matrix import (IdentificationCase, TwoStageCase, hold_control,
        identification_matrix, matrix_contract, progressive_identification_matrix, progressive_matrix_contract)
else:
    import collect_carla_vad_expert as capture
    from carla_goal_stop_profile import bounded_route_projection, source_motion_bounds
    from carla_low_speed_response_matrix import (IdentificationCase, TwoStageCase, hold_control,
        identification_matrix, matrix_contract, progressive_identification_matrix, progressive_matrix_contract)


SCHEMA = "carla.low_speed_response_calibration.v1"
PHYSICS_HZ = 20.0
SETTLE_SECONDS = 3.5
THROTTLE_SECONDS = 8.0
PREPARE_SECONDS = 15.0
BRAKE_SECONDS = 15.0
PREPARE_THROTTLE = 0.30
PREPARE_SPEED_MPS = 3.0
MAXIMUM_TRAVEL_M = 80.0
MAXIMUM_CTE_M = 3.0
MAXIMUM_SPEED_MPS = 10.0
VEHICLE_TYPE = "vehicle.toyota.prius"
ROLE_NAME = "autoware_e2e_low_speed_calibration"
WHEELBASE_M = 2.85
SPAWN_Z_OFFSET_M = 0.5
PHYSICS_REQUIRED = (
    "torque_curve", "max_rpm", "moi", "damping_rate_full_throttle",
    "damping_rate_zero_throttle_clutch_engaged", "damping_rate_zero_throttle_clutch_disengaged",
    "use_gear_autobox", "gear_switch_time", "clutch_strength", "final_ratio", "forward_gears",
    "mass", "drag_coefficient", "center_of_mass", "steering_curve", "wheels", "use_sweep_wheel_collision",
)
WHEEL_REQUIRED = (
    "tire_friction", "damping_rate", "max_steer_angle", "radius", "max_brake_torque",
    "max_handbrake_torque", "lat_stiff_max_load", "lat_stiff_value", "long_stiff_value", "position",
)


class CalibrationError(capture.CollectionError):
    """HH_260906 - Fail closed while preserving every already observed sample."""


@dataclass(frozen=True)
class ResponseCase:
    """HH_260906 - Freeze the requested twelve-case matrix before any simulator access."""

    case_id: str
    kind: str
    level: float


def case_matrix(matrix_id: str = "low_speed_v1") -> tuple[ResponseCase | IdentificationCase, ...]:
    # HH_260906 - Keep the original twelve commands as the default; opt in to a distinct identification matrix.
    if matrix_id == "low_speed_v2":
        return identification_matrix()
    if matrix_id == "low_speed_v3":
        return progressive_identification_matrix()
    if matrix_id != "low_speed_v1":
        raise CalibrationError("unknown low-speed response matrix")
    cases = []
    for kind, levels in (("throttle", (0.05, 0.10, 0.15, 0.20, 0.30, 0.40)),
                         ("brake", (0.02, 0.04, 0.06, 0.08, 0.10, 0.12))):
        for level in levels:
            cases.append(ResponseCase(f"{len(cases) + 1:02d}_{kind}_{level:.2f}", kind, level))
    return tuple(cases)


def serialize_physics_value(value: Any, depth: int = 0) -> Any:
    """HH_260906 - Serialize available public scalar/vector/gear/wheel fields, never opaque repr strings."""
    if depth > 8:
        raise CalibrationError("physics serialization exceeded its bounded nesting depth")
    if value is None or isinstance(value, (bool, str)):
        return value
    if isinstance(value, (int, float)):
        if not math.isfinite(value):
            raise CalibrationError("physics values must be finite")
        return value
    if isinstance(value, Mapping):
        if not all(isinstance(key, str) for key in value):
            raise CalibrationError("physics object keys must be strings")
        return {key: serialize_physics_value(item, depth + 1) for key, item in sorted(value.items())}
    if isinstance(value, (list, tuple)):
        if len(value) > 256:
            raise CalibrationError("physics sequence exceeds the versioned serialization limit")
        return [serialize_physics_value(item, depth + 1) for item in value]
    fields = {}
    for name in sorted(dir(value)):
        if name.startswith("_"):
            continue
        item = getattr(value, name)
        if not callable(item):
            fields[name] = serialize_physics_value(item, depth + 1)
    if not fields:
        raise CalibrationError(f"unsupported physics field type: {type(value).__name__}")
    return fields


def physics_snapshot(value: Any) -> dict[str, Any]:
    """HH_260906 - Require the complete known 0.9.15 contract and preserve additional available fields."""
    data = serialize_physics_value(value)
    if not isinstance(data, dict) or any(name not in data for name in PHYSICS_REQUIRED):
        raise CalibrationError("VehiclePhysicsControl is missing required 0.9.15 fields")
    if not isinstance(data["wheels"], list) or len(data["wheels"]) != 4:
        raise CalibrationError("the fixed Prius calibration requires four physics wheels")
    for wheel in data["wheels"]:
        if not isinstance(wheel, dict) or any(name not in wheel for name in WHEEL_REQUIRED):
            raise CalibrationError("WheelPhysicsControl is missing required 0.9.15 fields")
    def numeric(item: Any, name: str, positive: bool = False) -> None:
        if isinstance(item, bool) or not isinstance(item, (int, float)) or not math.isfinite(item):
            raise CalibrationError(f"physics {name} must be a finite numeric value")
        if item < 0 or (positive and item == 0):
            raise CalibrationError(f"physics {name} is outside its nonnegative/positive domain")
    def vector(item: Any, name: str, axes: tuple[str, ...]) -> None:
        if not isinstance(item, dict) or any(axis not in item for axis in axes):
            raise CalibrationError(f"physics {name} must contain {axes}")
        for axis in axes:
            component = item[axis]
            if isinstance(component, bool) or not isinstance(component, (int, float)) or not math.isfinite(component):
                raise CalibrationError(f"physics {name}.{axis} must be finite numeric")
    for name in ("max_rpm", "moi", "clutch_strength", "final_ratio", "mass"):
        numeric(data[name], name, positive=True)
    for name in ("damping_rate_full_throttle", "damping_rate_zero_throttle_clutch_engaged",
                 "damping_rate_zero_throttle_clutch_disengaged", "gear_switch_time", "drag_coefficient"):
        numeric(data[name], name)
    for name in ("use_gear_autobox", "use_sweep_wheel_collision"):
        if type(data[name]) is not bool:
            raise CalibrationError(f"physics {name} must be boolean")
    vector(data["center_of_mass"], "center_of_mass", ("x", "y", "z"))
    for name in ("torque_curve", "steering_curve"):
        if not isinstance(data[name], list) or not data[name]:
            raise CalibrationError(f"physics {name} must be a nonempty curve")
        for point in data[name]:
            vector(point, name, ("x", "y"))
    if not isinstance(data["forward_gears"], list) or not data["forward_gears"]:
        raise CalibrationError("physics forward_gears must be nonempty")
    for gear in data["forward_gears"]:
        if not isinstance(gear, dict) or any(name not in gear for name in ("ratio", "down_ratio", "up_ratio")):
            raise CalibrationError("physics gear must include ratio and shift ratios")
        for name in ("ratio", "down_ratio", "up_ratio"):
            numeric(gear[name], f"gear.{name}", positive=name == "ratio")
    for index, wheel in enumerate(data["wheels"]):
        for name in WHEEL_REQUIRED:
            if name != "position":
                numeric(wheel[name], f"wheel{index}.{name}", positive=name == "radius")
        vector(wheel["position"], f"wheel{index}.position", ("x", "y", "z"))
    return {
        "schema": "carla.vehicle_physics_snapshot.v1", "values": data,
        "extra_vehicle_fields": sorted(set(data) - set(PHYSICS_REQUIRED)),
        "notice": "Actual get_physics_control values; engine RPM, wheel RPM, filtered PhysX inputs and tire sticky state are not measured by this API.",
    }


def safety_reason(state: Mapping[str, Any], collision_count: int) -> str | None:
    """HH_260906 - Bound measurement trials without treating observed rate violations as data to discard."""
    values = [state[name] for name in ("vx", "vy", "travel_m", "route_cte_m")]
    if not all(math.isfinite(value) for value in values):
        return "nonfinite_motion"
    if collision_count:
        return "collision"
    if state["travel_m"] > MAXIMUM_TRAVEL_M:
        return "maximum_travel_exceeded"
    if state["route_cte_m"] > MAXIMUM_CTE_M:
        return "maximum_cross_track_error_exceeded"
    if math.hypot(state["vx"], state["vy"]) > MAXIMUM_SPEED_MPS:
        return "maximum_speed_exceeded"
    if state["vx"] < -0.1:
        return "reverse_motion_exceeded"
    return None


def analyze_rates(records: Sequence[Mapping[str, Any]], bounds: Mapping[str, Any]) -> dict[str, Any]:
    """HH_260906 - Retain startup, stop and phase-boundary impulses at 20 Hz and both 10 Hz offsets."""
    results = {}
    for name, rows, expected_dt in (("native_20hz", records, 0.05),
                                    ("derived_10hz_offset_0", records[::2], 0.1),
                                    ("derived_10hz_offset_1", records[1::2], 0.1)):
        intervals = []
        for first, second in zip(rows, rows[1:]):
            dt = second["timestamp"] - first["timestamp"]
            if not math.isfinite(dt) or dt <= 0:
                raise CalibrationError("raw measurements require increasing finite timestamps")
            initial_speed, speed = math.hypot(first["vx"], first["vy"]), math.hypot(second["vx"], second["vy"])
            rate = (speed - initial_speed) / dt
            if not math.isfinite(rate):
                raise CalibrationError("raw measurements require finite speed rates")
            intervals.append({"from_frame": first["frame"], "to_frame": second["frame"],
                              "from_phase": first["phase"], "to_phase": second["phase"],
                              "from_speed_mps": initial_speed, "to_speed_mps": speed,
                              "dt_sec": dt, "speed_rate_mps2": rate})
        phase_reports = {}
        phases = ("all", "settle", "prepare", "throttle_hold", "brake_hold") + tuple(
            phase for phase in ("coast_hold", "throttle_ramp", "launch_prepare", "post_handoff_ramp")
            if any(row["phase"] == phase for row in records))
        for phase in phases:
            selected = intervals if phase == "all" else [row for row in intervals if row["to_phase"] == phase]
            rates = [row["speed_rate_mps2"] for row in selected]
            report = {"interval_count": len(selected), "minimum_speed_rate_mps2": min(rates) if rates else None,
                      "maximum_speed_rate_mps2": max(rates) if rates else None,
                      "phase_boundary_intervals": [row for row in selected if row["from_phase"] != row["to_phase"]]}
            for kind in ("physical_decoder", "runtime_speed_rate_gate"):
                limit = bounds[kind]
                violations = [row for row in selected if row["speed_rate_mps2"] > limit["maximum_acceleration_mps2"] + 1.0e-9
                              or row["speed_rate_mps2"] < -limit["maximum_deceleration_mps2"] - 1.0e-9]
                report[kind] = {"violation_count": len(violations), "violation_intervals": violations}
            phase_reports[phase] = report
        results[name] = {"sample_count": len(rows),
                         "cadence_violation_count": sum(abs(row["dt_sec"] - expected_dt) > 1.0e-4 for row in intervals),
                         "frame_stride_violation_count": sum(row["to_frame"] - row["from_frame"] != round(expected_dt * PHYSICS_HZ) for row in intervals),
                         "phases": phase_reports}
    return {"speed_definition": "hypot(base_link vx, base_link vy), actual timestamp finite differences",
            "first_sample_has_no_preceding_rate": True,
            "ten_hz_notice": "Both deterministic decimations of measured 20 Hz states; no cameras were collected.",
            "training_data": False, "bounds": dict(bounds), "measurements": results}


def destroy_owned(actors: Sequence[Any]) -> list[str]:
    """HH_260906 - Destroy only actor handles created by this case, in reverse creation order."""
    errors = []
    for actor in reversed(actors):
        if str(getattr(actor, "type_id", "")).startswith("sensor."):
            try:
                actor.stop()
            except Exception as error:
                errors.append(f"stop owned actor {actor.id}: {error}")
        try:
            if actor.destroy() is not True:
                errors.append(f"destroy owned actor {actor.id} did not confirm success")
        except Exception as error:
            errors.append(f"destroy owned actor {actor.id}: {error}")
    return errors


def require_inherited_workspace_lock(expected_path: Path | None = None) -> dict[str, Any]:
    """HH_260906 - Require the wrapper's inherited workspace lock; never open/acquire a new server lock here."""
    raw_fd = os.environ.get("AUTOWARE_E2E_WORKSPACE_RUNTIME_LOCK_FD", "")
    expected = expected_path or Path(__file__).resolve().parents[2] / "data/locks/autoware_e2e_runtime.lock"
    if not raw_fd.isdecimal():
        raise CalibrationError("run through the owned wrapper: inherited workspace lock FD is missing")
    fd = int(raw_fd)
    try:
        target = Path(os.readlink(f"/proc/self/fd/{fd}"))
        if target.resolve(strict=True) != expected.resolve(strict=True):
            raise CalibrationError("inherited lock descriptor does not belong to this workspace")
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except (OSError, ValueError) as error:
        raise CalibrationError(f"inherited workspace lock is invalid or not exclusive: {error}") from error
    return {"inherited_fd": fd, "verified_exclusive": True, "scope": "wrapper-owned workspace runtime lock"}


def validate_world_timing(settings: Any) -> None:
    """HH_260906 - Validate actual server timing and substep capacity before spawning measurement actors."""
    delta = settings.fixed_delta_seconds
    capacity = settings.max_substep_delta_time * settings.max_substeps
    if (not settings.synchronous_mode or delta is None or not math.isfinite(delta)
            or abs(delta - 1.0 / PHYSICS_HZ) > 1.0e-9 or not settings.substepping
            or not math.isfinite(capacity) or settings.max_substep_delta_time <= 0
            or settings.max_substeps < 1 or delta > capacity + 1.0e-9):
        raise CalibrationError("applied server timing does not satisfy synchronous 20 Hz and its physics substep capacity")


def exclusive_world(world: Any) -> None:
    """HH_260906 - Also reject walkers and pedestrian controllers outside the old camera collector guard."""
    capture._preflight_exclusive(world)
    for pattern in ("walker.pedestrian.*", "controller.ai.walker"):
        if list(world.get_actors().filter(pattern)):
            raise CalibrationError("CARLA world contains foreign dynamic actors")


def run_two_stage(case: TwoStageCase, tick: Any, report: dict[str, Any]) -> None:
    """HH_260906 - Keep measured handoff and early completion explicit without swallowing failed startup data."""
    metadata = {"status": "launching", "launch_ticks": 0, "post_handoff_ticks": 0,
                "handoff_speed_threshold_mps": case.handoff_speed_mps,
                "measured_speed_stop_mps": case.measured_speed_stop_mps,
                "reached_measured_speed_stop": False}
    report["two_stage"] = metadata
    handoff = None
    for _ in range(round(case.launch_maximum_seconds * PHYSICS_HZ)):
        observed = tick("launch_prepare", case.launch_throttle, 0.0)
        metadata["launch_ticks"] += 1
        if math.hypot(observed["vx"], observed["vy"]) >= case.handoff_speed_mps:
            handoff = observed
            break
    if handoff is None:
        metadata.update(status="failed", completion_reason="handoff_not_reached_within_launch_limit")
        raise CalibrationError("two-stage launch did not reach measured handoff within eight seconds")
    metadata.update(status="post_handoff_ramp", handoff_frame=handoff["frame"],
                    handoff_timestamp=handoff["timestamp"], handoff_speed_mps=math.hypot(handoff["vx"], handoff["vy"]),
                    handoff_travel_m=handoff["travel_m"], handoff_route_progress_m=handoff["route_progress_m"])
    for index in range(round(case.hold_seconds * PHYSICS_HZ)):
        throttle, brake = hold_control(case, index)
        observed = tick("post_handoff_ramp", throttle, brake)
        metadata["post_handoff_ticks"] += 1
        reached = math.hypot(observed["vx"], observed["vy"]) >= case.measured_speed_stop_mps
        metadata.update(last_frame=observed["frame"], last_speed_mps=math.hypot(observed["vx"], observed["vy"]),
                        post_handoff_elapsed_seconds=observed["timestamp"] - handoff["timestamp"],
                        reached_measured_speed_stop=reached)
        if reached:
            break
    metadata.update(status="complete", completion_reason=("first_measured_30kph_crossing"
                    if metadata["reached_measured_speed_stop"] else "post_handoff_time_limit"))


def coast_stop_observation(records: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    """HH_260906 - Measure the full zero-pedal coast and observed continuous slow dwell without forcing a stop."""
    rows = [row for row in records if row["phase"] == "coast_hold"]
    result = {"maximum_speed_mps": 0.10, "required_continuous_seconds": 2.0,
              "sample_count": len(rows), "first_below_threshold_frame": None,
              "first_verified_dwell_frame": None, "longest_observed_dwell_seconds": 0.0,
              "verified_dwell_observed": False, "measurement_valid": True,
              "notice": "Observed scalar speed only, not goal-position completion; no final brake is applied."}
    if (any(not math.isfinite(row["timestamp"]) or not math.isfinite(math.hypot(row["vx"], row["vy"])) for row in rows)
            or any(second["frame"] - first["frame"] != 1 or abs(second["timestamp"] - first["timestamp"] - .05) > 1e-4
                   for first, second in zip(rows, rows[1:]))):
        # HH_260906 - A gap or nonfinite value cannot establish continuous dwell, even in retained failed evidence.
        result.update(measurement_valid=False, validity_failure="nonfinite_or_noncontiguous_coast_measurements")
        return result
    first = None
    for row in rows:
        if math.hypot(row["vx"], row["vy"]) <= 0.10:
            if first is None:
                first = row
            if result["first_below_threshold_frame"] is None:
                result["first_below_threshold_frame"] = row["frame"]
            duration = row["timestamp"] - first["timestamp"]
            result["longest_observed_dwell_seconds"] = max(result["longest_observed_dwell_seconds"], duration)
            if duration >= 2.0 and result["first_verified_dwell_frame"] is None:
                result.update(first_verified_dwell_frame=row["frame"], first_verified_dwell_timestamp=row["timestamp"],
                              first_verified_dwell_travel_m=row["travel_m"],
                              first_verified_dwell_route_progress_m=row["route_progress_m"], verified_dwell_observed=True)
        else:
            first = None
    return result


def collect_case(carla: Any, world: Any, args: argparse.Namespace, route: Mapping[str, Any],
                 case: ResponseCase | IdentificationCase, directory: Path, stop_requested: Any,
                 bounds: Mapping[str, Any]) -> dict[str, Any]:
    """HH_260906 - Run one fresh-actor case; no velocity injection, ROS node, learner or camera exists."""
    exclusive_world(world)
    directory.mkdir()
    actors, records = [], []
    events = capture.EventRecorder()
    report = {"schema": SCHEMA, "case": asdict(case), "status": "running", "started_at": capture.utc_now(),
              "matrix_id": getattr(args, "matrix", "low_speed_v1"),
              "training_data": False, "command_source": "predeclared direct CARLA VehicleControl pedal experiment"}
    error = None
    try:
        blueprint = world.get_blueprint_library().find(VEHICLE_TYPE)
        blueprint.set_attribute("role_name", ROLE_NAME)
        if blueprint.has_attribute("color"):
            colors = blueprint.get_attribute("color").recommended_values
            if colors:
                blueprint.set_attribute("color", colors[0])
        center_start = capture.shift_transform_local_x(route["start_carla_transform"], WHEELBASE_M / 2)
        spawn = capture.apply_spawn_z_offset(center_start, SPAWN_Z_OFFSET_M)
        ego = world.try_spawn_actor(blueprint, capture._carla_transform(carla, spawn))
        if ego is None:
            raise CalibrationError("failed to spawn the fresh calibration vehicle")
        actors.append(ego)
        report.update(actor_id=int(ego.id), actor_attributes=dict(ego.attributes), actor_center_spawn_carla=spawn)
        report["vehicle_physics"] = physics_snapshot(ego.get_physics_control())
        collision = world.spawn_actor(world.get_blueprint_library().find("sensor.other.collision"),
                                      carla.Transform(), attach_to=ego, attachment_type=carla.AttachmentType.Rigid)
        actors.append(collision)
        collision.listen(events.on_collision)
        capture._write_json(directory / "report.json", report)
        progress, travel, previous_xy = 0.0, 0.0, None

        def tick(phase: str, throttle: float, brake: float) -> dict[str, Any]:
            nonlocal progress, travel, previous_xy
            if stop_requested():
                raise CalibrationError("calibration interrupted by signal")
            command = carla.VehicleControl(throttle=throttle, brake=brake, steer=0.0,
                                           hand_brake=False, reverse=False, manual_gear_shift=False)
            ego.apply_control(command)
            frame = int(world.tick(args.timeout))
            snapshot = world.get_snapshot()
            if int(snapshot.frame) != frame:
                raise CalibrationError("world snapshot does not match the observed physics frame")
            actor_snapshot = snapshot.find(ego.id)
            if actor_snapshot is None:
                raise CalibrationError("ego is missing from the tick-exact world snapshot")
            transform = capture._transform_dict(actor_snapshot.get_transform())
            velocity = capture._vector_tuple(actor_snapshot.get_velocity())
            acceleration = capture._vector_tuple(actor_snapshot.get_acceleration())
            angular = capture._vector_tuple(actor_snapshot.get_angular_velocity())
            state = capture.base_link_state(transform, velocity, acceleration, angular[2], WHEELBASE_M)
            xy = (state["x"], state["y"])
            if previous_xy is not None:
                travel += math.dist(previous_xy, xy)
            previous_xy = xy
            progress, cte, segment = bounded_route_projection(route["route"], *xy, progress, 1.0)
            collisions, _ = events.snapshot()
            record = {"frame": frame, "timestamp": float(snapshot.timestamp.elapsed_seconds), "phase": phase,
                      **state, "actor_center_transform_carla": transform, "world_velocity_carla": velocity,
                      "world_acceleration_carla": acceleration, "world_angular_velocity_carla_deg_s": angular,
                      "travel_m": travel, "route_progress_m": progress, "route_cte_m": cte, "route_segment_index": segment,
                      "requested_control": capture.control_dict(command), "applied_control": capture.control_dict(ego.get_control()),
                      "collision": collisions.get(frame, [])}
            records.append(record)
            for name in ("throttle", "brake", "steer"):
                if abs(record["requested_control"][name] - record["applied_control"][name]) > 1.0e-6:
                    raise CalibrationError(f"applied {name} does not match the declared direct command")
            reason = safety_reason(record, sum(map(len, collisions.values())))
            if reason:
                report["safety_failure"] = reason
                raise CalibrationError(reason)
            return record

        for _ in range(round(SETTLE_SECONDS * PHYSICS_HZ)):
            tick("settle", 0.0, 1.0)
        if case.kind == "two_stage_ramp" and isinstance(case, TwoStageCase):
            run_two_stage(case, tick, report)
        elif case.kind == "throttle" and isinstance(case, ResponseCase):
            for _ in range(round(THROTTLE_SECONDS * PHYSICS_HZ)):
                tick("throttle_hold", case.level, 0.0)
        elif case.kind in ("throttle", "throttle_ramp") and isinstance(case, IdentificationCase):
            phase = "throttle_hold" if case.kind == "throttle" else "throttle_ramp"
            for index in range(round(case.hold_seconds * PHYSICS_HZ)):
                throttle, brake = hold_control(case, index)
                tick(phase, throttle, brake)
        elif case.kind in ("brake", "coast"):
            prepared = None
            for _ in range(round(PREPARE_SECONDS * PHYSICS_HZ)):
                prepared = tick("prepare", PREPARE_THROTTLE, 0.0)
                if math.hypot(prepared["vx"], prepared["vy"]) >= PREPARE_SPEED_MPS:
                    break
            if prepared is None or math.hypot(prepared["vx"], prepared["vy"]) < PREPARE_SPEED_MPS:
                raise CalibrationError("preparation did not reach measured 3 m/s within 15 seconds")
            report[f"{case.kind}_entry_speed_mps"] = math.hypot(prepared["vx"], prepared["vy"])
            report[f"{case.kind}_entry_frame"] = prepared["frame"]
            if case.kind == "brake":
                for _ in range(round(BRAKE_SECONDS * PHYSICS_HZ)):
                    tick("brake_hold", 0.0, case.level)
            else:
                for index in range(round(case.hold_seconds * PHYSICS_HZ)):
                    throttle, brake = hold_control(case, index)
                    tick("coast_hold", throttle, brake)
        else:
            raise CalibrationError("case kind is outside the selected immutable matrix")
        report["status"] = "complete"
    except BaseException as caught:
        error = caught
        report.update(status="failed", error=f"{type(caught).__name__}: {caught}")
    finally:
        # HH_260906 - Despawn the scoped vehicle; do not introduce an unrecorded braking/reset trajectory.
        cleanup_errors = destroy_owned(actors)
        if not cleanup_errors and actors:
            try:
                # HH_260906 - Flush destroyed actors between cases with an explicitly disclosed empty-world tick.
                report["post_despawn_empty_world_frame"] = int(world.tick(args.timeout))
                remaining = {int(actor.id) for actor in world.get_actors()}
                owned_ids = {int(actor.id) for actor in actors}
                if remaining & owned_ids:
                    cleanup_errors.append("owned actors remain visible after despawn")
            except Exception as cleanup_error:
                cleanup_errors.append(f"post-despawn verification: {cleanup_error}")
        collisions, _ = events.snapshot()
        for record in records:
            record["collision"] = collisions.get(record["frame"], [])
        report["collision_events"] = [event for bucket in collisions.values() for event in bucket]
        if report["collision_events"] and error is None:
            error = CalibrationError("collision observed in the post-despawn event snapshot")
            report.update(status="failed", error=str(error), safety_failure="collision")
        report["cleanup"] = {"completed": not cleanup_errors, "errors": cleanup_errors,
                             "policy": "destroy only this case's owned sensor and vehicle; no full-brake synthetic stop or pose reset"}
        if cleanup_errors and error is None:
            error = CalibrationError("owned actor cleanup failed")
            report.update(status="failed", error=str(error))
        report.update(finished_at=capture.utc_now(), state_count=len(records),
                      phase_counts={phase: sum(row["phase"] == phase for row in records)
                                    for phase in ("settle", "prepare", "throttle_hold", "brake_hold") + tuple(
                                        name for name in ("coast_hold", "throttle_ramp", "launch_prepare", "post_handoff_ramp")
                                        if any(row["phase"] == name for row in records))},
                      final_speed_mps=math.hypot(records[-1]["vx"], records[-1]["vy"]) if records else None,
                      maximum_speed_mps=max((math.hypot(row["vx"], row["vy"]) for row in records), default=None))
        capture._write_jsonl(directory / "states.jsonl", records)
        if getattr(args, "matrix", "low_speed_v1") == "low_speed_v3" and case.kind == "coast":
            report["coast_stop_observation"] = coast_stop_observation(records)
        try:
            report["motion_analysis"] = analyze_rates(records, bounds)
            if any(item["cadence_violation_count"] or item["frame_stride_violation_count"]
                   for item in report["motion_analysis"]["measurements"].values()):
                raise CalibrationError("measured timing/frame stride violates the declared 20 Hz sampling contract")
        except Exception as analysis_error:
            report["motion_analysis"] = {"status": "failed", "error": str(analysis_error)}
            if error is None:
                error = CalibrationError(str(analysis_error))
                report.update(status="failed", error=str(error))
        report["states_sha256"] = capture.sha256_file(directory / "states.jsonl")
        capture._write_json(directory / "report.json", report)
    if error is not None:
        raise error
    return report


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """HH_260906 - Expose only explicit ownership arguments; the measurement matrix is immutable."""
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("output", type=Path)
    parser.add_argument("route_file", type=Path)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2100)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--matrix", choices=("low_speed_v1", "low_speed_v2", "low_speed_v3"), default="low_speed_v1")
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535 or not math.isfinite(args.timeout) or not 0 < args.timeout <= 30:
        parser.error("port or bounded RPC timeout is invalid")
    args.allow_map_load = False
    args.physics_hz = PHYSICS_HZ
    return args


def run(args: argparse.Namespace) -> Path:
    """HH_260906 - Use an existing exclusive Town07 server and restore its settings even after failure."""
    output = args.output.expanduser().absolute()
    partial = Path(str(output) + ".partial")
    if output.exists() or output.is_symlink() or partial.exists() or partial.is_symlink():
        raise CalibrationError("output or partial output already exists")
    inherited_lock = require_inherited_workspace_lock()
    route_path = args.route_file.expanduser().resolve()
    route = capture.load_route(route_path)
    if route.get("town") != "Town07" or route.get("scenario") != "straight":
        raise CalibrationError("this declared response matrix requires a Town07 straight route")
    if float(route["route"][-1]["distance_m"]) < MAXIMUM_TRAVEL_M:
        raise CalibrationError("calibration route is shorter than its maximum travel guard")
    bounds = source_motion_bounds()
    selected_matrix = case_matrix(args.matrix)
    partial.mkdir(parents=True)
    shutil.copy2(route_path, partial / "route.json")
    started_at = capture.utc_now()
    manifest = {"schema": SCHEMA, "status": "running", "created_at": started_at, "training_data": False,
                "matrix_id": args.matrix, "run_id": f"{args.matrix}:{started_at}:{output.parent.name}/{output.name}",
                "route_sha256": capture.sha256_file(route_path), "physics_hz": PHYSICS_HZ,
                "matrix": [asdict(case) for case in selected_matrix], "completed_cases": [],
                "case_ledger": [{"case_id": case.case_id, "status": "not_started"} for case in selected_matrix],
                "workspace_ownership": inherited_lock,
                "limits": {"maximum_travel_m": MAXIMUM_TRAVEL_M, "maximum_cte_m": MAXIMUM_CTE_M,
                           "maximum_speed_mps": MAXIMUM_SPEED_MPS, "maximum_reverse_speed_mps": 0.1},
                "phase_contract": {"settle_seconds": SETTLE_SECONDS, "throttle_hold_seconds": THROTTLE_SECONDS,
                                   "prepare_maximum_seconds": PREPARE_SECONDS, "prepare_throttle": PREPARE_THROTTLE,
                                   "prepare_measured_speed_mps": PREPARE_SPEED_MPS, "brake_hold_seconds": BRAKE_SECONDS},
                "vehicle_type": VEHICLE_TYPE, "role_name": ROLE_NAME, "spawn_z_offset_m": SPAWN_Z_OFFSET_M,
                "wheelbase_m": WHEELBASE_M, "weather": "ClearNoon", "client_map_loading_allowed": False,
                "bounds": bounds, "source_sha256": {
                    name: capture.sha256_file(Path(__file__).with_name(name))
                    for name in (Path(__file__).name, "collect_carla_vad_expert.py", "carla_goal_stop_profile.py", "carla_low_speed_response_matrix.py")},
                "notice": "Measurement only, not expert training data or learned driving. Every observed settle/start/stop sample is retained; no fit-eligibility filtering."}
    if args.matrix == "low_speed_v2":
        # HH_260906 - Declare coast/ramp timing separately from the unchanged v1 brake-hold contract.
        manifest["identification_matrix_contract"] = matrix_contract()
        manifest["phase_contract"].update(coast_hold_seconds=20.0, throttle_ramp_seconds=8.0,
                                          throttle_ramp_definition=matrix_contract()["ramp_policy"])
    elif args.matrix == "low_speed_v3":
        # HH_260906 - Six-case v3 timing is opt-in; original v1 and v2 contracts remain byte-for-byte structured alike.
        contract = progressive_matrix_contract()
        manifest["identification_matrix_contract"] = contract
        manifest["phase_contract"].update(coast_hold_seconds=45.0, coast_hold_ticks=900,
                                          launch_prepare_maximum_seconds=8.0,
                                          two_stage_definition=contract["two_stage_policy"])
    capture._write_json(partial / "manifest.json", manifest)
    world = client = None
    original_settings = original_weather = None
    handlers, stop = {}, {"requested": False}
    error = None
    cleanup_errors = []
    try:
        import carla
        client = carla.Client(args.host, args.port)
        client.set_timeout(args.timeout)
        world = client.get_world()
        exclusive_world(world)
        if world.get_map().name.rsplit("/", 1)[-1] != "Town07":
            raise CalibrationError("running server is not Town07; no automatic map loading is allowed")
        original_settings, original_weather = world.get_settings(), world.get_weather()
        manifest["runtime"] = {"server_version": client.get_server_version(), "client_version": client.get_client_version(),
                               "original_world_settings": capture._settings_dict(original_settings)}
        def request_stop(signum: int, _frame: Any) -> None:
            stop["requested"] = True
            manifest["stop_signal"] = int(signum)
        for signum in (signal.SIGINT, signal.SIGTERM):
            handlers[signum] = signal.getsignal(signum)
            signal.signal(signum, request_stop)
        settings = world.get_settings()
        settings.synchronous_mode, settings.fixed_delta_seconds = True, 1.0 / PHYSICS_HZ
        world.apply_settings(settings)
        world.set_weather(carla.WeatherParameters.ClearNoon)
        actual_settings = world.get_settings()
        validate_world_timing(actual_settings)
        manifest["runtime"]["capture_world_settings"] = capture._settings_dict(actual_settings)
        for index, case in enumerate(selected_matrix):
            exclusive_world(world)
            manifest["case_ledger"][index]["status"] = "running"
            capture._write_json(partial / "manifest.json", manifest)
            report = collect_case(carla, world, args, route, case, partial / case.case_id,
                                  lambda: stop["requested"], bounds)
            manifest["case_ledger"][index]["status"] = report["status"]
            manifest["completed_cases"].append({"case_id": case.case_id, "status": report["status"],
                                              "report_sha256": capture.sha256_file(partial / case.case_id / "report.json")})
            capture._write_json(partial / "manifest.json", manifest)
    except BaseException as caught:
        error = caught
        manifest["error"] = f"{type(caught).__name__}: {caught}"
        for entry in manifest["case_ledger"]:
            if entry["status"] == "running":
                entry["status"] = "failed"
            elif entry["status"] == "not_started":
                entry["status"] = "not_run_after_failure"
    finally:
        # HH_260906 - World restoration belongs to this already-exclusive campaign, never a foreign server job.
        if world is not None and original_settings is not None:
            for operation, value in (("set_weather", original_weather), ("apply_settings", original_settings)):
                try:
                    getattr(world, operation)(value)
                except Exception as cleanup_error:
                    cleanup_errors.append(f"{operation}: {cleanup_error}")
        for signum, handler in handlers.items():
            signal.signal(signum, handler)
        manifest["cleanup"] = {"completed": not cleanup_errors, "errors": cleanup_errors}
        if cleanup_errors and error is None:
            error = CalibrationError("world restoration failed")
            manifest["error"] = str(error)
        manifest.update(status="failed" if error else "complete", finished_at=capture.utc_now())
        capture._write_json(partial / "manifest.json", manifest)
    if error:
        raise error
    capture.finalize_output(partial, output)
    return output


def main(argv: Sequence[str] | None = None) -> int:
    try:
        print(f"calibration={run(parse_args(argv))}")
    except (CalibrationError, capture.CollectionError, OSError, ValueError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
