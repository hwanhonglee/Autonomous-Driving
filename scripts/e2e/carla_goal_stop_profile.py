"""HH_260906 - Pure opt-in goal-stop planning and measured capture quality checks."""

from __future__ import annotations

from dataclasses import asdict, dataclass, replace
import hashlib
import math
from pathlib import Path
import sys
from typing import Any, Mapping, Sequence


@dataclass(frozen=True)
class ComfortableGoalStopConfig:
    """HH_260906 - Freeze this first trial; commanded comfort is not measured feasibility."""

    profile_id: str = "comfortable_v1"
    desired_deceleration_mps2: float = 1.5
    target_acceleration_limit_mps2: float = 1.5
    stop_buffer_m: float = 0.75
    goal_tolerance_m: float = 1.0
    stopped_speed_mps: float = 0.1
    hold_seconds: float = 2.0
    normal_brake_cap: float = 0.10
    normal_throttle_cap: float | None = None
    minimum_tail_seconds: float = 6.5
    maximum_projection_step_m: float = 1.0
    maximum_projection_error_m: float = 3.0


def configuration_from_args(args: Any) -> ComfortableGoalStopConfig | None:
    """HH_260906 - Require explicit compatible settings without changing legacy defaults."""
    profile = getattr(args, "goal_stop_profile", "disabled")
    if profile == "disabled":
        return None
    if profile not in ("comfortable_v1", "comfortable_v2"):
        raise ValueError("unknown goal-stop profile")
    config = ComfortableGoalStopConfig()
    if profile == "comfortable_v2":
        # HH_260906 - Joint calibration responds to measured launch spikes and braking lag, not looser QA.
        config = replace(config, profile_id=profile, desired_deceleration_mps2=1.0,
                         normal_throttle_cap=0.20, normal_brake_cap=0.15)
    expected = {"physics_hz": 20.0, "capture_hz": 10.0, "goal_tolerance_m": 1.0}
    for name, value in expected.items():
        if float(getattr(args, name)) != value:
            raise ValueError(f"{profile} requires {name}={value}")
    speed = float(args.target_speed_kmh)
    tail = float(getattr(args, "stationary_tail_sec", 0.0))
    if not math.isfinite(speed) or not 0.0 < speed <= 30.0:
        raise ValueError(f"{profile} requires target speed in (0, 30] km/h")
    if not math.isfinite(tail) or tail < config.minimum_tail_seconds:
        raise ValueError(f"{profile} requires at least 6.5 seconds of tail")
    return config


def source_motion_bounds() -> dict[str, Any]:
    """HH_260906 - Import real asymmetric gate limits only for the opt-in profile."""
    root = Path(__file__).resolve().parents[2]
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))
    from portable_e2e.model import PHYSICAL_MAXIMUM_ACCELERATION_MPS2
    from portable_e2e.runtime_contract import RuntimeGateConfig

    gate = RuntimeGateConfig()
    return {
        "physical_decoder": {
            "maximum_acceleration_mps2": PHYSICAL_MAXIMUM_ACCELERATION_MPS2,
            "maximum_deceleration_mps2": PHYSICAL_MAXIMUM_ACCELERATION_MPS2,
        },
        "runtime_speed_rate_gate": {
            "maximum_acceleration_mps2": gate.maximum_acceleration_mps2,
            "maximum_deceleration_mps2": gate.maximum_deceleration_mps2,
        },
        "source_sha256": {
            name: hashlib.sha256((root / name).read_bytes()).hexdigest()
            for name in ("portable_e2e/model.py", "portable_e2e/runtime_contract.py")
        },
        "notice": "Speed-rate checks alone do not establish complete decoder or runtime feasibility.",
    }


def bounded_route_projection(
    points: Sequence[Mapping[str, Any]], x: float, y: float,
    previous_progress_m: float, maximum_step_m: float,
) -> tuple[float, float, int]:
    """HH_260906 - Clip segment projections to a contiguous one-step arc window, not nearby loops."""
    if not all(math.isfinite(value) for value in (x, y, previous_progress_m, maximum_step_m)):
        raise ValueError("route projection inputs must be finite")
    if previous_progress_m < 0.0 or maximum_step_m <= 0.0:
        raise ValueError("route projection progress and step are invalid")
    start = previous_progress_m
    end = min(float(points[-1]["distance_m"]), start + maximum_step_m)
    best = None
    for index, (first, second) in enumerate(zip(points, points[1:])):
        first_arc, second_arc = float(first["distance_m"]), float(second["distance_m"])
        if second_arc < start or first_arc > end or second_arc <= first_arc:
            continue
        dx, dy = float(second["x"]) - float(first["x"]), float(second["y"]) - float(first["y"])
        squared_length = dx * dx + dy * dy
        if squared_length <= 1.0e-12:
            continue
        ratio = ((x - float(first["x"])) * dx + (y - float(first["y"])) * dy) / squared_length
        ratio = max(max(0.0, (start - first_arc) / (second_arc - first_arc)),
                    min(min(1.0, (end - first_arc) / (second_arc - first_arc)), ratio))
        progress = first_arc + ratio * (second_arc - first_arc)
        error = math.hypot(x - float(first["x"]) - ratio * dx, y - float(first["y"]) - ratio * dy)
        candidate = (error, progress, index)
        if best is None or candidate < best:
            best = candidate
    if best is None:
        raise ValueError("no contiguous route segment is available")
    return best[1], best[0], best[2]


def terminal_overshoot_m(points: Sequence[Mapping[str, Any]], x: float, y: float) -> float:
    """HH_260906 - Retain signed endpoint overshoot that clamped route progress cannot reveal."""
    final = points[-1]
    for first in reversed(points[:-1]):
        dx, dy = float(final["x"]) - float(first["x"]), float(final["y"]) - float(first["y"])
        length = math.hypot(dx, dy)
        if length > 1.0e-9:
            return ((x - float(final["x"])) * dx + (y - float(final["y"])) * dy) / length
    raise ValueError("route has no terminal tangent")


class GoalStopGovernor:
    """HH_260906 - Base completion on measured stop dwell, never BasicAgent.done alone."""

    def __init__(self, config: ComfortableGoalStopConfig, cruise_speed_mps: float, physics_hz: float):
        self.config = config
        self.cruise_speed_mps = cruise_speed_mps
        self.dt = 1.0 / physics_hz
        self.required_hold_ticks = math.ceil(config.hold_seconds * physics_hz)
        self.hold_ticks = 0
        self.was_stopped = False
        self.target_speed_mps = 0.0
        self.complete = False

    def update(self, remaining_arc_m: float, planar_distance_m: float, speed_mps: float,
               terminal_overshoot: float, *, count_hold: bool = True) -> dict[str, Any]:
        if not all(math.isfinite(v) for v in (remaining_arc_m, planar_distance_m, speed_mps, terminal_overshoot)):
            raise ValueError("goal-stop measurements must be finite")
        if min(remaining_arc_m, planar_distance_m, speed_mps) < 0.0:
            raise ValueError("goal-stop distances and speed must be nonnegative")
        cfg = self.config
        in_window = remaining_arc_m <= cfg.goal_tolerance_m and planar_distance_m <= cfg.goal_tolerance_m
        upstream = remaining_arc_m > 1.0e-6
        stopped = in_window and upstream and terminal_overshoot <= 0.0 and speed_mps <= cfg.stopped_speed_mps
        if count_hold:
            # HH_260906 - Count elapsed intervals, not the first stopped observation itself.
            self.hold_ticks = self.hold_ticks + 1 if stopped and self.was_stopped else 0
            self.was_stopped = stopped
        self.complete = self.hold_ticks >= self.required_hold_ticks
        envelope = min(self.cruise_speed_mps, math.sqrt(
            2.0 * cfg.desired_deceleration_mps2 * max(remaining_arc_m - cfg.stop_buffer_m, 0.0)))
        if stopped:
            envelope = 0.0
        self.target_speed_mps = min(envelope, self.target_speed_mps + cfg.target_acceleration_limit_mps2 * self.dt)
        phase = "complete" if self.complete else "holding" if stopped else "approach" if envelope < self.cruise_speed_mps else "cruise"
        return {
            "profile_id": cfg.profile_id, "phase": phase,
            "remaining_route_arc_m": remaining_arc_m, "planar_goal_distance_m": planar_distance_m,
            "terminal_overshoot_m": terminal_overshoot, "measured_speed_mps": speed_mps,
            "distance_speed_envelope_mps": envelope, "target_speed_mps": self.target_speed_mps,
            "goal_window": in_window, "upstream_of_endpoint": upstream,
            "hold_ticks": self.hold_ticks, "required_hold_ticks": self.required_hold_ticks,
            "hold_duration_sec": self.hold_ticks * self.dt, "complete": self.complete,
        }


def goal_stop_termination_reason(record: Mapping[str, Any], basic_agent_done: bool,
                                 cross_track_error_m: float,
                                 config: ComfortableGoalStopConfig) -> str | None:
    """HH_260906 - Fail early on incomplete planner endpoints; permit only measured goal dwell."""
    if cross_track_error_m > config.maximum_projection_error_m:
        return "comfortable_goal_route_projection_failure"
    if not record["upstream_of_endpoint"] or (record["goal_window"] and record["terminal_overshoot_m"] > 0.0):
        return "comfortable_goal_endpoint_overshoot"
    if record["complete"]:
        return "comfortable_goal_measured_stop_and_dwell"
    if basic_agent_done and not record["goal_window"]:
        return "basic_agent_done_before_comfortable_goal_window"
    return None


def measured_goal_completion(record: Mapping[str, Any], cross_track_error_m: float,
                             config: ComfortableGoalStopConfig) -> bool:
    """HH_260906 - Keep verified dwell meaningful in tail ticks only while the vehicle remains stopped."""
    return bool(record["complete"] and record["goal_window"] and record["upstream_of_endpoint"]
                and record["measured_speed_mps"] <= config.stopped_speed_mps
                and record["terminal_overshoot_m"] <= 0.0
                and cross_track_error_m <= config.maximum_projection_error_m)


def complete_terminal_plan(plan: Sequence[Any], goal_waypoint: Any,
                           requested_goal_xyz: Mapping[str, float],
                           sampling_resolution_m: float) -> tuple[list[Any], dict[str, Any]]:
    """HH_260906 - Append the real requested endpoint omitted by the planner's two-sample early exit."""
    if len(plan) < 2 or goal_waypoint is None or not math.isfinite(sampling_resolution_m) or sampling_resolution_m <= 0:
        raise ValueError("comfortable terminal plan requires a real goal waypoint and a nonempty sampled route")
    previous, road_option = plan[-1]
    identity = lambda wp: (int(wp.road_id), int(wp.section_id), int(wp.lane_id))
    if identity(previous) != identity(goal_waypoint):
        raise ValueError("comfortable terminal goal must stay on the same terminal road/section/lane")
    def position(wp: Any) -> dict[str, float]:
        loc = wp.transform.location
        return {name: float(getattr(loc, name)) for name in ("x", "y", "z")}
    old, goal = position(previous), position(goal_waypoint)
    values = [*old.values(), *goal.values(), *requested_goal_xyz.values()]
    if not all(math.isfinite(value) for value in values):
        raise ValueError("comfortable terminal positions must be finite")
    projection_error = math.hypot(goal["x"] - requested_goal_xyz["x"], goal["y"] - requested_goal_xyz["y"])
    if projection_error > 0.25:
        raise ValueError("comfortable terminal map projection differs from the requested goal by more than 0.25 m")
    gap = math.sqrt(sum((goal[name] - old[name]) ** 2 for name in ("x", "y", "z")))
    if gap > 2.0 * sampling_resolution_m + 0.25:
        raise ValueError("comfortable terminal gap exceeds the documented two-sample planner endpoint window")
    yaw = math.radians(float(previous.transform.rotation.yaw))
    longitudinal_gap = (goal["x"] - old["x"]) * math.cos(yaw) + (goal["y"] - old["y"]) * math.sin(yaw)
    if longitudinal_gap < -1.0e-3:
        raise ValueError("comfortable terminal goal lies behind the existing final waypoint")
    updated = list(plan)
    appended = gap > 1.0e-6
    if appended:
        updated.append((goal_waypoint, road_option))
    return updated, {
        "policy": "Append actual map-projected requested goal-center waypoint; never extend the catalog route or invent a waypoint.",
        "appended_goal_waypoint": appended, "original_plan_points": len(plan), "effective_plan_points": len(updated),
        "original_terminal_carla_xyz": old, "requested_goal_center_carla_xyz": dict(requested_goal_xyz),
        "effective_terminal_carla_xyz": goal, "terminal_road_section_lane": list(identity(goal_waypoint)),
        "original_to_goal_gap_m": gap, "map_projection_planar_error_m": projection_error,
        "maximum_map_projection_error_m": 0.25, "maximum_terminal_gap_m": 2.0 * sampling_resolution_m + 0.25,
        "catalog_route_changed": False, "goal_tolerance_changed": False,
    }


def install_normal_brake_cap(local_planner: Any, cap: float,
                             throttle_cap: float | None = None) -> dict[str, Any]:
    """HH_260906 - Cap planner braking BEFORE BasicAgent adds its untouched emergency override."""
    controller = local_planner._vehicle_controller
    original_cap = float(controller.max_brake)
    original_throttle = float(controller.max_throt) if throttle_cap is not None else None
    original_run_step = local_planner.run_step
    controller.max_brake = cap
    if throttle_cap is not None:
        controller.max_throt = throttle_cap

    def normal_step(*args: Any, **kwargs: Any) -> Any:
        control = original_run_step(*args, **kwargs)
        # HH_260906 - Empty-queue braking bypasses PID; cap it here, not at the agent output.
        control.brake = min(float(control.brake), cap)
        if throttle_cap is not None:
            control.throttle = min(float(control.throttle), throttle_cap)
        return control

    local_planner.run_step = normal_step
    return {"original_pid_max_brake": original_cap, "normal_brake_cap": cap,
            "original_pid_max_throttle": original_throttle, "normal_throttle_cap": throttle_cap,
            "empty_queue_brake_capped_before_hazard_override": True,
            "basic_agent_emergency_brake_modified": False}


def measured_stop_quality(records: Sequence[Mapping[str, Any]], camera_frames: Sequence[int],
                          config: ComfortableGoalStopConfig, bounds: Mapping[str, Any],
                          completed: bool) -> dict[str, Any]:
    """HH_260906 - Include moving-tail boundary intervals at native and actual camera cadences."""
    frame_set = set(camera_frames)
    # HH_260906 - A short suffix can have perfect cadence while silently omitting almost all images.
    expected_frames = {r["frame"] for r in records[::2]}
    coverage_ok = frame_set == expected_frames and len(camera_frames) == len(frame_set)
    measurements = {}
    for cadence, selected in (("native_20hz", list(records)),
                              ("camera_10hz", [r for r in records if r["frame"] in frame_set])):
        intervals = []
        for previous, current in zip(selected, selected[1:]):
            dt = float(current["timestamp"]) - float(previous["timestamp"])
            if not math.isfinite(dt) or dt <= 0.0:
                raise ValueError("measured stop quality requires increasing finite timestamps")
            first_speed = math.hypot(float(previous["vx"]), float(previous["vy"]))
            speed = math.hypot(float(current["vx"]), float(current["vy"]))
            rate = (speed - first_speed) / dt
            if not math.isfinite(rate):
                raise ValueError("measured stop quality requires finite speed rates")
            intervals.append({"from_frame": previous["frame"], "to_frame": current["frame"],
                              "from_phase": previous["capture_phase"], "to_phase": current["capture_phase"],
                              "from_timestamp": previous["timestamp"], "to_timestamp": current["timestamp"],
                              "from_speed_mps": first_speed, "to_speed_mps": speed,
                              "dt_sec": dt, "speed_rate_mps2": rate})
        cadence_report = {}
        expected_dt = 0.05 if cadence == "native_20hz" else 0.1
        cadence_report["cadence_violation_count"] = sum(abs(r["dt_sec"] - expected_dt) > 1.0e-4 for r in intervals)
        for phase in ("all", "stationary_warmup", "driving", "stationary_tail"):
            subset = intervals if phase == "all" else [r for r in intervals if r["to_phase"] == phase]
            rates = [r["speed_rate_mps2"] for r in subset]
            checks = {}
            for name in ("physical_decoder", "runtime_speed_rate_gate"):
                limits = bounds[name]
                violations = [r for r in subset if r["speed_rate_mps2"] > limits["maximum_acceleration_mps2"] + 1.0e-9
                              or r["speed_rate_mps2"] < -limits["maximum_deceleration_mps2"] - 1.0e-9]
                checks[name] = {"violation_count": len(violations), "violation_intervals": violations,
                                "speed_rate_check_pass": bool(subset) and not violations}
            cadence_report[phase] = {"interval_count": len(subset),
                                     "first_interval": dict(subset[0]) if subset else None,
                                     "phase_boundary_intervals": [r for r in subset if r["from_phase"] != r["to_phase"]],
                                     "minimum_speed_rate_mps2": min(rates) if rates else None,
                                     "maximum_speed_rate_mps2": max(rates) if rates else None,
                                     **checks}
        measurements[cadence] = cadence_report
    tail = [r for r in records if r["capture_phase"] == "stationary_tail"]
    tail_speeds = [math.hypot(float(r["vx"]), float(r["vy"])) for r in tail]
    driving = [r for r in records if r["capture_phase"] == "driving"]
    stop = driving[-1].get("goal_stop", {}) if driving else {}
    camera_tail = [r for r in tail if r["frame"] in frame_set]
    tail_ready = len(tail) >= math.ceil(config.minimum_tail_seconds * 20.0)
    tail_still = bool(tail_speeds) and max(tail_speeds) <= config.stopped_speed_mps
    tail_goal_held = bool(tail) and all(measured_goal_completion(
        r.get("goal_stop", {}), float(r.get("route_cte_m", math.inf)), config) for r in tail)
    geometry_ok = bool(driving) and all(
        float(r.get("route_cte_m", math.inf)) <= config.maximum_projection_error_m
        and (float(r.get("goal_stop", {}).get("remaining_route_arc_m", 0.0)) > config.goal_tolerance_m
             or float(r.get("goal_stop", {}).get("terminal_overshoot_m", math.inf)) <= 0.0)
        and bool(r.get("goal_stop", {}).get("upstream_of_endpoint")) for r in driving + tail)
    rates_ok = all(measurements[cadence]["all"]["physical_decoder"]["speed_rate_check_pass"]
                   and measurements[cadence]["cadence_violation_count"] == 0
                   for cadence in measurements)
    passed = bool(completed and stop.get("complete") and tail_ready and tail_still
                  and geometry_ok and rates_ok and camera_tail and coverage_ok and tail_goal_held)
    return {
        "schema": "carla_expert.comfortable_goal_stop_quality.v1", "status": "PASS" if passed else "FAIL",
        "config": asdict(config), "bounds": dict(bounds), "completed_stop_before_tail": bool(completed and stop.get("complete")),
        "driving_final_stop": dict(stop), "tail_state_count": len(tail), "tail_camera_count": len(camera_tail),
        "tail_minimum_duration_met": tail_ready, "tail_all_speeds_stopped": tail_still,
        "tail_measured_goal_held": tail_goal_held,
        "tail_maximum_speed_mps": max(tail_speeds) if tail_speeds else None,
        "moving_tail_frame_count": sum(speed > config.stopped_speed_mps for speed in tail_speeds),
        "route_geometry_check_pass": geometry_ok, "measurements": measurements,
        "camera_frame_coverage": {
            "complete": coverage_ok, "expected_count": len(expected_frames), "actual_count": len(camera_frames),
            "missing_frames": sorted(expected_frames - frame_set), "unexpected_frames": sorted(frame_set - expected_frames),
            "duplicate_count": len(camera_frames) - len(frame_set),
            "policy": "Every second recorded native state starting with the first state, exactly as the collector schedules cameras.",
        },
        "speed_definition": "hypot(measured base_link vx, measured base_link vy); finite difference over actual timestamps",
        "interval_phase_policy": "Assign each interval to its destination phase, retaining the previous-phase prefix state.",
        "notice": "Measured scalar speed-rate QA only; no learned actuation, target clipping, full runtime validation, or decoder feasibility guarantee.",
    }
