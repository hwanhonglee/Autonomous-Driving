#!/usr/bin/env python3

import copy
from bisect import bisect_right
from dataclasses import dataclass
import json
import math
from pathlib import Path

import numpy as np


COMMAND_NAMES = (
    "LEFT",
    "RIGHT",
    "STRAIGHT",
    "LANE_FOLLOW",
    "CHANGE_LANE_LEFT",
    "CHANGE_LANE_RIGHT",
)
MANEUVER_COMMANDS = {0, 1, 2, 4, 5}
DIRECTIONAL_MANEUVER_COMMANDS = {0, 1, 4, 5}
_NANOSECONDS_PER_SECOND = 1_000_000_000
_ROUTE_ENDPOINT_EPSILON_M = 1.0e-3
_INTERPOLATED_POINT_FIELDS = (
    "longitudinal_velocity_mps",
    "lateral_velocity_mps",
    "acceleration_mps2",
    "heading_rate_rps",
    "front_wheel_angle_rad",
    "rear_wheel_angle_rad",
)


def _goal_coordinate_alignment_z(payload):
    """Return the strict CARLA-map -> map Z offset for an aligned route."""
    alignment = payload.get("coordinate_alignment")
    if alignment is None:
        return 0.0, False
    if not isinstance(alignment, dict):
        raise ValueError("coordinate_alignment must be an object")
    if alignment.get("schema_version") != 1:
        raise ValueError("coordinate_alignment.schema_version must be 1")
    if (
        alignment.get("source_frame") != "carla_map"
        or alignment.get("target_frame") != "map"
    ):
        raise ValueError(
            "coordinate_alignment must describe the carla_map -> map transform"
        )
    transform = alignment.get("carla_to_map_transform")
    if not isinstance(transform, dict):
        raise ValueError("coordinate_alignment has no carla_to_map_transform object")
    values = {}
    for field in ("x_m", "y_m", "z_m", "yaw_rad"):
        raw = transform.get(field)
        if isinstance(raw, bool) or not isinstance(raw, (int, float)):
            raise ValueError(
                f"coordinate_alignment.carla_to_map_transform.{field} "
                "must be a number"
            )
        value = float(raw)
        if not math.isfinite(value):
            raise ValueError(
                f"coordinate_alignment.carla_to_map_transform.{field} "
                "must be finite"
            )
        values[field] = value
    return values["z_m"], True


def _runtime_goal_z(payload, goal_z, terminal_z):
    """Resolve legacy goal Z or validate the raw-road/aligned-runtime contract.

    Endpoint provenance and ``goal_carla_transform`` remain in the raw CARLA
    map frame even after route alignment.  ROS runtime poses are in ``map``;
    therefore an aligned route must equal the provenance road Z plus the
    strictly declared ``coordinate_alignment`` Z offset.
    """
    provenance = payload.get("goal_endpoint_provenance")
    if provenance is None:
        return goal_z, "legacy_goal_ros_pose_z"
    if not isinstance(provenance, dict):
        raise ValueError("goal_endpoint_provenance must be an object")
    endpoint_source = provenance.get("endpoint_source")
    endpoint_index = provenance.get("endpoint_index")
    if not isinstance(endpoint_source, str) or not endpoint_source:
        raise ValueError("goal_endpoint_provenance.endpoint_source is invalid")
    if (
        isinstance(endpoint_index, bool)
        or not isinstance(endpoint_index, int)
        or endpoint_index < 0
    ):
        raise ValueError("goal_endpoint_provenance.endpoint_index is invalid")

    normalization = provenance.get("terminal_z_normalization")
    if not isinstance(normalization, dict):
        raise ValueError(
            "goal_endpoint_provenance.terminal_z_normalization must be an object"
        )
    if normalization.get("policy") != "last_road_waypoint_z":
        raise ValueError("unsupported goal terminal Z normalization policy")
    numeric_fields = (
        "original_endpoint_z_m",
        "last_road_waypoint_z_m",
        "runtime_goal_z_m",
        "serialized_terminal_z_m",
        "applied_offset_m",
    )
    try:
        values = {name: float(normalization[name]) for name in numeric_fields}
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(
            "goal terminal Z normalization must contain numeric evidence"
        ) from error
    if not all(math.isfinite(value) for value in values.values()):
        raise ValueError("goal terminal Z normalization contains a non-finite value")
    raw_road_z = values["last_road_waypoint_z_m"]
    for name in ("runtime_goal_z_m", "serialized_terminal_z_m"):
        if not math.isclose(
            values[name], raw_road_z, rel_tol=0.0, abs_tol=1.0e-9
        ):
            raise ValueError(
                f"goal terminal Z normalization {name} differs from the raw "
                "last road waypoint Z"
            )
    alignment_z, aligned = _goal_coordinate_alignment_z(payload)
    expected_runtime_z = raw_road_z + alignment_z
    if not math.isclose(
        expected_runtime_z, terminal_z, rel_tol=0.0, abs_tol=1.0e-9
    ):
        raise ValueError(
            "raw goal terminal Z plus coordinate-alignment Z differs from route "
            "terminal Z"
        )
    if not math.isclose(goal_z, terminal_z, rel_tol=0.0, abs_tol=1.0e-9):
        raise ValueError("normalized goal_ros_pose.z differs from route terminal Z")
    if not math.isclose(
        values["applied_offset_m"],
        raw_road_z - values["original_endpoint_z_m"],
        rel_tol=0.0,
        abs_tol=1.0e-9,
    ):
        raise ValueError("goal terminal Z normalization applied offset is inconsistent")

    original_carla = provenance.get("original_goal_carla_transform")
    original_ros = provenance.get("original_goal_ros_pose")
    normalized_carla = payload.get("goal_carla_transform")
    if not all(
        isinstance(value, dict)
        for value in (original_carla, original_ros, normalized_carla)
    ):
        raise ValueError("goal terminal Z provenance is incomplete")
    try:
        original_z_values = (
            float(original_carla["z"]),
            float(original_ros["z"]),
        )
        normalized_carla_z = float(normalized_carla["z"])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError("goal terminal Z provenance lacks numeric Z values") from error
    if not all(
        math.isfinite(value) for value in (*original_z_values, normalized_carla_z)
    ):
        raise ValueError("goal terminal Z provenance contains a non-finite Z value")
    if not all(
        math.isclose(
            value,
            values["original_endpoint_z_m"],
            rel_tol=0.0,
            abs_tol=1.0e-9,
        )
        for value in original_z_values
    ):
        raise ValueError("original goal endpoint Z provenance is inconsistent")
    if not math.isclose(
        normalized_carla_z, raw_road_z, rel_tol=0.0, abs_tol=1.0e-9
    ):
        raise ValueError(
            "normalized goal_carla_transform.z differs from the raw last road "
            "waypoint Z"
        )
    policy = (
        "last_road_waypoint_z+coordinate_alignment_z"
        if aligned
        else "last_road_waypoint_z"
    )
    return terminal_z, policy


def route_alignment_blocked(required, aligned, heartbeat_age_sec=None, timeout_sec=None):
    """Return whether standard-route agreement must hold the VAD output stopped."""
    if not required:
        return False
    if aligned is not True:
        return True
    if heartbeat_age_sec is None and timeout_sec is None:
        return False
    if heartbeat_age_sec is None or timeout_sec is None:
        return True
    if not math.isfinite(heartbeat_age_sec) or not math.isfinite(timeout_sec):
        return True
    return heartbeat_age_sec < 0.0 or heartbeat_age_sec > timeout_sec


def zero_velocity_distance_for_goal(
    remaining_m, goal_tolerance_m, controller_stop_offset_m
):
    """Place the zero-speed point so the controller stops inside the goal."""
    return max(0.0, remaining_m - goal_tolerance_m + controller_stop_offset_m)


def trajectory_arc_lengths(points):
    """Return cumulative planar arc lengths for trajectory-like points."""
    if not points:
        return []
    distances = [0.0]
    for first, second in zip(points, points[1:]):
        distances.append(
            distances[-1]
            + math.hypot(
                second.pose.position.x - first.pose.position.x,
                second.pose.position.y - first.pose.position.y,
            )
        )
    return distances


def trajectory_sample_distances(total_distance_m, interval_m, extra_distances=()):
    """Build sorted, unique arc-length samples including both endpoints."""
    if not math.isfinite(total_distance_m) or total_distance_m < 0.0:
        raise ValueError("trajectory length must be finite and non-negative")
    if not math.isfinite(interval_m) or interval_m <= 0.0:
        raise ValueError("trajectory resample interval must be positive and finite")

    samples = [0.0]
    step = interval_m
    while step < total_distance_m - 1.0e-6:
        samples.append(step)
        step += interval_m
    if total_distance_m > 1.0e-6:
        samples.append(total_distance_m)

    for distance in extra_distances:
        if not math.isfinite(distance) or not 0.0 < distance < total_distance_m:
            continue
        closest_index = min(
            range(len(samples)), key=lambda index: abs(samples[index] - distance)
        )
        if abs(samples[closest_index] - distance) <= 1.0e-6:
            samples[closest_index] = float(distance)
        else:
            samples.append(float(distance))

    samples.sort()
    unique = []
    for distance in samples:
        if not unique or distance - unique[-1] > 1.0e-9:
            unique.append(distance)
    return unique


def _duration_nanoseconds(duration):
    return duration.sec * _NANOSECONDS_PER_SECOND + duration.nanosec


def _normalized_trajectory_point(source):
    point = copy.deepcopy(source)
    orientation = point.pose.orientation
    norm = math.sqrt(
        orientation.x * orientation.x
        + orientation.y * orientation.y
        + orientation.z * orientation.z
        + orientation.w * orientation.w
    )
    if norm <= 1.0e-9:
        raise ValueError("cannot normalize a zero-norm quaternion")
    orientation.x /= norm
    orientation.y /= norm
    orientation.z /= norm
    orientation.w /= norm
    return point


def _interpolate_trajectory_point(first, second, ratio):
    point = copy.deepcopy(first)
    first_position = first.pose.position
    second_position = second.pose.position
    for axis in ("x", "y", "z"):
        start = getattr(first_position, axis)
        end = getattr(second_position, axis)
        setattr(point.pose.position, axis, start + ratio * (end - start))

    first_quaternion = first.pose.orientation
    second_quaternion = second.pose.orientation
    first_values = [
        first_quaternion.x,
        first_quaternion.y,
        first_quaternion.z,
        first_quaternion.w,
    ]
    second_values = [
        second_quaternion.x,
        second_quaternion.y,
        second_quaternion.z,
        second_quaternion.w,
    ]
    if sum(a * b for a, b in zip(first_values, second_values)) < 0.0:
        second_values = [-value for value in second_values]
    interpolated = [
        start + ratio * (end - start)
        for start, end in zip(first_values, second_values)
    ]
    norm = math.sqrt(sum(value * value for value in interpolated))
    if norm <= 1.0e-9:
        raise ValueError("cannot interpolate a zero-norm quaternion")
    for field, value in zip(("x", "y", "z", "w"), interpolated):
        setattr(point.pose.orientation, field, value / norm)

    for field in _INTERPOLATED_POINT_FIELDS:
        start = getattr(first, field)
        end = getattr(second, field)
        setattr(point, field, start + ratio * (end - start))

    first_time = _duration_nanoseconds(first.time_from_start)
    second_time = _duration_nanoseconds(second.time_from_start)
    interpolated_time = int(round(first_time + ratio * (second_time - first_time)))
    point.time_from_start.sec, point.time_from_start.nanosec = divmod(
        max(0, interpolated_time), _NANOSECONDS_PER_SECOND
    )
    return point


def resample_trajectory_points(points, interval_m, extra_distances=()):
    """Densify trajectory-like points at fixed planar arc-length intervals."""
    if len(points) < 2:
        return copy.deepcopy(list(points))
    arc_lengths = trajectory_arc_lengths(points)
    total_distance = arc_lengths[-1]
    if total_distance <= 1.0e-6:
        raise ValueError("trajectory has no planar extent")

    sample_distances = trajectory_sample_distances(
        total_distance, interval_m, extra_distances
    )
    result = []
    segment_index = 0
    for sample_distance in sample_distances:
        if sample_distance <= 1.0e-6:
            result.append(_normalized_trajectory_point(points[0]))
            continue
        if sample_distance >= total_distance - 1.0e-6:
            result.append(_normalized_trajectory_point(points[-1]))
            continue

        while (
            segment_index + 1 < len(points) - 1
            and arc_lengths[segment_index + 1] < sample_distance - 1.0e-9
        ):
            segment_index += 1
        while (
            segment_index + 1 < len(points)
            and arc_lengths[segment_index + 1] - arc_lengths[segment_index] <= 1.0e-9
        ):
            segment_index += 1
        if segment_index + 1 >= len(points):
            result.append(_normalized_trajectory_point(points[-1]))
            continue
        segment_length = arc_lengths[segment_index + 1] - arc_lengths[segment_index]
        ratio = (sample_distance - arc_lengths[segment_index]) / segment_length
        result.append(
            _interpolate_trajectory_point(
                points[segment_index], points[segment_index + 1], ratio
            )
        )
    return result


def endpoint_fixed_whittaker(xy, strength, fixed_indices=()):
    """Smooth planar samples with fixed endpoints and optional fixed anchors."""
    coordinates = np.asarray(xy, dtype=float)
    if coordinates.ndim != 2 or coordinates.shape[1] != 2:
        raise ValueError("xy must be an Nx2 array")
    if len(coordinates) < 3:
        raise ValueError("geometry smoothing requires at least three points")
    if not np.all(np.isfinite(coordinates)):
        raise ValueError("geometry smoothing input contains a non-finite value")
    if not math.isfinite(strength) or strength < 0.0:
        raise ValueError("geometry smoothing strength must be finite and non-negative")
    if strength == 0.0:
        return coordinates.copy()

    count = len(coordinates)
    anchors = {0, count - 1}
    for raw_index in fixed_indices:
        index = int(raw_index)
        if index != raw_index or not 0 <= index < count:
            raise ValueError(f"invalid geometry smoothing anchor index {raw_index!r}")
        anchors.add(index)

    second_difference = np.zeros((count - 2, count), dtype=float)
    rows = np.arange(count - 2)
    second_difference[rows, rows] = 1.0
    second_difference[rows, rows + 1] = -2.0
    second_difference[rows, rows + 2] = 1.0
    system = np.eye(count) + strength * (
        second_difference.T @ second_difference
    )

    fixed = np.asarray(sorted(anchors), dtype=int)
    free = np.asarray(
        [index for index in range(count) if index not in anchors], dtype=int
    )
    result = np.empty_like(coordinates)
    result[fixed] = coordinates[fixed]
    if len(free) == 0:
        return result
    right_hand_side = (
        coordinates[free] - system[np.ix_(free, fixed)] @ result[fixed]
    )
    try:
        result[free] = np.linalg.solve(
            system[np.ix_(free, free)], right_hand_side
        )
    except np.linalg.LinAlgError as error:
        raise ValueError(f"geometry smoothing solver failed: {error}") from error
    if not np.all(np.isfinite(result)):
        raise ValueError("geometry smoothing solver produced a non-finite value")
    return result


@dataclass(frozen=True)
class RoutePoint:
    x: float
    y: float
    z: float
    yaw: float
    distance_m: float
    vad_command: int
    road_option: str


@dataclass(frozen=True)
class Projection:
    progress_m: float
    cross_track_error_m: float
    segment_index: int


@dataclass(frozen=True)
class RouteSample:
    x: float
    y: float
    z: float
    yaw: float
    distance_m: float


@dataclass(frozen=True)
class RouteOffsetProfile:
    """Route-progress indexed lateral offsets from a published trajectory."""

    progress_m: tuple
    lateral_offset_m: tuple


@dataclass(frozen=True)
class GeometrySmoothingResult:
    correction_m: float
    stop_distance_m: float | None


class RoutePlan:
    def __init__(self, metadata, points):
        if len(points) < 2:
            raise ValueError("route must contain at least two points")
        self.metadata = metadata
        self.points = tuple(points)
        self.distances = tuple(point.distance_m for point in self.points)
        self.length_m = self.points[-1].distance_m
        if not math.isfinite(self.length_m) or self.length_m <= 0.0:
            raise ValueError("route length must be positive and finite")
        previous = -math.inf
        for point in self.points:
            values = (point.x, point.y, point.z, point.yaw, point.distance_m)
            if not all(math.isfinite(value) for value in values):
                raise ValueError("route contains a non-finite value")
            if point.distance_m < previous:
                raise ValueError("route distance must be monotonic")
            if point.vad_command not in range(len(COMMAND_NAMES)):
                raise ValueError(f"invalid VAD command {point.vad_command}")
            previous = point.distance_m

    @classmethod
    def load(cls, path):
        route_path = Path(path).expanduser().resolve()
        payload = json.loads(route_path.read_text(encoding="utf-8"))
        return cls.from_payload(payload, route_file=route_path)

    @classmethod
    def from_payload(cls, payload, *, route_file="<memory>"):
        # HH_260906 - Parse caller-owned bytes without reopening a security-pinned route.
        route_path = str(route_file)
        # HH_260906 - Convert malformed route structures into stable validation failures.
        if not isinstance(payload, dict):
            raise ValueError(f"route payload must be an object in {route_path}")
        if payload.get("schema_version") != 1:
            raise ValueError(f"unsupported route schema in {route_path}")
        if payload.get("coordinate_reference", "base_link") != "base_link":
            raise ValueError(f"route coordinates must use base_link in {route_path}")
        route_items = payload.get("route")
        if not isinstance(route_items, list):
            raise ValueError(f"route must be a list in {route_path}")
        points = []
        for index, item in enumerate(route_items):
            if not isinstance(item, dict):
                raise ValueError(f"route point {index} must be an object in {route_path}")
            try:
                point = RoutePoint(
                    x=float(item["x"]),
                    y=float(item["y"]),
                    z=float(item.get("z", 0.0)),
                    yaw=float(item["yaw"]),
                    distance_m=float(item["distance_m"]),
                    vad_command=int(item["vad_command"]),
                    road_option=str(item["road_option"]),
                )
            except (KeyError, TypeError, ValueError) as error:
                raise ValueError(
                    f"route point {index} contains invalid fields in {route_path}"
                ) from error
            points.append(point)
        if len(points) < 2:
            raise ValueError("route must contain at least two points")

        goal_pose = payload.get("goal_ros_pose")
        if goal_pose is not None:
            if not isinstance(goal_pose, dict):
                raise ValueError("goal_ros_pose must be an object")
            try:
                goal_x, goal_y, goal_z, goal_yaw = (
                    float(goal_pose[name]) for name in ("x", "y", "z", "yaw")
                )
            except (KeyError, TypeError, ValueError) as error:
                raise ValueError(
                    "goal_ros_pose must contain numeric x, y, z, and yaw values"
                ) from error
            if not all(
                math.isfinite(value)
                for value in (goal_x, goal_y, goal_z, goal_yaw)
            ):
                raise ValueError("goal_ros_pose contains a non-finite value")

            terminal = points[-1]
            endpoint_gap_m = math.hypot(goal_x - terminal.x, goal_y - terminal.y)
            runtime_goal_z, runtime_goal_z_policy = _runtime_goal_z(
                payload, goal_z, terminal.z
            )
            exact_goal = RoutePoint(
                x=goal_x,
                y=goal_y,
                z=runtime_goal_z,
                yaw=goal_yaw,
                distance_m=(
                    terminal.distance_m + endpoint_gap_m
                    if endpoint_gap_m > _ROUTE_ENDPOINT_EPSILON_M
                    else terminal.distance_m
                ),
                vad_command=terminal.vad_command,
                road_option=terminal.road_option,
            )
            if endpoint_gap_m > _ROUTE_ENDPOINT_EPSILON_M:
                points.append(exact_goal)
            else:
                points[-1] = exact_goal

        metadata = {**payload, "route_file": route_path}
        if goal_pose is not None:
            metadata["route_length_m"] = points[-1].distance_m
            metadata["runtime_goal_z_m"] = points[-1].z
            metadata["runtime_goal_z_policy"] = runtime_goal_z_policy
        return cls(metadata, points)

    def project(self, x, y, previous_progress_m=0.0, backtrack_m=3.0, forward_m=80.0):
        start_distance = max(0.0, previous_progress_m - backtrack_m)
        end_distance = min(self.length_m, previous_progress_m + forward_m)
        best = None
        for index in range(len(self.points) - 1):
            first = self.points[index]
            second = self.points[index + 1]
            if second.distance_m < start_distance or first.distance_m > end_distance:
                continue
            dx = second.x - first.x
            dy = second.y - first.y
            squared_length = dx * dx + dy * dy
            if squared_length <= 1.0e-9:
                ratio = 0.0
            else:
                ratio = max(0.0, min(1.0, ((x - first.x) * dx + (y - first.y) * dy) / squared_length))
            projected_x = first.x + ratio * dx
            projected_y = first.y + ratio * dy
            error = math.hypot(x - projected_x, y - projected_y)
            progress = first.distance_m + ratio * (second.distance_m - first.distance_m)
            candidate = (error, -progress, index, progress)
            if best is None or candidate < best:
                best = candidate
        if best is None:
            raise ValueError("no route segment is available in the projection window")
        return Projection(
            progress_m=max(previous_progress_m, best[3]),
            cross_track_error_m=best[0],
            segment_index=best[2],
        )

    def command_at(self, progress_m, lookahead_m, exit_lookahead_m=0.0):
        progress_m = max(0.0, min(self.length_m, progress_m))
        current_index = len(self.points) - 1
        for index, point in enumerate(self.points):
            if point.distance_m + 1.0e-6 >= progress_m:
                current_index = index
                break
        current_command = self.points[current_index].vad_command
        limit = progress_m + max(0.0, lookahead_m)
        if current_command == 2:
            # CARLA can emit STRAIGHT immediately before LEFT/RIGHT at one
            # compound junction. Do not let that transitional command mask the
            # directional maneuver that the model must anticipate.
            for point in self.points[current_index:]:
                if point.distance_m > limit:
                    break
                if point.vad_command in DIRECTIONAL_MANEUVER_COMMANDS:
                    return point.vad_command

        if current_command in MANEUVER_COMMANDS:
            exit_limit = progress_m + max(0.0, exit_lookahead_m)
            for point in self.points[current_index:]:
                if point.distance_m > exit_limit:
                    break
                if point.vad_command not in MANEUVER_COMMANDS:
                    return point.vad_command
            return current_command
        first_maneuver = None
        for point in self.points[current_index:]:
            if point.distance_m > limit:
                break
            if point.vad_command in DIRECTIONAL_MANEUVER_COMMANDS:
                return point.vad_command
            if first_maneuver is None and point.vad_command in MANEUVER_COMMANDS:
                first_maneuver = point.vad_command
        return 3 if first_maneuver is None else first_maneuver

    def remaining(self, progress_m):
        return max(0.0, self.length_m - progress_m)

    def sample_at(self, distance_m):
        distance_m = max(0.0, min(self.length_m, distance_m))
        if distance_m <= 0.0:
            point = self.points[0]
            return RouteSample(point.x, point.y, point.z, point.yaw, distance_m)
        if distance_m >= self.length_m:
            point = self.points[-1]
            return RouteSample(point.x, point.y, point.z, point.yaw, distance_m)

        index = max(0, min(len(self.points) - 2, bisect_right(self.distances, distance_m) - 1))
        first = self.points[index]
        second = self.points[index + 1]
        span = second.distance_m - first.distance_m
        ratio = 0.0 if span <= 1.0e-9 else (distance_m - first.distance_m) / span
        yaw_delta = math.atan2(
            math.sin(second.yaw - first.yaw), math.cos(second.yaw - first.yaw)
        )
        return RouteSample(
            x=first.x + ratio * (second.x - first.x),
            y=first.y + ratio * (second.y - first.y),
            z=first.z + ratio * (second.z - first.z),
            yaw=first.yaw + ratio * yaw_delta,
            distance_m=distance_m,
        )

    @property
    def goal(self):
        return self.points[-1]


def _recompute_trajectory_headings(points, route, progress_m):
    distances = trajectory_arc_lengths(points)
    for index, point in enumerate(points):
        if index + 1 < len(points):
            other = points[index + 1]
            dx = other.pose.position.x - point.pose.position.x
            dy = other.pose.position.y - point.pose.position.y
        else:
            other = points[index - 1]
            dx = point.pose.position.x - other.pose.position.x
            dy = point.pose.position.y - other.pose.position.y
        yaw = math.atan2(dy, dx) if math.hypot(dx, dy) > 1.0e-6 else None
        if yaw is None:
            yaw = route.sample_at(progress_m + distances[index]).yaw
        point.pose.orientation.x = 0.0
        point.pose.orientation.y = 0.0
        point.pose.orientation.z = math.sin(yaw * 0.5)
        point.pose.orientation.w = math.cos(yaw * 0.5)


def _corridor_entry_envelope(
    distance_m,
    entry_distance_m,
    anchor_radial_error_m,
    anchor_lateral_offset_m,
    corridor_half_width_m,
    lateral_offset_min_m,
    lateral_offset_max_m,
):
    """Return hard-corridor bounds that close smoothly after the raw point zero."""
    if entry_distance_m <= 0.0:
        return (
            corridor_half_width_m,
            lateral_offset_min_m,
            lateral_offset_max_m,
        )
    ratio = min(1.0, max(0.0, distance_m / entry_distance_m))
    blend = ratio**3 * (10.0 + ratio * (-15.0 + 6.0 * ratio))
    initial_radial_bound = max(anchor_radial_error_m, corridor_half_width_m)
    initial_lateral_min = min(anchor_lateral_offset_m, lateral_offset_min_m)
    initial_lateral_max = max(anchor_lateral_offset_m, lateral_offset_max_m)
    return (
        initial_radial_bound
        + blend * (corridor_half_width_m - initial_radial_bound),
        initial_lateral_min
        + blend * (lateral_offset_min_m - initial_lateral_min),
        initial_lateral_max
        + blend * (lateral_offset_max_m - initial_lateral_max),
    )


def _c1_saturate_positive(value, limit, transition_width):
    """Return a bounded value with a continuous first derivative.

    ``transition_width`` defines an input band on both sides of ``limit``.
    Below ``limit - transition_width`` this is the identity; above
    ``limit + transition_width`` it is the constant ``limit``.  The quadratic
    bridge has matching slopes one and zero at those boundaries.  A zero width
    intentionally reduces to the legacy hard clamp.
    """
    if transition_width <= 0.0:
        return min(value, limit)
    inner = limit - transition_width
    outer = limit + transition_width
    if value <= inner:
        return value
    if value >= outer:
        return limit
    ratio = (value - inner) / (2.0 * transition_width)
    return inner + transition_width * (2.0 * ratio - ratio * ratio)


def endpoint_tapered_c1_corridor_offsets(
    offsets_xy,
    normals_xy,
    sample_distances,
    corridor_half_width_m,
    transition_width_m,
    endpoint_taper_m,
    lateral_offset_min_m=None,
    lateral_offset_max_m=None,
    fixed_indices=(),
):
    """Pure C1 corridor saturation for route-relative planar offsets.

    The returned array never aliases or mutates ``offsets_xy``.  Saturation is
    C1 across the corridor transition band.  Its correction is multiplied by
    a product of quintic smoothsteps between fixed positions, so correction
    value and station derivative are zero at every endpoint/stop anchor when
    the incoming route-relative offset is C1.  If that tangent-preserving
    blend cannot remain in the corridor, the helper rejects it rather than
    silently falling back to a hard corner.

    Fixed positions are validated and copied exactly.  This helper is
    deliberately opt-in: the route manager does not select it.
    """
    offsets = np.asarray(offsets_xy, dtype=float)
    normals = np.asarray(normals_xy, dtype=float)
    distances = np.asarray(sample_distances, dtype=float)
    if offsets.ndim != 2 or offsets.shape[1] != 2:
        raise ValueError("corridor offsets must be an Nx2 array")
    if normals.shape != offsets.shape:
        raise ValueError("corridor normals must match the offset array")
    if distances.ndim != 1 or len(distances) != len(offsets):
        raise ValueError("corridor sample distances must match the offset array")
    if len(offsets) < 2:
        raise ValueError("C1 corridor requires at least two points")
    if not (
        np.all(np.isfinite(offsets))
        and np.all(np.isfinite(normals))
        and np.all(np.isfinite(distances))
    ):
        raise ValueError("C1 corridor input contains a non-finite value")
    if np.any(np.diff(distances) < -1.0e-9):
        raise ValueError("C1 corridor sample distances must be monotonic")
    if not math.isfinite(corridor_half_width_m) or corridor_half_width_m <= 0.0:
        raise ValueError("C1 corridor half width must be positive and finite")
    if (
        not math.isfinite(transition_width_m)
        or transition_width_m < 0.0
        or transition_width_m > corridor_half_width_m
    ):
        raise ValueError(
            "C1 corridor transition width must be finite and inside the corridor"
        )
    if not math.isfinite(endpoint_taper_m) or endpoint_taper_m < 0.0:
        raise ValueError("C1 corridor endpoint taper must be finite and non-negative")

    normal_lengths = np.linalg.norm(normals, axis=1)
    if np.any(np.abs(normal_lengths - 1.0) > 1.0e-6):
        raise ValueError("C1 corridor normals must be unit length")
    lateral_min = (
        -corridor_half_width_m
        if lateral_offset_min_m is None
        else float(lateral_offset_min_m)
    )
    lateral_max = (
        corridor_half_width_m
        if lateral_offset_max_m is None
        else float(lateral_offset_max_m)
    )
    if not all(math.isfinite(value) for value in (lateral_min, lateral_max)):
        raise ValueError("C1 corridor lateral bounds must be finite")
    if (
        lateral_min < -corridor_half_width_m
        or lateral_max > corridor_half_width_m
        or lateral_min > 0.0
        or lateral_max < 0.0
        or lateral_min >= lateral_max
    ):
        raise ValueError(
            "C1 corridor lateral bounds must straddle zero inside the corridor"
        )

    anchors = {0, len(offsets) - 1}
    for raw_index in fixed_indices:
        index = int(raw_index)
        if index != raw_index or not 0 <= index < len(offsets):
            raise ValueError(f"invalid C1 corridor anchor index {raw_index!r}")
        anchors.add(index)
    ordered_anchors = sorted(anchors)

    result = offsets.copy()
    symmetric_lateral_bounds = math.isclose(
        lateral_min, -corridor_half_width_m, rel_tol=0.0, abs_tol=1.0e-12
    ) and math.isclose(
        lateral_max, corridor_half_width_m, rel_tol=0.0, abs_tol=1.0e-12
    )
    for index, offset in enumerate(offsets):
        radial_error = float(np.linalg.norm(offset))
        lateral_offset = float(np.dot(offset, normals[index]))
        if index in anchors:
            if (
                radial_error > corridor_half_width_m + 1.0e-6
                or lateral_offset < lateral_min - 1.0e-6
                or lateral_offset > lateral_max + 1.0e-6
            ):
                raise ValueError(
                    f"C1 corridor fixed point {index} lies outside corridor"
                )
            continue

        if endpoint_taper_m == 0.0:
            correction_taper = 1.0
        else:
            left_anchor = max(anchor for anchor in ordered_anchors if anchor < index)
            right_anchor = min(
                anchor for anchor in ordered_anchors if anchor > index
            )
            left_ratio = min(
                1.0,
                max(
                    0.0,
                    (distances[index] - distances[left_anchor]) / endpoint_taper_m,
                ),
            )
            right_ratio = min(
                1.0,
                max(
                    0.0,
                    (distances[right_anchor] - distances[index]) / endpoint_taper_m,
                ),
            )
            left_taper = left_ratio**3 * (
                10.0 + left_ratio * (-15.0 + 6.0 * left_ratio)
            )
            right_taper = right_ratio**3 * (
                10.0 + right_ratio * (-15.0 + 6.0 * right_ratio)
            )
            correction_taper = left_taper * right_taper

        saturated = offset.copy()

        constrained_radial_error = _c1_saturate_positive(
            radial_error, corridor_half_width_m, transition_width_m
        )
        if radial_error > 1.0e-12 and constrained_radial_error < radial_error:
            saturated *= constrained_radial_error / radial_error

        # A radial bound already proves the symmetric lateral bound.  Avoid a
        # second saturation pass, which would otherwise shrink the same normal
        # component twice on straight routes.
        if not symmetric_lateral_bounds:
            lateral_offset = float(np.dot(saturated, normals[index]))
            tangent_component = saturated - lateral_offset * normals[index]
            if lateral_offset > lateral_max:
                magnitude = _c1_saturate_positive(
                    lateral_offset,
                    lateral_max,
                    min(transition_width_m, lateral_max),
                )
                saturated = tangent_component + magnitude * normals[index]
            elif lateral_offset < lateral_min:
                magnitude = _c1_saturate_positive(
                    -lateral_offset,
                    -lateral_min,
                    min(transition_width_m, -lateral_min),
                )
                saturated = tangent_component - magnitude * normals[index]

        result[index] = offset + correction_taper * (saturated - offset)
        final_radial_error = float(np.linalg.norm(result[index]))
        final_lateral_offset = float(np.dot(result[index], normals[index]))
        if (
            final_radial_error > corridor_half_width_m + 1.0e-6
            or final_lateral_offset < lateral_min - 1.0e-6
            or final_lateral_offset > lateral_max + 1.0e-6
        ):
            raise ValueError(
                "C1 corridor endpoint taper cannot preserve the anchor tangent "
                f"at point {index} without leaving the corridor"
            )

    return result


def constrain_trajectory_points_to_route(
    points,
    route,
    progress_m,
    corridor_half_width_m,
    mode="hard",
    lateral_offset_min_m=None,
    lateral_offset_max_m=None,
    entry_distance_m=0.0,
):
    """Constrain an E2E trajectory to a route corridor and return max correction.

    ``hard`` preserves the original clamp exactly. ``soft`` uses a smooth tanh
    saturation, avoiding the derivative discontinuity where a path first meets
    the corridor boundary.
    """
    if not math.isfinite(corridor_half_width_m) or corridor_half_width_m <= 0.0:
        raise ValueError("route corridor half width must be positive and finite")
    if mode not in ("hard", "soft"):
        raise ValueError(f"unsupported route corridor mode {mode!r}")
    if not math.isfinite(entry_distance_m) or entry_distance_m < 0.0:
        raise ValueError("route corridor entry distance must be finite and non-negative")
    if entry_distance_m > 0.0 and mode != "hard":
        raise ValueError("route corridor entry ramp requires hard mode")
    lateral_offset_min_m = (
        -corridor_half_width_m
        if lateral_offset_min_m is None
        else float(lateral_offset_min_m)
    )
    lateral_offset_max_m = (
        corridor_half_width_m
        if lateral_offset_max_m is None
        else float(lateral_offset_max_m)
    )
    if not all(
        math.isfinite(value)
        for value in (lateral_offset_min_m, lateral_offset_max_m)
    ):
        raise ValueError("route corridor lateral bounds must be finite")
    if (
        lateral_offset_min_m < -corridor_half_width_m
        or lateral_offset_max_m > corridor_half_width_m
        or lateral_offset_min_m >= lateral_offset_max_m
    ):
        raise ValueError("route corridor lateral bounds must lie inside the corridor")
    if len(points) < 2:
        raise ValueError("route corridor requires at least two trajectory points")
    distances = trajectory_arc_lengths(points)
    if distances[-1] <= 1.0e-6:
        raise ValueError("route corridor trajectory has no planar extent")

    anchor_reference = route.sample_at(progress_m)
    anchor_dx = points[0].pose.position.x - anchor_reference.x
    anchor_dy = points[0].pose.position.y - anchor_reference.y
    anchor_radial_error_m = math.hypot(anchor_dx, anchor_dy)
    anchor_lateral_offset_m = (
        anchor_dx * -math.sin(anchor_reference.yaw)
        + anchor_dy * math.cos(anchor_reference.yaw)
    )

    # Keep a short, non-zero stopping horizon at the exact route end. Everything
    # farther ahead would otherwise be projected onto the same goal position.
    minimum_horizon_m = min(0.1, distances[-1])
    cutoff_distance_m = min(
        distances[-1], max(route.remaining(progress_m), minimum_horizon_m)
    )
    geometry_changed = cutoff_distance_m < distances[-1] - 1.0e-6
    if geometry_changed:
        right_index = bisect_right(distances, cutoff_distance_m)
        if (
            right_index > 0
            and abs(distances[right_index - 1] - cutoff_distance_m) <= 1.0e-6
        ):
            points[:] = points[:right_index]
        else:
            first_index = max(0, right_index - 1)
            second_index = min(len(points) - 1, first_index + 1)
            span = distances[second_index] - distances[first_index]
            if span <= 1.0e-9:
                raise ValueError("cannot truncate an overlapping trajectory segment")
            ratio = (cutoff_distance_m - distances[first_index]) / span
            terminal = _interpolate_trajectory_point(
                points[first_index], points[second_index], ratio
            )
            points[:] = points[: first_index + 1] + [terminal]
        distances = trajectory_arc_lengths(points)

    maximum_correction = 0.0
    for point, distance in zip(points, distances):
        reference = route.sample_at(progress_m + distance)
        (
            local_corridor_half_width_m,
            local_lateral_offset_min_m,
            local_lateral_offset_max_m,
        ) = _corridor_entry_envelope(
            distance,
            entry_distance_m,
            anchor_radial_error_m,
            anchor_lateral_offset_m,
            corridor_half_width_m,
            lateral_offset_min_m,
            lateral_offset_max_m,
        )
        original_x = point.pose.position.x
        original_y = point.pose.position.y
        dx = point.pose.position.x - reference.x
        dy = point.pose.position.y - reference.y
        error = math.hypot(dx, dy)
        if mode == "hard":
            constrained_error = min(error, local_corridor_half_width_m)
        else:
            constrained_error = (
                0.0
                if error <= 1.0e-9
                else local_corridor_half_width_m
                * math.tanh(error / local_corridor_half_width_m)
            )
        if error > 1.0e-9 and constrained_error < error - 1.0e-12:
            scale = constrained_error / error
            point.pose.position.x = reference.x + dx * scale
            point.pose.position.y = reference.y + dy * scale

        tangent_x = math.cos(reference.yaw)
        tangent_y = math.sin(reference.yaw)
        normal_x = -tangent_y
        normal_y = tangent_x
        dx = point.pose.position.x - reference.x
        dy = point.pose.position.y - reference.y
        lateral_offset = dx * normal_x + dy * normal_y
        constrained_lateral_offset = min(
            max(lateral_offset, local_lateral_offset_min_m),
            local_lateral_offset_max_m,
        )
        if abs(constrained_lateral_offset - lateral_offset) > 1.0e-12:
            lateral_delta = constrained_lateral_offset - lateral_offset
            point.pose.position.x += lateral_delta * normal_x
            point.pose.position.y += lateral_delta * normal_y

        correction = math.hypot(
            point.pose.position.x - original_x, point.pose.position.y - original_y
        )
        if correction > 1.0e-9:
            maximum_correction = max(maximum_correction, correction)
            geometry_changed = True

    deduplicated = [points[0]]
    for point in points[1:]:
        previous = deduplicated[-1]
        separation = math.hypot(
            point.pose.position.x - previous.pose.position.x,
            point.pose.position.y - previous.pose.position.y,
        )
        if separation > 1.0e-4:
            deduplicated.append(point)
    if len(deduplicated) < 2:
        raise ValueError("route corridor collapsed the trajectory")
    if len(deduplicated) != len(points):
        points[:] = deduplicated
        geometry_changed = True

    if geometry_changed:
        _recompute_trajectory_headings(points, route, progress_m)
    return maximum_correction


def _recompute_centered_tangent_headings(points):
    for index, point in enumerate(points):
        left = max(0, index - 1)
        right = min(len(points) - 1, index + 1)
        dx = points[right].pose.position.x - points[left].pose.position.x
        dy = points[right].pose.position.y - points[left].pose.position.y
        if math.hypot(dx, dy) <= 1.0e-9 and index + 1 < len(points):
            dx = points[index + 1].pose.position.x - point.pose.position.x
            dy = points[index + 1].pose.position.y - point.pose.position.y
        if math.hypot(dx, dy) <= 1.0e-9 and index > 0:
            dx = point.pose.position.x - points[index - 1].pose.position.x
            dy = point.pose.position.y - points[index - 1].pose.position.y
        if math.hypot(dx, dy) <= 1.0e-9:
            raise ValueError(f"geometry smoothing has a zero tangent at point {index}")
        yaw = math.atan2(dy, dx)
        point.pose.orientation.x = 0.0
        point.pose.orientation.y = 0.0
        point.pose.orientation.z = math.sin(yaw * 0.5)
        point.pose.orientation.w = math.cos(yaw * 0.5)


def _nearest_route_reference(route, x, y, expected_progress_m, window_m):
    lower = max(0.0, expected_progress_m - window_m)
    upper = min(route.length_m, expected_progress_m + window_m)
    best = None
    for index, (first, second) in enumerate(
        zip(route.points, route.points[1:])
    ):
        if second.distance_m < lower or first.distance_m > upper:
            continue
        distance_span = second.distance_m - first.distance_m
        if distance_span <= 1.0e-9:
            minimum_ratio = maximum_ratio = 0.0
        else:
            minimum_ratio = max(
                0.0, (lower - first.distance_m) / distance_span
            )
            maximum_ratio = min(
                1.0, (upper - first.distance_m) / distance_span
            )
        dx = second.x - first.x
        dy = second.y - first.y
        squared_length = dx * dx + dy * dy
        if squared_length <= 1.0e-9:
            ratio = minimum_ratio
        else:
            ratio = ((x - first.x) * dx + (y - first.y) * dy) / squared_length
            ratio = min(max(ratio, minimum_ratio), maximum_ratio)
        projected_x = first.x + ratio * dx
        projected_y = first.y + ratio * dy
        progress = first.distance_m + ratio * distance_span
        error = math.hypot(x - projected_x, y - projected_y)
        candidate = (error, abs(progress - expected_progress_m), -progress, index)
        if best is None or candidate < best[0]:
            best = (candidate, progress, projected_x, projected_y)
    if best is None:
        raise ValueError("no route segment is available for smoothing corridor")
    _, progress, projected_x, projected_y = best
    sample = route.sample_at(progress)
    return RouteSample(
        projected_x, projected_y, sample.z, sample.yaw, progress
    )


def _constrain_smoothed_geometry_to_route(
    points,
    sample_distances,
    route,
    progress_m,
    corridor_half_width_m,
    search_window_m,
    mode,
    lateral_offset_min_m,
    lateral_offset_max_m,
    entry_distance_m=0.0,
    fixed_indices=(),
):
    if len(points) != len(sample_distances):
        raise ValueError("smoothing corridor station count mismatch")
    if not math.isfinite(corridor_half_width_m) or corridor_half_width_m <= 0.0:
        raise ValueError("route corridor half width must be positive and finite")
    if mode not in ("hard", "soft"):
        raise ValueError(f"unsupported route corridor mode {mode!r}")
    if not math.isfinite(entry_distance_m) or entry_distance_m < 0.0:
        raise ValueError("route corridor entry distance must be finite and non-negative")
    if entry_distance_m > 0.0 and mode != "hard":
        raise ValueError("route corridor entry ramp requires hard mode")
    lateral_offset_min_m = (
        -corridor_half_width_m
        if lateral_offset_min_m is None
        else float(lateral_offset_min_m)
    )
    lateral_offset_max_m = (
        corridor_half_width_m
        if lateral_offset_max_m is None
        else float(lateral_offset_max_m)
    )
    if not all(
        math.isfinite(value)
        for value in (lateral_offset_min_m, lateral_offset_max_m)
    ):
        raise ValueError("route corridor lateral bounds must be finite")
    if (
        lateral_offset_min_m < -corridor_half_width_m
        or lateral_offset_max_m > corridor_half_width_m
        or lateral_offset_min_m >= lateral_offset_max_m
    ):
        raise ValueError("route corridor lateral bounds must lie inside the corridor")

    references = [
        _nearest_route_reference(
            route,
            point.pose.position.x,
            point.pose.position.y,
            progress_m + sample_distance,
            search_window_m,
        )
        for point, sample_distance in zip(points, sample_distances)
    ]
    anchor_reference = references[0]
    anchor_dx = points[0].pose.position.x - anchor_reference.x
    anchor_dy = points[0].pose.position.y - anchor_reference.y
    anchor_radial_error_m = math.hypot(anchor_dx, anchor_dy)
    anchor_lateral_offset_m = (
        anchor_dx * -math.sin(anchor_reference.yaw)
        + anchor_dy * math.cos(anchor_reference.yaw)
    )
    fixed_indices = set(fixed_indices)
    for index, (point, sample_distance, reference) in enumerate(
        zip(points, sample_distances, references)
    ):
        (
            local_corridor_half_width_m,
            local_lateral_offset_min_m,
            local_lateral_offset_max_m,
        ) = _corridor_entry_envelope(
            sample_distance,
            entry_distance_m,
            anchor_radial_error_m,
            anchor_lateral_offset_m,
            corridor_half_width_m,
            lateral_offset_min_m,
            lateral_offset_max_m,
        )
        dx = point.pose.position.x - reference.x
        dy = point.pose.position.y - reference.y
        error = math.hypot(dx, dy)
        tangent_x = math.cos(reference.yaw)
        tangent_y = math.sin(reference.yaw)
        normal_x = -tangent_y
        normal_y = tangent_x
        lateral_offset = dx * normal_x + dy * normal_y
        if index in fixed_indices:
            tolerance_m = 1.0e-6
            if (
                error > local_corridor_half_width_m + tolerance_m
                or lateral_offset < local_lateral_offset_min_m - tolerance_m
                or lateral_offset > local_lateral_offset_max_m + tolerance_m
            ):
                raise ValueError(
                    f"geometry smoothing fixed point {index} lies outside corridor"
                )
            continue
        if mode == "hard":
            constrained_error = min(error, local_corridor_half_width_m)
        else:
            constrained_error = (
                0.0
                if error <= 1.0e-9
                else local_corridor_half_width_m
                * math.tanh(error / local_corridor_half_width_m)
            )
        if error > 1.0e-9 and constrained_error < error - 1.0e-12:
            scale = constrained_error / error
            point.pose.position.x = reference.x + dx * scale
            point.pose.position.y = reference.y + dy * scale

        dx = point.pose.position.x - reference.x
        dy = point.pose.position.y - reference.y
        lateral_offset = dx * normal_x + dy * normal_y
        constrained_lateral_offset = min(
            max(lateral_offset, local_lateral_offset_min_m),
            local_lateral_offset_max_m,
        )
        if abs(constrained_lateral_offset - lateral_offset) > 1.0e-12:
            lateral_delta = constrained_lateral_offset - lateral_offset
            point.pose.position.x += lateral_delta * normal_x
            point.pose.position.y += lateral_delta * normal_y

    if any(
        math.hypot(
            second.pose.position.x - first.pose.position.x,
            second.pose.position.y - first.pose.position.y,
        )
        <= 1.0e-4
        for first, second in zip(points, points[1:])
    ):
        raise ValueError("geometry smoothing corridor collapsed a segment")


def _constrain_smoothed_geometry_to_route_endpoint_tapered_c1(
    points,
    sample_distances,
    route,
    progress_m,
    corridor_half_width_m,
    search_window_m,
    lateral_offset_min_m,
    lateral_offset_max_m,
    transition_width_m,
    endpoint_taper_m,
    entry_distance_m=0.0,
    fixed_indices=(),
):
    """Apply the isolated endpoint-tapered C1 prototype to smoothed points."""
    if len(points) != len(sample_distances):
        raise ValueError("C1 smoothing corridor station count mismatch")
    if entry_distance_m != 0.0:
        raise ValueError("C1 smoothing corridor does not support an entry ramp")
    references = [
        _nearest_route_reference(
            route,
            point.pose.position.x,
            point.pose.position.y,
            progress_m + sample_distance,
            search_window_m,
        )
        for point, sample_distance in zip(points, sample_distances)
    ]
    offsets = np.asarray(
        [
            (
                point.pose.position.x - reference.x,
                point.pose.position.y - reference.y,
            )
            for point, reference in zip(points, references)
        ],
        dtype=float,
    )
    normals = np.asarray(
        [(-math.sin(reference.yaw), math.cos(reference.yaw)) for reference in references],
        dtype=float,
    )
    constrained = endpoint_tapered_c1_corridor_offsets(
        offsets,
        normals,
        sample_distances,
        corridor_half_width_m,
        transition_width_m,
        endpoint_taper_m,
        lateral_offset_min_m=lateral_offset_min_m,
        lateral_offset_max_m=lateral_offset_max_m,
        fixed_indices=fixed_indices,
    )
    for point, reference, offset in zip(points, references, constrained):
        point.pose.position.x = reference.x + float(offset[0])
        point.pose.position.y = reference.y + float(offset[1])

    if any(
        math.hypot(
            second.pose.position.x - first.pose.position.x,
            second.pose.position.y - first.pose.position.y,
        )
        <= 1.0e-4
        for first, second in zip(points, points[1:])
    ):
        raise ValueError("C1 smoothing corridor collapsed a segment")


def _maximum_bidirectional_polyline_deviation(first_xy, second_xy):
    def point_to_polyline_distances(points, polyline):
        starts = polyline[:-1]
        vectors = polyline[1:] - starts
        length_squared = np.einsum("ij,ij->i", vectors, vectors)
        valid = length_squared > 1.0e-12
        if not np.any(valid):
            raise ValueError("geometry smoothing comparison path has no extent")
        relative = points[:, None, :] - starts[None, :, :]
        fractions = np.zeros((len(points), len(vectors)), dtype=float)
        fractions[:, valid] = np.clip(
            np.einsum("pij,ij->pi", relative[:, valid], vectors[valid])
            / length_squared[valid][None, :],
            0.0,
            1.0,
        )
        nearest = (
            starts[None, :, :] + fractions[:, :, None] * vectors[None, :, :]
        )
        squared = np.einsum(
            "pij,pij->pi", points[:, None, :] - nearest, points[:, None, :] - nearest
        )
        squared[:, ~valid] = np.inf
        return np.sqrt(np.min(squared, axis=1))

    forward = point_to_polyline_distances(first_xy, second_xy)
    reverse = point_to_polyline_distances(second_xy, first_xy)
    return float(max(np.max(forward), np.max(reverse)))


def smooth_trajectory_geometry(
    points,
    route,
    progress_m,
    strength,
    interval_m,
    max_deviation_m,
    corridor_half_width_m,
    mode="hard",
    lateral_offset_min_m=None,
    lateral_offset_max_m=None,
    stop_distance_m=None,
    entry_distance_m=0.0,
    corridor_saturation_mode="legacy",
    corridor_transition_width_m=0.0,
    corridor_endpoint_taper_m=0.0,
):
    """Apply guarded Whittaker smoothing and return correction/stop station.

    A zero strength is an exact no-op.  For an enabled pass, the endpoint and an
    in-horizon stop-distance sample are fixed.  The route corridor is applied to
    the tentative result before the guarded geometry replaces ``points``.
    """
    if not math.isfinite(strength) or strength < 0.0:
        raise ValueError("geometry smoothing strength must be finite and non-negative")
    if strength == 0.0:
        return GeometrySmoothingResult(0.0, stop_distance_m)
    if not math.isfinite(interval_m) or interval_m <= 0.0:
        raise ValueError("geometry smoothing interval must be positive and finite")
    if not math.isfinite(max_deviation_m) or max_deviation_m <= 0.0:
        raise ValueError(
            "geometry smoothing maximum deviation must be positive and finite"
        )
    if len(points) < 3:
        raise ValueError("geometry smoothing requires at least three points")
    if stop_distance_m is not None and not math.isfinite(stop_distance_m):
        raise ValueError("geometry smoothing stop distance must be finite")
    if corridor_saturation_mode not in ("legacy", "endpoint_tapered_c1"):
        raise ValueError(
            f"unsupported smoothing corridor saturation {corridor_saturation_mode!r}"
        )
    if corridor_saturation_mode == "legacy" and (
        corridor_transition_width_m != 0.0 or corridor_endpoint_taper_m != 0.0
    ):
        raise ValueError("legacy smoothing corridor does not accept C1 parameters")

    source_distances = trajectory_arc_lengths(points)
    if source_distances[-1] <= 1.0e-6:
        raise ValueError("geometry smoothing trajectory has no planar extent")
    extra_distances = ()
    if stop_distance_m is not None:
        extra_distances = (stop_distance_m,)
    baseline = resample_trajectory_points(points, interval_m, extra_distances)
    baseline_sample_distances = trajectory_sample_distances(
        source_distances[-1], interval_m, extra_distances
    )
    if len(baseline_sample_distances) != len(baseline):
        raise ValueError("geometry smoothing resample station count mismatch")
    baseline_xy = np.asarray(
        [
            (point.pose.position.x, point.pose.position.y)
            for point in baseline
        ],
        dtype=float,
    )

    fixed_indices = []
    stop_anchor_index = None
    if (
        stop_distance_m is not None
        and 0.0 <= stop_distance_m <= source_distances[-1]
    ):
        stop_index = min(
            range(len(baseline_sample_distances)),
            key=lambda index: abs(
                baseline_sample_distances[index] - stop_distance_m
            ),
        )
        if (
            abs(
                baseline_sample_distances[stop_index]
                - stop_distance_m
            )
            <= 1.0e-5
        ):
            fixed_indices.append(stop_index)
            stop_anchor_index = stop_index

    smoothed_xy = endpoint_fixed_whittaker(
        baseline_xy, strength, fixed_indices=fixed_indices
    )
    candidate = copy.deepcopy(baseline)
    for point, position in zip(candidate, smoothed_xy):
        point.pose.position.x = float(position[0])
        point.pose.position.y = float(position[1])

    fixed_corridor_indices = {0, len(candidate) - 1, *fixed_indices}
    search_window_m = max(3.0, 4.0 * corridor_half_width_m, 4.0 * interval_m)
    if corridor_saturation_mode == "endpoint_tapered_c1":
        if mode != "hard":
            raise ValueError("C1 smoothing corridor prototype requires hard mode")
        _constrain_smoothed_geometry_to_route_endpoint_tapered_c1(
            candidate,
            baseline_sample_distances,
            route,
            progress_m,
            corridor_half_width_m,
            search_window_m,
            lateral_offset_min_m,
            lateral_offset_max_m,
            corridor_transition_width_m,
            corridor_endpoint_taper_m,
            entry_distance_m=entry_distance_m,
            fixed_indices=fixed_corridor_indices,
        )
    else:
        _constrain_smoothed_geometry_to_route(
            candidate,
            baseline_sample_distances,
            route,
            progress_m,
            corridor_half_width_m,
            search_window_m,
            mode=mode,
            lateral_offset_min_m=lateral_offset_min_m,
            lateral_offset_max_m=lateral_offset_max_m,
            entry_distance_m=entry_distance_m,
            fixed_indices=fixed_corridor_indices,
        )

    displacements = [
        math.hypot(
            point.pose.position.x - original[0],
            point.pose.position.y - original[1],
        )
        for point, original in zip(candidate, baseline_xy)
    ]
    candidate_xy = np.asarray(
        [
            (point.pose.position.x, point.pose.position.y)
            for point in candidate
        ],
        dtype=float,
    )
    maximum_deviation = _maximum_bidirectional_polyline_deviation(
        baseline_xy, candidate_xy
    )
    if maximum_deviation > max_deviation_m + 1.0e-9:
        raise ValueError(
            f"geometry smoothing displacement {maximum_deviation:.3f} m "
            f"exceeds {max_deviation_m:.3f} m guard"
        )

    for index in fixed_corridor_indices:
        if displacements[index] > 1.0e-8:
            raise ValueError(
                f"geometry smoothing moved fixed point {index} by "
                f"{displacements[index]:.6f} m"
            )
    _recompute_centered_tangent_headings(candidate)
    adjusted_stop_distance_m = stop_distance_m
    if stop_anchor_index is not None:
        adjusted_stop_distance_m = trajectory_arc_lengths(candidate)[
            stop_anchor_index
        ]
    points[:] = candidate
    return GeometrySmoothingResult(
        maximum_deviation, adjusted_stop_distance_m
    )


def stabilize_trajectory_lateral_offsets(
    points,
    route,
    progress_m,
    previous_profile,
    current_gain,
    anchor_distance_m,
    corridor_half_width_m,
    lateral_offset_min_m=None,
    lateral_offset_max_m=None,
    entry_distance_m=0.0,
    activation_threshold_m=0.0,
):
    """Blend overlapping route-relative offsets and return the new profile.

    The first point always follows the current VAD update. The configured gain
    is approached over ``anchor_distance_m`` so temporal filtering cannot leave
    the controller with a trajectory detached from the current ego pose.
    """
    if len(points) < 2:
        raise ValueError("trajectory stabilization requires at least two points")
    if not math.isfinite(current_gain) or not 0.0 < current_gain <= 1.0:
        raise ValueError("trajectory stabilization gain must be in (0, 1]")
    if not math.isfinite(anchor_distance_m) or anchor_distance_m <= 0.0:
        raise ValueError("trajectory stabilization anchor distance must be positive")
    if not math.isfinite(corridor_half_width_m) or corridor_half_width_m <= 0.0:
        raise ValueError("route corridor half width must be positive and finite")
    if not math.isfinite(entry_distance_m) or entry_distance_m < 0.0:
        raise ValueError("route corridor entry distance must be finite and non-negative")
    if not math.isfinite(activation_threshold_m) or activation_threshold_m < 0.0:
        raise ValueError("filter activation threshold must be finite and non-negative")
    lateral_offset_min_m = (
        -corridor_half_width_m
        if lateral_offset_min_m is None
        else float(lateral_offset_min_m)
    )
    lateral_offset_max_m = (
        corridor_half_width_m
        if lateral_offset_max_m is None
        else float(lateral_offset_max_m)
    )
    if (
        not math.isfinite(lateral_offset_min_m)
        or not math.isfinite(lateral_offset_max_m)
        or lateral_offset_min_m < -corridor_half_width_m
        or lateral_offset_max_m > corridor_half_width_m
        or lateral_offset_min_m >= lateral_offset_max_m
        or lateral_offset_min_m > 0.0
        or lateral_offset_max_m < 0.0
    ):
        raise ValueError("trajectory stabilization lateral bounds are invalid")

    distances = trajectory_arc_lengths(points)
    if distances[-1] <= 1.0e-6:
        raise ValueError("trajectory stabilization path has no planar extent")
    route_progress = [progress_m + distance for distance in distances]
    anchor_reference = route.sample_at(progress_m)
    anchor_dx = points[0].pose.position.x - anchor_reference.x
    anchor_dy = points[0].pose.position.y - anchor_reference.y
    anchor_radial_error_m = math.hypot(anchor_dx, anchor_dy)
    anchor_lateral_offset_m = (
        anchor_dx * -math.sin(anchor_reference.yaw)
        + anchor_dy * math.cos(anchor_reference.yaw)
    )

    previous_progress = ()
    previous_offset = ()
    if previous_profile is not None:
        previous_progress = previous_profile.progress_m
        previous_offset = previous_profile.lateral_offset_m
        if len(previous_progress) != len(previous_offset) or len(previous_progress) < 2:
            raise ValueError("previous route offset profile is malformed")
        if any(
            second <= first
            for first, second in zip(previous_progress, previous_progress[1:])
        ):
            raise ValueError("previous route offset progress is not increasing")

    filter_active = previous_profile is not None
    if filter_active and activation_threshold_m > 0.0:
        overlap_deltas = []
        for point, station in zip(points, route_progress):
            if not previous_progress[0] <= station <= previous_progress[-1]:
                continue
            reference = route.sample_at(station)
            dx = point.pose.position.x - reference.x
            dy = point.pose.position.y - reference.y
            current_lateral = (
                dx * -math.sin(reference.yaw) + dy * math.cos(reference.yaw)
            )
            index = max(0, bisect_right(previous_progress, station) - 1)
            index = min(index, len(previous_progress) - 2)
            span = previous_progress[index + 1] - previous_progress[index]
            ratio = (station - previous_progress[index]) / span
            old_lateral = previous_offset[index] + ratio * (
                previous_offset[index + 1] - previous_offset[index]
            )
            overlap_deltas.append(abs(current_lateral - old_lateral))
        filter_active = bool(overlap_deltas) and (
            max(overlap_deltas) > activation_threshold_m
        )

    maximum_correction = 0.0
    lateral_offsets = []
    geometry_changed = False
    for point, distance, station in zip(points, distances, route_progress):
        reference = route.sample_at(station)
        tangent_x = math.cos(reference.yaw)
        tangent_y = math.sin(reference.yaw)
        normal_x = -tangent_y
        normal_y = tangent_x
        dx = point.pose.position.x - reference.x
        dy = point.pose.position.y - reference.y
        longitudinal = dx * tangent_x + dy * tangent_y
        lateral = dx * normal_x + dy * normal_y

        if filter_active and previous_progress[0] <= station <= previous_progress[-1]:
            index = max(0, bisect_right(previous_progress, station) - 1)
            index = min(index, len(previous_progress) - 2)
            span = previous_progress[index + 1] - previous_progress[index]
            ratio = (station - previous_progress[index]) / span
            old_lateral = previous_offset[index] + ratio * (
                previous_offset[index + 1] - previous_offset[index]
            )
            effective_gain = current_gain + (1.0 - current_gain) * math.exp(
                -distance / anchor_distance_m
            )
            lateral = (
                effective_gain * lateral + (1.0 - effective_gain) * old_lateral
            )

        (
            local_corridor_half_width_m,
            local_lateral_offset_min_m,
            local_lateral_offset_max_m,
        ) = _corridor_entry_envelope(
            distance,
            entry_distance_m,
            anchor_radial_error_m,
            anchor_lateral_offset_m,
            corridor_half_width_m,
            lateral_offset_min_m,
            lateral_offset_max_m,
        )
        radial_error = math.hypot(longitudinal, lateral)
        if radial_error > local_corridor_half_width_m:
            scale = local_corridor_half_width_m / radial_error
            longitudinal *= scale
            lateral *= scale
        lateral = min(
            max(lateral, local_lateral_offset_min_m),
            local_lateral_offset_max_m,
        )

        new_x = reference.x + longitudinal * tangent_x + lateral * normal_x
        new_y = reference.y + longitudinal * tangent_y + lateral * normal_y
        correction = math.hypot(
            new_x - point.pose.position.x, new_y - point.pose.position.y
        )
        if correction > 1.0e-9:
            point.pose.position.x = new_x
            point.pose.position.y = new_y
            maximum_correction = max(maximum_correction, correction)
            geometry_changed = True
        lateral_offsets.append(lateral)

    if geometry_changed:
        _recompute_trajectory_headings(points, route, progress_m)
    return (
        RouteOffsetProfile(tuple(route_progress), tuple(lateral_offsets)),
        maximum_correction,
    )


def trajectory_planar_curvatures(points):
    """Return signed three-point planar curvature for trajectory-like points."""
    if len(points) < 3:
        return [0.0] * len(points)
    curvature = [0.0] * len(points)
    for index in range(1, len(points) - 1):
        first = points[index - 1].pose.position
        middle = points[index].pose.position
        last = points[index + 1].pose.position
        first_length = math.hypot(middle.x - first.x, middle.y - first.y)
        second_length = math.hypot(last.x - middle.x, last.y - middle.y)
        chord_length = math.hypot(last.x - first.x, last.y - first.y)
        denominator = first_length * second_length * chord_length
        if denominator <= 1.0e-9:
            continue
        cross = (middle.x - first.x) * (last.y - first.y) - (
            middle.y - first.y
        ) * (last.x - first.x)
        curvature[index] = 2.0 * cross / denominator
    curvature[0] = curvature[1]
    curvature[-1] = curvature[-2]
    return curvature


def limit_trajectory_speed_for_curvature(
    points,
    maximum_lateral_acceleration_mps2,
    preview_distance_m,
    comfortable_deceleration_mps2,
):
    """Apply a previewed lateral-acceleration speed cap in place."""
    for name, value in (
        ("maximum lateral acceleration", maximum_lateral_acceleration_mps2),
        ("curvature preview distance", preview_distance_m),
        ("comfortable deceleration", comfortable_deceleration_mps2),
    ):
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be positive and finite")
    if len(points) < 3:
        return 0.0

    distances = trajectory_arc_lengths(points)
    curvature = trajectory_planar_curvatures(points)
    maximum_reduction = 0.0
    for index, point in enumerate(points):
        preview_end = distances[index] + preview_distance_m
        peak_curvature = max(
            abs(curvature[other_index])
            for other_index in range(index, len(points))
            if distances[other_index] <= preview_end + 1.0e-9
        )
        if peak_curvature <= 1.0e-6:
            continue
        speed_cap = math.sqrt(maximum_lateral_acceleration_mps2 / peak_curvature)
        original_speed = max(0.0, point.longitudinal_velocity_mps)
        point.longitudinal_velocity_mps = min(original_speed, speed_cap)
        maximum_reduction = max(
            maximum_reduction, original_speed - point.longitudinal_velocity_mps
        )

    for index in range(len(points) - 2, -1, -1):
        segment = distances[index + 1] - distances[index]
        following_speed = max(0.0, points[index + 1].longitudinal_velocity_mps)
        deceleration_cap = math.sqrt(
            following_speed**2 + 2.0 * comfortable_deceleration_mps2 * segment
        )
        original_speed = max(0.0, points[index].longitudinal_velocity_mps)
        points[index].longitudinal_velocity_mps = min(
            original_speed, deceleration_cap
        )
        maximum_reduction = max(
            maximum_reduction,
            original_speed - points[index].longitudinal_velocity_mps,
        )
    return maximum_reduction


def limit_trajectory_speed_for_route_curvature(
    points,
    route,
    progress_m,
    maximum_lateral_acceleration_mps2,
    lookahead_distance_m,
    comfortable_deceleration_mps2,
    sample_interval_m=0.5,
):
    """Cap speed for RoutePlan curvature beyond a short VAD horizon.

    For every output station, future route curvature is converted to a lateral
    acceleration speed limit and propagated backward with the configured
    comfortable deceleration. Route sampling errors are deliberately allowed
    to raise so the manager publishes its fail-closed stop trajectory.
    """
    for name, value in (
        ("route progress", progress_m),
        ("maximum lateral acceleration", maximum_lateral_acceleration_mps2),
        ("route curvature lookahead", lookahead_distance_m),
        ("comfortable deceleration", comfortable_deceleration_mps2),
        ("route curvature sample interval", sample_interval_m),
    ):
        if not math.isfinite(value):
            raise ValueError(f"{name} must be finite")
    if progress_m < 0.0:
        raise ValueError("route progress must be non-negative")
    if (
        maximum_lateral_acceleration_mps2 <= 0.0
        or lookahead_distance_m <= 0.0
        or comfortable_deceleration_mps2 <= 0.0
        or sample_interval_m <= 0.0
    ):
        raise ValueError("route curvature limits must be positive")
    if not points:
        return 0.0

    def curvature_at(station_m):
        first = route.sample_at(max(0.0, station_m - sample_interval_m))
        middle = route.sample_at(station_m)
        last = route.sample_at(
            min(route.length_m, station_m + sample_interval_m)
        )
        first_length = math.hypot(middle.x - first.x, middle.y - first.y)
        second_length = math.hypot(last.x - middle.x, last.y - middle.y)
        chord_length = math.hypot(last.x - first.x, last.y - first.y)
        denominator = first_length * second_length * chord_length
        if denominator <= 1.0e-9:
            return 0.0
        cross = (middle.x - first.x) * (last.y - first.y) - (
            middle.y - first.y
        ) * (last.x - first.x)
        return abs(2.0 * cross / denominator)

    local_distances = trajectory_arc_lengths(points)
    maximum_reduction = 0.0
    for point, local_distance in zip(points, local_distances):
        station = min(route.length_m, progress_m + local_distance)
        end_station = min(route.length_m, station + lookahead_distance_m)
        future_stations = trajectory_sample_distances(
            end_station - station,
            sample_interval_m,
        )
        speed_cap = math.inf
        for offset in future_stations:
            future_station = station + offset
            curvature = curvature_at(future_station)
            if curvature <= 1.0e-6:
                continue
            curve_speed = math.sqrt(
                maximum_lateral_acceleration_mps2 / curvature
            )
            speed_cap = min(
                speed_cap,
                math.sqrt(
                    curve_speed**2
                    + 2.0 * comfortable_deceleration_mps2 * offset
                ),
            )
        original_speed = max(0.0, point.longitudinal_velocity_mps)
        point.longitudinal_velocity_mps = min(original_speed, speed_cap)
        maximum_reduction = max(
            maximum_reduction,
            original_speed - point.longitudinal_velocity_mps,
        )
    return maximum_reduction


def limit_trajectory_speed_for_acceleration(
    points,
    maximum_acceleration_mps2,
    initial_speed_mps,
    initial_distance_m=0.0,
):
    """Cap forward speed increases using a distance-domain acceleration limit.

    ``initial_distance_m`` represents the short distance from the current
    vehicle state to the first trajectory sample. Supplying it explicitly
    prevents a high raw VAD speed at point zero from bypassing the ramp while
    still allowing a stopped vehicle to begin moving.
    """
    for name, value in (
        ("maximum longitudinal acceleration", maximum_acceleration_mps2),
        ("initial speed", initial_speed_mps),
        ("initial distance", initial_distance_m),
    ):
        if not math.isfinite(value):
            raise ValueError(f"{name} must be finite")
    if maximum_acceleration_mps2 <= 0.0:
        raise ValueError("maximum longitudinal acceleration must be positive")
    if initial_speed_mps < 0.0:
        raise ValueError("initial speed must be non-negative")
    if initial_distance_m < 0.0:
        raise ValueError("initial distance must be non-negative")
    if not points:
        return 0.0

    distances = trajectory_arc_lengths(points)
    previous_speed = initial_speed_mps
    previous_distance = -initial_distance_m
    maximum_reduction = 0.0
    for point, distance in zip(points, distances):
        segment = max(0.0, distance - previous_distance)
        acceleration_cap = math.sqrt(
            previous_speed**2 + 2.0 * maximum_acceleration_mps2 * segment
        )
        original_speed = max(0.0, point.longitudinal_velocity_mps)
        point.longitudinal_velocity_mps = min(original_speed, acceleration_cap)
        maximum_reduction = max(
            maximum_reduction,
            original_speed - point.longitudinal_velocity_mps,
        )
        previous_speed = point.longitudinal_velocity_mps
        previous_distance = distance
    return maximum_reduction


def limit_trajectory_speed_recovery(
    points,
    maximum_acceleration_mps2,
):
    """Limit point-to-point speed recovery without constraining point zero.

    This is intended for an explicit simulation cruise profile whose launch
    ramp is enforced by the stateful vehicle command gate. Starting from the
    already-shaped first point still prevents a curvature or terminal slowdown
    from jumping immediately back to cruise speed later in the trajectory.
    """
    if not math.isfinite(maximum_acceleration_mps2):
        raise ValueError("maximum longitudinal acceleration must be finite")
    if maximum_acceleration_mps2 <= 0.0:
        raise ValueError("maximum longitudinal acceleration must be positive")
    if len(points) < 2:
        return 0.0

    distances = trajectory_arc_lengths(points)
    previous_speed = max(0.0, points[0].longitudinal_velocity_mps)
    previous_distance = distances[0]
    maximum_reduction = 0.0
    for point, distance in zip(points[1:], distances[1:]):
        segment = max(0.0, distance - previous_distance)
        acceleration_cap = math.sqrt(
            previous_speed**2 + 2.0 * maximum_acceleration_mps2 * segment
        )
        original_speed = max(0.0, point.longitudinal_velocity_mps)
        point.longitudinal_velocity_mps = min(original_speed, acceleration_cap)
        maximum_reduction = max(
            maximum_reduction,
            original_speed - point.longitudinal_velocity_mps,
        )
        previous_speed = point.longitudinal_velocity_mps
        previous_distance = distance
    return maximum_reduction
