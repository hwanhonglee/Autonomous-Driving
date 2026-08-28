#!/usr/bin/env python3

import copy
import math
import re

import rclpy
from autoware_internal_planning_msgs.msg import CandidateTrajectories
from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, Float32, Int8, String
from visualization_msgs.msg import Marker, MarkerArray

from vad_route_logic import (
    COMMAND_NAMES,
    GeometrySmoothingResult,
    RoutePlan,
    constrain_trajectory_points_to_route,
    limit_trajectory_speed_for_curvature,
    resample_trajectory_points,
    route_alignment_blocked,
    smooth_trajectory_geometry,
    stabilize_trajectory_lateral_offsets,
    trajectory_arc_lengths,
    zero_velocity_distance_for_goal,
)


LIVE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
LATCHED_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
GENERATOR_PATTERN = re.compile(r"autoware_tensorrt_vad_cmd_([0-5])")


class VadRouteManager(Node):
    def __init__(self):
        super().__init__("vad_route_manager")
        route_file = self.declare_parameter("route_file", "").value
        if not route_file:
            raise RuntimeError("route_file parameter is required")
        self.route = RoutePlan.load(route_file)

        self.maneuver_lookahead_m = self._positive_parameter("maneuver_lookahead_m")
        self.maneuver_exit_lookahead_m = self._positive_parameter(
            "maneuver_exit_lookahead_m"
        )
        self.goal_tolerance_m = self._positive_parameter("goal_tolerance_m")
        self.goal_stop_speed_mps = self._positive_parameter("goal_stop_speed_mps")
        self.goal_hold_sec = self._positive_parameter("goal_hold_sec")
        self.controller_stop_offset_m = self._positive_parameter(
            "controller_stop_offset_m"
        )
        self.comfortable_deceleration_mps2 = self._positive_parameter(
            "comfortable_deceleration_mps2"
        )
        self.maximum_speed_mps = self._positive_parameter("maximum_speed_mps")
        self.route_corridor_half_width_m = self._positive_parameter(
            "route_corridor_half_width_m"
        )
        self.turn_inward_corridor_half_width_m = self._positive_parameter(
            "turn_inward_corridor_half_width_m"
        )
        self.turn_outward_corridor_half_width_m = self._positive_parameter(
            "turn_outward_corridor_half_width_m"
        )
        left_outward_override_m = self._nonnegative_parameter(
            "left_turn_outward_corridor_half_width_m"
        )
        right_outward_override_m = self._nonnegative_parameter(
            "right_turn_outward_corridor_half_width_m"
        )
        self.left_turn_outward_corridor_half_width_m = (
            self._resolve_turn_outward_corridor_width(
                self.turn_outward_corridor_half_width_m,
                left_outward_override_m,
                "left_turn_outward_corridor_half_width_m",
            )
        )
        self.right_turn_outward_corridor_half_width_m = (
            self._resolve_turn_outward_corridor_width(
                self.turn_outward_corridor_half_width_m,
                right_outward_override_m,
                "right_turn_outward_corridor_half_width_m",
            )
        )
        if (
            self.turn_inward_corridor_half_width_m
            > self.route_corridor_half_width_m
            or self.turn_outward_corridor_half_width_m
            > self.route_corridor_half_width_m
        ):
            raise RuntimeError(
                "turn corridor half widths must not exceed route_corridor_half_width_m"
            )
        self.route_corridor_mode = str(
            self.declare_parameter("route_corridor_mode", "hard").value
        )
        if self.route_corridor_mode not in ("hard", "soft"):
            raise RuntimeError("route_corridor_mode must be 'hard' or 'soft'")
        self.route_corridor_entry_distance_m = float(
            self.declare_parameter("route_corridor_entry_distance_m", 0.0).value
        )
        if (
            not math.isfinite(self.route_corridor_entry_distance_m)
            or self.route_corridor_entry_distance_m < 0.0
        ):
            raise RuntimeError(
                "route_corridor_entry_distance_m must be finite and non-negative"
            )
        if (
            self.route_corridor_entry_distance_m > 0.0
            and self.route_corridor_mode != "hard"
        ):
            raise RuntimeError("route corridor entry ramp requires hard mode")
        self.trajectory_lateral_filter_gain = float(
            self.declare_parameter("trajectory_lateral_filter_gain", 1.0).value
        )
        if (
            not math.isfinite(self.trajectory_lateral_filter_gain)
            or not 0.0 < self.trajectory_lateral_filter_gain <= 1.0
        ):
            raise RuntimeError("trajectory_lateral_filter_gain must be in (0, 1]")
        self.left_turn_trajectory_lateral_filter_gain = (
            self._resolve_trajectory_lateral_filter_gain(
                self.trajectory_lateral_filter_gain,
                self._nonnegative_parameter(
                    "left_turn_trajectory_lateral_filter_gain"
                ),
                "left_turn_trajectory_lateral_filter_gain",
            )
        )
        self.right_turn_trajectory_lateral_filter_gain = (
            self._resolve_trajectory_lateral_filter_gain(
                self.trajectory_lateral_filter_gain,
                self._nonnegative_parameter(
                    "right_turn_trajectory_lateral_filter_gain"
                ),
                "right_turn_trajectory_lateral_filter_gain",
            )
        )
        self.trajectory_lateral_filter_activation_threshold_m = (
            self._nonnegative_parameter(
                "trajectory_lateral_filter_activation_threshold_m"
            )
        )
        self.trajectory_lateral_filter_anchor_m = self._positive_parameter(
            "trajectory_lateral_filter_anchor_m"
        )
        self.trajectory_lateral_filter_timeout_sec = self._positive_parameter(
            "trajectory_lateral_filter_timeout_sec"
        )
        self.trajectory_geometry_smoothing_strength = float(
            self.declare_parameter(
                "trajectory_geometry_smoothing_strength", 0.0
            ).value
        )
        if (
            not math.isfinite(self.trajectory_geometry_smoothing_strength)
            or self.trajectory_geometry_smoothing_strength < 0.0
        ):
            raise RuntimeError(
                "trajectory_geometry_smoothing_strength must be finite and "
                "non-negative"
            )
        self.trajectory_geometry_smoothing_interval_m = self._positive_parameter(
            "trajectory_geometry_smoothing_interval_m"
        )
        if self.trajectory_geometry_smoothing_interval_m < 0.05:
            raise RuntimeError(
                "trajectory_geometry_smoothing_interval_m must be at least 0.05"
            )
        self.trajectory_geometry_smoothing_max_deviation_m = (
            self._positive_parameter(
                "trajectory_geometry_smoothing_max_deviation_m"
            )
        )
        self.maximum_lateral_acceleration_mps2 = float(
            self.declare_parameter("maximum_lateral_acceleration_mps2", 0.0).value
        )
        if (
            not math.isfinite(self.maximum_lateral_acceleration_mps2)
            or self.maximum_lateral_acceleration_mps2 < 0.0
        ):
            raise RuntimeError(
                "maximum_lateral_acceleration_mps2 must be finite and non-negative"
            )
        self.curvature_speed_preview_m = self._positive_parameter(
            "curvature_speed_preview_m"
        )
        self.trajectory_resample_interval_m = self._positive_parameter(
            "trajectory_resample_interval_m"
        )
        if self.trajectory_resample_interval_m < 0.05:
            raise RuntimeError("trajectory_resample_interval_m must be at least 0.05")
        self.max_route_deviation_m = self._positive_parameter("max_route_deviation_m")
        self.max_first_point_distance_m = self._positive_parameter(
            "max_first_point_distance_m"
        )
        self.max_trajectory_segment_m = self._positive_parameter(
            "max_trajectory_segment_m"
        )
        self.max_candidate_age_sec = self._positive_parameter("max_candidate_age_sec")
        self.candidate_timeout_sec = self._positive_parameter("candidate_timeout_sec")
        self.standard_route_alignment_timeout_sec = self._positive_parameter(
            "standard_route_alignment_timeout_sec"
        )
        self.route_projection_backtrack_m = self._positive_parameter(
            "route_projection_backtrack_m"
        )
        self.route_projection_forward_m = self._positive_parameter(
            "route_projection_forward_m"
        )
        self.allow_legacy_positional_candidates = self.declare_parameter(
            "allow_legacy_positional_candidates", False
        ).value
        self.require_standard_route_alignment = self.declare_parameter(
            "require_standard_route_alignment", False
        ).value

        self.odom = None
        self.progress_m = 0.0
        self.remaining_m = self.route.length_m
        self.cross_track_error_m = math.inf
        self.trajectory_correction_m = 0.0
        self.command = 3
        self.status = "waiting_for_odometry"
        self.fault = None
        self.goal_reached = False
        self.goal_stop_started_ns = None
        self.last_candidate_received_ns = None
        self.last_selected_candidate = None
        self.has_valid_candidate = False
        self.standard_route_aligned = None
        self.standard_route_alignment_received_ns = None
        self.standard_route_timeout_reported = False
        self.previous_route_offset_profile = None
        self.previous_route_offset_command = None
        self.previous_route_offset_ns = None
        self.last_geometry_smoothing_warning = None
        self.actual_path = Path()
        self.actual_path.header.frame_id = "map"

        self.trajectory_pub = self.create_publisher(
            Trajectory, "/planning/trajectory", LIVE_QOS
        )
        self.selected_raw_trajectory_pub = self.create_publisher(
            Trajectory, "/planning/vad_route/selected_raw_trajectory", LIVE_QOS
        )
        self.command_pub = self.create_publisher(
            Int8, "/planning/vad_route/command", LIVE_QOS
        )
        self.remaining_pub = self.create_publisher(
            Float32, "/planning/vad_route/remaining_distance", LIVE_QOS
        )
        self.cross_track_pub = self.create_publisher(
            Float32, "/planning/vad_route/cross_track_error", LIVE_QOS
        )
        self.trajectory_correction_pub = self.create_publisher(
            Float32, "/planning/vad_route/trajectory_correction", LIVE_QOS
        )
        self.goal_reached_pub = self.create_publisher(
            Bool, "/planning/vad_route/goal_reached", LIVE_QOS
        )
        self.status_pub = self.create_publisher(
            String, "/planning/vad_route/status", LIVE_QOS
        )
        self.route_path_pub = self.create_publisher(
            Path, "/planning/vad_route/reference_path", LATCHED_QOS
        )
        self.actual_path_pub = self.create_publisher(
            Path, "/planning/vad_route/actual_path", LIVE_QOS
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, "/planning/vad_route/markers", LIVE_QOS
        )

        self.create_subscription(
            Odometry, "/localization/kinematic_state", self._on_odometry, LIVE_QOS
        )
        self.create_subscription(
            CandidateTrajectories,
            "/planning/vad/candidate_trajectories",
            self._on_candidates,
            LIVE_QOS,
        )
        if self.require_standard_route_alignment:
            self.create_subscription(
                Bool,
                "/planning/vad_route/standard_route_aligned",
                self._on_standard_route_alignment,
                LATCHED_QOS,
            )
        self.timer = self.create_timer(0.1, self._on_timer)
        self._publish_reference_path()
        self.get_logger().info(
            f"Loaded {self.route.metadata.get('scenario', 'unknown')} route: "
            f"{self.route.length_m:.2f} m, {len(self.route.points)} points, "
            f"{self.route.metadata.get('town', 'unknown')}/"
            f"{self.route.metadata.get('weather', 'unknown')}"
        )

    def _positive_parameter(self, name):
        value = float(self.declare_parameter(name, 0.0).value)
        if not math.isfinite(value) or value <= 0.0:
            raise RuntimeError(f"{name} must be positive and finite")
        return value

    @staticmethod
    def _resolve_turn_outward_corridor_width(common_m, override_m, name):
        if not math.isfinite(override_m) or override_m < 0.0:
            raise RuntimeError(f"{name} must be finite and non-negative")
        if override_m > common_m:
            raise RuntimeError(f"{name} must not exceed the common outward width")
        return common_m if override_m == 0.0 else override_m

    @staticmethod
    def _resolve_trajectory_lateral_filter_gain(common_gain, override_gain, name):
        if not math.isfinite(override_gain) or not 0.0 <= override_gain <= 1.0:
            raise RuntimeError(f"{name} must be finite and in [0, 1]")
        return common_gain if override_gain == 0.0 else override_gain

    def _trajectory_lateral_filter_gain_for_command(self):
        if self.command == 0:
            return self.left_turn_trajectory_lateral_filter_gain
        if self.command == 1:
            return self.right_turn_trajectory_lateral_filter_gain
        return self.trajectory_lateral_filter_gain

    def _nonnegative_parameter(self, name):
        value = float(self.declare_parameter(name, 0.0).value)
        if not math.isfinite(value) or value < 0.0:
            raise RuntimeError(f"{name} must be finite and non-negative")
        return value

    @staticmethod
    def _lateral_corridor_bounds(
        command, corridor_half_width_m, inward_m, left_outward_m, right_outward_m
    ):
        if command == 0:  # LEFT: negative route lateral is outside the turn.
            return -left_outward_m, inward_m
        if command == 1:  # RIGHT: positive route lateral is outside the turn.
            return -inward_m, right_outward_m
        return -corridor_half_width_m, corridor_half_width_m

    def _on_odometry(self, msg):
        self.odom = msg
        position = msg.pose.pose.position
        try:
            projection = self.route.project(
                position.x,
                position.y,
                self.progress_m,
                self.route_projection_backtrack_m,
                self.route_projection_forward_m,
            )
        except ValueError as error:
            self._set_fault(f"route_projection:{error}")
            return
        self.progress_m = projection.progress_m
        self.remaining_m = self.route.remaining(self.progress_m)
        self.cross_track_error_m = projection.cross_track_error_m
        self.command = self.route.command_at(
            self.progress_m,
            self.maneuver_lookahead_m,
            self.maneuver_exit_lookahead_m,
        )

        if self.cross_track_error_m > self.max_route_deviation_m:
            self._set_fault(
                f"route_deviation:{self.cross_track_error_m:.2f}m>"
                f"{self.max_route_deviation_m:.2f}m"
            )

        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.actual_path.header.stamp = msg.header.stamp
        self.actual_path.poses.append(pose)
        if len(self.actual_path.poses) > 5000:
            del self.actual_path.poses[:1000]
        self.actual_path_pub.publish(self.actual_path)
        self._update_goal_state()

    def _on_standard_route_alignment(self, msg):
        aligned = bool(msg.data)
        self.standard_route_alignment_received_ns = self.get_clock().now().nanoseconds
        self.standard_route_timeout_reported = False
        if aligned != self.standard_route_aligned:
            if aligned:
                self.get_logger().info("Standard Autoware route alignment gate opened")
            else:
                self.get_logger().warning(
                    "Standard Autoware route is not aligned; holding a stop trajectory"
                )
        self.standard_route_aligned = aligned

    def _on_candidates(self, msg):
        self.last_candidate_received_ns = self.get_clock().now().nanoseconds
        if self.odom is None:
            self.status = "waiting_for_odometry"
            return
        try:
            candidates = self._candidate_command_map(msg)
            selected = candidates[self.command]
            self._validate_candidate(selected)
        except (KeyError, ValueError) as error:
            self._set_fault(f"candidate_validation:{error}")
            return

        self.last_selected_candidate = copy.deepcopy(selected)
        selected_raw = Trajectory()
        selected_raw.header = copy.deepcopy(selected.header)
        selected_raw.points = copy.deepcopy(selected.points)
        selected_raw.header.stamp = self.get_clock().now().to_msg()
        self.selected_raw_trajectory_pub.publish(selected_raw)
        try:
            if self.fault or self.goal_reached or self._route_alignment_blocked():
                output = self._stopped_trajectory(selected)
            else:
                output = self._shape_velocity(selected)
            self._validate_output_trajectory(output)
        except ValueError as error:
            self._set_fault(f"trajectory_shaping:{error}")
            try:
                output = self._stopped_trajectory(selected)
                self._validate_output_trajectory(output)
            except ValueError as stop_error:
                self.get_logger().error(
                    f"Unable to publish a safe stop trajectory: {stop_error}"
                )
                return
        self.has_valid_candidate = True
        self.trajectory_pub.publish(output)
        self._update_status(output)

    def _candidate_command_map(self, msg):
        candidates = msg.candidate_trajectories
        information = msg.generator_info
        if len(candidates) != 6 or len(information) != 6:
            raise ValueError(
                f"expected 6 candidates/info entries, got {len(candidates)}/{len(information)}"
            )

        info_by_uuid = {}
        positional_names = []
        for info in information:
            match = GENERATOR_PATTERN.fullmatch(info.generator_name.data)
            if match is None:
                raise ValueError(f"unexpected generator name {info.generator_name.data!r}")
            command = int(match.group(1))
            identifier = bytes(info.generator_id.uuid)
            if identifier in info_by_uuid:
                raise ValueError("duplicate generator UUID")
            info_by_uuid[identifier] = command
            positional_names.append(command)
        if set(positional_names) != set(range(6)):
            raise ValueError(f"generator commands are not 0..5: {positional_names}")

        mapped = {}
        for index, candidate in enumerate(candidates):
            identifier = bytes(candidate.generator_id.uuid)
            command = info_by_uuid.get(identifier)
            if command is None:
                if not self.allow_legacy_positional_candidates:
                    raise ValueError("candidate UUID does not match GeneratorInfo")
                command = positional_names[index]
            if command in mapped:
                raise ValueError(f"duplicate candidate command {command}")
            mapped[command] = candidate
        if set(mapped) != set(range(6)):
            raise ValueError(f"candidate commands are not 0..5: {sorted(mapped)}")
        return mapped

    def _validate_candidate(self, candidate):
        if candidate.header.frame_id != "map":
            raise ValueError(f"frame is {candidate.header.frame_id!r}, expected 'map'")
        if len(candidate.points) < 3:
            raise ValueError(f"trajectory has only {len(candidate.points)} points")

        now_ns = self.get_clock().now().nanoseconds
        stamp_ns = candidate.header.stamp.sec * 1_000_000_000 + candidate.header.stamp.nanosec
        age_sec = (now_ns - stamp_ns) * 1.0e-9
        if age_sec < -0.1 or age_sec > self.max_candidate_age_sec:
            raise ValueError(f"trajectory age is {age_sec:.3f} s")

        previous_time = None
        previous_position = None
        total_distance = 0.0
        for index, point in enumerate(candidate.points):
            position = point.pose.position
            orientation = point.pose.orientation
            values = (
                position.x,
                position.y,
                position.z,
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
                point.longitudinal_velocity_mps,
                point.lateral_velocity_mps,
                point.acceleration_mps2,
                point.heading_rate_rps,
                point.front_wheel_angle_rad,
                point.rear_wheel_angle_rad,
            )
            if not all(math.isfinite(value) for value in values):
                raise ValueError(f"point {index} contains a non-finite value")
            quaternion_norm = math.sqrt(
                orientation.x * orientation.x
                + orientation.y * orientation.y
                + orientation.z * orientation.z
                + orientation.w * orientation.w
            )
            if not 0.5 <= quaternion_norm <= 1.5:
                raise ValueError(f"point {index} quaternion norm is {quaternion_norm:.3f}")
            if point.longitudinal_velocity_mps < -0.1 or abs(point.longitudinal_velocity_mps) > 20.0:
                raise ValueError(
                    f"point {index} longitudinal speed is {point.longitudinal_velocity_mps:.2f} m/s"
                )
            time_sec = point.time_from_start.sec + point.time_from_start.nanosec * 1.0e-9
            if point.time_from_start.sec < 0 or point.time_from_start.nanosec >= 1_000_000_000:
                raise ValueError(f"point {index} time_from_start is invalid")
            if previous_time is not None and time_sec <= previous_time:
                raise ValueError(f"point {index} time_from_start is not increasing")
            if previous_position is not None:
                segment = math.hypot(
                    position.x - previous_position.x,
                    position.y - previous_position.y,
                )
                if segment > self.max_trajectory_segment_m:
                    raise ValueError(f"point {index} segment is {segment:.2f} m")
                total_distance += segment
            previous_time = time_sec
            previous_position = position

        if total_distance <= 1.0e-3:
            raise ValueError("trajectory has no planar extent")

        first = candidate.points[0].pose.position
        current = self.odom.pose.pose.position
        first_distance = math.hypot(first.x - current.x, first.y - current.y)
        if first_distance > self.max_first_point_distance_m:
            raise ValueError(f"first point is {first_distance:.2f} m from ego")

    def _shape_velocity(self, candidate):
        output = Trajectory()
        output.header = copy.deepcopy(candidate.header)
        output.header.stamp = self.get_clock().now().to_msg()

        # The PID enters its stopping state before the zero-velocity point.
        stop_distance = zero_velocity_distance_for_goal(
            self.remaining_m,
            self.goal_tolerance_m,
            self.controller_stop_offset_m,
        )
        raw_distances = trajectory_arc_lengths(candidate.points)
        output.points = resample_trajectory_points(
            candidate.points,
            self.trajectory_resample_interval_m,
            extra_distances=(*raw_distances[1:-1], stop_distance),
        )
        lateral_offset_min_m, lateral_offset_max_m = self._lateral_corridor_bounds(
            self.command,
            self.route_corridor_half_width_m,
            self.turn_inward_corridor_half_width_m,
            self.left_turn_outward_corridor_half_width_m,
            self.right_turn_outward_corridor_half_width_m,
        )
        self.trajectory_correction_m = constrain_trajectory_points_to_route(
            output.points,
            self.route,
            self.progress_m,
            self.route_corridor_half_width_m,
            mode=self.route_corridor_mode,
            lateral_offset_min_m=lateral_offset_min_m,
            lateral_offset_max_m=lateral_offset_max_m,
            entry_distance_m=self.route_corridor_entry_distance_m,
        )
        lateral_filter_gain = self._trajectory_lateral_filter_gain_for_command()
        if lateral_filter_gain < 1.0:
            now_ns = self.get_clock().now().nanoseconds
            previous_profile = None
            if (
                self.previous_route_offset_profile is not None
                and self.previous_route_offset_command == self.command
                and self.previous_route_offset_ns is not None
                and (now_ns - self.previous_route_offset_ns) * 1.0e-9
                <= self.trajectory_lateral_filter_timeout_sec
            ):
                previous_profile = self.previous_route_offset_profile
            profile, stabilization_correction = stabilize_trajectory_lateral_offsets(
                output.points,
                self.route,
                self.progress_m,
                previous_profile,
                lateral_filter_gain,
                self.trajectory_lateral_filter_anchor_m,
                self.route_corridor_half_width_m,
                lateral_offset_min_m=lateral_offset_min_m,
                lateral_offset_max_m=lateral_offset_max_m,
                entry_distance_m=self.route_corridor_entry_distance_m,
                activation_threshold_m=(
                    self.trajectory_lateral_filter_activation_threshold_m
                ),
            )
            self.previous_route_offset_profile = profile
            self.previous_route_offset_command = self.command
            self.previous_route_offset_ns = now_ns
            self.trajectory_correction_m = max(
                self.trajectory_correction_m, stabilization_correction
            )
        smoothing_result = self._apply_geometry_smoothing(
            output.points,
            stop_distance,
            lateral_offset_min_m,
            lateral_offset_max_m,
        )
        self.trajectory_correction_m = max(
            self.trajectory_correction_m, smoothing_result.correction_m
        )
        stop_distance = smoothing_result.stop_distance_m
        self._apply_velocity_profile(output.points, stop_distance)

        if self.maximum_lateral_acceleration_mps2 > 0.0:
            limit_trajectory_speed_for_curvature(
                output.points,
                self.maximum_lateral_acceleration_mps2,
                self.curvature_speed_preview_m,
                self.comfortable_deceleration_mps2,
            )

        self._recalculate_acceleration(output)
        return output

    def _apply_velocity_profile(self, points, stop_distance_m):
        distances = trajectory_arc_lengths(points)
        for point, path_distance in zip(points, distances):
            distance_to_stop = max(0.0, stop_distance_m - path_distance)
            speed_cap = math.sqrt(
                2.0 * self.comfortable_deceleration_mps2 * distance_to_stop
            )
            point.longitudinal_velocity_mps = float(
                min(
                    max(0.0, point.longitudinal_velocity_mps),
                    self.maximum_speed_mps,
                    speed_cap,
                )
            )
            if path_distance >= stop_distance_m - 1.0e-3:
                point.longitudinal_velocity_mps = 0.0
            point.lateral_velocity_mps = 0.0

    def _apply_geometry_smoothing(
        self,
        points,
        stop_distance_m,
        lateral_offset_min_m,
        lateral_offset_max_m,
    ):
        if self.trajectory_geometry_smoothing_strength == 0.0:
            return GeometrySmoothingResult(0.0, stop_distance_m)
        try:
            correction = smooth_trajectory_geometry(
                points,
                self.route,
                self.progress_m,
                self.trajectory_geometry_smoothing_strength,
                self.trajectory_geometry_smoothing_interval_m,
                self.trajectory_geometry_smoothing_max_deviation_m,
                self.route_corridor_half_width_m,
                mode=self.route_corridor_mode,
                lateral_offset_min_m=lateral_offset_min_m,
                lateral_offset_max_m=lateral_offset_max_m,
                stop_distance_m=stop_distance_m,
                entry_distance_m=self.route_corridor_entry_distance_m,
            )
        except ValueError as error:
            warning = str(error)
            if getattr(self, "last_geometry_smoothing_warning", None) != warning:
                self.get_logger().warning(
                    "Geometry smoothing rejected; keeping conditioned VAD "
                    f"geometry: {warning}"
                )
            self.last_geometry_smoothing_warning = warning
            return GeometrySmoothingResult(0.0, stop_distance_m)
        self.last_geometry_smoothing_warning = None
        return correction

    def _validate_output_trajectory(self, trajectory):
        if len(trajectory.points) < 2:
            raise ValueError("output trajectory has fewer than two points")
        previous_position = None
        previous_time = None
        for index, point in enumerate(trajectory.points):
            position = point.pose.position
            values = (
                position.x,
                position.y,
                position.z,
                point.longitudinal_velocity_mps,
                point.lateral_velocity_mps,
                point.acceleration_mps2,
            )
            if not all(math.isfinite(value) for value in values):
                raise ValueError(f"output point {index} contains a non-finite value")
            time_sec = (
                point.time_from_start.sec
                + point.time_from_start.nanosec * 1.0e-9
            )
            if previous_time is not None and time_sec <= previous_time:
                raise ValueError(f"output point {index} time is not increasing")
            if previous_position is not None:
                segment = math.hypot(
                    position.x - previous_position.x,
                    position.y - previous_position.y,
                )
                if segment <= 1.0e-4:
                    raise ValueError(f"output point {index} overlaps its predecessor")
                if segment > self.max_trajectory_segment_m:
                    raise ValueError(f"output point {index} segment is {segment:.2f} m")
            previous_position = position
            previous_time = time_sec

    def _recalculate_acceleration(self, trajectory):
        for index, point in enumerate(trajectory.points):
            if index + 1 >= len(trajectory.points):
                point.acceleration_mps2 = 0.0
                continue
            following = trajectory.points[index + 1]
            delta_distance = math.hypot(
                following.pose.position.x - point.pose.position.x,
                following.pose.position.y - point.pose.position.y,
            )
            if delta_distance <= 1.0e-3:
                acceleration = 0.0
            else:
                acceleration = (
                    following.longitudinal_velocity_mps**2
                    - point.longitudinal_velocity_mps**2
                ) / (2.0 * delta_distance)
            point.acceleration_mps2 = float(
                max(
                    -self.comfortable_deceleration_mps2,
                    min(self.comfortable_deceleration_mps2, acceleration),
                )
            )

    def _stopped_trajectory(self, candidate):
        output = self._emergency_stop_trajectory(candidate)
        output.header.stamp = self.get_clock().now().to_msg()
        for point in output.points:
            point.longitudinal_velocity_mps = 0.0
            point.lateral_velocity_mps = 0.0
            point.acceleration_mps2 = 0.0
        return output

    def _emergency_stop_trajectory(self, candidate):
        if self.odom is None:
            raise ValueError("cannot construct an emergency stop trajectory")
        output = Trajectory()
        if candidate is not None and len(candidate.points) >= 3:
            output.header = copy.deepcopy(candidate.header)
            output.points = copy.deepcopy(candidate.points[:3])
        else:
            output.header.frame_id = "map"
            output.points = [TrajectoryPoint(), TrajectoryPoint(), TrajectoryPoint()]
        output.points[0].pose = copy.deepcopy(self.odom.pose.pose)
        first_position = output.points[0].pose.position
        orientation = output.points[0].pose.orientation
        ego_yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        stop_path_spacing_m = 0.5
        stop_path_curvature = 1.0e-3
        for index, point in enumerate(output.points):
            arc_length = stop_path_spacing_m * index
            heading_delta = stop_path_curvature * arc_length
            local_x = math.sin(heading_delta) / stop_path_curvature
            local_y = (1.0 - math.cos(heading_delta)) / stop_path_curvature
            point.pose.position.x = (
                first_position.x
                + local_x * math.cos(ego_yaw)
                - local_y * math.sin(ego_yaw)
            )
            point.pose.position.y = (
                first_position.y
                + local_x * math.sin(ego_yaw)
                + local_y * math.cos(ego_yaw)
            )
            point.pose.position.z = first_position.z
            point_yaw = ego_yaw + heading_delta
            point.pose.orientation.x = 0.0
            point.pose.orientation.y = 0.0
            point.pose.orientation.z = math.sin(point_yaw * 0.5)
            point.pose.orientation.w = math.cos(point_yaw * 0.5)
            point.time_from_start.sec = 0
            point.time_from_start.nanosec = index * 100_000_000
        return output

    def _update_goal_state(self):
        if (
            self.goal_reached
            or self.fault
            or self.odom is None
            or self._route_alignment_blocked()
        ):
            return
        position = self.odom.pose.pose.position
        direct_distance = math.hypot(
            position.x - self.route.goal.x, position.y - self.route.goal.y
        )
        speed = abs(self.odom.twist.twist.linear.x)
        inside_goal = (
            self.remaining_m <= self.goal_tolerance_m
            and direct_distance <= self.goal_tolerance_m * 1.5
        )
        now_ns = self.get_clock().now().nanoseconds
        if inside_goal and speed <= self.goal_stop_speed_mps:
            if self.goal_stop_started_ns is None:
                self.goal_stop_started_ns = now_ns
            elif (now_ns - self.goal_stop_started_ns) * 1.0e-9 >= self.goal_hold_sec:
                self.goal_reached = True
                self.status = "goal_reached"
                self.get_logger().info(
                    f"Goal reached: remaining={self.remaining_m:.2f} m, "
                    f"distance={direct_distance:.2f} m, speed={speed:.2f} m/s"
                )
        else:
            self.goal_stop_started_ns = None

    def _update_status(self, trajectory):
        if self.fault:
            self.status = f"fault:{self.fault}"
        elif self.goal_reached:
            self.status = "goal_reached"
        elif self._route_alignment_blocked():
            self.status = (
                "waiting_for_standard_route"
                if self.standard_route_aligned is None
                else "standard_route_mismatch"
            )
        elif trajectory.points and trajectory.points[-1].longitudinal_velocity_mps <= 0.01:
            self.status = "stopping"
        else:
            self.status = "ready"

    def _set_fault(self, reason):
        if self.fault is None:
            self.fault = reason
            self.status = f"fault:{reason}"
            self.get_logger().error(f"VAD route safety fault: {reason}")

    def _route_alignment_blocked(self):
        heartbeat_age_sec = None
        if self.standard_route_alignment_received_ns is not None:
            heartbeat_age_sec = (
                self.get_clock().now().nanoseconds
                - self.standard_route_alignment_received_ns
            ) * 1.0e-9
        return route_alignment_blocked(
            self.require_standard_route_alignment,
            self.standard_route_aligned,
            heartbeat_age_sec,
            self.standard_route_alignment_timeout_sec,
        )

    def _on_timer(self):
        now_ns = self.get_clock().now().nanoseconds
        if (
            self.require_standard_route_alignment
            and self.standard_route_aligned is True
            and self._route_alignment_blocked()
            and not self.standard_route_timeout_reported
        ):
            self.standard_route_timeout_reported = True
            self.get_logger().error(
                "Standard route alignment heartbeat timed out; holding a stop trajectory"
            )
        if (
            self.has_valid_candidate
            and not self.goal_reached
            and not self.fault
            and self.last_candidate_received_ns is not None
            and (now_ns - self.last_candidate_received_ns) * 1.0e-9
            > self.candidate_timeout_sec
        ):
            self._set_fault("candidate_timeout")
        if (
            self.fault or self.goal_reached or self._route_alignment_blocked()
        ) and self.odom is not None:
            try:
                output = self._stopped_trajectory(self.last_selected_candidate)
                self._validate_output_trajectory(output)
                self.trajectory_pub.publish(output)
            except ValueError as error:
                self.get_logger().error(f"Unable to refresh safe stop trajectory: {error}")
        elif (
            self.odom is not None
            and not self.has_valid_candidate
            and self.fault is None
            and not self.goal_reached
        ):
            self.status = "waiting_for_candidates"
        if self._route_alignment_blocked() and not self.fault and not self.goal_reached:
            self.status = (
                "waiting_for_standard_route"
                if self.standard_route_aligned is None
                else "standard_route_mismatch"
            )
        self._publish_state()

    def _publish_state(self):
        self.command_pub.publish(Int8(data=self.command))
        self.remaining_pub.publish(Float32(data=float(self.remaining_m)))
        self.cross_track_pub.publish(Float32(data=float(self.cross_track_error_m)))
        self.trajectory_correction_pub.publish(
            Float32(data=float(self.trajectory_correction_m))
        )
        self.goal_reached_pub.publish(Bool(data=self.goal_reached))
        self.status_pub.publish(String(data=self.status))
        self.marker_pub.publish(self._markers())

    def _publish_reference_path(self):
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        for route_point in self.route.points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = route_point.x
            pose.pose.position.y = route_point.y
            pose.pose.position.z = route_point.z
            pose.pose.orientation.z = math.sin(route_point.yaw * 0.5)
            pose.pose.orientation.w = math.cos(route_point.yaw * 0.5)
            path.poses.append(pose)
        self.route_path_pub.publish(path)

    def _markers(self):
        now = self.get_clock().now().to_msg()
        goal = Marker()
        goal.header.frame_id = "map"
        goal.header.stamp = now
        goal.ns = "vad_route_goal"
        goal.id = 0
        goal.type = Marker.SPHERE
        goal.action = Marker.ADD
        goal.pose.position.x = self.route.goal.x
        goal.pose.position.y = self.route.goal.y
        goal.pose.position.z = self.route.goal.z + 0.5
        goal.pose.orientation.w = 1.0
        goal.scale.x = 1.2
        goal.scale.y = 1.2
        goal.scale.z = 1.2
        goal.color.a = 0.95
        goal.color.r = 0.1 if self.goal_reached else 1.0
        goal.color.g = 0.9 if self.goal_reached else 0.35
        goal.color.b = 0.2

        text = Marker()
        text.header.frame_id = "map"
        text.header.stamp = now
        text.ns = "vad_route_status"
        text.id = 0
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        if self.odom is not None:
            position = self.odom.pose.pose.position
            text.pose.position.x = position.x
            text.pose.position.y = position.y
            text.pose.position.z = position.z + 3.0
        else:
            text.pose.position.x = self.route.points[0].x
            text.pose.position.y = self.route.points[0].y
            text.pose.position.z = self.route.points[0].z + 3.0
        text.pose.orientation.w = 1.0
        text.scale.z = 0.7
        text.color.a = 1.0
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.text = (
            f"{COMMAND_NAMES[self.command]}  {self.remaining_m:.1f} m  "
            f"CTE {self.cross_track_error_m:.1f} m  "
            f"CORR {self.trajectory_correction_m:.1f} m  {self.status}"
        )
        return MarkerArray(markers=[goal, text])


def main():
    rclpy.init()
    node = None
    try:
        node = VadRouteManager()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
