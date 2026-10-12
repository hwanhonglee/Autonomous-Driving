#!/usr/bin/env python3

import argparse
import json
import math
import os
import signal
import tempfile
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from autoware_planning_msgs.msg import Trajectory
from autoware_vehicle_msgs.msg import Engage as EngageState
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, Float32, Int8, String
from tier4_external_api_msgs.srv import Engage as EngageService


LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class EvaluationFailure(RuntimeError):
    pass


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def finite(value):
    return value is not None and math.isfinite(value)


def percentile(values, quantile):
    """Return a linearly interpolated percentile for finite numeric samples."""
    if not math.isfinite(quantile) or not 0.0 <= quantile <= 1.0:
        raise ValueError("quantile must be finite and in [0, 1]")
    ordered = sorted(float(value) for value in values if finite(value))
    if not ordered:
        return None
    position = (len(ordered) - 1) * quantile
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def load_goal_ros_pose(route_file):
    route_path = Path(route_file).expanduser().resolve()
    try:
        payload = json.loads(route_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise ValueError(f"failed to read route JSON {route_path}: {error}") from error
    if not isinstance(payload, dict):
        raise ValueError(f"route JSON must contain an object: {route_path}")

    raw_goal = payload.get("goal_ros_pose")
    if not isinstance(raw_goal, dict):
        raise ValueError(f"route goal_ros_pose must be an object: {route_path}")

    goal = {}
    for key in ("x", "y", "z", "yaw"):
        value = raw_goal.get(key)
        if (
            not isinstance(value, (int, float))
            or isinstance(value, bool)
            or not math.isfinite(value)
        ):
            raise ValueError(
                f"route goal_ros_pose.{key} must be a finite number: {route_path}"
            )
        goal[key] = float(value)
    return route_path, goal


def direct_goal_distance(position, goal_ros_pose):
    if position is None or not all(finite(value) for value in position):
        return None
    if not isinstance(goal_ros_pose, dict):
        return None
    goal_x = goal_ros_pose.get("x")
    goal_y = goal_ros_pose.get("y")
    if not finite(goal_x) or not finite(goal_y):
        return None
    return math.hypot(position[0] - goal_x, position[1] - goal_y)


def goal_completion_failures(
    node, goal_ros_pose, max_goal_distance_m, max_stop_speed_mps
):
    failures = []
    if node.goal_reached is not True:
        failures.append(f"goal_reached Bool is {node.goal_reached!r}")
    if node.route_status != "goal_reached":
        failures.append(f"route status is {node.route_status!r}")

    distance = direct_goal_distance(node.position, goal_ros_pose)
    if not finite(distance):
        failures.append("direct goal distance is unavailable")
    elif distance > max_goal_distance_m:
        failures.append(
            f"direct goal distance {distance:.3f} m exceeds "
            f"{max_goal_distance_m:.3f} m"
        )

    speed = node.speed_mps
    if not finite(speed):
        failures.append("final speed is unavailable")
    elif speed > max_stop_speed_mps:
        failures.append(
            f"final speed {speed:.3f} m/s exceeds {max_stop_speed_mps:.3f} m/s"
        )
    return failures


def speed_exposure_failures(
    maximum_speed_mps,
    sustained_speed_duration_sec,
    maximum_lateral_acceleration_mps2,
    minimum_sustained_speed_mps,
    minimum_sustained_speed_sec,
    maximum_observed_speed_mps,
    maximum_allowed_lateral_acceleration_mps2,
):
    """Return fail-closed speed-profile violations for a completed route."""
    failures = []
    if not finite(maximum_speed_mps):
        failures.append("maximum observed speed is unavailable")
    elif (
        maximum_observed_speed_mps > 0.0
        and maximum_speed_mps > maximum_observed_speed_mps
    ):
        failures.append(
            f"maximum speed {maximum_speed_mps:.3f} m/s exceeds "
            f"{maximum_observed_speed_mps:.3f} m/s"
        )
    if minimum_sustained_speed_mps > 0.0:
        if not finite(sustained_speed_duration_sec):
            failures.append("sustained target-speed duration is unavailable")
        elif sustained_speed_duration_sec < minimum_sustained_speed_sec:
            failures.append(
                f"speed >= {minimum_sustained_speed_mps:.3f} m/s was sustained "
                f"for only {sustained_speed_duration_sec:.3f} s; require "
                f"{minimum_sustained_speed_sec:.3f} s"
            )
    if not finite(maximum_lateral_acceleration_mps2):
        failures.append("maximum lateral acceleration is unavailable")
    elif (
        maximum_allowed_lateral_acceleration_mps2 > 0.0
        and maximum_lateral_acceleration_mps2
        > maximum_allowed_lateral_acceleration_mps2
    ):
        failures.append(
            "maximum lateral acceleration "
            f"{maximum_lateral_acceleration_mps2:.3f} m/s^2 exceeds "
            f"{maximum_allowed_lateral_acceleration_mps2:.3f} m/s^2"
        )
    return failures


def advance_sustained_speed_duration(
    current_duration_sec,
    previous_speed_mps,
    current_speed_mps,
    sample_interval_sec,
    minimum_speed_mps,
    maximum_sample_interval_sec,
):
    """Advance a continuous dwell only across two fresh high-speed samples.

    Requiring both endpoints prevents a delayed DDS sample or a single peak
    from claiming the entire elapsed interval as high-speed exposure.
    """
    values = (
        current_duration_sec,
        previous_speed_mps,
        current_speed_mps,
        sample_interval_sec,
        minimum_speed_mps,
        maximum_sample_interval_sec,
    )
    if not all(finite(value) for value in values):
        return 0.0
    if (
        current_duration_sec < 0.0
        or sample_interval_sec < 0.0
        or minimum_speed_mps <= 0.0
        or maximum_sample_interval_sec <= 0.0
        or sample_interval_sec > maximum_sample_interval_sec
        or previous_speed_mps < minimum_speed_mps
        or current_speed_mps < minimum_speed_mps
    ):
        return 0.0
    return current_duration_sec + sample_interval_sec


def update_goal_completion_claim(node, odometry_count_at_claim):
    claimed = node.goal_reached is True and node.route_status == "goal_reached"
    if not claimed:
        return None, False
    current_odometry_count = node.message_counts.get("odometry", 0)
    if odometry_count_at_claim is None:
        return current_odometry_count, False
    return odometry_count_at_claim, current_odometry_count > odometry_count_at_claim


class RouteTestMonitor(Node):
    def __init__(self, path_sample_distance_m, full_stack=False):
        super().__init__("autoware_e2e_route_test")
        self.full_stack = full_stack
        self.position = None
        self.position_z = None
        self.speed_mps = None
        self.yaw_rate_rps = None
        self.lateral_acceleration_mps2 = None
        self.sim_time = None
        self.trajectory_points = None
        self.trajectory_valid = False
        self.route_status = None
        self.remaining_distance = None
        self.cross_track_error = None
        self.trajectory_correction = None
        self.command = None
        self.goal_reached = None
        self.engage_state = None
        self.localization_initialization_state = None
        self.standard_route_aligned = None
        self.aeb_configured = None
        self.operation_mode_state = None
        self.mrm_state = None
        self.stop_signal = None
        self.path_sample_distance_m = path_sample_distance_m
        self.actual_path = []
        self.record_path = False
        self.message_counts = {
            "odometry": 0,
            "trajectory": 0,
            "status": 0,
            "remaining_distance": 0,
            "cross_track_error": 0,
            "trajectory_correction": 0,
            "command": 0,
            "goal_reached": 0,
            "engage": 0,
            "localization_initialization": 0,
            "standard_route_alignment": 0,
            "aeb_configuration": 0,
            "operation_mode": 0,
            "mrm_state": 0,
        }
        self.last_received = {}

        self.create_subscription(Odometry, "/localization/kinematic_state", self._on_odom, 10)
        self.create_subscription(Trajectory, "/planning/trajectory", self._on_trajectory, 10)
        self.create_subscription(String, "/planning/vad_route/status", self._on_status, 10)
        self.create_subscription(
            Float32,
            "/planning/vad_route/remaining_distance",
            self._on_remaining_distance,
            10,
        )
        self.create_subscription(
            Float32,
            "/planning/vad_route/cross_track_error",
            self._on_cross_track_error,
            10,
        )
        self.create_subscription(
            Float32,
            "/planning/vad_route/trajectory_correction",
            self._on_trajectory_correction,
            10,
        )
        self.create_subscription(Int8, "/planning/vad_route/command", self._on_command, 10)
        self.create_subscription(Bool, "/planning/vad_route/goal_reached", self._on_goal, 10)
        self.engage_client = None
        self.operation_mode_service_type = None
        self.change_to_stop_client = None
        self.change_to_autonomous_client = None
        self.enable_autoware_control_client = None
        if full_stack:
            from autoware_adapi_v1_msgs.msg import (
                LocalizationInitializationState,
                MrmState,
                OperationModeState,
            )
            from autoware_adapi_v1_msgs.srv import ChangeOperationMode

            self.localization_initialized_value = (
                LocalizationInitializationState.INITIALIZED
            )
            self.operation_mode_stop_value = OperationModeState.STOP
            self.operation_mode_autonomous_value = OperationModeState.AUTONOMOUS
            self.mrm_normal_value = MrmState.NORMAL
            self.mrm_none_value = MrmState.NONE
            self.operation_mode_service_type = ChangeOperationMode
            self.create_subscription(
                LocalizationInitializationState,
                "/api/localization/initialization_state",
                self._on_localization_initialization,
                LATCHED_QOS,
            )
            self.create_subscription(
                Bool,
                "/planning/vad_route/standard_route_aligned",
                self._on_standard_route_alignment,
                LATCHED_QOS,
            )
            self.create_subscription(
                Bool,
                "/system/vad/aeb_configured",
                self._on_aeb_configuration,
                LATCHED_QOS,
            )
            self.create_subscription(
                OperationModeState,
                "/api/operation_mode/state",
                self._on_operation_mode,
                LATCHED_QOS,
            )
            self.create_subscription(
                MrmState,
                "/system/fail_safe/mrm_state",
                self._on_mrm_state,
                10,
            )
            self.change_to_stop_client = self.create_client(
                ChangeOperationMode, "/api/operation_mode/change_to_stop"
            )
            self.change_to_autonomous_client = self.create_client(
                ChangeOperationMode, "/api/operation_mode/change_to_autonomous"
            )
            self.enable_autoware_control_client = self.create_client(
                ChangeOperationMode,
                "/api/operation_mode/enable_autoware_control",
            )
        else:
            self.create_subscription(
                EngageState, "/api/autoware/get/engage", self._on_engage, 10
            )
            self.engage_client = self.create_client(
                EngageService, "/vehicle_cmd_gate/service/engage"
            )

    def _received(self, name):
        self.message_counts[name] += 1
        self.last_received[name] = time.monotonic()

    def _on_odom(self, msg):
        self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.position_z = msg.pose.pose.position.z
        linear = msg.twist.twist.linear
        self.speed_mps = math.sqrt(
            linear.x * linear.x + linear.y * linear.y + linear.z * linear.z
        )
        self.yaw_rate_rps = float(msg.twist.twist.angular.z)
        self.lateral_acceleration_mps2 = abs(
            self.speed_mps * self.yaw_rate_rps
        )
        self.sim_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.record_path:
            self._record_path_sample()
        self._received("odometry")

    def _record_path_sample(self, force=False):
        if self.position is None or self.sim_time is None:
            return
        if not all(
            finite(value)
            for value in (
                *self.position,
                self.position_z,
                self.speed_mps,
                self.yaw_rate_rps,
                self.lateral_acceleration_mps2,
                self.sim_time,
            )
        ):
            return
        if self.actual_path:
            previous = self.actual_path[-1]
            distance = math.hypot(
                self.position[0] - previous["x"], self.position[1] - previous["y"]
            )
            if not force and distance < self.path_sample_distance_m:
                return
            if (
                force
                and distance < 1.0e-6
                and abs(self.sim_time - previous["sim_time_sec"]) < 1.0e-6
            ):
                return
        self.actual_path.append(
            {
                "x": float(self.position[0]),
                "y": float(self.position[1]),
                "z": float(self.position_z),
                "sim_time_sec": float(self.sim_time),
                "speed_mps": float(self.speed_mps),
                "yaw_rate_rps": float(self.yaw_rate_rps),
                "command": int(self.command) if self.command is not None else None,
                "lateral_acceleration_mps2": float(
                    self.lateral_acceleration_mps2
                ),
            }
        )

    def start_path_recording(self):
        self.actual_path = []
        self.record_path = True
        self._record_path_sample(force=True)

    def stop_path_recording(self):
        if self.record_path:
            self._record_path_sample(force=True)
        self.record_path = False

    def _on_trajectory(self, msg):
        self.trajectory_points = len(msg.points)
        self.trajectory_valid = self.trajectory_points >= 2 and all(
            math.isfinite(point.pose.position.x)
            and math.isfinite(point.pose.position.y)
            and math.isfinite(point.longitudinal_velocity_mps)
            for point in msg.points
        )
        self._received("trajectory")

    def _on_status(self, msg):
        self.route_status = msg.data
        self._received("status")

    def _on_remaining_distance(self, msg):
        self.remaining_distance = float(msg.data)
        self._received("remaining_distance")

    def _on_cross_track_error(self, msg):
        self.cross_track_error = float(msg.data)
        self._received("cross_track_error")

    def _on_trajectory_correction(self, msg):
        self.trajectory_correction = float(msg.data)
        self._received("trajectory_correction")

    def _on_command(self, msg):
        self.command = int(msg.data)
        self._received("command")

    def _on_goal(self, msg):
        self.goal_reached = bool(msg.data)
        self._received("goal_reached")

    def _on_engage(self, msg):
        self.engage_state = bool(msg.engage)
        self._received("engage")

    def _on_localization_initialization(self, msg):
        self.localization_initialization_state = int(msg.state)
        self._received("localization_initialization")

    def _on_standard_route_alignment(self, msg):
        self.standard_route_aligned = bool(msg.data)
        self._received("standard_route_alignment")

    def _on_aeb_configuration(self, msg):
        self.aeb_configured = bool(msg.data)
        self._received("aeb_configuration")

    def _on_operation_mode(self, msg):
        self.operation_mode_state = msg
        self.engage_state = bool(msg.is_autoware_control_enabled)
        self._received("operation_mode")

    def _on_mrm_state(self, msg):
        self.mrm_state = msg
        self._received("mrm_state")

    def operation_mode_matches(self, mode, require_control=None):
        state = self.operation_mode_state
        if state is None or state.mode != mode or state.is_in_transition:
            return False
        if require_control is None:
            return True
        return bool(state.is_autoware_control_enabled) is require_control

    def missing_ready_inputs(self):
        missing = []
        if self.position is None or not all(finite(value) for value in self.position):
            missing.append("odometry")
        if self.sim_time is None or not finite(self.sim_time):
            missing.append("simulation timestamp")
        if not finite(self.speed_mps):
            missing.append("odometry speed")
        if not finite(self.yaw_rate_rps):
            missing.append("odometry yaw rate")
        if not finite(self.lateral_acceleration_mps2):
            missing.append("odometry lateral acceleration")
        if self.trajectory_points is None or not self.trajectory_valid:
            missing.append("valid trajectory")
        if self.route_status not in ("ready", "stopping"):
            missing.append(f"ready route status (current={self.route_status!r})")
        if not finite(self.remaining_distance):
            missing.append("remaining distance")
        if not finite(self.cross_track_error):
            missing.append("cross-track error")
        if not finite(self.trajectory_correction):
            missing.append("trajectory correction")
        if self.command is None:
            missing.append("route command")
        if self.goal_reached is None:
            missing.append("goal state")
        if self.full_stack:
            if (
                self.localization_initialization_state
                != self.localization_initialized_value
            ):
                missing.append(
                    "initialized localization "
                    f"(current={self.localization_initialization_state!r})"
                )
            if self.standard_route_aligned is not True:
                missing.append(
                    "standard route alignment "
                    f"(current={self.standard_route_aligned!r})"
                )
            if self.aeb_configured is not True:
                missing.append(
                    f"AEB VAD-object configuration (current={self.aeb_configured!r})"
                )
            if self.operation_mode_state is None:
                missing.append("operation mode state")
            else:
                if self.operation_mode_state.is_in_transition:
                    missing.append("stable operation mode")
                if not self.operation_mode_state.is_autonomous_mode_available:
                    missing.append("autonomous operation mode availability")
            if self.mrm_state is None:
                missing.append("MRM state")
            elif (
                self.mrm_state.state != self.mrm_normal_value
                or self.mrm_state.behavior != self.mrm_none_value
            ):
                missing.append(
                    "normal MRM state "
                    f"(state={self.mrm_state.state}, behavior={self.mrm_state.behavior})"
                )
            service_clients = (
                ("change-to-stop service", self.change_to_stop_client),
                (
                    "change-to-autonomous service",
                    self.change_to_autonomous_client,
                ),
                (
                    "enable-autoware-control service",
                    self.enable_autoware_control_client,
                ),
            )
            for label, client in service_clients:
                if client is None or not client.service_is_ready():
                    missing.append(label)
        else:
            if self.engage_state is None:
                missing.append("engage state")
            if not self.engage_client.service_is_ready():
                missing.append("engage service")
        return missing

    def validate_route_values(self):
        if self.route_status is not None and self.route_status.startswith("fault:"):
            raise EvaluationFailure(f"VAD route manager reported {self.route_status}")
        if self.full_stack:
            if (
                self.localization_initialization_state
                != self.localization_initialized_value
            ):
                raise EvaluationFailure(
                    "localization is no longer initialized: "
                    f"{self.localization_initialization_state!r}"
                )
            if self.standard_route_aligned is not True:
                raise EvaluationFailure("standard Autoware route is no longer aligned")
            if self.aeb_configured is not True:
                raise EvaluationFailure("standard AEB is no longer configured for VAD objects")
            if self.mrm_state is None:
                raise EvaluationFailure("MRM state is unavailable")
            if (
                self.mrm_state.state != self.mrm_normal_value
                or self.mrm_state.behavior != self.mrm_none_value
            ):
                raise EvaluationFailure(
                    "MRM left NORMAL/NONE: "
                    f"state={self.mrm_state.state}, behavior={self.mrm_state.behavior}"
                )
        if self.command is None or not 0 <= self.command <= 5:
            raise EvaluationFailure(f"invalid VAD command: {self.command}")
        if not finite(self.remaining_distance) or self.remaining_distance < -0.5:
            raise EvaluationFailure(
                f"invalid remaining distance: {self.remaining_distance}"
            )
        if not finite(self.cross_track_error):
            raise EvaluationFailure(f"invalid cross-track error: {self.cross_track_error}")
        if not finite(self.speed_mps) or self.speed_mps < 0.0:
            raise EvaluationFailure(f"invalid odometry speed: {self.speed_mps}")
        if not finite(self.yaw_rate_rps):
            raise EvaluationFailure(f"invalid odometry yaw rate: {self.yaw_rate_rps}")
        if (
            not finite(self.lateral_acceleration_mps2)
            or self.lateral_acceleration_mps2 < 0.0
        ):
            raise EvaluationFailure(
                "invalid odometry lateral acceleration: "
                f"{self.lateral_acceleration_mps2}"
            )
        if not finite(self.trajectory_correction) or self.trajectory_correction < 0.0:
            raise EvaluationFailure(
                f"invalid trajectory correction: {self.trajectory_correction}"
            )
        if not self.trajectory_valid:
            raise EvaluationFailure(
                f"invalid trajectory: points={self.trajectory_points}"
            )

    def wait_until_ready(self, timeout, stable_duration=0.0):
        deadline = time.monotonic() + timeout
        last_report = 0.0
        stable_since = None
        missing = self.missing_ready_inputs()
        while rclpy.ok() and time.monotonic() < deadline:
            if self.stop_signal is not None:
                raise EvaluationFailure(
                    f"received {signal.Signals(self.stop_signal).name} while waiting for inputs"
                )
            if self.route_status is not None and self.route_status.startswith("fault:"):
                raise EvaluationFailure(
                    f"VAD route manager reported {self.route_status}"
                )
            if not missing:
                self.validate_route_values()
                if self.goal_reached:
                    raise EvaluationFailure("route reports goal reached before test start")
                if stable_since is None:
                    stable_since = time.monotonic()
                if time.monotonic() - stable_since >= stable_duration:
                    return
            else:
                stable_since = None
            now = time.monotonic()
            if now - last_report >= 5.0:
                if missing:
                    print(f"Waiting for: {', '.join(missing)}", flush=True)
                else:
                    stable_elapsed = now - stable_since
                    print(
                        "Waiting for full-stack health stability: "
                        f"{stable_elapsed:.1f}/{stable_duration:.1f} wall seconds",
                        flush=True,
                    )
                last_report = now
            rclpy.spin_once(self, timeout_sec=0.2)
            missing = self.missing_ready_inputs()
        if missing:
            raise EvaluationFailure(
                f"readiness timeout; missing: {', '.join(missing)}"
            )
        raise EvaluationFailure(
            "readiness timeout before continuous full-stack health stability "
            f"reached {stable_duration:.1f} wall seconds"
        )

    def wait_for_engage_state(self, expected, timeout, honor_signal=True):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if honor_signal and self.stop_signal is not None:
                raise EvaluationFailure(
                    f"received {signal.Signals(self.stop_signal).name} "
                    f"while waiting for engage={expected}"
                )
            if self.engage_state is expected:
                return True
            rclpy.spin_once(self, timeout_sec=0.2)
        return self.engage_state is expected

    def set_engage(self, engage, timeout, honor_signal=True):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if self.engage_client.service_is_ready():
                break
            if honor_signal and self.stop_signal is not None:
                raise EvaluationFailure(
                    f"received {signal.Signals(self.stop_signal).name} "
                    "while waiting for engage service"
                )
            self.engage_client.wait_for_service(timeout_sec=0.2)
        else:
            raise EvaluationFailure("engage service is not available")

        request = EngageService.Request()
        request.engage = engage
        future = self.engage_client.call_async(request)
        while rclpy.ok() and time.monotonic() < deadline and not future.done():
            if honor_signal and self.stop_signal is not None:
                raise EvaluationFailure(
                    f"received {signal.Signals(self.stop_signal).name} "
                    "during engage service call"
                )
            rclpy.spin_once(self, timeout_sec=0.2)
        if not future.done() or future.result() is None:
            raise EvaluationFailure("engage service timed out")

        status = future.result().status
        accepted_codes = (status.SUCCESS, status.IGNORED, status.WARN)
        if status.code not in accepted_codes:
            raise EvaluationFailure(
                f"engage service failed: {status.code} {status.message}"
            )
        return {"code": int(status.code), "message": status.message}

    def transition_operation_mode(
        self,
        client,
        label,
        target_mode,
        target_name,
        overall_timeout,
        service_timeout,
        require_control=None,
        honor_signal=True,
        report=None,
        retry_interval=0.5,
    ):
        if not self.full_stack or self.operation_mode_service_type is None:
            raise EvaluationFailure(
                f"{label} is only available with --full-stack"
            )
        if not math.isfinite(overall_timeout) or overall_timeout <= 0.0:
            raise ValueError("overall_timeout must be positive and finite")
        if not math.isfinite(service_timeout) or service_timeout <= 0.0:
            raise ValueError("service_timeout must be positive and finite")
        if not math.isfinite(retry_interval) or retry_interval < 0.0:
            raise ValueError("retry_interval must be non-negative and finite")

        transition = report if report is not None else {}
        transition.clear()
        transition.update(
            {
                "label": label,
                "target_mode": target_name,
                "target_mode_value": int(target_mode),
                "require_control": require_control,
                "overall_timeout_wall_sec": float(overall_timeout),
                "service_timeout_wall_sec": float(service_timeout),
                "attempt_count": 0,
                "attempts": [],
                "already_at_target": False,
                "target_reached": False,
                "elapsed_wall_sec": 0.0,
                "last_error": None,
                "final_state": operation_mode_snapshot(self.operation_mode_state),
            }
        )
        started = time.monotonic()
        deadline = started + overall_timeout
        last_error = None
        unavailable_checks = 0
        wait_for_state_only = False
        transient_codes = {1, 2, 50001, 50002}
        response_type = getattr(self.operation_mode_service_type, "Response", None)
        for name in (
            "ERROR_NOT_AVAILABLE",
            "ERROR_IN_TRANSITION",
            "SERVICE_UNREADY",
            "SERVICE_TIMEOUT",
        ):
            value = getattr(response_type, name, None)
            if isinstance(value, int):
                transient_codes.add(value)

        def finish(target_reached):
            transition["attempt_count"] = len(transition["attempts"])
            transition["target_reached"] = bool(target_reached)
            transition["elapsed_wall_sec"] = max(0.0, time.monotonic() - started)
            transition["last_error"] = None if target_reached else last_error
            transition["service_unavailable_checks"] = unavailable_checks
            transition["final_state"] = operation_mode_snapshot(
                self.operation_mode_state
            )

        def cancel_future(future):
            cancel = getattr(future, "cancel", None)
            if callable(cancel) and not future.done():
                cancel()

        def check_signal(future=None):
            if honor_signal and self.stop_signal is not None:
                if future is not None:
                    cancel_future(future)
                finish(False)
                raise EvaluationFailure(
                    f"received {signal.Signals(self.stop_signal).name} during {label}"
                )

        def target_reached():
            return self.operation_mode_matches(target_mode, require_control)

        if target_reached():
            transition["already_at_target"] = True
            finish(True)
            return transition

        while rclpy.ok() and time.monotonic() < deadline:
            check_signal()
            if target_reached():
                finish(True)
                return transition

            now = time.monotonic()
            if wait_for_state_only:
                rclpy.spin_once(
                    self, timeout_sec=max(0.0, min(0.2, deadline - now))
                )
                continue
            if not client.service_is_ready():
                unavailable_checks += 1
                last_error = f"{label} service is not available"
                rclpy.spin_once(
                    self, timeout_sec=max(0.0, min(0.2, deadline - now))
                )
                continue

            attempt = {
                "attempt": len(transition["attempts"]) + 1,
                "started_after_wall_sec": max(0.0, now - started),
                "response": None,
                "outcome": None,
                "error": None,
                "finished_after_wall_sec": None,
            }
            transition["attempts"].append(attempt)
            try:
                future = client.call_async(self.operation_mode_service_type.Request())
            except Exception as error:
                last_error = f"{label} call_async raised: {error}"
                attempt["outcome"] = "call_async_error"
                attempt["error"] = last_error
                attempt["finished_after_wall_sec"] = max(
                    0.0, time.monotonic() - started
                )
                finish(False)
                raise EvaluationFailure(last_error) from error
            attempt_deadline = min(deadline, now + service_timeout)
            while (
                rclpy.ok()
                and not future.done()
                and time.monotonic() < attempt_deadline
            ):
                check_signal(future)
                if target_reached():
                    cancel_future(future)
                    attempt["outcome"] = "target_reached"
                    attempt["finished_after_wall_sec"] = max(
                        0.0, time.monotonic() - started
                    )
                    finish(True)
                    return transition
                remaining = attempt_deadline - time.monotonic()
                rclpy.spin_once(self, timeout_sec=max(0.0, min(0.2, remaining)))

            if target_reached():
                cancel_future(future)
                attempt["outcome"] = "target_reached"
                attempt["finished_after_wall_sec"] = max(
                    0.0, time.monotonic() - started
                )
                finish(True)
                return transition

            if not future.done():
                cancel_future(future)
                last_error = f"{label} service call timed out"
                attempt["outcome"] = "service_timeout"
                attempt["error"] = last_error
                wait_for_state_only = True
            else:
                try:
                    future_error = future.exception()
                    response = None if future_error is not None else future.result()
                except Exception as error:
                    future_error = error
                    response = None
                if future_error is not None:
                    last_error = f"{label} service call raised: {future_error}"
                    attempt["outcome"] = "service_error"
                    attempt["error"] = last_error
                    wait_for_state_only = True
                elif response is None:
                    last_error = f"{label} service returned no response"
                    attempt["outcome"] = "empty_response"
                    attempt["error"] = last_error
                    wait_for_state_only = True
                else:
                    status = response.status
                    attempt["response"] = {
                        "success": bool(status.success),
                        "code": int(status.code),
                        "message": status.message,
                    }
                    if status.success:
                        attempt["outcome"] = "accepted"
                        last_error = (
                            f"{label} service accepted the request, but target "
                            f"{target_name} state is not reached"
                        )
                        wait_for_state_only = True
                    elif int(status.code) in transient_codes:
                        attempt["outcome"] = "transient_rejection"
                        last_error = (
                            f"{label} service rejected the request transiently: "
                            f"code={status.code} message={status.message}"
                        )
                    else:
                        attempt["outcome"] = "terminal_rejection"
                        last_error = (
                            f"{label} service rejected the request: "
                            f"code={status.code} message={status.message}"
                        )
                    if not status.success:
                        attempt["error"] = last_error
            attempt["finished_after_wall_sec"] = max(
                0.0, time.monotonic() - started
            )

            if attempt["outcome"] == "terminal_rejection":
                finish(False)
                raise EvaluationFailure(last_error)
            if wait_for_state_only:
                continue

            retry_deadline = min(deadline, time.monotonic() + retry_interval)
            while rclpy.ok() and time.monotonic() < retry_deadline:
                check_signal()
                if target_reached():
                    finish(True)
                    return transition
                remaining = retry_deadline - time.monotonic()
                rclpy.spin_once(self, timeout_sec=max(0.0, min(0.2, remaining)))

        if target_reached():
            finish(True)
            return transition
        finish(False)
        detail = (
            f"{label} timed out after {overall_timeout:.1f} s before stable "
            f"{target_name} state"
        )
        if last_error:
            detail += f"; last error: {last_error}"
        raise EvaluationFailure(detail)


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Evaluate a goal-directed CARLA VAD route and always request a "
            "non-driving state afterward."
        )
    )
    parser.add_argument(
        "--full-stack",
        action="store_true",
        help=(
            "use the standard Autoware localization, route, and operation-mode "
            "interfaces instead of the legacy engage service"
        ),
    )
    parser.add_argument(
        "--result",
        default="artifacts/vad_route_test_result.json",
        help="atomic JSON result output path",
    )
    parser.add_argument(
        "--route-file",
        type=Path,
        required=True,
        help="prepared route JSON containing the expected goal_ros_pose",
    )
    parser.add_argument(
        "--ready-timeout",
        type=float,
        default=120.0,
        help="wall seconds allowed for required topics and service",
    )
    parser.add_argument(
        "--ready-stability",
        type=float,
        default=4.0,
        help="continuous healthy wall seconds required before a full-stack run",
    )
    parser.add_argument(
        "--service-timeout", type=float, default=20.0, help="wall seconds per engage service call"
    )
    parser.add_argument(
        "--engage-timeout",
        type=float,
        default=15.0,
        help="wall seconds allowed for engage state transitions",
    )
    parser.add_argument(
        "--sim-timeout", type=float, default=180.0, help="maximum simulated route seconds"
    )
    parser.add_argument(
        "--wall-timeout", type=float, default=900.0, help="maximum wall-clock route seconds"
    )
    parser.add_argument(
        "--stall-timeout",
        type=float,
        default=15.0,
        help="simulated seconds without minimum route progress",
    )
    parser.add_argument(
        "--min-progress",
        type=float,
        default=0.5,
        help="remaining-distance decrease that resets the stall timer in meters",
    )
    parser.add_argument(
        "--max-cte", type=float, default=2.0, help="maximum absolute cross-track error in meters"
    )
    parser.add_argument(
        "--max-trajectory-correction",
        type=float,
        default=15.0,
        help="maximum route-corridor correction of a selected VAD point in meters",
    )
    parser.add_argument(
        "--max-goal-distance",
        type=float,
        default=1.5,
        help="maximum final XY distance from route goal_ros_pose in meters",
    )
    parser.add_argument(
        "--max-stop-speed",
        type=float,
        default=0.15,
        help="maximum final odometry speed for verified goal completion in m/s",
    )
    parser.add_argument(
        "--min-sustained-speed",
        type=float,
        default=0.0,
        help="minimum speed exposure threshold in m/s; zero disables the gate",
    )
    parser.add_argument(
        "--min-sustained-speed-sec",
        type=float,
        default=0.0,
        help="continuous simulated seconds required above --min-sustained-speed",
    )
    parser.add_argument(
        "--max-observed-speed",
        type=float,
        default=0.0,
        help="maximum observed odometry speed in m/s; zero disables the gate",
    )
    parser.add_argument(
        "--max-lateral-acceleration",
        type=float,
        default=0.0,
        help="maximum observed |speed * yaw_rate| in m/s^2; zero disables the gate",
    )
    parser.add_argument(
        "--max-speed-sample-gap",
        type=float,
        default=0.25,
        help=(
            "maximum simulated interval counted toward continuous speed "
            "exposure in seconds"
        ),
    )
    parser.add_argument(
        "--longitudinal-speed-source",
        choices=("vad_prediction", "explicit_simulation_nominal"),
        default="vad_prediction",
    )
    parser.add_argument(
        "--vad-velocity-evaluated",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument(
        "--vad-geometry-evaluated",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument(
        "--data-stale-timeout",
        type=float,
        default=30.0,
        help="wall seconds without odometry or route metric updates",
    )
    parser.add_argument(
        "--report-interval", type=float, default=5.0, help="wall seconds between progress reports"
    )
    parser.add_argument(
        "--path-sample-distance",
        type=float,
        default=0.1,
        help="minimum odometry distance between recorded path samples in meters",
    )
    args = parser.parse_args()

    positive = (
        "ready_timeout",
        "ready_stability",
        "service_timeout",
        "engage_timeout",
        "sim_timeout",
        "wall_timeout",
        "stall_timeout",
        "min_progress",
        "max_cte",
        "max_trajectory_correction",
        "max_goal_distance",
        "max_stop_speed",
        "data_stale_timeout",
        "report_interval",
        "path_sample_distance",
        "max_speed_sample_gap",
    )
    for name in positive:
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            parser.error(f"--{name.replace('_', '-')} must be positive")
    nonnegative = (
        "min_sustained_speed",
        "min_sustained_speed_sec",
        "max_observed_speed",
        "max_lateral_acceleration",
    )
    for name in nonnegative:
        value = getattr(args, name)
        if not math.isfinite(value) or value < 0.0:
            parser.error(f"--{name.replace('_', '-')} must be finite and non-negative")
    if (args.min_sustained_speed > 0.0) != (
        args.min_sustained_speed_sec > 0.0
    ):
        parser.error(
            "--min-sustained-speed and --min-sustained-speed-sec must be "
            "enabled together"
        )
    try:
        args.route_file, args.goal_ros_pose = load_goal_ros_pose(args.route_file)
    except ValueError as error:
        parser.error(str(error))
    return args


def atomic_write_json(path, payload):
    output = Path(path).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{output.name}.", suffix=".tmp", dir=str(output.parent)
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, output)
    except Exception:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise
    return output


def json_number(value):
    return float(value) if finite(value) else None


def make_result(args):
    return {
        "schema_version": 1,
        "execution_mode": "full_stack" if args.full_stack else "minimal",
        "route_file": str(args.route_file),
        "goal_ros_pose": dict(args.goal_ros_pose),
        "profile_context": {
            "longitudinal_velocity_source": args.longitudinal_speed_source,
            "vad_velocity_evaluated": args.vad_velocity_evaluated,
            "vad_geometry_evaluated": args.vad_geometry_evaluated,
        },
        "success": False,
        "reason": "test did not start",
        "started_at": utc_now(),
        "finished_at": None,
        "limits": {
            "ready_timeout_wall_sec": args.ready_timeout,
            "ready_stability_wall_sec": args.ready_stability,
            "service_timeout_wall_sec": args.service_timeout,
            "engage_timeout_wall_sec": args.engage_timeout,
            "route_timeout_sim_sec": args.sim_timeout,
            "route_timeout_wall_sec": args.wall_timeout,
            "stall_timeout_sim_sec": args.stall_timeout,
            "minimum_progress_m": args.min_progress,
            "maximum_absolute_cte_m": args.max_cte,
            "maximum_trajectory_correction_m": args.max_trajectory_correction,
            "maximum_direct_goal_distance_m": args.max_goal_distance,
            "maximum_stop_speed_mps": args.max_stop_speed,
            "minimum_sustained_speed_mps": args.min_sustained_speed,
            "minimum_sustained_speed_sec": args.min_sustained_speed_sec,
            "maximum_observed_speed_mps": args.max_observed_speed,
            "maximum_lateral_acceleration_mps2": args.max_lateral_acceleration,
            "maximum_speed_sample_gap_sec": args.max_speed_sample_gap,
            "data_stale_timeout_wall_sec": args.data_stale_timeout,
            "path_sample_distance_m": args.path_sample_distance,
        },
        "initial": {},
        "final": {},
        "metrics": {
            "sim_elapsed_sec": 0.0,
            "wall_elapsed_sec": 0.0,
            "traveled_distance_m": 0.0,
            "minimum_remaining_distance_m": None,
            "maximum_absolute_cte_m": 0.0,
            "maximum_trajectory_correction_m": 0.0,
            "maximum_observed_speed_mps": 0.0,
            "maximum_lateral_acceleration_mps2": 0.0,
            "maximum_sustained_speed_duration_sec": 0.0,
            "maximum_speed_sample_gap_sec": 0.0,
            "speed_by_command": {},
            "commands_seen": [],
        },
        "assessment": {
            "planning_architecture": "vad_route_manager_hybrid",
            "route_completion": "FAIL",
            "trajectory_geometry": "not_assessed",
            "xy_corridor_correction_applied": None,
            "e2e_geometry_unassisted": False,
        },
        "engage": {
            "interface": (
                "operation_mode_api" if args.full_stack else "legacy_engage"
            ),
            "prepare_response": None,
            "engage_response": None,
            "enable_control_response": None,
            "disengage_response": None,
            "disengage_confirmed": False,
            "shutdown_operation_mode": None,
        },
        "signal": None,
        "message_counts": {},
        "actual_path": [],
    }


def route_snapshot(node, goal_ros_pose=None):
    position = None
    if node.position is not None and all(finite(value) for value in node.position):
        position = {"x": float(node.position[0]), "y": float(node.position[1])}
    snapshot = {
        "position": position,
        "sim_time_sec": json_number(node.sim_time),
        "speed_mps": json_number(node.speed_mps),
        "yaw_rate_rps": json_number(node.yaw_rate_rps),
        "lateral_acceleration_mps2": json_number(
            node.lateral_acceleration_mps2
        ),
        "direct_goal_distance_m": json_number(
            direct_goal_distance(node.position, goal_ros_pose)
        ),
        "remaining_distance_m": json_number(node.remaining_distance),
        "cross_track_error_m": json_number(node.cross_track_error),
        "trajectory_correction_m": json_number(node.trajectory_correction),
        "command": node.command,
        "route_status": node.route_status,
        "trajectory_points": node.trajectory_points,
        "goal_reached": node.goal_reached,
        "engaged": node.engage_state,
    }
    if node.full_stack:
        state = node.operation_mode_state
        snapshot.update(
            {
                "localization_initialization_state": (
                    node.localization_initialization_state
                ),
                "standard_route_aligned": node.standard_route_aligned,
                "aeb_configured_for_vad_objects": node.aeb_configured,
                "operation_mode": operation_mode_snapshot(state),
                "mrm_state": mrm_state_snapshot(node.mrm_state),
            }
        )
    return snapshot


def operation_mode_snapshot(state):
    if state is None:
        return None
    return {
        "mode": int(state.mode),
        "is_autoware_control_enabled": bool(
            state.is_autoware_control_enabled
        ),
        "is_in_transition": bool(state.is_in_transition),
        "is_autonomous_mode_available": bool(
            state.is_autonomous_mode_available
        ),
        "is_stop_mode_available": bool(state.is_stop_mode_available),
    }


def mrm_state_snapshot(state):
    if state is None:
        return None
    return {"state": int(state.state), "behavior": int(state.behavior)}


def run_evaluation(node, args, result):
    node.wait_until_ready(
        args.ready_timeout,
        args.ready_stability if node.full_stack else 0.0,
    )
    result["initial"] = {
        "position": {"x": node.position[0], "y": node.position[1]},
        "sim_time_sec": node.sim_time,
        "speed_mps": node.speed_mps,
        "direct_goal_distance_m": direct_goal_distance(
            node.position, args.goal_ros_pose
        ),
        "remaining_distance_m": node.remaining_distance,
        "cross_track_error_m": node.cross_track_error,
        "trajectory_correction_m": node.trajectory_correction,
        "command": node.command,
        "route_status": node.route_status,
        "trajectory_points": node.trajectory_points,
        "engaged": node.engage_state,
    }
    if node.full_stack:
        full_snapshot = route_snapshot(node, args.goal_ros_pose)
        result["initial"].update(
            {
                "localization_initialization_state": full_snapshot[
                    "localization_initialization_state"
                ],
                "standard_route_aligned": full_snapshot[
                    "standard_route_aligned"
                ],
                "operation_mode": full_snapshot["operation_mode"],
                "mrm_state": full_snapshot["mrm_state"],
            }
        )
    result["metrics"]["minimum_remaining_distance_m"] = node.remaining_distance
    result["metrics"]["maximum_absolute_cte_m"] = abs(node.cross_track_error)
    result["metrics"]["maximum_trajectory_correction_m"] = node.trajectory_correction
    result["metrics"]["commands_seen"] = [node.command]
    result["final"] = route_snapshot(node, args.goal_ros_pose)
    if abs(node.cross_track_error) > args.max_cte:
        raise EvaluationFailure(
            f"initial cross-track error exceeds limit: "
            f"{node.cross_track_error:.3f} m"
        )
    if node.trajectory_correction > args.max_trajectory_correction:
        raise EvaluationFailure(
            "initial trajectory correction exceeds limit: "
            f"{node.trajectory_correction:.3f} m"
        )

    if node.full_stack:
        result["engage"]["prepare_response"] = {}
        node.transition_operation_mode(
            node.change_to_stop_client,
            "change-to-stop service",
            node.operation_mode_stop_value,
            "STOP",
            args.service_timeout + args.engage_timeout,
            args.service_timeout,
            report=result["engage"]["prepare_response"],
        )

        result["engage"]["engage_response"] = {}
        node.transition_operation_mode(
            node.change_to_autonomous_client,
            "change-to-autonomous service",
            node.operation_mode_autonomous_value,
            "AUTONOMOUS",
            args.service_timeout + args.engage_timeout,
            args.service_timeout,
            report=result["engage"]["engage_response"],
        )

        result["engage"]["enable_control_response"] = {}
        node.transition_operation_mode(
            node.enable_autoware_control_client,
            "enable-autoware-control service",
            node.operation_mode_autonomous_value,
            "AUTONOMOUS with Autoware control",
            args.service_timeout + args.engage_timeout,
            args.service_timeout,
            require_control=True,
            report=result["engage"]["enable_control_response"],
        )
    else:
        result["engage"]["prepare_response"] = node.set_engage(
            False, args.service_timeout
        )
        if not node.wait_for_engage_state(False, args.engage_timeout):
            raise EvaluationFailure(
                "engage state did not become false during preparation"
            )

        result["engage"]["engage_response"] = node.set_engage(
            True, args.service_timeout
        )
        if not node.wait_for_engage_state(True, args.engage_timeout):
            raise EvaluationFailure("engage state did not become true")

    node.start_path_recording()
    wall_start = time.monotonic()
    last_sim_time = node.sim_time
    previous_position = node.position
    traveled_distance = 0.0
    sim_elapsed = 0.0
    minimum_remaining = node.remaining_distance
    progress_reference_remaining = node.remaining_distance
    max_abs_cte = abs(node.cross_track_error)
    max_trajectory_correction = node.trajectory_correction
    maximum_observed_speed = node.speed_mps
    maximum_lateral_acceleration = node.lateral_acceleration_mps2
    current_sustained_speed_sec = 0.0
    maximum_sustained_speed_sec = 0.0
    maximum_speed_sample_gap_sec = 0.0
    previous_evaluated_speed = node.speed_mps
    previous_evaluated_command = node.command
    command_maximum_speed = {node.command: node.speed_mps}
    command_current_sustained_speed = {node.command: 0.0}
    command_maximum_sustained_speed = {node.command: 0.0}
    last_progress_sim = 0.0
    last_report = wall_start - args.report_interval
    commands_seen = {node.command}
    unexpected_disengage_wall = None
    completion_claim_odom_count = None

    def metrics_snapshot():
        speed_by_command = {}
        for command in sorted(command_maximum_speed):
            speed_by_command[str(command)] = {
                "maximum_observed_speed_mps": command_maximum_speed[command],
                "maximum_sustained_speed_duration_sec": (
                    command_maximum_sustained_speed.get(command, 0.0)
                ),
            }
        return {
            "sim_elapsed_sec": sim_elapsed,
            "wall_elapsed_sec": time.monotonic() - wall_start,
            "traveled_distance_m": traveled_distance,
            "minimum_remaining_distance_m": minimum_remaining,
            "maximum_absolute_cte_m": max_abs_cte,
            "maximum_trajectory_correction_m": max_trajectory_correction,
            "maximum_observed_speed_mps": maximum_observed_speed,
            "maximum_lateral_acceleration_mps2": maximum_lateral_acceleration,
            "maximum_sustained_speed_duration_sec": maximum_sustained_speed_sec,
            "maximum_speed_sample_gap_sec": maximum_speed_sample_gap_sec,
            "speed_by_command": speed_by_command,
            "commands_seen": sorted(commands_seen),
        }

    print(
        f"Route test engaged: remaining={minimum_remaining:.2f} m, "
        f"command={node.command}, max_cte={args.max_cte:.2f} m",
        flush=True,
    )

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)
        now = time.monotonic()

        if node.stop_signal is not None:
            result["signal"] = signal.Signals(node.stop_signal).name
            raise EvaluationFailure(f"received {result['signal']}")

        node.validate_route_values()
        commands_seen.add(node.command)
        max_abs_cte = max(max_abs_cte, abs(node.cross_track_error))
        max_trajectory_correction = max(
            max_trajectory_correction, node.trajectory_correction
        )

        if node.sim_time < last_sim_time - 1e-3:
            raise EvaluationFailure(
                f"simulation time moved backwards: {last_sim_time:.3f} -> {node.sim_time:.3f}"
            )
        sim_step = max(0.0, node.sim_time - last_sim_time)
        sim_elapsed += sim_step
        last_sim_time = node.sim_time
        maximum_speed_sample_gap_sec = max(maximum_speed_sample_gap_sec, sim_step)

        maximum_observed_speed = max(maximum_observed_speed, node.speed_mps)
        maximum_lateral_acceleration = max(
            maximum_lateral_acceleration,
            node.lateral_acceleration_mps2,
        )
        command_maximum_speed[node.command] = max(
            command_maximum_speed.get(node.command, 0.0), node.speed_mps
        )
        if args.min_sustained_speed > 0.0:
            current_sustained_speed_sec = advance_sustained_speed_duration(
                current_sustained_speed_sec,
                previous_evaluated_speed,
                node.speed_mps,
                sim_step,
                args.min_sustained_speed,
                args.max_speed_sample_gap,
            )
            previous_command_duration = (
                command_current_sustained_speed.get(node.command, 0.0)
                if previous_evaluated_command == node.command
                else 0.0
            )
            command_current_sustained_speed[node.command] = (
                advance_sustained_speed_duration(
                    previous_command_duration,
                    previous_evaluated_speed,
                    node.speed_mps,
                    sim_step,
                    args.min_sustained_speed,
                    args.max_speed_sample_gap,
                )
                if previous_evaluated_command == node.command
                else 0.0
            )
        else:
            current_sustained_speed_sec = 0.0
            command_current_sustained_speed[node.command] = 0.0
        for command in list(command_current_sustained_speed):
            if command != node.command:
                command_current_sustained_speed[command] = 0.0
        maximum_sustained_speed_sec = max(
            maximum_sustained_speed_sec, current_sustained_speed_sec
        )
        command_maximum_sustained_speed[node.command] = max(
            command_maximum_sustained_speed.get(node.command, 0.0),
            command_current_sustained_speed[node.command],
        )
        previous_evaluated_speed = node.speed_mps
        previous_evaluated_command = node.command

        if node.position is not None and previous_position is not None:
            step = math.hypot(
                node.position[0] - previous_position[0],
                node.position[1] - previous_position[1],
            )
            if step < 5.0:
                traveled_distance += step
            previous_position = node.position

        minimum_remaining = min(minimum_remaining, node.remaining_distance)
        if node.remaining_distance <= progress_reference_remaining - args.min_progress:
            progress_reference_remaining = node.remaining_distance
            last_progress_sim = sim_elapsed

        result["metrics"] = metrics_snapshot()
        result["final"] = route_snapshot(node, args.goal_ros_pose)

        if abs(node.cross_track_error) > args.max_cte:
            raise EvaluationFailure(
                f"cross-track error limit exceeded: {node.cross_track_error:.3f} m"
            )
        if node.trajectory_correction > args.max_trajectory_correction:
            raise EvaluationFailure(
                "trajectory correction limit exceeded: "
                f"{node.trajectory_correction:.3f} m"
            )
        instantaneous_speed_failures = speed_exposure_failures(
            maximum_observed_speed,
            maximum_sustained_speed_sec,
            maximum_lateral_acceleration,
            0.0,
            0.0,
            args.max_observed_speed,
            args.max_lateral_acceleration,
        )
        if instantaneous_speed_failures:
            raise EvaluationFailure(instantaneous_speed_failures[0])

        previous_claim_odom_count = completion_claim_odom_count
        completion_claim_odom_count, completion_evidence_ready = (
            update_goal_completion_claim(node, completion_claim_odom_count)
        )
        if completion_evidence_ready:
            completion_failures = goal_completion_failures(
                node,
                args.goal_ros_pose,
                args.max_goal_distance,
                args.max_stop_speed,
            )
            if completion_failures:
                raise EvaluationFailure(
                    "invalid goal completion claim: "
                    + "; ".join(completion_failures)
                )
            exposure_failures = speed_exposure_failures(
                maximum_observed_speed,
                maximum_sustained_speed_sec,
                maximum_lateral_acceleration,
                args.min_sustained_speed,
                args.min_sustained_speed_sec,
                args.max_observed_speed,
                args.max_lateral_acceleration,
            )
            if exposure_failures:
                raise EvaluationFailure(
                    "speed exposure contract failed: "
                    + "; ".join(exposure_failures)
                )
            result["success"] = True
            result["reason"] = "goal reached"
            break
        if (
            completion_claim_odom_count is not None
            and previous_claim_odom_count is None
        ):
            continue
        if sim_elapsed >= args.sim_timeout:
            raise EvaluationFailure("simulation-time timeout")
        if now - wall_start >= args.wall_timeout:
            raise EvaluationFailure("wall-clock timeout")
        if sim_elapsed - last_progress_sim >= args.stall_timeout:
            raise EvaluationFailure(
                f"route progress stalled for {sim_elapsed - last_progress_sim:.1f} simulated seconds"
            )

        dynamic_inputs = (
            "odometry",
            "trajectory",
            "remaining_distance",
            "cross_track_error",
            "trajectory_correction",
        )
        stale = [
            name
            for name in dynamic_inputs
            if now - node.last_received.get(name, 0.0) >= args.data_stale_timeout
        ]
        if stale:
            raise EvaluationFailure(f"stale route data: {', '.join(stale)}")

        if node.full_stack:
            control_active = node.operation_mode_matches(
                node.operation_mode_autonomous_value, require_control=True
            )
        else:
            control_active = node.engage_state is True
        if not control_active:
            if unexpected_disengage_wall is None:
                unexpected_disengage_wall = now
            elif now - unexpected_disengage_wall >= 2.0:
                if node.full_stack:
                    raise EvaluationFailure(
                        "Autoware left active AUTONOMOUS mode before reaching the goal"
                    )
                raise EvaluationFailure(
                    "Autoware disengaged before reaching the goal"
                )
        else:
            unexpected_disengage_wall = None

        if now - last_report >= args.report_interval:
            print(
                f"remaining={node.remaining_distance:.2f} m "
                f"cte={node.cross_track_error:.2f} m command={node.command} "
                f"corr={node.trajectory_correction:.2f} m "
                f"sim={sim_elapsed:.1f} s wall={now - wall_start:.1f} s "
                f"engage={node.engage_state} status={node.route_status}",
                flush=True,
            )
            last_report = now

    result["metrics"] = metrics_snapshot()
    result["final"] = route_snapshot(node, args.goal_ros_pose)


def main():
    args = parse_args()
    result = make_result(args)
    node = None
    exit_code = 1

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = RouteTestMonitor(args.path_sample_distance, args.full_stack)

        def request_stop(signum, _frame):
            node.stop_signal = signum

        signal.signal(signal.SIGINT, request_stop)
        signal.signal(signal.SIGTERM, request_stop)

        try:
            run_evaluation(node, args, result)
            exit_code = 0 if result["success"] else 1
        except (EvaluationFailure, KeyboardInterrupt) as error:
            result["success"] = False
            result["reason"] = str(error) or "interrupted"
            exit_code = 1
            print(f"Route test failed: {result['reason']}", flush=True)
        except Exception as error:
            result["success"] = False
            result["reason"] = f"unexpected error: {error}"
            exit_code = 1
            print(f"Route test failed: {result['reason']}", flush=True)
        finally:
            try:
                node.stop_path_recording()
                result["actual_path"] = list(node.actual_path)
            except Exception as error:
                result["path_recording_error"] = str(error)
            try:
                if node.full_stack:
                    result["engage"]["disengage_response"] = {}
                    node.transition_operation_mode(
                        node.change_to_stop_client,
                        "change-to-stop service",
                        node.operation_mode_stop_value,
                        "STOP",
                        args.service_timeout + args.engage_timeout,
                        args.service_timeout,
                        honor_signal=False,
                        report=result["engage"]["disengage_response"],
                    )
                    result["engage"][
                        "shutdown_operation_mode"
                    ] = operation_mode_snapshot(node.operation_mode_state)
                else:
                    result["engage"]["disengage_response"] = node.set_engage(
                        False, args.service_timeout, honor_signal=False
                    )
                    if not node.wait_for_engage_state(
                        False, args.engage_timeout, honor_signal=False
                    ):
                        raise EvaluationFailure(
                            "disengage state did not become false"
                        )
                result["engage"]["disengage_confirmed"] = True
                if node.full_stack:
                    print("Autoware operation mode changed to STOP.", flush=True)
                else:
                    print("Autoware control disengaged.", flush=True)
            except Exception as error:
                result["engage"]["disengage_error"] = str(error)
                result["success"] = False
                if result["reason"] == "goal reached":
                    result["reason"] = f"goal reached, but disengage failed: {error}"
                elif "disengage failed" not in result["reason"]:
                    result["reason"] = f"{result['reason']}; disengage failed: {error}"
                exit_code = 1
                print(f"WARNING: failed to disengage: {error}", flush=True)

            result["finished_at"] = utc_now()
            result["message_counts"] = dict(node.message_counts)
            if node.stop_signal is not None and result["signal"] is None:
                result["signal"] = signal.Signals(node.stop_signal).name
            if result["final"] == {}:
                result["final"] = route_snapshot(node, args.goal_ros_pose)
    except Exception as error:
        result["success"] = False
        result["reason"] = f"failed to initialize route test: {error}"
        result["finished_at"] = utc_now()
        exit_code = 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    maximum_correction = result.get("metrics", {}).get(
        "maximum_trajectory_correction_m"
    )
    if result.get("initial"):
        assisted = finite(maximum_correction) and maximum_correction > 1.0e-3
        result["assessment"] = {
            "planning_architecture": "vad_route_manager_hybrid",
            "route_completion": "PASS" if result["success"] else "FAIL",
            "trajectory_geometry": "hybrid_route_assisted",
            "xy_corridor_correction_applied": assisted,
            "e2e_geometry_unassisted": False,
        }

    speed_contract_enabled = any(
        value > 0.0
        for value in (
            args.min_sustained_speed,
            args.max_observed_speed,
            args.max_lateral_acceleration,
        )
    )
    metrics = result.get("metrics", {})
    p95_lateral_acceleration = percentile(
        (
            sample.get("lateral_acceleration_mps2")
            for sample in result.get("actual_path", [])
            if isinstance(sample, dict)
        ),
        0.95,
    )
    lateral_acceleration_by_command = {}
    for sample in result.get("actual_path", []):
        if not isinstance(sample, dict):
            continue
        command = sample.get("command")
        value = sample.get("lateral_acceleration_mps2")
        if isinstance(command, int) and finite(value):
            lateral_acceleration_by_command.setdefault(str(command), []).append(value)
    p95_lateral_acceleration_by_command = {
        command: percentile(values, 0.95)
        for command, values in sorted(lateral_acceleration_by_command.items())
    }
    turn_lateral_acceleration = [
        value
        for command in ("0", "1")
        for value in lateral_acceleration_by_command.get(command, [])
    ]
    result["speed_exposure"] = {
        "status": (
            "PASS"
            if speed_contract_enabled and result["success"]
            else "FAIL"
            if speed_contract_enabled
            else "NOT_REQUESTED"
        ),
        "minimum_sustained_speed_mps": args.min_sustained_speed,
        "minimum_sustained_speed_sec": args.min_sustained_speed_sec,
        "maximum_observed_speed_limit_mps": args.max_observed_speed,
        "maximum_lateral_acceleration_limit_mps2": (
            args.max_lateral_acceleration
        ),
        "maximum_observed_speed_mps": metrics.get(
            "maximum_observed_speed_mps"
        ),
        "maximum_sustained_speed_duration_sec": metrics.get(
            "maximum_sustained_speed_duration_sec"
        ),
        "maximum_speed_sample_gap_sec": metrics.get(
            "maximum_speed_sample_gap_sec"
        ),
        "maximum_lateral_acceleration_mps2": metrics.get(
            "maximum_lateral_acceleration_mps2"
        ),
        "p95_lateral_acceleration_mps2": p95_lateral_acceleration,
        "p95_lateral_acceleration_mps2_by_command": (
            p95_lateral_acceleration_by_command
        ),
        "p95_turn_lateral_acceleration_mps2": percentile(
            turn_lateral_acceleration, 0.95
        ),
        "speed_by_command": metrics.get("speed_by_command", {}),
        "longitudinal_velocity_source": args.longitudinal_speed_source,
        "vad_velocity_evaluated": args.vad_velocity_evaluated,
        "vad_geometry_evaluated": args.vad_geometry_evaluated,
    }

    try:
        output = atomic_write_json(args.result, result)
        print(
            f"Route test {'passed' if result['success'] else 'failed'}: "
            f"{result['reason']} (result: {output})",
            flush=True,
        )
    except Exception as error:
        print(f"ERROR: failed to write result JSON: {error}", flush=True)
        exit_code = 1
    return 0 if exit_code == 0 and result["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
