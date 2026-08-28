#!/usr/bin/env python3

import csv
import json
import math
import os
from pathlib import Path
import signal
import threading
import time

from autoware_vehicle_msgs.msg import SteeringReport
from autoware_vehicle_msgs.msg import VelocityReport
from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from tier4_vehicle_msgs.msg import ActuationCommandStamped
from tier4_vehicle_msgs.msg import ActuationStatusStamped

from carla_actuation_sweep_logic import build_sweep_cases
from carla_actuation_sweep_logic import initialpose_input_z
from carla_actuation_sweep_logic import reverse_velocity_is_safe
from carla_actuation_sweep_logic import sample_is_fit_eligible
from carla_actuation_sweep_logic import update_stability_window


CSV_FIELDS = (
    "sim_time_s",
    "wall_time_s",
    "phase",
    "case_id",
    "case_kind",
    "repeat",
    "pedal_level",
    "requested_accel",
    "requested_brake",
    "applied_accel",
    "applied_brake",
    "velocity_mps",
    "steering_rad",
    "imu_accel_x_mps2",
    "pose_x_m",
    "pose_y_m",
    "pose_z_m",
    "pose_qx",
    "pose_qy",
    "pose_qz",
    "pose_qw",
    "status_age_wall_s",
    "watchdog_active",
    "safety_ok",
    "fit_eligible",
)


class CarlaActuationSweep(Node):
    def __init__(self):
        super().__init__("carla_actuation_sweep")
        self._declare_parameters()
        self._load_parameters()

        self._command_lock = threading.RLock()
        self._finish_lock = threading.Lock()
        self._output_lock = threading.Lock()
        self._desired_accel = 0.0
        self._desired_brake = 1.0
        self._emergency_latched = False
        self._finished = False
        self._finish_status = None
        self._finish_reason = None
        self._summary_written = False
        self._stop_watchdog = threading.Event()
        self._start_wall = time.monotonic()
        self._last_status_wall = None
        self._last_velocity = None
        self._last_steering = None
        self._last_applied_accel = None
        self._last_applied_brake = None
        self._last_imu_accel_x = None
        self._last_pose_x = None
        self._last_pose_y = None
        self._last_pose_z = None
        self._last_pose_qx = None
        self._last_pose_qy = None
        self._last_pose_qz = None
        self._last_pose_qw = None
        self._reset_reference_pose = None
        self._reference_pose_z_anchor = None
        self._reference_pose_stable_since_sim = None

        self._phase = "wait_status"
        self._phase_start_sim = 0.0
        self._stationary_since_sim = None
        self._reset_published = False
        self._reset_pose_observed = False
        self._case_origin = None
        self._case_index = -1
        self._active_case = None
        self._case_results = []
        self._active_eligible_samples = 0
        self._rows_written = 0

        self._open_outputs()
        self._command_pub = self.create_publisher(
            ActuationCommandStamped, "/control/command/actuation_cmd", 1
        )
        self._initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 1
        )
        self.create_subscription(
            VelocityReport, "/vehicle/status/velocity_status", self._on_velocity, 10
        )
        self.create_subscription(
            SteeringReport, "/vehicle/status/steering_status", self._on_steering, 10
        )
        self.create_subscription(
            ActuationStatusStamped,
            "/vehicle/status/actuation_status",
            self._on_actuation_status,
            10,
        )
        self.create_subscription(Imu, "/sensing/imu/tamagawa/imu_raw", self._on_imu, 10)
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/sensing/gnss/pose_with_covariance",
            self._on_pose,
            10,
        )
        self.create_subscription(Bool, self._emergency_topic, self._on_emergency, 10)

        self._state_timer = self.create_timer(1.0 / self._state_rate_hz, self._on_state_timer)
        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop, name="actuation-command-watchdog", daemon=True
        )
        self._watchdog_thread.start()
        self.get_logger().info(
            f"Prepared {len(self._cases)} deterministic actuation cases; output={self._output_csv}"
        )

    def _declare_parameters(self):
        parameters = {
            "output_csv": "",
            "summary_json": "",
            "emergency_topic": "/calibration/actuation/emergency_stop",
            "command_rate_hz": 20.0,
            "state_rate_hz": 20.0,
            "status_timeout_wall_s": 0.5,
            "startup_timeout_wall_s": 20.0,
            "shutdown_brake_wall_s": 1.0,
            "repeats": 3,
            "accel_levels": [0.1, 0.2, 0.3, 0.4],
            "brake_levels": [0.1, 0.2, 0.3, 0.4, 0.6, 0.8],
            "prepare_accel": 0.3,
            "prepare_velocity_mps": 2.8,
            "coast_before_brake_s": 0.5,
            "sample_min_velocity_mps": 0.1,
            "sample_max_velocity_mps": 3.0,
            "response_settle_s": 0.35,
            "minimum_sample_duration_s": 0.5,
            "maximum_case_duration_s": 25.0,
            "stop_velocity_mps": 0.03,
            "stop_hold_s": 0.5,
            "reset_settle_s": 0.5,
            "command_tolerance": 0.025,
            "maximum_velocity_mps": 3.2,
            "maximum_reverse_velocity_mps": 0.1,
            "maximum_settling_reverse_velocity_mps": 0.5,
            "maximum_steering_rad": 0.12,
            "maximum_sample_displacement_m": 40.0,
            "maximum_displacement_m": 50.0,
            "expected_spawn_x_m": 268.586578,
            "expected_spawn_y_m": 1.959914,
            "maximum_spawn_xy_error_m": 0.5,
            "maximum_reset_pose_error_m": 0.25,
            "reset_reference_stability_s": 0.75,
            "maximum_reset_reference_z_delta_m": 0.01,
            "initialpose_interface_z_offset_m": 2.0,
            "reset_timeout_s": 5.0,
            "reset_stop_hold_s": 0.5,
        }
        for name, default in parameters.items():
            self.declare_parameter(name, default)

    def _load_parameters(self):
        def value(name):
            return self.get_parameter(name).value

        self._output_csv = Path(str(value("output_csv"))).expanduser()
        self._summary_json = Path(str(value("summary_json"))).expanduser()
        if not str(self._output_csv) or str(self._output_csv) == ".":
            raise ValueError("output_csv must be provided")
        if not str(self._summary_json) or str(self._summary_json) == ".":
            raise ValueError("summary_json must be provided")
        self._emergency_topic = str(value("emergency_topic"))
        self._command_rate_hz = float(value("command_rate_hz"))
        self._state_rate_hz = float(value("state_rate_hz"))
        self._status_timeout_wall_s = float(value("status_timeout_wall_s"))
        self._startup_timeout_wall_s = float(value("startup_timeout_wall_s"))
        self._shutdown_brake_wall_s = float(value("shutdown_brake_wall_s"))
        self._prepare_accel = float(value("prepare_accel"))
        self._prepare_velocity_mps = float(value("prepare_velocity_mps"))
        self._coast_before_brake_s = float(value("coast_before_brake_s"))
        self._sample_min_velocity_mps = float(value("sample_min_velocity_mps"))
        self._sample_max_velocity_mps = float(value("sample_max_velocity_mps"))
        self._response_settle_s = float(value("response_settle_s"))
        self._minimum_sample_duration_s = float(value("minimum_sample_duration_s"))
        self._maximum_case_duration_s = float(value("maximum_case_duration_s"))
        self._stop_velocity_mps = float(value("stop_velocity_mps"))
        self._stop_hold_s = float(value("stop_hold_s"))
        self._reset_settle_s = float(value("reset_settle_s"))
        self._command_tolerance = float(value("command_tolerance"))
        self._maximum_velocity_mps = float(value("maximum_velocity_mps"))
        self._maximum_reverse_velocity_mps = float(
            value("maximum_reverse_velocity_mps")
        )
        self._maximum_settling_reverse_velocity_mps = float(
            value("maximum_settling_reverse_velocity_mps")
        )
        self._maximum_steering_rad = float(value("maximum_steering_rad"))
        self._maximum_sample_displacement_m = float(value("maximum_sample_displacement_m"))
        self._maximum_displacement_m = float(value("maximum_displacement_m"))
        self._expected_spawn_xy = (
            float(value("expected_spawn_x_m")),
            float(value("expected_spawn_y_m")),
        )
        self._maximum_spawn_xy_error_m = float(value("maximum_spawn_xy_error_m"))
        self._maximum_reset_pose_error_m = float(value("maximum_reset_pose_error_m"))
        self._reset_reference_stability_s = float(value("reset_reference_stability_s"))
        self._maximum_reset_reference_z_delta_m = float(
            value("maximum_reset_reference_z_delta_m")
        )
        self._initialpose_interface_z_offset_m = float(
            value("initialpose_interface_z_offset_m")
        )
        self._reset_timeout_s = float(value("reset_timeout_s"))
        self._reset_stop_hold_s = float(value("reset_stop_hold_s"))
        self._cases = build_sweep_cases(
            value("accel_levels"), value("brake_levels"), int(value("repeats"))
        )
        positive_values = (
            self._command_rate_hz,
            self._state_rate_hz,
            self._status_timeout_wall_s,
            self._startup_timeout_wall_s,
            self._shutdown_brake_wall_s,
            self._maximum_case_duration_s,
            self._reset_timeout_s,
            self._reset_stop_hold_s,
            self._reset_reference_stability_s,
            self._maximum_reset_reference_z_delta_m,
        )
        if any(not math.isfinite(number) or number <= 0.0 for number in positive_values):
            raise ValueError("rates and timeouts must be positive and finite")
        if not 0.0 < self._prepare_accel <= 1.0:
            raise ValueError("prepare_accel must be within (0, 1]")
        if not 0.0 <= self._sample_min_velocity_mps < self._sample_max_velocity_mps:
            raise ValueError("sample velocity range is invalid")
        if self._sample_max_velocity_mps >= self._maximum_velocity_mps:
            raise ValueError("maximum_velocity_mps must exceed sample_max_velocity_mps")
        if not 0.0 < self._maximum_sample_displacement_m < self._maximum_displacement_m:
            raise ValueError("sample displacement limit must be below the safety limit")
        if not 0.0 < self._maximum_reset_pose_error_m < self._maximum_displacement_m:
            raise ValueError("reset pose error limit is invalid")
        reverse_limits = (
            self._maximum_reverse_velocity_mps,
            self._maximum_settling_reverse_velocity_mps,
        )
        if any(not math.isfinite(number) or number < 0.0 for number in reverse_limits):
            raise ValueError("reverse velocity limits must be finite and non-negative")
        if self._maximum_settling_reverse_velocity_mps < self._maximum_reverse_velocity_mps:
            raise ValueError("settling reverse limit must not be below the driving limit")
        initialpose_input_z(0.0, self._initialpose_interface_z_offset_m)

    def _open_outputs(self):
        for path in (self._output_csv, self._summary_json):
            path.parent.mkdir(parents=True, exist_ok=True)
            if path.exists() or path.is_symlink():
                raise FileExistsError(f"calibration output already exists: {path}")
        self._csv_handle = self._output_csv.open("x", newline="", encoding="utf-8")
        self._csv_writer = csv.DictWriter(self._csv_handle, fieldnames=CSV_FIELDS)
        self._csv_writer.writeheader()
        self._csv_handle.flush()

    def _on_velocity(self, message):
        self._last_velocity = float(message.longitudinal_velocity)

    def _on_steering(self, message):
        self._last_steering = float(message.steering_tire_angle)

    def _on_actuation_status(self, message):
        self._last_applied_accel = float(message.status.accel_status)
        self._last_applied_brake = float(message.status.brake_status)
        self._last_status_wall = time.monotonic()

    def _on_imu(self, message):
        self._last_imu_accel_x = float(message.linear_acceleration.x)

    def _on_pose(self, message):
        pose = message.pose.pose
        self._last_pose_x = float(pose.position.x)
        self._last_pose_y = float(pose.position.y)
        self._last_pose_z = float(pose.position.z)
        self._last_pose_qx = float(pose.orientation.x)
        self._last_pose_qy = float(pose.orientation.y)
        self._last_pose_qz = float(pose.orientation.z)
        self._last_pose_qw = float(pose.orientation.w)

    def _on_emergency(self, message):
        if message.data:
            self._finish("failed", "external_emergency_stop")

    def _status_ready(self):
        return all(
            value is not None
            for value in (
                self._last_velocity,
                self._last_steering,
                self._last_applied_accel,
                self._last_applied_brake,
                self._last_imu_accel_x,
                self._last_pose_x,
                self._last_pose_y,
                self._last_pose_z,
                self._last_pose_qx,
                self._last_pose_qy,
                self._last_pose_qz,
                self._last_pose_qw,
            )
        )

    def _sim_time(self):
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _set_desired(self, accel, brake):
        with self._command_lock:
            self._desired_accel = float(accel)
            self._desired_brake = float(brake)

    def _watchdog_loop(self):
        period = 1.0 / self._command_rate_hz
        while not self._stop_watchdog.wait(period):
            now_wall = time.monotonic()
            failure_reason = None
            with self._command_lock:
                status_age = (
                    math.inf
                    if self._last_status_wall is None
                    else now_wall - self._last_status_wall
                )
                startup_expired = (
                    self._last_status_wall is None
                    and now_wall - self._start_wall > self._startup_timeout_wall_s
                )
                stale = self._last_status_wall is not None and (
                    status_age > self._status_timeout_wall_s
                )
                if startup_expired:
                    failure_reason = "vehicle_status_startup_timeout"
                elif stale:
                    failure_reason = "vehicle_status_timeout"
                watchdog_active = self._emergency_latched or self._last_status_wall is None or stale
                accel = 0.0 if watchdog_active else self._desired_accel
                brake = 1.0 if watchdog_active else self._desired_brake
            if failure_reason:
                self._finish("failed", failure_reason)
            self._publish_command(accel, brake)

    def _publish_command(self, accel, brake):
        try:
            message = ActuationCommandStamped()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = "base_link"
            message.actuation.accel_cmd = float(accel)
            message.actuation.brake_cmd = float(brake)
            message.actuation.steer_cmd = 0.0
            self._command_pub.publish(message)
        except Exception as error:
            if not self._finished:
                self.get_logger().error(f"Failed to publish actuation command: {error}")

    def _capture_reset_reference(self):
        reference = (
            self._last_pose_x,
            self._last_pose_y,
            self._last_pose_z,
            self._last_pose_qx,
            self._last_pose_qy,
            self._last_pose_qz,
            self._last_pose_qw,
        )
        if not all(math.isfinite(value) for value in reference):
            raise ValueError("reset reference pose is not finite")
        spawn_error = math.hypot(
            reference[0] - self._expected_spawn_xy[0],
            reference[1] - self._expected_spawn_xy[1],
        )
        if spawn_error > self._maximum_spawn_xy_error_m:
            raise ValueError(
                f"stable spawn pose differs from expected position by {spawn_error:.3f} m"
            )
        quaternion_norm = math.sqrt(sum(value * value for value in reference[3:]))
        if quaternion_norm < 1.0e-6:
            raise ValueError("reset reference orientation is invalid")
        self._reset_reference_pose = (
            *reference[:3],
            *(value / quaternion_norm for value in reference[3:]),
        )
        self.get_logger().info(
            "Captured stable CARLA reset pose "
            f"x={reference[0]:.3f}, y={reference[1]:.3f}, z={reference[2]:.3f}"
        )

    def _publish_reset_pose(self):
        if self._reset_reference_pose is None:
            raise RuntimeError("reset reference pose is not available")
        x, y, z, qx, qy, qz, qw = self._reset_reference_pose
        message = PoseWithCovarianceStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        message.pose.pose.position.z = initialpose_input_z(
            z, self._initialpose_interface_z_offset_m
        )
        message.pose.pose.orientation.x = qx
        message.pose.pose.orientation.y = qy
        message.pose.pose.orientation.z = qz
        message.pose.pose.orientation.w = qw
        self._initialpose_pub.publish(message)

    def _reset_pose_error(self):
        if self._reset_reference_pose is None:
            return math.inf
        return math.sqrt(
            (self._last_pose_x - self._reset_reference_pose[0]) ** 2
            + (self._last_pose_y - self._reset_reference_pose[1]) ** 2
            + (self._last_pose_z - self._reset_reference_pose[2]) ** 2
        )

    def _transition(self, phase, sim_time):
        self._phase = phase
        self._phase_start_sim = sim_time

    def _begin_next_case(self, sim_time):
        self._case_index += 1
        if self._case_index >= len(self._cases):
            self._finish("completed", "all_cases_completed")
            return
        self._active_case = self._cases[self._case_index]
        self._active_eligible_samples = 0
        self._stationary_since_sim = None
        self._reset_published = False
        self._reset_pose_observed = False
        self._reference_pose_z_anchor = None
        self._reference_pose_stable_since_sim = None
        self._case_origin = None
        self._set_desired(0.0, 1.0)
        self._transition("stop_before_reset", sim_time)
        self.get_logger().info(
            f"Case {self._case_index + 1}/{len(self._cases)}: {self._active_case.case_id}"
        )

    def _start_active_case(self, sim_time):
        if self._active_case.kind == "accel":
            self._set_desired(self._active_case.level, 0.0)
            self._transition("sample", sim_time)
        else:
            self._set_desired(self._prepare_accel, 0.0)
            self._transition("prepare", sim_time)

    def _complete_active_case(self, sim_time, outcome):
        self._case_results.append(
            {
                "case_id": self._active_case.case_id,
                "kind": self._active_case.kind,
                "level": self._active_case.level,
                "repeat": self._active_case.repeat,
                "outcome": outcome,
                "eligible_samples": self._active_eligible_samples,
            }
        )
        self._begin_next_case(sim_time)

    def _safety_reason(self):
        if self._last_velocity is not None:
            if self._last_velocity > self._maximum_velocity_mps:
                return "maximum_velocity_exceeded"
            if not reverse_velocity_is_safe(
                phase=self._phase,
                velocity_mps=self._last_velocity,
                maximum_reverse_velocity_mps=self._maximum_reverse_velocity_mps,
                maximum_settling_reverse_velocity_mps=(
                    self._maximum_settling_reverse_velocity_mps
                ),
            ):
                return "unexpected_reverse_velocity"
        if self._last_steering is not None and abs(self._last_steering) > self._maximum_steering_rad:
            return "maximum_steering_exceeded"
        if self._case_origin and self._last_pose_x is not None and self._last_pose_y is not None:
            displacement = math.hypot(
                self._last_pose_x - self._case_origin[0],
                self._last_pose_y - self._case_origin[1],
            )
            if displacement > self._maximum_displacement_m:
                return "maximum_displacement_exceeded"
        if (
            self._last_applied_accel is not None
            and self._last_applied_brake is not None
            and self._last_applied_accel > self._command_tolerance
            and self._last_applied_brake > self._command_tolerance
        ):
            return "simultaneous_accel_and_brake"
        return None

    def _case_displacement(self):
        if self._case_origin is None or self._last_pose_x is None or self._last_pose_y is None:
            return 0.0
        return math.hypot(
            self._last_pose_x - self._case_origin[0],
            self._last_pose_y - self._case_origin[1],
        )

    def _on_state_timer(self):
        if self._finished:
            return
        sim_time = self._sim_time()
        if self._phase == "wait_status":
            if self._status_ready():
                self._begin_next_case(sim_time)
            self._write_row(sim_time, safety_ok=True)
            return

        safety_reason = self._safety_reason()
        if safety_reason:
            self._write_row(sim_time, safety_ok=False)
            self._finish("failed", safety_reason)
            return

        elapsed = max(0.0, sim_time - self._phase_start_sim)
        if self._phase == "stop_before_reset":
            reference_pose_stable = True
            if self._reset_reference_pose is None:
                (
                    self._reference_pose_z_anchor,
                    self._reference_pose_stable_since_sim,
                    reference_pose_stable,
                ) = update_stability_window(
                    anchor_value=self._reference_pose_z_anchor,
                    stable_since_s=self._reference_pose_stable_since_sim,
                    current_value=self._last_pose_z,
                    current_time_s=sim_time,
                    maximum_delta=self._maximum_reset_reference_z_delta_m,
                    required_duration_s=self._reset_reference_stability_s,
                )
            if abs(self._last_velocity) <= self._stop_velocity_mps:
                if self._stationary_since_sim is None:
                    self._stationary_since_sim = sim_time
                elif (
                    sim_time - self._stationary_since_sim >= self._stop_hold_s
                    and reference_pose_stable
                ):
                    if self._reset_reference_pose is None:
                        try:
                            self._capture_reset_reference()
                        except ValueError as error:
                            self._finish("failed", f"invalid_reset_reference: {error}")
                            return
                    self._publish_reset_pose()
                    self._reset_published = True
                    self._stationary_since_sim = None
                    self._transition("reset_settle", sim_time)
            else:
                self._stationary_since_sim = None
            if (
                self._phase == "stop_before_reset"
                and elapsed >= self._maximum_case_duration_s
            ):
                self._finish("failed", "stop_before_reset_timeout")
        elif self._phase == "reset_settle":
            pose_error = self._reset_pose_error()
            if not self._reset_pose_observed:
                reset_ack_delay_s = 1.0 / self._state_rate_hz
                if elapsed >= reset_ack_delay_s and pose_error <= self._maximum_reset_pose_error_m:
                    self._reset_pose_observed = True
                    self._stationary_since_sim = None
            elif pose_error > self._maximum_reset_pose_error_m:
                self._finish("failed", "reset_pose_error_exceeded")
                return
            if (
                self._reset_pose_observed
                and
                elapsed >= self._reset_settle_s
                and abs(self._last_velocity) <= self._stop_velocity_mps
            ):
                if self._stationary_since_sim is None:
                    self._stationary_since_sim = sim_time
                elif sim_time - self._stationary_since_sim >= self._reset_stop_hold_s:
                    self._case_origin = (self._last_pose_x, self._last_pose_y)
                    self._start_active_case(sim_time)
            else:
                self._stationary_since_sim = None
            if self._phase == "reset_settle" and elapsed >= self._reset_timeout_s:
                self._finish("failed", "reset_settle_timeout")
        elif self._phase == "prepare":
            if self._last_velocity >= self._prepare_velocity_mps:
                self._set_desired(0.0, 0.0)
                if self._active_case.kind == "coast":
                    self._transition("sample", sim_time)
                else:
                    self._transition("coast_before_brake", sim_time)
            elif elapsed >= self._maximum_case_duration_s:
                self._complete_active_case(sim_time, "prepare_timeout")
        elif self._phase == "coast_before_brake" and elapsed >= self._coast_before_brake_s:
            self._set_desired(0.0, self._active_case.level)
            self._transition("sample", sim_time)
        elif self._phase == "sample":
            complete = False
            outcome = "completed"
            if self._active_case.kind == "accel":
                complete = self._last_velocity >= self._sample_max_velocity_mps
            elif elapsed >= self._minimum_sample_duration_s:
                complete = self._last_velocity <= self._sample_min_velocity_mps
            if self._case_displacement() >= self._maximum_sample_displacement_m:
                complete = True
                outcome = "distance_limit_partial"
            if complete:
                self._complete_active_case(sim_time, outcome)
            elif elapsed >= self._maximum_case_duration_s:
                self._complete_active_case(sim_time, "sample_timeout_partial")
        if not self._finished:
            self._write_row(sim_time, safety_ok=True)

    def _write_row(self, sim_time, safety_ok):
        now_wall = time.monotonic()
        status_age = (
            math.inf if self._last_status_wall is None else now_wall - self._last_status_wall
        )
        with self._command_lock:
            requested_accel = self._desired_accel
            requested_brake = self._desired_brake
            watchdog_active = self._emergency_latched or status_age > self._status_timeout_wall_s
        sample_elapsed = (
            max(0.0, sim_time - self._phase_start_sim) if self._phase == "sample" else 0.0
        )
        eligible = sample_is_fit_eligible(
            phase=self._phase,
            sample_elapsed_s=sample_elapsed,
            response_settle_s=self._response_settle_s,
            requested_accel=requested_accel,
            requested_brake=requested_brake,
            applied_accel=self._last_applied_accel,
            applied_brake=self._last_applied_brake,
            command_tolerance=self._command_tolerance,
            velocity_mps=self._last_velocity,
            minimum_velocity_mps=self._sample_min_velocity_mps,
            maximum_velocity_mps=self._sample_max_velocity_mps,
            steering_rad=self._last_steering,
            maximum_steering_rad=self._maximum_steering_rad,
            status_age_wall_s=status_age,
            status_timeout_wall_s=self._status_timeout_wall_s,
            watchdog_active=watchdog_active,
            safety_ok=safety_ok,
        )
        if eligible:
            self._active_eligible_samples += 1
        case = self._active_case
        row = {
            "sim_time_s": f"{sim_time:.9f}",
            "wall_time_s": f"{now_wall - self._start_wall:.9f}",
            "phase": self._phase,
            "case_id": case.case_id if case else "",
            "case_kind": case.kind if case else "",
            "repeat": case.repeat if case else "",
            "pedal_level": f"{case.level:.6f}" if case else "",
            "requested_accel": f"{requested_accel:.9f}",
            "requested_brake": f"{requested_brake:.9f}",
            "applied_accel": self._format_optional(self._last_applied_accel),
            "applied_brake": self._format_optional(self._last_applied_brake),
            "velocity_mps": self._format_optional(self._last_velocity),
            "steering_rad": self._format_optional(self._last_steering),
            "imu_accel_x_mps2": self._format_optional(self._last_imu_accel_x),
            "pose_x_m": self._format_optional(self._last_pose_x),
            "pose_y_m": self._format_optional(self._last_pose_y),
            "pose_z_m": self._format_optional(self._last_pose_z),
            "pose_qx": self._format_optional(self._last_pose_qx),
            "pose_qy": self._format_optional(self._last_pose_qy),
            "pose_qz": self._format_optional(self._last_pose_qz),
            "pose_qw": self._format_optional(self._last_pose_qw),
            "status_age_wall_s": "inf" if not math.isfinite(status_age) else f"{status_age:.9f}",
            "watchdog_active": int(watchdog_active),
            "safety_ok": int(safety_ok),
            "fit_eligible": int(eligible),
        }
        with self._output_lock:
            self._csv_writer.writerow(row)
            self._rows_written += 1
            if self._rows_written % 20 == 0:
                self._csv_handle.flush()

    @staticmethod
    def _format_optional(value):
        return "" if value is None else f"{value:.9f}"

    def _finish(self, status, reason):
        with self._finish_lock:
            if self._finished:
                return
            self._finished = True
            self._finish_status = status
            self._finish_reason = reason
            with self._command_lock:
                self._emergency_latched = True
                self._desired_accel = 0.0
                self._desired_brake = 1.0
            with self._output_lock:
                self._csv_handle.flush()
                self._write_summary()
            log = self.get_logger().info if status == "completed" else self.get_logger().error
            log(f"Actuation sweep {status}: {reason}")

    def _write_summary(self):
        if self._summary_written:
            return
        payload = {
            "schema_version": 1,
            "status": self._finish_status,
            "reason": self._finish_reason,
            "cases_total": len(self._cases),
            "cases_finished": len(self._case_results),
            "rows_written": self._rows_written,
            "case_results": self._case_results,
            "output_csv": str(self._output_csv),
            "safety_limits": {
                "maximum_velocity_mps": self._maximum_velocity_mps,
                "maximum_reverse_velocity_mps": self._maximum_reverse_velocity_mps,
                "maximum_settling_reverse_velocity_mps": (
                    self._maximum_settling_reverse_velocity_mps
                ),
                "reset_reference_stability_s": self._reset_reference_stability_s,
                "maximum_reset_reference_z_delta_m": (
                    self._maximum_reset_reference_z_delta_m
                ),
                "maximum_steering_rad": self._maximum_steering_rad,
                "maximum_sample_displacement_m": self._maximum_sample_displacement_m,
                "maximum_displacement_m": self._maximum_displacement_m,
                "status_timeout_wall_s": self._status_timeout_wall_s,
            },
        }
        temporary = self._summary_json.with_name(self._summary_json.name + ".tmp")
        with temporary.open("x", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2, sort_keys=True)
            handle.write("\n")
        os.replace(temporary, self._summary_json)
        self._summary_written = True

    def shutdown_safely(self):
        with self._command_lock:
            self._emergency_latched = True
            self._desired_accel = 0.0
            self._desired_brake = 1.0
        deadline = time.monotonic() + self._shutdown_brake_wall_s
        while time.monotonic() < deadline:
            self._publish_command(0.0, 1.0)
            time.sleep(1.0 / self._command_rate_hz)
        self._stop_watchdog.set()
        if self._watchdog_thread.is_alive():
            self._watchdog_thread.join(timeout=2.0)
        if not self._finished:
            self._finish("failed", "process_interrupted")
        with self._output_lock:
            self._csv_handle.flush()
            self._csv_handle.close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    stop_requested = threading.Event()

    def handle_signal(_signum, _frame):
        stop_requested.set()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)
    try:
        node = CarlaActuationSweep()
        while rclpy.ok() and not stop_requested.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.shutdown_safely()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
