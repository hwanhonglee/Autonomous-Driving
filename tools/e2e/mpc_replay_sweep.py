#!/usr/bin/env python3

"""Deterministic plant-in-loop sweep for Autoware's standard lateral MPC.

The source bag is only opened through rosbag2_py.  Nothing is played back into
ROS: one planning trajectory and its contemporaneous initial state are copied
once, then every controller candidate sees the same frozen trajectory and the
same deterministic bicycle/steering plant.
"""

from __future__ import annotations

import argparse
import copy
import csv
import hashlib
import json
import math
import os
import signal
import subprocess
import sys
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rclpy
import rosbag2_py
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_control_msgs.msg import Control
from autoware_planning_msgs.msg import Trajectory
from autoware_vehicle_msgs.msg import SteeringReport
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import AccelWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rosgraph_msgs.msg import Clock
from rosidl_runtime_py.utilities import get_message


TRAJECTORY_TOPIC = "/planning/trajectory"
ODOMETRY_TOPIC = "/localization/kinematic_state"
STEERING_TOPIC = "/vehicle/status/steering_status"
DEFAULT_DELAYS = "0.09,0.12,0.18,0.24,0.30"
DEFAULT_TAUS = "0.15,0.20,0.24,0.27,0.33"


class ReplayError(RuntimeError):
    pass


@dataclass
class FrozenSnapshot:
    trajectory: Trajectory
    odometry: Odometry
    steering: SteeringReport
    trajectory_index: int
    trajectory_bag_ns: int
    odometry_bag_ns: int
    steering_bag_ns: int
    trajectory_hash: str
    selection: str
    net_heading_change_rad: float
    trajectory_length_m: float


@dataclass
class PlantState:
    x: float
    y: float
    yaw: float
    speed: float
    steer: float


def wrap_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def yaw_from_quaternion(quaternion: Any) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw))


def time_message(seconds: float) -> TimeMsg:
    sec = math.floor(seconds)
    nanosec = int(round((seconds - sec) * 1.0e9))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    return TimeMsg(sec=int(sec), nanosec=nanosec)


def parse_grid(raw: str, label: str) -> list[float]:
    try:
        values = [float(value.strip()) for value in raw.split(",") if value.strip()]
    except ValueError as error:
        raise ReplayError(f"{label} must be a comma-separated number list") from error
    if not values or any(not math.isfinite(value) or value < 0.0 for value in values):
        raise ReplayError(f"{label} values must be finite and non-negative")
    return sorted(set(values))


def trajectory_xy(trajectory: Trajectory) -> np.ndarray:
    return np.asarray(
        [[point.pose.position.x, point.pose.position.y] for point in trajectory.points],
        dtype=float,
    )


def trajectory_geometry(trajectory: Trajectory) -> tuple[float, float]:
    xy = trajectory_xy(trajectory)
    if len(xy) < 2:
        return 0.0, 0.0
    yaws = np.unwrap(
        np.asarray([yaw_from_quaternion(point.pose.orientation) for point in trajectory.points])
    )
    length = float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum())
    return length, float(abs(yaws[-1] - yaws[0]))


def trajectory_digest(trajectory: Trajectory) -> str:
    digest = hashlib.sha256()
    for point in trajectory.points:
        values = (
            point.pose.position.x,
            point.pose.position.y,
            point.pose.position.z,
            point.pose.orientation.x,
            point.pose.orientation.y,
            point.pose.orientation.z,
            point.pose.orientation.w,
            point.longitudinal_velocity_mps,
            point.lateral_velocity_mps,
            point.acceleration_mps2,
            point.heading_rate_rps,
            point.front_wheel_angle_rad,
            point.rear_wheel_angle_rad,
            point.time_from_start.sec,
            point.time_from_start.nanosec,
        )
        digest.update(("|".join(f"{value:.17g}" for value in values) + "\n").encode())
    return digest.hexdigest()


def choose_trajectory_index(
    trajectories: Sequence[tuple[int, Trajectory]], requested_index: int | None
) -> int:
    if not trajectories:
        raise ReplayError(f"bag contains no {TRAJECTORY_TOPIC} messages")
    if requested_index is not None:
        index = requested_index if requested_index >= 0 else len(trajectories) + requested_index
        if not 0 <= index < len(trajectories):
            raise ReplayError(
                f"trajectory index {requested_index} is outside [0, {len(trajectories) - 1}]"
            )
        if len(trajectories[index][1].points) < 3:
            raise ReplayError(f"trajectory index {requested_index} has fewer than three points")
        return index

    candidates: list[tuple[float, float, int]] = []
    for index, (_, trajectory) in enumerate(trajectories):
        length, heading_change = trajectory_geometry(trajectory)
        if len(trajectory.points) >= 10 and length >= 5.0:
            candidates.append((heading_change, length, index))
    if not candidates:
        raise ReplayError("bag has no trajectory with at least 10 points and 5 m length")
    # A large endpoint heading change identifies a complete corner and avoids
    # selecting a short high-curvature kink inside a later snapshot.
    return max(candidates)[2]


def _nearest_record(records: Sequence[tuple[int, Any]], timestamp_ns: int, label: str) -> tuple[int, Any]:
    if not records:
        raise ReplayError(f"bag contains no {label} messages")
    return min(records, key=lambda record: abs(record[0] - timestamp_ns))


def load_frozen_snapshot(bag_path: Path, trajectory_index: int | None = None) -> FrozenSnapshot:
    resolved = bag_path.expanduser().resolve()
    if not resolved.exists():
        raise ReplayError(f"bag path does not exist: {resolved}")

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(resolved), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    required = (TRAJECTORY_TOPIC, ODOMETRY_TOPIC, STEERING_TOPIC)
    missing = [topic for topic in required if topic not in topic_types]
    if missing:
        raise ReplayError(f"bag is missing required topics: {', '.join(missing)}")
    message_types = {topic: get_message(topic_types[topic]) for topic in required}
    trajectories: list[tuple[int, Trajectory]] = []
    odometry: list[tuple[int, Odometry]] = []
    steering: list[tuple[int, SteeringReport]] = []
    destinations = {
        TRAJECTORY_TOPIC: trajectories,
        ODOMETRY_TOPIC: odometry,
        STEERING_TOPIC: steering,
    }
    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        if topic in destinations:
            destinations[topic].append(
                (timestamp_ns, deserialize_message(data, message_types[topic]))
            )

    index = choose_trajectory_index(trajectories, trajectory_index)
    trajectory_ns, trajectory = trajectories[index]
    odometry_ns, odometry_msg = _nearest_record(odometry, trajectory_ns, ODOMETRY_TOPIC)
    steering_ns, steering_msg = _nearest_record(steering, trajectory_ns, STEERING_TOPIC)
    length, heading_change = trajectory_geometry(trajectory)
    return FrozenSnapshot(
        trajectory=trajectory,
        odometry=odometry_msg,
        steering=steering_msg,
        trajectory_index=index,
        trajectory_bag_ns=trajectory_ns,
        odometry_bag_ns=odometry_ns,
        steering_bag_ns=steering_ns,
        trajectory_hash=trajectory_digest(trajectory),
        selection=("explicit_index" if trajectory_index is not None else "maximum_endpoint_heading_change"),
        net_heading_change_rad=heading_change,
        trajectory_length_m=length,
    )


def point_to_polyline(
    point: Sequence[float], polyline: np.ndarray
) -> tuple[float, float, float, int]:
    """Return signed CTE, path progress, reference yaw and nearest segment."""
    if len(polyline) < 2:
        raise ReplayError("trajectory must have at least two distinct points")
    starts = polyline[:-1]
    vectors = polyline[1:] - starts
    length_squared = np.einsum("ij,ij->i", vectors, vectors)
    valid = length_squared > 1.0e-12
    if not np.any(valid):
        raise ReplayError("trajectory has no non-zero-length segment")
    relative = np.asarray(point, dtype=float) - starts
    fractions = np.zeros(len(vectors), dtype=float)
    fractions[valid] = np.clip(
        np.einsum("ij,ij->i", relative[valid], vectors[valid]) / length_squared[valid],
        0.0,
        1.0,
    )
    projections = starts + fractions[:, None] * vectors
    deltas = np.asarray(point, dtype=float) - projections
    distances_squared = np.einsum("ij,ij->i", deltas, deltas)
    distances_squared[~valid] = np.inf
    index = int(np.argmin(distances_squared))
    distance = math.sqrt(float(distances_squared[index]))
    cross = vectors[index, 0] * deltas[index, 1] - vectors[index, 1] * deltas[index, 0]
    signed = math.copysign(distance, cross) if distance > 0.0 and cross != 0.0 else 0.0
    segment_lengths = np.sqrt(length_squared)
    progress = float(segment_lengths[:index].sum() + fractions[index] * segment_lengths[index])
    reference_yaw = math.atan2(vectors[index, 1], vectors[index, 0])
    return signed, progress, reference_yaw, index


def advance_plant(
    state: PlantState,
    command: float,
    command_buffer: deque[float],
    dt: float,
    wheel_base: float,
    steer_tau: float,
    max_steer: float,
) -> tuple[PlantState, float]:
    command_buffer.append(float(np.clip(command, -max_steer, max_steer)))
    delayed_command = command_buffer.popleft()
    if steer_tau <= 1.0e-9:
        steer = delayed_command
    else:
        alpha = 1.0 - math.exp(-dt / steer_tau)
        steer = state.steer + alpha * (delayed_command - state.steer)
    steer = float(np.clip(steer, -max_steer, max_steer))
    yaw_rate = state.speed * math.tan(steer) / wheel_base
    midpoint_yaw = state.yaw + 0.5 * yaw_rate * dt
    return (
        PlantState(
            x=state.x + state.speed * math.cos(midpoint_yaw) * dt,
            y=state.y + state.speed * math.sin(midpoint_yaw) * dt,
            yaw=wrap_angle(state.yaw + yaw_rate * dt),
            speed=state.speed,
            steer=steer,
        ),
        delayed_command,
    )


def summarize_trace(trace: Sequence[dict[str, float]], missing_commands: int) -> dict[str, float]:
    if not trace:
        raise ReplayError("controller produced no trace samples")
    absolute_cte = np.abs(np.asarray([sample["cte_m"] for sample in trace], dtype=float))
    heading = np.abs(np.asarray([sample["heading_error_rad"] for sample in trace], dtype=float))
    commands = np.asarray([sample["steer_command_rad"] for sample in trace], dtype=float)
    raw_commands = np.asarray(
        [sample.get("steer_raw_command_rad", sample["steer_command_rad"]) for sample in trace],
        dtype=float,
    )
    measured = np.asarray([sample["steer_measured_rad"] for sample in trace], dtype=float)
    cte_rms = float(np.sqrt(np.mean(np.square(absolute_cte))))
    cte_p95 = float(np.percentile(absolute_cte, 95))
    cte_max = float(np.max(absolute_cte))
    score = 0.5 * cte_rms + 0.3 * cte_p95 + 0.2 * cte_max + 10.0 * missing_commands
    return {
        "samples": len(trace),
        "missing_commands": missing_commands,
        "cte_rms_m": cte_rms,
        "cte_p95_m": cte_p95,
        "cte_max_m": cte_max,
        "signed_cte_min_m": float(min(sample["cte_m"] for sample in trace)),
        "signed_cte_max_m": float(max(sample["cte_m"] for sample in trace)),
        "heading_error_p95_rad": float(np.percentile(heading, 95)),
        "heading_error_max_rad": float(np.max(heading)),
        "steer_command_peak_rad": float(np.max(np.abs(commands))),
        "steer_raw_command_peak_rad": float(np.max(np.abs(raw_commands))),
        "steer_measured_peak_rad": float(np.max(np.abs(measured))),
        "steer_command_total_variation_rad": float(np.abs(np.diff(commands)).sum()),
        "final_progress_m": float(trace[-1]["progress_m"]),
        "score": score,
    }


class ReplayNode(Node):
    def __init__(self, namespace: str):
        super().__init__(f"plant_{namespace}")
        prefix = f"/{namespace}/controller"
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        transient = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        clock_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.clock_pub = self.create_publisher(Clock, "/clock", clock_qos)
        self.trajectory_pub = self.create_publisher(
            Trajectory, f"{prefix}/input/reference_trajectory", qos
        )
        self.odometry_pub = self.create_publisher(Odometry, f"{prefix}/input/current_odometry", qos)
        self.steering_pub = self.create_publisher(
            SteeringReport, f"{prefix}/input/current_steering", qos
        )
        self.accel_pub = self.create_publisher(
            AccelWithCovarianceStamped, f"{prefix}/input/current_accel", qos
        )
        self.operation_pub = self.create_publisher(
            OperationModeState, f"{prefix}/input/current_operation_mode", transient
        )
        self.commands: list[Control] = []
        self.command_sub = self.create_subscription(
            Control, f"{prefix}/output/control_cmd", self.commands.append, transient
        )

    def connected(self) -> bool:
        publishers = (
            self.trajectory_pub,
            self.odometry_pub,
            self.steering_pub,
            self.accel_pub,
            self.operation_pub,
        )
        return all(publisher.get_subscription_count() >= 1 for publisher in publishers)


def controller_paths() -> tuple[Path, list[Path]]:
    executable = (
        Path(get_package_prefix("autoware_trajectory_follower_node"))
        / "lib/autoware_trajectory_follower_node/controller_node_exe"
    )
    trajectory_share = Path(get_package_share_directory("autoware_trajectory_follower_node"))
    mpc_share = Path(get_package_share_directory("autoware_mpc_lateral_controller"))
    pid_share = Path(get_package_share_directory("autoware_pid_longitudinal_controller"))
    launch_share = Path(get_package_share_directory("autoware_launch"))
    vehicle_share = Path(get_package_share_directory("sample_vehicle_description"))
    params = [
        mpc_share / "param/lateral_controller_defaults.param.yaml",
        mpc_share / "param/steer_offset.param.yaml",
        pid_share / "config/autoware_pid_longitudinal_controller.param.yaml",
        trajectory_share / "param/trajectory_follower_node.param.yaml",
        launch_share / "config/control/common/nearest_search.param.yaml",
        vehicle_share / "config/vehicle_info.param.yaml",
    ]
    missing = [path for path in (executable, *params) if not path.exists()]
    if missing:
        raise ReplayError("missing installed controller files: " + ", ".join(map(str, missing)))
    return executable, params


def controller_overrides(args: argparse.Namespace) -> dict[str, Any]:
    overrides: dict[str, Any] = {}
    uniform_weights = {
        "steering_input": args.steering_input_weight,
        "heading_error_squared_vel": args.heading_error_squared_vel_weight,
        "steer_rate": args.steer_rate_weight,
        "steer_acc": args.steer_acc_weight,
    }
    for suffix, value in uniform_weights.items():
        if value is None:
            continue
        overrides[f"mpc_weight_{suffix}"] = float(value)
        overrides[f"mpc_low_curvature_weight_{suffix}"] = float(value)
    if args.steering_lpf_cutoff_hz is not None:
        overrides["steering_lpf_cutoff_hz"] = float(args.steering_lpf_cutoff_hz)
    if args.steer_rate_limit_dps is not None:
        overrides["steer_rate_lim_dps_list_by_curvature"] = [
            float(args.steer_rate_limit_dps)
        ] * 3
        overrides["steer_rate_lim_dps_list_by_velocity"] = [
            float(args.steer_rate_limit_dps)
        ] * 3
    if args.max_controller_steer_angle_rad is not None:
        overrides["max_steer_angle"] = float(args.max_controller_steer_angle_rad)
    return overrides


def write_overlay(
    path: Path,
    input_delay: float,
    steer_tau: float,
    ctrl_period: float,
    overrides: dict[str, Any] | None = None,
) -> None:
    parameters: dict[str, Any] = {
        "input_delay": float(input_delay),
        "vehicle_model_steer_tau": float(steer_tau),
        "ctrl_period": float(ctrl_period),
        "lateral_controller_mode": "mpc",
        "longitudinal_controller_mode": "pid",
        "steer_offset_param_name": "steer_offset",
        "use_sim_time": True,
    }
    parameters.update(overrides or {})
    path.write_text(
        yaml.safe_dump({"/**": {"ros__parameters": parameters}}, sort_keys=False),
        encoding="utf-8",
    )


def make_initial_state(snapshot: FrozenSnapshot, speed_override: float | None) -> PlantState:
    pose = snapshot.odometry.pose.pose
    recorded_speed = float(snapshot.odometry.twist.twist.linear.x)
    speed = recorded_speed if speed_override is None else speed_override
    if speed <= 0.0:
        positive = [
            float(point.longitudinal_velocity_mps)
            for point in snapshot.trajectory.points
            if point.longitudinal_velocity_mps > 0.0
        ]
        speed = float(np.median(positive)) if positive else 2.5
    return PlantState(
        x=float(pose.position.x),
        y=float(pose.position.y),
        yaw=yaw_from_quaternion(pose.orientation),
        speed=speed,
        steer=float(snapshot.steering.steering_tire_angle),
    )


def publish_inputs(
    node: ReplayNode,
    snapshot: FrozenSnapshot,
    state: PlantState,
    sim_time: float,
) -> None:
    stamp = time_message(sim_time)
    trajectory = copy.deepcopy(snapshot.trajectory)
    trajectory.header.stamp = stamp
    trajectory.header.frame_id = trajectory.header.frame_id or "map"
    odometry = Odometry()
    odometry.header.stamp = stamp
    odometry.header.frame_id = trajectory.header.frame_id
    odometry.child_frame_id = "base_link"
    odometry.pose.pose.position.x = state.x
    odometry.pose.pose.position.y = state.y
    qx, qy, qz, qw = quaternion_from_yaw(state.yaw)
    odometry.pose.pose.orientation.x = qx
    odometry.pose.pose.orientation.y = qy
    odometry.pose.pose.orientation.z = qz
    odometry.pose.pose.orientation.w = qw
    odometry.twist.twist.linear.x = state.speed
    odometry.twist.twist.angular.z = state.speed * math.tan(state.steer) / 2.79
    steering = SteeringReport(stamp=stamp, steering_tire_angle=state.steer)
    accel = AccelWithCovarianceStamped()
    accel.header.stamp = stamp
    operation = OperationModeState()
    operation.stamp = stamp
    operation.mode = OperationModeState.AUTONOMOUS
    operation.is_autoware_control_enabled = True
    operation.is_autonomous_mode_available = True
    node.trajectory_pub.publish(trajectory)
    node.odometry_pub.publish(odometry)
    node.steering_pub.publish(steering)
    node.accel_pub.publish(accel)
    node.operation_pub.publish(operation)


def publish_clock(node: ReplayNode, sim_time: float) -> None:
    node.clock_pub.publish(Clock(clock=time_message(sim_time)))


def wait_until(node: ReplayNode, predicate: Any, timeout: float) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=min(0.02, max(0.0, deadline - time.monotonic())))
        if predicate():
            return True
    return bool(predicate())


def terminate_process(process: subprocess.Popen[Any], timeout: float = 5.0) -> None:
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=timeout)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=2.0)


def run_candidate(
    snapshot: FrozenSnapshot,
    output_dir: Path,
    input_delay: float,
    controller_tau: float,
    args: argparse.Namespace,
    run_number: int,
) -> dict[str, Any]:
    effective_delay = round(input_delay / args.ctrl_period) * args.ctrl_period
    simulation_step = args.simulation_step or args.ctrl_period
    candidate_name = (
        f"delay_{input_delay:.3f}_tau_{controller_tau:.3f}_run_{run_number:02d}".replace(
            ".", "p"
        )
    )
    candidate_dir = output_dir / "candidates" / candidate_name
    candidate_dir.mkdir(parents=True, exist_ok=True)
    overlay = candidate_dir / "controller_override.yaml"
    smoothing_overrides = controller_overrides(args)
    write_overlay(
        overlay,
        input_delay,
        controller_tau,
        args.ctrl_period,
        smoothing_overrides,
    )
    executable, param_paths = controller_paths()
    namespace = f"mpc_replay_{os.getpid()}_{candidate_name}"
    command = [str(executable), "--ros-args", "-r", f"__ns:=/{namespace}"]
    for path in param_paths:
        command.extend(["--params-file", str(path)])
    command.extend(["--params-file", str(overlay)])
    log_path = candidate_dir / "controller.log"
    log_handle = log_path.open("w", encoding="utf-8")
    process = subprocess.Popen(
        command,
        stdout=log_handle,
        stderr=subprocess.STDOUT,
        start_new_session=True,
        env=os.environ.copy(),
        text=True,
    )
    node: ReplayNode | None = None
    try:
        node = ReplayNode(namespace)
        if not wait_until(node, lambda: node.connected() or process.poll() is not None, 10.0):
            raise ReplayError(f"controller discovery timed out; see {log_path}")
        if process.poll() is not None:
            raise ReplayError(f"controller exited with {process.returncode}; see {log_path}")

        state = make_initial_state(snapshot, args.speed)
        xy = trajectory_xy(snapshot.trajectory)
        path_length = float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum())
        delay_steps = round(args.plant_delay / simulation_step)
        command_buffer: deque[float] = deque([state.steer] * delay_steps)
        trace: list[dict[str, float]] = []
        missing_commands = 0
        sim_time = 1.0
        publish_clock(node, sim_time)
        publish_inputs(node, snapshot, state, sim_time)
        # Let DDS deliver all five inputs while simulated time is held fixed.
        wait_until(node, lambda: False, 0.08)

        steps = int(math.ceil(args.duration / simulation_step))
        for step in range(steps):
            publish_inputs(node, snapshot, state, sim_time)
            wait_until(node, lambda: False, args.input_settle_wall_time)
            previous_count = len(node.commands)
            sim_time += simulation_step
            publish_clock(node, sim_time)
            received = wait_until(
                node,
                lambda: len(node.commands) > previous_count or process.poll() is not None,
                args.command_timeout,
            )
            if process.poll() is not None:
                raise ReplayError(f"controller exited with {process.returncode}; see {log_path}")
            if not received or len(node.commands) <= previous_count:
                missing_commands += 1
                if missing_commands > args.max_missing_commands:
                    raise ReplayError(
                        f"controller missed {missing_commands} commands; see {log_path}"
                    )
                command_angle = state.steer
            else:
                command_angle = float(node.commands[-1].lateral.steering_tire_angle)
            raw_command_angle = command_angle
            if args.plant_command_steer_cap_rad is not None:
                command_angle = float(
                    np.clip(
                        command_angle,
                        -args.plant_command_steer_cap_rad,
                        args.plant_command_steer_cap_rad,
                    )
                )
            state, delayed_command = advance_plant(
                state,
                command_angle,
                command_buffer,
                simulation_step,
                args.wheel_base,
                args.plant_tau,
                args.max_steer,
            )
            cte, progress, reference_yaw, _ = point_to_polyline((state.x, state.y), xy)
            trace.append(
                {
                    "step": float(step),
                    "time_sec": (step + 1) * simulation_step,
                    "x_m": state.x,
                    "y_m": state.y,
                    "yaw_rad": state.yaw,
                    "cte_m": cte,
                    "progress_m": progress,
                    "remaining_m": max(0.0, path_length - progress),
                    "heading_error_rad": wrap_angle(state.yaw - reference_yaw),
                    "steer_raw_command_rad": raw_command_angle,
                    "steer_command_rad": command_angle,
                    "steer_delayed_command_rad": delayed_command,
                    "steer_measured_rad": state.steer,
                }
            )
            if path_length - progress <= args.end_margin:
                break

        metrics = summarize_trace(trace, missing_commands)
        result: dict[str, Any] = {
            "status": "pass",
            "candidate": candidate_name,
            "requested_input_delay_sec": input_delay,
            "effective_input_delay_sec": effective_delay,
            "controller_steer_tau_sec": controller_tau,
            "controller_overrides": smoothing_overrides,
            "plant_command_steer_cap_rad": args.plant_command_steer_cap_rad,
            "plant_delay_sec": args.plant_delay,
            "effective_plant_delay_sec": delay_steps * simulation_step,
            "plant_delay_steps": delay_steps,
            "simulation_step_sec": simulation_step,
            "plant_steer_tau_sec": args.plant_tau,
            "trajectory_hash": snapshot.trajectory_hash,
            "metrics": metrics,
            "trace": trace,
        }
        write_candidate_files(candidate_dir, result)
        return result
    finally:
        if node is not None:
            node.destroy_node()
        terminate_process(process)
        log_handle.close()


def write_candidate_files(candidate_dir: Path, result: dict[str, Any]) -> None:
    (candidate_dir / "result.json").write_text(
        json.dumps({key: value for key, value in result.items() if key != "trace"}, indent=2)
        + "\n",
        encoding="utf-8",
    )
    if result.get("trace"):
        fieldnames = list(result["trace"][0])
        with (candidate_dir / "trace.csv").open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(result["trace"])


def snapshot_payload(snapshot: FrozenSnapshot, bag_path: Path) -> dict[str, Any]:
    odometry = snapshot.odometry
    steering = snapshot.steering
    return {
        "source_bag": str(bag_path.expanduser().resolve()),
        "selection": snapshot.selection,
        "trajectory_index": snapshot.trajectory_index,
        "trajectory_bag_timestamp_ns": snapshot.trajectory_bag_ns,
        "odometry_bag_timestamp_ns": snapshot.odometry_bag_ns,
        "steering_bag_timestamp_ns": snapshot.steering_bag_ns,
        "state_time_offset_sec": (snapshot.odometry_bag_ns - snapshot.trajectory_bag_ns) / 1.0e9,
        "steering_time_offset_sec": (snapshot.steering_bag_ns - snapshot.trajectory_bag_ns) / 1.0e9,
        "trajectory_hash": snapshot.trajectory_hash,
        "trajectory_points": len(snapshot.trajectory.points),
        "trajectory_length_m": snapshot.trajectory_length_m,
        "net_heading_change_rad": snapshot.net_heading_change_rad,
        "initial_state": {
            "x_m": odometry.pose.pose.position.x,
            "y_m": odometry.pose.pose.position.y,
            "yaw_rad": yaw_from_quaternion(odometry.pose.pose.orientation),
            "speed_mps": odometry.twist.twist.linear.x,
            "steering_tire_angle_rad": steering.steering_tire_angle,
        },
        "trajectory": [
            {
                "x_m": point.pose.position.x,
                "y_m": point.pose.position.y,
                "yaw_rad": yaw_from_quaternion(point.pose.orientation),
                "velocity_mps": point.longitudinal_velocity_mps,
                "time_from_start_sec": point.time_from_start.sec
                + point.time_from_start.nanosec / 1.0e9,
            }
            for point in snapshot.trajectory.points
        ],
    }


def aggregate_results(results: Sequence[dict[str, Any]]) -> list[dict[str, Any]]:
    groups: dict[tuple[float, float], list[dict[str, Any]]] = {}
    for result in results:
        key = (result["requested_input_delay_sec"], result["controller_steer_tau_sec"])
        groups.setdefault(key, []).append(result)
    aggregate = []
    for (delay, tau), runs in groups.items():
        metric_names = list(runs[0]["metrics"])
        metrics = {
            name: float(np.mean([run["metrics"][name] for run in runs]))
            for name in metric_names
        }
        metric_std = {
            name: float(np.std([run["metrics"][name] for run in runs]))
            for name in metric_names
        }
        aggregate.append(
            {
                "requested_input_delay_sec": delay,
                "effective_input_delay_sec": runs[0]["effective_input_delay_sec"],
                "controller_steer_tau_sec": tau,
                "runs": len(runs),
                "metrics_mean": metrics,
                "metrics_std": metric_std,
                "representative_result": runs[0],
            }
        )
    return sorted(aggregate, key=lambda result: result["metrics_mean"]["score"])


def write_summary_csv(path: Path, aggregate: Sequence[dict[str, Any]]) -> None:
    fields = [
        "rank",
        "requested_input_delay_sec",
        "effective_input_delay_sec",
        "controller_steer_tau_sec",
        "runs",
        "score",
        "cte_rms_m",
        "cte_p95_m",
        "cte_max_m",
        "heading_error_p95_rad",
        "steer_command_peak_rad",
        "steer_command_total_variation_rad",
    ]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for rank, result in enumerate(aggregate, start=1):
            metrics = result["metrics_mean"]
            writer.writerow(
                {
                    "rank": rank,
                    "requested_input_delay_sec": result["requested_input_delay_sec"],
                    "effective_input_delay_sec": result["effective_input_delay_sec"],
                    "controller_steer_tau_sec": result["controller_steer_tau_sec"],
                    "runs": result["runs"],
                    **{field: metrics[field] for field in fields[5:]},
                }
            )


def find_baseline(aggregate: Sequence[dict[str, Any]]) -> dict[str, Any] | None:
    return next(
        (
            result
            for result in aggregate
            if math.isclose(result["requested_input_delay_sec"], 0.24, abs_tol=1.0e-9)
            and math.isclose(result["controller_steer_tau_sec"], 0.27, abs_tol=1.0e-9)
        ),
        None,
    )


def render_summary(
    path: Path,
    snapshot: FrozenSnapshot,
    aggregate: Sequence[dict[str, Any]],
    delays: Sequence[float],
    taus: Sequence[float],
) -> None:
    best = aggregate[0]
    baseline = find_baseline(aggregate)
    selected = [best]
    if baseline is not None and baseline is not best:
        selected.append(baseline)
    fig, axes = plt.subplots(2, 2, figsize=(13, 9), constrained_layout=True)
    reference = trajectory_xy(snapshot.trajectory)
    axes[0, 0].plot(reference[:, 0], reference[:, 1], "k--", linewidth=2, label="frozen final")
    for result in selected:
        trace = result["representative_result"]["trace"]
        label = (
            f"d={result['requested_input_delay_sec']:.2f}, "
            f"tau={result['controller_steer_tau_sec']:.2f}"
        )
        axes[0, 0].plot(
            [sample["x_m"] for sample in trace],
            [sample["y_m"] for sample in trace],
            linewidth=2,
            label=label,
        )
    axes[0, 0].set_title("Closed-loop path")
    axes[0, 0].axis("equal")
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].legend()

    for result in selected:
        trace = result["representative_result"]["trace"]
        label = (
            f"d={result['requested_input_delay_sec']:.2f}, "
            f"tau={result['controller_steer_tau_sec']:.2f}"
        )
        axes[0, 1].plot(
            [sample["time_sec"] for sample in trace],
            [sample["cte_m"] for sample in trace],
            label=label,
        )
    axes[0, 1].axhline(0.0, color="black", linewidth=1)
    axes[0, 1].set_title("Signed final-trajectory CTE")
    axes[0, 1].set_xlabel("time [s]")
    axes[0, 1].set_ylabel("CTE [m]")
    axes[0, 1].grid(True, alpha=0.3)
    axes[0, 1].legend()

    matrix = np.full((len(taus), len(delays)), np.nan)
    for result in aggregate:
        row = list(taus).index(result["controller_steer_tau_sec"])
        column = list(delays).index(result["requested_input_delay_sec"])
        matrix[row, column] = result["metrics_mean"]["score"]
    image = axes[1, 0].imshow(matrix, aspect="auto", origin="lower", cmap="viridis_r")
    axes[1, 0].set_xticks(range(len(delays)), [f"{value:.2f}" for value in delays])
    axes[1, 0].set_yticks(range(len(taus)), [f"{value:.2f}" for value in taus])
    axes[1, 0].set_xlabel("requested input delay [s]")
    axes[1, 0].set_ylabel("controller steer tau [s]")
    axes[1, 0].set_title("Tracking score (lower is better)")
    fig.colorbar(image, ax=axes[1, 0], label="score")

    for result in selected:
        trace = result["representative_result"]["trace"]
        label = (
            f"d={result['requested_input_delay_sec']:.2f}, "
            f"tau={result['controller_steer_tau_sec']:.2f}"
        )
        axes[1, 1].plot(
            [sample["time_sec"] for sample in trace],
            [sample["steer_command_rad"] for sample in trace],
            label=f"command {label}",
        )
        axes[1, 1].plot(
            [sample["time_sec"] for sample in trace],
            [sample["steer_measured_rad"] for sample in trace],
            linestyle="--",
            label=f"plant {label}",
        )
    axes[1, 1].set_title("Steering command and plant response")
    axes[1, 1].set_xlabel("time [s]")
    axes[1, 1].set_ylabel("tire angle [rad]")
    axes[1, 1].grid(True, alpha=0.3)
    axes[1, 1].legend(fontsize=8)
    fig.suptitle(
        f"Autoware MPC frozen-trajectory sweep | snapshot {snapshot.trajectory_index} | "
        f"{snapshot.trajectory_hash[:12]}"
    )
    fig.savefig(path, dpi=150)
    plt.close(fig)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="rosbag2 directory containing a valid run")
    parser.add_argument("output_dir", type=Path, help="new artifact directory")
    parser.add_argument(
        "--trajectory-index",
        type=int,
        help="planning trajectory message index; default selects the largest endpoint turn",
    )
    parser.add_argument("--input-delays", default=DEFAULT_DELAYS)
    parser.add_argument("--steer-taus", default=DEFAULT_TAUS)
    parser.add_argument("--repeats", type=int, default=1)
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--ctrl-period", type=float, default=0.03)
    parser.add_argument(
        "--simulation-step",
        type=float,
        help="clock and plant step; default matches --ctrl-period",
    )
    parser.add_argument("--plant-delay", type=float, default=0.10)
    parser.add_argument("--plant-tau", type=float, default=0.20)
    parser.add_argument("--wheel-base", type=float, default=2.79)
    parser.add_argument("--max-steer", type=float, default=0.70)
    parser.add_argument("--speed", type=float, help="constant plant speed; default uses bag odometry")
    parser.add_argument("--end-margin", type=float, default=0.5)
    parser.add_argument("--command-timeout", type=float, default=0.5)
    parser.add_argument("--input-settle-wall-time", type=float, default=0.004)
    parser.add_argument("--max-missing-commands", type=int, default=0)
    parser.add_argument("--ros-domain-id", type=int, default=142)
    parser.add_argument("--steering-input-weight", type=float)
    parser.add_argument("--heading-error-squared-vel-weight", type=float)
    parser.add_argument("--steer-rate-weight", type=float)
    parser.add_argument("--steer-acc-weight", type=float)
    parser.add_argument("--steering-lpf-cutoff-hz", type=float)
    parser.add_argument("--steer-rate-limit-dps", type=float)
    parser.add_argument("--max-controller-steer-angle-rad", type=float)
    parser.add_argument(
        "--plant-command-steer-cap-rad",
        type=float,
        help="clamp the command applied to the plant while preserving raw MPC output",
    )
    args = parser.parse_args(argv)
    positive = (
        "repeats",
        "duration",
        "ctrl_period",
        "plant_tau",
        "wheel_base",
        "max_steer",
        "command_timeout",
    )
    for name in positive:
        value = getattr(args, name)
        if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
            parser.error(f"--{name.replace('_', '-')} must be positive")
    if args.simulation_step is not None and (
        not math.isfinite(args.simulation_step) or args.simulation_step <= 0.0
    ):
        parser.error("--simulation-step must be positive and finite")
    if args.plant_delay < 0 or args.end_margin < 0 or args.input_settle_wall_time < 0:
        parser.error("delay, end margin and settle time must be non-negative")
    if args.max_missing_commands < 0:
        parser.error("--max-missing-commands must be non-negative")
    if args.speed is not None and (not math.isfinite(args.speed) or args.speed <= 0.0):
        parser.error("--speed must be positive")
    for name in (
        "steering_input_weight",
        "heading_error_squared_vel_weight",
        "steer_rate_weight",
        "steer_acc_weight",
    ):
        value = getattr(args, name)
        if value is not None and (not math.isfinite(value) or value < 0.0):
            parser.error(f"--{name.replace('_', '-')} must be finite and non-negative")
    for name in (
        "steering_lpf_cutoff_hz",
        "steer_rate_limit_dps",
        "max_controller_steer_angle_rad",
        "plant_command_steer_cap_rad",
    ):
        value = getattr(args, name)
        if value is not None and (not math.isfinite(value) or value <= 0.0):
            parser.error(f"--{name.replace('_', '-')} must be positive and finite")
    if not 0 <= args.ros_domain_id <= 232:
        parser.error("--ros-domain-id must be in [0, 232]")
    return args


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        delays = parse_grid(args.input_delays, "--input-delays")
        taus = parse_grid(args.steer_taus, "--steer-taus")
        snapshot = load_frozen_snapshot(args.bag, args.trajectory_index)
        output_dir = args.output_dir.expanduser().resolve()
        if output_dir.exists() and any(output_dir.iterdir()):
            raise ReplayError(f"output directory is not empty: {output_dir}")
        output_dir.mkdir(parents=True, exist_ok=True)
        os.environ["ROS_DOMAIN_ID"] = str(args.ros_domain_id)
        rclpy.init(args=None)
        results: list[dict[str, Any]] = []
        try:
            total = len(delays) * len(taus) * args.repeats
            completed = 0
            for delay in delays:
                for tau in taus:
                    for run_number in range(1, args.repeats + 1):
                        completed += 1
                        print(
                            f"[{completed}/{total}] input_delay={delay:.3f} "
                            f"steer_tau={tau:.3f} run={run_number}",
                            flush=True,
                        )
                        results.append(
                            run_candidate(
                                snapshot, output_dir, delay, tau, args, run_number
                            )
                        )
        finally:
            rclpy.shutdown()

        aggregate = aggregate_results(results)
        baseline = find_baseline(aggregate)
        best = aggregate[0]
        baseline_score = baseline["metrics_mean"]["score"] if baseline else None
        improvement = (
            100.0 * (baseline_score - best["metrics_mean"]["score"]) / baseline_score
            if baseline_score and baseline_score > 0.0
            else None
        )
        snapshot_data = snapshot_payload(snapshot, args.bag)
        (output_dir / "frozen_snapshot.json").write_text(
            json.dumps(snapshot_data, indent=2) + "\n", encoding="utf-8"
        )
        summary = {
            "schema_version": 1,
            "method": "official_autoware_mpc_deterministic_lateral_plant_in_loop",
            "isolation": {
                "ros_domain_id": args.ros_domain_id,
                "bag_play_used": False,
                "trajectory_geometry_frozen": True,
                "trajectory_hash": snapshot.trajectory_hash,
                "recorded_odometry_replayed": False,
                "recorded_control_replayed": False,
                "longitudinal_plant": "constant recorded speed",
            },
            "configuration": {
                "ctrl_period_sec": args.ctrl_period,
                "simulation_step_sec": args.simulation_step or args.ctrl_period,
                "plant_delay_sec": args.plant_delay,
                "effective_plant_delay_sec": round(
                    args.plant_delay / (args.simulation_step or args.ctrl_period)
                )
                * (args.simulation_step or args.ctrl_period),
                "plant_tau_sec": args.plant_tau,
                "wheel_base_m": args.wheel_base,
                "duration_sec": args.duration,
                "repeats": args.repeats,
                "controller_overrides": controller_overrides(args),
                "plant_command_steer_cap_rad": args.plant_command_steer_cap_rad,
            },
            "snapshot": {key: value for key, value in snapshot_data.items() if key != "trajectory"},
            "best": {key: value for key, value in best.items() if key != "representative_result"},
            "baseline": (
                {key: value for key, value in baseline.items() if key != "representative_result"}
                if baseline
                else None
            ),
            "best_score_improvement_vs_default_percent": improvement,
            "ranking": [
                {key: value for key, value in result.items() if key != "representative_result"}
                for result in aggregate
            ],
        }
        (output_dir / "sweep_summary.json").write_text(
            json.dumps(summary, indent=2) + "\n", encoding="utf-8"
        )
        write_summary_csv(output_dir / "sweep_summary.csv", aggregate)
        render_summary(output_dir / "sweep_summary.png", snapshot, aggregate, delays, taus)
        print(
            "best: "
            f"input_delay={best['requested_input_delay_sec']:.3f} "
            f"(effective={best['effective_input_delay_sec']:.3f}), "
            f"steer_tau={best['controller_steer_tau_sec']:.3f}, "
            f"score={best['metrics_mean']['score']:.6f}",
            flush=True,
        )
        if improvement is not None:
            print(f"score improvement vs 0.24/0.27: {improvement:.2f}%", flush=True)
        return 0
    except (ReplayError, OSError, RuntimeError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
