#!/usr/bin/env python3

import argparse
from datetime import date
import json
import math
import os
from pathlib import Path
import signal
import statistics
import subprocess
import sys
import time


def percentile(values, quantile):
    if not values:
        return None
    ordered = sorted(values)
    index = max(0, math.ceil(quantile * len(ordered)) - 1)
    return ordered[index]


def stats(values):
    if not values:
        return {"count": 0, "mean": None, "p95": None, "max": None}
    return {
        "count": len(values),
        "mean": statistics.mean(values),
        "p95": percentile(values, 0.95),
        "max": max(values),
    }


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run a CARLA-free nominal Smart MPC iLQR command smoke test."
    )
    parser.add_argument("--domain-id", type=int, default=142)
    parser.add_argument("--timeout", type=float, default=45.0)
    parser.add_argument("--sample-seconds", type=float, default=5.0)
    parser.add_argument(
        "--scenario",
        choices=("straight", "distant_goal_stop", "near_goal_stop"),
        default="straight",
    )
    parser.add_argument("--runtime-param-file", type=Path)
    parser.add_argument(
        "--expect-goal-stop-guard",
        choices=("ignore", "active", "inactive"),
        default="ignore",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(
            f"artifacts/diagnostics/{date.today().isoformat()}_smart_mpc_synthetic_smoke"
        ),
    )
    return parser.parse_args()


def main():
    args = parse_args()
    project_domain = int(os.environ.get("AUTOWARE_E2E_ROS_DOMAIN_ID", "42"))
    if args.domain_id == project_domain:
        raise SystemExit(
            f"Refusing to use active project ROS domain {project_domain}; "
            "select an isolated domain."
        )

    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)

    existing = subprocess.run(
        ["ros2", "node", "list", "--no-daemon"],
        check=False,
        capture_output=True,
        text=True,
        timeout=10,
    )
    if existing.returncode == 0 and existing.stdout.strip():
        raise SystemExit(
            f"ROS domain {args.domain_id} is not empty:\n{existing.stdout.strip()}"
        )

    from autoware_control_msgs.msg import Control
    from autoware_internal_debug_msgs.msg import BoolStamped
    from autoware_internal_debug_msgs.msg import Float32MultiArrayStamped
    from autoware_internal_debug_msgs.msg import Float32Stamped
    from autoware_planning_msgs.msg import Trajectory
    from autoware_planning_msgs.msg import TrajectoryPoint
    from autoware_vehicle_msgs.msg import SteeringReport
    from geometry_msgs.msg import AccelWithCovarianceStamped
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    args.output_dir.mkdir(parents=True, exist_ok=True)
    log_path = args.output_dir / "smart_mpc.log"
    result_path = args.output_dir / "result.json"
    log_stream = log_path.open("w", encoding="utf-8")
    command = [
        "ros2",
        "run",
        "autoware_smart_mpc_trajectory_follower",
        "pympc_trajectory_follower.py",
        "--ros-args",
        "-r",
        "__node:=smart_mpc_synthetic_smoke",
    ]
    if args.runtime_param_file is not None:
        command.extend(["--params-file", str(args.runtime_param_file.resolve())])
    process = subprocess.Popen(
        command,
        stdout=log_stream,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )

    rclpy.init()
    node = Node("smart_mpc_synthetic_driver")
    trajectory_pub = node.create_publisher(Trajectory, "/planning/trajectory", 10)
    odometry_pub = node.create_publisher(Odometry, "/localization/kinematic_state", 10)
    acceleration_pub = node.create_publisher(
        AccelWithCovarianceStamped, "/localization/acceleration", 10
    )
    steering_pub = node.create_publisher(
        SteeringReport, "/vehicle/status/steering_status", 10
    )
    goal_pub = node.create_publisher(
        PoseStamped, "/planning/mission_planning/echo_back_goal_pose", 10
    )

    command_times = []
    commands = []
    predictions = []
    calculation_times = []
    total_times = []
    guard_active = []
    guard_stop_distances = []
    guard_floored_points = []
    guard_reasons = []
    desired_velocity_horizons = []
    node.create_subscription(
        Control,
        "/control/trajectory_follower/control_cmd",
        lambda message: (commands.append(message), command_times.append(time.monotonic())),
        10,
    )
    node.create_subscription(
        Trajectory,
        "/control/trajectory_follower/lateral/predicted_trajectory",
        predictions.append,
        10,
    )
    node.create_subscription(
        Float32Stamped,
        "/debug_mpc_calc_u_opt_time",
        lambda message: calculation_times.append(float(message.data)),
        10,
    )
    node.create_subscription(
        Float32Stamped,
        "/debug_mpc_total_ctrl_time",
        lambda message: total_times.append(float(message.data)),
        10,
    )
    node.create_subscription(
        BoolStamped,
        "/debug_mpc/reference_horizon_goal_stop_guard_active",
        lambda message: guard_active.append(bool(message.data)),
        10,
    )
    node.create_subscription(
        Float32Stamped,
        "/debug_mpc/reference_horizon_goal_stop_distance",
        lambda message: guard_stop_distances.append(float(message.data)),
        10,
    )
    node.create_subscription(
        Float32Stamped,
        "/debug_mpc/reference_horizon_goal_stop_floored_points",
        lambda message: guard_floored_points.append(int(message.data)),
        10,
    )
    node.create_subscription(
        String,
        "/debug_mpc/reference_horizon_goal_stop_guard_reason",
        lambda message: guard_reasons.append(message.data),
        10,
    )
    node.create_subscription(
        Float32MultiArrayStamped,
        "/debug_mpc_v_des",
        lambda message: desired_velocity_horizons.append([float(value) for value in message.data]),
        10,
    )

    trajectory = Trajectory()
    trajectory.header.frame_id = "map"
    if args.scenario == "straight":
        point_count = 121
        point_spacing_m = 0.25
        velocities = [2.0] * point_count
        odometry_x = 0.0
        odometry_velocity_mps = 0.0
        goal = None
        trajectory_name = "straight_30m_2mps"
    else:
        point_count = 9
        point_spacing_m = 0.5
        velocities = [2.5, 2.33, 2.06, 1.74, 1.35, 0.79, 0.0, 0.0, 0.0]
        odometry_x = 0.0 if args.scenario == "distant_goal_stop" else 1.5
        odometry_velocity_mps = 0.0
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x = 4.0
        goal.pose.orientation.w = 1.0
        trajectory_name = args.scenario

    for index in range(point_count):
        point = TrajectoryPoint()
        point.pose.position.x = index * point_spacing_m
        point.pose.orientation.w = 1.0
        point.longitudinal_velocity_mps = velocities[index]
        trajectory.points.append(point)
    odometry = Odometry()
    odometry.header.frame_id = "map"
    odometry.child_frame_id = "base_link"
    odometry.pose.pose.position.x = odometry_x
    odometry.pose.pose.orientation.w = 1.0
    odometry.twist.twist.linear.x = odometry_velocity_mps
    acceleration = AccelWithCovarianceStamped()
    acceleration.header.frame_id = "base_link"
    steering = SteeringReport()

    started = time.monotonic()
    first_command_time = None
    try:
        while time.monotonic() - started < args.timeout:
            if process.poll() is not None:
                raise RuntimeError(f"Smart MPC exited early with status {process.returncode}")

            stamp = node.get_clock().now().to_msg()
            trajectory.header.stamp = stamp
            odometry.header.stamp = stamp
            acceleration.header.stamp = stamp
            steering.stamp = stamp
            if goal is not None:
                goal.header.stamp = stamp
                goal_pub.publish(goal)
            acceleration_pub.publish(acceleration)
            steering_pub.publish(steering)
            trajectory_pub.publish(trajectory)
            odometry_pub.publish(odometry)
            for _ in range(4):
                rclpy.spin_once(node, timeout_sec=0.02)

            if command_times and first_command_time is None:
                first_command_time = command_times[0]
            if (
                first_command_time is not None
                and predictions
                and guard_active
                and guard_reasons
                and desired_velocity_horizons
                and time.monotonic() - first_command_time >= args.sample_seconds
            ):
                break
        else:
            raise TimeoutError("Timed out waiting for Smart MPC command and prediction output")

        intervals = [
            second - first for first, second in zip(command_times, command_times[1:])
        ]
        steady_calculation_times = [value for value in calculation_times if value < 1.0]
        steady_total_times = [value for value in total_times if value < 1.0]
        last_command = commands[-1]
        last_desired_velocity_horizon = desired_velocity_horizons[-1]
        finite_guard_distances = [
            value for value in guard_stop_distances if math.isfinite(value)
        ]
        observed_guard_active = bool(guard_active[-1])
        if args.expect_goal_stop_guard == "active" and not observed_guard_active:
            raise RuntimeError("Goal-stop guard did not become active")
        if args.expect_goal_stop_guard == "inactive" and observed_guard_active:
            raise RuntimeError("Goal-stop guard unexpectedly became active")
        result = {
            "status": "PASS",
            "ros_domain_id": args.domain_id,
            "profile": {
                "mode": "ilqr",
                "use_trained_model": False,
                "trajectory": trajectory_name,
                "runtime_param_file": (
                    None
                    if args.runtime_param_file is None
                    else str(args.runtime_param_file.resolve())
                ),
            },
            "startup_to_first_command_s": first_command_time - started,
            "command_interval_s": stats(intervals),
            "steady_calc_u_opt_s": stats(steady_calculation_times),
            "steady_total_control_s": stats(steady_total_times),
            "commands": len(commands),
            "predicted_trajectory_points": len(predictions[-1].points),
            "reference_horizon_goal_stop_guard": {
                "active": observed_guard_active,
                "active_samples": sum(guard_active),
                "samples": len(guard_active),
                "last_stop_distance_m": (
                    finite_guard_distances[-1] if finite_guard_distances else None
                ),
                "last_floored_points": (
                    guard_floored_points[-1] if guard_floored_points else None
                ),
                "last_reason": guard_reasons[-1],
                "last_v_des_min_mps": min(last_desired_velocity_horizon),
                "last_v_des_max_mps": max(last_desired_velocity_horizon),
            },
            "last_command": {
                "velocity_mps": float(last_command.longitudinal.velocity),
                "acceleration_mps2": float(last_command.longitudinal.acceleration),
                "steering_tire_angle_rad": float(last_command.lateral.steering_tire_angle),
                "is_defined_acceleration": bool(
                    last_command.longitudinal.is_defined_acceleration
                ),
            },
        }
        if not result["last_command"]["is_defined_acceleration"]:
            raise RuntimeError("Smart MPC acceleration command is marked undefined")
        result_path.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(result, indent=2))
        print(f"result: {result_path.resolve()}")
        return 0
    except Exception as error:
        result_path.write_text(
            json.dumps({"status": "FAIL", "error": str(error)}, indent=2) + "\n",
            encoding="utf-8",
        )
        print(f"Smart MPC smoke failed: {error}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
        log_stream.close()


if __name__ == "__main__":
    raise SystemExit(main())
