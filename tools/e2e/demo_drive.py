#!/usr/bin/env python3

import argparse
import math
import signal
import time

import rclpy
from autoware_vehicle_msgs.msg import Engage as EngageState
from autoware_vehicle_msgs.msg import VelocityReport
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from tier4_external_api_msgs.srv import Engage as EngageService


class DriveMonitor(Node):
    def __init__(self):
        super().__init__("autoware_e2e_demo_drive")
        self.position = None
        self.sim_time = None
        self.velocity = 0.0
        self.engage_state = None
        self.stop_signal = None
        self.create_subscription(Odometry, "/localization/kinematic_state", self._on_odom, 10)
        self.create_subscription(
            VelocityReport, "/vehicle/status/velocity_status", self._on_velocity, 10
        )
        self.create_subscription(EngageState, "/api/autoware/get/engage", self._on_engage, 10)
        self.engage_client = self.create_client(
            EngageService, "/vehicle_cmd_gate/service/engage"
        )

    def _on_odom(self, msg):
        self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.sim_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _on_velocity(self, msg):
        self.velocity = msg.longitudinal_velocity

    def _on_engage(self, msg):
        self.engage_state = msg.engage

    def wait_for_odom(self, timeout):
        deadline = time.monotonic() + timeout
        while (
            rclpy.ok()
            and self.position is None
            and self.stop_signal is None
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(self, timeout_sec=0.2)
        return self.position is not None

    def wait_for_engage_state(self, expected, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.engage_state is expected:
                return True
        return False

    def set_engage(self, engage, timeout=30.0):
        if not self.engage_client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("engage service is not available")
        request = EngageService.Request()
        request.engage = engage
        future = self.engage_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            raise RuntimeError("engage service timed out")
        status = future.result().status
        if status.code not in (status.SUCCESS, status.IGNORED, status.WARN):
            raise RuntimeError(f"engage service failed: {status.code} {status.message}")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Drive a short measured CARLA distance and always disengage afterward."
    )
    parser.add_argument(
        "distance", nargs="?", type=float, default=10.0, help="maximum travel distance in meters"
    )
    parser.add_argument(
        "--sim-timeout", type=float, default=10.0, help="maximum simulated seconds"
    )
    parser.add_argument(
        "--wall-timeout", type=float, default=180.0, help="maximum wall-clock seconds"
    )
    args = parser.parse_args()
    if args.distance <= 0 or args.sim_timeout <= 0 or args.wall_timeout <= 0:
        parser.error("all limits must be positive")
    return args


def main():
    args = parse_args()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = DriveMonitor()
    cleanup_needed = False
    exit_code = 0

    def request_stop(signum, _frame):
        node.stop_signal = signum

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        if not node.wait_for_odom(30.0):
            if node.stop_signal is not None:
                raise RuntimeError("stopped before localization odometry was received")
            raise RuntimeError("no localization odometry received")

        start_position = node.position
        start_sim_time = node.sim_time
        previous_position = node.position
        distance = 0.0
        peak_speed = 0.0
        wall_start = time.monotonic()
        last_report = wall_start - 5.0

        cleanup_needed = True
        node.set_engage(True)
        if not node.wait_for_engage_state(True, 10.0):
            raise RuntimeError("engage state did not become true")
        print(
            f"Autoware engaged: limit={args.distance:.1f} m, "
            f"sim_timeout={args.sim_timeout:.1f} s",
            flush=True,
        )

        reason = "wall-clock timeout"
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            now = time.monotonic()
            peak_speed = max(peak_speed, abs(node.velocity))
            if node.position is not None and previous_position is not None:
                step = math.hypot(
                    node.position[0] - previous_position[0],
                    node.position[1] - previous_position[1],
                )
                if step < 5.0:
                    distance += step
                previous_position = node.position

            sim_elapsed = max(0.0, (node.sim_time or start_sim_time) - start_sim_time)
            if now - last_report >= 3.0:
                print(
                    f"distance={distance:.2f} m sim={sim_elapsed:.2f} s "
                    f"speed={node.velocity:.2f} m/s engage={node.engage_state}",
                    flush=True,
                )
                last_report = now

            if node.stop_signal is not None:
                reason = f"signal {signal.Signals(node.stop_signal).name}"
                exit_code = 128 + node.stop_signal
                break
            if distance >= args.distance:
                reason = "distance limit"
                break
            if sim_elapsed >= args.sim_timeout:
                reason = "simulation-time limit"
                break
            if now - wall_start >= args.wall_timeout:
                break

        if distance < 0.25 and exit_code == 0:
            exit_code = 1
        end_position = node.position or start_position
        print(
            f"Stopping on {reason}: traveled={distance:.2f} m, "
            f"peak_speed={peak_speed:.2f} m/s, "
            f"start=({start_position[0]:.2f}, {start_position[1]:.2f}), "
            f"end=({end_position[0]:.2f}, {end_position[1]:.2f})",
            flush=True,
        )
    except (KeyboardInterrupt, RuntimeError) as error:
        print(f"Demo drive stopped: {error}", flush=True)
        exit_code = 1
    finally:
        if cleanup_needed:
            try:
                if not rclpy.ok():
                    raise RuntimeError("ROS context stopped before disengage")
                node.set_engage(False)
                if not node.wait_for_engage_state(False, 10.0):
                    raise RuntimeError("disengage state did not become false")
                print("Autoware control disengaged.", flush=True)
            except Exception as error:
                print(f"WARNING: failed to disengage: {error}", flush=True)
                exit_code = 1
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
