#!/usr/bin/env python3

import json
import math
from pathlib import Path

import rclpy
from autoware_planning_msgs.msg import LaneletRoute, RouteState
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


LIVE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def load_route_pose(route_file, key, fallback_index):
    payload = json.loads(Path(route_file).expanduser().read_text(encoding="utf-8"))
    pose = payload.get(key)
    if pose is None:
        point = payload["route"][fallback_index]
        pose = {name: point[name] for name in ("x", "y", "z", "yaw")}
    values = tuple(float(pose[name]) for name in ("x", "y", "z", "yaw"))
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f"{key} contains a non-finite value")
    return values


def quaternion_yaw(orientation):
    sin_yaw = 2.0 * (
        orientation.w * orientation.z + orientation.x * orientation.y
    )
    cos_yaw = 1.0 - 2.0 * (
        orientation.y * orientation.y + orientation.z * orientation.z
    )
    return math.atan2(sin_yaw, cos_yaw)


def angular_distance(first, second):
    return abs(math.atan2(math.sin(first - second), math.cos(first - second)))


def route_state_supports_alignment(state):
    return state in (RouteState.SET, RouteState.ARRIVED)


def route_alignment_metrics(message, start, goal):
    return {
        "start_distance_m": math.hypot(
            message.start_pose.position.x - start[0],
            message.start_pose.position.y - start[1],
        ),
        "goal_distance_m": math.hypot(
            message.goal_pose.position.x - goal[0],
            message.goal_pose.position.y - goal[1],
        ),
        "goal_yaw_error_rad": angular_distance(
            quaternion_yaw(message.goal_pose.orientation), goal[3]
        ),
        "segment_count": len(message.segments),
    }


class VadStandardRouteAdapter(Node):
    """Keep the standard Autoware route UI aligned with the VAD JSON route."""

    def __init__(self):
        super().__init__("vad_standard_route_adapter")
        route_file = self.declare_parameter("route_file", "").value
        if not route_file:
            raise RuntimeError("route_file parameter is required")
        self.start = load_route_pose(route_file, "start_ros_pose", 0)
        self.goal = load_route_pose(route_file, "goal_ros_pose", -1)
        self.start_tolerance_m = float(
            self.declare_parameter("start_alignment_tolerance_m", 2.5).value
        )
        self.goal_tolerance_m = float(
            self.declare_parameter("goal_alignment_tolerance_m", 1.5).value
        )
        self.goal_yaw_tolerance_rad = float(
            self.declare_parameter("goal_yaw_alignment_tolerance_rad", 0.35).value
        )
        tolerances = (
            ("start_alignment_tolerance_m", self.start_tolerance_m),
            ("goal_alignment_tolerance_m", self.goal_tolerance_m),
            ("goal_yaw_alignment_tolerance_rad", self.goal_yaw_tolerance_rad),
        )
        for name, value in tolerances:
            if not math.isfinite(value) or value <= 0.0:
                raise RuntimeError(f"{name} must be positive and finite")

        self.have_odometry = False
        self.route_state = RouteState.UNKNOWN
        self.route_geometry_aligned = False
        self.route_aligned = False
        self.publish_attempts = 0

        self.goal_pub = self.create_publisher(
            PoseStamped, "/planning/mission_planning/goal", LIVE_QOS
        )
        self.aligned_pub = self.create_publisher(
            Bool, "/planning/vad_route/standard_route_aligned", LATCHED_QOS
        )
        self.create_subscription(
            Odometry,
            "/localization/kinematic_state",
            self._on_odometry,
            LIVE_QOS,
        )
        self.create_subscription(
            RouteState, "/planning/route_state", self._on_route_state, LATCHED_QOS
        )
        self.create_subscription(
            LaneletRoute,
            "/planning/mission_planning/route",
            self._on_standard_route,
            LATCHED_QOS,
        )
        self.timer = self.create_timer(1.0, self._on_timer)
        self.get_logger().info(
            f"Standard route target loaded from {route_file}: "
            f"({self.goal[0]:.2f}, {self.goal[1]:.2f})"
        )

    def _on_odometry(self, _msg):
        self.have_odometry = True

    def _on_route_state(self, msg):
        self.route_state = msg.state
        aligned = self.route_geometry_aligned and route_state_supports_alignment(
            self.route_state
        )
        if self.route_aligned and not aligned:
            self.get_logger().warning(
                "Standard route is no longer active; closing the VAD route gate"
            )
        self.route_aligned = aligned
        self.aligned_pub.publish(Bool(data=aligned))

    def _on_standard_route(self, msg):
        metrics = route_alignment_metrics(msg, self.start, self.goal)
        geometry_aligned = (
            metrics["start_distance_m"] <= self.start_tolerance_m
            and metrics["goal_distance_m"] <= self.goal_tolerance_m
            and metrics["goal_yaw_error_rad"] <= self.goal_yaw_tolerance_rad
            and metrics["segment_count"] > 0
        )
        if geometry_aligned != self.route_geometry_aligned:
            if geometry_aligned:
                self.get_logger().info(
                    "Standard Autoware route aligned with the VAD JSON route "
                    f"(start={metrics['start_distance_m']:.2f} m, "
                    f"goal={metrics['goal_distance_m']:.2f} m, "
                    f"yaw={metrics['goal_yaw_error_rad']:.2f} rad, "
                    f"segments={metrics['segment_count']})"
                )
            else:
                self.get_logger().error(
                    "Standard route differs from the VAD JSON route "
                    f"(start={metrics['start_distance_m']:.2f} m, "
                    f"goal={metrics['goal_distance_m']:.2f} m, "
                    f"yaw={metrics['goal_yaw_error_rad']:.2f} rad, "
                    f"segments={metrics['segment_count']})"
                )
        self.route_geometry_aligned = geometry_aligned
        aligned = geometry_aligned and route_state_supports_alignment(self.route_state)
        self.route_aligned = aligned
        self.aligned_pub.publish(Bool(data=aligned))

    def _on_timer(self):
        self.aligned_pub.publish(Bool(data=self.route_aligned))
        if not self.have_odometry or self.route_aligned:
            return
        if self.route_state not in (RouteState.UNKNOWN, RouteState.UNSET):
            return

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = self.goal[0]
        goal.pose.position.y = self.goal[1]
        goal.pose.position.z = self.goal[2]
        goal.pose.orientation.z = math.sin(self.goal[3] * 0.5)
        goal.pose.orientation.w = math.cos(self.goal[3] * 0.5)
        self.goal_pub.publish(goal)
        self.publish_attempts += 1
        if self.publish_attempts == 1 or self.publish_attempts % 10 == 0:
            self.get_logger().info(
                f"Requesting standard mission route for the VAD goal "
                f"(attempt {self.publish_attempts})"
            )


def main():
    rclpy.init()
    node = VadStandardRouteAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
