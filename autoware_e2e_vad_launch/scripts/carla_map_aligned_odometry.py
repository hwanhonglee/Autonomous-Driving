#!/usr/bin/env python3

from copy import deepcopy
import math

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def rotate_pose_covariance(covariance, yaw_rad):
    """Rotate a 6x6 [xyz, rotation-vector] pose covariance about z."""
    if len(covariance) != 36:
        raise ValueError("pose covariance must contain exactly 36 values")

    cosine = math.cos(yaw_rad)
    sine = math.sin(yaw_rad)
    rotation = [
        [cosine, -sine, 0.0, 0.0, 0.0, 0.0],
        [sine, cosine, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, cosine, -sine, 0.0],
        [0.0, 0.0, 0.0, sine, cosine, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    ]
    source = [list(covariance[row * 6 : (row + 1) * 6]) for row in range(6)]
    left_product = [
        [
            sum(rotation[row][index] * source[index][column] for index in range(6))
            for column in range(6)
        ]
        for row in range(6)
    ]
    rotated = [
        [
            sum(
                left_product[row][index] * rotation[column][index]
                for index in range(6)
            )
            for column in range(6)
        ]
        for row in range(6)
    ]
    return [rotated[row][column] for row in range(6) for column in range(6)]


def multiply_quaternions(left, right):
    x = left[3] * right[0] + left[0] * right[3] + left[1] * right[2] - left[2] * right[1]
    y = left[3] * right[1] - left[0] * right[2] + left[1] * right[3] + left[2] * right[0]
    z = left[3] * right[2] + left[0] * right[1] - left[1] * right[0] + left[2] * right[3]
    w = left[3] * right[3] - left[0] * right[0] - left[1] * right[1] - left[2] * right[2]
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        raise ValueError("input odometry contains an invalid zero-norm orientation")
    return x / norm, y / norm, z / norm, w / norm


def transform_odometry(
    message,
    translation_x_m,
    translation_y_m,
    translation_z_m,
    yaw_rad,
    output_frame_id="map",
    output_child_frame_id="base_link",
):
    """Apply map_T_carla to an odometry pose while preserving child-frame twist."""
    values = (translation_x_m, translation_y_m, translation_z_m, yaw_rad)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("map alignment values must be finite")

    cosine = math.cos(yaw_rad)
    sine = math.sin(yaw_rad)
    source_position = message.pose.pose.position
    source_orientation = message.pose.pose.orientation

    output = Odometry()
    output.header.stamp = message.header.stamp
    output.header.frame_id = output_frame_id
    output.child_frame_id = output_child_frame_id
    output.pose.pose.position.x = (
        translation_x_m + cosine * source_position.x - sine * source_position.y
    )
    output.pose.pose.position.y = (
        translation_y_m + sine * source_position.x + cosine * source_position.y
    )
    output.pose.pose.position.z = translation_z_m + source_position.z

    alignment_quaternion = (0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0))
    source_quaternion = (
        source_orientation.x,
        source_orientation.y,
        source_orientation.z,
        source_orientation.w,
    )
    (
        output.pose.pose.orientation.x,
        output.pose.pose.orientation.y,
        output.pose.pose.orientation.z,
        output.pose.pose.orientation.w,
    ) = multiply_quaternions(alignment_quaternion, source_quaternion)
    output.pose.covariance = rotate_pose_covariance(message.pose.covariance, yaw_rad)

    # nav_msgs/Odometry defines twist in child_frame_id. A world-frame basis
    # change therefore rotates the pose covariance, but not the child-frame twist.
    output.twist = deepcopy(message.twist)
    return output


def odometry_to_transform(message):
    transform = TransformStamped()
    transform.header = deepcopy(message.header)
    transform.child_frame_id = message.child_frame_id
    transform.transform.translation.x = message.pose.pose.position.x
    transform.transform.translation.y = message.pose.pose.position.y
    transform.transform.translation.z = message.pose.pose.position.z
    transform.transform.rotation = deepcopy(message.pose.pose.orientation)
    return transform


class CarlaMapAlignedOdometry(Node):
    """Publish CARLA truth state in an explicitly aligned Autoware map frame."""

    def __init__(self):
        super().__init__("carla_map_aligned_odometry")
        numeric_descriptor = ParameterDescriptor(dynamic_typing=True)
        self._translation_x_m = float(
            self.declare_parameter("translation_x_m", 0.0, numeric_descriptor).value
        )
        self._translation_y_m = float(
            self.declare_parameter("translation_y_m", 0.0, numeric_descriptor).value
        )
        self._translation_z_m = float(
            self.declare_parameter("translation_z_m", 0.0, numeric_descriptor).value
        )
        self._yaw_rad = float(
            self.declare_parameter("yaw_rad", 0.0, numeric_descriptor).value
        )
        self._output_frame_id = str(self.declare_parameter("output_frame_id", "map").value)
        self._output_child_frame_id = str(
            self.declare_parameter("output_child_frame_id", "base_link").value
        )
        self._publish_tf = bool(self.declare_parameter("publish_tf", True).value)

        if not all(
            math.isfinite(value)
            for value in (
                self._translation_x_m,
                self._translation_y_m,
                self._translation_z_m,
                self._yaw_rad,
            )
        ):
            raise ValueError("map alignment values must be finite")
        if not self._output_frame_id or not self._output_child_frame_id:
            raise ValueError("output frame IDs must not be empty")

        self._publisher = self.create_publisher(Odometry, "~/output/odometry", 10)
        self._subscription = self.create_subscription(
            Odometry,
            "~/input/odometry",
            self._on_odometry,
            10,
        )
        self._tf_broadcaster = TransformBroadcaster(self) if self._publish_tf else None

        self.get_logger().info(
            "CARLA map alignment enabled: x=%.6f m, y=%.6f m, z=%.6f m, yaw=%.9f rad"
            % (
                self._translation_x_m,
                self._translation_y_m,
                self._translation_z_m,
                self._yaw_rad,
            )
        )

    def _on_odometry(self, message):
        try:
            output = transform_odometry(
                message,
                self._translation_x_m,
                self._translation_y_m,
                self._translation_z_m,
                self._yaw_rad,
                output_frame_id=self._output_frame_id,
                output_child_frame_id=self._output_child_frame_id,
            )
        except ValueError as error:
            self.get_logger().error(str(error))
            return

        self._publisher.publish(output)
        if self._tf_broadcaster is not None:
            self._tf_broadcaster.sendTransform(odometry_to_transform(output))


def main():
    rclpy.init()
    node = CarlaMapAlignedOdometry()
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
