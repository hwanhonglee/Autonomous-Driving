#!/usr/bin/env python3

from geometry_msgs.msg import AccelWithCovarianceStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def imu_to_acceleration(message, output_frame_id="base_link"):
    """Convert an aligned CARLA IMU sample without filtering or gravity removal."""
    output = AccelWithCovarianceStamped()
    output.header.stamp = message.header.stamp
    output.header.frame_id = output_frame_id or message.header.frame_id

    output.accel.accel.linear.x = message.linear_acceleration.x
    output.accel.accel.linear.y = message.linear_acceleration.y
    output.accel.accel.linear.z = message.linear_acceleration.z

    # AccelWithCovariance uses a 6x6 [linear, angular] layout. Preserve the
    # available 3x3 linear-acceleration covariance in its upper-left block.
    for row in range(3):
        for column in range(3):
            output.accel.covariance[row * 6 + column] = (
                message.linear_acceleration_covariance[row * 3 + column]
            )

    return output


class VadImuAccelerationAdapter(Node):
    """Expose CARLA's gravity-inclusive IMU acceleration to VAD only."""

    def __init__(self):
        super().__init__("vad_imu_acceleration_adapter")
        output_frame_id = self.declare_parameter("output_frame_id", "base_link").value
        self._output_frame_id = str(output_frame_id)
        self._publisher = self.create_publisher(
            AccelWithCovarianceStamped,
            "~/output/acceleration",
            qos_profile_sensor_data,
        )
        self._subscription = self.create_subscription(
            Imu,
            "~/input/imu",
            self._on_imu,
            qos_profile_sensor_data,
        )

    def _on_imu(self, message):
        self._publisher.publish(
            imu_to_acceleration(message, output_frame_id=self._output_frame_id)
        )


def main():
    rclpy.init()
    node = VadImuAccelerationAdapter()
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
