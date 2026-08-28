#!/usr/bin/env python3

import rclpy
from autoware_system_msgs.msg import AutowareState
from autoware_vehicle_msgs.msg import Engage
from rclpy.node import Node


class CarlaAutowareStatePublisher(Node):
    def __init__(self):
        super().__init__("carla_autoware_state_publisher")
        self.engaged = False
        self.publisher = self.create_publisher(AutowareState, "/autoware/state", 1)
        self.create_subscription(Engage, "/api/autoware/get/engage", self.on_engage, 1)
        self.timer = self.create_timer(0.1, self.publish_state)

    def on_engage(self, msg):
        self.engaged = msg.engage
        self.publish_state()

    def publish_state(self):
        msg = AutowareState()
        msg.stamp = self.get_clock().now().to_msg()
        msg.state = AutowareState.DRIVING if self.engaged else AutowareState.WAITING_FOR_ENGAGE
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = CarlaAutowareStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
