#!/usr/bin/env python3

import time

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
REQUEST_TIMEOUT_SEC = 5.0
REAPPLY_PERIOD_SEC = 2.0


def aeb_parameter_overrides():
    return (
        ("use_pointcloud_data", False),
        ("use_predicted_object_data", True),
    )


def bool_parameter(name, value):
    return Parameter(
        name=name,
        value=ParameterValue(
            type=ParameterType.PARAMETER_BOOL,
            bool_value=bool(value),
        ),
    )


class VadAebConfigurator(Node):
    """Switch standard AEB to the VAD PredictedObjects input in simulation."""

    def __init__(self):
        super().__init__("vad_aeb_configurator")
        service_name = self.declare_parameter(
            "service_name",
            "/control/autonomous_emergency_braking/set_parameters_atomically",
        ).value
        self.client = self.create_client(SetParametersAtomically, service_name)
        self.configured_pub = self.create_publisher(
            Bool, "/system/vad/aeb_configured", LATCHED_QOS
        )
        self.future = None
        self.future_started_at = None
        self.last_success_at = None
        self.configured = False
        self.timer = self.create_timer(0.5, self._on_timer)
        self.configured_pub.publish(Bool(data=False))
        self.get_logger().info(f"Waiting for the standard AEB parameter service: {service_name}")

    def _on_timer(self):
        now = time.monotonic()
        if self.future is not None:
            if now - self.future_started_at > REQUEST_TIMEOUT_SEC:
                self.get_logger().error("AEB parameter request timed out; retrying")
                self.future.cancel()
                self.future = None
                self.future_started_at = None
                self._publish_configured(False)
            return
        if not self.client.service_is_ready():
            self._publish_configured(False)
            return
        if self.last_success_at is not None and now - self.last_success_at < REAPPLY_PERIOD_SEC:
            return
        request = SetParametersAtomically.Request()
        request.parameters = [
            bool_parameter(name, value) for name, value in aeb_parameter_overrides()
        ]
        self.future = self.client.call_async(request)
        self.future_started_at = now
        self.future.add_done_callback(self._on_response)

    def _on_response(self, future):
        if future is not self.future:
            return
        self.future = None
        self.future_started_at = None
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(f"AEB parameter request failed: {error}")
            self._publish_configured(False)
            return
        if response is None or not response.result.successful:
            reason = "no response" if response is None else response.result.reason
            self.get_logger().error(f"AEB rejected VAD object configuration: {reason}")
            self._publish_configured(False)
            return

        first_success = not self.configured
        self.last_success_at = time.monotonic()
        self._publish_configured(True)
        if first_success:
            self.get_logger().info(
                "Standard AEB configured for VAD PredictedObjects; pointcloud input disabled"
            )

    def _publish_configured(self, configured):
        configured = bool(configured)
        if self.configured and not configured:
            self.get_logger().error(
                "Standard AEB parameter service was lost; closing readiness gate"
            )
        self.configured = configured
        self.configured_pub.publish(Bool(data=configured))


def main():
    rclpy.init()
    node = VadAebConfigurator()
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
