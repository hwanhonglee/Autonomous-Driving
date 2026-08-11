# Copyright 2026 Hwanhong Lee
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""ROS 2 node selecting a primary or fallback NavSatFix source."""

# HH_260811 - Publish a debounced NovAtel-first position selection with diagnostics.

import math
import time
from typing import Dict

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

from .selector_logic import FailoverSelector
from .selector_logic import is_fix_valid
from .selector_logic import SelectorConfig
from .selector_logic import Source


class GnssFailoverSelectorNode(Node):
    """Forward valid fixes from NovAtel unless primary input becomes unavailable."""

    def __init__(self) -> None:
        super().__init__("gnss_failover_selector")
        self._minimum_status = int(self.declare_parameter("minimum_navsat_status", 0).value)
        self._require_known_covariance = bool(
            self.declare_parameter("require_known_covariance", False).value
        )
        max_variance = float(
            self.declare_parameter("max_horizontal_variance_m2", -1.0).value
        )
        self._max_horizontal_variance_m2 = (
            None if max_variance < 0.0 else max_variance
        )
        evaluation_period = float(
            self.declare_parameter("evaluation_period_sec", 0.1).value
        )
        diagnostic_period = float(
            self.declare_parameter("diagnostic_period_sec", 1.0).value
        )
        if self._minimum_status < 0 or self._minimum_status > 2:
            raise ValueError("minimum_navsat_status must be in [0, 2]")
        if not math.isfinite(evaluation_period) or evaluation_period <= 0.0:
            raise ValueError("evaluation_period_sec must be finite and positive")
        if not math.isfinite(diagnostic_period) or diagnostic_period <= 0.0:
            raise ValueError("diagnostic_period_sec must be finite and positive")

        config = SelectorConfig(
            primary_timeout_sec=float(
                self.declare_parameter("primary_timeout_sec", 1.0).value
            ),
            fallback_timeout_sec=float(
                self.declare_parameter("fallback_timeout_sec", 2.0).value
            ),
            primary_failure_hold_sec=float(
                self.declare_parameter("primary_failure_hold_sec", 1.0).value
            ),
            fallback_activation_hold_sec=float(
                self.declare_parameter("fallback_activation_hold_sec", 1.0).value
            ),
            primary_recovery_hold_sec=float(
                self.declare_parameter("primary_recovery_hold_sec", 5.0).value
            ),
            fallback_failure_hold_sec=float(
                self.declare_parameter("fallback_failure_hold_sec", 1.0).value
            ),
            primary_initial_hold_sec=float(
                self.declare_parameter("primary_initial_hold_sec", 0.0).value
            ),
        )
        now = time.monotonic()
        self._selector = FailoverSelector(config, now=now)
        self._latest: Dict[Source, NavSatFix] = {}

        input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._fix_pub = self.create_publisher(NavSatFix, "~/output/fix", output_qos)
        self._source_pub = self.create_publisher(
            String, "~/selected_source", state_qos
        )
        self._diagnostic_pub = self.create_publisher(
            DiagnosticArray, "/diagnostics", output_qos
        )
        self._primary_sub = self.create_subscription(
            NavSatFix,
            "~/input/primary",
            lambda msg: self._on_fix(Source.PRIMARY, msg),
            input_qos,
        )
        self._fallback_sub = self.create_subscription(
            NavSatFix,
            "~/input/fallback",
            lambda msg: self._on_fix(Source.FALLBACK, msg),
            input_qos,
        )
        self._evaluation_timer = self.create_timer(
            evaluation_period, self._on_evaluation_timer
        )
        self._diagnostic_timer = self.create_timer(
            diagnostic_period, self._publish_diagnostic
        )
        self._publish_source()

    def _valid_fix(self, msg: NavSatFix) -> bool:
        return is_fix_valid(
            status=int(msg.status.status),
            latitude=float(msg.latitude),
            longitude=float(msg.longitude),
            altitude=float(msg.altitude),
            covariance_type=int(msg.position_covariance_type),
            covariance=msg.position_covariance,
            minimum_status=self._minimum_status,
            require_known_covariance=self._require_known_covariance,
            max_horizontal_variance_m2=self._max_horizontal_variance_m2,
        )

    def _on_fix(self, source: Source, msg: NavSatFix) -> None:
        now = time.monotonic()
        valid = self._valid_fix(msg)
        self._latest[source] = msg
        self._selector.observe(source, valid, now)
        changed = self._evaluate(now)

        if self._selector.selected == source and valid:
            self._fix_pub.publish(msg)
        elif changed:
            self._publish_selected_latest(now)

    def _evaluate(self, now: float) -> bool:
        previous = self._selector.selected
        selected = self._selector.evaluate(now)
        if selected == previous:
            return False
        self._publish_source()
        if selected == Source.FALLBACK:
            self.get_logger().warning("GNSS selected source changed to fallback")
        elif selected == Source.PRIMARY:
            self.get_logger().info("GNSS selected source changed to primary")
        else:
            self.get_logger().error("No healthy GNSS source is selected")
        return True

    def _on_evaluation_timer(self) -> None:
        now = time.monotonic()
        if self._evaluate(now):
            self._publish_selected_latest(now)

    def _publish_selected_latest(self, now: float) -> None:
        source = self._selector.selected
        if source == Source.PRIMARY:
            healthy = self._selector.primary.is_healthy(
                now, self._selector.config.primary_timeout_sec
            )
        elif source == Source.FALLBACK:
            healthy = self._selector.fallback.is_healthy(
                now, self._selector.config.fallback_timeout_sec
            )
        else:
            healthy = False
        if healthy and source in self._latest:
            self._fix_pub.publish(self._latest[source])

    def _publish_source(self) -> None:
        self._source_pub.publish(String(data=self._selector.selected.value))

    @staticmethod
    def _age_text(age: float) -> str:
        return "never" if age is None else f"{age:.3f}"

    def _publish_diagnostic(self) -> None:
        now = time.monotonic()
        if self._evaluate(now):
            self._publish_selected_latest(now)
        primary_healthy = self._selector.primary.is_healthy(
            now, self._selector.config.primary_timeout_sec
        )
        fallback_healthy = self._selector.fallback.is_healthy(
            now, self._selector.config.fallback_timeout_sec
        )
        selected = self._selector.selected
        selected_healthy = (
            primary_healthy
            if selected == Source.PRIMARY
            else fallback_healthy if selected == Source.FALLBACK else False
        )

        if selected == Source.PRIMARY and selected_healthy:
            level = DiagnosticStatus.OK
            message = "NovAtel primary selected"
        elif selected == Source.FALLBACK and selected_healthy:
            level = DiagnosticStatus.WARN
            message = "u-blox fallback selected"
        else:
            level = DiagnosticStatus.ERROR
            message = "No selected GNSS source is currently healthy"

        status = DiagnosticStatus(
            level=level,
            name=f"{self.get_fully_qualified_name()}: GNSS failover",
            message=message,
            hardware_id="novatel_primary_ublox_fallback",
            values=[
                KeyValue(key="selected_source", value=selected.value),
                KeyValue(key="primary_healthy", value=str(primary_healthy).lower()),
                KeyValue(key="fallback_healthy", value=str(fallback_healthy).lower()),
                KeyValue(
                    key="primary_age_sec",
                    value=self._age_text(self._selector.primary.age(now)),
                ),
                KeyValue(
                    key="fallback_age_sec",
                    value=self._age_text(self._selector.fallback.age(now)),
                ),
            ],
        )
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self._diagnostic_pub.publish(array)


def main(args=None) -> None:
    """Run the GNSS failover selector node."""
    rclpy.init(args=args)
    node = GnssFailoverSelectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # A launch-level Ctrl+C can reach the process both through its process
        # group and through launch's propagated SIGINT. Keep cleanup idempotent.
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == "__main__":
    main()
