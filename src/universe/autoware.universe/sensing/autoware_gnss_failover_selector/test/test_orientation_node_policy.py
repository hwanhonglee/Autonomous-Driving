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

"""Node-level policy tests for HEADING2 and INSPVAX orientation output."""

from novatel_oem7_msgs.msg import HEADING2
from novatel_oem7_msgs.msg import INSPVAX
import rclpy

from autoware_gnss_failover_selector.novatel_orientation_node import (
    NovatelInspvaxOrientationNode,
)
from autoware_gnss_failover_selector.orientation_logic import OrientationSource


class CapturePublisher:
    """Capture published messages without placing them on DDS."""

    def __init__(self) -> None:
        self.messages = []

    def publish(self, msg) -> None:
        self.messages.append(msg)


def make_heading2(node: NovatelInspvaxOrientationNode) -> HEADING2:
    msg = HEADING2()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.sol_status.status = 0
    msg.pos_type.type = 50
    msg.length = 1.2
    msg.heading = 45.0
    msg.pitch = 1.0
    msg.heading_stdev = 0.2
    msg.pitch_stdev = 0.3
    return msg


def make_inspvax(
    node: NovatelInspvaxOrientationNode, status: int, yaw_stdev: float
) -> INSPVAX:
    msg = INSPVAX()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.ins_status.status = status
    msg.roll = 1.0
    msg.pitch = -2.0
    msg.azimuth = 45.0
    msg.roll_stdev = 0.2
    msg.pitch_stdev = 0.3
    msg.azimuth_stdev = yaw_stdev
    return msg


# HH_260811 - Prove valid HEADING2 and WAITING_AZIMUTH cannot publish body attitude.
def test_only_solution_good_inspvax_publishes_orientation() -> None:
    rclpy.init(domain_id=232)
    node = NovatelInspvaxOrientationNode()
    capture = CapturePublisher()
    node._orientation_pub = capture
    try:
        node._on_heading2(make_heading2(node))
        assert node._published_heading2_valid
        assert node._published_source == OrientationSource.NONE
        assert capture.messages == []

        node._on_inspvax(make_inspvax(node, status=10, yaw_stdev=180.0))
        assert node._published_source == OrientationSource.NONE
        assert capture.messages == []

        node._on_inspvax(make_inspvax(node, status=3, yaw_stdev=0.4))
        assert node._published_source == OrientationSource.INSPVAX
        assert len(capture.messages) == 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
