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

"""Fail-closed NovAtel INSPVAX orientation with HEADING2 readiness diagnostics."""

# HH_260811 - Publish only validated complete INS attitude and keep HEADING2 diagnostic-only.

import math
import time

from autoware_sensing_msgs.msg import GnssInsOrientationStamped
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue
from novatel_oem7_msgs.msg import HEADING2
from novatel_oem7_msgs.msg import INSPVAX
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import Bool
from std_msgs.msg import String

from .orientation_logic import decode_inspvax_alignment
from .orientation_logic import Heading2Config
from .orientation_logic import INS_SOLUTION_GOOD
from .orientation_logic import is_heading2_orientation_valid
from .orientation_logic import is_inspvax_orientation_valid
from .orientation_logic import NARROW_INT
from .orientation_logic import OrientationConfig
from .orientation_logic import OrientationSource
from .orientation_logic import quaternion_from_novatel_attitude
from .orientation_logic import rmse_radians
from .orientation_logic import SOLUTION_COMPUTED


class NovatelInspvaxOrientationNode(Node):
    """Publish only a complete, fresh, uncertainty-gated OEM7 INS attitude."""

    def __init__(self) -> None:
        super().__init__("novatel_inspvax_orientation")
        self._output_frame = str(self.declare_parameter("output_frame", "imu_link").value)
        self._max_input_age_sec = float(
            self.declare_parameter("max_input_age_sec", 2.5).value
        )
        # HH_260811 - Track future dual-antenna HEADING2 freshness for readiness only.
        self._max_heading2_age_sec = float(
            self.declare_parameter("max_heading2_age_sec", 1.0).value
        )
        self._max_future_stamp_sec = float(
            self.declare_parameter("max_future_stamp_sec", 0.5).value
        )
        diagnostic_period = float(
            self.declare_parameter("diagnostic_period_sec", 1.0).value
        )
        self._config = OrientationConfig(
            required_ins_status=int(
                self.declare_parameter(
                    "required_ins_status", INS_SOLUTION_GOOD
                ).value
            ),
            max_yaw_rmse_deg=float(
                self.declare_parameter("max_yaw_rmse_deg", 5.0).value
            ),
            max_roll_pitch_rmse_deg=float(
                self.declare_parameter("max_roll_pitch_rmse_deg", 10.0).value
            ),
        )
        allowed_position_types = tuple(
            int(value)
            for value in self.declare_parameter(
                "dual_heading_allowed_position_types", [NARROW_INT]
            ).value
        )
        self._heading2_config = Heading2Config(
            required_solution_status=int(
                self.declare_parameter(
                    "dual_heading_required_solution_status", SOLUTION_COMPUTED
                ).value
            ),
            allowed_position_types=allowed_position_types,
            max_heading_rmse_deg=float(
                self.declare_parameter("max_dual_heading_rmse_deg", 5.0).value
            ),
            max_pitch_rmse_deg=float(
                self.declare_parameter("max_dual_pitch_rmse_deg", 10.0).value
            ),
        )
        positive = (
            self._max_input_age_sec,
            self._max_heading2_age_sec,
            diagnostic_period,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError("Input age and diagnostic periods must be finite and positive")
        if (
            not math.isfinite(self._max_future_stamp_sec)
            or self._max_future_stamp_sec < 0.0
        ):
            raise ValueError("max_future_stamp_sec must be finite and nonnegative")

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
        self._orientation_pub = self.create_publisher(
            GnssInsOrientationStamped, "~/output/orientation", output_qos
        )
        self._valid_pub = self.create_publisher(Bool, "~/valid", state_qos)
        # HH_260811 - Expose the actual output source separately from dual readiness.
        self._source_pub = self.create_publisher(
            String, "~/selected_source", state_qos
        )
        self._heading2_valid_pub = self.create_publisher(
            Bool, "~/heading2_valid", state_qos
        )
        self._diagnostic_pub = self.create_publisher(
            DiagnosticArray, "/diagnostics", output_qos
        )
        self._inspvax_sub = self.create_subscription(
            INSPVAX, "~/input/inspvax", self._on_inspvax, input_qos
        )
        # HH_260811 - Observe HEADING2 without converting it directly into body attitude.
        self._heading2_sub = self.create_subscription(
            HEADING2, "~/input/heading2", self._on_heading2, input_qos
        )
        self._diagnostic_timer = self.create_timer(
            diagnostic_period, self._publish_diagnostic
        )

        self._last_inspvax = None
        self._last_inspvax_arrival = None
        self._last_inspvax_fields_valid = False
        self._last_heading2 = None
        self._last_heading2_arrival = None
        self._last_heading2_fields_valid = False
        self._orientation_reason = "No INSPVAX received"
        self._heading2_reason = "No HEADING2 received (optional dual antenna)"
        self._published_valid_state = None
        self._published_source = None
        self._published_heading2_valid = None
        self._update_orientation_state(False)
        self._update_heading2_state(False)

    @staticmethod
    def _stamp_ns(msg) -> int:
        return int(msg.header.stamp.sec) * 1_000_000_000 + int(
            msg.header.stamp.nanosec
        )

    def _header_age(self, msg) -> float:
        stamp_ns = self._stamp_ns(msg)
        if stamp_ns <= 0:
            return math.inf
        return (self.get_clock().now().nanoseconds - stamp_ns) / 1.0e9

    def _message_fresh(self, msg, arrival, timeout_sec: float, now: float) -> bool:
        if msg is None or arrival is None or now - arrival > timeout_sec:
            return False
        header_age = self._header_age(msg)
        return -self._max_future_stamp_sec <= header_age <= timeout_sec

    def _orientation_currently_valid(self, now: float) -> bool:
        return self._last_inspvax_fields_valid and self._message_fresh(
            self._last_inspvax,
            self._last_inspvax_arrival,
            self._max_input_age_sec,
            now,
        )

    def _heading2_currently_valid(self, now: float) -> bool:
        return self._last_heading2_fields_valid and self._message_fresh(
            self._last_heading2,
            self._last_heading2_arrival,
            self._max_heading2_age_sec,
            now,
        )

    def _on_inspvax(self, msg: INSPVAX) -> None:
        self._last_inspvax = msg
        self._last_inspvax_arrival = time.monotonic()
        self._last_inspvax_fields_valid = is_inspvax_orientation_valid(
            ins_status=int(msg.ins_status.status),
            roll_deg=float(msg.roll),
            pitch_deg=float(msg.pitch),
            azimuth_deg=float(msg.azimuth),
            roll_stdev_deg=float(msg.roll_stdev),
            pitch_stdev_deg=float(msg.pitch_stdev),
            azimuth_stdev_deg=float(msg.azimuth_stdev),
            config=self._config,
        )
        valid = self._orientation_currently_valid(time.monotonic())
        self._orientation_reason = self._describe_orientation(valid)
        self._update_orientation_state(valid)
        if not valid:
            return

        try:
            quaternion = quaternion_from_novatel_attitude(
                float(msg.roll), float(msg.pitch), float(msg.azimuth)
            )
            rmse = rmse_radians(
                float(msg.roll_stdev),
                float(msg.pitch_stdev),
                float(msg.azimuth_stdev),
            )
        except (TypeError, ValueError):
            self._orientation_reason = "INSPVAX could not be converted safely"
            self._update_orientation_state(False)
            return

        output = GnssInsOrientationStamped()
        output.header.stamp = msg.header.stamp
        output.header.frame_id = self._output_frame or msg.header.frame_id
        output.orientation.orientation.x = quaternion[0]
        output.orientation.orientation.y = quaternion[1]
        output.orientation.orientation.z = quaternion[2]
        output.orientation.orientation.w = quaternion[3]
        output.orientation.rmse_rotation_x = rmse[0]
        output.orientation.rmse_rotation_y = rmse[1]
        output.orientation.rmse_rotation_z = rmse[2]
        self._orientation_pub.publish(output)

    def _on_heading2(self, msg: HEADING2) -> None:
        self._last_heading2 = msg
        self._last_heading2_arrival = time.monotonic()
        self._last_heading2_fields_valid = is_heading2_orientation_valid(
            solution_status=int(msg.sol_status.status),
            position_type=int(msg.pos_type.type),
            baseline_length_m=float(msg.length),
            heading_deg=float(msg.heading),
            pitch_deg=float(msg.pitch),
            heading_stdev_deg=float(msg.heading_stdev),
            pitch_stdev_deg=float(msg.pitch_stdev),
            config=self._heading2_config,
        )
        valid = self._heading2_currently_valid(time.monotonic())
        self._heading2_reason = self._describe_heading2(valid)
        self._update_heading2_state(valid)
        # HH_260811 - HEADING2 is diagnostic-only until ALIGN/mounting produces valid INSPVAX.

    def _describe_orientation(self, valid: bool) -> str:
        if valid:
            return "Fresh INS_SOLUTION_GOOD INSPVAX orientation"
        if self._last_inspvax is None:
            return "No INSPVAX received"
        status = int(self._last_inspvax.ins_status.status)
        if status != self._config.required_ins_status:
            return (
                f"INS status {status} is not INS_SOLUTION_GOOD "
                f"({self._config.required_ins_status})"
            )
        yaw_rmse = float(self._last_inspvax.azimuth_stdev)
        if math.isfinite(yaw_rmse) and yaw_rmse > self._config.max_yaw_rmse_deg:
            return (
                f"Azimuth RMSE {yaw_rmse:.3f}deg exceeds "
                f"{self._config.max_yaw_rmse_deg:.3f}deg"
            )
        if not self._last_inspvax_fields_valid:
            return "INSPVAX attitude or RMSE fields are invalid"
        return "INSPVAX input is stale or its header stamp is invalid"

    def _describe_heading2(self, valid: bool) -> str:
        if valid:
            return "Valid fixed HEADING2 observed (diagnostic only)"
        if self._last_heading2 is None:
            return "No HEADING2 received (optional dual antenna)"
        if not self._last_heading2_fields_valid:
            return "HEADING2 is not a computed allowed fixed solution"
        return "HEADING2 input is stale or its header stamp is invalid"

    def _update_orientation_state(self, valid: bool) -> None:
        source = OrientationSource.INSPVAX if valid else OrientationSource.NONE
        if valid != self._published_valid_state:
            self._published_valid_state = valid
            self._valid_pub.publish(Bool(data=valid))
        if source != self._published_source:
            self._published_source = source
            self._source_pub.publish(String(data=source.value))

    def _update_heading2_state(self, valid: bool) -> None:
        if valid == self._published_heading2_valid:
            return
        self._published_heading2_valid = valid
        self._heading2_valid_pub.publish(Bool(data=valid))

    @staticmethod
    def _optional_value(value, formatter=str) -> str:
        return "never" if value is None else formatter(value)

    def _publish_diagnostic(self) -> None:
        now = time.monotonic()
        orientation_valid = self._orientation_currently_valid(now)
        heading2_valid = self._heading2_currently_valid(now)
        self._orientation_reason = self._describe_orientation(orientation_valid)
        self._heading2_reason = self._describe_heading2(heading2_valid)
        self._update_orientation_state(orientation_valid)
        self._update_heading2_state(heading2_valid)

        inspvax_age = (
            None
            if self._last_inspvax_arrival is None
            else now - self._last_inspvax_arrival
        )
        ins_status = (
            None
            if self._last_inspvax is None
            else int(self._last_inspvax.ins_status.status)
        )
        ins_yaw_rmse = (
            None
            if self._last_inspvax is None
            else float(self._last_inspvax.azimuth_stdev)
        )
        ins_ext_status = (
            None
            if self._last_inspvax is None
            else int(self._last_inspvax.ext_sol_status.status)
        )
        alignment_source, align_update_active = (
            ("never", False)
            if ins_ext_status is None
            else decode_inspvax_alignment(ins_ext_status)
        )
        orientation_status = DiagnosticStatus(
            level=(
                DiagnosticStatus.OK if orientation_valid else DiagnosticStatus.ERROR
            ),
            name=f"{self.get_fully_qualified_name()}: NovAtel INS orientation",
            message=self._orientation_reason,
            hardware_id="novatel_oem7_inspvax",
            values=[
                KeyValue(
                    key="valid_orientation", value=str(orientation_valid).lower()
                ),
                KeyValue(
                    key="selected_source",
                    value=(
                        OrientationSource.INSPVAX.value
                        if orientation_valid
                        else OrientationSource.NONE.value
                    ),
                ),
                KeyValue(
                    key="ins_status", value=self._optional_value(ins_status)
                ),
                KeyValue(
                    key="azimuth_rmse_deg",
                    value=self._optional_value(
                        ins_yaw_rmse, lambda value: f"{value:.6f}"
                    ),
                ),
                KeyValue(
                    key="arrival_age_sec",
                    value=self._optional_value(
                        inspvax_age, lambda value: f"{value:.3f}"
                    ),
                ),
                # HH_260811 - Surface receiver-reported ALIGN evidence without changing the gate.
                KeyValue(
                    key="ins_ext_solution_status",
                    value=(
                        "never" if ins_ext_status is None else f"0x{ins_ext_status:08x}"
                    ),
                ),
                KeyValue(key="alignment_indication", value=alignment_source),
                KeyValue(
                    key="align_heading_update_active",
                    value=str(align_update_active).lower(),
                ),
            ],
        )

        heading_age = (
            None
            if self._last_heading2_arrival is None
            else now - self._last_heading2_arrival
        )
        heading_status_value = (
            None
            if self._last_heading2 is None
            else int(self._last_heading2.sol_status.status)
        )
        heading_type = (
            None
            if self._last_heading2 is None
            else int(self._last_heading2.pos_type.type)
        )
        heading_rmse = (
            None
            if self._last_heading2 is None
            else float(self._last_heading2.heading_stdev)
        )
        # HH_260811 - Report dual readiness without claiming its baseline is vehicle yaw.
        heading_status = DiagnosticStatus(
            level=(DiagnosticStatus.OK if heading2_valid else DiagnosticStatus.WARN),
            name=f"{self.get_fully_qualified_name()}: NovAtel HEADING2 readiness",
            message=self._heading2_reason,
            hardware_id="novatel_oem7_heading2",
            values=[
                KeyValue(
                    key="heading2_valid", value=str(heading2_valid).lower()
                ),
                KeyValue(key="used_for_orientation", value="false"),
                KeyValue(
                    key="vehicle_baseline_calibration",
                    value="not_verified_by_this_node",
                ),
                KeyValue(
                    key="solution_status",
                    value=self._optional_value(heading_status_value),
                ),
                KeyValue(
                    key="position_type", value=self._optional_value(heading_type)
                ),
                KeyValue(
                    key="heading_rmse_deg",
                    value=self._optional_value(
                        heading_rmse, lambda value: f"{value:.6f}"
                    ),
                ),
                KeyValue(
                    key="arrival_age_sec",
                    value=self._optional_value(
                        heading_age, lambda value: f"{value:.3f}"
                    ),
                ),
            ],
        )
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [orientation_status, heading_status]
        self._diagnostic_pub.publish(array)


def main(args=None) -> None:
    """Run the NovAtel INSPVAX adapter and HEADING2 readiness monitor."""
    rclpy.init(args=args)
    node = NovatelInspvaxOrientationNode()
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
