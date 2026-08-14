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

import hashlib
import os
import tempfile
import time
import unittest

from autoware_perception_msgs.msg import ObjectClassification
from autoware_perception_msgs.msg import Shape
from autoware_perception_msgs.msg import TrackedObject
from autoware_perception_msgs.msg import TrackedObjects
from autoware_vehicle_msgs.msg import ControlModeReport
from autoware_vehicle_msgs.msg import GearReport
from autoware_vehicle_msgs.msg import VelocityReport
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue
import launch
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.serialization import serialize_message
from vils_interfaces.msg import AcceptedPc4Status
from vils_interfaces.srv import ArmPc4Source


_OBJECT_TOPIC = '/test/vils/virtual_objects'
_DIAGNOSTIC_TOPIC = '/test/vils/diagnostics'
_PHYSICAL_TOPIC = '/test/vils/physical_objects'
_OUTPUT_TOPIC = '/test/vils/candidate_objects'
_STATUS_TOPIC = '/test/vils/accepted_status'
_VELOCITY_TOPIC = '/test/vils/velocity'
_GEAR_TOPIC = '/test/vils/gear'
_CONTROL_MODE_TOPIC = '/test/vils/control_mode'
_MAP_DIGEST = 'synthetic-map-digest'
_TRANSFORM_DIGEST = 'synthetic-transform-digest'
_SERIALIZATION_CONTRACT = 'receiver-reserialization-cdr-v1'
_SESSION_ID = 'synthetic-session-1'

_PROVENANCE_FD, _PROVENANCE_PATH = tempfile.mkstemp(
    prefix='pc2_vils_shadow_test.', suffix='.jsonl'
)
os.close(_PROVENANCE_FD)


# HH_260810 - Launch one approved shadow node only on isolated non-vehicle test topics.
@pytest.mark.launch_test
def generate_test_description():
    integration = Node(
        package='autoware_vils_object_integration',
        executable='vils_object_integration_node',
        name='vils_object_integration',
        output='screen',
        parameters=[
            {
                'mode': 'shadow',
                'contract_approved': True,
                'enable_canonical_selection': False,
                'enable_required_modes': False,
                'topics.physical_objects': _PHYSICAL_TOPIC,
                'topics.virtual_objects': _OBJECT_TOPIC,
                'topics.pc4_diagnostics': _DIAGNOSTIC_TOPIC,
                'topics.output_objects': _OUTPUT_TOPIC,
                'topics.accepted_status': _STATUS_TOPIC,
                'topics.velocity_report': _VELOCITY_TOPIC,
                'topics.gear_report': _GEAR_TOPIC,
                'topics.control_mode_report': _CONTROL_MODE_TOPIC,
                'contract.expected_frame_id': 'map',
                'contract.expected_map_digest': _MAP_DIGEST,
                'contract.expected_transform_digest': _TRANSFORM_DIGEST,
                'contract.expected_serialization_contract': _SERIALIZATION_CONTRACT,
                'contract.approved_object_publisher_node_fqn': '/synthetic_pc4_adapter',
                'contract.approved_diagnostic_publisher_node_fqn': '/synthetic_pc4_adapter',
                'contract.diagnostic_name': 'pc4_object_adapter',
                'contract.diagnostic_hardware_id': 'pc4/digital_twin',
                'contract.allow_empty_snapshot': True,
                'timing.virtual_ttl_ms': 1000,
                'timing.metadata_join_timeout_ms': 500,
                'timing.max_source_age_sec': 5.0,
                'timing.max_future_skew_sec': 0.2,
                'timing.max_fusion_skew_sec': 0.5,
                'timing.safe_state_timeout_sec': 1.0,
                'limits.max_objects': 16,
                'limits.max_abs_position_m': 1000.0,
                'limits.max_abs_velocity_mps': 100.0,
                'limits.max_abs_acceleration_mps2': 50.0,
                'limits.max_covariance': 1000.0,
                'limits.min_dimension_m': 0.01,
                'limits.max_dimension_m': 20.0,
                'limits.max_footprint_points': 32,
                'limits.max_pending_samples': 8,
                'fusion.association_distance_m': 1.0,
                'fusion.require_matching_primary_classification': True,
                'arming.stationary_velocity_mps': 0.05,
                'arming.stationary_yaw_rate_rps': 0.05,
                'arming.identity_timeout_sec': 5.0,
                'arming.require_safe_vehicle_state': True,
                'evidence.provenance_log_path': _PROVENANCE_PATH,
            }
        ],
    )
    return launch.LaunchDescription([integration, launch_testing.actions.ReadyToTest()])


def _qos():
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def _make_object(identifier, x, label):
    tracked = TrackedObject()
    tracked.object_id.uuid = [identifier] + [0] * 15
    tracked.existence_probability = 0.9
    classification = ObjectClassification()
    classification.label = label
    classification.probability = 1.0
    tracked.classification = [classification]
    tracked.kinematics.pose_with_covariance.pose.position.x = x
    tracked.kinematics.pose_with_covariance.pose.orientation.w = 1.0
    tracked.kinematics.orientation_availability = 2
    tracked.shape.type = Shape.BOUNDING_BOX
    tracked.shape.dimensions.x = 4.0
    tracked.shape.dimensions.y = 2.0
    tracked.shape.dimensions.z = 1.5
    return tracked


class TestVilsShadowContract(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        # HH_260810 - One adapter owns both approved PC4 writers for exact FQN/GID checks.
        self.node = rclpy.create_node('synthetic_pc4_adapter')
        qos = _qos()
        self.object_publisher = self.node.create_publisher(TrackedObjects, _OBJECT_TOPIC, qos)
        self.diagnostic_publisher = self.node.create_publisher(
            DiagnosticArray, _DIAGNOSTIC_TOPIC, qos
        )
        self.physical_publisher = self.node.create_publisher(
            TrackedObjects, _PHYSICAL_TOPIC, qos
        )
        self.velocity_publisher = self.node.create_publisher(
            VelocityReport, _VELOCITY_TOPIC, qos
        )
        self.gear_publisher = self.node.create_publisher(GearReport, _GEAR_TOPIC, qos)
        self.control_mode_publisher = self.node.create_publisher(
            ControlModeReport, _CONTROL_MODE_TOPIC, qos
        )
        self.status_messages = []
        self.output_messages = []
        self.status_subscription = self.node.create_subscription(
            AcceptedPc4Status, _STATUS_TOPIC, self.status_messages.append, qos
        )
        self.output_subscription = self.node.create_subscription(
            TrackedObjects, _OUTPUT_TOPIC, self.output_messages.append, qos
        )
        self.arm_client = self.node.create_client(
            ArmPc4Source, '/vils_object_integration/arm_pc4_source'
        )

    def tearDown(self):
        self.node.destroy_node()

    def _spin_until(self, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def _wait_for_contract_connections(self):
        publishers = [
            self.object_publisher,
            self.diagnostic_publisher,
            self.physical_publisher,
            self.velocity_publisher,
            self.gear_publisher,
            self.control_mode_publisher,
        ]
        return self._spin_until(
            lambda: all(publisher.get_subscription_count() == 1 for publisher in publishers)
            and self.arm_client.service_is_ready()
        )

    def _publisher_gid(self, topic):
        for endpoint in self.node.get_publishers_info_by_topic(topic):
            if endpoint.node_name == self.node.get_name():
                return bytes(endpoint.endpoint_gid).hex()
        self.fail(f'publisher GID was not discovered for {topic}')

    def _objects(self, entries):
        message = TrackedObjects()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        message.objects = [
            _make_object(identifier, x, label) for identifier, x, label in entries
        ]
        return message

    def _metadata(self, objects, sequence, object_gid):
        serialized = serialize_message(objects)
        digest = hashlib.sha256(serialized).hexdigest()
        stamp_ns = objects.header.stamp.sec * 1_000_000_000 + objects.header.stamp.nanosec
        values = {
            'output_cdr_sha256': digest,
            'session_id': _SESSION_ID,
            'source_node_fqn': '/synthetic_pc4_adapter',
            'source_writer_gid': object_gid,
            'map_digest': _MAP_DIGEST,
            'transform_digest': _TRANSFORM_DIGEST,
            'serialization_contract': _SERIALIZATION_CONTRACT,
            'sequence': str(sequence),
            'output_stamp_ns': str(stamp_ns),
            'output_object_count': str(len(objects.objects)),
            'source_ready': 'true',
            'full_snapshot': 'true',
        }
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.OK
        status.name = 'pc4_object_adapter'
        status.hardware_id = 'pc4/digital_twin'
        status.values = [KeyValue(key=key, value=value) for key, value in values.items()]
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = self.node.get_clock().now().to_msg()
        diagnostics.status = [status]
        return diagnostics

    def _publish_safe_state(self):
        self.velocity_publisher.publish(VelocityReport())
        gear = GearReport()
        gear.report = GearReport.PARK
        self.gear_publisher.publish(gear)
        control_mode = ControlModeReport()
        control_mode.mode = ControlModeReport.MANUAL
        self.control_mode_publisher.publish(control_mode)

    # HH_260810 - Cover metadata-first join, manual identity arm, dedup, and TTL physical fallback.
    def test_shadow_acceptance_fusion_and_ttl(self):
        self.assertTrue(self._wait_for_contract_connections())
        object_gid = self._publisher_gid(_OBJECT_TOPIC)
        diagnostic_gid = self._publisher_gid(_DIAGNOSTIC_TOPIC)

        first = self._objects([(20, 0.2, ObjectClassification.CAR)])
        self.diagnostic_publisher.publish(self._metadata(first, 1, object_gid))
        time.sleep(0.05)
        self.object_publisher.publish(first)
        identity_observed = self._spin_until(
                lambda: any(
                    status.session_id == _SESSION_ID
                    and status.source_writer_gid == object_gid
                    and status.diagnostic_writer_gid == diagnostic_gid
                    and not status.armed
                    for status in self.status_messages
                )
            )
        status_summary = [
            (
                status.reason,
                status.session_id,
                status.source_writer_gid,
                status.diagnostic_writer_gid,
            )
            for status in self.status_messages[-10:]
        ]
        self.assertTrue(
            identity_observed,
            f'identity was not observed; object_gid={object_gid}, '
            f'diagnostic_gid={diagnostic_gid}, statuses={status_summary}',
        )

        for _ in range(3):
            self._publish_safe_state()
            rclpy.spin_once(self.node, timeout_sec=0.05)
        request = ArmPc4Source.Request()
        request.arm = True
        request.session_id = _SESSION_ID
        request.writer_gid = object_gid
        request.diagnostic_writer_gid = diagnostic_gid
        request.map_digest = _MAP_DIGEST
        request.transform_digest = _TRANSFORM_DIGEST
        future = self.arm_client.call_async(request)
        self.assertTrue(self._spin_until(lambda: future.done()))
        self.assertTrue(future.result().success, future.result().reason)

        accepted = self._objects(
            [
                (20, 0.2, ObjectClassification.CAR),
                (30, 10.0, ObjectClassification.PEDESTRIAN),
            ]
        )
        self.object_publisher.publish(accepted)
        time.sleep(0.05)
        self.diagnostic_publisher.publish(self._metadata(accepted, 2, object_gid))
        self.assertTrue(
            self._spin_until(lambda: any(status.accepted for status in self.status_messages))
        )

        self.output_messages.clear()
        physical = self._objects([(10, 0.0, ObjectClassification.CAR)])
        self.physical_publisher.publish(physical)
        self.assertTrue(self._spin_until(lambda: len(self.output_messages) >= 1))
        fused_ids = [message.object_id.uuid[0] for message in self.output_messages[-1].objects]
        self.assertEqual(fused_ids, [10, 30])

        time.sleep(1.2)
        self.output_messages.clear()
        physical_after_ttl = self._objects([(10, 0.0, ObjectClassification.CAR)])
        self.physical_publisher.publish(physical_after_ttl)
        self.assertTrue(self._spin_until(lambda: len(self.output_messages) >= 1))
        fallback_ids = [
            message.object_id.uuid[0] for message in self.output_messages[-1].objects
        ]
        self.assertEqual(fallback_ids, [10])
        self.assertTrue(
            self._spin_until(
                lambda: any(
                    not status.armed
                    and not status.accepted
                    and status.reason == 'virtual_ttl_expired'
                    for status in self.status_messages
                )
            )
        )

        # HH_260810 - A fresh empty full snapshot is valid, but replaying it must disarm.
        for _ in range(3):
            self._publish_safe_state()
            rclpy.spin_once(self.node, timeout_sec=0.05)
        second_arm = ArmPc4Source.Request()
        second_arm.arm = True
        second_arm.session_id = _SESSION_ID
        second_arm.writer_gid = object_gid
        second_arm.diagnostic_writer_gid = diagnostic_gid
        second_arm.map_digest = _MAP_DIGEST
        second_arm.transform_digest = _TRANSFORM_DIGEST
        second_arm_future = self.arm_client.call_async(second_arm)
        self.assertTrue(self._spin_until(lambda: second_arm_future.done()))
        self.assertTrue(second_arm_future.result().success, second_arm_future.result().reason)

        empty_snapshot = self._objects([])
        self.diagnostic_publisher.publish(self._metadata(empty_snapshot, 3, object_gid))
        time.sleep(0.05)
        self.object_publisher.publish(empty_snapshot)
        self.assertTrue(
            self._spin_until(
                lambda: any(
                    status.accepted and status.sequence == 3 and status.object_count == 0
                    for status in self.status_messages
                )
            )
        )
        self.object_publisher.publish(empty_snapshot)
        self.assertTrue(
            self._spin_until(
                lambda: any(
                    not status.armed and status.reason == 'replayed_object_digest'
                    for status in self.status_messages
                )
            )
        )

        with open(_PROVENANCE_PATH, encoding='utf-8') as provenance:
            evidence = provenance.read()
        self.assertIn('"event":"snapshot_accepted"', evidence)
        self.assertIn('"event":"physical_triggered_fusion"', evidence)
        self.assertIn('"reason":"virtual_ttl_expired"', evidence)
        self.assertIn('"reason":"replayed_object_digest"', evidence)


@launch_testing.post_shutdown_test()
class TestVilsShadowProcess(unittest.TestCase):

    def test_exit_code_and_remove_fixture(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
        if os.path.exists(_PROVENANCE_PATH):
            os.unlink(_PROVENANCE_PATH)
