#!/usr/bin/env python3

# HH_260810 - Exercise the Stage-2 recorder with isolated local DDS publishers only.
import hashlib
import json
import os
from pathlib import Path
import signal
import stat
import subprocess
import sys
import tempfile
import time
import unittest

from autoware_perception_msgs.msg import TrackedObject
from autoware_perception_msgs.msg import TrackedObjects
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.serialization import deserialize_message
import rosbag2_py


# HH_260810 - Match the recorder's exact requested QoS for deterministic local discovery.
TEST_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
OBJECTS_TOPIC = '/test/pc2/vils/objects'
DIAGNOSTICS_TOPIC = '/test/pc2/vils/diagnostics'
ACCEPTED_STATUS_TOPIC = '/test/pc2/vils/accepted_status'
CANDIDATE_TOPIC = '/test/pc2/vils/candidate_objects'
CANONICAL_TRACKED_TOPIC = '/test/pc2/vils/canonical_tracked_objects'
RECORDER = Path(__file__).with_name('record_pc2_vils_stage2_raw.py')


# HH_260810 - Validate raw integrity, graph evidence, fail-closed files, and clean signals.
class Stage2RecorderSyntheticTest(unittest.TestCase):

    # HH_260810 - Use one local synthetic writer and no vehicle or distributed endpoints.
    @classmethod
    def setUpClass(cls) -> None:
        if os.environ.get('ROS_LOCALHOST_ONLY') != '1':
            raise RuntimeError('synthetic recorder tests require ROS_LOCALHOST_ONLY=1')
        domain_id = os.environ.get('ROS_DOMAIN_ID', '')
        if not domain_id.isdecimal() or int(domain_id) == 10:
            raise RuntimeError('synthetic recorder tests require an explicit non-vehicle domain')
        rclpy.init(args=None)
        cls.node = Node(
            'pc2_stage2_synthetic_publisher',
            enable_rosout=False,
            start_parameter_services=False,
        )
        parameter_event_publisher = cls.node._parameter_event_publisher
        if parameter_event_publisher is None or not cls.node.destroy_publisher(
            parameter_event_publisher
        ):
            raise RuntimeError('failed to remove the synthetic parameter event publisher')
        cls.node._parameter_event_publisher = None
        cls.objects_publisher = cls.node.create_publisher(
            TrackedObjects,
            OBJECTS_TOPIC,
            TEST_QOS,
        )
        cls.diagnostics_publisher = cls.node.create_publisher(
            DiagnosticArray,
            DIAGNOSTICS_TOPIC,
            TEST_QOS,
        )

    # HH_260810 - Release only the synthetic test participant.
    @classmethod
    def tearDownClass(cls) -> None:
        cls.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    # HH_260810 - Start a recorder with an explicit fresh run directory.
    def start_recorder(self, output_dir: Path, duration: float) -> subprocess.Popen[str]:
        command = [
            sys.executable,
            str(RECORDER),
            '--output-dir',
            str(output_dir),
            '--duration',
            str(duration),
            '--graph-interval',
            '0.2',
            '--source-absence-threshold',
            '0.2',
            '--objects-topic',
            OBJECTS_TOPIC,
            '--diagnostics-topic',
            DIAGNOSTICS_TOPIC,
            '--accepted-status-topic',
            ACCEPTED_STATUS_TOPIC,
            '--candidate-topic',
            CANDIDATE_TOPIC,
            '--canonical-tracked-topic',
            CANONICAL_TRACKED_TOPIC,
        ]
        return subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )

    # HH_260810 - Wait for both subscriptions and audit that the recorder exposes no writers.
    def wait_for_recorder(self, timeout_sec: float = 4.0) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            names = set(self.node.get_node_names_and_namespaces())
            if ('pc2_vils_stage2_raw_recorder', '/') not in names:
                continue
            subscriptions = self.node.get_subscriber_names_and_types_by_node(
                'pc2_vils_stage2_raw_recorder',
                '/',
            )
            topics = {name for name, _types in subscriptions}
            if OBJECTS_TOPIC not in topics or DIAGNOSTICS_TOPIC not in topics:
                continue
            publishers = self.node.get_publisher_names_and_types_by_node(
                'pc2_vils_stage2_raw_recorder',
                '/',
            )
            services = self.node.get_service_names_and_types_by_node(
                'pc2_vils_stage2_raw_recorder',
                '/',
            )
            clients = self.node.get_client_names_and_types_by_node(
                'pc2_vils_stage2_raw_recorder',
                '/',
            )
            self.assertEqual([], publishers)
            self.assertEqual([], services)
            self.assertEqual([], clients)
            return
        self.fail('recorder subscriptions were not discovered before the timeout')

    # HH_260810 - Construct nonempty and explicit empty snapshots with an observable source stamp.
    @staticmethod
    def make_objects(object_count: int) -> TrackedObjects:
        message = TrackedObjects()
        message.header.frame_id = 'map'
        message.header.stamp.sec = 123
        message.header.stamp.nanosec = 456
        for index in range(object_count):
            tracked_object = TrackedObject()
            tracked_object.object_id.uuid[15] = index + 1
            message.objects.append(tracked_object)
        return message

    # HH_260810 - Construct Stage-2 identity metadata including an intentional sequence gap.
    @staticmethod
    def make_diagnostics(sequence: int) -> DiagnosticArray:
        message = DiagnosticArray()
        message.header.stamp.sec = 123
        message.header.stamp.nanosec = 456
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.OK
        status.name = 'pc4_vils_adapter'
        status.message = 'ready'
        status.hardware_id = 'pc4'
        values = {
            'full_snapshot': 'true',
            'map_digest': 'map-sha256',
            'output_cdr_sha256': 'f' * 64,
            'output_object_count': '0',
            'output_stamp_ns': '123000000456',
            'sequence': str(sequence),
            'serialization_contract': 'cdr-v1',
            'session_id': 'synthetic-session',
            'source_node_fqn': '/pc2_stage2_synthetic_publisher',
            'source_ready': 'true',
            'source_writer_gid': 'a' * 48,
            'transform_digest': 'transform-sha256',
        }
        status.values = [KeyValue(key=key, value=value) for key, value in values.items()]
        message.status = [status]
        return message

    # HH_260810 - Load append-only JSONL evidence after the child has sealed its summary.
    @staticmethod
    def load_events(output_dir: Path) -> list[dict[str, object]]:
        return [
            json.loads(line)
            for line in (output_dir / 'events.jsonl').read_text().splitlines()
        ]

    # HH_260810 - Reopen the sealed sqlite3 bag and verify every callback CDR was persisted.
    @staticmethod
    def load_bag_messages(output_dir: Path) -> list[tuple[str, bytes, int]]:
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(
                uri=str(output_dir / 'raw_cdr'),
                storage_id='sqlite3',
            ),
            rosbag2_py.ConverterOptions('', ''),
        )
        messages = []
        while reader.has_next():
            topic, payload, timestamp = reader.read_next()
            messages.append((topic, bytes(payload), int(timestamp)))
        return messages

    # HH_260810 - Verify typed summaries never replace the exact received CDR hash.
    def test_duration_recording_and_non_overwrite(self) -> None:
        with tempfile.TemporaryDirectory(prefix='pc2-vils-stage2-test-') as temporary:
            output_dir = Path(temporary) / 'run'
            process = self.start_recorder(output_dir, duration=2.4)
            self.wait_for_recorder()
            nonempty_objects = self.make_objects(1)
            empty_objects = self.make_objects(0)
            diagnostic_one = self.make_diagnostics(1)
            diagnostic_three = self.make_diagnostics(3)

            # HH_260810 - Publish only after DDS confirms the recorder subscriptions exist.
            time.sleep(0.25)
            self.objects_publisher.publish(nonempty_objects)
            self.diagnostics_publisher.publish(diagnostic_one)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            time.sleep(0.15)
            self.objects_publisher.publish(empty_objects)
            self.diagnostics_publisher.publish(diagnostic_three)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            stdout, stderr = process.communicate(timeout=6.0)
            self.assertEqual(0, process.returncode, msg=f'{stdout}\n{stderr}')

            # HH_260810 - Confirm owner-only directory and file modes.
            self.assertEqual(0o700, stat.S_IMODE(output_dir.stat().st_mode))
            for name in (
                'COMPLETE.json',
                'checksums.json',
                'events.jsonl',
                'manifest.json',
                'summary.json',
            ):
                self.assertEqual(
                    0o600,
                    stat.S_IMODE((output_dir / name).stat().st_mode),
                )
            self.assertEqual(0o700, stat.S_IMODE((output_dir / 'raw_cdr').stat().st_mode))
            manifest = json.loads((output_dir / 'manifest.json').read_text())
            config_bytes = json.dumps(
                manifest['config'],
                allow_nan=False,
                ensure_ascii=False,
                separators=(',', ':'),
                sort_keys=True,
            ).encode('utf-8')
            self.assertEqual(
                hashlib.sha256(config_bytes).hexdigest(),
                manifest['config_sha256'],
            )
            self.assertEqual(0, manifest['steady_state_ros_contract']['publishers'])
            self.assertIn('not per-message writer attribution', manifest['identity_limitations'])
            self.assertEqual('rosbag2/sqlite3', manifest['raw_payload_storage']['format'])

            # HH_260810 - Verify exact raw hashes and the empty-message/no-message distinction.
            events = self.load_events(output_dir)
            object_events = [
                event
                for event in events
                if event.get('event') == 'raw_message' and event.get('source') == 'objects'
            ]
            diagnostic_events = [
                event
                for event in events
                if event.get('event') == 'raw_message'
                and event.get('source') == 'diagnostics'
            ]
            self.assertEqual(2, len(object_events))
            self.assertEqual([1, 0], [event['typed']['object_count'] for event in object_events])
            self.assertEqual(
                [False, True],
                [event['typed']['empty_snapshot'] for event in object_events],
            )
            for object_event in object_events:
                self.assertRegex(object_event['raw_sha256'], r'^[0-9a-f]{64}$')
                self.assertFalse(object_event['callback_gid_available'])
                self.assertIsNone(object_event['callback_publication_gid'])
                self.assertEqual(
                    123000000456,
                    object_event['typed']['source_stamp']['stamp_ns'],
                )
            self.assertEqual(2, len(diagnostic_events))
            received_diagnostic_hashes = {
                event['raw_sha256'] for event in diagnostic_events
            }
            self.assertEqual(2, len(received_diagnostic_hashes))
            for received_hash in received_diagnostic_hashes:
                self.assertRegex(received_hash, r'^[0-9a-f]{64}$')
            for diagnostic_event in diagnostic_events:
                self.assertIn('statuses', diagnostic_event['typed'], msg=diagnostic_event)
            sequence_summary = diagnostic_events[-1]['typed']['statuses'][0][
                'contract_values'
            ]['sequence'][0]
            self.assertEqual('3', sequence_summary['display'])

            # HH_260810 - Require a periodic graph view with the synthetic writer identity and QoS.
            graph_events = [event for event in events if event.get('event') == 'graph_snapshot']
            self.assertTrue(graph_events)
            self.assertTrue(
                all(event['stage2_graph_invariant']['passed'] for event in graph_events)
            )
            self.assertTrue(
                all(not event['stage2_graph_invariant']['violations'] for event in graph_events)
            )
            matching_graphs = [
                event
                for event in graph_events
                if event['topics']['objects']['publisher_cardinality'] == 1
                and event['topics']['diagnostics']['publisher_cardinality'] == 1
            ]
            self.assertTrue(matching_graphs)
            object_endpoint = matching_graphs[-1]['topics']['objects']['publishers'][0]
            self.assertEqual('/pc2_stage2_synthetic_publisher', object_endpoint['node_fqn'])
            self.assertTrue(object_endpoint['endpoint_gid'])
            self.assertEqual('RELIABLE', object_endpoint['qos']['reliability'])
            self.assertEqual('VOLATILE', object_endpoint['qos']['durability'])
            self.assertEqual('KEEP_LAST', object_endpoint['qos']['history'])
            self.assertEqual(1, object_endpoint['qos']['depth'])
            final_graph = graph_events[-1]
            self.assertEqual(
                1, final_graph['topics']['objects']['subscription_cardinality']
            )
            self.assertEqual(
                1, final_graph['topics']['diagnostics']['subscription_cardinality']
            )
            self.assertEqual(
                0,
                final_graph['protected_outputs']['accepted_status'][
                    'publisher_cardinality'
                ],
            )
            self.assertEqual(
                0,
                final_graph['protected_outputs']['candidate']['publisher_cardinality'],
            )
            self.assertEqual([], final_graph['recorder_entities']['publishers'])
            self.assertEqual([], final_graph['recorder_entities']['services'])
            self.assertEqual([], final_graph['recorder_entities']['clients'])

            # HH_260810 - Confirm absence and partial-drop metrics survive in the sealed summary.
            summary = json.loads((output_dir / 'summary.json').read_text())
            self.assertTrue(summary['clean_shutdown'])
            self.assertEqual('duration_elapsed', summary['stop_reason'])
            self.assertEqual(2, summary['sources']['objects']['received_count'])
            self.assertEqual(1, summary['sources']['objects']['empty_snapshot_count'])
            self.assertEqual(2, summary['sources']['diagnostics']['received_count'])
            inferred = summary['sources']['diagnostics'][
                'diagnostic_sequence_drop_inference'
            ]
            self.assertEqual(1, inferred['gap_count'])
            self.assertEqual(1, inferred['missing_count'])
            self.assertGreaterEqual(
                summary['sources']['objects']['absence']['interval_count'],
                1,
            )
            self.assertEqual(4, summary['rosbag2']['write_count'])
            sealed_files = summary['rosbag2']['sealed_files']
            self.assertTrue(sealed_files)
            for sealed_file in sealed_files:
                sealed_path = output_dir / sealed_file['path']
                self.assertTrue(sealed_path.is_file())
                self.assertEqual(0o600, stat.S_IMODE(sealed_path.stat().st_mode))
                self.assertRegex(sealed_file['sha256'], r'^[0-9a-f]{64}$')
                self.assertEqual(sealed_file['size'], sealed_path.stat().st_size)
                self.assertEqual(
                    sealed_file['sha256'], hashlib.sha256(sealed_path.read_bytes()).hexdigest()
                )

            # HH_260810 - Match bag order, timestamps, and payload hashes to callback event order.
            bag_messages = self.load_bag_messages(output_dir)
            self.assertEqual(4, len(bag_messages))
            raw_events = [event for event in events if event.get('event') == 'raw_message']
            self.assertEqual(list(range(1, 5)), [event['receive_index'] for event in raw_events])
            expected_bag_order = [
                (
                    event['topic'],
                    event['raw_sha256'],
                    event['bag_timestamp_ns'],
                )
                for event in raw_events
            ]
            actual_bag_order = [
                (topic, hashlib.sha256(payload).hexdigest(), timestamp)
                for topic, payload, timestamp in bag_messages
            ]
            self.assertEqual(expected_bag_order, actual_bag_order)

            # HH_260810 - Re-deserialize the sealed callbacks to preserve typed empty semantics.
            bag_object_counts = []
            bag_diagnostic_sequences = []
            for topic, payload, _timestamp in bag_messages:
                if topic == OBJECTS_TOPIC:
                    bag_object_counts.append(
                        len(deserialize_message(payload, TrackedObjects).objects)
                    )
                elif topic == DIAGNOSTICS_TOPIC:
                    diagnostics = deserialize_message(payload, DiagnosticArray)
                    sequence = next(
                        value.value
                        for value in diagnostics.status[0].values
                        if value.key == 'sequence'
                    )
                    bag_diagnostic_sequences.append(sequence)
            self.assertEqual([1, 0], bag_object_counts)
            self.assertEqual(['1', '3'], bag_diagnostic_sequences)

            # HH_260810 - COMPLETE is authoritative only when all root checksum bytes match.
            complete = json.loads((output_dir / 'COMPLETE.json').read_text())
            self.assertTrue(complete['clean_shutdown'])
            checksums_bytes = (output_dir / complete['checksums_file']).read_bytes()
            self.assertEqual(
                complete['checksums_sha256'], hashlib.sha256(checksums_bytes).hexdigest()
            )
            checksums = json.loads(checksums_bytes)['files']
            self.assertEqual(complete['file_count'], len(checksums))
            for checksum in checksums:
                checked_path = output_dir / checksum['path']
                self.assertEqual(checksum['size'], checked_path.stat().st_size)
                self.assertEqual(
                    checksum['sha256'], hashlib.sha256(checked_path.read_bytes()).hexdigest()
                )

            # HH_260810 - A second invocation must fail without changing the existing run.
            original_manifest = (output_dir / 'manifest.json').read_bytes()
            duplicate = subprocess.run(
                [
                    sys.executable,
                    str(RECORDER),
                    '--output-dir',
                    str(output_dir),
                    '--duration',
                    '0.1',
                    '--objects-topic',
                    OBJECTS_TOPIC,
                    '--diagnostics-topic',
                    DIAGNOSTICS_TOPIC,
                    '--accepted-status-topic',
                    ACCEPTED_STATUS_TOPIC,
                    '--candidate-topic',
                    CANDIDATE_TOPIC,
                    '--canonical-tracked-topic',
                    CANONICAL_TRACKED_TOPIC,
                ],
                capture_output=True,
                check=False,
                text=True,
                timeout=4.0,
            )
            self.assertNotEqual(0, duplicate.returncode)
            self.assertEqual(original_manifest, (output_dir / 'manifest.json').read_bytes())

    # HH_260810 - Verify an operator SIGINT produces a clean, sealed, finite run.
    def test_sigint_shutdown(self) -> None:
        with tempfile.TemporaryDirectory(prefix='pc2-vils-stage2-sigint-') as temporary:
            output_dir = Path(temporary) / 'run'
            process = self.start_recorder(output_dir, duration=0.0)
            self.wait_for_recorder()
            time.sleep(0.3)
            process.send_signal(signal.SIGINT)
            stdout, stderr = process.communicate(timeout=5.0)
            self.assertEqual(0, process.returncode, msg=f'{stdout}\n{stderr}')
            summary = json.loads((output_dir / 'summary.json').read_text())
            self.assertTrue(summary['clean_shutdown'])
            self.assertEqual('sigint', summary['stop_reason'])
            self.assertGreater(summary['duration_ns'], 0)
            self.assertTrue((output_dir / 'COMPLETE.json').is_file())

    # HH_260810 - Abort Stage 2 if a candidate writer or second raw consumer appears.
    def test_stage2_graph_conflict_fails_without_complete_marker(self) -> None:
        with tempfile.TemporaryDirectory(prefix='pc2-vils-stage2-conflict-') as temporary:
            output_dir = Path(temporary) / 'run'
            process = self.start_recorder(output_dir, duration=3.0)
            self.wait_for_recorder()
            candidate_publisher = self.node.create_publisher(
                TrackedObjects, CANDIDATE_TOPIC, TEST_QOS
            )
            unexpected_subscription = self.node.create_subscription(
                TrackedObjects, OBJECTS_TOPIC, lambda _message: None, TEST_QOS
            )
            try:
                stdout, stderr = process.communicate(timeout=5.0)
            finally:
                self.node.destroy_subscription(unexpected_subscription)
                self.node.destroy_publisher(candidate_publisher)
            self.assertNotEqual(0, process.returncode, msg=f'{stdout}\n{stderr}')
            self.assertFalse((output_dir / 'COMPLETE.json').exists())
            summary = json.loads((output_dir / 'summary.json').read_text())
            self.assertFalse(summary['clean_shutdown'])
            graph_events = [
                event
                for event in self.load_events(output_dir)
                if event.get('event') == 'graph_snapshot'
            ]
            failed_graphs = [
                event
                for event in graph_events
                if not event['stage2_graph_invariant']['passed']
            ]
            self.assertTrue(failed_graphs)
            violations = failed_graphs[-1]['stage2_graph_invariant']['violations']
            self.assertIn('candidate_publisher_present', violations)
            self.assertIn('objects_subscriber_cardinality_not_one', violations)

    # HH_260810 - Reject non-finite timing values before creating any evidence path.
    def test_nonfinite_timing_is_rejected_before_output_creation(self) -> None:
        with tempfile.TemporaryDirectory(prefix='pc2-vils-stage2-invalid-') as temporary:
            cases = (
                ('--duration', 'nan'),
                ('--graph-interval', 'inf'),
                ('--source-absence-threshold', '-inf'),
            )
            for index, (option, value) in enumerate(cases):
                with self.subTest(option=option, value=value):
                    output_dir = Path(temporary) / f'run-{index}'
                    result = subprocess.run(
                        [
                            sys.executable,
                            str(RECORDER),
                            '--output-dir',
                            str(output_dir),
                            f'{option}={value}',
                        ],
                        capture_output=True,
                        check=False,
                        text=True,
                        timeout=4.0,
                    )
                    self.assertNotEqual(0, result.returncode)
                    self.assertIn('must be finite', result.stderr)
                    self.assertFalse(output_dir.exists())

            # HH_260810 - Existing symlinks are never accepted as a new evidence root.
            symlink_target = Path(temporary) / 'symlink-target'
            symlink_target.mkdir()
            symlink_output = Path(temporary) / 'symlink-output'
            symlink_output.symlink_to(symlink_target, target_is_directory=True)
            result = subprocess.run(
                [
                    sys.executable,
                    str(RECORDER),
                    '--output-dir',
                    str(symlink_output),
                    '--duration',
                    '0.1',
                ],
                capture_output=True,
                check=False,
                text=True,
                timeout=4.0,
            )
            self.assertNotEqual(0, result.returncode)
            self.assertEqual([], list(symlink_target.iterdir()))


# HH_260810 - Keep the synthetic check directly runnable outside colcon/CMake.
if __name__ == '__main__':
    unittest.main(verbosity=2)
