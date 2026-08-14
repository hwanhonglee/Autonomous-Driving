#!/usr/bin/env python3

# HH_260810 - Record PC4 Stage-2 inputs without publishing or mutating any ROS state.
import argparse
import gc
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import socket
import stat
import sys
import threading
import time
from typing import Any, Optional

from autoware_perception_msgs.msg import TrackedObjects
from diagnostic_msgs.msg import DiagnosticArray
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos_event import SubscriptionEventCallbacks
from rclpy.serialization import deserialize_message
import rosbag2_py


# HH_260810 - Pin the evidence schema and the reviewed PC4 input defaults from the Stage-2 plan.
SCHEMA_VERSION = 'pc2_vils_stage2_raw_v1'
DEFAULT_OBJECTS_TOPIC = '/perception/pc4/virtual_obstacles/tracked_objects'
DEFAULT_DIAGNOSTICS_TOPIC = '/diagnostics/pc4/object_adapter'
DEFAULT_ACCEPTED_STATUS_TOPIC = '/perception/pc2/vils/accepted_pc4_status'
DEFAULT_CANDIDATE_TOPIC = '/perception/pc2/vils/candidate_tracked_objects'
DEFAULT_CANONICAL_TRACKED_TOPIC = '/perception/object_recognition/tracking/objects'
FORBIDDEN_INTEGRATION_NODE_NAME = 'vils_object_integration'
OBJECTS_TYPE = 'autoware_perception_msgs/msg/TrackedObjects'
DIAGNOSTICS_TYPE = 'diagnostic_msgs/msg/DiagnosticArray'
ACCEPTED_STATUS_TYPE = 'vils_interfaces/msg/AcceptedPc4Status'
CONTRACT_DIAGNOSTIC_KEYS = frozenset(
    {
        'full_snapshot',
        'map_digest',
        'output_cdr_sha256',
        'output_object_count',
        'output_stamp_ns',
        'sequence',
        'serialization_contract',
        'session_id',
        'source_node_fqn',
        'source_ready',
        'source_writer_gid',
        'transform_digest',
    }
)
RECEIVE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


# HH_260810 - Serialize evidence deterministically and reject non-finite JSON values.
def canonical_json_bytes(value: Any) -> bytes:
    return json.dumps(
        value,
        allow_nan=False,
        ensure_ascii=False,
        separators=(',', ':'),
        sort_keys=True,
    ).encode('utf-8')


# HH_260810 - Convert ROS policy enums to stable names for graph snapshots.
def policy_name(value: Any) -> str:
    return str(getattr(value, 'name', value))


# HH_260810 - Hash oversized diagnostic text instead of duplicating its raw payload.
def bounded_text(value: str, limit: int = 512) -> dict[str, Any]:
    encoded = value.encode('utf-8', errors='surrogatepass')
    displayed = (
        value if len(encoded) <= limit else encoded[:limit].decode('utf-8', 'replace')
    )
    return {
        'display': displayed,
        'sha256': hashlib.sha256(encoded).hexdigest(),
        'truncated': len(encoded) > limit,
        'utf8_bytes': len(encoded),
    }


# HH_260810 - Write every file through an owner-only, no-follow, exclusive descriptor.
class ExclusiveEvidenceDirectory:

    def __init__(self, output_dir: Path) -> None:
        self.output_dir = output_dir
        self.directory_fd = -1
        self.events_fd = -1

    # HH_260810 - Refuse an existing output path instead of merging or overwriting evidence.
    def create(self) -> None:
        os.mkdir(self.output_dir, mode=0o700)
        os.chmod(self.output_dir, 0o700, follow_symlinks=False)
        self.directory_fd = os.open(
            self.output_dir,
            os.O_RDONLY | os.O_DIRECTORY | getattr(os, 'O_NOFOLLOW', 0),
        )
        directory_stat = os.fstat(self.directory_fd)
        if not stat.S_ISDIR(directory_stat.st_mode):
            raise RuntimeError('evidence output is not a directory')
        if directory_stat.st_uid != os.geteuid():
            raise RuntimeError('evidence output is not owned by the recorder user')
        self.events_fd = self._open_exclusive('events.jsonl')
        os.fsync(self.directory_fd)

    # HH_260810 - Validate each newly created descriptor before accepting it as evidence storage.
    def _open_exclusive(self, name: str) -> int:
        descriptor = os.open(
            name,
            os.O_WRONLY
            | os.O_CREAT
            | os.O_EXCL
            | os.O_APPEND
            | getattr(os, 'O_CLOEXEC', 0)
            | getattr(os, 'O_NOFOLLOW', 0),
            0o600,
            dir_fd=self.directory_fd,
        )
        file_stat = os.fstat(descriptor)
        if not stat.S_ISREG(file_stat.st_mode):
            os.close(descriptor)
            raise RuntimeError(f'evidence target is not a regular file: {name}')
        if file_stat.st_uid != os.geteuid():
            os.close(descriptor)
            raise RuntimeError(f'evidence target is not owned by the recorder user: {name}')
        os.fchmod(descriptor, 0o600)
        return descriptor

    # HH_260810 - Complete short writes and surface storage errors before acknowledging an event.
    @staticmethod
    def _write_all(descriptor: int, payload: bytes) -> None:
        offset = 0
        while offset < len(payload):
            try:
                written = os.write(descriptor, payload[offset:])
            except InterruptedError:
                continue
            if written <= 0:
                raise OSError('evidence write returned no progress')
            offset += written

    # HH_260810 - Make each JSONL event durable so an interrupted run remains auditable.
    def append_event(self, event: dict[str, Any]) -> None:
        payload = canonical_json_bytes(event) + b'\n'
        self._write_all(self.events_fd, payload)
        os.fsync(self.events_fd)

    # HH_260810 - Create immutable manifest and summary files without temporary-path races.
    def write_json_file(self, name: str, value: dict[str, Any]) -> None:
        descriptor = self._open_exclusive(name)
        try:
            self._write_all(descriptor, canonical_json_bytes(value) + b'\n')
            os.fsync(descriptor)
        finally:
            os.close(descriptor)
        os.fsync(self.directory_fd)

    # HH_260810 - Close only descriptors owned by this recorder.
    def close(self) -> None:
        self.close_events()
        if self.directory_fd >= 0:
            os.close(self.directory_fd)
            self.directory_fd = -1

    # HH_260810 - Close the event stream before hashing its final immutable byte sequence.
    def close_events(self) -> None:
        if self.events_fd >= 0:
            os.fsync(self.events_fd)
            os.close(self.events_fd)
            self.events_fd = -1

    # HH_260810 - Seal every rosbag file as an owner-private regular file and record its digest.
    def seal_rosbag_tree(self, relative_name: str) -> list[dict[str, Any]]:
        root = self.output_dir / relative_name
        if not root.is_dir() or root.is_symlink():
            raise RuntimeError('raw rosbag output is missing or is not a real directory')
        sealed_files = []
        for directory in sorted((root, *[path for path in root.rglob('*') if path.is_dir()])):
            if directory.is_symlink() or directory.stat().st_uid != os.geteuid():
                raise RuntimeError(f'untrusted rosbag directory: {directory}')
            os.chmod(directory, 0o700, follow_symlinks=False)
        for path in sorted(path for path in root.rglob('*') if not path.is_dir()):
            if path.is_symlink():
                raise RuntimeError(f'rosbag output contains a symlink: {path}')
            descriptor = os.open(
                path,
                os.O_RDWR | getattr(os, 'O_CLOEXEC', 0) | getattr(os, 'O_NOFOLLOW', 0),
            )
            try:
                information = os.fstat(descriptor)
                if not stat.S_ISREG(information.st_mode) or information.st_uid != os.geteuid():
                    raise RuntimeError(f'untrusted rosbag file: {path}')
                os.fchmod(descriptor, 0o600)
                digest = hashlib.sha256()
                size = 0
                while True:
                    chunk = os.read(descriptor, 1024 * 1024)
                    if not chunk:
                        break
                    digest.update(chunk)
                    size += len(chunk)
                os.fsync(descriptor)
            finally:
                os.close(descriptor)
            sealed_files.append(
                {
                    'path': str(path.relative_to(self.output_dir)),
                    'sha256': digest.hexdigest(),
                    'size': size,
                }
            )
        if not sealed_files:
            raise RuntimeError('raw rosbag output contains no files')
        raw_directory_fd = os.open(
            root,
            os.O_RDONLY | os.O_DIRECTORY | getattr(os, 'O_NOFOLLOW', 0),
        )
        try:
            os.fsync(raw_directory_fd)
        finally:
            os.close(raw_directory_fd)
        os.fsync(self.directory_fd)
        return sealed_files

    # HH_260810 - Hash the final evidence root without recursively including its own checksum.
    def write_root_checksums(
        self, relative_paths: list[str]
    ) -> tuple[list[dict[str, Any]], str]:
        checksums = []
        for relative_path in sorted(set(relative_paths)):
            path = self.output_dir / relative_path
            if path.is_symlink():
                raise RuntimeError(f'evidence checksum target is a symlink: {relative_path}')
            descriptor = os.open(
                path,
                os.O_RDONLY | getattr(os, 'O_CLOEXEC', 0) | getattr(os, 'O_NOFOLLOW', 0),
            )
            try:
                information = os.fstat(descriptor)
                if not stat.S_ISREG(information.st_mode) or information.st_uid != os.geteuid():
                    raise RuntimeError(f'untrusted evidence checksum target: {relative_path}')
                digest = hashlib.sha256()
                size = 0
                while True:
                    chunk = os.read(descriptor, 1024 * 1024)
                    if not chunk:
                        break
                    digest.update(chunk)
                    size += len(chunk)
            finally:
                os.close(descriptor)
            checksums.append(
                {
                    'path': relative_path,
                    'sha256': digest.hexdigest(),
                    'size': size,
                }
            )
        document = {
            'files': checksums,
            'schema': SCHEMA_VERSION,
        }
        self.write_json_file('checksums.json', document)
        checksum_digest = hashlib.sha256(
            (self.output_dir / 'checksums.json').read_bytes()
        ).hexdigest()
        return checksums, checksum_digest


# HH_260810 - Preserve exact callback CDR bytes in rosbag2 beside JSON summaries.
class RawCdrBagWriter:

    def __init__(self, output_uri: Path, objects_topic: str, diagnostics_topic: str) -> None:
        if output_uri.exists() or output_uri.is_symlink():
            raise RuntimeError(f'raw rosbag output already exists: {output_uri}')
        self.output_uri = output_uri
        self.writer: Optional[rosbag2_py.SequentialWriter] = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py.StorageOptions(
            uri=str(output_uri),
            storage_id='sqlite3',
            max_cache_size=0,
        )
        converter_options = rosbag2_py.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)
        self.writer.create_topic(
            rosbag2_py.TopicMetadata(
                name=objects_topic,
                type=OBJECTS_TYPE,
                serialization_format='cdr',
                offered_qos_profiles='',
            )
        )
        self.writer.create_topic(
            rosbag2_py.TopicMetadata(
                name=diagnostics_topic,
                type=DIAGNOSTICS_TYPE,
                serialization_format='cdr',
                offered_qos_profiles='',
            )
        )
        self.write_count = 0
        self.last_bag_timestamp_ns = 0

    # HH_260810 - Use receiver wall time for the bag index while retaining source time in JSONL.
    def write(self, topic: str, raw_bytes: bytes, wall_ns: int) -> int:
        if self.writer is None:
            raise RuntimeError('raw rosbag writer is already closed')
        bag_timestamp_ns = max(wall_ns, self.last_bag_timestamp_ns + 1)
        self.writer.write(topic, raw_bytes, bag_timestamp_ns)
        self.last_bag_timestamp_ns = bag_timestamp_ns
        self.write_count += 1
        return bag_timestamp_ns

    # HH_260810 - Drop the final Python reference so rosbag2 flushes metadata before sealing.
    def close(self) -> None:
        writer = self.writer
        self.writer = None
        del writer
        gc.collect()


# HH_260810 - Track receive counts and thresholded source-absence intervals per input topic.
class SourceState:

    def __init__(self, source: str, started_monotonic_ns: int) -> None:
        self.source = source
        self.received_count = 0
        self.received_bytes = 0
        self.deserialization_errors = 0
        self.empty_snapshot_count = 0
        self.first_receive_monotonic_ns: Optional[int] = None
        self.last_receive_monotonic_ns: Optional[int] = None
        self.last_receive_wall_ns: Optional[int] = None
        self.max_receive_gap_ns = 0
        self.absence_open_ns: Optional[int] = started_monotonic_ns
        self.absence_interval_count = 0
        self.absence_total_ns = 0
        self.absence_max_ns = 0
        self.middleware_lost_total = 0
        self.middleware_lost_events = 0
        self.diagnostic_sequence_gap_count = 0
        self.diagnostic_sequence_missing_count = 0
        self.diagnostic_sequence_replay_or_reorder_count = 0
        self.diagnostic_sequences: dict[str, int] = {}

    # HH_260810 - Close a qualifying absence when a source resumes or the recorder ends.
    def close_absence(
        self,
        end_monotonic_ns: int,
        threshold_ns: int,
        reason: str,
    ) -> Optional[dict[str, Any]]:
        if self.absence_open_ns is None:
            return None
        duration_ns = max(0, end_monotonic_ns - self.absence_open_ns)
        start_ns = self.absence_open_ns
        self.absence_open_ns = None
        if duration_ns < threshold_ns:
            return None
        self.absence_interval_count += 1
        self.absence_total_ns += duration_ns
        self.absence_max_ns = max(self.absence_max_ns, duration_ns)
        return {
            'duration_ns': duration_ns,
            'end_monotonic_ns': end_monotonic_ns,
            'event': 'source_absence_interval',
            'reason': reason,
            'source': self.source,
            'start_monotonic_ns': start_ns,
        }

    # HH_260810 - Count every callback, including a valid zero-object snapshot.
    def received(self, wall_ns: int, monotonic_ns: int, raw_size: int) -> None:
        if self.last_receive_monotonic_ns is not None:
            self.max_receive_gap_ns = max(
                self.max_receive_gap_ns,
                monotonic_ns - self.last_receive_monotonic_ns,
            )
        self.received_count += 1
        self.received_bytes += raw_size
        if self.first_receive_monotonic_ns is None:
            self.first_receive_monotonic_ns = monotonic_ns
        self.last_receive_monotonic_ns = monotonic_ns
        self.last_receive_wall_ns = wall_ns

    # HH_260810 - Infer diagnostic sequence gaps separately from middleware loss notifications.
    def observe_diagnostic_sequence(self, identity: str, sequence: int) -> None:
        previous = self.diagnostic_sequences.get(identity)
        if previous is not None and sequence > previous + 1:
            self.diagnostic_sequence_gap_count += 1
            self.diagnostic_sequence_missing_count += sequence - previous - 1
        elif previous is not None and sequence < previous:
            self.diagnostic_sequence_replay_or_reorder_count += 1
        if previous is None or sequence > previous:
            self.diagnostic_sequences[identity] = sequence

    # HH_260810 - Keep the final summary bounded while exact intervals remain in JSONL.
    def summary(self) -> dict[str, Any]:
        return {
            'absence': {
                'interval_count': self.absence_interval_count,
                'max_ns': self.absence_max_ns,
                'total_ns': self.absence_total_ns,
            },
            'deserialization_error_count': self.deserialization_errors,
            'diagnostic_sequence_drop_inference': {
                'gap_count': self.diagnostic_sequence_gap_count,
                'missing_count': self.diagnostic_sequence_missing_count,
                'replay_or_reorder_count': self.diagnostic_sequence_replay_or_reorder_count,
            },
            'empty_snapshot_count': self.empty_snapshot_count,
            'first_receive_monotonic_ns': self.first_receive_monotonic_ns,
            'last_receive_monotonic_ns': self.last_receive_monotonic_ns,
            'last_receive_wall_ns': self.last_receive_wall_ns,
            'max_receive_gap_ns': self.max_receive_gap_ns,
            'middleware_drop_report': {
                'event_count': self.middleware_lost_events,
                'total_count': self.middleware_lost_total,
            },
            'received_bytes': self.received_bytes,
            'received_count': self.received_count,
        }


# HH_260810 - Subscribe to exactly two PC4 inputs and expose no publishers, services, or actions.
class Stage2RawRecorder(Node):
    def __init__(
        self,
        evidence: ExclusiveEvidenceDirectory,
        raw_bag: RawCdrBagWriter,
        objects_topic: str,
        diagnostics_topic: str,
        accepted_status_topic: str,
        candidate_topic: str,
        canonical_tracked_topic: str,
        started_monotonic_ns: int,
        absence_threshold_ns: int,
    ) -> None:
        super().__init__(
            'pc2_vils_stage2_raw_recorder',
            enable_rosout=False,
            start_parameter_services=False,
        )
        # HH_260810 - Remove rclpy's implicit parameter-event publisher before discovery.
        parameter_event_publisher = self._parameter_event_publisher
        if parameter_event_publisher is None or not self.destroy_publisher(
            parameter_event_publisher
        ):
            raise RuntimeError('failed to remove the implicit parameter event publisher')
        self._parameter_event_publisher = None
        self.evidence = evidence
        self.raw_bag = raw_bag
        self.objects_topic = objects_topic
        self.diagnostics_topic = diagnostics_topic
        self.accepted_status_topic = accepted_status_topic
        self.candidate_topic = candidate_topic
        self.canonical_tracked_topic = canonical_tracked_topic
        self.absence_threshold_ns = absence_threshold_ns
        self.receive_index = 0
        self.states = {
            'objects': SourceState('objects', started_monotonic_ns),
            'diagnostics': SourceState('diagnostics', started_monotonic_ns),
        }

        # HH_260810 - Capture middleware loss as an explicitly partial drop metric.
        objects_events = SubscriptionEventCallbacks(
            message_lost=lambda event: self._on_message_lost('objects', event),
        )
        diagnostics_events = SubscriptionEventCallbacks(
            message_lost=lambda event: self._on_message_lost('diagnostics', event),
        )
        self.objects_subscription = self.create_subscription(
            TrackedObjects,
            self.objects_topic,
            self._on_objects,
            RECEIVE_QOS,
            event_callbacks=objects_events,
            raw=True,
        )
        self.diagnostics_subscription = self.create_subscription(
            DiagnosticArray,
            self.diagnostics_topic,
            self._on_diagnostics,
            RECEIVE_QOS,
            event_callbacks=diagnostics_events,
            raw=True,
        )

    # HH_260810 - Record a loss notification as middleware evidence, not as an exhaustive count.
    def _on_message_lost(self, source: str, event: Any) -> None:
        state = self.states[source]
        state.middleware_lost_events += 1
        state.middleware_lost_total = int(event.total_count)
        self.evidence.append_event(
            {
                'event': 'middleware_message_lost',
                'source': source,
                'total_count': int(event.total_count),
                'total_count_change': int(event.total_count_change),
                'wall_ns': time.time_ns(),
                'monotonic_ns': time.monotonic_ns(),
            }
        )

    # HH_260810 - Close any thresholded no-message interval before recording a resumed source.
    def _begin_receive(self, source: str, raw_size: int) -> tuple[int, int, int, int]:
        wall_ns = time.time_ns()
        monotonic_ns = time.monotonic_ns()
        self.receive_index += 1
        state = self.states[source]
        absence = state.close_absence(
            monotonic_ns,
            self.absence_threshold_ns,
            'source_resumed',
        )
        if absence is not None:
            self.evidence.append_event(absence)
        state.received(wall_ns, monotonic_ns, raw_size)
        return wall_ns, monotonic_ns, self.receive_index, state.received_count

    # HH_260810 - Hash the exact serialized bytes delivered by rclpy before typed inspection.
    @staticmethod
    def _raw_identity(raw_message: bytes) -> tuple[bytes, str]:
        raw_bytes = bytes(raw_message)
        return raw_bytes, hashlib.sha256(raw_bytes).hexdigest()

    # HH_260810 - Preserve frame, source stamp, count, and empty-snapshot semantics for objects.
    def _on_objects(self, raw_message: bytes) -> None:
        raw_bytes, raw_sha256 = self._raw_identity(raw_message)
        wall_ns, monotonic_ns, receive_index, source_receive_index = self._begin_receive(
            'objects', len(raw_bytes)
        )
        bag_timestamp_ns = self.raw_bag.write(self.objects_topic, raw_bytes, wall_ns)
        typed: dict[str, Any]
        try:
            message = deserialize_message(raw_bytes, TrackedObjects)
            stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            object_count = len(message.objects)
            empty_snapshot = object_count == 0
            if empty_snapshot:
                self.states['objects'].empty_snapshot_count += 1
            typed = {
                'empty_snapshot': empty_snapshot,
                'frame_id': message.header.frame_id,
                'object_count': object_count,
                'source_stamp': {
                    'nanosec': int(message.header.stamp.nanosec),
                    'sec': int(message.header.stamp.sec),
                    'stamp_ns': int(stamp_ns),
                },
            }
        except Exception as error:  # noqa: B902
            self.states['objects'].deserialization_errors += 1
            typed = {
                'deserialization_error': f'{type(error).__name__}: {error}',
            }
        self.evidence.append_event(
            {
                'callback_gid_available': False,
                'callback_publication_gid': None,
                'bag_timestamp_ns': bag_timestamp_ns,
                'event': 'raw_message',
                'monotonic_ns': monotonic_ns,
                'raw_sha256': raw_sha256,
                'raw_size': len(raw_bytes),
                'source': 'objects',
                'receive_index': receive_index,
                'source_receive_index': source_receive_index,
                'topic': self.objects_topic,
                'type': OBJECTS_TYPE,
                'typed': typed,
                'wall_ns': wall_ns,
            }
        )

    # HH_260810 - Summarize identity keys while raw bytes remain the exact source record.
    def _diagnostic_status_summary(self, status: Any) -> dict[str, Any]:
        keys = [item.key for item in status.values]
        duplicate_keys = sorted({key for key in keys if keys.count(key) > 1})
        contract_values: dict[str, list[dict[str, Any]]] = {}
        for item in status.values:
            if item.key in CONTRACT_DIAGNOSTIC_KEYS:
                contract_values.setdefault(item.key, []).append(bounded_text(item.value))
        sequence_values = contract_values.get('sequence', [])
        session_values = contract_values.get('session_id', [])
        if len(sequence_values) == 1 and len(session_values) == 1:
            sequence_text = sequence_values[0]['display']
            if not sequence_values[0]['truncated'] and sequence_text.isdecimal():
                identity = '|'.join((status.name, status.hardware_id, session_values[0]['sha256']))
                self.states['diagnostics'].observe_diagnostic_sequence(
                    identity,
                    int(sequence_text),
                )
        key_value_hash = hashlib.sha256(
            canonical_json_bytes([[item.key, item.value] for item in status.values])
        ).hexdigest()
        # HH_260810 - Humble exposes the uint8 level as either an integer or a one-byte octet.
        if isinstance(status.level, (bytes, bytearray)):
            if len(status.level) != 1:
                raise ValueError('DiagnosticStatus.level octet must contain exactly one byte')
            level = status.level[0]
        else:
            level = int(status.level)
        return {
            'contract_values': contract_values,
            'duplicate_keys': duplicate_keys,
            'hardware_id': bounded_text(status.hardware_id),
            'key_count': len(keys),
            'key_value_sha256': key_value_hash,
            'keys': [bounded_text(key) for key in keys],
            'level': level,
            'message': bounded_text(status.message),
            'name': bounded_text(status.name),
        }

    # HH_260810 - Deserialize diagnostics only after preserving their exact received byte hash.
    def _on_diagnostics(self, raw_message: bytes) -> None:
        raw_bytes, raw_sha256 = self._raw_identity(raw_message)
        wall_ns, monotonic_ns, receive_index, source_receive_index = self._begin_receive(
            'diagnostics', len(raw_bytes)
        )
        bag_timestamp_ns = self.raw_bag.write(self.diagnostics_topic, raw_bytes, wall_ns)
        typed: dict[str, Any]
        try:
            message = deserialize_message(raw_bytes, DiagnosticArray)
            stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            typed = {
                'source_stamp': {
                    'nanosec': int(message.header.stamp.nanosec),
                    'sec': int(message.header.stamp.sec),
                    'stamp_ns': int(stamp_ns),
                },
                'status_count': len(message.status),
                'statuses': [
                    self._diagnostic_status_summary(status) for status in message.status
                ],
            }
        except Exception as error:  # noqa: B902
            self.states['diagnostics'].deserialization_errors += 1
            typed = {
                'deserialization_error': f'{type(error).__name__}: {error}',
            }
        self.evidence.append_event(
            {
                'callback_gid_available': False,
                'callback_publication_gid': None,
                'bag_timestamp_ns': bag_timestamp_ns,
                'event': 'raw_message',
                'monotonic_ns': monotonic_ns,
                'raw_sha256': raw_sha256,
                'raw_size': len(raw_bytes),
                'source': 'diagnostics',
                'receive_index': receive_index,
                'source_receive_index': source_receive_index,
                'topic': self.diagnostics_topic,
                'type': DIAGNOSTICS_TYPE,
                'typed': typed,
                'wall_ns': wall_ns,
            }
        )

    # HH_260810 - Describe endpoint QoS and graph GID without claiming per-message attribution.
    @staticmethod
    def _endpoint_summary(endpoint: Any) -> dict[str, Any]:
        namespace = endpoint.node_namespace.rstrip('/')
        node_fqn = f'{namespace}/{endpoint.node_name}' if namespace else f'/{endpoint.node_name}'
        qos = endpoint.qos_profile
        return {
            'endpoint_gid': bytes(endpoint.endpoint_gid).hex(),
            'node_fqn': node_fqn,
            'qos': {
                'avoid_ros_namespace_conventions': bool(
                    qos.avoid_ros_namespace_conventions
                ),
                'deadline_ns': int(qos.deadline.nanoseconds),
                'depth': int(qos.depth),
                'durability': policy_name(qos.durability),
                'history': policy_name(qos.history),
                'lifespan_ns': int(qos.lifespan.nanoseconds),
                'liveliness': policy_name(qos.liveliness),
                'liveliness_lease_duration_ns': int(
                    qos.liveliness_lease_duration.nanoseconds
                ),
                'reliability': policy_name(qos.reliability),
            },
            'topic_type': endpoint.topic_type,
        }

    # HH_260810 - Convert node graph name/type pairs to deterministic evidence entries.
    @staticmethod
    def _name_type_summary(entries: list[tuple[str, list[str]]]) -> list[dict[str, Any]]:
        return [
            {'name': name, 'types': sorted(types)}
            for name, types in sorted(entries, key=lambda item: item[0])
        ]

    # HH_260810 - Snapshot Stage-2 entities and reject any validator or selected-output path.
    def graph_snapshot(self, phase: str) -> dict[str, Any]:
        topics = {}
        for source, topic, expected_type in (
            ('objects', self.objects_topic, OBJECTS_TYPE),
            ('diagnostics', self.diagnostics_topic, DIAGNOSTICS_TYPE),
        ):
            publisher_endpoints = self.get_publishers_info_by_topic(topic)
            subscription_endpoints = self.get_subscriptions_info_by_topic(topic)
            topics[source] = {
                'expected_type': expected_type,
                'publisher_cardinality': len(publisher_endpoints),
                'publishers': [
                    self._endpoint_summary(endpoint) for endpoint in publisher_endpoints
                ],
                'subscription_cardinality': len(subscription_endpoints),
                'subscriptions': [
                    self._endpoint_summary(endpoint) for endpoint in subscription_endpoints
                ],
                'topic': topic,
            }
        protected_outputs = {}
        for name, topic, expected_type in (
            ('accepted_status', self.accepted_status_topic, ACCEPTED_STATUS_TYPE),
            ('candidate', self.candidate_topic, OBJECTS_TYPE),
            ('canonical_tracked', self.canonical_tracked_topic, OBJECTS_TYPE),
        ):
            endpoints = self.get_publishers_info_by_topic(topic)
            protected_outputs[name] = {
                'expected_type': expected_type,
                'publisher_cardinality': len(endpoints),
                'publishers': [self._endpoint_summary(endpoint) for endpoint in endpoints],
                'topic': topic,
            }

        node_name = self.get_name()
        node_namespace = self.get_namespace()
        self_entities = {
            'clients': self._name_type_summary(
                self.get_client_names_and_types_by_node(node_name, node_namespace)
            ),
            'publishers': self._name_type_summary(
                self.get_publisher_names_and_types_by_node(node_name, node_namespace)
            ),
            'services': self._name_type_summary(
                self.get_service_names_and_types_by_node(node_name, node_namespace)
            ),
            'subscriptions': self._name_type_summary(
                self.get_subscriber_names_and_types_by_node(node_name, node_namespace)
            ),
        }
        graph_nodes = sorted(
            f"{namespace.rstrip('/')}/{name}" if namespace != '/' else f'/{name}'
            for name, namespace in self.get_node_names_and_namespaces()
        )
        violations = []
        if self_entities['publishers']:
            violations.append('recorder_has_steady_state_publishers')
        if self_entities['services']:
            violations.append('recorder_has_services')
        if self_entities['clients']:
            violations.append('recorder_has_clients')
        expected_subscriptions = {self.objects_topic, self.diagnostics_topic}
        observed_subscriptions = {
            entry['name'] for entry in self_entities['subscriptions']
        }
        if observed_subscriptions != expected_subscriptions:
            violations.append('recorder_subscription_set_mismatch')
        for source in ('objects', 'diagnostics'):
            subscriptions = topics[source]['subscriptions']
            if len(subscriptions) != 1:
                violations.append(f'{source}_subscriber_cardinality_not_one')
                continue
            if subscriptions[0]['node_fqn'] != self.get_fully_qualified_name():
                violations.append(f'{source}_subscriber_is_not_recorder')
        for name in ('accepted_status', 'candidate'):
            if protected_outputs[name]['publisher_cardinality'] != 0:
                violations.append(f'{name}_publisher_present')
        if any(
            node.rsplit('/', 1)[-1] == FORBIDDEN_INTEGRATION_NODE_NAME
            for node in graph_nodes
        ):
            violations.append('vils_integration_node_present')
        if any(
            endpoint['node_fqn'].rsplit('/', 1)[-1] == FORBIDDEN_INTEGRATION_NODE_NAME
            for endpoint in protected_outputs['canonical_tracked']['publishers']
        ):
            violations.append('vils_canonical_tracked_publisher_present')
        return {
            'event': 'graph_snapshot',
            'gid_semantics': (
                'Graph endpoint GIDs are observations at snapshot time and are not '
                'per-message writer attribution.'
            ),
            'monotonic_ns': time.monotonic_ns(),
            'phase': phase,
            'protected_outputs': protected_outputs,
            'recorder_entities': self_entities,
            'stage2_graph_invariant': {
                'passed': not violations,
                'violations': violations,
            },
            'topics': topics,
            'wall_ns': time.time_ns(),
        }

    # HH_260810 - Fail the capture when Stage-3 consumers or outputs appear during Stage 2.
    @staticmethod
    def require_stage2_graph(snapshot: dict[str, Any]) -> None:
        violations = snapshot['stage2_graph_invariant']['violations']
        if violations:
            raise RuntimeError(
                'Stage-2 graph invariant violation: ' + ','.join(violations)
            )

    # HH_260810 - Mark an inter-message source absence after its threshold is crossed.
    def update_absence_state(self, now_monotonic_ns: int) -> None:
        for state in self.states.values():
            if state.last_receive_monotonic_ns is None:
                continue
            if state.absence_open_ns is None and (
                now_monotonic_ns - state.last_receive_monotonic_ns
                >= self.absence_threshold_ns
            ):
                state.absence_open_ns = state.last_receive_monotonic_ns

    # HH_260810 - Close final no-message tails and leave exact intervals in the event stream.
    def finish_absence_intervals(self, end_monotonic_ns: int) -> None:
        for state in self.states.values():
            if state.last_receive_monotonic_ns is not None and state.absence_open_ns is None:
                if (
                    end_monotonic_ns - state.last_receive_monotonic_ns
                    >= self.absence_threshold_ns
                ):
                    state.absence_open_ns = state.last_receive_monotonic_ns
            absence = state.close_absence(
                end_monotonic_ns,
                self.absence_threshold_ns,
                'recorder_stopped',
            )
            if absence is not None:
                self.evidence.append_event(absence)


# HH_260810 - Require explicit bounded evidence settings and reject ambiguous topic names.
def parse_arguments(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Subscribe-only PC2 recorder for PC4 VILS Stage-2 raw evidence.'
    )
    parser.add_argument('--output-dir', required=True, type=Path)
    parser.add_argument('--duration', type=float, default=30.0)
    parser.add_argument('--graph-interval', type=float, default=1.0)
    parser.add_argument('--source-absence-threshold', type=float, default=2.0)
    parser.add_argument('--objects-topic', default=DEFAULT_OBJECTS_TOPIC)
    parser.add_argument('--diagnostics-topic', default=DEFAULT_DIAGNOSTICS_TOPIC)
    parser.add_argument('--accepted-status-topic', default=DEFAULT_ACCEPTED_STATUS_TOPIC)
    parser.add_argument('--candidate-topic', default=DEFAULT_CANDIDATE_TOPIC)
    parser.add_argument('--canonical-tracked-topic', default=DEFAULT_CANONICAL_TRACKED_TOPIC)
    arguments = parser.parse_args(argv)
    if not math.isfinite(arguments.duration) or arguments.duration < 0.0:
        parser.error('--duration must be finite and >= 0; zero means run until SIGINT/SIGTERM')
    if not math.isfinite(arguments.graph_interval) or arguments.graph_interval <= 0.0:
        parser.error('--graph-interval must be finite and > 0')
    if (
        not math.isfinite(arguments.source_absence_threshold)
        or arguments.source_absence_threshold <= 0.0
    ):
        parser.error('--source-absence-threshold must be finite and > 0')
    for option, topic in (
        ('--objects-topic', arguments.objects_topic),
        ('--diagnostics-topic', arguments.diagnostics_topic),
        ('--accepted-status-topic', arguments.accepted_status_topic),
        ('--candidate-topic', arguments.candidate_topic),
        ('--canonical-tracked-topic', arguments.canonical_tracked_topic),
    ):
        if not topic.startswith('/') or topic == '/' or any(char.isspace() for char in topic):
            parser.error(f'{option} must be an absolute ROS topic without whitespace')
    if arguments.objects_topic == arguments.diagnostics_topic:
        parser.error('object and diagnostic topics must be distinct')
    protected_topics = {
        arguments.accepted_status_topic,
        arguments.candidate_topic,
        arguments.canonical_tracked_topic,
    }
    if len(protected_topics) != 3 or protected_topics & {
        arguments.objects_topic,
        arguments.diagnostics_topic,
    }:
        parser.error('input and protected output topics must all be distinct')
    arguments.output_dir = Path(os.path.abspath(arguments.output_dir.expanduser()))
    return arguments


# HH_260810 - Build a hashed, reviewable run contract before DDS discovery begins.
def build_manifest(arguments: argparse.Namespace) -> dict[str, Any]:
    script_path = Path(__file__).resolve()
    config = {
        'accepted_status_topic': arguments.accepted_status_topic,
        'candidate_topic': arguments.candidate_topic,
        'canonical_tracked_topic': arguments.canonical_tracked_topic,
        'diagnostics_topic': arguments.diagnostics_topic,
        'duration_sec': arguments.duration,
        'graph_interval_sec': arguments.graph_interval,
        'objects_topic': arguments.objects_topic,
        'qos': {
            'depth': 1,
            'durability': 'VOLATILE',
            'history': 'KEEP_LAST',
            'reliability': 'RELIABLE',
        },
        'source_absence_threshold_sec': arguments.source_absence_threshold,
    }
    return {
        'config': config,
        'config_sha256': hashlib.sha256(canonical_json_bytes(config)).hexdigest(),
        'created_wall_ns': time.time_ns(),
        'environment': {
            'hostname': socket.gethostname(),
            'rmw_implementation': os.environ.get('RMW_IMPLEMENTATION', ''),
            'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', ''),
            'ros_localhost_only': os.environ.get('ROS_LOCALHOST_ONLY', ''),
        },
        'identity_limitations': (
            'Raw SHA-256 covers the exact serialized bytes delivered to the rclpy callback. '
            'Those same callback bytes are stored in the raw_cdr rosbag2 stream. '
            'Graph endpoint GIDs are periodic observations and are not per-message writer '
            'attribution. Middleware loss callbacks and diagnostic sequence gaps are partial, '
            'separately labelled drop indicators.'
        ),
        'output_dir': str(arguments.output_dir),
        'raw_payload_storage': {
            'format': 'rosbag2/sqlite3',
            'relative_uri': 'raw_cdr',
            'serialization_format': 'cdr',
        },
        'steady_state_ros_contract': {
            'actions': 0,
            'parameter_mutations': 0,
            'publishers': 0,
            'services': 0,
            'subscriptions': 2,
        },
        'transient_entity_note': (
            'Humble rclpy constructs an implicit parameter-events publisher during Node '
            'initialization; the recorder destroys it before its first graph snapshot. '
            'Stage-2 acceptance depends on every recorded steady-state graph invariant passing.'
        ),
        'remote_plan_commit': 'ac12565e03e166138a684e5e95d03b59a3cbba50',
        'schema': SCHEMA_VERSION,
        'script_path': str(script_path),
        'script_sha256': hashlib.sha256(script_path.read_bytes()).hexdigest(),
    }


# HH_260810 - Run until the bounded duration or a clean operator signal, then seal a summary.
def run(arguments: argparse.Namespace) -> int:
    old_umask = os.umask(0o077)
    evidence = ExclusiveEvidenceDirectory(arguments.output_dir)
    node: Optional[Stage2RawRecorder] = None
    raw_bag: Optional[RawCdrBagWriter] = None
    sealed_rosbag_files: list[dict[str, Any]] = []
    rclpy_started = False
    stop_event = threading.Event()
    stop_reason = 'duration_elapsed'
    prior_signal_handlers: dict[int, Any] = {}
    started_wall_ns = time.time_ns()
    started_monotonic_ns = time.monotonic_ns()
    clean_shutdown = False
    complete_marker_written = False
    fatal_error: Optional[str] = None

    # HH_260810 - Keep signal handlers side-effect free; the main loop writes the evidence.
    def request_stop(signum: int, _frame: Any) -> None:
        nonlocal stop_reason
        stop_reason = signal.Signals(signum).name.lower()
        stop_event.set()

    try:
        evidence.create()
        manifest = build_manifest(arguments)
        evidence.write_json_file('manifest.json', manifest)
        evidence.append_event(
            {
                'config_sha256': manifest['config_sha256'],
                'event': 'recorder_started',
                'monotonic_ns': started_monotonic_ns,
                'wall_ns': started_wall_ns,
            }
        )
        rclpy.init(args=None)
        rclpy_started = True
        raw_bag = RawCdrBagWriter(
            arguments.output_dir / 'raw_cdr',
            arguments.objects_topic,
            arguments.diagnostics_topic,
        )
        node = Stage2RawRecorder(
            evidence,
            raw_bag,
            arguments.objects_topic,
            arguments.diagnostics_topic,
            arguments.accepted_status_topic,
            arguments.candidate_topic,
            arguments.canonical_tracked_topic,
            started_monotonic_ns,
            int(arguments.source_absence_threshold * 1_000_000_000),
        )
        for handled_signal in (signal.SIGINT, signal.SIGTERM):
            prior_signal_handlers[handled_signal] = signal.getsignal(handled_signal)
            signal.signal(handled_signal, request_stop)
        graph = node.graph_snapshot('start')
        evidence.append_event(graph)
        node.require_stage2_graph(graph)
        next_graph_ns = time.monotonic_ns() + int(arguments.graph_interval * 1_000_000_000)
        deadline_ns = (
            started_monotonic_ns + int(arguments.duration * 1_000_000_000)
            if arguments.duration > 0.0
            else None
        )
        while not stop_event.is_set():
            now_ns = time.monotonic_ns()
            if deadline_ns is not None and now_ns >= deadline_ns:
                stop_reason = 'duration_elapsed'
                break
            rclpy.spin_once(node, timeout_sec=0.05)
            now_ns = time.monotonic_ns()
            node.update_absence_state(now_ns)
            if now_ns >= next_graph_ns:
                graph = node.graph_snapshot('periodic')
                evidence.append_event(graph)
                node.require_stage2_graph(graph)
                next_graph_ns = now_ns + int(arguments.graph_interval * 1_000_000_000)
        ended_monotonic_ns = time.monotonic_ns()
        ended_wall_ns = time.time_ns()
        node.update_absence_state(ended_monotonic_ns)
        graph = node.graph_snapshot('end')
        evidence.append_event(graph)
        node.require_stage2_graph(graph)
        node.finish_absence_intervals(ended_monotonic_ns)
        raw_bag.close()
        sealed_rosbag_files = evidence.seal_rosbag_tree('raw_cdr')
        clean_shutdown = True
    except Exception as error:  # noqa: B902
        fatal_error = f'{type(error).__name__}: {error}'
        if evidence.events_fd >= 0:
            try:
                evidence.append_event(
                    {
                        'error': fatal_error,
                        'event': 'recorder_fatal_error',
                        'monotonic_ns': time.monotonic_ns(),
                        'wall_ns': time.time_ns(),
                    }
                )
            except Exception:
                pass
    finally:
        ended_monotonic_ns = time.monotonic_ns()
        ended_wall_ns = time.time_ns()
        for handled_signal, prior_handler in prior_signal_handlers.items():
            signal.signal(handled_signal, prior_handler)
        if raw_bag is not None and raw_bag.writer is not None:
            try:
                raw_bag.close()
            except Exception as error:
                clean_shutdown = False
                if fatal_error is None:
                    fatal_error = f'{type(error).__name__}: {error}'
        raw_bag_path = arguments.output_dir / 'raw_cdr'
        if (
            evidence.directory_fd >= 0
            and not sealed_rosbag_files
            and raw_bag_path.exists()
        ):
            try:
                sealed_rosbag_files = evidence.seal_rosbag_tree('raw_cdr')
            except Exception as error:
                clean_shutdown = False
                if fatal_error is None:
                    fatal_error = f'{type(error).__name__}: {error}'
        states = node.states if node is not None else {}
        if node is not None:
            try:
                node.destroy_node()
                node = None
            except Exception as error:
                clean_shutdown = False
                if fatal_error is None:
                    fatal_error = f'{type(error).__name__}: {error}'
        if rclpy_started and rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception as error:
                clean_shutdown = False
                if fatal_error is None:
                    fatal_error = f'{type(error).__name__}: {error}'
        summary = {
            'clean_shutdown': clean_shutdown,
            'completion_marker_required': True,
            'duration_ns': max(0, ended_monotonic_ns - started_monotonic_ns),
            'ended_monotonic_ns': ended_monotonic_ns,
            'ended_wall_ns': ended_wall_ns,
            'fatal_error': fatal_error,
            'schema': SCHEMA_VERSION,
            'rosbag2': {
                'format': 'sqlite3',
                'sealed_files': sealed_rosbag_files,
                'serialization_format': 'cdr',
                'write_count': raw_bag.write_count if raw_bag is not None else 0,
            },
            'sources': {name: state.summary() for name, state in states.items()},
            'started_monotonic_ns': started_monotonic_ns,
            'started_wall_ns': started_wall_ns,
            'stop_reason': stop_reason,
        }
        summary_written = False
        if evidence.directory_fd >= 0:
            try:
                evidence.write_json_file('summary.json', summary)
                summary_written = True
            except Exception as error:
                print(f'failed to seal summary: {error}', file=sys.stderr)
                clean_shutdown = False
            try:
                evidence.close_events()
            except Exception as error:
                print(f'failed to close event evidence: {error}', file=sys.stderr)
                clean_shutdown = False
            if summary_written:
                try:
                    root_paths = ['events.jsonl', 'manifest.json', 'summary.json']
                    root_paths.extend(
                        item['path'] for item in sealed_rosbag_files
                    )
                    root_checksums, checksums_sha256 = evidence.write_root_checksums(
                        root_paths
                    )
                    if clean_shutdown:
                        evidence.write_json_file(
                            'COMPLETE.json',
                            {
                                'checksums_file': 'checksums.json',
                                'checksums_sha256': checksums_sha256,
                                'clean_shutdown': True,
                                'file_count': len(root_checksums),
                                'schema': SCHEMA_VERSION,
                            },
                        )
                        complete_marker_written = True
                except Exception as error:
                    print(f'failed to seal evidence root: {error}', file=sys.stderr)
                    clean_shutdown = False
        evidence.close()
        os.umask(old_umask)
    return 0 if clean_shutdown and complete_marker_written else 1


# HH_260810 - Print only actionable failures; run data stays in the protected evidence directory.
def main(argv: Optional[list[str]] = None) -> int:
    try:
        arguments = parse_arguments(sys.argv[1:] if argv is None else argv)
        return run(arguments)
    except Exception as error:  # noqa: B902
        message = f'PC2 VILS Stage-2 recorder failed: {type(error).__name__}: {error}'
        print(message, file=sys.stderr)
        return 1


# HH_260810 - Keep direct execution explicit for operator and standalone synthetic use.
if __name__ == '__main__':
    raise SystemExit(main())
