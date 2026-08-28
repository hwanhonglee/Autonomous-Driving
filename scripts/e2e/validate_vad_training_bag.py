#!/usr/bin/env python3
"""Validate a ROS 2 bag against the real-vehicle VAD capture contract."""

from __future__ import annotations

import argparse
from bisect import bisect_left, bisect_right
from collections import defaultdict
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

import numpy as np
import yaml

try:
    from vad_training_data_contract import CAMERA_ORDER
    from vad_training_data_contract import TopicSpec
    from vad_training_data_contract import find_placeholders
    from vad_training_data_contract import load_profile
    from vad_training_data_contract import unique_topic_specs
    from vad_calibration_contract import analyze_tf_tree
    from vad_calibration_contract import calibration_path_from_profile
    from vad_calibration_contract import expected_camera_pose
    from vad_calibration_contract import load_calibration
    from vad_calibration_contract import pose_error
    from vad_calibration_contract import quaternion_transform
    from vad_commissioning_contract import load_commissioning
    from vad_commissioning_contract import load_profile_commissioning
except ModuleNotFoundError:  # Imported as scripts.e2e.* by the root pytest suite.
    from scripts.e2e.vad_training_data_contract import CAMERA_ORDER
    from scripts.e2e.vad_training_data_contract import TopicSpec
    from scripts.e2e.vad_training_data_contract import find_placeholders
    from scripts.e2e.vad_training_data_contract import load_profile
    from scripts.e2e.vad_training_data_contract import unique_topic_specs
    from scripts.e2e.vad_calibration_contract import analyze_tf_tree
    from scripts.e2e.vad_calibration_contract import calibration_path_from_profile
    from scripts.e2e.vad_calibration_contract import expected_camera_pose
    from scripts.e2e.vad_calibration_contract import load_calibration
    from scripts.e2e.vad_calibration_contract import pose_error
    from scripts.e2e.vad_calibration_contract import quaternion_transform
    from scripts.e2e.vad_commissioning_contract import load_commissioning
    from scripts.e2e.vad_commissioning_contract import load_profile_commissioning


GATE_NAMES = (
    "raw_capture",
    "runtime_replay",
    "planning_finetune_labels_present",
    "full_vad_multitask_labels_present",
)


def stamp_to_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def percentile(values: Iterable[float], fraction: float) -> float | None:
    ordered = sorted(values)
    if not ordered:
        return None
    if len(ordered) == 1:
        return float(ordered[0])
    position = (len(ordered) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return float(ordered[lower])
    weight = position - lower
    return float(ordered[lower] * (1.0 - weight) + ordered[upper] * weight)


def nearest(stamps: Sequence[int], target: int) -> int | None:
    position = bisect_left(stamps, target)
    candidates = stamps[max(0, position - 1) : min(len(stamps), position + 1)]
    if not candidates:
        return None
    return min(candidates, key=lambda value: (abs(value - target), value))


def sequence_timing(stamps_ns: Sequence[int]) -> dict[str, Any]:
    positive = [value for value in stamps_ns if value > 0]
    gaps = [right - left for left, right in zip(stamps_ns, stamps_ns[1:])]
    positive_gaps = [value for value in gaps if value > 0]
    unique = sorted(set(positive))
    span_ns = unique[-1] - unique[0] if len(unique) > 1 else 0
    return {
        "count": len(stamps_ns),
        "unique_count": len(set(stamps_ns)),
        "zero_stamp_count": len(stamps_ns) - len(positive),
        "duplicate_or_nonmonotonic_count": sum(value <= 0 for value in gaps),
        "first_stamp_ns": unique[0] if unique else None,
        "last_stamp_ns": unique[-1] if unique else None,
        "duration_s": span_ns / 1e9,
        "rate_hz": (len(unique) - 1) / (span_ns / 1e9) if span_ns > 0 else 0.0,
        "p95_gap_ms": (
            percentile([value / 1e6 for value in positive_gaps], 0.95)
            if positive_gaps
            else None
        ),
        "p99_gap_ms": (
            percentile([value / 1e6 for value in positive_gaps], 0.99)
            if positive_gaps
            else None
        ),
        "maximum_gap_ms": max(positive_gaps) / 1e6 if positive_gaps else None,
    }


def anchored_bundle_coverage(
    stamps_by_camera: Mapping[str, Sequence[int]], tolerance_ms: float
) -> dict[str, Any]:
    front = sorted(set(stamps_by_camera.get("CAM_FRONT", ())))
    sorted_by_camera = {
        name: sorted(set(stamps_by_camera.get(name, ()))) for name in CAMERA_ORDER
    }
    spans_ms: list[float] = []
    tolerance_ns = int(tolerance_ms * 1e6)
    for anchor in front:
        bundle = [anchor]
        for camera in CAMERA_ORDER[1:]:
            match = nearest(sorted_by_camera[camera], anchor)
            if match is None or abs(match - anchor) > tolerance_ns:
                break
            bundle.append(match)
        if len(bundle) == len(CAMERA_ORDER):
            spans_ms.append((max(bundle) - min(bundle)) / 1e6)
    return {
        "tolerance_ms": tolerance_ms,
        "matched_bundle_count": len(spans_ms),
        "front_frame_count": len(front),
        "coverage_percent": 100.0 * len(spans_ms) / len(front) if front else 0.0,
        "p95_skew_ms": percentile(spans_ms, 0.95),
        "p99_skew_ms": percentile(spans_ms, 0.99),
        "maximum_skew_ms": max(spans_ms) if spans_ms else None,
    }


def alignment_summary(anchors: Sequence[int], targets: Sequence[int], tolerance_ms: float) -> dict[str, Any]:
    target_stamps = sorted(set(targets))
    deltas_ms: list[float] = []
    for anchor in sorted(set(anchors)):
        match = nearest(target_stamps, anchor)
        if match is not None:
            deltas_ms.append(abs(match - anchor) / 1e6)
    within = sum(value <= tolerance_ms for value in deltas_ms)
    anchor_count = len(set(anchors))
    return {
        "tolerance_ms": tolerance_ms,
        "anchor_count": anchor_count,
        "matched_count": len(deltas_ms),
        "within_tolerance_count": within,
        "coverage_percent": 100.0 * within / anchor_count if anchor_count else 0.0,
        "p95_delta_ms": percentile(deltas_ms, 0.95),
        "p99_delta_ms": percentile(deltas_ms, 0.99),
        "maximum_delta_ms": max(deltas_ms) if deltas_ms else None,
    }


def future_window_count(
    anchors: Sequence[int], odometry_stamps: Sequence[int], horizon_s: float, step_s: float
) -> int:
    odometry = sorted(set(odometry_stamps))
    tolerance_ns = int(min(0.1, step_s * 0.25) * 1e9)
    offsets_ns = [int(index * step_s * 1e9) for index in range(1, round(horizon_s / step_s) + 1)]
    valid = 0
    for anchor in sorted(set(anchors)):
        if all(
            (match := nearest(odometry, anchor + offset)) is not None
            and abs(match - (anchor + offset)) <= tolerance_ns
            for offset in offsets_ns
        ):
            valid += 1
    return valid


def command_alignment_summary(
    image_bag_stamps: Sequence[int],
    command_samples: Sequence[tuple[int, int]],
    maximum_age_ms: float,
) -> dict[str, Any]:
    ordered = sorted(command_samples)
    command_times = [stamp for stamp, _ in ordered]
    valid_values = {0, 1, 2, 3, 4, 5}
    aligned_ages_ms: list[float] = []
    valid_aligned = 0
    for image_stamp in sorted(set(image_bag_stamps)):
        position = bisect_left(command_times, image_stamp)
        if position < len(command_times) and command_times[position] == image_stamp:
            match_index = position
        else:
            match_index = position - 1
        if match_index < 0:
            continue
        command_stamp, value = ordered[match_index]
        age_ms = (image_stamp - command_stamp) / 1e6
        if 0.0 <= age_ms <= maximum_age_ms and value in valid_values:
            valid_aligned += 1
            aligned_ages_ms.append(age_ms)
    image_count = len(set(image_bag_stamps))
    return {
        "image_count": image_count,
        "command_count": len(command_samples),
        "valid_value_count": sum(value in valid_values for _, value in command_samples),
        "invalid_values": sorted({value for _, value in command_samples if value not in valid_values}),
        "aligned_count": valid_aligned,
        "coverage_percent": 100.0 * valid_aligned / image_count if image_count else 0.0,
        "maximum_age_ms": max(aligned_ages_ms) if aligned_ages_ms else None,
        "p99_age_ms": percentile(aligned_ages_ms, 0.99),
    }


def causal_status_alignment(
    image_bag_stamps: Sequence[int], status_stamps: Sequence[int], maximum_age_ms: float
) -> dict[str, Any]:
    """Measure whether every image has a recent status published no later than it."""
    ordered_status = sorted(set(status_stamps))
    ages_ms: list[float] = []
    for image_stamp in sorted(set(image_bag_stamps)):
        position = bisect_right(ordered_status, image_stamp) - 1
        if position < 0:
            continue
        age_ms = (image_stamp - ordered_status[position]) / 1e6
        if age_ms <= maximum_age_ms:
            ages_ms.append(age_ms)
    image_count = len(set(image_bag_stamps))
    return {
        "image_count": image_count,
        "status_count": len(status_stamps),
        "unique_status_count": len(ordered_status),
        "aligned_count": len(ages_ms),
        "coverage_percent": 100.0 * len(ages_ms) / image_count if image_count else 0.0,
        "maximum_age_ms": max(ages_ms) if ages_ms else None,
        "p99_age_ms": percentile(ages_ms, 0.99),
    }


def transport_delay_summary(
    header_stamps: Sequence[int], bag_stamps: Sequence[int], maximum_delay_ms: float
) -> dict[str, Any]:
    delays_ms = [
        (bag_stamp - header_stamp) / 1e6
        for header_stamp, bag_stamp in zip(header_stamps, bag_stamps)
    ]
    valid = [delay for delay in delays_ms if 0.0 <= delay <= maximum_delay_ms]
    return {
        "sample_count": len(delays_ms),
        "stamp_count_match": len(header_stamps) == len(bag_stamps),
        "negative_delay_count": sum(delay < 0.0 for delay in delays_ms),
        "over_limit_count": sum(delay > maximum_delay_ms for delay in delays_ms),
        "coverage_percent": 100.0 * len(valid) / len(delays_ms) if delays_ms else 0.0,
        "minimum_delay_ms": min(delays_ms) if delays_ms else None,
        "p99_delay_ms": percentile(delays_ms, 0.99),
        "maximum_delay_ms": max(delays_ms) if delays_ms else None,
    }


def exposure_binding_summary(
    exposure_counters: Sequence[int],
    exposure_stamps: Sequence[int],
    status_bag_stamps: Sequence[int],
    image_header_stamps: Sequence[int],
    image_bag_stamps: Sequence[int],
    maximum_status_delta_ms: float,
) -> dict[str, Any]:
    counter_monotonic = all(
        right > left for left, right in zip(exposure_counters, exposure_counters[1:])
    )
    count_match = (
        len(exposure_counters)
        == len(exposure_stamps)
        == len(status_bag_stamps)
        == len(image_header_stamps)
        == len(image_bag_stamps)
    )
    stamp_order_match = list(exposure_stamps) == list(image_header_stamps)
    status_deltas_ms = [
        abs(status_stamp - image_stamp) / 1e6
        for status_stamp, image_stamp in zip(status_bag_stamps, image_bag_stamps)
    ]
    status_delta_pass = bool(status_deltas_ms) and all(
        delta <= maximum_status_delta_ms for delta in status_deltas_ms
    )
    return {
        "pass": count_match and counter_monotonic and stamp_order_match and status_delta_pass,
        "counter_count": len(exposure_counters),
        "counter_monotonic": counter_monotonic,
        "stamp_count": len(exposure_stamps),
        "image_header_stamp_order_match": stamp_order_match,
        "pairing_count_match": count_match,
        "maximum_status_delta_ms": max(status_deltas_ms) if status_deltas_ms else None,
        "status_delta_pass": status_delta_pass,
    }


def diagnostic_key_values(message: Any) -> tuple[dict[str, str], list[str]]:
    values: dict[str, str] = {}
    duplicates: set[str] = set()
    for item in message.values:
        key = str(item.key)
        if key in values:
            duplicates.add(key)
        values[key] = str(item.value)
    return values, sorted(duplicates)


def uint8_value(value: Any) -> int:
    if isinstance(value, (bytes, bytearray)):
        if len(value) != 1:
            raise ValueError("uint8 diagnostic value must contain exactly one byte")
        return value[0]
    return int(value)


def count_imbalance_percent(counts: Sequence[int]) -> float:
    if not counts or max(counts) == 0:
        return 100.0
    return 100.0 * (max(counts) - min(counts)) / max(counts)


def graph_has_path(edges: Iterable[tuple[str, str]], start: str, goal: str) -> bool:
    adjacency: dict[str, set[str]] = defaultdict(set)
    for left, right in edges:
        adjacency[left].add(right)
        adjacency[right].add(left)
    pending = [start]
    visited: set[str] = set()
    while pending:
        node = pending.pop()
        if node == goal:
            return True
        if node in visited:
            continue
        visited.add(node)
        pending.extend(adjacency[node] - visited)
    return False


def qos_profiles(raw: Any) -> list[Mapping[str, Any]]:
    if not raw:
        return []
    if isinstance(raw, list):
        return [item for item in raw if isinstance(item, Mapping)]
    if isinstance(raw, str):
        try:
            parsed = yaml.safe_load(raw)
        except yaml.YAMLError:
            return []
        return [item for item in parsed or [] if isinstance(item, Mapping)]
    return []


def qos_has_policy(raw: Any, policy: str, accepted: set[Any]) -> bool:
    return any(profile.get(policy) in accepted for profile in qos_profiles(raw))


def read_metadata(bag: Path) -> tuple[dict[str, Any], dict[str, dict[str, Any]]]:
    metadata_path = bag / "metadata.yaml"
    if not metadata_path.is_file():
        raise ValueError(f"metadata.yaml not found in bag: {bag}")
    document = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
    information = document.get("rosbag2_bagfile_information", {})
    topics: dict[str, dict[str, Any]] = {}
    for entry in information.get("topics_with_message_count", []):
        topic_metadata = entry.get("topic_metadata", {})
        name = topic_metadata.get("name")
        if not name:
            continue
        topics[name] = {
            "type": topic_metadata.get("type"),
            "serialization_format": topic_metadata.get("serialization_format"),
            "offered_qos_profiles": topic_metadata.get("offered_qos_profiles"),
            "message_count": int(entry.get("message_count", 0)),
        }
    return information, topics


def required_topic_report(
    specs: Sequence[TopicSpec], metadata_topics: Mapping[str, Mapping[str, Any]]
) -> dict[str, Any]:
    missing: list[str] = []
    empty: list[str] = []
    mismatched: list[dict[str, Any]] = []
    optional_present: list[str] = []
    optional_missing: list[str] = []
    optional_empty: list[str] = []
    optional_mismatched: list[dict[str, Any]] = []
    for spec in specs:
        actual = metadata_topics.get(spec.topic)
        if not spec.required_for_capture:
            if actual is None:
                optional_missing.append(spec.topic)
            elif actual.get("type") != spec.message_type:
                optional_mismatched.append(
                    {
                        "topic": spec.topic,
                        "expected": spec.message_type,
                        "actual": actual.get("type"),
                    }
                )
            elif int(actual.get("message_count", 0)) <= 0:
                optional_empty.append(spec.topic)
            else:
                optional_present.append(spec.topic)
            continue
        if actual is None:
            missing.append(spec.topic)
            continue
        if actual.get("type") != spec.message_type:
            mismatched.append(
                {"topic": spec.topic, "expected": spec.message_type, "actual": actual.get("type")}
            )
        if int(actual.get("message_count", 0)) <= 0:
            empty.append(spec.topic)
    return {
        "pass": not missing and not empty and not mismatched,
        "missing": sorted(missing),
        "empty": sorted(empty),
        "type_mismatches": mismatched,
        "optional_present": sorted(optional_present),
        "optional_missing": sorted(optional_missing),
        "optional_empty": sorted(optional_empty),
        "optional_type_mismatches": optional_mismatched,
    }


def calibration_signature(message: Any) -> str:
    data = {
        "width": int(message.width),
        "height": int(message.height),
        "distortion_model": str(message.distortion_model),
        "d": list(message.d),
        "k": list(message.k),
        "r": list(message.r),
        "p": list(message.p),
        "binning_x": int(message.binning_x),
        "binning_y": int(message.binning_y),
    }
    encoded = json.dumps(data, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def finite(values: Iterable[float]) -> bool:
    return all(math.isfinite(float(value)) for value in values)


def observe_bag(
    bag: Path, profile: Mapping[str, Any], metadata_topics: Mapping[str, Mapping[str, Any]]
) -> dict[str, Any]:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    camera_by_image = {camera["image_topic"]: camera for camera in profile["cameras"]}
    camera_by_info = {camera["camera_info_topic"]: camera for camera in profile["cameras"]}
    camera_by_device = {camera["device_info_topic"]: camera for camera in profile["cameras"]}
    runtime = profile["runtime_topics"]
    time_sync_topic = profile["capture_topics"]["time_sync_status"]["topic"]
    image_stamps: dict[str, list[int]] = {name: [] for name in CAMERA_ORDER}
    image_bag_stamps: dict[str, list[int]] = {name: [] for name in CAMERA_ORDER}
    image_dimensions: dict[str, set[tuple[int, int]]] = {name: set() for name in CAMERA_ORDER}
    image_encodings: dict[str, set[str]] = {name: set() for name in CAMERA_ORDER}
    image_frames: dict[str, set[str]] = {name: set() for name in CAMERA_ORDER}
    invalid_image_layout: dict[str, int] = {name: 0 for name in CAMERA_ORDER}
    camera_info: dict[str, dict[str, Any]] = {
        name: {
            "count": 0,
            "frames": set(),
            "dimensions": set(),
            "signatures": set(),
            "intrinsics": set(),
            "distortions": set(),
            "distortion_models": set(),
            "invalid_k_count": 0,
            "invalid_matrix_count": 0,
            "nonzero_distortion_count": 0,
        }
        for name in CAMERA_ORDER
    }
    camera_device_samples: dict[str, list[dict[str, Any]]] = {
        name: [] for name in CAMERA_ORDER
    }
    time_sync_samples: list[dict[str, Any]] = []
    odometry_stamps: list[int] = []
    acceleration_stamps: list[int] = []
    odometry_positions: list[tuple[float, float, float]] = []
    odometry_frames: set[str] = set()
    odometry_child_frames: set[str] = set()
    acceleration_frames: set[str] = set()
    invalid_odometry_count = 0
    invalid_acceleration_count = 0
    maximum_quaternion_norm_error = 0.0
    command_samples: list[tuple[int, int]] = []
    tf_edges: set[tuple[str, str]] = set()
    tf_signatures: dict[tuple[str, str], set[tuple[float, ...]]] = defaultdict(set)
    tf_transforms: list[tuple[str, str, np.ndarray]] = []
    invalid_tf_count = 0
    invalid_tf_edges: list[tuple[str, str]] = []
    deserialization_errors: list[dict[str, str]] = []

    selected_topics = set(camera_by_image) | set(camera_by_info) | set(camera_by_device)
    selected_topics.update(entry["topic"] for entry in runtime.values())
    selected_topics.add(time_sync_topic)
    command_entry = profile["label_topics"]["command"]
    selected_topics.add(command_entry["topic"])
    selected_topics &= set(metadata_topics)

    information, _ = read_metadata(bag)
    storage_id = information.get("storage_identifier", "sqlite3")
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id=storage_id),
        rosbag2_py.ConverterOptions("", ""),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=sorted(selected_topics)))
    message_classes = {
        topic: get_message(str(metadata_topics[topic]["type"])) for topic in selected_topics
    }

    while reader.has_next():
        topic, serialized, bag_stamp = reader.read_next()
        try:
            message = deserialize_message(serialized, message_classes[topic])
        except Exception as error:  # Keep auditing the rest of a partially corrupt capture.
            deserialization_errors.append({"topic": topic, "error": str(error)})
            continue

        if topic in camera_by_image:
            camera = camera_by_image[topic]
            name = camera["name"]
            image_stamps[name].append(stamp_to_ns(message.header.stamp))
            image_bag_stamps[name].append(int(bag_stamp))
            width, height = int(message.width), int(message.height)
            image_dimensions[name].add((width, height))
            image_encodings[name].add(str(message.encoding))
            image_frames[name].add(str(message.header.frame_id))
            channels = 3 if message.encoding == "bgr8" else 4 if message.encoding == "bgra8" else 0
            minimum_step = width * channels
            if channels == 0 or int(message.step) < minimum_step or len(message.data) < int(message.step) * height:
                invalid_image_layout[name] += 1
            continue

        if topic in camera_by_info:
            camera = camera_by_info[topic]
            name = camera["name"]
            summary = camera_info[name]
            summary["count"] += 1
            summary["frames"].add(str(message.header.frame_id))
            summary["dimensions"].add((int(message.width), int(message.height)))
            summary["signatures"].add(calibration_signature(message))
            summary["intrinsics"].add(tuple(float(value) for value in message.k))
            summary["distortions"].add(tuple(float(value) for value in message.d))
            summary["distortion_models"].add(str(message.distortion_model))
            matrix_values = (*message.k, *message.r, *message.p, *message.d)
            if not finite(matrix_values):
                summary["invalid_matrix_count"] += 1
            if (
                not finite(message.k)
                or float(message.k[0]) <= 0
                or float(message.k[4]) <= 0
                or abs(float(message.k[8]) - 1.0) > 1e-9
                or not 0 <= float(message.k[2]) < int(message.width)
                or not 0 <= float(message.k[5]) < int(message.height)
                or abs(float(message.k[1])) > 1e-9
                or abs(float(message.k[3])) > 1e-9
            ):
                summary["invalid_k_count"] += 1
            if not finite(message.d) or any(abs(float(value)) > 1e-9 for value in message.d):
                summary["nonzero_distortion_count"] += 1
            continue

        if topic in camera_by_device:
            camera = camera_by_device[topic]
            values, duplicate_keys = diagnostic_key_values(message)
            camera_device_samples[camera["name"]].append(
                {
                    "bag_stamp_ns": int(bag_stamp),
                    "level": uint8_value(message.level),
                    "name": str(message.name),
                    "message": str(message.message),
                    "hardware_id": str(message.hardware_id),
                    "values": values,
                    "duplicate_keys": duplicate_keys,
                }
            )
            continue

        if topic == time_sync_topic:
            values, duplicate_keys = diagnostic_key_values(message)
            time_sync_samples.append(
                {
                    "bag_stamp_ns": int(bag_stamp),
                    "level": uint8_value(message.level),
                    "name": str(message.name),
                    "message": str(message.message),
                    "hardware_id": str(message.hardware_id),
                    "values": values,
                    "duplicate_keys": duplicate_keys,
                }
            )
            continue

        if topic == runtime["kinematic_state"]["topic"]:
            odometry_stamps.append(stamp_to_ns(message.header.stamp))
            pose = message.pose.pose
            twist = message.twist.twist
            values = (
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
                twist.linear.x,
                twist.linear.y,
                twist.linear.z,
                twist.angular.x,
                twist.angular.y,
                twist.angular.z,
            )
            if not finite(values):
                invalid_odometry_count += 1
            quaternion_norm = math.sqrt(sum(float(value) ** 2 for value in values[3:7]))
            maximum_quaternion_norm_error = max(
                maximum_quaternion_norm_error, abs(quaternion_norm - 1.0)
            )
            odometry_positions.append(
                (float(pose.position.x), float(pose.position.y), float(pose.position.z))
            )
            odometry_frames.add(str(message.header.frame_id))
            odometry_child_frames.add(str(message.child_frame_id))
            continue

        if topic == runtime["acceleration"]["topic"]:
            acceleration_stamps.append(stamp_to_ns(message.header.stamp))
            acceleration_frames.add(str(message.header.frame_id))
            linear = message.accel.accel.linear
            angular = message.accel.accel.angular
            if not finite((linear.x, linear.y, linear.z, angular.x, angular.y, angular.z)):
                invalid_acceleration_count += 1
            continue

        if topic == runtime["tf_static"]["topic"]:
            for transform in message.transforms:
                parent = str(transform.header.frame_id).lstrip("/")
                child = str(transform.child_frame_id).lstrip("/")
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                values = (
                    float(translation.x),
                    float(translation.y),
                    float(translation.z),
                    float(rotation.x),
                    float(rotation.y),
                    float(rotation.z),
                    float(rotation.w),
                )
                norm = math.sqrt(sum(value * value for value in values[3:]))
                if not parent or not child or not finite(values) or abs(norm - 1.0) > 1e-3:
                    invalid_tf_count += 1
                    invalid_tf_edges.append((parent, child))
                    continue
                edge = (parent, child)
                tf_edges.add(edge)
                tf_signatures[edge].add(tuple(round(value, 12) for value in values))
                tf_transforms.append((parent, child, quaternion_transform(values[:3], values[3:])))
            continue

        if topic == command_entry["topic"]:
            command_samples.append((int(bag_stamp), int(message.data)))

    path_distance_m = sum(
        math.dist(left, right) for left, right in zip(odometry_positions, odometry_positions[1:])
    )
    return {
        "image_stamps": image_stamps,
        "image_bag_stamps": image_bag_stamps,
        "image_dimensions": image_dimensions,
        "image_encodings": image_encodings,
        "image_frames": image_frames,
        "invalid_image_layout": invalid_image_layout,
        "camera_info": camera_info,
        "camera_device_samples": camera_device_samples,
        "time_sync_samples": time_sync_samples,
        "odometry_stamps": odometry_stamps,
        "acceleration_stamps": acceleration_stamps,
        "odometry_path_distance_m": path_distance_m,
        "odometry_frames": odometry_frames,
        "odometry_child_frames": odometry_child_frames,
        "acceleration_frames": acceleration_frames,
        "invalid_odometry_count": invalid_odometry_count,
        "invalid_acceleration_count": invalid_acceleration_count,
        "maximum_quaternion_norm_error": maximum_quaternion_norm_error,
        "command_samples": command_samples,
        "tf_edges": tf_edges,
        "tf_signatures": tf_signatures,
        "tf_transforms": tf_transforms,
        "invalid_tf_count": invalid_tf_count,
        "invalid_tf_edges": invalid_tf_edges,
        "deserialization_errors": deserialization_errors,
    }


def gate(name: str, checks: Mapping[str, bool], note: str) -> dict[str, Any]:
    return {"name": name, "pass": all(checks.values()), "checks": dict(checks), "note": note}


def build_report(
    bag: Path,
    profile: Mapping[str, Any],
    calibration: Mapping[str, Any],
    commissioning: Mapping[str, Any],
) -> dict[str, Any]:
    information, metadata_topics = read_metadata(bag)
    specs = unique_topic_specs(profile)
    required_topics = required_topic_report(specs, metadata_topics)
    observed = observe_bag(bag, profile, metadata_topics)
    validation = profile["validation"]
    model = profile["model_contract"]
    calibration_by_name = {camera["name"]: camera for camera in calibration["cameras"]}

    camera_reports: dict[str, Any] = {}
    camera_capture_checks: list[bool] = []
    camera_calibration_checks: list[bool] = []
    camera_device_checks: list[bool] = []
    image_counts: list[int] = []
    for camera in profile["cameras"]:
        name = camera["name"]
        timing = sequence_timing(observed["image_stamps"][name])
        image_counts.append(timing["count"])
        expected_dimensions = {(int(model["input_width"]), int(model["input_height"]))}
        expected_frames = {camera["optical_frame"]}
        expected_encodings = {model["input_encoding"]}
        expected_calibration = calibration_by_name[name]
        transport_delay = transport_delay_summary(
            observed["image_stamps"][name],
            observed["image_bag_stamps"][name],
            float(validation["maximum_image_transport_delay_ms"]),
        )
        image_check = all(
            (
                timing["count"] > 0,
                timing["zero_stamp_count"] == 0,
                timing["duplicate_or_nonmonotonic_count"] == 0,
                timing["rate_hz"] >= float(validation["minimum_camera_rate_hz"]),
                timing["p99_gap_ms"] is not None
                and timing["p99_gap_ms"] <= float(validation["maximum_p99_gap_ms"]),
                observed["image_dimensions"][name] == expected_dimensions,
                observed["image_encodings"][name] == expected_encodings,
                observed["image_frames"][name] == expected_frames,
                observed["invalid_image_layout"][name] == 0,
                transport_delay["stamp_count_match"],
                transport_delay["coverage_percent"] == 100.0,
            )
        )
        info = observed["camera_info"][name]
        intrinsic_match = (
            len(info["intrinsics"]) == 1
            and np.allclose(
                np.asarray(next(iter(info["intrinsics"]))),
                np.asarray(expected_calibration["k"]),
                rtol=0.0,
                atol=float(validation["maximum_intrinsic_error"]),
            )
        )
        distortion_match = (
            len(info["distortions"]) == 1
            and len(next(iter(info["distortions"]))) == len(expected_calibration["d"])
            and np.allclose(
                np.asarray(next(iter(info["distortions"]))),
                np.asarray(expected_calibration["d"]),
                rtol=0.0,
                atol=1e-9,
            )
            and info["distortion_models"] == {expected_calibration["distortion_model"]}
        )
        info_check = all(
            (
                info["count"] > 0,
                info["frames"] == expected_frames,
                info["dimensions"] == expected_dimensions,
                len(info["signatures"]) == 1,
                info["invalid_k_count"] == 0,
                info["invalid_matrix_count"] == 0,
                not validation["require_zero_distortion"]
                or info["nonzero_distortion_count"] == 0,
                intrinsic_match,
                distortion_match,
                expected_calibration["optical_frame"] == camera["optical_frame"],
            )
        )
        device_samples = observed["camera_device_samples"][name]
        expected_device_values = {
            "camera_name": name,
            "serial": expected_calibration["serial"],
            "image_topic": camera["image_topic"],
            "optical_frame": camera["optical_frame"],
            "timestamp_source": expected_calibration["timestamp_source"],
            "trigger_mode": expected_calibration["trigger_mode"],
            "firmware": str(profile["provenance"]["camera_firmware"]),
            "exposure_mode": str(profile["provenance"]["exposure_mode"]),
            "phc_clock_source": str(profile["provenance"]["ptp_clock_source"]),
        }
        invalid_device_samples = 0
        exposure_counters: list[int] = []
        exposure_stamps: list[int] = []
        for sample in device_samples:
            try:
                exposure_counter = int(sample["values"].get("exposure_counter", ""), 10)
                exposure_stamp = int(sample["values"].get("exposure_stamp_ns", ""), 10)
            except ValueError:
                exposure_counter = -1
                exposure_stamp = -1
            sample_valid = (
                sample["level"] == 0
                and sample["hardware_id"] == expected_calibration["serial"]
                and not sample["duplicate_keys"]
                and all(
                    sample["values"].get(key) == expected
                    for key, expected in expected_device_values.items()
                )
                and exposure_counter >= 0
                and exposure_stamp > 0
            )
            invalid_device_samples += not sample_valid
            if exposure_counter >= 0:
                exposure_counters.append(exposure_counter)
            if exposure_stamp > 0:
                exposure_stamps.append(exposure_stamp)
        image_header_stamps = observed["image_stamps"][name]
        image_bag_stamps = observed["image_bag_stamps"][name]
        exposure_binding = exposure_binding_summary(
            exposure_counters,
            exposure_stamps,
            [sample["bag_stamp_ns"] for sample in device_samples],
            image_header_stamps,
            image_bag_stamps,
            float(validation["maximum_device_status_delta_ms"]),
        )
        device_check = all(
            (
                bool(device_samples),
                invalid_device_samples == 0,
                len(exposure_counters) == len(device_samples),
                exposure_binding["pass"],
            )
        )
        camera_capture_checks.append(image_check)
        camera_calibration_checks.append(info_check)
        camera_device_checks.append(device_check)
        camera_reports[name] = {
            "model_index": camera["model_index"],
            "image_topic": camera["image_topic"],
            "camera_info_topic": camera["camera_info_topic"],
            "timing": timing,
            "image_dimensions": [list(item) for item in sorted(observed["image_dimensions"][name])],
            "image_encodings": sorted(observed["image_encodings"][name]),
            "image_frames": sorted(observed["image_frames"][name]),
            "invalid_image_layout_count": observed["invalid_image_layout"][name],
            "transport_delay": transport_delay,
            "image_pass": image_check,
            "camera_info": {
                "count": info["count"],
                "frames": sorted(info["frames"]),
                "dimensions": [list(item) for item in sorted(info["dimensions"])],
                "calibration_hashes": sorted(info["signatures"]),
                "invalid_k_count": info["invalid_k_count"],
                "invalid_matrix_count": info["invalid_matrix_count"],
                "nonzero_distortion_count": info["nonzero_distortion_count"],
                "intrinsic_manifest_match": intrinsic_match,
                "distortion_manifest_match": distortion_match,
                "pass": info_check,
            },
            "device_info": {
                "topic": camera["device_info_topic"],
                "expected_serial": expected_calibration["serial"],
                "observed_hardware_ids": sorted(
                    {sample["hardware_id"] for sample in device_samples}
                ),
                "observed_values": {
                    key: sorted(
                        {
                            sample["values"][key]
                            for sample in device_samples
                            if key in sample["values"]
                        }
                    )
                    for key in expected_device_values
                },
                "sample_count": len(device_samples),
                "invalid_sample_count": invalid_device_samples,
                "exposure_binding": exposure_binding,
                "pass": device_check,
            },
        }

    runtime_sync = anchored_bundle_coverage(
        observed["image_stamps"], float(validation["runtime_sync_tolerance_ms"])
    )
    dataset_sync = anchored_bundle_coverage(
        observed["image_stamps"], float(validation["dataset_sync_tolerance_ms"])
    )
    front_stamps = observed["image_stamps"]["CAM_FRONT"]
    odometry_timing = sequence_timing(observed["odometry_stamps"])
    acceleration_timing = sequence_timing(observed["acceleration_stamps"])
    odometry_alignment = alignment_summary(
        front_stamps,
        observed["odometry_stamps"],
        float(validation["maximum_state_delta_ms"]),
    )
    acceleration_alignment = alignment_summary(
        front_stamps,
        observed["acceleration_stamps"],
        float(validation["maximum_state_delta_ms"]),
    )
    state_pass = all(
        (
            odometry_timing["zero_stamp_count"] == 0,
            odometry_timing["duplicate_or_nonmonotonic_count"] == 0,
            acceleration_timing["zero_stamp_count"] == 0,
            acceleration_timing["duplicate_or_nonmonotonic_count"] == 0,
            odometry_timing["rate_hz"] >= float(validation["minimum_state_rate_hz"]),
            acceleration_timing["rate_hz"] >= float(validation["minimum_state_rate_hz"]),
            odometry_alignment["coverage_percent"]
            >= float(validation["minimum_bundle_coverage_percent"]),
            acceleration_alignment["coverage_percent"]
            >= float(validation["minimum_bundle_coverage_percent"]),
            observed["invalid_odometry_count"] == 0,
            observed["invalid_acceleration_count"] == 0,
            observed["maximum_quaternion_norm_error"] <= 1e-3,
            observed["odometry_frames"] == {model["map_frame"]},
            observed["odometry_child_frames"] == {model["base_frame"]},
            observed["acceleration_frames"] == {model["acceleration_frame"]},
        )
    )

    tf_tree = analyze_tf_tree(
        observed["tf_transforms"],
        model["base_frame"],
        [camera["optical_frame"] for camera in profile["cameras"]],
    )
    optical_paths = {
        camera["name"]: bool(tf_tree["target_paths"].get(camera["optical_frame"], False))
        for camera in profile["cameras"]
    }
    extrinsic_errors: dict[str, dict[str, float | bool | None]] = {}
    extrinsics_match = True
    for camera in profile["cameras"]:
        name = camera["name"]
        actual = tf_tree["poses_in_root"].get(camera["optical_frame"])
        if actual is None:
            translation_error = None
            rotation_error = None
            matches = False
        else:
            translation_error, rotation_error = pose_error(
                actual, expected_camera_pose(calibration_by_name[name])
            )
            matches = (
                translation_error
                <= float(validation["maximum_extrinsic_translation_error_m"])
                and rotation_error <= float(validation["maximum_extrinsic_rotation_error_deg"])
            )
        extrinsic_errors[name] = {
            "translation_error_m": translation_error,
            "rotation_error_deg": rotation_error,
            "manifest_match": matches,
        }
        extrinsics_match = extrinsics_match and matches
    # Quaternion q and -q encode the same rotation. The tree analyzer compares
    # normalized transform matrices, so equivalent signs remain stable.
    stable_tf = not tf_tree["conflicting_value_edges"]
    relevant_tf_nodes = set(tf_tree["relevant_nodes"])
    relevant_invalid_tf_count = sum(
        child in relevant_tf_nodes for _parent, child in observed["invalid_tf_edges"]
    )
    tf_pass = (
        bool(tf_tree["valid"])
        and all(optical_paths.values())
        and extrinsics_match
        and stable_tf
        and relevant_invalid_tf_count == 0
        and bool(observed["tf_edges"])
    )

    image_qos = {
        camera["name"]: qos_has_policy(
            metadata_topics.get(camera["image_topic"], {}).get("offered_qos_profiles"),
            "reliability",
            {1, "1", "RELIABLE", "RMW_QOS_POLICY_RELIABILITY_RELIABLE"},
        )
        for camera in profile["cameras"]
    }
    tf_static_topic = profile["runtime_topics"]["tf_static"]["topic"]
    tf_static_transient_local = qos_has_policy(
        metadata_topics.get(tf_static_topic, {}).get("offered_qos_profiles"),
        "durability",
        {1, "1", "TRANSIENT_LOCAL", "RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL"},
    )
    qos_runtime_pass = (
        (not validation["require_reliable_images"] or all(image_qos.values()))
        and (
            not validation["require_transient_local_tf_static"] or tf_static_transient_local
        )
    )

    duration_s = odometry_timing["duration_s"]
    imbalance = count_imbalance_percent(image_counts)
    future_windows = future_window_count(
        front_stamps,
        observed["odometry_stamps"],
        float(validation["future_horizon_s"]),
        float(validation["future_step_s"]),
    )
    command_alignment = command_alignment_summary(
        observed["image_bag_stamps"]["CAM_FRONT"],
        observed["command_samples"],
        float(validation["maximum_command_age_ms"]),
    )
    command_topic = profile["label_topics"]["command"]["topic"]
    object_topic = profile["label_topics"]["object_ground_truth"]["topic"]
    map_topic = profile["label_topics"]["vector_map_ground_truth"]["topic"]
    label_counts = {
        "command": int(metadata_topics.get(command_topic, {}).get("message_count", 0)),
        "object_ground_truth": int(metadata_topics.get(object_topic, {}).get("message_count", 0)),
        "vector_map_ground_truth": int(metadata_topics.get(map_topic, {}).get("message_count", 0)),
    }

    expected_clock_source = str(profile["provenance"]["ptp_clock_source"])
    valid_lock_values = {"1", "true", "yes", "locked"}
    ptp_offsets_ns: list[float] = []
    invalid_time_sync_samples = 0
    for sample in observed["time_sync_samples"]:
        raw_offset = sample["values"].get("offset_ns")
        try:
            offset_ns = float(raw_offset)
        except (TypeError, ValueError):
            offset_ns = math.nan
        if math.isfinite(offset_ns):
            ptp_offsets_ns.append(offset_ns)
        sample_valid = all(
            (
                sample["level"] == 0,
                sample["hardware_id"] == expected_clock_source,
                not sample["duplicate_keys"],
                sample["values"].get("clock_source") == expected_clock_source,
                sample["values"].get("timestamp_source") == "hardware_exposure",
                sample["values"].get("locked", "").strip().lower()
                in valid_lock_values,
                math.isfinite(offset_ns),
                abs(offset_ns) <= float(validation["maximum_ptp_offset_ns"]),
            )
        )
        invalid_time_sync_samples += not sample_valid
    time_sync_freshness = causal_status_alignment(
        observed["image_bag_stamps"]["CAM_FRONT"],
        [sample["bag_stamp_ns"] for sample in observed["time_sync_samples"]],
        float(validation["maximum_status_age_ms"]),
    )
    time_sync_pass = all(
        (
            bool(observed["time_sync_samples"]),
            invalid_time_sync_samples == 0,
            time_sync_freshness["coverage_percent"] == 100.0,
        )
    )
    commissioning_approved = all(
        (
            commissioning.get("approved") is True,
            commissioning.get("projection_preview_approved") is True,
            commissioning.get("calibration_sha256") == calibration.get("_sha256"),
            isinstance(commissioning.get("_sha256"), str),
            isinstance(commissioning.get("_rectification_config_sha256"), str),
            set(commissioning.get("_evidence_files", {}))
            == {
                "checkerboard_evidence",
                "reprojection_report",
                "projection_preview",
                "device_inventory",
                "device_status_adapter",
                "device_status_adapter_config",
            },
        )
    )

    raw_checks = {
        "profile_provenance_resolved": not find_placeholders(profile),
        "required_topics": bool(required_topics["pass"]),
        "minimum_duration": duration_s >= float(validation["minimum_duration_s"]),
        "six_camera_images": all(camera_capture_checks),
        "camera_info_stable_and_rectified": all(camera_calibration_checks),
        "camera_device_inventory": all(camera_device_checks),
        "hardware_exposure_clock": time_sync_pass,
        "rectification_commissioning_approved": commissioning_approved,
        "camera_count_balance": imbalance <= float(validation["maximum_count_imbalance_percent"]),
        "dataset_camera_synchronization": dataset_sync["coverage_percent"]
        >= float(validation["minimum_bundle_coverage_percent"]),
        "ego_state": state_pass,
        "base_to_camera_tf": tf_pass,
        "deserialization": not observed["deserialization_errors"],
    }
    raw_gate = gate(
        "raw_capture",
        raw_checks,
        "Checks raw synchronized sensor/state capture only; it does not verify annotation semantics.",
    )
    runtime_gate = gate(
        "runtime_replay",
        {
            "raw_capture": raw_gate["pass"],
            "strict_camera_synchronization": runtime_sync["coverage_percent"]
            >= float(validation["minimum_bundle_coverage_percent"]),
            "runtime_qos": qos_runtime_pass,
        },
        "Matches the current recommended VAD transport and strict frame-assembly contract.",
    )
    planning_gate = gate(
        "planning_finetune_labels_present",
        {
            "raw_capture": raw_gate["pass"],
            "command_labels_present": label_counts["command"] > 0,
            "command_values_valid": not command_alignment["invalid_values"],
            "command_camera_coverage": command_alignment["coverage_percent"]
            >= float(validation["minimum_command_coverage_percent"]),
            "valid_future_ego_windows": future_windows > 0,
            "minimum_ego_motion": observed["odometry_path_distance_m"]
            >= float(validation["minimum_motion_distance_m"]),
        },
        "Presence gate only. Human/expert quality, route split leakage, and label conversion remain unverified.",
    )
    multitask_gate = gate(
        "full_vad_multitask_labels_present",
        {
            "planning_labels_present": planning_gate["pass"],
            "object_track_labels_present": label_counts["object_ground_truth"] > 0,
            "vector_map_labels_present": label_counts["vector_map_ground_truth"] > 0,
        },
        "Presence gate only. Object tracks and vector-map annotation correctness remain unverified.",
    )

    relative_paths = information.get("relative_file_paths", [])
    bag_files = []
    for relative_path in relative_paths:
        path = bag / relative_path
        bag_files.append(
            {"path": relative_path, "size_bytes": path.stat().st_size if path.is_file() else None}
        )
    return {
        "schema_version": 1,
        "bag": str(bag),
        "profile": profile.get("_profile_path"),
        "profile_sha256": profile.get("_profile_sha256"),
        "model_contract": model,
        "calibration_manifest": {
            "path": calibration.get("_path"),
            "sha256": calibration.get("_sha256"),
        },
        "commissioning_manifest": {
            "path": commissioning.get("_path"),
            "sha256": commissioning.get("_sha256"),
            "approved_by": commissioning.get("approved_by"),
            "approved_utc": commissioning.get("approved_utc"),
            "valid_until_utc": commissioning.get("valid_until_utc"),
            "expired_at_validation": commissioning.get("_expired"),
            "checkerboard_dataset_sha256": commissioning.get(
                "checkerboard_dataset_sha256"
            ),
            "rectification_config_path": commissioning.get(
                "_rectification_config_path"
            ),
            "rectification_config_sha256": commissioning.get(
                "_rectification_config_sha256"
            ),
            "mean_reprojection_error_px": commissioning.get(
                "mean_reprojection_error_px"
            ),
            "maximum_reprojection_error_px": commissioning.get(
                "maximum_reprojection_error_px"
            ),
            "projection_preview": commissioning.get("_projection_preview"),
            "evidence_files": commissioning.get("_evidence_files"),
        },
        "metadata": {
            "storage_identifier": information.get("storage_identifier"),
            "duration_s": float(information.get("duration", {}).get("nanoseconds", 0)) / 1e9,
            "message_count": int(information.get("message_count", 0)),
            "compression_mode": information.get("compression_mode"),
            "compression_format": information.get("compression_format"),
            "files": bag_files,
        },
        "required_topics": required_topics,
        "cameras": camera_reports,
        "synchronization": {"runtime": runtime_sync, "dataset": dataset_sync},
        "camera_count_imbalance_percent": imbalance,
        "time_sync": {
            "pass": time_sync_pass,
            "topic": profile["capture_topics"]["time_sync_status"]["topic"],
            "expected_clock_source": expected_clock_source,
            "sample_count": len(observed["time_sync_samples"]),
            "invalid_sample_count": invalid_time_sync_samples,
            "maximum_absolute_offset_ns": (
                max(abs(value) for value in ptp_offsets_ns) if ptp_offsets_ns else None
            ),
            "freshness": time_sync_freshness,
        },
        "ego_state": {
            "pass": state_pass,
            "odometry_timing": odometry_timing,
            "acceleration_timing": acceleration_timing,
            "odometry_alignment": odometry_alignment,
            "acceleration_alignment": acceleration_alignment,
            "odometry_frames": sorted(observed["odometry_frames"]),
            "odometry_child_frames": sorted(observed["odometry_child_frames"]),
            "acceleration_frames": sorted(observed["acceleration_frames"]),
            "path_distance_m": observed["odometry_path_distance_m"],
            "maximum_quaternion_norm_error": observed["maximum_quaternion_norm_error"],
            "invalid_odometry_count": observed["invalid_odometry_count"],
            "invalid_acceleration_count": observed["invalid_acceleration_count"],
        },
        "calibration_tf": {
            "pass": tf_pass,
            "base_frame": model["base_frame"],
            "optical_frame_paths": optical_paths,
            "edge_count": len(observed["tf_edges"]),
            "stable": stable_tf,
            "tree_valid": bool(tf_tree["valid"]),
            "conflicting_parent_children": tf_tree["conflicting_parent_children"],
            "conflicting_value_edges": tf_tree["conflicting_value_edges"],
            "cycle": tf_tree["cycle"],
            "extrinsic_manifest_errors": extrinsic_errors,
            "invalid_transform_count": observed["invalid_tf_count"],
            "relevant_invalid_transform_count": relevant_invalid_tf_count,
        },
        "qos": {
            "runtime_pass": qos_runtime_pass,
            "reliable_camera_publishers": image_qos,
            "tf_static_transient_local": tf_static_transient_local,
        },
        "labels": {
            "message_counts": label_counts,
            "command_values": sorted({value for _, value in observed["command_samples"]}),
            "command_alignment": command_alignment,
            "future_ego_window_count": future_windows,
            "semantic_quality_verified": False,
        },
        "deserialization_errors": observed["deserialization_errors"],
        "gates": {
            "raw_capture": raw_gate,
            "runtime_replay": runtime_gate,
            "planning_finetune_labels_present": planning_gate,
            "full_vad_multitask_labels_present": multitask_gate,
        },
    }


def write_json_atomic(path: Path, report: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(path)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument(
        "--profile",
        type=Path,
        default=Path(__file__).resolve().parents[2]
        / "autoware_e2e_vad_launch/config/vad_real_data_collection.yaml",
    )
    parser.add_argument("--output", type=Path)
    parser.add_argument("--calibration", type=Path)
    parser.add_argument("--commissioning", type=Path)
    parser.add_argument("--rectification-config", type=Path)
    parser.add_argument("--require-gate", choices=GATE_NAMES, default="raw_capture")
    parser.add_argument("--print-full", action="store_true")
    args = parser.parse_args()

    try:
        bag = args.bag.expanduser().resolve()
        profile = load_profile(args.profile)
        calibration = load_calibration(
            args.calibration.expanduser().resolve()
            if args.calibration
            else calibration_path_from_profile(profile)
        )
        if args.commissioning is None and args.rectification_config is None:
            commissioning = load_profile_commissioning(profile, calibration)
        elif args.commissioning is not None and args.rectification_config is not None:
            commissioning = load_commissioning(
                args.commissioning.expanduser().resolve(),
                calibration,
                args.rectification_config.expanduser().resolve(),
                profile,
            )
        else:
            raise ValueError(
                "--commissioning and --rectification-config must be provided together"
            )
        report = build_report(bag, profile, calibration, commissioning)
    except (OSError, RuntimeError, ValueError) as error:
        parser.error(str(error))

    if args.output:
        write_json_atomic(args.output.expanduser().resolve(), report)
    passed = bool(report["gates"][args.require_gate]["pass"])
    summary = {
        "pass": passed,
        "required_gate": args.require_gate,
        "gates": {name: bool(value["pass"]) for name, value in report["gates"].items()},
        "dataset_bundle_coverage_percent": report["synchronization"]["dataset"][
            "coverage_percent"
        ],
        "runtime_bundle_coverage_percent": report["synchronization"]["runtime"][
            "coverage_percent"
        ],
        "future_ego_window_count": report["labels"]["future_ego_window_count"],
        "output": str(args.output.expanduser().resolve()) if args.output else None,
    }
    print(json.dumps(report if args.print_full else summary, indent=2, sort_keys=True))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
