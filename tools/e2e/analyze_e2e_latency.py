#!/usr/bin/env python3
"""Measure VAD-to-control latency and trajectory age from a ROS 2 bag.

Receipt timestamps and ROS message stamps are kept as separate clock domains.
The analyzer never subtracts one from the other.  Sensor-to-VAD attribution is
necessarily inferred because the current VAD output does not retain its input
camera stamp; trajectory-to-controller age is causally matched and exact.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
from pathlib import Path
from typing import Any, Sequence

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError as error:  # pragma: no cover - requires sourced ROS
    raise SystemExit(
        "ROS 2 Python modules are unavailable. Source scripts/e2e/env.sh first."
    ) from error


CAMERAS = (
    "CAM_FRONT",
    "CAM_BACK",
    "CAM_FRONT_LEFT",
    "CAM_BACK_LEFT",
    "CAM_FRONT_RIGHT",
    "CAM_BACK_RIGHT",
)
CAMERA_INFO_TOPICS = tuple(f"/sensing/camera/{name}/camera_info" for name in CAMERAS)
SENSOR_TOPIC_CANDIDATES = (
    "/sensing/camera/CAM_FRONT/camera_info",
    "/sensing/camera/CAM_FRONT/image_stamp",
    "/sensing/camera/CAM_FRONT/image_raw",
    "/sensing/camera/CAM_FRONT/image_raw/compressed",
)
VAD_OUTPUT_TOPICS = (
    "/planning/vad/candidate_trajectories",
    "/planning/vad/raw_trajectory",
    "/planning/vad_route/selected_raw_trajectory",
)
FINAL_TOPIC = "/planning/trajectory"
CONTROL_TOPICS = (
    "/control/trajectory_follower/control_cmd",
    "/trajectory_follower/control_cmd",
    "/control/command/control_cmd",
)
ACTUATION_TOPIC = "/control/command/actuation_cmd"
PROCESSING_TIME_TOPIC = (
    "/control/trajectory_follower/controller_node_exe/lateral/debug/processing_time_ms"
)
ODOMETRY_TOPIC = "/localization/kinematic_state"
ACCELERATION_TOPIC = "/localization/acceleration"

CSV_FIELDS = (
    "stage",
    "pair_policy",
    "upstream_topic",
    "downstream_topic",
    "upstream_index",
    "downstream_index",
    "downstream_relative_wall_sec",
    "receipt_latency_sec",
    "stamp_delta_sec",
)


class LatencyError(RuntimeError):
    pass


def _time_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def message_stamp_ns(message: Any) -> tuple[int, str]:
    """Extract a physics/simulation stamp and explain where it came from."""
    header = getattr(message, "header", None)
    if header is not None and hasattr(header, "stamp"):
        return _time_ns(header.stamp), "header.stamp"
    stamp = getattr(message, "stamp", None)
    if stamp is not None and hasattr(stamp, "sec"):
        return _time_ns(stamp), "stamp"
    candidates = getattr(message, "candidate_trajectories", None)
    if candidates:
        stamps = [_time_ns(candidate.header.stamp) for candidate in candidates]
        nonzero = [value for value in stamps if value > 0]
        if nonzero:
            return nonzero[0], "candidate_trajectories[0].header.stamp"
    clock = getattr(message, "clock", None)
    if clock is not None and hasattr(clock, "sec"):
        return _time_ns(clock), "clock"
    return 0, "unavailable"


def _storage_identifier(bag: Path) -> str:
    metadata_path = bag / "metadata.yaml"
    if not metadata_path.is_file():
        raise LatencyError(f"rosbag metadata does not exist: {metadata_path}")
    with metadata_path.open(encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream)
    information = metadata.get("rosbag2_bagfile_information", {})
    storage_id = information.get("storage_identifier")
    if not storage_id:
        raise LatencyError(f"storage_identifier is missing from {metadata_path}")
    return str(storage_id)


def select_first_available(
    available: Sequence[str] | set[str], candidates: Sequence[str]
) -> str | None:
    return next((topic for topic in candidates if topic in available), None)


def read_bag(
    bag: Path, sensor_topic_override: str | None = None
) -> tuple[dict[str, list[dict[str, Any]]], dict[str, str], dict[str, str | None]]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id=_storage_identifier(bag)),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    available = set(topic_types)
    if sensor_topic_override is not None and sensor_topic_override not in available:
        raise LatencyError(f"requested sensor topic is not in bag: {sensor_topic_override}")
    sensor_topic = sensor_topic_override or select_first_available(
        available, SENSOR_TOPIC_CANDIDATES
    )
    vad_topic = select_first_available(available, VAD_OUTPUT_TOPICS)
    control_topic = select_first_available(available, CONTROL_TOPICS)
    selected = {
        "sensor": sensor_topic,
        "vad_output": vad_topic,
        "final_trajectory": FINAL_TOPIC if FINAL_TOPIC in available else None,
        "control": control_topic,
        "actuation": ACTUATION_TOPIC if ACTUATION_TOPIC in available else None,
        "processing_time": PROCESSING_TIME_TOPIC if PROCESSING_TIME_TOPIC in available else None,
    }
    requested = {
        topic
        for topic in (
            *CAMERA_INFO_TOPICS,
            sensor_topic,
            vad_topic,
            selected["final_trajectory"],
            control_topic,
            selected["actuation"],
            selected["processing_time"],
            ODOMETRY_TOPIC if ODOMETRY_TOPIC in available else None,
            ACCELERATION_TOPIC if ACCELERATION_TOPIC in available else None,
        )
        if topic is not None and topic in available
    }
    if hasattr(reader, "set_filter"):
        reader.set_filter(rosbag2_py.StorageFilter(topics=sorted(requested)))
    classes = {topic: get_message(topic_types[topic]) for topic in requested}
    records: dict[str, list[dict[str, Any]]] = {topic: [] for topic in requested}
    while reader.has_next():
        topic, serialized, bag_ns = reader.read_next()
        if topic not in classes:
            continue
        message = deserialize_message(serialized, classes[topic])
        stamp_ns, stamp_source = message_stamp_ns(message)
        record: dict[str, Any] = {
            "bag_ns": int(bag_ns),
            "stamp_ns": int(stamp_ns),
            "stamp_source": stamp_source,
        }
        if topic == PROCESSING_TIME_TOPIC:
            record["value"] = float(message.data)
        records[topic].append(record)
    for values in records.values():
        values.sort(key=lambda record: record["bag_ns"])
        for index, record in enumerate(values):
            record["index"] = index
    return records, topic_types, selected


def summary(values: Sequence[float]) -> dict[str, Any]:
    array = np.asarray(values, dtype=float)
    array = array[np.isfinite(array)]
    if len(array) == 0:
        return {"available": False, "count": 0}
    return {
        "available": True,
        "count": int(len(array)),
        "min": float(np.min(array)),
        "mean": float(np.mean(array)),
        "median": float(np.median(array)),
        "p95": float(np.percentile(array, 95)),
        "p99": float(np.percentile(array, 99)),
        "max": float(np.max(array)),
    }


def event_rate(records: Sequence[dict[str, Any]]) -> dict[str, Any]:
    receipt = np.asarray([record["bag_ns"] for record in records], dtype=float) * 1.0e-9
    stamp = np.asarray(
        [record["stamp_ns"] for record in records if record["stamp_ns"] > 0], dtype=float
    ) * 1.0e-9
    receipt_period = np.diff(receipt)
    stamp_period = np.diff(stamp)
    receipt_period = receipt_period[receipt_period > 0.0]
    stamp_period = stamp_period[stamp_period > 0.0]
    return {
        "count": len(records),
        "receipt_period_sec": summary(receipt_period),
        "stamp_period_sec": summary(stamp_period),
        "receipt_rate_hz": (
            float(1.0 / np.median(receipt_period)) if len(receipt_period) else None
        ),
        "effective_receipt_rate_hz": (
            float(1.0 / np.mean(receipt_period)) if len(receipt_period) else None
        ),
        "stamp_rate_hz": float(1.0 / np.median(stamp_period)) if len(stamp_period) else None,
    }


def candidate_front_acceptance_metrics(
    candidate_topic: str | None,
    front_topic: str | None,
    candidates: Sequence[dict[str, Any]],
    front_frames: Sequence[dict[str, Any]],
) -> dict[str, Any]:
    front_count = len(front_frames)
    candidate_count = len(candidates)
    return {
        "available": candidate_topic is not None and front_topic is not None and front_count > 0,
        "candidate_topic": candidate_topic,
        "front_topic": front_topic,
        "candidate_count": candidate_count,
        "front_count": front_count,
        "acceptance_percent": (
            100.0 * candidate_count / front_count if front_count > 0 else None
        ),
        "note": "candidate output count divided by recorded front-camera input count",
    }


def latest_prior_pairs(
    upstream: Sequence[dict[str, Any]],
    downstream: Sequence[dict[str, Any]],
    max_receipt_gap_sec: float | None = None,
) -> list[tuple[int, int]]:
    """Pair every downstream event with the latest causally received upstream event."""
    upstream_receipts = [record["bag_ns"] for record in upstream]
    maximum_ns = (
        int(round(max_receipt_gap_sec * 1.0e9)) if max_receipt_gap_sec is not None else None
    )
    pairs = []
    for downstream_index, record in enumerate(downstream):
        upstream_index = bisect.bisect_right(upstream_receipts, record["bag_ns"]) - 1
        if upstream_index < 0:
            continue
        gap_ns = record["bag_ns"] - upstream[upstream_index]["bag_ns"]
        if maximum_ns is not None and gap_ns > maximum_ns:
            continue
        pairs.append((upstream_index, downstream_index))
    return pairs


def first_after_pairs(
    upstream: Sequence[dict[str, Any]],
    downstream: Sequence[dict[str, Any]],
    max_receipt_gap_sec: float | None = None,
) -> list[tuple[int, int]]:
    """Pair every upstream update with the first downstream event received after it."""
    downstream_receipts = [record["bag_ns"] for record in downstream]
    maximum_ns = (
        int(round(max_receipt_gap_sec * 1.0e9)) if max_receipt_gap_sec is not None else None
    )
    pairs = []
    for upstream_index, record in enumerate(upstream):
        downstream_index = bisect.bisect_left(downstream_receipts, record["bag_ns"])
        if downstream_index >= len(downstream):
            continue
        gap_ns = downstream[downstream_index]["bag_ns"] - record["bag_ns"]
        if maximum_ns is not None and gap_ns > maximum_ns:
            continue
        pairs.append((upstream_index, downstream_index))
    return pairs


def samples_from_pairs(
    stage: str,
    policy: str,
    upstream_topic: str,
    downstream_topic: str,
    upstream: Sequence[dict[str, Any]],
    downstream: Sequence[dict[str, Any]],
    pairs: Sequence[tuple[int, int]],
    bag_start_ns: int,
) -> list[dict[str, Any]]:
    output = []
    for upstream_index, downstream_index in pairs:
        source = upstream[upstream_index]
        target = downstream[downstream_index]
        stamp_delta = None
        if source["stamp_ns"] > 0 and target["stamp_ns"] > 0:
            stamp_delta = (target["stamp_ns"] - source["stamp_ns"]) * 1.0e-9
        output.append(
            {
                "stage": stage,
                "pair_policy": policy,
                "upstream_topic": upstream_topic,
                "downstream_topic": downstream_topic,
                "upstream_index": upstream_index,
                "downstream_index": downstream_index,
                "downstream_relative_wall_sec": (target["bag_ns"] - bag_start_ns) * 1.0e-9,
                "receipt_latency_sec": (target["bag_ns"] - source["bag_ns"]) * 1.0e-9,
                "stamp_delta_sec": stamp_delta,
            }
        )
    return output


def stage_metrics(
    samples: Sequence[dict[str, Any]], upstream_count: int, downstream_count: int
) -> dict[str, Any]:
    receipt = [sample["receipt_latency_sec"] for sample in samples]
    stamp = [
        sample["stamp_delta_sec"]
        for sample in samples
        if sample["stamp_delta_sec"] is not None
    ]
    return {
        "available": bool(samples),
        "pair_count": len(samples),
        "upstream_count": upstream_count,
        "downstream_count": downstream_count,
        "upstream_coverage_percent": (
            100.0 * len({sample["upstream_index"] for sample in samples}) / upstream_count
            if upstream_count
            else 0.0
        ),
        "downstream_coverage_percent": (
            100.0 * len({sample["downstream_index"] for sample in samples}) / downstream_count
            if downstream_count
            else 0.0
        ),
        "receipt_latency_sec": summary(receipt),
        "stamp_delta_sec": summary(stamp),
        "negative_receipt_count": sum(value < 0.0 for value in receipt),
        "negative_stamp_count": sum(value < -1.0e-6 for value in stamp),
    }


def unavailable_stage(reason: str, upstream_count: int = 0, downstream_count: int = 0):
    return {
        "available": False,
        "reason": reason,
        "pair_count": 0,
        "upstream_count": upstream_count,
        "downstream_count": downstream_count,
        "receipt_latency_sec": summary([]),
        "stamp_delta_sec": summary([]),
    }


def freshness_metrics(
    samples: Sequence[dict[str, Any]], upstream_count: int = 0, downstream_count: int = 0
) -> dict[str, Any]:
    metrics = stage_metrics(samples, upstream_count, downstream_count or len(samples))
    for clock_name, key in (
        ("receipt", "receipt_latency_sec"),
        ("stamp", "stamp_delta_sec"),
    ):
        values = np.asarray(
            [sample[key] for sample in samples if sample[key] is not None], dtype=float
        )
        for threshold in (0.5, 1.0, 2.0):
            metrics[f"{clock_name}_older_than_{threshold:.1f}_sec_percent"] = (
                float(100.0 * np.mean(values > threshold)) if len(values) else None
            )
    return metrics


def camera_bundle_metrics(records: dict[str, list[dict[str, Any]]]) -> dict[str, Any]:
    available = {topic: records.get(topic, []) for topic in CAMERA_INFO_TOPICS}
    if not all(available.values()):
        return unavailable_stage(
            "all six camera_info topics are required for bundle skew",
            downstream_count=sum(bool(values) for values in available.values()),
        )
    stamped = {
        topic: [record for record in values if record["stamp_ns"] > 0]
        for topic, values in available.items()
    }
    stamp_arrays = {
        topic: [record["stamp_ns"] for record in values] for topic, values in stamped.items()
    }
    receipt_spans = []
    stamp_spans = []
    matched = 0
    tolerance_ns = 100_000_000
    for anchor in stamped[CAMERA_INFO_TOPICS[0]]:
        bundle = [anchor]
        for topic in CAMERA_INFO_TOPICS[1:]:
            stamps = stamp_arrays[topic]
            position = bisect.bisect_left(stamps, anchor["stamp_ns"])
            candidates = [index for index in (position - 1, position) if 0 <= index < len(stamps)]
            if not candidates:
                break
            nearest = min(candidates, key=lambda index: abs(stamps[index] - anchor["stamp_ns"]))
            if abs(stamps[nearest] - anchor["stamp_ns"]) > tolerance_ns:
                break
            bundle.append(stamped[topic][nearest])
        if len(bundle) != len(CAMERA_INFO_TOPICS):
            continue
        matched += 1
        receipts = [record["bag_ns"] for record in bundle]
        stamps = [record["stamp_ns"] for record in bundle]
        receipt_spans.append((max(receipts) - min(receipts)) * 1.0e-9)
        stamp_spans.append((max(stamps) - min(stamps)) * 1.0e-9)
    return {
        "available": bool(receipt_spans),
        "camera_count": len(CAMERA_INFO_TOPICS),
        "front_frame_count": len(stamped[CAMERA_INFO_TOPICS[0]]),
        "matched_bundle_count": matched,
        "bundle_coverage_percent": (
            100.0 * matched / len(stamped[CAMERA_INFO_TOPICS[0]])
            if stamped[CAMERA_INFO_TOPICS[0]]
            else 0.0
        ),
        "stamp_span_sec": summary(stamp_spans),
        "receipt_span_sec": summary(receipt_spans),
        "matching_tolerance_sec": tolerance_ns * 1.0e-9,
        "note": "camera_info is a lightweight timestamp proxy for the paired CARLA image",
    }


def nearest_stamp_delta(
    anchors: Sequence[dict[str, Any]], others: Sequence[dict[str, Any]]
) -> list[float]:
    stamped = [record for record in others if record["stamp_ns"] > 0]
    stamps = [record["stamp_ns"] for record in stamped]
    output = []
    for anchor in anchors:
        if anchor["stamp_ns"] <= 0 or not stamps:
            continue
        position = bisect.bisect_left(stamps, anchor["stamp_ns"])
        candidates = [index for index in (position - 1, position) if 0 <= index < len(stamps)]
        nearest = min(candidates, key=lambda index: abs(stamps[index] - anchor["stamp_ns"]))
        output.append(abs(stamps[nearest] - anchor["stamp_ns"]) * 1.0e-9)
    return output


def vad_input_sync_metrics(
    sensor: Sequence[dict[str, Any]],
    odometry: Sequence[dict[str, Any]],
    acceleration: Sequence[dict[str, Any]],
) -> dict[str, Any]:
    if not sensor:
        return {"available": False, "reason": "front-camera sensor timestamps were not recorded"}
    odometry_delta = nearest_stamp_delta(sensor, odometry)
    acceleration_delta = nearest_stamp_delta(sensor, acceleration)
    return {
        "available": bool(odometry_delta or acceleration_delta),
        "front_sensor_count": len(sensor),
        "camera_to_odometry_abs_stamp_delta_sec": summary(odometry_delta),
        "camera_to_acceleration_abs_stamp_delta_sec": summary(acceleration_delta),
        "note": "VAD synchronization is anchored on the front camera physics stamp",
    }


def chained_samples(
    sensor_topic: str,
    vad_topic: str,
    final_topic: str,
    control_topic: str,
    sensor: Sequence[dict[str, Any]],
    vad: Sequence[dict[str, Any]],
    final: Sequence[dict[str, Any]],
    control: Sequence[dict[str, Any]],
    sensor_vad_pairs: Sequence[tuple[int, int]],
    vad_final_pairs: Sequence[tuple[int, int]],
    final_control_pairs: Sequence[tuple[int, int]],
    bag_start_ns: int,
) -> list[dict[str, Any]]:
    sensor_by_vad = {vad_index: sensor_index for sensor_index, vad_index in sensor_vad_pairs}
    vad_by_final = {final_index: vad_index for vad_index, final_index in vad_final_pairs}
    control_by_final = {
        final_index: control_index for final_index, control_index in final_control_pairs
    }
    output = []
    for final_index, vad_index in vad_by_final.items():
        sensor_index = sensor_by_vad.get(vad_index)
        control_index = control_by_final.get(final_index)
        if sensor_index is None or control_index is None:
            continue
        source = sensor[sensor_index]
        target = control[control_index]
        stamp_delta = None
        if source["stamp_ns"] > 0 and target["stamp_ns"] > 0:
            stamp_delta = (target["stamp_ns"] - source["stamp_ns"]) * 1.0e-9
        output.append(
            {
                "stage": "sensor_to_first_control",
                "pair_policy": "inferred_sensor_latest_prior_then_causal_stage_chain",
                "upstream_topic": sensor_topic,
                "downstream_topic": control_topic,
                "upstream_index": sensor_index,
                "downstream_index": control_index,
                "downstream_relative_wall_sec": (target["bag_ns"] - bag_start_ns) * 1.0e-9,
                "receipt_latency_sec": (target["bag_ns"] - source["bag_ns"]) * 1.0e-9,
                "stamp_delta_sec": stamp_delta,
                "vad_index": vad_index,
                "final_index": final_index,
                "vad_topic": vad_topic,
                "final_topic": final_topic,
            }
        )
    return output


def analyze_records(
    records: dict[str, list[dict[str, Any]]],
    selected: dict[str, str | None],
    max_sensor_vad_gap_sec: float = 10.0,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    nonempty = [values for values in records.values() if values]
    if not nonempty:
        raise LatencyError("bag has no selected latency topics")
    bag_start_ns = min(record["bag_ns"] for values in nonempty for record in values)
    sensor_topic = selected.get("sensor")
    vad_topic = selected.get("vad_output")
    final_topic = selected.get("final_trajectory")
    control_topic = selected.get("control")
    actuation_topic = selected.get("actuation")
    sensor = records.get(sensor_topic, []) if sensor_topic else []
    vad = records.get(vad_topic, []) if vad_topic else []
    final = records.get(final_topic, []) if final_topic else []
    control = records.get(control_topic, []) if control_topic else []
    actuation = records.get(actuation_topic, []) if actuation_topic else []
    all_samples: list[dict[str, Any]] = []
    stages: dict[str, dict[str, Any]] = {}

    sensor_vad_pairs: list[tuple[int, int]] = []
    if sensor and vad and sensor_topic and vad_topic:
        sensor_vad_pairs = latest_prior_pairs(sensor, vad, max_sensor_vad_gap_sec)
        values = samples_from_pairs(
            "sensor_to_vad_output",
            "latest_sensor_receipt_before_vad_output_lower_bound",
            sensor_topic,
            vad_topic,
            sensor,
            vad,
            sensor_vad_pairs,
            bag_start_ns,
        )
        all_samples.extend(values)
        stages["sensor_to_vad_output"] = stage_metrics(values, len(sensor), len(vad))
    else:
        stages["sensor_to_vad_output"] = unavailable_stage(
            "sensor or VAD output topic was not recorded", len(sensor), len(vad)
        )

    vad_final_pairs: list[tuple[int, int]] = []
    if vad and final and vad_topic and final_topic:
        # Direct route-manager processing is millisecond scale. A wider window
        # would mislabel independently refreshed goal/fault stop trajectories as
        # VAD-derived outputs after candidate publication has already stopped.
        vad_final_pairs = first_after_pairs(vad, final, 0.2)
        values = samples_from_pairs(
            "vad_output_to_final_trajectory",
            "first_causal_receipt",
            vad_topic,
            final_topic,
            vad,
            final,
            vad_final_pairs,
            bag_start_ns,
        )
        all_samples.extend(values)
        stages["vad_output_to_final_trajectory"] = stage_metrics(
            values, len(vad), len(final)
        )
    else:
        stages["vad_output_to_final_trajectory"] = unavailable_stage(
            "VAD output or final trajectory topic was not recorded", len(vad), len(final)
        )

    final_control_pairs: list[tuple[int, int]] = []
    if final and control and final_topic and control_topic:
        final_control_pairs = first_after_pairs(final, control, 1.0)
        values = samples_from_pairs(
            "trajectory_update_to_first_control",
            "first_causal_receipt",
            final_topic,
            control_topic,
            final,
            control,
            final_control_pairs,
            bag_start_ns,
        )
        all_samples.extend(values)
        stages["trajectory_update_to_first_control"] = stage_metrics(
            values, len(final), len(control)
        )
        age_pairs = latest_prior_pairs(final, control)
        age_values = samples_from_pairs(
            "trajectory_age_at_control",
            "latest_causal_trajectory_for_each_control",
            final_topic,
            control_topic,
            final,
            control,
            age_pairs,
            bag_start_ns,
        )
        all_samples.extend(age_values)
        stages["trajectory_age_at_control"] = freshness_metrics(
            age_values, len(final), len(control)
        )
        stages["trajectory_age_at_control"]["control_count"] = len(control)
        stages["trajectory_age_at_control"]["coverage_percent"] = (
            100.0 * len(age_values) / len(control) if control else 0.0
        )
    else:
        stages["trajectory_update_to_first_control"] = unavailable_stage(
            "final trajectory or control topic was not recorded", len(final), len(control)
        )
        stages["trajectory_age_at_control"] = unavailable_stage(
            "final trajectory or control topic was not recorded", len(final), len(control)
        )

    if control and actuation and control_topic and actuation_topic:
        pairs = first_after_pairs(control, actuation, 0.2)
        values = samples_from_pairs(
            "control_to_actuation",
            "first_causal_receipt",
            control_topic,
            actuation_topic,
            control,
            actuation,
            pairs,
            bag_start_ns,
        )
        all_samples.extend(values)
        stages["control_to_actuation"] = stage_metrics(values, len(control), len(actuation))
    else:
        stages["control_to_actuation"] = unavailable_stage(
            "control or actuation topic was not recorded", len(control), len(actuation)
        )

    if (
        sensor
        and vad
        and final
        and control
        and sensor_topic
        and vad_topic
        and final_topic
        and control_topic
    ):
        values = chained_samples(
            sensor_topic,
            vad_topic,
            final_topic,
            control_topic,
            sensor,
            vad,
            final,
            control,
            sensor_vad_pairs,
            vad_final_pairs,
            final_control_pairs,
            bag_start_ns,
        )
        all_samples.extend(values)
        stages["sensor_to_first_control"] = stage_metrics(values, len(sensor), len(control))
    else:
        stages["sensor_to_first_control"] = unavailable_stage(
            "complete sensor/VAD/final/control chain was not recorded",
            len(sensor),
            len(control),
        )

    rates = {topic: event_rate(values) for topic, values in records.items() if values}
    acceptance_front_topic = (
        CAMERA_INFO_TOPICS[0]
        if records.get(CAMERA_INFO_TOPICS[0])
        else sensor_topic
    )
    acceptance_front = (
        records.get(acceptance_front_topic, []) if acceptance_front_topic else []
    )
    candidate_front_acceptance = candidate_front_acceptance_metrics(
        vad_topic, acceptance_front_topic, vad, acceptance_front
    )
    warnings = [
        "receipt_latency_sec uses rosbag recorder receipt time (wall/system clock)",
        "stamp_delta_sec uses only ROS message stamps (simulation/physics clock)",
        "receipt timestamps are never subtracted from message stamps",
    ]
    if not sensor:
        warnings.append(
            "sensor timestamps are absent; sensor-to-VAD and sensor-to-control are unavailable"
        )
    else:
        warnings.append(
            "sensor-to-VAD pairing is a lower-bound inference: VAD output does not preserve the "
            "front-camera source stamp"
        )
    if vad_topic == "/planning/vad_route/selected_raw_trajectory":
        warnings.append(
            "selected_raw_trajectory is a route-manager fallback whose header stamp is rewritten "
            "after VAD inference"
        )
    report = {
        "schema_version": 1,
        "clock_domains": {
            "receipt": "rosbag recorder wall/system clock",
            "message_stamp": "ROS simulation/physics clock",
            "cross_domain_subtraction_used": False,
        },
        "selected_topics": selected,
        "message_counts": {topic: len(values) for topic, values in records.items()},
        "event_rates": rates,
        "candidate_front_acceptance": candidate_front_acceptance,
        "camera_bundle": camera_bundle_metrics(records),
        "vad_input_sync": vad_input_sync_metrics(
            sensor,
            records.get(ODOMETRY_TOPIC, []),
            records.get(ACCELERATION_TOPIC, []),
        ),
        "stages": stages,
        "data_quality_warnings": warnings,
    }
    return report, all_samples


def _clean_json(value: Any) -> Any:
    if isinstance(value, dict):
        return {key: _clean_json(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_clean_json(item) for item in value]
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def write_samples(path: Path, samples: Sequence[dict[str, Any]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(samples)


def plot_report(
    path: Path, report: dict[str, Any], samples: Sequence[dict[str, Any]]
) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(13, 8), constrained_layout=True)
    by_stage: dict[str, list[dict[str, Any]]] = {}
    for sample in samples:
        by_stage.setdefault(sample["stage"], []).append(sample)

    age = by_stage.get("trajectory_age_at_control", [])
    if age:
        x = [sample["downstream_relative_wall_sec"] for sample in age]
        axes[0, 0].plot(x, [sample["stamp_delta_sec"] for sample in age], label="stamp age")
        axes[0, 0].plot(
            x, [sample["receipt_latency_sec"] for sample in age], label="receipt age", alpha=0.8
        )
        axes[0, 0].legend()
    else:
        axes[0, 0].text(0.5, 0.5, "trajectory age unavailable", ha="center", va="center")
    axes[0, 0].set_title("Final trajectory age at controller")
    axes[0, 0].set_xlabel("bag relative wall time [s]")
    axes[0, 0].set_ylabel("age [s]")
    axes[0, 0].grid(True, alpha=0.3)

    stage_names = []
    medians = []
    p95s = []
    for name, metrics in report["stages"].items():
        latency = metrics.get("receipt_latency_sec", {})
        if latency.get("available") and name != "trajectory_age_at_control":
            stage_names.append(name.replace("_", "\n"))
            medians.append(latency["median"])
            p95s.append(latency["p95"])
    if stage_names:
        positions = np.arange(len(stage_names))
        width = 0.36
        axes[0, 1].bar(positions - width / 2, medians, width, label="median")
        axes[0, 1].bar(positions + width / 2, p95s, width, label="p95")
        axes[0, 1].set_xticks(positions, stage_names, fontsize=8)
        axes[0, 1].legend()
    else:
        axes[0, 1].text(0.5, 0.5, "stage latency unavailable", ha="center", va="center")
    axes[0, 1].set_title("Receipt-time stage latency")
    axes[0, 1].set_ylabel("wall time [s]")
    axes[0, 1].grid(True, axis="y", alpha=0.3)

    stamp_age = [
        sample["stamp_delta_sec"]
        for sample in age
        if sample["stamp_delta_sec"] is not None
    ]
    if stamp_age:
        axes[1, 0].hist(stamp_age, bins=min(40, max(10, int(math.sqrt(len(stamp_age))))))
    else:
        axes[1, 0].text(0.5, 0.5, "stamp-age distribution unavailable", ha="center")
    axes[1, 0].set_title("Trajectory stamp-age distribution")
    axes[1, 0].set_xlabel("simulation time [s]")
    axes[1, 0].set_ylabel("control samples")
    axes[1, 0].grid(True, alpha=0.3)

    axes[1, 1].axis("off")
    selected = report["selected_topics"]
    age_metrics = report["stages"]["trajectory_age_at_control"].get("stamp_delta_sec", {})
    lines = [
        "Selected topics",
        f"sensor: {selected.get('sensor') or 'not recorded'}",
        f"VAD: {selected.get('vad_output') or 'not recorded'}",
        f"final: {selected.get('final_trajectory') or 'not recorded'}",
        f"control: {selected.get('control') or 'not recorded'}",
        "",
        "Trajectory stamp age",
        (
            f"median {age_metrics['median']:.3f} s | p95 {age_metrics['p95']:.3f} s | "
            f"max {age_metrics['max']:.3f} s"
            if age_metrics.get("available")
            else "unavailable"
        ),
        "",
        "Sensor attribution is inferred until VAD propagates its input stamp.",
    ]
    axes[1, 1].text(0.0, 1.0, "\n".join(lines), va="top", family="monospace", fontsize=10)
    fig.suptitle("Autoware VAD end-to-end latency and freshness")
    fig.savefig(path, dpi=150)
    plt.close(fig)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, type=Path, help="rosbag2 directory")
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument(
        "--sensor-topic",
        help="override the auto-selected front-camera timestamp topic",
    )
    parser.add_argument(
        "--max-sensor-vad-gap-sec",
        type=float,
        default=10.0,
        help="discard inferred sensor/VAD pairs farther apart in receipt time",
    )
    args = parser.parse_args(argv)
    if not math.isfinite(args.max_sensor_vad_gap_sec) or args.max_sensor_vad_gap_sec <= 0.0:
        parser.error("--max-sensor-vad-gap-sec must be positive")
    return args


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        bag = args.bag.expanduser().resolve()
        output_dir = args.output_dir.expanduser().resolve()
        output_dir.mkdir(parents=True, exist_ok=True)
        records, topic_types, selected = read_bag(bag, args.sensor_topic)
        report, samples = analyze_records(records, selected, args.max_sensor_vad_gap_sec)
        report["inputs"] = {"bag": str(bag)}
        report["topic_types"] = {
            topic: topic_types[topic] for topic in records if topic in topic_types
        }
        with (output_dir / "e2e_latency.json").open("w", encoding="utf-8") as stream:
            json.dump(_clean_json(report), stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
        write_samples(output_dir / "e2e_latency_samples.csv", samples)
        plot_report(output_dir / "e2e_latency.png", report, samples)
        age = report["stages"]["trajectory_age_at_control"]["stamp_delta_sec"]
        if age.get("available"):
            print(
                f"trajectory stamp age: median={age['median']:.3f}s "
                f"p95={age['p95']:.3f}s max={age['max']:.3f}s"
            )
        else:
            print("trajectory stamp age: unavailable")
        if not report["stages"]["sensor_to_vad_output"]["available"]:
            print("sensor-to-VAD: unavailable in this bag")
        print(f"wrote {output_dir / 'e2e_latency.json'}")
        return 0
    except (LatencyError, OSError, RuntimeError, ValueError) as error:
        print(f"ERROR: {error}", file=__import__("sys").stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
