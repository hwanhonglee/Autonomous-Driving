#!/usr/bin/env python3
"""Produce reproducible PC4-to-PC2 VILS evidence from the PC1 rosbag.

Safety properties:

* Every SQLite database is opened with ``mode=ro&immutable=1`` and
  ``PRAGMA query_only=ON``.
* The script never starts a ROS node, replays a bag, or publishes a message.
* ROS is used only for local CDR deserialization.

The analysis intentionally reports two scopes:

* ``conservative_db0_12`` uses the crash-surviving original DB0--DB12 files.
* ``extended_db0_13`` additionally uses DB13, which is a separately recovered
  and currently ``quick_check=ok`` candidate. It must not be described as the
  untouched crash-time DB13.

Run this with the ROS 2 Humble and Autoware install environments sourced. The
default JSON and CSV outputs are written next to this script.
"""

from __future__ import annotations

import argparse
import bisect
import collections
import csv
import datetime
import hashlib
import json
import math
import sqlite3
import statistics
import sys
from pathlib import Path
from typing import Any, Callable, Iterable, Sequence

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


RUN_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BAG_DIR = RUN_ROOT / "rosbag"
DEFAULT_OUTPUT_DIR = Path(__file__).resolve().parent
JSON_NAME = "VILS_PC4_INTEGRATION_EVIDENCE.json"
CSV_NAME = "VILS_PC4_PC2_ALIGNMENT.csv"

PC4_TOPIC = "/perception/pc4/virtual_obstacles/tracked_objects"
TRACKER_TOPIC = "/perception/object_recognition/tracking/objects"
CANONICAL_TOPIC = "/perception/object_recognition/objects"
LOCALIZATION_TOPIC = "/localization/kinematic_state"
VELOCITY_TOPIC = "/vehicle/status/velocity_status"
CONTROL_TOPIC = "/control/command/control_cmd"

ANALYZED_TOPICS = (
    PC4_TOPIC,
    TRACKER_TOPIC,
    CANONICAL_TOPIC,
    LOCALIZATION_TOPIC,
    VELOCITY_TOPIC,
    CONTROL_TOPIC,
)

EXPECTED_VILS_EVIDENCE_TOPICS = (
    "/diagnostics/pc4/object_adapter",
    "/perception/pc2/vils/accepted_pc4_status",
    "/perception/pc2/vils/candidate_objects",
)

LABEL_NAMES = {
    0: "UNKNOWN",
    1: "CAR",
    2: "TRUCK",
    3: "BUS",
    4: "TRAILER",
    5: "MOTORCYCLE",
    6: "BICYCLE",
    7: "PEDESTRIAN",
}

KST = datetime.timezone(datetime.timedelta(hours=9))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag-dir", type=Path, default=DEFAULT_BAG_DIR)
    parser.add_argument("--json-output", type=Path)
    parser.add_argument("--csv-output", type=Path)
    parser.add_argument("--quiet", action="store_true")
    return parser.parse_args()


def portable_path(path: Path) -> str:
    resolved = path.resolve()
    try:
        return str(resolved.relative_to(RUN_ROOT))
    except ValueError:
        return str(resolved)


def database_index(path: Path) -> int:
    try:
        return int(path.stem.rsplit("_", 1)[1])
    except (IndexError, ValueError) as error:
        raise ValueError(f"Cannot obtain split index from {path.name}") from error


def database_files(bag_dir: Path) -> dict[int, Path]:
    files = {database_index(path): path for path in bag_dir.glob("*.db3")}
    required = set(range(14))
    missing = sorted(required - set(files))
    if missing:
        raise RuntimeError(f"Required DB files are missing: {missing}")
    return {index: files[index] for index in sorted(required)}


def connect_read_only(path: Path) -> sqlite3.Connection:
    connection = sqlite3.connect(f"file:{path}?mode=ro&immutable=1", uri=True)
    connection.execute("PRAGMA query_only=ON")
    return connection


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def iso_kst(timestamp_ns: int | None) -> str | None:
    if timestamp_ns is None:
        return None
    return datetime.datetime.fromtimestamp(timestamp_ns / 1_000_000_000, KST).isoformat(
        timespec="microseconds"
    )


def header_stamp_ns(message: Any) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


def uuid_hex(obj: Any) -> str:
    return bytes(obj.object_id.uuid).hex()


def primary_label(obj: Any) -> tuple[int, str, float | None]:
    if not obj.classification:
        return -1, "MISSING", None
    classification = max(obj.classification, key=lambda item: float(item.probability))
    label = int(classification.label)
    return label, LABEL_NAMES.get(label, f"LABEL_{label}"), float(classification.probability)


def tracked_position(obj: Any) -> Any:
    return obj.kinematics.pose_with_covariance.pose.position


def predicted_position(obj: Any) -> Any:
    return obj.kinematics.initial_pose_with_covariance.pose.position


def numeric_summary(values: Iterable[float]) -> dict[str, float | int | None]:
    finite = sorted(float(value) for value in values if value is not None and math.isfinite(value))
    if not finite:
        return {
            "count": 0,
            "min": None,
            "median": None,
            "mean": None,
            "p95": None,
            "max": None,
        }
    p95_index = min(len(finite) - 1, math.ceil(0.95 * len(finite)) - 1)
    return {
        "count": len(finite),
        "min": finite[0],
        "median": statistics.median(finite),
        "mean": statistics.fmean(finite),
        "p95": finite[p95_index],
        "max": finite[-1],
    }


def topic_gap_summary(samples: Sequence[tuple[int, Any, int]]) -> dict[str, Any]:
    timestamps = [sample[0] for sample in samples]
    if not timestamps:
        return {
            "messages": 0,
            "first_record_ns": None,
            "last_record_ns": None,
            "first_record_kst": None,
            "last_record_kst": None,
            "span_s": 0.0,
            "effective_rate_hz": None,
            "maximum_gap": None,
            "gaps_over_1_s": [],
        }
    gaps = [
        (timestamps[index - 1], timestamps[index], timestamps[index] - timestamps[index - 1])
        for index in range(1, len(timestamps))
    ]
    span_ns = timestamps[-1] - timestamps[0]

    def gap_record(item: tuple[int, int, int]) -> dict[str, Any]:
        start, end, duration = item
        return {
            "start_ns": start,
            "end_ns": end,
            "start_kst": iso_kst(start),
            "end_kst": iso_kst(end),
            "duration_s": duration / 1_000_000_000,
        }

    return {
        "messages": len(timestamps),
        "first_record_ns": timestamps[0],
        "last_record_ns": timestamps[-1],
        "first_record_kst": iso_kst(timestamps[0]),
        "last_record_kst": iso_kst(timestamps[-1]),
        "span_s": span_ns / 1_000_000_000,
        "effective_rate_hz": (
            (len(timestamps) - 1) / (span_ns / 1_000_000_000)
            if len(timestamps) > 1 and span_ns > 0
            else None
        ),
        "maximum_gap": gap_record(max(gaps, key=lambda item: item[2])) if gaps else None,
        "gaps_over_1_s": [gap_record(item) for item in gaps if item[2] > 1_000_000_000],
    }


def nearest_sample(
    samples: Sequence[tuple[int, Any, int]], timestamps: Sequence[int], target_ns: int
) -> tuple[int, Any, int] | None:
    if not samples:
        return None
    insertion = bisect.bisect_left(timestamps, target_ns)
    candidates = [index for index in (insertion - 1, insertion) if 0 <= index < len(samples)]
    return samples[min(candidates, key=lambda index: abs(timestamps[index] - target_ns))]


def nearest_object(
    objects: Sequence[Any], target_position: Any, position_getter: Callable[[Any], Any]
) -> dict[str, Any]:
    if not objects:
        return {
            "object_count": 0,
            "nearest_xy_distance_m": None,
            "nearest_uuid_hex": None,
            "nearest_label_id": None,
            "nearest_label": None,
            "nearest_x": None,
            "nearest_y": None,
            "nearest_z": None,
        }
    best: tuple[float, Any, Any] | None = None
    for obj in objects:
        position = position_getter(obj)
        distance = math.hypot(position.x - target_position.x, position.y - target_position.y)
        if best is None or distance < best[0]:
            best = (distance, obj, position)
    assert best is not None
    distance, obj, position = best
    label_id, label_name, _probability = primary_label(obj)
    return {
        "object_count": len(objects),
        "nearest_xy_distance_m": distance,
        "nearest_uuid_hex": uuid_hex(obj),
        "nearest_label_id": label_id,
        "nearest_label": label_name,
        "nearest_x": float(position.x),
        "nearest_y": float(position.y),
        "nearest_z": float(position.z),
    }


def scope_sql_summary(paths: Sequence[Path]) -> dict[str, Any]:
    total_messages = 0
    first_ns: int | None = None
    last_ns: int | None = None
    topic_counts: collections.Counter[str] = collections.Counter()
    for path in paths:
        connection = connect_read_only(path)
        count, minimum, maximum = connection.execute(
            "SELECT COUNT(*), MIN(timestamp), MAX(timestamp) FROM messages"
        ).fetchone()
        total_messages += int(count)
        if minimum is not None:
            first_ns = int(minimum) if first_ns is None else min(first_ns, int(minimum))
        if maximum is not None:
            last_ns = int(maximum) if last_ns is None else max(last_ns, int(maximum))
        for name, topic_count in connection.execute(
            "SELECT topics.name, COUNT(messages.id) "
            "FROM topics LEFT JOIN messages ON messages.topic_id=topics.id "
            "GROUP BY topics.id"
        ):
            topic_counts[str(name)] += int(topic_count)
        connection.close()
    return {
        "database_files": [path.name for path in paths],
        "message_count": total_messages,
        "first_record_ns": first_ns,
        "last_record_ns": last_ns,
        "first_record_kst": iso_kst(first_ns),
        "last_record_kst": iso_kst(last_ns),
        "duration_s": (last_ns - first_ns) / 1_000_000_000 if first_ns and last_ns else 0.0,
        "topic_message_counts": dict(sorted(topic_counts.items())),
    }


def load_semantic_samples(paths_by_index: dict[int, Path]) -> dict[str, list[tuple[int, Any, int]]]:
    samples: dict[str, list[tuple[int, Any, int]]] = {topic: [] for topic in ANALYZED_TOPICS}
    type_cache: dict[str, Any] = {}
    for db_index, path in paths_by_index.items():
        connection = connect_read_only(path)
        selected = {
            int(topic_id): (str(name), str(type_name))
            for topic_id, name, type_name in connection.execute("SELECT id,name,type FROM topics")
            if name in samples
        }
        if selected:
            placeholders = ",".join("?" for _ in selected)
            query = (
                "SELECT timestamp,topic_id,data FROM messages "
                f"WHERE topic_id IN ({placeholders}) ORDER BY timestamp"
            )
            for timestamp, topic_id, cdr in connection.execute(query, tuple(selected)):
                topic, type_name = selected[int(topic_id)]
                message_type = type_cache.setdefault(type_name, get_message(type_name))
                message = deserialize_message(cdr, message_type)
                samples[topic].append((int(timestamp), message, db_index))
        connection.close()
    for topic in samples:
        samples[topic].sort(key=lambda sample: sample[0])
    return samples


def scoped(
    samples: dict[str, list[tuple[int, Any, int]]], maximum_db_index: int
) -> dict[str, list[tuple[int, Any, int]]]:
    return {
        topic: [sample for sample in topic_samples if sample[2] <= maximum_db_index]
        for topic, topic_samples in samples.items()
    }


def stable_object_summary(pc4_samples: Sequence[tuple[int, Any, int]]) -> dict[str, Any]:
    occurrences: collections.Counter[tuple[str, int]] = collections.Counter()
    for _timestamp, message, _db_index in pc4_samples:
        for obj in message.objects:
            occurrences[(uuid_hex(obj), primary_label(obj)[0])] += 1
    if not occurrences:
        return {"available": False}
    stable_key, occurrence_count = occurrences.most_common(1)[0]
    stable_samples: list[tuple[int, Any]] = []
    for timestamp, message, _db_index in pc4_samples:
        for obj in message.objects:
            if (uuid_hex(obj), primary_label(obj)[0]) == stable_key:
                stable_samples.append((timestamp, obj))

    first_timestamp, first_object = stable_samples[0]
    last_timestamp, last_object = stable_samples[-1]
    label_id, label_name, _probability = primary_label(first_object)

    def positions(axis: str) -> list[float]:
        return [float(getattr(tracked_position(obj), axis)) for _timestamp, obj in stable_samples]

    def dimensions(axis: str) -> list[float]:
        return [float(getattr(obj.shape.dimensions, axis)) for _timestamp, obj in stable_samples]

    def pose_record(timestamp: int, obj: Any) -> dict[str, Any]:
        position = tracked_position(obj)
        return {
            "record_ns": timestamp,
            "record_kst": iso_kst(timestamp),
            "x": float(position.x),
            "y": float(position.y),
            "z": float(position.z),
        }

    return {
        "available": True,
        "selection_method": "most frequent (UUID, primary classification label) object occurrence",
        "uuid_hex": stable_key[0],
        "classification_id": label_id,
        "classification": label_name,
        "messages": occurrence_count,
        "first_record_ns": first_timestamp,
        "last_record_ns": last_timestamp,
        "first_record_kst": iso_kst(first_timestamp),
        "last_record_kst": iso_kst(last_timestamp),
        "duration_s": (last_timestamp - first_timestamp) / 1_000_000_000,
        "frame_counts": dict(
            collections.Counter(
                message.header.frame_id
                for _timestamp, message, _db_index in pc4_samples
                if any(
                    (uuid_hex(obj), primary_label(obj)[0]) == stable_key
                    for obj in message.objects
                )
            )
        ),
        "first_pose": pose_record(first_timestamp, first_object),
        "last_pose": pose_record(last_timestamp, last_object),
        "pose_statistics": {
            "x": numeric_summary(positions("x")),
            "y": numeric_summary(positions("y")),
            "z": numeric_summary(positions("z")),
        },
        "dimension_statistics": {
            "x": numeric_summary(dimensions("x")),
            "y": numeric_summary(dimensions("y")),
            "z": numeric_summary(dimensions("z")),
        },
        "existence_probability": numeric_summary(
            float(obj.existence_probability) for _timestamp, obj in stable_samples
        ),
        "classification_probability": numeric_summary(
            primary_label(obj)[2] for _timestamp, obj in stable_samples
        ),
    }


def pc4_summary(pc4_samples: Sequence[tuple[int, Any, int]]) -> dict[str, Any]:
    nonempty = [sample for sample in pc4_samples if sample[1].objects]
    object_counts = [len(message.objects) for _timestamp, message, _db_index in pc4_samples]
    frames = collections.Counter(message.header.frame_id for _timestamp, message, _db_index in pc4_samples)
    header_ages = [
        (timestamp - header_stamp_ns(message)) / 1_000_000
        for timestamp, message, _db_index in pc4_samples
    ]
    transitions: list[dict[str, Any]] = []
    previous_state: tuple[Any, ...] | None = None
    for timestamp, message, _db_index in pc4_samples:
        if not message.objects:
            state: tuple[Any, ...] = ("empty",)
        else:
            obj = message.objects[0]
            label_id, label_name, _probability = primary_label(obj)
            state = ("object", uuid_hex(obj), label_id, label_name)
        if state != previous_state:
            transitions.append(
                {
                    "record_ns": timestamp,
                    "record_kst": iso_kst(timestamp),
                    "state": list(state),
                }
            )
            previous_state = state
    return {
        "timing": topic_gap_summary(pc4_samples),
        "frame_counts": dict(frames),
        "object_count": numeric_summary(object_counts),
        "empty_messages": len(pc4_samples) - len(nonempty),
        "nonempty_messages": len(nonempty),
        "first_nonempty_ns": nonempty[0][0] if nonempty else None,
        "last_nonempty_ns": nonempty[-1][0] if nonempty else None,
        "first_nonempty_kst": iso_kst(nonempty[0][0]) if nonempty else None,
        "last_nonempty_kst": iso_kst(nonempty[-1][0]) if nonempty else None,
        "nonempty_span_s": (nonempty[-1][0] - nonempty[0][0]) / 1_000_000_000
        if len(nonempty) > 1
        else 0.0,
        "bag_record_minus_header_stamp_ms": numeric_summary(header_ages),
        "state_transitions": transitions,
        "stable_object": stable_object_summary(pc4_samples),
    }


def build_alignment_rows(samples: dict[str, list[tuple[int, Any, int]]]) -> list[dict[str, Any]]:
    pc4 = [sample for sample in samples[PC4_TOPIC] if sample[1].objects]
    tracker = samples[TRACKER_TOPIC]
    canonical = samples[CANONICAL_TOPIC]
    localization = samples[LOCALIZATION_TOPIC]
    velocity = samples[VELOCITY_TOPIC]
    control = samples[CONTROL_TOPIC]
    timestamps = {
        "tracker": [sample[0] for sample in tracker],
        "canonical": [sample[0] for sample in canonical],
        "localization": [sample[0] for sample in localization],
        "velocity": [sample[0] for sample in velocity],
        "control": [sample[0] for sample in control],
    }
    rows: list[dict[str, Any]] = []
    for pc4_timestamp, pc4_message, pc4_db_index in pc4:
        pc4_object = pc4_message.objects[0]
        pc4_uuid = uuid_hex(pc4_object)
        pc4_label_id, pc4_label, pc4_probability = primary_label(pc4_object)
        pc4_position = tracked_position(pc4_object)

        tracker_sample = nearest_sample(tracker, timestamps["tracker"], pc4_timestamp)
        canonical_sample = nearest_sample(canonical, timestamps["canonical"], pc4_timestamp)
        localization_sample = nearest_sample(
            localization, timestamps["localization"], pc4_timestamp
        )
        velocity_sample = nearest_sample(velocity, timestamps["velocity"], pc4_timestamp)
        control_sample = nearest_sample(control, timestamps["control"], pc4_timestamp)

        tracker_result = (
            nearest_object(tracker_sample[1].objects, pc4_position, tracked_position)
            if tracker_sample
            else nearest_object([], pc4_position, tracked_position)
        )
        canonical_result = (
            nearest_object(canonical_sample[1].objects, pc4_position, predicted_position)
            if canonical_sample
            else nearest_object([], pc4_position, predicted_position)
        )

        row: dict[str, Any] = {
            "pc4_record_ns": pc4_timestamp,
            "pc4_record_kst": iso_kst(pc4_timestamp),
            "pc4_db_index": pc4_db_index,
            "pc4_header_ns": header_stamp_ns(pc4_message),
            "pc4_frame": pc4_message.header.frame_id,
            "pc4_uuid_hex": pc4_uuid,
            "pc4_label_id": pc4_label_id,
            "pc4_label": pc4_label,
            "pc4_label_probability": pc4_probability,
            "pc4_x": float(pc4_position.x),
            "pc4_y": float(pc4_position.y),
            "pc4_z": float(pc4_position.z),
            "pc4_existence_probability": float(pc4_object.existence_probability),
        }

        for prefix, sample, result in (
            ("tracker", tracker_sample, tracker_result),
            ("canonical", canonical_sample, canonical_result),
        ):
            row[f"{prefix}_record_ns"] = sample[0] if sample else None
            row[f"{prefix}_record_delta_ms"] = (
                (sample[0] - pc4_timestamp) / 1_000_000 if sample else None
            )
            row[f"{prefix}_object_count"] = result["object_count"]
            row[f"{prefix}_exact_uuid_match"] = bool(
                sample and any(uuid_hex(obj) == pc4_uuid for obj in sample[1].objects)
            )
            for key in (
                "nearest_xy_distance_m",
                "nearest_uuid_hex",
                "nearest_label_id",
                "nearest_label",
                "nearest_x",
                "nearest_y",
                "nearest_z",
            ):
                row[f"{prefix}_{key}"] = result[key]

        if localization_sample:
            ego_position = localization_sample[1].pose.pose.position
            row.update(
                {
                    "ego_record_ns": localization_sample[0],
                    "ego_record_delta_ms": (localization_sample[0] - pc4_timestamp) / 1_000_000,
                    "ego_x": float(ego_position.x),
                    "ego_y": float(ego_position.y),
                    "ego_z": float(ego_position.z),
                    "ego_pc4_xy_distance_m": math.hypot(
                        ego_position.x - pc4_position.x, ego_position.y - pc4_position.y
                    ),
                }
            )
        else:
            row.update(
                {
                    "ego_record_ns": None,
                    "ego_record_delta_ms": None,
                    "ego_x": None,
                    "ego_y": None,
                    "ego_z": None,
                    "ego_pc4_xy_distance_m": None,
                }
            )

        if velocity_sample:
            row.update(
                {
                    "velocity_record_ns": velocity_sample[0],
                    "velocity_record_delta_ms": (velocity_sample[0] - pc4_timestamp)
                    / 1_000_000,
                    "vehicle_longitudinal_velocity_mps": float(
                        velocity_sample[1].longitudinal_velocity
                    ),
                }
            )
        else:
            row.update(
                {
                    "velocity_record_ns": None,
                    "velocity_record_delta_ms": None,
                    "vehicle_longitudinal_velocity_mps": None,
                }
            )

        if control_sample:
            row.update(
                {
                    "control_record_ns": control_sample[0],
                    "control_record_delta_ms": (control_sample[0] - pc4_timestamp)
                    / 1_000_000,
                    "control_target_velocity_mps": float(
                        control_sample[1].longitudinal.velocity
                    ),
                    "control_target_acceleration_mps2": float(
                        control_sample[1].longitudinal.acceleration
                    ),
                }
            )
        else:
            row.update(
                {
                    "control_record_ns": None,
                    "control_record_delta_ms": None,
                    "control_target_velocity_mps": None,
                    "control_target_acceleration_mps2": None,
                }
            )
        rows.append(row)
    return rows


def comparison_summary(rows: Sequence[dict[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {"pc4_nonempty_frames_compared": len(rows)}
    for prefix in ("tracker", "canonical"):
        distances = [
            row[f"{prefix}_nearest_xy_distance_m"]
            for row in rows
            if row[f"{prefix}_nearest_xy_distance_m"] is not None
        ]
        result[prefix] = {
            "exact_uuid_match_frames": sum(
                bool(row[f"{prefix}_exact_uuid_match"]) for row in rows
            ),
            "aligned_empty_frames": sum(row[f"{prefix}_object_count"] == 0 for row in rows),
            "nearest_xy_distance_m": numeric_summary(distances),
            "nearest_within_0_5_m_frames": sum(distance <= 0.5 for distance in distances),
            "nearest_within_1_m_frames": sum(distance <= 1.0 for distance in distances),
            "nearest_within_2_m_frames": sum(distance <= 2.0 for distance in distances),
            "record_time_delta_ms": numeric_summary(
                row[f"{prefix}_record_delta_ms"] for row in rows
            ),
        }
    return result


def ego_minimum_event(rows: Sequence[dict[str, Any]]) -> dict[str, Any] | None:
    candidates = [row for row in rows if row["ego_pc4_xy_distance_m"] is not None]
    if not candidates:
        return None
    row = min(candidates, key=lambda item: item["ego_pc4_xy_distance_m"])
    keys = (
        "pc4_record_ns",
        "pc4_record_kst",
        "pc4_uuid_hex",
        "pc4_label_id",
        "pc4_label",
        "pc4_x",
        "pc4_y",
        "pc4_z",
        "ego_record_ns",
        "ego_record_delta_ms",
        "ego_x",
        "ego_y",
        "ego_z",
        "ego_pc4_xy_distance_m",
        "velocity_record_ns",
        "velocity_record_delta_ms",
        "vehicle_longitudinal_velocity_mps",
        "control_record_ns",
        "control_record_delta_ms",
        "control_target_velocity_mps",
        "control_target_acceleration_mps2",
        "tracker_object_count",
        "tracker_nearest_xy_distance_m",
        "canonical_object_count",
        "canonical_nearest_xy_distance_m",
    )
    return {key: row[key] for key in keys}


def write_csv(path: Path, rows: Sequence[dict[str, Any]]) -> None:
    if not rows:
        raise RuntimeError("No PC4 nonempty alignment rows were produced")
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    args = parse_args()
    bag_dir = args.bag_dir.resolve()
    json_output = (args.json_output or DEFAULT_OUTPUT_DIR / JSON_NAME).resolve()
    csv_output = (args.csv_output or DEFAULT_OUTPUT_DIR / CSV_NAME).resolve()
    paths_by_index = database_files(bag_dir)

    integrity: dict[str, str] = {}
    database_hashes: dict[str, str] = {}
    for _index, path in paths_by_index.items():
        connection = connect_read_only(path)
        integrity[path.name] = str(connection.execute("PRAGMA quick_check").fetchone()[0])
        connection.close()
        database_hashes[path.name] = sha256_file(path)
    if any(result != "ok" for result in integrity.values()):
        raise RuntimeError(f"Database quick_check failed: {integrity}")

    all_samples = load_semantic_samples(paths_by_index)
    conservative_samples = scoped(all_samples, 12)
    extended_samples = scoped(all_samples, 13)
    alignment_rows = build_alignment_rows(extended_samples)
    stable_uuid = stable_object_summary(extended_samples[PC4_TOPIC]).get("uuid_hex")
    stable_rows = [row for row in alignment_rows if row["pc4_uuid_hex"] == stable_uuid]

    discovered_topic_counts = scope_sql_summary(list(paths_by_index.values()))[
        "topic_message_counts"
    ]
    evidence_topic_counts = {
        topic: int(discovered_topic_counts.get(topic, 0))
        for topic in EXPECTED_VILS_EVIDENCE_TOPICS
    }

    scope_results: dict[str, Any] = {}
    for scope_name, maximum_index, sample_set in (
        ("conservative_db0_12", 12, conservative_samples),
        ("extended_db0_13", 13, extended_samples),
    ):
        pc4_scope_rows = [row for row in alignment_rows if row["pc4_db_index"] <= maximum_index]
        stable_scope_rows = [row for row in pc4_scope_rows if row["pc4_uuid_hex"] == stable_uuid]
        scope_results[scope_name] = {
            "scope_definition": (
                "DB0-DB12 crash-surviving original files"
                if maximum_index == 12
                else "DB0-DB12 plus separately recovered quick_check=ok DB13 candidate"
            ),
            "bag": scope_sql_summary(
                [paths_by_index[index] for index in range(maximum_index + 1)]
            ),
            "pc4": pc4_summary(sample_set[PC4_TOPIC]),
            "pc2_tracker_timing": topic_gap_summary(sample_set[TRACKER_TOPIC]),
            "pc2_canonical_timing": topic_gap_summary(sample_set[CANONICAL_TOPIC]),
            "all_pc4_nonempty_comparison": comparison_summary(pc4_scope_rows),
            "stable_pc4_object_comparison": comparison_summary(stable_scope_rows),
            "ego_minimum_xy_event_all_pc4_nonempty": ego_minimum_event(pc4_scope_rows),
            "ego_minimum_xy_event_stable_pc4_object": ego_minimum_event(stable_scope_rows),
        }

    output = {
        "analysis_schema": "vils_pc4_integration_evidence/v1",
        "generator": Path(__file__).name,
        "bag_directory": portable_path(bag_dir),
        "safety": {
            "sqlite_open_mode": "mode=ro&immutable=1 with PRAGMA query_only=ON",
            "ros_usage": "CDR deserialization only; no ROS node, replay, subscription, or publish",
            "record_time_basis": "SQLite messages.timestamp",
        },
        "methodology": {
            "scope_comparison": "Conservative DB0-12 and extended DB0-13 are reported separately",
            "pc4_object_selection": "Most frequent UUID and primary classification pair",
            "pc2_alignment": "Nearest PC2 message by SQLite record timestamp",
            "object_comparison": "Euclidean XY distance in the reported map frame",
            "ego_alignment": "Nearest localization and vehicle velocity messages by record timestamp",
            "csv_rows": "One row per nonempty PC4 message; all such rows occur in DB0-12",
        },
        "database_integrity": integrity,
        "database_sha256": database_hashes,
        "vils_evidence_topic_counts": evidence_topic_counts,
        "scope_results": scope_results,
        "alignment_csv": {
            "path": portable_path(csv_output),
            "rows": len(alignment_rows),
            "stable_object_rows": len(stable_rows),
        },
        "limitations": [
            "DB13 is a separately recovered quick_check=ok candidate, not the untouched crash-time DB13; conservative paper results should use DB0-12.",
            "The PC4 topic name identifies virtual obstacles, but this bag does not contain simulator ground truth or independent source-provenance evidence.",
            "PC4 adapter diagnostics, PC2 candidate objects, and PC2 accepted-PC4 status topics contain zero recorded messages.",
            "An exact UUID match is sufficient evidence of identity preservation but its absence is not conclusive because a tracker or fusion stage may reassign UUIDs.",
            "Nearest XY proximity is supporting evidence only; it cannot prove semantic fusion, object identity, lane occupancy, collision risk, or causal control response.",
            "Bag-record minus header timestamps are not one-way network latency because four-host clock synchronization was not independently validated.",
            "The ego comparison is two-dimensional map-frame distance and does not account for object footprint, lane topology, heading, or trajectory intersection.",
            "High-bandwidth PC3 LiDAR and PC2/PC3/PC4 local resource logs are outside this PC1 dataset.",
        ],
        "python_version": sys.version.split()[0],
    }

    write_csv(csv_output, alignment_rows)
    with json_output.open("w", encoding="utf-8") as stream:
        json.dump(output, stream, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")

    if not args.quiet:
        print(f"JSON: {json_output}")
        print(f"CSV:  {csv_output}")
        print(f"Alignment rows: {len(alignment_rows)}")
        print(f"Stable object rows: {len(stable_rows)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
