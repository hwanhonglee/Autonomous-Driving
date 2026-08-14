#!/usr/bin/env python3
import argparse
import collections
import datetime
import json
import math
import re
import sqlite3
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


parser = argparse.ArgumentParser(description="Read-only PC1 VILS rosbag semantic summary")
parser.add_argument(
    "--bag-dir",
    type=Path,
    default=Path(__file__).resolve().parents[1] / "rosbag",
    help="rosbag2 directory containing metadata.yaml and split SQLite files",
)
args = parser.parse_args()
RUN_ROOT = Path(__file__).resolve().parents[1]
BAG_DIR = args.bag_dir.resolve()
SELECTED_TOPICS = {
    "/api/operation_mode/state",
    "/control/command/control_cmd",
    "/current/cruise_mode_status",
    "/localization/acceleration",
    "/localization/kinematic_state",
    "/perception/object_recognition/objects",
    "/perception/object_recognition/tracking/objects",
    "/perception/pc4/virtual_obstacles/tracked_objects",
    "/planning/scenario_planning/trajectory",
    "/system/emergency/hazard_status",
    "/system/fail_safe/mrm_state",
    "/tf",
    "/to_can_bus",
    "/vehicle/status/control_mode",
    "/vehicle/status/velocity_status",
}


def portable_path(path):
    resolved = path.resolve()
    try:
        return str(resolved.relative_to(RUN_ROOT))
    except ValueError:
        return str(resolved)


def bag_files():
    def sequence(path):
        match = re.search(r"_(\d+)\.db3$", path.name)
        return int(match.group(1)) if match else 10**9

    return sorted(BAG_DIR.glob("pc1_20260814_181555_*.db3"), key=sequence)


def stamp_ns(header):
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def iso_kst(timestamp_ns):
    return datetime.datetime.fromtimestamp(
        timestamp_ns / 1_000_000_000,
        tz=datetime.timezone(datetime.timedelta(hours=9)),
    ).isoformat()


def summarize_numeric(values):
    if not values:
        return {"count": 0}
    ordered = sorted(values)
    p95_index = min(len(ordered) - 1, math.ceil(0.95 * len(ordered)) - 1)
    return {
        "count": len(values),
        "min": ordered[0],
        "max": ordered[-1],
        "mean": sum(values) / len(values),
        "p95": ordered[p95_index],
    }


def transitions(samples, active_predicate):
    if not samples:
        return []
    intervals = []
    active_start = None
    previous_active = False
    for timestamp, value in samples:
        current_active = bool(active_predicate(value))
        if current_active and not previous_active:
            active_start = timestamp
        elif previous_active and not current_active and active_start is not None:
            intervals.append((active_start, timestamp))
            active_start = None
        previous_active = current_active
    if previous_active and active_start is not None:
        intervals.append((active_start, samples[-1][0]))
    return [
        {
            "start_ns": start,
            "end_ns": end,
            "start_kst": iso_kst(start),
            "end_kst": iso_kst(end),
            "duration_s": (end - start) / 1_000_000_000,
        }
        for start, end in intervals
    ]


object_stats = {
    topic: {
        "messages": 0,
        "bag_first_ns": None,
        "bag_last_ns": None,
        "frames": collections.Counter(),
        "object_counts": [],
        "ages_ms": [],
        "nonempty_messages": 0,
        "first_nonempty_ns": None,
        "last_nonempty_ns": None,
    }
    for topic in (
        "/perception/object_recognition/objects",
        "/perception/object_recognition/tracking/objects",
        "/perception/pc4/virtual_obstacles/tracked_objects",
    )
}
deserialize_errors = collections.Counter()
topic_seen_counts = collections.Counter()
request_samples = []
cruise_samples = []
control_mode_samples = []
operation_samples = []
velocity_samples = []
control_velocity = []
control_acceleration = []
control_steering = []
trajectory_point_counts = []
trajectory_max_velocity = []
trajectory_min_velocity = []
trajectory_final_velocity = []
hazard_levels = collections.Counter()
hazard_emergency = collections.Counter()
mrm_states = collections.Counter()
mrm_behaviors = collections.Counter()
localization_age_ms = []
acceleration_age_ms = []
tf_map_base_ages_ms = []
type_cache = {}


for db_path in bag_files():
    connection = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True)
    topic_rows = connection.execute("SELECT id, name, type FROM topics").fetchall()
    selected_by_id = {
        topic_id: (name, type_name)
        for topic_id, name, type_name in topic_rows
        if name in SELECTED_TOPICS
    }
    if not selected_by_id:
        connection.close()
        continue
    placeholders = ",".join("?" for _ in selected_by_id)
    query = (
        "SELECT timestamp, topic_id, data FROM messages "
        f"WHERE topic_id IN ({placeholders}) ORDER BY timestamp"
    )
    for timestamp, topic_id, data in connection.execute(query, tuple(selected_by_id)):
        topic_name, type_name = selected_by_id[topic_id]
        topic_seen_counts[topic_name] += 1
        try:
            message_type = type_cache.setdefault(type_name, get_message(type_name))
            message = deserialize_message(data, message_type)
        except Exception as error:  # preserve counts even if a recovered CDR sample is damaged
            deserialize_errors[f"{topic_name}: {type(error).__name__}"] += 1
            continue

        if topic_name in object_stats:
            stats = object_stats[topic_name]
            stats["messages"] += 1
            stats["bag_first_ns"] = timestamp if stats["bag_first_ns"] is None else stats["bag_first_ns"]
            stats["bag_last_ns"] = timestamp
            stats["frames"][message.header.frame_id] += 1
            object_count = len(message.objects)
            stats["object_counts"].append(object_count)
            header_time = stamp_ns(message.header)
            stats["ages_ms"].append((timestamp - header_time) / 1_000_000)
            if object_count > 0:
                stats["nonempty_messages"] += 1
                stats["first_nonempty_ns"] = (
                    timestamp if stats["first_nonempty_ns"] is None else stats["first_nonempty_ns"]
                )
                stats["last_nonempty_ns"] = timestamp

        elif topic_name == "/to_can_bus":
            if int(message.id) == 0x630 and int(message.dlc) >= 4:
                request_samples.append((timestamp, int(message.data[3])))
        elif topic_name == "/current/cruise_mode_status":
            cruise_samples.append((timestamp, bool(message.data)))
        elif topic_name == "/vehicle/status/control_mode":
            control_mode_samples.append((timestamp, int(message.mode)))
        elif topic_name == "/api/operation_mode/state":
            operation_samples.append(
                (
                    timestamp,
                    {
                        "mode": int(message.mode),
                        "enabled": bool(message.is_autoware_control_enabled),
                    },
                )
            )
        elif topic_name == "/vehicle/status/velocity_status":
            velocity_samples.append(
                (timestamp, float(message.longitudinal_velocity), float(message.lateral_velocity))
            )
        elif topic_name == "/control/command/control_cmd":
            control_velocity.append(float(message.longitudinal.velocity))
            control_acceleration.append(float(message.longitudinal.acceleration))
            control_steering.append(float(message.lateral.steering_tire_angle))
        elif topic_name == "/planning/scenario_planning/trajectory":
            velocities = [float(point.longitudinal_velocity_mps) for point in message.points]
            trajectory_point_counts.append(len(velocities))
            if velocities:
                trajectory_max_velocity.append(max(velocities))
                trajectory_min_velocity.append(min(velocities))
                trajectory_final_velocity.append(velocities[-1])
        elif topic_name == "/system/emergency/hazard_status":
            hazard_levels[int(message.status.level)] += 1
            hazard_emergency[bool(message.status.emergency)] += 1
        elif topic_name == "/system/fail_safe/mrm_state":
            mrm_states[int(message.state)] += 1
            mrm_behaviors[int(message.behavior)] += 1
        elif topic_name == "/localization/kinematic_state":
            localization_age_ms.append((timestamp - stamp_ns(message.header)) / 1_000_000)
        elif topic_name == "/localization/acceleration":
            acceleration_age_ms.append((timestamp - stamp_ns(message.header)) / 1_000_000)
        elif topic_name == "/tf":
            for transform in message.transforms:
                if transform.header.frame_id == "map" and transform.child_frame_id == "base_link":
                    tf_map_base_ages_ms.append((timestamp - stamp_ns(transform.header)) / 1_000_000)
    connection.close()


objects_summary = {}
for topic, stats in object_stats.items():
    duration_s = (
        (stats["bag_last_ns"] - stats["bag_first_ns"]) / 1_000_000_000
        if stats["messages"] > 1
        else 0.0
    )
    objects_summary[topic] = {
        "messages": stats["messages"],
        "bag_first_kst": iso_kst(stats["bag_first_ns"]) if stats["bag_first_ns"] else None,
        "bag_last_kst": iso_kst(stats["bag_last_ns"]) if stats["bag_last_ns"] else None,
        "duration_s": duration_s,
        "average_rate_hz": (stats["messages"] - 1) / duration_s if duration_s > 0 else 0.0,
        "frames": dict(stats["frames"]),
        "object_count": summarize_numeric(stats["object_counts"]),
        "nonempty_messages": stats["nonempty_messages"],
        "first_nonempty_kst": (
            iso_kst(stats["first_nonempty_ns"]) if stats["first_nonempty_ns"] else None
        ),
        "last_nonempty_kst": (
            iso_kst(stats["last_nonempty_ns"]) if stats["last_nonempty_ns"] else None
        ),
        "bag_minus_header_age_ms": summarize_numeric(stats["ages_ms"]),
    }


request_intervals = transitions(request_samples, lambda value: value != 0)
cruise_intervals = transitions(cruise_samples, bool)
autonomous_intervals = transitions(control_mode_samples, lambda value: value == 1)
operation_enabled_intervals = transitions(operation_samples, lambda value: value["enabled"])

speed_during_request = []
speed_without_request = []
request_index = 0
current_request = 0
for timestamp, longitudinal, _lateral in velocity_samples:
    while request_index < len(request_samples) and request_samples[request_index][0] <= timestamp:
        current_request = request_samples[request_index][1]
        request_index += 1
    (speed_during_request if current_request else speed_without_request).append(longitudinal)

summary = {
    "bag_directory": portable_path(BAG_DIR),
    "database_files": [path.name for path in bag_files()],
    "topic_deserialized_counts": dict(topic_seen_counts),
    "deserialize_errors": dict(deserialize_errors),
    "objects": objects_summary,
    "request_0x630": {
        "samples": len(request_samples),
        "value_counts": dict(collections.Counter(value for _, value in request_samples)),
        "active_intervals": request_intervals,
        "total_active_duration_s": sum(item["duration_s"] for item in request_intervals),
    },
    "cruise": {
        "samples": len(cruise_samples),
        "value_counts": {str(key): value for key, value in collections.Counter(v for _, v in cruise_samples).items()},
        "true_intervals": cruise_intervals,
        "total_true_duration_s": sum(item["duration_s"] for item in cruise_intervals),
    },
    "vehicle_control_mode": {
        "samples": len(control_mode_samples),
        "mode_counts": dict(collections.Counter(value for _, value in control_mode_samples)),
        "mode_1_intervals": autonomous_intervals,
    },
    "operation_mode": {
        "samples": len(operation_samples),
        "values": [
            {"time_kst": iso_kst(timestamp), **value} for timestamp, value in operation_samples
        ],
        "enabled_intervals": operation_enabled_intervals,
    },
    "vehicle_velocity": {
        "longitudinal_mps": summarize_numeric([item[1] for item in velocity_samples]),
        "lateral_mps": summarize_numeric([item[2] for item in velocity_samples]),
        "longitudinal_during_request_mps": summarize_numeric(speed_during_request),
        "longitudinal_without_request_mps": summarize_numeric(speed_without_request),
    },
    "control_command": {
        "velocity_mps": summarize_numeric(control_velocity),
        "acceleration_mps2": summarize_numeric(control_acceleration),
        "steering_tire_angle_rad": summarize_numeric(control_steering),
    },
    "trajectory": {
        "point_count": summarize_numeric(trajectory_point_counts),
        "max_velocity_mps": summarize_numeric(trajectory_max_velocity),
        "min_velocity_mps": summarize_numeric(trajectory_min_velocity),
        "final_velocity_mps": summarize_numeric(trajectory_final_velocity),
    },
    "hazard": {
        "level_counts": dict(hazard_levels),
        "emergency_counts": {str(key): value for key, value in hazard_emergency.items()},
    },
    "mrm": {
        "state_counts": dict(mrm_states),
        "behavior_counts": dict(mrm_behaviors),
    },
    "freshness_ms": {
        "localization_kinematic_bag_minus_header": summarize_numeric(localization_age_ms),
        "localization_acceleration_bag_minus_header": summarize_numeric(acceleration_age_ms),
        "tf_map_to_base_link_bag_minus_header": summarize_numeric(tf_map_base_ages_ms),
    },
}

print(json.dumps(summary, indent=2, sort_keys=True))
