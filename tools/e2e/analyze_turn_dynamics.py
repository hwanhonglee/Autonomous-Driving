#!/usr/bin/env python3
"""Separate VAD path-shape errors from MPC/vehicle tracking errors in a ROS bag."""

from __future__ import annotations

import argparse
import bisect
import json
import math
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError as error:  # pragma: no cover - depends on a sourced ROS environment
    raise SystemExit(
        "ROS 2 Python modules are unavailable. Run `source install/setup.bash` first."
    ) from error


RAW_TOPIC = "/planning/vad_route/selected_raw_trajectory"
FINAL_TOPIC = "/planning/trajectory"
PREDICTED_TOPICS = (
    "/control/trajectory_follower/lateral/predicted_trajectory",
    "/control/trajectory_follower/predicted_trajectory",
)
PREDICTED_TOPIC = PREDICTED_TOPICS[0]
ODOMETRY_TOPIC = "/localization/kinematic_state"
STEERING_TOPIC = "/vehicle/status/steering_status"
DIAGNOSTIC_TOPIC = "/control/trajectory_follower/lateral/diagnostic"
FULL_CONTROL_TOPIC = "/control/trajectory_follower/control_cmd"
LEGACY_CONTROL_TOPIC = "/trajectory_follower/control_cmd"
GATED_CONTROL_TOPIC = "/control/command/control_cmd"
RAW_CONTROL_TOPICS = (FULL_CONTROL_TOPIC, LEGACY_CONTROL_TOPIC)
CONTROL_TOPICS = (*RAW_CONTROL_TOPICS, GATED_CONTROL_TOPIC)
ACTUATION_TOPIC = "/control/command/actuation_cmd"

WHEEL_BASE_M = 2.79
WHEEL_TREAD_M = 1.64


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, type=Path, help="rosbag2 directory")
    parser.add_argument("--route-file", required=True, type=Path)
    parser.add_argument("--result-dir", required=True, type=Path)
    parser.add_argument(
        "--mpc-input-delay-sec",
        type=float,
        default=None,
        help="MPC input_delay used by this run; omit to disable configuration comparison",
    )
    parser.add_argument(
        "--mpc-steer-tau-sec",
        type=float,
        default=None,
        help="MPC vehicle_model_steer_tau used by this run; omit to disable comparison",
    )
    parser.add_argument(
        "--steering-report-mode",
        choices=("legacy_fl", "virtual"),
        default="legacy_fl",
        help="legacy_fl converts CARLA FL wheel reports; virtual uses the report as-is",
    )
    parser.add_argument(
        "--maneuver-lookahead-m",
        type=float,
        default=0.0,
        help=(
            "route-manager maneuver lookahead; include the command-switch approach "
            "in turn geometry diagnostics"
        ),
    )
    parser.add_argument("--wheel-base-m", type=float, default=WHEEL_BASE_M)
    parser.add_argument("--wheel-tread-m", type=float, default=WHEEL_TREAD_M)
    return parser.parse_args()


def _time_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _yaw_from_quaternion(orientation: Any) -> float:
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )


def _duration_sec(duration: Any) -> float:
    return float(duration.sec) + float(duration.nanosec) * 1.0e-9


def _trajectory_record(message: Any, bag_ns: int) -> dict[str, Any]:
    points = list(message.points)
    return {
        "bag_ns": int(bag_ns),
        "stamp_ns": _time_ns(message.header.stamp),
        "frame_id": message.header.frame_id,
        "xy": np.asarray(
            [[point.pose.position.x, point.pose.position.y] for point in points], dtype=float
        ),
        "yaw": np.asarray([_yaw_from_quaternion(point.pose.orientation) for point in points]),
        "speed": np.asarray([point.longitudinal_velocity_mps for point in points], dtype=float),
        "relative_time": np.asarray(
            [_duration_sec(point.time_from_start) for point in points], dtype=float
        ),
    }


def _storage_identifier(bag: Path) -> str:
    metadata_path = bag / "metadata.yaml"
    if not metadata_path.is_file():
        raise FileNotFoundError(f"rosbag metadata does not exist: {metadata_path}")
    with metadata_path.open(encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream)
    information = metadata.get("rosbag2_bagfile_information", {})
    storage_id = information.get("storage_identifier")
    if not storage_id:
        raise RuntimeError(f"storage_identifier is missing from {metadata_path}")
    return str(storage_id)


def _read_bag(bag: Path) -> tuple[dict[str, list[dict[str, Any]]], dict[str, str]]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id=_storage_identifier(bag)),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    requested = {
        RAW_TOPIC,
        FINAL_TOPIC,
        *PREDICTED_TOPICS,
        ODOMETRY_TOPIC,
        STEERING_TOPIC,
        DIAGNOSTIC_TOPIC,
        ACTUATION_TOPIC,
        *CONTROL_TOPICS,
    }
    available = sorted(requested.intersection(topic_types))
    if hasattr(reader, "set_filter"):
        reader.set_filter(rosbag2_py.StorageFilter(topics=available))

    records: dict[str, list[dict[str, Any]]] = {topic: [] for topic in available}
    message_classes = {topic: get_message(topic_types[topic]) for topic in available}

    while reader.has_next():
        topic, serialized, bag_ns = reader.read_next()
        if topic not in message_classes:
            continue
        message = deserialize_message(serialized, message_classes[topic])
        if topic in (RAW_TOPIC, FINAL_TOPIC, *PREDICTED_TOPICS):
            record = _trajectory_record(message, bag_ns)
        elif topic == ODOMETRY_TOPIC:
            record = {
                "bag_ns": int(bag_ns),
                "stamp_ns": _time_ns(message.header.stamp),
                "x": float(message.pose.pose.position.x),
                "y": float(message.pose.pose.position.y),
                "yaw": _yaw_from_quaternion(message.pose.pose.orientation),
                "speed": float(message.twist.twist.linear.x),
                "yaw_rate": float(message.twist.twist.angular.z),
            }
        elif topic == STEERING_TOPIC:
            record = {
                "bag_ns": int(bag_ns),
                "stamp_ns": _time_ns(message.stamp),
                "angle": float(message.steering_tire_angle),
            }
        elif topic == DIAGNOSTIC_TOPIC:
            record = {
                "bag_ns": int(bag_ns),
                "stamp_ns": _time_ns(message.stamp),
                "data": np.asarray(message.data, dtype=float),
            }
        elif topic == ACTUATION_TOPIC:
            record = {
                "bag_ns": int(bag_ns),
                "stamp_ns": _time_ns(message.header.stamp),
                "angle": float(message.actuation.steer_cmd),
            }
        else:
            record = {
                "bag_ns": int(bag_ns),
                "stamp_ns": _time_ns(message.stamp),
                "angle": float(message.lateral.steering_tire_angle),
                "rate": float(message.lateral.steering_tire_rotation_rate),
                "speed": float(message.longitudinal.velocity),
            }
        records[topic].append(record)

    for values in records.values():
        values.sort(key=lambda value: value["bag_ns"])
    return records, topic_types


def _load_route(path: Path) -> tuple[dict[str, Any], np.ndarray, np.ndarray, np.ndarray]:
    with path.open(encoding="utf-8") as stream:
        route = json.load(stream)
    points = route.get("route", [])
    if len(points) < 2:
        raise ValueError(f"route has fewer than two points: {path}")
    xy = np.asarray([[point["x"], point["y"]] for point in points], dtype=float)
    if all("distance_m" in point for point in points):
        progress = np.asarray([point["distance_m"] for point in points], dtype=float)
    else:
        progress = _arc_length(xy)
    command = np.asarray([point.get("vad_command", 3) for point in points], dtype=int)
    return route, xy, progress, command


def _arc_length(xy: np.ndarray) -> np.ndarray:
    if len(xy) == 0:
        return np.empty(0, dtype=float)
    segment = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    return np.concatenate(([0.0], np.cumsum(segment)))


def _project_to_polyline(
    points: np.ndarray, line: np.ndarray, line_progress: np.ndarray | None = None
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Return signed distance, progress, nearest heading, and nearest points."""
    points = np.atleast_2d(np.asarray(points, dtype=float))
    line = np.asarray(line, dtype=float)
    if len(points) == 0 or len(line) < 2:
        empty = np.full(len(points), np.nan)
        return empty, empty.copy(), empty.copy(), np.full((len(points), 2), np.nan)

    start = line[:-1]
    delta = line[1:] - start
    length_sq = np.einsum("ij,ij->i", delta, delta)
    valid = length_sq > 1.0e-12
    if not np.any(valid):
        empty = np.full(len(points), np.nan)
        return empty, empty.copy(), empty.copy(), np.full((len(points), 2), np.nan)
    start = start[valid]
    delta = delta[valid]
    length_sq = length_sq[valid]
    segment_length = np.sqrt(length_sq)

    relative = points[:, None, :] - start[None, :, :]
    fraction = np.einsum("pij,ij->pi", relative, delta) / length_sq[None, :]
    fraction = np.clip(fraction, 0.0, 1.0)
    nearest = start[None, :, :] + fraction[:, :, None] * delta[None, :, :]
    difference = points[:, None, :] - nearest
    squared_distance = np.einsum("pij,pij->pi", difference, difference)
    segment_index = np.argmin(squared_distance, axis=1)
    point_index = np.arange(len(points))
    selected_nearest = nearest[point_index, segment_index]
    selected_difference = points - selected_nearest
    tangent = delta[segment_index] / segment_length[segment_index, None]
    signed_distance = tangent[:, 0] * selected_difference[:, 1] - tangent[:, 1] * selected_difference[:, 0]
    heading = np.arctan2(tangent[:, 1], tangent[:, 0])

    if line_progress is None:
        original_progress = _arc_length(line)
    else:
        original_progress = np.asarray(line_progress, dtype=float)
    valid_segment_indices = np.flatnonzero(valid)
    original_segment = valid_segment_indices[segment_index]
    selected_fraction = fraction[point_index, segment_index]
    progress = original_progress[original_segment] + selected_fraction * (
        original_progress[original_segment + 1] - original_progress[original_segment]
    )
    return signed_distance, progress, heading, selected_nearest


def _wrap_angle(values: np.ndarray | float) -> np.ndarray:
    return (np.asarray(values) + np.pi) % (2.0 * np.pi) - np.pi


def _polyline_curvature(xy: np.ndarray, sample_distance_m: float = 0.8) -> np.ndarray:
    xy = np.asarray(xy, dtype=float)
    output = np.full(len(xy), np.nan)
    if len(xy) < 3:
        return output
    segment = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    positive = segment[segment > 1.0e-4]
    if len(positive) == 0:
        return output
    stride = max(1, int(round(sample_distance_m / float(np.median(positive)))))
    if 2 * stride >= len(xy):
        stride = max(1, (len(xy) - 1) // 2)
    for index in range(stride, len(xy) - stride):
        a = xy[index - stride]
        b = xy[index]
        c = xy[index + stride]
        ab = b - a
        bc = c - b
        ac = c - a
        denominator = np.linalg.norm(ab) * np.linalg.norm(bc) * np.linalg.norm(ac)
        if denominator > 1.0e-9:
            output[index] = 2.0 * np.cross(ab, ac) / denominator
    return output


def _latest_index(records: list[dict[str, Any]], bag_ns: int) -> int:
    times = [record["bag_ns"] for record in records]
    return bisect.bisect_right(times, bag_ns) - 1


def _physics_ns(record: dict[str, Any]) -> int:
    stamp_ns = int(record.get("stamp_ns", 0))
    return stamp_ns if stamp_ns > 0 else int(record["bag_ns"])


def _summary(values: list[float] | np.ndarray) -> dict[str, float | int | None]:
    array = np.asarray(values, dtype=float)
    array = array[np.isfinite(array)]
    if len(array) == 0:
        return {"count": 0, "mean": None, "rmse": None, "p95_abs": None, "max_abs": None}
    return {
        "count": int(len(array)),
        "mean": float(np.mean(array)),
        "rmse": float(np.sqrt(np.mean(np.square(array)))),
        "p95_abs": float(np.percentile(np.abs(array), 95)),
        "max_abs": float(np.max(np.abs(array))),
    }


def _deduplicate_series(times: np.ndarray, values: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    order = np.argsort(times, kind="stable")
    times = np.asarray(times, dtype=float)[order]
    values = np.asarray(values, dtype=float)[order]
    if len(times) < 2:
        return times, values
    keep = np.r_[times[1:] != times[:-1], True]
    return times[keep], values[keep]


def _pose_derived_yaw_rate(times: np.ndarray, yaw: np.ndarray) -> np.ndarray:
    times = np.asarray(times, dtype=float)
    yaw = np.asarray(yaw, dtype=float)
    if len(times) < 3 or len(times) != len(yaw):
        return np.full(len(times), np.nan)
    order = np.argsort(times, kind="stable")
    sorted_time = times[order]
    sorted_yaw = yaw[order]
    keep = np.r_[sorted_time[1:] != sorted_time[:-1], True]
    unique_time = sorted_time[keep]
    unique_yaw = np.unwrap(sorted_yaw[keep])
    if len(unique_time) < 3:
        return np.full(len(times), np.nan)
    rate = np.gradient(unique_yaw, unique_time)
    median_dt = float(np.median(np.diff(unique_time)))
    window = max(1, int(round(0.15 / max(median_dt, 1.0e-3))))
    if window > 1:
        window = min(window, len(rate))
        left = window // 2
        right = window - 1 - left
        padded = np.pad(rate, (left, right), mode="edge")
        rate = np.convolve(padded, np.ones(window) / window, mode="valid")
    return np.interp(times, unique_time, rate)


def _fl_wheel_to_virtual(
    angle: np.ndarray,
    wheel_base_m: float = WHEEL_BASE_M,
    wheel_tread_m: float = WHEEL_TREAD_M,
) -> np.ndarray:
    """Convert CARLA's FL wheel report to the bicycle-model virtual tire angle."""
    angle = np.asarray(angle, dtype=float)
    output = np.zeros_like(angle)
    for index, value in np.ndenumerate(angle):
        magnitude = abs(float(value))
        if magnitude < 1.0e-6:
            continue
        wheel_radius = wheel_base_m / math.tan(magnitude)
        if value > 0.0:  # FL is the inner wheel in a left turn.
            center_radius = wheel_radius + 0.5 * wheel_tread_m
        else:  # FL is the outer wheel in a right turn.
            center_radius = wheel_radius - 0.5 * wheel_tread_m
        if center_radius > 1.0e-3:
            output[index] = math.copysign(
                math.atan(wheel_base_m / center_radius), value
            )
    return output


def _correlation(left: np.ndarray, right: np.ndarray) -> float | None:
    mask = np.isfinite(left) & np.isfinite(right)
    if np.count_nonzero(mask) < 5:
        return None
    x = left[mask]
    y = right[mask]
    if np.std(x) < 1.0e-9 or np.std(y) < 1.0e-9:
        return None
    return float(np.corrcoef(x, y)[0, 1])


def _fit_steering_dynamics(
    command_time: np.ndarray,
    command: np.ndarray,
    steering_time: np.ndarray,
    steering_virtual: np.ndarray,
) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    command_time, command = _deduplicate_series(command_time, command)
    steering_time, steering_virtual = _deduplicate_series(steering_time, steering_virtual)
    if len(command_time) < 5 or len(steering_time) < 5:
        return {"available": False}, {}
    begin = max(command_time[0], steering_time[0])
    end = min(command_time[-1], steering_time[-1])
    if end - begin < 1.0:
        return {"available": False}, {}
    intervals = np.r_[np.diff(command_time), np.diff(steering_time)]
    intervals = intervals[intervals > 1.0e-5]
    dt = float(np.clip(np.median(intervals), 0.01, 0.05)) if len(intervals) else 0.05
    grid = np.arange(begin, end + 0.5 * dt, dt)
    input_angle = np.interp(grid, command_time, command)
    measured = np.interp(grid, steering_time, steering_virtual)
    active = (np.abs(input_angle) > 0.015) | (np.abs(measured) > 0.015)
    if np.count_nonzero(active) < 20:
        active = np.ones_like(grid, dtype=bool)

    window = max(3, int(round(0.15 / dt)))
    kernel = np.ones(window) / window
    input_smooth = np.convolve(input_angle, kernel, mode="same")
    measured_smooth = np.convolve(measured, kernel, mode="same")
    input_derivative = np.gradient(input_smooth, dt)
    measured_derivative = np.gradient(measured_smooth, dt)

    best_angle = (-math.inf, 0.0)
    best_derivative = (-math.inf, 0.0)
    delays = np.arange(0.0, 0.501, dt)
    for delay in delays:
        shifted_angle = np.interp(grid - delay, grid, input_angle, left=np.nan, right=np.nan)
        shifted_derivative = np.interp(
            grid - delay, grid, input_derivative, left=np.nan, right=np.nan
        )
        angle_corr = _correlation(shifted_angle[active], measured[active])
        derivative_active = active & (
            (np.abs(shifted_derivative) > 0.01) | (np.abs(measured_derivative) > 0.01)
        )
        derivative_corr = _correlation(
            shifted_derivative[derivative_active], measured_derivative[derivative_active]
        )
        if angle_corr is not None and angle_corr > best_angle[0]:
            best_angle = (angle_corr, float(delay))
        if derivative_corr is not None and derivative_corr > best_derivative[0]:
            best_derivative = (derivative_corr, float(delay))

    best_fit: dict[str, Any] | None = None
    best_prediction: np.ndarray | None = None
    tau_values = np.linspace(0.02, 0.60, 40)
    for delay in delays:
        delayed = np.interp(grid - delay, grid, input_angle, left=input_angle[0], right=input_angle[-1])
        for tau in tau_values:
            filtered = np.empty_like(delayed)
            filtered[0] = delayed[0]
            alpha = dt / (float(tau) + dt)
            for index in range(1, len(filtered)):
                filtered[index] = filtered[index - 1] + alpha * (
                    delayed[index] - filtered[index - 1]
                )
            denominator = float(np.dot(filtered[active], filtered[active]))
            if denominator < 1.0e-9:
                continue
            gain = float(np.dot(filtered[active], measured[active]) / denominator)
            gain = float(np.clip(gain, 0.1, 3.0))
            prediction = gain * filtered
            rmse = float(np.sqrt(np.mean(np.square(prediction[active] - measured[active]))))
            if best_fit is None or rmse < best_fit["rmse_rad"]:
                best_fit = {
                    "delay_sec": float(delay),
                    "time_constant_sec": float(tau),
                    "gain": gain,
                    "rmse_rad": rmse,
                }
                best_prediction = prediction

    shifted_at_derivative_delay = np.interp(
        grid - best_derivative[1], grid, input_angle, left=np.nan, right=np.nan
    )
    unity_error = shifted_at_derivative_delay - measured
    unity_mask = active & np.isfinite(unity_error)
    metrics: dict[str, Any] = {
        "available": True,
        "sample_interval_sec": dt,
        "angle_correlation": None if best_angle[0] == -math.inf else best_angle[0],
        "angle_correlation_delay_sec": best_angle[1],
        "derivative_correlation": (
            None if best_derivative[0] == -math.inf else best_derivative[0]
        ),
        "derivative_correlation_delay_sec": best_derivative[1],
        "unity_gain_rmse_rad": (
            float(np.sqrt(np.mean(np.square(unity_error[unity_mask]))))
            if np.any(unity_mask)
            else None
        ),
        "command_peak_abs_rad": float(np.max(np.abs(input_angle[active]))),
        "measured_virtual_peak_abs_rad": float(np.max(np.abs(measured[active]))),
    }
    if best_fit is not None:
        metrics["first_order_fit"] = best_fit
    plot_data = {
        "time": grid,
        "command": input_angle,
        "measured_virtual": measured,
        "model_prediction": (
            best_prediction if best_prediction is not None else np.full_like(grid, np.nan)
        ),
        "command_derivative": input_derivative,
        "measured_derivative": measured_derivative,
    }
    return metrics, plot_data


def _record_times(records: list[dict[str, Any]]) -> np.ndarray:
    """Return ROS message time for physics calculations, with receipt-time fallback."""
    return np.asarray([_physics_ns(record) * 1.0e-9 for record in records], dtype=float)


def _receipt_times(records: list[dict[str, Any]]) -> np.ndarray:
    return np.asarray([record["bag_ns"] * 1.0e-9 for record in records], dtype=float)


def _trajectory_route_profile(
    record: dict[str, Any], route_xy: np.ndarray, route_progress: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    trajectory_xy = _clip_trajectory_to_route_horizon(
        record["xy"], route_xy, route_progress
    )
    trajectory_xy = _densify_polyline(trajectory_xy)
    if len(trajectory_xy) < 2:
        return np.empty(0), np.empty(0)
    offset, progress, _, _ = _project_to_polyline(
        trajectory_xy, route_xy, route_progress
    )
    finite = np.isfinite(offset) & np.isfinite(progress)
    if np.count_nonzero(finite) < 2:
        return np.empty(0), np.empty(0)
    return _deduplicate_series(progress[finite], offset[finite])


def _clip_trajectory_to_route_horizon(
    trajectory_xy: np.ndarray,
    route_xy: np.ndarray,
    route_progress: np.ndarray,
) -> np.ndarray:
    """Approximate route-end truncation so metrics use the controllable horizon."""
    trajectory_xy = np.asarray(trajectory_xy, dtype=float)
    if len(trajectory_xy) < 2 or len(route_progress) < 2:
        return trajectory_xy

    _, first_progress, _, _ = _project_to_polyline(
        trajectory_xy[:1], route_xy, route_progress
    )
    if not len(first_progress) or not math.isfinite(float(first_progress[0])):
        return trajectory_xy

    distances = _arc_length(trajectory_xy)
    if not len(distances) or distances[-1] <= 1.0e-9:
        return trajectory_xy
    remaining_m = max(0.0, float(route_progress[-1] - first_progress[0]))
    minimum_horizon_m = min(0.1, float(distances[-1]))
    cutoff_m = min(float(distances[-1]), max(remaining_m, minimum_horizon_m))
    if cutoff_m >= distances[-1] - 1.0e-6:
        return trajectory_xy

    right = int(np.searchsorted(distances, cutoff_m, side="right"))
    if right > 0 and abs(float(distances[right - 1] - cutoff_m)) <= 1.0e-6:
        return trajectory_xy[:right]
    left = max(0, right - 1)
    right = min(len(trajectory_xy) - 1, left + 1)
    span = float(distances[right] - distances[left])
    if span <= 1.0e-9:
        return trajectory_xy[: max(2, right + 1)]
    ratio = (cutoff_m - float(distances[left])) / span
    terminal = trajectory_xy[left] + ratio * (
        trajectory_xy[right] - trajectory_xy[left]
    )
    return np.vstack((trajectory_xy[: left + 1], terminal))


def _densify_polyline(xy: np.ndarray, interval_m: float = 0.25) -> np.ndarray:
    """Sample a sparse trajectory polyline before projecting it into route coordinates."""
    xy = np.asarray(xy, dtype=float)
    if len(xy) < 2:
        return xy
    distances = _arc_length(xy)
    if distances[-1] <= interval_m:
        return xy
    samples = np.arange(0.0, float(distances[-1]), interval_m)
    if not len(samples) or distances[-1] - samples[-1] > 1.0e-9:
        samples = np.append(samples, distances[-1])
    return np.column_stack(
        (
            np.interp(samples, distances, xy[:, 0]),
            np.interp(samples, distances, xy[:, 1]),
        )
    )


def _uniform_route_offsets(
    record: dict[str, Any],
    route_xy: np.ndarray,
    route_progress: np.ndarray,
    sample_count: int = 41,
    progress_ranges: list[tuple[float, float]] | None = None,
) -> np.ndarray:
    progress, offset = _trajectory_route_profile(record, route_xy, route_progress)
    if len(progress) < 2 or progress[-1] - progress[0] < 1.0e-3:
        return np.empty(0)
    ranges = progress_ranges or [(float(progress[0]), float(progress[-1]))]
    grids = []
    for lower, upper in ranges:
        start = max(float(progress[0]), lower)
        end = min(float(progress[-1]), upper)
        if end - start >= 0.25:
            grids.append(np.linspace(start, end, sample_count))
    if not grids:
        return np.empty(0)
    return np.interp(np.concatenate(grids), progress, offset)


def _trajectory_route_metrics(
    records: list[dict[str, Any]],
    route_xy: np.ndarray,
    route_progress: np.ndarray,
    progress_ranges: list[tuple[float, float]] | None = None,
) -> tuple[dict[str, Any], list[float], list[float]]:
    all_error: list[float] = []
    snapshot_p95: list[float] = []
    peak_curvature: list[float] = []
    for record in records:
        error = _uniform_route_offsets(
            record, route_xy, route_progress, progress_ranges=progress_ranges
        )
        if not len(error):
            continue
        all_error.extend(error[np.isfinite(error)].tolist())
        snapshot_p95.append(float(np.percentile(np.abs(error), 95)))
        curvature = _polyline_curvature(
            _clip_trajectory_to_route_horizon(
                record["xy"], route_xy, route_progress
            )
        )
        finite = curvature[np.isfinite(curvature)]
        peak_curvature.append(float(np.max(np.abs(finite))) if len(finite) else math.nan)
    metrics = _summary(all_error)
    metrics["route_progress_ranges_m"] = progress_ranges
    curvature_summary = _summary(peak_curvature)
    metrics["snapshot_p95_route_offset_m"] = _summary(snapshot_p95)
    metrics["snapshot_peak_curvature_per_m"] = curvature_summary
    return metrics, all_error, peak_curvature


def _pair_trajectory_geometry(
    source: list[dict[str, Any]],
    target: list[dict[str, Any]],
    route_xy: np.ndarray,
    route_progress: np.ndarray,
    progress_ranges: list[tuple[float, float]] | None = None,
) -> tuple[dict[str, Any], list[dict[str, float]]]:
    samples: list[dict[str, float]] = []
    for target_record in target:
        index = _latest_index(source, target_record["bag_ns"])
        if index < 0:
            continue
        source_record = source[index]
        age = (_physics_ns(target_record) - _physics_ns(source_record)) * 1.0e-9
        if age < -1.0e-6 or age > 1.0:
            continue
        source_progress, source_offset = _trajectory_route_profile(
            source_record, route_xy, route_progress
        )
        target_progress, target_offset = _trajectory_route_profile(
            target_record, route_xy, route_progress
        )
        if len(source_progress) < 2 or len(target_progress) < 2:
            continue
        common_lower = max(float(source_progress[0]), float(target_progress[0]))
        common_upper = min(float(source_progress[-1]), float(target_progress[-1]))
        ranges = progress_ranges or [(common_lower, common_upper)]
        grids = []
        overlap = 0.0
        for lower, upper in ranges:
            start = max(common_lower, lower)
            end = min(common_upper, upper)
            length = end - start
            if length < 0.25:
                continue
            overlap += length
            grids.append(
                np.linspace(start, end, max(2, int(math.ceil(length / 0.25)) + 1))
            )
        if not grids:
            continue
        grid = np.concatenate(grids)
        difference = np.abs(
            np.interp(grid, target_progress, target_offset)
            - np.interp(grid, source_progress, source_offset)
        )
        samples.append(
            {
                "bag_sec": target_record["bag_ns"] * 1.0e-9,
                "time_sec": _physics_ns(target_record) * 1.0e-9,
                "age_sec": age,
                "common_progress_p95_m": float(np.percentile(difference, 95)),
                "common_progress_max_m": float(np.max(difference)),
                "overlap_m": overlap,
            }
        )
    return {
        "pair_count": len(samples),
        "route_progress_ranges_m": progress_ranges,
        "common_progress_p95_m": _summary(
            [sample["common_progress_p95_m"] for sample in samples]
        ),
        "common_progress_max_m": _summary(
            [sample["common_progress_max_m"] for sample in samples]
        ),
        "common_progress_overlap_m": _summary([sample["overlap_m"] for sample in samples]),
        "source_age_sec": _summary([sample["age_sec"] for sample in samples]),
    }, samples


def _tracking_metrics(
    odometry: list[dict[str, Any]],
    final: list[dict[str, Any]],
    route_xy: np.ndarray,
    route_progress: np.ndarray,
) -> tuple[dict[str, Any], list[dict[str, float]]]:
    samples: list[dict[str, float]] = []
    route_curvature = _polyline_curvature(route_xy)
    curvature_valid = np.isfinite(route_curvature) & np.isfinite(route_progress)
    for odom in odometry:
        route_error, progress, route_heading, _ = _project_to_polyline(
            np.asarray([[odom["x"], odom["y"]]]), route_xy, route_progress
        )
        sample = {
            "bag_sec": odom["bag_ns"] * 1.0e-9,
            "time_sec": _physics_ns(odom) * 1.0e-9,
            "route_cte_m": float(route_error[0]),
            "route_progress_m": float(progress[0]),
            "route_yaw_error_rad": float(_wrap_angle(odom["yaw"] - route_heading[0])),
            "speed_mps": odom["speed"],
            "yaw_rate_rps": odom["yaw_rate"],
            "route_curvature_per_m": (
                float(
                    np.interp(
                        progress[0],
                        route_progress[curvature_valid],
                        route_curvature[curvature_valid],
                    )
                )
                if np.count_nonzero(curvature_valid) >= 2
                else math.nan
            ),
            "final_cte_m": math.nan,
            "final_yaw_error_rad": math.nan,
            "trajectory_age_sec": math.nan,
        }
        final_index = _latest_index(final, odom["bag_ns"])
        if final_index >= 0 and len(final[final_index]["xy"]) >= 2:
            trajectory = final[final_index]
            age = (_physics_ns(odom) - _physics_ns(trajectory)) * 1.0e-9
            if -1.0e-6 <= age <= 1.0:
                error, _, heading, _ = _project_to_polyline(
                    np.asarray([[odom["x"], odom["y"]]]), trajectory["xy"]
                )
                sample["final_cte_m"] = float(error[0])
                sample["final_yaw_error_rad"] = float(
                    _wrap_angle(odom["yaw"] - heading[0])
                )
                sample["trajectory_age_sec"] = age
        samples.append(sample)
    sample_time = _record_times(odometry)
    sample_yaw = np.asarray([odom["yaw"] for odom in odometry])
    interpolated_rate = _pose_derived_yaw_rate(sample_time, sample_yaw)
    for sample, pose_rate in zip(samples, interpolated_rate):
        sample["pose_yaw_rate_rps"] = float(pose_rate)

    twist_rate = np.asarray([sample["yaw_rate_rps"] for sample in samples])
    pose_rate = np.asarray([sample["pose_yaw_rate_rps"] for sample in samples])
    speed = np.asarray([sample["speed_mps"] for sample in samples])
    ratio_mask = (
        np.isfinite(twist_rate)
        & np.isfinite(pose_rate)
        & (np.abs(pose_rate) > 0.03)
        & (np.abs(speed) > 0.3)
    )
    twist_pose_gain = (
        float(np.median(twist_rate[ratio_mask] / pose_rate[ratio_mask]))
        if np.count_nonzero(ratio_mask) >= 5
        else None
    )
    return {
        "actual_to_final_cte_m": _summary([sample["final_cte_m"] for sample in samples]),
        "actual_to_final_yaw_error_rad": _summary(
            [sample["final_yaw_error_rad"] for sample in samples]
        ),
        "actual_to_route_cte_m": _summary([sample["route_cte_m"] for sample in samples]),
        "actual_to_route_yaw_error_rad": _summary(
            [sample["route_yaw_error_rad"] for sample in samples]
        ),
        "final_trajectory_age_sec": _summary(
            [sample["trajectory_age_sec"] for sample in samples]
        ),
        "pose_derived_yaw_rate_rps": _summary(
            [sample["pose_yaw_rate_rps"] for sample in samples]
        ),
        "odometry_twist_yaw_rate_rps": _summary(
            [sample["yaw_rate_rps"] for sample in samples]
        ),
        "odometry_twist_to_pose_yaw_rate_gain": twist_pose_gain,
        "yaw_rate_note": (
            "Pose-derived yaw rate is used in plots. A gain magnitude near 57.3 (possibly "
            "with opposite sign) indicates a CARLA deg/s and ROS rad/s convention mismatch."
        ),
    }, samples


def _prediction_future_metrics(
    predicted: list[dict[str, Any]], odometry: list[dict[str, Any]]
) -> tuple[dict[str, Any], dict[float, list[dict[str, float]]]]:
    horizons = (0.5, 1.0, 2.0)
    output: dict[float, list[dict[str, float]]] = {horizon: [] for horizon in horizons}
    if len(odometry) < 2:
        return {}, output
    odom_time = _record_times(odometry)
    odom_x = np.asarray([record["x"] for record in odometry])
    odom_y = np.asarray([record["y"] for record in odometry])
    odom_speed = np.asarray([record["speed"] for record in odometry])
    order = np.argsort(odom_time, kind="stable")
    odom_time = odom_time[order]
    odom_x = odom_x[order]
    odom_y = odom_y[order]
    odom_speed = odom_speed[order]
    keep = np.r_[odom_time[1:] != odom_time[:-1], True]
    odom_time = odom_time[keep]
    odom_x = odom_x[keep]
    odom_y = odom_y[keep]
    odom_speed = odom_speed[keep]
    for record in predicted:
        if not len(record["xy"]) or not len(record["relative_time"]):
            continue
        origin = _physics_ns(record) * 1.0e-9
        if abs(float(np.interp(origin, odom_time, odom_speed))) < 0.3:
            continue
        for horizon in horizons:
            point_index = int(np.argmin(np.abs(record["relative_time"] - horizon)))
            point_horizon = float(record["relative_time"][point_index])
            if abs(point_horizon - horizon) > 0.30:
                continue
            future_time = origin + point_horizon
            if future_time < odom_time[0] or future_time > odom_time[-1]:
                continue
            actual = np.asarray(
                [np.interp(future_time, odom_time, odom_x), np.interp(future_time, odom_time, odom_y)]
            )
            error = float(np.linalg.norm(record["xy"][point_index] - actual))
            output[horizon].append({"time_sec": origin, "error_m": error})
    metrics = {
        f"horizon_{horizon:.1f}_sec_error_m": _summary(
            [sample["error_m"] for sample in output[horizon]]
        )
        for horizon in horizons
    }
    return metrics, output


def _diagnostic_metrics(records: list[dict[str, Any]]) -> dict[str, Any]:
    valid = [record for record in records if len(record["data"]) >= 18]
    if not valid:
        return {"available": False, "count": 0}
    values = np.vstack([record["data"][:18] for record in valid])
    return {
        "available": True,
        "count": len(valid),
        "final_steer_command_rad": _summary(values[:, 0]),
        "raw_mpc_steer_command_rad": _summary(values[:, 1]),
        "feedforward_filtered_steer_rad": _summary(values[:, 2]),
        "feedforward_raw_steer_rad": _summary(values[:, 3]),
        "controller_current_steer_rad": _summary(values[:, 4]),
        "lateral_error_m": _summary(values[:, 5]),
        "yaw_error_rad": _summary(values[:, 8]),
        "path_curvature_per_m": _summary(values[:, 14]),
        "raw_path_curvature_per_m": _summary(values[:, 15]),
        "predicted_steer_rad": _summary(values[:, 16]),
    }


def _interpolate_record_value(
    records: list[dict[str, Any]], time_sec: float, key: str
) -> float | None:
    if not records:
        return None
    times = _record_times(records)
    values = np.asarray([record[key] for record in records], dtype=float)
    times, values = _deduplicate_series(times, values)
    if not len(times) or time_sec < times[0] or time_sec > times[-1]:
        return None
    return float(np.interp(time_sec, times, values))


def _turn_progress_ranges(
    route_progress: np.ndarray,
    route_command: np.ndarray,
    margin_m: float = 2.0,
    maneuver_lookahead_m: float = 0.0,
) -> list[tuple[float, float]]:
    if not math.isfinite(maneuver_lookahead_m) or maneuver_lookahead_m < 0.0:
        raise ValueError("maneuver lookahead must be finite and non-negative")
    indices = np.flatnonzero(np.isin(route_command, (0, 1)))
    if not len(indices):
        return []
    groups = np.split(indices, np.flatnonzero(np.diff(indices) > 1) + 1)
    return [
        (
            float(route_progress[group[0]]) - maneuver_lookahead_m - margin_m,
            float(route_progress[group[-1]]) + margin_m,
        )
        for group in groups
        if len(group)
    ]


def _progress_in_ranges(progress: float, ranges: list[tuple[float, float]]) -> bool:
    return any(lower <= progress <= upper for lower, upper in ranges)


def _profile_value_at(
    progress: np.ndarray, values: np.ndarray, target: float, tolerance_m: float = 0.25
) -> float | None:
    if len(progress) < 2 or target < progress[0] - tolerance_m or target > progress[-1] + tolerance_m:
        return None
    return float(np.interp(float(np.clip(target, progress[0], progress[-1])), progress, values))


def _trajectory_frenet_context(
    snapshot: dict[str, Any],
    actual_route_offset_m: float,
    actual_progress_m: float,
    route_xy: np.ndarray,
    route_progress: np.ndarray,
    lookahead_m: float = 5.0,
) -> dict[str, float | None]:
    progress, offset = _trajectory_route_profile(snapshot, route_xy, route_progress)
    path_offset = _profile_value_at(progress, offset, actual_progress_m)
    lookahead_progress = min(float(route_progress[-1]), actual_progress_m + lookahead_m)
    lookahead_offset = _profile_value_at(progress, offset, lookahead_progress)
    if path_offset is None:
        tracking_offset = None
        residual = None
    else:
        tracking_offset = actual_route_offset_m - path_offset
        residual = actual_route_offset_m - path_offset - tracking_offset
    return {
        "path_offset_at_actual_progress_m": path_offset,
        "actual_minus_path_offset_m": tracking_offset,
        "decomposition_residual_m": residual,
        "lookahead_progress_m": lookahead_progress,
        "path_offset_at_lookahead_m": lookahead_offset,
    }


def _causal_record_value(
    records: list[dict[str, Any]], bag_ns: int, key: str
) -> float | None:
    index = _latest_index(records, bag_ns)
    if index < 0:
        return None
    return float(records[index][key])


def _peak_corner_context(
    tracking: list[dict[str, float]],
    raw: list[dict[str, Any]],
    final: list[dict[str, Any]],
    control: list[dict[str, Any]],
    actuation: list[dict[str, Any]],
    steering: list[dict[str, Any]],
    diagnostic: list[dict[str, Any]],
    odometry: list[dict[str, Any]],
    route_xy: np.ndarray,
    route_progress: np.ndarray,
    route_command: np.ndarray,
    t0: float,
    steering_report_mode: str,
    wheel_base_m: float,
    wheel_tread_m: float,
) -> dict[str, Any]:
    turn_ranges = _turn_progress_ranges(route_progress, route_command)
    valid = [
        sample
        for sample in tracking
        if math.isfinite(sample["route_cte_m"])
        and abs(sample["speed_mps"]) > 0.3
        and _progress_in_ranges(sample["route_progress_m"], turn_ranges)
    ]
    if not valid:
        return {
            "available": False,
            "reason": "no moving odometry sample was inside a commanded turn window",
        }
    inward = [
        sample
        for sample in valid
        if math.isfinite(sample["route_curvature_per_m"])
        and abs(sample["route_curvature_per_m"]) > 1.0e-4
        and sample["route_cte_m"] * math.copysign(1.0, sample["route_curvature_per_m"]) > 0.0
    ]
    if inward:
        peak = max(
            inward,
            key=lambda sample: sample["route_cte_m"]
            * math.copysign(1.0, sample["route_curvature_per_m"]),
        )
        deviation_type = "inward_corner_cut"
    else:
        peak = max(valid, key=lambda sample: abs(sample["route_cte_m"]))
        deviation_type = "outward_or_unclassified_turn_deviation"
    bag_sec = peak["bag_sec"]
    time_sec = peak["time_sec"]
    odom_record = min(odometry, key=lambda record: abs(record["bag_ns"] * 1.0e-9 - bag_sec))
    output: dict[str, Any] = {
        "available": True,
        "selection": deviation_type,
        "receipt_time_sec": bag_sec,
        "simulation_time_sec": time_sec,
        "elapsed_sec": time_sec - t0,
        "route_progress_m": peak["route_progress_m"],
        "actual_to_route_signed_cte_m": peak["route_cte_m"],
        "actual_to_final_signed_cte_m": peak["final_cte_m"],
        "actual_route_yaw_error_rad": peak["route_yaw_error_rad"],
        "actual_final_yaw_error_rad": peak["final_yaw_error_rad"],
        "speed_mps": peak["speed_mps"],
        "pose_yaw_rate_rps": peak["pose_yaw_rate_rps"],
        "route_curvature_per_m": peak["route_curvature_per_m"],
        "turn_progress_ranges_m": turn_ranges,
        "route_frenet": {
            "actual_route_offset_m": peak["route_cte_m"],
            "lookahead_distance_m": 5.0,
        },
    }

    for label, trajectories in (("raw", raw), ("final", final)):
        snapshot = _trajectory_at(trajectories, bag_sec)
        if snapshot is None or len(snapshot["xy"]) < 2:
            continue
        age = (_physics_ns(odom_record) - _physics_ns(snapshot)) * 1.0e-9
        if age < -1.0e-6 or age > 1.0:
            continue
        frenet = _trajectory_frenet_context(
            snapshot,
            peak["route_cte_m"],
            peak["route_progress_m"],
            route_xy,
            route_progress,
        )
        frenet["trajectory_age_sec"] = age
        output["route_frenet"][label] = frenet

    command_angle = _causal_record_value(control, odom_record["bag_ns"], "angle")
    actuation_angle = _causal_record_value(actuation, odom_record["bag_ns"], "angle")
    steering_angle = _interpolate_record_value(steering, time_sec, "angle")
    output["mpc_control_command_rad"] = command_angle
    output["actuation_steer_command"] = actuation_angle
    output["steering_report_rad"] = steering_angle
    output["steering_report_as_virtual_rad"] = (
        float(
            _fl_wheel_to_virtual(
                np.asarray([steering_angle]), wheel_base_m, wheel_tread_m
            )[0]
        )
        if steering_angle is not None and steering_report_mode == "legacy_fl"
        else steering_angle
    )
    diagnostic_index = _latest_index(diagnostic, odom_record["bag_ns"])
    if diagnostic_index >= 0 and len(diagnostic[diagnostic_index]["data"]) >= 18:
        data = diagnostic[diagnostic_index]["data"]
        output["mpc_diagnostic"] = {
            "final_steer_command_rad": float(data[0]),
            "feedforward_steer_rad": float(data[2]),
            "controller_reported_steer_rad": float(data[4]),
            "lateral_error_m": float(data[5]),
            "yaw_error_rad": float(data[8]),
            "required_yaw_rate_from_path_rps": float(data[13]),
            "path_curvature_per_m": float(data[14]),
        }
    return output


def _representative_time(
    route_progress: np.ndarray,
    route_command: np.ndarray,
    route_xy: np.ndarray,
    tracking: list[dict[str, float]],
) -> tuple[float, float]:
    turn_indices = np.flatnonzero(np.isin(route_command, (0, 1)))
    route_curvature = _polyline_curvature(route_xy)
    if len(turn_indices):
        target_progress = float(np.median(route_progress[turn_indices]))
    elif np.any(np.isfinite(route_curvature)):
        target_progress = float(route_progress[int(np.nanargmax(np.abs(route_curvature)))])
    else:
        target_progress = float(np.median(route_progress))
    if not tracking:
        return target_progress, math.nan
    progress = np.asarray([sample["route_progress_m"] for sample in tracking])
    index = int(np.nanargmin(np.abs(progress - target_progress)))
    return target_progress, tracking[index]["bag_sec"]


def _trajectory_at(records: list[dict[str, Any]], bag_sec: float) -> dict[str, Any] | None:
    index = _latest_index(records, int(round(bag_sec * 1.0e9)))
    return records[index] if index >= 0 else None


def _metric_value(metrics: dict[str, Any], *keys: str) -> float | None:
    value: Any = metrics
    for key in keys:
        if not isinstance(value, dict) or key not in value:
            return None
        value = value[key]
    if value is None or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        return None
    return float(value)


def _classify(
    metrics: dict[str, Any],
    mpc_input_delay_sec: float | None = None,
    mpc_steer_tau_sec: float | None = None,
) -> dict[str, Any]:
    reasons: list[str] = []
    planning_score = 0
    control_score = 0

    raw_final = _metric_value(
        metrics, "raw_to_final_geometry", "common_progress_p95_m", "p95_abs"
    )
    raw_route = _metric_value(metrics, "raw_path", "p95_abs")
    final_route = _metric_value(metrics, "final_path", "p95_abs")
    actual_final = _metric_value(
        metrics, "tracking", "actual_to_final_cte_m", "p95_abs"
    )
    actual_final_peak = _metric_value(
        metrics, "tracking", "actual_to_final_cte_m", "max_abs"
    )
    actual_route_peak = _metric_value(
        metrics, "tracking", "actual_to_route_cte_m", "max_abs"
    )
    final_route_peak = _metric_value(metrics, "final_path", "max_abs")
    steering_delay = _metric_value(
        metrics, "steering_tracking", "first_order_fit", "delay_sec"
    )
    steering_tau = _metric_value(
        metrics, "steering_tracking", "first_order_fit", "time_constant_sec"
    )
    derivative_delay = _metric_value(
        metrics, "steering_tracking", "derivative_correlation_delay_sec"
    )

    if raw_final is not None and raw_final > 0.30:
        planning_score += 2
        reasons.append(
            f"VAD raw/command-horizon path requires large final-path intervention (p95 {raw_final:.2f} m)"
        )
    if raw_route is not None and final_route is not None and raw_route > final_route + 0.30:
        planning_score += 2
        reasons.append(
            f"raw VAD path is farther from the route than final path ({raw_route:.2f} vs {final_route:.2f} m p95)"
        )
    tracking_evidence = False
    if actual_final is not None and actual_final > 0.35:
        tracking_evidence = True
        control_score += 2
        reasons.append(f"vehicle-to-final-path tracking error is large (p95 {actual_final:.2f} m)")
    elif actual_final_peak is not None and actual_final_peak > 0.50:
        tracking_evidence = True
        control_score += 2
        if actual_route_peak is not None and final_route_peak is not None:
            reasons.append(
                "visible corner cut contains a control-tracking component: "
                f"actual-to-route peak {actual_route_peak:.2f} m, final-path route offset up to "
                f"{final_route_peak:.2f} m, actual-to-final peak {actual_final_peak:.2f} m"
            )
        else:
            reasons.append(
                f"vehicle-to-final-path peak tracking error is large ({actual_final_peak:.2f} m)"
            )
        if derivative_delay is not None:
            reasons.append(
                f"measured virtual steering-rate response lags the command by about {derivative_delay:.2f} s"
            )
    if (
        steering_delay is not None
        and mpc_input_delay_sec is not None
        and abs(steering_delay - mpc_input_delay_sec) > 0.10
    ):
        if tracking_evidence:
            control_score += 1
        reasons.append(
            f"fitted steering delay {steering_delay:.2f} s differs from MPC input_delay "
            f"{mpc_input_delay_sec:.2f} s"
        )
    if (
        steering_tau is not None
        and mpc_steer_tau_sec is not None
        and abs(steering_tau - mpc_steer_tau_sec) > 0.10
    ):
        if tracking_evidence:
            control_score += 1
        reasons.append(
            f"fitted steering time constant {steering_tau:.2f} s differs from MPC steer_tau "
            f"{mpc_steer_tau_sec:.2f} s"
        )

    screening_metrics = (
        raw_final,
        raw_route,
        final_route,
        actual_final,
        actual_final_peak,
    )
    if planning_score >= 2 and control_score >= 2:
        classification = "mixed_path_and_control"
    elif planning_score >= 2:
        classification = "path_dominant"
    elif control_score >= 2:
        classification = "control_dominant"
    elif reasons:
        classification = "weak_evidence"
    elif all(value is not None for value in screening_metrics):
        classification = "within_thresholds"
        reasons.append("screening path and tracking metrics remained within thresholds")
    else:
        classification = "insufficient_evidence"
        reasons.append("one or more required path or tracking metrics were unavailable")
    return {
        "classification": classification,
        "planning_score": planning_score,
        "control_score": control_score,
        "reasons": reasons,
        "threshold_note": "Thresholds are screening values; confirm with repeated A/B runs.",
    }


def _plot_path_dashboard(
    output: Path,
    route_xy: np.ndarray,
    route_progress: np.ndarray,
    route_command: np.ndarray,
    odometry: list[dict[str, Any]],
    raw: list[dict[str, Any]],
    final: list[dict[str, Any]],
    predicted: list[dict[str, Any]],
    tracking: list[dict[str, float]],
    raw_final_samples: list[dict[str, float]],
    predicted_final_samples: list[dict[str, float]],
    future_samples: dict[float, list[dict[str, float]]],
    verdict: dict[str, Any],
    t0: float,
) -> None:
    target_progress, representative_sec = _representative_time(
        route_progress, route_command, route_xy, tracking
    )
    has_turn = bool(np.any(np.isin(route_command, (0, 1))))
    raw_snapshot = _trajectory_at(raw, representative_sec)
    final_snapshot = _trajectory_at(final, representative_sec)
    predicted_snapshot = _trajectory_at(predicted, representative_sec)

    figure, axes = plt.subplots(2, 2, figsize=(16, 11), constrained_layout=True)
    figure.suptitle(
        f"{'Turn' if has_turn else 'Path/control'} diagnosis: "
        f"{verdict['classification'].replace('_', ' ')}",
        fontsize=17,
        fontweight="bold",
    )

    axis = axes[0, 0]
    axis.plot(route_xy[:, 0], route_xy[:, 1], color="#7f8c8d", linewidth=3, label="CARLA route")
    if odometry:
        actual_xy = np.asarray([[record["x"], record["y"]] for record in odometry])
        axis.plot(actual_xy[:, 0], actual_xy[:, 1], color="#111111", linewidth=2.2, label="Actual")
    for snapshot, label, color, style in (
        (raw_snapshot, "VAD selected raw", "#e67e22", "--"),
        (final_snapshot, "Controller input", "#2980b9", "-"),
        (predicted_snapshot, "MPC predicted", "#8e44ad", ":"),
    ):
        if snapshot is not None and len(snapshot["xy"]):
            plot_xy = _clip_trajectory_to_route_horizon(
                snapshot["xy"], route_xy, route_progress
            )
            axis.plot(
                plot_xy[:, 0], plot_xy[:, 1],
                color=color, linestyle=style, linewidth=2.2, label=label,
            )
    reference_label = "turn center" if has_turn else "representative point"
    axis.set_title(f"XY paths near {reference_label} (route s={target_progress:.1f} m)")
    axis.set_xlabel("map x [m]")
    axis.set_ylabel("map y [m]")
    axis.axis("equal")
    axis.grid(alpha=0.25)
    axis.legend(fontsize=9)

    axis = axes[0, 1]
    route_curvature = _polyline_curvature(route_xy)
    axis.plot(
        route_progress - target_progress, route_curvature,
        color="#7f8c8d", linewidth=2.5, label="Route",
    )
    for snapshot, label, color, style in (
        (raw_snapshot, "VAD raw", "#e67e22", "--"),
        (final_snapshot, "Final", "#2980b9", "-"),
        (predicted_snapshot, "MPC predicted", "#8e44ad", ":"),
    ):
        if snapshot is None or len(snapshot["xy"]) < 3:
            continue
        plot_xy = _clip_trajectory_to_route_horizon(
            snapshot["xy"], route_xy, route_progress
        )
        if len(plot_xy) < 3:
            continue
        _, progress, _, _ = _project_to_polyline(plot_xy, route_xy, route_progress)
        axis.plot(
            progress - target_progress,
            _polyline_curvature(plot_xy),
            color=color,
            linestyle=style,
            linewidth=2.0,
            label=label,
        )
    axis.axvline(0.0, color="#333333", linewidth=1, alpha=0.5)
    axis.set_xlim(-12.0, 18.0)
    axis.set_title("Path curvature at the representative update")
    axis.set_xlabel(f"route progress relative to {reference_label} [m]")
    axis.set_ylabel("signed curvature [1/m]")
    axis.grid(alpha=0.25)
    axis.legend(fontsize=9)

    axis = axes[1, 0]
    if tracking:
        time = np.asarray([sample["time_sec"] - t0 for sample in tracking])
        axis.plot(time, [sample["route_cte_m"] for sample in tracking], color="#7f8c8d", label="Actual to route")
        axis.plot(time, [sample["final_cte_m"] for sample in tracking], color="#2980b9", label="Actual to final")
        axis.axhline(0.0, color="#333333", linewidth=1)
    axis.set_title("Signed cross-track error")
    axis.set_xlabel("simulation time [s]")
    axis.set_ylabel("CTE [m]")
    axis.grid(alpha=0.25)
    axis.legend(fontsize=9)

    axis = axes[1, 1]
    if raw_final_samples:
        axis.plot(
            [sample["time_sec"] - t0 for sample in raw_final_samples],
            [sample["common_progress_p95_m"] for sample in raw_final_samples],
            color="#e67e22", label="Raw/final common-progress p95",
        )
    if predicted_final_samples:
        axis.plot(
            [sample["time_sec"] - t0 for sample in predicted_final_samples],
            [sample["common_progress_p95_m"] for sample in predicted_final_samples],
            color="#8e44ad", label="MPC/final common-progress p95",
        )
    for horizon, color in ((1.0, "#16a085"), (2.0, "#c0392b")):
        samples = future_samples.get(horizon, [])
        if samples:
            axis.scatter(
                [sample["time_sec"] - t0 for sample in samples],
                [sample["error_m"] for sample in samples],
                s=10, alpha=0.5, color=color, label=f"MPC future error @ {horizon:.0f}s",
            )
    axis.set_title("Planning intervention and MPC prediction error")
    axis.set_xlabel("simulation time [s]")
    axis.set_ylabel("distance [m]")
    axis.grid(alpha=0.25)
    axis.legend(fontsize=9)

    figure.savefig(output, dpi=140)
    plt.close(figure)


def _turn_window(
    tracking: list[dict[str, float]], route_progress: np.ndarray, route_command: np.ndarray
) -> tuple[float, float] | None:
    turn_indices = np.flatnonzero(np.isin(route_command, (0, 1)))
    if not len(turn_indices) or not tracking:
        return None
    lower = float(route_progress[turn_indices[0]])
    upper = float(route_progress[turn_indices[-1]])
    selected = [
        sample["time_sec"]
        for sample in tracking
        if lower - 2.0 <= sample["route_progress_m"] <= upper + 2.0
    ]
    return (min(selected), max(selected)) if selected else None


def _plot_steering_dashboard(
    output: Path,
    control: list[dict[str, Any]],
    steering: list[dict[str, Any]],
    diagnostic: list[dict[str, Any]],
    tracking: list[dict[str, float]],
    route_progress: np.ndarray,
    route_command: np.ndarray,
    dynamics_plot: dict[str, np.ndarray],
    dynamics_metrics: dict[str, Any],
    t0: float,
    plant_input_label: str,
    steering_report_mode: str,
    wheel_base_m: float,
    wheel_tread_m: float,
) -> None:
    figure, axes = plt.subplots(2, 2, figsize=(16, 11), constrained_layout=True)
    fit = dynamics_metrics.get("first_order_fit", {})
    figure.suptitle(
        "Steering tracking"
        + (
            f" | fit delay={fit.get('delay_sec', math.nan):.2f}s, "
            f"tau={fit.get('time_constant_sec', math.nan):.2f}s, K={fit.get('gain', math.nan):.2f}"
            if fit
            else ""
        ),
        fontsize=17,
        fontweight="bold",
    )
    turn_window = _turn_window(tracking, route_progress, route_command)

    axis = axes[0, 0]
    if control:
        axis.plot(
            _record_times(control) - t0,
            [record["angle"] for record in control],
            color="#2980b9", linewidth=1.8, label=plant_input_label,
        )
    if steering:
        report = np.asarray([record["angle"] for record in steering])
        report_label = "CARLA FL wheel report" if steering_report_mode == "legacy_fl" else "Virtual tire report"
        axis.plot(
            _record_times(steering) - t0,
            report,
            color="#e67e22",
            alpha=0.65,
            label=report_label,
        )
        if steering_report_mode == "legacy_fl":
            axis.plot(
                _record_times(steering) - t0,
                _fl_wheel_to_virtual(report, wheel_base_m, wheel_tread_m),
                color="#c0392b", linewidth=1.8, label="FL converted to virtual tire",
            )
    if dynamics_plot:
        axis.plot(
            dynamics_plot["time"] - t0,
            dynamics_plot["model_prediction"],
            color="#8e44ad", linestyle="--", label="Fitted first-order model",
        )
    axis.axhline(0.0, color="#333333", linewidth=1)
    axis.set_title("Steering angle")
    axis.set_xlabel("simulation time [s]")
    axis.set_ylabel("angle [rad]")
    axis.grid(alpha=0.25)
    axis.legend(fontsize=9)

    axis = axes[0, 1]
    valid_diag = [record for record in diagnostic if len(record["data"]) >= 18]
    if valid_diag:
        diag_time = _record_times(valid_diag) - t0
        diag = np.vstack([record["data"][:18] for record in valid_diag])
        axis.plot(diag_time, diag[:, 13], color="#7f8c8d", label="Required yaw rate from path")
        axis.plot(diag_time, diag[:, 11], color="#2980b9", label="Yaw rate from steer command")
        axis.plot(diag_time, diag[:, 12], color="#e67e22", label="Yaw rate from reported steer")
    if tracking:
        axis.plot(
            [sample["time_sec"] - t0 for sample in tracking],
            [sample["pose_yaw_rate_rps"] for sample in tracking],
            color="#111111", linewidth=1.8, label="Actual yaw rate from pose",
        )
    axis.axhline(0.0, color="#333333", linewidth=1)
    axis.set_title("Requested versus actual yaw rate")
    axis.set_xlabel("simulation time [s]")
    axis.set_ylabel("yaw rate [rad/s]")
    axis.grid(alpha=0.25)
    axis.legend(fontsize=9)

    axis = axes[1, 0]
    if valid_diag:
        axis.plot(diag_time, diag[:, 5], color="#2980b9", label="MPC lateral error [m]")
        axis.plot(
            diag_time, diag[:, 8], color="#c0392b", label="MPC yaw error [rad]"
        )
    if tracking:
        axis.plot(
            [sample["time_sec"] - t0 for sample in tracking],
            [sample["final_cte_m"] for sample in tracking],
            color="#111111", alpha=0.7, label="Recomputed actual-to-final CTE [m]",
        )
    axis.axhline(0.0, color="#333333", linewidth=1)
    axis.set_title("Controller path-tracking error")
    axis.set_xlabel("simulation time [s]")
    axis.grid(alpha=0.25)
    axis.legend(fontsize=9)

    axis = axes[1, 1]
    if dynamics_plot:
        axis.plot(
            dynamics_plot["time"] - t0,
            dynamics_plot["command_derivative"],
            color="#2980b9", alpha=0.8, label="Command rate",
        )
        axis.plot(
            dynamics_plot["time"] - t0,
            dynamics_plot["measured_derivative"],
            color="#c0392b", alpha=0.8, label="Virtual steer rate",
        )
    derivative_delay = dynamics_metrics.get("derivative_correlation_delay_sec")
    derivative_corr = dynamics_metrics.get("derivative_correlation")
    text = (
        f"Derivative delay: {derivative_delay:.3f} s\nCorrelation: {derivative_corr:.3f}"
        if derivative_delay is not None and derivative_corr is not None
        else "Delay metric unavailable"
    )
    axis.text(
        0.02, 0.98, text, transform=axis.transAxes, va="top",
        bbox={"facecolor": "white", "alpha": 0.85, "edgecolor": "#cccccc"},
    )
    axis.axhline(0.0, color="#333333", linewidth=1)
    axis.set_title("Steering rate used for lag estimation")
    axis.set_xlabel("simulation time [s]")
    axis.set_ylabel("rate [rad/s]")
    axis.grid(alpha=0.25)
    axis.legend(fontsize=9)

    if turn_window is not None:
        for axis in axes.flat:
            axis.axvspan(turn_window[0] - t0, turn_window[1] - t0, color="#f1c40f", alpha=0.10)

    figure.savefig(output, dpi=140)
    plt.close(figure)


def _clean_json(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): _clean_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_clean_json(item) for item in value]
    if isinstance(value, np.generic):
        value = value.item()
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def _missing_required_topics(records: dict[str, list[dict[str, Any]]]) -> list[str]:
    return [topic for topic in (RAW_TOPIC, FINAL_TOPIC, ODOMETRY_TOPIC) if not records.get(topic)]


def _first_topic_with_records(
    records: dict[str, list[dict[str, Any]]], topics: tuple[str, ...]
) -> tuple[str | None, list[dict[str, Any]]]:
    topic = next((candidate for candidate in topics if records.get(candidate)), None)
    return topic, records.get(topic, []) if topic else []


def _control_selection_metadata(control_topic: str | None) -> dict[str, Any]:
    if control_topic in RAW_CONTROL_TOPICS:
        return {
            "selected_control_stage": "trajectory_follower_raw",
            "quality_warnings": [],
        }
    if control_topic == GATED_CONTROL_TOPIC:
        return {
            "selected_control_stage": "vehicle_cmd_gate",
            "quality_warnings": [
                "Raw trajectory-follower control command is unavailable; controller-command "
                f"metrics use {GATED_CONTROL_TOPIC} after the vehicle command gate and may "
                "include operation-mode or MRM intervention."
            ],
        }
    return {"selected_control_stage": None, "quality_warnings": []}


def main() -> int:
    args = _parse_args()
    if args.wheel_base_m <= 0.0 or args.wheel_tread_m <= 0.0:
        raise ValueError("wheel dimensions must be positive")
    if args.mpc_input_delay_sec is not None and args.mpc_input_delay_sec < 0.0:
        raise ValueError("--mpc-input-delay-sec must be non-negative")
    if args.mpc_steer_tau_sec is not None and args.mpc_steer_tau_sec <= 0.0:
        raise ValueError("--mpc-steer-tau-sec must be positive")
    if (
        not math.isfinite(args.maneuver_lookahead_m)
        or args.maneuver_lookahead_m < 0.0
    ):
        raise ValueError("--maneuver-lookahead-m must be finite and non-negative")
    args.result_dir.mkdir(parents=True, exist_ok=True)
    records, topic_types = _read_bag(args.bag)
    route, route_xy, route_progress, route_command = _load_route(args.route_file)

    raw = records.get(RAW_TOPIC, [])
    final = records.get(FINAL_TOPIC, [])
    predicted_topic, predicted = _first_topic_with_records(records, PREDICTED_TOPICS)
    odometry = records.get(ODOMETRY_TOPIC, [])
    steering = records.get(STEERING_TOPIC, [])
    diagnostic = records.get(DIAGNOSTIC_TOPIC, [])
    actuation = records.get(ACTUATION_TOPIC, [])
    control_topic, control = _first_topic_with_records(records, CONTROL_TOPICS)
    control_selection_metadata = _control_selection_metadata(control_topic)
    plant_input_topic = ACTUATION_TOPIC if actuation else control_topic
    plant_input = actuation if actuation else control

    missing = _missing_required_topics(records)
    if missing:
        raise RuntimeError("bag has no messages for required topics: " + ", ".join(missing))

    turn_progress_ranges = _turn_progress_ranges(route_progress, route_command)
    command_switch_ranges = _turn_progress_ranges(
        route_progress,
        route_command,
        margin_m=0.0,
        maneuver_lookahead_m=args.maneuver_lookahead_m,
    )
    geometry_progress_ranges = _turn_progress_ranges(
        route_progress,
        route_command,
        maneuver_lookahead_m=args.maneuver_lookahead_m,
    )
    geometry_ranges = geometry_progress_ranges or None
    raw_metrics, _, _ = _trajectory_route_metrics(
        raw, route_xy, route_progress, geometry_ranges
    )
    final_metrics, _, _ = _trajectory_route_metrics(
        final, route_xy, route_progress, geometry_ranges
    )
    predicted_metrics, _, _ = _trajectory_route_metrics(
        predicted, route_xy, route_progress, geometry_ranges
    )
    raw_final_metrics, raw_final_samples = _pair_trajectory_geometry(
        raw, final, route_xy, route_progress, geometry_ranges
    )
    predicted_final_metrics, predicted_final_samples = _pair_trajectory_geometry(
        final, predicted, route_xy, route_progress, geometry_ranges
    )
    tracking_metrics, tracking_samples = _tracking_metrics(
        odometry, final, route_xy, route_progress
    )
    future_metrics, future_samples = _prediction_future_metrics(predicted, odometry)
    diagnostic_metrics = _diagnostic_metrics(diagnostic)

    steering_angle = np.asarray([record["angle"] for record in steering])
    if args.steering_report_mode == "legacy_fl":
        steering_for_fit = _fl_wheel_to_virtual(
            steering_angle, args.wheel_base_m, args.wheel_tread_m
        )
    else:
        steering_for_fit = steering_angle
    if plant_input and steering:
        steering_metrics, steering_plot = _fit_steering_dynamics(
            _record_times(plant_input),
            np.asarray([record["angle"] for record in plant_input]),
            _record_times(steering),
            steering_for_fit,
        )
    else:
        steering_metrics, steering_plot = {"available": False}, {}
    steering_metrics["report_angle_rad"] = _summary(steering_angle)
    steering_metrics["fit_angle_rad"] = _summary(steering_for_fit)
    steering_metrics["report_interpretation"] = {
        "mode": args.steering_report_mode,
        "wheel_base_m": args.wheel_base_m,
        "wheel_tread_m": args.wheel_tread_m,
        "note": (
            "legacy CARLA FL wheel report converted to a bicycle-model virtual tire angle"
            if args.steering_report_mode == "legacy_fl"
            else "report already satisfies the virtual bicycle-model tire-angle contract"
        ),
    }

    all_times = [_physics_ns(record) * 1.0e-9 for values in records.values() for record in values]
    t0 = min(all_times)
    peak_context = _peak_corner_context(
        tracking_samples,
        raw,
        final,
        control,
        actuation,
        steering,
        diagnostic,
        odometry,
        route_xy,
        route_progress,
        route_command,
        t0,
        args.steering_report_mode,
        args.wheel_base_m,
        args.wheel_tread_m,
    )
    metrics: dict[str, Any] = {
        "raw_path": raw_metrics,
        "final_path": final_metrics,
        "mpc_predicted_path": predicted_metrics,
        "raw_to_final_geometry": raw_final_metrics,
        "predicted_to_final_geometry": predicted_final_metrics,
        "tracking": tracking_metrics,
        "mpc_future_prediction": future_metrics,
        "mpc_diagnostic": diagnostic_metrics,
        "steering_tracking": steering_metrics,
        "peak_corner_cut": peak_context,
        "turn_geometry_window": {
            "basis": "route_manager_directional_command_switch",
            "maneuver_lookahead_m": args.maneuver_lookahead_m,
            "command_switch_progress_ranges_m": command_switch_ranges,
            "analysis_progress_ranges_m": geometry_progress_ranges,
            "analysis_margin_m": 2.0,
            "commanded_turn_progress_ranges_m": turn_progress_ranges,
        },
    }
    verdict = _classify(metrics, args.mpc_input_delay_sec, args.mpc_steer_tau_sec)
    diagnosis = {
        "schema_version": 2,
        "inputs": {
            "bag": str(args.bag.resolve()),
            "route_file": str(args.route_file.resolve()),
            "town": route.get("town"),
            "weather": route.get("weather"),
            "scenario": route.get("scenario"),
        },
        "alignment": {
            "primary_time": "ROS message/header stamp (simulation clock)",
            "policy": (
                "receipt timestamp selects the latest causally available message; header stamps "
                "measure physics intervals, prediction horizons, and plot time"
            ),
            "maximum_trajectory_age_sec": 1.0,
        },
        "runtime_configuration": {
            "mpc_input_delay_sec": args.mpc_input_delay_sec,
            "mpc_steer_tau_sec": args.mpc_steer_tau_sec,
            "maneuver_lookahead_m": args.maneuver_lookahead_m,
            "steering_report_mode": args.steering_report_mode,
            "wheel_base_m": args.wheel_base_m,
            "wheel_tread_m": args.wheel_tread_m,
        },
        "selected_control_topic": control_topic,
        **control_selection_metadata,
        "selected_predicted_topic": predicted_topic,
        "selected_plant_input_topic": plant_input_topic,
        "topic_types": {topic: topic_types[topic] for topic in records},
        "message_counts": {topic: len(values) for topic, values in records.items()},
        "metrics": metrics,
        "verdict": verdict,
    }
    diagnosis_path = args.result_dir / "diagnosis.json"
    with diagnosis_path.open("w", encoding="utf-8") as stream:
        json.dump(_clean_json(diagnosis), stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")

    _plot_path_dashboard(
        args.result_dir / "path_vs_control.png",
        route_xy,
        route_progress,
        route_command,
        odometry,
        raw,
        final,
        predicted,
        tracking_samples,
        raw_final_samples,
        predicted_final_samples,
        future_samples,
        verdict,
        t0,
    )
    _plot_steering_dashboard(
        args.result_dir / "steering_tracking.png",
        plant_input,
        steering,
        diagnostic,
        tracking_samples,
        route_progress,
        route_command,
        steering_plot,
        steering_metrics,
        t0,
        plant_input_topic or "Steering command unavailable",
        args.steering_report_mode,
        args.wheel_base_m,
        args.wheel_tread_m,
    )

    print(f"classification: {verdict['classification']}")
    print(f"diagnosis: {diagnosis_path}")
    print(f"path plot: {args.result_dir / 'path_vs_control.png'}")
    print(f"steering plot: {args.result_dir / 'steering_tracking.png'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
