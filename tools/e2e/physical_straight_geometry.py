#!/usr/bin/env python3
"""Dependency-free physical-straight route admission geometry.

The catalog generator and the Autoware town-matrix selector both call this
module so generation-time filtering and selection-time verification cannot
silently diverge.
"""

from __future__ import annotations

import math
from typing import Any, Mapping, Sequence


SPEED_30KPH_STRAIGHT_GEOMETRY_CONTRACT = {
    "maximum_arc_to_direct_ratio": 1.005,
    "maximum_chord_deviation_m": 2.5,
    "maximum_endpoint_tangent_to_chord_deg": 10.0,
    "maximum_net_heading_change_deg": 10.0,
    "maximum_cumulative_heading_change_deg": 25.0,
    "maximum_p95_abs_curvature_per_m": 0.01,
    # a_y = v^2 * curvature at 8.333333 m/s and the 1.8 m/s^2 gate.
    "maximum_abs_curvature_per_m": 0.02592,
}


class PhysicalStraightGeometryError(ValueError):
    """Raised when serialized route data cannot be measured safely."""


def _normalize_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def _linear_percentile(values: Sequence[float], percentile: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return 0.0
    position = (len(ordered) - 1) * percentile / 100.0
    lower = int(math.floor(position))
    upper = min(lower + 1, len(ordered) - 1)
    fraction = position - lower
    return ordered[lower] + fraction * (ordered[upper] - ordered[lower])


def analyze_serialized_physical_straight(
    payload: Mapping[str, Any], required: Mapping[str, Any]
) -> dict[str, Any]:
    """Measure one serialized ROS-frame route against a physical-straight gate.

    Yaw values are radians and ``distance_m`` is the serialized cumulative arc
    distance, including the exact requested terminal goal when it is appended
    by ``prepare_carla_route.serialize_route``.
    """
    points = payload.get("route")
    if not isinstance(points, list) or len(points) < 2:
        raise PhysicalStraightGeometryError(
            "30 kph physical-straight route needs at least two points"
        )

    poses: list[tuple[float, float, float, float]] = []
    for index, point in enumerate(points):
        if not isinstance(point, dict):
            raise PhysicalStraightGeometryError(
                f"physical-straight route point {index} is not an object"
            )
        values = tuple(point.get(name) for name in ("x", "y", "yaw", "distance_m"))
        if not all(
            isinstance(value, (int, float))
            and not isinstance(value, bool)
            and math.isfinite(float(value))
            for value in values
        ):
            raise PhysicalStraightGeometryError(
                f"physical-straight route point {index} lacks finite "
                "x/y/yaw/distance_m"
            )
        pose = tuple(float(value) for value in values)
        if poses and pose[3] < poses[-1][3]:
            raise PhysicalStraightGeometryError(
                "physical-straight route distance_m is not monotonic"
            )
        poses.append(pose)  # type: ignore[arg-type]

    route_length = poses[-1][3]
    x0, y0 = poses[0][0], poses[0][1]
    dx = poses[-1][0] - x0
    dy = poses[-1][1] - y0
    direct_distance = math.hypot(dx, dy)
    chord_yaw = math.atan2(dy, dx)
    arc_to_direct = (
        route_length / direct_distance if direct_distance > 1.0e-6 else math.inf
    )
    chord_deviations = (
        [
            abs(dx * (y - y0) - dy * (x - x0)) / direct_distance
            for x, y, _, _ in poses
        ]
        if direct_distance > 1.0e-6
        else [math.inf]
    )

    unwrapped_yaws = [poses[0][2]]
    for _, _, yaw, _ in poses[1:]:
        unwrapped_yaws.append(
            unwrapped_yaws[-1] + _normalize_angle(yaw - unwrapped_yaws[-1])
        )
    net_heading_deg = abs(
        math.degrees(unwrapped_yaws[-1] - unwrapped_yaws[0])
    )
    cumulative_heading_rad = sum(
        abs(end - start)
        for start, end in zip(unwrapped_yaws, unwrapped_yaws[1:])
    )
    cumulative_heading_deg = math.degrees(cumulative_heading_rad)
    curvatures = []
    for index in range(1, len(poses)):
        segment_length = poses[index][3] - poses[index - 1][3]
        if segment_length <= 1.0e-6:
            continue
        curvatures.append(
            abs(unwrapped_yaws[index] - unwrapped_yaws[index - 1])
            / segment_length
        )
    p95_curvature = _linear_percentile(curvatures, 95.0)
    maximum_curvature = max(curvatures, default=0.0)
    maximum_chord_deviation = max(chord_deviations)
    maximum_endpoint_tangent_to_chord_deg = max(
        abs(math.degrees(_normalize_angle(poses[0][2] - chord_yaw))),
        abs(math.degrees(_normalize_angle(poses[-1][2] - chord_yaw))),
    )
    metrics = {
        "route_length_m": route_length,
        "direct_distance_m": direct_distance,
        "arc_to_direct_ratio": arc_to_direct,
        "maximum_chord_deviation_m": maximum_chord_deviation,
        "maximum_endpoint_tangent_to_chord_deg": (
            maximum_endpoint_tangent_to_chord_deg
        ),
        "absolute_net_heading_change_deg": net_heading_deg,
        "cumulative_absolute_heading_change_deg": cumulative_heading_deg,
        "p95_absolute_curvature_per_m": p95_curvature,
        "maximum_absolute_curvature_per_m": maximum_curvature,
    }
    checks = (
        (
            arc_to_direct > float(required["maximum_arc_to_direct_ratio"]),
            "arc/direct ratio exceeds the maximum",
        ),
        (
            maximum_chord_deviation
            > float(required["maximum_chord_deviation_m"]),
            "chord deviation exceeds the maximum",
        ),
        (
            maximum_endpoint_tangent_to_chord_deg
            > float(required["maximum_endpoint_tangent_to_chord_deg"]),
            "endpoint tangent/chord angle exceeds the maximum",
        ),
        (
            net_heading_deg
            > float(required["maximum_net_heading_change_deg"]),
            "net heading change exceeds the maximum",
        ),
        (
            cumulative_heading_deg
            > float(required["maximum_cumulative_heading_change_deg"]),
            "cumulative heading change exceeds the maximum",
        ),
        (
            p95_curvature
            > float(required["maximum_p95_abs_curvature_per_m"]),
            "p95 absolute curvature exceeds the maximum",
        ),
        (
            maximum_curvature
            > float(required["maximum_abs_curvature_per_m"]),
            "peak absolute curvature exceeds the lateral-acceleration bound",
        ),
    )
    reasons = [message for failed, message in checks if failed]
    return {
        "status": "FAIL" if reasons else "PASS",
        **metrics,
        "limits": dict(required),
        "failure_reasons": reasons,
    }
