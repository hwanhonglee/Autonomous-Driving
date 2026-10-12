#!/usr/bin/env python3
"""Deterministic custom-map turn geometry over the serialized ROS route."""

from __future__ import annotations

import math
from typing import Any, Mapping, Sequence


TURN_MANEUVER_OPTIONS = frozenset(
    {
        "LEFT",
        "RIGHT",
        "STRAIGHT",
        "CHANGELANELEFT",
        "CHANGELANERIGHT",
    }
)


class SerializedCustomTurnGeometryError(ValueError):
    """Raised when serialized custom-turn inputs cannot be measured."""


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


def _first_progress_crossing(
    samples: Sequence[tuple[float, float]], threshold: float
) -> float | None:
    for (start_distance, start_value), (end_distance, end_value) in zip(
        samples, samples[1:]
    ):
        if start_value >= threshold:
            return start_distance
        if end_value < threshold:
            continue
        if end_value <= start_value:
            return end_distance
        ratio = (threshold - start_value) / (end_value - start_value)
        return start_distance + ratio * (end_distance - start_distance)
    if samples and samples[-1][1] >= threshold:
        return samples[-1][0]
    return None


def analyze_serialized_custom_turn(
    route_points: Sequence[Mapping[str, Any]],
    scenario: str,
    required: Mapping[str, Any],
) -> dict[str, Any]:
    """Measure one isolated turn using only the final JSON route fields.

    ``yaw`` is the serialized ROS yaw in radians and ``distance_m`` is the
    serialized 3D route-progress axis.  Both the catalog generator and matrix
    selector call this function so persisted provenance can be compared by
    exact structural equality without floating-point tolerances.
    """
    if not isinstance(route_points, Sequence) or isinstance(
        route_points, (str, bytes)
    ):
        raise SerializedCustomTurnGeometryError(
            "custom-map turn geometry route is not an array"
        )
    if len(route_points) < 2:
        raise SerializedCustomTurnGeometryError(
            "custom-map turn geometry needs at least two route points"
        )
    if scenario not in ("left", "right"):
        raise SerializedCustomTurnGeometryError(
            "custom-map turn geometry requires left or right scenario"
        )

    target = scenario.upper()
    options: list[str] = []
    distances: list[float] = []
    yaws: list[float] = []
    for index, point in enumerate(route_points):
        if not isinstance(point, Mapping):
            raise SerializedCustomTurnGeometryError(
                f"custom-map route point {index} is not an object"
            )
        option = point.get("road_option")
        distance = point.get("distance_m")
        yaw = point.get("yaw")
        if not isinstance(option, str):
            raise SerializedCustomTurnGeometryError(
                f"custom-map route point {index} has no road option"
            )
        if not isinstance(distance, (int, float)) or not math.isfinite(
            float(distance)
        ):
            raise SerializedCustomTurnGeometryError(
                f"custom-map route point {index} has invalid distance"
            )
        if not isinstance(yaw, (int, float)) or not math.isfinite(float(yaw)):
            raise SerializedCustomTurnGeometryError(
                f"custom-map route point {index} has invalid yaw"
            )
        if distances and float(distance) < distances[-1]:
            raise SerializedCustomTurnGeometryError(
                "custom-map route distance_m is not monotonic"
            )
        options.append(option)
        distances.append(float(distance))
        yaws.append(float(yaw))

    indices = [index for index, option in enumerate(options) if option == target]
    blocks: list[list[int]] = []
    for index in indices:
        if not blocks or index != blocks[-1][-1] + 1:
            blocks.append([index])
        else:
            blocks[-1].append(index)
    additional = sorted((set(options) & TURN_MANEUVER_OPTIONS) - {target})
    reasons = []
    if len(blocks) != 1:
        reasons.append(
            f"expected exactly one contiguous {target} block, found {len(blocks)}"
        )
    if additional:
        reasons.append("additional maneuver commands present: " + ", ".join(additional))

    def block_geometry(block: Sequence[int]) -> dict[str, Any]:
        start_index = block[0]
        end_index = block[-1]
        support_start = max(0, start_index - 1)
        support_end = min(len(route_points) - 1, end_index + 1)
        block_start = (
            0.5 * (distances[start_index - 1] + distances[start_index])
            if start_index
            else distances[start_index]
        )
        block_end = (
            0.5 * (distances[end_index] + distances[end_index + 1])
            if end_index + 1 < len(route_points)
            else distances[end_index]
        )
        unwrapped = [yaws[support_start]]
        for yaw in yaws[support_start + 1 : support_end + 1]:
            unwrapped.append(unwrapped[-1] + _normalize_angle(yaw - unwrapped[-1]))
        net_heading_rad = unwrapped[-1] - unwrapped[0]
        net_heading_deg = abs(math.degrees(net_heading_rad))
        cumulative_heading_rad = sum(
            abs(unwrapped[index] - unwrapped[index - 1])
            for index in range(1, len(unwrapped))
        )
        cumulative_heading_deg = math.degrees(cumulative_heading_rad)
        direction = 1.0 if net_heading_rad >= 0.0 else -1.0
        progress = [
            (
                distances[support_start + index],
                max(0.0, math.degrees(direction * (yaw - unwrapped[0]))),
            )
            for index, yaw in enumerate(unwrapped)
        ]
        curvatures = []
        for index in range(support_start + 1, support_end + 1):
            segment_length = distances[index] - distances[index - 1]
            if segment_length <= 1.0e-6:
                continue
            curvatures.append(
                abs(_normalize_angle(yaws[index] - yaws[index - 1]))
                / segment_length
            )
        arc_length = block_end - block_start
        return {
            "start_index": start_index,
            "end_index": end_index,
            "command_arc_length_m": arc_length,
            "absolute_net_heading_change_deg": net_heading_deg,
            "cumulative_absolute_heading_change_deg": cumulative_heading_deg,
            "heading_excess_deg": max(
                0.0, cumulative_heading_deg - net_heading_deg
            ),
            "mean_absolute_curvature_per_m": (
                cumulative_heading_rad / arc_length
                if arc_length > 0.0
                else math.inf
            ),
            "p95_absolute_curvature_per_m": _linear_percentile(
                curvatures, 95.0
            ),
            "block_start_distance_m": block_start,
            "block_end_distance_m": block_end,
            "_heading_progress": progress,
        }

    block_metrics = [block_geometry(block) for block in blocks]
    selected = block_metrics[0] if len(block_metrics) == 1 else None
    if selected is not None:
        margin = float(required["alignment_heading_margin_deg"])
        net_heading = float(selected["absolute_net_heading_change_deg"])
        lead_crossing = _first_progress_crossing(
            selected["_heading_progress"], margin
        )
        tail_crossing = _first_progress_crossing(
            selected["_heading_progress"], max(0.0, net_heading - margin)
        )
        command_lead = (
            math.inf
            if lead_crossing is None
            else max(
                0.0, lead_crossing - float(selected["block_start_distance_m"])
            )
        )
        command_tail = (
            math.inf
            if tail_crossing is None
            else max(0.0, float(selected["block_end_distance_m"]) - tail_crossing)
        )
        selected["command_lead_distance_m"] = command_lead
        selected["command_tail_distance_m"] = command_tail

        checks = (
            (
                float(selected["command_arc_length_m"])
                < float(required["minimum_arc_length_m"]),
                "turn command arc length is below the minimum",
            ),
            (
                float(selected["command_arc_length_m"])
                > float(required["maximum_arc_length_m"]),
                "turn command arc length exceeds the maximum",
            ),
            (
                net_heading < float(required["minimum_heading_change_deg"]),
                "turn heading change is below the minimum",
            ),
            (
                net_heading > float(required["maximum_heading_change_deg"]),
                "turn heading change exceeds the maximum",
            ),
            (
                float(selected["heading_excess_deg"])
                > float(required["maximum_heading_excess_deg"]),
                "turn heading excess exceeds the maximum",
            ),
            (
                command_lead > float(required["maximum_command_lead_m"]),
                "turn command lead distance exceeds the maximum",
            ),
            (
                command_tail > float(required["maximum_command_tail_m"]),
                "turn command tail distance exceeds the maximum",
            ),
            (
                float(selected["p95_absolute_curvature_per_m"])
                > float(required["maximum_p95_abs_curvature_per_m"]),
                "turn p95 absolute curvature exceeds the maximum",
            ),
        )
        reasons.extend(message for failed, message in checks if failed)

    for metrics in block_metrics:
        metrics.pop("_heading_progress", None)
    return {
        "status": "FAIL" if reasons else "PASS",
        "scenario": scenario,
        "directional_command": target,
        "directional_block_count": len(blocks),
        "additional_maneuver_commands": additional,
        "selected_block": selected,
        "directional_blocks": block_metrics,
        "limits": dict(required),
        "failure_reasons": reasons,
    }
