#!/usr/bin/env python3
"""Dependency-free physical-turn route admission geometry.

The catalog generator and the Autoware town-matrix selector use the same
serialized-route analysis so a route cannot pass generation with one geometry
and then be selected under another.  The input is the final JSON payload from
``prepare_carla_route.serialize_route``; consequently an appended exact goal is
part of the measurement, including its position, yaw, and road option.
"""

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

# This is a simulation-screening route-shape contract, not a real-vehicle
# operating envelope.  The runtime maneuver lookahead is 4 m, while the
# independently conservative 25 m route-lead minimum remains pinned.  It keeps
# the initial command LANEFOLLOW instead of exposing a turn command while the
# vehicle is still at its spawn pose.
SPEED_30KPH_TURN_GEOMETRY_CONTRACT: dict[str, Any] = {
    "profile_id": "carla_vad_30kph_v2",
    "maneuver_lookahead_m": 4.0,
    "lead_buffer_m": 5.0,
    "minimum_route_lead_m": 25.0,
    "minimum_route_tail_m": 10.0,
    "minimum_turn_arc_length_m": 10.0,
    "maximum_turn_arc_length_m": 30.0,
    "minimum_turn_heading_change_deg": 60.0,
    "maximum_turn_heading_change_deg": 120.0,
    "maximum_turn_heading_excess_deg": 20.0,
    "turn_alignment_heading_margin_deg": 10.0,
    "maximum_turn_command_lead_m": 8.0,
    "maximum_turn_command_tail_m": 8.0,
    "maximum_p95_abs_curvature_per_m": 0.20,
    # Permit one discretization sample above the p95 limit, but reject sharp
    # terminal-yaw glue and other local discontinuities deterministically.
    "maximum_abs_curvature_per_m": 0.30,
    "initial_approach_distance_m": 15.0,
    "maximum_initial_lateral_deviation_m": 1.5,
    "maximum_initial_heading_change_deg": 30.0,
    # ``distance_m`` is the serialized 3D route-progress axis, so dz/ds is
    # sin(pitch), not tan(pitch).  Admission stays pinned to the configured
    # slope-compensation pitch envelope.  The PID adds its gravity term before
    # the downstream vehicle-command gate applies the delivered acceleration
    # cap, so pre-gate command capability and delivered uphill reserve are
    # recorded separately below.
    "longitudinal_grade_window_m": 5.0,
    "controller_maximum_output_mps2": 1.5,
    "slope_compensation_max_pitch_rad": 0.1,
    "standard_gravity_mps2": 9.81,
    "maximum_absolute_grade_ratio": 0.09983341664682815,
    "maximum_compensated_gravity_mps2": 0.9793658173053843,
    "maximum_pre_gate_total_acceleration_mps2": 2.4793658173053843,
    "downstream_vehicle_cmd_gate_acceleration_cap_mps2": 1.5,
    "ideal_net_uphill_acceleration_reserve_mps2": 0.5206341826946157,
}

SPEED_30KPH_TURN_CONTRACT_PROVENANCE = {
    "scope": "CARLA-only 30 kph simulation route-shape screening",
    "maneuver_lookahead_source": (
        "autoware_e2e_vad_launch/scripts/vad_route_logic.py::"
        "Route.command_at"
    ),
    "lead_distance_basis": (
        "runtime route_manager maneuver_lookahead_m 4.0 plus deterministic "
        "5.0 m LANEFOLLOW buffer, with an independently conservative "
        "minimum_route_lead_m of 25.0"
    ),
    "tail_distance_basis": (
        "10.0 m post-command lane-follow recovery before the exact route goal"
    ),
    "turn_shape_basis": (
        "existing isolated-turn 10-30 m, 60-120 deg, <=20 deg excess, "
        "and <=0.20/m p95 curvature contract"
    ),
    "peak_curvature_basis": (
        "1.5 times the 0.20/m p95 allowance to retain one discretization "
        "sample while rejecting serialized endpoint discontinuities"
    ),
    "initial_approach_basis": (
        "existing 15 m, 1.5 m lateral, 30 deg heading custom-map entrance gate "
        "applied identically to packaged Town routes"
    ),
    "longitudinal_grade_measurement_source": (
        "exact serialized route[].z over route[].distance_m; piecewise-linear "
        "elevation at every exact 5.0 m sliding-window breakpoint"
    ),
    "longitudinal_grade_capability_basis": {
        "controller_maximum_output_mps2": 1.5,
        "slope_compensation_max_pitch_rad": 0.1,
        "standard_gravity_mps2": 9.81,
        "maximum_compensated_gravity_mps2": 0.9793658173053843,
        "maximum_pre_gate_total_acceleration_mps2": 2.4793658173053843,
        "downstream_vehicle_cmd_gate_acceleration_cap_mps2": 1.5,
        "ideal_net_uphill_acceleration_reserve_mps2": 0.5206341826946157,
        "admission_maximum_absolute_grade_ratio": 0.09983341664682815,
        "conservative_policy": (
            "admit only the configured slope-compensation envelope; distinguish "
            "the PID-plus-slope pre-gate command from the downstream gate cap "
            "and its ideal net uphill reserve"
        ),
    },
    "selection_policy": (
        "geometry PASS is mandatory before the declared outcome-independent "
        "curvature/deceleration ranking"
    ),
    "real_vehicle_ready": False,
}


class PhysicalTurnGeometryError(ValueError):
    """Raised when a turn contract or serialized route cannot be measured."""

    def __init__(
        self,
        message: str,
        *,
        error_scope: str = "candidate",
        error_code: str = "invalid_candidate_geometry",
    ) -> None:
        super().__init__(message)
        self.error_scope = error_scope
        self.error_code = error_code

    @property
    def fatal(self) -> bool:
        """Whether catalog generation must stop instead of trying another pair."""
        return self.error_scope in {"contract", "schema", "nonfinite"}

    def evidence(self) -> dict[str, Any]:
        """Return stable machine-readable evidence for catalog rejection logs."""
        return {
            "error_type": type(self).__name__,
            "error_scope": self.error_scope,
            "error_code": self.error_code,
            "fatal": self.fatal,
            "message": str(self),
        }


def _contract_error(message: str) -> PhysicalTurnGeometryError:
    return PhysicalTurnGeometryError(
        message,
        error_scope="contract",
        error_code="invalid_physical_turn_contract",
    )


def _nonfinite_route_error(message: str) -> PhysicalTurnGeometryError:
    return PhysicalTurnGeometryError(
        message,
        error_scope="nonfinite",
        error_code="nonfinite_serialized_route",
    )


def _schema_route_error(message: str) -> PhysicalTurnGeometryError:
    return PhysicalTurnGeometryError(
        message,
        error_scope="schema",
        error_code="invalid_serialized_route_schema",
    )


def _finite_number(value: Any) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


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


def _validated_contract(required: Mapping[str, Any]) -> dict[str, float]:
    numeric_names = (
        "maneuver_lookahead_m",
        "lead_buffer_m",
        "minimum_route_lead_m",
        "minimum_route_tail_m",
        "minimum_turn_arc_length_m",
        "maximum_turn_arc_length_m",
        "minimum_turn_heading_change_deg",
        "maximum_turn_heading_change_deg",
        "maximum_turn_heading_excess_deg",
        "turn_alignment_heading_margin_deg",
        "maximum_turn_command_lead_m",
        "maximum_turn_command_tail_m",
        "maximum_p95_abs_curvature_per_m",
        "maximum_abs_curvature_per_m",
        "initial_approach_distance_m",
        "maximum_initial_lateral_deviation_m",
        "maximum_initial_heading_change_deg",
        "longitudinal_grade_window_m",
        "controller_maximum_output_mps2",
        "slope_compensation_max_pitch_rad",
        "standard_gravity_mps2",
        "maximum_absolute_grade_ratio",
        "maximum_compensated_gravity_mps2",
        "maximum_pre_gate_total_acceleration_mps2",
        "downstream_vehicle_cmd_gate_acceleration_cap_mps2",
        "ideal_net_uphill_acceleration_reserve_mps2",
    )
    missing = [name for name in numeric_names if name not in required]
    if missing:
        raise _contract_error(
            "physical-turn contract is missing " + ", ".join(missing)
        )
    numeric: dict[str, float] = {}
    for name in numeric_names:
        value = required[name]
        if not _finite_number(value) or float(value) <= 0.0:
            raise _contract_error(
                f"physical-turn contract {name} must be positive and finite"
            )
        numeric[name] = float(value)
    if numeric["minimum_route_lead_m"] + 1.0e-9 < (
        numeric["maneuver_lookahead_m"] + numeric["lead_buffer_m"]
    ):
        raise _contract_error(
            "minimum route lead must cover maneuver lookahead plus lead buffer"
        )
    if numeric["minimum_turn_arc_length_m"] >= numeric[
        "maximum_turn_arc_length_m"
    ]:
        raise _contract_error(
            "minimum turn arc length must be below its maximum"
        )
    if numeric["minimum_turn_heading_change_deg"] >= numeric[
        "maximum_turn_heading_change_deg"
    ]:
        raise _contract_error(
            "minimum turn heading change must be below its maximum"
        )
    if numeric["maximum_turn_heading_change_deg"] > 180.0:
        raise _contract_error(
            "maximum turn heading change cannot exceed 180 degrees"
        )
    if numeric["turn_alignment_heading_margin_deg"] >= numeric[
        "minimum_turn_heading_change_deg"
    ]:
        raise _contract_error(
            "turn alignment margin must be below minimum heading change"
        )
    if numeric["maximum_p95_abs_curvature_per_m"] > numeric[
        "maximum_abs_curvature_per_m"
    ]:
        raise _contract_error(
            "p95 curvature limit cannot exceed peak curvature limit"
        )
    pitch = numeric["slope_compensation_max_pitch_rad"]
    if pitch >= math.pi / 2.0:
        raise _contract_error(
            "slope compensation maximum pitch must be below pi/2"
        )
    expected_grade = math.sin(pitch)
    if not math.isclose(
        numeric["maximum_absolute_grade_ratio"],
        expected_grade,
        rel_tol=0.0,
        abs_tol=1.0e-12,
    ):
        raise _contract_error(
            "maximum absolute grade must equal sin(slope compensation max pitch)"
        )
    expected_compensation = numeric["standard_gravity_mps2"] * expected_grade
    if not math.isclose(
        numeric["maximum_compensated_gravity_mps2"],
        expected_compensation,
        rel_tol=0.0,
        abs_tol=1.0e-12,
    ):
        raise _contract_error(
            "maximum compensated gravity does not match the grade capability"
        )
    expected_pre_gate = (
        numeric["controller_maximum_output_mps2"] + expected_compensation
    )
    if not math.isclose(
        numeric["maximum_pre_gate_total_acceleration_mps2"],
        expected_pre_gate,
        rel_tol=0.0,
        abs_tol=1.0e-12,
    ):
        raise _contract_error(
            "pre-gate total acceleration does not match PID plus slope compensation"
        )
    delivered_cap = numeric[
        "downstream_vehicle_cmd_gate_acceleration_cap_mps2"
    ]
    expected_reserve = delivered_cap - expected_compensation
    if expected_reserve <= 0.0 or not math.isclose(
        numeric["ideal_net_uphill_acceleration_reserve_mps2"],
        expected_reserve,
        rel_tol=0.0,
        abs_tol=1.0e-12,
    ):
        raise _contract_error(
            "ideal net uphill reserve does not match gate cap minus gravity"
        )
    return numeric


def _serialized_points(payload: Mapping[str, Any]) -> list[dict[str, Any]]:
    raw_points = payload.get("route")
    if not isinstance(raw_points, list) or len(raw_points) < 2:
        raise _schema_route_error(
            "30 kph physical-turn route needs at least two points"
        )
    points: list[dict[str, Any]] = []
    previous_point: dict[str, Any] | None = None
    for index, point in enumerate(raw_points):
        if not isinstance(point, dict):
            raise _schema_route_error(
                f"physical-turn route point {index} is not an object"
            )
        values = tuple(
            point.get(name) for name in ("x", "y", "z", "yaw", "distance_m")
        )
        if not all(_finite_number(value) for value in values):
            raise _nonfinite_route_error(
                f"physical-turn route point {index} lacks finite "
                "x/y/z/yaw/distance_m"
            )
        road_option = point.get("road_option")
        if not isinstance(road_option, str) or not road_option:
            raise _schema_route_error(
                f"physical-turn route point {index} lacks road_option"
            )
        distance = float(point["distance_m"])
        normalized = {
            "x": float(point["x"]),
            "y": float(point["y"]),
            "z": float(point["z"]),
            "yaw": float(point["yaw"]),
            "distance_m": distance,
            "road_option": road_option.upper(),
        }
        if previous_point is not None and distance < previous_point["distance_m"]:
            raise PhysicalTurnGeometryError(
                "physical-turn route distance_m is not monotonic",
                error_code="non_monotonic_route_progress",
            )
        if previous_point is not None:
            serialized_increment = distance - previous_point["distance_m"]
            spatial_increment = math.sqrt(
                (normalized["x"] - previous_point["x"]) ** 2
                + (normalized["y"] - previous_point["y"]) ** 2
                + (normalized["z"] - previous_point["z"]) ** 2
            )
            if not math.isclose(
                serialized_increment,
                spatial_increment,
                rel_tol=0.0,
                abs_tol=1.0e-6,
            ):
                raise PhysicalTurnGeometryError(
                    "physical-turn route distance_m is not the 3D point distance "
                    f"between points {index - 1} and {index}",
                    error_code="inconsistent_3d_route_progress",
                )
        previous_point = normalized
        points.append(normalized)
    declared_length = payload.get("route_length_m")
    measured_length = points[-1]["distance_m"]
    if not _finite_number(declared_length):
        raise _nonfinite_route_error(
            "physical-turn route_length_m must be finite"
        )
    if not math.isclose(
        float(declared_length), measured_length, rel_tol=0.0, abs_tol=1.0e-6
    ):
        raise _schema_route_error(
            "physical-turn route_length_m does not match its serialized endpoint"
        )
    return points


def _interpolated_elevation(
    samples: Sequence[tuple[float, float]], distance_m: float
) -> float:
    if distance_m <= samples[0][0]:
        return samples[0][1]
    for (start_distance, start_z), (end_distance, end_z) in zip(
        samples, samples[1:]
    ):
        if distance_m > end_distance:
            continue
        ratio = (distance_m - start_distance) / (end_distance - start_distance)
        return start_z + ratio * (end_z - start_z)
    return samples[-1][1]


def _longitudinal_grade_geometry(
    points: Sequence[Mapping[str, Any]], limits: Mapping[str, float]
) -> dict[str, Any]:
    """Measure all extrema of the exact piecewise-linear 5 m Z window."""
    samples: list[tuple[float, float]] = []
    for index, point in enumerate(points):
        sample = (float(point["distance_m"]), float(point["z"]))
        if samples and sample[0] == samples[-1][0]:
            if not math.isclose(
                sample[1], samples[-1][1], rel_tol=0.0, abs_tol=1.0e-9
            ):
                raise PhysicalTurnGeometryError(
                    "physical-turn route changes elevation at zero distance "
                    f"between points {index - 1} and {index}",
                    error_code="zero_distance_elevation_change",
                )
            continue
        samples.append(sample)

    window = limits["longitudinal_grade_window_m"]
    first_distance = samples[0][0]
    last_distance = samples[-1][0]
    available = last_distance - first_distance
    capability = {
        "controller_maximum_output_mps2": limits[
            "controller_maximum_output_mps2"
        ],
        "slope_compensation_max_pitch_rad": limits[
            "slope_compensation_max_pitch_rad"
        ],
        "standard_gravity_mps2": limits["standard_gravity_mps2"],
        "maximum_compensated_gravity_mps2": limits[
            "maximum_compensated_gravity_mps2"
        ],
        "maximum_pre_gate_total_acceleration_mps2": limits[
            "maximum_pre_gate_total_acceleration_mps2"
        ],
        "downstream_vehicle_cmd_gate_acceleration_cap_mps2": limits[
            "downstream_vehicle_cmd_gate_acceleration_cap_mps2"
        ],
        "ideal_net_uphill_acceleration_reserve_mps2": limits[
            "ideal_net_uphill_acceleration_reserve_mps2"
        ],
        "admission_policy": (
            "slope_envelope_with_pre_gate_and_delivered_cap_distinguished"
        ),
        "real_vehicle_ready": False,
    }
    base = {
        "measurement_source": (
            "exact_serialized_route_z_over_distance_m_piecewise_linear_window"
        ),
        "window_length_m": window,
        "available_route_distance_m": available,
        "maximum_allowed_absolute_grade_ratio": limits[
            "maximum_absolute_grade_ratio"
        ],
        "maximum_allowed_absolute_grade_percent": 100.0
        * limits["maximum_absolute_grade_ratio"],
        "capability": capability,
        "real_vehicle_ready": False,
    }
    if available + 1.0e-9 < window:
        return {
            **base,
            "status": "FAIL",
            "maximum_absolute_grade_ratio": None,
            "maximum_absolute_grade_percent": None,
            "maximum_uphill_grade_ratio": None,
            "maximum_downhill_grade_ratio": None,
            "worst_window": None,
            "failure_reasons": [
                "route does not cover the longitudinal grade window"
            ],
        }

    last_start = last_distance - window
    candidate_starts = {first_distance, last_start}
    for distance, _ in samples:
        if first_distance <= distance <= last_start:
            candidate_starts.add(distance)
        shifted = distance - window
        if first_distance <= shifted <= last_start:
            candidate_starts.add(shifted)

    windows: list[dict[str, float]] = []
    for start in sorted(candidate_starts):
        end = start + window
        start_z = _interpolated_elevation(samples, start)
        end_z = _interpolated_elevation(samples, end)
        rise = end_z - start_z
        signed_grade = rise / window
        windows.append(
            {
                "start_distance_m": start,
                "end_distance_m": end,
                "start_z_m": start_z,
                "end_z_m": end_z,
                "rise_m": rise,
                "signed_grade_ratio": signed_grade,
                "absolute_grade_ratio": abs(signed_grade),
            }
        )
    worst = max(
        windows,
        key=lambda item: (
            item["absolute_grade_ratio"],
            -item["start_distance_m"],
        ),
    )
    maximum_absolute = worst["absolute_grade_ratio"]
    failure_reasons = []
    if maximum_absolute > limits["maximum_absolute_grade_ratio"] + 1.0e-12:
        failure_reasons.append(
            "longitudinal grade exceeds the controller capability"
        )
    return {
        **base,
        "status": "FAIL" if failure_reasons else "PASS",
        "maximum_absolute_grade_ratio": maximum_absolute,
        "maximum_absolute_grade_percent": 100.0 * maximum_absolute,
        "maximum_uphill_grade_ratio": max(
            0.0, max(item["signed_grade_ratio"] for item in windows)
        ),
        "maximum_downhill_grade_ratio": max(
            0.0, -min(item["signed_grade_ratio"] for item in windows)
        ),
        "worst_window": worst,
        "failure_reasons": failure_reasons,
    }


def _initial_approach_geometry(
    points: Sequence[Mapping[str, Any]], distance_limit_m: float
) -> dict[str, float]:
    first = points[0]
    first_distance = float(first["distance_m"])
    first_x = float(first["x"])
    first_y = float(first["y"])
    first_yaw = float(first["yaw"])
    normal_x = -math.sin(first_yaw)
    normal_y = math.cos(first_yaw)
    maximum_lateral = 0.0
    maximum_heading = 0.0
    covered_distance = 0.0

    def measure(x: float, y: float, yaw: float) -> None:
        nonlocal maximum_lateral, maximum_heading
        dx = x - first_x
        dy = y - first_y
        maximum_lateral = max(
            maximum_lateral, abs(dx * normal_x + dy * normal_y)
        )
        maximum_heading = max(
            maximum_heading,
            abs(math.degrees(_normalize_angle(yaw - first_yaw))),
        )

    previous = first
    for current in points[1:]:
        segment_start = float(previous["distance_m"]) - first_distance
        segment_end = float(current["distance_m"]) - first_distance
        segment_length = segment_end - segment_start
        if segment_length <= 1.0e-9:
            previous = current
            continue
        if segment_end >= distance_limit_m:
            ratio = max(
                0.0,
                min(1.0, (distance_limit_m - segment_start) / segment_length),
            )
            yaw_delta = _normalize_angle(
                float(current["yaw"]) - float(previous["yaw"])
            )
            measure(
                float(previous["x"])
                + ratio * (float(current["x"]) - float(previous["x"])),
                float(previous["y"])
                + ratio * (float(current["y"]) - float(previous["y"])),
                float(previous["yaw"]) + ratio * yaw_delta,
            )
            covered_distance = distance_limit_m
            break
        measure(
            float(current["x"]),
            float(current["y"]),
            float(current["yaw"]),
        )
        covered_distance = segment_end
        previous = current
    return {
        "distance_m": distance_limit_m,
        "covered_distance_m": covered_distance,
        "maximum_lateral_deviation_m": maximum_lateral,
        "maximum_heading_change_deg": maximum_heading,
    }


def analyze_serialized_physical_turn(
    payload: Mapping[str, Any], required: Mapping[str, Any]
) -> dict[str, Any]:
    """Measure an exact serialized left/right route before closed-loop execution."""
    limits = _validated_contract(required)
    points = _serialized_points(payload)
    scenario = payload.get("scenario")
    if scenario not in ("left", "right"):
        raise _contract_error(
            "physical-turn route scenario must be left or right"
        )
    target = str(scenario).upper()
    expected_sign = 1.0 if scenario == "left" else -1.0
    options = [str(point["road_option"]) for point in points]
    target_indices = [
        index for index, option in enumerate(options) if option == target
    ]
    blocks: list[list[int]] = []
    for index in target_indices:
        if not blocks or index != blocks[-1][-1] + 1:
            blocks.append([index])
        else:
            blocks[-1].append(index)
    additional = sorted((set(options) & TURN_MANEUVER_OPTIONS) - {target})
    reasons: list[str] = []
    if len(blocks) != 1:
        reasons.append(
            f"expected exactly one contiguous {target} block, found {len(blocks)}"
        )
    if additional:
        reasons.append("additional maneuver commands present: " + ", ".join(additional))

    first_distance = float(points[0]["distance_m"])
    last_distance = float(points[-1]["distance_m"])
    route_length = last_distance - first_distance
    longitudinal_grade = _longitudinal_grade_geometry(points, limits)
    reasons.extend(longitudinal_grade["failure_reasons"])
    initial_approach = _initial_approach_geometry(
        points, limits["initial_approach_distance_m"]
    )
    initial_checks = (
        (
            initial_approach["covered_distance_m"]
            < limits["initial_approach_distance_m"] - 1.0e-6,
            "route does not cover the required initial approach distance",
        ),
        (
            initial_approach["maximum_lateral_deviation_m"]
            > limits["maximum_initial_lateral_deviation_m"],
            "initial lateral deviation exceeds the maximum",
        ),
        (
            initial_approach["maximum_heading_change_deg"]
            > limits["maximum_initial_heading_change_deg"],
            "initial heading change exceeds the maximum",
        ),
    )
    reasons.extend(message for failed, message in initial_checks if failed)

    selected_block: dict[str, Any] | None = None
    if len(blocks) == 1:
        start_index = blocks[0][0]
        end_index = blocks[0][-1]
        block_start = (
            0.5
            * (
                float(points[start_index - 1]["distance_m"])
                + float(points[start_index]["distance_m"])
            )
            if start_index
            else float(points[start_index]["distance_m"])
        )
        block_end = (
            0.5
            * (
                float(points[end_index]["distance_m"])
                + float(points[end_index + 1]["distance_m"])
            )
            if end_index + 1 < len(points)
            else float(points[end_index]["distance_m"])
        )
        support_start = max(0, start_index - 1)
        support_end = min(len(points) - 1, end_index + 1)
        unwrapped = [float(points[support_start]["yaw"])]
        for point in points[support_start + 1 : support_end + 1]:
            yaw = float(point["yaw"])
            unwrapped.append(unwrapped[-1] + _normalize_angle(yaw - unwrapped[-1]))
        signed_heading_deg = math.degrees(unwrapped[-1] - unwrapped[0])
        absolute_heading_deg = abs(signed_heading_deg)
        cumulative_heading_deg = math.degrees(
            sum(
                abs(end - start)
                for start, end in zip(unwrapped, unwrapped[1:])
            )
        )
        curvatures = []
        for offset in range(1, len(unwrapped)):
            point_index = support_start + offset
            segment_length = float(points[point_index]["distance_m"]) - float(
                points[point_index - 1]["distance_m"]
            )
            if segment_length <= 1.0e-6:
                continue
            curvatures.append(
                abs(unwrapped[offset] - unwrapped[offset - 1]) / segment_length
            )
        progress = [
            (
                float(points[support_start + offset]["distance_m"]),
                max(0.0, expected_sign * (yaw - unwrapped[0]) * 180.0 / math.pi),
            )
            for offset, yaw in enumerate(unwrapped)
        ]
        margin = limits["turn_alignment_heading_margin_deg"]
        lead_crossing = _first_progress_crossing(progress, margin)
        tail_crossing = _first_progress_crossing(
            progress, max(0.0, absolute_heading_deg - margin)
        )
        command_lead = (
            math.inf
            if lead_crossing is None
            else max(0.0, lead_crossing - block_start)
        )
        command_tail = (
            math.inf
            if tail_crossing is None
            else max(0.0, block_end - tail_crossing)
        )
        route_lead = block_start - first_distance
        route_tail = last_distance - block_end
        turn_arc = block_end - block_start
        p95_curvature = _linear_percentile(curvatures, 95.0)
        peak_curvature = max(curvatures, default=0.0)
        heading_excess = max(0.0, cumulative_heading_deg - absolute_heading_deg)
        selected_block = {
            "start_index": start_index,
            "end_index": end_index,
            "route_lead_distance_m": route_lead,
            "route_tail_distance_m": route_tail,
            "command_arc_length_m": turn_arc,
            "signed_net_heading_change_deg": signed_heading_deg,
            "absolute_net_heading_change_deg": absolute_heading_deg,
            "cumulative_absolute_heading_change_deg": cumulative_heading_deg,
            "heading_excess_deg": heading_excess,
            "command_lead_distance_m": command_lead,
            "command_tail_distance_m": command_tail,
            "p95_absolute_curvature_per_m": p95_curvature,
            "maximum_absolute_curvature_per_m": peak_curvature,
        }
        checks = (
            (
                route_lead < limits["minimum_route_lead_m"],
                "route lead distance is below the maneuver-lookahead buffer",
            ),
            (
                route_tail < limits["minimum_route_tail_m"],
                "route tail distance is below the post-turn recovery minimum",
            ),
            (
                turn_arc < limits["minimum_turn_arc_length_m"],
                "turn command arc length is below the minimum",
            ),
            (
                turn_arc > limits["maximum_turn_arc_length_m"],
                "turn command arc length exceeds the maximum",
            ),
            (
                absolute_heading_deg
                < limits["minimum_turn_heading_change_deg"],
                "turn heading change is below the minimum",
            ),
            (
                absolute_heading_deg
                > limits["maximum_turn_heading_change_deg"],
                "turn heading change exceeds the maximum",
            ),
            (
                expected_sign * signed_heading_deg <= 0.0,
                "turn heading direction disagrees with the directional command",
            ),
            (
                heading_excess > limits["maximum_turn_heading_excess_deg"],
                "turn heading excess exceeds the maximum",
            ),
            (
                command_lead > limits["maximum_turn_command_lead_m"],
                "turn command lead distance exceeds the maximum",
            ),
            (
                command_tail > limits["maximum_turn_command_tail_m"],
                "turn command tail distance exceeds the maximum",
            ),
            (
                p95_curvature > limits["maximum_p95_abs_curvature_per_m"],
                "turn p95 absolute curvature exceeds the maximum",
            ),
            (
                peak_curvature > limits["maximum_abs_curvature_per_m"],
                "turn peak absolute curvature exceeds the maximum",
            ),
        )
        reasons.extend(message for failed, message in checks if failed)

    return {
        "status": "FAIL" if reasons else "PASS",
        "scenario": scenario,
        "directional_command": target,
        "route_length_m": route_length,
        "directional_block_count": len(blocks),
        "additional_maneuver_commands": additional,
        "longitudinal_grade": longitudinal_grade,
        "initial_approach": {
            **initial_approach,
            "limits": {
                "maximum_lateral_deviation_m": limits[
                    "maximum_initial_lateral_deviation_m"
                ],
                "maximum_heading_change_deg": limits[
                    "maximum_initial_heading_change_deg"
                ],
            },
        },
        "initial_command_contract": {
            "required_command": "LANEFOLLOW",
            "maneuver_lookahead_m": limits["maneuver_lookahead_m"],
            "lead_buffer_m": limits["lead_buffer_m"],
            "minimum_route_lead_m": limits["minimum_route_lead_m"],
        },
        "selected_block": selected_block,
        "limits": dict(required),
        "contract_provenance": dict(SPEED_30KPH_TURN_CONTRACT_PROVENANCE),
        "failure_reasons": reasons,
    }
