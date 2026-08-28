#!/usr/bin/env python3
"""Move a CARLA route goal backward along its route polyline."""

from __future__ import annotations

import argparse
from collections import Counter
from copy import deepcopy
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
from typing import Any, Mapping, NamedTuple, Sequence
import xml.etree.ElementTree as ET


_EPSILON_M = 1.0e-9
_GEOMETRY_EPSILON_M = 1.0e-8
DEFAULT_VEHICLE_INFO = (
    Path(__file__).resolve().parents[2]
    / "src/launcher/autoware_launch/vehicle/sample_vehicle_launch/"
    "sample_vehicle_description/config/vehicle_info.param.yaml"
)


class GoalSetbackError(RuntimeError):
    """Raised when a route cannot be shortened without breaking its contract."""


class RigidTransform(NamedTuple):
    x_m: float
    y_m: float
    z_m: float
    yaw_rad: float


class VehicleDimensions(NamedTuple):
    wheel_base_m: float
    wheel_tread_m: float
    front_overhang_m: float
    rear_overhang_m: float
    left_overhang_m: float
    right_overhang_m: float


class RoadLanelet(NamedTuple):
    lanelet_id: str
    centerline_way_id: str | None
    centerline: tuple[tuple[float, float, float], ...]
    polygon: tuple[tuple[float, float, float], ...]
    bounds: tuple[float, float, float, float]


class CenterlineProjection(NamedTuple):
    lanelet_id: str
    centerline_way_id: str
    segment_index: int
    segment_ratio: float
    x: float
    y: float
    z: float
    yaw: float
    distance_m: float
    direction_cosine: float


def _finite_number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise GoalSetbackError(f"{label} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise GoalSetbackError(f"{label} must be finite")
    return result


def _normalize_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def _normalize_degrees(angle_deg: float) -> float:
    return (angle_deg + 180.0) % 360.0 - 180.0


def _route_pose(point: Any, label: str) -> dict[str, float]:
    if not isinstance(point, dict):
        raise GoalSetbackError(f"{label} must be an object")
    return {
        name: _finite_number(point.get(name), f"{label}.{name}")
        for name in ("x", "y", "z", "yaw")
    }


def _segment_length(first: Mapping[str, float], second: Mapping[str, float]) -> float:
    dx = second["x"] - first["x"]
    dy = second["y"] - first["y"]
    planar_length = math.hypot(dx, dy)
    if planar_length <= _EPSILON_M:
        # Generated CARLA routes can end with a same-XY spawn pose several metres
        # above the road waypoint. It is a pose offset, not traveled route length.
        return 0.0
    return math.hypot(planar_length, second["z"] - first["z"])


def _interpolate_angle(first: float, second: float, ratio: float) -> float:
    return _normalize_angle(first + ratio * _normalize_angle(second - first))


def _interpolated_point(
    first: Mapping[str, Any], second: Mapping[str, Any], ratio: float
) -> dict[str, Any]:
    first_pose = _route_pose(first, "segment start")
    second_pose = _route_pose(second, "segment end")
    output = deepcopy(first)
    for name in ("x", "y", "z"):
        output[name] = first_pose[name] + ratio * (second_pose[name] - first_pose[name])
    output["yaw"] = _interpolate_angle(first_pose["yaw"], second_pose["yaw"], ratio)
    return output


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with path.open("rb") as stream:
            while block := stream.read(1024 * 1024):
                digest.update(block)
    except OSError as error:
        raise GoalSetbackError(f"cannot hash {path}: {error}") from error
    return digest.hexdigest()


def _apply_transform_pose(
    pose: Mapping[str, float], transform: RigidTransform
) -> dict[str, float]:
    cosine = math.cos(transform.yaw_rad)
    sine = math.sin(transform.yaw_rad)
    return {
        "x": transform.x_m + cosine * pose["x"] - sine * pose["y"],
        "y": transform.y_m + sine * pose["x"] + cosine * pose["y"],
        "z": transform.z_m + pose["z"],
        "yaw": _normalize_angle(pose["yaw"] + transform.yaw_rad),
    }


def _inverse_transform_pose(
    pose: Mapping[str, float], transform: RigidTransform
) -> dict[str, float]:
    cosine = math.cos(transform.yaw_rad)
    sine = math.sin(transform.yaw_rad)
    dx = pose["x"] - transform.x_m
    dy = pose["y"] - transform.y_m
    return {
        "x": cosine * dx + sine * dy,
        "y": -sine * dx + cosine * dy,
        "z": pose["z"] - transform.z_m,
        "yaw": _normalize_angle(pose["yaw"] - transform.yaw_rad),
    }


def load_map_bundle(
    path: Path,
) -> tuple[dict[str, Any], RigidTransform, Path, Path, str, str]:
    requested_path = path.expanduser().resolve()
    bundle_path = requested_path / "map_bundle.json" if requested_path.is_dir() else requested_path
    try:
        bundle_bytes = bundle_path.read_bytes()
        bundle = json.loads(bundle_bytes.decode("utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise GoalSetbackError(f"cannot read map bundle {bundle_path}: {error}") from error
    if not isinstance(bundle, dict) or bundle.get("schema_version") != 1:
        raise GoalSetbackError("map bundle must be a schema-version-1 object")
    raw_transform = bundle.get("carla_to_map_transform")
    if not isinstance(raw_transform, dict):
        raise GoalSetbackError("map bundle has no carla_to_map_transform object")
    transform = RigidTransform(
        x_m=_finite_number(raw_transform.get("x_m"), "carla_to_map_transform.x_m"),
        y_m=_finite_number(raw_transform.get("y_m"), "carla_to_map_transform.y_m"),
        z_m=_finite_number(raw_transform.get("z_m"), "carla_to_map_transform.z_m"),
        yaw_rad=_finite_number(
            raw_transform.get("yaw_rad"), "carla_to_map_transform.yaw_rad"
        ),
    )

    local_osm = bundle_path.parent / "lanelet2_map.osm"
    source = bundle.get("bundle_sources", {}).get("lanelet2_map", {})
    if local_osm.is_file():
        osm_path = local_osm.resolve()
    elif isinstance(source, dict) and isinstance(source.get("resolved_path"), str):
        osm_path = Path(source["resolved_path"]).expanduser().resolve()
    else:
        raise GoalSetbackError("map bundle does not resolve a Lanelet2 OSM file")
    if not osm_path.is_file():
        raise GoalSetbackError(f"Lanelet2 OSM does not exist: {osm_path}")
    osm_sha256 = _sha256_file(osm_path)
    derivation = bundle.get("lanelet2_derivation")
    derived_result = derivation.get("result", {}) if isinstance(derivation, dict) else {}
    declared_sha256 = (
        derived_result.get("output_sha256")
        if local_osm.is_file() and isinstance(derived_result, dict)
        else source.get("sha256") if isinstance(source, dict) else None
    )
    if declared_sha256 is not None and declared_sha256 != osm_sha256:
        raise GoalSetbackError(
            f"Lanelet2 OSM SHA256 mismatch: {osm_sha256} != {declared_sha256}"
        )
    return (
        bundle,
        transform,
        bundle_path,
        osm_path,
        hashlib.sha256(bundle_bytes).hexdigest(),
        osm_sha256,
    )


def load_vehicle_dimensions(path: Path) -> VehicleDimensions:
    try:
        import yaml
    except ImportError as error:
        raise GoalSetbackError("PyYAML is required to read vehicle-info YAML") from error
    try:
        payload = yaml.safe_load(path.expanduser().resolve().read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise GoalSetbackError(f"cannot read vehicle-info YAML {path}: {error}") from error
    if not isinstance(payload, dict):
        raise GoalSetbackError("vehicle-info YAML root must be an object")
    parameter_sets = [
        value.get("ros__parameters")
        for value in payload.values()
        if isinstance(value, dict) and isinstance(value.get("ros__parameters"), dict)
    ]
    if len(parameter_sets) != 1:
        raise GoalSetbackError("vehicle-info YAML must contain one ros__parameters object")
    parameters = parameter_sets[0]
    field_map = {
        "wheel_base_m": "wheel_base",
        "wheel_tread_m": "wheel_tread",
        "front_overhang_m": "front_overhang",
        "rear_overhang_m": "rear_overhang",
        "left_overhang_m": "left_overhang",
        "right_overhang_m": "right_overhang",
    }
    values = {
        field: _finite_number(parameters.get(parameter), f"vehicle_info.{parameter}")
        for field, parameter in field_map.items()
    }
    if any(values[field] <= 0.0 for field in ("wheel_base_m", "wheel_tread_m")):
        raise GoalSetbackError("vehicle wheel_base and wheel_tread must be positive")
    if any(
        values[field] < 0.0
        for field in (
            "front_overhang_m",
            "rear_overhang_m",
            "left_overhang_m",
            "right_overhang_m",
        )
    ):
        raise GoalSetbackError("vehicle overhang values must be non-negative")
    return VehicleDimensions(**values)


def _local_name(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def _children(element: ET.Element, name: str):
    return (child for child in element if _local_name(child.tag) == name)


def _element_tags(element: ET.Element) -> dict[str, str]:
    return {
        child.attrib.get("k", ""): child.attrib.get("v", "")
        for child in _children(element, "tag")
    }


def _deduplicate_points(
    points: Sequence[tuple[float, float, float]],
) -> list[tuple[float, float, float]]:
    result: list[tuple[float, float, float]] = []
    for point in points:
        if not result or point != result[-1]:
            result.append(point)
    if len(result) > 1 and result[0] == result[-1]:
        result.pop()
    return result


def load_road_lanelets(osm_path: Path) -> list[RoadLanelet]:
    try:
        root = ET.parse(osm_path).getroot()
    except (OSError, ET.ParseError) as error:
        raise GoalSetbackError(f"cannot parse Lanelet2 OSM {osm_path}: {error}") from error

    nodes: dict[str, tuple[float, float, float]] = {}
    for element in _children(root, "node"):
        node_id = element.attrib.get("id")
        tags = _element_tags(element)
        if not node_id or not all(name in tags for name in ("local_x", "local_y", "ele")):
            continue
        try:
            point = tuple(float(tags[name]) for name in ("local_x", "local_y", "ele"))
        except ValueError as error:
            raise GoalSetbackError(f"OSM node {node_id} has invalid local coordinates") from error
        if not all(math.isfinite(value) for value in point):
            raise GoalSetbackError(f"OSM node {node_id} has non-finite local coordinates")
        nodes[node_id] = point

    ways: dict[str, tuple[tuple[float, float, float], ...]] = {}
    for element in _children(root, "way"):
        way_id = element.attrib.get("id")
        if not way_id:
            continue
        refs = [child.attrib.get("ref", "") for child in _children(element, "nd")]
        if any(reference not in nodes for reference in refs):
            continue
        ways[way_id] = tuple(_deduplicate_points([nodes[reference] for reference in refs]))

    lanelets: list[RoadLanelet] = []
    explicit_centerline_count = 0
    for relation in _children(root, "relation"):
        tags = _element_tags(relation)
        if tags.get("type") != "lanelet" or tags.get("subtype") != "road":
            continue
        lanelet_id = relation.attrib.get("id", "<unknown>")
        members = {
            member.attrib.get("role", ""): member.attrib.get("ref", "")
            for member in _children(relation, "member")
            if member.attrib.get("type") == "way"
        }
        if not all(role in members and members[role] in ways for role in ("left", "right")):
            continue
        left = list(ways[members["left"]])
        right = list(ways[members["right"]])
        if len(left) < 2 or len(right) < 2:
            continue
        same_direction_cost = math.dist(left[0][:2], right[0][:2]) + math.dist(
            left[-1][:2], right[-1][:2]
        )
        opposite_direction_cost = math.dist(left[0][:2], right[-1][:2]) + math.dist(
            left[-1][:2], right[0][:2]
        )
        polygon = left + (list(reversed(right)) if same_direction_cost <= opposite_direction_cost else right)
        polygon = _deduplicate_points(polygon)
        if len(polygon) < 3:
            continue
        twice_area = abs(
            sum(
                polygon[index - 1][0] * polygon[index][1]
                - polygon[index][0] * polygon[index - 1][1]
                for index in range(len(polygon))
            )
        )
        if twice_area <= _EPSILON_M:
            continue
        centerline_way_id = members.get("centerline")
        centerline = ways.get(centerline_way_id, ())
        if len(centerline) < 2:
            centerline_way_id = None
            centerline = ()
        else:
            explicit_centerline_count += 1
        xs = [point[0] for point in polygon]
        ys = [point[1] for point in polygon]
        lanelets.append(
            RoadLanelet(
                lanelet_id=lanelet_id,
                centerline_way_id=centerline_way_id,
                centerline=tuple(centerline),
                polygon=tuple(polygon),
                bounds=(min(xs), min(ys), max(xs), max(ys)),
            )
        )
    if not lanelets:
        raise GoalSetbackError("Lanelet2 OSM has no usable road lanelets")
    if not explicit_centerline_count:
        raise GoalSetbackError("Lanelet2 OSM has no explicit road-lanelet centerlines")
    return lanelets


def nearest_forward_centerline(
    x: float,
    y: float,
    route_yaw: float,
    lanelets: Sequence[RoadLanelet],
    minimum_direction_cosine: float = 0.0,
) -> CenterlineProjection:
    minimum_direction_cosine = _finite_number(
        minimum_direction_cosine, "minimum_direction_cosine"
    )
    if not -1.0 <= minimum_direction_cosine <= 1.0:
        raise GoalSetbackError("minimum_direction_cosine must be in [-1, 1]")
    route_x = math.cos(route_yaw)
    route_y = math.sin(route_yaw)
    candidates: list[tuple[tuple[float, float, str, int], CenterlineProjection]] = []
    for lanelet in lanelets:
        if lanelet.centerline_way_id is None:
            continue
        for segment_index, (first, second) in enumerate(
            zip(lanelet.centerline, lanelet.centerline[1:])
        ):
            dx = second[0] - first[0]
            dy = second[1] - first[1]
            length_squared = dx * dx + dy * dy
            if length_squared <= _EPSILON_M:
                continue
            length = math.sqrt(length_squared)
            direction_cosine = (route_x * dx + route_y * dy) / length
            if direction_cosine < minimum_direction_cosine - 1.0e-12:
                continue
            ratio = ((x - first[0]) * dx + (y - first[1]) * dy) / length_squared
            ratio = max(0.0, min(1.0, ratio))
            projected_x = first[0] + ratio * dx
            projected_y = first[1] + ratio * dy
            projection = CenterlineProjection(
                lanelet_id=lanelet.lanelet_id,
                centerline_way_id=lanelet.centerline_way_id,
                segment_index=segment_index,
                segment_ratio=ratio,
                x=projected_x,
                y=projected_y,
                z=first[2] + ratio * (second[2] - first[2]),
                yaw=math.atan2(dy, dx),
                distance_m=math.hypot(x - projected_x, y - projected_y),
                direction_cosine=direction_cosine,
            )
            key = (
                projection.distance_m,
                -projection.direction_cosine,
                projection.lanelet_id,
                projection.segment_index,
            )
            candidates.append((key, projection))
    if not candidates:
        raise GoalSetbackError(
            "no explicit Lanelet2 centerline preserves the route's forward direction"
        )
    return min(candidates, key=lambda item: item[0])[1]


def vehicle_footprint(
    pose: Mapping[str, float], vehicle: VehicleDimensions
) -> tuple[tuple[float, float], ...]:
    x_front = vehicle.wheel_base_m + vehicle.front_overhang_m
    x_center = vehicle.wheel_base_m / 2.0
    x_rear = -vehicle.rear_overhang_m
    y_left = vehicle.wheel_tread_m / 2.0 + vehicle.left_overhang_m
    y_right = -(vehicle.wheel_tread_m / 2.0 + vehicle.right_overhang_m)
    local_points = (
        (x_front, y_left),
        (x_front, y_right),
        (x_center, y_right),
        (x_rear, y_right),
        (x_rear, y_left),
        (x_center, y_left),
    )
    cosine = math.cos(pose["yaw"])
    sine = math.sin(pose["yaw"])
    return tuple(
        (
            pose["x"] + cosine * local_x - sine * local_y,
            pose["y"] + sine * local_x + cosine * local_y,
        )
        for local_x, local_y in local_points
    )


def _cross(first: tuple[float, float], second: tuple[float, float]) -> float:
    return first[0] * second[1] - first[1] * second[0]


def _point_on_segment(
    point: tuple[float, float], start: tuple[float, ...], end: tuple[float, ...]
) -> bool:
    segment = end[0] - start[0], end[1] - start[1]
    offset = point[0] - start[0], point[1] - start[1]
    if abs(_cross(segment, offset)) > _GEOMETRY_EPSILON_M * max(1.0, math.hypot(*segment)):
        return False
    return (
        min(start[0], end[0]) - _GEOMETRY_EPSILON_M
        <= point[0]
        <= max(start[0], end[0]) + _GEOMETRY_EPSILON_M
        and min(start[1], end[1]) - _GEOMETRY_EPSILON_M
        <= point[1]
        <= max(start[1], end[1]) + _GEOMETRY_EPSILON_M
    )


def _point_covered_by_polygon(
    point: tuple[float, float], polygon: Sequence[tuple[float, ...]]
) -> bool:
    inside = False
    previous = polygon[-1]
    for current in polygon:
        if _point_on_segment(point, previous, current):
            return True
        if (current[1] > point[1]) != (previous[1] > point[1]):
            crossing_x = (previous[0] - current[0]) * (point[1] - current[1]) / (
                previous[1] - current[1]
            ) + current[0]
            if point[0] < crossing_x:
                inside = not inside
        previous = current
    return inside


def _intersection_parameters(
    first_start: tuple[float, float],
    first_end: tuple[float, float],
    second_start: tuple[float, ...],
    second_end: tuple[float, ...],
) -> list[float]:
    first_vector = first_end[0] - first_start[0], first_end[1] - first_start[1]
    second_vector = second_end[0] - second_start[0], second_end[1] - second_start[1]
    offset = second_start[0] - first_start[0], second_start[1] - first_start[1]
    denominator = _cross(first_vector, second_vector)
    if abs(denominator) > _GEOMETRY_EPSILON_M:
        first_ratio = _cross(offset, second_vector) / denominator
        second_ratio = _cross(offset, first_vector) / denominator
        if (
            -_GEOMETRY_EPSILON_M <= first_ratio <= 1.0 + _GEOMETRY_EPSILON_M
            and -_GEOMETRY_EPSILON_M <= second_ratio <= 1.0 + _GEOMETRY_EPSILON_M
        ):
            return [max(0.0, min(1.0, first_ratio))]
        return []
    if abs(_cross(offset, first_vector)) > _GEOMETRY_EPSILON_M:
        return []
    length_squared = first_vector[0] ** 2 + first_vector[1] ** 2
    if length_squared <= _EPSILON_M:
        return [0.0] if _point_on_segment(first_start, second_start, second_end) else []
    ratios = []
    for point in (second_start, second_end):
        ratio = (
            (point[0] - first_start[0]) * first_vector[0]
            + (point[1] - first_start[1]) * first_vector[1]
        ) / length_squared
        if -_GEOMETRY_EPSILON_M <= ratio <= 1.0 + _GEOMETRY_EPSILON_M:
            ratios.append(max(0.0, min(1.0, ratio)))
    return ratios


def _segment_covered_by_polygon(
    start: tuple[float, float],
    end: tuple[float, float],
    polygon: Sequence[tuple[float, ...]],
) -> bool:
    parameters = [0.0, 1.0]
    for boundary_start, boundary_end in zip((polygon[-1], *polygon[:-1]), polygon):
        parameters.extend(_intersection_parameters(start, end, boundary_start, boundary_end))
    parameters.sort()
    unique = [parameters[0]]
    for value in parameters[1:]:
        if value - unique[-1] > 1.0e-10:
            unique.append(value)
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    for first_ratio, second_ratio in zip(unique, unique[1:]):
        ratio = (first_ratio + second_ratio) / 2.0
        if not _point_covered_by_polygon((start[0] + ratio * dx, start[1] + ratio * dy), polygon):
            return False
    return _point_covered_by_polygon(start, polygon) and _point_covered_by_polygon(end, polygon)


def polygon_covers_footprint(
    polygon: Sequence[tuple[float, ...]], footprint: Sequence[tuple[float, float]]
) -> bool:
    return all(
        _segment_covered_by_polygon(start, end, polygon)
        for start, end in zip((footprint[-1], *footprint[:-1]), footprint)
    )


def _point_segment_distance(
    point: tuple[float, float], start: tuple[float, ...], end: tuple[float, ...]
) -> float:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length_squared = dx * dx + dy * dy
    if length_squared <= _EPSILON_M:
        return math.hypot(point[0] - start[0], point[1] - start[1])
    ratio = ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length_squared
    ratio = max(0.0, min(1.0, ratio))
    return math.hypot(point[0] - start[0] - ratio * dx, point[1] - start[1] - ratio * dy)


def _segment_distance(
    first_start: tuple[float, float],
    first_end: tuple[float, float],
    second_start: tuple[float, ...],
    second_end: tuple[float, ...],
) -> float:
    if _intersection_parameters(first_start, first_end, second_start, second_end):
        return 0.0
    return min(
        _point_segment_distance(first_start, second_start, second_end),
        _point_segment_distance(first_end, second_start, second_end),
        _point_segment_distance((second_start[0], second_start[1]), first_start, first_end),
        _point_segment_distance((second_end[0], second_end[1]), first_start, first_end),
    )


def footprint_boundary_clearance(
    polygon: Sequence[tuple[float, ...]], footprint: Sequence[tuple[float, float]]
) -> float:
    return min(
        _segment_distance(footprint_start, footprint_end, boundary_start, boundary_end)
        for footprint_start, footprint_end in zip((footprint[-1], *footprint[:-1]), footprint)
        for boundary_start, boundary_end in zip((polygon[-1], *polygon[:-1]), polygon)
    )


def footprint_coverage(
    footprint: Sequence[tuple[float, float]], lanelets: Sequence[RoadLanelet]
) -> list[tuple[float, RoadLanelet]]:
    xs = [point[0] for point in footprint]
    ys = [point[1] for point in footprint]
    footprint_bounds = min(xs), min(ys), max(xs), max(ys)
    covered = []
    for lanelet in lanelets:
        min_x, min_y, max_x, max_y = lanelet.bounds
        if (
            footprint_bounds[0] < min_x - _GEOMETRY_EPSILON_M
            or footprint_bounds[1] < min_y - _GEOMETRY_EPSILON_M
            or footprint_bounds[2] > max_x + _GEOMETRY_EPSILON_M
            or footprint_bounds[3] > max_y + _GEOMETRY_EPSILON_M
        ):
            continue
        if polygon_covers_footprint(lanelet.polygon, footprint):
            covered.append((footprint_boundary_clearance(lanelet.polygon, footprint), lanelet))
    return sorted(covered, key=lambda item: (-item[0], item[1].lanelet_id))


def _inverse_map_alignment(pose: Mapping[str, float], route: Mapping[str, Any]) -> dict[str, float]:
    metadata = route.get("coordinate_alignment")
    if metadata is None:
        return dict(pose)
    if not isinstance(metadata, dict):
        raise GoalSetbackError("coordinate_alignment must be an object")
    if metadata.get("source_frame") != "carla_map" or metadata.get("target_frame") != "map":
        raise GoalSetbackError("coordinate_alignment must describe carla_map -> map")
    transform = metadata.get("carla_to_map_transform")
    if not isinstance(transform, dict):
        raise GoalSetbackError("coordinate_alignment has no carla_to_map_transform object")
    x_offset = _finite_number(transform.get("x_m"), "coordinate_alignment.x_m")
    y_offset = _finite_number(transform.get("y_m"), "coordinate_alignment.y_m")
    z_offset = _finite_number(transform.get("z_m"), "coordinate_alignment.z_m")
    yaw_offset = _finite_number(transform.get("yaw_rad"), "coordinate_alignment.yaw_rad")
    cosine = math.cos(yaw_offset)
    sine = math.sin(yaw_offset)
    dx = pose["x"] - x_offset
    dy = pose["y"] - y_offset
    return {
        "x": cosine * dx + sine * dy,
        "y": -sine * dx + cosine * dy,
        "z": pose["z"] - z_offset,
        "yaw": _normalize_angle(pose["yaw"] - yaw_offset),
    }


def _goal_carla_transform(
    goal_pose: Mapping[str, float], route: Mapping[str, Any]
) -> tuple[dict[str, Any], list[str], str]:
    original = route.get("goal_carla_transform")
    if not isinstance(original, dict):
        raise GoalSetbackError("goal_carla_transform must be an object")
    for name in ("x", "y", "z", "yaw"):
        _finite_number(original.get(name), f"goal_carla_transform.{name}")

    carla_ros_pose = _inverse_map_alignment(goal_pose, route)
    output = deepcopy(original)
    output.update(
        {
            "x": carla_ros_pose["x"],
            "y": -carla_ros_pose["y"],
            "z": carla_ros_pose["z"],
            "yaw": _normalize_degrees(-math.degrees(carla_ros_pose["yaw"])),
        }
    )
    preserved = [name for name in ("roll", "pitch") if name in original]
    conversion = (
        "inverse_map_alignment_then_ros_to_carla"
        if route.get("coordinate_alignment") is not None
        else "ros_to_carla"
    )
    return output, preserved, conversion


def _recompute_route_metadata(points: list[dict[str, Any]]) -> float:
    poses = [_route_pose(point, f"route[{index}]") for index, point in enumerate(points)]
    distances = [0.0]
    for first, second in zip(poses, poses[1:]):
        distances.append(distances[-1] + _segment_length(first, second))
    total = distances[-1]
    for index, (point, distance) in enumerate(zip(points, distances)):
        point["index"] = index
        point["distance_m"] = distance
        point["remaining_m"] = max(0.0, total - distance)
    return total


def trim_route_payload(
    route: Mapping[str, Any],
    setback_m: float,
    *,
    source_route: str,
    source_sha256: str,
) -> dict[str, Any]:
    """Return a shortened schema-v1 route without mutating ``route``."""
    setback_m = _finite_number(setback_m, "setback_m")
    if setback_m <= 0.0:
        raise GoalSetbackError("setback_m must be positive")
    if not isinstance(route, dict):
        raise GoalSetbackError("route JSON root must be an object")
    if route.get("schema_version") != 1:
        raise GoalSetbackError("route schema_version must be 1")
    if route.get("coordinate_reference", "base_link") != "base_link":
        raise GoalSetbackError("route coordinates must use ROS base_link")
    if "goal_adjustment" in route:
        raise GoalSetbackError(
            "route already contains goal_adjustment; apply setback to its recorded source"
        )

    raw_points = route.get("route")
    if not isinstance(raw_points, list) or len(raw_points) < 2:
        raise GoalSetbackError("route must contain at least two points")
    points = deepcopy(raw_points)
    poses = [_route_pose(point, f"route[{index}]") for index, point in enumerate(points)]
    for index, point in enumerate(points):
        option = point.get("road_option")
        if not isinstance(option, str) or not option:
            raise GoalSetbackError(f"route[{index}].road_option must be a non-empty string")

    cumulative = [0.0]
    for first, second in zip(poses, poses[1:]):
        cumulative.append(cumulative[-1] + _segment_length(first, second))
    original_length = cumulative[-1]
    if original_length <= _EPSILON_M:
        raise GoalSetbackError("route has no traveled arc length")
    if setback_m >= original_length - _EPSILON_M:
        raise GoalSetbackError(
            f"setback_m ({setback_m:.6g}) must be shorter than route length "
            f"({original_length:.6g})"
        )
    target_length = original_length - setback_m

    segment_index = None
    ratio = None
    for index in range(len(points) - 1):
        segment_length = cumulative[index + 1] - cumulative[index]
        if segment_length <= _EPSILON_M or cumulative[index + 1] < target_length - _EPSILON_M:
            continue
        segment_index = index
        ratio = (target_length - cumulative[index]) / segment_length
        ratio = max(0.0, min(1.0, ratio))
        break
    if segment_index is None or ratio is None:
        raise GoalSetbackError("could not locate setback target on route polyline")

    if ratio <= _EPSILON_M:
        shortened = points[: segment_index + 1]
        terminal_source_index = segment_index
        ratio = 0.0
    elif ratio >= 1.0 - _EPSILON_M:
        shortened = points[: segment_index + 2]
        terminal_source_index = segment_index + 1
        ratio = 1.0
    else:
        shortened = points[: segment_index + 1]
        shortened.append(_interpolated_point(points[segment_index], points[segment_index + 1], ratio))
        terminal_source_index = None

    new_length = _recompute_route_metadata(shortened)
    if len(shortened) < 2 or new_length <= _EPSILON_M:
        raise GoalSetbackError("setback leaves fewer than two usable route points")
    if not math.isclose(new_length, target_length, rel_tol=0.0, abs_tol=1.0e-7):
        raise GoalSetbackError(
            f"internal route length mismatch: expected {target_length:.9f}, got {new_length:.9f}"
        )

    terminal_pose = _route_pose(shortened[-1], "shortened route terminal")
    original_goal_pose = route.get("goal_ros_pose")
    _route_pose(original_goal_pose, "goal_ros_pose")
    goal_pose = deepcopy(original_goal_pose)
    goal_pose.update(terminal_pose)
    carla_goal, preserved_orientation, conversion = _goal_carla_transform(terminal_pose, route)

    output = deepcopy(route)
    output["route"] = shortened
    output["route_length_m"] = new_length
    output["goal_ros_pose"] = goal_pose
    output["goal_carla_transform"] = carla_goal
    if "goal_spawn_index" in output:
        output["goal_spawn_index"] = None
    output["option_counts"] = dict(Counter(point["road_option"] for point in shortened))
    output["goal_adjustment"] = {
        "schema_version": 1,
        "method": "route_arc_length_setback",
        "distance_metric": "xyz_arc_length_ignoring_zero_xy_pose_offsets",
        "requested_setback_m": setback_m,
        "actual_setback_m": original_length - new_length,
        "source_route": source_route,
        "source_route_sha256": source_sha256,
        "original_route_length_m": original_length,
        "original_declared_route_length_m": route.get("route_length_m"),
        "original_goal_spawn_index": route.get("goal_spawn_index"),
        "original_goal_ros_pose": deepcopy(original_goal_pose),
        "original_goal_carla_transform": deepcopy(route.get("goal_carla_transform")),
        "terminal_source_segment": [segment_index, segment_index + 1],
        "terminal_interpolation_ratio": ratio,
        "terminal_source_index": terminal_source_index,
        "goal_carla_transform_conversion": conversion,
        "preserved_carla_orientation_fields": preserved_orientation,
    }
    return output


def trim_route_payload_map_aware(
    route: Mapping[str, Any],
    transform: RigidTransform,
    lanelets: Sequence[RoadLanelet],
    vehicle: VehicleDimensions,
    *,
    minimum_setback_m: float,
    minimum_footprint_clearance_m: float,
    maximum_centerline_snap_m: float,
    minimum_direction_cosine: float,
    source_route: str,
    source_sha256: str,
    map_provenance: Mapping[str, Any],
) -> dict[str, Any]:
    """Select and snap the latest footprint-safe raw-route terminal."""
    minimum_setback_m = _finite_number(minimum_setback_m, "minimum_setback_m")
    minimum_footprint_clearance_m = _finite_number(
        minimum_footprint_clearance_m, "minimum_footprint_clearance_m"
    )
    maximum_centerline_snap_m = _finite_number(
        maximum_centerline_snap_m, "maximum_centerline_snap_m"
    )
    if minimum_setback_m < 0.0:
        raise GoalSetbackError("minimum_setback_m must be non-negative")
    if minimum_footprint_clearance_m < 0.0:
        raise GoalSetbackError("minimum_footprint_clearance_m must be non-negative")
    if maximum_centerline_snap_m <= 0.0:
        raise GoalSetbackError("maximum_centerline_snap_m must be positive")
    if not isinstance(route, dict) or route.get("schema_version") != 1:
        raise GoalSetbackError("route must be a schema-version-1 object")
    if route.get("coordinate_reference", "base_link") != "base_link":
        raise GoalSetbackError("route coordinates must use ROS base_link")
    if "coordinate_alignment" in route:
        raise GoalSetbackError("map-aware mode requires a raw, unaligned CARLA route")
    if "goal_adjustment" in route:
        raise GoalSetbackError(
            "route already contains goal_adjustment; apply map-aware setback to its recorded source"
        )
    raw_points = route.get("route")
    if not isinstance(raw_points, list) or len(raw_points) < 2:
        raise GoalSetbackError("route must contain at least two points")
    points = deepcopy(raw_points)
    raw_poses = [_route_pose(point, f"route[{index}]") for index, point in enumerate(points)]
    for index, point in enumerate(points):
        if not isinstance(point.get("road_option"), str) or not point["road_option"]:
            raise GoalSetbackError(f"route[{index}].road_option must be a non-empty string")
    original_goal_pose = route.get("goal_ros_pose")
    _route_pose(original_goal_pose, "goal_ros_pose")
    if not isinstance(route.get("goal_carla_transform"), dict):
        raise GoalSetbackError("goal_carla_transform must be an object")

    cumulative = [0.0]
    for first, second in zip(raw_poses, raw_poses[1:]):
        cumulative.append(cumulative[-1] + _segment_length(first, second))
    original_length = cumulative[-1]
    if original_length <= _EPSILON_M:
        raise GoalSetbackError("route has no traveled arc length")
    aligned_poses = [_apply_transform_pose(pose, transform) for pose in raw_poses]

    selected = None
    rejection_counts: Counter[str] = Counter()
    for candidate_index in range(len(points) - 1, 0, -1):
        original_remaining = original_length - cumulative[candidate_index]
        if original_remaining + _GEOMETRY_EPSILON_M < minimum_setback_m:
            rejection_counts["minimum_setback"] += 1
            continue
        aligned_candidate = aligned_poses[candidate_index]
        try:
            projection = nearest_forward_centerline(
                aligned_candidate["x"],
                aligned_candidate["y"],
                aligned_candidate["yaw"],
                lanelets,
                minimum_direction_cosine,
            )
        except GoalSetbackError:
            rejection_counts["forward_centerline"] += 1
            continue
        if projection.distance_m > maximum_centerline_snap_m + _GEOMETRY_EPSILON_M:
            rejection_counts["centerline_snap_distance"] += 1
            continue
        aligned_goal = {
            "x": projection.x,
            "y": projection.y,
            "z": projection.z,
            "yaw": projection.yaw,
        }
        footprint = vehicle_footprint(aligned_goal, vehicle)
        covered = footprint_coverage(footprint, lanelets)
        matching = [item for item in covered if item[1].lanelet_id == projection.lanelet_id]
        if not matching:
            rejection_counts["footprint_not_covered_by_centerline_lanelet"] += 1
            continue
        clearance, covering_lanelet = matching[0]
        if clearance + _GEOMETRY_EPSILON_M < minimum_footprint_clearance_m:
            rejection_counts["footprint_clearance"] += 1
            continue

        raw_goal = _inverse_transform_pose(aligned_goal, transform)
        shortened = deepcopy(points[: candidate_index + 1])
        shortened[-1].update(raw_goal)
        new_length = _recompute_route_metadata(shortened)
        actual_setback = original_length - new_length
        if actual_setback + _GEOMETRY_EPSILON_M < minimum_setback_m:
            rejection_counts["minimum_setback_after_snap"] += 1
            continue
        selected = {
            "candidate_index": candidate_index,
            "original_remaining_m": original_remaining,
            "projection": projection,
            "aligned_goal": aligned_goal,
            "raw_goal": raw_goal,
            "footprint": footprint,
            "covered": covered,
            "covering_lanelet": covering_lanelet,
            "clearance_m": clearance,
            "shortened": shortened,
            "new_length": new_length,
            "actual_setback_m": actual_setback,
        }
        break
    if selected is None:
        diagnostics = ", ".join(
            f"{name}={count}" for name, count in sorted(rejection_counts.items())
        )
        raise GoalSetbackError(
            "no route terminal has a forward explicit centerline and a covered vehicle "
            f"footprint ({diagnostics or 'no usable candidates'})"
        )

    raw_goal = selected["raw_goal"]
    goal_pose = deepcopy(original_goal_pose)
    goal_pose.update(raw_goal)
    carla_goal, preserved_orientation, _ = _goal_carla_transform(raw_goal, route)
    projection = selected["projection"]
    covered = selected["covered"]
    output = deepcopy(route)
    output["route"] = selected["shortened"]
    output["route_length_m"] = selected["new_length"]
    output["goal_ros_pose"] = goal_pose
    output["goal_carla_transform"] = carla_goal
    if "goal_spawn_index" in output:
        output["goal_spawn_index"] = None
    output["option_counts"] = dict(
        Counter(point["road_option"] for point in selected["shortened"])
    )
    output["goal_adjustment"] = {
        "schema_version": 1,
        "method": "lanelet_explicit_centerline_safe_setback",
        "distance_metric": "xyz_arc_length_ignoring_zero_xy_pose_offsets",
        "minimum_requested_setback_m": minimum_setback_m,
        "candidate_original_remaining_m": selected["original_remaining_m"],
        "actual_setback_m": selected["actual_setback_m"],
        "source_route": source_route,
        "source_route_sha256": source_sha256,
        "original_route_length_m": original_length,
        "original_declared_route_length_m": route.get("route_length_m"),
        "original_goal_spawn_index": route.get("goal_spawn_index"),
        "original_goal_ros_pose": deepcopy(original_goal_pose),
        "original_goal_carla_transform": deepcopy(route.get("goal_carla_transform")),
        "terminal_source_index": selected["candidate_index"],
        "terminal_source_point_index": points[selected["candidate_index"]].get("index"),
        "aligned_source_terminal_pose": aligned_poses[selected["candidate_index"]],
        "aligned_snapped_goal_pose": selected["aligned_goal"],
        "raw_snapped_goal_pose": raw_goal,
        "centerline": {
            "source": "explicit_lanelet_relation_member",
            "lanelet_id": projection.lanelet_id,
            "way_id": projection.centerline_way_id,
            "segment_index": projection.segment_index,
            "segment_ratio": projection.segment_ratio,
            "snap_distance_m": projection.distance_m,
            "direction_cosine": projection.direction_cosine,
            "direction_was_reversed": False,
            "maximum_snap_distance_m": maximum_centerline_snap_m,
            "minimum_direction_cosine": minimum_direction_cosine,
        },
        "footprint_validation": {
            "coverage_contract": "fully_covered_by_explicit_centerline_lanelet",
            "vehicle_dimensions": vehicle._asdict(),
            "aligned_polygon": [list(point) for point in selected["footprint"]],
            "centerline_lanelet_id": projection.lanelet_id,
            "covering_lanelet_id": selected["covering_lanelet"].lanelet_id,
            "nearby_individually_covering_lanelet_ids": [
                lanelet.lanelet_id for _, lanelet in covered
            ],
            "boundary_clearance_m": selected["clearance_m"],
            "minimum_boundary_clearance_m": minimum_footprint_clearance_m,
            "status": "PASS",
        },
        "map": dict(map_provenance),
        "carla_to_map_transform": transform._asdict(),
        "goal_carla_transform_conversion": "map_alignment_inverse_then_ros_to_carla",
        "preserved_carla_orientation_fields": preserved_orientation,
        "candidate_rejections_before_selection": dict(rejection_counts),
    }
    return output


def _json_bytes(payload: Mapping[str, Any]) -> bytes:
    return (
        json.dumps(payload, indent=2, sort_keys=False, ensure_ascii=True, allow_nan=False)
        + "\n"
    ).encode("utf-8")


def _write_new_or_identical(path: Path, data: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        if not path.is_file():
            raise GoalSetbackError(f"output is not a regular file: {path}")
        if path.read_bytes() == data:
            return
        raise GoalSetbackError(f"refusing to overwrite different existing output: {path}")
    descriptor, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, path)
    finally:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass


def trim_route_file(input_path: Path, output_path: Path, setback_m: float) -> dict[str, Any]:
    input_path = input_path.expanduser().resolve()
    output_path = output_path.expanduser().resolve()
    if input_path == output_path:
        raise GoalSetbackError("input and output paths must be different")
    try:
        source_bytes = input_path.read_bytes()
        route = json.loads(source_bytes.decode("utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise GoalSetbackError(f"cannot read route JSON {input_path}: {error}") from error
    payload = trim_route_payload(
        route,
        setback_m,
        source_route=str(input_path),
        source_sha256=hashlib.sha256(source_bytes).hexdigest(),
    )
    _write_new_or_identical(output_path, _json_bytes(payload))
    return payload


def trim_route_file_map_aware(
    input_path: Path,
    output_path: Path,
    map_bundle_path: Path,
    vehicle_info_path: Path = DEFAULT_VEHICLE_INFO,
    *,
    minimum_setback_m: float = 0.0,
    minimum_footprint_clearance_m: float = 0.35,
    maximum_centerline_snap_m: float = 1.0,
    minimum_direction_cosine: float = 0.0,
) -> dict[str, Any]:
    input_path = input_path.expanduser().resolve()
    output_path = output_path.expanduser().resolve()
    if input_path == output_path:
        raise GoalSetbackError("input and output paths must be different")
    try:
        source_bytes = input_path.read_bytes()
        route = json.loads(source_bytes.decode("utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise GoalSetbackError(f"cannot read route JSON {input_path}: {error}") from error
    bundle, transform, bundle_path, osm_path, bundle_sha256, osm_sha256 = load_map_bundle(
        map_bundle_path
    )
    lanelets = load_road_lanelets(osm_path)
    vehicle_info_path = vehicle_info_path.expanduser().resolve()
    vehicle = load_vehicle_dimensions(vehicle_info_path)
    payload = trim_route_payload_map_aware(
        route,
        transform,
        lanelets,
        vehicle,
        minimum_setback_m=minimum_setback_m,
        minimum_footprint_clearance_m=minimum_footprint_clearance_m,
        maximum_centerline_snap_m=maximum_centerline_snap_m,
        minimum_direction_cosine=minimum_direction_cosine,
        source_route=str(input_path),
        source_sha256=hashlib.sha256(source_bytes).hexdigest(),
        map_provenance={
            "bundle": str(bundle_path),
            "bundle_sha256": bundle_sha256,
            "profile": bundle.get("profile", ""),
            "lanelet2_map": str(osm_path),
            "lanelet2_map_sha256": osm_sha256,
            "vehicle_info": str(vehicle_info_path),
            "vehicle_info_sha256": _sha256_file(vehicle_info_path),
        },
    )
    _write_new_or_identical(output_path, _json_bytes(payload))
    return payload


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("route_json", type=Path, help="source CARLA route JSON")
    parser.add_argument("--output", type=Path, required=True, help="new shortened route JSON")
    parser.add_argument(
        "--setback-m",
        type=float,
        help=(
            "exact arc length in basic mode; minimum setback in map-aware mode "
            "(default there: 0)"
        ),
    )
    parser.add_argument(
        "--map-bundle",
        type=Path,
        help="enable explicit-Lanelet-centerline and vehicle-footprint-safe selection",
    )
    parser.add_argument("--vehicle-info", type=Path, default=DEFAULT_VEHICLE_INFO)
    parser.add_argument("--minimum-footprint-clearance-m", type=float, default=0.35)
    parser.add_argument("--maximum-centerline-snap-m", type=float, default=1.0)
    parser.add_argument("--minimum-direction-cosine", type=float, default=0.0)
    parser.add_argument("--json", action="store_true", help="print result summary as JSON")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    if args.map_bundle is None:
        if args.setback_m is None:
            raise GoalSetbackError("--setback-m is required without --map-bundle")
        result = trim_route_file(args.route_json, args.output, args.setback_m)
    else:
        result = trim_route_file_map_aware(
            args.route_json,
            args.output,
            args.map_bundle,
            args.vehicle_info,
            minimum_setback_m=args.setback_m or 0.0,
            minimum_footprint_clearance_m=args.minimum_footprint_clearance_m,
            maximum_centerline_snap_m=args.maximum_centerline_snap_m,
            minimum_direction_cosine=args.minimum_direction_cosine,
        )
    summary = {
        "status": "PASS",
        "output": str(args.output.expanduser().resolve()),
        "setback_m": result["goal_adjustment"]["actual_setback_m"],
        "route_length_m": result["route_length_m"],
        "route_points": len(result["route"]),
        "goal_ros_pose": result["goal_ros_pose"],
        "goal_carla_transform": result["goal_carla_transform"],
        "method": result["goal_adjustment"]["method"],
    }
    if "footprint_validation" in result["goal_adjustment"]:
        summary["map_aware"] = {
            "terminal_source_index": result["goal_adjustment"]["terminal_source_index"],
            "aligned_goal_ros_pose": result["goal_adjustment"]["aligned_snapped_goal_pose"],
            "centerline": result["goal_adjustment"]["centerline"],
            "footprint_validation": result["goal_adjustment"]["footprint_validation"],
        }
    if args.json:
        print(json.dumps(summary, indent=2, sort_keys=False, allow_nan=False))
    else:
        print(
            f"route={summary['output']} setback={summary['setback_m']:.3f}m "
            f"length={summary['route_length_m']:.3f}m points={summary['route_points']}"
        )
        goal = summary["goal_ros_pose"]
        print(
            f"goal_ros_pose={goal['x']:.6f},{goal['y']:.6f},"
            f"{goal['z']:.6f},{goal['yaw']:.6f}"
        )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except GoalSetbackError as error:
        raise SystemExit(f"ERROR: {error}") from error
