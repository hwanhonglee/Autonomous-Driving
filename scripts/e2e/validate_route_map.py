#!/usr/bin/env python3

"""Validate that a CARLA route and an Autoware Local map describe the same road."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import re
import struct
import sys
import xml.etree.ElementTree as ET


REQUIRED_MAP_FILES = (
    "lanelet2_map.osm",
    "pointcloud_map.pcd",
    "map_projector_info.yaml",
)
TOWN_PATTERN = re.compile(r"town[0-9]+(?:hd)?(?:_opt)?", re.IGNORECASE)


class ValidationError(RuntimeError):
    """Raised when the route/map bundle is structurally inconsistent."""

    def __init__(self, errors: list[str]):
        self.errors = errors
        super().__init__("\n".join(errors))


@dataclass(frozen=True)
class LaneletPolygon:
    lanelet_id: str
    subtype: str
    points: tuple[tuple[float, float, float], ...]
    bounds: tuple[float, float, float, float]


def _local_name(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def _children(element: ET.Element, name: str):
    return (child for child in element if _local_name(child.tag) == name)


def _element_tags(element: ET.Element) -> dict[str, str]:
    return {
        child.attrib.get("k", ""): child.attrib.get("v", "")
        for child in _children(element, "tag")
    }


def _finite_number(value, label: str, errors: list[str]) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        errors.append(f"{label} must be a number")
        return None
    number = float(value)
    if not math.isfinite(number):
        errors.append(f"{label} must be finite")
        return None
    return number


def _route_pose(value, label: str, errors: list[str]) -> tuple[float, float, float] | None:
    if not isinstance(value, dict):
        errors.append(f"{label} must be an object")
        return None
    values = {
        key: _finite_number(value.get(key), f"{label}.{key}", errors)
        for key in ("x", "y", "z", "yaw")
    }
    if any(number is None for number in values.values()):
        return None
    return values["x"], values["y"], values["z"]


def load_route(route_path: Path) -> tuple[str, list[tuple[str, float, float, float]]]:
    errors: list[str] = []
    try:
        with route_path.open(encoding="utf-8") as stream:
            route_data = json.load(stream)
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise ValidationError([f"cannot read route JSON {route_path}: {error}"]) from error

    if not isinstance(route_data, dict):
        raise ValidationError(["route JSON root must be an object"])

    town = route_data.get("town")
    if not isinstance(town, str) or not town.strip():
        errors.append("route town metadata must be a non-empty string")
        town = ""
    else:
        town = town.strip()

    points: list[tuple[str, float, float, float]] = []
    start = _route_pose(route_data.get("start_ros_pose"), "start_ros_pose", errors)
    if start is not None:
        points.append(("start_ros_pose", *start))
    goal = _route_pose(route_data.get("goal_ros_pose"), "goal_ros_pose", errors)
    if goal is not None:
        points.append(("goal_ros_pose", *goal))

    route = route_data.get("route")
    route_poses: list[tuple[float, float, float]] = []
    if not isinstance(route, list) or not route:
        errors.append("route must be a non-empty array")
    else:
        for index, point in enumerate(route):
            pose = _route_pose(point, f"route[{index}]", errors)
            if pose is not None:
                route_poses.append(pose)
                points.append((f"route[{index}]", *pose))

    if goal is not None and route_poses:
        terminal_gap = math.hypot(
            goal[0] - route_poses[-1][0], goal[1] - route_poses[-1][1]
        )
        if terminal_gap > 1.0e-3:
            errors.append(
                "route terminal must match goal_ros_pose; "
                f"planar gap is {terminal_gap:.6f} m"
            )

    for key in ("sampling_resolution_m", "route_length_m"):
        if key in route_data:
            value = _finite_number(route_data[key], key, errors)
            if value is not None and value < 0.0:
                errors.append(f"{key} must not be negative")

    if errors:
        raise ValidationError(errors)
    return town, points


def validate_projector(projector_path: Path) -> str:
    try:
        lines = projector_path.read_text(encoding="utf-8").splitlines()
    except (OSError, UnicodeError) as error:
        raise ValidationError([f"cannot read projector YAML {projector_path}: {error}"]) from error

    projector_values = []
    for raw_line in lines:
        line = raw_line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        if key.strip() == "projector_type":
            projector_values.append(value.strip().strip("'\""))

    if not projector_values:
        raise ValidationError(["map_projector_info.yaml has no projector_type"])
    if len(set(projector_values)) != 1:
        raise ValidationError([f"projector_type is defined inconsistently: {projector_values}"])
    projector_type = projector_values[0]
    if projector_type != "Local":
        raise ValidationError(
            [f"projector_type must be 'Local' for CARLA ROS coordinates, got {projector_type!r}"]
        )
    return projector_type


def validate_pcd_header(pcd_path: Path) -> dict[str, object]:
    header: dict[str, str] = {}
    data_offset = 0
    errors: list[str] = []
    try:
        with pcd_path.open("rb") as stream:
            while stream.tell() < 65_536:
                raw_line = stream.readline()
                if not raw_line:
                    break
                try:
                    line = raw_line.decode("ascii").strip()
                except UnicodeDecodeError as error:
                    raise ValidationError(
                        ["PCD binary data begins before a valid DATA header"]
                    ) from error
                if not line or line.startswith("#"):
                    continue
                parts = line.split(None, 1)
                key = parts[0].upper()
                value = parts[1].strip() if len(parts) == 2 else ""
                if key in header:
                    errors.append(f"PCD header contains duplicate {key}")
                header[key] = value
                if key == "DATA":
                    data_offset = stream.tell()
                    break
    except OSError as error:
        raise ValidationError([f"cannot read PCD {pcd_path}: {error}"]) from error

    required = {"VERSION", "FIELDS", "SIZE", "TYPE", "COUNT", "WIDTH", "HEIGHT", "POINTS", "DATA"}
    missing = sorted(required - header.keys())
    if missing:
        errors.append(f"PCD header is missing: {', '.join(missing)}")
        raise ValidationError(errors)

    fields = header["FIELDS"].split()
    sizes_text = header["SIZE"].split()
    types = header["TYPE"].split()
    counts_text = header["COUNT"].split()
    if not fields or len(set(fields)) != len(fields):
        errors.append("PCD FIELDS must be non-empty and unique")
    if not {"x", "y", "z"}.issubset(fields):
        errors.append(f"PCD FIELDS must contain x y z, got {fields}")
    if not (len(fields) == len(sizes_text) == len(types) == len(counts_text)):
        errors.append("PCD FIELDS/SIZE/TYPE/COUNT column counts disagree")

    try:
        sizes = [int(value) for value in sizes_text]
        counts = [int(value) for value in counts_text]
        width = int(header["WIDTH"])
        height = int(header["HEIGHT"])
        point_count = int(header["POINTS"])
    except ValueError as error:
        errors.append(f"PCD numeric header value is invalid: {error}")
        sizes = counts = []
        width = height = point_count = 0

    if sizes and any(value <= 0 for value in sizes):
        errors.append("PCD SIZE values must be positive")
    if counts and any(value <= 0 for value in counts):
        errors.append("PCD COUNT values must be positive")
    if types and any(value not in {"F", "I", "U"} for value in types):
        errors.append(f"PCD TYPE values must be F, I, or U, got {types}")
    if width <= 0 or height <= 0 or point_count <= 0:
        errors.append("PCD WIDTH, HEIGHT, and POINTS must be positive")
    if width > 0 and height > 0 and point_count != width * height:
        errors.append(
            f"PCD POINTS ({point_count}) does not equal WIDTH*HEIGHT ({width * height})"
        )

    encoding = header["DATA"].lower()
    if encoding not in {"ascii", "binary", "binary_compressed"}:
        errors.append(f"unsupported PCD DATA encoding {header['DATA']!r}")

    try:
        file_size = pcd_path.stat().st_size
    except OSError as error:
        errors.append(f"cannot stat PCD: {error}")
        file_size = 0
    if not data_offset or file_size <= data_offset:
        errors.append("PCD has no point data after its header")

    point_stride = sum(size * count for size, count in zip(sizes, counts))
    if encoding == "binary" and point_stride and point_count > 0:
        required_size = data_offset + point_stride * point_count
        if file_size < required_size:
            errors.append(
                f"binary PCD is truncated: {file_size} bytes, expected at least {required_size}"
            )
    elif encoding == "binary_compressed" and file_size >= data_offset + 8:
        try:
            with pcd_path.open("rb") as stream:
                stream.seek(data_offset)
                compressed_size, uncompressed_size = struct.unpack("<II", stream.read(8))
        except (OSError, struct.error) as error:
            errors.append(f"cannot read compressed PCD size metadata: {error}")
        else:
            expected_uncompressed = point_stride * point_count
            if compressed_size <= 0:
                errors.append("compressed PCD payload size must be positive")
            if uncompressed_size != expected_uncompressed:
                errors.append(
                    "compressed PCD uncompressed size does not match header: "
                    f"{uncompressed_size} != {expected_uncompressed}"
                )
            if file_size < data_offset + 8 + compressed_size:
                errors.append("binary_compressed PCD payload is truncated")

    if errors:
        raise ValidationError(errors)
    return {
        "fields": fields,
        "points": point_count,
        "encoding": encoding,
        "header_bytes": data_offset,
    }


def _distance(first: tuple[float, float], second: tuple[float, float]) -> float:
    return math.hypot(first[0] - second[0], first[1] - second[1])


def _deduplicate(points: list[tuple[float, ...]]) -> list[tuple[float, ...]]:
    result = []
    for point in points:
        if not result or point != result[-1]:
            result.append(point)
    if len(result) > 1 and result[0] == result[-1]:
        result.pop()
    return result


def load_lanelet_polygons(osm_path: Path) -> tuple[list[LaneletPolygon], int, int]:
    try:
        root = ET.parse(osm_path).getroot()
    except (OSError, ET.ParseError) as error:
        raise ValidationError([f"cannot parse Lanelet2 OSM {osm_path}: {error}"]) from error

    errors: list[str] = []
    nodes: dict[str, tuple[float, float, float]] = {}
    node_count = 0
    for element in _children(root, "node"):
        node_count += 1
        node_id = element.attrib.get("id")
        tags = _element_tags(element)
        if not node_id:
            errors.append("OSM contains a node without an id")
            continue
        if "local_x" not in tags or "local_y" not in tags or "ele" not in tags:
            continue
        try:
            point = float(tags["local_x"]), float(tags["local_y"]), float(tags["ele"])
        except ValueError:
            errors.append(f"OSM node {node_id} has invalid local_x/local_y/ele")
            continue
        if not all(math.isfinite(value) for value in point):
            errors.append(f"OSM node {node_id} has non-finite local_x/local_y/ele")
            continue
        if node_id in nodes:
            errors.append(f"OSM contains duplicate node id {node_id}")
        nodes[node_id] = point

    ways: dict[str, list[str]] = {}
    for element in _children(root, "way"):
        way_id = element.attrib.get("id")
        if not way_id:
            errors.append("OSM contains a way without an id")
            continue
        if way_id in ways:
            errors.append(f"OSM contains duplicate way id {way_id}")
        ways[way_id] = [child.attrib.get("ref", "") for child in _children(element, "nd")]

    polygons: list[LaneletPolygon] = []
    lanelet_count = 0
    road_lanelet_count = 0
    for relation in _children(root, "relation"):
        tags = _element_tags(relation)
        if tags.get("type") != "lanelet":
            continue
        lanelet_count += 1
        if tags.get("subtype") != "road":
            continue
        road_lanelet_count += 1
        lanelet_id = relation.attrib.get("id", "<unknown>")
        boundaries: dict[str, str] = {}
        for member in _children(relation, "member"):
            role = member.attrib.get("role")
            if member.attrib.get("type") == "way" and role in {"left", "right"}:
                boundaries[role] = member.attrib.get("ref", "")
        if not all(role in boundaries for role in ("left", "right")):
            errors.append(f"lanelet {lanelet_id} has no complete left/right boundary")
            continue

        boundary_points: dict[str, list[tuple[float, float, float]]] = {}
        invalid = False
        for role in ("left", "right"):
            way_id = boundaries[role]
            if way_id not in ways:
                errors.append(f"lanelet {lanelet_id} {role} way {way_id} does not exist")
                invalid = True
                continue
            refs = ways[way_id]
            missing = [ref for ref in refs if ref not in nodes]
            if missing:
                preview = ", ".join(missing[:3])
                errors.append(
                    f"lanelet {lanelet_id} {role} way {way_id} references nodes without "
                    f"finite local_x/local_y/ele: {preview}"
                )
                invalid = True
                continue
            boundary_points[role] = _deduplicate([nodes[ref] for ref in refs])
            if len(boundary_points[role]) < 2:
                errors.append(f"lanelet {lanelet_id} {role} boundary has fewer than two points")
                invalid = True
        if invalid:
            continue

        left = boundary_points["left"]
        right = boundary_points["right"]
        same_direction_cost = _distance(left[0], right[0]) + _distance(left[-1], right[-1])
        opposite_direction_cost = _distance(left[0], right[-1]) + _distance(left[-1], right[0])
        if same_direction_cost <= opposite_direction_cost:
            polygon = left + list(reversed(right))
        else:
            polygon = left + right
        polygon = _deduplicate(polygon)
        if len(polygon) < 3:
            errors.append(f"lanelet {lanelet_id} does not form a polygon")
            continue
        twice_area = abs(
            sum(
                polygon[index - 1][0] * polygon[index][1]
                - polygon[index][0] * polygon[index - 1][1]
                for index in range(len(polygon))
            )
        )
        if twice_area <= 1.0e-9:
            errors.append(f"lanelet {lanelet_id} forms a zero-area polygon")
            continue
        xs = [point[0] for point in polygon]
        ys = [point[1] for point in polygon]
        bounds = min(xs), min(ys), max(xs), max(ys)
        polygons.append(
            LaneletPolygon(lanelet_id, tags.get("subtype", ""), tuple(polygon), bounds)
        )

    if node_count == 0 or not nodes:
        errors.append("OSM has no nodes with finite local_x/local_y")
    elif len(nodes) != node_count:
        errors.append(
            "every OSM node must have finite local_x/local_y/ele for a Local map "
            f"({len(nodes)}/{node_count})"
        )
    if lanelet_count == 0:
        errors.append("OSM has no type=lanelet relations")
    elif road_lanelet_count == 0:
        errors.append("OSM has no subtype=road lanelet relations")
    if not polygons:
        errors.append("OSM has no usable road-lanelet polygons")
    if errors:
        raise ValidationError(errors)
    return polygons, node_count, len(nodes)


def _point_segment_distance(
    point: tuple[float, float], start: tuple[float, ...], end: tuple[float, ...]
) -> float:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length_squared = dx * dx + dy * dy
    if length_squared == 0.0:
        return _distance(point, start)
    scale = ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length_squared
    scale = max(0.0, min(1.0, scale))
    closest = start[0] + scale * dx, start[1] + scale * dy
    return _distance(point, closest)


def _point_in_polygon(
    point: tuple[float, float], polygon: tuple[tuple[float, ...], ...]
) -> bool:
    inside = False
    previous = polygon[-1]
    for current in polygon:
        if (current[1] > point[1]) != (previous[1] > point[1]):
            crossing_x = (previous[0] - current[0]) * (point[1] - current[1]) / (
                previous[1] - current[1]
            ) + current[0]
            if point[0] < crossing_x:
                inside = not inside
        previous = current
    return inside


def _polygon_distance(point: tuple[float, float], polygon: LaneletPolygon) -> float:
    if _point_in_polygon(point, polygon.points):
        return 0.0
    return min(
        _point_segment_distance(point, polygon.points[index - 1], polygon.points[index])
        for index in range(len(polygon.points))
    )


def _segment_elevation(
    point: tuple[float, float], start: tuple[float, ...], end: tuple[float, ...]
) -> tuple[float, float]:
    """Return planar distance and linearly interpolated elevation on a 3D segment."""
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length_squared = dx * dx + dy * dy
    if length_squared == 0.0:
        return _distance(point, start), start[2]
    scale = ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length_squared
    scale = max(0.0, min(1.0, scale))
    closest = start[0] + scale * dx, start[1] + scale * dy
    elevation = start[2] + scale * (end[2] - start[2])
    return _distance(point, closest), elevation


def _polygon_elevation(point: tuple[float, float], polygon: LaneletPolygon) -> float:
    candidates = [
        _segment_elevation(point, polygon.points[index - 1], polygon.points[index])
        for index in range(len(polygon.points))
    ]
    return min(candidates, key=lambda candidate: candidate[0])[1]


def _canonical_town(value: str) -> str:
    return "".join(character.lower() for character in value if character.isalnum())


def validate_town_paths(town: str, map_path: Path, map_files: list[Path]) -> list[str]:
    route_town = _canonical_town(town)
    bundle_path = map_path / "map_bundle.json"
    if bundle_path.is_file():
        try:
            bundle = json.loads(bundle_path.read_text(encoding="utf-8"))
        except (OSError, UnicodeError, json.JSONDecodeError) as error:
            raise ValidationError([f"cannot read map identity from {bundle_path}: {error}"]) from error
        canonical_map = bundle.get("canonical_carla_map") if isinstance(bundle, dict) else None
        if not isinstance(canonical_map, str) or not canonical_map.strip():
            raise ValidationError(
                [f"{bundle_path} has no non-empty canonical_carla_map identity"]
            )
        bundle_town = canonical_map.rstrip("/").rsplit("/", 1)[-1]
        if _canonical_town(bundle_town) != route_town:
            raise ValidationError(
                [
                    f"route town {town!r} does not match map bundle identity "
                    f"{bundle_town!r}"
                ]
            )
        return []

    candidates: dict[str, set[str]] = {}
    inspected = [map_path, *map_files]
    for path in inspected:
        variants = {str(path)}
        try:
            variants.add(str(path.resolve()))
        except OSError:
            pass
        for variant in variants:
            for match in TOWN_PATTERN.findall(variant):
                candidates.setdefault(_canonical_town(match), set()).add(match)

    if not candidates:
        return [
            f"could not infer a Town name from map path {map_path}; "
            "route geometry was validated but map identity is unconfirmed"
        ]
    if set(candidates) != {route_town}:
        names = sorted({name for values in candidates.values() for name in values})
        raise ValidationError(
            [f"route town {town!r} does not uniquely match map path/source names {names}"]
        )
    return []


def validate_route_points(
    route_points: list[tuple[str, float, float, float]],
    polygons: list[LaneletPolygon],
    tolerance_m: float,
    vertical_tolerance_m: float,
) -> tuple[float, float, int]:
    errors: list[str] = []
    maximum_distance = 0.0
    maximum_vertical_distance = 0.0
    matched_lanelets: set[str] = set()
    for label, x, y, z in route_points:
        point = x, y
        best_distance = math.inf
        best_vertical_distance = math.inf
        best_lanelet = "<none>"
        for polygon in polygons:
            min_x, min_y, max_x, max_y = polygon.bounds
            if x < min_x - tolerance_m or x > max_x + tolerance_m:
                continue
            if y < min_y - tolerance_m or y > max_y + tolerance_m:
                continue
            distance = _polygon_distance(point, polygon)
            vertical_distance = abs(z - _polygon_elevation(point, polygon))
            if (distance, vertical_distance) < (best_distance, best_vertical_distance):
                best_distance = distance
                best_vertical_distance = vertical_distance
                best_lanelet = polygon.lanelet_id
        if best_distance > tolerance_m:
            # The expanded bounding-box filter may exclude every lanelet. Compute a
            # precise nearest value only for the failing diagnostic path.
            for polygon in polygons:
                distance = _polygon_distance(point, polygon)
                vertical_distance = abs(z - _polygon_elevation(point, polygon))
                if (distance, vertical_distance) < (best_distance, best_vertical_distance):
                    best_distance = distance
                    best_vertical_distance = vertical_distance
                    best_lanelet = polygon.lanelet_id
            errors.append(
                f"{label} ({x:.3f}, {y:.3f}) is outside every lanelet by "
                f"{best_distance:.3f} m (nearest lanelet {best_lanelet}, "
                f"tolerance {tolerance_m:.3f} m)"
            )
        else:
            matched_lanelets.add(best_lanelet)
            maximum_distance = max(maximum_distance, best_distance)
            maximum_vertical_distance = max(maximum_vertical_distance, best_vertical_distance)
            if best_vertical_distance > vertical_tolerance_m:
                errors.append(
                    f"{label} ({x:.3f}, {y:.3f}, {z:.3f}) differs from lanelet "
                    f"{best_lanelet} elevation by {best_vertical_distance:.3f} m "
                    f"(vertical tolerance {vertical_tolerance_m:.3f} m)"
                )
    if errors:
        raise ValidationError(errors)
    return maximum_distance, maximum_vertical_distance, len(matched_lanelets)


def validate_route_map(
    route_path: Path,
    map_path: Path,
    tolerance_m: float = 0.75,
    vertical_tolerance_m: float = 5.0,
) -> dict:
    route_path = Path(route_path)
    map_path = Path(map_path)
    if tolerance_m < 0.0 or not math.isfinite(tolerance_m):
        raise ValidationError(["lanelet tolerance must be a finite non-negative number"])
    if vertical_tolerance_m < 0.0 or not math.isfinite(vertical_tolerance_m):
        raise ValidationError(["vertical tolerance must be a finite non-negative number"])
    if not route_path.is_file():
        raise ValidationError([f"route file does not exist: {route_path}"])
    if not map_path.is_dir():
        raise ValidationError([f"map directory does not exist: {map_path}"])

    map_files = [map_path / name for name in REQUIRED_MAP_FILES]
    missing = [str(path) for path in map_files if not path.is_file()]
    if missing:
        raise ValidationError([f"required map file does not exist: {path}" for path in missing])
    osm_path, pcd_path, projector_path = map_files

    town, route_points = load_route(route_path)
    warnings = validate_town_paths(town, map_path, map_files)
    projector_type = validate_projector(projector_path)
    pcd = validate_pcd_header(pcd_path)
    polygons, osm_node_count, local_node_count = load_lanelet_polygons(osm_path)
    maximum_distance, maximum_vertical_distance, matched_lanelets = validate_route_points(
        route_points, polygons, tolerance_m, vertical_tolerance_m
    )
    return {
        "status": "PASS",
        "town": town,
        "route_file": str(route_path.resolve()),
        "map_path": str(map_path.resolve()),
        "projector_type": projector_type,
        "route_pose_count": len(route_points),
        "lanelet_count": len(polygons),
        "matched_lanelet_count": matched_lanelets,
        "maximum_lanelet_distance_m": maximum_distance,
        "maximum_lanelet_vertical_distance_m": maximum_vertical_distance,
        "tolerance_m": tolerance_m,
        "vertical_tolerance_m": vertical_tolerance_m,
        "osm_node_count": osm_node_count,
        "osm_local_node_count": local_node_count,
        "pcd": pcd,
        "warnings": warnings,
    }


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description=(
            "Check route JSON poses against Local Lanelet2 polygons and validate the "
            "Autoware map bundle before launching CARLA/VAD."
        )
    )
    parser.add_argument("route_json", type=Path)
    parser.add_argument("map_path", type=Path)
    parser.add_argument(
        "--tolerance-m",
        type=float,
        default=0.75,
        help="maximum distance outside a lanelet polygon (default: 0.75)",
    )
    parser.add_argument(
        "--vertical-tolerance-m",
        type=float,
        default=5.0,
        help="maximum route-to-lanelet elevation difference (default: 5.0)",
    )
    parser.add_argument("--json", action="store_true", help="emit the result as JSON")
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    try:
        result = validate_route_map(
            args.route_json,
            args.map_path,
            args.tolerance_m,
            args.vertical_tolerance_m,
        )
    except ValidationError as error:
        if args.json:
            print(json.dumps({"status": "FAIL", "errors": error.errors}, indent=2))
        else:
            print("Route/map preflight: FAIL", file=sys.stderr)
            for message in error.errors:
                print(f"  - {message}", file=sys.stderr)
        return 1

    if args.json:
        print(json.dumps(result, indent=2))
    else:
        print(
            "Route/map preflight: PASS "
            f"(town={result['town']}, route_poses={result['route_pose_count']}, "
            f"lanelets={result['lanelet_count']}, pcd_points={result['pcd']['points']})"
        )
        print(
            "  Lanelet match: "
            f"{result['matched_lanelet_count']} lanelets, "
            f"maximum outside distance={result['maximum_lanelet_distance_m']:.3f} m, "
            "maximum vertical difference="
            f"{result['maximum_lanelet_vertical_distance_m']:.3f} m"
        )
        for warning in result["warnings"]:
            print(f"  WARNING: {warning}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
