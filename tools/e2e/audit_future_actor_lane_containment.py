#!/usr/bin/env python3
"""HH_260906 - Audit a future CARLA actor's sampled body against its recorded intended lane."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any
import xml.etree.ElementTree as ET

from shapely.geometry import MultiPoint, Polygon, box as geometry_box
from shapely.ops import unary_union


# HH_260906 - These guards make a sampled inner corridor stricter, not an exact OpenDRIVE boundary proof.
LANE_SAMPLE_M = 0.05
LANE_INSET_M = 0.10
SEAM_GAP_CLOSE_M = 0.005
XODR_SEARCH_COARSE_M = 0.25
MAX_ROUTE_POINT_CENTER_ERROR_M = 0.08
SWEEP_POSITION_GUARD_M = 0.02
MAX_ROUTE_DISTANCE_M = 4.0
MAX_FRAME_TRANSLATION_M = 1.5
MAX_FRAME_ANGLE_DEG = 5.0
MAX_FRAME_PROGRESS_M = 2.0


class Unsupported(Exception):
    """HH_260906 - Required provenance or geometric correspondence cannot be established."""


def _require(condition: bool, reason: str) -> None:
    if not condition:
        raise Unsupported(reason)


def _finite_number(value: Any, name: str) -> float:
    _require(type(value) in (float, int) and math.isfinite(value), f"invalid {name}")
    return float(value)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _safe_episode_file(episode: Path, value: Any, expected: str) -> Path:
    _require(value == expected, f"unexpected episode file name for {expected}")
    path = episode / expected
    _require(path.is_file() and not path.is_symlink(), f"missing or symlinked {expected}")
    return path


def _triple(point: dict[str, Any]) -> tuple[int, int, int]:
    values = tuple(point.get(key) for key in ("road_id", "section_id", "lane_id"))
    _require(all(type(value) is int for value in values), "route lane IDs must be explicit integers")
    _require(values[2] != 0, "route cannot use the OpenDRIVE center lane")
    return values


def verify_sources(episode: Path) -> tuple[dict[str, Any], dict[str, Any], list[dict[str, Any]], bytes, dict[str, str]]:
    """HH_260906 - Bind the exact archived route, map, actor, states and collector implementation."""
    manifest_path = _safe_episode_file(episode, "manifest.json", "manifest.json")
    manifest = json.loads(manifest_path.read_text())
    _require(manifest.get("status") == "complete", "episode is not complete")
    files = manifest.get("files", {})
    route_path = _safe_episode_file(episode, files.get("route"), "route.json")
    states_path = _safe_episode_file(episode, files.get("states"), "states.jsonl")
    provenance = manifest.get("provenance", {})
    _require(_sha256(route_path) == provenance.get("route_sha256"), "archived route SHA256 mismatch")
    collector_path = Path(__file__).with_name("collect_carla_vad_expert.py")
    _require(collector_path.is_file() and _sha256(collector_path) == provenance.get("collector_sha256"),
             "collector source SHA256 mismatch; audit this version explicitly before use")
    contract = manifest.get("capture_contract", {})
    _require(_finite_number(contract.get("physics_hz"), "physics_hz") == 20.0,
             "this gate requires a 20 Hz capture")
    _require(contract.get("control_transport", {}).get("mode") == "acknowledged_batch",
             "tick-bound actor snapshots are required")
    body = contract.get("actor_body_geometry", {})
    _require(body.get("status") == "RECORDED" and body.get("source") == "carla.Vehicle.bounding_box" and
             body.get("reference_frame") == "CARLA actor-local; not ROS rear-axle base_link" and
             body.get("lane_body_containment_verified") is False, "actor-local bounding-box provenance unavailable")
    actor_id = body.get("actor_id")
    _require(type(actor_id) is int and actor_id > 0, "invalid recorded actor ID")
    construction = contract.get("agent_initialization", {}).get("construction", {})
    _require(construction.get("status") == "PASS" and
             all(construction.get(side, {}).get("actor_id") == actor_id for side in ("before", "after")),
             "body actor ID is not tied to the initialized snapshot actor")
    map_record = contract.get("active_map_opendrive", {})
    _require(map_record.get("status") == "RECORDED" and map_record.get("source") == "carla.Map.to_opendrive" and
             map_record.get("lane_body_containment_verified") is False,
             "active-map OpenDRIVE provenance unavailable")
    map_path = _safe_episode_file(episode, map_record.get("file"), "active_map_opendrive.xodr")
    _require(files.get("active_map_opendrive") == map_record.get("file"), "active-map file reference mismatch")
    map_bytes = map_path.read_bytes()
    _require(hashlib.sha256(map_bytes).hexdigest() == map_record.get("sha256") and
             len(map_bytes) == map_record.get("size_bytes"), "active-map OpenDRIVE SHA256 or size mismatch")
    route = json.loads(route_path.read_text())
    _require(route.get("coordinate_reference") == "base_link",
             "unsupported route coordinate reference")
    points = route.get("route")
    _require(isinstance(points, list) and len(points) >= 2, "missing route waypoints")
    for index, point in enumerate(points):
        _triple(point)
        for name in ("x", "y", "z", "distance_m"):
            _finite_number(point.get(name), f"route[{index}].{name}")
        if index:
            before = points[index - 1]
            _require(point["distance_m"] >= before["distance_m"], "route arc distance goes backwards")
            if point["distance_m"] == before["distance_m"]:
                # HH_260906 - CARLA may repeat one physical waypoint when its maneuver command changes.
                _require(_triple(point) == _triple(before) and
                         all(point[name] == before[name] for name in ("x", "y", "z")),
                         "zero-length route node changes geometric lane correspondence")
    states = []
    with states_path.open() as stream:
        for line in stream:
            states.append(json.loads(line))
    _require(states, "empty states journal")
    hashes = {name: _sha256(path) for name, path in
              (("manifest", manifest_path), ("route", route_path), ("states", states_path),
               ("active_map_opendrive", map_path), ("collector", collector_path))}
    return manifest, route, states, map_bytes, hashes


def _route_projection(point: tuple[float, float], route: list[dict[str, Any]]) -> tuple[float, int, float]:
    """HH_260906 - Match raw CARLA actor location, never the collector's route_progress field."""
    px, py = point
    candidates: list[tuple[float, float, int]] = []
    for index, (a, b) in enumerate(zip(route, route[1:])):
        ax, ay = float(a["x"]), -float(a["y"])
        bx, by = float(b["x"]), -float(b["y"])
        dx, dy = bx - ax, by - ay
        length2 = dx * dx + dy * dy
        _require(length2 > 1e-8, "route has a degenerate segment")
        t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length2))
        distance = math.hypot(px - (ax + t * dx), py - (ay + t * dy))
        arc = float(a["distance_m"]) + t * (float(b["distance_m"]) - float(a["distance_m"]))
        candidates.append((distance, arc, index))
    candidates.sort()
    distance, arc, index = candidates[0]
    _require(distance <= MAX_ROUTE_DISTANCE_M, "actor cannot be matched to the intended route")
    _require(not any(d <= distance + 0.25 and abs(other_arc - arc) > 4.0
                     for d, other_arc, _ in candidates[1:]), "ambiguous route projection at a crossing")
    return arc, index, distance


def _body_box(manifest: dict[str, Any], carla: Any) -> tuple[Any, float]:
    record = manifest["capture_contract"]["actor_body_geometry"]["bounding_box"]
    location, extent, rotation = (record.get(key, {}) for key in ("location", "extent", "rotation"))
    for name in ("x", "y", "z"):
        _finite_number(location.get(name), f"box.location.{name}")
        _require(_finite_number(extent.get(name), f"box.extent.{name}") > 0.0,
                 "body-box extents must be positive")
    for name in ("roll", "pitch", "yaw"):
        _finite_number(rotation.get(name), f"box.rotation.{name}")
    box = carla.BoundingBox(
        carla.Location(**{name: float(location[name]) for name in ("x", "y", "z")}),
        carla.Vector3D(**{name: float(extent[name]) for name in ("x", "y", "z")}))
    box.rotation = carla.Rotation(**{name: float(rotation[name]) for name in ("roll", "pitch", "yaw")})
    radius = math.sqrt(sum((abs(location[name]) + extent[name]) ** 2 for name in ("x", "y", "z")))
    _require(radius < 20.0, "implausibly large actor-local box")
    return box, radius


def body_footprint(box: Any, pose: dict[str, Any], carla: Any) -> Polygon:
    """HH_260906 - Project all eight CARLA-transformed 3-D box vertices, including roll and pitch."""
    for name in ("x", "y", "z", "roll", "pitch", "yaw"):
        _finite_number(pose.get(name), f"actor_snapshot_transform_carla.{name}")
    transform = carla.Transform(
        carla.Location(**{name: float(pose[name]) for name in ("x", "y", "z")}),
        carla.Rotation(**{name: float(pose[name]) for name in ("roll", "pitch", "yaw")}))
    vertices = box.get_world_vertices(transform)
    _require(len(vertices) == 8, "CARLA did not return eight body-box vertices")
    footprint = MultiPoint([(float(vertex.x), float(vertex.y)) for vertex in vertices]).convex_hull
    _require(isinstance(footprint, Polygon) and footprint.is_valid and footprint.area > 0.01,
             "actor body projection is not a valid polygon")
    return footprint


def _angle_delta_degrees(a: float, b: float) -> float:
    return abs((b - a + 180.0) % 360.0 - 180.0)


def swept_body(p0: Polygon, p1: Polygon, pose0: dict[str, Any], pose1: dict[str, Any],
               body_radius_m: float) -> Polygon:
    """HH_260906 - Enclose endpoint hull plus a rigid-rotation bound for snapshot-linear interpolation."""
    translation = math.hypot(float(p1.centroid.x - p0.centroid.x), float(p1.centroid.y - p0.centroid.y))
    _require(translation <= MAX_FRAME_TRANSLATION_M, "interframe translation exceeds bounded interpolation")
    angle = sum(_angle_delta_degrees(float(pose0[name]), float(pose1[name]))
                for name in ("roll", "pitch", "yaw"))
    _require(angle <= MAX_FRAME_ANGLE_DEG, "interframe rotation exceeds bounded interpolation")
    # HH_260906 - The angular term upper-bounds motion of any box vertex under short Euler interpolation.
    slack = body_radius_m * math.radians(angle) + SWEEP_POSITION_GUARD_M
    envelope = unary_union((p0, p1)).convex_hull.buffer(slack)
    _require(isinstance(envelope, Polygon) and envelope.is_valid, "invalid swept body envelope")
    return envelope


def _section_bounds(xml: bytes) -> dict[tuple[int, int], tuple[float, float]]:
    root = ET.fromstring(xml)
    _require(root.tag == "OpenDRIVE", "active-map payload is not OpenDRIVE")
    bounds: dict[tuple[int, int], tuple[float, float]] = {}
    for road in root.findall("road"):
        road_id = int(road.attrib["id"])
        length = float(road.attrib["length"])
        sections = road.findall("./lanes/laneSection")
        for index, section in enumerate(sections):
            start = float(section.attrib["s"])
            end = float(sections[index + 1].attrib["s"]) if index + 1 < len(sections) else length
            _require(math.isfinite(start) and math.isfinite(end) and end > start,
                     "invalid OpenDRIVE lane-section extent")
            bounds[(road_id, index)] = (start, end)
    return bounds


def lane_corridors(carla_map: Any, carla: Any, route: list[dict[str, Any]], xml: bytes,
                   requested: set[tuple[int, int, int]],
                   relevant_indices: set[int]) -> dict[tuple[int, int, int], Any]:
    """HH_260906 - Resolve route IDs through XODR, then tessellate only their inner portions."""
    bounds = _section_bounds(xml)
    s_ranges: dict[tuple[int, int, int], list[float]] = {triple: [] for triple in requested}
    search_banks = {}
    for triple in sorted(requested):
        _require(triple[:2] in bounds, f"missing OpenDRIVE lane-section {triple}")
        lo, hi = bounds[triple[:2]]
        lo, hi = lo + 0.001, hi - 0.001
        count = max(2, math.ceil((hi - lo) / XODR_SEARCH_COARSE_M) + 1)
        bank = []
        for index in range(count):
            s = lo + (hi - lo) * index / (count - 1)
            waypoint = carla_map.get_waypoint_xodr(triple[0], triple[2], s)
            if waypoint is None:
                continue
            _require((waypoint.road_id, waypoint.section_id, waypoint.lane_id) == triple,
                     f"XODR search returned a different lane than {triple}")
            center = waypoint.transform.location
            bank.append((s, float(center.x), float(center.y)))
        _require(len(bank) >= 2, f"no XODR centerline support for {triple}")
        search_banks[triple] = bank

    for index, point in enumerate(route):
        if index not in relevant_indices:
            continue
        triple = _triple(point)
        if triple not in requested:
            continue
        x, y = float(point["x"]), -float(point["y"])
        bank = search_banks[triple]
        ranked = sorted(((math.hypot(x - cx, y - cy), s) for s, cx, cy in bank))
        _, coarse_s = ranked[0]
        _require(not any(distance <= MAX_ROUTE_POINT_CENTER_ERROR_M and abs(s - coarse_s) > 1.0
                         for distance, s in ranked[1:]),
                 f"same-ID OpenDRIVE centerline is ambiguous at route point {triple}")
        section_lo, section_hi = bounds[triple[:2]]
        left = max(section_lo + 0.001, coarse_s - XODR_SEARCH_COARSE_M)
        right = min(section_hi - 0.001, coarse_s + XODR_SEARCH_COARSE_M)

        def distance_sq(s: float) -> float:
            candidate = carla_map.get_waypoint_xodr(triple[0], triple[2], s)
            if candidate is None:
                return math.inf
            _require((candidate.road_id, candidate.section_id, candidate.lane_id) == triple,
                     f"XODR refinement returned a different lane than {triple}")
            center = candidate.transform.location
            return (float(center.x) - x) ** 2 + (float(center.y) - y) ** 2

        # HH_260906 - A bounded same-ID search avoids get_waypoint's arbitrary choice on overlapping junction roads.
        for _ in range(28):
            a = left + (right - left) * 0.3819660112501051
            b = left + (right - left) * 0.6180339887498949
            if distance_sq(a) <= distance_sq(b):
                right = b
            else:
                left = a
        resolved_s = (left + right) / 2.0
        candidates = [(distance_sq(resolved_s), resolved_s),
                      (distance_sq(section_lo + 0.001), section_lo + 0.001),
                      (distance_sq(section_hi - 0.001), section_hi - 0.001)]
        error2, resolved_s = min(candidates)
        _require(math.isfinite(error2) and math.sqrt(error2) <= MAX_ROUTE_POINT_CENTER_ERROR_M,
                 f"recorded route point cannot be bound to its own XODR lane {triple}: "
                 f"nearest center {math.sqrt(error2):.3f}m")
        s_ranges[triple].append(resolved_s)
    corridors = {}
    for triple in sorted(requested):
        _require(bool(s_ranges[triple]) and triple[:2] in bounds, f"unresolved route lane {triple}")
        section_lo, section_hi = bounds[triple[:2]]
        start = max(section_lo + 0.001, min(s_ranges[triple]) - 6.0)
        end = min(section_hi - 0.001, max(s_ranges[triple]) + 6.0)
        _require(end - start >= 0.1, f"insufficient OpenDRIVE lane support for {triple}")
        count = max(2, math.ceil((end - start) / LANE_SAMPLE_M) + 1)
        boundary = []
        for index in range(count):
            s = start + (end - start) * index / (count - 1)
            waypoint = carla_map.get_waypoint_xodr(triple[0], triple[2], s)
            _require(waypoint is not None and
                     (waypoint.road_id, waypoint.section_id, waypoint.lane_id) == triple,
                     f"missing XODR lane sample {triple} at {s:.3f}m")
            center = waypoint.transform.location
            yaw = math.radians(float(waypoint.transform.rotation.yaw))
            half_width = _finite_number(waypoint.lane_width, "lane width") / 2.0
            _require(half_width > LANE_INSET_M, "lane too narrow for an inner corridor")
            nx, ny = -math.sin(yaw) * half_width, math.cos(yaw) * half_width
            boundary.append(((center.x + nx, center.y + ny), (center.x - nx, center.y - ny)))
        quads = []
        for left_right_a, left_right_b in zip(boundary, boundary[1:]):
            left_a, right_a = left_right_a
            left_b, right_b = left_right_b
            quad = Polygon((left_a, right_a, right_b, left_b))
            _require(quad.is_valid and quad.area > 1e-8, f"invalid lane tessellation {triple}")
            quads.append(quad)
        sampled_lane = unary_union(quads)
        _require(not sampled_lane.is_empty and sampled_lane.is_valid,
                 f"empty or invalid sampled lane {triple}")
        corridors[triple] = sampled_lane
    return corridors


def _selected_states(states: list[dict[str, Any]], route: list[dict[str, Any]],
                     start_m: float | None, end_m: float | None) -> list[dict[str, Any]]:
    selected = []
    for state in states:
        if state.get("capture_phase") != "driving":
            continue
        _require(state.get("control_transport", {}).get("status") == "PASS",
                 "driving state lacks a tick-bound acknowledged snapshot")
        pose = state.get("actor_snapshot_transform_carla")
        _require(isinstance(pose, dict), "driving state lacks raw CARLA actor pose")
        arc, segment, distance = _route_projection(
            (_finite_number(pose.get("x"), "actor pose x"),
             _finite_number(pose.get("y"), "actor pose y")), route)
        if (start_m is None or arc >= start_m) and (end_m is None or arc <= end_m):
            selected.append({"state": state, "pose": pose, "arc_m": arc,
                             "route_segment": segment, "route_distance_m": distance})
    _require(len(selected) >= 2, "fewer than two driving frames in the requested route window")
    for before, after in zip(selected, selected[1:]):
        a, b = before["state"], after["state"]
        _require(type(a.get("frame")) is int and type(b.get("frame")) is int and
                 b["frame"] == a["frame"] + 1, "selected frames are not contiguous at 20 Hz")
        dt = _finite_number(b.get("timestamp"), "timestamp") - _finite_number(a.get("timestamp"), "timestamp")
        _require(0.045 <= dt <= 0.055, "selected timestamps are not 20 Hz")
        _require(-0.25 <= after["arc_m"] - before["arc_m"] <= MAX_FRAME_PROGRESS_M,
                 "independent route projection has a jump or rollback")
        _require(0 <= after["route_segment"] - before["route_segment"] <= 3,
                 "route segment progression skips or reverses")
    return selected


def _route_lanes_for_arc_interval(route: list[dict[str, Any]],
                                  lo_m: float, hi_m: float) -> set[tuple[int, int, int]]:
    """HH_260906 - Include only recorded consecutive lanes intersected by the bounded actor body."""
    _require(math.isfinite(lo_m) and math.isfinite(hi_m) and lo_m <= hi_m,
             "invalid body route-arc interval")
    lanes = {_triple(point) for a, b in zip(route, route[1:])
             if a["distance_m"] <= hi_m and b["distance_m"] >= lo_m
             for point in (a, b)}
    _require(bool(lanes), "body route-arc interval has no intended lane")
    return lanes


def authorized_lane_region(corridors: dict[tuple[int, int, int], Any],
                           lanes: set[tuple[int, int, int]], inset_m: float) -> Any:
    """HH_260906 - Close only a sub-5mm sampled seam between already authorized route lanes."""
    raw = unary_union([corridors[triple] for triple in sorted(lanes)])
    # HH_260906 - XODR section endpoint sampling excludes 1mm on either side; never bridge a wider gap.
    if len(lanes) > 1 and raw.geom_type == "MultiPolygon":
        components = list(raw.geoms)
        for index, first in enumerate(components):
            for second in components[index + 1:]:
                _require(first.distance(second) <= SEAM_GAP_CLOSE_M,
                         "authorized route lanes have a gap wider than the 5mm endpoint tolerance")
        raw = raw.buffer(SEAM_GAP_CLOSE_M / 2.0 + 1e-6).buffer(-(SEAM_GAP_CLOSE_M / 2.0 + 1e-6))
    return raw.buffer(-inset_m) if inset_m else raw


def audit_episode(episode: Path, *, start_m: float | None = None,
                  end_m: float | None = None, carla_module: Any = None) -> dict[str, Any]:
    """HH_260906 - Return PASS only for bounded 20 Hz samples and interpolated sweeps."""
    if start_m is not None:
        _require(math.isfinite(start_m) and start_m >= 0.0, "invalid start progress")
    if end_m is not None:
        _require(math.isfinite(end_m) and (start_m is None or end_m > start_m), "invalid end progress")
    manifest, route, states, xml, hashes = verify_sources(episode)
    if carla_module is None:
        try:
            import carla as carla_module
        except ImportError as error:
            raise Unsupported("offline CARLA Python map bindings unavailable") from error
    try:
        carla_map = carla_module.Map("archived-active-map", xml.decode("utf-8"))
    except Exception as error:
        raise Unsupported(f"CARLA cannot load archived OpenDRIVE: {error}") from error
    points = [route["route"][0]]
    for point in route["route"][1:]:
        if point["distance_m"] != points[-1]["distance_m"]:
            points.append(point)
    duplicate_geometry_nodes = len(route["route"]) - len(points)
    selected = _selected_states(states, points, start_m, end_m)
    box, radius = _body_box(manifest, carla_module)
    lo = selected[0]["arc_m"] - radius
    hi = selected[-1]["arc_m"] + radius
    relevant_indices = {node_index for segment_index, (a, b) in enumerate(zip(points, points[1:]))
                        if a["distance_m"] <= hi and b["distance_m"] >= lo
                        for node_index in (segment_index, segment_index + 1)}
    requested = {_triple(points[index]) for index in relevant_indices}
    corridors = lane_corridors(carla_map, carla_module, points, xml, requested, relevant_indices)
    cache: dict[tuple[frozenset[tuple[int, int, int]], float], Any] = {}

    def allowed(lanes: set[tuple[int, int, int]], inset_m: float) -> Any:
        key = frozenset(lanes), inset_m
        if key not in cache:
            # HH_260906 - Erode after union: an intended road-ID seam is not a lane boundary.
            cache[key] = authorized_lane_region(corridors, lanes, inset_m)
        return cache[key]

    footprints = []
    maximum_outside_m2 = 0.0
    maximum_raw_outside_m2 = 0.0
    body_inset_violations = body_raw_violations = 0
    sweep_inset_violations = sweep_raw_violations = 0
    violations: list[dict[str, Any]] = []
    for item in selected:
        lanes = _route_lanes_for_arc_interval(points, item["arc_m"] - radius,
                                             item["arc_m"] + radius)
        footprint = body_footprint(box, item["pose"], carla_module)
        outside = footprint.difference(allowed(lanes, LANE_INSET_M)).area
        raw_outside = footprint.difference(allowed(lanes, 0.0)).area
        _require(raw_outside <= outside + 1e-6,
                 "sampled raw/inset body areas violate numerical containment monotonicity")
        maximum_outside_m2 = max(maximum_outside_m2, outside)
        maximum_raw_outside_m2 = max(maximum_raw_outside_m2, raw_outside)
        body_raw_violations += int(raw_outside > 1e-6)
        if outside > 1e-6:
            body_inset_violations += 1
            detail = {"kind": "sampled_body", "frame": item["state"]["frame"],
                      "independent_progress_m": item["arc_m"], "actor_pose_carla": item["pose"],
                      "selected_intended_lanes": [list(triple) for triple in sorted(lanes)],
                      "raw_sampled_lane_outside_m2": raw_outside,
                      "inset_10cm_lane_outside_m2": outside,
                      "body_polygon_carla_xy": [list(xy) for xy in footprint.exterior.coords],
                      "interpretation": "raw_sampled_lane_intrusion" if raw_outside > 1e-6
                      else "inset_only_certification_failure"}
            violations.append(detail)
        footprints.append(footprint)
    maximum_sweep_outside_m2 = 0.0
    maximum_raw_sweep_outside_m2 = 0.0
    for index, (before, after) in enumerate(zip(selected, selected[1:])):
        lanes = _route_lanes_for_arc_interval(points, before["arc_m"] - radius,
                                             after["arc_m"] + radius)
        envelope = swept_body(footprints[index], footprints[index + 1], before["pose"], after["pose"], radius)
        outside = envelope.difference(allowed(lanes, LANE_INSET_M)).area
        raw_outside = envelope.difference(allowed(lanes, 0.0)).area
        _require(raw_outside <= outside + 1e-6,
                 "sampled raw/inset sweep areas violate numerical containment monotonicity")
        maximum_sweep_outside_m2 = max(maximum_sweep_outside_m2, outside)
        maximum_raw_sweep_outside_m2 = max(maximum_raw_sweep_outside_m2, raw_outside)
        sweep_raw_violations += int(raw_outside > 1e-6)
        if outside > 1e-6:
            sweep_inset_violations += 1
            detail = {"kind": "bounded_snapshot_linear_sweep",
                      "frame_pair": [before["state"]["frame"], after["state"]["frame"]],
                      "independent_progress_range_m": [before["arc_m"], after["arc_m"]],
                      "actor_pose_carla": [before["pose"], after["pose"]],
                      "selected_intended_lanes": [list(triple) for triple in sorted(lanes)],
                      "raw_sampled_lane_outside_m2": raw_outside,
                      "inset_10cm_lane_outside_m2": outside,
                      "body_endpoint_polygons_carla_xy": [
                          [list(xy) for xy in polygon.exterior.coords]
                          for polygon in (footprints[index], footprints[index + 1])],
                      "interpretation": "raw_sampled_lane_intrusion" if raw_outside > 1e-6
                      else "inset_only_certification_failure"}
            violations.append(detail)
    violations.sort(key=lambda row: row.get("frame", row.get("frame_pair", [0])[0]))
    return {"status": "FAIL" if violations else "PASS",
            "reason": f"{body_inset_violations} bodies and {sweep_inset_violations} sweeps outside 10cm inner corridor"
            if violations else None,
            "failure_detail": violations[0] if violations else None,
            "worst_inset_violation": max(violations, key=lambda row: row["inset_10cm_lane_outside_m2"])
            if violations else None,
            "scope": "sampled 20 Hz actor body and bounded snapshot-linear sweeps only",
            "frames_checked": len(selected), "sweeps_checked": len(selected) - 1,
            "frame_range": [selected[0]["state"]["frame"], selected[-1]["state"]["frame"]],
            "independent_progress_range_m": [selected[0]["arc_m"], selected[-1]["arc_m"]],
            "selected_intended_lanes": [list(triple) for triple in sorted(requested)],
            "maximum_body_outside_m2": maximum_outside_m2,
            "maximum_body_raw_sampled_lane_outside_m2": maximum_raw_outside_m2,
            "maximum_sweep_outside_m2": maximum_sweep_outside_m2,
            "maximum_sweep_raw_sampled_lane_outside_m2": maximum_raw_sweep_outside_m2,
            "body_raw_sampled_lane_violation_count": body_raw_violations,
            "body_inset_10cm_violation_count": body_inset_violations,
            "sweep_raw_sampled_lane_violation_count": sweep_raw_violations,
            "sweep_inset_10cm_violation_count": sweep_inset_violations,
            "identical_geometry_command_nodes_collapsed": duplicate_geometry_nodes,
            "lane_sample_m": LANE_SAMPLE_M, "lane_inset_m": LANE_INSET_M,
            "authorized_seam_gap_close_m": SEAM_GAP_CLOSE_M,
            "physical_continuous_time_proven": False, "training_data_approved": False,
            "source_sha256": hashes}


def render_failure_witness(episode: Path, witness: dict[str, Any], output: Path,
                           *, carla_module: Any = None) -> None:
    """HH_260906 - Plot the worst raw/inset lane witness without altering the source episode."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    if carla_module is None:
        import carla as carla_module
    manifest, route, states, xml, _ = verify_sources(episode)
    points = [route["route"][0]]
    for point in route["route"][1:]:
        if point["distance_m"] != points[-1]["distance_m"]:
            points.append(point)
    lane_ids = {tuple(item) for item in witness["selected_intended_lanes"]}
    pose_values = witness["actor_pose_carla"]
    poses = pose_values if isinstance(pose_values, list) else [pose_values]
    box, radius = _body_box(manifest, carla_module)
    arcs = [_route_projection((pose["x"], pose["y"]), points)[0] for pose in poses]
    relevant = {node_index for segment_index, (a, b) in enumerate(zip(points, points[1:]))
                if a["distance_m"] <= max(arcs) + radius and b["distance_m"] >= min(arcs) - radius
                for node_index in (segment_index, segment_index + 1)}
    carla_map = carla_module.Map("archived-active-map", xml.decode("utf-8"))
    corridors = lane_corridors(carla_map, carla_module, points, xml, lane_ids, relevant)
    footprints = [body_footprint(box, pose, carla_module) for pose in poses]
    body = (swept_body(footprints[0], footprints[1], poses[0], poses[1], radius)
            if len(poses) == 2 else footprints[0])
    raw = authorized_lane_region(corridors, lane_ids, 0.0)
    inner = authorized_lane_region(corridors, lane_ids, LANE_INSET_M)
    xmin, ymin, xmax, ymax = body.bounds
    clip = geometry_box(xmin - 4.0, ymin - 4.0, xmax + 4.0, ymax + 4.0)

    def draw_shape(axis: Any, shape: Any, color: str, alpha: float, edge: str) -> None:
        for polygon in (shape.geoms if shape.geom_type == "MultiPolygon" else [shape]):
            if polygon.is_empty or polygon.geom_type != "Polygon":
                continue
            xs, ys = polygon.exterior.xy
            axis.fill(xs, ys, facecolor=color, alpha=alpha, edgecolor=edge, linewidth=1.3)

    figure, axis = plt.subplots(figsize=(9, 8), dpi=160)
    draw_shape(axis, raw.intersection(clip), "#95d6a4", 0.62, "#27844c")
    draw_shape(axis, inner.intersection(clip), "#abcaf4", 0.32, "#285995")
    draw_shape(axis, body, "#e34440", 0.50, "#991b1b")
    for polygon in footprints:
        xs, ys = polygon.exterior.xy
        axis.plot(xs, ys, color="#8b1010", linewidth=1.3)
    route_xy = [(point["x"], -point["y"]) for point in points]
    axis.plot([xy[0] for xy in route_xy], [xy[1] for xy in route_xy],
              color="#333333", linestyle="--", linewidth=1, label="recorded route center")
    axis.set_xlim(clip.bounds[0], clip.bounds[2])
    axis.set_ylim(clip.bounds[1], clip.bounds[3])
    axis.set_aspect("equal", adjustable="box")
    axis.grid(alpha=0.2)
    axis.set_xlabel("CARLA x (m)")
    axis.set_ylabel("CARLA y (m)")
    axis.set_title(f"Intended lane witness: {witness['kind']} | raw outside "
                   f"{witness['raw_sampled_lane_outside_m2']:.3f} m²")
    axis.text(0.02, 0.02, "Green: sampled intended lane (+≤5 mm seam closure)\n"
              "Blue: 10 cm inner corridor   Red: body/sweep",
              transform=axis.transAxes, va="bottom", fontsize=9,
              bbox=dict(facecolor="white", alpha=0.8, edgecolor="none"))
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, bbox_inches="tight")
    plt.close(figure)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--start-progress-m", type=float)
    parser.add_argument("--end-progress-m", type=float)
    parser.add_argument("--plot-output", type=Path,
                        help="optional zoomed PNG of the worst failed body/sweep")
    args = parser.parse_args()
    episode = args.episode.expanduser().resolve()
    output = args.output.expanduser().resolve()
    if output == episode or episode in output.parents:
        parser.error("output must be outside the immutable source episode")
    plot_output = args.plot_output.expanduser().resolve() if args.plot_output else None
    if plot_output and (plot_output == episode or episode in plot_output.parents or plot_output.exists()):
        parser.error("plot output must be new and outside the immutable source episode")
    try:
        report = audit_episode(episode, start_m=args.start_progress_m, end_m=args.end_progress_m)
    except Unsupported as error:
        report = {"status": "UNSUPPORTED", "reason": str(error), "training_data_approved": False}
    except (OSError, ValueError, KeyError, TypeError, ET.ParseError) as error:
        report = {"status": "UNSUPPORTED", "reason": f"invalid source or unsupported geometry: {error}",
                  "training_data_approved": False}
    if output.exists():
        parser.error("output already exists")
    if plot_output and report["status"] == "FAIL" and report.get("worst_inset_violation"):
        render_failure_witness(episode, report["worst_inset_violation"], plot_output)
        report["worst_witness_plot"] = str(plot_output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps(report, sort_keys=True))
    return 0 if report["status"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
