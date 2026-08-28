#!/usr/bin/env python3

"""Finalize a CommonRoad OpenDRIVE conversion for Autoware's Local projector."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import tempfile
from pathlib import Path

import carla
from lxml import etree
import numpy as np
from pyproj import CRS, Transformer
from scipy.spatial import cKDTree


class FinalizationError(RuntimeError):
    """Raised when converted map geometry cannot be finalized safely."""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Add authoritative local coordinates, elevation, and Autoware road "
            "attributes to CommonRoad's OpenDRIVE-to-Lanelet2 output."
        )
    )
    parser.add_argument("source_osm", type=Path)
    parser.add_argument("source_xodr", type=Path)
    parser.add_argument("output_osm", type=Path)
    parser.add_argument("--translation-x-m", type=float, default=0.0)
    parser.add_argument("--translation-y-m", type=float, default=0.0)
    parser.add_argument("--translation-z-m", type=float, default=0.0)
    parser.add_argument("--yaw-rad", type=float, default=0.0)
    parser.add_argument("--waypoint-resolution-m", type=float, default=0.25)
    parser.add_argument("--bounds-margin-m", type=float, default=50.0)
    parser.add_argument("--speed-limit-kmh", type=float, default=30.0)
    parser.add_argument("--map-version", default="xodr-commonroad-0.8.5")
    parser.add_argument("--json", action="store_true")
    return parser.parse_args()


def finite(value: float, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise FinalizationError(f"{name} must be finite")
    return result


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def xodr_projection(xodr_root: etree._Element) -> Transformer:
    text = xodr_root.findtext(".//geoReference")
    if not text or not text.strip():
        raise FinalizationError("OpenDRIVE has no geoReference")
    tokens = [
        token
        for token in text.strip().split()
        if not token.startswith("+geoidgrids=")
    ]
    projection = CRS.from_proj4(" ".join(tokens))
    return Transformer.from_crs(CRS.from_epsg(4326), projection, always_xy=True)


def xodr_bounds(xodr_root: etree._Element, margin_m: float) -> tuple[float, ...]:
    if not math.isfinite(margin_m) or margin_m < 0.0:
        raise FinalizationError("bounds margin must be finite and non-negative")
    header = xodr_root.find(".//header")
    if header is None:
        raise FinalizationError("OpenDRIVE has no header")
    try:
        west = float(header.get("west"))
        east = float(header.get("east"))
        south = float(header.get("south"))
        north = float(header.get("north"))
    except (TypeError, ValueError) as error:
        raise FinalizationError("OpenDRIVE header has invalid bounds") from error
    if not all(math.isfinite(value) for value in (west, east, south, north)):
        raise FinalizationError("OpenDRIVE header bounds must be finite")
    if west > east or south > north:
        raise FinalizationError("OpenDRIVE header bounds are inverted")
    return (
        west - margin_m,
        east + margin_m,
        south - margin_m,
        north + margin_m,
    )


def prune_out_of_bounds_geometry(
    osm_root: etree._Element,
    nodes: list[etree._Element],
    source_xy: np.ndarray,
    bounds: tuple[float, ...],
) -> dict[str, int]:
    source_xy = np.asarray(source_xy, dtype=np.float64)
    if source_xy.shape != (len(nodes), 2):
        raise FinalizationError("source coordinates must match OSM nodes")

    node_ids = [node.get("id") for node in nodes]
    if any(node_id is None for node_id in node_ids) or len(set(node_ids)) != len(
        node_ids
    ):
        raise FinalizationError("OSM nodes must have unique IDs")

    west, east, south, north = bounds
    valid_mask = (
        np.isfinite(source_xy).all(axis=1)
        & (source_xy[:, 0] >= west)
        & (source_xy[:, 0] <= east)
        & (source_xy[:, 1] >= south)
        & (source_xy[:, 1] <= north)
    )
    invalid_nodes = {
        node.get("id") for node, valid in zip(nodes, valid_mask) if not valid
    }
    for node in nodes:
        if node.get("id") in invalid_nodes:
            osm_root.remove(node)

    ways = list(osm_root.findall("way"))
    way_ids = [way.get("id") for way in ways]
    if any(way_id is None for way_id in way_ids) or len(set(way_ids)) != len(
        way_ids
    ):
        raise FinalizationError("OSM ways must have unique IDs")

    remaining_nodes = set(node_ids) - invalid_nodes
    removed_ways = set()
    for way in ways:
        references = [node.get("ref") for node in way.findall("nd")]
        if not references or any(
            reference not in remaining_nodes for reference in references
        ):
            removed_ways.add(way.get("id"))
            osm_root.remove(way)

    relations = list(osm_root.findall("relation"))
    relation_ids = [relation.get("id") for relation in relations]
    if any(relation_id is None for relation_id in relation_ids) or len(
        set(relation_ids)
    ) != len(relation_ids):
        raise FinalizationError("OSM relations must have unique IDs")

    remaining_ways = set(way_ids) - removed_ways
    removed_relations = set()
    for relation in relations:
        tags = relation_tags(relation)
        if tags.get("type") == "lanelet":
            boundary_refs = {
                role: [
                    member.get("ref")
                    for member in relation.findall("member")
                    if member.get("type") == "way" and member.get("role") == role
                ]
                for role in ("left", "right")
            }
            if any(
                len(boundary_refs[role]) != 1
                or boundary_refs[role][0] not in remaining_ways
                for role in ("left", "right")
            ):
                removed_relations.add(relation.get("id"))
                osm_root.remove(relation)

    # Non-lanelet relations can reference one another, so remove dangling chains
    # to a fixed point before stripping optional members from surviving lanelets.
    while True:
        remaining_relations = {
            relation.get("id") for relation in osm_root.findall("relation")
        }
        invalid_relations = []
        for relation in osm_root.findall("relation"):
            if relation_tags(relation).get("type") == "lanelet":
                continue
            valid_ids = {
                "node": remaining_nodes,
                "way": remaining_ways,
                "relation": remaining_relations,
            }
            if any(
                member.get("type") not in valid_ids
                or member.get("ref") not in valid_ids[member.get("type")]
                for member in relation.findall("member")
            ):
                invalid_relations.append(relation)
        if not invalid_relations:
            break
        for relation in invalid_relations:
            removed_relations.add(relation.get("id"))
            osm_root.remove(relation)

    remaining_relations = {
        relation.get("id") for relation in osm_root.findall("relation")
    }
    valid_ids = {
        "node": remaining_nodes,
        "way": remaining_ways,
        "relation": remaining_relations,
    }
    for relation in osm_root.findall("relation"):
        for member in list(relation.findall("member")):
            if member.get("type") not in valid_ids or member.get("ref") not in valid_ids[
                member.get("type")
            ]:
                relation.remove(member)

    return {
        "node_count": len(invalid_nodes),
        "way_count": len(removed_ways),
        "relation_count": len(removed_relations),
    }


def xodr_waypoint_surface(
    xodr_path: Path, resolution_m: float
) -> tuple[cKDTree, np.ndarray, int]:
    if not math.isfinite(resolution_m) or resolution_m <= 0.0:
        raise FinalizationError("waypoint resolution must be positive and finite")
    try:
        carla_map = carla.Map(xodr_path.stem, xodr_path.read_text(encoding="utf-8"))
        waypoints = carla_map.generate_waypoints(resolution_m)
    except (OSError, UnicodeError, RuntimeError) as error:
        raise FinalizationError(f"failed to sample OpenDRIVE with CARLA: {error}") from error
    if not waypoints:
        raise FinalizationError("OpenDRIVE produced no driving waypoints")
    xy = np.asarray(
        [
            (waypoint.transform.location.x, -waypoint.transform.location.y)
            for waypoint in waypoints
        ],
        dtype=np.float64,
    )
    elevation = np.asarray(
        [waypoint.transform.location.z for waypoint in waypoints], dtype=np.float64
    )
    return cKDTree(xy), elevation, len(waypoints)


def set_tag(element: etree._Element, key: str, value: str) -> None:
    for tag in element.findall("tag"):
        if tag.get("k") == key:
            tag.set("v", value)
            return
    etree.SubElement(element, "tag", k=key, v=value)


def relation_tags(relation: etree._Element) -> dict[str, str]:
    return {tag.get("k", ""): tag.get("v", "") for tag in relation.findall("tag")}


def prune_collapsed_lanelets(
    osm_root: etree._Element,
    local_xy: dict[str, tuple[float, float]],
    tolerance_m: float = 1.0e-6,
) -> dict[str, object]:
    ways = {
        way.get("id"): [node.get("ref") for node in way.findall("nd")]
        for way in osm_root.findall("way")
    }
    removed_ids: list[str] = []
    for relation in list(osm_root.findall("relation")):
        if relation_tags(relation).get("type") != "lanelet":
            continue
        members = {
            member.get("role"): member.get("ref")
            for member in relation.findall("member")
            if member.get("type") == "way"
        }
        collapsed = False
        for role in ("left", "right"):
            references = ways.get(members.get(role, ""), [])
            distinct: list[tuple[float, float]] = []
            for reference in references:
                point = local_xy.get(reference)
                if point is None:
                    collapsed = True
                    break
                if not distinct or math.dist(distinct[-1], point) > tolerance_m:
                    distinct.append(point)
            if collapsed or len(distinct) < 2:
                collapsed = True
                break
        if collapsed:
            removed_ids.append(relation.get("id"))
            osm_root.remove(relation)

    removed = set(removed_ids)
    if removed:
        for relation in osm_root.findall("relation"):
            for member in list(relation.findall("member")):
                if (
                    member.get("type") == "relation"
                    and member.get("ref") in removed
                ):
                    relation.remove(member)
    return {"relation_count": len(removed_ids), "relation_ids": removed_ids}


def atomic_write(tree: etree._ElementTree, output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{output_path.name}.", dir=output_path.parent
    )
    try:
        with os.fdopen(descriptor, "wb") as stream:
            tree.write(
                stream,
                encoding="UTF-8",
                xml_declaration=True,
                pretty_print=True,
            )
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, output_path)
    finally:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass


def finalize(args: argparse.Namespace) -> dict:
    source_osm = args.source_osm.expanduser().resolve()
    source_xodr = args.source_xodr.expanduser().resolve()
    output_osm = args.output_osm.expanduser().resolve()
    for path, label in ((source_osm, "source OSM"), (source_xodr, "source XODR")):
        if not path.is_file():
            raise FinalizationError(f"{label} does not exist: {path}")

    tx = finite(args.translation_x_m, "translation x")
    ty = finite(args.translation_y_m, "translation y")
    tz = finite(args.translation_z_m, "translation z")
    yaw = finite(args.yaw_rad, "yaw")
    speed_limit = finite(args.speed_limit_kmh, "speed limit")
    if speed_limit <= 0.0:
        raise FinalizationError("speed limit must be positive")

    parser = etree.XMLParser(remove_blank_text=True, huge_tree=True)
    osm_tree = etree.parse(str(source_osm), parser)
    xodr_tree = etree.parse(str(source_xodr), parser)
    osm_root = osm_tree.getroot()
    if osm_root.tag != "osm":
        raise FinalizationError("source OSM root must be <osm>")

    xodr_root = xodr_tree.getroot()
    projector = xodr_projection(xodr_root)
    nodes = osm_root.findall("node")
    if not nodes:
        raise FinalizationError("source OSM has no nodes")
    source_xy = np.asarray(
        [
            projector.transform(float(node.get("lon")), float(node.get("lat")))
            for node in nodes
        ],
        dtype=np.float64,
    )
    pruned = prune_out_of_bounds_geometry(
        osm_root,
        nodes,
        source_xy,
        xodr_bounds(xodr_root, args.bounds_margin_m),
    )
    nodes = osm_root.findall("node")
    source_xy = np.asarray(
        [
            projector.transform(float(node.get("lon")), float(node.get("lat")))
            for node in nodes
        ],
        dtype=np.float64,
    )
    if not nodes or not np.isfinite(source_xy).all():
        raise FinalizationError("source OSM has no finite in-bounds geometry")

    surface_tree, surface_z, waypoint_count = xodr_waypoint_surface(
        source_xodr, args.waypoint_resolution_m
    )
    elevation_distance, elevation_index = surface_tree.query(source_xy, workers=-1)
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    target_x = tx + cosine * source_xy[:, 0] - sine * source_xy[:, 1]
    target_y = ty + sine * source_xy[:, 0] + cosine * source_xy[:, 1]
    target_z = tz + surface_z[elevation_index]

    local_xy: dict[str, tuple[float, float]] = {}
    for index, node in enumerate(nodes):
        x = float(target_x[index])
        y = float(target_y[index])
        z = float(target_z[index])
        set_tag(node, "local_x", f"{x:.9f}")
        set_tag(node, "local_y", f"{y:.9f}")
        set_tag(node, "ele", f"{z:.6f}")
        local_xy[node.get("id")] = (x, y)

    pruned_collapsed = prune_collapsed_lanelets(osm_root, local_xy)

    virtual_boundary_count = 0
    for way in osm_root.findall("way"):
        tags = {
            tag.get("k", ""): tag.get("v", "") for tag in way.findall("tag")
        }
        if "type" not in tags:
            set_tag(way, "type", "virtual")
            virtual_boundary_count += 1

    road_lanelets = 0
    inferred_road_lanelets = 0
    inferred_road_shoulder_lanelets = 0
    direction_counts = {"left": 0, "right": 0, "straight": 0}
    for relation in osm_root.findall("relation"):
        tags = relation_tags(relation)
        if tags.get("type") != "lanelet":
            continue
        subtype = tags.get("subtype")
        if subtype is None:
            # CommonRoad leaves OpenDRIVE shoulder/border strips untyped. They
            # are narrow and outside CARLA's driving centers, so keep them out
            # of the vehicle routing graph while satisfying Autoware's map
            # contract.
            set_tag(relation, "subtype", "road_shoulder")
            set_tag(relation, "location", tags.get("location") or "urban")
            inferred_road_shoulder_lanelets += 1
            continue
        if subtype != "road":
            continue
        road_lanelets += 1
        set_tag(relation, "location", tags.get("location") or "urban")
        set_tag(relation, "one_way", "yes")
        set_tag(relation, "speed_limit", f"{speed_limit:g}")
        direction = tags.get("turn_direction")
        if direction is None:
            # Autoware treats tag presence itself as an intersection marker.
            continue
        if direction not in direction_counts:
            raise FinalizationError(
                f"lanelet {relation.get('id')} has invalid turn_direction: "
                f"{direction!r}"
            )
        direction_counts[direction] += 1

    metadata = osm_root.find("MetaInfo")
    if metadata is None:
        metadata = etree.Element("MetaInfo")
        osm_root.insert(0, metadata)
    metadata.set("format_version", "1")
    metadata.set("map_version", args.map_version)
    osm_root.set("generator", "commonroad-scenario-designer + autoware_e2e")
    atomic_write(osm_tree, output_osm)

    quantiles = np.quantile(elevation_distance, [0.5, 0.9, 0.99])
    return {
        "status": "PASS",
        "source_osm": str(source_osm),
        "source_osm_sha256": sha256(source_osm),
        "source_xodr": str(source_xodr),
        "source_xodr_sha256": sha256(source_xodr),
        "output_osm": str(output_osm),
        "output_osm_sha256": sha256(output_osm),
        "node_count": len(nodes),
        "waypoint_count": waypoint_count,
        "road_lanelet_count": road_lanelets,
        "inferred_road_lanelet_count": inferred_road_lanelets,
        "inferred_road_shoulder_lanelet_count": inferred_road_shoulder_lanelets,
        "pruned_out_of_bounds": pruned,
        "pruned_collapsed_lanelets": pruned_collapsed,
        "virtual_boundary_count": virtual_boundary_count,
        "turn_direction_counts": direction_counts,
        "carla_to_map_transform": {
            "x_m": tx,
            "y_m": ty,
            "z_m": tz,
            "yaw_rad": yaw,
        },
        "elevation_source_distance_m": {
            "median": float(quantiles[0]),
            "p90": float(quantiles[1]),
            "p99": float(quantiles[2]),
            "maximum": float(np.max(elevation_distance)),
        },
    }


def main() -> int:
    args = parse_args()
    try:
        result = finalize(args)
    except (FinalizationError, etree.XMLSyntaxError, ValueError) as error:
        print(f"XODR Lanelet finalization failed: {error}")
        return 1
    if result.get("status") != "PASS":
        return 1
    if args.json:
        print(json.dumps(result, indent=2, sort_keys=False))
    else:
        print(
            "XODR Lanelet finalization passed: "
            f"{result['road_lanelet_count']} road lanelets, "
            f"{result['node_count']} nodes -> {result['output_osm']}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
