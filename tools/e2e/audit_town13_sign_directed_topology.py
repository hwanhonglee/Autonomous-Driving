#!/usr/bin/env python3
"""HH_260928 - Screen Town13 signs by an over-approximated directed OpenDRIVE road graph."""

from __future__ import annotations

import heapq
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
from typing import Any

from scripts.e2e import audit_town13_sign_cluster_feasibility as cluster


OUTPUT = cluster.ROOT / "docs/assets/validation/2026-09-28/map_town13_sign_cluster_feasibility/directed_topology.json"
MAX_SPAN_M = cluster.TARGET_MAX_ROUTE_M - cluster.MIN_APPROACH_M - cluster.MIN_TAIL_M
DISPLAY_LIMIT = 30
MAX_ROADS_PER_PATH = 20


def parse_road_graph_source(xodr: Path) -> tuple[dict[int, dict[str, Any]], dict[int, list[dict[str, Any]]]]:
    """HH_260928 - Read road endpoint links and junction connections without a running CARLA map."""
    roads: dict[int, dict[str, Any]] = {}
    junctions: dict[int, list[dict[str, Any]]] = {}
    try:
        for _, item in ET.iterparse(xodr, events=("end",)):
            if item.tag == "road":
                road_id = int(item.attrib["id"])
                cluster.require(road_id not in roads, f"duplicate XODR road: {road_id}")
                length = float(item.attrib["length"])
                cluster.require(math.isfinite(length) and length > 0,
                                f"invalid road length: {road_id}")
                links = {}
                for name in ("predecessor", "successor"):
                    part = item.find(f"link/{name}")
                    if part is not None:
                        links[name] = dict(part.attrib)
                roads[road_id] = {"length_m": length, "links": links}
                item.clear()
            elif item.tag == "junction":
                junction_id = int(item.attrib["id"])
                cluster.require(junction_id not in junctions,
                                f"duplicate XODR junction: {junction_id}")
                junctions[junction_id] = [dict(connection.attrib)
                                          for connection in item.findall("connection")]
                item.clear()
    except (ET.ParseError, OSError, KeyError, TypeError, ValueError) as error:
        if isinstance(error, cluster.AuditError):
            raise
        raise cluster.AuditError(f"Town13 road graph parse failed: {error}") from error
    cluster.require(roads and junctions, "Town13 road graph or junctions absent")
    return roads, junctions


def directed_neighbors(roads: dict[int, dict[str, Any]],
                       junctions: dict[int, list[dict[str, Any]]]) -> dict[tuple[int, str], list[tuple[int, str]]]:
    """HH_260928 - Include possible endpoint transitions, not lane-legal route proofs."""
    edges: dict[tuple[int, str], list[tuple[int, str]]] = {}
    for road_id, road in roads.items():
        for direction in ("+", "-"):
            endpoint = "successor" if direction == "+" else "predecessor"
            link = road["links"].get(endpoint)
            neighbors = set()
            if link is not None:
                element_type = link.get("elementType")
                destination = int(link["elementId"])
                if element_type == "road":
                    cluster.require(destination in roads,
                                    f"linked road absent: {road_id} -> {destination}")
                    contact = link.get("contactPoint")
                    cluster.require(contact in ("start", "end"),
                                    f"direct road contact point absent: {road_id}")
                    neighbors.add((destination, "+" if contact == "start" else "-"))
                elif element_type == "junction":
                    cluster.require(destination in junctions,
                                    f"linked junction absent: {road_id} -> {destination}")
                    for connection in junctions[destination]:
                        if int(connection["incomingRoad"]) != road_id:
                            continue
                        connecting = int(connection["connectingRoad"])
                        cluster.require(connecting in roads,
                                        f"junction connecting road absent: {connecting}")
                        contact = connection.get("contactPoint")
                        cluster.require(contact in ("start", "end"),
                                        f"junction contact point absent: {destination}")
                        neighbors.add((connecting, "+" if contact == "start" else "-"))
                else:
                    raise cluster.AuditError(f"unsupported XODR road link type: {element_type}")
            edges[(road_id, direction)] = sorted(neighbors)
    return edges


def directed_sign_sequences(signs: list[dict[str, Any]],
                            roads: dict[int, dict[str, Any]],
                            edges: dict[tuple[int, str], list[tuple[int, str]]],
                            max_span_m: float = MAX_SPAN_M) -> list[dict[str, Any]]:
    """HH_260928 - Find three oriented records on a simple directed road sequence."""
    per_road: dict[tuple[int, str], list[dict[str, Any]]] = {}
    for sign in signs:
        orientation = sign["orientation"]
        cluster.require(orientation in ("+", "-"),
                        f"unsupported sign orientation: {sign['record_id']}")
        cluster.require(sign["road_id"] in roads, f"sign road absent: {sign['record_id']}")
        per_road.setdefault((sign["road_id"], orientation), []).append(sign)
    for key, records in per_road.items():
        records.sort(key=lambda row: (row["s_m"] if key[1] == "+" else -row["s_m"],
                                      row["record_id"]))
    found: dict[tuple[str, str, str], dict[str, Any]] = {}
    serial = 0
    for first in signs:
        road_id = first["road_id"]
        direction = first["orientation"]
        heap: list[tuple[float, int, int, str, float, tuple[str, ...], tuple[int, ...]]] = [
            (0.0, serial, road_id, direction, first["s_m"],
             (first["record_id"],), (road_id,))]
        while heap:
            distance, _order, current_road, current_direction, position, seen, path = heapq.heappop(heap)
            candidates = [sign for sign in per_road.get((current_road, current_direction), ())
                          if sign["record_id"] not in seen
                          and (sign["s_m"] - position if current_direction == "+"
                               else position - sign["s_m"]) >= -1e-6]
            if candidates:
                following = candidates[0]
                step = max(0.0, abs(following["s_m"] - position))
                next_distance = distance + step
                if next_distance <= max_span_m + 1e-6:
                    next_seen = seen + (following["record_id"],)
                    if len(next_seen) == 3:
                        row = {"record_ids_in_travel_order": list(next_seen),
                               "road_ids_in_travel_order": [int(record.split(":", 1)[0])
                                                            for record in next_seen],
                               "road_path": list(path),
                               "sign_to_sign_route_span_m": round(next_distance, 3),
                               "minimum_route_with_20m_ends_m": round(
                                   next_distance + cluster.MIN_APPROACH_M + cluster.MIN_TAIL_M, 3),
                               "road_endpoint_graph_only": True,
                               "lane_legal_route_proven": False,
                               "camera_visibility_proven": False}
                        prior = found.get(next_seen)
                        if prior is None or (row["sign_to_sign_route_span_m"], row["road_path"]) < (
                                prior["sign_to_sign_route_span_m"], prior["road_path"]):
                            found[next_seen] = row
                    else:
                        serial += 1
                        heapq.heappush(heap, (next_distance, serial, current_road,
                                              current_direction, following["s_m"], next_seen, path))
                continue
            length = roads[current_road]["length_m"]
            exit_position = length if current_direction == "+" else 0.0
            remaining = abs(exit_position - position)
            next_distance = distance + remaining
            if next_distance > max_span_m + 1e-6 or len(path) >= MAX_ROADS_PER_PATH:
                continue
            for next_road, next_direction in edges[(current_road, current_direction)]:
                if next_road in path:
                    continue
                serial += 1
                start = 0.0 if next_direction == "+" else roads[next_road]["length_m"]
                heapq.heappush(heap, (next_distance, serial, next_road, next_direction,
                                      start, seen, path + (next_road,)))
    return sorted(found.values(), key=lambda row: (
        row["sign_to_sign_route_span_m"], row["record_ids_in_travel_order"]))


def build_report(xodr: Path = cluster.XODR,
                 expected_sha256: str = cluster.XODR_SHA256) -> dict[str, Any]:
    observed_sha256 = cluster.digest(xodr)
    cluster.require(observed_sha256 == expected_sha256,
                    "Town13 XODR SHA256 differs from pinned source")
    signs, counts = cluster.parse_signs(xodr)
    roads, junctions = parse_road_graph_source(xodr)
    edges = directed_neighbors(roads, junctions)
    sequences = directed_sign_sequences(signs, roads, edges)
    return {
        "schema": "portable_e2e.town13_sign_directed_topology_audit.v1",
        "status": ("POTENTIAL_DIRECTED_ROAD_SEQUENCE" if sequences
                   else "STOP_NO_PARSED_DIRECTED_THREE_SIGN_SEQUENCE"),
        "source_xodr": str(xodr), "source_sha256": observed_sha256,
        "counts": {"road_count": len(roads), "junction_count": len(junctions),
                   "sign_count": len(signs), "stop_count": counts["stop_count"],
                   "yield_count": counts["yield_count"]},
        "rules": {"orientation_plus": "increasing OpenDRIVE s",
                  "orientation_minus": "decreasing OpenDRIVE s",
                  "road_sequence": "direct road endpoint links and junction incomingRoad/connectingRoad only",
                  "same_road_revisit_allowed": False,
                  "maximum_road_count_per_sequence": MAX_ROADS_PER_PATH,
                  "maximum_sign_to_sign_span_m": MAX_SPAN_M,
                  "minimum_approach_m": cluster.MIN_APPROACH_M,
                  "minimum_tail_m": cluster.MIN_TAIL_M,
                  "lane_link_continuity_checked": False,
                  "route_length_range_m": [cluster.TARGET_MIN_ROUTE_M, cluster.TARGET_MAX_ROUTE_M]},
        "sequence_count": len(sequences),
        "nominal_distance_buckets": {str(bound): sum(
            row["minimum_route_with_20m_ends_m"] <= bound for row in sequences)
            for bound in (60, 100, 120, 180, 250)},
        "best_sequences": sequences[:DISPLAY_LIMIT],
        "catalog_authorized": False, "capture_authorized": False,
        "limitations": [
            "This is an over-approximation of directed road endpoint connectivity, not lane-level graph reachability.",
            "Lane link continuity, traffic law, route planner output, start/goal endpoints and scenario filter are unverified.",
            "Static XODR signs may be absent, reversed, occluded or too small in RGB/semantic/depth imagery.",
            "The simple-road and 20-road bounds may omit a revisiting or much more fragmented route.",
            "240 frames at 10 Hz cover nominally 60 m at 9 kph or 100 m at 15 kph only if speed is maintained.",
            "No CARLA/UE4Editor, camera, GPU, SSH, training, TEST map or dataset admission was used."],
    }


def main() -> int:
    try:
        report = build_report()
        cluster.require(not OUTPUT.exists() and not OUTPUT.is_symlink(),
                        "refusing to overwrite a directed topology report")
        OUTPUT.parent.mkdir(parents=True, exist_ok=True)
        with OUTPUT.open("x", encoding="utf-8") as stream:
            json.dump(report, stream, indent=2, sort_keys=True)
            stream.write("\n")
    except (cluster.AuditError, OSError) as error:
        print(f"STOP_TOWN13_SIGN_TOPOLOGY: {error}", file=sys.stderr)
        return 2
    print(f"Town13 sign topology: {report['status']}; sequences={report['sequence_count']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
