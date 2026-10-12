#!/usr/bin/env python3
"""HH_260928 - Reproduce a bounded offline right-turn scout on non-TEST C-track."""

from __future__ import annotations

import argparse
import hashlib
import inspect
import json
import xml.etree.ElementTree as ET
from pathlib import Path

from networkx.exception import NetworkXNoPath

from scripts.e2e import physical_turn_geometry as physical
from scripts.e2e import prepare_carla_route as route_helper


ROOT = Path(__file__).resolve().parents[2]
XODR = Path(
    "/home/a/carla-autoware-universe/CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/"
    "OpenDrive/C_track_1_0_7.xodr"
)
XODR_SHA256 = "3f760b8ec8cb117a975c9631a74948b620c991aeaef5c1b07497ebb37ec7be27"
SPLIT_SOURCE = ROOT / "storage/experiments/training/2026-09-15/moving_cohort_v1/cohort.json"
SPLIT_SOURCE_SHA256 = "32765291218e35ce1ea002db68f8b2e0b694889252c82e5e38bc8cde8f271863"
PHYSICAL_SOURCE_SHA256 = "94359e82390ab8c115c514bae6cb874f233bc46b548a6cab239f2e49551bcdfd"
SERIALIZER_SOURCE_SHA256 = "482f0ea2af2f6e5b53619237b1442ddb78e211b9374f6529480768346f304c5d"
OUTPUT = ROOT / "docs/assets/validation/2026-09-28/drive_next_independent_scout/report.json"
SEARCH_SCOPES_M = ((50, 90), (60, 80), (60, 100), (70, 80))
WINDOW_M = (30.0, 120.0)
TARGET_SPEED_KMH = 9.0
FUTURE_SEC = 6.4
FUTURE_GUARD_M = 10.0
MIN_LENGTH_M = WINDOW_M[1] + TARGET_SPEED_KMH / 3.6 * FUTURE_SEC + FUTURE_GUARD_M


class ScoutError(ValueError):
    """HH_260928 - Reject changed source or ambiguous output without running a server."""


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ScoutError(message)


def sha256(path: Path) -> str:
    require(path.is_file() and not path.is_symlink(), f"regular source required: {path}")
    return hashlib.sha256(path.read_bytes()).hexdigest()


def source_roads(xodr_bytes: bytes) -> dict[int, ET.Element]:
    """HH_260928 - Bind signal-reference screening to the exact pinned XODR bytes."""
    root = ET.fromstring(xodr_bytes)
    return {int(road.attrib["id"]): road for road in root.findall("road")}


def right_blocks(points: list[dict]) -> list[tuple[int, int]]:
    """HH_260928 - Count distinct RIGHT command blocks without merging maneuvers."""
    blocks: list[tuple[int, int]] = []
    first = None
    for index, point in enumerate(points):
        if point["road_option"] == "RIGHT":
            if first is None:
                first = index
        elif first is not None:
            blocks.append((first, index - 1))
            first = None
    if first is not None:
        blocks.append((first, len(points) - 1))
    return blocks


def route_digest(points: list[dict]) -> str:
    payload = json.dumps(points, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(payload).hexdigest()


def endpoint(waypoint: object) -> dict:
    loc = waypoint.transform.location
    return {
        "road_id": int(waypoint.road_id),
        "lane_id": int(waypoint.lane_id),
        "xyz_carla_m": [float(loc.x), float(loc.y), float(loc.z)],
        "yaw_carla_deg": float(waypoint.transform.rotation.yaw),
    }


def witness_key(row: dict) -> tuple:
    """HH_260928 - Rank rejections by fewest failed physical rules, then grade excess."""
    return (
        len(row["physical_failure_reasons"]),
        row["maximum_grade_ratio"] / row["allowed_grade_ratio"],
        -row["route_length_m"],
        row["scope_index"],
        row["junction_id"],
        row["connector_road_id"],
        row["start_index"],
        row["goal_index"],
    )


def scan(carla: object, planner_class: object, xodr: Path = XODR) -> dict:
    """HH_260928 - Search four exact endpoint grids without CARLA client/server calls."""
    require(sha256(xodr) == XODR_SHA256, "pinned C-track XODR changed")
    require(sha256(SPLIT_SOURCE) == SPLIT_SOURCE_SHA256, "pinned TRAIN split plan changed")
    split_plan = json.loads(SPLIT_SOURCE.read_text())
    c_track_slots = [slot for slot in split_plan["slots"]
                     if slot.get("map_id") == "c_track_1_0_7"]
    require(len(c_track_slots) == 6 and all(slot.get("split") == "train" for slot in c_track_slots),
            "C-track TRAIN split changed")
    require(sha256(Path(physical.__file__)) == PHYSICAL_SOURCE_SHA256,
            "physical-turn source changed")
    require(sha256(Path(route_helper.__file__)) == SERIALIZER_SOURCE_SHA256,
            "route serializer source changed")
    xodr_bytes = xodr.read_bytes()
    roads = source_roads(xodr_bytes)
    carla_map = carla.Map("offline-C_track_1_0_7-independent-scout", xodr_bytes.decode())
    planner = planner_class(carla_map, 1.0)
    junctions = {point.get_junction().id: point.get_junction()
                 for point in carla_map.generate_waypoints(3.0) if point.is_junction}
    connectors = []
    signal_controlled = 0
    for junction_id, junction in sorted(junctions.items()):
        for entry, exit_ in junction.get_waypoints(carla.LaneType.Driving):
            angle = (exit_.transform.rotation.yaw - entry.transform.rotation.yaw + 180) % 360 - 180
            if not 60 <= angle <= 120:
                continue
            require(entry.road_id in roads, "right-turn connector absent from pinned XODR")
            if roads[entry.road_id].findall("./signals/signalReference"):
                signal_controlled += 1
                continue
            connectors.append((junction_id, entry, exit_))
    candidates = []
    scopes = []
    for scope_index, (upstream_m, downstream_m) in enumerate(SEARCH_SCOPES_M):
        stages = {
            "endpoint_pairs": 0,
            "unreachable_endpoint_pairs": 0,
            "length_at_least_146m": 0,
            "traverses_selected_connector": 0,
            "single_right_no_other_maneuver": 0,
            "right_turn_in_fixed_window": 0,
            "physical_turn_pass": 0,
        }
        for junction_id, entry, exit_ in connectors:
            for start_index, start in enumerate(entry.previous(float(upstream_m))):
                for goal_index, goal in enumerate(exit_.next(float(downstream_m))):
                    stages["endpoint_pairs"] += 1
                    try:
                        traced = planner.trace_route(start.transform.location, goal.transform.location)
                    except NetworkXNoPath:
                        stages["unreachable_endpoint_pairs"] += 1
                        continue
                    points = route_helper.serialize_route(traced, goal.transform)
                    if len(points) < 2 or float(points[-1]["distance_m"]) < MIN_LENGTH_M:
                        continue
                    stages["length_at_least_146m"] += 1
                    if not any(point["road_id"] == entry.road_id and
                               point["lane_id"] == entry.lane_id for point in points):
                        continue
                    stages["traverses_selected_connector"] += 1
                    if {point["road_option"] for point in points} - {"RIGHT", "LANEFOLLOW"}:
                        continue
                    blocks = right_blocks(points)
                    if len(blocks) != 1:
                        continue
                    stages["single_right_no_other_maneuver"] += 1
                    turn_start = float(points[blocks[0][0]]["distance_m"])
                    turn_end = float(points[blocks[0][1]]["distance_m"])
                    if not WINDOW_M[0] <= turn_start <= turn_end <= WINDOW_M[1]:
                        continue
                    stages["right_turn_in_fixed_window"] += 1
                    route = {"route": points, "route_length_m": points[-1]["distance_m"],
                             "scenario": "right", "coordinate_reference": "base_link"}
                    try:
                        physical_result = physical.analyze_serialized_physical_turn(
                            route, physical.SPEED_30KPH_TURN_GEOMETRY_CONTRACT)
                    except physical.PhysicalTurnGeometryError as error:
                        if error.fatal:
                            raise ScoutError(f"physical-turn contract failed: {error}") from error
                        continue
                    if physical_result["status"] == "PASS":
                        stages["physical_turn_pass"] += 1
                        continue
                    grade = physical_result["longitudinal_grade"]
                    candidates.append({
                        "scope_index": scope_index,
                        "upstream_m": upstream_m,
                        "downstream_m": downstream_m,
                        "junction_id": junction_id,
                        "connector_road_id": int(entry.road_id),
                        "connector_lane_id": int(entry.lane_id),
                        "start_index": start_index,
                        "goal_index": goal_index,
                        "start": endpoint(start),
                        "goal": endpoint(goal),
                        "route_sha256": route_digest(points),
                        "route_length_m": float(points[-1]["distance_m"]),
                        "right_turn_m": [turn_start, turn_end],
                        "physical_failure_reasons": physical_result["failure_reasons"],
                        "maximum_grade_ratio": grade["maximum_absolute_grade_ratio"],
                        "allowed_grade_ratio": grade["maximum_allowed_absolute_grade_ratio"],
                        "worst_grade_window": grade["worst_window"],
                    })
        scopes.append({"upstream_m": upstream_m, "downstream_m": downstream_m,
                       "stages": stages})
    total_pairs = sum(scope["stages"]["endpoint_pairs"] for scope in scopes)
    total_pass = sum(scope["stages"]["physical_turn_pass"] for scope in scopes)
    best = min(candidates, key=witness_key) if candidates else None
    if best is not None:
        best = {key: value for key, value in best.items() if key != "scope_index"}
    return {
        "schema": "portable_e2e.independent_c_track_right_offline_scout.v1",
        "status": "NO_PHYSICAL_TURN_PASS" if total_pass == 0 else "PHYSICAL_ONLY_NOT_CAPTURE_READY",
        "split": "TRAIN_CANDIDATE_C_TRACK_1_0_7",
        "split_source_path": str(SPLIT_SOURCE),
        "split_source_sha256": SPLIT_SOURCE_SHA256,
        "split_source_ct_slot_count": len(c_track_slots),
        "source_xodr_path": str(xodr),
        "source_xodr_sha256": XODR_SHA256,
        "scout_tool_sha256": sha256(Path(__file__)),
        "physical_turn_source_sha256": PHYSICAL_SOURCE_SHA256,
        "route_serializer_source_sha256": SERIALIZER_SOURCE_SHA256,
        "planner_source_sha256": sha256(Path(inspect.getsourcefile(planner_class))),
        "physical_turn_profile": physical.SPEED_30KPH_TURN_GEOMETRY_CONTRACT["profile_id"],
        "right_heading_range_deg": [60, 120],
        "sampling_resolution_m": 1.0,
        "fixed_window_m": list(WINDOW_M),
        "minimum_route_length_m": MIN_LENGTH_M,
        "future_at_9kmh_m": TARGET_SPEED_KMH / 3.6 * FUTURE_SEC,
        "future_guard_m": FUTURE_GUARD_M,
        "moving_contract": {"minimum_continuous_sec": 30.0, "speed_strictly_above_mps": 0.5},
        "goal_tolerance_m": 2.5,
        "connector_count": len(connectors),
        "signal_controlled_connectors_excluded": signal_controlled,
        "scopes": scopes,
        "total_endpoint_pairs": total_pairs,
        "total_physical_turn_pass": total_pass,
        "best_rejected_witness": best,
        "live_map_sha_verified": False,
        "nominal_body_corridor_verified": False,
        "continuous_moving_verified": False,
        "actual_goal_verified": False,
        "capture_authorized": False,
        "dataset_admitted": False,
        "training_executed": False,
    }


def publish(output: Path = OUTPUT, xodr: Path = XODR) -> dict:
    """HH_260928 - Write only a fresh report after all offline checks pass."""
    require(not output.exists() and not output.is_symlink(), "scout report already exists")
    require(output.parent.is_dir() and not output.parent.is_symlink(),
            "create a regular report directory before scouting")
    import carla
    from agents.navigation.global_route_planner import GlobalRoutePlanner

    report = scan(carla, GlobalRoutePlanner, xodr)
    output.write_text(json.dumps(report, indent=2) + "\n")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--xodr", type=Path, default=XODR)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    args = parser.parse_args()
    report = publish(args.output, args.xodr)
    print(json.dumps({key: report[key] for key in
                      ("status", "total_endpoint_pairs", "total_physical_turn_pass")}, indent=2))


if __name__ == "__main__":
    main()
