#!/usr/bin/env python3
"""HH_260929 - Reproduce a bounded, server-free right-turn site search."""

from __future__ import annotations

import argparse
import hashlib
import inspect
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

from networkx.exception import NetworkXNoPath

from scripts.e2e import physical_turn_geometry as physical
from scripts.e2e import prepare_carla_route as route_helper
from scripts.e2e import rank_town01_v5_long_turns as nominal_helper


ROOT = Path(__file__).resolve().parents[2]
XODR_ROOT = Path(
    "/home/a/carla-autoware-universe/CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive"
)
MAP_SHA256 = {
    "Town03": "abf177bcd9b66cfefb7ac4919f6456815f1384f4229f1343c8a6a90bbb7713c9",
    "Town02_Opt": "953c05f17def231239ffcadba3307628a82d0d335b67b4e29c3098f7aed1dd7d",
    "Town05_Opt": "7edc53b0be840e177598da7a4ed9571146d7b2595ce280d154c90f92e8a48851",
}
SOURCE_SHA256 = {
    "tools/e2e/physical_turn_geometry.py": "94359e82390ab8c115c514bae6cb874f233bc46b548a6cab239f2e49551bcdfd",
    "tools/e2e/prepare_carla_route.py": "482f0ea2af2f6e5b53619237b1442ddb78e211b9374f6529480768346f304c5d",
    "tools/e2e/rank_town01_v5_long_turns.py":
        "dfb2dd5647ec27707d535f69f34b6612454d7b7d499b9be81230e33da78e26fa",
    "tools/e2e/rank_town01_wide_turns.py":
        "9cf96cdd0730d68b6da4177076b3f957fe3b693b3c2fc82a684faef562d5cef1",
    "tools/e2e/audit_future_actor_lane_containment.py":
        "75d73449c8ffb17e27d1ddb419bd064a58ab052043b2a7b0ada69de8a65fa57d",
    "tools/e2e/audit_new_long_turn_candidates.py":
        "bb9f8c863d5bbb9e3b3f8f3b0f869af9c6a66a921b7980b10ee21c948198cc59",
    "storage/datasets/prepared/carla-common10-trainval-research-20260914-v1/dataset.json":
        "c73900cedce870a0f55663fde08b3a64b90fae139506fbe2099fdc8bf991c4a9",
    "docs/assets/validation/2026-09-27/map_next_training_design/README.md":
        "656d38b18d39b1847c2b5ced606140edf97456c530128a1cc965a2358aafe630",
    "storage/datasets/raw/carla/common10_v1/2026-09-05/30kph/town03/turn/qualified/"
    "b1_compact_lookahead/ClearNoon/seed_0000/run_002/episode/route.json":
        "47ea507b9a4228fdce79ef58eff1e1d2cf149c02dd2df413ebadf53756f771ee",
    "storage/experiments/training/2026-09-27/"
    "drive_long_route_town01_right_s0002_p06_9kph_steer045_purge15_trial_v1/"
    "episode/manifest.json":
        "9258274bd2c426c186db2b811fca0e546b7423fcec75c11d41702c8bea14dbb8",
}
OUTPUT = ROOT / "docs/assets/validation/2026-09-29/drive_other_site_right_static/report_v3.json"
SCOPES_M = tuple((up, down) for up in (50, 60, 70, 80, 90, 100)
                 for down in (80, 100, 120, 140, 160))
WINDOW_M = (30.0, 120.0)
SPEED_KMH = 9.0
FUTURE_SEC = 6.4
FUTURE_GUARD_M = 10.0
MIN_LENGTH_M = WINDOW_M[1] + SPEED_KMH / 3.6 * FUTURE_SEC + FUTURE_GUARD_M


class ScoutError(ValueError):
    """HH_260929 - Fail closed when pinned evidence or a route is ambiguous."""


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ScoutError(message)


def sha256(path: Path) -> str:
    require(path.is_file() and not path.is_symlink(), f"regular source required: {path}")
    return hashlib.sha256(path.read_bytes()).hexdigest()


def right_block(points: list[dict]) -> tuple[int, int] | None:
    """HH_260929 - Reject separated RIGHT commands and every other maneuver."""
    if {point["road_option"] for point in points} - {"RIGHT", "LANEFOLLOW"}:
        return None
    indices = [index for index, point in enumerate(points)
               if point["road_option"] == "RIGHT"]
    if not indices or len(indices) != indices[-1] - indices[0] + 1:
        return None
    return indices[0], indices[-1]


def route_digest(points: list[dict]) -> str:
    encoded = json.dumps(points, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(encoded).hexdigest()


def right_block_on_connector(points: list[dict], block: tuple[int, int],
                             road_id: int, lane_id: int) -> bool:
    """HH_260929 - Bind the RIGHT command to the connector being enumerated."""
    return any(point["road_id"] == road_id and point["lane_id"] == lane_id
               for point in points[block[0]:block[1] + 1])


def pose(waypoint: object) -> dict:
    location = waypoint.transform.location
    return {
        "road_id": int(waypoint.road_id),
        "lane_id": int(waypoint.lane_id),
        "xyz_carla_m": [float(location.x), float(location.y), float(location.z)],
        "yaw_carla_deg": float(waypoint.transform.rotation.yaw),
    }


def physical_site(points: list[dict], block: tuple[int, int]) -> dict:
    selected = points[block[0]:block[1] + 1]
    transitions = []
    for point in selected:
        lane = [point[key] for key in ("road_id", "section_id", "lane_id")]
        if not transitions or transitions[-1] != lane:
            transitions.append(lane)
    center = selected[len(selected) // 2]
    return {
        "lane_transitions": transitions,
        "center_xy_ros_m": [float(center["x"]), float(center["y"])],
    }


def signal_records(roads: dict[int, ET.Element], points: list[dict]) -> dict:
    """HH_260929 - Include direct signals as well as road signal references."""
    ids = {int(point["road_id"]) for point in points}
    require(ids <= roads.keys(), "route road absent from pinned OpenDRIVE")
    return {
        "direct_signal_road_ids": sorted(
            road_id for road_id in ids if roads[road_id].findall("./signals/signal")),
        "signal_reference_road_ids": sorted(
            road_id for road_id in ids if roads[road_id].findall("./signals/signalReference")),
    }


def other_junction_ids(roads: dict[int, ET.Element], points: list[dict],
                       selected_junction_id: int) -> list[int]:
    """HH_260929 - Treat separate connector roads in one junction as one site."""
    result = set()
    for point in points:
        progress = float(point["distance_m"])
        if not WINDOW_M[0] <= progress <= MIN_LENGTH_M:
            continue
        road = roads[int(point["road_id"])]
        junction_id = int(road.attrib.get("junction", "-1"))
        if junction_id >= 0 and junction_id != selected_junction_id:
            result.add(junction_id)
    return sorted(result)


def old_dev_site(carla_map: object, carla: object) -> dict:
    path = next(ROOT / relative for relative in SOURCE_SHA256
                if relative.endswith("run_002/episode/route.json"))
    old = json.loads(path.read_text())
    selected = old["physical_turn_preflight"]["selected_block"]
    block = selected["start_index"], selected["end_index"]
    site = physical_site(old["route"], block)
    center = old["route"][(block[0] + block[1]) // 2]
    waypoint = carla_map.get_waypoint(
        carla.Location(center["x"], -center["y"], center["z"]),
        project_to_road=False, lane_type=carla.LaneType.Driving)
    require(waypoint is not None and waypoint.is_junction,
            "existing Town03 DEV turn is not on a junction")
    site["junction_id"] = int(waypoint.get_junction().id)
    site["route_sha256"] = sha256(path)
    return site


def scan_map(town: str, carla: object, planner_class: object) -> dict:
    """HH_260929 - Search the declared finite grid without a CARLA client."""
    xodr_path = XODR_ROOT / f"{town}.xodr"
    require(sha256(xodr_path) == MAP_SHA256[town], f"pinned {town} XODR changed")
    xodr = xodr_path.read_bytes()
    carla_map = carla.Map(f"offline-{town}-right-20260929", xodr.decode("utf-8"))
    planner = planner_class(carla_map, 1.0)
    roads = {int(road.attrib["id"]): road for road in
             ET.fromstring(xodr).findall("road")}
    junctions = {point.get_junction().id: point.get_junction()
                 for point in carla_map.generate_waypoints(3.0) if point.is_junction}
    old_site = old_dev_site(carla_map, carla) if town == "Town03" else None
    counts = {key: 0 for key in (
        "right_connectors", "endpoint_pairs", "unreachable_pairs", "length_at_least_146m",
        "traverses_connector", "single_right_no_other_maneuver",
        "right_block_on_selected_connector", "turn_inside_30_120m",
        "physical_turn_pass", "signal_record_free_pass", "novel_junction_pass",
        "no_other_junction_in_protected_span_pass")}
    passes = []
    physical_rejections: dict[str, int] = {}
    candidate_errors: dict[str, int] = {}
    for junction_id, junction in sorted(junctions.items()):
        for entry, exit_ in junction.get_waypoints(carla.LaneType.Driving):
            heading = (exit_.transform.rotation.yaw - entry.transform.rotation.yaw + 180) % 360 - 180
            if not 60.0 <= heading <= 120.0:
                continue
            counts["right_connectors"] += 1
            for upstream_m, downstream_m in SCOPES_M:
                for start_index, start in enumerate(entry.previous(float(upstream_m))):
                    for goal_index, goal in enumerate(exit_.next(float(downstream_m))):
                        counts["endpoint_pairs"] += 1
                        try:
                            traced = planner.trace_route(start.transform.location,
                                                         goal.transform.location)
                        except NetworkXNoPath:
                            counts["unreachable_pairs"] += 1
                            continue
                        points = route_helper.serialize_route(traced, goal.transform)
                        if len(points) < 2 or float(points[-1]["distance_m"]) < MIN_LENGTH_M:
                            continue
                        counts["length_at_least_146m"] += 1
                        if not any(point["road_id"] == entry.road_id and
                                   point["lane_id"] == entry.lane_id for point in points):
                            continue
                        counts["traverses_connector"] += 1
                        block = right_block(points)
                        if block is None:
                            continue
                        counts["single_right_no_other_maneuver"] += 1
                        if not right_block_on_connector(
                                points, block, int(entry.road_id), int(entry.lane_id)):
                            continue
                        counts["right_block_on_selected_connector"] += 1
                        turn_start = float(points[block[0]]["distance_m"])
                        turn_end = float(points[block[1]]["distance_m"])
                        if not WINDOW_M[0] <= turn_start <= turn_end <= WINDOW_M[1]:
                            continue
                        counts["turn_inside_30_120m"] += 1
                        route = {"route": points, "route_length_m": points[-1]["distance_m"],
                                 "scenario": "right", "coordinate_reference": "base_link"}
                        try:
                            result = physical.analyze_serialized_physical_turn(
                                route, physical.SPEED_30KPH_TURN_GEOMETRY_CONTRACT)
                        except physical.PhysicalTurnGeometryError as error:
                            if error.fatal:
                                raise ScoutError(f"physical contract failed: {error}") from error
                            candidate_errors[error.error_code] = candidate_errors.get(error.error_code, 0) + 1
                            continue
                        if result["status"] != "PASS":
                            for reason in result["failure_reasons"]:
                                physical_rejections[reason] = physical_rejections.get(reason, 0) + 1
                            continue
                        counts["physical_turn_pass"] += 1
                        signals = signal_records(roads, points)
                        if not any(signals.values()):
                            counts["signal_record_free_pass"] += 1
                        novel = old_site is None or int(junction_id) != old_site["junction_id"]
                        if novel:
                            counts["novel_junction_pass"] += 1
                        extra_junctions = other_junction_ids(roads, points, int(junction_id))
                        if not extra_junctions:
                            counts["no_other_junction_in_protected_span_pass"] += 1
                        site = physical_site(points, block)
                        if old_site is not None:
                            site["center_distance_to_existing_dev_m"] = math.dist(
                                site["center_xy_ros_m"], old_site["center_xy_ros_m"])
                        passes.append({
                            "junction_id": int(junction_id),
                            "connector_road_id": int(entry.road_id),
                            "connector_lane_id": int(entry.lane_id),
                            "connector_heading_deg": float(heading),
                            "upstream_m": upstream_m, "downstream_m": downstream_m,
                            "start_index": start_index, "goal_index": goal_index,
                            "start": pose(start), "goal": pose(goal),
                            "route_length_m": float(points[-1]["distance_m"]),
                            "right_turn_m": [turn_start, turn_end],
                            "route_sha256": route_digest(points),
                            **signals,
                            "other_junction_ids_in_30_146m": extra_junctions,
                            "different_junction_from_existing_dev": novel,
                            "site": site,
                        })
    passes.sort(key=lambda row: (row["junction_id"], row["connector_road_id"],
                                 row["connector_lane_id"], row["upstream_m"],
                                 row["downstream_m"], row["start_index"], row["goal_index"]))
    # HH_260929 - One fixed representative per physical connector; this is a
    # negative screen, not proof about untested endpoint variants at that site.
    groups: dict[tuple[int, int, int], list[dict]] = {}
    for row in passes:
        key = (row["junction_id"], row["connector_road_id"], row["connector_lane_id"])
        groups.setdefault(key, []).append(row)
    manifest_path = next(ROOT / relative for relative in SOURCE_SHA256
                         if relative.endswith("episode/manifest.json"))
    manifest = json.loads(manifest_path.read_text())
    require(manifest["capture_contract"]["actor_body_geometry"]["actor_type_id"] ==
            "vehicle.toyota.prius" and manifest["coordinate_contract"]["wheelbase_m"] == 2.85,
            "pinned Prius body geometry changed")
    nominal_witnesses = []
    for key, rows in sorted(groups.items()):
        selected = min(rows, key=lambda row: (
            len(row["direct_signal_road_ids"]) + len(row["signal_reference_road_ids"]),
            len(row["other_junction_ids_in_30_146m"]),
            abs(row["right_turn_m"][0] - 70.0),
            -row["route_length_m"], row["route_sha256"]))
        traced_points = None
        for entry, exit_ in junctions[key[0]].get_waypoints(carla.LaneType.Driving):
            if (entry.road_id, entry.lane_id) != key[1:]:
                continue
            starts = entry.previous(float(selected["upstream_m"]))
            goals = exit_.next(float(selected["downstream_m"]))
            if (len(starts) <= selected["start_index"] or
                    len(goals) <= selected["goal_index"]):
                continue
            start = starts[selected["start_index"]]
            goal = goals[selected["goal_index"]]
            candidate = route_helper.serialize_route(
                planner.trace_route(start.transform.location, goal.transform.location),
                goal.transform)
            if route_digest(candidate) == selected["route_sha256"]:
                traced_points = candidate
                break
        require(traced_points is not None, f"nominal witness route changed: {key}")
        route = {"route": traced_points,
                 "route_length_m": traced_points[-1]["distance_m"],
                 "scenario": "right", "coordinate_reference": "base_link"}
        try:
            geometry = nominal_helper.provisional_nominal_clearance(
                route, manifest, xodr, carla, *WINDOW_M)
        except Exception as error:
            geometry = {"status": "UNSUPPORTED", "reason": str(error),
                        "error_type": type(error).__name__}
        nominal_witnesses.append({
            "junction_id": key[0], "connector_road_id": key[1],
            "connector_lane_id": key[2],
            "route_sha256": selected["route_sha256"],
            "upstream_m": selected["upstream_m"],
            "downstream_m": selected["downstream_m"],
            "start_index": selected["start_index"],
            "goal_index": selected["goal_index"],
            "source_prius_manifest_sha256": sha256(manifest_path),
            "signals": signal_records(roads, traced_points),
            "geometry": geometry,
        })
    return {
        "town": town, "xodr_path": str(xodr_path), "xodr_sha256": MAP_SHA256[town],
        "old_dev_site": old_site, "counts": counts,
        "candidate_errors": candidate_errors, "physical_rejections": physical_rejections,
        "physical_pass_routes": passes, "nominal_witnesses": nominal_witnesses,
    }


def scan(carla: object, planner_class: object) -> dict:
    for relative, expected in SOURCE_SHA256.items():
        require(sha256(ROOT / relative) == expected, f"pinned source changed: {relative}")
    maps = [scan_map(town, carla, planner_class) for town in MAP_SHA256]
    return {
        "schema": "portable_e2e.other_non_test_right_static_scout.v3",
        "status": "BOUNDED_OFFLINE_AUDIT",
        "scope": "Town03 DEV, Town02_Opt TRAIN candidate, Town05_Opt split-conflicted DEV candidate",
        "source_sha256": SOURCE_SHA256,
        "planner_source_sha256": sha256(Path(inspect.getsourcefile(planner_class))),
        "physical_turn_profile": physical.SPEED_30KPH_TURN_GEOMETRY_CONTRACT["profile_id"],
        "right_heading_range_deg": [60, 120], "sampling_resolution_m": 1.0,
        "upstream_grid_m": [50, 60, 70, 80, 90, 100],
        "downstream_grid_m": [80, 100, 120, 140, 160],
        "fixed_window_m": list(WINDOW_M),
        "target_speed_kmh_for_distance_budget_only": SPEED_KMH,
        "minimum_route_length_m": MIN_LENGTH_M,
        "future_sec": FUTURE_SEC, "future_distance_at_9kmh_m": 16.0,
        "future_guard_m": FUTURE_GUARD_M,
        "moving_contract": {"speed_strictly_above_mps": 0.5,
                            "minimum_continuous_sec": 30.0},
        "goal_tolerance_m": 2.5,
        "maps": maps,
        "live_map_sha_verified": False, "nominal_body_corridor_verified": False,
        "continuous_moving_verified": False, "actual_goal_verified": False,
        "capture_authorized": False, "dataset_admitted": False,
        "training_executed": False, "learned_closed_loop_tested": False,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    args = parser.parse_args()
    require(not args.output.exists() and not args.output.is_symlink(),
            "output already exists")
    require(args.output.parent.is_dir() and not args.output.parent.is_symlink(),
            "create a new regular output directory before scanning")
    import carla
    from agents.navigation.global_route_planner import GlobalRoutePlanner

    report = scan(carla, GlobalRoutePlanner)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({item["town"]: item["counts"] for item in report["maps"]}, indent=2))


if __name__ == "__main__":
    main()
