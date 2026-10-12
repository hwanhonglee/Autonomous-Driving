#!/usr/bin/env python3
"""HH_260906 - Rank distinct Town01 right turns using archived geometry only.

This offline screen does not start CARLA, grant capture, admit data, or train a model.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

from scripts.e2e import audit_future_actor_lane_containment as lane
from scripts.e2e import audit_new_long_turn_candidates as catalog_audit


ROOT = Path(__file__).resolve().parents[2]
CATALOG = ROOT / "storage/experiments/training/2026-09-27/drive_long_route_town01_train_catalog_v4"
STATIC_AUDIT = ROOT / "docs/assets/validation/2026-09-27/drive_long_route_prep/v4_candidate_audit.json"
ARCHIVED_EPISODE = ROOT / (
    "storage/experiments/training/2026-09-27/"
    "drive_long_route_town01_right_s0002_p06_9kph_steer045_purge15_trial_v1/episode"
)
OUTPUT = ROOT / "docs/assets/validation/2026-09-27/drive_next_wide_turn_rank/rank.json"
SELECT_START_M = 30.0
SELECT_END_M = 120.0
SPEED_KMH = 9.0
FUTURE_S = 6.4
SAMPLE_STEP_M = 0.10


def require(condition: bool, reason: str) -> None:
    if not condition:
        raise ValueError(reason)


def digest(path: Path) -> str:
    require(path.is_file() and not path.is_symlink(), f"missing regular file: {path}")
    return hashlib.sha256(path.read_bytes()).hexdigest()


def distinct_points(points: list[dict]) -> list[dict]:
    """HH_260906 - Remove command-only duplicate nodes before continuous interpolation."""
    output = [points[0]]
    for point in points[1:]:
        delta = float(point["distance_m"]) - float(output[-1]["distance_m"])
        require(delta >= 0, "route arc goes backwards")
        if delta == 0:
            require(all(point[key] == output[-1][key] for key in ("x", "y", "z", "yaw", "road_id", "section_id", "lane_id")),
                    "zero-length node changes geometry or lane identity")
        else:
            output.append(point)
    return output


def pose_at_arc(points: list[dict], arc: float, segment_hint: int) -> tuple[dict, int]:
    """HH_260906 - Put the actor 1.425 m ahead of the serialized rear-axle base_link."""
    index = segment_hint
    while index + 2 < len(points) and float(points[index + 1]["distance_m"]) < arc:
        index += 1
    a, b = points[index], points[index + 1]
    start, end = float(a["distance_m"]), float(b["distance_m"])
    require(start - 1e-6 <= arc <= end + 1e-6 and end > start, "arc interpolation outside route")
    t = max(0.0, min(1.0, (arc - start) / (end - start)))
    ros_yaw_a, ros_yaw_b = float(a["yaw"]), float(b["yaw"])
    yaw_delta = (ros_yaw_b - ros_yaw_a + math.pi) % (2.0 * math.pi) - math.pi
    carla_yaw = -(ros_yaw_a + t * yaw_delta)
    rear_x = float(a["x"]) + t * (float(b["x"]) - float(a["x"]))
    rear_y = -(float(a["y"]) + t * (float(b["y"]) - float(a["y"])))
    return ({"x": rear_x + 1.425 * math.cos(carla_yaw),
             "y": rear_y + 1.425 * math.sin(carla_yaw), "z": 0.0,
             "roll": 0.0, "pitch": 0.0, "yaw": math.degrees(carla_yaw)}, index)


def sampled_nominal_clearance(route: dict, manifest: dict, xodr: bytes, carla: object) -> dict:
    """HH_260906 - Check sampled nominal actor bodies and conservative short sweeps."""
    points = distinct_points(route["route"])
    require(float(points[-1]["distance_m"]) >= SELECT_END_M + SPEED_KMH / 3.6 * FUTURE_S,
            "30-120 m window lacks a 9 km/h 6.4 s route tail")
    carla_map = carla.Map("archived-active-Town01", xodr.decode("utf-8"))
    box, radius = lane._body_box(manifest, carla)
    lo, hi = SELECT_START_M - radius, SELECT_END_M + radius
    relevant = {node for i, (a, b) in enumerate(zip(points, points[1:]))
                if float(a["distance_m"]) <= hi and float(b["distance_m"]) >= lo
                for node in (i, i + 1)}
    requested = {lane._triple(points[index]) for index in relevant}
    corridors = lane.lane_corridors(carla_map, carla, points, xodr, requested, relevant)
    regions: dict[tuple[frozenset[tuple[int, int, int]], float], object] = {}

    def allowed(lanes: set[tuple[int, int, int]], inset: float) -> object:
        key = frozenset(lanes), inset
        if key not in regions:
            regions[key] = lane.authorized_lane_region(corridors, lanes, inset)
        return regions[key]

    count = math.ceil((SELECT_END_M - SELECT_START_M) / SAMPLE_STEP_M)
    arcs = [SELECT_START_M + (SELECT_END_M - SELECT_START_M) * i / count for i in range(count + 1)]
    poses, bodies, index = [], [], 0
    for arc in arcs:
        pose, index = pose_at_arc(points, arc, index)
        poses.append(pose)
        bodies.append(lane.body_footprint(box, pose, carla))
    result = {"status": "PASS", "scope": "nominal route centerline, sampled 0.10 m; not a driven trajectory",
              "sample_step_m": SAMPLE_STEP_M, "actor_rear_axle_offset_m": 1.425,
              "body_count": len(bodies), "sweep_count": len(bodies) - 1,
              "raw_body_outside_count": 0, "inset_body_outside_count": 0,
              "raw_sweep_outside_count": 0, "inset_sweep_outside_count": 0,
              "maximum_raw_outside_m2": 0.0, "maximum_inset_outside_m2": 0.0,
              "minimum_raw_body_clearance_m": math.inf, "minimum_inset_body_clearance_m": math.inf,
              "minimum_raw_sweep_clearance_m": math.inf, "minimum_inset_sweep_clearance_m": math.inf,
              "worst_inset_arc_m": None, "failure_reasons": [],
              "physical_continuous_time_proven": False, "capture_authorized": False}

    def record(shape: object, arc: float, kind: str, lanes: set[tuple[int, int, int]]) -> None:
        for inset, prefix in ((0.0, "raw"), (lane.LANE_INSET_M, "inset")):
            region = allowed(lanes, inset)
            outside = float(shape.difference(region).area)
            if outside > 1e-6:
                result[f"{prefix}_{kind}_outside_count"] += 1
                if inset == lane.LANE_INSET_M and outside > result["maximum_inset_outside_m2"]:
                    result["worst_inset_arc_m"] = arc
            key = "maximum_raw_outside_m2" if inset == 0.0 else "maximum_inset_outside_m2"
            result[key] = max(float(result[key]), outside)
            margin = 0.0 if outside > 1e-6 else float(shape.distance(region.boundary))
            clearance_key = f"minimum_{prefix}_{kind}_clearance_m"
            result[clearance_key] = min(float(result[clearance_key]), margin)

    for arc, body in zip(arcs, bodies):
        identities = lane._route_lanes_for_arc_interval(points, arc - radius, arc + radius)
        record(body, arc, "body", identities)
    for index, (a, b) in enumerate(zip(bodies, bodies[1:])):
        identities = lane._route_lanes_for_arc_interval(points, arcs[index] - radius,
                                                         arcs[index + 1] + radius)
        sweep = lane.swept_body(a, b, poses[index], poses[index + 1], radius)
        record(sweep, (arcs[index] + arcs[index + 1]) / 2.0, "sweep", identities)
    if any(result[f"{prefix}_{kind}_outside_count"] for prefix in ("raw", "inset")
           for kind in ("body", "sweep")):
        result["status"] = "FAIL"
        result["failure_reasons"].append("nominal_body_or_sweep_exceeds_sampled_intended_lane")
    return result


def rank(output: Path) -> dict:
    """HH_260906 - Re-bind archived sources and fail closed on missing physical evidence."""
    import carla

    static = json.loads(STATIC_AUDIT.read_text())
    owner = json.loads((CATALOG / "owner_result.json").read_text())
    require(static.get("status") == "STATIC_REVIEW_ONLY" and
            digest(CATALOG / "route_catalog.json") == static.get("catalog_sha256") == owner.get("catalog_sha256"),
            "static catalog provenance changed")
    manifest_path = ARCHIVED_EPISODE / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    source_map = manifest["capture_contract"]["active_map_opendrive"]
    require(source_map.get("map_name") == "Carla/Maps/Town01" and
            source_map.get("status") == "RECORDED" and
            digest(ARCHIVED_EPISODE / "active_map_opendrive.xodr") == source_map.get("sha256"),
            "archived active Town01 XODR missing or changed")
    require(manifest["coordinate_contract"]["wheelbase_m"] == 2.85 and
            manifest["capture_contract"]["actor_body_geometry"]["actor_type_id"] == "vehicle.toyota.prius",
            "pinned Prius body/rear-axle geometry differs")
    xodr = (ARCHIVED_EPISODE / "active_map_opendrive.xodr").read_bytes()
    results = []
    for item in static["candidates"]:
        route_path = ROOT / item["path"]
        require(digest(route_path) == item["sha256"], f"catalog route changed: {item['route_id']}")
        route = json.loads(route_path.read_text())
        geometry = catalog_audit.turn_geometry(route)
        block = route["physical_turn_preflight"]["selected_block"]
        turn_end = float(route["route"][block["end_index"]]["distance_m"])
        selected_turn = SELECT_START_M <= geometry["lead_m"] and turn_end <= SELECT_END_M
        future_tail = geometry["length_m"] >= SELECT_END_M + SPEED_KMH / 3.6 * FUTURE_S
        same_as_p06 = geometry["transitions"] == [(208, 0, -1), (12, 0, 1)]
        same_as_train = item["same_turn_transitions_as_train"] or \
            item["turn_center_distance_to_train_m"] < 30.0 or \
            item["fraction_waypoints_within_2m_of_train"] >= 0.2
        exclusion = []
        if not selected_turn:
            exclusion.append("turn_not_fully_inside_fixed_30_to_120_m_window")
        if geometry["lead_m"] < SELECT_START_M:
            exclusion.append("less_than_30_m_pre_turn_lead")
        if not future_tail:
            exclusion.append("9_kmh_6p4_s_future_tail_unavailable")
        if same_as_train:
            exclusion.append("historical_train_turn_or_nearby_site_reuse")
        if same_as_p06:
            exclusion.append("same_physical_turn_as_existing_p06_expert_capture")
        row = {"route_id": item["route_id"], "route_sha256": item["sha256"],
               "physical_turn_transitions": [list(triple) for triple in geometry["transitions"]],
               "turn_center_xy_ros_m": list(geometry["center"]),
               "turn_start_m": geometry["lead_m"], "turn_end_m": turn_end,
               "pre_turn_selected_context_m": max(0.0, geometry["lead_m"] - SELECT_START_M),
               "post_turn_selected_context_m": max(0.0, SELECT_END_M - turn_end),
               "route_future_tail_after_window_m": geometry["length_m"] - SELECT_END_M,
               "exclusion_reasons": exclusion, "nominal": None,
               "capture_authorized": False, "training_data_approved": False}
        if not exclusion:
            try:
                row["nominal"] = sampled_nominal_clearance(route, manifest, xodr, carla)
            except Exception as error:
                row["nominal"] = {"status": "UNSUPPORTED", "reason": str(error),
                                  "capture_authorized": False}
                row["exclusion_reasons"].append("nominal_geometry_unsupported")
            else:
                if row["nominal"]["status"] != "PASS":
                    row["exclusion_reasons"].append("nominal_body_or_sweep_exceeds_intended_lane")
        results.append(row)
    contenders = [row for row in results if not row["exclusion_reasons"]]
    # HH_260906 - The same road/lane transition and turn center is one physical site, not two training episodes.
    groups = {}
    for row in contenders:
        key = tuple(map(tuple, row["physical_turn_transitions"]))
        groups.setdefault(str(key), []).append(row)
    ranked = []
    for members in groups.values():
        members.sort(key=lambda row: (-row["nominal"]["minimum_inset_sweep_clearance_m"],
                                      -row["pre_turn_selected_context_m"], row["route_id"]))
        winner = members[0]
        ranked.append({"physical_turn_transitions": winner["physical_turn_transitions"],
                       "preferred_route_id": winner["route_id"],
                       "same_site_alternatives": [row["route_id"] for row in members[1:]],
                       "status": "OFFLINE_PREREGISTRATION_CANDIDATE_ONLY",
                       "capture_authorized": False})
    ranked.sort(key=lambda group: group["preferred_route_id"])
    report = {"schema": "portable_e2e.town01_wide_turn_rank.v1",
              "status": "OFFLINE_PREREGISTRATION_ONLY" if ranked else "NO_NOMINAL_SAFE_NOVEL_CANDIDATE",
              "map_sha256": source_map["sha256"], "manifest_sha256": digest(manifest_path),
              "static_audit_sha256": digest(STATIC_AUDIT),
              "actor_type_id": "vehicle.toyota.prius", "wheelbase_m": 2.85,
              "selected_route_window_m": [SELECT_START_M, SELECT_END_M],
              "future_requirement_s": FUTURE_S,
              "candidate_count": len(results), "distinct_offline_ranked_site_count": len(ranked),
              "ranked_sites": ranked, "candidates": results,
              "capture_executed": False, "dataset_admitted": False,
              "training_executed": False, "learned_driving_tested": False}
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    args = parser.parse_args()
    report = rank(args.output)
    print(json.dumps({key: report[key] for key in ("status", "candidate_count",
                                               "distinct_offline_ranked_site_count", "ranked_sites")}, indent=2))


if __name__ == "__main__":
    main()
