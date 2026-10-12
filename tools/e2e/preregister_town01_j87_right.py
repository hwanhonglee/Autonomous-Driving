#!/usr/bin/env python3
"""HH_260928 - Reconstruct one offline Town01 right-turn route without starting CARLA."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

from scripts.e2e import physical_turn_geometry as physical
from scripts.e2e import prepare_carla_route as route_helper
from scripts.e2e import rank_town01_v5_long_turns as previous_rank
from scripts.e2e import rank_town01_wide_turns as nominal


ROOT = Path(__file__).resolve().parents[2]
ARCHIVED = nominal.ARCHIVED_EPISODE
XODR = ARCHIVED / "active_map_opendrive.xodr"
MANIFEST = ARCHIVED / "manifest.json"
XODR_SHA256 = "97a7f6ac67812567e5c8ee0599cd823b23f80f30f3f97c502212e38b72e2b709"
MANIFEST_SHA256 = "9258274bd2c426c186db2b811fca0e546b7423fcec75c11d41702c8bea14dbb8"
PRIOR_RANKS = (
    (ROOT / "docs/assets/validation/2026-09-27/drive_town01_v5_rank/rank.json",
     "534b920d41769ccd625d94116f31ee2f5ee359446a30b492e7538f0414bc6f6b"),
    (ROOT / "docs/assets/validation/2026-09-27/drive_town01_multi_rank/rank.json",
     "8638e57a83924e70c0d36db43bcc75fcec0904210df27de61478716623e073d0"),
)
OUTPUT = ROOT / "docs/assets/validation/2026-09-28/drive_town01_j87_prereg"
JUNCTION_ID = 87
CONNECTOR_ROAD_ID = 103
CONNECTOR_LANE_ID = -1
UPSTREAM_M = 60.0
DOWNSTREAM_M = 80.0
WINDOW_START_M = 30.0
WINDOW_END_M = 120.0
FUTURE_M = 9.0 / 3.6 * 6.4
FUTURE_GUARD_M = 10.0


class PreflightError(ValueError):
    """HH_260928 - Refuse ambiguous geometry or changed frozen evidence."""


def require(condition: bool, reason: str) -> None:
    if not condition:
        raise PreflightError(reason)


def sha256(path: Path) -> str:
    require(path.is_file() and not path.is_symlink(), f"regular source required: {path}")
    return hashlib.sha256(path.read_bytes()).hexdigest()


def turn_fingerprint(route: dict, preflight: dict) -> tuple[tuple[tuple[int, int, int], ...], tuple[float, float]]:
    """HH_260928 - Key a physical turn by its connector lanes and route-center position."""
    block = preflight.get("selected_block")
    require(isinstance(block, dict), "missing selected turn block")
    points = route["route"][block["start_index"]:block["end_index"] + 1]
    require(bool(points) and all(point["road_option"] == "RIGHT" for point in points),
            "selected turn block is not a right turn")
    lanes: list[tuple[int, int, int]] = []
    for point in points:
        lane = tuple(point[key] for key in ("road_id", "section_id", "lane_id"))
        require(all(type(value) is int for value in lane), "turn lane identity missing")
        if not lanes or lanes[-1] != lane:
            lanes.append(lane)
    center = points[len(points) // 2]
    return tuple(lanes), (float(center["x"]), float(center["y"]))


def choose_connector(carla_map: object, carla: object) -> tuple[object, object]:
    """HH_260928 - Select a road/lane connector, never a random endpoint seed."""
    junctions = {waypoint.get_junction().id: waypoint.get_junction()
                 for waypoint in carla_map.generate_waypoints(3.0) if waypoint.is_junction}
    require(JUNCTION_ID in junctions, "pinned Town01 junction disappeared")
    pairs = [(entry, exit_) for entry, exit_ in
             junctions[JUNCTION_ID].get_waypoints(carla.LaneType.Driving)
             if entry.road_id == CONNECTOR_ROAD_ID and entry.lane_id == CONNECTOR_LANE_ID]
    require(len(pairs) == 1, "pinned right-turn connector is ambiguous")
    entry, exit_ = pairs[0]
    heading = (exit_.transform.rotation.yaw - entry.transform.rotation.yaw + 180) % 360 - 180
    require(60.0 <= heading <= 120.0, "pinned connector no longer turns right")
    return entry, exit_


def prior_search_sites() -> list[tuple[tuple[tuple[int, int, int], ...], tuple[float, float]]]:
    """HH_260928 - Include V5 and multi-window sites, even if their captures were rejected."""
    sites = []
    for path, expected in PRIOR_RANKS:
        require(sha256(path) == expected, f"previous search report changed: {path}")
        report = json.loads(path.read_text())
        require(report.get("schema") in (
            "portable_e2e.town01_v5_long_turn_rank.v1",
            "portable_e2e.town01_multi_turn_window_rank.v1",
        ), "previous search report schema changed")
        for row in report["routes"]:
            if report["schema"] == "portable_e2e.town01_v5_long_turn_rank.v1":
                lanes = row["physical_turn_transitions"]
                center = row["physical_turn_center_xy_ros_m"]
            else:
                site = row["selected_site"]
                if site is None:
                    continue
                lanes = site["turn_lane_transitions"]
                center = site["turn_center_xy_ros_m"]
            sites.append((tuple(tuple(int(value) for value in lane) for lane in lanes),
                          (float(center[0]), float(center[1]))))
    return sites


def reconstruct(carla: object, planner_class: object) -> tuple[dict, dict]:
    """HH_260928 - Build and screen one exact route using an archived live XODR only."""
    require(sha256(XODR) == XODR_SHA256, "archived live Town01 XODR changed")
    require(sha256(MANIFEST) == MANIFEST_SHA256, "archived Prius body manifest changed")
    xodr = XODR.read_bytes()
    manifest = json.loads(MANIFEST.read_text())
    require(manifest["capture_contract"]["actor_body_geometry"]["actor_type_id"] ==
            "vehicle.toyota.prius" and manifest["coordinate_contract"]["wheelbase_m"] == 2.85,
            "body geometry is not the pinned Prius")
    carla_map = carla.Map("archived-live-Town01-j87-offline", xodr.decode("utf-8"))
    entry, exit_ = choose_connector(carla_map, carla)
    starts, goals = entry.previous(UPSTREAM_M), exit_.next(DOWNSTREAM_M)
    require(len(starts) == len(goals) == 1, "upstream or downstream endpoint branches")
    start, goal = starts[0], goals[0]
    planner = planner_class(carla_map, 1.0)
    traced = planner.trace_route(start.transform.location, goal.transform.location)
    require(len(traced) >= 2, "planner produced no physical route")
    points = route_helper.serialize_route(traced, goal.transform)
    require(points[0]["road_id"] == start.road_id and points[0]["lane_id"] == start.lane_id,
            "planner start changed lane")
    route = {
        "schema_version": 1,
        "coordinate_reference": "base_link",
        "spawn_point_reference": "base_link",
        "town": "Town01",
        "weather": "ClearNoon",
        "scenario": "right",
        "sampling_resolution_m": 1.0,
        "route_length_m": points[-1]["distance_m"],
        "start_carla_transform": route_helper.transform_dict(start.transform),
        "start_ros_pose": route_helper.ros_pose_dict(start.transform),
        "goal_carla_transform": route_helper.transform_dict(goal.transform),
        "goal_ros_pose": route_helper.ros_pose_dict(goal.transform),
        "spawn_point": ",".join(f"{value:.6f}" for value in (
            start.transform.location.x, start.transform.location.y, start.transform.location.z,
            start.transform.rotation.roll, start.transform.rotation.pitch,
            start.transform.rotation.yaw)),
        "option_counts": {name: sum(point["road_option"] == name for point in points)
                          for name in sorted({point["road_option"] for point in points})},
        "route": points,
    }
    preflight = physical.analyze_serialized_physical_turn(
        route, physical.SPEED_30KPH_TURN_GEOMETRY_CONTRACT)
    require(preflight["status"] == "PASS", "unchanged physical-turn geometry gate failed")
    route["physical_turn_preflight"] = preflight
    fingerprint = turn_fingerprint(route, preflight)
    require(fingerprint[0] == ((103, 0, -1), (5, 0, 1)), "right-turn lane sequence changed")
    known, _, _ = previous_rank.known_sites()
    require(not any(previous_rank.same_site(fingerprint, item["fingerprint"]) for item in known),
            "turn matches historical TRAIN, V4, or captured C/p06 site")
    prior = prior_search_sites()
    require(not any(previous_rank.same_site(fingerprint, item) for item in prior),
            "turn matches a previous V5 or multi-window search site")
    nearest_prior_center_m = min(math.dist(fingerprint[1], item[1]) for item in prior)
    block = preflight["selected_block"]
    turn_start = float(points[block["start_index"]]["distance_m"])
    turn_end = float(points[block["end_index"]]["distance_m"])
    length = float(route["route_length_m"])
    require(WINDOW_START_M <= turn_start <= turn_end <= WINDOW_END_M and
            length >= WINDOW_END_M + FUTURE_M + FUTURE_GUARD_M,
            "fixed turn window or 6.4 s future plus 10 m guard unavailable")
    extra_junction = any(point["is_junction"] and
                         turn_end + 5.0 < float(point["distance_m"]) <=
                         WINDOW_END_M + FUTURE_M + FUTURE_GUARD_M
                         for point in points)
    require(not extra_junction, "another junction intersects the protected future")
    nominal_result = previous_rank.provisional_nominal_clearance(
        route, manifest, xodr, carla, WINDOW_START_M, WINDOW_END_M)
    require(nominal_result["status"] == "PASS" and
            nominal_result["production_lane_gate_supported"] is True and
            all(nominal_result[f"{scope}_{kind}_outside_count"] == 0
                for scope in ("raw", "inset") for kind in ("body", "sweep")),
            "unchanged nominal 10 cm inset body/sweep gate failed")
    require(math.isfinite(nominal_result["minimum_inset_sweep_clearance_m"]),
            "nominal sweep margin is not finite")
    # HH_260928 - This marker is not an ownership receipt or permission to capture.
    route["offline_preregistration"] = {
        "status": "OFFLINE_PROVISIONAL_ONLY",
        "archived_live_town01_xodr_sha256": XODR_SHA256,
        "source_prius_manifest_sha256": MANIFEST_SHA256,
        "junction_id": JUNCTION_ID,
        "connector_road_id": CONNECTOR_ROAD_ID,
        "connector_lane_id": CONNECTOR_LANE_ID,
        "upstream_m": UPSTREAM_M,
        "downstream_m": DOWNSTREAM_M,
        "capture_authorized": False,
        "dataset_admitted": False,
    }
    report = {
        "schema": "portable_e2e.town01_j87_offline_prereg.v1",
        "status": "STATIC_FEASIBLE_OWNER_REQUIRED",
        "split": "TRAIN_CANDIDATE_TOWN01_ONLY",
        "source_map_sha256": XODR_SHA256,
        "source_body_manifest_sha256": MANIFEST_SHA256,
        "endpoint_policy": "junction87 road103 lane-1 previous(60m)[0] to next(80m)[0]",
        "start_carla_transform": route["start_carla_transform"],
        "goal_carla_transform": route["goal_carla_transform"],
        "route_length_m": length,
        "physical_turn_transitions": [list(lane) for lane in fingerprint[0]],
        "turn_center_xy_ros_m": list(fingerprint[1]),
        "turn_start_m": turn_start,
        "turn_end_m": turn_end,
        "fixed_window_m": [WINDOW_START_M, WINDOW_END_M],
        "future_distance_at_9kmh_m": FUTURE_M,
        "future_guard_m": FUTURE_GUARD_M,
        "protected_future_has_extra_junction": extra_junction,
        "known_site_count_compared": len(known),
        "prior_search_route_sites_compared": len(prior),
        "nearest_previous_screened_turn_center_m": nearest_prior_center_m,
        "independent_junction_certified": False,
        "nominal": nominal_result,
        "capture_authorized": False,
        "dataset_admitted": False,
        "training_executed": False,
        "learned_closed_loop_tested": False,
    }
    return route, report


def publish(output: Path) -> dict:
    """HH_260928 - Write a new provisional route and report without touching earlier evidence."""
    import carla
    from agents.navigation.global_route_planner import GlobalRoutePlanner

    require(not output.exists() and not output.is_symlink(), "prereg output already exists")
    route, report = reconstruct(carla, GlobalRoutePlanner)
    route_bytes = (json.dumps(route, ensure_ascii=False, indent=2) + "\n").encode()
    report["provisional_route_sha256"] = hashlib.sha256(route_bytes).hexdigest()
    output.mkdir(parents=True)
    (output / "provisional_route.json").write_bytes(route_bytes)
    (output / "feasibility.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    args = parser.parse_args()
    report = publish(args.output)
    print(json.dumps({key: report[key] for key in (
        "status", "provisional_route_sha256", "route_length_m", "physical_turn_transitions",
    )}, indent=2))


if __name__ == "__main__":
    main()
