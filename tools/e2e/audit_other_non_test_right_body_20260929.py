#!/usr/bin/env python3
"""HH_260929 - Audit every pinned offline physical-pass right-turn body."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from scripts.e2e import rank_town01_v5_long_turns as nominal
from scripts.e2e import scout_other_non_test_right_20260929 as scout


ROOT = Path(__file__).resolve().parents[2]
SCOUT_REPORT = scout.OUTPUT
SCOUT_REPORT_SHA256 = "0f183bbeedb09e1ec391cd13e2bc45cb09d5af8ef5d4743c7a0aef5ac6a51b2a"
SCOUT_SOURCE_SHA256 = "62cb450830571721de867527a875ae3c87803c036e6baaf1ed3c13fdaff0a4ee"
OUTPUT = ROOT / "docs/assets/validation/2026-09-29/drive_other_site_right_static/body_all_report.json"


def reconstruct(row: dict, junction: object, planner: object, carla: object) -> list[dict]:
    """HH_260929 - Require a byte-identical serialized route for each saved row."""
    for entry, exit_ in junction.get_waypoints(carla.LaneType.Driving):
        if (entry.road_id, entry.lane_id) != (
                row["connector_road_id"], row["connector_lane_id"]):
            continue
        starts = entry.previous(float(row["upstream_m"]))
        goals = exit_.next(float(row["downstream_m"]))
        if len(starts) <= row["start_index"] or len(goals) <= row["goal_index"]:
            continue
        start = starts[row["start_index"]]
        goal = goals[row["goal_index"]]
        traced = planner.trace_route(start.transform.location, goal.transform.location)
        points = scout.route_helper.serialize_route(traced, goal.transform)
        if scout.route_digest(points) == row["route_sha256"]:
            return points
    raise scout.ScoutError(f"pinned route cannot be reconstructed: {row['route_sha256']}")


def audit(carla: object, planner_class: object) -> dict:
    scout.require(scout.sha256(SCOUT_REPORT) == SCOUT_REPORT_SHA256,
                  "pinned v3 report changed")
    scout.require(scout.sha256(Path(scout.__file__)) == SCOUT_SOURCE_SHA256,
                  "pinned scout source changed")
    for relative, expected in scout.SOURCE_SHA256.items():
        scout.require(scout.sha256(ROOT / relative) == expected,
                      f"pinned source changed: {relative}")
    original = json.loads(SCOUT_REPORT.read_text())
    scout.require(original["schema"] == "portable_e2e.other_non_test_right_static_scout.v3",
                  "wrong static scout schema")
    manifest_path = next(ROOT / relative for relative in scout.SOURCE_SHA256
                         if relative.endswith("episode/manifest.json"))
    manifest = json.loads(manifest_path.read_text())
    rows = []
    for map_report in original["maps"]:
        town = map_report["town"]
        xodr_path = scout.XODR_ROOT / f"{town}.xodr"
        scout.require(scout.sha256(xodr_path) == scout.MAP_SHA256[town],
                      f"pinned {town} XODR changed")
        xodr = xodr_path.read_bytes()
        carla_map = carla.Map(f"offline-{town}-all-body-20260929", xodr.decode())
        planner = planner_class(carla_map, 1.0)
        junctions = {point.get_junction().id: point.get_junction()
                     for point in carla_map.generate_waypoints(3.0) if point.is_junction}
        for entry in map_report["physical_pass_routes"]:
            points = reconstruct(entry, junctions[entry["junction_id"]], planner, carla)
            route = {"route": points, "route_length_m": points[-1]["distance_m"],
                     "scenario": "right", "coordinate_reference": "base_link"}
            try:
                geometry = nominal.provisional_nominal_clearance(
                    route, manifest, xodr, carla, *scout.WINDOW_M)
            except Exception as error:
                geometry = {"status": "UNSUPPORTED", "error_type": type(error).__name__,
                            "reason": str(error)}
            rows.append({
                "town": town, "junction_id": entry["junction_id"],
                "connector_road_id": entry["connector_road_id"],
                "connector_lane_id": entry["connector_lane_id"],
                "upstream_m": entry["upstream_m"],
                "downstream_m": entry["downstream_m"],
                "start_index": entry["start_index"], "goal_index": entry["goal_index"],
                "route_sha256": entry["route_sha256"],
                "different_junction_from_existing_dev":
                    entry["different_junction_from_existing_dev"],
                "direct_signal_road_ids": entry["direct_signal_road_ids"],
                "signal_reference_road_ids": entry["signal_reference_road_ids"],
                "other_junction_ids_in_30_146m":
                    entry["other_junction_ids_in_30_146m"],
                "nominal": {
                    key: geometry.get(key) for key in (
                        "status", "reason", "error_type", "raw_body_outside_count",
                        "raw_sweep_outside_count", "inset_body_outside_count",
                        "inset_sweep_outside_count", "maximum_inset_outside_m2",
                        "minimum_inset_sweep_clearance_m", "production_lane_gate_supported")
                    if key in geometry
                },
            })
    expected_count = sum(map_report["counts"]["physical_turn_pass"]
                         for map_report in original["maps"])
    scout.require(len(rows) == expected_count, "body replay skipped a physical-pass route")
    status_counts = {town: {status: sum(row["town"] == town and
                                       row["nominal"]["status"] == status for row in rows)
                            for status in ("PASS", "FAIL", "UNSUPPORTED")}
                     for town in scout.MAP_SHA256}
    signal_free_nominal_pass = [row["route_sha256"] for row in rows
                                if row["nominal"]["status"] == "PASS" and
                                not row["direct_signal_road_ids"] and
                                not row["signal_reference_road_ids"]]
    return {
        "schema": "portable_e2e.other_non_test_right_all_nominal_body.v1",
        "static_scout_sha256": SCOUT_REPORT_SHA256,
        "source_prius_manifest_sha256": scout.sha256(manifest_path),
        "fixed_window_m": list(scout.WINDOW_M), "lane_inset_m": 0.10,
        "status_counts": status_counts,
        "signal_free_nominal_pass_route_sha256": signal_free_nominal_pass,
        "rows": rows,
        "live_xodr_or_body_verified": False, "actual_movement_verified": False,
        "capture_authorized": False, "dataset_admitted": False,
        "training_executed": False, "learned_closed_loop_tested": False,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    args = parser.parse_args()
    scout.require(not args.output.exists() and not args.output.is_symlink(),
                  "body report output already exists")
    scout.require(args.output.parent.is_dir() and not args.output.parent.is_symlink(),
                  "create a regular output directory first")
    import carla
    from agents.navigation.global_route_planner import GlobalRoutePlanner

    report = audit(carla, GlobalRoutePlanner)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report["status_counts"], indent=2))


if __name__ == "__main__":
    main()
