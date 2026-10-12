#!/usr/bin/env python3
"""HH_260929 - Reconstruct only pinned Town12 junction 3742 during V4 scan."""

from __future__ import annotations

import inspect
import json
import xml.etree.ElementTree as ET
from pathlib import Path

from networkx.exception import NetworkXNoPath

from scripts.e2e import scout_town12_right_static_20260929 as base
from scripts.e2e import scout_town12_right_static_20260929_v2 as v2
from scripts.e2e import scout_town12_right_static_20260929_v3 as v3
from scripts.e2e import scout_town12_right_static_20260929_v4 as v4


ROOT = Path(__file__).resolve().parents[2]
DIR = ROOT / "docs/assets/validation/2026-09-29/drive_town12_right_static"
PREREG = DIR / "preregister_junction3742_derivative.json"
OUTPUT = DIR / "junction3742_derivative.json"
JUNCTION_ID = 3742
COUNT_KEYS = (
    "right_connectors", "endpoint_pairs", "unreachable_pairs", "length_at_least_146m",
    "traverses_connector", "single_right_no_other_maneuver",
    "right_block_on_selected_connector", "turn_inside_30_120m", "physical_turn_pass",
    "nominal_pass", "nominal_fail", "nominal_unsupported",
    "signal_record_free_nominal_pass", "no_other_junction_nominal_pass",
)


def preflight() -> tuple[dict, dict]:
    """HH_260929 - Fail closed on the frozen inputs and ledger-row identity."""
    manifest = base.preregistered_inputs()
    base.require(PREREG.is_file() and not PREREG.is_symlink(), "derivative prereg missing")
    plan = json.loads(PREREG.read_text())
    base.require(plan.get("schema") == "portable_e2e.town12_junction3742_derivative_preregister.v1" and
                 plan.get("junction_id") == JUNCTION_ID and
                 plan.get("output_path") == str(OUTPUT.relative_to(ROOT)) and
                 plan.get("source_xodr_sha256") == base.digest(base.SOURCE_XODR) and
                 plan.get("packaged_xodr_sha256") == base.digest(base.PACKAGED_XODR) and
                 plan.get("planner_sha256") == base.digest(base.SOURCE_PLANNER) and
                 plan.get("carla_egg_sha256") == base.digest(base.SOURCE_EGG) and
                 plan.get("base_prereg_sha256") == base.digest(base.PREREG) and
                 plan.get("v4_prereg_sha256") == base.digest(v4.PREREG) and
                 plan.get("base_script_sha256") == base.digest(Path(base.__file__)) and
                 plan.get("v2_script_sha256") == base.digest(Path(v2.__file__)) and
                 plan.get("v3_script_sha256") == base.digest(Path(v3.__file__)) and
                 plan.get("v4_script_sha256") == base.digest(Path(v4.__file__)) and
                 plan.get("derivative_script_sha256") == base.digest(Path(__file__)) and
                 plan.get("lane_helper_sha256") ==
                 base.digest(ROOT / "tools/e2e/audit_future_actor_lane_containment.py") and
                 plan.get("nominal_helper_sha256") ==
                 base.digest(ROOT / "tools/e2e/rank_town01_v5_long_turns.py") and
                 plan.get("scope_m") == [list(scope) for scope in base.SCOPES_M] and
                 plan.get("window_m") == list(base.WINDOW_M) and
                 plan.get("minimum_length_m") == base.MIN_LENGTH_M and
                 plan.get("lane_inset_m") == 0.1 and
                 plan.get("seam_tolerance_m") == 0.005 and
                 plan.get("physical_profile") == "carla_vad_30kph_v2" and
                 plan.get("sample_resolution_m") == 1.0 and
                 plan.get("wall_cap_seconds") == 300 and
                 plan.get("derivative_partial_only") is True and
                 plan.get("capture_authorized") is False and
                 set(plan.get("expected_count_delta", {})) == set(COUNT_KEYS),
                 "derivative source, scope, or safety contract changed")
    base.require(not OUTPUT.exists() and not OUTPUT.is_symlink(),
                 "derivative output must be fresh")
    progress = v4.PROGRESS
    base.require(progress.is_file() and not progress.is_symlink(),
                 "V4 progress ledger unavailable")
    lines = progress.read_bytes().splitlines(keepends=True)
    base.require(len(lines) >= 69 and
                 base.digest(base.SOURCE_XODR) == plan["source_xodr_sha256"],
                 "pinned junction not yet completed")
    rows = [json.loads(lines[index]) for index in (67, 68)]
    base.require(base.hashlib.sha256(lines[67]).hexdigest() == plan.get("row68_sha256") and
                 base.hashlib.sha256(lines[68]).hexdigest() == plan.get("row69_sha256") and
                 rows[0].get("completed_junctions") == 68 and
                 rows[1].get("completed_junctions") == 69 and
                 rows[1].get("last_completed_junction_id") == JUNCTION_ID and
                 rows[1].get("total_junctions") == 628 and
                 rows[1].get("partial_not_qualifying_evidence") is True and
                 all(rows[1]["cumulative_counts"][key] -
                     rows[0]["cumulative_counts"][key] == plan["expected_count_delta"][key]
                     for key in COUNT_KEYS),
                 "pinned row-69 count delta changed")
    return manifest, plan


def reconstruct(carla: object, planner_class: object, manifest: dict,
                cached_map: v3.CachedCarlaMap, bounds_cache: v4.SectionBoundsCache,
                expected: dict) -> dict:
    """HH_260929 - Use V4's exact gates on one completed junction only."""
    base.require(Path(inspect.getsourcefile(planner_class)).resolve() == base.SOURCE_PLANNER,
                 "wrong GlobalRoutePlanner imported")
    base.require(str(carla.__file__).startswith(str(base.SOURCE_EGG) + "/"),
                 "wrong CARLA Python API imported")
    xodr = base.SOURCE_XODR.read_bytes()
    carla_map = cached_map.Map("offline-Town12-right-20260929", xodr.decode("utf-8"))
    planner = planner_class(carla_map, 1.0)
    roads = {int(road.attrib["id"]): road for road in ET.fromstring(xodr).findall("road")}
    junctions = {point.get_junction().id: point.get_junction()
                 for point in carla_map.generate_waypoints(3.0) if point.is_junction}
    base.require(len(junctions) == 628 and JUNCTION_ID in junctions,
                 "registered junction topology changed")
    junction = junctions[JUNCTION_ID]
    counts = {key: 0 for key in COUNT_KEYS}
    rows = []
    from agents.navigation.local_planner import RoadOption

    for entry, exit_ in junction.get_waypoints(carla.LaneType.Driving):
        heading = (exit_.transform.rotation.yaw - entry.transform.rotation.yaw + 180) % 360 - 180
        if not 60.0 <= heading <= 120.0:
            continue
        counts["right_connectors"] += 1
        for upstream_m, downstream_m in base.SCOPES_M:
            for start_index, start in enumerate(entry.previous(float(upstream_m))):
                for goal_index, goal in enumerate(exit_.next(float(downstream_m))):
                    counts["endpoint_pairs"] += 1
                    planner._intersection_end_node = -1
                    planner._previous_decision = RoadOption.VOID
                    try:
                        traced = planner.trace_route(start.transform.location,
                                                     goal.transform.location)
                    except NetworkXNoPath:
                        counts["unreachable_pairs"] += 1
                        continue
                    points = base.route_helper.serialize_route(traced, goal.transform)
                    if len(points) < 2 or float(points[-1]["distance_m"]) < base.MIN_LENGTH_M:
                        continue
                    counts["length_at_least_146m"] += 1
                    if not any((point["road_id"], point["section_id"], point["lane_id"]) ==
                               (entry.road_id, entry.section_id, entry.lane_id)
                               for point in points):
                        continue
                    counts["traverses_connector"] += 1
                    block = base.previous.right_block(points)
                    if block is None:
                        continue
                    counts["single_right_no_other_maneuver"] += 1
                    if not base.connector_on_block(points, block, entry):
                        continue
                    counts["right_block_on_selected_connector"] += 1
                    turn_start = float(points[block[0]]["distance_m"])
                    turn_end = float(points[block[1]]["distance_m"])
                    if not base.WINDOW_M[0] <= turn_start <= turn_end <= base.WINDOW_M[1]:
                        continue
                    counts["turn_inside_30_120m"] += 1
                    route = {"route": points, "route_length_m": points[-1]["distance_m"],
                             "scenario": "right", "coordinate_reference": "base_link"}
                    try:
                        geometry = base.physical.analyze_serialized_physical_turn(
                            route, base.physical.SPEED_30KPH_TURN_GEOMETRY_CONTRACT)
                    except base.physical.PhysicalTurnGeometryError as error:
                        if error.fatal:
                            raise base.ScoutError(f"physical contract failed: {error}") from error
                        continue
                    if geometry["status"] != "PASS":
                        continue
                    counts["physical_turn_pass"] += 1
                    signals = base.previous.signal_records(roads, points)
                    approach_signals = base.previous.signal_records(roads, points[:block[0] + 1])
                    other_junctions = base.previous.other_junction_ids(roads, points, JUNCTION_ID)
                    try:
                        body = base.nominal.provisional_nominal_clearance(
                            route, manifest, xodr, cached_map, *base.WINDOW_M)
                    except (ValueError, RuntimeError, KeyError) as error:
                        body = {"status": "UNSUPPORTED", "error_type": type(error).__name__,
                                "reason": str(error)}
                    base.require(body.get("status") in {"PASS", "FAIL", "UNSUPPORTED"},
                                 "nominal sweep returned unknown status")
                    counts[{"PASS": "nominal_pass", "FAIL": "nominal_fail",
                            "UNSUPPORTED": "nominal_unsupported"}[body["status"]]] += 1
                    if body["status"] == "PASS" and not any(signals.values()):
                        counts["signal_record_free_nominal_pass"] += 1
                    if body["status"] == "PASS" and not other_junctions:
                        counts["no_other_junction_nominal_pass"] += 1
                    rows.append({
                        "junction_id": JUNCTION_ID,
                        "connector_road_id": int(entry.road_id),
                        "connector_section_id": int(entry.section_id),
                        "connector_lane_id": int(entry.lane_id),
                        "connector_heading_deg": float(heading),
                        "upstream_m": upstream_m, "downstream_m": downstream_m,
                        "start_index": start_index, "goal_index": goal_index,
                        "start": base.previous.pose(start), "goal": base.previous.pose(goal),
                        "route_length_m": float(points[-1]["distance_m"]),
                        "right_turn_m": [turn_start, turn_end],
                        "route_sha256": base.previous.route_digest(points),
                        "site": base.previous.physical_site(points, block),
                        "full_route_signals": signals,
                        "approach_signals": approach_signals,
                        "other_junction_ids_in_30_146m": other_junctions,
                        "nominal": {key: body.get(key) for key in (
                            "status", "reason", "error_type", "body_count", "sweep_count",
                            "raw_body_outside_count", "raw_sweep_outside_count",
                            "inset_body_outside_count", "inset_sweep_outside_count",
                            "maximum_inset_outside_m2", "minimum_inset_sweep_clearance_m",
                            "production_lane_gate_supported") if key in body},
                        "route_xy_ros_m": [[float(point["x"]), float(point["y"])]
                                           for point in points],
                    })
    rows.sort(key=lambda row: (row["junction_id"], row["connector_road_id"],
                               row["connector_section_id"], row["connector_lane_id"],
                               row["upstream_m"], row["downstream_m"], row["start_index"],
                               row["goal_index"]))
    base.require(counts == expected and len(rows) == counts["physical_turn_pass"] and
                 counts["nominal_pass"] + counts["nominal_fail"] +
                 counts["nominal_unsupported"] == counts["physical_turn_pass"],
                 "focused result differs from pinned V4 junction delta")
    nominal_pass_rows = [row for row in rows if row["nominal"]["status"] == "PASS"]
    full_record_free = sum(not any(row["full_route_signals"].values())
                           for row in nominal_pass_rows)
    approach_record_free = sum(not any(row["approach_signals"].values())
                               for row in nominal_pass_rows)
    connector_keys = sorted({(row["junction_id"], row["connector_road_id"],
                              row["connector_section_id"], row["connector_lane_id"])
                             for row in rows})
    return {
        "schema": "portable_e2e.town12_junction3742_derivative.v1",
        "status": "DERIVATIVE_PARTIAL_NOT_FULL_SCOUT",
        "junction_id": JUNCTION_ID,
        "source_xodr_sha256": base.digest(base.SOURCE_XODR),
        "preregistration_sha256": base.digest(PREREG),
        "script_sha256": base.digest(Path(__file__)),
        "v4_progress_row69_sha256": json.loads(PREREG.read_text())["row69_sha256"],
        "counts": counts,
        "physical_pass_routes": rows,
        "unique_physical_site_count": len({row["junction_id"] for row in rows}),
        "physical_site_count_definition":
            "unique selected junction IDs, not endpoint pairs or independent DEV sites",
        "unique_connector_lane_section_count": len(connector_keys),
        "connector_lane_section_keys": connector_keys,
        "nominal_pass_signal_record_summary": {
            "nominal_pass_route_count": len(nominal_pass_rows),
            "full_route_record_free_count": full_record_free,
            "approach_record_free_count": approach_record_free,
            "approach_record_present_count": len(nominal_pass_rows) - approach_record_free,
            "interpretation": "OpenDRIVE road signal-record presence only; no live signal state, "
                              "stop policy, site qualification, or capture authorization",
        },
        "map_reuse": {"map_requests": cached_map.requests, "map_constructions": 1},
        "section_bounds_cache": {"requests": bounds_cache.requests,
                                 "original_parses": bounds_cache.parse_count},
        "capture_authorized": False,
        "dataset_admitted": False,
        "independent_dev_sites": 0,
    }


def main() -> None:
    manifest, plan = preflight()
    import carla
    from agents.navigation.global_route_planner import GlobalRoutePlanner

    cached_map = v3.CachedCarlaMap(carla, base.digest(base.SOURCE_XODR))
    bounds_cache = v4.SectionBoundsCache(base.digest(base.SOURCE_XODR))
    base.nominal.lane._section_bounds = bounds_cache
    base.nominal.provisional_nominal_clearance = v2.guarded_clearance
    report = reconstruct(carla, GlobalRoutePlanner, manifest, cached_map, bounds_cache,
                         plan["expected_count_delta"])
    base.require(not OUTPUT.exists() and not OUTPUT.is_symlink(),
                 "registered derivative output appeared during scan")
    OUTPUT.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report["counts"], indent=2))


if __name__ == "__main__":
    main()
