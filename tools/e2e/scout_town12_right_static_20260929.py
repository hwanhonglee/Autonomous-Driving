#!/usr/bin/env python3
"""HH_260929 - Run the preregistered, server-free Town12 right-turn search."""

from __future__ import annotations

import hashlib
import inspect
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml
from networkx.exception import NetworkXNoPath

from scripts.e2e import physical_turn_geometry as physical
from scripts.e2e import prepare_carla_route as route_helper
from scripts.e2e import rank_town01_v5_long_turns as nominal
from scripts.e2e import scout_other_non_test_right_20260929 as previous


ROOT = Path(__file__).resolve().parents[2]
SOURCE_ROOT = Path("/home/a/carla")
PACKAGED_ROOT = Path("/home/a/carla-autoware-universe/CARLA_0.9.15")
SOURCE_XODR = SOURCE_ROOT / "Unreal/CarlaUE4/Content/Carla/Maps/Town12/OpenDrive/Town12.xodr"
PACKAGED_XODR = PACKAGED_ROOT / "CarlaUE4/Content/Carla/Maps/Town12/OpenDrive/Town12.xodr"
SOURCE_LEVEL = SOURCE_ROOT / "Unreal/CarlaUE4/Content/Carla/Maps/Town12/Town12.umap"
SOURCE_EGG = SOURCE_ROOT / "PythonAPI/carla/dist/carla-0.9.15-py3.10-linux-x86_64.egg"
SOURCE_PLANNER = SOURCE_ROOT / "PythonAPI/carla/agents/navigation/global_route_planner.py"
PREREG = ROOT / "docs/assets/validation/2026-09-29/drive_town12_right_static/preregister.json"
OUTPUT = ROOT / "docs/assets/validation/2026-09-29/drive_town12_right_static/report.json"
SCOPES_M = tuple((up, down) for up in (60, 70, 80) for down in (100, 120))
WINDOW_M = (30.0, 120.0)
MIN_LENGTH_M = 146.0


class ScoutError(ValueError):
    """HH_260929 - Reject a changed prerequisite or ambiguous route."""


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ScoutError(message)


def digest(path: Path) -> str:
    require(path.is_file() and not path.is_symlink(), f"regular file required: {path}")
    return hashlib.sha256(path.read_bytes()).hexdigest()


def preregistered_inputs() -> dict:
    """HH_260929 - Verify the plan and all pinned bytes before parsing the map."""
    require(PREREG.is_file() and not PREREG.is_symlink(), "preregistration is missing")
    plan = json.loads(PREREG.read_text())
    require(plan.get("schema") == "portable_e2e.town12_right_static_preregister.v1",
            "wrong preregistration schema")
    require(plan.get("output_path") == str(OUTPUT.relative_to(ROOT)) and
            plan.get("scope_m") == [list(scope) for scope in SCOPES_M] and
            plan.get("sample_resolution_m") == 1.0 and
            plan.get("window_m") == list(WINDOW_M) and
            plan.get("minimum_length_m") == MIN_LENGTH_M and
            plan.get("map_role") == "TRAIN_CANDIDATE_ONLY" and
            plan.get("static_only") is True, "preregistered scope changed")
    expected_paths = {
        "town12_source_xodr": SOURCE_XODR,
        "town12_packaged_xodr": PACKAGED_XODR,
        "town12_source_level": SOURCE_LEVEL,
        "source_carla_egg": SOURCE_EGG,
        "source_global_route_planner": SOURCE_PLANNER,
        "search_script": Path(__file__),
        "suite": ROOT / "tools/e2e/carla_expert_suite.yaml",
        "next_plan": ROOT / "docs/assets/validation/2026-09-29/drive_other_site_right_static/NEXT.md",
        "previous_scout": ROOT / "tools/e2e/scout_other_non_test_right_20260929.py",
        "physical_turn_geometry": ROOT / "tools/e2e/physical_turn_geometry.py",
        "prepare_carla_route": ROOT / "tools/e2e/prepare_carla_route.py",
        "nominal_clearance": ROOT / "tools/e2e/rank_town01_v5_long_turns.py",
        "nominal_route_geometry": ROOT / "tools/e2e/rank_town01_wide_turns.py",
        "lane_containment": ROOT / "tools/e2e/audit_future_actor_lane_containment.py",
        "turn_candidates": ROOT / "tools/e2e/audit_new_long_turn_candidates.py",
        "dataset_manifest": ROOT / "storage/datasets/prepared/carla-common10-trainval-research-20260914-v1/dataset.json",
        "integrated_split_plan": ROOT / "docs/assets/validation/2026-09-27/map_next_training_design/README.md",
        "nominal_prius_manifest": ROOT / "storage/experiments/training/2026-09-27/drive_long_route_town01_right_s0002_p06_9kph_steer045_purge15_trial_v1/episode/manifest.json",
    }
    require(set(plan.get("input_sha256", {})) == set(expected_paths),
            "preregistered input set changed")
    for name, path in expected_paths.items():
        require(digest(path) == plan["input_sha256"][name], f"pinned input changed: {name}")
    require(plan["input_sha256"]["town12_source_xodr"] ==
            plan["input_sha256"]["town12_packaged_xodr"],
            "source and packaged Town12 XODR differ")
    suite = yaml.safe_load(expected_paths["suite"].read_text())
    matches = [item for item in suite["maps"] if item.get("id") == "town12"]
    require(len(matches) == 1 and matches[0].get("canonical_name") == "Town12" and
            matches[0].get("split") == "train" and
            matches[0].get("server_profile") == "source_editor_0915_4ws" and
            matches[0].get("status") == "source_editor_required" and
            matches[0].get("opendrive_path") ==
            "${CARLA_SOURCE_ROOT}/Unreal/CarlaUE4/Content/Carla/Maps/Town12/OpenDrive/Town12.xodr",
            "Town12 source/split suite contract changed")
    manifest = json.loads(expected_paths["nominal_prius_manifest"].read_text())
    require(manifest["capture_contract"]["actor_body_geometry"]["actor_type_id"] ==
            "vehicle.toyota.prius" and
            manifest["coordinate_contract"]["wheelbase_m"] == 2.85,
            "nominal Prius body contract changed")
    require(physical.SPEED_30KPH_TURN_GEOMETRY_CONTRACT["profile_id"] ==
            "carla_vad_30kph_v2" and
            math.isclose(previous.MIN_LENGTH_M, MIN_LENGTH_M) and
            previous.WINDOW_M == WINDOW_M,
            "original physical profile or window changed")
    require(not OUTPUT.exists() and not OUTPUT.is_symlink() and
            OUTPUT.parent.is_dir() and not OUTPUT.parent.is_symlink(),
            "registered output must be new in a regular directory")
    return manifest


def connector_on_block(points: list[dict], block: tuple[int, int], entry: object) -> bool:
    """HH_260929 - Bind the RIGHT block to the exact junction lane section."""
    return any((point["road_id"], point["section_id"], point["lane_id"]) ==
               (entry.road_id, entry.section_id, entry.lane_id)
               for point in points[block[0]:block[1] + 1])


def scan(carla: object, planner_class: object, manifest: dict) -> dict:
    require(Path(inspect.getsourcefile(planner_class)).resolve() == SOURCE_PLANNER,
            "wrong GlobalRoutePlanner imported")
    require(str(carla.__file__).startswith(str(SOURCE_EGG) + "/"),
            "wrong CARLA Python API imported")
    xodr = SOURCE_XODR.read_bytes()
    carla_map = carla.Map("offline-Town12-right-20260929", xodr.decode("utf-8"))
    planner = planner_class(carla_map, 1.0)
    roads = {int(road.attrib["id"]): road for road in ET.fromstring(xodr).findall("road")}
    junctions = {point.get_junction().id: point.get_junction()
                 for point in carla_map.generate_waypoints(3.0) if point.is_junction}
    counts = {key: 0 for key in (
        "right_connectors", "endpoint_pairs", "unreachable_pairs", "length_at_least_146m",
        "traverses_connector", "single_right_no_other_maneuver",
        "right_block_on_selected_connector", "turn_inside_30_120m",
        "physical_turn_pass", "nominal_pass", "nominal_fail", "nominal_unsupported",
        "signal_record_free_nominal_pass", "no_other_junction_nominal_pass")}
    physical_rejections: dict[str, int] = {}
    candidate_errors: dict[str, int] = {}
    rows = []
    from agents.navigation.local_planner import RoadOption

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
                        # HH_260929 - CARLA's planner retains a prior junction decision.
                        planner._intersection_end_node = -1
                        planner._previous_decision = RoadOption.VOID
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
                        if not any((point["road_id"], point["section_id"], point["lane_id"]) ==
                                   (entry.road_id, entry.section_id, entry.lane_id)
                                   for point in points):
                            continue
                        counts["traverses_connector"] += 1
                        block = previous.right_block(points)
                        if block is None:
                            continue
                        counts["single_right_no_other_maneuver"] += 1
                        if not connector_on_block(points, block, entry):
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
                            geometry = physical.analyze_serialized_physical_turn(
                                route, physical.SPEED_30KPH_TURN_GEOMETRY_CONTRACT)
                        except physical.PhysicalTurnGeometryError as error:
                            if error.fatal:
                                raise ScoutError(f"physical contract failed: {error}") from error
                            candidate_errors[error.error_code] = candidate_errors.get(error.error_code, 0) + 1
                            continue
                        if geometry["status"] != "PASS":
                            for reason in geometry["failure_reasons"]:
                                physical_rejections[reason] = physical_rejections.get(reason, 0) + 1
                            continue
                        counts["physical_turn_pass"] += 1
                        signals = previous.signal_records(roads, points)
                        approach_signals = previous.signal_records(roads, points[:block[0] + 1])
                        other_junctions = previous.other_junction_ids(roads, points, int(junction_id))
                        try:
                            body = nominal.provisional_nominal_clearance(
                                route, manifest, xodr, carla, *WINDOW_M)
                        except (ValueError, RuntimeError, KeyError) as error:
                            body = {"status": "UNSUPPORTED", "error_type": type(error).__name__,
                                    "reason": str(error)}
                        require(body.get("status") in {"PASS", "FAIL", "UNSUPPORTED"},
                                "nominal sweep returned unknown status")
                        counts[{"PASS": "nominal_pass", "FAIL": "nominal_fail",
                                "UNSUPPORTED": "nominal_unsupported"}[body["status"]]] += 1
                        if body["status"] == "PASS" and not any(signals.values()):
                            counts["signal_record_free_nominal_pass"] += 1
                        if body["status"] == "PASS" and not other_junctions:
                            counts["no_other_junction_nominal_pass"] += 1
                        rows.append({
                            "junction_id": int(junction_id),
                            "connector_road_id": int(entry.road_id),
                            "connector_section_id": int(entry.section_id),
                            "connector_lane_id": int(entry.lane_id),
                            "connector_heading_deg": float(heading),
                            "upstream_m": upstream_m, "downstream_m": downstream_m,
                            "start_index": start_index, "goal_index": goal_index,
                            "start": previous.pose(start), "goal": previous.pose(goal),
                            "route_length_m": float(points[-1]["distance_m"]),
                            "right_turn_m": [turn_start, turn_end],
                            "route_sha256": previous.route_digest(points),
                            "site": previous.physical_site(points, block),
                            "full_route_signals": signals,
                            "approach_signals": approach_signals,
                            "other_junction_ids_in_30_146m": other_junctions,
                            "nominal": {key: body.get(key) for key in (
                                "status", "reason", "error_type", "body_count", "sweep_count",
                                "raw_body_outside_count", "raw_sweep_outside_count",
                                "inset_body_outside_count", "inset_sweep_outside_count",
                                "maximum_inset_outside_m2", "minimum_inset_sweep_clearance_m",
                                "production_lane_gate_supported") if key in body},
                        })
    rows.sort(key=lambda row: (row["junction_id"], row["connector_road_id"],
                               row["connector_section_id"], row["connector_lane_id"],
                               row["upstream_m"], row["downstream_m"], row["start_index"],
                               row["goal_index"]))
    require(counts["nominal_pass"] + counts["nominal_fail"] +
            counts["nominal_unsupported"] == counts["physical_turn_pass"] == len(rows),
            "physical-pass route was not body-audited")
    return {
        "schema": "portable_e2e.town12_right_static_scout.v1",
        "status": "BOUNDED_OFFLINE_AUDIT",
        "map_role": "TRAIN_CANDIDATE_ONLY",
        "source_xodr_path": str(SOURCE_XODR),
        "packaged_xodr_path": str(PACKAGED_XODR),
        "xodr_sha256": digest(SOURCE_XODR),
        "preregistration_sha256": digest(PREREG),
        "search_source_sha256": digest(Path(__file__)),
        "planner_source_sha256": digest(SOURCE_PLANNER),
        "physical_turn_profile": physical.SPEED_30KPH_TURN_GEOMETRY_CONTRACT["profile_id"],
        "sampling_resolution_m": 1.0,
        "right_heading_range_deg": [60.0, 120.0],
        "endpoint_scopes_m": [list(scope) for scope in SCOPES_M],
        "fixed_window_m": list(WINDOW_M),
        "minimum_route_length_m": MIN_LENGTH_M,
        "future_sec": 6.4,
        "future_distance_at_9kmh_m": 16.0,
        "future_guard_m": 10.0,
        "nominal_body_source": "pinned Town01 Prius manifest; cross-map hypothesis",
        "lane_inset_m": 0.10,
        "counts": counts,
        "physical_rejections": physical_rejections,
        "candidate_errors": candidate_errors,
        "physical_pass_routes": rows,
        "independent_dev_sites": 0,
        "live_xodr_or_body_verified": False,
        "continuous_moving_verified": False,
        "future_64_points_verified": False,
        "nine_motion_bounds_verified": False,
        "actual_collision_lane_goal_verified": False,
        "capture_authorized": False,
        "dataset_admitted": False,
        "training_executed": False,
        "learned_closed_loop_tested": False,
    }


def main() -> None:
    manifest = preregistered_inputs()
    import carla
    from agents.navigation.global_route_planner import GlobalRoutePlanner

    report = scan(carla, GlobalRoutePlanner, manifest)
    require(not OUTPUT.exists() and not OUTPUT.is_symlink(), "registered output appeared during scan")
    OUTPUT.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report["counts"], indent=2))


if __name__ == "__main__":
    main()
