#!/usr/bin/env python3
"""HH_260929 - Prepared Town12 rerun with bounded XML cache and junction ledger."""

from __future__ import annotations

import hashlib
import inspect
import json
import math
import os
import xml.etree.ElementTree as ET
from pathlib import Path

from networkx.exception import NetworkXNoPath

from scripts.e2e import scout_town12_right_static_20260929 as base
from scripts.e2e import scout_town12_right_static_20260929_v2 as v2
from scripts.e2e import scout_town12_right_static_20260929_v3 as v3


ROOT = Path(__file__).resolve().parents[2]
DIR = ROOT / "docs/assets/validation/2026-09-29/drive_town12_right_static"
PREREG = DIR / "preregister_v4.json"
OUTPUT = DIR / "report_v4.json"
PROGRESS = DIR / "progress_v4.jsonl"
V3_STOP = DIR / "v3_stop.json"
ORIGINAL_SECTION_BOUNDS = base.nominal.lane._section_bounds


class SectionBoundsCache:
    """HH_260929 - Reuse only pure road/section bounds from identical XODR bytes."""

    def __init__(self, expected_sha256: str):
        self.expected_sha256 = expected_sha256
        self._bounds = None
        self.parse_count = 0
        self.requests = 0

    def __call__(self, xml: bytes) -> dict:
        base.require(type(xml) is bytes and
                     hashlib.sha256(xml).hexdigest() == self.expected_sha256,
                     "section-bounds request differs from pinned Town12 XODR")
        self.requests += 1
        if self._bounds is None:
            self._bounds = ORIGINAL_SECTION_BOUNDS(xml)
            self.parse_count += 1
        # HH_260929 - Callers receive a fresh mapping as in the original helper.
        return dict(self._bounds)


class ProgressLedger:
    """HH_260929 - Write one append-only row after a whole junction completes."""

    def __init__(self, path: Path, total_junctions: int):
        base.require(total_junctions > 0 and not path.exists() and not path.is_symlink(),
                     "progress ledger must be new and have junctions")
        self.path = path
        self.total_junctions = total_junctions
        self.completed = 0
        path.open("x", encoding="utf-8").close()

    def record(self, junction_id: int, counts: dict, rows: list[dict]) -> None:
        self.completed += 1
        base.require(self.completed <= self.total_junctions, "progress exceeds junction count")
        route_hashes = sorted(row["route_sha256"] for row in rows)
        digest = hashlib.sha256(json.dumps(route_hashes, separators=(",", ":")).encode()).hexdigest()
        line = {
            "schema": "portable_e2e.town12_right_static_progress.v4",
            "completed_junctions": self.completed,
            "total_junctions": self.total_junctions,
            "last_completed_junction_id": junction_id,
            "cumulative_counts": dict(counts),
            "cumulative_physical_route_hashes_sha256": digest,
            "partial_not_qualifying_evidence": True,
        }
        with self.path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(line, sort_keys=True, separators=(",", ":")) + "\n")
            stream.flush()
            os.fsync(stream.fileno())


def verify_v3_terminal(plan: dict) -> dict:
    """HH_260929 - Refuse a second scan while the exact V3 owner is active."""
    pid = plan.get("v3_pid")
    ticks = plan.get("v3_process_start_ticks")
    base.require(pid == 1857040 and ticks == 275931049,
                 "pinned V3 process identity changed")
    proc = Path(f"/proc/{pid}")
    base.require(not proc.exists(), "V3 process is still active")
    if v3.OUTPUT.is_file() and not v3.OUTPUT.is_symlink():
        report = json.loads(v3.OUTPUT.read_text())
        base.require(report.get("schema") == "portable_e2e.town12_right_static_scout.v3" and
                     report.get("preregistration_sha256") == base.digest(v3.PREREG) and
                     report.get("search_source_sha256") == base.digest(Path(v3.__file__)) and
                     report.get("counts", {}).get("physical_turn_pass") ==
                     len(report.get("physical_pass_routes", [])) and
                     report.get("map_reuse", {}).get("map_constructions") == 1,
                     "V3 report is incomplete or changed")
        return {"kind": "COMPLETE_REPORT", "sha256": base.digest(v3.OUTPUT)}
    base.require(not v3.OUTPUT.exists() and V3_STOP.is_file() and not V3_STOP.is_symlink(),
                 "V3 needs a completed report or documented stop")
    stop = json.loads(V3_STOP.read_text())
    base.require(stop.get("schema") == "portable_e2e.town12_v3_stop.v1" and
                 stop.get("status") == "STOPPED_WITHOUT_REPORT" and
                 stop.get("pid") == pid and
                 stop.get("process_start_ticks") == ticks and
                 stop.get("termination_signal") == "SIGTERM" and
                 stop.get("process_exit_observed") is True and
                 stop.get("post_stop_proc_path_absent") is True and
                 stop.get("post_stop_ps_entry_absent") is True and
                 stop.get("report_absent") is True and
                 stop.get("v3_preregistration_sha256") == base.digest(v3.PREREG) and
                 stop.get("v3_script_sha256") == base.digest(Path(v3.__file__)) and
                 stop.get("v3_plan_sha256") == base.digest(DIR / "PLAN_V3.md") and
                 stop.get("source_xodr_sha256") == base.digest(base.SOURCE_XODR) and
                 stop.get("log_path") == "/tmp/town12_static_v3_20260929.log" and
                 stop.get("log_size_bytes") == 0 and
                 stop.get("log_sha256") == base.digest(Path(stop["log_path"])),
                 "V3 stop evidence is incomplete or changed")
    return {"kind": "DOCUMENTED_STOP", "sha256": base.digest(V3_STOP)}


def preregistered_inputs() -> tuple[dict, dict]:
    manifest = base.preregistered_inputs()
    base.require(PREREG.is_file() and not PREREG.is_symlink(),
                 "V4 preregistration is missing")
    plan = json.loads(PREREG.read_text())
    base.require(plan.get("schema") == "portable_e2e.town12_right_static_preregister.v4" and
                 plan.get("output_path") == str(OUTPUT.relative_to(ROOT)) and
                 plan.get("progress_path") == str(PROGRESS.relative_to(ROOT)) and
                 plan.get("scope_m") == [list(scope) for scope in base.SCOPES_M] and
                 plan.get("window_m") == list(base.WINDOW_M) and
                 plan.get("minimum_length_m") == base.MIN_LENGTH_M and
                 plan.get("sample_resolution_m") == 1.0 and
                 plan.get("lane_inset_m") == 0.1 and
                 plan.get("seam_tolerance_m") == 0.005 and
                 plan.get("physical_profile") == "carla_vad_30kph_v2" and
                 plan.get("source_xodr_sha256") == base.digest(base.SOURCE_XODR) and
                 plan.get("base_preregistration_sha256") == base.digest(base.PREREG) and
                 plan.get("v2_preregistration_sha256") == base.digest(v2.PREREG) and
                 plan.get("v3_preregistration_sha256") == base.digest(v3.PREREG) and
                 plan.get("base_script_sha256") == base.digest(Path(base.__file__)) and
                 plan.get("v2_script_sha256") == base.digest(Path(v2.__file__)) and
                 plan.get("v3_script_sha256") == base.digest(Path(v3.__file__)) and
                 plan.get("v4_script_sha256") == base.digest(Path(__file__)) and
                 plan.get("lane_helper_sha256") ==
                 base.digest(ROOT / "tools/e2e/audit_future_actor_lane_containment.py") and
                 plan.get("nominal_helper_sha256") ==
                 base.digest(ROOT / "tools/e2e/rank_town01_v5_long_turns.py") and
                 plan.get("no_threshold_change") is True,
                 "V4 pinned source or scope changed")
    base.require(not OUTPUT.exists() and not OUTPUT.is_symlink() and
                 not PROGRESS.exists() and not PROGRESS.is_symlink(),
                 "V4 output or progress ledger already exists")
    return manifest, verify_v3_terminal(plan)


def scan(carla: object, planner_class: object, manifest: dict,
         cached_map: v3.CachedCarlaMap, bounds_cache: SectionBoundsCache) -> dict:
    """HH_260929 - Keep V3's six-scope gates, logging only completed junctions."""
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
    ledger = ProgressLedger(PROGRESS, len(junctions))
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
                            candidate_errors[error.error_code] = candidate_errors.get(error.error_code, 0) + 1
                            continue
                        if geometry["status"] != "PASS":
                            for reason in geometry["failure_reasons"]:
                                physical_rejections[reason] = physical_rejections.get(reason, 0) + 1
                            continue
                        counts["physical_turn_pass"] += 1
                        signals = base.previous.signal_records(roads, points)
                        approach_signals = base.previous.signal_records(roads, points[:block[0] + 1])
                        other_junctions = base.previous.other_junction_ids(roads, points, int(junction_id))
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
                            "junction_id": int(junction_id),
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
                        })
        ledger.record(int(junction_id), counts, rows)
    rows.sort(key=lambda row: (row["junction_id"], row["connector_road_id"],
                               row["connector_section_id"], row["connector_lane_id"],
                               row["upstream_m"], row["downstream_m"], row["start_index"],
                               row["goal_index"]))
    base.require(ledger.completed == ledger.total_junctions and
                 counts["nominal_pass"] + counts["nominal_fail"] +
                 counts["nominal_unsupported"] == counts["physical_turn_pass"] == len(rows),
                 "incomplete V4 junction or body audit")
    return {
        "schema": "portable_e2e.town12_right_static_scout.v4",
        "status": "BOUNDED_OFFLINE_AUDIT",
        "map_role": "TRAIN_CANDIDATE_ONLY",
        "source_xodr_path": str(base.SOURCE_XODR),
        "packaged_xodr_path": str(base.PACKAGED_XODR),
        "xodr_sha256": base.digest(base.SOURCE_XODR),
        "preregistration_sha256": base.digest(PREREG),
        "search_source_sha256": base.digest(Path(__file__)),
        "planner_source_sha256": base.digest(base.SOURCE_PLANNER),
        "physical_turn_profile": base.physical.SPEED_30KPH_TURN_GEOMETRY_CONTRACT["profile_id"],
        "sampling_resolution_m": 1.0,
        "right_heading_range_deg": [60.0, 120.0],
        "endpoint_scopes_m": [list(scope) for scope in base.SCOPES_M],
        "fixed_window_m": list(base.WINDOW_M),
        "minimum_route_length_m": base.MIN_LENGTH_M,
        "future_sec": 6.4,
        "future_distance_at_9kmh_m": 16.0,
        "future_guard_m": 10.0,
        "nominal_body_source": "pinned Town01 Prius manifest; cross-map hypothesis",
        "lane_inset_m": 0.10,
        "counts": counts,
        "physical_rejections": physical_rejections,
        "candidate_errors": candidate_errors,
        "physical_pass_routes": rows,
        "progress_completed_junctions": ledger.completed,
        "progress_ledger_sha256": base.digest(PROGRESS),
        "map_reuse": {"map_requests": cached_map.requests, "map_constructions": 1},
        "section_bounds_cache": {"requests": bounds_cache.requests,
                                 "original_parses": bounds_cache.parse_count},
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
    manifest, terminal = preregistered_inputs()
    import carla
    from agents.navigation.global_route_planner import GlobalRoutePlanner

    cached_map = v3.CachedCarlaMap(carla, base.digest(base.SOURCE_XODR))
    bounds_cache = SectionBoundsCache(base.digest(base.SOURCE_XODR))
    base.nominal.lane._section_bounds = bounds_cache
    base.nominal.provisional_nominal_clearance = v2.guarded_clearance
    report = scan(carla, GlobalRoutePlanner, manifest, cached_map, bounds_cache)
    report["v3_terminal_evidence"] = terminal
    base.require(not OUTPUT.exists() and not OUTPUT.is_symlink(),
                 "registered V4 output appeared during scan")
    OUTPUT.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report["counts"], indent=2))


if __name__ == "__main__":
    main()
