#!/usr/bin/env python3
"""HH_261001 - Strict sampled body/sweep diagnostics for a failed immutable Town12 episode."""

import argparse
import json
from pathlib import Path
import sys

from scripts.e2e import audit_town12_j15824_capture_20261001 as qa


def audit(episode, qualification, carla):
    lane, require = qa.lane, qa.require
    manifest_path, route_path = episode / "manifest.json", episode / "route.json"
    state_path, map_path = episode / "states.jsonl", episode / "active_map_opendrive.xodr"
    hashes = {path.name: qa.sha(path) for path in (manifest_path, route_path, state_path, map_path)}
    manifest, route = json.loads(manifest_path.read_text()), json.loads(route_path.read_text())
    require(manifest["status"] == "failed", "forensics is limited to retained failed episodes; use normal admission gate for complete data")
    require(qualification["episode"] == str(episode) and qualification["acknowledged_controls"]["status"] == "PASS"
        and qualification["snapshot_kinematics"]["coordinate_conversion_status"] == "RECORDED_FORMULA_MATCH",
        "prior immutable ACK/snapshot qualification is absent")
    prefix = str(episode.relative_to(Path(qualification["trial"])))
    for name in ("manifest.json", "route.json", "states.jsonl"):
        require(hashes[name] == qualification["input_sha256"][prefix + "/" + name], "failed raw metadata differs from prior QA")
    require(hashes["route.json"] == manifest["provenance"]["route_sha256"], "archived route hash changed")
    require(qa.sha(Path(lane.__file__).with_name("collect_carla_vad_expert.py")) == manifest["provenance"]["collector_sha256"],
        "collector implementation changed")
    require(qa.capture.scout.route_digest(route["route"]) == qa.capture.scout.ROUTE_SHA256, "fixed intended route differs")
    capture = manifest["capture_contract"]
    require(capture["physics_hz"] == 20 and capture["control_transport"]["mode"] == "acknowledged_batch", "20Hz ACK snapshots required")
    require(capture["active_map_opendrive"]["sha256"] == hashes["active_map_opendrive.xodr"] == qa.capture.scout.XODR_SHA256,
        "archived active XODR differs")
    body = capture["actor_body_geometry"]
    require(body["status"] == "RECORDED" and body["source"] == "carla.Vehicle.bounding_box"
        and body["reference_frame"] == "CARLA actor-local; not ROS rear-axle base_link"
        and body["actor_type_id"] == "vehicle.toyota.prius", "actual actor bounding box provenance unavailable")
    construction = capture["agent_initialization"]["construction"]
    require(construction["status"] == "PASS" and all(construction[side]["actor_id"] == body["actor_id"] for side in ("before", "after")),
        "body identity not bound to original actor initialization")
    states = [json.loads(line) for line in state_path.read_text().splitlines()]
    points = [route["route"][0]]
    for point in route["route"][1:]:
        if point["distance_m"] != points[-1]["distance_m"]:
            points.append(point)
        else:
            require(all(point[key] == points[-1][key] for key in ("road_id", "section_id", "lane_id", "x", "y", "z")),
                    "duplicate distance changes route geometry")
    selected = lane._selected_states(states, points, None, None)
    box, radius = lane._body_box(manifest, carla)
    xml = map_path.read_bytes()
    map_ = carla.Map("failed-episode-offline-forensics", xml.decode())
    lo, hi = selected[0]["arc_m"] - radius, selected[-1]["arc_m"] + radius
    indices = {node for index, (a, b) in enumerate(zip(points, points[1:]))
        if a["distance_m"] <= hi and b["distance_m"] >= lo for node in (index, index + 1)}
    requested = {lane._triple(points[index]) for index in indices}
    corridors = lane.lane_corridors(map_, carla, points, xml, requested, indices)
    cache = {}
    def allowed(lanes, inset):
        key = frozenset(lanes), inset
        if key not in cache:
            cache[key] = lane.authorized_lane_region(corridors, lanes, inset)
        return cache[key]
    footprints, records = [], []
    def assess(geometry, lanes, row):
        outside = geometry.difference(allowed(lanes, lane.LANE_INSET_M)).area
        raw_outside = geometry.difference(allowed(lanes, 0.)).area
        require(raw_outside <= outside + 1e-6, "raw/inset area monotonicity failed")
        records.append({**row, "raw_lane_outside_m2": raw_outside, "inset_10cm_outside_m2": outside,
            "violates_raw_lane": raw_outside > 1e-6, "violates_10cm_inset": outside > 1e-6,
            "intended_lanes": [list(lane_id) for lane_id in sorted(lanes)]})
    for item in selected:
        footprint = lane.body_footprint(box, item["pose"], carla)
        lanes = lane._route_lanes_for_arc_interval(points, item["arc_m"] - radius, item["arc_m"] + radius)
        assess(footprint, lanes, {"kind": "sampled_body", "frame": item["state"]["frame"],
            "timestamp": item["state"]["timestamp"], "progress_m": item["arc_m"], "pose_carla": item["pose"]})
        footprints.append(footprint)
    for index, (before, after) in enumerate(zip(selected, selected[1:])):
        lanes = lane._route_lanes_for_arc_interval(points, before["arc_m"] - radius, after["arc_m"] + radius)
        sweep = lane.swept_body(footprints[index], footprints[index + 1], before["pose"], after["pose"], radius)
        assess(sweep, lanes, {"kind": "bounded_snapshot_linear_sweep",
            "frame_pair": [before["state"]["frame"], after["state"]["frame"]],
            "progress_range_m": [before["arc_m"], after["arc_m"]]})
    violations = [row for row in records if row["violates_10cm_inset"]]
    for name, expected in hashes.items():
        require(qa.sha(episode / name) == expected, "immutable failed input changed while reading")
    return {"status": "FAIL_GEOMETRY" if violations else "PASS_SAMPLED_GEOMETRY_ONLY_PARTIAL_EPISODE",
        "episode_status_unchanged": manifest["status"], "training_data_admitted": False,
        "original_complete_episode_gate": qualification["body_and_bounded_sweep"],
        "interpretation": "Diagnostic only: same geometric primitives/numerical limits, but incomplete episode remains inadmissible regardless of geometry.",
        "frames_checked": len(selected), "sweeps_checked": len(selected) - 1,
        "lane_inset_m": lane.LANE_INSET_M, "lane_sample_m": lane.LANE_SAMPLE_M,
        "authorized_seam_gap_close_m": lane.SEAM_GAP_CLOSE_M,
        "body_inset_violation_count": sum(row["kind"] == "sampled_body" for row in violations),
        "sweep_inset_violation_count": sum(row["kind"] != "sampled_body" for row in violations),
        "body_raw_lane_violation_count": sum(row["violates_raw_lane"] and row["kind"] == "sampled_body" for row in records),
        "sweep_raw_lane_violation_count": sum(row["violates_raw_lane"] and row["kind"] != "sampled_body" for row in records),
        "first_violation": violations[0] if violations else None,
        "worst_violation": max(violations, key=lambda row: row["inset_10cm_outside_m2"]) if violations else None,
        "source_sha256": hashes, "physical_continuous_time_proven": False}, records


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--episode", type=Path, required=True)
    parser.add_argument("--qualification", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    episode, output = args.episode.resolve(strict=True), args.output.absolute()
    qa.require(not output.exists() and not output.is_symlink() and not output.resolve().is_relative_to(episode.parent.parent),
               "new forensic output must be outside immutable raw trial")
    qualification_sha = qa.sha(args.qualification)
    qualification = json.loads(args.qualification.read_text())
    sys.path[:0] = [str(qa.capture.scout.SOURCE_EGG), str(qa.capture.scout.SOURCE_PLANNER.parents[2])]
    carla, _ = qa.capture.scout.source_api()
    report, records = audit(episode, qualification, carla)
    report.update(qualification_sha256=qualification_sha, script_sha256=qa.sha(Path(__file__)),
        geometry_primitives_sha256=qa.sha(Path(qa.lane.__file__)))
    qa.require(qa.sha(args.qualification) == qualification_sha, "qualification changed while reading")
    output.mkdir(parents=True, exist_ok=False)
    qa.capture.scout.write_new(output / "summary.json", report)
    with (output / "all_body_and_sweep_records.jsonl").open("x") as stream:
        for row in records:
            stream.write(json.dumps(row, separators=(",", ":"), allow_nan=False) + "\n")
    print(json.dumps({key: report[key] for key in ("status", "frames_checked", "sweeps_checked",
        "body_inset_violation_count", "sweep_inset_violation_count", "body_raw_lane_violation_count",
        "sweep_raw_lane_violation_count", "training_data_admitted")}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
