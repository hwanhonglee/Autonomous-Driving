#!/usr/bin/env python3
"""HH_260906 - Probe the recorded route's nominal vehicle-body envelope without replaying CARLA."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "e2e"))
import audit_future_actor_lane_containment as lane_gate  # noqa: E402
import carla  # noqa: E402


def nominal_pose(a: dict, b: dict, fraction: float, wheelbase_m: float) -> tuple[float, dict]:
    """HH_260906 - Convert an interpolated ROS rear-axle route point to the CARLA actor center."""
    arc = float(a["distance_m"]) + fraction * (float(b["distance_m"]) - float(a["distance_m"]))
    x = float(a["x"]) + fraction * (float(b["x"]) - float(a["x"]))
    y = -(float(a["y"]) + fraction * (float(b["y"]) - float(a["y"])))
    yaw_a, yaw_b = float(a["yaw"]), float(b["yaw"])
    yaw_delta = math.atan2(math.sin(yaw_b - yaw_a), math.cos(yaw_b - yaw_a))
    carla_yaw = -(yaw_a + fraction * yaw_delta)
    return arc, {"x": x + wheelbase_m / 2 * math.cos(carla_yaw),
                 "y": y + wheelbase_m / 2 * math.sin(carla_yaw),
                 "z": float(a["z"]) + fraction * (float(b["z"]) - float(a["z"])),
                 "roll": 0.0, "pitch": 0.0, "yaw": math.degrees(carla_yaw)}


def probe(episode: Path, start_m: float, end_m: float, step_m: float) -> dict:
    """HH_260906 - Check only the piecewise-linear nominal route and its bounded yaw sweep."""
    manifest, route, _states, xml, hashes = lane_gate.verify_sources(episode)
    points = [route["route"][0]]
    for point in route["route"][1:]:
        if point["distance_m"] != points[-1]["distance_m"]:
            points.append(point)
    wheelbase_m = float(manifest["coordinate_contract"]["wheelbase_m"])
    if not math.isfinite(wheelbase_m) or wheelbase_m <= 0:
        raise lane_gate.Unsupported("invalid wheelbase provenance")
    box, radius = lane_gate._body_box(manifest, carla)
    relevant = {j for i, (a, b) in enumerate(zip(points, points[1:]))
                if a["distance_m"] <= end_m + radius and b["distance_m"] >= start_m - radius
                for j in (i, i + 1)}
    requested = {lane_gate._triple(points[i]) for i in relevant}
    carla_map = carla.Map("archived-active-map", xml.decode("utf-8"))
    corridors = lane_gate.lane_corridors(carla_map, carla, points, xml, requested, relevant)
    allowed_cache: dict[tuple[frozenset, float], object] = {}

    def allowed(lanes: set, inset_m: float):
        key = (frozenset(lanes), inset_m)
        if key not in allowed_cache:
            allowed_cache[key] = lane_gate.authorized_lane_region(corridors, lanes, inset_m)
        return allowed_cache[key]

    samples = []
    for a, b in zip(points, points[1:]):
        length = float(b["distance_m"] - a["distance_m"])
        if b["distance_m"] < start_m or a["distance_m"] > end_m:
            continue
        count = math.ceil(length / step_m)
        fractions = {part / count for part in range(count + 1)}
        for boundary in (start_m, end_m):
            if float(a["distance_m"]) <= boundary <= float(b["distance_m"]):
                fractions.add((boundary - float(a["distance_m"])) / length)
        for fraction in sorted(fractions):
            arc, pose = nominal_pose(a, b, fraction, wheelbase_m)
            if not start_m <= arc <= end_m:
                continue
            if samples and abs(arc - samples[-1]["arc_m"]) <= 1e-8:
                continue
            if samples and arc < samples[-1]["arc_m"]:
                raise lane_gate.Unsupported("nominal route sample progression is not strictly increasing")
            lanes = lane_gate._route_lanes_for_arc_interval(points, arc - radius, arc + radius)
            body = lane_gate.body_footprint(box, pose, carla)
            raw_lane = allowed(lanes, 0.0)
            inset_lane = allowed(lanes, lane_gate.LANE_INSET_M)
            raw = body.difference(raw_lane).area
            inset = body.difference(inset_lane).area
            samples.append({"arc_m": arc, "pose": pose, "lanes": lanes, "body": body,
                            "raw_outside_m2": raw, "inset_outside_m2": inset,
                            "raw_clearance_m": body.distance(raw_lane.boundary),
                            "inset_clearance_m": body.distance(inset_lane.boundary)})
    if len(samples) < 2:
        raise lane_gate.Unsupported("nominal probe has too few samples")
    body_raw_max = max(samples, key=lambda row: row["raw_outside_m2"])
    body_inset_max = max(samples, key=lambda row: row["inset_outside_m2"])
    min_body_raw_clearance = min(samples, key=lambda row: row["raw_clearance_m"])
    min_body_inset_clearance = min(samples, key=lambda row: row["inset_clearance_m"])
    sweep_raw_count = sweep_inset_count = 0
    sweep_raw_max = (0.0, None)
    sweep_inset_max = (0.0, None)
    sweep_min_raw_clearance = (math.inf, None)
    sweep_min_inset_clearance = (math.inf, None)
    for before, after in zip(samples, samples[1:]):
        lanes = lane_gate._route_lanes_for_arc_interval(
            points, before["arc_m"] - radius, after["arc_m"] + radius)
        sweep = lane_gate.swept_body(before["body"], after["body"],
                                     before["pose"], after["pose"], radius)
        raw_lane = allowed(lanes, 0.0)
        inset_lane = allowed(lanes, lane_gate.LANE_INSET_M)
        raw = sweep.difference(raw_lane).area
        inset = sweep.difference(inset_lane).area
        raw_clearance = sweep.distance(raw_lane.boundary)
        inset_clearance = sweep.distance(inset_lane.boundary)
        sweep_raw_count += raw > 1e-6
        sweep_inset_count += inset > 1e-6
        if raw > sweep_raw_max[0]:
            sweep_raw_max = (raw, [before["arc_m"], after["arc_m"]])
        if inset > sweep_inset_max[0]:
            sweep_inset_max = (inset, [before["arc_m"], after["arc_m"]])
        if raw_clearance < sweep_min_raw_clearance[0]:
            sweep_min_raw_clearance = (raw_clearance, [before["arc_m"], after["arc_m"]])
        if inset_clearance < sweep_min_inset_clearance[0]:
            sweep_min_inset_clearance = (inset_clearance, [before["arc_m"], after["arc_m"]])
    raw_count = sum(row["raw_outside_m2"] > 1e-6 for row in samples)
    inset_count = sum(row["inset_outside_m2"] > 1e-6 for row in samples)
    return {
        "status": "PROVISIONAL_FAIL" if raw_count or inset_count or sweep_raw_count or sweep_inset_count
        else "PROVISIONAL_PASS",
        "scope": "interpolated nominal route body and conservative short-step swept envelope; not observed control or physical feasibility",
        "route_window_m": [start_m, end_m], "nominal_step_max_m": step_m,
        "sample_count": len(samples), "sweep_count": len(samples) - 1,
        "sample_arc_range_m": [samples[0]["arc_m"], samples[-1]["arc_m"]],
        "wheelbase_m": wheelbase_m, "body_radius_bound_m": radius,
        "intended_lanes": [list(triple) for triple in sorted(requested)],
        "body_raw_violation_count": raw_count, "body_inset_10cm_violation_count": inset_count,
        "body_max_raw_outside_m2": body_raw_max["raw_outside_m2"],
        "body_max_raw_arc_m": body_raw_max["arc_m"],
        "body_max_inset_outside_m2": body_inset_max["inset_outside_m2"],
        "body_max_inset_arc_m": body_inset_max["arc_m"],
        "body_min_raw_boundary_clearance_m": min_body_raw_clearance["raw_clearance_m"],
        "body_min_raw_clearance_arc_m": min_body_raw_clearance["arc_m"],
        "body_min_inset_boundary_clearance_m": min_body_inset_clearance["inset_clearance_m"],
        "body_min_inset_clearance_arc_m": min_body_inset_clearance["arc_m"],
        "sweep_raw_violation_count": sweep_raw_count,
        "sweep_inset_10cm_violation_count": sweep_inset_count,
        "sweep_max_raw_outside_m2": sweep_raw_max[0],
        "sweep_max_raw_arc_range_m": sweep_raw_max[1],
        "sweep_max_inset_outside_m2": sweep_inset_max[0],
        "sweep_max_inset_arc_range_m": sweep_inset_max[1],
        "sweep_min_raw_boundary_clearance_m": sweep_min_raw_clearance[0],
        "sweep_min_raw_clearance_arc_range_m": sweep_min_raw_clearance[1],
        "sweep_min_inset_boundary_clearance_m": sweep_min_inset_clearance[0],
        "sweep_min_inset_clearance_arc_range_m": sweep_min_inset_clearance[1],
        "physical_continuous_time_proven": False, "training_data_approved": False,
        "source_sha256": hashes,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode", type=Path, required=True)
    parser.add_argument("--start-m", type=float, default=30.0)
    parser.add_argument("--end-m", type=float, default=120.0)
    parser.add_argument("--step-m", type=float, default=0.05)
    args = parser.parse_args()
    if not all(math.isfinite(value) for value in (args.start_m, args.end_m, args.step_m)) or not (
            0 <= args.start_m < args.end_m and 0 < args.step_m <= 0.05):
        parser.error("require 0 <= start < end and 0 < step <= 0.05m")
    print(json.dumps(probe(args.episode.resolve(), args.start_m, args.end_m, args.step_m),
                     indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
