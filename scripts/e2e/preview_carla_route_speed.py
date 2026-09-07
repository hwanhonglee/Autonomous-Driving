#!/usr/bin/env python3
"""HH_260906 - Offline curvature/approach envelope proposal; no simulator, controller adoption or data admission."""

from __future__ import annotations

import argparse
from bisect import bisect_right
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path


@dataclass(frozen=True)
class PreviewConfig:
    """HH_260906 - These are planning assumptions, not measured zero-brake deceleration authority."""

    nominal_speed_mps: float = 8.0
    planned_lateral_acceleration_mps2: float = 2.0
    planned_backward_deceleration_mps2: float = 0.6
    curvature_forward_lookahead_m: float = 2.0
    approach_reference_speed_mps: float = 3.0396464855480536
    approach_reference_distance_m: float = 35.0
    empirical_coast_seconds: float = 6.93
    stop_buffer_m: float = 0.75
    cruise_minimum_speed_mps: float = 7.8
    launch_acceleration_plan_mps2: float = 1.0


def finite(value):
    if type(value) not in (float, int) or not math.isfinite(value):
        raise ValueError("geometry and planning inputs must be finite numbers, not booleans")
    return float(value)


def route_geometry(route):
    """HH_260906 - Recompute XY arc while retaining the distinct recorded 3D/catalog arc for future mapping."""
    source = route["route"]
    if not isinstance(source, list) or len(source) < 3:
        raise ValueError("at least three ordered route points are required")
    rows = []
    duplicate_count = 0
    for point in source:
        x, y, catalog = (finite(point[key]) for key in ("x", "y", "distance_m"))
        if not rows:
            if catalog != 0:
                raise ValueError("route catalog arc must begin at zero")
            rows.append(dict(x=x, y=y, planar_s_m=0., catalog_s_m=catalog))
            continue
        previous = rows[-1]
        distance = math.hypot(x - previous["x"], y - previous["y"])
        if catalog < previous["catalog_s_m"]:
            raise ValueError("route catalog arc decreases")
        if distance <= 1e-9:
            if catalog != previous["catalog_s_m"]:
                raise ValueError("vertical-only or inconsistent duplicate route geometry cannot be flattened silently")
            duplicate_count += 1
            continue
        if catalog <= previous["catalog_s_m"]:
            raise ValueError("moving geometry requires increasing catalog arc")
        rows.append(dict(x=x, y=y, planar_s_m=previous["planar_s_m"] + distance, catalog_s_m=catalog))
    if len(rows) < 3:
        raise ValueError("route collapses to fewer than three distinct planar points")
    return rows, duplicate_count


def interpolate(rows, distance, field="planar_s_m"):
    """HH_260906 - Interpolate only the offline route polyline, never recorded vehicle or label samples."""
    distance = finite(distance)
    if distance < rows[0][field] - 1e-9 or distance > rows[-1][field] + 1e-9:
        raise ValueError("query is outside the route")
    index = min(max(0, bisect_right([r[field] for r in rows], distance) - 1), len(rows) - 2)
    a, b = rows[index:index + 2]
    fraction = min(1., max(0., (distance - a[field]) / (b[field] - a[field])))
    return {key: a[key] + fraction * (b[key] - a[key]) for key in a}


def resample(rows, spacing):
    spacing = finite(spacing)
    if spacing <= 0 or spacing > rows[-1]["planar_s_m"] / 2:
        raise ValueError("resampling spacing must be positive and leave at least three points")
    length = rows[-1]["planar_s_m"]
    distances = [i * spacing for i in range(math.floor(length / spacing) + 1)]
    if length - distances[-1] > 1e-9:
        distances.append(length)
    return [interpolate(rows, value) for value in distances]


def signed_curvature(a, b, c):
    """HH_260906 - Signed reciprocal circumradius from three XY points; reversals are not silently straight."""
    ux, uy, vx, vy = b["x"] - a["x"], b["y"] - a["y"], c["x"] - b["x"], c["y"] - b["y"]
    first, second, chord = math.hypot(ux, uy), math.hypot(vx, vy), math.hypot(c["x"] - a["x"], c["y"] - a["y"])
    if min(first, second, chord) <= 1e-9:
        raise ValueError("degenerate or reversing three-point geometry")
    cross, dot = ux * vy - uy * vx, ux * vx + uy * vy
    if dot < 0 and abs(cross) <= 1e-9 * first * second:
        raise ValueError("collinear reversal has undefined finite curve preview")
    return 2 * cross / (first * second * chord)


def backward_envelope(distances, local_caps, deceleration):
    """HH_260906 - Enforce a backward kinematic target only; this does not prove pedals can realize it."""
    deceleration = finite(deceleration)
    if deceleration <= 0 or len(distances) != len(local_caps) or not distances:
        raise ValueError("invalid backward planning input")
    distances, result = [finite(v) for v in distances], [finite(v) for v in local_caps]
    if any(v < 0 for v in result) or any(b <= a for a, b in zip(distances, distances[1:])):
        raise ValueError("caps must be nonnegative and arc strictly increasing")
    for index in range(len(result) - 2, -1, -1):
        result[index] = min(result[index], math.sqrt(result[index + 1] ** 2 + 2 * deceleration * (distances[index + 1] - distances[index])))
    return result


def qualifying_spans(rows, minimum):
    """HH_260906 - Report sampled endpoint-qualified intervals, not measured steady-cruise seconds."""
    spans, start, end = [], None, None
    for a, b in zip(rows, rows[1:]):
        if a["planned_from_rest_speed_mps"] >= minimum and b["planned_from_rest_speed_mps"] >= minimum:
            start = a["planar_s_m"] if start is None else start
            end = b["planar_s_m"]
        elif start is not None:
            spans.append(dict(start_planar_s_m=start, end_planar_s_m=end, length_m=end - start))
            start = end = None
    if start is not None:
        spans.append(dict(start_planar_s_m=start, end_planar_s_m=end, length_m=end - start))
    return spans


def preview(route, spacing, config=PreviewConfig()):
    """HH_260906 - Combine all bends, fixed forward preview, goal approach and backward planning constraints."""
    if type(config) is not PreviewConfig or any(type(value) is not float or value != asdict(PreviewConfig())[key]
                                              for key, value in asdict(config).items()):
        raise ValueError("this bounded proposal uses only the frozen planning configuration")
    geometry, duplicates = route_geometry(route)
    rows = resample(geometry, spacing)
    curves = [signed_curvature(*rows[index - 1:index + 2]) for index in range(1, len(rows) - 1)]
    curves = [curves[0], *curves, curves[-1]]
    length = geometry[-1]["planar_s_m"]
    for index, row in enumerate(rows):
        local_max = max(abs(curves[j]) for j in range(index, len(rows))
                        if rows[j]["planar_s_m"] <= row["planar_s_m"] + config.curvature_forward_lookahead_m + 1e-9)
        curvature_cap = min(config.nominal_speed_mps, math.sqrt(config.planned_lateral_acceleration_mps2 / local_max)) if local_max > 0 else config.nominal_speed_mps
        remaining = length - row["planar_s_m"]
        approach_cap = min(config.nominal_speed_mps, math.sqrt(config.approach_reference_speed_mps ** 2
            + 2 * config.planned_backward_deceleration_mps2 * max(remaining - config.approach_reference_distance_m, 0.)))
        row.update(signed_curvature_per_m=curves[index], forward_max_abs_curvature_per_m=local_max,
            curvature_cap_mps=curvature_cap, approach_cap_mps=approach_cap, local_speed_cap_mps=min(curvature_cap, approach_cap))
    envelope = backward_envelope([r["planar_s_m"] for r in rows], [r["local_speed_cap_mps"] for r in rows], config.planned_backward_deceleration_mps2)
    from_rest = 0.
    for index, (row, cap) in enumerate(zip(rows, envelope)):
        if index:
            from_rest = min(cap, math.sqrt(from_rest ** 2 + 2 * config.launch_acceleration_plan_mps2 * (row["planar_s_m"] - rows[index - 1]["planar_s_m"])))
        row.update(backward_speed_cap_mps=cap, planned_from_rest_speed_mps=from_rest)
    spans = qualifying_spans(rows, config.cruise_minimum_speed_mps)
    nominal_coast_distance = config.stop_buffer_m + config.empirical_coast_seconds * config.approach_reference_speed_mps
    coast = interpolate(rows, length - nominal_coast_distance)
    return {"schema": "carla.offline_curvature_speed_preview.v1", "spacing_m": spacing, "config": asdict(config),
        "source_point_count": len(route["route"]), "duplicate_identical_planar_points_removed_for_geometry_only": duplicates,
        "sample_count": len(rows), "planar_route_length_m": length, "catalog_route_length_m": geometry[-1]["catalog_s_m"],
        "planar_minus_catalog_length_m": length - geometry[-1]["catalog_s_m"],
        "maximum_abs_curvature_per_m": max(abs(k) for k in curves),
        "minimum_curvature_cap_mps": min(r["curvature_cap_mps"] for r in rows),
        "sampled_planned_cruise_spans": spans, "total_sampled_planned_cruise_length_m": sum(s["length_m"] for s in spans),
        "longest_sampled_planned_cruise_length_m": max((s["length_m"] for s in spans), default=0),
        "coast_reference_probe": {"remaining_planar_m": nominal_coast_distance, "planar_s_m": coast["planar_s_m"],
            "backward_speed_cap_mps": coast["backward_speed_cap_mps"], "curvature_cap_mps": coast["curvature_cap_mps"],
            "required_actual_entry_band_mps": [2.8, 3.2], "actual_coast_feasibility_measured": False},
        "samples": rows, "policy_implemented": False, "training_data_approved": False,
        "notice": "Offline polyline target only. Curvature endpoint estimates repeat the adjacent interior estimate. No spline smoothing or vehicle/target label rewriting. Backward -0.6 and from-rest +1.0 are ideal planning assumptions, not measured zero-brake authority. The terminal target is deliberately not zero: unchanged V4 actual-speed/coast/goal-dwell logic would still be required, but is not adopted or executed here."}


def build_report(plan_path, repository):
    """HH_260906 - Read only the exact two train/validation planning routes; no test, image or trajectory payload."""
    plan_path, repository = Path(plan_path), Path(repository)
    raw_plan = plan_path.read_bytes()
    plan = json.loads(raw_plan)
    routes = plan["longer_existing_routes"]
    if [(r["site_id"], r["split"], r["scenario"]) for r in routes] != [("C_track_1_0_7", "train", "left"), ("Town03", "val", "right")]:
        raise ValueError("only the exact proposed C-track train-left and Town03 validation-right route records are accepted")
    results, ledgers = [], []
    for item in routes:
        loaded = []
        for kind in ("native", "source"):
            relative = item[kind + "_route_path"]
            if Path(relative).is_absolute() or ".." in Path(relative).parts:
                raise ValueError("route path must be repository-relative")
            path = repository / relative
            data = path.read_bytes()
            digest = hashlib.sha256(data).hexdigest()
            if digest != item[kind + "_route_sha256"]:
                raise ValueError("frozen route SHA differs")
            loaded.append(json.loads(data))
            ledgers.append(dict(path=relative, sha256=digest, size_bytes=len(data)))
        native, original = loaded
        if native["town"] != item["site_id"] or native["scenario"] != item["scenario"]:
            raise ValueError("route map or turn direction mismatch")
        matching_xy = [(r["x"], r["y"]) for r in native["route"]] == [(r["x"], r["y"]) for r in original["route"]]
        previews = [preview(native, spacing) for spacing in (1., 2.)]
        one, two = previews
        sensitivity = [abs(r["backward_speed_cap_mps"] - interpolate(two["samples"], r["planar_s_m"])["backward_speed_cap_mps"]) for r in one["samples"]]
        results.append({"site_id": item["site_id"], "split": item["split"], "scenario": item["scenario"],
            "native_route_path": item["native_route_path"], "native_route_sha256": item["native_route_sha256"],
            "original_native_xy_geometry_identical": matching_xy, "route_coordinate_alignment": item["coordinate_alignment"],
            "command_tag_lead_m": item["route_lead_distance_m"], "command_tag_tail_m": item["route_tail_distance_m"],
            "curvature_not_limited_to_command_tags": True, "previews": previews,
            "spacing_sensitivity_max_speed_cap_difference_mps": max(sensitivity),
            "spacing_sensitivity_maximum_at_planar_s_m": one["samples"][sensitivity.index(max(sensitivity))]["planar_s_m"],
            "same_route_repetition_not_new_independent_geometry": True})
    for row in ledgers:
        if hashlib.sha256((repository / row["path"]).read_bytes()).hexdigest() != row["sha256"]:
            raise ValueError("route changed during geometry preview")
    return {"schema": "portable_e2e.offline_turn_speed_preview_proposal.v1", "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "source_plan_sha256": hashlib.sha256(raw_plan).hexdigest(), "source_code_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "route_source_manifest": ledgers, "routes": results,
        "scope": {"policy_implemented": False, "collector_modified": False, "live_simulator_access": False, "model_loaded": False,
            "test_payload_used": False, "old_corpus_changed": False, "training_data_approved": False},
        "fixed_operating_constraints_if_later_reviewed": {"physics_hz": 20, "camera_hz": 10, "transport": "acknowledged_batch",
            "normal_brake_cap": 0, "nominal_speed_mps": 8, "maximum_actual_speed_mps": 30 / 3.6,
            "emergency_control_unchanged": True, "goal_tolerance_m": 1, "stop_speed_mps": .1, "stop_dwell_seconds": 2,
            "moving_tail_forbidden": True, "existing_Town07_v4_guard_not_changed": True},
        "risks": ["Zero normal brake cannot guarantee following a backward deceleration target; actual overspeed or coast-entry failure must remain a failed trial.",
            "1m/2m sampling sensitivity is a diagnostic, not a proof of continuous road curvature bounds; endpoint seams and polyline corners are retained.",
            "Catalog 3D arc and planar preview arc differ; a reviewed integration must explicitly map progress rather than substitute one coordinate silently.",
            "No road grade, tire dynamics, collision clearance, traffic hazards, camera quality, future XY feasibility or measured cruise is established.",
            "C-track requires its own approved map/rendering preflight; the existing Low/Epic comparison is not transferable performance evidence."]}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--repository", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    if args.output.exists() or args.output.is_symlink():
        raise ValueError("create-only output already exists")
    report = build_report(args.plan, args.repository)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write("\n")
    print(json.dumps({"output": str(args.output), "routes": len(report["routes"]), "policy_implemented": False}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
