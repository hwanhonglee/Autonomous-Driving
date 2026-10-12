#!/usr/bin/env python3
"""HH_260906 - Resolve DEV corridor proximity using continuous XY arcs, never simulator or model outputs."""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path

from scripts.e2e import audit_carla_moving_dev_preflight as preflight

ROOT = Path(__file__).resolve().parents[2]
require, safe, read = preflight.require, preflight.safe, preflight.read_json
START, END, FUTURE = 30.0, 165.0, 165.0 + 15.0 / 3.6 * 6.4
RADIUS = 5.0
BASELINE_SHA = "339f946dce3cc491ecf4dd09c231cefaf05309fb617f589574c24dc5bf677e7c"
POLICY = dict(version="dev_straight_corridor_selection.v1", target_speed_kmh=15.0,
    selected_arc_m=[START, END], nominal_future_end_arc_m=FUTURE, minimum_nominal_end_margin_m=2.5,
    review_distance_m=RADIUS, require_no_shared_road_sections=True,
    require_no_5m_proximity_over_full_route=True,
    order="Eligible routes first; largest full-route minimum distance, then route ID.",
    notice="Prospective geometry policy, not a statistical independence certificate or raw quality admission.")


def intersect(first, second):
    lo, hi = max(first[0], second[0]), min(first[1], second[1])
    return (lo, hi) if lo <= hi else None


def linear_band(offset, slope, lo, hi):
    if abs(slope) < 1e-15:
        return (0.0, 1.0) if lo <= offset <= hi else None
    limits = sorted(((lo-offset)/slope, (hi-offset)/slope))
    return intersect((0.0, 1.0), limits)


def circle_interval(a, b, center, radius):
    dx, dy, x, y = b[0]-a[0], b[1]-a[1], a[0]-center[0], a[1]-center[1]
    q = dx*dx + dy*dy
    c = x*x + y*y - radius*radius
    if q == 0:
        return (0.0, 1.0) if c <= 0 else None
    linear = 2*(x*dx+y*dy)
    discriminant = linear*linear - 4*q*c
    if discriminant < 0:
        return None
    root = math.sqrt(discriminant)
    return intersect((0.0, 1.0), ((-linear-root)/(2*q), (-linear+root)/(2*q)))


def merge_intervals(intervals):
    result = []
    for lo, hi in sorted(intervals):
        if result and lo <= result[-1][1] + 1e-9:
            result[-1][1] = max(hi, result[-1][1])
        else:
            result.append([lo, hi])
    return result


def capsule_intervals(a, b, c, d, radius):
    """HH_260906 - Intersect a moving point with a segment's closed-radius capsule analytically."""
    require(type(radius) in (float, int) and math.isfinite(radius) and radius >= 0, "finite nonnegative radius required")
    intervals = [value for value in (circle_interval(a, b, c, radius), circle_interval(a, b, d, radius)) if value is not None]
    ux, uy, vx, vy = d[0]-c[0], d[1]-c[1], b[0]-a[0], b[1]-a[1]
    length2 = ux*ux+uy*uy
    if length2 > 0:
        wx, wy = a[0]-c[0], a[1]-c[1]
        along = linear_band(wx*ux+wy*uy, vx*ux+vy*uy, 0.0, length2)
        width = radius*math.sqrt(length2)
        lateral = linear_band(ux*wy-uy*wx, ux*vy-uy*vx, -width, width)
        if along is not None and lateral is not None:
            value = intersect(along, lateral)
            if value is not None:
                intervals.append(value)
    return merge_intervals(intervals)


def clip_intervals(intervals, lo, hi):
    return merge_intervals([value for item in intervals if (value := intersect(item, (lo, hi))) is not None])


def summarize_region(intervals, lo, hi):
    require(0 <= lo <= hi, "ordered nonnegative arc region required")
    clipped = clip_intervals(intervals, lo, hi)
    length = math.fsum(b-a for a, b in clipped)
    return dict(arc_range_m=[lo, hi], proximity_intervals_m=clipped, proximity_length_m=length,
                region_length_m=hi-lo, proximity_fraction=length/(hi-lo) if hi > lo else None)


def lane_key(point):
    values = [point.get(name) for name in ("road_id", "section_id", "lane_id")]
    require(all(type(value) is int for value in values), "native road/section/lane identity required")
    return values


def arc_analysis(candidate, previous):
    xy = preflight.polyline([[p["x"], p["y"]] for p in candidate["route"]])
    prior = preflight.polyline([[p["x"], p["y"]] for p in previous["route"]])
    reference_segments = list(zip(prior, prior[1:]))
    arc, intervals, details, distance_samples = 0.0, [], [], []
    for index, (a, b) in enumerate(zip(xy, xy[1:])):
        length = math.dist(a, b)
        distance_samples.append([arc, min(preflight.point_segment_distance(a, c, d) for c, d in reference_segments)])
        if length == 0:
            continue
        for old_index, (c, d) in enumerate(reference_segments):
            near = capsule_intervals(a, b, c, d, RADIUS)
            if not near:
                continue
            mapped = [[arc+lo*length, arc+hi*length] for lo, hi in near]
            intervals.extend(mapped)
            first, second = lane_key(candidate["route"][index]), lane_key(previous["route"][old_index])
            old_length = math.dist(c, d)
            cosine = None if old_length == 0 else ((b[0]-a[0])*(d[0]-c[0]) + (b[1]-a[1])*(d[1]-c[1]))/(length*old_length)
            angle = None if cosine is None else math.degrees(math.acos(max(-1.0, min(1.0, cosine))))
            details.append(dict(candidate_segment=index, reference_segment=old_index,
                candidate_arc_intervals_m=mapped, candidate_road_section_lane=first,
                reference_road_section_lane=second, same_road_section=first[:2] == second[:2],
                same_lane_identity=first == second, heading_difference_deg=angle))
        arc += length
    distance_samples.append([arc, min(preflight.point_segment_distance(xy[-1], c, d) for c, d in reference_segments)])
    merged = merge_intervals(intervals)
    same_roads = {tuple(lane_key(p)[:2]) for p in candidate["route"]} & {tuple(lane_key(p)[:2]) for p in previous["route"]}
    relations = Counter(("same_lane" if row["same_lane_identity"] else "same_road_other_lane" if row["same_road_section"] else "different_road") for row in details)
    angles = [row["heading_difference_deg"] for row in details if row["heading_difference_deg"] is not None]
    return dict(route_length_m=arc, full_route=summarize_region(merged, 0.0, arc),
        selected_window=summarize_region(merged, min(START, arc), min(END, arc)),
        nominal_future_support=summarize_region(merged, END, min(FUTURE, arc)) if arc >= END else None,
        shared_road_sections=sorted(same_roads), proximity_segment_pair_counts=dict(relations),
        proximity_heading_difference_deg_range=[min(angles), max(angles)] if angles else None,
        minimum_polyline_distance_m=min(preflight.segment_distance(a, b, c, d)
            for a, b in zip(xy, xy[1:]) for c, d in reference_segments),
        distance_vs_planar_arc_m=distance_samples, proximity_segment_pairs=details,
        full_route_xy=xy, reference_route_xy=prior,
        method="Analytic segment/capsule intersection and union of candidate planar-arc intervals; no vertex-count approximation.")


def check_candidate_frame(route):
    require(route.get("town") == "Town03" and route.get("coordinate_reference") == "base_link", "Town03 ROS route required")
    # HH_260906 - The catalog uses CARLA-to-ROS handedness, not a map-dependent translation or local ego origin.
    for prefix in ("start", "goal"):
        carla, ros = route[prefix+"_carla_transform"], route[prefix+"_ros_pose"]
        for key, expected in (("x", carla["x"]), ("y", -carla["y"])):
            require(abs(preflight.cohort.numeric(ros[key], key)-preflight.cohort.numeric(expected, key)) <= 1e-6,
                    "candidate CARLA/ROS endpoint frame differs")


def build_report(catalog_root, baseline_report):
    ledger = {}
    for name in (*preflight.SOURCE_FILES, "scripts/e2e/analyze_carla_dev_route_overlap.py"):
        preflight.cohort.read(ROOT/name, ledger)
    baseline = read(baseline_report, ledger, BASELINE_SHA)
    require(baseline.get("schema") == "carla.moving_dev_preflight.v1"
            and baseline.get("status") == "PREFLIGHT_ONLY_NOT_CAPTURE_AUTHORIZED", "audited baseline required")
    require(baseline.get("proposed_case", {}).get("route_sha256") == preflight.ROUTE_SHA, "baseline candidate pin differs")
    require(baseline["input_sha256_before"] == baseline["input_sha256_after"], "baseline input snapshots differ")
    for name, digest in baseline["input_sha256_after"].items():
        preflight.cohort.read(name, ledger, digest)
    reference = read(ROOT/preflight.FRAME_REFERENCE_PATH, ledger, preflight.FRAME_REFERENCE_SHA)
    prepared_points = preflight.polyline([[p["x"], p["y"]] for p in reference["route"]])
    dedup = []
    for point in prepared_points:
        if not dedup or math.dist(dedup[-1], point) > 1e-9:
            dedup.append(point)
    frame = preflight.verify_reference_frame(dedup, ledger)
    frame.pop("candidate_route_sha256")
    candidates, _ = preflight.cohort.load_candidates(catalog_root, "town03", ledger)
    records = []
    for row in sorted(candidates, key=lambda x: x["route_id"]):
        if row["scenario"] != "straight":
            continue
        native = read(row["route_path"], ledger, row["route_sha256"])
        check_candidate_frame(native)
        analysis = arc_analysis(native, reference)
        require(abs(analysis["route_length_m"]-row["recomputed_planar_length_m"]) < 1e-7, "planar arc mismatch")
        margin = analysis["route_length_m"]-FUTURE
        reasons = []
        if analysis["minimum_polyline_distance_m"] <= RADIUS:
            reasons.append("WITHIN_5M_OF_PRIOR_DEV_ROUTE")
        if analysis["shared_road_sections"]:
            reasons.append("SHARES_PRIOR_DEV_ROAD_SECTION")
        if margin < POLICY["minimum_nominal_end_margin_m"]:
            reasons.append("INSUFFICIENT_NOMINAL_FUTURE_AND_END_MARGIN")
        records.append(dict(route_id=row["route_id"], route_path=row["route_path"], route_sha256=row["route_sha256"],
            geometry_candidate_eligible=not reasons, exclusion_reasons=reasons,
            nominal_remaining_route_m=margin, analysis=analysis))
    require(len(records) == 8, "expected all eight fixed Town03 straight candidates")
    eligible = sorted((r for r in records if r["geometry_candidate_eligible"]),
                      key=lambda r: (-r["analysis"]["minimum_polyline_distance_m"], r["route_id"]))
    selected = None if not eligible else {k: eligible[0][k] for k in ("route_id", "route_path", "route_sha256")}
    for path, digest in tuple(ledger.items()):
        preflight.cohort.read(path, {}, digest)
    return dict(schema="carla.dev_route_overlap_resolution.v1", status="GEOMETRY_ROUTE_SELECTED_NOT_CAPTURE_AUTHORIZED" if selected else "NO_ELIGIBLE_GEOMETRY_CANDIDATE",
        generated_at_utc=datetime.now(timezone.utc).isoformat(), policy=POLICY,
        baseline_candidate_id=preflight.ROUTE_ID, reference_frame_proof=frame,
        candidate_count=len(records), eligible_candidate_count=len(eligible), selected_candidate=selected,
        candidates=records, proposed_split="val", source_session_id=None,
        execution_allowed=False, training_executed=False, dataset_admission_approved=False,
        world_rpc_performed=False, test_payloads_read=False,
        limitations=["Candidate selection uses route geometry only, not model scores or TEST observations.",
            "The 5m corridor is a conservative review policy, not statistical independence or image-distribution proof.",
            "Road/lane labels in segment-pair details are those at the segment start; transition boundaries are approximate.",
            "Future support distance assumes nominal 15km/h; actual 30s and 6.4s timestamps must be checked after capture.",
            "No raw driven-trajectory overlap, new session uniqueness, capture, conversion or model test occurred."],
        source_and_input_sha256=ledger)


def render(report, output):
    # HH_260906 - These plots visualize authenticated route geometry, not camera footage or an autonomous drive.
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    selected = report["selected_candidate"]
    chosen = [next(r for r in report["candidates"] if r["route_id"] == report["baseline_candidate_id"])]
    if selected and selected["route_id"] != chosen[0]["route_id"]:
        chosen.append(next(r for r in report["candidates"] if r["route_id"] == selected["route_id"]))
    fig, axes = plt.subplots(2, len(chosen), figsize=(8*len(chosen), 11), squeeze=False)
    for column, row in enumerate(chosen):
        data = row["analysis"]
        ax = axes[0, column]
        old, new = data["reference_route_xy"], data["full_route_xy"]
        ax.plot(*zip(*old), color="black", linewidth=2, label="Existing DEV right turn")
        ax.plot(*zip(*new), color="#0072b2", linewidth=2, label="Candidate straight")
        ax.scatter([new[0][0]], [new[0][1]], color="#009e73", marker="o", s=65, label="Candidate start")
        ax.scatter([new[-1][0]], [new[-1][1]], color="#d55e00", marker="x", s=65, label="Candidate end")
        ax.set_title(row["route_id"]+"\n"+f"Minimum separation: {data['minimum_polyline_distance_m']:.3f} m")
        ax.set_xlabel("ROS world X [m]"); ax.set_ylabel("ROS world Y [m]")
        ax.axis("equal"); ax.grid(alpha=.25); ax.legend(loc="best", fontsize=9)
        ax = axes[1, column]
        ax.plot(*zip(*data["distance_vs_planar_arc_m"]), color="#0072b2", label="Vertex distance to prior polyline")
        ax.axhline(RADIUS, color="#d55e00", linestyle="--", label="5 m review distance")
        ax.axvspan(START, END, color="#009e73", alpha=.12, label="Selected anchors: 30-165 m")
        ax.axvspan(END, FUTURE, color="#e69f00", alpha=.14, label="Nominal 6.4 s future support")
        for lo, hi in data["full_route"]["proximity_intervals_m"]:
            ax.axvspan(lo, hi, color="#d55e00", alpha=.2)
        ax.set_xlabel("Candidate planar route arc [m]"); ax.set_ylabel("Distance to existing DEV [m]")
        ax.set_title(f"Exact near-arc length in selected window: {data['selected_window']['proximity_length_m']:.3f} m")
        ax.grid(alpha=.25); ax.legend(loc="best", fontsize=8)
    fig.suptitle("OFFLINE ROUTE GEOMETRY - NOT A DRIVE / NO NEW TRAINING", fontsize=16)
    fig.tight_layout(rect=(0, 0, 1, .96))
    fig.savefig(output, dpi=140)
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--catalog-maps-root", required=True, type=Path)
    parser.add_argument("--baseline-preflight", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--render", action="store_true")
    args = parser.parse_args(argv)
    catalogs, baseline, output = safe(args.catalog_maps_root), safe(args.baseline_preflight), safe(args.output_dir)
    require(not output.exists() and not output.is_relative_to(catalogs) and not baseline.is_relative_to(output), "fresh disjoint output directory required")
    report = build_report(catalogs, baseline)
    output.mkdir(parents=True)
    target = output/"route_resolution.json"
    target.write_text(json.dumps(report, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    if args.render:
        render(report, output/"route_comparison.png")
    print(json.dumps(dict(status=report["status"], selected=report["selected_candidate"],
        eligible=report["eligible_candidate_count"], report_sha256=hashlib.sha256(target.read_bytes()).hexdigest())))


if __name__ == "__main__":
    main()
