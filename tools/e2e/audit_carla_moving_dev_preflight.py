#!/usr/bin/env python3
"""HH_260906 - Audit a prospective DEV route without changing sealed capture or admission code."""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re

from scripts.e2e import plan_portable_moving_cohort as cohort
from portable_e2e.workspace_paths import workspace_path

ROOT = Path(__file__).resolve().parents[2]
ROUTE_ID = "town03_straight_s0000_p04"
ROUTE_SHA = "fe40145556a1e946b87f6c6c846df4c1e8cc08cc2fc75047b6b426bd12408812"
SLOT_ID = "V03-S01"
REVIEW_DISTANCE_M = 5.0
FRAME_REFERENCE_PATH = "datasets/raw/carla/common10_v1/2026-09-05/30kph/town03/turn/qualified/b1_compact_lookahead/ClearNoon/seed_0000/run_002/episode/route.json"
FRAME_REFERENCE_SHA = "47ea507b9a4228fdce79ef58eff1e1d2cf149c02dd2df413ebadf53756f771ee"
MAPS = {"Town07": "train", "Town01": "train", "C_track_1_0_7": "train", "Town03": "val"}
SOURCE_FILES = (
    "scripts/e2e/audit_carla_moving_dev_preflight.py",
    "scripts/e2e/plan_portable_moving_cohort.py",
    "scripts/e2e/plan_portable_map_coverage.py",
    "scripts/e2e/inventory_carla_training_maps.py",
    "scripts/e2e/carla_moving_capture.py",
    "scripts/e2e/qualify_carla_moving_common10.py",
)
require, safe = cohort.require, cohort.safe


def read_json(path, ledger, expected=None):
    return cohort.decode(cohort.read(path, ledger, expected))


def polyline(value):
    require(type(value) is list and 2 <= len(value) <= 5000, "bounded polyline required")
    points = []
    for point in value:
        require(type(point) is list and len(point) == 2, "XY route point required")
        points.append(tuple(cohort.numeric(x, "route coordinate") for x in point))
    require(any(a != b for a, b in zip(points, points[1:])), "route is degenerate")
    return points


def point_segment_distance(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    squared = dx * dx + dy * dy
    t = 0.0 if squared == 0 else max(0.0, min(1.0, ((p[0]-a[0])*dx + (p[1]-a[1])*dy)/squared))
    return math.hypot(p[0]-a[0]-t*dx, p[1]-a[1]-t*dy)


def segment_distance(a, b, c, d):
    # HH_260906 - Include interior crossings; endpoint-only comparisons can miss route intersections.
    def cross(u, v, w):
        return (v[0]-u[0])*(w[1]-u[1]) - (v[1]-u[1])*(w[0]-u[0])
    ab_c, ab_d, cd_a, cd_b = cross(a, b, c), cross(a, b, d), cross(c, d, a), cross(c, d, b)
    if ab_c * ab_d < 0 and cd_a * cd_b < 0:
        return 0.0
    return min(point_segment_distance(a, c, d), point_segment_distance(b, c, d),
               point_segment_distance(c, a, b), point_segment_distance(d, a, b))


def compare_routes(candidate, previous):
    segments = list(zip(previous, previous[1:]))
    distances = [min(point_segment_distance(p, a, b) for a, b in segments) for p in candidate]
    minimum = min(segment_distance(a, b, c, d)
                  for a, b in zip(candidate, candidate[1:]) for c, d in segments)
    near = sum(distance <= REVIEW_DISTANCE_M for distance in distances)
    return dict(minimum_polyline_distance_m=minimum, candidate_vertex_count=len(candidate),
                candidate_vertices_within_review_distance=near,
                candidate_vertex_fraction_within_review_distance=near/len(candidate),
                spatial_review_needed=minimum <= REVIEW_DISTANCE_M,
                review_distance_m=REVIEW_DISTANCE_M,
                notice="5m is a conservative review trigger, not an admission threshold or proof of independence.")


def verify_reference_frame(previous, ledger):
    # HH_260906 - A frame name alone is insufficient; pin the native identity alignment and exact prepared XY.
    native = read_json(ROOT / FRAME_REFERENCE_PATH, ledger, FRAME_REFERENCE_SHA)
    alignment = native.get("coordinate_alignment", {})
    require(native.get("town") == "Town03" and alignment.get("source_frame") == "carla_map"
            and alignment.get("target_frame") == "map" and alignment.get("map_bundle_profile") == "town03",
            "reference native map alignment differs")
    transform = alignment.get("carla_to_map_transform", {})
    require(set(transform) == {"x_m", "y_m", "z_m", "yaw_rad"}
            and all(cohort.numeric(value, "frame transform") == 0.0 for value in transform.values()),
            "reference frame is not the pinned identity alignment")
    points = polyline([[point["x"], point["y"]] for point in native["route"]])
    prepared = []
    for point in points:
        if not prepared or math.dist(prepared[-1], point) > 1e-9:
            prepared.append(point)
    require(prepared == previous, "prepared geometry does not match pinned native reference")
    return dict(status="PINNED_IDENTITY_ALIGNMENT_AND_NATIVE_XY_MATCH", native_route_sha256=FRAME_REFERENCE_SHA,
                native_route_path=FRAME_REFERENCE_PATH, alignment=alignment,
                candidate_route_sha256=ROUTE_SHA, coordinate_space="ROS world XY in Town03")


def reference_metadata(roots, ledger):
    # HH_260906 - Finish all split checks before opening any route geometry, samples or provenance payload.
    episodes, identities, groups, sessions = [], set(), {}, {}
    for root in roots:
        manifest = read_json(root / "dataset.json", ledger)
        require(manifest.get("schema_id") == "autoware-e2e.common10.dataset.v1"
                and manifest.get("contract_id") == "common_10hz_v1", "Common10 dataset required")
        refs = manifest.get("episodes")
        require(type(refs) is list and 1 <= len(refs) <= 1000, "bounded episode references required")
        for ref in refs:
            eid = ref.get("episode_id")
            require(type(eid) is str and re.fullmatch(r"[A-Za-z0-9_.-]+", eid)
                    and eid not in identities, "unique safe episode ID required")
            require(not re.search(r"town(?:04|15)(?:[_.-]|$)", eid.lower()), "protected episode reference")
            require(ref.get("manifest") == f"episodes/{eid}/episode.json", "contained episode path required")
            path = root / ref["manifest"]
            episode = read_json(path, ledger, cohort.pin(ref.get("sha256")))
            require(episode.get("split") in ("train", "val"), "TEST/unknown split metadata refused")
            require(episode.get("map_id") in MAPS, "protected or unreviewed map refused")
            require(episode["split"] == MAPS[episode["map_id"]], "active map split policy differs")
            require(episode.get("schema_id") == "autoware-e2e.common10.episode.v1"
                    and episode.get("episode_id") == eid and episode.get("domain") == "carla",
                    "episode identity differs")
            require(episode.get("site_id") == episode["map_id"], "site/map identity differs")
            for field, assignments in (("split_group_id", groups), ("source_session_id", sessions)):
                value = episode.get(field)
                require(type(value) is str and value, "missing split/session identity")
                require(value not in assignments or assignments[value] == episode["split"],
                        "cross-split group/session leakage")
                assignments[value] = episode["split"]
            require(type(episode.get("route_id")) is str and episode["route_id"], "route ID required")
            require(episode.get("route_geometry_file") == "route_geometry.json", "fixed geometry path required")
            cohort.pin(episode.get("route_geometry_sha256"))
            episodes.append((path.parent, episode))
            identities.add(eid)
    require({ep["split"] for _, ep in episodes} == {"train", "val"}, "both TRAIN and DEV references required")
    return episodes


def build_report(catalog_root, reference_roots):
    ledger = {}
    for name in SOURCE_FILES:
        # HH_260906 - Open canonical source files without rewriting source identifiers.
        cohort.read(workspace_path(ROOT, name), ledger)
    candidates, _ = cohort.load_candidates(catalog_root, "town03", ledger)
    matches = [row for row in candidates if row["route_id"] == ROUTE_ID]
    require(len(matches) == 1 and matches[0]["route_sha256"] == ROUTE_SHA, "prospective DEV route pin differs")
    route = matches[0]
    require(route["scenario"] == "straight" and route["canonical_name"] == "Town03", "DEV route identity differs")
    native = read_json(route["route_path"], ledger, ROUTE_SHA)
    candidate = polyline([[p["x"], p["y"]] for p in native["route"]])
    references = reference_metadata(reference_roots, ledger)
    comparisons = []
    for directory, episode in references:
        item = {key: episode[key] for key in ("episode_id", "split", "map_id", "route_id", "source_session_id", "split_group_id")}
        if episode["map_id"] != "Town03":
            # HH_260906 - Numeric world coordinates from different maps are not a common spatial frame.
            item.update(status="DIFFERENT_MAP_NO_XY_COMPARISON", geometry_payload_read=False)
        else:
            geometry = read_json(directory / "route_geometry.json", ledger, episode["route_geometry_sha256"])
            require(geometry.get("frame_id") == "map" and geometry.get("route_id") == episode["route_id"],
                    "prepared route frame/identity differs")
            previous = polyline(geometry.get("polyline_m"))
            frame_proof = verify_reference_frame(previous, ledger)
            metrics = compare_routes(candidate, previous)
            item.update(metrics, status="SPATIAL_REVIEW_REQUIRED" if metrics["spatial_review_needed"] else "NO_5M_PROXIMITY_OBSERVED",
                        geometry_payload_read=True, shared_frame_proof=frame_proof)
        comparisons.append(item)
    end_with_future = 165.0 + (15.0/3.6)*6.4
    margin = route["recomputed_planar_length_m"] - end_with_future
    geometry_blockers = []
    if margin <= 0:
        geometry_blockers.append("INSUFFICIENT_ROUTE_AFTER_FIXED_WINDOW_AND_NOMINAL_FUTURE")
    if any(row.get("spatial_review_needed") for row in comparisons):
        geometry_blockers.append("PRIOR_DEV_ROUTE_PROXIMITY_REVIEW_REQUIRED")
    remaining = ["VERSIONED_CAPTURE_CONTRACT_AND_OWNER_SOURCE_CLOSURE_NOT_IMPLEMENTED",
                 "GENERALIZED_RAW_AUDIT_AND_DEV_CONVERSION_NOT_IMPLEMENTED",
                 "FRESH_OWNER_PORT_READINESS_AND_SOURCE_SESSION_NOT_OBSERVED"]
    for path, digest in tuple(ledger.items()):
        cohort.read(path, {}, digest)
    return dict(schema="carla.moving_dev_preflight.v1", status="PREFLIGHT_ONLY_NOT_CAPTURE_AUTHORIZED",
        generated_at_utc=datetime.now(timezone.utc).isoformat(),
        proposed_case=dict(slot_id=SLOT_ID, split="val", use="development_only_not_release_test", town="Town03",
            route_id=ROUTE_ID, route_sha256=ROUTE_SHA, route_path=route["route_path"],
            traffic_seed=201, weather="ClearNoon", target_speed_kmh=15.0, camera_hz=10, physics_hz=20,
            selected_progress_m=[30.0, 165.0], minimum_selected_span_s=30.0, required_future_s=6.4,
            maximum_attempts=1, automatic_retry=False, source_session_id=None),
        split_policy_notice="Explicit proposed DEV role; the legacy suite TRAIN label is not changed or inherited.",
        geometry=dict(route_planar_length_m=route["recomputed_planar_length_m"],
            nominal_window_seconds=(165.0-30.0)/(15.0/3.6), nominal_future_end_progress_m=end_with_future,
            nominal_remaining_route_m=margin, measured_timing_available=False,
            preliminary_checks_clear=not geometry_blockers, blockers=geometry_blockers),
        reference_episode_count=len(references), reference_split_counts=dict(Counter(ep["split"] for _, ep in references)),
        comparisons=comparisons, remaining_before_capture=remaining,
        remaining_after_capture=["MEASURED_30S_WINDOW_AND_6_4S_FUTURE_NOT_OBSERVED",
                                 "RAW_QUALITY_AND_EXPLICIT_DEV_ADMISSION_NOT_PERFORMED"],
        limitations=["Only the explicitly supplied datasets were checked, not every historical run.",
            "Prepared route geometry is not an audit of actual native driven trajectories.",
            "Same-map separation does not certify statistical independence or a new source session.",
            "This report is not a capture plan, admission receipt, speed validation or model evaluation.",
            "Legacy manifests, sealed outputs, capture code and quality thresholds remain unchanged."],
        execution_allowed=False, dataset_admission_approved=False, training_executed=False,
        world_rpc_performed=False, gpu_used=False, test_payloads_read=False,
        images_or_samples_read=False, input_sha256_before=ledger, input_sha256_after=dict(ledger))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--catalog-maps-root", required=True, type=Path)
    parser.add_argument("--reference-dataset", required=True, action="append", type=Path)
    parser.add_argument("--output-json", required=True, type=Path)
    args = parser.parse_args(argv)
    catalogs, output = safe(args.catalog_maps_root), safe(args.output_json)
    roots = [safe(root) for root in args.reference_dataset]
    require(len(set(roots)) == len(roots), "duplicate reference roots")
    require(not output.exists() and not output.is_relative_to(catalogs)
            and not any(output.is_relative_to(root) for root in roots), "fresh output disjoint from inputs required")
    report = build_report(catalogs, roots)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write("\n")
    digest = hashlib.sha256(output.read_bytes()).hexdigest()
    print(json.dumps(dict(status=report["status"], reference_episodes=report["reference_episode_count"],
        geometry=report["geometry"], output_sha256=digest)))


if __name__ == "__main__":
    main()
