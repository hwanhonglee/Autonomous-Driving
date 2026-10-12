#!/usr/bin/env python3
"""HH_260906 - Fail-closed offline ranking for the completed Town01 V5 right-turn catalog."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

from scripts.e2e import audit_new_long_turn_candidates as turn
from scripts.e2e import audit_future_actor_lane_containment as lane
from scripts.e2e import rank_town01_wide_turns as nominal


ROOT = Path(__file__).resolve().parents[2]
OWNER = ROOT / "storage/experiments/training/2026-09-27/drive_long_route_town01_train_catalog_v5"
V4 = nominal.CATALOG
V4_AUDIT = nominal.STATIC_AUDIT
HISTORICAL_TRAIN = turn.REFERENCE
C_EPISODE = nominal.ARCHIVED_EPISODE
OUTPUT = ROOT / "docs/assets/validation/2026-09-27/drive_town01_v5_rank/rank.json"
MAP_SHA256 = "97a7f6ac67812567e5c8ee0599cd823b23f80f30f3f97c502212e38b72e2b709"
START_M = 30.0
END_M = 120.0
FUTURE_AT_9_KMH_M = 9.0 / 3.6 * 6.4
SAMPLE_STEP_M = 0.10


class InvalidEvidence(ValueError):
    """HH_260906 - Missing or inconsistent ownership cannot create a ranked route."""


def require(condition: bool, reason: str) -> None:
    if not condition:
        raise InvalidEvidence(reason)


def digest(path: Path) -> str:
    require(path.is_file() and not path.is_symlink(), f"regular file required: {path}")
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path: Path) -> dict:
    try:
        document, _ = turn.read_json(path)
    except turn.AuditError as error:
        raise InvalidEvidence(str(error)) from error
    return document


def completed_v5_owner(owner_root: Path) -> tuple[dict, dict, dict]:
    """HH_260906 - Recheck final owner, archived sources and three CARLA lifecycle receipts."""
    plan = read(owner_root / "owner_plan.json")
    require(plan.get("schema") == "portable_e2e.owned_turn_route_catalog.v1" and
            plan.get("map_id") == "town01" and plan.get("map") == "Town01" and
            plan.get("split_intent") == "TRAIN_CANDIDATE" and
            plan.get("quality") == "Epic" and plan.get("allow_map_load") is False and
            plan.get("capture_executed") is False and plan.get("training_executed") is False and
            plan.get("dataset_admission_granted") is False and plan.get("learned_model_control") is False,
            "V5 owner is not the preregistered TRAIN-candidate catalog-only plan")
    argv = plan.get("catalog_argv", [])
    require(isinstance(argv, list), "V5 search arguments missing")

    def option_value(name: str) -> str:
        require(argv.count(name) == 1 and argv.index(name) + 1 < len(argv),
                f"V5 search argument missing or repeated: {name}")
        return argv[argv.index(name) + 1]

    require(option_value("--seeds") == "4,5" and
            option_value("--preferred-distance") == "160" and
            option_value("--physical-turn-profile") == "speed_30kph",
            "V5 search flags changed")
    result = read(owner_root / "owner_result.json")
    require(result.get("schema") == "portable_e2e.owned_turn_route_catalog_result.v1" and
            result.get("exit_code") == 0 and result.get("catalog_status") == "complete" and
            result.get("source_bytes_unchanged_and_archived") is True and
            result.get("lifecycle_receipts_present") is True and
            result.get("capture_executed") is False and result.get("training_executed") is False and
            result.get("dataset_admission_granted") is False and result.get("learned_model_control") is False,
            "V5 owner has not completed cleanly")
    sources = plan.get("source_sha256")
    checks = result.get("source_checks")
    require(isinstance(sources, dict) and len(sources) == 13 and
            isinstance(checks, dict) and set(checks) == set(sources) and
            all(value is True for value in checks.values()), "V5 archived-source check ledger incomplete")
    for relative, expected in sources.items():
        name = Path(relative)
        require(not name.is_absolute() and ".." not in name.parts and
                isinstance(expected, str) and len(expected) == 64 and
                digest(owner_root / "provenance" / name) == expected,
                f"V5 archived source changed: {relative}")
    for stage in ("ready", "after_catalog", "stopped"):
        receipt = read(owner_root / "lifecycle" / f"{stage}.json")
        require(receipt.get("stage") == stage and receipt.get("status") == "PASS" and
                receipt.get("expected_map") == "Town01" and
                receipt.get("mode") == ("stopped" if stage == "stopped" else "running"),
                f"V5 lifecycle {stage} failed")
        if stage == "stopped":
            require(receipt.get("port_released") is True, "V5 CARLA port not released")
    require(digest(owner_root / "route_catalog.json") == result.get("catalog_sha256"),
            "V5 catalog hash differs from sealed owner")
    catalog = read(owner_root / "route_catalog.json")
    require(catalog.get("status") == "complete" and catalog.get("map_id") == "town01" and
            catalog.get("canonical_name") == "Town01" and
            catalog.get("server", {}).get("active_map_name") == "Carla/Maps/Town01" and
            catalog.get("server", {}).get("active_server_profile") == "packaged_0915" and
            catalog.get("server", {}).get("map_load_performed") is False,
            "V5 catalog map/provenance changed")
    rights = [entry for entry in catalog.get("routes", []) if entry.get("scenario") == "right"]
    require(len(rights) == result.get("right_candidate_count") == 16,
            "V5 right-candidate count differs from sealed owner")
    return plan, result, catalog


def fingerprint(route: dict) -> tuple[tuple[tuple[int, int, int], ...], tuple[float, float]]:
    shape = turn.turn_geometry(route)
    return tuple(shape["transitions"]), tuple(shape["center"])


def same_site(left: tuple, right: tuple) -> bool:
    """HH_260906 - Match one road/lane maneuver at one physical turn center."""
    return left[0] == right[0] and math.dist(left[1], right[1]) <= 5.0


def grouped_seam_geometry(route_points: list[dict]) -> tuple[list[dict], list[dict]]:
    """HH_260906 - Collapse pose-identical zero-arc nodes while retaining both lane IDs separately."""
    require(isinstance(route_points, list) and len(route_points) >= 2,
            "serialized route requires at least two points")
    geometry = [route_points[0]]
    seams = []
    for previous, point in zip(route_points, route_points[1:]):
        before = float(previous["distance_m"])
        after = float(point["distance_m"])
        require(math.isfinite(before) and math.isfinite(after) and after >= before,
                "route progress is nonfinite or backwards")
        if after == before:
            for name in ("x", "y", "z", "yaw"):
                left, right = float(previous[name]), float(point[name])
                require(math.isfinite(left) and math.isfinite(right) and abs(left - right) <= 1e-9,
                        "zero-arc seam changes pose")
            old_id, new_id = lane._triple(previous), lane._triple(point)
            seams.append({"arc_m": after, "from_lane": list(old_id), "to_lane": list(new_id),
                          "pose_identical": True,
                          "production_lane_gate_supported": old_id == new_id})
        else:
            geometry.append(point)
    require(len(geometry) >= 2, "route has no positive-length geometry")
    return geometry, seams


def provisional_nominal_clearance(route: dict, manifest: dict, xodr: bytes, carla: object,
                                  start_m: float = START_M, end_m: float = END_M) -> dict:
    """HH_260906 - Check an explicit fixed window while retaining both seam lane identities."""
    require(math.isfinite(start_m) and math.isfinite(end_m) and 0 <= start_m < end_m <=
            float(route["route_length_m"]), "invalid nominal window bounds")
    full_points = route["route"]
    geometry, seams = grouped_seam_geometry(full_points)
    carla_map = carla.Map("archived-active-Town01-V5-provisional", xodr.decode("utf-8"))
    box, radius = lane._body_box(manifest, carla)
    relevant = {node for i, (a, b) in enumerate(zip(full_points, full_points[1:]))
                if float(a["distance_m"]) <= end_m + radius and
                float(b["distance_m"]) >= start_m - radius
                for node in (i, i + 1)}
    requested = {lane._triple(full_points[index]) for index in relevant}
    corridors = lane.lane_corridors(carla_map, carla, full_points, xodr, requested, relevant)
    count = math.ceil((end_m - start_m) / SAMPLE_STEP_M)
    arcs = [start_m + (end_m - start_m) * i / count for i in range(count + 1)]
    poses, bodies, cursor = [], [], 0
    for arc in arcs:
        pose, cursor = nominal.pose_at_arc(geometry, arc, cursor)
        poses.append(pose)
        bodies.append(lane.body_footprint(box, pose, carla))
    regions = {}
    metrics = {"status": "PASS", "scope": "offline nominal route with both zero-arc seam lane IDs; not a driven trajectory",
               "sample_step_m": SAMPLE_STEP_M, "actor_rear_axle_offset_m": 1.425,
               "zero_arc_seams": seams,
               "production_lane_gate_supported": all(item["production_lane_gate_supported"] for item in seams),
               "body_count": len(bodies), "sweep_count": len(bodies) - 1,
               "raw_body_outside_count": 0, "inset_body_outside_count": 0,
               "raw_sweep_outside_count": 0, "inset_sweep_outside_count": 0,
               "maximum_raw_outside_m2": 0.0, "maximum_inset_outside_m2": 0.0,
               "minimum_raw_body_clearance_m": math.inf, "minimum_inset_body_clearance_m": math.inf,
               "minimum_raw_sweep_clearance_m": math.inf, "minimum_inset_sweep_clearance_m": math.inf,
               "physical_continuous_time_proven": False, "capture_authorized": False}

    def measure(shape: object, kind: str, lo: float, hi: float) -> None:
        identities = lane._route_lanes_for_arc_interval(full_points, lo - radius, hi + radius)
        for inset, label in ((0.0, "raw"), (lane.LANE_INSET_M, "inset")):
            key = frozenset(identities), inset
            if key not in regions:
                regions[key] = lane.authorized_lane_region(corridors, identities, inset)
            region = regions[key]
            outside = float(shape.difference(region).area)
            if outside > 1e-6:
                metrics[f"{label}_{kind}_outside_count"] += 1
            metrics[f"maximum_{label}_outside_m2"] = max(metrics[f"maximum_{label}_outside_m2"], outside)
            margin = 0.0 if outside > 1e-6 else float(shape.distance(region.boundary))
            clearance = f"minimum_{label}_{kind}_clearance_m"
            metrics[clearance] = min(metrics[clearance], margin)

    for arc, body in zip(arcs, bodies):
        measure(body, "body", arc, arc)
    for index, (first, second) in enumerate(zip(bodies, bodies[1:])):
        sweep = lane.swept_body(first, second, poses[index], poses[index + 1], radius)
        measure(sweep, "sweep", arcs[index], arcs[index + 1])
    if any(metrics[f"{label}_{kind}_outside_count"] for label in ("raw", "inset")
           for kind in ("body", "sweep")):
        metrics["status"] = "FAIL"
    return metrics


def known_sites() -> tuple[list[dict], dict, dict]:
    """HH_260906 - Bind historical TRAIN, every V4 site and captured C/p06 separately."""
    reference, _ = turn.read_json(HISTORICAL_TRAIN, turn.REFERENCE_SHA256)
    known = [{"source": "historical_TRAIN", "route_id": "town01_right_s0001_p00",
              "fingerprint": fingerprint(reference), "route_sha256": turn.REFERENCE_SHA256}]
    v4_static = read(V4_AUDIT)
    require(v4_static.get("status") == "STATIC_REVIEW_ONLY" and
            digest(V4 / "route_catalog.json") == v4_static.get("catalog_sha256"),
            "V4 static inventory changed")
    for item in v4_static["candidates"]:
        path = ROOT / item["path"]
        require(digest(path) == item["sha256"], f"V4 route changed: {item['route_id']}")
        route = read(path)
        known.append({"source": "V4_site", "route_id": item["route_id"],
                      "fingerprint": fingerprint(route), "route_sha256": item["sha256"]})
    c_manifest = read(C_EPISODE / "manifest.json")
    c_route = read(C_EPISODE / "route.json")
    require(c_manifest.get("provenance", {}).get("route_sha256") == digest(C_EPISODE / "route.json") and
            c_manifest.get("capture_contract", {}).get("active_map_opendrive", {}).get("sha256") == MAP_SHA256 and
            c_manifest.get("capture_contract", {}).get("actor_body_geometry", {}).get("actor_type_id") ==
            "vehicle.toyota.prius" and
            c_manifest.get("coordinate_contract", {}).get("wheelbase_m") == 2.85 and
            digest(C_EPISODE / "active_map_opendrive.xodr") == MAP_SHA256,
            "C/p06 live XODR, route or Prius body provenance changed")
    known.append({"source": "C_p06_expert_capture", "route_id": "town01_right_s0002_p06",
                  "fingerprint": fingerprint(c_route),
                  "route_sha256": digest(C_EPISODE / "route.json")})
    return known, c_manifest, v4_static


def audit(owner_root: Path, output: Path) -> dict:
    require(owner_root.resolve() == OWNER.resolve(), "only the completed V5 owner root is accepted")
    plan, owner, catalog = completed_v5_owner(owner_root)
    previous, manifest, v4_static = known_sites()
    import carla

    rows = []
    for entry in (entry for entry in catalog["routes"] if entry.get("scenario") == "right"):
        identity = entry.get("id")
        relative = Path(entry.get("path", ""))
        require(isinstance(identity, str) and identity.startswith("town01_right_s") and
                not relative.is_absolute() and ".." not in relative.parts and
                relative.parts[:3] == ("routes", "town01", "right") and
                relative.name == identity + ".json" and entry.get("status") == "ready",
                "V5 route path/identity invalid")
        path = owner_root / relative
        require(digest(path) == entry.get("sha256"), f"V5 route hash differs: {identity}")
        route = read(path)
        require(route.get("town") == "Town01" and route.get("scenario") == "right" and
                route.get("coordinate_reference") == "base_link", "V5 route identity changed")
        shape = turn.turn_geometry(route)
        turn_start = float(route["route"][shape["turn_start_index"]]["distance_m"])
        turn_end = float(route["route"][shape["turn_end_index"]]["distance_m"])
        site = fingerprint(route)
        reused = [{"source": item["source"], "route_id": item["route_id"]} for item in previous
                  if same_site(site, item["fingerprint"])]
        nearby_train = math.dist(site[1], previous[0]["fingerprint"][1]) < 30.0
        exclusions = []
        if turn_start < START_M or turn_end > END_M:
            exclusions.append("turn_not_fully_inside_fixed_30_to_120m_window")
        if turn_start < 30.0:
            exclusions.append("less_than_30m_route_lead")
        if shape["length_m"] < END_M + FUTURE_AT_9_KMH_M:
            exclusions.append("9kmh_full_6p4s_future_route_tail_short")
        if reused:
            exclusions.append("existing_physical_turn_site")
        if nearby_train:
            exclusions.append("within_30m_of_historical_TRAIN_turn")
        row = {"route_id": identity, "path": str(path.relative_to(ROOT)),
               "route_sha256": entry["sha256"], "turn_start_m": turn_start,
               "turn_end_m": turn_end, "route_length_m": shape["length_m"],
               "selected_pre_turn_context_m": max(0.0, turn_start - START_M),
               "selected_post_turn_context_m": max(0.0, END_M - turn_end),
               "route_tail_after_120m_m": shape["length_m"] - END_M,
               "physical_turn_transitions": [list(triple) for triple in site[0]],
               "physical_turn_center_xy_ros_m": list(site[1]),
               "known_site_matches": reused, "near_historical_train_30m": nearby_train,
               "exclusion_reasons": exclusions, "nominal": None,
               "capture_authorized": False, "dataset_admitted": False}
        if not exclusions:
            try:
                row["nominal"] = provisional_nominal_clearance(
                    route, manifest, (C_EPISODE / "active_map_opendrive.xodr").read_bytes(), carla)
            except Exception as error:
                row["nominal"] = {"status": "UNSUPPORTED", "reason": str(error)}
                row["exclusion_reasons"].append("nominal_geometry_unsupported")
            else:
                if row["nominal"]["status"] != "PASS":
                    row["exclusion_reasons"].append("nominal_body_or_sweep_outside_10cm_inner_lane")
                if not row["nominal"]["production_lane_gate_supported"]:
                    row["exclusion_reasons"].append("production_lane_gate_rejects_zero_arc_lane_transition")
        rows.append(row)
    groups = []
    for row in rows:
        site = (tuple(tuple(triple) for triple in row["physical_turn_transitions"]),
                tuple(row["physical_turn_center_xy_ros_m"]))
        group = next((item for item in groups if same_site(site, item["site"])), None)
        if group is None:
            group = {"site": site, "routes": []}
            groups.append(group)
        group["routes"].append(row)
    ranked_sites = []
    for group in groups:
        members = group["routes"]
        eligible = [row for row in members if not row["exclusion_reasons"]]
        eligible.sort(key=lambda row: (-row["nominal"]["minimum_inset_sweep_clearance_m"],
                                       -row["selected_pre_turn_context_m"], row["route_id"]))
        ranked_sites.append({"route_ids": [row["route_id"] for row in members],
                             "provisional_top_route_id": eligible[0]["route_id"] if eligible else None,
                             "nominal_pass_route_ids": [row["route_id"] for row in eligible],
                             "known_site_matches": members[0]["known_site_matches"],
                             "capture_authorized": False})
    ranked_sites.sort(key=lambda item: (item["provisional_top_route_id"] is None,
                                        item["provisional_top_route_id"] or item["route_ids"][0]))
    report = {"schema": "portable_e2e.town01_v5_long_turn_rank.v1",
              "status": "OFFLINE_PREREGISTRATION_CANDIDATES" if
              any(item["provisional_top_route_id"] for item in ranked_sites)
              else "NO_NOMINAL_SAFE_NOVEL_CANDIDATE",
              "owner_plan_sha256": digest(owner_root / "owner_plan.json"),
              "owner_result_sha256": digest(owner_root / "owner_result.json"),
              "catalog_sha256": owner["catalog_sha256"],
              "v4_static_audit_sha256": digest(V4_AUDIT),
              "historical_train_route_sha256": turn.REFERENCE_SHA256,
              "c_p06_route_sha256": digest(C_EPISODE / "route.json"),
              "live_town01_xodr_sha256": MAP_SHA256,
              "prius_body_manifest_sha256": digest(C_EPISODE / "manifest.json"),
              "window_m": [START_M, END_M], "future_requirement_m_at_9kmh": FUTURE_AT_9_KMH_M,
              "right_candidate_count": len(rows), "physical_site_count": len(groups),
              "ranked_sites": ranked_sites, "routes": rows,
              "capture_executed": False, "capture_authorized": False,
              "dataset_admitted": False, "training_executed": False,
              "learned_driving_tested": False}
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    args = parser.parse_args()
    result = audit(OWNER, args.output)
    print(json.dumps({key: result[key] for key in ("status", "right_candidate_count", "physical_site_count",
                                               "ranked_sites", "capture_authorized")}, indent=2))


if __name__ == "__main__":
    main()
