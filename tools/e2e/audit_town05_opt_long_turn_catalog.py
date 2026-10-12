#!/usr/bin/env python3
"""HH_260906 - Fail closed on unfinished Town05_Opt catalogs and provisional map geometry."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any

from scripts.e2e import audit_future_actor_lane_containment as lane
from scripts.e2e import audit_new_long_turn_candidates as catalog_geometry
from scripts.e2e import rank_town01_wide_turns as nominal_pose


ROOT = Path(__file__).resolve().parents[2]
OWNER = ROOT / "storage/experiments/validation/2026-09-27/drive_long_route_town05_opt_dev_catalog_v1"
PACKAGED_XODR = Path(
    "/home/a/carla-autoware-universe/CARLA_0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05_Opt.xodr"
)
PACKAGED_XODR_SHA256 = "7edc53b0be840e177598da7a4ed9571146d7b2595ce280d154c90f92e8a48851"
BODY_EPISODE = nominal_pose.ARCHIVED_EPISODE
PREVIOUS_RIGHT = ROOT / (
    "storage/experiments/validation/2026-09-01/autoware_vad_town_matrix_30kph_v16_owned_window_final/"
    "maps/town05_opt/catalog/turn/routes/town05_opt/right/town05_opt_right_s0000_p00.json"
)
PREVIOUS_RIGHT_SHA256 = "2f7a25e8ace753bfdd412ae7ef65710a001d1f4b1ecd5765f3ee7b021ced29e3"
OUTPUT = ROOT / "docs/assets/validation/2026-09-27/drive_town05_opt_long_turn_rank/rank.json"
START_M = 30.0
END_M = 120.0
FUTURE_S = 6.4
SPEED_KMH = 9.0
SAMPLE_STEP_M = 0.10


class IncompleteOwner(ValueError):
    """HH_260906 - Refuse owner bytes until complete receipts and source checks exist."""


def require(condition: bool, reason: str) -> None:
    if not condition:
        raise IncompleteOwner(reason)


def sha(path: Path) -> str:
    require(path.is_file() and not path.is_symlink(), f"missing regular file: {path}")
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path: Path) -> dict:
    try:
        payload, _ = catalog_geometry.read_json(path)
    except catalog_geometry.AuditError as error:
        raise IncompleteOwner(str(error)) from error
    return payload


def completed_owner(owner_root: Path) -> tuple[dict, dict, dict]:
    """HH_260906 - Check lifecycle and source archives before opening a candidate route."""
    plan = read(owner_root / "owner_plan.json")
    require(plan.get("schema") == "portable_e2e.owned_turn_route_catalog.v1" and
            plan.get("map_id") == "town05_opt" and plan.get("map") == "Town05_Opt" and
            plan.get("split_intent") == "PROVISIONAL_DEV" and plan.get("quality") == "Epic" and
            plan.get("capture_executed") is False and plan.get("training_executed") is False and
            plan.get("dataset_admission_granted") is False and plan.get("allow_map_load") is False,
            "owner plan is not the provisional catalog-only Town05_Opt search")
    owner = read(owner_root / "owner_result.json")
    # HH_260906 - Report a blocked search directly instead of implying its archived source failed.
    require(owner.get("exit_code") == 0 and owner.get("catalog_status") == "complete",
            f"owner search blocked: exit={owner.get('exit_code')} catalog={owner.get('catalog_status')}")
    require(owner.get("schema") == "portable_e2e.owned_turn_route_catalog_result.v1" and
            owner.get("source_bytes_unchanged_and_archived") is True and
            owner.get("lifecycle_receipts_present") is True and
            owner.get("capture_executed") is False and owner.get("training_executed") is False and
            owner.get("dataset_admission_granted") is False and
            owner.get("learned_model_control") is False,
            "owner has not completed as a source-preserved catalog-only search")
    sources = plan.get("source_sha256")
    checks = owner.get("source_checks")
    require(isinstance(sources, dict) and sources and isinstance(checks, dict) and
            set(checks) == set(sources) and all(value is True for value in checks.values()),
            "owner source-check ledger incomplete")
    for relative, expected in sources.items():
        name = Path(relative)
        require(not name.is_absolute() and ".." not in name.parts and
                isinstance(expected, str) and len(expected) == 64 and
                sha(owner_root / "provenance" / name) == expected,
                f"archived source mismatch: {relative}")
    for stage in ("ready", "after_catalog", "stopped"):
        receipt = read(owner_root / "lifecycle" / f"{stage}.json")
        require(receipt.get("stage") == stage and receipt.get("status") == "PASS" and
                receipt.get("expected_map") == "Town05_Opt" and
                receipt.get("mode") == ("stopped" if stage == "stopped" else "running"),
                f"{stage} lifecycle receipt is not complete")
        if stage == "stopped":
            require(receipt.get("port_released") is True, "owned CARLA port not released")
    require(sha(owner_root / "route_catalog.json") == owner.get("catalog_sha256"),
            "catalog digest differs from sealed owner result")
    catalog = read(owner_root / "route_catalog.json")
    require(catalog.get("status") == "complete" and catalog.get("map_id") == "town05_opt" and
            catalog.get("canonical_name") == "Town05_Opt" and
            catalog.get("server", {}).get("active_map_name") == "Carla/Maps/Town05_Opt" and
            catalog.get("server", {}).get("map_load_performed") is False and
            catalog.get("server", {}).get("active_server_profile") == "packaged_0915",
            "catalog map/status/provenance differs")
    right = [entry for entry in catalog.get("routes", []) if entry.get("scenario") == "right"]
    require(len(right) == owner.get("right_candidate_count") and bool(right),
            "right-turn count differs from sealed owner result")
    return plan, owner, catalog


def turn_fingerprint(route: dict) -> tuple[tuple[tuple[int, int, int], ...], tuple[float, float]]:
    """HH_260906 - Identify a physical maneuver even when the older route is short."""
    preflight = route.get("physical_turn_preflight", {})
    require(preflight.get("status") == "PASS" and preflight.get("scenario") == "right" and
            preflight.get("directional_block_count") == 1, "single right-turn preflight required")
    block = preflight.get("selected_block", {})
    points = route.get("route")
    start, end = block.get("start_index"), block.get("end_index")
    require(isinstance(points, list) and type(start) is int and type(end) is int and
            0 <= start <= end < len(points), "turn block outside serialized route")
    transitions = []
    for point in points[start:end + 1]:
        triple = tuple(point.get(key) for key in ("road_id", "section_id", "lane_id"))
        require(all(type(value) is int for value in triple), "route lacks integer road/lane identity")
        if not transitions or transitions[-1] != triple:
            transitions.append(triple)
    midpoint = points[(start + end) // 2]
    center = (float(midpoint["x"]), float(midpoint["y"]))
    require(all(math.isfinite(value) for value in center), "nonfinite turn center")
    return tuple(transitions), center


def same_site(a: tuple[tuple[tuple[int, int, int], ...], tuple[float, float]],
              b: tuple[tuple[tuple[int, int, int], ...], tuple[float, float]]) -> bool:
    """HH_260906 - Do not count two starts leading through one physical turn twice."""
    return a[0] == b[0] and math.dist(a[1], b[1]) <= 5.0


def nominal_clearance(route: dict, body_manifest: dict, carla_map: Any, carla: Any,
                      xodr: bytes) -> dict:
    """HH_260906 - Screen only the nominal rear-axle route, not driven body trajectories."""
    points = nominal_pose.distinct_points(route["route"])
    box, radius = lane._body_box(body_manifest, carla)
    relevant = {node for i, (a, b) in enumerate(zip(points, points[1:]))
                if float(a["distance_m"]) <= END_M + radius and
                float(b["distance_m"]) >= START_M - radius
                for node in (i, i + 1)}
    requested = {lane._triple(points[index]) for index in relevant}
    corridors = lane.lane_corridors(carla_map, carla, points, xodr, requested, relevant)
    count = math.ceil((END_M - START_M) / SAMPLE_STEP_M)
    arcs = [START_M + (END_M - START_M) * i / count for i in range(count + 1)]
    poses, bodies, cursor = [], [], 0
    for arc in arcs:
        pose, cursor = nominal_pose.pose_at_arc(points, arc, cursor)
        poses.append(pose)
        bodies.append(lane.body_footprint(box, pose, carla))
    regions = {}
    metrics = {"status": "PASS", "scope": "packaged-XODR provisional nominal route only",
               "sample_step_m": SAMPLE_STEP_M, "actor_rear_axle_offset_m": 1.425,
               "body_count": len(bodies), "sweep_count": len(bodies) - 1,
               "raw_body_outside_count": 0, "inset_body_outside_count": 0,
               "raw_sweep_outside_count": 0, "inset_sweep_outside_count": 0,
               "maximum_raw_outside_m2": 0.0, "maximum_inset_outside_m2": 0.0,
               "minimum_raw_body_clearance_m": math.inf, "minimum_inset_body_clearance_m": math.inf,
               "minimum_raw_sweep_clearance_m": math.inf, "minimum_inset_sweep_clearance_m": math.inf,
               "physical_continuous_time_proven": False, "capture_authorized": False}

    def measure(shape: Any, kind: str, lo: float, hi: float) -> None:
        identities = lane._route_lanes_for_arc_interval(points, lo - radius, hi + radius)
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


def audit(owner_root: Path, output: Path) -> dict:
    """HH_260906 - Publish provisional geometry only after an immutable completed owner."""
    require(owner_root.resolve() == OWNER.resolve(), "only the preregistered Town05_Opt owner is accepted")
    plan, owner, catalog = completed_owner(owner_root)
    require(sha(PACKAGED_XODR) == PACKAGED_XODR_SHA256,
            "packaged Town05_Opt OpenDRIVE digest changed")
    require(sha(PREVIOUS_RIGHT) == PREVIOUS_RIGHT_SHA256, "earlier Town05 right route changed")
    previous = read(PREVIOUS_RIGHT)
    previous_fingerprint = turn_fingerprint(previous)
    body_manifest_path = BODY_EPISODE / "manifest.json"
    body_manifest = read(body_manifest_path)
    body = body_manifest.get("capture_contract", {}).get("actor_body_geometry", {})
    require(body.get("status") == "RECORDED" and body.get("actor_type_id") == "vehicle.toyota.prius" and
            body_manifest.get("coordinate_contract", {}).get("wheelbase_m") == 2.85,
            "archived Prius body provenance unavailable")
    import carla

    xodr = PACKAGED_XODR.read_bytes()
    carla_map = carla.Map("packaged-Town05_Opt-provisional", xodr.decode("utf-8"))
    rows = []
    right_entries = [entry for entry in catalog["routes"] if entry.get("scenario") == "right"]
    for entry in right_entries:
        identity = entry.get("id")
        relative = Path(entry.get("path", ""))
        require(isinstance(identity, str) and identity.startswith("town05_opt_right_s") and
                not relative.is_absolute() and ".." not in relative.parts and
                relative.parts[:3] == ("routes", "town05_opt", "right") and
                relative.name == identity + ".json" and entry.get("status") == "ready",
                "catalog route identity/path is invalid")
        route_path = owner_root / relative
        require(sha(route_path) == entry.get("sha256"), f"route hash mismatch: {identity}")
        route = read(route_path)
        require(route.get("town") == "Town05_Opt" and route.get("scenario") == "right" and
                route.get("coordinate_reference") == "base_link", "route town/scenario/reference changed")
        shape = catalog_geometry.turn_geometry(route)
        fingerprint = turn_fingerprint(route)
        block = route["physical_turn_preflight"]["selected_block"]
        start = float(route["route"][block["start_index"]]["distance_m"])
        end = float(route["route"][block["end_index"]]["distance_m"])
        exclusions = []
        if start < START_M or end > END_M:
            exclusions.append("right_turn_not_fully_inside_30_to_120_m_window")
        if start < 30.0:
            exclusions.append("less_than_30m_total_route_lead")
        if shape["length_m"] - END_M < SPEED_KMH / 3.6 * FUTURE_S:
            exclusions.append("9kmh_6p4s_future_route_tail_short")
        if same_site(fingerprint, previous_fingerprint):
            exclusions.append("same_physical_turn_as_previous_town05_right_p00")
        row = {"route_id": identity, "path": str(route_path.relative_to(ROOT)),
               "sha256": entry["sha256"],
               "turn_transitions": [list(triple) for triple in fingerprint[0]],
               "turn_center_xy_ros_m": list(fingerprint[1]),
               "turn_start_m": start, "turn_end_m": end,
               "selected_pre_turn_context_m": max(0.0, start - START_M),
               "selected_post_turn_context_m": max(0.0, END_M - end),
               "future_route_tail_after_window_m": shape["length_m"] - END_M,
               "same_site_as_previous_right": same_site(fingerprint, previous_fingerprint),
               "exclusion_reasons": exclusions, "nominal": None,
               "capture_authorized": False, "independent_dev_approved": False}
        if not exclusions:
            try:
                row["nominal"] = nominal_clearance(route, body_manifest, carla_map, carla, xodr)
            except Exception as error:
                row["nominal"] = {"status": "UNSUPPORTED", "reason": str(error)}
                row["exclusion_reasons"].append("nominal_geometry_unsupported")
            else:
                if row["nominal"]["status"] != "PASS":
                    row["exclusion_reasons"].append("nominal_body_or_sweep_outside_10cm_inner_lane")
        rows.append(row)
    groups = []
    for row in rows:
        fingerprint = (tuple(tuple(triple) for triple in row["turn_transitions"]),
                       tuple(row["turn_center_xy_ros_m"]))
        group = next((item for item in groups if same_site(fingerprint, item["fingerprint"])), None)
        if group is None:
            group = {"fingerprint": fingerprint, "route_ids": []}
            groups.append(group)
        group["route_ids"].append(row["route_id"])
    sites = []
    for group in groups:
        candidates = [row for row in rows if row["route_id"] in group["route_ids"]]
        candidates.sort(key=lambda row: (bool(row["exclusion_reasons"]),
                                         -row["selected_pre_turn_context_m"], row["route_id"]))
        sites.append({"route_ids": group["route_ids"], "preferred_route_id": candidates[0]["route_id"],
                      "provisional_nominal_pass_ids": [row["route_id"] for row in candidates
                                                      if not row["exclusion_reasons"]],
                      "capture_authorized": False, "independent_dev_approved": False})
    report = {"schema": "portable_e2e.town05_opt_long_turn_rank.v1",
              "status": "PROVISIONAL_GEOMETRY_ONLY", "catalog_complete": True,
              "owner_plan_sha256": sha(owner_root / "owner_plan.json"),
              "owner_result_sha256": sha(owner_root / "owner_result.json"),
              "catalog_sha256": owner["catalog_sha256"],
              "packaged_xodr_sha256": PACKAGED_XODR_SHA256,
              "live_xodr_match_verified": False, "split_exposure_resolved": False,
              "previous_town05_right_sha256": PREVIOUS_RIGHT_SHA256,
              "prius_body_manifest_sha256": sha(body_manifest_path),
              "selected_route_window_m": [START_M, END_M],
              "future_requirement_s": FUTURE_S, "right_candidate_count": len(rows),
              "physical_site_count": len(sites), "sites": sites, "routes": rows,
              "capture_executed": False, "capture_authorized": False,
              "independent_dev_approved": False, "dataset_admitted": False,
              "training_executed": False, "learned_driving_tested": False}
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    args = parser.parse_args()
    try:
        result = audit(OWNER, args.output)
    except IncompleteOwner as error:
        # HH_260906 - Keep a failed catalog as a concise stop result, without publishing a rank file.
        print(json.dumps({"status": "STOP_OWNER_BLOCKED", "reason": str(error),
                          "rank_created": False, "capture_authorized": False,
                          "independent_dev_approved": False}, indent=2))
        return 2
    print(json.dumps({key: result[key] for key in ("status", "right_candidate_count", "physical_site_count",
                                               "capture_authorized", "independent_dev_approved")}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
