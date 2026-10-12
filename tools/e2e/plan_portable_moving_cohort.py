#!/usr/bin/env python3
"""HH_260906 - Bind 24 prospective moving slots to non-TEST route bytes without launching or admitting data."""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path

from scripts.e2e import plan_portable_map_coverage as coverage
from portable_e2e.workspace_paths import workspace_path

ROOT = Path(__file__).resolve().parents[2]
COHORT = (("town07", "T07", "train", 2), ("c_track_1_0_7", "CT", "train", 2),
          ("town01", "T01", "train", 2), ("town03", "V03", "dev_val", 1),
          ("town02_opt", "V02", "dev_val", 1))
BEHAVIORS = (("straight", "S"), ("left", "L"), ("right", "R"))
SOURCE_NAMES = ("scripts/e2e/plan_portable_moving_cohort.py", "scripts/e2e/plan_portable_map_coverage.py",
                "scripts/e2e/inventory_carla_training_maps.py", "scripts/e2e/collect_carla_vad_expert.py",
                "scripts/e2e/prepare_carla_common10_dataset.py",
                "scripts/e2e/prepare_carla_route.py",
                "portable_e2e/config/common_10hz_v1.contract.json",
                "docs/portable-e2e-data-expansion-2026-09-15.md")
MAX_BYTES = 16 * 1024 * 1024
SPEED_CEILING_MPS = 15. / 3.6
FUTURE_SECONDS, LEAD_IN_M, MINIMUM_ANCHOR_SECONDS = 6.4, 30., 30.
safe, require, decode = coverage.checked_path, coverage.require, coverage.strict_json


def numeric(value, name):
    require(type(value) in (int, float) and math.isfinite(value), "finite non-boolean " + name + " required")
    return float(value)


def read(path, ledger, expected=None):
    path = safe(path)
    require(path.is_file() and path.stat().st_size <= MAX_BYTES, "bounded regular input required: " + str(path))
    payload = path.read_bytes()
    require(len(payload) <= MAX_BYTES, "input grew past size bound")
    digest = hashlib.sha256(payload).hexdigest()
    require(expected is None or digest == expected, "input SHA mismatch: " + str(path))
    require(str(path) not in ledger or ledger[str(path)] == digest, "input changed during planning")
    ledger[str(path)] = digest
    return payload


def pin(value):
    require(type(value) is str and len(value) == 64 and set(value) <= set("0123456789abcdef"), "mandatory SHA missing or invalid")
    return value


def horizon_geometry(points, length, scenario, reference_speed_kph=15.):
    """HH_260906 - A conservative distance budget is not recorded speed, temporal coverage or a new admission gate."""
    speed = reference_speed_kph / 3.6
    tail = speed * FUTURE_SECONDS
    end = length - tail
    maneuver = [p["distance_m"] for p in points if p["road_option"] == scenario.upper()] if scenario != "straight" else []
    start = LEAD_IN_M
    reasons = []
    if end - start < speed * MINIMUM_ANCHOR_SECONDS:
        reasons.append("SHORT_ANCHOR_SPAN_AT_DECLARED_SPEED_REFERENCE")
    if scenario != "straight":
        if not maneuver:
            reasons.append("DECLARED_TURN_OPTION_NOT_FOUND")
        else:
            if min(maneuver) < start:
                reasons.append("TURN_BEGINS_BEFORE_PROPOSED_LEAD_IN")
            if max(maneuver) > end:
                reasons.append("INSUFFICIENT_POST_TURN_ARC_AT_DECLARED_SPEED_REFERENCE")
    return dict(status="CONSERVATIVE_ARC_BUDGET_AVAILABLE" if not reasons else "ROUTE_EXTENSION_OR_SPEED_PLAN_NEEDED",
        distance_reference="RECOMPUTED_PLANAR_ROUTE_ARC",
        reference_speed_ceiling_kph=reference_speed_kph, reference_speed_ceiling_mps=speed, future_seconds=FUTURE_SECONDS,
        reserved_future_arc_m=tail, proposed_first_anchor_arc_m=start,
        proposed_last_anchor_arc_m=end if end >= start else None,
        available_anchor_arc_m=max(0., end-start), reference_anchor_span_seconds=max(0., end-start)/speed,
        minimum_reference_anchor_span_seconds=MINIMUM_ANCHOR_SECONDS,
        minimum_route_length_m=start+speed*(MINIMUM_ANCHOR_SECONDS+FUTURE_SECONDS),
        turn_first_arc_m=min(maneuver) if maneuver else None, turn_last_arc_m=max(maneuver) if maneuver else None,
        reasons=reasons, measured_timing_available=False,
        notice="Prospective geometry budget only. Slower measured motion can have more temporal context; no constant turn speed, completed future mask or native QA is inferred.")


def load_candidates(catalog_root, map_id, ledger):
    # HH_260906 - Reject protected maps before constructing any catalog or route-payload path.
    require(map_id not in coverage.PROTECTED_MAPS, "protected map access forbidden")
    require(map_id in {row[0] for row in COHORT}, "map outside fixed moving cohort")
    canonical = coverage.inventory.MAP_CONTRACT[map_id][0]
    candidates, seen, unavailable = [], set(), []
    for kind in ("straight", "turn"):
        directory = safe(catalog_root / map_id / "catalog" / kind)
        catalog = directory / "route_catalog.json"
        if not catalog.is_file():
            unavailable.append(dict(catalog=str(catalog), reason="CATALOG_NOT_AVAILABLE"))
            continue
        document = decode(read(catalog, ledger))
        require(document.get("status") == "complete" and document.get("map_id") == map_id
                and document.get("canonical_name") == canonical, "catalog identity/status differs")
        require(type(document.get("routes")) is list, "catalog routes must be a list")
        for entry in document["routes"]:
            identity, scenario = entry.get("id"), entry.get("scenario")
            require(type(identity) is str and identity and identity not in seen, "duplicate or invalid catalog route ID")
            seen.add(identity)
            require(entry.get("status") == "ready" and scenario in (("straight",) if kind == "straight" else ("left", "right")), "catalog route status/direction differs")
            relative = Path(entry["path"])
            require(not relative.is_absolute() and ".." not in relative.parts
                    and relative.parts[:3] == ("routes", map_id, scenario), "route reference escapes its map/direction")
            route_path = safe(directory / relative)
            expected = pin(entry.get("sha256"))
            if not route_path.is_file():
                unavailable.append(dict(route_id=identity, route_path=str(route_path), reason="ROUTE_PAYLOAD_NOT_AVAILABLE"))
                continue
            payload = read(route_path, ledger, expected)
            route = decode(payload)
            require(route.get("town") == canonical and route.get("scenario") == scenario
                    and route.get("coordinate_reference") == "base_link", "route payload identity/frame differs")
            length = numeric(route["route_length_m"], "route length")
            require(length > 0. and length == numeric(entry["route_length_m"], "catalog route length"), "route/catalog length mismatch")
            points = route["route"]
            require(type(points) is list and len(points) >= 2, "route needs a polyline")
            corridors = set()
            xy, planar_points, last, geometric_length, xyz_length, yaw_change, previous_yaw = [], [], -1., 0., 0., 0., None
            for point in points:
                x,y,z,arc = (numeric(point[name], name) for name in ("x", "y", "z", "distance_m"))
                require(last <= arc <= length + .001, "route arc order/length differs")
                yaw = numeric(point["yaw"], "yaw")
                if previous_yaw is not None:yaw_change += math.atan2(math.sin(yaw-previous_yaw),math.cos(yaw-previous_yaw))
                previous_yaw=yaw
                require(type(point.get("road_option")) is str, "road option required")
                for name in ("road_id", "section_id", "lane_id"):
                    require(type(point.get(name)) is int, "integer road/section/lane IDs required")
                corridors.add((point["road_id"], point["section_id"]))
                if xy:
                    geometric_length += math.hypot(x-xy[-1][0], y-xy[-1][1])
                    xyz_length += math.dist((x,y,z),xy[-1])
                planar_points.append(dict(distance_m=geometric_length,road_option=point["road_option"]))
                xy.append([x,y,z]); last=arc
            # HH_260906 - Catalog distance is XYZ arc; the speed/context budget uses independently accumulated XY arc.
            require(abs(numeric(points[0]["distance_m"], "first arc")) <= .001
                    and abs(last-length) <= .001 and abs(xyz_length-length) <= .001 and math.isfinite(yaw_change),
                    "serialized arc is not consistent with full XYZ polyline")
            geometry = json.dumps(xy, separators=(",", ":"), allow_nan=False).encode()
            reverse = json.dumps(list(reversed(xy)), separators=(",", ":"), allow_nan=False).encode()
            candidates.append(dict(map_id=map_id, canonical_name=canonical, route_id=identity,
                scenario=scenario, route_path=str(route_path), route_sha256=ledger[str(route_path)],
                catalog_path=str(catalog), catalog_sha256=ledger[str(catalog)],
                route_length_m=length, catalog_distance_metric="XYZ_ARC_LENGTH", recomputed_xyz_length_m=xyz_length,
                recomputed_planar_length_m=geometric_length, waypoint_count=len(points),
                signed_serialized_yaw_change_rad=yaw_change,
                turn_yaw_sign_agrees=(yaw_change>0. if scenario=="left" else yaw_change<0. if scenario=="right" else None),
                geometry_sha256=hashlib.sha256(min(geometry, reverse)).hexdigest(),
                road_sections=[list(pair) for pair in sorted(corridors)],
                source_weather=route.get("weather"), catalog_geometry_is_driving_evidence=False,
                horizon=horizon_geometry(planar_points, geometric_length, scenario),
                full_30kph_horizon=horizon_geometry(planar_points, geometric_length, scenario,30.)))
    return candidates, unavailable


def fixed_slots():
    result = []
    for map_id,prefix,split,repeats in COHORT:
        for scenario,letter in BEHAVIORS:
            for repeat in range(1,repeats+1):
                weather = ("ClearNoon" if repeat == 1 else "CloudyNoon") if split == "train" else (
                    "ClearNoon" if (scenario != "left") == (map_id == "town03") else "CloudyNoon")
                result.append(dict(slot_id=f"{prefix}-{letter}{repeat:02d}", map_id=map_id, split=split,
                    scenario=scenario, weather=weather, traffic_seed=(100+repeat if split == "train" else 201 if map_id == "town03" else 202)))
    return result


def select_slots(candidates, map_notes):
    selected, used_hashes, used_geometry = [], set(), set()
    for slot in fixed_slots():
        pool = [r for r in candidates if r["map_id"] == slot["map_id"] and r["scenario"] == slot["scenario"]
                and r["route_sha256"] not in used_hashes and (r["map_id"],r["geometry_sha256"]) not in used_geometry]
        def conflicts(candidate):
            roads = {tuple(x) for x in candidate["road_sections"]}
            return [dict(other_slot_id=row["slot_id"], other_split=row["split"], shared_road_sections=sorted(
                roads & {tuple(x) for x in row["route"]["road_sections"]})) for row in selected
                if row.get("route") and row["map_id"] == candidate["map_id"]
                and roads & {tuple(x) for x in row["route"]["road_sections"]}]
        pool.sort(key=lambda r:(bool(r["horizon"]["reasons"]), len(conflicts(r)),
                               sum(len(c["shared_road_sections"]) for c in conflicts(r)), -r["route_length_m"], r["route_id"]))
        route = pool[0] if pool else None
        overlaps = conflicts(route) if route else []
        reasons = list(map_notes[slot["map_id"]]["blockers"])
        if route is None:
            reasons.append("NO_DISTINCT_AVAILABLE_ROUTE_FOR_DIRECTION")
        else:
            used_hashes.add(route["route_sha256"]);used_geometry.add((route["map_id"],route["geometry_sha256"]))
            reasons.extend(route["horizon"]["reasons"])
            if route["turn_yaw_sign_agrees"] is False:reasons.append("DECLARED_TURN_YAW_DIRECTION_REVIEW_NEEDED")
            if overlaps:reasons.append("SHARED_ROAD_SECTION_REVIEW_NEEDED")
        selected.append(dict(**slot, route=route, status="ROUTE_BOUND_NOT_EXECUTABLE" if route else "ROUTE_UNAVAILABLE",
            route_group_independence="NOT_CERTIFIED", corridor_conflicts=overlaps,
            split_notes=map_notes[slot["map_id"]], technical_conditions_unmet=reasons+[
                "MOVING_CAPTURE_COMPLETION_AND_FULL_CONTEXT_IMPLEMENTATION_NEEDED",
                "FRESH_SOURCE_SESSION_AND_RAW_QA_NOT_YET_OBSERVED"],
            execution_allowed=False, training_data_approved=False))
    # HH_260906 - Surface each pair in both slots; selection order must not hide earlier overlapping routes.
    for row in selected:
        for conflict in list(row["corridor_conflicts"]):
            earlier=next(other for other in selected if other["slot_id"] == conflict["other_slot_id"])
            earlier["corridor_conflicts"].append(dict(other_slot_id=row["slot_id"], other_split=row["split"],
                shared_road_sections=conflict["shared_road_sections"]))
            if "SHARED_ROAD_SECTION_REVIEW_NEEDED" not in earlier["technical_conditions_unmet"]:
                earlier["technical_conditions_unmet"].append("SHARED_ROAD_SECTION_REVIEW_NEEDED")
    return selected


def run(catalog_root, output, manifest_path=coverage.inventory.DEFAULT_MANIFEST):
    catalog_root,output,manifest_path=safe(catalog_root),safe(output),safe(manifest_path)
    require(not output.exists() and not output.is_relative_to(catalog_root) and output != manifest_path, "fresh output disjoint from catalogs required")
    ledger={}
    # HH_260906 - Separate logical source identities from canonical local file locations.
    for name in SOURCE_NAMES:read(workspace_path(ROOT, name),ledger)
    contract=decode(read(workspace_path(ROOT,"portable_e2e/config/common_10hz_v1.contract.json"),ledger))
    require(contract["capture"]["minimum_episode_duration_s"]==MINIMUM_ANCHOR_SECONDS
            and contract["trajectory"]["horizon_s"]==FUTURE_SECONDS
            and contract["trajectory"]["point_count"]==64, "Common10 duration/future contract differs")
    read(manifest_path,ledger)
    manifest,_=coverage.inventory.load_manifest(manifest_path)
    assets=coverage.inventory.collect_static_inventory(manifest)
    maps={r["id"]:r for r in manifest["maps"]};observed={r["id"]:r for r in assets["maps"]}
    runtime=next(r for r in assets["profiles"] if r["id"]=="packaged_0915")
    candidates,unavailable,notes=[],[],{}
    for map_id,_,split,_ in COHORT:
        require(map_id not in coverage.PROTECTED_MAPS and maps[map_id]["split"]!="test", "cohort contains protected map")
        blockers=[]
        if observed[map_id]["assets_complete"] is not True:blockers.append("REQUIRED_MAP_ASSETS_NOT_CONFIRMED")
        if maps[map_id]["status"]!="ready" or maps[map_id]["server_profile"]!="packaged_0915":blockers.append("PACKAGED_MAP_NOT_READY")
        if not all(runtime.get(k) is True for k in ("executable_exists","executable_is_runnable","python_api_exists","python_api_sha256_matches")):
            blockers.append("PACKAGED_RUNTIME_FILES_NOT_CONFIRMED")
        notes[map_id]=dict(legacy_manifest_split=maps[map_id]["split"],proposed_split=split,
            legacy_split_conflict=maps[map_id]["split"]!=("validation" if split=="dev_val" else "train"),
            new_dev_map_proposal=map_id=="town02_opt",blockers=blockers)
        found,missing=load_candidates(catalog_root,map_id,ledger)
        candidates.extend(found);unavailable.extend(missing)
    slots=select_slots(candidates,notes)
    pilots=[]
    for identity in ("T07-S01","V03-S01"):
        slot=next(row for row in slots if row["slot_id"]==identity)
        route=slot["route"]
        pilots.append(dict(slot_id=identity,route_id=route["route_id"] if route else None,
            route_sha256=route["route_sha256"] if route else None,proposal_only=True,
            first_anchor_rule="First saved camera anchor whose measured planar progress is at least30m",
            last_anchor_rule="Last saved camera anchor at or below165m; no quality-based trimming",
            selected_window_arc_m=[30.,165.],nominal_window_seconds=(165.-30.)/SPEED_CEILING_MPS,
            required_observed_selected_span_seconds=MINIMUM_ANCHOR_SECONDS,
            completion_rule="Continue actual recording through last selected anchor timestamp plus6.4s before cleanup braking; abort conditions remain authoritative",
            nominal_future_end_arc_m=165.+SPEED_CEILING_MPS*FUTURE_SECONDS,
            nominal_remaining_route_margin_m=(route["recomputed_planar_length_m"]-165.-SPEED_CEILING_MPS*FUTURE_SECONDS) if route else None,
            execution_allowed=False,training_data_approved=False))
    report=dict(schema="portable_e2e.moving_cohort_plan.v1",status="PROVISIONAL_NON_EXECUTING_COHORT",
        generated_at_utc=datetime.now(timezone.utc).isoformat(),slot_count=len(slots),split_counts=dict(Counter(r["split"] for r in slots)),
        route_bound_count=sum(r["route"] is not None for r in slots),candidate_count=len(candidates),
        geometry_budget_available_slots=sum(r["route"] is not None and not r["route"]["horizon"]["reasons"] for r in slots),
        full_30kph_geometry_budget_available_slots=sum(r["route"] is not None and not r["route"]["full_30kph_horizon"]["reasons"] for r in slots),
        proposed_speed_policy=dict(pilot_speed_ceiling_kph=15., eventual_speed_ceiling_kph=30.,
            reduced_speed_pilot_is_not_30kph_validation=True, common10_selected_anchor_span_s=MINIMUM_ANCHOR_SECONDS,
            future_context_s=FUTURE_SECONDS, lead_in_arc_m=LEAD_IN_M, gates_changed=False),
        first_pipeline_pilot_proposals=pilots,
        slots=slots,candidates=candidates,unavailable=unavailable,local_static_inventory=assets,
        selection_rule="Fixed cohort order; unused exact/reversed geometry; sufficient conservative arc first, then fewer shared road sections, then longer route, then route ID.",
        protected_maps=sorted(coverage.PROTECTED_MAPS),test_catalogs_or_payloads_opened=False,world_rpc_performed=False,
        gpu_used=False,execution_commands=[],training_data_approved=False,existing_dataset_changed=False,
        source_and_input_sha256_before=dict(ledger),source_and_input_sha256_after=dict(ledger),
        limitations=["Route bytes and conservative geometry are not native motion, admission or learned closed-loop evidence.",
            "Shared road sections are conservative corridor warnings, not a metric proof of exact spatial overlap; distinct SHA is not statistical independence.",
            "Existing TRAIN/VAL source trajectories were not opened or spatially compared; legacy cohort overlap remains to be checked before capture declaration.",
            "The new moving capture mode is not implemented here; no omitted launch/stop failures or favorable retiming is authorized.",
            "The unchanged Common10 contract requires at least 30s between first and last selected anchors, not merely 30s total raw duration.",
            "All raw phases and predetermined scored anchors need their declared checks; unchanged nine-metric QA covers every selected64-point future, not only anchor speed. Task eligibility is separate from contiguous-prefix availability masks; denied historical data remains denied."])
    for name,digest in tuple(ledger.items()):read(name,{},digest)
    output.parent.mkdir(parents=True,exist_ok=True)
    with output.open("x") as stream:json.dump(report,stream,indent=2,allow_nan=False);stream.write("\n")
    return report


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument("--catalog-maps-root",required=True,type=Path)
    parser.add_argument("--output-json",required=True,type=Path)
    args=parser.parse_args(argv)
    result=run(args.catalog_maps_root,args.output_json)
    print(json.dumps({key:result[key] for key in ("status","slot_count","split_counts","route_bound_count","geometry_budget_available_slots")}))


if __name__=="__main__":main()
