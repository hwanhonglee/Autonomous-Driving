"""HH_261002 - Census every original TRAIN turn using frozen nominal geometry, never measured driving or admission."""

from __future__ import annotations

import argparse
from collections import Counter
import gzip
import hashlib
import json
from pathlib import Path
import sys

from scripts.e2e import audit_town01_train_curve_nominal_20261002 as nominal

ROOT = nominal.ROOT
SCHEMA = "portable_e2e.town01_train_turn_nominal_census.20261002.v2"
BASE = ROOT/"storage/experiments/training/2026-09-27/drive_long_route_town01_train_catalog_v4"
CATALOG = BASE/"route_catalog.json"
PLAN = BASE/"owner_plan.json"
OWNER = BASE/"owner_result.json"
OLD_FLAGS = ROOT/"docs/assets/validation/2026-09-27/drive_long_route_prep/v4_candidate_audit.json"
OUTPUT = ROOT/"storage/experiments/diagnostics/2026-10-02/town01_train_turn_nominal_census_v2"
CATALOG_SHA = "0c0cade80ec797de39320157e35c90661ac172caef5fa7a21368036a814c5897"
PLAN_SHA = "b26406801fa21bd44f9e992dc7ce559da131f85a30e8c8061ae746a55c447d9f"
OWNER_SHA = "64468bdcac0e4002b773c73cef2722b5b6a8b428c2564ed769fe21efaea9b393"
FLAGS_SHA = "c42e02bed2560869547862240af14c4ccf789a1eaefd3a2722c3436ac4f7f00f"
FROZEN_NOMINAL_SHA = "821cad90fcb515533f97434ed46321f3f26eb5921a4872af000c72a62179cb20"
CASE_IDS = tuple(f"town01_{scenario}_s{seed:04d}_p{pair:02d}" for scenario in ("left","right") for seed in (2,3) for pair in range(8))
require = nominal.require


def source_pins():
    pins = {str(p.relative_to(ROOT)):nominal.geometry.sha(nominal.canonical(p)) for p in nominal.archive.source_closure(Path(__file__).resolve())}
    require(pins.get("tools/e2e/audit_town01_train_curve_nominal_20261002.py") == FROZEN_NOMINAL_SHA, "frozen nominal provider changed")
    require(all(pins.get(k) == v for k,v in nominal.FROZEN.items()), "frozen body geometry changed")
    return dict(sorted(pins.items()))


def expected_inputs():
    # HH_261002 - Authenticate the complete old catalogue first; no outcome-dependent candidate subset is accepted.
    pins = {}
    catalog = json.loads(nominal.payload(CATALOG,CATALOG_SHA,pins))
    plan = json.loads(nominal.payload(PLAN,PLAN_SHA,pins))
    owner = json.loads(nominal.payload(OWNER,OWNER_SHA,pins))
    require(catalog["status"] == "complete" and catalog["canonical_name"] == "Town01" and plan["split_intent"] == "TRAIN", "original catalogue is not complete Town01 TRAIN")
    require(owner["catalog_sha256"] == CATALOG_SHA and owner["exit_code"] == 0 and owner["source_bytes_unchanged_and_archived"] is True, "catalogue owner proof differs")
    require(plan["capture_executed"] is False and plan["dataset_admission_granted"] is False and owner["dataset_admission_granted"] is False, "catalogue must not be interpreted as captured/admitted data")
    entries = catalog["routes"]
    require(tuple(e["id"] for e in entries) == CASE_IDS and len({e["path"] for e in entries}) == len(CASE_IDS), "all32 exact original cases/order required")
    require(catalog["generation"]["scenarios"] == ["left","right"] and catalog["generation"]["seeds"] == [2,3] and catalog["generation"]["pairs_per_seed"] == 8, "old enumeration changed")
    for entry in entries:
        require(entry["status"] == "ready" and entry["scenario"] in ("left","right"), "catalogue member is not an original turn")
        path = nominal.canonical(BASE/entry["path"])
        require(path.is_relative_to(BASE/"routes") and path.name == entry["id"]+".json", "catalogue route path escapes/identity differs")
        pins[str(path)] = entry["sha256"]
    for name,sha in plan["source_sha256"].items():
        path = nominal.canonical(BASE/"provenance"/name)
        require(path.is_relative_to(BASE/"provenance") and owner["source_checks"].get(name) is True, "original generation source proof differs")
        pins[str(path)] = sha
    pins.update({str(nominal.REFERENCE):nominal.INPUTS[nominal.REFERENCE],str(nominal.XODR):nominal.INPUTS[nominal.XODR],str(OLD_FLAGS):FLAGS_SHA})
    return dict(sorted(pins.items())), catalog


def run_census(manifest, routes, xml, carla, original_flags):
    # HH_261002 - Invoke the unchanged whole-route probe. An unsupported endpoint/body never becomes a shortened or approved route.
    results, bodies = [], {}
    for entry,route in routes:
        require(route["town"] == "Town01" and route["scenario"] == entry["scenario"], "source route map/scenario differs")
        row = dict(case_id=entry["id"],source_route_sha256=entry["sha256"],original_encoded_node_count=len(route["route"]),original_route_length_m=route["route_length_m"],original_goal_endpoint_provenance=route["goal_endpoint_provenance"],original_physical_turn_preflight=route["physical_turn_preflight"],original_proximity_context_review=original_flags.get(entry["id"]),original_proximity_context_review_status="EXISTING_RIGHT_REVIEW" if entry["id"] in original_flags else "NO_EXISTING_LEFT_REVIEW_NOT_INFERRED_CLEAR",training_data_approved=False,vehicle_control_approved=False,capture_authorized=False)
        try:
            samples,census,groups = nominal.probe(manifest,route,xml,carla)
            clear = bool(samples) and all(not c["violation_sample_indices"] for c in census.values())
            row.update(status="NOMINAL_CLEAR_NOT_DRIVING_PROOF" if clear else "NOMINAL_BODY_FAIL_RETAINED",geometry=census,geometry_source_index_groups=groups,nominal_sample_count=len(samples),nominal_sweep_count=len(samples)-1,body_geometry_assessed=True)
            bodies[entry["id"]] = samples
        except (nominal.geometry.body.Unsupported,ValueError) as error:
            row.update(status="NOMINAL_UNSUPPORTED_NOT_PROVEN",error_type=type(error).__name__,error=str(error),body_geometry_assessed=False,geometry=None,nominal_sample_count=0,nominal_sweep_count=0)
        results.append(row)
    return results,bodies


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--caller-pins",type=Path); parser.add_argument("--caller-pins-sha256")
    parser.add_argument("--validate-only",action="store_true"); parser.add_argument("--print-contract",action="store_true")
    args = parser.parse_args(argv); expected,catalog = expected_inputs(); sources = source_pins()
    contract = dict(schema=SCHEMA+".caller_pins",nominal_only=True,case_ids=list(CASE_IDS),input_sha256=expected,source_sha256=sources,output=str(OUTPUT))
    if args.print_contract:
        require(args.caller_pins is None and args.caller_pins_sha256 is None and not args.validate_only,"contract-only mode cannot execute/validate")
        print(json.dumps(contract,indent=2,allow_nan=False)); return 0
    require(args.caller_pins is not None and args.caller_pins_sha256 is not None,"external caller path+SHA required")
    pins = {}; caller = json.loads(nominal.payload(args.caller_pins,args.caller_pins_sha256,pins))
    require(all(caller.get(k) == v for k,v in contract.items()),"full32 external nominal contract differs")
    output = nominal.canonical(caller["output"]); require(not output.exists(),"create-only census output already exists")
    for name,sha in sources.items(): nominal.payload(ROOT/name,sha,pins)
    data = {name:nominal.payload(name,sha,pins) for name,sha in expected.items()}
    nominal.payload(nominal.EGG,caller["carla_egg_sha256"],pins)
    flags = json.loads(data[str(OLD_FLAGS)])
    require(flags["catalog_sha256"] == CATALOG_SHA and flags["owner_plan_sha256"] == PLAN_SHA and flags["owner_result_sha256"] == OWNER_SHA,"original proximity/context audit differs")
    require(len(flags["candidates"]) == 16 and {f["route_id"] for f in flags["candidates"]} == {i for i in CASE_IDS if "_right_" in i},"all original right flags required")
    old_flags = {f["route_id"]:f for f in flags["candidates"]}
    routes = [(e,json.loads(data[str(nominal.canonical(BASE/e["path"]))])) for e in catalog["routes"]]
    require(all(old_flags[e["id"]]["sha256"] == e["sha256"] for e,_ in routes if e["id"] in old_flags),"old candidate review route SHA differs")
    if args.validate_only:
        require(all(nominal.geometry.sha(nominal.canonical(p)) == sha for p,sha in pins.items()),"validate input/source/API changed")
        print(json.dumps(dict(status="VALIDATED_ALL32_NO_GEOMETRY_NO_SIMULATOR",case_count=len(routes),source_count=len(sources),training_data_approved=False))); return 0
    require(sys.version_info[:2] == (3,10),"pinned offline CARLA Map binding requires Python3.10")
    saved = list(sys.path)
    try:
        sys.path.insert(0,str(nominal.EGG)); import carla
        require(str(carla.__file__).startswith(str(nominal.EGG)+"/"),"foreign offline CARLA provider")
        results,bodies = run_census(json.loads(data[str(nominal.REFERENCE)]),routes,data[str(nominal.XODR)],carla,old_flags)
    finally: sys.path[:] = saved
    require(all(nominal.geometry.sha(nominal.canonical(p)) == sha for p,sha in pins.items()),"first-read input/source/API changed before output")
    report = dict(schema=SCHEMA,status="ALL32_NOMINAL_CENSUS_COMPLETE_NOT_DRIVING_PROOF",case_count=len(results),case_ids=list(CASE_IDS),outcome_counts=dict(Counter(r["status"] for r in results)),results=results,input_sha256=pins,source_sha256=sources,original_generation_trace_coverage=catalog["generation"]["trace_coverage"],step_max_m=nominal.STEP_M,lane_center_error_limit_m=nominal.geometry.body.MAX_ROUTE_POINT_CENTER_ERROR_M,inset_m=nominal.geometry.body.LANE_INSET_M,sweep_position_guard_m=nominal.geometry.body.SWEEP_POSITION_GUARD_M,reference_capture_known_body_failed=True,nominal_roll_pitch_deg=[0.,0.],source_route_coordinates_modified=False,simulator_started=False,training_data_approved=False,vehicle_control_approved=False,capture_authorized=False,scope="All32 original encoded TRAIN routes; nominal road-tangent poses/all corners/snapshot-linear sweeps only. No observed motion, independent-site claim, teacher capture, future labels or admission.")
    output.mkdir(parents=True,exist_ok=False)
    for case_id,rows in bodies.items():
        with gzip.open(output/(case_id+".all_nominal_bodies_and_sweeps.jsonl.gz"),"xt") as stream:
            for row in rows: stream.write(json.dumps(row,allow_nan=False)+"\n")
    with (output/"summary.json").open("x") as stream: stream.write(json.dumps(report,indent=2,allow_nan=False)+"\n")
    print(json.dumps(dict(status=report["status"],case_count=len(results),outcome_counts=report["outcome_counts"],training_data_approved=False)))
    return 0


if __name__ == "__main__": raise SystemExit(main())
