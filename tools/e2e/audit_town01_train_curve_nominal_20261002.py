"""HH_261002 - Pin one TRAIN route and test nominal body geometry only; never synthesize observed driving."""

from __future__ import annotations

import argparse
import gzip
import hashlib
import json
import math
from pathlib import Path
import sys

from scripts.e2e import audit_simulation_research_body_lane_20261001 as geometry
from scripts.e2e import audit_temporal_route_recovery_fast45_batch_20261001 as archive

ROOT = Path(__file__).resolve().parents[2]
SCHEMA = "portable_e2e.town01_train_right_nominal_body_preflight.20261002.v1"
ROUTE = ROOT/"storage/experiments/training/2026-09-27/drive_long_route_town01_train_catalog_v4/routes/town01/right/town01_right_s0003_p00.json"
REFERENCE = ROOT/"storage/experiments/training/2026-09-27/drive_long_route_town01_right_s0002_p06_9kph_steer045_trial_v1/episode/manifest.json"
XODR = REFERENCE.with_name("active_map_opendrive.xodr")
EGG = Path("/home/a/carla-autoware-universe/CARLA_0.9.15/PythonAPI/carla/dist/carla-0.9.15-py3.10-linux-x86_64.egg")
INPUTS = {ROUTE:"b539a2a4dd88e303bce631793a240dba96b11a1bd2087123536410d5f5dd5e70", REFERENCE:"09c9bcf90496d98a71698056ff81993bac7f136ff962f1b538dbc22fcd6d0fc4", XODR:"97a7f6ac67812567e5c8ee0599cd823b23f80f30f3f97c502212e38b72e2b709"}
FROZEN = {"tools/e2e/audit_simulation_research_body_lane_20261001.py":"6e932dcb39fa40e664218d3d1dddf20dd524f54e99ca55d98793c76cce8c03ac", "tools/e2e/audit_future_actor_lane_containment.py":"75d73449c8ffb17e27d1ddb419bd064a58ab052043b2a7b0ada69de8a65fa57d", "tools/e2e/audit_temporal_route_recovery_fast45_batch_20261001.py":"f95c0c424287120dbeff8212658497b38892c8a5dab7cb11f57a2e7d0b15bd87"}
STEP_M = .12
require = geometry.require


def canonical(path):
    path = Path(path).absolute()
    require(path.resolve() == path and not any(p.is_symlink() for p in (path, *path.parents)), "canonical non-symlink path required")
    return path


def payload(path, expected, pins):
    path = canonical(path); data = path.read_bytes()
    require(hashlib.sha256(data).hexdigest() == expected, "external SHA differs: "+str(path))
    require(str(path) not in pins or pins[str(path)] == expected, "conflicting first-read pin")
    pins[str(path)] = expected
    return data


def source_pins():
    result = {str(path.relative_to(ROOT)):geometry.sha(canonical(path)) for path in archive.source_closure(Path(__file__).resolve())}
    require(all(result.get(name) == sha for name,sha in FROZEN.items()), "frozen geometry/source provider changed")
    return dict(sorted(result.items()))


def dedup_points(route):
    require(route["coordinate_reference"] == "base_link", "ROS rear-reference route required")
    points, groups = [], []
    for i,p in enumerate(route["route"]):
        geometry.body._triple(p)
        require(all(type(p[k]) in (int,float) and math.isfinite(p[k]) for k in ("x","y","z","yaw","distance_m")), "finite encoded route required")
        if points and p["distance_m"] == points[-1]["distance_m"]:
            allowed = {"index","road_option","road_option_value","vad_command"}
            require({k:v for k,v in p.items() if k not in allowed} == {k:v for k,v in points[-1].items() if k not in allowed}, "duplicate command node changes geometry/lane")
            groups[-1].append(i)
        else:
            require(not points or p["distance_m"] > points[-1]["distance_m"], "route distance reverses")
            points.append(p); groups.append([i])
    require(len(points) >= 2 and points[0]["distance_m"] == 0., "complete source route required")
    return points, groups


def nominal_samples(points, wheelbase):
    # HH_261002 - Encoded road-tangent yaw is interpolated; roll/pitch zero and arc steps are hypotheses, not native time or physics.
    samples = []
    for segment,(a,b) in enumerate(zip(points,points[1:])):
        length = b["distance_m"]-a["distance_m"]
        require(math.hypot(b["x"]-a["x"],b["y"]-a["y"]) > 1e-4, "degenerate encoded XY segment")
        count = math.ceil(length/STEP_M)
        delta = math.atan2(math.sin(b["yaw"]-a["yaw"]),math.cos(b["yaw"]-a["yaw"]))
        for part in range(count+1):
            fraction = part/count; arc = a["distance_m"]+fraction*length
            if samples and arc == samples[-1]["nominal_rear_arc_m"]: continue
            rear = {k:a[k]+fraction*(b[k]-a[k]) for k in ("x","y","z")}; yaw = -(a["yaw"]+fraction*delta)
            pose = dict(x=rear["x"]+wheelbase/2*math.cos(yaw), y=-rear["y"]+wheelbase/2*math.sin(yaw), z=rear["z"], roll=0.,pitch=0.,yaw=math.degrees(yaw))
            samples.append(dict(nominal_sample_index=len(samples), nominal_rear_arc_m=arc, geometry_segment=segment, fraction=fraction, rear_ROS_xyz=rear, nominal_actor_pose_carla=pose))
    return samples


def probe(manifest, route, xml, carla):
    points, groups = dedup_points(route); wheelbase = manifest["coordinate_contract"]["wheelbase_m"]
    record = manifest["capture_contract"]["actor_body_geometry"]; construction = manifest["capture_contract"]["agent_initialization"]["construction"]
    require(wheelbase == 2.85 and record["status"] == "RECORDED" and record["source"] == "carla.Vehicle.bounding_box" and record["actor_type_id"] == "vehicle.toyota.prius" and record["reference_frame"] == "CARLA actor-local; not ROS rear-axle base_link", "actual reference Prius geometry required")
    require(construction["status"] == "PASS" and all(construction[side]["actor_id"] == record["actor_id"] for side in ("before","after")), "reference body actor is not bound to constructor witnesses")
    m = manifest["capture_contract"]["active_map_opendrive"]
    require(m["status"] == "RECORDED" and m["source"] == "carla.Map.to_opendrive" and m["map_name"] == "Carla/Maps/Town01" and m["sha256"] == hashlib.sha256(xml).hexdigest() and m["size_bytes"] == len(xml), "actual archived Town01 map identity differs")
    box,radius = geometry.body._body_box(manifest,carla)
    corridors = geometry.body.lane_corridors(carla.Map("nominal-Town01-NOT-observed-drive",xml.decode()),carla,points,xml,{geometry.body._triple(p) for p in points},set(range(len(points))))
    rows, previous, cache = nominal_samples(points,wheelbase), None, {}
    def regions(lo,hi):
        ids = frozenset(geometry.body._route_lanes_for_arc_interval(points,lo,hi))
        if ids not in cache: cache[ids] = {name:geometry.body.authorized_lane_region(corridors,ids,inset) for name,inset in (("raw_lane",0.),("inset_10cm_lane",.1))}
        return cache[ids], [list(k) for k in sorted(ids)]
    for row in rows:
        pose = row["nominal_actor_pose_carla"]; arc,segment,distance = geometry.body._route_projection((pose["x"],pose["y"]),points)
        row.update(independent_nominal_actor_arc_m=arc, independent_nominal_actor_route_distance_m=distance)
        footprint = geometry.body.body_footprint(box,pose,carla)
        allowed,ids = regions(arc-radius,arc+radius)
        transform = carla.Transform(carla.Location(**{k:pose[k] for k in ("x","y","z")}),carla.Rotation(**{k:pose[k] for k in ("roll","pitch","yaw")}))
        row.update(intended_lane_ids=ids, all_eight_box_vertices_carla_xyz=[[v.x,v.y,v.z] for v in box.get_world_vertices(transform)], body_polygon_carla_xy=list(footprint.exterior.coords), sampled_body={name:geometry.region_measurement(footprint,r) for name,r in allowed.items()}, previous_sweep=None)
        if previous:
            old,old_shape = previous; sweep = geometry.body.swept_body(old_shape,footprint,old["nominal_actor_pose_carla"],pose,radius)
            allowed,ids = regions(old["independent_nominal_actor_arc_m"]-radius,arc+radius)
            row["previous_sweep"] = dict(from_nominal_sample_index=old["nominal_sample_index"], intended_lane_ids=ids, envelope_carla_xy=list(sweep.exterior.coords), regions={name:geometry.region_measurement(sweep,r) for name,r in allowed.items()})
        previous = row,footprint
    census = {}
    for kind in ("sampled_body","previous_sweep"):
        for region in ("raw_lane","inset_10cm_lane"):
            values = [(r["nominal_sample_index"],r[kind][region] if kind == "sampled_body" else r[kind]["regions"][region]) for r in rows if r[kind] is not None]
            margins = [(i,v["minimum_boundary_clearance_m"]) for i,v in values if v["minimum_boundary_clearance_m"] is not None]
            census[kind+"_"+region] = dict(assessed_count=len(values), violation_sample_indices=[i for i,v in values if not v["area_gate_clear"]], maximum_outside_m2=max(v["outside_m2"] for _,v in values), minimum_interior_clearance_m=min(v for _,v in margins) if margins else None)
    return rows,census,groups


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__); parser.add_argument("--caller-pins",type=Path,required=True); parser.add_argument("--caller-pins-sha256",required=True); parser.add_argument("--validate-only",action="store_true"); args = parser.parse_args(argv)
    pins = {}; caller = json.loads(payload(args.caller_pins,args.caller_pins_sha256,pins))
    require(caller["schema"] == SCHEMA+".caller_pins" and caller["nominal_only"] is True and caller["input_sha256"] == {str(p):s for p,s in INPUTS.items()}, "fixed external nominal input contract differs")
    output = canonical(caller["output"]); require(output.is_relative_to(ROOT/"storage/experiments/diagnostics/2026-10-02") and not output.exists(), "fresh diagnostic directory required")
    require(caller["source_sha256"] == source_pins(), "exact source closure differs")
    for name,sha in caller["source_sha256"].items(): payload(ROOT/name,sha,pins)
    data = {p:payload(p,sha,pins) for p,sha in INPUTS.items()}; payload(EGG,caller["carla_egg_sha256"],pins)
    if args.validate_only: print(json.dumps(dict(status="VALIDATED_NO_GEOMETRY_NO_SIMULATOR",source_count=len(caller["source_sha256"]),training_data_approved=False))); return 0
    require(sys.version_info[:2] == (3,10), "pinned offline CARLA Map binding requires Python3.10")
    saved = list(sys.path)
    try:
        sys.path.insert(0,str(EGG)); import carla
        require(str(carla.__file__).startswith(str(EGG)+"/"), "foreign offline CARLA provider")
        rows,census,groups = probe(json.loads(data[REFERENCE]),json.loads(data[ROUTE]),data[XODR],carla)
    finally: sys.path[:] = saved
    require(all(geometry.sha(canonical(p)) == sha for p,sha in pins.items()), "first-read input/source/API changed")
    clear = all(not c["violation_sample_indices"] for c in census.values())
    report = dict(schema=SCHEMA,status="NOMINAL_CLEAR_NOT_DRIVING_PROOF" if clear else "NOMINAL_FAIL_RETAINED",input_sha256=pins,source_sha256=caller["source_sha256"],original_encoded_node_count=228,geometry_source_index_groups=groups,nominal_sample_count=len(rows),nominal_sweep_count=len(rows)-1,step_max_m=STEP_M,wheelbase_m=2.85,reference_capture_known_body_failed=True,nominal_roll_pitch_deg=[0.,0.],source_route_coordinates_modified=False,nominal_tangent_yaw_source="wrapped interpolation of original encoded road-tangent yaw",geometry=census,scope="All nominal route poses/corners/snapshot-linear sweeps only; no measured frames/timestamps, actual motion, new actor identity, controller, admission or continuous-time proof.",simulator_started=False,training_data_approved=False,vehicle_control_approved=False,capture_authorized=False)
    output.mkdir(parents=True,exist_ok=False)
    with gzip.open(output/"all_nominal_bodies_and_sweeps.jsonl.gz","xt") as stream:
        for row in rows: stream.write(json.dumps(row,allow_nan=False)+"\n")
    with (output/"summary.json").open("x") as stream: stream.write(json.dumps(report,indent=2,allow_nan=False)+"\n")
    print(json.dumps(dict(status=report["status"],sample_count=len(rows),sweep_count=len(rows)-1,geometry=census,training_data_approved=False)))
    return 0 if clear else 2


if __name__ == "__main__": raise SystemExit(main())
