"""HH_261002 - Add declared rear-reference transforms without changing the frozen nominal navigation geometry."""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import shutil

from scripts.e2e import town01_low_speed_left45_task_20261002 as frozen

ROOT, require = frozen.ROOT, frozen.require
SCHEMA = "carla.prospective_town01_right70_capture_route.20261002.v1"
BASE = ROOT/"storage/experiments/training/2026-10-02/town01_train_right_gentle_centerline_catalog_v1"
SOURCE = BASE/"routes/right/town01_right_centerline_6p1_108p1_24p1_20261002_v1.json"
SOURCE_SHA = "9e9160c13103455e73ea4560283502acf8b6d96deb974dd9db543c54ed8c5ea9"
SUMMARY = BASE/"summary.json"
SUMMARY_SHA = "3e404983959d4cbe54371286cc7d009558c1c193d34727402087b225d8f8b2b4"
OUTPUT = ROOT/"storage/experiments/plans/2026-10-02/town01_low_speed_right70_v1/capture_route.json"
GENERATOR_SHA = "1bab7f97146a0ca99388676ff9cc0f4a7ba04cf5bfe50edeb69af969a67e9f03"


def encoded(value):
    return (json.dumps(value, indent=2, sort_keys=True, allow_nan=False)+"\n").encode()


def build(route):
    # HH_261002 - These are prospective declarations, NOT measured poses. The unchanged collector applies +wheelbase/2 exactly once.
    require(route.get("coordinate_reference")==route.get("spawn_point_reference")=="base_link"
        and route.get("route_id")=="town01_right_centerline_6p1_108p1_24p1_20261002_v1"
        and route.get("town")=="Town01" and route.get("weather")=="ClearNoon", "exact generated TRAIN route identity required")
    require("start_carla_transform" not in route and "goal_carla_transform" not in route,
        "a previously transformed route cannot be translated again")
    poses=[route[label+"_ros_pose"] for label in ("start","goal")]
    require(all(set(p)=={"x","y","z","yaw"} and all(type(v) in (int,float) and math.isfinite(v) for v in p.values()) for p in poses),
        "complete finite generated rear poses required")
    require(all(poses[i]=={k:route["route"][index][k] for k in poses[i]} for i,index in ((0,0),(1,-1))),
        "full generated endpoints cannot be changed")
    provenance=route["generation_provenance"]
    require(provenance["schema"]=="portable_e2e.town01_train_right_gentle_centerline_nominal.20261002.v1.generation"
        and provenance["contract"]["lane_triples"]==[[6,0,1],[108,0,1],[24,0,1]]
        and provenance["contract"]["source_formulas_reused_unchanged"] is True,
        "original generated topology/provenance required")
    spawn=[float(v) for v in route["spawn_point"].split(",")]
    start=poses[0]
    require(len(spawn)==6 and all(math.isfinite(v) for v in spawn)
        and spawn==[start["x"],-start["y"],start["z"]+.3,0.,0.,-math.degrees(start["yaw"])],
        "authenticated generated spawn Z/roll/pitch and rear-reference signs required")
    result=copy.deepcopy(route)
    for label,pose in zip(("start","goal"),poses):
        result[label+"_carla_transform"]=dict(x=pose["x"],y=-pose["y"],z=spawn[2] if label=="start" else pose["z"],
            roll=spawn[3],pitch=spawn[4],yaw=-math.degrees(pose["yaw"]))
    result["capture_container_provenance"]=dict(schema=SCHEMA,source_generated_route_sha256=SOURCE_SHA,
        source_nominal_summary_sha256=SUMMARY_SHA,source_generator_sha256=GENERATOR_SHA,
        all_original_navigation_points_and_provenance_retained=True,transforms_reference="DECLARED_ROS_REAR_BASE_NOT_MEASURED",
        conversion="CARLA X=ROS X; Y=-ROS Y; yaw=-degrees(ROS yaw); zero roll/pitch from authenticated generated spawn.",
        start_z_source="Original generated spawn_point Z=map endpoint Z+0.3; unchanged collector adds its declared spawn_z_offset_m separately.",
        goal_z_source="Original full generated map endpoint Z; no pose/speed modification.",
        rear_to_actor_shift_applied_here=False,unchanged_collector_wheelbase_half_shift_required=True,
        admission_status="PENDING_QA",training_admission_state="PENDING_QA_NOT_AUTHORIZED",vehicle_control_approved=False)
    return result


def prepared():
    pins={}
    for path,expected in ((SOURCE,SOURCE_SHA),(SUMMARY,SUMMARY_SHA),(ROOT/"tools/e2e/generate_town01_train_right_gentle_centerline_nominal_20261002.py",GENERATOR_SHA)):
        require(path==path.resolve() and not any(p.is_symlink() for p in (path,*path.parents)),"canonical immutable inputs required")
        raw=path.read_bytes();require(hashlib.sha256(raw).hexdigest()==expected,"frozen nominal input/source changed")
        pins[str(path)]=expected
        if path==SOURCE: route=json.loads(raw)
        if path==SUMMARY: summary=json.loads(raw)
    require(summary["status"]=="NOMINAL_CLEAR_NOT_DRIVING_PROOF" and summary["generated_route_sha256"]==SOURCE_SHA
        and len(summary["results"])==1 and summary["results"][0]["nominal_sample_count"]==1941
        and summary["results"][0]["nominal_sweep_count"]==1940
        and all(not m["violation_sample_indices"] for m in summary["results"][0]["geometry"].values()),"whole nominal witness required, not driving approval")
    return encoded(build(route)),pins


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument("--caller-pins",type=Path);parser.add_argument("--caller-pins-sha256")
    parser.add_argument("--print-contract",action="store_true")
    args=parser.parse_args(argv)
    raw,pins=prepared(); contract=dict(schema=SCHEMA,output=str(OUTPUT),input_sha256=pins,
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),capture_route_sha256=hashlib.sha256(raw).hexdigest(),
        prospective_declared_not_observed=True,capture_authorized=False,research_training_authorized=False)
    if not args.print_contract:
        require(args.caller_pins is not None and args.caller_pins_sha256 is not None,"ROOT external container authority required")
        caller=args.caller_pins.absolute();require(caller==caller.resolve() and caller.is_relative_to(ROOT)
            and not any(p.is_symlink() for p in (caller,*caller.parents)),"canonical external authority required")
        authority=caller.read_bytes();require(hashlib.sha256(authority).hexdigest()==args.caller_pins_sha256,"external authority SHA differs")
        require(json.loads(authority)==dict(contract,container_creation_authorized=True),"exact ROOT container permission, not capture/admission permission, required")
        require(not OUTPUT.exists() and OUTPUT==OUTPUT.resolve() and not any(p.is_symlink() for p in (OUTPUT,*OUTPUT.parents)),"fresh canonical container only")
        require(shutil.disk_usage(ROOT).free>=5*1024**3,"retain the original5GiB workspace disk floor")
        for path,sha in pins.items():require(hashlib.sha256(Path(path).read_bytes()).hexdigest()==sha,"input changed before output")
        require(hashlib.sha256(Path(__file__).read_bytes()).hexdigest()==contract["source_sha256"],"container source changed")
        OUTPUT.parent.mkdir(parents=True,exist_ok=True)
        with OUTPUT.open("xb") as stream:stream.write(raw)
        require(OUTPUT.read_bytes()==raw,"new container bytes differ from the externally sealed derivation")
        for path,sha in pins.items():require(hashlib.sha256(Path(path).read_bytes()).hexdigest()==sha,"input changed during metadata creation")
        require(caller.read_bytes()==authority and hashlib.sha256(Path(__file__).read_bytes()).hexdigest()==contract["source_sha256"],
            "external authority/source changed during metadata creation")
    print(json.dumps(contract,indent=2,allow_nan=False));return 0


if __name__=="__main__":raise SystemExit(main())
