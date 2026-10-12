#!/usr/bin/env python3
"""HH_261001 - Verify every paired RGB/raw sensor frame and offline oracle replay."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/"learning"))
import numpy as np
from PIL import Image
from portable_e2e.mapping import SemanticEvidenceMap
from portable_e2e.mapping_projection import project_frame
try:
    from . import collect_map_paired_rgb_20261001 as capture
    from . import audit_ctrack_isolated_moving_20261001 as geometry
except ImportError:
    import collect_map_paired_rgb_20261001 as capture
    import audit_ctrack_isolated_moving_20261001 as geometry

LABELS = {1:"road",24:"lane_marking",2:"sidewalk",7:"traffic_light_fixture",8:"traffic_sign",6:"pole"}


def require(value,message):
    if not value:
        raise ValueError(message)


def audit(source):
    source = source.absolute()
    require(source.is_dir() and source.resolve() == source,"regular absolute capture directory required")
    owner = source.parent
    receipts = {name:owner/name for name in ("owner_plan.json","owner_started.json","owner_result.json",
        "lifecycle/ready.json","lifecycle/after_capture.json","lifecycle/stopped.json")}
    receipts.update({"summary.json":source/"summary.json","records.json":source/"records.json","map.json":source/"map.json"})
    before = {name:capture.digest(path) for name,path in receipts.items()}
    objects = {name:json.loads(path.read_text()) for name,path in receipts.items()}
    plan,result,summary,records = (objects[name] for name in
        ("owner_plan.json","owner_result.json","summary.json","records.json"))
    require(plan["capture_mode"] == result["capture_mode"] == "mapping-paired-rgb"
        and result["exit_code"] == 0 and result["source_bytes_unchanged_and_archived"] is True,
        "paired owner did not complete")
    require(all(objects[name]["status"] == "PASS" for name in
        ("lifecycle/ready.json","lifecycle/after_capture.json","lifecycle/stopped.json")),"owned lifecycle failed")
    require(all(item.get("intended_role") == capture.ROLE
        and item.get("train_eligible") is False and item.get("dataset_admitted") is False
        and item.get("ctrack_train_eligible") is False for item in (plan,result,summary)),"diagnostic roles changed")
    for logical,sha in plan["source_sha256"].items():
        current = ROOT/(logical.replace("scripts/","tools/",1) if logical.startswith("scripts/") else
            "learning/"+logical if logical.startswith("portable_e2e/") else logical)
        require(capture.digest(current) == capture.digest(owner/"provenance"/logical) == sha,
                "archived/current owner source differs")
    external = plan.get("external_source_provenance",{})
    require(any(name.endswith("/basic_agent.py") for name in external),"BasicAgent archive missing")
    for row in external.values():
        require(capture.digest(Path(row["path"])) == capture.digest(owner/row["archived_path"]) == row["sha256"],
                "agent source differs")
    route_path = Path(plan["route_path"])
    route_sha = capture.digest(route_path)
    route = json.loads(route_path.read_text())
    require(route_sha in capture.ROUTES and route_sha == plan["route_sha256"] == summary["route_sha256"]
        and (route["town"],route["scenario"]) == capture.ROUTES[route_sha],"route bytes or TRAIN scope changed")
    require(summary["status"] == "COMPLETE_ORACLE_PLUMBING_ONLY" and summary["frame_count"] == len(records) == 240
        and summary["collision_events"] == 0 and summary["cleanup_errors"] == []
        and summary["live_replay_exact_match"] is True and summary["source_assets_unchanged"] is True,
        "paired sensor capture/replay/cleanup incomplete")
    require(summary["source_assets"] == capture.package_assets(route["town"]),"packaged assets changed")
    binding = summary["owner_binding"]
    require(binding["owner_plan_sha256"] == before["owner_plan.json"]
        and binding["owner_started_sha256"] == before["owner_started.json"]
        and binding["live_xodr_sha256"] == capture.XODR_SHA[route["town"]],"pre-actor binding changed")
    require(summary["camera_mount"] == {"x":1.5,"y":0.0,"z":2.0,"pitch":-12.0}
        and summary["sensor_intrinsics"]["K"] == [[320.,0.,320.],[0.,320.,180.],[0.,0.,1.]],"fixed calibration differs")
    attributes = summary["sensor_actor_attributes"]
    first,second = dict(attributes["rgb_false"]),dict(attributes["rgb_true"])
    require(first.pop("enable_postprocess_effects") == "false"
        and second.pop("enable_postprocess_effects") == "true" and first == second,"RGB arms differ beyond postprocess")
    require(attributes["depth"]["enable_postprocess_effects"] == attributes["semantic"]["enable_postprocess_effects"] == "false",
        "raw target rendering changed")
    require(summary["label_table"] == {str(k):v for k,v in LABELS.items()},"semantic mapping changed")
    require(len(list((source/"frames").glob("*.npz"))) == 240
        and len(list((source/"frames").glob("*.png"))) == 480,"complete four-sensor artifact count required")
    xy = np.asarray([(row["x"],-row["y"]) for row in route["route"]],dtype=np.float64)
    distances = np.asarray([row["distance_m"] for row in route["route"]],dtype=np.float64)
    positions,stations,offsets,frames,hashes = [],[],[],[],{}
    evidence = SemanticEvidenceMap(f"carla_ue:{route['town']}","oracle_sensor",max_cells=200000)
    total_road = total_lane = lane_frames = 0
    for index,row in enumerate(records):
        stem = f"frames/{index:04d}"
        require(row["stem"] == stem,"frame sequence differs")
        require(math.isfinite(row["timestamp_s"]) and math.isfinite(row["speed_mps"]),"nonfinite frame metadata")
        if index:
            require(row["frame"] == records[index-1]["frame"]+1
                and abs(row["timestamp_s"]-records[index-1]["timestamp_s"]-.1) < .001,"frame/timestamp cadence differs")
        for suffix in (".false.png",".true.png",".npz"):
            value = capture.digest(source/(stem+suffix))
            require(value == row["artifact_sha256"][suffix],"raw artifact hash differs")
            hashes[stem+suffix] = value
        with np.load(source/(stem+".npz"),allow_pickle=False) as data:
            require(set(data.files) == {"rgb_false","rgb_true","depth","semantic","world_from_camera"},"raw archive keys differ")
            arrays = {name:data[name].copy() for name in ("rgb_false","rgb_true","depth","semantic")}
            matrix = data["world_from_camera"].copy()
        for name,array in arrays.items():
            require(array.shape == (360,640,4) and array.dtype == np.uint8,"native sensor ABI differs")
            metadata = row["sensors"][name]
            require(metadata["frame"] == row["frame"] and abs(metadata["timestamp_s"]-row["timestamp_s"]) < 1e-6
                and metadata["width"] == 640 and metadata["height"] == 360
                and np.allclose(metadata["world_from_camera"],matrix,atol=1e-5)
                and metadata["raw_bgra_sha256"] == hashlib.sha256(array.tobytes()).hexdigest(),
                "independent sensor sync/pose/digest differs")
        for arm in ("false","true"):
            with Image.open(source/(stem+"."+arm+".png")) as image:
                require(image.mode == "RGB" and np.array_equal(np.asarray(image),arrays["rgb_"+arm][...,[2,1,0]]),
                        "saved PNG differs from raw sensor")
        errors = geometry._pose_errors(matrix,row)
        require(errors is not None and errors[0] <= .3 and errors[1] <= 5,"camera mount/rigidity differs")
        depth = geometry._depth_m(arrays["depth"])
        valid_depth = float(((depth >= .1)&(depth <= 40)&np.isfinite(depth)).mean())
        require(valid_depth >= .05,"insufficient valid projection depth")
        tags = arrays["semantic"][...,2]
        road,lane = int((tags == 1).sum()),int((tags == 24).sum())
        require((road,lane) == (row["road_pixels"],row["lane_pixels"]),"semantic counts changed")
        total_road += road; total_lane += lane; lane_frames += int(lane > 0)
        point = (float(row["ego_x"]),float(row["ego_y"]))
        station,offset = geometry._nearest_route_s(point,xy,distances)
        require(offset <= 5,"trajectory departed pinned route")
        if positions:
            require(math.dist(point,positions[-1]) <= 5,"trajectory teleported")
        positions.append(point);stations.append(station);offsets.append(offset)
        observation = project_frame(arrays["depth"],arrays["semantic"],matrix,frame_id=evidence.frame_id,
            timestamp_s=row["timestamp_s"],fov_deg=90,label_table=LABELS)
        require(len(observation.points) == row["points"],"saved point projection count differs")
        evidence.update(observation)
        difference = np.abs(arrays["rgb_false"][...,:3].astype(np.float32)-arrays["rgb_true"][...,:3].astype(np.float32))
        frames.append({"index":index,"frame":row["frame"],"road_pixels":road,"lane_pixels":lane,
            "valid_depth_fraction":valid_depth,"route_station_m":station,"route_offset_m":offset,
            "paired_rgb_mean_absolute_pixel_difference":float(difference.mean()),
            "class_pixels_native_raw":np.bincount(tags.ravel(),minlength=256).tolist()})
    require(evidence.snapshot() == objects["map.json"],"independent raw replay differs from live map")
    travel = sum(math.dist(a,b) for a,b in zip(positions,positions[1:]))
    retained = geometry._retained_count(positions)
    regression,drawdown = geometry._station_regression(stations)
    require(retained >= 120 and travel >= 3 and abs(travel-summary["travel_m"]) <= max(.1,.001*travel),"motion coverage differs")
    require(regression <= 2 and drawdown <= 3,"route station regressed")
    require(total_road >= .01*240*360*640 and total_lane >= .0001*240*360*640 and lane_frames >= 24,
        "road/lane coverage insufficient")
    require(total_road == summary["road_pixels"] and total_lane == summary["lane_pixels"],"summary semantic counts differ")
    require(all(capture.digest(path) == before[name] for name,path in receipts.items())
        and all(capture.digest(source/name) == sha for name,sha in hashes.items()),"source bytes changed during audit")
    return {"schema":"portable_e2e.map_paired_rgb_qa.v1","status":"QA_CONDITIONAL",
        "source":str(source),"intended_role":capture.ROLE,"train_eligible":False,"dataset_admitted":False,
        "ctrack_train_eligible":False,"independent_dev_approved":False,"manual_render_review_required":True,
        "frame_count":240,"all_four_sensor_sync_pose_hashes_verified":True,"live_raw_map_replay_exact":True,
        "receipt_sha256":before,"frame_artifact_sha256":hashes,"frames":frames,"route_sha256":route_sha,
        "retained_positions_0p25m":retained,"travel_m":travel,"maximum_route_offset_m":max(offsets),
        "road_pixels":total_road,"lane_pixels":total_lane,"lane_positive_frames":lane_frames,
        "source_fingerprints_unchanged":True,"whole_route_completion_required":False,
        "scope":"240-frame existing TRAIN prefix, not a new route/site or whole-junction claim",
        "auditor_sha256":capture.digest(Path(__file__).resolve()),"model_predictions_made":False}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source",type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True)
    args = parser.parse_args()
    require(not args.output.exists() and not args.output.is_symlink()
        and not args.output.absolute().is_relative_to(args.source.absolute()),"fresh report outside capture required")
    report = audit(args.source)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open("x") as stream:
        json.dump(report,stream,indent=2,sort_keys=True,allow_nan=False);stream.write("\n")
    print(json.dumps({key:report[key] for key in ("status","frame_count","travel_m","road_pixels","lane_pixels")}))
