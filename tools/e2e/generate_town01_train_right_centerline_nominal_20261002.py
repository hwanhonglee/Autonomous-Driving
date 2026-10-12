"""HH_261002 - Generate one new XODR-centred TRAIN-corridor candidate; unchanged whole-body nominal gates are not driving or admission proof."""
from __future__ import annotations

import argparse
from collections import Counter
import copy
import gzip
import hashlib
import json
import math
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET

from scripts.e2e import audit_town01_train_curve_nominal_20261002 as nominal
from scripts.e2e import site_split_guard_v2 as site_guard

ROOT = nominal.ROOT
SCHEMA = "portable_e2e.town01_train_right_centerline_nominal.20261002.v1"
CASE_ID = "town01_right_centerline_24p1_256p1_12m1_20261002_v1"
OUTPUT = ROOT/"storage/experiments/training/2026-10-02/town01_train_right_centerline_catalog_v1"
BASE = nominal.ROUTE.parents[3]
HISTORY = ROOT/"storage/experiments/diagnostics/2026-10-02/town01_train_turn_nominal_census_v2/summary.json"
REGISTRY = ROOT/"learning/config/site_split_guard_20260929_v2.json"
FROZEN = {**nominal.FROZEN,
    "tools/e2e/audit_town01_train_curve_nominal_20261002.py": "821cad90fcb515533f97434ed46321f3f26eb5921a4872af000c72a62179cb20",
    "tools/e2e/site_split_guard_v2.py": "c9066558339a4ac7d11ff81e04798dbfb098d4dd3b571fef1fcce2e3231c7fd3"}
INPUTS = {**nominal.INPUTS,
    BASE/"route_catalog.json": "0c0cade80ec797de39320157e35c90661ac172caef5fa7a21368036a814c5897",
    BASE/"owner_plan.json": "b26406801fa21bd44f9e992dc7ce559da131f85a30e8c8061ae746a55c447d9f",
    BASE/"owner_result.json": "64468bdcac0e4002b773c73cef2722b5b6a8b428c2564ed769fe21efaea9b393",
    HISTORY: "94943ff013c17b3c80017d1ac4538dad0cdddc9c62b5c6063722688b8d598475",
    REGISTRY: "323b68c690724bdb63a9955c9573a249d5c49f5001160a421e797249dce7ec0b",
    ROOT/"learning/config/site_split_guard_20260929.json": site_guard.PARENT_V1_SHA256[site_guard.PARENT_REGISTRY]}
CORRIDOR = ((24,0,1),(256,0,1),(12,0,-1))
RESOLUTION = 1.0
require = nominal.require


def generation_contract():
    return dict(case_ids=[CASE_ID], original_case="town01_right_s0003_p00", map="Town01", split_intent="TRAIN_CANDIDATE",
        lane_triples=[list(t) for t in CORRIDOR], endpoint_policy="Project exact original first/last encoded XY onto their straight XODR reference lines; generate NEW lane-centre endpoints at those longitudinal s values.",
        sampling_policy="Fixed 1.0m reference-s lattice plus every geometry breakpoint and both endpoints on ALL three lanes; only roundoff-equivalent shared endpoints use one position with both source memberships; no outcome-dependent spacing, clipping or retrace.",
        shared_endpoint_representation=dict(maximum_binary64_ULPs=64, only_adjacent_topology_endpoints=True,
            exact_upstream_pose_retained=True, downstream_lane_and_command_primary=True,
            both_original_endpoint_poses_and_lane_s_memberships_retained=True,
            tiny_noncoincident_segments_remain_rejected_by_original_1e_minus4_guard=True),
        allowed_XODR_geometry=["line","arc"], lane_width_m=4., lane_offset_and_elevation_and_superelevation=0.,
        seam_xy_max_m=.005, seam_heading_max_rad=.001, original_228_nodes_and_failed_outcome_retained=True,
        nominal_step_max_m=.12, lane_match_max_m=.08, body_inset_m=.1, sweep_position_guard_m=.02,
        body_vertices_per_pose=8, whole_route_all_N_bodies_and_N_minus_1_sweeps=True,
        source_site_permission="Town01/drive/reuse_existing_train", same_TRAIN_corridor_not_new_independent_site=True,
        independent_DEV_or_TEST_claim=False, simulator_started=False, capture_authorized=False, training_data_approved=False)


def source_pins():
    paths = nominal.archive.source_closure(Path(__file__).resolve(), ROOT/"tools/e2e/site_split_guard.py")
    pins = {str(p.relative_to(ROOT)):nominal.geometry.sha(nominal.canonical(p)) for p in paths}
    require(all(pins.get(k) == v for k,v in {**FROZEN, "tools/e2e/site_split_guard.py":site_guard.PARENT_V1_SHA256["tools/e2e/site_split_guard.py"]}.items()), "frozen nominal or site-policy source changed")
    require((nominal.STEP_M,nominal.geometry.body.MAX_ROUTE_POINT_CENTER_ERROR_M,nominal.geometry.body.LANE_INSET_M,nominal.geometry.body.SWEEP_POSITION_GUARD_M) == (.12,.08,.1,.02), "nominal safety constants changed")
    return dict(sorted(pins.items()))


def number(value):
    require(type(value) in (float,int) and math.isfinite(value), "finite numeric geometry required")
    return float(value)


def road_model(xml_road, lane_id):
    # HH_261002 - This narrowly declared corridor supports constant 4m lanes and flat line/arc geometry only. Unsupported XML is a retained failure.
    require(xml_road is not None and lane_id in (-1,1), "required road/lane missing")
    length = number(float(xml_road.get("length"))); require(length > 0, "positive road length required")
    for tag in ("lanes/laneOffset","elevationProfile/elevation","lateralProfile/superelevation"):
        values = xml_road.findall(tag)
        require(values and all(all(float(v.get(k)) == 0. for k in "abcd") for v in values), "only declared flat zero-offset geometry is supported")
    sections = xml_road.findall("lanes/laneSection")
    require(len(sections) == 1 and float(sections[0].get("s")) == 0., "exact single lane section required")
    lane = sections[0].find(("left" if lane_id > 0 else "right")+f"/lane[@id='{lane_id}']")
    require(lane is not None and lane.get("type") == "driving", "declared driving lane missing")
    widths = lane.findall("width")
    require(len(widths) == 1 and float(widths[0].get("sOffset")) == 0. and [float(widths[0].get(k)) for k in "abcd"] == [4.,0.,0.,0.], "exact constant4m driving width required")
    geometries = []
    for g in xml_road.findall("planView/geometry"):
        p = {k:number(float(g.get(k))) for k in ("s","x","y","hdg","length")}
        require(len(g) == 1 and g[0].tag in ("line","arc") and p["length"] > 0, "only positive line/arc geometry supported")
        p["curvature"] = number(float(g[0].get("curvature"))) if g[0].tag == "arc" else 0.
        require(1-lane_id*2*p["curvature"] > 0., "offset lane reverses/cusps")
        require(p["s"] == 0. if not geometries else abs(p["s"]-(geometries[-1]["s"]+geometries[-1]["length"])) <= 1e-9, "full contiguous planView required")
        if geometries:
            previous = reference_pose(geometries[-1],geometries[-1]["length"])
            require(math.hypot(p["x"]-previous[0],p["y"]-previous[1]) <= .005 and abs(wrap(p["hdg"]-previous[2])) <= .001, "encoded geometry seam discontinuity")
        geometries.append(p)
    require(geometries and abs(geometries[-1]["s"]+geometries[-1]["length"]-length) <= 1e-9, "planView must span whole road")
    return dict(road_id=int(xml_road.get("id")), lane_id=lane_id, length=length, geometries=geometries, junction=xml_road.get("junction") != "-1")


def wrap(angle):
    return math.atan2(math.sin(angle),math.cos(angle))


def reference_pose(g, ds):
    h,k = g["hdg"],g["curvature"]
    if k == 0.: return g["x"]+ds*math.cos(h),g["y"]+ds*math.sin(h),h
    return g["x"]+(math.sin(h+k*ds)-math.sin(h))/k,g["y"]-(math.cos(h+k*ds)-math.cos(h))/k,h+k*ds


def centre_pose(road, s):
    require(0 <= s <= road["length"], "s outside declared full lane")
    g = next(g for g in reversed(road["geometries"]) if g["s"] <= s)
    x,y,h = reference_pose(g,s-g["s"]); t = road["lane_id"]*2.
    return dict(x=x-t*math.sin(h), y=y+t*math.cos(h), z=0., yaw=wrap(h+(math.pi if road["lane_id"] > 0 else 0.)))


def endpoint_s(road, source):
    require(len(road["geometries"]) == 1 and road["geometries"][0]["curvature"] == 0., "source endpoint must be on declared straight road")
    g = road["geometries"][0]
    s = (number(source["x"])-g["x"])*math.cos(g["hdg"])+(number(source["y"])-g["y"])*math.sin(g["hdg"])
    require(0 < s < road["length"], "source endpoint projects outside lane interior; never clamp")
    return s


def lattice(road, first, last):
    lo,hi = sorted((first,last))
    values = {first,last,*(float(i)*RESOLUTION for i in range(math.ceil(lo/RESOLUTION),math.floor(hi/RESOLUTION)+1))}
    values.update(g["s"] for g in road["geometries"] if lo <= g["s"] <= hi)
    return sorted(values,reverse=first > last)


def generate(xml, original):
    require(original.get("town") == "Town01" and original.get("weather") == "ClearNoon" and original.get("scenario") == "right" and original.get("coordinate_reference") == "base_link", "only original Town01 TRAIN right corridor is supported")
    observed = [nominal.geometry.body._triple(p) for p in original["route"]]
    groups = [t for i,t in enumerate(observed) if i == 0 or observed[i-1] != t]
    require(tuple(groups) == CORRIDOR, "original corridor/order changed; no alternate lane or site selection")
    root = ET.fromstring(xml); roads = [road_model(root.find(f"road[@id='{r}']"),lane) for r,_,lane in CORRIDOR]
    middle = root.find("road[@id='256']")
    require(middle.find("link/predecessor").attrib == dict(elementType="road",elementId="12",contactPoint="start") and middle.find("link/successor").attrib == dict(elementType="road",elementId="24",contactPoint="start"), "exact reverse-connecting road topology required")
    lane = middle.find("lanes/laneSection/left/lane[@id='1']")
    require(lane.find("link/predecessor").get("id") == "-1" and lane.find("link/successor").get("id") == "1", "lane topology changed")
    matches = [c for c in root.findall("junction[@id='255']/connection") if c.get("incomingRoad") == "24" and c.get("connectingRoad") == "256" and c.get("contactPoint") == "end" and any(l.attrib == {"from":"1","to":"1"} for l in c.findall("laneLink"))]
    require(len(matches) == 1, "unique original junction255 lane connection required")
    start,goal = endpoint_s(roads[0],original["route"][0]),endpoint_s(roads[-1],original["route"][-1])
    spans = [(start,0.),(roads[1]["length"],0.),(0.,goal)]
    points, seams = [], []
    for j,(road,(first,last)) in enumerate(zip(roads,spans)):
        for s in lattice(road,first,last):
            p = centre_pose(road,s)
            if points:
                distance = math.dist([points[-1][k] for k in ("x","y","z")],[p[k] for k in ("x","y","z")])
                if points[-1]["road_id"] != road["road_id"]:
                    angle = abs(wrap(p["yaw"]-points[-1]["yaw"]))
                    require(distance <= .005 and angle <= .001, "whole-corridor lane-centre seam discontinuity")
                    position_roundoff = 64*max(math.ulp(v) for v in [1.,*[points[-1][k] for k in ("x","y","z")],*[p[k] for k in ("x","y","z")]])
                    yaw_roundoff = 64*max(math.ulp(1.),math.ulp(p["yaw"]),math.ulp(points[-1]["yaw"]))
                    if distance <= position_roundoff and angle <= yaw_roundoff:
                        # HH_261002 - A shared topological endpoint is one geometric pose, not a short segment. Retain both exact analytic source poses/IDs and authorize the unchanged interval's two lanes.
                        require(j > 0 and points[-1]["xodr_s_m"] == spans[j-1][1] and s == first, "only consecutive full-lane endpoints may share a geometric node")
                        previous = points[-1]
                        memberships = [dict(road_id=previous["road_id"],section_id=0,lane_id=previous["lane_id"],xodr_s_m=previous["xodr_s_m"],pose={k:previous[k] for k in ("x","y","z","yaw")},road_option=previous["road_option"]),dict(road_id=road["road_id"],section_id=0,lane_id=road["lane_id"],xodr_s_m=s,pose=dict(p),road_option="RIGHT" if j == 1 else "LANEFOLLOW")]
                        previous.update(road_id=road["road_id"],lane_id=road["lane_id"],is_junction=road["junction"],xodr_s_m=s,road_option="RIGHT" if j == 1 else "LANEFOLLOW",road_option_value=2 if j == 1 else 4,vad_command=1 if j == 1 else 3,shared_topology_endpoint_memberships=memberships)
                        seams.append(dict(from_index=len(points)-1,to_index=len(points)-1,kind="ONE_POSE_BOTH_TOPOLOGY_ENDPOINTS",source_endpoint_gap_m=distance,source_endpoint_heading_gap_rad=angle,position_roundoff_bound_m=position_roundoff,yaw_roundoff_bound_rad=yaw_roundoff,exact_upstream_pose_retained=True,memberships=memberships))
                        continue
                    seams.append(dict(from_index=len(points)-1,to_index=len(points),distance_m=distance,heading_difference_rad=angle))
                require(distance > 1e-4, "encoded seam/tiny segment cannot be assessed by unchanged nominal geometry; noncoincident tiny geometry is never merged")
                cumulative = points[-1]["distance_m"]+distance
            else: cumulative = 0.
            points.append(dict(index=len(points),**p,distance_m=cumulative,road_option="RIGHT" if j == 1 else "LANEFOLLOW",road_option_value=2 if j == 1 else 4,vad_command=1 if j == 1 else 3,road_id=road["road_id"],section_id=0,lane_id=road["lane_id"],is_junction=road["junction"],xodr_s_m=s))
    total = points[-1]["distance_m"]
    for p in points: p["remaining_m"] = total-p["distance_m"]
    endpoints = {}
    for label,new,old in (("start",points[0],original["route"][0]),("goal",points[-1],original["route"][-1])):
        endpoints[label] = dict(original_encoded_pose={k:old[k] for k in ("x","y","z","yaw")},new_centerline_pose={k:new[k] for k in ("x","y","z","yaw")},shift_ros_xyz_m={k:new[k]-old[k] for k in ("x","y","z")},shift_xy_norm_m=math.hypot(new["x"]-old["x"],new["y"]-old["y"]),source_longitudinal_projection_s_m=new["xodr_s_m"],original_spawn_transform_unchanged=copy.deepcopy(original.get(label+"_carla_transform")))
    return dict(schema_version=1,route_id=CASE_ID,coordinate_reference="base_link",spawn_point_reference="base_link",town="Town01",weather="ClearNoon",scenario="right",sampling_resolution_m=RESOLUTION,route_length_m=total,route=points,option_counts=dict(Counter(p["road_option"] for p in points)),start_ros_pose=endpoints["start"]["new_centerline_pose"],goal_ros_pose=endpoints["goal"]["new_centerline_pose"],spawn_point=",".join(format(v,".17g") for v in (points[0]["x"],-points[0]["y"],points[0]["z"]+.3,0.,0.,-math.degrees(points[0]["yaw"]))),admission_status="PENDING_QA",training_admission_state="PENDING_QA_NOT_AUTHORIZED",generation_provenance=dict(schema=SCHEMA+".generation",contract=generation_contract(),original_case="town01_right_s0003_p00",original_route_sha256=nominal.INPUTS[nominal.ROUTE],original_encoded_node_count=len(original["route"]),original_coordinates_modified=False,original_goal_endpoint_provenance=copy.deepcopy(original.get("goal_endpoint_provenance")),source_XODR_sha256=hashlib.sha256(xml).hexdigest(),endpoint_projection=endpoints,seams=seams,reference_s_spans=[[r["road_id"],r["lane_id"],a,b] for r,(a,b) in zip(roads,spans)],coordinate_source="Exact XODR line/arc + constant lane-centre offset; ROS XY is XODR XY, positive lane travels reverse, CARLA Y/yaw negated only for nominal actor geometry.",new_route_independent_of_original_spawn_goal_transform=True,original_failed_route_not_reclassified=True))


def validate_origin(data):
    catalog,plan,owner = [json.loads(data[str(BASE/name)]) for name in ("route_catalog.json","owner_plan.json","owner_result.json")]
    require(catalog["status"] == "complete" and catalog["canonical_name"] == "Town01" and plan["split_intent"] == "TRAIN" and owner["exit_code"] == 0 and owner["catalog_sha256"] == INPUTS[BASE/"route_catalog.json"], "original complete TRAIN catalogue provenance differs")
    entry = [e for e in catalog["routes"] if e["id"] == "town01_right_s0003_p00"]
    require(len(entry) == 1 and entry[0]["sha256"] == nominal.INPUTS[nominal.ROUTE] and nominal.canonical(BASE/entry[0]["path"]) == nominal.ROUTE, "original candidate is not the exact catalogue member")
    history = json.loads(data[str(HISTORY)])
    require(history["case_count"] == 32 and len(history["results"]) == 32 and history["outcome_counts"] == {"NOMINAL_UNSUPPORTED_NOT_PROVEN":25,"NOMINAL_BODY_FAIL_RETAINED":6,"NOMINAL_CLEAR_NOT_DRIVING_PROOF":1}, "all32 original outcomes required")
    old = [r for r in history["results"] if r["case_id"] == "town01_right_s0003_p00"]
    require(len(old) == 1 and old[0]["source_route_sha256"] == nominal.INPUTS[nominal.ROUTE] and old[0]["status"] == "NOMINAL_UNSUPPORTED_NOT_PROVEN", "original failed p00 outcome must remain failed")
    require(site_guard.check("Town01","drive","reuse_existing_train",json.loads(data[str(REGISTRY)])), "site guard denies existing TRAIN corridor")
    return history


def run_case(original, manifest, xml, carla):
    route,rows = None,[]
    result = dict(case_id=CASE_ID,status="GENERATION_NOT_PROVEN",generation_contract=generation_contract(),geometry=None)
    try:
        route = generate(xml,original)
        rows,census,groups = nominal.probe(manifest,route,xml,carla)
        require(rows and all(c["assessed_count"] == len(rows)-(1 if k.startswith("previous_sweep") else 0) for k,c in census.items()) and all(len(r["all_eight_box_vertices_carla_xyz"]) == 8 for r in rows), "whole nominal body/sweep denominator incomplete")
        clear = all(not c["violation_sample_indices"] for c in census.values())
        result.update(status="NOMINAL_CLEAR_NOT_DRIVING_PROOF" if clear else "NOMINAL_BODY_FAIL_RETAINED",geometry=census,geometry_source_index_groups=groups,nominal_sample_count=len(rows),nominal_sweep_count=len(rows)-1)
    except (nominal.geometry.body.Unsupported,ValueError) as error:
        result.update(status="GENERATION_OR_NOMINAL_UNSUPPORTED_NOT_PROVEN",error_type=type(error).__name__,error=str(error))
    return result,route,rows


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--caller-pins",type=Path);parser.add_argument("--caller-pins-sha256")
    parser.add_argument("--print-contract",action="store_true");parser.add_argument("--validate-only",action="store_true")
    args = parser.parse_args(argv); sources = source_pins()
    contract = dict(schema=SCHEMA+".caller_pins",nominal_only=True,generation_contract=generation_contract(),input_sha256={str(p):s for p,s in INPUTS.items()},source_sha256=sources,carla_egg_sha256="6eb690ef4304b34a919b27a23129273029008c8e95a5f164d52a1bd494a96c3a",output=str(OUTPUT))
    if args.print_contract:
        require(args.caller_pins is None and args.caller_pins_sha256 is None and not args.validate_only,"contract-only mode cannot execute")
        print(json.dumps(contract,indent=2,allow_nan=False));return 0
    require(args.caller_pins is not None and args.caller_pins_sha256 is not None,"root external caller path+SHA required before generation/API")
    pins = {};caller = json.loads(nominal.payload(args.caller_pins,args.caller_pins_sha256,pins))
    require(all(caller.get(k) == v for k,v in contract.items()),"exact prospective generation/source/input contract differs")
    output = nominal.canonical(caller["output"]); require(not output.exists(),"fresh new catalogue output required")
    for name,sha in sources.items():nominal.payload(ROOT/name,sha,pins)
    data = {str(p):nominal.payload(p,sha,pins) for p,sha in INPUTS.items()}
    nominal.payload(nominal.EGG,caller["carla_egg_sha256"],pins); history = validate_origin(data)
    if args.validate_only:
        require(all(nominal.geometry.sha(nominal.canonical(p)) == s for p,s in pins.items()),"validate source/input/API changed")
        print(json.dumps(dict(status="VALIDATED_ONE_NEW_CANDIDATE_NO_GENERATION_NO_MAP",source_count=len(sources),original_case_count=32,training_data_approved=False)));return 0
    require(sys.version_info[:2] == (3,10),"pinned offline CARLA Map binding requires Python3.10")
    saved = list(sys.path); route,rows = None,[]
    try:
        sys.path.insert(0,str(nominal.EGG));import carla
        require(str(carla.__file__).startswith(str(nominal.EGG)+"/"),"foreign offline CARLA provider")
        result,route,rows = run_case(json.loads(data[str(nominal.ROUTE)]),json.loads(data[str(nominal.REFERENCE)]),data[str(nominal.XODR)],carla)
    except Exception as error:
        result = dict(case_id=CASE_ID,status="RUNTIME_NAMESPACE_FAILURE_NOT_PROVEN",error_type=type(error).__name__,error=str(error),geometry=None)
    finally:sys.path[:] = saved
    require(all(nominal.geometry.sha(nominal.canonical(p)) == s for p,s in pins.items()),"first-read source/input/API changed before output")
    ancestor = output.parent
    while not ancestor.exists():ancestor = ancestor.parent
    require(shutil.disk_usage(ancestor).free >= 5*1024**3,"5GiB output reserve required")
    summary = dict(schema=SCHEMA,status=result["status"],prospective_new_case_count=1,results=[result],original_all32_case_outcomes=[dict(case_id=r["case_id"],status=r["status"],source_route_sha256=r["source_route_sha256"]) for r in history["results"]],input_sha256=pins,source_sha256=sources,reference_capture_known_body_failed=True,nominal_roll_pitch_deg=[0.,0.],scope="New geometry on the SAME previously exposed TRAIN corridor, not a new independent site. Entire generated route; no observed motion, teacher capture, TRAIN admission, DEV/TEST or vehicle approval.",source_route_coordinates_modified=False,split_registry_modified=False,simulator_started=False,capture_authorized=False,training_data_approved=False,vehicle_control_approved=False)
    encoded_route = json.dumps(route,indent=2,allow_nan=False)+"\n" if route is not None else None
    relative = f"routes/right/{CASE_ID}.json" if route is not None else None
    if encoded_route:summary["generated_route_sha256"] = hashlib.sha256(encoded_route.encode()).hexdigest()
    output.mkdir(parents=True,exist_ok=False)
    if encoded_route:
        target = output/relative;target.parent.mkdir(parents=True,exist_ok=False)
        with target.open("x") as stream:stream.write(encoded_route)
    catalog = dict(schema=SCHEMA+".catalog",status="ONE_PROSPECTIVE_CANDIDATE_OUTCOME_RETAINED",split_intent="TRAIN_CANDIDATE",generation_contract=generation_contract(),routes=[dict(id=CASE_ID,path=relative,sha256=summary.get("generated_route_sha256"),status=result["status"])],training_data_approved=False,capture_authorized=False)
    with (output/"route_catalog.json").open("x") as stream:stream.write(json.dumps(catalog,indent=2,allow_nan=False)+"\n")
    with gzip.open(output/"all_nominal_bodies_and_sweeps.jsonl.gz","xt") as stream:
        for row in rows:stream.write(json.dumps(row,allow_nan=False)+"\n")
    with (output/"summary.json").open("x") as stream:stream.write(json.dumps(summary,indent=2,allow_nan=False)+"\n")
    require(all(nominal.geometry.sha(nominal.canonical(p)) == s for p,s in pins.items()),"first-read source/input/API changed after output")
    print(json.dumps(dict(status=result["status"],prospective_new_case_count=1,original_case_count=32,generated_route_sha256=summary.get("generated_route_sha256"),nominal_sample_count=len(rows),training_data_approved=False)))
    return 0 if result["status"] == "NOMINAL_CLEAR_NOT_DRIVING_PROOF" else 2


if __name__ == "__main__":raise SystemExit(main())
