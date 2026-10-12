"""HH_261002 - One new gentle TRAIN topology; reuse frozen nominal gates, not driving/admission proof."""
from __future__ import annotations

import ast
from collections import Counter
import copy
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from scripts.e2e import generate_town01_train_right_centerline_nominal_20261002 as previous

ROOT, nominal, require = previous.ROOT, previous.nominal, previous.require
SCHEMA = "portable_e2e.town01_train_right_gentle_centerline_nominal.20261002.v1"
CASE_ID = "town01_right_centerline_6p1_108p1_24p1_20261002_v1"
ORIGINAL_CASE = "town01_right_s0005_p01"
BASE = ROOT/"storage/experiments/training/2026-09-27/drive_long_route_town01_train_catalog_v5"
SOURCE_ROUTE = BASE/f"routes/town01/right/{ORIGINAL_CASE}.json"
SOURCE_ROUTE_SHA = "c02e2c6de6b2d7b236acdcca1a2f08d541278e88f7727a2874454949779f01d0"
OUTPUT = ROOT/"storage/experiments/training/2026-10-02/town01_train_right_gentle_centerline_catalog_v1"
PRIOR_SUMMARY = previous.OUTPUT/"summary.json"
PRIOR_CATALOG = previous.OUTPUT/"route_catalog.json"
PRIOR_ROUTE = previous.OUTPUT/f"routes/right/{previous.CASE_ID}.json"
CORRIDOR = ((6,0,1),(108,0,1),(24,0,1))
ORIGINAL_GROUPS = ((6,0,1),(93,0,1),(108,0,1),(24,0,1))
PREVIOUS_SHA = "16a27f69ff5d4cf3432923f5a6d4476501dc5973a24014e60a9cc1b587ba99b0"
FROZEN = {**previous.FROZEN,"tools/e2e/generate_town01_train_right_centerline_nominal_20261002.py":PREVIOUS_SHA}
INPUTS = {**previous.INPUTS,SOURCE_ROUTE:SOURCE_ROUTE_SHA,
    BASE/"route_catalog.json":"8155365f2843ef3d6ecd12a6eef81727b9937340fdf1890371a4555c6d72d7d8",
    BASE/"owner_plan.json":"3a83ff92d59040201091c9c816e41d9eef756dc9cd388c6dec3578c60e092813",
    BASE/"owner_result.json":"438363a5ebf67b3c2390864ba4365a0261aeaf6201c9224e024982f91abe270b",
    PRIOR_SUMMARY:"5f460289e50a5e33f4a7cd3589314316550a9e51ec173a967cc74bcae656a3f9",
    PRIOR_CATALOG:"c16d30d9bb83e801252ba3d379ac9840f177e6018f4d78f8125fd719b865f478",
    PRIOR_ROUTE:"3df189029e24177e5e08b586adbaf3bc7ee54653b936df4e4368b9304b2ca339"}
# HH_261002 - All continuous geometry/lattice formulas and whole-body probes are the exact frozen functions.
number, road_model, reference_pose, centre_pose, lattice, wrap = (previous.number, previous.road_model,
    previous.reference_pose, previous.centre_pose, previous.lattice, previous.wrap)


def generation_contract():
    result = previous.generation_contract()
    result.pop("original_228_nodes_and_failed_outcome_retained")
    result.update(case_ids=[CASE_ID],original_case=ORIGINAL_CASE,lane_triples=[list(t) for t in CORRIDOR],
        original_encoded_lane_groups=[list(t) for t in ORIGINAL_GROUPS],original_full_nodes_and_topology_retained=True,
        original_road93_duplicate_not_reclassified=True,previous_all32_and_generated_p00_FAILURE_retained=True,
        endpoint_policy="Select the unique interior LINE planView projection of each original full first/last encoded XY; preserve longitudinal extents, never clamp or shorten.",
        sampling_policy="Frozen fixed1m reference-s lattice, every geometry breakpoint and all full spans; only adjacent topology endpoints whose XY coincide within64ULP share one upstream pose, with BOTH original headings retained under unchanged .001rad seam; no adaptive spacing, clipping or retrace.",
        shared_endpoint_representation=dict(maximum_position_binary64_ULPs=64,only_adjacent_topology_endpoints=True,
            heading_seam_max_rad=.001,heading_roundoff_equivalence_claimed=False,exact_upstream_pose_retained=True,
            downstream_lane_and_command_primary=True,both_original_endpoint_poses_and_lane_s_memberships_retained=True,
            tiny_noncoincident_segments_remain_rejected_by_original_1e_minus4_guard=True),
        genuine_XODR_topology="junction87 connection6: road6 lane+1 -> reverse road108 lane+1 -> reverse road24 lane+1",
        original_encoded_road93_is_provenance_not_new_navigation=True,intended_teacher_seconds=70,
        teacher_protocol_implemented=False,source_formulas_reused_unchanged=True)
    return result


def endpoint_s(road, source):
    # HH_261002 - Road6 has several records. Only its unique interior LINE projection is admissible; no nearest-point clamp.
    candidates = []
    for g in road["geometries"]:
        if g["curvature"] == 0.:
            ds = (number(source["x"])-g["x"])*math.cos(g["hdg"])+(number(source["y"])-g["y"])*math.sin(g["hdg"])
            if 0. < ds < g["length"]:candidates.append(g["s"]+ds)
    require(len(candidates) == 1 and 0. < candidates[0] < road["length"], "unique interior LINE endpoint projection required; never clamp")
    return candidates[0]


def assemble(roads, spans):
    points, seams = [], []
    for j,(road,(first,last)) in enumerate(zip(roads,spans)):
        for s in lattice(road,first,last):
            p = centre_pose(road,s); option = "RIGHT" if j == 1 else "LANEFOLLOW"
            membership = dict(road_id=road["road_id"],section_id=0,lane_id=road["lane_id"],xodr_s_m=s,pose=dict(p),road_option=option)
            if points:
                distance = math.dist([points[-1][k] for k in ("x","y","z")],[p[k] for k in ("x","y","z")])
                if points[-1]["road_id"] != road["road_id"]:
                    angle = abs(wrap(p["yaw"]-points[-1]["yaw"]))
                    require(distance <= .005 and angle <= .001, "whole-corridor lane-centre seam discontinuity")
                    bound = 64*max(math.ulp(v) for v in [1.,*[points[-1][k] for k in ("x","y","z")],*[p[k] for k in ("x","y","z")]])
                    if distance <= bound:
                        # HH_261002 - A NEW one-position/two-source-headings ABI; this is not a change to the frozen p00 yaw-roundoff predicate.
                        require(j > 0 and points[-1]["xodr_s_m"] == spans[j-1][1] and s == first, "only consecutive full-lane endpoints may share a geometric node")
                        old = points[-1]; members = [dict(road_id=old["road_id"],section_id=0,lane_id=old["lane_id"],xodr_s_m=old["xodr_s_m"],pose={k:old[k] for k in ("x","y","z","yaw")},road_option=old["road_option"]),membership]
                        old.update(road_id=road["road_id"],lane_id=road["lane_id"],is_junction=road["junction"],xodr_s_m=s,
                            road_option=option,road_option_value=2 if j == 1 else 4,vad_command=1 if j == 1 else 3,shared_topology_endpoint_memberships=members)
                        seams.append(dict(from_index=len(points)-1,to_index=len(points)-1,kind="ONE_POSITION_TWO_ORIGINAL_SOURCE_HEADINGS",
                            source_endpoint_gap_m=distance,source_endpoint_heading_gap_rad=angle,position_roundoff_bound_m=bound,
                            heading_seam_bound_rad=.001,heading_roundoff_equivalence_claimed=False,exact_upstream_pose_retained=True,memberships=members))
                        continue
                    seams.append(dict(from_index=len(points)-1,to_index=len(points),distance_m=distance,heading_difference_rad=angle))
                require(distance > 1e-4, "encoded seam/tiny segment cannot be assessed by unchanged nominal geometry; noncoincident tiny geometry is never merged")
                arc = points[-1]["distance_m"]+distance
            else:arc = 0.
            points.append(dict(index=len(points),**p,distance_m=arc,road_option=option,road_option_value=2 if j == 1 else 4,
                vad_command=1 if j == 1 else 3,road_id=road["road_id"],section_id=0,lane_id=road["lane_id"],is_junction=road["junction"],xodr_s_m=s))
    for p in points:p["remaining_m"] = points[-1]["distance_m"]-p["distance_m"]
    return points,seams


def generate(xml, original):
    require(original.get("town") == "Town01" and original.get("weather") == "ClearNoon" and original.get("scenario") == "right"
        and original.get("coordinate_reference") == "base_link", "only exact original TRAIN Town01 right metadata supported")
    observed = [nominal.geometry.body._triple(p) for p in original["route"]]
    groups = tuple(t for i,t in enumerate(observed) if i == 0 or observed[i-1] != t)
    require(groups == ORIGINAL_GROUPS, "original full topology/provenance changed")
    duplicate = [i for i,t in enumerate(observed) if t == (93,0,1)]
    require(len(duplicate) == 1 and duplicate[0]+1 < len(observed) and observed[duplicate[0]+1] == (108,0,1)
        and all(original["route"][duplicate[0]][k] == original["route"][duplicate[0]+1][k] for k in ("x","y","z")), "exact original road93/108 encoded duplicate must remain in provenance")
    root = ET.fromstring(xml); roads = [road_model(root.find(f"road[@id='{r}']"),lane) for r,_,lane in CORRIDOR]
    middle = root.find("road[@id='108']");lane = middle.find("lanes/laneSection/left/lane[@id='1']")
    require(middle.find("link/predecessor").attrib == dict(elementType="road",elementId="24",contactPoint="end")
        and middle.find("link/successor").attrib == dict(elementType="road",elementId="6",contactPoint="start")
        and lane.find("link/predecessor").attrib == {"id":"1"} and lane.find("link/successor").attrib == {"id":"1"}, "exact genuine reverse road108 lane topology required")
    matches = [c for c in root.findall("junction[@id='87']/connection") if c.get("id") == "6" and c.get("incomingRoad") == "6"
        and c.get("connectingRoad") == "108" and c.get("contactPoint") == "end" and [l.attrib for l in c.findall("laneLink")] == [{"from":"1","to":"1"}]]
    require(len(matches) == 1, "unique genuine junction87 connection6 required")
    spans = [(endpoint_s(roads[0],original["route"][0]),0.),(roads[1]["length"],0.),(roads[2]["length"],endpoint_s(roads[2],original["route"][-1]))]
    points,seams = assemble(roads,spans); endpoints = {}
    for label,new,old in (("start",points[0],original["route"][0]),("goal",points[-1],original["route"][-1])):
        endpoints[label] = dict(original_encoded_pose={k:old[k] for k in ("x","y","z","yaw")},new_centerline_pose={k:new[k] for k in ("x","y","z","yaw")},
            shift_ros_xyz_m={k:new[k]-old[k] for k in ("x","y","z")},source_longitudinal_projection_s_m=new["xodr_s_m"],original_spawn_transform_unchanged=copy.deepcopy(original.get(label+"_carla_transform")))
    return dict(schema_version=1,route_id=CASE_ID,coordinate_reference="base_link",spawn_point_reference="base_link",town="Town01",weather="ClearNoon",scenario="right",
        sampling_resolution_m=previous.RESOLUTION,route_length_m=points[-1]["distance_m"],route=points,option_counts=dict(Counter(p["road_option"] for p in points)),
        start_ros_pose=endpoints["start"]["new_centerline_pose"],goal_ros_pose=endpoints["goal"]["new_centerline_pose"],
        spawn_point=",".join(format(v,".17g") for v in (points[0]["x"],-points[0]["y"],points[0]["z"]+.3,0.,0.,-math.degrees(points[0]["yaw"]))),
        admission_status="PENDING_QA",training_admission_state="PENDING_QA_NOT_AUTHORIZED",generation_provenance=dict(schema=SCHEMA+".generation",contract=generation_contract(),
            original_case=ORIGINAL_CASE,original_route_sha256=SOURCE_ROUTE_SHA,original_encoded_node_count=len(original["route"]),original_encoded_route=copy.deepcopy(original["route"]),
            original_encoded_topology_groups=[list(t) for t in groups],original_road93_duplicate_source_indices=[duplicate[0],duplicate[0]+1],original_coordinates_modified=False,
            original_goal_endpoint_provenance=copy.deepcopy(original.get("goal_endpoint_provenance")),source_XODR_sha256=hashlib.sha256(xml).hexdigest(),
            endpoint_projection=endpoints,seams=seams,reference_s_spans=[[r["road_id"],r["lane_id"],a,b] for r,(a,b) in zip(roads,spans)],
            coordinate_source="Frozen XODR line/arc lane-centre equations: XODR XY is ROS XY; positive lanes reverse; CARLA Y/yaw negated only in frozen nominal actor geometry.",
            original_candidate_not_reclassified=True,new_route_independent_of_original_spawn_goal_transform=True))


def validate_origin(data):
    history = copy.deepcopy(previous.validate_origin(data))
    catalog,plan,owner = [json.loads(data[str(BASE/name)]) for name in ("route_catalog.json","owner_plan.json","owner_result.json")]
    require(catalog["status"] == "complete" and catalog["canonical_name"] == "Town01" and plan["split_intent"] == "TRAIN_CANDIDATE"
        and type(owner["exit_code"]) is int and owner["exit_code"] == 0 and owner["catalog_sha256"] == INPUTS[BASE/"route_catalog.json"], "original complete v5 TRAIN-candidate provenance differs")
    entries = [r for r in catalog["routes"] if r["id"] == ORIGINAL_CASE]
    require(len(entries) == 1 and entries[0]["sha256"] == SOURCE_ROUTE_SHA and nominal.canonical(BASE/entries[0]["path"]) == SOURCE_ROUTE, "exact original v5 candidate catalogue member required")
    prior,prior_catalog = json.loads(data[str(PRIOR_SUMMARY)]),json.loads(data[str(PRIOR_CATALOG)])
    old_outcomes = [dict(case_id=r["case_id"],status=r["status"],source_route_sha256=r["source_route_sha256"]) for r in history["results"]]
    require(prior["schema"] == previous.SCHEMA and prior["status"] == "NOMINAL_BODY_FAIL_RETAINED" and prior["prospective_new_case_count"] == 1
        and prior["generated_route_sha256"] == INPUTS[PRIOR_ROUTE] and prior["original_all32_case_outcomes"] == old_outcomes
        and len(prior["results"]) == 1 and prior["results"][0]["case_id"] == previous.CASE_ID
        and prior["results"][0]["status"] == "NOMINAL_BODY_FAIL_RETAINED"
        and prior_catalog["routes"] == [dict(id=previous.CASE_ID,path=f"routes/right/{previous.CASE_ID}.json",sha256=INPUTS[PRIOR_ROUTE],status="NOMINAL_BODY_FAIL_RETAINED")], "all original32 and new p00 body/sweep failure must remain visible")
    history["previous_generated_candidates_retained"] = [dict(source_summary_sha256=INPUTS[PRIOR_SUMMARY],source_generated_route_sha256=INPUTS[PRIOR_ROUTE],**copy.deepcopy(prior["results"][0]))]
    return history


def provider():
    # HH_261002 - Reuse the frozen source/read/create-only/namespace machinery privately. Only one additive outcome-table keyword changes its AST.
    raw = Path(previous.__file__).read_bytes();require(hashlib.sha256(raw).hexdigest() == PREVIOUS_SHA, "frozen p00 generator changed")
    selected = [copy.deepcopy(n) for n in ast.parse(raw).body if isinstance(n,ast.FunctionDef) and n.name in ("source_pins","run_case","main")]
    require([n.name for n in selected] == ["source_pins","run_case","main"], "exact frozen source/main/probe APIs required")
    summary = [n for n in ast.walk(selected[-1]) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id == "summary" for t in n.targets)]
    require(len(summary) == 1 and isinstance(summary[0].value,ast.Call), "exact parent outcome constructor required")
    summary[0].value.keywords.append(ast.keyword(arg="previous_generated_candidates_retained",value=ast.Subscript(value=ast.Name(id="history",ctx=ast.Load()),slice=ast.Constant(value="previous_generated_candidates_retained"),ctx=ast.Load())))
    private_nominal = type("NominalNamespace",(),{})()
    private_nominal.__dict__.update(vars(nominal));private_nominal.ROUTE = SOURCE_ROUTE
    namespace = dict(vars(previous),__file__=__file__,ROOT=ROOT,SCHEMA=SCHEMA,CASE_ID=CASE_ID,OUTPUT=OUTPUT,FROZEN=FROZEN,INPUTS=INPUTS,
        nominal=private_nominal,generation_contract=generation_contract,generate=generate,validate_origin=validate_origin)
    exec(compile(ast.fix_missing_locations(ast.Module(body=selected,type_ignores=[])),__file__,"exec"),namespace)
    return namespace


def source_pins():return provider()["source_pins"]()
def run_case(*args):return provider()["run_case"](*args)
def main(argv=None):return provider()["main"](argv)


if __name__ == "__main__":raise SystemExit(main())
