"""HH_261002 - Synthetic nominal geometry and pin tests never open a simulator or real capture."""

import copy
import hashlib
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
from shapely.geometry import box
from shapely.ops import unary_union

from scripts.e2e import audit_town01_train_curve_nominal_20261002 as audit


def route():
    points = [dict(index=i,x=float(i),y=0.,z=0.,yaw=0.,distance_m=float(i),road_id=1,section_id=0,lane_id=1,
        road_option="LANEFOLLOW",road_option_value=4,vad_command=3,remaining_m=2.-i) for i in range(3)]
    return dict(coordinate_reference="base_link",route=points,route_length_m=2.)


def manifest(xml=b"map"):
    return dict(coordinate_contract=dict(wheelbase_m=2.85),capture_contract=dict(
        actor_body_geometry=dict(status="RECORDED",source="carla.Vehicle.bounding_box",actor_type_id="vehicle.toyota.prius",
            reference_frame="CARLA actor-local; not ROS rear-axle base_link",actor_id=174),
        agent_initialization=dict(construction=dict(status="PASS",before=dict(actor_id=174),after=dict(actor_id=174))),
        active_map_opendrive=dict(status="RECORDED",source="carla.Map.to_opendrive",map_name="Carla/Maps/Town01",
            sha256=hashlib.sha256(xml).hexdigest(),size_bytes=len(xml))))


def test_exact_command_duplicates_only_and_original_unchanged():
    data=route(); duplicate=copy.deepcopy(data["route"][1]); duplicate.update(index=99,road_option="RIGHT",road_option_value=2,vad_command=1)
    data["route"].insert(2,duplicate); original=copy.deepcopy(data)
    points,groups=audit.dedup_points(data)
    assert groups==[[0],[1,2],[3]] and len(points)==3 and data==original


@pytest.mark.parametrize("field,value",[("x",1.001),("yaw",.01),("lane_id",2),("remaining_m",.9)])
def test_changed_geometry_or_noncommand_metadata_duplicate_denied(field,value):
    data=route(); duplicate=copy.deepcopy(data["route"][1]); duplicate[field]=value; data["route"].insert(2,duplicate)
    with pytest.raises(ValueError,match="duplicate command"): audit.dedup_points(data)


@pytest.mark.parametrize("field,value",[("x",float("nan")),("yaw",True),("distance_m",float("inf")),("lane_id",True)])
def test_invalid_route_denied(field,value):
    data=route(); data["route"][1][field]=value
    with pytest.raises((ValueError,audit.geometry.body.Unsupported)): audit.dedup_points(data)


def test_reversed_distance_and_wrong_coordinate_denied():
    data=route();data["route"][2]["distance_m"]=-1
    with pytest.raises(ValueError,match="reverses"): audit.dedup_points(data)
    data=route(); data["coordinate_reference"]="actor_center"
    with pytest.raises(ValueError,match="rear-reference"): audit.dedup_points(data)


def test_full_sampling_and_exact_rear_to_actor_sign_and_no_observed_fields():
    points,_=audit.dedup_points(route()); rows=audit.nominal_samples(points,2.85)
    assert rows[0]["nominal_rear_arc_m"]==0 and rows[-1]["nominal_rear_arc_m"]==2
    assert len(rows)==19 and all(0<b["nominal_rear_arc_m"]-a["nominal_rear_arc_m"]<=.12 for a,b in zip(rows,rows[1:]))
    assert rows[0]["nominal_actor_pose_carla"]["x"]==1.425
    assert all("frame" not in r and "timestamp" not in r and "current_control" not in r for r in rows)
    points=route()["route"]
    for p in points: p.update(x=0.,y=p["distance_m"],yaw=math.pi/2)
    row=audit.nominal_samples(points,2.85)[0]
    assert row["nominal_actor_pose_carla"]["y"]==pytest.approx(-1.425)
    assert row["nominal_actor_pose_carla"]["yaw"]==pytest.approx(-90)
    assert row["nominal_actor_pose_carla"]["roll"]==row["nominal_actor_pose_carla"]["pitch"]==0


def test_wrapped_encoded_tangent_yaw_has_no_360_degree_sweep():
    points=route()["route"][:2];points[0]["yaw"]=math.radians(179);points[1]["yaw"]=math.radians(-179)
    yaws=[r["nominal_actor_pose_carla"]["yaw"] for r in audit.nominal_samples(points,2.85)]
    assert max(yaws)-min(yaws)==pytest.approx(2)


def fake_geometry(monkeypatch, narrow=False):
    calls=[]
    class FakeBox:
        def get_world_vertices(self,transform):
            c=transform.location
            return [SimpleNamespace(x=c.x+dx,y=c.y+dy,z=c.z+dz) for dx in (-1.,1.) for dy in (-.4,.4) for dz in (-.5,.5)]
    carla=SimpleNamespace(Map=lambda name,xml:calls.append(("offline_Map",name,xml)),Location=lambda **k:SimpleNamespace(**k),
        Rotation=lambda **k:SimpleNamespace(**k),Transform=lambda location,rotation:SimpleNamespace(location=location,rotation=rotation))
    monkeypatch.setattr(audit.geometry.body,"_body_box",lambda *a:(FakeBox(),1.2))
    monkeypatch.setattr(audit.geometry.body,"lane_corridors",lambda *a:{(1,0,1):box(-3,-(.3 if narrow else 2),6,(.3 if narrow else 2))})
    original=audit.geometry.body.authorized_lane_region
    def region(corridors,lanes,inset): calls.append(("inset",inset));return original(corridors,lanes,inset)
    monkeypatch.setattr(audit.geometry.body,"authorized_lane_region",region)
    monkeypatch.setattr(audit.geometry.body,"body_footprint",lambda bbox,p,c:box(p["x"]-1,p["y"]-.4,p["x"]+1,p["y"]+.4))
    def sweep(a,b,pa,pb,r): calls.append(("sweep",pa,pb,r));return unary_union((a,b)).convex_hull.buffer(.02)
    monkeypatch.setattr(audit.geometry.body,"swept_body",sweep)
    return carla,calls


@pytest.mark.parametrize("narrow",[False,True])
def test_probe_full_eight_corners_all_sweeps_original_region_math(monkeypatch,narrow):
    carla,calls=fake_geometry(monkeypatch,narrow); raw=route();before=copy.deepcopy(raw)
    rows,census,groups=audit.probe(manifest(),raw,b"map",carla)
    assert raw==before and groups==[[0],[1],[2]] and len(rows)==19
    assert len([c for c in calls if c[0]=="sweep"])==18
    assert all(len(r["all_eight_box_vertices_carla_xyz"])==8 for r in rows)
    assert rows[0]["independent_nominal_actor_arc_m"]==pytest.approx(1.425)
    assert rows[0]["previous_sweep"] is None and all(r["previous_sweep"] is not None for r in rows[1:])
    assert {c[1] for c in calls if c[0]=="inset"}=={0.,.1}
    assert all(c["assessed_count"]==(18 if k.startswith("previous") else 19) for k,c in census.items())
    assert all(bool(c["violation_sample_indices"])==narrow for c in census.values())
    if narrow: assert all(c["minimum_interior_clearance_m"] is None for c in census.values())


@pytest.mark.parametrize("fault",["actor","vehicle","wheelbase","map","xml"])
def test_actual_reference_geometry_provenance_denied(monkeypatch,fault):
    carla,_=fake_geometry(monkeypatch);m=manifest()
    if fault=="actor":m["capture_contract"]["agent_initialization"]["construction"]["after"]["actor_id"]=9
    if fault=="vehicle":m["capture_contract"]["actor_body_geometry"]["actor_type_id"]="vehicle.other"
    if fault=="wheelbase":m["coordinate_contract"]["wheelbase_m"]=2.9
    if fault=="map":m["capture_contract"]["active_map_opendrive"]["map_name"]="Town03"
    if fault=="xml":m["capture_contract"]["active_map_opendrive"]["sha256"]="0"*64
    with pytest.raises(ValueError):audit.probe(m,route(),b"map",carla)


def synthetic_caller(monkeypatch,tmp_path):
    root=tmp_path.resolve(); monkeypatch.setattr(audit,"ROOT",root)
    files={root/name:b"synthetic input" for name in ("route.json","manifest.json","map.xodr")}
    for p,data in files.items():p.write_bytes(data)
    inputs={p:hashlib.sha256(data).hexdigest() for p,data in files.items()};monkeypatch.setattr(audit,"INPUTS",inputs)
    egg=root/"api.egg";egg.write_bytes(b"fake API, never imported");monkeypatch.setattr(audit,"EGG",egg)
    source=root/"code.py";source.write_bytes(b"synthetic pinned source")
    sources={"code.py":hashlib.sha256(source.read_bytes()).hexdigest()};monkeypatch.setattr(audit,"source_pins",lambda:sources)
    output=root/"storage/experiments/diagnostics/2026-10-02/nominal"
    caller=root/"caller.json";caller.write_text(json.dumps(dict(schema=audit.SCHEMA+".caller_pins",nominal_only=True,input_sha256={str(p):s for p,s in inputs.items()},source_sha256=sources,output=str(output),carla_egg_sha256=hashlib.sha256(egg.read_bytes()).hexdigest())))
    return caller,hashlib.sha256(caller.read_bytes()).hexdigest(),output,source


def test_validate_only_no_API_geometry_capture_or_output(monkeypatch,tmp_path):
    caller,sha,output,_=synthetic_caller(monkeypatch,tmp_path)
    monkeypatch.setattr(audit,"probe",lambda *a:pytest.fail("validate-only must not call geometry"))
    assert audit.main(["--caller-pins",str(caller),"--caller-pins-sha256",sha,"--validate-only"])==0
    assert not output.exists()


@pytest.mark.parametrize("fault",["caller_sha","input_sha","source_sha","existing","outside","symlink"])
def test_fail_closed_before_output_creation(monkeypatch,tmp_path,fault):
    caller,sha,output,source=synthetic_caller(monkeypatch,tmp_path);c=json.loads(caller.read_bytes())
    if fault=="caller_sha":sha="0"*64
    if fault=="input_sha":c["input_sha256"][next(iter(c["input_sha256"]))]="0"*64
    if fault=="source_sha":source.write_bytes(b"mutation")
    if fault=="existing":output.mkdir(parents=True)
    if fault=="outside":c["output"]=str(tmp_path/"outside")
    if fault=="symlink":
        actual=tmp_path/"actual";actual.mkdir();link=tmp_path/"link";link.symlink_to(actual,target_is_directory=True);c["output"]=str(link/"nominal")
    if fault not in ("caller_sha","source_sha","existing"):
        caller.write_text(json.dumps(c));sha=hashlib.sha256(caller.read_bytes()).hexdigest()
    with pytest.raises(ValueError):audit.main(["--caller-pins",str(caller),"--caller-pins-sha256",sha,"--validate-only"])
    if fault!="existing":assert not output.exists()


def test_import_and_probe_source_have_no_live_client_or_actuator_APIs():
    import ast
    tree=ast.parse(Path(audit.__file__).read_bytes())
    forbidden={"Client","apply_control","try_spawn_actor","spawn_actor","get_world","tick","load_world"}
    assert not [n for n in ast.walk(tree) if isinstance(n,ast.Attribute) and n.attr in forbidden]
