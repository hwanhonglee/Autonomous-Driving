"""HH_261002 - Synthetic full-catalogue tests never execute CARLA Map or open real driving data."""

import ast
import copy
import gzip
import hashlib
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

from scripts.e2e import audit_town01_train_turn_nominal_census_20261002 as audit


def route(entry):
    points = [dict(index=i,x=float(i),y=0.,z=0.,yaw=0.,distance_m=float(i),road_id=1,section_id=0,lane_id=1,road_option="LANEFOLLOW",road_option_value=4,vad_command=3) for i in range(3)]
    return dict(town="Town01",scenario=entry["scenario"],coordinate_reference="base_link",route=points,route_length_m=2.,goal_endpoint_provenance=dict(endpoint_source="spawn_points",endpoint_index=7,original_goal_ros_pose=dict(x=2.,y=.095,z=.3,yaw=0.)),physical_turn_preflight=dict(status="PASS",old_static_shape_only=True))


def entries():
    return [dict(id=i,status="ready",scenario="left" if "_left_" in i else "right",path=f"routes/town01/{'left' if '_left_' in i else 'right'}/{i}.json",sha256=hashlib.sha256(i.encode()).hexdigest()) for i in audit.CASE_IDS]


def synthetic_probe(monkeypatch):
    calls = []
    def probe(manifest,data,xml,carla):
        calls.append(data)
        i = len(calls)-1
        if i % 3 == 0:
            raise audit.nominal.geometry.body.Unsupported("recorded route point cannot be bound to its own XODR lane (12, 0, -1): nearest center 0.095m")
        bad = i % 3 == 1
        census = {name:dict(assessed_count=count,violation_sample_indices=[0] if bad else [],maximum_outside_m2=.01 if bad else 0.,minimum_interior_clearance_m=None if bad else .04) for name,count in (("sampled_body_raw_lane",2),("sampled_body_inset_10cm_lane",2),("previous_sweep_raw_lane",1),("previous_sweep_inset_10cm_lane",1))}
        return [dict(nominal_sample_index=0),dict(nominal_sample_index=1)],census,[[0],[1],[2]]
    monkeypatch.setattr(audit.nominal,"probe",probe)
    return calls


def test_all32_unchanged_routes_denominator_both_failures_and_clear_preserved(monkeypatch):
    cases=[(e,route(e)) for e in entries()]; original=copy.deepcopy(cases)
    flags={e["id"]:dict(sha256=e["sha256"],review_flags=["OLD_PROXIMITY_FLAG"],capture_authorized=False) for e in entries() if e["scenario"] == "right"}
    calls=synthetic_probe(monkeypatch)
    rows,bodies=audit.run_census({},cases,b"synthetic XML",object(),flags)
    assert len(rows)==len(calls)==32 and tuple(r["case_id"] for r in rows)==audit.CASE_IDS and cases==original
    assert sum(r["status"]=="NOMINAL_UNSUPPORTED_NOT_PROVEN" for r in rows)==11
    assert sum(r["status"]=="NOMINAL_BODY_FAIL_RETAINED" for r in rows)==11
    assert sum(r["status"]=="NOMINAL_CLEAR_NOT_DRIVING_PROOF" for r in rows)==10
    assert len(bodies)==21 and rows[0]["geometry"] is None and rows[0]["body_geometry_assessed"] is False
    assert rows[0]["original_goal_endpoint_provenance"]==original[0][1]["goal_endpoint_provenance"]
    assert all(r["training_data_approved"] is False and r["vehicle_control_approved"] is False and r["capture_authorized"] is False for r in rows)
    assert rows[0]["original_proximity_context_review"] is None
    assert rows[0]["original_proximity_context_review_status"]=="NO_EXISTING_LEFT_REVIEW_NOT_INFERRED_CLEAR"
    assert rows[16]["original_proximity_context_review"]["review_flags"]==["OLD_PROXIMITY_FLAG"]


def test_empty_nominal_denominator_is_not_clear(monkeypatch):
    monkeypatch.setattr(audit.nominal,"probe",lambda *a:([],{},[]))
    e=entries()[0]; rows,_=audit.run_census({},[(e,route(e))],b"map",object(),{})
    assert rows[0]["status"]=="NOMINAL_BODY_FAIL_RETAINED"


@pytest.mark.parametrize("exception",[RuntimeError("unexpected engine error"),KeyboardInterrupt()])
def test_unexpected_error_does_not_become_success_or_silent_candidate_drop(monkeypatch,exception):
    def fail(*a):raise exception
    monkeypatch.setattr(audit.nominal,"probe",fail)
    e=entries()[0]
    with pytest.raises(type(exception)):audit.run_census({},[(e,route(e))],b"map",object(),{})


@pytest.mark.parametrize("field,value",[("town","Town03"),("scenario","right")])
def test_dev_or_wrong_turn_route_rejected_before_probe(monkeypatch,field,value):
    monkeypatch.setattr(audit.nominal,"probe",lambda *a:pytest.fail("must reject wrong role first"))
    e=entries()[0];r=route(e);r[field]=value
    with pytest.raises(ValueError,match="map/scenario"):audit.run_census({},[(e,r)],b"map",object(),{})


def fixture_files(monkeypatch,tmp_path):
    root=tmp_path.resolve();base=root/"catalogue";base.mkdir()
    monkeypatch.setattr(audit,"ROOT",root);monkeypatch.setattr(audit,"BASE",base)
    monkeypatch.setattr(audit,"CATALOG",base/"route_catalog.json");monkeypatch.setattr(audit,"PLAN",base/"owner_plan.json");monkeypatch.setattr(audit,"OWNER",base/"owner_result.json")
    monkeypatch.setattr(audit,"OLD_FLAGS",root/"old_flags.json");monkeypatch.setattr(audit,"OUTPUT",root/"storage/experiments/diagnostics/2026-10-02/full32")
    def write(p,value):
        p.parent.mkdir(parents=True,exist_ok=True);data=value if isinstance(value,bytes) else json.dumps(value).encode();p.write_bytes(data);return hashlib.sha256(data).hexdigest()
    es=entries()
    for e in es:e["sha256"]=write(base/e["path"],route(e))
    catalogue=dict(status="complete",canonical_name="Town01",routes=es,generation=dict(scenarios=["left","right"],seeds=[2,3],pairs_per_seed=8,trace_coverage=dict(attempted=54833,accepted=32,skipped=54801)))
    csha=write(audit.CATALOG,catalogue);monkeypatch.setattr(audit,"CATALOG_SHA",csha)
    archived=base/"provenance/tools/generator.py";ash=write(archived,b"original generator bytes")
    plan=dict(split_intent="TRAIN",capture_executed=False,dataset_admission_granted=False,source_sha256={"tools/generator.py":ash})
    psha=write(audit.PLAN,plan);monkeypatch.setattr(audit,"PLAN_SHA",psha)
    owner=dict(catalog_sha256=csha,exit_code=0,source_bytes_unchanged_and_archived=True,dataset_admission_granted=False,source_checks={"tools/generator.py":True})
    osha=write(audit.OWNER,owner);monkeypatch.setattr(audit,"OWNER_SHA",osha)
    flags=dict(catalog_sha256=csha,owner_plan_sha256=psha,owner_result_sha256=osha,candidates=[dict(route_id=e["id"],sha256=e["sha256"],review_flags=["CONTEXT_FLAG"],training_data_approved=False) for e in es if e["scenario"]=="right"])
    fsha=write(audit.OLD_FLAGS,flags);monkeypatch.setattr(audit,"FLAGS_SHA",fsha)
    ref=root/"manifest.json";xml=root/"map.xodr";rsha=write(ref,{});xsha=write(xml,b"synthetic map")
    monkeypatch.setattr(audit.nominal,"REFERENCE",ref);monkeypatch.setattr(audit.nominal,"XODR",xml);monkeypatch.setattr(audit.nominal,"INPUTS",{ref:rsha,xml:xsha})
    egg=root/"api.egg";esha=write(egg,b"never import actual API");monkeypatch.setattr(audit.nominal,"EGG",egg)
    source=root/"code.py";ssha=write(source,b"synthetic source")
    sources={"code.py":ssha};monkeypatch.setattr(audit,"source_pins",lambda:sources)
    expected,_=audit.expected_inputs()
    caller=root/"caller.json";c=dict(schema=audit.SCHEMA+".caller_pins",nominal_only=True,case_ids=list(audit.CASE_IDS),input_sha256=expected,source_sha256=sources,output=str(audit.OUTPUT),carla_egg_sha256=esha)
    sha=write(caller,c)
    return caller,sha,es,write


def test_expected_inputs_all32_and_original_generator_archive(monkeypatch,tmp_path):
    fixture_files(monkeypatch,tmp_path);pins,c=audit.expected_inputs()
    assert len(c["routes"])==32 and len(pins)==39
    assert str(audit.BASE/"provenance/tools/generator.py") in pins
    assert tuple(e["id"] for e in c["routes"])==audit.CASE_IDS


def test_contract_and_validate_only_never_import_api_or_create_output(monkeypatch,tmp_path,capsys):
    caller,sha,_,_=fixture_files(monkeypatch,tmp_path)
    monkeypatch.setattr(audit,"run_census",lambda *a:pytest.fail("no geometry in preflight"))
    assert audit.main(["--print-contract"])==0
    contract=json.loads(capsys.readouterr().out);assert len(contract["case_ids"])==32
    assert audit.main(["--caller-pins",str(caller),"--caller-pins-sha256",sha,"--validate-only"])==0
    assert json.loads(capsys.readouterr().out)["status"]=="VALIDATED_ALL32_NO_GEOMETRY_NO_SIMULATOR"
    assert not audit.OUTPUT.exists()


@pytest.mark.parametrize("fault",["caller_sha","subset","reorder","source","route","archive","existing","output","extra_input","old_flags","symlink"])
def test_pin_or_set_mutation_fail_closed_before_outputs(monkeypatch,tmp_path,fault):
    caller,sha,es,write=fixture_files(monkeypatch,tmp_path);c=json.loads(caller.read_bytes())
    if fault=="caller_sha":sha="0"*64
    if fault=="subset":c["case_ids"]=c["case_ids"][:-1]
    if fault=="reorder":c["case_ids"].reverse()
    if fault=="source":(tmp_path/"code.py").write_bytes(b"mutation")
    if fault=="route":(audit.BASE/es[0]["path"]).write_bytes(b"changed coordinate")
    if fault=="archive":(audit.BASE/"provenance/tools/generator.py").write_bytes(b"changed original generator")
    if fault=="existing":audit.OUTPUT.mkdir(parents=True)
    if fault=="output":c["output"]=str(tmp_path/"unapproved-output")
    if fault=="extra_input":c["input_sha256"][str(tmp_path/"unknown")]= "0"*64
    if fault=="old_flags":audit.OLD_FLAGS.write_bytes(b"changed old context flag")
    if fault=="symlink":
        p=audit.BASE/es[0]["path"];data=p.read_bytes();other=tmp_path/"real-route.json";other.write_bytes(data);p.unlink();p.symlink_to(other)
    if fault in ("subset","reorder","output","extra_input"):sha=write(caller,c)
    with pytest.raises(ValueError):audit.main(["--caller-pins",str(caller),"--caller-pins-sha256",sha,"--validate-only"])
    assert not audit.OUTPUT.exists() or fault=="existing"


@pytest.mark.parametrize("fault",["missing","duplicate","wrong_role","capture_claim"])
def test_original_catalogue_membership_and_role_not_reinterpreted(monkeypatch,tmp_path,fault):
    fixture_files(monkeypatch,tmp_path)
    if fault in ("missing","duplicate"):
        c=json.loads(audit.CATALOG.read_bytes())
        if fault=="missing":c["routes"].pop()
        else:c["routes"][-1]=copy.deepcopy(c["routes"][0])
        audit.CATALOG.write_text(json.dumps(c));monkeypatch.setattr(audit,"CATALOG_SHA",hashlib.sha256(audit.CATALOG.read_bytes()).hexdigest())
        owner=json.loads(audit.OWNER.read_bytes());owner["catalog_sha256"]=audit.CATALOG_SHA;audit.OWNER.write_text(json.dumps(owner));monkeypatch.setattr(audit,"OWNER_SHA",hashlib.sha256(audit.OWNER.read_bytes()).hexdigest())
    else:
        p=json.loads(audit.PLAN.read_bytes());p["split_intent" if fault=="wrong_role" else "capture_executed"]="DEV" if fault=="wrong_role" else True
        audit.PLAN.write_text(json.dumps(p));monkeypatch.setattr(audit,"PLAN_SHA",hashlib.sha256(audit.PLAN.read_bytes()).hexdigest())
    with pytest.raises(ValueError):audit.expected_inputs()


def fake_api(monkeypatch):
    monkeypatch.setattr(audit.sys,"version_info",(3,10))
    monkeypatch.setitem(sys.modules,"carla",SimpleNamespace(__file__=str(audit.nominal.EGG)+"/carla/__init__.py"))


def test_actual_path_fixture_full32_publication_keeps_all_failure_rows(monkeypatch,tmp_path,capsys):
    caller,sha,_,_=fixture_files(monkeypatch,tmp_path);fake_api(monkeypatch);synthetic_probe(monkeypatch)
    assert audit.main(["--caller-pins",str(caller),"--caller-pins-sha256",sha])==0
    report=json.loads((audit.OUTPUT/"summary.json").read_bytes())
    assert report["case_count"]==32 and sum(report["outcome_counts"].values())==32
    assert report["original_generation_trace_coverage"]["attempted"]==54833
    assert report["training_data_approved"] is False and report["simulator_started"] is False
    files=list(audit.OUTPUT.glob("*.gz"));assert len(files)==21
    assert all(len(gzip.open(p,"rt").readlines())==2 for p in files)
    assert json.loads(capsys.readouterr().out)["case_count"]==32


def test_post_geometry_input_changed_denied_before_first_mkdir(monkeypatch,tmp_path):
    caller,sha,es,_=fixture_files(monkeypatch,tmp_path);fake_api(monkeypatch)
    def mutate(*a):
        (audit.BASE/es[0]["path"]).write_bytes(b"mid-run mutation");return [],{}
    monkeypatch.setattr(audit,"run_census",mutate)
    with pytest.raises(ValueError,match="before output"):audit.main(["--caller-pins",str(caller),"--caller-pins-sha256",sha])
    assert not audit.OUTPUT.exists()


def test_no_live_simulator_actuator_API_or_geometry_reimplementation():
    tree=ast.parse(Path(audit.__file__).read_bytes())
    forbidden={"Client","apply_control","try_spawn_actor","spawn_actor","get_world","tick","load_world","get_waypoint_xodr","body_footprint","swept_body"}
    assert not [n for n in ast.walk(tree) if isinstance(n,ast.Attribute) and n.attr in forbidden]
    calls=[n for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=="probe"]
    assert len(calls)==1 and isinstance(calls[0].func.value,ast.Name) and calls[0].func.value.id=="nominal"
