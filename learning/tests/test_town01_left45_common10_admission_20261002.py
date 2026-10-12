"""HH_261002 - Source/ABI and synthetic one-episode admission guards; no real LEFT archive/conversion or GPU."""
from __future__ import annotations

import ast
import copy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts import e2e
from scripts.e2e import qualify_town01_left45_common10_20261002 as admission
from scripts.e2e import prepare_town01_left45_common10_20261002 as convert


@pytest.fixture
def native_manifest():
    pending=dict(admission_status="PENDING_QA",training_admission_state="PENDING_QA_NOT_AUTHORIZED",vehicle_control_approved=False)
    return dict(pending,status="complete",cleanup=dict(completed=True),
        control_source="Disclosed_BasicAgent_original_LEFT_teacher_not_NN",
        capture_contract=dict(left45_protocol=admission.task.fixed_contract("v2","pilot_00")),
        result=dict(pending,task_complete=True,goal_reached=False,collision_event_count=0,lane_invasion_event_count=0,
            moving_task=dict(pending)))


def test_pending_is_eligible_not_an_approval(native_manifest):
    admission.scope_manifest(native_manifest)
    assert "training_data_approved" not in native_manifest and native_manifest["vehicle_control_approved"] is False


@pytest.mark.parametrize("path", [(),("result",),("capture_contract",),("capture_contract","agent_initialization"),
    ("result","goal_stop_quality","driving_final_stop")])
@pytest.mark.parametrize("key,value", [("training_data_approved",False),("development_only",True),("training_data_approved",0)])
def test_all_original_denial_paths_remain_strict(native_manifest,path,key,value):
    obj=native_manifest
    for name in path:obj=obj.setdefault(name,{})
    obj[key]=value
    with pytest.raises((admission.adapter.AdapterError,admission.frozen.contract.ContractError)):
        admission.scope_manifest(native_manifest)


@pytest.mark.parametrize("mutate", [lambda m:m.update(status="failed"),lambda m:m.update(control_source="NN"),
    lambda m:m["result"].update(goal_reached=True),lambda m:m["result"].update(lane_invasion_event_count=1),
    lambda m:m["capture_contract"]["left45_protocol"].update(run_id="v1")])
def test_failed_original_or_wrong_scope_cannot_qualify(native_manifest,mutate):
    mutate(native_manifest)
    with pytest.raises(admission.adapter.AdapterError):admission.scope_manifest(native_manifest)


@pytest.fixture
def qualification(tmp_path,monkeypatch):
    monkeypatch.setattr(admission,"safe",lambda path,**_:Path(path).absolute())
    episode=tmp_path/"episode";episode.mkdir()
    entry=dict(native_episode=str(episode))
    q=dict(document=dict(schema=admission.SCHEMA,research_training_authorized=True,cases=[entry]),native_objects=[])
    return SimpleNamespace(q=q,spec=SimpleNamespace(split="train",path=episode),episode=episode)


def test_exact_one_train_only_entry(qualification):
    assert admission.entry_for(qualification.spec,qualification.q) is qualification.q["document"]["cases"][0]


@pytest.mark.parametrize("split", ["val","test"])
def test_no_dev_or_test_entry(qualification,split):
    qualification.spec.split=split
    with pytest.raises(admission.adapter.AdapterError):admission.entry_for(qualification.spec,qualification.q)


@pytest.mark.parametrize("mutation", ["unknown_schema","unauthorized","second_episode","foreign_episode"])
def test_no_case_reinterpretation(qualification,mutation,tmp_path):
    value=qualification.q["document"]
    if mutation=="unknown_schema":value["schema"]="v1"
    elif mutation=="unauthorized":value["research_training_authorized"]=False
    elif mutation=="second_episode":value["cases"].append(dict(value["cases"][0]))
    else:qualification.spec.path=tmp_path/"other"
    with pytest.raises(admission.adapter.AdapterError):admission.entry_for(qualification.spec,qualification.q)


def test_private_validation_retains_all_original_numerical_ast():
    fn=admission.validation_function()
    old=next(n for n in ast.parse(Path(admission.frozen.__file__).read_bytes()).body
        if isinstance(n,ast.FunctionDef) and n.name=="validate_prepared")
    assert isinstance(old.body[-1],ast.Return)
    assert fn.__globals__["COUNT"]==347 and fn.__globals__["CAMERA"]==528
    assert fn.__globals__["entry_for"] is admission.entry_for
    assert fn.__globals__["recheck"] is admission.recheck
    assert fn.__globals__["signed_route_state"] is admission.frozen.signed_route_state
    assert admission.frozen.COUNT==302 and admission.frozen.CAMERA==483
    assert fn.__globals__["prepared_receipt"] is admission.prepared_receipt
    assert fn.__code__.co_filename==admission.__file__


def test_scoped_generic_providers_restore_on_exception(qualification):
    old_provider=e2e.qualify_carla_moving_common10;load,write=convert.base._load_native,convert.base._write_json
    with pytest.raises(LookupError):
        with convert.bound_left_authority():
            assert e2e.qualify_carla_moving_common10 is admission
            assert convert.base._load_native is not load and convert.base._write_json is not write
            with pytest.raises(admission.adapter.AdapterError):
                with convert.bound_left_authority():pass
            raise LookupError("synthetic failure")
    assert e2e.qualify_carla_moving_common10 is old_provider
    assert (convert.base._load_native,convert.base._write_json)==(load,write) and convert._active is False


def test_generic_native_loader_and_denial_function_not_replaced():
    old=convert.base._require_native_admission_not_denied
    with convert.bound_left_authority():
        assert convert.base._require_native_admission_not_denied is old
        with pytest.raises(admission.adapter.AdapterError):convert.base._load_native(SimpleNamespace(),moving_qualification=None)


def test_independent_metadata_only_wrapper_does_not_rewrite_native_payload(monkeypatch):
    captured=[]
    def record(path,value):captured.append((path,value))
    record.__module__=convert.base.__name__
    monkeypatch.setattr(convert.base,"_write_json",record)
    native=dict(status="complete",training_admission_state="PENDING_QA_NOT_AUTHORIZED")
    collection=dict(adapter_id="carla_fixed_moving_task_to_common10_v1",native_result=native)
    with convert.bound_left_authority():convert.base._write_json(Path("collection_config.json"),collection)
    assert collection["adapter_id"]=="carla_fixed_moving_task_to_common10_v1"
    assert captured[0][1]["adapter_id"]==admission.ADAPTER_ID and captured[0][1]["native_result"] is native
    assert captured[0][1]["STOP_or_stationary_tail_supervision"] is False


def test_conversion_opt_in_required_before_output_or_authority():
    with pytest.raises(admission.adapter.AdapterError):convert.prepare_dataset(SimpleNamespace())
    for args in ([],["--execute-reviewed-left45-conversion"]*2):
        with pytest.raises(admission.adapter.AdapterError):convert.main(args)


def test_first_read_json_hashes_exact_parsed_payload(tmp_path,monkeypatch):
    path=tmp_path/"evidence.json";raw=b'{"phase":"LEFT","true_future":64}'
    path.write_bytes(raw);monkeypatch.setattr(admission,"safe",lambda p,**_:Path(p))
    monkeypatch.setattr(admission.adapter,"_json",lambda *_:pytest.fail("second JSON read forbidden"))
    result=admission.read_pinned(path,hashlib.sha256(raw).hexdigest(),{})
    assert result==dict(phase="LEFT",true_future=64)


@pytest.mark.parametrize("raw", [b'{"a":1,"a":2}',b'{"a":NaN}',b'[]'])
def test_external_json_duplicate_nonfinite_or_nonobject_rejected(tmp_path,monkeypatch,raw):
    path=tmp_path/"bad.json";path.write_bytes(raw);monkeypatch.setattr(admission,"safe",lambda p,**_:Path(p))
    with pytest.raises(admission.adapter.AdapterError):admission.read_pinned(path,hashlib.sha256(raw).hexdigest(),{})


def test_unapproved_authority_before_any_native_read(tmp_path,monkeypatch):
    root=tmp_path;plans=root/"storage/experiments/plans/2026-10-02";plans.mkdir(parents=True)
    path=plans/"auth.json";raw=json.dumps(dict(schema=admission.SCHEMA+".external_ROOT_authority",contract={},
        research_TRAIN_admission_authorized=False,Common10_conversion_authorized=True)).encode();path.write_bytes(raw)
    monkeypatch.setattr(admission,"ROOT",root);monkeypatch.setattr(admission,"safe",lambda p,**_:Path(p))
    monkeypatch.setattr(admission,"public_contract",lambda:{})
    monkeypatch.setattr(admission.qa,"inspect",lambda *_:pytest.fail("unapproved actual QA access"))
    with pytest.raises(admission.adapter.AdapterError):admission.evidence(path,hashlib.sha256(raw).hexdigest())


@pytest.fixture
def full_evidence(tmp_path,monkeypatch,native_manifest):
    root=tmp_path;plans=root/"storage/experiments/plans/2026-10-02";plans.mkdir(parents=True)
    episode=root/"actual_v2/pilot_00/episode";episode.mkdir(parents=True)
    summary_path=root/"prior_QA/summary.json";summary_path.parent.mkdir()
    windows_path=summary_path.parent/"all347.jsonl";caller_path=plans/"QA_caller.json"
    source_route=root/"route.json";route=dict(town="Town01",scenario="left")
    (episode/"route.json").write_text(json.dumps(route));route_sha=hashlib.sha256((episode/"route.json").read_bytes()).hexdigest()
    (episode/"manifest.json").write_text(json.dumps(native_manifest))
    for name in ("states.jsonl","camera_frames.jsonl"):(episode/name).write_text("{}\n")
    artifacts=[episode/name for name in ("manifest.json","route.json","states.jsonl","camera_frames.jsonl")]
    for index in range(12):
        path=root/("proof"+str(index)+".json");path.write_text("{}");artifacts.append(path)
    windows=[dict(frame=1080+2*i,timestamp_ns=4_000_000_000+100_000_000*i,camera_index=52+i,
        failure_reasons=[],necessary_checks_clear=True,training_data_approved=False) for i in range(347)]
    summary=dict(status="NECESSARY_CHECKS_CLEAR_NOT_ADMITTED",necessary_checks_clear=True,
        actual_teacher_capture_passed=True,fixed_anchor_count=347,true_future_point_count=22208,
        failed_fixed_window_count=0,raw_native_denials=[],raw_native_denials_overridden=False,case_plan_sha256="a"*64)
    summary_path.write_text(json.dumps(summary));windows_path.write_text("".join(json.dumps(w)+"\n" for w in windows))
    caller_path.write_text(json.dumps(dict(artifact_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in artifacts})))
    wanted=dict(native_episode=str(episode),source_sha256={})
    authority=dict(schema=admission.SCHEMA+".external_ROOT_authority",contract=wanted,
        research_TRAIN_admission_authorized=True,Common10_conversion_authorized=True,
        vehicle_control_approved=False,model_promoted=False,formal_feature_approval=False,
        QA_caller_path=str(caller_path),QA_caller_sha256=hashlib.sha256(caller_path.read_bytes()).hexdigest(),
        QA_summary_sha256=hashlib.sha256(summary_path.read_bytes()).hexdigest(),QA_windows_sha256=hashlib.sha256(windows_path.read_bytes()).hexdigest())
    path=plans/"authority.json";path.write_text(json.dumps(authority));expected=hashlib.sha256(path.read_bytes()).hexdigest()
    monkeypatch.setattr(admission,"ROOT",root);monkeypatch.setattr(admission,"safe",lambda p,**_:Path(p))
    monkeypatch.setattr(admission,"document",lambda p:json.loads(Path(p).read_bytes()))
    monkeypatch.setattr(admission,"public_contract",lambda:copy.deepcopy(wanted))
    monkeypatch.setattr(admission,"QA_SUMMARY",summary_path);monkeypatch.setattr(admission,"QA_WINDOWS",windows_path)
    monkeypatch.setattr(admission.task,"fixed_contract",lambda *_:copy.deepcopy(native_manifest["capture_contract"]["left45_protocol"]))
    monkeypatch.setattr(admission.task,"ROUTE_SHA",route_sha);monkeypatch.setattr(admission.task,"ROUTE",source_route)
    monkeypatch.setattr(admission.qa,"artifact_paths",lambda:tuple(artifacts))
    monkeypatch.setattr(admission.qa,"inspect",lambda *_:(copy.deepcopy(summary),copy.deepcopy(windows),{}))
    return SimpleNamespace(authority_path=path,authority_sha=expected,summary=summary,windows=windows,
        summary_path=summary_path,windows_path=windows_path,authority=authority,caller=caller_path)


def test_full347_authority_proof_retains_denominator_without_copying_denial_labels(full_evidence):
    value=admission.evidence(full_evidence.authority_path,full_evidence.authority_sha)
    assert value["schema"]==admission.SCHEMA and value["research_training_authorized"] is True
    assert value["selected_camera_anchors"]==347 and value["true_future_points"]==22208 and value["episode_count"]==1
    assert len(value["cases"][0]["selected_anchors"])==347
    assert all(set(row)=={"frame","timestamp_ns","camera_index"} for row in value["cases"][0]["selected_anchors"])
    assert all(not value[key] for key in ("vehicle_control_approved","model_promoted","raw_native_denials_overridden","labels_clipped"))
    assert value["input_sha256"][str(full_evidence.windows_path)]==full_evidence.authority["QA_windows_sha256"]
    assert full_evidence.windows[0]["training_data_approved"] is False


@pytest.mark.parametrize("field,value", [("necessary_checks_clear",False),("actual_teacher_capture_passed",False),
    ("true_future_point_count",22207),("failed_fixed_window_count",1),("raw_native_denials",["immutable denial"]),
    ("raw_native_denials_overridden",True)])
def test_source_failed_not_admitted_even_under_root_authority(full_evidence,field,value):
    full_evidence.summary[field]=value
    full_evidence.summary_path.write_text(json.dumps(full_evidence.summary))
    full_evidence.authority["QA_summary_sha256"]=hashlib.sha256(full_evidence.summary_path.read_bytes()).hexdigest()
    full_evidence.authority_path.write_text(json.dumps(full_evidence.authority))
    expected=hashlib.sha256(full_evidence.authority_path.read_bytes()).hexdigest()
    with pytest.raises(admission.adapter.AdapterError):admission.evidence(full_evidence.authority_path,expected)


def test_replayed_journal_mismatch_never_quality_filters(full_evidence):
    full_evidence.windows[12]["frame"]+=1
    with pytest.raises(admission.adapter.AdapterError):admission.evidence(full_evidence.authority_path,full_evidence.authority_sha)
