"""HH_261002 - Synthetic whole597 admission/conversion guards; no actual RIGHT raw payload, receipt, dataset copy or TRAIN approval."""
from __future__ import annotations

import ast
import copy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts import e2e
from scripts.e2e import qualify_town01_right70_common10_v2_20261002 as admission
from scripts.e2e import prepare_town01_right70_common10_v2_20261002 as convert


def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


@pytest.fixture
def native_manifest():
    pending=dict(admission_status="PENDING_QA",training_admission_state="PENDING_QA_NOT_AUTHORIZED",vehicle_control_approved=False)
    return dict(pending,status="complete",cleanup=dict(completed=True),control_source="Disclosed_BasicAgent_original_RIGHT_teacher_not_NN",
        capture_contract=dict(right70_protocol=admission.task.fixed_contract("v2","pilot_00")),
        result=dict(pending,task_complete=True,goal_reached=False,collision_event_count=0,lane_invasion_event_count=0,moving_task=dict(pending)))


def test_exact_inverse_AST_json_normalization_and_frozen_reconstruction_math():
    admission.admission_ast();convert.converter_ast()
    assert admission.qa is admission.declared_qa and convert.authority is admission
    assert admission.json_containers is admission.previous.json_containers
    function=admission.validation_function();before=admission.previous.validation_function()
    assert function.__code__.co_code==before.__code__.co_code and function.__code__.co_consts==before.__code__.co_consts
    assert function.__globals__["COUNT"]==597 and function.__globals__["CAMERA"]==778
    assert function.__globals__["ADAPTER_ID"]==admission.ADAPTER_ID
    assert admission.frozen.COUNT==302 and admission.previous.validation_function().__globals__["COUNT"]==347
    assert not hasattr(admission,"verify_weather_provenance")


def test_public_contract_sources_597_true38208_and_single_existing_TRAIN_corridor():
    value=admission.public_contract()
    assert value["schema"]==admission.SCHEMA and value["selected_camera_anchors"]==597 and value["true_future_points"]==38208
    assert value["prepared_output"].endswith("town01-low-speed-right70-train-20261002-v2")
    assert value["native_episode"]==str(admission.task.paths("v2","pilot_00")[0]/"episode")
    assert value["QA_windows"].endswith("all597_true_future_windows.jsonl")
    assert value["fixed_driving_relative_anchor_offsets"]==[80,1272] and value["fixed_anchor_driving_seconds"]==[4.,63.6]
    assert value["rolling_RIGHT_only"] is True and value["STOP_or_stationary_tail_supervision"] is False
    assert value["new_independent_site_or_sealed_TEST_claimed"] is False and value["original32_and_failed_generated_p00_not_requalified"] is True
    for module in (admission.previous,admission.previous_converter,admission.qa,admission,convert):
        assert value["source_sha256"][str(Path(module.__file__).relative_to(admission.ROOT))]==digest(module.__file__)


def test_pending_only_is_not_permission(native_manifest):
    admission.scope_manifest(native_manifest)
    assert "training_data_approved" not in native_manifest and native_manifest["vehicle_control_approved"] is False


@pytest.mark.parametrize("path",[(),("result",),("capture_contract",),("result","goal_stop_quality","driving_final_stop")])
@pytest.mark.parametrize("key,value",[("training_data_approved",False),("training_data_approved",0),("development_only",True)])
def test_permanent_original_denials_never_overridden(native_manifest,path,key,value):
    target=native_manifest
    for name in path:target=target.setdefault(name,{})
    target[key]=value
    with pytest.raises((admission.adapter.AdapterError,admission.frozen.contract.ContractError)):admission.scope_manifest(native_manifest)


@pytest.mark.parametrize("fault",["failed","LEFT","v1","events"])
def test_wrong_or_failed_native_source_rejected(native_manifest,fault):
    if fault=="failed":native_manifest["status"]="failed"
    elif fault=="LEFT":native_manifest["control_source"]="Disclosed_BasicAgent_original_LEFT_teacher_not_NN"
    elif fault=="v1":native_manifest["capture_contract"]["right70_protocol"]["run_id"]="v1"
    else:native_manifest["result"]["lane_invasion_event_count"]=1
    with pytest.raises(admission.adapter.AdapterError):admission.scope_manifest(native_manifest)


@pytest.fixture
def evidence(tmp_path,monkeypatch,native_manifest):
    root=tmp_path;plans=root/"storage/experiments/plans/2026-10-02";plans.mkdir(parents=True)
    episode=root/"actual_right_v2/episode";episode.mkdir(parents=True)
    (episode/"manifest.json").write_text(json.dumps(native_manifest))
    route=dict(town="Town01",scenario="right");(episode/"route.json").write_text(json.dumps(route))
    for name in ("states.jsonl","camera_frames.jsonl"):(episode/name).write_text("{}\n")
    artifacts=[episode/name for name in ("manifest.json","route.json","states.jsonl","camera_frames.jsonl")]
    for i in range(12):
        path=root/("proof_%d.json"%i);path.write_text("{}");artifacts.append(path)
    windows=[dict(frame=1080+2*i,timestamp_ns=2**63+31+100_000_000*i,camera_index=52+i,
        failure_reasons=[],necessary_checks_clear=True,training_data_approved=False,
        true_future_original_diagnostic=dict(valid_future_points=64,discrete_bound_diagnostic=dict(steps=[dict(index=j,
            metrics={name:(0.00012345678901234567,False) for name in admission.frozen.raw.targets.METRICS}) for j in range(64)]))) for i in range(597)]
    summary=dict(status="NECESSARY_CHECKS_CLEAR_NOT_ADMITTED",necessary_checks_clear=True,actual_teacher_capture_passed=True,
        fixed_anchor_count=597,true_future_point_count=38208,failed_fixed_window_count=0,raw_native_denials=[],
        raw_native_denials_overridden=False,case_plan_sha256="a"*64)
    qa=root/"QA";qa.mkdir();summary_path=qa/"summary.json";window_path=qa/"all597.jsonl"
    summary_path.write_text(json.dumps(summary));window_path.write_text("".join(json.dumps(row)+"\n" for row in windows))
    caller=plans/"caller.json";caller.write_text(json.dumps(dict(artifact_sha256={str(p):digest(p) for p in artifacts})))
    contract=dict(native_episode=str(episode),source_sha256={})
    authority=dict(schema=admission.SCHEMA+".external_ROOT_authority",contract=contract,research_TRAIN_admission_authorized=True,
        Common10_conversion_authorized=True,vehicle_control_approved=False,model_promoted=False,formal_feature_approval=False,
        QA_caller_path=str(caller),QA_caller_sha256=digest(caller),QA_summary_sha256=digest(summary_path),QA_windows_sha256=digest(window_path))
    path=plans/"auth.json";path.write_text(json.dumps(authority))
    monkeypatch.setattr(admission,"ROOT",root);monkeypatch.setattr(admission,"safe",lambda p,**_:Path(p))
    monkeypatch.setattr(admission,"document",lambda p:json.loads(Path(p).read_bytes()))
    monkeypatch.setattr(admission,"public_contract",lambda:copy.deepcopy(contract))
    monkeypatch.setattr(admission,"QA_SUMMARY",summary_path);monkeypatch.setattr(admission,"QA_WINDOWS",window_path)
    monkeypatch.setattr(admission.task,"fixed_contract",lambda *_:copy.deepcopy(native_manifest["capture_contract"]["right70_protocol"]))
    monkeypatch.setattr(admission.task,"ROUTE_SHA",digest(episode/"route.json"));monkeypatch.setattr(admission.task,"ROUTE",root/"generated_capture.json")
    monkeypatch.setattr(admission.qa,"artifact_paths",lambda:tuple(artifacts))
    monkeypatch.setattr(admission.qa,"inspect",lambda *_:(copy.deepcopy(summary),copy.deepcopy(windows),{}))
    return SimpleNamespace(path=path,authority=authority,summary=summary,windows=windows,summary_path=summary_path,window_path=window_path)


def test_full597_nested_tuple_json_equivalence_retains_all38208(evidence):
    raw=evidence.window_path.read_bytes();saved=[json.loads(line) for line in raw.splitlines()]
    assert saved!=evidence.windows and saved==admission.json_containers(evidence.windows)
    value=admission.evidence(evidence.path,digest(evidence.path))
    assert value["research_training_authorized"] is True and value["selected_camera_anchors"]==597 and value["true_future_points"]==38208
    assert value["status"]=="QUALIFIED_RESEARCH_TRAIN_RIGHT597" and len(value["cases"][0]["selected_anchors"])==597
    assert value["route_identity"]["identity_source"]=="NEW_PINNED_XODR_CENTRELINE_SAME_EXISTING_TRAIN_CORRIDOR"
    assert evidence.window_path.read_bytes()==raw and all(row["training_data_approved"] is False for row in evidence.windows)
    assert all(value[k] is False for k in ("raw_native_denials_overridden","labels_or_pixels_transformed","vehicle_control_approved","model_promoted"))


@pytest.mark.parametrize("fault",["metric","future_count","missing","reordered","denied","capture_failed","window_failure"])
def test_any_true_window_or_summary_failure_denies_whole_pilot(evidence,fault):
    if fault=="metric":evidence.windows[296]["true_future_original_diagnostic"]["discrete_bound_diagnostic"]["steps"][63]["metrics"]["xy_curvature"]=(.201,True)
    elif fault=="missing":evidence.windows.pop()
    elif fault=="reordered":evidence.windows[5],evidence.windows[6]=evidence.windows[6],evidence.windows[5]
    else:
        key,value={"future_count":("true_future_point_count",38207),"denied":("raw_native_denials",["permanent False"]),
            "capture_failed":("actual_teacher_capture_passed",False),"window_failure":("failed_fixed_window_count",1)}[fault]
        evidence.summary[key]=value;evidence.summary_path.write_text(json.dumps(evidence.summary))
        evidence.authority["QA_summary_sha256"]=digest(evidence.summary_path);evidence.path.write_text(json.dumps(evidence.authority))
    with pytest.raises(admission.adapter.AdapterError):admission.evidence(evidence.path,digest(evidence.path))


def test_external_authority_required_before_any_native_read(evidence,monkeypatch):
    evidence.authority["research_TRAIN_admission_authorized"]=False;evidence.path.write_text(json.dumps(evidence.authority))
    monkeypatch.setattr(admission.qa,"inspect",lambda *_:pytest.fail("denied authority must not open actual QA"))
    with pytest.raises(admission.adapter.AdapterError):admission.evidence(evidence.path,digest(evidence.path))


def test_new_scoped_converter_restores_and_preserves_generic_denial():
    provider=e2e.qualify_carla_moving_common10;load,write=convert.base._load_native,convert.base._write_json
    denial=convert.base._require_native_admission_not_denied
    with pytest.raises(LookupError):
        with convert.bound_right_authority():
            assert e2e.qualify_carla_moving_common10 is admission and convert.base._require_native_admission_not_denied is denial
            with pytest.raises(admission.adapter.AdapterError):
                with convert.bound_right_authority():pass
            raise LookupError("synthetic failure")
    assert e2e.qualify_carla_moving_common10 is provider and (convert.base._load_native,convert.base._write_json)==(load,write)
    assert not convert._active and not convert.previous._active


def test_exact_new_opt_in_flag_and_old_LEFT_flag_rejected():
    for argv in ([],["--execute-reviewed-left45-conversion"],["--execute-reviewed-right70-conversion"]*2):
        with pytest.raises(admission.adapter.AdapterError):convert.main(argv)
    with pytest.raises(admission.adapter.AdapterError):convert.prepare_dataset(SimpleNamespace())
    # HH_261002 - Both converter and original authorize must use the same current opt-in name, not a stale LEFT attribute.
    assert "execute_reviewed_right70_conversion" in admission.authorize.__code__.co_consts
    assert "execute_reviewed_left45_conversion" not in admission.authorize.__code__.co_consts


def test_metadata_only_changes_no_original_native_payload(monkeypatch):
    captured=[]
    def write(path,value):captured.append((path,value))
    write.__module__=convert.base.__name__;monkeypatch.setattr(convert.base,"_write_json",write)
    native=dict(training_admission_state="PENDING_QA_NOT_AUTHORIZED")
    old=dict(adapter_id="carla_fixed_moving_task_to_common10_v1",native_result=native)
    with convert.bound_right_authority():convert.base._write_json(Path("collection_config.json"),old)
    assert old["adapter_id"]=="carla_fixed_moving_task_to_common10_v1" and captured[0][1]["native_result"] is native
    assert captured[0][1]["future_selection_policy"]=="prospective_actual_native_frames80_through1272_all597_RIGHT_anchors_no_quality_filter"
    assert captured[0][1]["adapter_id"]==admission.ADAPTER_ID and captured[0][1]["STOP_or_stationary_tail_supervision"] is False


def test_prepared_receipt_history589_588_and_original_raw_guard(monkeypatch,tmp_path):
    monkeypatch.setattr(admission,"sha",lambda _:"a"*64)
    values=dict(qualification=dict(receipt_sha256="b"*64,document=dict(source_sha256={},external_ROOT_authority_sha256="c"*64)),
        root=tmp_path,loaded=SimpleNamespace(fingerprint_sha256="d"*64),view=SimpleNamespace(fingerprint_sha256="e"*64),
        case_records=[{}],full_camera=589,full_ego=588,boundaries=[],signed_path=tmp_path/"signed.jsonl",measured={},supports=[18.],measured_speeds=[1.2])
    result=admission.prepared_receipt(values)
    assert result["train_examples"]==597 and result["true_future_points"]==38208 and result["full_five_camera_history_examples"]==589
    assert result["full_ten_ego_history_examples"]==588 and result["val_examples"]==result["test_examples"]==0
    values["full_camera"]=588
    with pytest.raises(admission.adapter.AdapterError):admission.prepared_receipt(values)
