"""HH_261002 - Synthetic V2 current-source identities and byte-exact inherited future checks; no real capture/QA/TRAIN."""
import ast
import copy
import json
from pathlib import Path
from unittest import mock

import pytest

from scripts.e2e import audit_town01_right70_true_future64_v2_20261002 as qa
from test_town01_left45_true_future64_20261002 import future


def test_inverse_identity_and_numerical_functions_identical():
    original=qa.previous.identity_ast();current=qa.identity_ast()
    for name in ("canonical","payload","strict_future","analyze"):
        old=next(n for n in original.body if isinstance(n,ast.FunctionDef) and n.name==name)
        new=next(n for n in current.body if isinstance(n,ast.FunctionDef) and n.name==name)
        assert ast.dump(old)==ast.dump(new)
        before=getattr(qa.previous,name);after=getattr(qa,name)
        assert before.__code__.co_code==after.__code__.co_code and before.__code__.co_consts==after.__code__.co_consts
    assert qa.owner is qa.declared_owner and qa.inspect.__globals__["owner"] is qa.owner
    assert qa.main.__globals__["inspect"] is qa.inspect and qa.task is qa.owner.task


def test_exact_current_paths_sources_and_no_v1_requalification():
    assert qa.SCHEMA=="carla.actual_town01_right70_true_future64_diagnostic.20261002.v2"
    assert qa.OUTPUT.name=="town01_low_speed_right70_future64_v2"
    paths=qa.artifact_paths()
    assert len(paths)==len(set(paths))==16 and all("town01_low_speed_right70_teacher_v2" in str(p) for p in paths)
    assert qa.FROZEN.items()>=qa.previous.FROZEN.items()
    for module in (qa.owner,qa.task,qa.owner.worker_module):
        key=str(Path(module.__file__).relative_to(qa.ROOT));assert qa.FROZEN[key]==qa.owner.digest(module.__file__)
    assert qa.previous.OUTPUT.name.endswith("_v1") and qa.previous.owner.task.SCHEMA.endswith(".v1")
    assert qa.task.fixed_contract("v2","pilot_00")["closing_receipt_reader_schema"]==qa.owner.closing.SCHEMA
    assert qa.task.ANCHORS==597 and qa.task.NATIVE_COUNT==1555 and qa.task.CAMERA_COUNT==778


@pytest.mark.parametrize("speed",[0.,.02,.1,2.4])
def test_futures_identical_with_undefined_stationary_and_defined_bad_curve(speed):
    data=future(speed=speed,kappa=.3);before=copy.deepcopy(data)
    assert qa.strict_future(data)==qa.previous.strict_future(data) and data==before
    if speed==0.:assert all(r["metrics"]["xy_curvature"]==[None,False] for r in qa.strict_future(data))
    else:assert any(r["metrics"]["xy_curvature"][1] for r in qa.strict_future(data))


@pytest.mark.parametrize("fault",["old_schema","old_output","old_artifacts","missing_artifact","extra_artifact","wrong_source"])
def test_incorrect_external_contract_fails_before_raw_payload(fault):
    sources={"synthetic.py":"a"*64}
    caller=dict(schema=qa.SCHEMA+".caller_pins",output=str(qa.OUTPUT),source_sha256=sources,
        artifact_sha256={str(p):"b"*64 for p in qa.artifact_paths()})
    if fault=="old_schema":caller["schema"]=qa.previous.SCHEMA+".caller_pins"
    elif fault=="old_output":caller["output"]=str(qa.previous.OUTPUT)
    elif fault=="old_artifacts":caller["artifact_sha256"]={str(p):"b"*64 for p in qa.previous.artifact_paths()}
    elif fault=="missing_artifact":caller["artifact_sha256"].pop(next(iter(caller["artifact_sha256"])))
    elif fault=="extra_artifact":caller["artifact_sha256"]["/tmp/foreign"]="c"*64
    else:caller["source_sha256"]={}
    with mock.patch.object(qa,"source_pins",return_value=sources),mock.patch.object(qa,"payload",return_value=json.dumps(caller).encode()) as read:
        with pytest.raises(RuntimeError):qa.inspect(Path("synthetic-caller"),"d"*64)
    assert read.call_count==1


def test_all597_failed_windows_and_all38208_original_points_are_kept():
    values=[future() for _ in range(597)]
    windows=[dict(camera_index=i,failure_reasons=["body_inset_failed"] if i==596 else [],necessary_checks_clear=i!=596) for i in range(597)]
    with mock.patch.object(qa.frozen,"window_checks",return_value=(windows,dict(protected=[{"frame":1272}],outside_protected=[]))):
        rows,defects,_=qa.analyze([],[],[],[],[],values)
    assert len(rows)==597 and sum(len(r["strict_defined_geometry_witness"]) for r in rows)==38208
    assert rows[-1]["necessary_checks_clear"] is False and defects["protected"]==[{"frame":1272}]
    assert all(r["training_data_approved"] is False for r in rows)
