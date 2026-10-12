"""HH_261002 - Synthetic all597/true64 identity and unchanged numerical-denial checks; no actual pilot audit or TRAIN approval."""
import ast
import copy
import json
import math
from pathlib import Path
from unittest import mock

import pytest

from scripts.e2e import audit_town01_right70_true_future64_20261002 as qa
from test_town01_left45_true_future64_20261002 import future


def test_exact_identity_inverse_and_all_numerical_functions_unchanged():
    tree=qa.identity_ast()
    for name in ("canonical","payload","strict_future","analyze"):
        old=next(n for n in ast.parse(Path(qa.previous.__file__).read_bytes()).body if isinstance(n,ast.FunctionDef) and n.name==name)
        new=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name==name)
        assert ast.dump(old)==ast.dump(new)
    assert qa.owner is qa.declared_owner and qa.task is qa.owner.task
    assert qa.inspect.__globals__["owner"] is qa.owner and qa.main.__globals__["inspect"] is qa.inspect
    assert qa.SCHEMA=="carla.actual_town01_right70_true_future64_diagnostic.20261002.v1"
    assert qa.OUTPUT.name=="town01_low_speed_right70_future64_v1"


def test_all16_actual_full_capture_artifacts_not_old_LEFT_or_unknown():
    paths=qa.artifact_paths()
    assert len(paths)==len(set(paths))==16
    assert all("town01_low_speed_right70_teacher_v1" in str(p) for p in paths)
    assert {p.name for p in paths}>={"states.jsonl","control_receipts.jsonl","camera_frames.jsonl",
        "all_native_bodies_and_sweeps.jsonl.gz","owned_green_signal_fixture.json"}
    assert qa.FROZEN["tools/e2e/audit_town01_left45_true_future64_20261002.py"]==qa.PREVIOUS_SHA
    assert qa.FROZEN["tools/e2e/audit_town01_right70_closing_control_20261002.py"]==qa.owner.CLOSING_SHA
    for module in (qa.owner,qa.task,qa.owner.worker_module,qa.task.container):
        key=str(Path(module.__file__).relative_to(qa.ROOT))
        assert qa.FROZEN[key]==qa.owner.digest(module.__file__)


@pytest.mark.parametrize("speed",[.02,.1,1.,2.4])
def test_defined_curve_even_low_speed_exceeded_is_never_masked(speed):
    data=future(speed=speed,kappa=.3);before=copy.deepcopy(data)
    rows=qa.strict_future(data)
    assert len(rows)==64 and any(r["metrics"]["xy_curvature"][1] for r in rows) and data==before


def test_stationary_geometry_null_not_fabricated_zero_and_no_speed_clamp():
    data=future(speed=0.);rows=qa.strict_future(data)
    assert all(r["metrics"]["xy_curvature"]==[None,False] for r in rows)
    assert all(r["null_is_unassessed_not_zero_label"] for r in rows)
    data["current_recorded_velocity_base_mps"]=[1.,2.,0.]
    with mock.patch.object(qa.strict,"defined_geometry",return_value=[]) as kernel:qa.strict_future(data)
    assert kernel.call_args.kwargs["initial_speed_mps"]==math.hypot(1.,2.)


def test_all597_original_failed_windows_and38208_true_points_retained():
    values=[future() for _ in range(597)]
    windows=[dict(camera_index=i,failure_reasons=["original_inset_failure"] if i%2 else [],necessary_checks_clear=not i%2) for i in range(597)]
    with mock.patch.object(qa.frozen,"window_checks",return_value=(windows,dict(protected=[dict(frame=42)],outside_protected=[]))):
        rows,defects,census=qa.analyze([],[],[],[],[],values)
    assert len(rows)==597 and sum(len(r["strict_defined_geometry_witness"]) for r in rows)==38208
    assert sum(not r["necessary_checks_clear"] for r in rows)==298
    assert all(r["training_data_approved"] is False for r in rows)
    assert defects["protected"]==[dict(frame=42)] and census["xy_curvature"]["unassessed"]==597


@pytest.mark.parametrize("fault",["schema","output","source","missing_artifact","extra_artifact"])
def test_external_unknown_or_incomplete_authority_rejected_before_raw_reads(fault):
    sources={"synthetic.py":"a"*64}
    caller=dict(schema=qa.SCHEMA+".caller_pins",output=str(qa.OUTPUT),source_sha256=sources,
        artifact_sha256={str(p):"b"*64 for p in qa.artifact_paths()})
    if fault=="schema":caller["schema"]=qa.previous.SCHEMA+".caller_pins"
    elif fault=="output":caller["output"]="/tmp/foreign"
    elif fault=="source":caller["source_sha256"]={}
    elif fault=="missing_artifact":caller["artifact_sha256"].pop(next(iter(caller["artifact_sha256"])))
    else:caller["artifact_sha256"]["/tmp/foreign"]="c"*64
    with mock.patch.object(qa,"source_pins",return_value=sources),mock.patch.object(qa,"payload",return_value=json.dumps(caller).encode()) as reader:
        with pytest.raises(RuntimeError):qa.inspect(Path("synthetic-caller"),"d"*64)
    assert reader.call_count==1


def test_unavailable_future_never_extrapolated_or_restarted():
    data=future();data["valid_mask"][20:]=[False]*44
    for name in ("diagnostic_future_xy_m","diagnostic_body_yaw_delta_rad","diagnostic_endpoint_planar_speed_mps"):
        data[name][20:]=([[None,None]]*44 if name=="diagnostic_future_xy_m" else [None]*44)
    rows=qa.strict_future(data)
    assert all(r["actual_XY_chord_length_m"] is None for r in rows[20:])
    data["valid_mask"][21]=True
    with pytest.raises(ValueError):qa.strict_future(data)
