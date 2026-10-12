"""HH_261002 - Synthetic unchanged true64 numerical bounds and fixed artifact authorization; never inspect/capture actual data."""
import copy
import hashlib
import json
import math
from pathlib import Path
from unittest import mock

import pytest

from scripts.e2e import audit_town01_left45_true_future64_20261002 as qa


def future(speed=1.,kappa=0.):
    xy=[]
    for i in range(64):
        arc=(i+1)*speed*.1
        xy.append([math.sin(kappa*arc)/kappa,(1.-math.cos(kappa*arc))/kappa] if kappa else [arc,0.])
    return dict(valid_mask=[True]*64,valid_future_points=64,diagnostic_future_xy_m=xy,
        diagnostic_body_yaw_delta_rad=[kappa*(i+1)*speed*.1 for i in range(64)],
        diagnostic_endpoint_planar_speed_mps=[speed]*64,current_recorded_velocity_base_mps=[speed,0.,0.])


def test_straight_true64_and_original_arrays_not_mutated():
    data=future(); before=copy.deepcopy(data)
    result=qa.strict_future(data)
    assert len(result)==64 and data==before
    assert result[0]["metrics"]["xy_curvature"]==[None,False]
    assert all(row["metrics"]["xy_curvature"]==[0.,False] for row in result[1:])


@pytest.mark.parametrize("speed",[.02,.1,1.,2.4])
def test_genuinely_defined_curvature_not_masked_at_low_measured_speed(speed):
    result=qa.strict_future(future(speed=speed,kappa=.3))
    assert any(row["metrics"]["xy_curvature"][1] for row in result)
    assert all(row["positive_or_low_speed_never_used_as_curvature_mask"] for row in result)


def test_stationary_chords_remain_null_not_zero_label():
    value=future(speed=0.)
    result=qa.strict_future(value)
    assert len(result)==64
    assert all(row["metrics"]["xy_curvature"]==[None,False] for row in result)
    assert all(row["null_is_unassessed_not_zero_label"] for row in result)
    assert value["diagnostic_endpoint_planar_speed_mps"]==[0.]*64


def test_unavailable_contiguous_future_never_extrapolated_or_resume():
    value=future(); value["valid_mask"][20:]=[False]*44
    for key in ("diagnostic_future_xy_m","diagnostic_body_yaw_delta_rad","diagnostic_endpoint_planar_speed_mps"):
        value[key][20:]=([[None,None]]*44 if key=="diagnostic_future_xy_m" else [None]*44)
    result=qa.strict_future(value)
    assert all(row["actual_XY_chord_length_m"] is None for row in result[20:])
    value["valid_mask"][21]=True
    with pytest.raises(ValueError): qa.strict_future(value)


def test_actual_initial_planar_norm_not_clipped_longitudinal_proxy():
    value=future();value["current_recorded_velocity_base_mps"]=[1.,2.,0.]
    with mock.patch.object(qa.strict,"defined_geometry",return_value=[]) as kernel:
        qa.strict_future(value)
    assert kernel.call_args.kwargs["initial_speed_mps"]==math.hypot(1.,2.)


def test_additive_strict_failures_cannot_erase_old_reasons_or_raw_labels():
    original=future(speed=.02,kappa=.3); snapshots=copy.deepcopy(original)
    windows=[dict(camera_index=0,failure_reasons=["original_all_nine_future_bounds"],necessary_checks_clear=False)]
    with mock.patch.object(qa.frozen,"window_checks",return_value=(windows,dict(protected=[],outside_protected=[]))):
        result,_,census=qa.analyze([],[],[],[],[],[original])
    assert result[0]["failure_reasons"]==["original_all_nine_future_bounds","defined_geometry_exceeded_regardless_low_speed"]
    assert result[0]["necessary_checks_clear"] is False and result[0]["training_data_approved"] is False
    assert original==snapshots and result[0]["true_future_original_diagnostic"] is original
    assert census["xy_curvature"]["violations"]>0


def test_all_declared347_windows_even_failed_preserved_no_subset():
    values=[future() for _ in range(347)]
    windows=[dict(camera_index=i,failure_reasons=["body_inset"] if i%2 else [],necessary_checks_clear=not i%2) for i in range(347)]
    with mock.patch.object(qa.frozen,"window_checks",return_value=(windows,dict(protected=[dict(frame=12)],outside_protected=[]))):
        result,defects,_=qa.analyze([],[],[],[],[],values)
    assert len(result)==347 and sum(not w["necessary_checks_clear"] for w in result)==173
    assert sum(len(w["strict_defined_geometry_witness"]) for w in result)==22208
    assert defects["protected"]==[dict(frame=12)]


def test_exact_fixed16_artifacts_and_source_only_contract():
    paths=qa.artifact_paths()
    assert len(paths)==16 and len(set(paths))==16
    assert any(path.name=="owned_green_signal_fixture.json" for path in paths)
    assert all("town01_low_speed_left45_teacher_v2" in str(path) for path in paths)
    assert qa.OUTPUT.name=="town01_low_speed_left45_future64_v2"
    assert qa.SCHEMA.endswith(".v2") and qa.task.SCHEMA.endswith(".v2")
    assert qa.FROZEN["tools/e2e/run_owned_town01_low_speed_left45_20261002.py"].startswith("87afc96b")
    assert qa.FROZEN["tools/e2e/run_owned_town01_low_speed_left45_v2_20261002.py"].startswith("dd4960ee")
    assert qa.FROZEN["tools/e2e/audit_town01_left45_closing_control_20261002.py"].startswith("2d567108")


@pytest.mark.parametrize("path",["/tmp/foreign-input","/home/a/autoware_e2e/../escaped"])
def test_arbitrary_output_or_input_escape_denied(path):
    with pytest.raises(RuntimeError): qa.canonical(path)


def test_external_pin_mismatch_and_firstread_mutation_denied(tmp_path):
    path=tmp_path/"metadata.json";path.write_bytes(b'{"captured":true}')
    pins={};sha=hashlib.sha256(path.read_bytes()).hexdigest()
    with mock.patch.object(qa,"ROOT",tmp_path):
        assert qa.payload(path,sha,pins)==b'{"captured":true}'
        with pytest.raises(RuntimeError): qa.payload(path,"0"*64,pins)
        path.write_bytes(b'{"captured":false}')
        new=hashlib.sha256(path.read_bytes()).hexdigest()
        with pytest.raises(RuntimeError): qa.payload(path,new,pins)


def test_symlink_ancestor_not_only_leaf_rejected(tmp_path):
    real=tmp_path/"real";real.mkdir();(real/"raw.json").write_text("{}")
    link=tmp_path/"linked";link.symlink_to(real,target_is_directory=True)
    with mock.patch.object(qa,"ROOT",tmp_path):
        with pytest.raises(RuntimeError): qa.canonical(link/"raw.json")
