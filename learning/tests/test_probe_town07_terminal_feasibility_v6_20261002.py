"""HH_261002 - Synthetic retained-denial terminal boundary tests only; no actual probe, neural network or simulator."""

from copy import deepcopy
import hashlib
import inspect
import json
import math
from pathlib import Path
import subprocess
import sys

import pytest

from scripts.e2e import probe_town07_terminal_feasibility_v6_20261002 as probe


def mission(length=100.):
    return dict(schema=probe.witness.MISSION_SCHEMA,coordinates=dict(probe.witness.COORDINATES),mission_id="test",
        map_id="Town07",route_id="test-route",source_route_artifact_sha256="a"*64,source_available_timestamp_ns=0,
        complete_mission_route=True,polyline_map_xy_m=[[0.,0.],[length,0.]],goal_map_xy_m=[length,0.],stop_reference_offset_m=.75)


def make_window(states,index=0):
    anchor=states[index]
    window=dict(source_frame=anchor["frame"],source_timestamp_ns=probe.stamp(anchor),failure_reasons=["known_geometric_failure"],
        necessary_defined_domain_checks_clear=False,current_semantic_witness=dict(intent=anchor["goal_stop"]["terminal_mission_phase"],measured_current_speed_mps=anchor["vx"]),true_future_rows=[])
    for j in range(64):
        idx=index+(j+1)*2;s=states[idx]
        window["true_future_rows"].append(dict(index=j,valid=True,target_timestamp_ns=probe.stamp(anchor)+(j+1)*100_000_000,
            xy=[s["x"]-anchor["x"],s["y"]-anchor["y"]],speed_mps=s["vx"],yaw_rad=0.,
            source_native_bracket_frames=[s["frame"],s["frame"]],source_native_bracket_timestamps_ns=[probe.stamp(s)]*2,
            initial_measured_planar_acceleration_mps2=(s["vx"]-anchor["vx"])/.1 if j==0 else None,
            strict_defined_geometry_witness=dict(metrics=dict(xy_curvature=[None,False])),future_capture_phases=[s["capture_phase"]]*2))
    return window


def fixture(length=100.,hold_start=12):
    states=[];x=10.;prior=.01
    for i in range(hold_start+139):
        speed=.01 if i<hold_start+1 else 0.
        if i:x+=.05*(prior+speed)/2
        phase="GO" if i<hold_start//2 else "BRAKE_TO_STOP" if i<hold_start else "HOLD"
        states.append(dict(frame=100+i,timestamp=1.+i*.05,capture_phase="driving" if i<hold_start+9 else "stationary_tail",
            x=x,y=0.,z=.1,yaw=0.,vx=speed,vy=0.,goal_stop=dict(terminal_mission_phase=phase)))
        prior=speed
    cameras=[]
    for state in states[::2]:
        cameras.append(dict(frame=state["frame"],timestamp=state["timestamp"],camera_order=list(probe.CAMERAS),
            source_timestamps={c:state["timestamp"] for c in probe.CAMERAS},
            original_BGRA_source_sha256={c:hashlib.sha256(c.encode()).hexdigest() for c in probe.CAMERAS}))
    mp=probe.encoded(mission(length));ms=probe.sha(mp)
    by_frame={s["frame"]:(i,s) for i,s in enumerate(states)}
    window=make_window(states)
    return mp,ms,states,cameras,[window],by_frame


def test_exact_zero_is_later_observed_event_not_first_hold_threshold():
    _,_,states,_,_,_=fixture();original=deepcopy(states)
    result,index=probe.exact_zero_reference(states)
    assert states==original and index==13 and result["first_HOLD_speed_mps"]==.01
    assert result["first_HOLD_frame"]==112 and result["frame"]==113 and result["stationary_tail_count"]==130
    assert result["maximum_later_pose_displacement_m"]==0. and result["later_actual_pose_drift_arc_m"]==0.


def test_exact_zero_does_not_mean_pose_stationary_or_curvature_zero():
    _,_,states,_,_,_=fixture();states[15]["x"]+=.0003
    result,index=probe.exact_zero_reference(states)
    assert index==13 and result["maximum_later_pose_displacement_m"]==pytest.approx(.0003)
    assert result["later_actual_pose_drift_arc_m"]==pytest.approx(.0006)
    assert result["event_does_not_assert_pose_stationary"]


def test_transient_zero_before_reacceleration_is_not_sustained_reference():
    _,_,states,_,_,_=fixture();states[13]["vx"]=0.;states[14]["vx"]=1e-30
    result,index=probe.exact_zero_reference(states)
    assert index==15 and result["frame"]==115


@pytest.mark.parametrize("kind",["no_zero","tail_nonzero","tail_missing","no_hold"])
def test_missing_exact_event_is_unassessed_not_faked(kind):
    _,_,states,_,_,_=fixture()
    if kind=="no_zero":
        for s in states:s["vx"]=1e-30
    elif kind=="tail_nonzero":states[-1]["vx"]=1e-30
    elif kind=="tail_missing":states.pop()
    else:
        for s in states:s["goal_stop"]["terminal_mission_phase"]="GO"
    if kind in ("tail_missing","no_hold"):
        with pytest.raises(ValueError):probe.exact_zero_reference(states)
    else:
        result,index=probe.exact_zero_reference(states);assert index is None and result["status"].startswith("NO_")


@pytest.mark.parametrize("v0,v1,D,phase,expected",[
    (1.,1.,1.,"GO",True),(1.,1.,.01,"GO",False),(2.5,2.5,1.9,"GO",True),
    (.04,0.,.0005,"HOLD",True),(.04,0.,.0003,"HOLD",False),(.04,0.,.003,"HOLD",False),
    (0.,0.,0.,"HOLD",True),(0.,0.,.01,"HOLD",False),(.01,.02,.01,"HOLD",False),
    (0.,.01,.01,"GO",True),(0.,0.,.01,"GO",True),(1.,1.16,2.,"GO",False),
    (1.,.79,2.,"BRAKE_TO_STOP",False),(0.,0.,0.,"BRAKE_TO_STOP",False),
])
def test_necessary_scalar_and_true_prefix_constraints(v0,v1,D,phase,expected):
    report=probe.boundary_checks(v0,v1,D,phase,support=64.,terminal=True)
    assert report["all_necessary_boundaries_pass"] is expected
    assert report["necessary_not_sufficient_for_decoder_representability"]


def test_exact_first_prefix_boundary_and_earlier_stop_interval():
    result=probe.boundary_checks(1.,1.,.35,"GO",support=1.,terminal=True)
    assert result["prefix_plus_braking_distance_m"]==.35 and result["checks"]["exact_first100_prefix_and_remaining_braking"]
    bad=probe.boundary_checks(1.,1.,math.nextafter(.35,0.),"GO",support=1.,terminal=True)
    assert not bad["checks"]["exact_first100_prefix_and_remaining_braking"]
    for distance in (.04*.04/4.,.05*.04):
        report=probe.boundary_checks(.04,0.,distance,"HOLD",support=1.,terminal=True)
        assert report["early_sub100ms_stop_case"]


def test_zero_GO_prefix_is_necessary_not_positive_preferred_speed_reachability():
    # HH_261002 - Prepared GO may depart after the initial stationary 100ms; this probe never supplies a speed head.
    report=probe.boundary_checks(0.,0.,.01,"GO",support=1.,terminal=True)
    assert report["all_necessary_boundaries_pass"]
    assert report["zero_preferred_positive_arc_unreachable_not_tested"]
    assert report["necessary_not_sufficient_for_decoder_representability"]


@pytest.mark.parametrize("field",["v0","v1","D","support"])
@pytest.mark.parametrize("bad",[True,float("nan"),float("inf"),-.01])
def test_no_boolean_nonfinite_or_negative_measurement(field,bad):
    values=dict(v0=.1,v1=.1,D=1.,support=2.)
    values[field]=bad
    with pytest.raises(ValueError):probe.boundary_checks(values["v0"],values["v1"],values["D"],"GO",support=values["support"],terminal=True)


def test_all_original_labels_flags_and_nulls_preserved_no_input_mutation():
    mp,ms,states,cameras,windows,_=fixture();before=deepcopy((states,cameras,windows))
    summary,result=probe.analyze(mp,ms,"a"*64,states,cameras,windows)
    assert (states,cameras,windows)==before and len(result)==1
    assert summary["counts"]["true_future_rows"]==64 and summary["counts"]["original_failed_anchors"]==1
    assert summary["source_failure_counts"]=={"known_geometric_failure":1}
    assert result[0]["source_true_future_rows"]==windows[0]["true_future_rows"]
    assert result[0]["source_true_future_rows"][0]["strict_defined_geometry_witness"]["metrics"]["xy_curvature"]==[None,False]
    assert not summary["training_authorized"] and not summary["vehicle_control_approved"] and not summary["model_promoted"]
    assert summary["main_functions_approved"]==summary["subfunctions_approved"]==0


def test_far_future_ordinary_branch_not_failed_due_terminal_D_or_support():
    mp,ms,states,cameras,windows,_=fixture(300.)
    summary,result=probe.analyze(mp,ms,"a"*64,states,cameras,windows)
    assert result[0]["causal_Witness"]["mode"]==probe.witness.ORDINARY
    assert result[0]["boundary"]["terminal_label_checks_applicable"] is False
    assert result[0]["boundary"]["terminal_label_assessment"]=="ORDINARY_BRANCH_TERMINAL_LABEL_TESTS_INAPPLICABLE"
    assert summary["boundary_failure_counts"]=={} and summary["counts"]["causal_terminal_dispatch_unavailable"]==1


def test_far_ordinary_true_stop_beyond64_is_only_diagnostic_not_terminal_rejection():
    mp,ms,states,cameras,_,_=fixture(500.,hold_start=2000)
    for i,s in enumerate(states):
        s["vx"]=1.5 if i<=2000 else 0.
        s["x"]=10.+min(i,2001)*.075
    windows=[make_window(states)]
    summary,result=probe.analyze(mp,ms,"a"*64,states,cameras,windows)
    boundary=result[0]["boundary"]
    assert boundary["observed_stop_distance_label_m"]>64. and boundary["actual_reference_event_within_6p4"] is False
    assert not boundary["checks"]["actual_terminal_support_contains_D"]
    assert boundary["terminal_label_checks_applicable"] is False and summary["boundary_failure_counts"]=={}
    assert summary["all_label_diagnostic_inequality_violations_not_ordinary_failures"]["actual_terminal_support_contains_D"]==1


def test_stop_event_after64_horizon_reported_separately_without_label_cut():
    mp,ms,states,cameras,windows,_=fixture(hold_start=140)
    summary,result=probe.analyze(mp,ms,"a"*64,states,cameras,windows)
    assert len(result[0]["source_true_future_rows"])==64 and summary["counts"]["reference_event_after_6p4"]==1
    assert result[0]["boundary"]["terminal_label_assessment"]=="TERMINAL_BOUNDARY_ASSESSED_EVENT_AFTER_6p4"
    assert result[0]["boundary"]["actual_reference_event_within_6p4"] is False


def test_current_backward_projection_rejects_stream_without_reset_or_clipping():
    mp,ms,states,cameras,_,_=fixture();states[14]["x"]=states[0]["x"]-.0003
    windows=[make_window(states,index) for index in (0,14,16)]
    summary,result=probe.analyze(mp,ms,"a"*64,states,cameras,windows)
    assert summary["first_stream_witness_rejection"]["reason_code"]=="NONMONOTONIC_MISSION_PROGRESS"
    assert summary["first_stream_witness_rejection"]["frame"]==114
    assert summary["actual_current_pose_projection_regression_count"]==1
    assert summary["maximum_actual_current_pose_projection_regression_m"]==pytest.approx(.0003)
    assert result[2]["stream_Witness"]["status"]=="UNASSESSED_AFTER_FIRST_STREAM_REJECTION"
    assert result[1]["causal_Witness"]["terminal_permission_is_prepared_not_live"]
    assert result[1]["independent_anchor_sequence0_is_not_stream_validation"]


def test_duplicate_anchor_is_rejected_not_repeated_view():
    mp,ms,states,cameras,windows,_=fixture()
    with pytest.raises(ValueError):probe.analyze(mp,ms,"a"*64,states,cameras,windows+windows)


def test_full20Hz_projection_keeps_between_camera_regression_and_no_clamped_progress():
    mp,_,states,_,_,_=fixture();states[13]["x"]-=.0003
    for row in states:row["route_progress_m"]=100.
    census=probe.native_projection_census(mp,states)
    assert census["recorded_native_count"]==151 and census["regression_count"]>=1
    assert any(r["frame"]==113 and r["signed_projection_delta_m"]<0 for r in census["regressions"])
    assert census["maximum_regression_m"]>0. and census["current_XY_not_recorded_monotone_route_progress_used"]


def test_causal_witness_api_cannot_accept_future_stop_or_semantic_label():
    signature=inspect.signature(probe.causal_observation)
    assert tuple(signature.parameters)==("mission_payload","mission_sha","route_sha","state","camera")
    mp,ms,states,cameras,windows,_=fixture();first=probe.causal_observation(mp,ms,"a"*64,states[0],cameras[0])
    windows[0]["true_future_rows"][0]["speed_mps"]=99.
    second=probe.causal_observation(mp,ms,"a"*64,states[0],cameras[0])
    assert first==second
    with pytest.raises(TypeError):probe.causal_observation(mp,ms,"a"*64,states[0],cameras[0],future_D=5.)


@pytest.mark.parametrize("change",["stamp","frame","camera_count","BGRA_SHA","future_mission"])
def test_current_causal_input_binding_fails_closed(change):
    mp,ms,states,cameras,_,_=fixture();camera=cameras[0]
    if change=="stamp":camera["source_timestamps"][probe.CAMERAS[0]]+=.05
    elif change=="frame":camera["frame"]+=1
    elif change=="camera_count":camera["camera_order"].pop()
    elif change=="BGRA_SHA":camera["original_BGRA_source_sha256"][probe.CAMERAS[0]]="broken"
    else:
        value=probe.decode(mp);value["source_available_timestamp_ns"]=2_000_000_000;mp=probe.encoded(value);ms=probe.sha(mp)
    with pytest.raises(ValueError):probe.causal_observation(mp,ms,"a"*64,states[0],camera)


@pytest.mark.parametrize("change",["XY","speed","yaw","bracket","timestamp"])
def test_all64_label_source_brackets_authenticated_without_rewriting(change):
    _,_,states,_,windows,by_frame=fixture();true=windows[0]["true_future_rows"]
    row=true[31]
    if change=="XY":row["xy"][0]+=.01
    elif change=="speed":row["speed_mps"]+=.01
    elif change=="yaw":row["yaw_rad"]+=.01
    elif change=="bracket":row["source_native_bracket_frames"][1]+=1
    else:row["source_native_bracket_timestamps_ns"][1]+=1
    with pytest.raises(ValueError):probe.true_heading_diagnostics(states[0],true,by_frame)


def test_signed_backward_XY_kept_and_body_yaw_not_velocity_heading():
    _,_,states,_,windows,by_frame=fixture();states[2]["x"]-=.002
    row=windows[0]["true_future_rows"][0];row["xy"][0]=states[2]["x"]-states[0]["x"]
    result=probe.true_heading_diagnostics(states[0],windows[0]["true_future_rows"],by_frame)
    assert result[0]["actual_XY_chord_vector_m"][0]<0.
    assert result[0]["XY_chord_heading_relative_to_anchor_rad"]==math.pi
    assert result[0]["measured_body_yaw_relative_to_anchor_rad"]==0.
    assert result[0]["measured_velocity_heading_relative_to_anchor_rad"]==0.
    assert result[10]["measured_velocity_heading_relative_to_anchor_rad"] is None


def test_source_stream_sha_and_duplicate_keys_fail_without_partial_output(tmp_path):
    p=tmp_path/"a.jsonl";p.write_bytes(b'{"x":1}\n')
    pins={};assert probe.rows(p,probe.sha(p.read_bytes()),pins)==[{"x":1}]
    p.write_bytes(b'{"x":1,"x":2}\n')
    with pytest.raises(ValueError):probe.rows(p,probe.sha(p.read_bytes()),{})
    p.write_bytes(b'{"x":1}\n')
    with pytest.raises(ValueError):probe.rows(p,"f"*64,{})
    p.write_bytes(b'{"x":1}')
    with pytest.raises(ValueError):probe.rows(p,probe.sha(p.read_bytes()),{})


def test_no_symlink_ancestor_or_lexical_escape(tmp_path):
    actual=tmp_path/"actual";actual.mkdir();link=tmp_path/"link";link.symlink_to(actual,target_is_directory=True)
    with pytest.raises(ValueError):probe.canonical(link/"new-output")
    with pytest.raises(ValueError):probe.canonical(str(actual)+"/../escape")


def test_cli_import_does_not_load_torch_numpy_or_cuda():
    script="import runpy; runpy.run_path('tools/workspace_python.py')['configure'](); from scripts.e2e import probe_town07_terminal_feasibility_v6_20261002; import sys; assert 'torch' not in sys.modules and 'numpy' not in sys.modules"
    result=subprocess.run([sys.executable,"-c",script],cwd=probe.ROOT,capture_output=True,text=True)
    assert result.returncode==0,result.stderr


def test_probe_cli_requires_external_authorization_before_access_or_output(tmp_path):
    with pytest.raises(ValueError):probe.main(["--mode","probe","--output",str(tmp_path/"out")])
    assert not (tmp_path/"out").exists()
