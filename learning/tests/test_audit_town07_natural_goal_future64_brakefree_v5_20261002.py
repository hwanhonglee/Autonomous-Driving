"""HH_261002 - Synthetic immutable future64 geometry/mission evidence only; never real capture, model or TRAIN admission."""

import ast
from copy import deepcopy
from dataclasses import asdict
from pathlib import Path

import pytest

from scripts.e2e import audit_town07_natural_goal_future64_brakefree_v5_20261002 as audit
from learning.tests import test_audit_town07_natural_goal_future64_20261002 as fixtures


def add_mission(row,phase="GO",coast=None,hold=None):
    row["goal_stop"].update(profile_id=audit.PROFILE,terminal_mission_phase=phase,
        coast_source_measurements=deepcopy(coast),low_speed_service_hold_source=deepcopy(hold),
        coast_elapsed_s=None if coast is None else row["timestamp"]-coast["timestamp"],
        pilot_failure_reason=None,source_frame=row["frame"],source_timestamp=row["timestamp"],
        training_admission_state="PENDING_QA_NOT_AUTHORIZED",target_speed_mps=1.5 if phase=="GO" else 0.)
    return row


def fixture():
    plan,manifest,states,cameras=fixtures.fixture()
    plan.update(schema=audit.TASK_SCHEMA,seed=audit.task.SEED)
    coast=dict(frame=states[741]["frame"],timestamp=states[741]["timestamp"],remaining_route_arc_m=11.5,measured_speed_mps=1.5)
    hold=dict(frame=states[781]["frame"],timestamp=states[781]["timestamp"],remaining_route_arc_m=.75,
        planar_goal_distance_m=.75,terminal_overshoot_m=-.75,measured_speed_mps=0.)
    for i,row in enumerate(states):add_mission(row,"GO" if i<741 else "BRAKE_TO_STOP" if i<781 else "HOLD",
        coast if i>=741 else None,hold if i>=781 else None)
    return plan,manifest,states,cameras


def core(inputs):
    with audit.activated():return audit.audit_core(*inputs)


def test_all360_fixed_anchors_all23040_actualfuture_keep_original_labels_and_bounds():
    inputs=fixture();before=deepcopy(inputs)
    summary,windows=core(inputs)
    with audit.activated():oldsummary,oldwindows=audit.definition("audit_core")(*inputs)
    assert inputs==before and summary["selected_anchor_count"]==360 and summary["true_future_point_count"]==23040
    assert summary["failed_fixed_window_count"]==0 and summary["strict_defined_geometry_failed_window_count"]==0
    assert summary["research_training_authorized"] is False and summary["vehicle_control_approved"] is False
    assert summary["raw_native_denials_overridden"] is False
    for fresh,original in zip(windows,oldwindows):
        for row,oldrow in zip(fresh["true_future_rows"],original["true_future_rows"]):
            assert {k:v for k,v in row.items() if k!="strict_defined_geometry_witness"}==oldrow
    assert all(r["strict_defined_geometry_witness"]["metrics"]["xy_curvature"]==[None,False]
        for r in windows[-1]["true_future_rows"])
    assert windows[-1]["true_future_rows"][-1]["speed_mps"]==0.
    assert summary["anchor_teacher_intent_counts"]["HOLD"]>0
    assert oldsummary["future_discrete_metric_counts"]==summary["future_discrete_metric_counts"]


def future_rows(speed=.001,delta=.0004):
    # HH_261002 - These are declared synthetic coordinates, not altered captured pixels or source poses.
    return [dict(index=i,valid=True,xy=[0.,delta if i%2==0 else 0.],yaw_rad=0.,speed_mps=speed) for i in range(64)]


def test_true_defined_curvature_fails_low_speed_without_relabeling_or_masking_it():
    rows=future_rows();before=deepcopy(rows)
    checked=audit.defined_geometry(rows,initial_speed_mps=.001)
    assert rows==before and checked[0]["metrics"]["xy_curvature"]==[None,False]
    assert all(r["metrics"]["xy_curvature"][1] for r in checked[1:])
    assert all(r["positive_or_low_speed_never_used_as_curvature_mask"] for r in checked)
    rows0=future_rows(speed=0.)
    assert audit.defined_geometry(rows0,initial_speed_mps=0.)[1]["metrics"]["xy_curvature"][1]


@pytest.mark.parametrize("distance",[0.,.00001,.0001])
def test_exact_zero_or_tiny_chords_have_null_curvature_never_zero_fabricated_label(distance):
    rows=future_rows(speed=0.,delta=distance);before=deepcopy(rows)
    result=audit.defined_geometry(rows,initial_speed_mps=0.)
    assert rows==before and len(result)==64
    assert all(r["metrics"]["xy_curvature"]==[None,False] and r["metrics"]["xy_lateral_acceleration"]==[None,False] for r in result)
    assert all(r["null_is_unassessed_not_zero_label"] for r in result)


def test_stationary_old_noise_mask_retained_but_new_realchord_witness_fails():
    states=[add_mission(fixtures.state(i,speed=.001,x=0.,phase="stationary_tail")) for i in range(140)]
    coast=dict(frame=1000,timestamp=0.);hold=dict(frame=1000,timestamp=0.)
    for i,row in enumerate(states):
        row["y"]=(.0002 if i%4==2 else -.0002 if i%4==0 else 0.)
        row["goal_stop"].update(phase="complete",goal_window=True,remaining_route_arc_m=.75,terminal_overshoot_m=-.75,complete=True,hold_duration_sec=2.)
        add_mission(row,"HOLD",coast,hold)
    before=deepcopy(states)
    with audit.activated():rows=audit.previous.future_window([(audit.previous.stamp(r),r) for r in states],states[0])
    checked=audit.defined_geometry(rows,initial_speed_mps=.001)
    assert states==before and all(r["stationary_domain_metrics"]["xy_curvature"]==[None,False] for r in rows)
    assert all(r["curvature_label_valid"] is False for r in rows)
    assert any(r["metrics"]["xy_curvature"][1] for r in checked) and all(r["speed_mps"]==.001 for r in rows)


def test_missing_future64_denominator_not_extrapolated_and_cannot_resume():
    rows=future_rows();rows[4:]=[dict(index=i,valid=False,xy=None,yaw_rad=None,speed_mps=None) for i in range(4,64)]
    checked=audit.defined_geometry(rows,initial_speed_mps=.001)
    assert len(checked)==64 and checked[-1]["actual_XY_chord_length_m"] is None
    assert checked[-1]["metrics"]["xy_curvature"]==[None,False]
    rows[6]=future_rows()[6]
    with pytest.raises(ValueError,match="resume"):audit.defined_geometry(rows,initial_speed_mps=.001)


@pytest.mark.parametrize("kind",["short","unordered","bool_index","bool_valid","negative_speed","nan_xy","negative_v0"])
def test_strict_numeric_membership_and_actual_planar_speed_norm(kind):
    rows=future_rows();v0=.001
    if kind=="short":rows.pop()
    elif kind=="unordered":rows[1]["index"]=2
    elif kind=="bool_index":rows[1]["index"]=True
    elif kind=="bool_valid":rows[0]["valid"]=1
    elif kind=="negative_speed":rows[0]["speed_mps"]=-1.
    elif kind=="nan_xy":rows[0]["xy"][0]=float("nan")
    else:v0=-1.
    with pytest.raises(ValueError):audit.defined_geometry(rows,initial_speed_mps=v0)


def test_actual_terminal_semantics_not_original_envelope_cruise_or_positive_brake_label():
    row=add_mission(fixtures.state(5,speed=.5),"BRAKE_TO_STOP",dict(frame=1004,timestamp=.2))
    row["goal_stop"]["phase"]="cruise"
    witness=audit.intent_witness(row)
    assert witness["intent"]=="BRAKE_TO_STOP" and witness["teacher_phase"]=="cruise" and witness["valid"]
    row["goal_stop"]["terminal_mission_phase"]="GO"
    assert not audit.intent_witness(row)["valid"]
    row=add_mission(fixtures.state(5,speed=.05),"HOLD",dict(frame=1004,timestamp=.2),dict(frame=1005,timestamp=.25))
    row["goal_stop"].update(goal_window=True,terminal_overshoot_m=-.75)
    assert audit.intent_witness(row)["valid"]
    row["vx"]+=1e-6
    assert not audit.intent_witness(row)["valid"]


@pytest.mark.parametrize("mutation",["future_frame","future_time","wrong_pending","not_latched","target_nonzero","failure"])
def test_semantics_cannot_fake_prior_source_or_pending_authority(mutation):
    row=add_mission(fixtures.state(5,speed=.5),"BRAKE_TO_STOP",dict(frame=1004,timestamp=.2))
    record=row["goal_stop"]
    if mutation=="future_frame":record["coast_source_measurements"]["frame"]=1006
    elif mutation=="future_time":record["coast_source_measurements"]["timestamp"]+=1.
    elif mutation=="wrong_pending":record["training_admission_state"]="TRAIN_APPROVED"
    elif mutation=="not_latched":record["coast_source_measurements"]=None
    elif mutation=="target_nonzero":record["target_speed_mps"]=.1
    else:record["pilot_failure_reason"]="actual_failure"
    assert not audit.intent_witness(row)["valid"]


def test_original_native2p95_and_hard_deceleration_still_fail_allfixed_windows():
    inputs=fixture();inputs[2][767]["vx"],inputs[2][768]["vx"],inputs[2][769]["vx"]=1.5371,.721294,.000851
    inputs[1]["status"]="failed";inputs[1]["result"]["goal_stop_quality"]["status"]="FAIL"
    inputs[1]["result"]["moving_task"]["task_complete"]=False
    summary,windows=core(inputs)
    assert summary["source_capture_failed"] and summary["selected_anchor_count"]==360
    assert any(r["actual_scalar_dvdt_mps2"]<-16. for r in summary["all_raw_native_interval_failures"])
    assert any("protected_native20Hz_dynamics_or_events" in w["failure_reasons"] for w in windows)
    timeline=[(audit.previous.stamp(r),r) for r in [fixtures.state(i,speed=1. if i==0 else 1.1475) for i in range(140)]]
    edge=audit.previous.native_intervals(timeline,0)[0]
    assert edge["native_scalar_dvdt_exceeds_original_physical2p9"] and not edge["native_scalar_dvdt_exceeds3"]


def test_existing_denial_guard_not_changed_or_bypassed_by_diagnostic():
    inputs=fixture();inputs[1]["result"]["training_data_approved"]=False;before=deepcopy(inputs[1])
    report,_=core(inputs)
    assert report["native_explicit_denial_present"] and report["raw_native_denials_overridden"] is False
    assert inputs[1]==before and report["training_data_approved"] is False


@pytest.mark.parametrize("kind",["native_gap","camera_gap","camera_order","source_stamp","tail_anchor","hold"])
def test_all_original_raw_camera_history_tail_completion_guards_remain(kind):
    inputs=fixture()
    if kind=="native_gap":inputs[2].pop(240)
    elif kind=="camera_gap":inputs[3].pop(240)
    elif kind=="camera_order":inputs[3][120]["camera_order"].reverse()
    elif kind=="source_stamp":inputs[3][120]["source_timestamps"][audit.previous.contract.CAMERA_ORDER[0]]+=.001
    elif kind=="tail_anchor":inputs[3][-1]["moving_task_selected"]=True
    else:inputs[2][823]["goal_stop"]["hold_duration_sec"]=1.95
    with pytest.raises(ValueError):core(inputs)


def governor_pair():
    row,receipt=fixtures.governor_pair();add_mission(row)
    receipt.update(schema=audit.TASK_SCHEMA+".actual_governor_update",governor_math_changed=True,
        original_governor_update=deepcopy(row["goal_stop"]))
    return row,receipt


def test_governor_receipt_binds_actual_mission_sameframe_without_legacyFalse_pass_claim():
    row,receipt=governor_pair()
    with audit.activated():report=audit.governor_receipt_binding([row],[receipt])
    assert report["actual_new_mission_fields_bind_sameframe_receipts"]
    assert report["original_governor_math_unchanged"] is False
    assert report["full_causal_owner_mission_replay_is_prior_pinned_evidence"] is True


@pytest.mark.parametrize("kind",["old_governor_flag","missing_mission","wrong_mission","native_source_future","wrong_v0"])
def test_actual_governor_label_bytes_frame_and_v0_must_match(kind):
    row,receipt=governor_pair()
    if kind=="old_governor_flag":receipt["governor_math_changed"]=False
    elif kind=="missing_mission":receipt["original_governor_update"].pop("terminal_mission_phase")
    elif kind=="wrong_mission":receipt["original_governor_update"]["terminal_mission_phase"]="HOLD"
    elif kind=="native_source_future":row["goal_stop"]["source_frame"]+=1;receipt["original_governor_update"]["source_frame"]+=1
    else:row["goal_stop"]["measured_speed_mps"]=0.;receipt["original_governor_update"]["measured_speed_mps"]=0.
    with audit.activated():
        with pytest.raises(ValueError):audit.governor_receipt_binding([row],[receipt])


def revision_fixture():
    plan=audit.task.fixed_contract("v5","pilot_00");config=deepcopy(plan["natural_goal_configuration"])
    config["effective_control"]=dict(normal_brake_cap=0.,normal_throttle_cap=.20,basic_agent_emergency_brake_modified=False)
    manifest=dict(control_source="Disclosed_BasicAgent_manual1_1p50_latched_brakefree_coast_low_speed_hold_v5",
        capture_contract=dict(goal_stop_profile=config,teacher_controller=dict(normal_brake_cap=0.,normal_throttle_cap=.20,
            normal_launch_controller_revision=audit.TASK_SCHEMA+".normal_control",manual_gear_shift=True,gear=1,
            inherited_BasicAgent_emergency_unchanged=True,causal_brakefree_terminal_mission=audit.task.mission_contract())))
    return plan,manifest


def test_exact_actual_configuration_reservation_manualgear_and_disclosed_controller():
    plan,manifest=revision_fixture();r=audit.teacher_revision_witness(plan,manifest)
    assert r["normal_brake_cap"]==0. and r["low_speed_service_hold_brake"]==1.
    assert r["episode_quota_bytes"]==3*1024**3 and r["new_pending_marker_is_not_TRAIN_authorization"]


@pytest.mark.parametrize("kind",["quota1gib","wrong_config","old_source","native_brake","gear_bool","hazard_modified","wrong_contract"])
def test_version_and_actual_installed_configuration_not_rename_only(kind):
    plan,manifest=revision_fixture();capture=manifest["capture_contract"]
    if kind=="quota1gib":plan["episode_quota_bytes"]=1024**3
    elif kind=="wrong_config":capture["goal_stop_profile"]["desired_deceleration_mps2"]=.2
    elif kind=="old_source":manifest["control_source"]=manifest["control_source"].replace("v5","v4")
    elif kind=="native_brake":capture["goal_stop_profile"]["effective_control"]["normal_brake_cap"]=.01
    elif kind=="gear_bool":capture["teacher_controller"]["gear"]=True
    elif kind=="hazard_modified":capture["goal_stop_profile"]["effective_control"]["basic_agent_emergency_brake_modified"]=True
    else:capture["teacher_controller"]["causal_brakefree_terminal_mission"]["coast_release_remaining_arc_m"]=12.
    with pytest.raises(ValueError):audit.teacher_revision_witness(plan,manifest)


def test_exact_guarded_AST_revision_and_all_context_scopes_restored(monkeypatch):
    saved={k:getattr(audit.previous,k) for k in ("SCHEMA","TASK_SCHEMA","PROFILE","SOURCE_PINS","natural_task","audit_core","intent_witness","governor_receipt_binding")}
    with pytest.raises(RuntimeError):
        with audit.activated():
            for name in ("inspect","audit_core","governor_receipt_binding"):assert callable(audit.definition(name))
            assert audit.previous.natural_task is audit.task and audit.previous.SCHEMA==audit.SCHEMA
            with pytest.raises(ValueError):
                with audit.activated():pass
            raise RuntimeError("synthetic")
    assert all(getattr(audit.previous,k) is v for k,v in saved.items()) and not audit._active
    assert audit.SOURCE_PINS[audit.FROZEN_AUDIT_PATH]==audit.FROZEN_AUDIT_SHA
    assert all(audit.SOURCE_PINS[k]==v for k,v in audit.task.FROZEN.items())
    tree=ast.parse((audit.ROOT/audit.FROZEN_AUDIT_PATH).read_bytes())
    base=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=="future_window")
    original=Path(audit.previous.__file__).read_text()
    assert ast.get_source_segment(original,base) is not None
    with pytest.raises(ValueError):audit.definition("future_window")


def test_low_speed_defined_geometry_window_failure_added_without_old_metric_removal():
    inputs=fixture()
    # HH_261002 - A synthetic HOLD-coordinate fixture exposes legacy masking; every selected anchor remains in the denominator.
    for i,row in enumerate(inputs[2][781:],781):
        row["vx"] = .001
        row["y"] = .0004 if i%4==0 else 0.
    before=deepcopy(inputs)
    summary,windows=core(inputs)
    assert inputs==before and summary["selected_anchor_count"]==360
    assert summary["strict_defined_geometry_failed_window_count"]>0 and summary["failed_fixed_window_count"]>0
    last=windows[-1]
    assert "defined_geometry_exceeded_even_at_low_speed" in last["failure_reasons"]
    assert all(r["stationary_domain_metrics"]["xy_curvature"]==[None,False] for r in last["true_future_rows"])
    assert any(r["strict_defined_geometry_witness"]["metrics"]["xy_curvature"][1] for r in last["true_future_rows"])
    assert all(r["speed_mps"]==.001 for r in last["true_future_rows"])


def test_main_fail_retained_creates_only_new_derived_files_and_no_admission(monkeypatch,tmp_path,capsys):
    import json
    root=tmp_path/"workspace";root.mkdir();raw=root/"raw/pilot_00/episode";raw.mkdir(parents=True)
    output=root/"derived/audit";summary=dict(failed_fixed_window_count=1,full_raw_event_count=0,source_capture_failed=True,
        source_artifact_sha256={},training_data_approved=False,research_training_authorized=False,
        vehicle_control_approved=False)
    monkeypatch.setattr(audit,"ROOT",root)
    monkeypatch.setattr(audit,"inspect",lambda *a:(summary,[dict(source_frame=1,true_future_rows=[])],{}))
    assert audit.main(["--episode",str(raw),"--case-plan",str(root/"plan.json"),"--case-plan-sha256","a"*64,"--output",str(output)])==2
    assert set(p.name for p in output.iterdir())=={"summary.json","all_fixed_anchor_true_future64.jsonl"}
    assert json.loads((output/"summary.json").read_text())["research_training_authorized"] is False
    assert not any(raw.iterdir())
    with pytest.raises(ValueError):audit.main(["--episode",str(raw),"--case-plan",str(root/"plan.json"),"--case-plan-sha256","a"*64,"--output",str(output)])
    with pytest.raises(ValueError):audit.main(["--episode",str(raw),"--case-plan",str(root/"plan.json"),"--case-plan-sha256","a"*64,"--output",str(raw.parent/"illegal")])


def test_declared_closure_reuses_exact_pinned_full_reader_and_all_v5_parent_sources():
    # HH_261002 - Inspect compiled closure identity without opening any actual capture or prediction payload.
    source=Path(audit.support.__file__).read_bytes();tree=ast.parse(source)
    original=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=="declared_closure")
    namespace=dict(vars(audit.support),task=type("Task",(),dict(FROZEN_V1=audit.task.FROZEN)),V2_SOURCE_PINS=audit.V5_SOURCE_PINS)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[original],type_ignores=[])),"synthetic unchanged closure","exec"),namespace)
    assert namespace["declared_closure"].__code__.co_code==audit.support.declared_closure.__code__.co_code
    assert set(audit.V5_SOURCE_PINS)=={"tools/e2e/"+n for n in ("town07_natural_goal_stop_brakefree_v5_20261002.py",
        "collect_town07_natural_goal_stop_brakefree_v5_20261002.py","run_owned_town07_natural_goal_stop_brakefree_v5_20261002.py")}
