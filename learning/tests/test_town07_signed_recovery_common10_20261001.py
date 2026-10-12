"""HH_261001 - Guard whole fixed recovery groups, native denials, causal signatures and scoped unchanged conversion."""

from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts import e2e
from scripts.e2e import qualify_town07_signed_recovery_common10_20261001 as authority
from scripts.e2e import prepare_town07_signed_recovery_common10_20261001 as converter


def pending_manifest():
    pending=dict(admission_status="PENDING_QA",training_admission_state="PENDING_QA_NOT_AUTHORIZED",vehicle_control_approved=False)
    return dict(pending,status="complete",cleanup=dict(completed=True),
                result=dict(pending,task_complete=True,goal_reached=False,collision_event_count=0,lane_invasion_event_count=0,
                            moving_task=dict(pending)),capture_contract={})


def test_fresh_pending_is_not_fake_training_approval():
    value=pending_manifest();authority.scope_manifest(value)
    assert "training_data_approved" not in value and value["vehicle_control_approved"] is False


@pytest.mark.parametrize("path",[(),("result",),("capture_contract",),("capture_contract","agent_initialization"),
    ("result","goal_stop_quality"),("result","goal_stop_quality","development_pilot")])
@pytest.mark.parametrize("marker",["training_data_approved","development_only"])
def test_explicit_native_denial_never_overridden(path,marker):
    value=pending_manifest();container=value
    for key in path:container=container.setdefault(key,{})
    container[marker]=False if marker=="training_data_approved" else True
    with pytest.raises((authority.adapter.AdapterError,ValueError)):authority.scope_manifest(value)


def test_holdout_cannot_be_changed_to_train(tmp_path,monkeypatch):
    monkeypatch.setattr(authority,"ROOT",tmp_path)
    entry=dict(native_episode=str(tmp_path),dataset_split_intent="dev")
    q=dict(document=dict(cases=[entry]))
    assert authority.entry_for(SimpleNamespace(path=tmp_path,split="val"),q)==entry
    with pytest.raises(RuntimeError):authority.entry_for(SimpleNamespace(path=tmp_path,split="train"),q)


def test_exact_groups_302_and_common10_minimum_remains30():
    assert authority.identities("train")==tuple(f"train_{i:02d}" for i in range(9))
    assert authority.identities("holdout")==("holdout_00","holdout_01")
    assert authority.COUNT==302 and authority.NATIVE==965 and authority.CAMERA==483
    assert authority.contract.load_contract()["capture"]["minimum_episode_duration_s"]==30


@pytest.mark.parametrize("y,yaw",[(-.35,-.05),(0.,0.),(.35,.05)])
def test_current_signed_route_state_ros_left_and_CCW_only(y,yaw):
    route=dict(route=[dict(x=0.,y=0.),dict(x=20.,y=0.)])
    cte,heading=authority.signed_route_state(route,dict(x=5.,y=y,yaw=yaw))
    assert cte==pytest.approx(y) and heading==pytest.approx(yaw)


def test_signed_current_projection_matches_canonical_heading_tie_not_first_segment():
    # HH_261001 - Opposite overlapping route segments must select causal current heading as the frozen canonical navigator does.
    route=dict(route=[dict(x=0.,y=0.),dict(x=10.,y=0.),dict(x=0.,y=0.)])
    cte,yaw=authority.signed_route_state(route,dict(x=5.,y=.1,yaw=3.141592653589793))
    assert cte==pytest.approx(-.1) and yaw==pytest.approx(0.)


def test_original_denial_and_label_helpers_not_patched_in_conversion_scope():
    base=converter.base
    original=(base._require_native_admission_not_denied,base._interpolate_state,base._relative_xy_yaw,base._rig_document,
              base._load_native,base._write_json,e2e.qualify_carla_moving_common10)
    with pytest.raises(ValueError):
        with converter.bound_recovery_authority():
            assert (base._require_native_admission_not_denied,base._interpolate_state,base._relative_xy_yaw,base._rig_document)==original[:4]
            assert e2e.qualify_carla_moving_common10 is authority
            with pytest.raises(RuntimeError):
                with converter.bound_recovery_authority():pass
            raise ValueError("exit")
    assert (base._require_native_admission_not_denied,base._interpolate_state,base._relative_xy_yaw,base._rig_document,
            base._load_native,base._write_json,e2e.qualify_carla_moving_common10)==original


def test_no_unreviewed_converter_fallthrough_or_output(monkeypatch):
    monkeypatch.setattr(converter.base,"prepare_dataset",lambda args:pytest.fail("no original converter before authority flag"))
    with pytest.raises(RuntimeError):converter.prepare_dataset(SimpleNamespace(execute_reviewed_recovery_conversion=False))


def window_fixture(monkeypatch):
    states=[dict(frame=i,timestamp=i*.05,capture_phase="driving") for i in range(151)]
    cameras=[dict(frame=i,capture_phase="driving") for i in range(1,20,2)]
    native=[dict(frame=s["frame"],timestamp_ns=round(s["timestamp"]*1e9),capture_phase=s["capture_phase"],speed_mps=.5,
                 scalar_rate_mps2=0.,longitudinal_dvx_dt_mps2=0.,checks={k:(s["capture_phase"]=="driving" if k=="driving_phase" else True)
                        for k in authority.native_math.NATIVE_CHECKS}) for s in states]
    monkeypatch.setattr(authority.native_math,"native_checks",lambda *_:native)
    metric={k:(0.,False) for k in authority.raw.targets.METRICS}
    future=dict(valid_future_points=64,future_crosses_stationary_tail=False,
        current_recorded_velocity_base_mps=[.5,0.,0.],anchor_timestamp_ns=950_000_000,
        diagnostic_endpoint_planar_speed_mps=[.5]*64,diagnostic_future_xy_m=[[.05,0.]]*64,
        diagnostic_body_yaw_delta_rad=[0.]*64,diagnostic_target_timestamp_ns=[1_050_000_000+i*100_000_000 for i in range(64)],
        discrete_bound_diagnostic=dict(steps=[dict(metrics=metric) for _ in range(64)]))
    futures=[deepcopy(future) for _ in cameras]
    anchors=[dict(same_recorded_frame_and_timestamp=True) for _ in cameras]
    selected=[dict(camera_index=9,frame=19,timestamp_ns=950_000_000)]
    return selected,cameras,anchors,states,[],futures,native


def test_prior_guard_real_history_and_future_must_all_be_driving(monkeypatch):
    values=window_fixture(monkeypatch)
    windows,_=authority.window_checks(*values[:6]);assert windows[0]["necessary_checks_clear"]
    values[6][0]["checks"]["driving_phase"]=False
    values[3][0]["capture_phase"]="stationary_warmup"
    windows,_=authority.window_checks(*values[:6]);assert "driving_phase" in windows[0]["failure_reasons"]
    assert len(windows)==1


@pytest.mark.parametrize("failure",["body","speed","ninebounds","tail","pixels","history"])
def test_fixed_bad_window_retained_never_quality_filtered(monkeypatch,failure):
    values=window_fixture(monkeypatch)
    if failure=="body":values[6][20]["checks"]["body_inset"]=False
    elif failure=="speed":values[6][20]["speed_mps"]=2.500001
    elif failure=="ninebounds":values[5][9]["discrete_bound_diagnostic"]["steps"][0]["metrics"]["speed_limit"]=(20.,True)
    elif failure=="tail":values[5][9]["future_crosses_stationary_tail"]=True
    elif failure=="pixels":values[2][0]["same_recorded_frame_and_timestamp"]=False
    else:values[1][0]["capture_phase"]="stationary_warmup"
    windows,_=authority.window_checks(*values[:6]);assert len(windows)==1 and windows[0]["failure_reasons"]


def test_per_case_frame_sets_do_not_leak_across_native_readers(monkeypatch,tmp_path):
    monkeypatch.setattr(authority,"ROOT",tmp_path)
    base=converter.base
    cases=[dict(native_episode=str(tmp_path/str(i)),dataset_split_intent="train",selected_camera_frames=[10+i],route_identity=dict(route_id=str(i))) for i in range(2)]
    q=dict(document=dict(cases=cases),native_objects=[])
    observed=[]
    def original(spec,*,moving_qualification):
        observed.append(moving_qualification["document"]["selected_camera_frames"])
        return SimpleNamespace(episode_id=str(spec.path),moving_qualification=moving_qualification)
    original.__module__=base.__name__
    monkeypatch.setattr(base,"_load_native",original)
    with converter.bound_recovery_authority():
        for c in cases:base._load_native(SimpleNamespace(path=Path(c["native_episode"]),split="train"),moving_qualification=q)
    assert observed==[[10],[11]] and len(q["native_objects"])==2 and "selected_camera_frames" not in q["document"]


def test_native_json_tuple_reader_meets_frozen_ACK_list_ABI(monkeypatch,tmp_path):
    # HH_261001 - This reproduces the actual-center API mismatch without touching a frozen raw file.
    row={"frame":1};monkeypatch.setattr(authority.adapter,"_jsonl",lambda _: (row,))
    values=authority.load_raw_journals(tmp_path,("states","cameras","ACK","teacher"))
    assert len(values)==4 and all(isinstance(value,list) and value==[row] and value[0] is row for value in values)


@pytest.mark.parametrize("run_id",("v1","v2","v3","latest"))
def test_failed_old_or_undeclared_arbitrary_cohort_is_not_training(run_id):
    with pytest.raises(RuntimeError):authority.prepared_path(run_id,"train")
    with pytest.raises(RuntimeError):authority.evidence(run_id,"0"*64,"train")
    with pytest.raises(RuntimeError):authority.case_evidence(run_id,"train_04","0"*64,lambda *_:None)


def test_v4_only_schema_actual_smaller_grid_and_rolling_not_startup_scope():
    assert authority.SCHEMA.endswith(".v4") and authority.RESULT_SCHEMA.endswith(".v4")
    assert authority.prepared_path("v4","train").name=="town07-signed-recovery-train-20261001-v4"
    assert [(row["signed_lateral_offset_ros_m"],row["signed_yaw_offset_ros_deg"]) for row in authority.cohort.CASES[:9]]==[
        (lateral,yaw) for lateral in (-.20,0.,.20) for yaw in (-1.5,0.,1.5)]
    assert [row["seed"] for row in authority.cohort.CASES[:9]]==list(range(2026100710,2026100719))
    assert [row["seed"] for row in authority.cohort.CASES[9:]]==[2026100810,2026100811]
    assert authority.ROLLING_SCOPE==dict(rolling_recovery_only=True,startup_anchors_used_for_training=False,
        raw_startup_retained_as_context_only=True,fixed_driving_relative_anchor_offsets=[80,682],
        fixed_anchor_driving_seconds=[4.0,34.1])


def test_measured_hypot_first_edge_never_clips_or_mutates_true_labels(monkeypatch):
    values=window_fixture(monkeypatch);future=values[5][9]
    future["current_recorded_velocity_base_mps"]=[.3,.4,0.]
    before=deepcopy(future)
    assert authority.measured_v0_first_edge_checks(future)==[] and future==before
    future["diagnostic_endpoint_planar_speed_mps"][0]=.800001
    assert "measured_v0_speed_acceleration" in authority.measured_v0_first_edge_checks(future)


def test_measured_first_edge_failure_retains_fixed_anchor(monkeypatch):
    values=window_fixture(monkeypatch)
    values[5][9]["diagnostic_endpoint_planar_speed_mps"][0]=1.
    windows,_=authority.window_checks(*values[:6])
    assert len(windows)==1 and "measured_v0_speed_acceleration" in windows[0]["failure_reasons"]


def test_one_case_debug_never_admits_or_creates_receipt(monkeypatch,capsys):
    seen=[]
    monkeypatch.setattr(authority,"inspect_case",lambda *args: seen.append(args) or {"research_training_authorized":False,"admission_receipt_written":False})
    assert authority.main(["--inspect-case","train_04","--case-plan-sha256","a"*64])==0
    assert seen==[("v4","train_04","a"*64)] and '"research_training_authorized": false' in capsys.readouterr().out


@pytest.mark.parametrize("extra",[["--output","/tmp/must-not-write"],["--group","train"],["--batch-plan-sha256","b"*64]])
def test_debug_case_cannot_mix_admission_output_or_batch(extra,monkeypatch):
    monkeypatch.setattr(authority,"inspect_case",lambda *_:pytest.fail("ambiguous authority must fail before evidence"))
    with pytest.raises(RuntimeError):authority.main(["--inspect-case","train_04","--case-plan-sha256","a"*64,*extra])


def test_new_teacher_metadata_must_report_actual_launch_revision():
    manifest={"capture_contract":{}}
    authority.cohort.annotate_manifest(manifest);authority.verify_teacher_metadata(manifest)
    manifest["capture_contract"]["teacher_controller"]["launch_throttle"]=.13
    with pytest.raises(RuntimeError):authority.verify_teacher_metadata(manifest)


@pytest.mark.parametrize("key,value",[("normal_launch_controls_inherited_v2",True),("measured_speed_clipped",True),
    ("native_acceleration_qa_limit_mps2",3.2),("inherited_BasicAgent_emergency_unchanged",False),
    ("post_handoff_throttle_ramp_per_second",.05)])
def test_teacher_metadata_cannot_hide_different_caps_clipping_or_hazard(key,value):
    manifest={"capture_contract":{}};authority.cohort.annotate_manifest(manifest)
    manifest["capture_contract"]["teacher_controller"][key]=value
    with pytest.raises(RuntimeError):authority.verify_teacher_metadata(manifest)


@pytest.mark.parametrize("key,value",[("startup_training_claimed",True),("all_raw_startup_retained",False),
    ("selected_driving_native_frames",[18,620]),("per_case_adaptive_or_quality_crop",True),
    ("expected_native_states",903)])
def test_new_rolling_metadata_never_claims_startup_or_old_membership(key,value):
    # HH_261001 - The new V4 interval is common and prospective, not a successful old-case crop.
    manifest={"capture_contract":{}};authority.cohort.annotate_manifest(manifest)
    manifest["capture_contract"]["moving_recovery_training_scope"][key]=value
    with pytest.raises(RuntimeError):authority.verify_teacher_metadata(manifest)


def initialization_fixture():
    # HH_261001 - Synthetic contract-only witnesses; no claim that these rows are actual captured data.
    control=dict(throttle=0.,brake=1.,steer=0.,hand_brake=False,reverse=False,gear=1,manual_gear_shift=True)
    pose=dict(x=0.,y=0.,z=0.,yaw=0.,pitch=0.,roll=0.)
    def state(frame,phase):
        return dict(frame=frame,timestamp=frame*.05,capture_phase=phase,current_control=deepcopy(control),
            actor_snapshot_transform_carla=deepcopy(pose),world_velocity_carla=[0.,0.,0.],goal_stop={},control_transport={})
    bootstrap=state(100,"bootstrap");bootstrap["alignment"]=dict(status="PASS",expected_receipt_sequence=0)
    states=[state(frame,"stationary_warmup") for frame in range(101,125)]+[state(125,"driving")]
    sent=dict(control,throttle=.11,brake=0.)
    states[-2]["next_control"]=deepcopy(sent);states[-2]["goal_stop"]["next_control_starts_driving"]=True
    states[-2]["control_transport"]["next_command_receipt_sequence"]=25
    states[-1]["current_control"]=deepcopy(sent);states[-1]["control_transport"]=dict(status="PASS",expected_receipt_sequence=25)
    def witness(reference,pid=False):
        value=dict(status="PASS",expected_frame=reference["frame"],frame_before=reference["frame"],frame_after=reference["frame"],
            timestamp=reference["timestamp"],actor_id=7,actor_snapshot_transform_carla=deepcopy(pose),current_control=deepcopy(control),
            raw_state_reference="CARLA actor snapshot API reference point; physical COM identity is unverified.")
        if pid:value["pid_past_steering"]=0.
        return value
    br=dict(reason="pre_capture_bootstrap",status="ACKNOWLEDGED",server_accepted=True,response_actor_id=7,actor_id=7,
        do_tick=False,before_frame=99,after_ack_frame=99,sequence=0)
    ir=dict(reason="initial_drive_control",status="ACKNOWLEDGED",server_accepted=True,response_actor_id=7,actor_id=7,
        do_tick=False,before_frame=124,expected_before_frame=124,after_ack_frame=124,sequence=25,
        requested_control=deepcopy(sent),physical_actuation_proven=False)
    metadata=dict(schema="carla.basic_agent_initialization.v1",mode="after_bootstrap",explicit_opt_in=True,
        required_control_transport="acknowledged_batch",existing_bootstrap_tick_reused=True,additional_world_ticks=0,
        constructor_requires_verified_bootstrap=True,pid_history_overwritten=False,observed_zero_steering_required=True,
        first_proposed_steering_delta_limit=.1,comparison_absolute_tolerance=1e-6,physical_cause_proven=False,
        construction=dict(status="PASS",before=witness(bootstrap),after=witness(bootstrap,True)),
        engagement=dict(status="PASS",before=witness(states[-2],True),first_command_acknowledged=True,
            first_proposed_control=deepcopy(sent),first_sent_control=deepcopy(sent),frame_after_agent_step=124,
            first_proposed_steering_delta=0.,first_command_receipt_sequence=25))
    manifest=dict(capture_contract=dict(agent_initialization=metadata,control_transport=dict(bootstrap_observation=bootstrap)))
    return manifest,states,[br,ir]


def test_new_manual_first_initialization_QA_not_old_automatic_auditor():
    result=authority.initialization_audit(*initialization_fixture())
    assert result["status"]=="PASS" and result["actual_manual_first_gear_preselected_before_throttle"] is True
    assert result["historical_automatic_only_auditor_relabelled"] is False and result["physical_actuation_proven"] is False


def test_disabled_goal_stop_actual_native_layout_has_no_optional_goal_stop_object():
    # HH_261001 - Actual disabled-profile rows legitimately omit goal_stop; receipt-bound next driving state stays mandatory.
    manifest,states,receipts=initialization_fixture()
    for row in states:row.pop("goal_stop")
    assert authority.initialization_audit(manifest,states,receipts)["status"]=="PASS"
    states[-1]["capture_phase"]="stationary_warmup"
    with pytest.raises(RuntimeError):authority.initialization_audit(manifest,states,receipts)


@pytest.mark.parametrize("stage,witness,key,value",[("construction","before","frame_after",101),
    ("construction","after","timestamp",5.01),("construction","after","actor_id",8),
    ("construction","after","pid_past_steering",.01),("engagement","before","pid_past_steering",True)])
def test_initialization_frame_actor_timestamp_and_zero_PID_are_exact(stage,witness,key,value):
    manifest,states,receipts=initialization_fixture()
    manifest["capture_contract"]["agent_initialization"][stage][witness][key]=value
    with pytest.raises(RuntimeError):authority.initialization_audit(manifest,states,receipts)


@pytest.mark.parametrize("key,value",[("gear",0),("gear",True),("manual_gear_shift",False),("steer",.01),
    ("throttle",.01),("brake",.99),("reverse",True),("hand_brake",True)])
def test_initialization_never_proxies_actual_full_brake_or_manual_first(key,value):
    manifest,states,receipts=initialization_fixture()
    manifest["capture_contract"]["agent_initialization"]["construction"]["after"]["current_control"][key]=value
    with pytest.raises((RuntimeError,ValueError)):authority.initialization_audit(manifest,states,receipts)


@pytest.mark.parametrize("failure",["warmup","subsequent_gear","receipt","PID_reset","snapshot"])
def test_initialization_full_warmup_first_ack_and_observed_application_preserved(failure):
    manifest,states,receipts=initialization_fixture()
    if failure=="warmup":states.pop(2)
    elif failure=="subsequent_gear":states[-1]["current_control"]["gear"]=0
    elif failure=="receipt":receipts[-1]["before_frame"]+=1
    elif failure=="PID_reset":manifest["capture_contract"]["agent_initialization"]["pid_history_overwritten"]=True
    else:manifest["capture_contract"]["agent_initialization"]["construction"]["after"]["actor_snapshot_transform_carla"]["x"]=.5
    with pytest.raises((RuntimeError,ValueError)):authority.initialization_audit(manifest,states,receipts)
