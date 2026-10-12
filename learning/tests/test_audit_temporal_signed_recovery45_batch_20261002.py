"""HH_261002 - Prove independent signed recovery, true manual1 witnesses, negative outcomes and restored archival/publication scopes."""

import ast
from copy import deepcopy
import math
from pathlib import Path
from unittest import mock

import pytest

from scripts.e2e import audit_temporal_signed_recovery45_batch_20261002 as audit
from scripts.e2e import publish_temporal_signed_recovery45_batch_20261002 as publish


def route():
    return dict(route=[dict(x=0.,y=0.,distance_m=0.,vad_command=3),dict(x=300.,y=0.,distance_m=300.,vad_command=3)])


def fixture(*,sign=1.,first_cte=.20,first_yaw=1.5,converged_at=1.,duration=15.,nonlearned_at=None):
    states=[];commands=[];receipts=[]
    for i in range(round(duration*20)+1):
        elapsed=i*.05; seq=i+1
        states.append(dict(frame=100+i,timestamp=10.+elapsed,x=2.*elapsed,
            y=sign*first_cte if elapsed<converged_at else sign*.02,
            yaw=math.radians(sign*first_yaw if elapsed<converged_at else .1),
            capture_phase="driving",control_transport=dict(status="PASS",expected_receipt_sequence=seq)))
        commands.append(dict(ack_receipt_sequence=seq,application=dict(learned_control_applied=nonlearned_at!=i)))
        receipts.append(dict(sequence=seq,before_frame=99+i,status="ACKNOWLEDGED",server_accepted=True))
    return states,commands,receipts


@pytest.mark.parametrize("arm,sign",[(0,-1.),(1,1.)])
def test_signed_errors_use_actual_prior_ACK_then_exact_native_entry_and_hold(arm,sign):
    states,commands,receipts=fixture(sign=sign)
    result=audit.independent_recovery(states,commands,receipts,route(),audit.PROFILES[arm])
    assert result["case_screen_passed"] and result["recovery_screen_passed"]
    assert result["first_learned_effect"]["native_frame"]==100
    assert result["first_learned_effect"]["expected_prior_ACK_sequence"]==1
    assert result["convergence"]["first_converged_simulation_seconds"]==1.
    assert result["convergence"]["confirmation_native_frame"]==160
    assert result["post_effect_native_count"]==len(states)


@pytest.mark.parametrize("kwargs",[dict(first_cte=.09),dict(first_yaw=.49),dict(sign=-1.),
    dict(converged_at=10.05),dict(duration=2.),dict(nonlearned_at=250)])
def test_removed_wrong_sign_late_short_and_nonlearned_tails_are_not_approved(kwargs):
    states,commands,receipts=fixture(**kwargs)
    result=audit.independent_recovery(states,commands,receipts,route(),audit.PROFILES[1])
    assert not result["case_screen_passed"] and not result["recovery_screen_passed"]
    assert result["post_effect_native_count"]==len(states)


def test_entry_by10s_can_confirm_at12s_but_not_after_a_broken_two_second_band():
    states,commands,receipts=fixture(converged_at=10.,duration=12.)
    result=audit.independent_recovery(states,commands,receipts,route(),audit.PROFILES[1])
    assert result["case_screen_passed"] and result["convergence"]["first_converged_simulation_seconds"]==10.
    assert result["convergence"]["confirmation_native_frame"]==340
    states[-1]["y"]=.101
    assert audit.independent_recovery(states,commands,receipts,route(),audit.PROFILES[1])["case_screen_passed"] is False


def test_latest_prior_ANY_source_prevents_outgoing_same_frame_teacher_or_stale_model_claims():
    states,commands,receipts=fixture(duration=3.)
    # HH_261002 - At frame100 sequence1 is the previous teacher command, while sequence2 is outgoing and unobserved.
    commands[0]["application"]["learned_control_applied"]=False
    result=audit.independent_recovery(states,commands,receipts,route(),audit.PROFILES[1])
    assert result["first_learned_effect"]["native_frame"]==101
    assert result["first_learned_effect"]["expected_prior_ACK_sequence"]==2
    states[2]["control_transport"]["expected_receipt_sequence"]=2
    with pytest.raises(Exception,match="latest prior ACK"):
        audit.independent_recovery(states,commands,receipts,route(),audit.PROFILES[1])


def test_no_learned_effect_is_explicit_failure_not_vacuous_and_center_not_recovery():
    states,commands,receipts=fixture()
    for row in commands:row["application"]["learned_control_applied"]=False
    result=audit.independent_recovery(states,commands,receipts,route(),audit.PROFILES[1])
    assert result["first_learned_effect"] is None and result["convergence"] is None and not result["case_screen_passed"]
    for row in commands:row["application"]["learned_control_applied"]=True
    centered=audit.independent_recovery(states,commands,receipts,route(),audit.PROFILES[2])
    assert centered["case_screen_passed"] and centered["centered_setup_screen_passed"] and not centered["recovery_screen_passed"]


def test_signed_projection_uses_rear_actual_pose_and_wrapped_yaw_not_absolute_heading():
    reverse=dict(route=[dict(x=0.,y=0.,distance_m=0.),dict(x=-300.,y=.001,distance_m=300.)])
    row=dict(frame=1,timestamp=1.,x=-10.,y=-.2,yaw=-math.pi+.001)
    projected=audit.measured_rear_errors([row],reverse)[0]
    assert projected["signed_rear_CTE_m"]>.19 and abs(projected["signed_rear_yaw_error_deg"])<.1
    for field in ("x","y","yaw","timestamp"):
        invalid=dict(row);invalid[field]=float("nan")
        with pytest.raises(Exception,match="finite"):audit.measured_rear_errors([invalid],reverse)


def full_control(**overrides):
    return dict(throttle=0.,steer=0.,brake=1.,hand_brake=False,reverse=False,manual_gear_shift=True,gear=1)|overrides


def initialization_fixture():
    pose=dict(x=10.,y=20.,z=1.,pitch=0.,roll=0.,yaw=0.)
    bootstrap=dict(frame=1000,timestamp=1.,actor_snapshot_transform_carla=pose,current_control=full_control(),
        alignment=dict(status="PASS",expected_receipt_sequence=1))
    states=[dict(frame=1001+i,timestamp=1.05+i*.05,capture_phase="stationary_warmup",
        actor_snapshot_transform_carla=deepcopy(pose),current_control=full_control(),world_velocity_carla=[0.,0.,0.],
        control_transport=dict(status="PASS",expected_receipt_sequence=i+2,next_command_receipt_sequence=i+3)) for i in range(24)]
    sent=full_control(throttle=.11,brake=0.)
    states[-1]["next_control"]=sent
    states[-1]["control_transport"]["next_command_receipt_sequence"]=27
    states.append(dict(frame=1025,timestamp=2.25,capture_phase="driving",current_control=sent,
        control_transport=dict(status="PASS",expected_receipt_sequence=27)))
    def witness(reference,pid=False):
        value=dict(status="PASS",expected_frame=reference["frame"],frame_before=reference["frame"],frame_after=reference["frame"],
            actor_id=42,timestamp=reference["timestamp"],actor_snapshot_transform_carla=deepcopy(reference["actor_snapshot_transform_carla"]),
            current_control=deepcopy(reference["current_control"]),
            raw_state_reference="CARLA actor snapshot API reference point; physical COM identity is unverified.")
        if pid:value["pid_past_steering"]=0.
        return value
    metadata=dict(schema="carla.basic_agent_initialization.v1",mode="after_bootstrap",explicit_opt_in=True,
        required_control_transport="acknowledged_batch",existing_bootstrap_tick_reused=True,additional_world_ticks=0,
        constructor_requires_verified_bootstrap=True,pid_history_overwritten=False,observed_zero_steering_required=True,
        first_proposed_steering_delta_limit=.1,comparison_absolute_tolerance=1e-6,physical_cause_proven=False,training_data_approved=False,
        construction=dict(status="PASS",before=witness(bootstrap),after=witness(bootstrap,True)),
        engagement=dict(status="PASS",first_command_acknowledged=True,before=witness(states[23],True),
            first_proposed_control=sent,first_sent_control=sent,frame_after_agent_step=1024,
            first_proposed_steering_delta=0.,first_command_receipt_sequence=27))
    receipts=[dict(reason="pre_capture_bootstrap",sequence=1,status="ACKNOWLEDGED",server_accepted=True,
        actor_id=42,response_actor_id=42,do_tick=False,before_frame=999,after_ack_frame=999),
        dict(reason="initial_drive_control",sequence=27,status="ACKNOWLEDGED",server_accepted=True,actor_id=42,response_actor_id=42,
            expected_before_frame=1024,before_frame=1024,after_ack_frame=1024,do_tick=False,physical_actuation_proven=False,requested_control=sent)]
    manifest=dict(capture_contract=dict(agent_initialization=metadata,control_transport=dict(bootstrap_observation=bootstrap)))
    return manifest,states,receipts


def test_real_manual1_constructor_engagement24_ticks_and_next_ACK_effect_are_independently_bound():
    manifest,states,receipts=initialization_fixture()
    result=audit.independent_initialization(manifest,states,receipts)
    assert result["manual_first_gear_initialization_passed"] and result["all_original_warmup_ticks_retained"]==24
    assert result["first_command_receipt_sequence"]==27 and result["first_driving_observation_frame"]==1025
    assert not result["observed_flags_or_snapshots_proxied"] and not result["physical_actuation_timing_proven"]


@pytest.mark.parametrize("field,value",[("manual_gear_shift",False),("gear",0),("gear",2),("throttle",.01),("brake",.99),("steer",.01),
    ("reverse",True),("hand_brake",True)])
def test_manual_first_gear_witness_must_match_actual_control_not_proxy(field,value):
    manifest,states,receipts=initialization_fixture()
    manifest["capture_contract"]["agent_initialization"]["construction"]["after"]["current_control"][field]=value
    with pytest.raises(Exception):audit.independent_initialization(manifest,states,receipts)


@pytest.mark.parametrize("mutation",["PID","advance","pose","ACK","observe","warmup","contract"])
def test_initialization_missing_original_guard_or_extra_tick_is_rejected(mutation):
    manifest,states,receipts=initialization_fixture();metadata=manifest["capture_contract"]["agent_initialization"]
    if mutation=="PID":metadata["construction"]["after"]["pid_past_steering"]=.1
    elif mutation=="advance":metadata["engagement"]["frame_after_agent_step"]+=1
    elif mutation=="pose":metadata["engagement"]["before"]["actor_snapshot_transform_carla"]["x"]+=.1
    elif mutation=="ACK":receipts[-1]["server_accepted"]=False
    elif mutation=="observe":states[-1]["control_transport"]["expected_receipt_sequence"]=26
    elif mutation=="warmup":states.pop(0)
    else:metadata["additional_world_ticks"]=1
    with pytest.raises(Exception):audit.independent_initialization(manifest,states,receipts)


def test_original_failed_initialization_witness_remains_failure_not_fabricated_success():
    manifest,states,receipts=initialization_fixture()
    manifest["capture_contract"]["agent_initialization"]["construction"]["status"]="FAIL"
    result=audit.independent_initialization(manifest,states,receipts)
    assert not result["manual_first_gear_initialization_passed"] and result["status"]=="FAIL"


def test_actual_spawn_reconstruction_uses_ros_to_carla_sign_before_camera_not_pixel_warp():
    center=dict(x=10.,y=20.,z=1.,yaw=0.,pitch=0.,roll=0.)
    for spec in audit.PROFILES:
        changed,rear=audit.independently_perturb(center,spec,2.85)
        assert rear["y"]==20.-spec["signed_lateral_offset_ros_m"] and changed["yaw"]==-spec["signed_yaw_offset_ros_deg"]
        assert center["y"]==20. and center["yaw"]==0.


def test_original_full_reader_ast_preserved_with_only_additional_new_gate_and_evidence():
    reader=audit.transformed_trial_reader()
    assert reader.__globals__["verify_contract"] is audit.verify_contract
    assert reader.__globals__["audit_post_ACK_recording"] is audit.writer.audit_post_ACK_recording
    assert reader.__code__.co_filename.endswith(":original_authenticator_plus_signed_evidence")
    assert reader.__globals__["independent_motion"] is audit.previous.independent_motion
    assert reader.__globals__["audit_body"] is audit.previous.audit_body


@pytest.mark.parametrize("failure",[False,True])
def test_offline_schema_reader_writer_order_bindings_restore_even_if_read_fails(failure):
    names=("SCHEMA","RECEIPT_SCHEMA","OWNER_SCHEMA","FROZEN_HELPERS","verify_contract","load_trial")
    saved={name:getattr(audit.previous,name) for name in names};cameras=audit.frozen.CAMERAS
    with mock.patch.object(audit,"verify_frozen"):
        try:
            with audit.activated():
                assert audit.previous.SCHEMA==audit.SCHEMA and audit.writer._active
                assert audit.previous.audit_post_ACK_recording is audit.writer.audit_post_ACK_recording
                with pytest.raises(Exception,match="nest"):
                    with audit.activated():pass
                if failure:raise OSError("all failure attempts retained")
        except OSError:assert failure
    assert not audit._active and not audit.writer._active and audit.frozen.CAMERAS==cameras
    assert all(getattr(audit.previous,name) is value for name,value in saved.items())


def minimal_report():
    cases=[]
    for i,spec in enumerate(audit.PROFILES):
        recovery=dict(status="NOT_PROVEN_NO_LEARNED_EFFECT",case_screen_passed=False,recovery_screen_passed=False,
            first_learned_effect=None,convergence=None,post_effect_native_count=0)
        cases.append(dict(trial_index=i,signed_recovery_profile=dict(spec),independent_signed_recovery=recovery,
            native_actor_progress_m=None,delivery_to_ACK_ms={},joint_motion_body_policy_wall100ms_passed=False))
    return dict(schema=audit.SCHEMA,signed_recovery_case_count=2,centered_setup_comparator_count=1,trials=cases,
        all_three_are_not_same_seed_repetitions=True,first_effect_is_latest_prior_actual_learned_ACK=True,
        no_teacher_recovery_counted_as_learned=True,full_MAIN9_completion_claimed=False,full_route_goal_stop_approved=False,
        all_failed_tail_retained=True,signed_recovery_pass_count=0,centered_setup_comparator_passed=False,
        joint_motion_body_policy_wall_trial_pass_count=0)


def test_failed_no_handoff_markdown_and_publication_schema_do_not_crash_or_count_center():
    report=minimal_report();publish.verify_signed_report(report)
    text=audit.markdown(report)
    assert "미측정" in text and "0/2" in text and "9개 중 0개" in text
    report["trials"][2]["independent_signed_recovery"]["recovery_screen_passed"]=True
    with pytest.raises(Exception,match="denominator"):publish.verify_signed_report(report)


@pytest.mark.parametrize("mutation",["order","seed","tail","main9","count"])
def test_publisher_requires_all_distinct_fixed_arms_and_honest_full_failed_tail(mutation):
    report=minimal_report()
    if mutation=="order":report["trials"]=report["trials"][:2]
    elif mutation=="seed":report["trials"][1]["signed_recovery_profile"]["seed"]=0
    elif mutation=="tail":report["all_failed_tail_retained"]=False
    elif mutation=="main9":report["full_MAIN9_completion_claimed"]=True
    else:report["signed_recovery_case_count"]=3
    with pytest.raises(Exception):publish.verify_signed_report(report)


@pytest.mark.parametrize("failure",[False,True])
def test_publisher_scoped_schema_audit_renderer_restore_after_validation_or_failure(failure):
    names=("audit","SCHEMA","load_archive","render")
    saved={name:getattr(publish.previous,name) for name in names}
    with mock.patch.object(audit,"verify_frozen"):
        try:
            with publish.activated():
                assert publish.previous.audit is audit and publish.previous.render is publish.render
                with pytest.raises(Exception,match="nest"):
                    with publish.activated():pass
                if failure:raise OSError("bad actual source")
        except OSError:assert failure
    assert not publish._active and all(getattr(publish.previous,name) is value for name,value in saved.items())


def test_fixed90_bins_and_last_camera_png_survive_all_outcomes_without_shortening():
    assert publish.indices()==list(range(0,450,5)) and len(publish.indices())==90
    bundle=dict(cameras=[dict(frame=i*2) for i in range(450)],trial=dict(first_latched_failure=None))
    assert set(publish.png_indices(bundle).values())=={0,112,224,337,449}
    bundle["trial"]["first_latched_failure"]=dict(outgoing_command_frame=899,first_observed_emergency_frame=None)
    indices=publish.png_indices(bundle)
    assert indices["05_last.png"]==449 and indices["03_no_camera_at_terminal_rejection.png"]==449


def test_frozen_actual_sources_are_exact_and_auditor_imports_no_model_or_owner():
    audit.verify_frozen()
    for node in ast.walk(ast.parse(Path(audit.__file__).read_bytes())):
        if isinstance(node,ast.Import):assert not any(n.name.startswith(("torch","carla")) for n in node.names)
        elif isinstance(node,ast.ImportFrom):assert not (node.module or "").startswith(("torch","portable_e2e","research_","collect_"))
