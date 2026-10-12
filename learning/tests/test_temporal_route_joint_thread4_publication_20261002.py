"""HH_261002 - Mock-only proof of strict thread4 archive/publisher boundaries and every retained negative outcome."""

from contextlib import contextmanager
import copy
from pathlib import Path
from unittest import mock

import pytest

from scripts.e2e import audit_temporal_route_joint_thread4_signed45_batch_20261002 as audit
from scripts.e2e import publish_temporal_route_joint_thread4_signed45_20261002 as publish


def endpoint():
    return {name+"_sha256":letter*64 for name,letter in (("weights","a"),("metadata","b"),("checkpoint","c"),("declaration","d"))}


def thread_fixture():
    settings=[];timing=[]
    for frame in range(3):
        settings.append(dict(schema=audit.THREAD_SCHEMA,command_frame=frame,command_reason="next_driving_control",
            intra_op_threads=4,inter_op_threads=1,before_original_propose=True,observed_monotonic_ns=101+frame*10))
        timing.append(dict(command_frame=frame,command_reason="next_driving_control",proposal_started_monotonic_ns=100+frame*10,
            proposal_finished_monotonic_ns=102+frame*10,ACK_started_monotonic_ns=103+frame*10,ACK_finished_monotonic_ns=104+frame*10))
    preset=dict(schema=audit.THREAD_SCHEMA,intra_op_threads=4,inter_op_threads=1,strict_original_offline_parity_passed=False,
        exact_bitwise_or_strict_decoded_equivalence_claimed=False,readback_journal="cpu_thread_readback.jsonl",
        every_normal_proposal_checked_before_handoff=True,hard_policy_deadline_seconds=.1,observed_bundle_to_ACK_deadline_seconds=.1,
        no_safety_gate_or_prediction_replacement=True,model_promoted=False,vehicle_control_approved=False)
    return settings,timing,dict(capture_contract=dict(cpu_thread_numerical_preset=preset))


@pytest.mark.parametrize("change",[None,"missing_held","duplicate","wrong_threads","Boolean_threads","outside_clock","empty"])
def test_readback_full_normal_denominator_independent_including_failed_commands(tmp_path,change):
    settings,timing,manifest=thread_fixture()
    if change=="missing_held":settings.pop(1)
    elif change=="duplicate":settings.append(settings[-1])
    elif change=="wrong_threads":settings[1]["intra_op_threads"]=1
    elif change=="Boolean_threads":settings[1]["inter_op_threads"]=True
    elif change=="outside_clock":settings[1]["observed_monotonic_ns"]=0
    elif change=="empty":settings=[];timing=[]
    diagnostic=dict(actual_thread_settings=dict(passed=change is None,normal_proposal_denominator=len(timing),readback_count=len(settings)))
    pins={}
    with mock.patch.object(audit,"rows",side_effect=lambda p:settings if p.name=="cpu_thread_readback.jsonl" else timing), \
         mock.patch.object(audit,"add_pin",side_effect=lambda p,path:p.update({str(path):"e"*64})):
        result=audit.independent_thread_readback(tmp_path,manifest,diagnostic,pins)
    assert result["passed"] is (change is None)
    assert result["readback_count"]==len(settings)
    assert str(tmp_path/"cpu_thread_readback.jsonl") in pins


@pytest.mark.parametrize("change",["owner_false","owner_missing","owner_wrong_count","manifest_parity_true","deadline_relaxed"])
def test_source_claims_cannot_replace_independent_readback(tmp_path,change):
    settings,timing,manifest=thread_fixture()
    reported=dict(passed=True,normal_proposal_denominator=3,readback_count=3)
    if change=="owner_false":reported["passed"]=False
    elif change=="owner_missing":reported={}
    elif change=="owner_wrong_count":reported["readback_count"]=2
    elif change=="manifest_parity_true":manifest["capture_contract"]["cpu_thread_numerical_preset"]["strict_original_offline_parity_passed"]=True
    else:manifest["capture_contract"]["cpu_thread_numerical_preset"]["hard_policy_deadline_seconds"]=.101
    with mock.patch.object(audit,"rows",side_effect=lambda p:settings if p.name=="cpu_thread_readback.jsonl" else timing), \
         mock.patch.object(audit,"add_pin",side_effect=lambda p,path:p.update({str(path):"e"*64})),pytest.raises(Exception):
        audit.independent_thread_readback(tmp_path,manifest,dict(actual_thread_settings=reported),{})


def test_original_reader_AST_removes_only_new_evidence_and_metadata_label():
    reader=audit.transformed_trial_reader()
    assert reader.__name__=="load_trial"
    assert reader.__globals__["verify_contract"] is audit.verify_contract
    assert reader.__globals__["extra_trial_evidence"] is audit.extra_trial_evidence
    assert "torch" not in reader.__globals__


def test_only_extra_thread_witness_augments_original_independent_spawn_recovery():
    args=tuple(range(9));original={"independent_actual_spawn":{"actual_signed_spawn_identity_passed":True}}
    with mock.patch.object(audit,"_extra",return_value=original) as old, \
         mock.patch.object(audit,"independent_thread_readback",return_value={"passed":False}) as thread:
        result=audit.extra_trial_evidence(*args)
    old.assert_called_once_with(*args)
    thread.assert_called_once_with(8,0,6,7)
    assert result["independent_actual_spawn"]==original["independent_actual_spawn"]
    assert not result["independent_cpu_thread_preset"]["passed"]


@pytest.mark.parametrize("failure",[False,True])
def test_auditor_scope_restores_frozen_names_even_on_exception(failure):
    names=("SCHEMA","RECEIPT_SCHEMA","OWNER_SCHEMA","COLLECTOR_SCHEMA","FROZEN_HELPERS","NEW_FROZEN","verify_contract","transformed_trial_reader")
    saved={name:getattr(audit.previous,name) for name in names}
    @contextmanager
    def delegated(pin):
        assert pin==endpoint() and audit.previous.SCHEMA==audit.SCHEMA
        assert audit.previous.transformed_trial_reader is audit.transformed_trial_reader
        if failure:raise OSError("preserved")
        yield
    with mock.patch.object(audit,"verify_frozen"),mock.patch.object(audit.previous,"activated",side_effect=delegated):
        try:
            with audit.activated(endpoint()):
                with pytest.raises(Exception):
                    with audit.activated(endpoint()):pass
        except OSError:assert failure
    assert all(getattr(audit.previous,name) is value for name,value in saved.items())
    assert not audit._active


def report_fixture():
    trials=[]
    for i,spec in enumerate(audit.PROFILES):
        trials.append(dict(trial_index=i,signed_recovery_profile=spec,source_checkpoint_sha256="c"*64,
            fixed_export_sha256=dict(weights="a"*64,metadata="b"*64),joint_motion_body_policy_wall100ms_passed=False,
            independent_cpu_thread_preset=dict(passed=True),native_actor_progress_m=None if i==0 else 89.,
            post_handoff_base_norm_maximum_mps=None if i==0 else 2.4,post_handoff_actor_CTE_p95_m=None if i==0 else .03,
            post_handoff_actor_CTE_maximum_m=None if i==0 else .1,delivery_to_ACK_ms=dict(maximum=None if i==0 else 90.),
            fresh_policy100ms_passed=False if i==0 else True,original_body_screen_passed=True,
            independent_signed_recovery=dict(recovery_screen_passed=False,case_screen_passed=False)))
    return dict(schema=audit.SCHEMA,batch="batch",prospective_trials=3,trials=trials,all_failed_tail_retained=True,
        signed_recovery_case_count=2,centered_setup_comparator_count=1,signed_recovery_pass_count=0,
        all_three_are_not_same_seed_repetitions=True,first_effect_is_latest_prior_actual_learned_ACK=True,
        no_teacher_recovery_counted_as_learned=True,full_MAIN9_completion_claimed=False,full_route_goal_stop_approved=False,
        joint_motion_body_policy_wall_trial_pass_count=0,source_sha256={},final_endpoint_sha256=endpoint(),
        optimizer_updates=6000,known_total_optimizer_updates=12200,trainable_state_tensor_count=54,frozen_state_tensor_count=47,
        original_backbone_weights_byte_unchanged=False,semantic_outputs_preserved=False,visual_encoder_frozen=True,
        semantic_head_parameters_frozen=True,original47_bytefixed_check_enforced_by_pinned_numeric_loader=True,
        all101_tensor_values_independently_reloaded_by_this_audit=False,no_torch_pickle_loaded=True,
        model_weights_and_teacher_controller_unchanged=True,CPU_thread_numerical_preset_changed=True,
        model_change_is_learned_weights_only_not_teacher_controller_or_threshold_change=False,
        cpu_thread_numerical_preset=dict(strict_original_offline_parity_passed=False),all_three_complete_thread_readbacks_passed=True)


def comparison_fixture(report):
    return dict(baseline_report_sha256=audit.BASELINE_SHA,baseline_joint_pass_count=0,thread4_joint_pass_count=0,
        same_fixed_final6000_weights_profiles_seeds_teacher_PI_and_thresholds=True,
        difference_is_CPU_thread_numerical_preset_not_new_weights=True,strict_offline_parity_passed=False,
        all_three_parent_and_candidate_attempts_retained=True,
        per_arm=[dict(trial_index=row["trial_index"],case_id=row["signed_recovery_profile"]["case_id"],baseline=row,thread4=row) for row in report["trials"]])


@pytest.mark.parametrize("change",[None,"parity_true","new_weights_claim","wrong_updates","thread_denominator","missing_case"])
def test_publisher_requires_truthful_new_thread_scope_with_all_negative_outcomes(change):
    report=report_fixture();report["parent_comparison"]=comparison_fixture(report)
    if change=="parity_true":report["cpu_thread_numerical_preset"]["strict_original_offline_parity_passed"]=True
    elif change=="new_weights_claim":report["model_change_is_learned_weights_only_not_teacher_controller_or_threshold_change"]=True
    elif change=="wrong_updates":report["optimizer_updates"]=5999
    elif change=="thread_denominator":report["all_three_complete_thread_readbacks_passed"]=False
    elif change=="missing_case":report["trials"].pop()
    with mock.patch.object(audit,"pure_preset",return_value=(dict(strict_original_offline_parity_passed=False),{})), \
         mock.patch.object(publish.previous.previous,"audit",audit):
        if change is None:publish.verify_joint_report(report)
        else:
            with pytest.raises(Exception):publish.verify_joint_report(report)


def test_markdown_retains_no_first_effect_as_unmeasured_not_zero():
    report=report_fixture();report["parent_comparison"]=comparison_fixture(report)
    text=audit.markdown(report)
    assert "미측정" in text and "CPU1 0/3 → CPU4 0/3" in text and "0/9" in text and "FALSE" in text


@pytest.mark.parametrize("failure",[False,True])
def test_publication_scope_binds_exact_missing_alias_and_restores_every_old_name(failure):
    assert audit.measured_rear_errors is audit.previous.previous.measured_rear_errors
    names=("audit","SCHEMA","verify_joint_report","render","load_archive")
    saved={name:getattr(publish.previous,name) for name in names}
    with mock.patch.object(audit,"verify_frozen"),mock.patch.object(audit,"digest",return_value=publish.PUBLISHER_SHA):
        try:
            with publish.activated():
                assert publish.previous.audit is audit and publish.previous.verify_joint_report is publish.verify_joint_report
                with pytest.raises(Exception):
                    with publish.activated():pass
                if failure:raise OSError("retain")
        except OSError:assert failure
    assert all(getattr(publish.previous,name) is value for name,value in saved.items())
    assert not publish._active


def test_frozen_whole45s_GIF_and_five_PNG_indices_are_not_trimmed():
    assert publish.indices() == publish.previous.indices()
    assert len(publish.indices()) == 90 and publish.indices()[-1] == 445
    bundle=dict(cameras=[dict(frame=i) for i in range(450)],trial=report_fixture()["trials"][1])
    assert publish.png_indices(bundle) == publish.previous.png_indices(bundle)
    assert publish.png_indices(bundle)["05_last.png"] == 449
