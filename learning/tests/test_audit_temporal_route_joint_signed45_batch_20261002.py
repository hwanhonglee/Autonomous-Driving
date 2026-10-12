"""HH_261002 - Test pure final6000 metadata, preserved physical authentication, genuine parent comparison and restored publication scopes."""

from copy import deepcopy
from pathlib import Path
from unittest import mock

import pytest

from scripts.e2e import audit_temporal_route_joint_signed45_batch_20261002 as audit
from scripts.e2e import publish_temporal_route_joint_signed45_batch_20261002 as publish


def endpoint():return dict(weights_sha256="a"*64,metadata_sha256="b"*64,checkpoint_sha256="c"*64,declaration_sha256="d"*64)


def metadata():
    keys=sorted(["fusion.param_%02d"%i for i in range(54)]+["image_encoder.param_%02d"%i for i in range(47)])
    sources={"tools/e2e/train_temporal_route_joint_20261002.py":"a"*64,
        "learning/portable_e2e/temporal_route_joint_export_research_20261002.py":"b"*64,
        "tools/e2e/train_temporal_route_recovery_moving_v4_20261001.py":"c"*64,
        "learning/portable_e2e/temporal_route_recovery_v3_20261001.py":"d"*64}
    sources.update({"learning/portable_e2e/source_%02d.py"%i:"e"*64 for i in range(22)})
    return dict(schema=audit.EXPORT_SCHEMA,training_schema=audit.TRAINING_SCHEMA,architecture_schema=audit.ARCHITECTURE_SCHEMA,model_id=audit.MODEL_ID,
        optimizer_updates=6000,additional_optimizer_updates=6000,maximum_optimizer_updates=6000,effective_batch_size=4,examples_seen=24000,
        recovery_examples_seen=12000,v4_examples_seen=6000,legacy_examples_seen=6000,training_episode_count=18,
        recovery_training_episode_count=9,replay_training_episode_count=9,holdout_episode_count=2,parent_optimizer_updates=3000,
        parent_known_total_optimizer_updates=6200,known_total_optimizer_updates=12200,trainable_state_tensor_count=54,frozen_state_tensor_count=47,
        trainable_parameter_count=2277827,training_only=True,simulation_only=True,model_promoted=False,vehicle_control_approved=False,
        inference_loader_authorized=False,stop_intent_trained=False,backbone_frozen=False,visual_encoder_frozen=True,
        semantic_head_parameters_frozen=True,semantic_outputs_preserved=False,legacy_decoded=False,independent_validation=False,
        checkpoint_selection_by_holdout=False,labels_clipped=False,labels_extrapolated=False,startup_recovery_fitted=False,
        batch_domains=list(audit.BATCH_DOMAINS),runtime_intent="explicit_cruise_GO_not_head",
        parent_checkpoint_sha256=audit.PARENT_CHECKPOINT_SHA,parent_npz_sha256=audit.PARENT_NPZ_SHA,
        parent_metadata_sha256=audit.PARENT_METADATA_SHA,parent_declaration_sha256=audit.PARENT_DECLARATION_SHA,
        learning_rate=dict(schedule="fixed6000_linear_warmup_cosine_two_groups",warmup_updates=300,
            shared_peak=1e-5,shared_final=1e-6,heads_peak=3e-5,heads_final=3e-6,gradient_clip_l2=5.,weight_decay=1e-4,units="AdamW optimizer learning rate"),
        checkpoint_sha256="c"*64,npz_sha256="a"*64,declaration_sha256="d"*64,frozen_state_sha256="e"*64,
        training_dataset_fingerprint_sha256="f"*64,holdout_dataset_fingerprint_sha256="9"*64,
        numerical_revision=dict(kernel_id=audit.KERNEL_ID,strict_deterministic_algorithms=True,
            tf32_matrix_multiplication=False,finite_precision_bitwise_equivalence_claimed=False),source_sha256=sources,
        new_signed_recovery_cohort_moving_only=True,fixed_recovery_native_frame_offsets=[80,682],
        fixed_recovery_driving_relative_window_seconds=[4.,34.1],current_training_purpose="causal_basic_driving_joint_curvature_speed_continuation",
        state_tensor_shapes={k:[1] for k in keys},state_tensor_dtypes={k:"float32" for k in keys},
        trainable_state_names=sorted(k for k in keys if k.startswith(audit.TRAINABLE_PREFIXES)),
        frozen_state_names=sorted(k for k in keys if not k.startswith(audit.TRAINABLE_PREFIXES)))


def export_plan(document):
    return dict(source_checkpoint_sha256="c"*64,new_fixed_endpoint_export_sha256=dict(weights="a"*64,metadata="b"*64),
        source_sha256=document["source_sha256"],asset_sha256={str(audit.PARENT/"model_state.npz"):audit.PARENT_NPZ_SHA,
            str(audit.PARENT/"metadata.json"):audit.PARENT_METADATA_SHA,str(audit.BACKBONE/"model_state.npz"):audit.BACKBONE_NPZ_SHA,
            str(audit.BACKBONE/"metadata.json"):audit.BACKBONE_METADATA_SHA})


def test_exact_pure_numeric_protocol_without_torch_or_pickle_accepts_only_final6000_partition():
    document=metadata();validator=audit.explicit_numeric_protocol()
    validator(document)
    assert validator.__code__.co_filename.endswith(":exact_pure_joint_metadata_protocol")
    audit.verify_export(document,export_plan(document),caller_pins=endpoint())


@pytest.mark.parametrize("field,value",[("optimizer_updates",5999),("known_total_optimizer_updates",6200),
    ("trainable_state_tensor_count",4),("frozen_state_tensor_count",97),("backbone_frozen",True),
    ("semantic_outputs_preserved",True),("stop_intent_trained",True),("vehicle_control_approved",True),
    ("labels_clipped",True),("checkpoint_selection_by_holdout",True),("schema",audit.previous.previous.EXPORT_SCHEMA),
    ("parent_npz_sha256","0"*64),("runtime_intent","trained_stop_head"),("fixed_recovery_native_frame_offsets",[18,620])])
def test_joint_schema_cannot_masquerade_as_old_frozen97_or_weaken_scope(field,value):
    document=metadata();document[field]=value
    with pytest.raises(Exception):audit.verify_export(document,export_plan(document),caller_pins=endpoint())


@pytest.mark.parametrize("mutation",["missing_final","wrong_final","missing_parent","wrong_parent","source26","partition","dtype","shape"])
def test_four_endpoint_identities_both_parent_pairs_and_all101_ABI_are_required(mutation):
    document=metadata();plan=export_plan(document);pins=endpoint()
    if mutation=="missing_final":pins.pop("declaration_sha256")
    elif mutation=="wrong_final":pins["checkpoint_sha256"]="0"*64
    elif mutation=="missing_parent":plan["asset_sha256"].pop(str(audit.BACKBONE/"model_state.npz"))
    elif mutation=="wrong_parent":plan["asset_sha256"][str(audit.PARENT/"metadata.json")]="0"*64
    elif mutation=="source26":document["source_sha256"].pop("learning/portable_e2e/source_00.py")
    elif mutation=="partition":document["frozen_state_names"]=document["trainable_state_names"]
    elif mutation=="dtype":document["state_tensor_dtypes"]["fusion.param_00"]="float64"
    else:document["state_tensor_shapes"]["fusion.param_00"]=[True]
    with pytest.raises(Exception):audit.verify_export(document,plan,caller_pins=pins)


@pytest.mark.parametrize("failure",[False,True])
def test_all_joint_schema_contract_reader_and_export_bindings_restore(failure):
    names=("SCHEMA","RECEIPT_SCHEMA","OWNER_SCHEMA","COLLECTOR_SCHEMA","FROZEN_HELPERS","NEW_FROZEN","verify_contract","transformed_trial_reader")
    saved={k:getattr(audit.previous,k) for k in names};old=audit.previous.previous.verify_export
    with mock.patch.object(audit,"verify_frozen"):
        try:
            with audit.activated(endpoint()):
                assert audit.previous.verify_contract is audit.verify_contract and audit.previous.previous.verify_export is audit.verify_export
                with pytest.raises(Exception,match="nest"):
                    with audit.activated(endpoint()):pass
                if failure:raise OSError("all failed outcomes kept")
        except OSError:assert failure
    assert all(getattr(audit.previous,k) is v for k,v in saved.items()) and audit.previous.previous.verify_export is old
    assert not audit._active and audit._endpoint is None


def test_reader_reuses_full_signed_authentication_and_changes_only_truthful_metadata_label():
    with mock.patch.object(audit,"verify_frozen"),audit.activated(endpoint()):
        reader=audit.transformed_trial_reader()
        assert reader.__globals__["verify_contract"] is audit.verify_contract
        assert reader.__globals__["verify_export"] is audit.verify_export
        assert reader.__globals__["independent_motion"] is audit.previous.previous.independent_motion
        assert reader.__globals__["audit_body"] is audit.previous.previous.audit_body
        assert "current_training_purpose" in reader.__code__.co_consts and "recovery_data_revision" not in reader.__code__.co_consts


def trial(index,*,candidate=False):
    return dict(trial_index=index,signed_recovery_profile=dict(audit.PROFILES[index]),
        source_checkpoint_sha256="c"*64 if candidate else audit.PARENT_CHECKPOINT_SHA,
        fixed_export_sha256=dict(weights="a"*64,metadata="b"*64) if candidate else dict(weights=audit.PARENT_NPZ_SHA,metadata=audit.PARENT_METADATA_SHA),
        independent_signed_recovery=dict(recovery_screen_passed=index!=2,centered_setup_screen_passed=index==2,
            convergence=dict(first_converged_simulation_seconds=6.,continuous_converged_seconds=2.)),
        joint_motion_body_policy_wall100ms_passed=False,native_actor_progress_m=92.+index,
        post_handoff_base_norm_maximum_mps=2.51,post_handoff_actor_CTE_p95_m=.1,post_handoff_actor_CTE_maximum_m=.2,
        delivery_to_ACK_ms=dict(maximum=98.),first_latched_failure=None,fresh_policy100ms_passed=True,original_body_screen_passed=True)


def baseline_report():
    return dict(schema=audit._baseline_schema,batch=str(audit.BASELINE_BATCH),prospective_trials=3,all_failed_tail_retained=True,
        trials=[trial(i) for i in range(3)],source_sha256={},joint_motion_body_policy_wall_trial_pass_count=0)


def test_parent_comparison_preserves_all_three_real_failures_under_same_profiles_not_offline_loss(tmp_path):
    path=tmp_path/"baseline.json";baseline=baseline_report();new=dict(trials=[trial(i,candidate=True) for i in range(3)],joint_motion_body_policy_wall_trial_pass_count=0)
    caller=dict(baseline_report=dict(path=str(path),sha256="a"*64))
    with mock.patch.object(audit,"ROOT",tmp_path),mock.patch.object(audit,"add_pin"),mock.patch.object(audit,"read_json",return_value=baseline):
        result=audit.parent_comparison(new,caller,{})
    assert result["baseline_joint_pass_count"]==result["candidate_joint_pass_count"]==0
    assert len(result["per_arm"])==3 and result["all_three_parent_and_candidate_attempts_retained"]
    assert not result["main9_or_generalized_performance_equivalence_claimed"]


@pytest.mark.parametrize("mutation",["two_of_three","seed","parent","tail","corridor"])
def test_comparison_rejects_selected_parent_runs_or_wrong_setup(mutation,tmp_path):
    baseline=baseline_report();new=dict(trials=[trial(i,candidate=True) for i in range(3)],joint_motion_body_policy_wall_trial_pass_count=0)
    if mutation=="two_of_three":baseline["trials"].pop()
    elif mutation=="seed":baseline["trials"][0]["signed_recovery_profile"]["seed"]=0
    elif mutation=="parent":baseline["trials"][0]["source_checkpoint_sha256"]="0"*64
    elif mutation=="tail":baseline["all_failed_tail_retained"]=False
    else:baseline["batch"]=str(tmp_path/"selected_good_run")
    caller=dict(baseline_report=dict(path=str(tmp_path/"report.json"),sha256="a"*64))
    with mock.patch.object(audit,"ROOT",tmp_path),mock.patch.object(audit,"add_pin"),mock.patch.object(audit,"read_json",return_value=baseline),pytest.raises(Exception):
        audit.parent_comparison(new,caller,{})


def publication_report():
    result=dict(schema=audit.SCHEMA,signed_recovery_case_count=2,centered_setup_comparator_count=1,trials=[trial(i,candidate=True) for i in range(3)],
        all_three_are_not_same_seed_repetitions=True,first_effect_is_latest_prior_actual_learned_ACK=True,no_teacher_recovery_counted_as_learned=True,
        full_MAIN9_completion_claimed=False,full_route_goal_stop_approved=False,all_failed_tail_retained=True,signed_recovery_pass_count=2,
        optimizer_updates=6000,known_total_optimizer_updates=12200,trainable_state_tensor_count=54,frozen_state_tensor_count=47,
        original_backbone_weights_byte_unchanged=False,semantic_outputs_preserved=False,visual_encoder_frozen=True,
        semantic_head_parameters_frozen=True,original47_bytefixed_check_enforced_by_pinned_numeric_loader=True,
        all101_tensor_values_independently_reloaded_by_this_audit=False,model_change_is_learned_weights_only_not_teacher_controller_or_threshold_change=True,
        no_torch_pickle_loaded=True,final_endpoint_sha256=endpoint(),joint_motion_body_policy_wall_trial_pass_count=0)
    result["parent_comparison"]=dict(baseline_checkpoint_sha256=audit.PARENT_CHECKPOINT_SHA,all_three_parent_and_candidate_attempts_retained=True,
        same_actual_profiles_seeds_teacher_PI_and_thresholds=True,per_arm=[],baseline_joint_pass_count=0,candidate_joint_pass_count=0)
    for i in range(3):result["parent_comparison"]["per_arm"].append(dict(case_id=audit.PROFILES[i]["case_id"],baseline=dict(progress_m=92.,base_native_speed_maximum_mps=2.51,
        observed_delivery_to_ACK_ms=dict(maximum=98.),joint_all_physical_gates_passed=False),joint_candidate=dict(progress_m=92.,base_native_speed_maximum_mps=2.51,
        observed_delivery_to_ACK_ms=dict(maximum=98.),joint_all_physical_gates_passed=False)))
    return result


def test_publication_truthfully_distinguishes54learning47fixed_from_oldfrozen97():
    report=publication_report()
    with mock.patch.object(publish.previous,"audit",audit):publish.verify_joint_report(report)
    text=audit.markdown(report)
    assert "54개 tensor" in text and "47개" in text and "FAIL → FAIL" in text and "0/9" in text
    report["original_frozen97_check_enforced_by_pinned_numeric_loader"]=True
    with mock.patch.object(publish.previous,"audit",audit),pytest.raises(Exception,match="frozen97"):publish.verify_joint_report(report)


@pytest.mark.parametrize("failure",[False,True])
def test_joint_publisher_scopes_restore_baseline_globals_even_when_failed_media_validation(failure):
    names=("audit","SCHEMA","render","load_archive")
    saved={k:getattr(publish.previous,k) for k in names}
    with mock.patch.object(audit,"verify_frozen"):
        try:
            with publish.activated():
                assert publish.previous.audit is audit and publish.previous.render is publish.render
                assert publish.previous.previous.audit is audit
                with pytest.raises(Exception,match="nest"):
                    with publish.activated():pass
                if failure:raise OSError("rejected actual source")
        except OSError:assert failure
    assert all(getattr(publish.previous,k) is v for k,v in saved.items()) and not publish._active and not publish.previous._active


def test_all_gif_bins_and_last_png_are_inherited_without_threshold_or_tail_change():
    assert publish.indices()==list(range(0,450,5))
    assert publish.png_indices is publish.previous.png_indices and publish.displayed_receipt is publish.previous.displayed_receipt
