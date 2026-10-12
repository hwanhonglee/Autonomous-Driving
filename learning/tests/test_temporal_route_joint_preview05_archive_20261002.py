"""HH_261002 - Synthetic-only proof of0.5 scalar scope, allbracket failures and restored full-duration publication."""

import ast
from contextlib import ExitStack, contextmanager
from copy import deepcopy
import json
from pathlib import Path
from unittest import mock

import pytest

from scripts.e2e import audit_temporal_route_joint_preview05_signed45_batch_20261002 as audit
from scripts.e2e import publish_temporal_route_joint_preview05_signed45_20261002 as publish


def endpoint():
    return {name+"_sha256":letter*64 for name,letter in (("weights","a"),("metadata","b"),("checkpoint","c"),("declaration","d"))}


def config():
    return dict(wheelbase_m=2.819186218917395,maximum_steer_angle_rad=.61,speed_cap_mps=2.5,
        minimum_lookahead_m=1.5,lookahead_time_s=.8,speed_preview_s=.5,throttle_gain=.7,brake_gain=.7,maximum_throttle=.5)


def selected(old=2.1,new=2.2,**extras):
    speeds=[2.3]*64; speeds[2]=old; speeds[4]=new; speeds[7]=2.4
    return dict(candidate_index=0,xy_base_m=[[.25*(i+1),.003*(i+1)] for i in range(64)],speed_mps=speeds,timestep_s=.1,**extras)


@pytest.mark.parametrize("old,new,intent,guard",[(2.1,2.2,None,False),(.05,2.2,None,True),(2.1,.05,None,True),
    (0.,2.2,None,True),(2.1,2.2,1,True),(2.1,2.2,True,True),(2.1,2.2,0,False)])
def test_exact0p5_knot_and_whole_original0p3_STOP_or_nonGO(old,new,intent,guard):
    plan=selected(old,new,**({} if intent is None else {"intent":intent})); original=deepcopy(plan)
    receipt=audit.preview_targets(plan,config())
    assert receipt["original_preview_index"]==2 and receipt["proposed_preview_index"]==4
    assert receipt["configured_speed_preview_s"]==.5 and receipt["effective_speed_preview_s"]==(.3 if guard else .5)
    assert receipt["effective_selected_target_mps"]==(old if guard else new) and receipt["original_STOP_tracker_preserved"] is guard
    assert plan==original and config()["lookahead_time_s"]==.8


@pytest.mark.parametrize("name",["preview_targets","preview_census","extra_trial_evidence","verify_contract"])
def test_inverse_AST_scalar_only_adapters_keep_exact_oldstatements(name):
    function=audit.transformed_definition(name)
    assert function.__name__==name
    if name=="preview_census":
        assert .5 in function.__code__.co_consts and .8 in function.__code__.co_consts
    if name=="verify_contract": assert function.__globals__["NEW_FROZEN"]==audit.NEW_FROZEN


@pytest.mark.parametrize("mutation",["extra_speed_scalar","wrong_speed_keyword"])
def test_unknown_AST_revision_is_rejected_not_broadly_replaced(mutation):
    original=ast.parse
    def changed_parse(source,*args,**kwargs):
        tree=original(source,*args,**kwargs)
        if isinstance(source,bytes) and b"Independently verify causal preview08" in source:
            function=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name==("preview_targets" if mutation=="extra_speed_scalar" else "preview_census"))
            if mutation=="extra_speed_scalar":function.body.append(ast.Expr(value=ast.Constant(value=.8)))
            else:next(n for n in ast.walk(function) if isinstance(n,ast.keyword) and n.arg=="speed_preview_s").value.value=.7
        return tree
    with mock.patch.object(audit.ast,"parse",side_effect=changed_parse),pytest.raises(Exception):
        audit.transformed_definition("preview_targets" if mutation=="extra_speed_scalar" else "preview_census")


def preview_fixture():
    # HH_261002 - Reuse only the frozen synthetic fixture body under the NEW0.5 pure arithmetic namespace.
    path=audit.ROOT/"learning/tests/test_temporal_route_joint_preview08_archive_20261002.py"
    tree=ast.parse(path.read_bytes()); function=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=="preview_fixture")
    namespace=dict(audit=audit,deepcopy=deepcopy,config=config,selected=selected)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[function],type_ignores=[])),"unchanged_synthetic_census_fixture","exec"),namespace)
    return namespace["preview_fixture"]()


def test_allfresh_held_STOP_and_rejected_PI_chronology_uses0p5_withoutadditionalintegrations():
    result=audit.preview_census(*preview_fixture())
    assert result["normal_proposal_denominator"]==8 and result["fresh_tracker_count"]==result["independent_PI_update_count"]==4
    assert result["held_ACK_without_new_PI_count"]==2 and result["original_STOP_or_non_GO_fallback_count"]==1
    assert result["actual_controller"]["speed_preview_s"]==.5 and result["actual_controller"]["lookahead_time_s"]==.8


@pytest.mark.parametrize("mutation",["drop_held","old_speed","lateral","candidate_axis","selected","knot","PI","held_PI","ACK","v0","empty"])
def test_controllerproof_failures_cannot_be_hidden_in_newscalaradapter(mutation):
    values=list(preview_fixture());records,timing,commands,receipts,states=values
    if mutation=="drop_held":records.pop(2)
    elif mutation=="old_speed":records[1]["actual_policy_controller"]["speed_preview_s"]=.8
    elif mutation=="lateral":records[1]["actual_policy_controller"]["lookahead_time_s"]=.5
    elif mutation=="candidate_axis":commands[1]["decoded_output"]["candidate_speed"].append([1.]*64)
    elif mutation=="selected":commands[1]["decoded_output"]["candidate_speed"][0][4]+=.01
    elif mutation=="knot":records[1]["tracker_receipt"]["proposed_preview_index"]=7
    elif mutation=="PI":commands[3]["longitudinal_pi_research"]["integral_before"]+=.01
    elif mutation=="held_PI":commands[2]["longitudinal_pi_research"]["integral_after"]+=.01
    elif mutation=="ACK":receipts[1]["requested_control"]["throttle"]+=2.e-6
    elif mutation=="v0":states[1]["vx"]+=1.e-4
    else:values=[[],[],[],[],[]]
    with pytest.raises(Exception):audit.preview_census(*values)


@pytest.mark.parametrize("failure",[False,True])
def test_newscope_restores_each0p8_global_on_error_and_does_not_change_originalreader(failure):
    names=("SCHEMA","RECEIPT_SCHEMA","OWNER_SCHEMA","COLLECTOR_SCHEMA","PREVIEW_SCHEMA","NEW_FROZEN","FROZEN_HELPERS", "verify_frozen", "calibration_contract", "preview_targets", "preview_census", "extra_trial_evidence", "verify_contract")
    saved={k:getattr(audit.previous,k) for k in names}; oldreader=audit.signed.previous.independent_motion
    with mock.patch.object(audit,"verify_frozen"):
        try:
            with audit.activated(({},{})):
                assert audit.previous.SCHEMA==audit.SCHEMA and audit.previous.preview_census is audit.preview_census
                assert audit.previous.transformed_trial_reader is not audit.extra_trial_evidence
                with pytest.raises(Exception):
                    with audit.activated(({},{})): pass
                if failure: raise OSError("retain allfailures")
        except OSError: assert failure
    assert all(getattr(audit.previous,k) is v for k,v in saved.items())
    assert not audit._active and audit._prior is None and audit.signed.previous.independent_motion is oldreader


def metrics(progress=89.,speed=2.45,passed=False):
    return dict(joint_motion_body_policy_wall100ms_passed=passed,native_actor_progress_m=progress,
        post_handoff_base_norm_maximum_mps=speed,post_handoff_actor_CTE_p95_m=.2,post_handoff_actor_CTE_maximum_m=.25,
        delivery_to_ACK_ms=dict(maximum=80.),fresh_policy100ms_passed=True,original_body_screen_passed=True)


def publication_report():
    trials=[]; oldtrials=[]; parent=[]
    for i,spec in enumerate(audit.PROFILES):
        trial=dict(metrics(91.,2.48,True),trial_index=i,signed_recovery_profile=deepcopy(spec),
            independent_cpu_thread_preset=dict(passed=True),independent_preview_configuration=dict(passed=True),
            independent_additive_native_acceleration=dict(passed=True,native_speed_rate_outside_minus6_plus3_count=0),
            independent_signed_recovery=dict(case_screen_passed=True,recovery_screen_passed=i<2))
        trials.append(trial); old=dict(metrics(93.,2.516,False),independent_additive_native_acceleration=deepcopy(trial["independent_additive_native_acceleration"])); oldtrials.append(old)
        parent.append(dict(trial_index=i,case_id=spec["case_id"],baseline=metrics(),preview08={k:trial[k] for k in audit.METRICS},
            baseline_additive_native_rate=deepcopy(trial["independent_additive_native_acceleration"]),candidate_additive_native_rate=deepcopy(trial["independent_additive_native_acceleration"])))
    report=dict(schema=audit.SCHEMA,trials=trials,prospective_trials=3,all_failed_tail_retained=True,signed_recovery_case_count=2,
        centered_setup_comparator_count=1,signed_recovery_pass_count=2,all_three_are_not_same_seed_repetitions=True,
        first_effect_is_latest_prior_actual_learned_ACK=True,no_teacher_recovery_counted_as_learned=True,
        full_MAIN9_completion_claimed=False,full_route_goal_stop_approved=False,joint_motion_body_policy_wall_trial_pass_count=3,
        optimizer_updates=6000,known_total_optimizer_updates=12200,trainable_state_tensor_count=54,frozen_state_tensor_count=47,
        original_backbone_weights_byte_unchanged=False,semantic_outputs_preserved=False,visual_encoder_frozen=True,semantic_head_parameters_frozen=True,
        original47_bytefixed_check_enforced_by_pinned_numeric_loader=True,all101_tensor_values_independently_reloaded_by_this_audit=False,
        no_torch_pickle_loaded=True,model_weights_unchanged=True,teacher_startup_and_PI2p4_parameters_unchanged=True,
        CPU_thread_numerical_preset_changed=False,learned_tracker_speed_preview_changed=True,
        model_change_is_learned_weights_only_not_teacher_controller_or_threshold_change=False,
        final_endpoint_sha256=endpoint(),cpu_thread_numerical_preset=audit.thread.pure_preset()[0],
        causal_selected_speed_preview_calibration=audit.calibration_contract(),all_three_complete_thread_readbacks_passed=True,
        all_three_actual_preview_readbacks_passed=True,all_three_additive_native_acceleration_screens_passed=True,
        original0p3_and0p8_reports_and_all_failed_source_outputs_preserved=True)
    report["parent_comparison"]=dict(baseline_report_path=str(audit.BASELINE),baseline_report_sha256=audit.BASELINE_SHA,
        baseline_original_joint_pass_count=0,candidate_joint_pass_count=3,same_fixed_final6000_weights_profiles_seeds_teacher_PI_threads_and_old_thresholds=True,
        difference_is_causal_speed_preview_not_new_weights_or_changed_threads=True,additional_rate_guard_applied_to_both_all_three_arms=True,
        original_baseline_results_rewritten=False,all_three_parent_and_candidate_attempts_retained=True,no_best_attempt_selection=True,per_arm=parent)
    report["all_preview_bracket_comparison"]=audit.bracket_comparison(report,dict(trials=oldtrials,joint_motion_body_policy_wall_trial_pass_count=0))
    return report


@pytest.mark.parametrize("mutation",[None,"prior_dropped","best","wrongprior","candidate_replaced","speed_hidden","count","trainedSTOP","frozen97","CPUchange"])
def test_publication_must_keep_allnine_outcomes_and_honest54_47_denials(mutation):
    report=publication_report(); bracket=report["all_preview_bracket_comparison"]
    if mutation=="prior_dropped":bracket["per_arm"].pop()
    elif mutation=="best":bracket["no_best_trial_or_checkpoint_selection"]=False
    elif mutation=="wrongprior":bracket["original0p8_report"]["sha256"]="0"*64
    elif mutation=="candidate_replaced":bracket["per_arm"][0]["preview05"]["native_actor_progress_m"]=93.
    elif mutation=="speed_hidden":bracket["per_arm"][0]["preview05_additive_native_rate"]["passed"]=False
    elif mutation=="count":bracket["pass_counts"]["preview05"]=2
    elif mutation=="trainedSTOP":report["semantic_outputs_preserved"]=True
    elif mutation=="frozen97":report["original_frozen97_check_enforced_by_pinned_numeric_loader"]=True
    elif mutation=="CPUchange":report["CPU_thread_numerical_preset_changed"]=True
    with mock.patch.object(publish.previous.previous.previous.previous,"audit",audit):
        if mutation is None: publish.verify_joint_report(report)
        else:
            with pytest.raises(Exception): publish.verify_joint_report(report)


def test_readable_bracket_retains_every_condition_and_formalzero_even_ifsyntheticbasicpass():
    report=publication_report(); text=audit.markdown(report)
    assert "0.3초 0/3 · 0.5초 3/3 · 0.8초 0/3" in text and "0/9" in text and "0/30" in text
    assert text.count("| FAIL |") == 6 and text.count("| PASS |") == 3 and "2.516" in text
    assert "기존0.3/0.8의 모든 결과" in text and "2,700" in text
    assert all("preview08" not in row for row in report["parent_comparison"]["per_arm"])


@pytest.mark.parametrize("failure",[False,True])
def test_newmedia_scope_restores0p8_publisher_names(failure):
    names=("audit","SCHEMA","verify_joint_report","render","load_archive"); saved={k:getattr(publish.previous,k) for k in names}
    with mock.patch.object(audit,"verify_frozen"),mock.patch.object(audit,"digest",return_value=publish.FROZEN_PUBLISHER_SHA):
        try:
            with publish.activated():
                assert publish.previous.audit is audit and publish.previous.render is publish.render
                with pytest.raises(Exception):
                    with publish.activated(): pass
                if failure:raise OSError("retained")
        except OSError:assert failure
    assert all(getattr(publish.previous,k) is v for k,v in saved.items()) and not publish._active


def test_full45s_media_lastframe_no_selected_goodtail():
    assert publish.indices()==list(range(0,450,5))
    bundle=dict(cameras=[dict(frame=i) for i in range(450)],trial=publication_report()["trials"][0])
    assert len(publish.png_indices(bundle))==5 and publish.png_indices(bundle)["05_last.png"]==449


@pytest.mark.parametrize("target",["raw","report","symlink","existing"])
def test_no_createonly_media_can_mutate_runs(tmp_path,target):
    batch=tmp_path/"raw";batch.mkdir();reports=tmp_path/"report";reports.mkdir();path=reports/"report.json"
    path.write_text(json.dumps(dict(batch=str(batch))));output=tmp_path/"new"
    if target=="raw":output=batch/"media"
    elif target=="report":output=reports/"media"
    elif target=="symlink":
        link=tmp_path/"linked";link.symlink_to(batch,target_is_directory=True);output=link/"media"
    else:output.mkdir()
    with mock.patch.object(publish,"_publish") as delegated,pytest.raises(Exception):publish.publish(output,path,audit.digest(path))
    delegated.assert_not_called()


def test_firstread_changedbytes_abort_beforewriter(tmp_path):
    reports=tmp_path/"report";reports.mkdir();path=reports/"report.json";path.write_text(json.dumps(dict(batch=str(tmp_path/"raw"))))
    original=audit.read_json
    def mutate(p):
        value=original(p);path.write_text(json.dumps(dict(batch=str(tmp_path/"changed"))));return value
    with mock.patch.object(audit,"read_json",side_effect=mutate),mock.patch.object(publish,"_publish") as delegated,pytest.raises(Exception):
        publish.publish(tmp_path/"new",path,audit.digest(path))
    delegated.assert_not_called()
