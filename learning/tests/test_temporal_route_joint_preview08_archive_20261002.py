"""HH_261002 - Synthetic-only proof of independent preview/PI/native-rate evidence, restored scopes and full45s publication."""

import ast
from contextlib import ExitStack, contextmanager
from copy import deepcopy
import json
import math
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import pytest

from scripts.e2e import audit_temporal_route_joint_preview08_signed45_batch_20261002 as audit
from scripts.e2e import publish_temporal_route_joint_preview08_signed45_20261002 as publish
from portable_e2e.longitudinal_pi_headroom_research_20261001 import LongitudinalPIHeadroomResearch


def endpoint():
    return {name+"_sha256":letter*64 for name,letter in (("weights","a"),("metadata","b"),("checkpoint","c"),("declaration","d"))}


def config():
    return dict(wheelbase_m=2.819186218917395,maximum_steer_angle_rad=.61,speed_cap_mps=2.5,
        minimum_lookahead_m=1.5,lookahead_time_s=.8,speed_preview_s=.8,throttle_gain=.7,brake_gain=.7,maximum_throttle=.5)


def selected(old=2.1,new=2.2,**extras):
    speeds=[2.3]*64;speeds[2]=old;speeds[7]=new
    return dict(candidate_index=0,xy_base_m=[[.25*(i+1),.003*(i+1)] for i in range(64)],speed_mps=speeds,timestep_s=.1,**extras)


def native_states(first=20):
    return [dict(frame=100+i,timestamp=10.+i*.05,capture_phase="stationary_warmup" if i<first else "driving",
        world_velocity_carla=[1.,0.,0.],world_acceleration_carla=[.1,0.,0.]) for i in range(900)]


def original_tracker():
    # HH_261002 - Execute only the unchanged pure frozen function AST; never import a learned model or physical owner.
    tree=ast.parse((audit.ROOT/"learning/portable_e2e/simulation_research.py").read_bytes())
    node=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=="trajectory_control")
    node.args.args[0].annotation=node.args.args[1].annotation=node.args.args[2].annotation=None
    namespace=dict(math=math,_require=audit.require)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[node],type_ignores=[])),"frozen_pure_tracker_only","exec"),namespace)
    return namespace["trajectory_control"]


@pytest.mark.parametrize("old,new,intent,guard",[(2.1,2.2,None,False),(.05,2.2,None,True),
    (2.1,.05,None,True),(0.,2.2,None,True),(2.1,2.2,1,True),(2.1,2.2,True,True),(2.1,2.2,0,False)])
def test_exact_indices_preserve_whole_original_STOP_or_explicit_nonGO(old,new,intent,guard):
    plan=selected(old,new,**({} if intent is None else {"intent":intent}))
    receipt=audit.preview_targets(plan,config())
    assert receipt["original_preview_index"]==2 and receipt["proposed_preview_index"]==7
    assert receipt["original_STOP_tracker_preserved"] is guard
    assert receipt["effective_speed_preview_s"]==(.3 if guard else .8)
    assert receipt["effective_selected_target_mps"]==(old if guard else new)
    assert not receipt["selected_trajectory_changed"] and not receipt["actual_measured_speed_clipped"]


@pytest.mark.parametrize("mutation",["count","Boolean","negative","NaN","dt"])
def test_selected_ABI_cannot_be_shortened_clipped_or_relabelled(mutation):
    plan=selected()
    if mutation=="count":plan["speed_mps"].pop()
    elif mutation=="dt":plan["timestep_s"]=.2
    else:plan["speed_mps"][7]={"Boolean":True,"negative":-.01,"NaN":float("nan")}[mutation]
    with pytest.raises(Exception):audit.preview_targets(plan,config())


@pytest.mark.parametrize("current,old,new",[(0.,2.1,2.2),(2.,2.1,2.2),(2.4,2.1,2.2),(2.,0.,2.2),(2.,2.1,0.)])
def test_independent_tracker_matches_frozen_geometry_control_without_nominal_override(current,old,new):
    plan=selected(old,new);cfg=config();target=audit.preview_targets(plan,cfg)
    frozen_cfg=SimpleNamespace(**dict(cfg,speed_preview_s=target["effective_speed_preview_s"]),validate=lambda:None)
    actual=original_tracker()(SimpleNamespace(**plan),current,frozen_cfg)
    assert audit.equivalent(actual,audit.tracker_math(plan,current,cfg,target["effective_selected_target_mps"]))
    assert cfg["wheelbase_m"]!=2.85


@pytest.mark.parametrize("current,target,integral",[(.6,2.5,0.),(2.,2.2,.1),(2.4,2.5,.7),(2.4,0.,.8),(2.4,1.7,-.8)])
def test_independent_PI_matches_unchanged_headroom_antiwindup_and_STOP(current,target,integral):
    base=audit.tracker_math(selected(target,target),current,config(),target)
    actual=LongitudinalPIHeadroomResearch();actual.integral_error=integral
    request,receipt=actual.update(target,current)
    expected=deepcopy(base)
    if request is not None:expected.update(request)
    control,after,reconstruction=audit.pi_math(base,current,integral)
    assert audit.equivalent(control,expected) and after==actual.integral_error
    assert all(audit.equivalent(receipt[k],v) for k,v in reconstruction.items())


def preview_fixture():
    records,timing,commands,receipts,states=[],[],[],[],[]
    integral,last=0.,None
    for i,kind in enumerate(("startup","fresh","held","fresh2","stop","held","rejected","emergency")):
        frame=100+i;seq=i+1;clock=1000+i*20
        wall=dict(command_frame=frame,command_reason="next_driving_control",ACK_sequence=seq,
            proposal_started_monotonic_ns=clock,proposal_finished_monotonic_ns=clock+10,
            ACK_started_monotonic_ns=clock+11,ACK_finished_monotonic_ns=clock+12)
        raw=dict(ack_receipt_sequence=seq,command_reason=wall["command_reason"],control_source="teacher_startup",
            learned_control_proposed=False,held_between_camera_frames=False,inference_frame=None,control=None)
        tracker=None
        if kind in ("fresh","fresh2","stop","rejected"):
            plan=selected(new=2.3 if kind=="fresh2" else 2.2)
            if kind=="stop":plan=selected(old=0.,new=2.2)
            witness=audit.preview_targets(plan,config());current=2.
            original=audit.tracker_math(plan,current,config(),witness["effective_selected_target_mps"])
            control,after,pi=audit.pi_math(original,current,integral)
            pi.update(original_normal_tracker_control=original,proposed_normal_tracker_control=deepcopy(control),
                controller_target_cap_mps=2.4,original_model_target_mps=original["desired_speed_mps"],
                governed_controller_target_mps=min(original["desired_speed_mps"],2.4),
                one_fresh_camera_update_one_PI_integration=True,observed_speed_clipped=False)
            raw.update(selected=plan,decoded_output=dict(candidate_xy=[deepcopy(plan["xy_base_m"])],candidate_speed=[list(plan["speed_mps"])]),
                measured_current_speed_mps=current,controller=config(),longitudinal_pi_research=pi,
                inference_frame=frame,control_source="learned_trajectory_tracker",learned_control_proposed=True,control=control)
            tracker=dict(witness,original_tracker_control=original)
            if kind=="rejected":
                raw.update(learned_control_proposed=False,control_source="emergency_brake",
                    control=dict(throttle=0.,brake=1.,steer=0.,reverse=False,hand_brake=False));integral=0.
            else:integral=after;last=deepcopy(raw)
        elif kind=="held":
            raw.update(control_source=last["control_source"],learned_control_proposed=True,held_between_camera_frames=True,
                inference_frame=last["inference_frame"],control=deepcopy(last["control"]),longitudinal_pi_research=deepcopy(last["longitudinal_pi_research"]))
        elif kind=="emergency":
            raw.update(control_source="emergency_brake",control=dict(throttle=0.,brake=1.,steer=0.,reverse=False,hand_brake=False))
        requested=dict(throttle=.11,brake=0.,steer=0.,reverse=False,hand_brake=False) if raw["control"] is None else {
            k:raw["control"][k] for k in ("throttle","brake","steer","reverse","hand_brake")}
        row=dict(schema=audit.PREVIEW_SCHEMA,command_frame=frame,command_reason=wall["command_reason"],
            proposal_started_monotonic_ns=clock+1,proposal_finished_monotonic_ns=clock+9,
            actual_policy_controller=config(),tracker_receipt=tracker,control_source=raw["control_source"],
            learned_control_proposed=raw["learned_control_proposed"],held_between_camera_frames=raw["held_between_camera_frames"],
            inference_frame=raw["inference_frame"],actual_returned_control=deepcopy(raw["control"]))
        records.append(row);timing.append(wall);commands.append(raw);receipts.append(dict(sequence=seq,requested_control=requested))
        states.append(dict(frame=frame,vx=2.,vy=0.))
    return records,timing,commands,receipts,states


def test_every_normal_readback_fresh_PI_STOP_held_and_rejected_attempt_is_retained():
    result=audit.preview_census(*preview_fixture())
    assert result["normal_proposal_denominator"]==result["controller_readback_count"]==8
    assert result["fresh_tracker_count"]==result["independent_PI_update_count"]==4
    assert result["original_STOP_or_non_GO_fallback_count"]==1 and result["held_ACK_without_new_PI_count"]==2
    assert result["complete_PI_integral_chronology_reconstructed"] and not result["physical_speed_gate_tolerance_changed"]


@pytest.mark.parametrize("mutation",["drop_held","duplicate","config","geometry","clock","selected","preview", "PI", "held_PI", "ACK", "ACK_bool", "v0", "fresh_missing", "empty"])
def test_no_source_claim_or_held_reintegration_can_manufacture_pass(mutation):
    values=list(preview_fixture());records,timing,commands,receipts,states=values
    if mutation=="drop_held":records.pop(2)
    elif mutation=="duplicate":records.append(deepcopy(records[-1]))
    elif mutation=="config":records[1]["actual_policy_controller"]["speed_preview_s"]=.3
    elif mutation=="geometry":records[1]["actual_policy_controller"]["wheelbase_m"]=2.85
    elif mutation=="clock":records[1]["proposal_finished_monotonic_ns"]+=30
    elif mutation=="selected":commands[1]["decoded_output"]["candidate_speed"][0][7]+=1e-3
    elif mutation=="preview":records[1]["tracker_receipt"]["proposed_preview_index"]=8
    elif mutation=="PI":commands[3]["longitudinal_pi_research"]["integral_before"]+=.01
    elif mutation=="held_PI":commands[2]["longitudinal_pi_research"]["integral_after"]+=.01
    elif mutation=="ACK":receipts[1]["requested_control"]["throttle"]+=2e-6
    elif mutation=="ACK_bool":receipts[1]["requested_control"]["reverse"]=0
    elif mutation=="v0":states[1]["vx"]+=1e-4
    elif mutation=="fresh_missing":records[1]["tracker_receipt"]=None
    else:values=[[],[],[],[],[]]
    with pytest.raises(Exception):audit.preview_census(*values)


def test_native_double_float32_reconstruction_does_not_relax2p5_physical_envelope():
    values=list(preview_fixture());values[-1][1]["vx"]+=4e-7
    assert audit.preview_census(*values)["passed"]
    values=list(preview_fixture());values[2][1]["measured_current_speed_mps"]=2.50000001
    with pytest.raises(Exception):audit.preview_census(*values)


@pytest.mark.parametrize("first",[0,20])
def test_full900_native_rate_matches_literal_physical_owner_and_includes_warm_boundary(first):
    states=native_states(first);tree=ast.parse(audit.OWNER_SOURCE.read_bytes())
    node=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=="acceleration_report")
    namespace=dict(parent=SimpleNamespace(jsonl=lambda _:states),math=math,require=audit.require)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[node],type_ignores=[])),"frozen_preview_owner_rate_only","exec"),namespace)
    actual=namespace["acceleration_report"](Path("no_data_read"))
    assert audit.equivalent(actual,audit.acceleration_census(states))
    assert actual["driving_speed_rate_interval_denominator"]==900-first-(first==0)
    assert actual["warmup_to_first_driving_interval_included"] is (first>0)


@pytest.mark.parametrize("mutation",["prefix_above3","belowminus6","wrong_dt","missing","future_warmup","Boolean_frame","NaN"])
def test_rate_failures_full_denominator_and_other_acceleration_observations_are_not_hidden(mutation):
    states=native_states()
    if mutation=="prefix_above3":states[19]["world_velocity_carla"]=[.7,0.,0.]
    elif mutation=="belowminus6":states[21]["world_velocity_carla"]=[.5,0.,0.]
    elif mutation=="wrong_dt":states[25]["timestamp"]+=.01
    elif mutation=="missing":states.pop()
    elif mutation=="future_warmup":states[899]["capture_phase"]="stationary_warmup"
    elif mutation=="Boolean_frame":states[0]["frame"]=True
    else:states[21]["world_velocity_carla"]=[float("nan"),0.,0.]
    if mutation in ("prefix_above3","belowminus6"):
        result=audit.acceleration_census(states)
        assert not result["passed"] and result["native_speed_rate_outside_minus6_plus3_count"]>0
        assert result["driving_speed_rate_interval_denominator"]==880 and result["full_native_count"]==900
    else:
        with pytest.raises(Exception):audit.acceleration_census(states)


def test_abs2p9_and_API_acceleration_norm_are_distinct_observations_not_relaxed_new_guard():
    states=native_states();states[20]["world_velocity_carla"]=[1.149,0.,0.];states[20]["world_acceleration_carla"]=[4.,0.,0.]
    result=audit.acceleration_census(states)
    assert result["passed"] and result["native_speed_rate_abs_above2p9_observation_count"]==2
    assert result["actual_acceleration_above3_observation_count"]==1


def scopes():
    return (mock.patch.object(audit,"verify_frozen"),mock.patch.object(audit.thread,"verify_frozen"),
        mock.patch.object(audit.joint,"verify_frozen"))


@pytest.mark.parametrize("failure",[False,True])
def test_full_nested_archive_scope_restores_all_old_names_and_declared_contract_on_error(failure):
    names=("SCHEMA","RECEIPT_SCHEMA","OWNER_SCHEMA","COLLECTOR_SCHEMA","FROZEN_HELPERS","NEW_FROZEN","verify_contract","transformed_trial_reader")
    saved=[(module,{k:getattr(module,k) for k in names}) for module in (audit.thread,audit.joint,audit.signed)]
    old_export=audit.signed.previous.verify_export
    with ExitStack() as stack:
        for patch in scopes():stack.enter_context(patch)
        try:
            with audit.activated(endpoint(),({},[])):
                reader=audit.transformed_trial_reader()
                assert reader.__globals__["verify_contract"] is audit.verify_contract
                assert reader.__globals__["verify_export"] is audit.joint.verify_export
                assert reader.__globals__["independent_motion"] is audit.signed.previous.independent_motion
                assert reader.__globals__["audit_body"] is audit.signed.previous.audit_body
                assert reader.__globals__["EXACT_METADATA"]==audit.previous.EXACT_METADATA
                assert "current_training_purpose" in reader.__code__.co_consts
                assert "recovery_data_revision" not in reader.__code__.co_consts
                contract=audit.preview_signed_contract()
                assert contract.__globals__["OWNER_SCHEMA"]==audit.OWNER_SCHEMA
                with pytest.raises(Exception):
                    with audit.activated(endpoint(),({},[])):pass
                if failure:raise OSError("all failures preserved")
        except OSError:assert failure
    assert all(all(getattr(module,k) is v for k,v in original.items()) for module,original in saved)
    assert audit.signed.previous.verify_export is old_export and audit.joint.joint_baseline_contract is audit._joint_contract
    assert not audit._active and audit._baseline is None


def test_source_census_has_no_pin_filter_and_is_order_independent():
    first=audit.source_census(dict(source_sha256={"a":"a"*64,"b":"b"*64}))
    assert first==audit.source_census(dict(source_sha256={"b":"b"*64,"a":"a"*64}))
    assert first["pin_count"]==2 and first["no_camera_source_pin_dropped"]
    assert first!=audit.source_census(dict(source_sha256={"a":"a"*64}))


def publication_report():
    trials=[]
    for i,spec in enumerate(audit.PROFILES):
        trials.append(dict(trial_index=i,signed_recovery_profile=deepcopy(spec),joint_motion_body_policy_wall100ms_passed=False,
            independent_cpu_thread_preset=dict(passed=True),independent_preview_configuration=dict(passed=True),
            independent_additive_native_acceleration=dict(passed=True,native_speed_rate_outside_minus6_plus3_count=0),
            native_actor_progress_m=89.,post_handoff_base_norm_maximum_mps=2.5159,
            delivery_to_ACK_ms=dict(maximum=90.),independent_signed_recovery=dict(case_screen_passed=True,recovery_screen_passed=i<2)))
    result=dict(schema=audit.SCHEMA,trials=trials,prospective_trials=3,all_failed_tail_retained=True,
        signed_recovery_case_count=2,centered_setup_comparator_count=1,signed_recovery_pass_count=2,
        all_three_are_not_same_seed_repetitions=True,first_effect_is_latest_prior_actual_learned_ACK=True,
        no_teacher_recovery_counted_as_learned=True,full_MAIN9_completion_claimed=False,full_route_goal_stop_approved=False,
        joint_motion_body_policy_wall_trial_pass_count=0,optimizer_updates=6000,known_total_optimizer_updates=12200,
        trainable_state_tensor_count=54,frozen_state_tensor_count=47,original_backbone_weights_byte_unchanged=False,
        semantic_outputs_preserved=False,visual_encoder_frozen=True,semantic_head_parameters_frozen=True,
        original47_bytefixed_check_enforced_by_pinned_numeric_loader=True,all101_tensor_values_independently_reloaded_by_this_audit=False,
        no_torch_pickle_loaded=True,model_weights_unchanged=True,teacher_startup_and_PI2p4_parameters_unchanged=True,
        CPU_thread_numerical_preset_changed=False,learned_tracker_speed_preview_changed=True,
        model_change_is_learned_weights_only_not_teacher_controller_or_threshold_change=False,
        final_endpoint_sha256=endpoint(),cpu_thread_numerical_preset=audit.thread.pure_preset()[0],
        causal_selected_speed_preview_calibration=audit.calibration_contract(),all_three_complete_thread_readbacks_passed=True,
        all_three_actual_preview_readbacks_passed=True,all_three_additive_native_acceleration_screens_passed=True)
    result["parent_comparison"]=dict(baseline_report_path=str(audit.BASELINE),baseline_report_sha256=audit.BASELINE_SHA,
        baseline_original_joint_pass_count=0,candidate_joint_pass_count=0,
        same_fixed_final6000_weights_profiles_seeds_teacher_PI_threads_and_old_thresholds=True,
        difference_is_causal_speed_preview_not_new_weights_or_changed_threads=True,
        additional_rate_guard_applied_to_both_all_three_arms=True,original_baseline_results_rewritten=False,
        all_three_parent_and_candidate_attempts_retained=True,no_best_attempt_selection=True,
        per_arm=[dict(case_id=spec["case_id"],baseline=deepcopy(row),preview08=deepcopy(row),
            candidate_additive_native_rate=deepcopy(row["independent_additive_native_acceleration"])) for spec,row in zip(audit.PROFILES,trials)])
    return result


@pytest.mark.parametrize("mutation",[None,"trained_STOP","old97","CPUchanged","new_weights","partial","best", "wrongbaseline", "wrongpreview", "census"])
def test_publication_denies_new_model_approval_or_selected_good_subset(mutation):
    report=publication_report()
    if mutation=="trained_STOP":report["semantic_outputs_preserved"]=True
    elif mutation=="old97":report["original_frozen97_check_enforced_by_pinned_numeric_loader"]=True
    elif mutation=="CPUchanged":report["CPU_thread_numerical_preset_changed"]=True
    elif mutation=="new_weights":report["model_weights_unchanged"]=False
    elif mutation=="partial":report["trials"].pop()
    elif mutation=="best":report["parent_comparison"]["no_best_attempt_selection"]=False
    elif mutation=="wrongbaseline":report["parent_comparison"]["baseline_report_sha256"]="0"*64
    elif mutation=="wrongpreview":report["causal_selected_speed_preview_calibration"]["configured_speed_preview_s"]=1.
    elif mutation=="census":report["all_three_actual_preview_readbacks_passed"]=False
    with mock.patch.object(publish.previous.previous.previous,"audit",audit):
        if mutation is None:publish.verify_joint_report(report)
        else:
            with pytest.raises(Exception):publish.verify_joint_report(report)


def test_readable_report_keeps_speed_failure_all_attempts_and_zero_formal_approvals():
    text=audit.markdown(publication_report())
    assert "FAIL → FAIL" in text and "2.516" in text and "0/9" in text and "0/30" in text
    assert "0.3→0.8" in text and "STOP 학습 완료" in text and "2,700" in text


@pytest.mark.parametrize("failure",[False,True])
def test_new_publisher_full_delegation_restores_every_frozen_name(failure):
    names=("audit","SCHEMA","verify_joint_report","render","load_archive")
    saved={k:getattr(publish.previous,k) for k in names}
    @contextmanager
    def delegate():
        assert publish.previous.audit is audit and publish.previous.render is publish.render
        if failure:raise OSError("retain")
        yield
    with mock.patch.object(audit,"verify_frozen"),mock.patch.object(audit,"digest",return_value=publish.FROZEN_PUBLISHER_SHA), \
         mock.patch.object(publish.previous,"activated",side_effect=delegate):
        try:
            with publish.activated():
                with pytest.raises(Exception):
                    with publish.activated():pass
        except OSError:assert failure
    assert all(getattr(publish.previous,k) is v for k,v in saved.items()) and not publish._active


def test_frozen_all45s_GIF_and_final_camera_PNG_are_retained():
    assert publish.indices()==list(range(0,450,5))
    bundle=dict(cameras=[dict(frame=i) for i in range(450)],trial=publication_report()["trials"][0])
    assert publish.png_indices(bundle)["05_last.png"]==449
    assert len(publish.png_indices(bundle))==5 and publish.displayed_receipt is publish.previous.displayed_receipt


@pytest.mark.parametrize("target",["raw","report","symlink","existing"])
def test_create_only_publication_cannot_change_input_runs_or_follow_ancestor_symlinks(tmp_path,target):
    batch=tmp_path/"raw";batch.mkdir();reports=tmp_path/"report";reports.mkdir()
    path=reports/"batch_outcome.json";path.write_text(json.dumps(dict(batch=str(batch))))
    output=tmp_path/"new_media"
    if target=="raw":output=batch/"media"
    elif target=="report":output=reports/"media"
    elif target=="symlink":
        link=tmp_path/"linked";link.symlink_to(batch,target_is_directory=True);output=link/"media"
    else:output.mkdir()
    with mock.patch.object(publish,"_publish") as delegated,pytest.raises(Exception):
        publish.publish(output,path,audit.digest(path))
    delegated.assert_not_called()


def test_report_first_read_identity_is_checked_before_any_media_writer(tmp_path):
    reports=tmp_path/"report";reports.mkdir();path=reports/"report.json";path.write_text(json.dumps(dict(batch=str(tmp_path/"raw"))))
    original=audit.read_json
    def mutate(p):
        value=original(p);path.write_text(json.dumps(dict(batch=str(tmp_path/"different"))));return value
    with mock.patch.object(audit,"read_json",side_effect=mutate),mock.patch.object(publish,"_publish") as delegated,pytest.raises(Exception):
        publish.publish(tmp_path/"new_media",path,audit.digest(path))
    delegated.assert_not_called()
