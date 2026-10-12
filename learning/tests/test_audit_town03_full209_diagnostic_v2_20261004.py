"""HH_261004 - Test real private full209 ACK/parser and witness adapters on synthetic CPU evidence only."""

import copy
import hashlib
import json
import math
from pathlib import Path

import pytest

from scripts.e2e import audit_town03_full209_diagnostic_v2_20261004 as m
from test_audit_carla_acknowledged_control import raw_row, control, chain
from test_temporal_route_joint_town03_straight_full209_v1_20261004 import synthetic_terminal_replay_fixture


def full_context():
    # HH_261004 - The original sealed scalar manual1 producer supplies this fixture; it is not inferred from a requested receipt.
    old,_=m._parents()
    blank=m.expected_scoped_blank_control()
    route=old.read_json(old.ROUTE)
    total=old.live.collector.RouteProjector(route).length_m
    endpoint=route['route'][-1]
    states,receipts=[],[]
    def send(frame,reason,requested):
        receipt=dict(sequence=len(receipts)+1,actor_id=42,response_actor_id=42,before_frame=frame,after_ack_frame=frame,
            expected_before_frame=frame,mode='acknowledged_batch',do_tick=False,status='ACKNOWLEDGED',server_accepted=True,
            physical_actuation_proven=False,reason=reason,requested_control=copy.deepcopy(requested))
        receipts.append(receipt)
        return receipt['sequence']
    send(99,'pre_capture_bootstrap',blank)
    bootstrap=raw_row(100,'stationary_warmup',blank,1)
    bootstrap['alignment']=bootstrap.pop('control_transport');bootstrap['recorded_training_state']=False
    send(100,'stationary_warmup_start',blank)
    normal=control(throttle=.1,gear=1,manual_gear_shift=True)
    for index in range(30):
        frame=101+index;phase='stationary_warmup' if index<24 else 'driving'
        row=raw_row(frame,phase,receipts[-1]['requested_control'],receipts[-1]['sequence'])
        row.update(route_progress_m=0. if index<29 else total-2.,collision=[],lane_invasion=[])
        if index==29:
            row.update(x=endpoint['x']-2.,y=endpoint['y'],z=0.,yaw=0.)
            row['actor_snapshot_transform_carla']=dict(x=row['x']+1.425,y=-row['y'],z=0.,yaw=0.,pitch=0.,roll=0.)
        chosen=blank if index<24 or index==29 else normal
        row['next_control']=copy.deepcopy(chosen)
        reason=m.GOAL_REASON if index==29 else 'next_'+phase+'_control'
        row['control_transport']['next_command_receipt_sequence']=send(frame,reason,chosen)
        if index==23:
            row['next_control']=copy.deepcopy(normal)
            row['control_transport']['next_command_receipt_sequence']=send(frame,'initial_drive_control',normal)
        states.append(row)
    send(states[-1]['frame'],'driving_end_brake',blank)
    states[-1]['control_transport']['next_command_receipt_sequence']=receipts[-1]['sequence']
    frame=states[-1]['frame'];prior=states[-1]['control_transport']['expected_receipt_sequence']
    status=old.catalog_statuses(route,states)[-1]
    guard=dict(frame=frame,measured_actor_planar_speed_mps=1.,maximum_actor_planar_speed_mps=2.7,
        free_disk_bytes=6*1024**3,required_free_bytes=5*1024**3+16*1024**2)
    goal=dict(schema=old.goal_guard.SCHEMA,status='ACKNOWLEDGED',command_reason=m.GOAL_REASON,command_frame=frame,
        source_timestamp_simulation_seconds=states[-1]['timestamp'],actor_id=42,prior_learned_ACK_sequence=prior,
        prior_learned_ACK_after_frame=frame-1,cleanup_ACK_sequence=prior+1,original_goal_tolerance_m=2.5,
        original_catalog_length_m=total,remaining_catalog_route_m=status.remaining_route_m,
        actual_planar_catalog_goal_distance_m=status.planar_distance_m,requested_original_cleanup_control=copy.deepcopy(blank),
        original_catalog_two_predicates_passed=True,actual_prior_learned_ACK_effect_proven=True,
        owner_source_PID_check_passed=True,observed_event_snapshot_empty=True,existing_emergency_latch_clear=True,
        native_guard_attempted=True,native_guard_passed=True,cleanup_ACK_received=True,native_guard=guard,
        learned_control_proposed=False,learned_STOP_proven=False,physical_cleanup_actuation_proven=False,
        vehicle_control_approved=False,current_tick_hazard_detector_clear_proven=False)
    goal['actual_state_binding_sha256']=old.goal_guard._state_binding(states[-1])
    states[-1]['catalog_goal_cleanup_receipt']=goal
    wrapper=dict(schema=old.live.SCHEMA+'.catalog_goal_cleanup',receipt=copy.deepcopy(goal),
        failed_or_absent_receipt_retained=True,learned_STOP_claim=False)
    commands=[dict(command_frame=frame-1,ack_receipt_sequence=prior,source_checkpoint_sha256='a'*64,
        application=dict(learned_control_applied=True),control_source='learned_trajectory_tracker')]
    timings=[dict(ACK_sequence=r['sequence'],ACK_server_accepted=True,learned_control_proposed=True,
        source_checkpoint_sha256='a'*64,stream_source_sha256='b'*64,control_source='learned_trajectory_tracker',
        command_frame=r['after_ack_frame']) for r in receipts]
    manual_tree=m.ast.parse(m._source('tools/e2e/run_owned_town07_signed_recovery_v3_20261001.py'))
    pin=next(node.value.value for node in manual_tree.body if isinstance(node,m.ast.Assign)
        and any(isinstance(target,m.ast.Name) and target.id=='PARENT_CHECKER_SOURCE_SHA' for target in node.targets))
    manifest=dict(status='complete',capture_contract=dict(
        signed_recovery_initialization_gear_contract=dict(actual_manual_gear_shift=True,actual_gear=1,
            checker_function_source_sha256=pin,original_frame_actor_PID_finite_and_full_brake_guards_preserved=True,
            observed_controls_or_snapshots_proxied=False),
        learned_research=dict(checkpoint_sha256='a'*64,stream_identity=dict(source_sha256='b'*64))),
        result=dict(goal_reached=True))
    return states,receipts,bootstrap,dict(commands=commands,timings=timings,native_guards=[guard],wrapper=wrapper,manifest=manifest)


def test_frozen_dependencies_and_original_scoped_blank_source():
    assert m.verify_frozen()==m.FROZEN_PINS
    assert m.expected_scoped_blank_control()==control(brake=1.,gear=1,manual_gear_shift=True)


def test_complete_real_extended_ACK_parser_without_success_stub():
    states,receipts,bootstrap,context=full_context()
    before=copy.deepcopy((states,receipts,bootstrap,context))
    report=m.analyze_transport(states,receipts,bootstrap,**context)
    assert report['status']=='PASS' and all(report['flags'].values())
    assert report['receipt_count']==34 and report['recorded_state_count']==30
    assert report['command_reason_counts'][m.GOAL_REASON]==1
    assert report['command_reason_counts']['next_driving_control']==5
    assert report['goal_cleanup_authentication']['original_manual1_blank_source_reconstructed'] is True
    assert (states,receipts,bootstrap,context)==before


def test_ordinary_legacy_exact_delegate_parity(chain):
    expected=m.original_ack.analyze_transport(*chain)
    assert m.analyze_transport(*chain)==expected


def test_failed_no_goal_interval_with_context_preserves_original_ACK_diagnostics(chain):
    expected=m.original_ack.analyze_transport(*chain)
    assert m.analyze_transport(*chain,commands=[],timings=[],native_guards=[],wrapper={},
        manifest=dict(status='failed',result={}))==expected


@pytest.mark.parametrize('fault',['missing','duplicate','early','wrong_phase','wrong_frame','actor','source','NN',
    'brake','gear','manual','end_gear','no_prior','event','state_pose','remaining','manual_contract','unknown_reason','partial_context'])
def test_goal_reason_never_admits_unauthenticated_or_wrong_actual_ABI(fault):
    states,receipts,bootstrap,context=full_context()
    goal=states[-1]['catalog_goal_cleanup_receipt'];cleanup=receipts[-2]
    if fault=='missing': cleanup['reason']='next_driving_control'
    elif fault=='duplicate': receipts[-3]['reason']=m.GOAL_REASON
    elif fault=='early': receipts[-3]['reason'],cleanup['reason']=m.GOAL_REASON,'next_driving_control'
    elif fault=='wrong_phase': states[-1]['capture_phase']='stationary_tail'
    elif fault=='wrong_frame': cleanup['before_frame']-=1
    elif fault=='actor': cleanup['actor_id']=43
    elif fault=='source': context['timings'][-3]['stream_source_sha256']='f'*64
    elif fault=='NN': context['commands'][0]['source_checkpoint_sha256']='f'*64
    elif fault=='brake': cleanup['requested_control']['brake']=.9
    elif fault=='gear': cleanup['requested_control']['gear']=0
    elif fault=='manual': cleanup['requested_control']['manual_gear_shift']=False
    elif fault=='end_gear': receipts[-1]['requested_control']['gear']=0
    elif fault=='no_prior': context['commands'][0]['application']['learned_control_applied']=False
    elif fault=='event': states[-2]['lane_invasion']=[{'synthetic_test':True}]
    elif fault=='state_pose': states[-1]['x']+=10.
    elif fault=='remaining': goal['remaining_catalog_route_m']+=.01
    elif fault=='manual_contract': context['manifest']['capture_contract']['signed_recovery_initialization_gear_contract']['actual_gear']=0
    elif fault=='unknown_reason': receipts[2]['reason']='arbitrary_new_unknown_reason'
    else: del context['commands']
    with pytest.raises((m.original_ack.base.EvidenceError,RuntimeError)):
        m.analyze_transport(states,receipts,bootstrap,**context)


@pytest.mark.parametrize('fault',['snapshot','frame_chain','control_alignment','perframe_reason'])
def test_original_transport_failure_math_is_still_real_and_unchanged(fault):
    states,receipts,bootstrap,context=full_context()
    if fault=='snapshot': states[0]['x']+=1.
    elif fault=='frame_chain': states[0]['timestamp']+=.01
    elif fault=='control_alignment': states[0]['current_control']['throttle']=.4
    else: receipts[2]['reason']='next_driving_control'
    result=m.analyze_transport(states,receipts,bootstrap,**context)
    assert result['status']=='FAIL' and result['failed_flags']


def test_extended_goal_reason_requires_context_not_only_membership():
    states,receipts,bootstrap,_=full_context()
    with pytest.raises(m.original_ack.base.EvidenceError,match='unknown receipt reason/status'):
        m.analyze_transport(states,receipts,bootstrap)


def test_original_automatic_NN_control_and_manual_readback_gear_equality_semantics_preserved():
    states,receipts,bootstrap,context=full_context()
    prior=receipts[-3]
    prior['requested_control'].update(gear=0,manual_gear_shift=False)
    states[-2]['next_control']=copy.deepcopy(prior['requested_control'])
    states[-1]['current_control']=copy.deepcopy(prior['requested_control'])
    states[-1]['current_control']['gear']=1
    old,_=m._parents()
    states[-1]['catalog_goal_cleanup_receipt']['actual_state_binding_sha256']=old.goal_guard._state_binding(states[-1])
    context['wrapper']['receipt']=copy.deepcopy(states[-1]['catalog_goal_cleanup_receipt'])
    assert m.analyze_transport(states,receipts,bootstrap,**context)['status']=='PASS'


def test_private_view_retains_delegate_module_missing_attribute_resolution():
    class Delegate:
        def __getattr__(self,name):
            return 'DELEGATED_'+name
    view=m._PrivateView(Delegate(),audit_terminal_navigation='FRESH')
    assert view.audit_terminal_navigation=='FRESH' and view.audit_receipts=='DELEGATED_audit_receipts'


def test_sealed_owner_function_does_not_follow_activated_diagnostic_alias(monkeypatch):
    old,original_live=m._parents()
    def wrong(*args):
        raise AssertionError('must not recurse into the fresh owner alias')
    monkeypatch.setattr(old,'diagnostic_report',wrong)
    monkeypatch.setattr(old,'live',object())
    function=m._sealed_owner_function('diagnostic_report',old)
    assert function is not wrong and function.__code__.co_filename=='sealed_original_full209_owner:diagnostic_report'
    assert m._parents()[1] is original_live


def test_real_three_layer_inherited_diagnostic_reaches_actual_extended_ACK_parser(tmp_path,monkeypatch):
    # HH_261004 - Synthetic file-read fixtures drive all three real diagnostic functions; the ACK positive itself is never stubbed.
    old,_=m._parents()
    states,receipts,bootstrap,context=full_context()
    for row in states:
        row['command']=0
    route=old.read_json(old.ROUTE)
    cameras=[dict(frame=row['frame'],timestamp=row['timestamp'],capture_phase=row['capture_phase'],
        images={},source_timestamps={},camera_order=[]) for row in states[::2]]
    manifest=context['manifest']
    manifest['coordinate_contract']=dict(wheelbase_m=2.85)
    manifest['capture_contract']['control_transport']=dict(bootstrap_observation=bootstrap)
    manifest['capture_contract']['learned_research']['stream_identity']['split']='val'
    values={'states.jsonl':states,'camera_frames.jsonl':cameras,'learned_control.jsonl':context['commands'],
        'control_receipts.jsonl':receipts,'wall_timing.jsonl':context['timings']}
    nested=old._goal_diagnostic.__globals__['diagnostic_function']().__globals__['_diagnostic']
    monkeypatch.setitem(nested.__globals__,'jsonl',lambda path:values.get(Path(path).name,[]))
    monkeypatch.setitem(nested.__globals__,'read_json',lambda path:manifest if Path(path).name=='manifest.json' else route)
    monkeypatch.setitem(nested.__globals__,'independent_event',old.recomputed_straight_interval)
    calls=[]
    def actual_ack(s,r,b,*,wheelbase=2.85):
        result=m.analyze_transport(s,r,b,wheelbase=wheelbase,**context)
        calls.append(result)
        return result
    function=m._private_goal_function(actual_ack)
    function(tmp_path,{},dict(cleanup_errors=['synthetic_body_not_requested'],source_hashes_match=False,
        external_source_hashes_match=False))
    assert len(calls)==1 and calls[0]['status']=='PASS' and calls[0]['receipt_count']==34


def test_actual_synthetic_terminal_decoder_uses_same_exact_witness_fields(monkeypatch):
    payload,states,commands,manifest=synthetic_terminal_replay_fixture(monkeypatch)
    result=m.audit_terminal_navigation(payload,states,commands,manifest=manifest,
        stream_source_sha256='a'*64,episode_id='temporal_v2_SYNTHETIC_CPU_ONLY')
    assert result['passed'] is True and result['actual_terminal_inference_count']==1


def test_one_ULP_projection_difference_is_not_a_changed_decoder_D_or_a_relaxed_threshold(monkeypatch):
    # HH_261004 - This is a scalar numerical regression at documented geometry, with synthetic source SHAs and zero NN curvature; never actual replay.
    import torch
    import test_temporal_navigation_terminal_policy_v2_20261004 as fixtures
    from portable_e2e import temporal_navigation_terminal_policy_v2_20261004 as nav
    original=fixtures.bridge_fixture
    def numeric_geometry_fixture():
        bridge,frame,policy,args,calls=original()
        frame.position_map_m=(245.18047087861464,124.4733068551627,.03609436303412797)
        frame.orientation_xyzw=(0.,0.,.7000463312665407,.7140974261823501)
        provided,_,_=nav._canonical_route_in_base(bridge.points,position_m=frame.position_map_m,
            orientation_xyzw=frame.orientation_xyzw,contract=nav.CANONICAL_NAVIGATION,context='SYNTHETIC_SCALAR_NUMERIC_REGRESSION')
        frame.inputs.route_xy_base_m=provided
        args[5]=torch.tensor([sum(math.dist(a,b) for a,b in zip(provided,provided[1:]))],dtype=torch.float32)
        return bridge,frame,policy,args,calls
    monkeypatch.setattr(fixtures,'bridge_fixture',numeric_geometry_fixture)
    payload,states,commands,manifest=synthetic_terminal_replay_fixture(monkeypatch)
    receipt=commands[0]['navigation_terminal_research']
    commands[0]['causal_navigation_support_m']=receipt['inherited_tensor_support_m']
    witness=receipt['witness_decision']['witness']
    assert witness['remaining_mission_arc_m']!=receipt['observation_document']['remaining_mission_arc_m']
    assert receipt['navigation_owned_D_m']==min(witness['provided_route_arc_m'],witness['remaining_mission_arc_m'],64.)
    kwargs=dict(manifest=manifest,stream_source_sha256='a'*64,episode_id='temporal_v2_SYNTHETIC_CPU_ONLY')
    _,old_live=m._parents()
    with pytest.raises(Exception,match='terminal D was rounded'):
        old_live.audit_terminal_navigation(payload,states,commands,**kwargs)
    assert m.audit_terminal_navigation(payload,states,commands,**kwargs)['passed'] is True


def test_interval_evidence_survives_a_separate_recording_error(tmp_path,monkeypatch):
    # HH_261004 - Exercise independent channel retention on synthetic scalars; missing recording files remain an explicit failure.
    old,_=m._parents()
    states,_,_,context=full_context()
    route=old.read_json(old.ROUTE)
    for row in states:
        row['command']=0
    monkeypatch.setattr(old,'read_json',lambda _:route)
    monkeypatch.setattr(old,'digest',lambda _:old.live.ROUTE_SHA)
    result=m.independent_interval_and_recording(tmp_path,states=states,
        cameras=[dict(frame=row['frame']) for row in states[::2]],commands=context['commands'],
        timings=context['timings'],manifest=context['manifest'])
    assert isinstance(result.get('independent_event'),dict)
    assert 'independent_recording_error' in result and 'recording' not in result


@pytest.mark.parametrize('name',['navigation_owned_D_m','actual_provided_arc_m','actual_remaining_mission_arc_m'])
def test_witness_arc_adapter_still_rejects_actual_numeric_field_tampering(monkeypatch,name):
    payload,states,commands,manifest=synthetic_terminal_replay_fixture(monkeypatch)
    commands[0]['navigation_terminal_research'][name]+=.00001
    with pytest.raises(Exception,match='terminal D was rounded'):
        m.audit_terminal_navigation(payload,states,commands,manifest=manifest,
            stream_source_sha256='a'*64,episode_id='temporal_v2_SYNTHETIC_CPU_ONLY')
