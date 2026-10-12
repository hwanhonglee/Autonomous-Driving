"""HH_261001 - Fixed-window audit rejects trimming, old failures and unbound controls."""

from copy import deepcopy
import pytest

from scripts.e2e import audit_town07_low_speed_moving_pending_20261001 as audit


@pytest.fixture
def capture():
    route = {'route': [dict(x=0., y=0.), dict(x=210., y=0.)]}
    states, cameras, native, anchors, futures = [], [], [], [], []
    for frame in range(1289):
        phase = 'stationary_warmup' if frame < 70 else 'driving' if frame <= 1158 else 'stationary_tail'
        progress = max(0, min(frame, 1158) - 70) * .125
        state = dict(frame=frame, timestamp=frame * .05, capture_phase=phase, x=progress, y=0.,
            moving_task=dict(measured_route_progress_m=progress, original_termination_reason=None))
        states.append(state)
        checks = {key: True for key in audit.NATIVE_CHECKS}
        checks['driving_phase'] = phase == 'driving'
        native.append(dict(frame=frame, timestamp_ns=frame * 50_000_000, capture_phase=phase,
                           checks=checks, scalar_rate_mps2=0., longitudinal_dvx_dt_mps2=0.))
        if frame % 2 == 0:
            cameras.append(dict(frame=frame, timestamp=frame * .05, capture_phase=phase,
                source_timestamps={'camera': frame * .05}, moving_task_selected=390 <= frame <= 1030,
                moving_task_measured_route_progress_m=progress))
            anchors.append(dict(timestamp_ns=frame * 50_000_000, same_recorded_frame_and_timestamp=True))
            futures.append(dict(valid_future_points=64, future_crosses_stationary_tail=False,
                discrete_bound_diagnostic=dict(steps=[dict(metrics={metric: (0., False) for metric in audit.raw.targets.METRICS})
                                                    for _ in range(64)])))
    return route, states, cameras, native, anchors, futures


def census(capture):
    route, states, cameras, native, anchors, futures = capture
    membership, _ = audit.reconstruct_membership(route, states, cameras)
    windows, protected = audit.window_census(membership, cameras, native, anchors, futures)
    return membership, windows, protected


def test_full_fixed_denominator_and_real_native_guard(capture):
    original = deepcopy(capture)
    membership, windows, protected = census(capture)
    assert all(membership['flags'].values())
    assert len(windows) == 321 and all(w['necessary_window_checks_clear'] for w in windows)
    assert windows[0]['frame'] == 390 and windows[-1]['frame'] == 1030
    assert min(protected) == 371 and max(protected) == 1158
    assert windows[0]['protected_native_frames'] == [371, 518]
    assert capture == original


@pytest.mark.parametrize('native_index,blocks', [(100, False), (371, True), (500, True), (1158, True), (1159, False)])
def test_pulse_scope_is_fixed_and_outside_failures_remain(capture, native_index, blocks):
    capture[3][native_index]['checks']['native_longitudinal_rate'] = False
    capture[3][native_index]['longitudinal_dvx_dt_mps2'] = 18.
    membership, windows, protected = census(capture)
    assert all(membership['flags'].values())
    assert len(windows) == 321
    assert any(not row['necessary_window_checks_clear'] for row in windows) is blocks
    evidence = audit.scoped_failures(capture[3], protected)
    assert evidence['protected' if blocks else 'outside_protected'][0]['frame'] == native_index
    assert evidence['protected' if blocks else 'outside_protected'][0]['longitudinal_dvx_dt_mps2'] == 18.


def test_saved_membership_cannot_drop_bad_anchor(capture):
    capture[2][195]['moving_task_selected'] = False
    membership, windows, _ = census(capture)
    assert not membership['flags']['saved_membership_and_progress_clear']
    assert len(windows) == 321


def test_future_and_history_fail_without_trimming(capture):
    capture[5][195]['valid_future_points'] = 63
    capture[5][195]['future_crosses_stationary_tail'] = True
    capture[4][186]['same_recorded_frame_and_timestamp'] = False
    capture[5][194]['discrete_bound_diagnostic']['steps'][0]['metrics']['xy_curvature'] = (.21, True)
    _, windows, _ = census(capture)
    assert len(windows) == 321
    assert set(windows[0]['failure_reasons']) == {'full_64_actual_future_before_tail',
        'causal_history_camera_native_binding', 'all_nine_history_edge_bounds'}


def test_all_nine_and_body_sweep_failures_remain(capture):
    capture[3][500]['checks']['sweep_inset'] = False
    capture[5][200]['discrete_bound_diagnostic']['steps'][63]['metrics']['xy_lateral_acceleration'] = (2.81, True)
    _, windows, _ = census(capture)
    assert 'sweep_inset' in windows[0]['failure_reasons']
    assert 'all_nine_future_bounds' in windows[5]['failure_reasons']
    assert len(windows) == 321


def test_missing_history_guard_and_native_future_bracket_are_not_passes(capture):
    capture[2][186]['frame'] += 1
    # HH_261001 - Missing frame fails rather than choosing a later anchor with easier history.
    with pytest.raises((KeyError, ValueError)):
        census(capture)


def test_missing_last_future_bracket_keeps_last_anchor_failed(capture):
    route, states, cameras, native, anchors, futures = capture
    membership, _ = audit.reconstruct_membership(route, states, cameras)
    windows, _ = audit.window_census(membership, cameras, native[:1158], anchors, futures)
    assert len(windows) == 321
    assert 'full_64_actual_future_before_tail' in windows[-1]['failure_reasons']


def identity_fixture():
    plan = audit.task.plan_contract()
    plan.update(output=str(audit.task.RUN / 'episode'), declared_at_utc='2026-10-01T00:00:00+00:00')
    manifest = dict(created_at='2026-10-01T00:01:00+00:00', admission_status='PENDING_QA',
        control_source='BasicAgent_fixed_v2_normal_policy_new_moving_task',
        capture_contract=dict(moving_task=dict(plan_sha256=audit.PLAN_SHA, cohort_slot_id=audit.task.SLOT)))
    return plan, manifest


def test_identity_requires_new_plan_not_old_failed_source():
    plan, manifest = identity_fixture()
    audit.validate_identity(audit.task.RUN, audit.PLAN_SHA, plan, manifest)
    with pytest.raises(ValueError):
        audit.validate_identity(audit.task.RUN, '0' * 64, plan, manifest)
    with pytest.raises(ValueError):
        audit.validate_identity(audit.task.RUN.parent / 'town07_low_speed_start_expert_v2', audit.PLAN_SHA, plan, manifest)
    manifest['capture_contract']['low_speed_start_research'] = {'status': 'failed'}
    with pytest.raises(ValueError):
        audit.validate_identity(audit.task.RUN, audit.PLAN_SHA, plan, manifest)


def test_retroactive_declaration_and_true_approval_rejected():
    plan, manifest = identity_fixture()
    plan['declared_at_utc'] = '2026-10-02T00:00:00+00:00'
    with pytest.raises(ValueError):
        audit.validate_identity(audit.task.RUN, audit.PLAN_SHA, plan, manifest)
    plan, manifest = identity_fixture()
    manifest['training_data_approved'] = True
    with pytest.raises(ValueError):
        audit.validate_identity(audit.task.RUN, audit.PLAN_SHA, plan, manifest)


def control_fixture():
    policy, rows, receipts, states = audit.v2.CombinedControl(), [], [], []
    for i in range(3):
        frame, timestamp, speed = 10 + i, (10 + i) * .05, i * .1
        state = dict(frame=frame, timestamp=timestamp, vx=speed, vy=0., moving_task=dict(termination_reason=None))
        states.append(state)
        proposal = dict(throttle=.75, brake=0., steer=.1, reverse=False, hand_brake=False, manual_gear_shift=False, gear=0)
        control = audit.RecordedFloatControl(dict(proposal, throttle=.2))
        if i < 2:
            policy.apply(control, speed=speed, timestamp=timestamp, frame=frame, normal_proposal=proposal)
            normal = deepcopy(policy.last)
        else:
            normal = None
            state['moving_task']['termination_reason'] = 'moving_task_future_support_complete'
            control = audit.RecordedFloatControl(dict(proposal, throttle=0., brake=1., steer=0.))
        final = audit.v2.base.control_dict(control)
        receipt = dict(reason='initial_drive_control' if i == 0 else 'next_driving_control',
                       before_frame=frame, sequence=i + 1, requested_control=final)
        receipts.append(receipt)
        rows.append(dict(ack_receipt=receipt, normal_policy=normal,
            target_ramp=dict(source_frame=frame if i < 2 else frame - 1, target_speed_mps=2.5 if i == 0 else .075),
            final_basic_agent_proposal=final, learned_control=False))
    manifest = dict(capture_contract=dict(fixed_moving_normal_control=dict(effective_control=dict(inherited_basic_agent_max_brake=.5))))
    return rows, receipts, states, manifest


def test_control_replay_float32_and_cleanup_are_not_teacher_fallback():
    fixture = control_fixture()
    original = deepcopy(fixture)
    summary, rows = audit.control_audit(*fixture)
    assert summary['status'] == 'PASS' and summary['emergency_override_count'] == 0
    assert rows[-1]['cleanup_command']
    assert fixture == original


def test_control_wrong_frame_or_missing_ack_rejected():
    fixture = control_fixture()
    fixture[0][0]['normal_policy']['source_frame'] += 1
    with pytest.raises(ValueError):
        audit.control_audit(*fixture)
    fixture = control_fixture()
    fixture[0].pop()
    with pytest.raises(ValueError):
        audit.control_audit(*fixture)


def test_helper_pins_and_frozen_capture_sources_are_unchanged():
    for name, digest in audit.AUDIT_HELPER_PINS.items():
        assert audit.sha(audit.task.ROOT / name) == digest
    assert audit.sha(audit.task.PLAN) == audit.PLAN_SHA
    for name, digest in audit.read(audit.task.PLAN)['source_sha256'].items():
        assert audit.sha(audit.task.ROOT / name) == digest
