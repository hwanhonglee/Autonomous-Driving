"""HH_260906 - Check fixed moving-window reconstruction without CARLA, training, or changed raw data."""

import pytest

from scripts.e2e import audit_carla_moving_capture as audit
from scripts.e2e import carla_moving_capture as moving


def sealed_plan(slot_id):
    """HH_260906 - Build a complete prospective fixture from one sealed moving-slot contract."""
    fixed = moving.plan_contract(slot_id, 15.)
    return fixed | dict(
        declared_at_utc='2026-09-21T00:00:00+00:00',
        output=moving.slot_spec(slot_id)['capture_path'],
        input_paths={'route_file': 'route.json', 'mapping': 'mapping.json', 'calibration': 'calibration.json'},
        input_sha256={'route_file': '1' * 64, 'mapping': '2' * 64, 'calibration': '3' * 64},
        source_sha256={name: '4' * 64 for name in moving.SOURCE_PATHS},
    )


@pytest.mark.parametrize('slot_id', moving.RECOVERY_SLOT_IDS)
def test_sealed_capture_scope_accepts_each_declared_recovery_slot(monkeypatch, slot_id):
    """HH_260906 - Admit metadata only for the five predeclared TRAIN straight recovery slots."""
    monkeypatch.setattr(moving, 'verify_cohort_slot',
                        lambda observed: observed == slot_id or pytest.fail('wrong sealed slot checked'))
    spec = moving.slot_spec(slot_id)
    owner = dict(capture_mode='expert', map=spec['town'], route_sha256=spec['route_sha256'], quality='Epic')
    assert audit.sealed_capture_scope(owner, sealed_plan(slot_id)) == dict(
        slot_id=slot_id, town=spec['town'], route_id=spec['route_id'],
        route_sha256=spec['route_sha256'], plan_schema='carla.moving_train_straight_capture_plan.v1')


def test_sealed_capture_scope_retains_legacy_t07_s01_contract(monkeypatch):
    """HH_260906 - Preserve the original T07-S01 audit path while sealing recovery-slot expansion."""
    slot_id = 'T07-S01'
    monkeypatch.setattr(moving, 'verify_cohort_slot',
                        lambda observed: observed == slot_id or pytest.fail('wrong sealed slot checked'))
    spec = moving.slot_spec(slot_id)
    owner = dict(capture_mode='expert', map=spec['town'], route_sha256=spec['route_sha256'], quality='Epic')
    scope = audit.sealed_capture_scope(owner, sealed_plan(slot_id))
    assert scope['slot_id'] == slot_id
    assert scope['plan_schema'] == 'carla.moving_task_capture_plan.v1'


@pytest.mark.parametrize('changed', ['town', 'route', 'slot'])
def test_sealed_capture_scope_rejects_wrong_town_route_or_slot(monkeypatch, changed):
    """HH_260906 - Reject owner or plan metadata that crosses the declared slot boundary."""
    slot_id = 'T07-S02'
    monkeypatch.setattr(moving, 'verify_cohort_slot', lambda observed: None)
    spec = moving.slot_spec(slot_id)
    owner = dict(capture_mode='expert', map=spec['town'], route_sha256=spec['route_sha256'], quality='Epic')
    plan = sealed_plan(slot_id)
    if changed == 'town':
        owner['map'] = 'Town01'
    elif changed == 'route':
        owner['route_sha256'] = '0' * 64
    else:
        plan['cohort_slot_id'] = 'CT-S01'
    with pytest.raises(Exception, match='scope|contract'):
        audit.sealed_capture_scope(owner, plan)


def fixture():
    route = dict(route=[dict(x=0., y=0., distance_m=0.), dict(x=300., y=0., distance_m=300.)])
    rows, cameras = [], []
    last_selected_ns = None
    support = False
    for frame in range(1, 3000):
        if frame <= 70:
            phase, progress = 'stationary_warmup', 0.
        elif support:
            phase, progress = 'stationary_tail', rows[-1]['x']
        else:
            phase, progress = 'driving', (frame - 70) / 5.
        stamp = 100. + frame / 20.
        row = dict(frame=frame, timestamp=stamp, capture_phase=phase, x=progress, y=0.,
                   moving_task=dict(measured_route_progress_m=progress, original_termination_reason=None))
        rows.append(row)
        if frame % 2:
            selected = phase == 'driving' and 30. <= progress <= 165.
            cameras.append(dict(frame=frame, timestamp=stamp, capture_phase=phase,
                                source_timestamps={'CAM_FRONT': stamp}, moving_task_selected=selected,
                                moving_task_measured_route_progress_m=progress))
            if selected:
                last_selected_ns = round(stamp * 1e9)
        if phase == 'driving' and progress > 165. and last_selected_ns is not None and round(stamp * 1e9) >= last_selected_ns + 6_400_000_000:
            support = True
        if sum(r['capture_phase'] == 'stationary_tail' for r in rows) == 130:
            break
    return route, rows, cameras


def test_observed_window_and_future_before_tail():
    route, rows, cameras = fixture()
    report, _ = audit.reconstruct_window(route, rows, cameras)
    assert report['status'] == 'MOVING_SCOPE_REPLAY_CLEAR'
    assert report['selected_camera_anchor_count'] == 338
    assert report['selected_camera_span_ns'] == 33_700_000_000


def test_missing_membership_cannot_remove_bad_anchor():
    route, rows, cameras = fixture()
    item = next(c for c in cameras if c['moving_task_selected'])
    item['moving_task_selected'] = False
    report, _ = audit.reconstruct_window(route, rows, cameras)
    assert report['selected_camera_anchor_count'] == 338
    assert report['status'] == 'MOVING_SCOPE_REPLAY_FAILED'
    assert report['failures'][0]['reason'] == 'saved_camera_membership_mismatch'


def test_short_context_is_not_extrapolated():
    route, rows, cameras = fixture()
    before_tail = next(i for i, r in enumerate(rows) if r['capture_phase'] == 'stationary_tail')
    removed = rows[before_tail - 1]['frame']
    rows.pop(before_tail - 1)
    cameras = [c for c in cameras if c['frame'] != removed]
    report, _ = audit.reconstruct_window(route, rows, cameras)
    assert report['flags']['actual_full_future_before_tail'] is False


def test_projection_is_recomputed_not_taken_on_trust():
    route, rows, cameras = fixture()
    rows[100]['moving_task']['measured_route_progress_m'] += 1.
    with pytest.raises(Exception, match='stored moving projection differs'):
        audit.reconstruct_window(route, rows, cameras)


def test_selection_uses_planar_arc_not_elevated_catalog_arc():
    route = dict(route=[dict(x=0., y=0., distance_m=0.), dict(x=300., y=0., distance_m=500.)])
    arc, error = audit.independent_projection(route, dict(x=150., y=0.))
    assert arc == 150.
    assert error == 0.


def test_missing_future_denominator_is_failure(monkeypatch):
    monkeypatch.setattr(audit.targets, 'summarize_samples', lambda rows, horizon: dict(metrics={}))
    report = audit.selected_future_summary([dict(frame=1, valid_points=63, diagnostic={})], [1])
    assert report['selected_anchor_count'] == 1
    assert report['all_selected_necessary_bounds_clear'] is False


def test_failure_metrics_remain_in_selected_summary(monkeypatch):
    monkeypatch.setattr(audit.targets, 'summarize_samples', lambda rows, horizon: dict(metrics={'xy_curvature': dict(violating_anchor_count=1)}))
    report = audit.selected_future_summary([dict(frame=1, valid_points=64, diagnostic={})], [1])
    assert report['unchanged_nine_metric_failures']['xy_curvature'] == 1
    assert report['all_selected_necessary_bounds_clear'] is False


def test_recorded_failure_inside_window_does_not_remove_geometric_membership():
    route, rows, cameras = fixture()
    failed = next(row for row in rows if row['capture_phase']=='driving' and row['x']>=90.)
    failed['moving_task']['original_termination_reason']='synthetic_original_failure'
    report,_=audit.reconstruct_window(route,rows,cameras)
    assert report['selected_camera_anchor_count']==338
    assert report['status']=='MOVING_SCOPE_REPLAY_FAILED'
    assert any(row['frame']==failed['frame'] and row['reason']=='catalog_or_agent_end_before_support'
               for row in report['failures'])


def test_first_crossing_above_end_closes_selection_permanently_on_reentry():
    route,rows,cameras=fixture()
    closing=next(row for row in rows if row['capture_phase']=='driving' and row['x']>165.)
    reentry=next(row for row in rows if row['frame']==closing['frame']+1)
    reentry['x']=reentry['moving_task']['measured_route_progress_m']=164.
    camera=next(row for row in cameras if row['frame']==reentry['frame'])
    camera['moving_task_measured_route_progress_m']=164.
    assert camera['moving_task_selected'] is False
    report,_=audit.reconstruct_window(route,rows,cameras)
    assert report['first_closing_observation']['frame']==closing['frame']
    assert report['selected_camera_anchor_count']==338
    assert reentry['frame'] not in report['selected_camera_frames']
    assert any(row['reason']=='decreasing_progress' for row in report['failures'])


def test_monotonicity_starts_at_first_selected_saved_camera_not_unsaved_crossing():
    route,rows,cameras=fixture()
    first=next(camera for camera in cameras if camera['moving_task_selected'])
    row=next(row for row in rows if row['frame']==first['frame'])
    row['x']=row['moving_task']['measured_route_progress_m']=29.9
    first['moving_task_measured_route_progress_m']=29.9
    first['moving_task_selected']=False
    report,_=audit.reconstruct_window(route,rows,cameras)
    assert report['status']=='MOVING_SCOPE_REPLAY_CLEAR'
    assert report['selected_camera_anchor_count']==337
    assert report['first_selected_camera']['frame']>first['frame']


def test_short_tail_cannot_satisfy_fixed_capture_completion():
    route,rows,cameras=fixture()
    removed=rows[-1]['frame'];rows=rows[:-1]
    cameras=[row for row in cameras if row['frame']!=removed]
    report,_=audit.reconstruct_window(route,rows,cameras)
    assert report['flags']['exact_tail_130_states'] is False
    assert report['status']=='MOVING_SCOPE_REPLAY_FAILED'


@pytest.mark.parametrize('kind',['existing','inside_raw'])
def test_output_refusal_precedes_any_raw_or_diagnostic_execution(tmp_path,monkeypatch,kind):
    raw=tmp_path/'raw';raw.mkdir()
    output=raw/'new-output' if kind=='inside_raw' else tmp_path/'existing'
    if kind=='existing':output.mkdir();(output/'sentinel').write_text('unchanged')
    monkeypatch.setattr(audit,'audit',lambda *_:pytest.fail('raw data must not be read for unsafe output'))
    with pytest.raises(Exception,match='fresh output'):
        audit.main(['--trial',str(raw),'--owner-plan-sha256','0'*64,'--output',str(output)])
    if kind=='existing':assert (output/'sentinel').read_text()=='unchanged'


def test_selected_future_summary_is_read_only_and_keeps_failure_record(monkeypatch):
    from copy import deepcopy
    rows=[dict(frame=1,valid_points=64,diagnostic={'sentinel':'failing original'})]
    original=deepcopy(rows)
    def summarize(selected,horizon):
        assert selected==[rows[0]['diagnostic']] and horizon==64
        return dict(metrics={'xy_curvature':dict(violating_anchor_count=1)})
    monkeypatch.setattr(audit.targets,'summarize_samples',summarize)
    report=audit.selected_future_summary(rows,[1])
    assert rows==original
    assert report['all_selected_necessary_bounds_clear'] is False
