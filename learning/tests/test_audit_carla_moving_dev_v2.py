"""HH_260906 - Check separate DEV fixed moving-window reconstruction without CARLA, training, or changed raw data."""

import pytest

from scripts.e2e import audit_carla_moving_dev_v2 as audit

# HH_260906 - Keep logical source IDs, but open canonical nonsymlink files in the moved workspace.
from learning.portable_e2e.workspace_paths import workspace_path as _hh_source_path


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


class NativeRowsReached(Exception):
    """HH_260906 - Stop a synthetic metadata audit before any actual native row payload."""


@pytest.fixture
def metadata_case(tmp_path, monkeypatch):
    from types import SimpleNamespace
    monkeypatch.setattr(audit, 'ROOT', tmp_path)
    route_bytes, selection_bytes = b'fixture route', b'fixture route selection'
    route_sha, selection_sha = audit.raw.digest(route_bytes), audit.raw.digest(selection_bytes)
    monkeypatch.setattr(audit, 'ROUTE_SHA', route_sha)
    monkeypatch.setattr(audit.newdev, 'ROUTE_SHA', route_sha)
    monkeypatch.setattr(audit.newdev, 'SELECTION_SHA', selection_sha)
    trial = tmp_path / audit.CASE
    (trial / 'episode').mkdir(parents=True)
    source_bytes = {name: ('fixture source ' + name).encode() for name in audit.newdev.SOURCE_PATHS}
    source_pins = {name: audit.raw.digest(payload) for name, payload in source_bytes.items()}
    input_paths = dict(route_file=audit.newdev.ROUTE_PATH, mapping='fixtures/mapping.yaml', calibration='fixtures/calibration.yaml')
    input_bytes = dict(route_file=route_bytes, mapping=b'mapping', calibration=b'calibration')
    input_pins = {name: audit.raw.digest(payload) for name, payload in input_bytes.items()}
    plan = audit.newdev.plan_contract() | dict(declared_at_utc='2026-09-16T00:00:00+00:00',
        output=audit.newdev.CAPTURE_PATH, input_paths=input_paths, input_sha256=input_pins, source_sha256=source_pins)
    inputs = {name: dict(path=str(tmp_path / input_paths[name]), sha256=input_pins[name],
                        archived_path='provenance/moving_inputs/' + name) for name in input_paths}
    refs = dict(schema='carla.moving_task_capture_plan.v2',
        plan=dict(path=str(tmp_path / audit.newdev.PLAN_PATH), sha256=None, archived_path='provenance/moving_task_plan.json'),
        inputs=inputs, route_selection_evidence=dict(path=str(tmp_path / audit.newdev.SELECTION_PATH),
            sha256=selection_sha, archived_path='provenance/route_selection_evidence.json'))
    owner = dict(capture_mode='expert', map='Town03', route_sha256=route_sha, quality='Epic',
        worker_path='scripts/e2e/collect_carla_moving_dev_v2.py', route_path=str(tmp_path / audit.newdev.ROUTE_PATH),
        source_sha256=dict(source_pins), moving_task_capture=refs, planned_at_utc='2026-09-16T00:01:00+00:00')
    result = dict(source_bytes_unchanged_and_archived=True, source_checks={name: True for name in source_pins},
        moving_task_capture=dict(final_manifest_sha256=None), exit_code=0)
    manifest = dict(provenance=dict(moving_task_plan_sha256=None,
        mapping_sha256=input_pins['mapping'], calibration_sha256=input_pins['calibration']))
    documents = {'owner_plan.json': owner, 'owner_result.json': result,
        'provenance/moving_task_plan.json': plan, 'episode/moving_task_plan.json': plan,
        'episode/manifest.json': manifest, 'episode/route.json': {}}
    documents.update({f'lifecycle/{stage}.json': dict(status='PASS', expected_map='Town03')
                      for stage in ('ready', 'after_capture', 'stopped')})
    archives = {'provenance/' + name: payload for name, payload in source_bytes.items()}
    archives.update({'provenance/moving_inputs/' + name: payload for name, payload in input_bytes.items()})
    archives['provenance/route_selection_evidence.json'] = selection_bytes
    row_reads = []

    def metadata(_trial, name, ledger, *, lines=False):
        if lines:
            row_reads.append(name)
            raise NativeRowsReached(name)
        document = documents[name]
        ledger[name] = dict(sha256=audit.raw.digest(audit.newdev.encoded(document)))
        return document

    def checked_bytes(_trial, name, ledger):
        payload = archives[name]
        ledger[name] = dict(sha256=audit.raw.digest(payload))
        return payload

    monkeypatch.setattr(audit.raw, 'read_metadata', metadata)
    monkeypatch.setattr(audit.raw.control_audit, 'checked_bytes', checked_bytes)

    def run():
        refs['plan']['sha256'] = audit.raw.digest(audit.newdev.encoded(plan))
        manifest['provenance']['moving_task_plan_sha256'] = refs['plan']['sha256']
        result['moving_task_capture']['final_manifest_sha256'] = audit.raw.digest(audit.newdev.encoded(manifest))
        return audit.audit(trial, audit.raw.digest(audit.newdev.encoded(owner)))

    return SimpleNamespace(run=run, owner=owner, plan=plan, refs=refs, archives=archives, row_reads=row_reads)


def test_exact_dev_metadata_reaches_native_rows(metadata_case):
    with pytest.raises(NativeRowsReached, match='episode/states.jsonl'):
        metadata_case.run()
    assert metadata_case.row_reads == ['episode/states.jsonl']


@pytest.mark.parametrize('mutation,message', [
    ('town', 'unreviewed moving owner scope'), ('owner_schema', 'DEV owner task schema differs'),
    ('plan_schema', 'DEV fixed prospective contract differs'), ('split', 'DEV fixed prospective contract differs'),
    ('source_keys', 'DEV task source closure differs'), ('input_keys', 'DEV task input closure differs'),
    ('selection_hash', 'DEV route-selection reference differs'), ('selection_bytes', 'archived DEV route-selection evidence changed'),
    ('selection_plan', 'DEV fixed prospective contract differs'), ('selection_path', 'DEV route-selection reference differs'),
])
def test_scope_rejections_precede_native_rows(metadata_case, mutation, message):
    case = metadata_case
    if mutation == 'town':
        case.owner['map'] = 'Town07'
    elif mutation == 'owner_schema':
        case.refs['schema'] = 'carla.moving_task_capture_plan.v1'
    elif mutation == 'plan_schema':
        case.plan['schema'] = 'carla.moving_task_capture_plan.v1'
    elif mutation == 'split':
        case.plan['dataset_split_intent'] = 'train'
    elif mutation == 'source_keys':
        case.plan['source_sha256']['unreviewed.py'] = '0' * 64
    elif mutation == 'input_keys':
        case.plan['input_paths']['extra'] = 'extra.json'
    elif mutation == 'selection_hash':
        case.refs['route_selection_evidence']['sha256'] = '0' * 64
    elif mutation == 'selection_bytes':
        case.archives['provenance/route_selection_evidence.json'] = b'changed'
    elif mutation == 'selection_plan':
        case.plan['route_selection_evidence']['path'] = 'other.json'
    elif mutation == 'selection_path':
        case.refs['route_selection_evidence']['path'] = '/other.json'
    with pytest.raises(ValueError, match=message):
        case.run()
    assert case.row_reads == []


def test_wrong_case_rejected_before_metadata(tmp_path, monkeypatch):
    monkeypatch.setattr(audit.raw, 'read_metadata', lambda *_: pytest.fail('wrong case must not be opened'))
    with pytest.raises(ValueError, match='fixed local capture case'):
        audit.audit(tmp_path / 'other_case', '0' * 64)


def test_diagnostic_source_archive_retains_numerical_bounds(tmp_path, monkeypatch):
    from pathlib import Path
    numerical = 'portable_e2e/model.py'
    monkeypatch.setattr(audit.raw, 'source_identity', lambda: dict(files={numerical: audit.raw.scalar.sha(_hh_source_path(audit.ROOT, numerical))}, functions={}))
    report = dict(schema='carla.moving_capture_diagnostic.v2', status='DIAGNOSED_NOT_ADMITTED',
        moving_scope_necessary_checks_clear=False, moving_scope=dict(selected_camera_anchor_count=0))
    monkeypatch.setattr(audit, 'audit', lambda *_: (report, [], [], []))
    output = tmp_path / 'diagnostic'
    audit.main(['--trial', str(_hh_source_path(audit.ROOT, audit.CASE)), '--owner-plan-sha256', '0' * 64, '--output', str(output)])
    archived = report['diagnostic_source']['files']
    assert numerical in archived and len(archived) == 4
    for name, expected in archived.items():
        assert audit.raw.scalar.sha(output / 'source_archive' / name) == expected
        assert (output / 'source_archive' / name).read_bytes() == (_hh_source_path(audit.ROOT, name)).read_bytes()
    assert str(Path(audit.__file__).relative_to(audit.ROOT)) in archived
