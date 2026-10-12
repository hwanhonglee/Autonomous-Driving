"""HH_260906 - Evidence/aggregation tests without neural execution, TEST data or checkpoint loading."""
from copy import deepcopy
from dataclasses import asdict
from datetime import datetime, timedelta, timezone
import hashlib
import json
from pathlib import Path

import pytest

from scripts.e2e import summarize_stopmix_regret_research as m

# HH_260906 - Keep logical source IDs, but open canonical nonsymlink files in the moved workspace.
from learning.portable_e2e.workspace_paths import workspace_path as _hh_source_path


def row(index=0, selected=0, group='continuing_motion'):
    candidates, gates = [], []
    for ci in range(12):
        stop = ci >= 6
        candidates.append(dict(candidate_index=ci, family='STOP' if stop else 'DRIVE',
            ade_m=float(ci+1), fde_m=float(ci+2), speed_mae_mps=ci/10.,
            terminal_speed_mps=0. if stop else 1., terminal_exact_zero=stop,
            terminal_at_or_below_0p1_mps=stop, first_exact_zero_future_index=20 if stop else None,
            reacceleration_after_exact_future_zero=False, nonincreasing_speed_exact=True))
        gates.append(dict(candidate_index=ci, geometry_pass=ci != 2, failure_codes=['curvature'] if ci == 2 else []))
    chosen = candidates[selected]
    return dict(index=index, sample_id=f'sample:{index}', episode_id='episode', sequence_index=index,
        anchor_timestamp_ns=1000000000+index*100000000, camera_sha256=['a'*64]*6, source_manifest_sha256='b'*64,
        target_motion_group=group, valid_future_points=32 if group == 'unavailable_masks' else 64,
        raw_current_vx_mps=1., candidate_count=12, selected_candidate_index=selected,
        selected_stop_candidate=selected >= 6, selected_ade_m=chosen['ade_m'], selected_fde_m=chosen['fde_m'],
        selected_speed_mae_mps=chosen['speed_mae_mps'], oracle_ade_m=1., ade_selection_regret_m=chosen['ade_m']-1.,
        selected_speed_behavior={k: chosen[k] for k in m.BEHAVIOR}, candidates=candidates,
        runtime_geometry=dict(candidates=gates, all_candidates_geometry_pass=False, any_candidate_geometry_pass=True,
            selected_candidate_index=selected, selected_geometry_pass=gates[selected]['geometry_pass'],
            selected_failure_codes=gates[selected]['failure_codes']))


def test_reconstructs_all_scalar_metrics_counts_geometry_and_empty_group():
    result = m.verify_rows([row(), row(1, 6)], 2)
    all_rows = result['all_samples']
    assert all_rows['metrics']['selected_ade_m'] == 4.
    assert all_rows['metrics']['selected_fde_m'] == 5.
    assert all_rows['metrics']['selected_speed_mae_mps'] == .3
    assert all_rows['metrics']['ade_selection_regret_m'] == 3.
    assert all_rows['stop_selection_count'] == all_rows['selected_terminal_exact_zero_count'] == 1
    assert all_rows['selected_reacceleration_after_exact_future_zero_count'] == 0
    geometry = all_rows['geometry']
    assert geometry['selected_result']['geometry_pass_count'] == 2
    assert geometry['candidate_results'][2]['failure_counts'] == {'curvature': 2}
    assert geometry['all_candidates_geometry_pass_count'] == 0
    assert geometry['any_candidate_geometry_pass_count'] == 2
    assert geometry['selector']['dominant_candidate_index'] == 0
    assert geometry['selector']['normalized_selection_entropy'] == pytest.approx(.27894294565112987)
    assert result['target_motion_groups']['stationary_hold'] == dict(sample_count=0, metrics=None, geometry=None, stop_selection_count=None)


def test_only_entropy_accepts_one_e_minus_16_and_records_exact_difference():
    stored = {'geometry': {'selector': {'normalized_selection_entropy': .4}}}
    measured = deepcopy(stored)
    measured['geometry']['selector']['normalized_selection_entropy'] += 1e-16
    differences = []
    m.compare_aggregates(stored, measured, 'case/train', differences)
    assert len(differences) == 1
    assert differences[0]['path'] == 'case/train/geometry/selector/normalized_selection_entropy'
    assert differences[0]['absolute_delta'] == abs(.4-measured['geometry']['selector']['normalized_selection_entropy'])


@pytest.mark.parametrize('field,delta', [('normalized_selection_entropy', 1e-14), ('geometry_pass_rate', 1e-16), ('geometry_pass_count', 1)])
def test_entropy_beyond_bound_and_every_other_aggregate_difference_fail(field, delta):
    stored = {'geometry': {'selector': {field: .4 if field != 'geometry_pass_count' else 1}}}
    measured = deepcopy(stored); measured['geometry']['selector'][field] += delta
    with pytest.raises(ValueError, match='aggregate scalar differs'):
        m.compare_aggregates(stored, measured, 'case/train', [])


def test_boolean_cannot_impersonate_an_equal_aggregate_count():
    with pytest.raises(ValueError, match='aggregate type differs'):
        m.compare_aggregates({'sample_count': 1}, {'sample_count': True}, 'case/train', [])


@pytest.mark.parametrize('mutation', [
    lambda r: r.update(selected_ade_m=99.),
    lambda r: r.update(oracle_ade_m=99.),
    lambda r: r.update(ade_selection_regret_m=99.),
    lambda r: r.update(selected_stop_candidate=True),
    lambda r: r.update(selected_candidate_index=True),
    lambda r: r.update(valid_future_points=0),
    lambda r: r.update(valid_future_points=32),
    lambda r: r.update(raw_current_vx_mps=float('nan')),
    lambda r: r['candidates'][0].update(ade_m=True),
    lambda r: r['candidates'][0].update(family='STOP'),
    lambda r: r['candidates'][0].update(terminal_exact_zero=True),
    lambda r: r['candidates'][0].update(reacceleration_after_exact_future_zero=True),
    lambda r: r['candidates'][0].update(first_exact_zero_future_index=64),
    lambda r: r['runtime_geometry'].update(selected_geometry_pass=False),
    lambda r: r['runtime_geometry'].update(all_candidates_geometry_pass=True),
    lambda r: r['runtime_geometry']['candidates'][0].update(failure_codes=['curvature']),
    lambda r: r['runtime_geometry']['candidates'][2].update(failure_codes=['unknown']),
])
def test_contradictory_nonfinite_or_malformed_scalars_fail(mutation):
    value = row(); mutation(value)
    with pytest.raises(ValueError):
        m.verify_rows([value], 1)


@pytest.mark.parametrize('rows,count', [([], 0), ([row(), row()], 2), ([row(1)], 1), ([row()], 2)])
def test_full_canonical_row_identity_required(rows, count):
    with pytest.raises(ValueError):
        m.verify_rows(rows, count)


def history(arm):
    values = []
    for step in range(1, 1541):
        seen = 4*step-step//287
        value = dict(global_step=step, epoch=(step-1)//287, samples_seen=seen,
            domain_samples_seen={'carla': seen}, batch_domain_sample_counts={'carla': 3 if step%287 == 0 else 4},
            loss=.5, regression_loss=.4, candidate_score_loss=.1, gradient_norm=.2,
            selected_ade_m=.5, selected_fde_m=1., selected_speed_mae_mps=.1,
            selected_yaw_mae_rad=.1, selected_kinematic_speed_mae_mps=.1)
        if arm == 'B_cost_aware':
            value['candidate_regret_loss'] = .3
        values.append(value)
    return values


@pytest.mark.parametrize('arm', ['A_original_loss', 'B_cost_aware'])
def test_complete_optimizer_exposure_includes_five_short_batches(arm):
    values = history(arm)
    m.verify_history(values, arm)
    assert values[-1]['samples_seen'] == 6155
    assert sum(v['batch_domain_sample_counts']['carla'] == 3 for v in values) == 5


@pytest.mark.parametrize('kind', ['missing', 'duplicate', 'short_batch', 'nonfinite', 'wrong_loss'])
def test_optimizer_history_gaps_exposure_and_objective_fail(kind):
    values = history('A_original_loss')
    if kind == 'missing':
        values.pop()
    elif kind == 'duplicate':
        values[1] = values[0]
    elif kind == 'short_batch':
        values[286]['batch_domain_sample_counts']['carla'] = 4
    elif kind == 'nonfinite':
        values[0]['gradient_norm'] = float('inf')
    else:
        values[0]['candidate_regret_loss'] = .1
    with pytest.raises(ValueError):
        m.verify_history(values, 'A_original_loss')


def arm_result(ade=2., fde=4., speed=1., geometry=1, selected=0):
    aggregate = m.verify_rows([row(0, selected)], 1)
    return dict(native_metrics=dict(selected_ade_m=ade, selected_fde_m=fde, selected_speed_mae_mps=speed,
        ade_1p0s_m=.5, ade_3p0s_m=1., ade_6p4s_m=2., fde_6p4s_m=4., loss=1000.),
        geometry_pass_count=geometry, behavior={'train': deepcopy(aggregate), 'val': deepcopy(aggregate)})


def test_absolute_equality_is_allowed_but_paired_ade_fde_must_strictly_improve():
    limits = m.runner.plan_contract()['decision']['absolute_limits_m']
    same = m.pair_decision(arm_result(), arm_result(), limits)
    assert same['absolute_gate_pass'] is True and same['paired_comparison_pass'] is False
    improved = m.pair_decision(arm_result(), arm_result(1.9, 3.9, 1.05), limits)
    assert improved['paired_comparison_pass'] is True
    assert 'loss' not in improved['metric_delta_B_minus_A']
    assert improved['subgroup_deltas']['val']['stationary_hold']['metric_delta_B_minus_A'] is None


@pytest.mark.parametrize('kwargs', [dict(ade=2.1, fde=3.9), dict(ade=1.9, fde=4.1),
    dict(ade=1.9, fde=3.9, speed=1.0500001), dict(ade=1.9, fde=3.9, geometry=0)])
def test_every_individual_paired_regression_fails_without_averaging(kwargs):
    value = m.pair_decision(arm_result(), arm_result(**kwargs), m.runner.plan_contract()['decision']['absolute_limits_m'])
    assert value['paired_comparison_pass'] is False


def test_stop_selection_increase_and_other_subgroup_regressions_remain_visible():
    value = m.pair_decision(arm_result(), arm_result(1.9, 3.9, selected=6), m.runner.plan_contract()['decision']['absolute_limits_m'])
    group = value['subgroup_deltas']['train']['continuing_motion']
    assert group['metric_delta_B_minus_A']['selected_ade_m'] == 6.
    assert group['count_delta_B_minus_A']['stop_selection_count'] == 1
    assert group['count_delta_B_minus_A']['selected_terminal_exact_zero_count'] == 1


def transport_fixture(tmp_path):
    root = tmp_path/'mirror'; root.mkdir()
    payload = b'{"status":"fixture"}\n'
    (root/'summary.json').write_bytes(payload)
    value = hashlib.sha256(payload).hexdigest()
    remote = {'summary.json': value, 'seed_20260903/A_original_loss/training/checkpoints/latest.pt': 'a'*64}
    receipt = tmp_path/'transport.json'
    report = dict(schema='portable_e2e.stopmix_regret_transport.v1', status='COLLECTED',
        plan_sha256=m.PLAN_SHA, ssh_exit_code=0, checkpoint_payloads_copied=False,
        remote_inventory_unchanged=True, collected_at_utc='2026-09-14T09:00:00Z',
        remote_file_sha256=remote, copied_file_sha256={'summary.json': value})
    receipt.write_text(json.dumps(report))
    return root, receipt, report


def test_exact_transport_mirror_never_opens_remote_checkpoint(tmp_path, monkeypatch):
    root, receipt, report = transport_fixture(tmp_path)
    original = Path.open
    def guard(path, *args, **kwargs):
        assert path.suffix != '.pt', 'checkpoint payload must not be opened'
        return original(path, *args, **kwargs)
    monkeypatch.setattr(Path, 'open', guard)
    result = m.verify_transport(root, receipt, m.digest(receipt))
    assert result == report


@pytest.mark.parametrize('kind', ['bad_receipt_sha', 'nonzero_exit', 'wrong_plan', 'changed_remote', 'missing', 'extra', 'copied_checkpoint', 'symlink', 'unsafe_name', 'changed_payload'])
def test_untrusted_incomplete_or_mutated_transport_fails(tmp_path, kind):
    root, receipt, report = transport_fixture(tmp_path)
    if kind == 'nonzero_exit': report['ssh_exit_code'] = 1
    elif kind == 'wrong_plan': report['plan_sha256'] = 'f'*64
    elif kind == 'changed_remote': report['remote_inventory_unchanged'] = False
    elif kind == 'missing': (root/'summary.json').unlink()
    elif kind == 'extra': (root/'unexpected.json').write_text('{}')
    elif kind == 'copied_checkpoint': (root/'latest.pt').write_bytes(b'forbidden')
    elif kind == 'symlink': (root/'alias.json').symlink_to(root/'summary.json')
    elif kind == 'unsafe_name': report['remote_file_sha256']['../outside'] = 'a'*64
    elif kind == 'changed_payload': (root/'summary.json').write_text('changed')
    receipt.write_text(json.dumps(report))
    pin = '0'*64 if kind == 'bad_receipt_sha' else m.digest(receipt)
    with pytest.raises(ValueError):
        m.verify_transport(root, receipt, pin)


def test_strict_json_rows_reject_duplicate_keys_and_nonfinite(tmp_path):
    path = tmp_path/'rows.jsonl'
    for payload in ('{"x":1,"x":2}\n', '{"x":NaN}\n'):
        path.write_text(payload)
        with pytest.raises(ValueError):
            m.json_rows(path)


def test_external_prospective_plan_and_normalized_archive_have_distinct_byte_pins(tmp_path, monkeypatch):
    plan, archive = tmp_path/'prospective.json', tmp_path/'archived.json'
    value = {'prospective': True, 'settings': [1, 2, 3]}
    plan.write_text(json.dumps(value, indent=2)+'\n')
    archive.write_bytes(m.runner.encoded(value))
    monkeypatch.setattr(m, 'PLAN_SHA', m.digest(plan))
    assert m.digest(plan) != m.digest(archive)
    assert m.verify_plan_archive(plan, archive) == value
    archive.write_bytes(plan.read_bytes())
    with pytest.raises(ValueError, match='normalization'):
        m.verify_plan_archive(plan, archive)
    archive.write_bytes(m.runner.encoded(dict(value, prospective=False)))
    with pytest.raises(ValueError, match='normalization'):
        m.verify_plan_archive(plan, archive)


def test_failed_verification_writes_no_publication(tmp_path, monkeypatch):
    root = tmp_path/'mirror'; root.mkdir()
    plan, receipt = tmp_path/'plan.json', tmp_path/'receipt.json'
    plan.write_text('{}'); receipt.write_text('{}')
    output = tmp_path/'output'
    def fail(*args):
        raise ValueError('incomplete fits')
    monkeypatch.setattr(m, 'summarize', fail)
    with pytest.raises(ValueError, match='incomplete fits'):
        m.main(['--input-root', str(root), '--plan', str(plan), '--transport-receipt', str(receipt),
            '--transport-sha256', 'a'*64, '--output-dir', str(output)])
    assert not output.exists()


def test_only_two_create_only_output_files(tmp_path, monkeypatch):
    root = tmp_path/'mirror'; root.mkdir()
    plan, receipt = tmp_path/'plan.json', tmp_path/'receipt.json'
    plan.write_text('{}'); receipt.write_text('{}')
    output = tmp_path/'output'
    monkeypatch.setattr(m, 'summarize', lambda *args: ({'status': 'PASS_NOT_PROMOTED'}, {'status': 'SIX_FRESH_FITS_VERIFIED_NOT_PROMOTED'}))
    args = ['--input-root', str(root), '--plan', str(plan), '--transport-receipt', str(receipt),
        '--transport-sha256', 'a'*64, '--output-dir', str(output)]
    m.main(args)
    assert {p.name for p in output.iterdir()} == {'verification.json', 'paired_summary.json'}
    with pytest.raises(ValueError, match='fresh nonsymlink'):
        m.main(args)


def test_complete_six_fit_transport_receipts_and_scalars_without_checkpoint_payloads(tmp_path, monkeypatch):
    """HH_260906 - Exercise the real frozen stage schemas at full counts using entirely synthetic model outputs."""
    root = tmp_path/'mirror'; root.mkdir()
    def write(path, value):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(m.runner.encoded(value))
    def lines(path, values):
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open('w') as stream:
            for value in values:
                stream.write(json.dumps(value, separators=(',', ':'))+'\n')
    sources = {name: m.digest(_hh_source_path(m.ROOT, name)) for name in m.runner.SOURCE_PATHS}
    for name in sources:
        target = root/'source'/name; target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes((_hh_source_path(m.ROOT, name)).read_bytes())
    (root/'source/run_stopmix_regret_research.py').write_bytes(Path(m.runner.__file__).read_bytes())
    (root/'source/profile_portable_training.py').write_bytes((_hh_source_path(m.ROOT, 'scripts/e2e/profile_portable_training.py')).read_bytes())
    plan = dict(m.runner.plan_contract(), declared_at_utc='2026-09-14T00:00:00Z', worker_sha256=m.WORKER_SHA,
        source_sha256=sources, dataset='datasets/prepared/synthetic-trainval', dataset_manifest_sha256='d'*64,
        view_report_path='receipts/view.json', view_report_sha256='e'*64, view_file_sha256={'dataset.json': 'd'*64},
        split_contract={'train': {'sample_count': 1147, 'fingerprint_sha256': 'a'*64},
            'val': {'sample_count': 337, 'fingerprint_sha256': 'b'*64}}, corpus_fingerprint_sha256='c'*64)
    plan_path = tmp_path/'prospective.json'; plan_path.write_text(json.dumps(plan, indent=2)+'\n')
    monkeypatch.setattr(m, 'PLAN_SHA', m.digest(plan_path))
    write(root/'plan.json', plan)
    instant = datetime(2026, 9, 14, 0, 1, tzinfo=timezone.utc)
    idle = dict(physical_index=0, uuid=m.runner.GPU_UUID, pre_cuda_compute_processes=0)
    started = dict(schema='portable_e2e.stopmix_regret_research.v1', status='RUNNING',
        started_at_utc=instant.isoformat(), plan_sha256=m.PLAN_SHA, pre_cuda_gpu=idle,
        stages=[], runs={name: 'NOT_RUN' for name in m.runner.RUNS}, **m.runner.DENIALS)
    write(root/'started.json', started)
    config = m.read(root/'source'/m.runner.MODEL_PATH)
    config_sha = hashlib.sha256(json.dumps(config, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
    workspace = Path('/synthetic/personal/hwanhong/portable_e2e')
    remote_output = workspace/'runs/diagnostics/stopmix_regret_20260914_v1'
    checkpoints, receipts = {}, []
    for run in m.runner.RUNS:
        item = root/run; arm = run.split('/')[1]; selected = 6 if arm == 'B_cost_aware' else 0
        checkpoint = hashlib.sha256(run.encode()).hexdigest()
        checkpoints[run+'/training/checkpoints/latest.pt'] = checkpoint
        scalar = {}; raw = {}
        renders = []
        for split, count in m.COUNTS.items():
            values = []
            for index in range(count):
                value = row(index, selected)
                value['sample_id'] = f'{split}:sample:{index}'
                value['episode_id'] = f'train_episode_{index%3}' if split == 'train' else 'val_episode'
                values.append(value)
            raw[split] = values
            lines(item/'behavior'/f'{split}_samples.jsonl', values)
            scalar[split] = dict(m.verify_rows(values, count), samples_sha256=m.digest(item/'behavior'/f'{split}_samples.jsonl'))
        for index in m.runner.RENDER_INDICES:
            name = f'trajectories/val_{index:03d}.png'
            path = item/'behavior'/name; path.parent.mkdir(parents=True, exist_ok=True); path.write_bytes(b'synthetic plot bytes')
            renders.append(dict(index=index, path=name, sha256=m.digest(path)))
        behavior = dict(schema='portable_e2e.stopmix_regret_behavior.v1', status='COMPLETE_NOT_PROMOTED',
            checkpoint_sha256=checkpoint, corpus_fingerprint_sha256='c'*64, splits=scalar, renders=renders,
            vehicle_control_approved=False, test_read_or_use=False)
        write(item/'behavior/summary.json', behavior)
        train_history = history(arm)
        lines(item/'training/metrics.jsonl', train_history)
        loss = dict(m.runner.LOSS)
        if selected: loss['candidate_regret_weight'] = .1
        train = dict(corpus_fingerprint_sha256='c'*64, dataset_fingerprint_sha256='a'*64,
            device='cuda:0', hardware={'device_uuid': m.runner.GPU_UUID.removeprefix('GPU-')},
            status='TRAINING_TARGET_REACHED', warning='RESEARCH CHECKPOINT — NOT APPROVED FOR VEHICLE CONTROL',
            dataset_size=1147, model_parameter_count=1056362, training_split='train',
            training_episode_ids=[f'train_episode_{i}' for i in range(3)],
            state={'global_step': 1540, 'samples_seen': 6155, 'domain_samples_seen': {'carla': 6155}},
            last_metrics=train_history[-1], model_config=config, loss_config=loss,
            train_config=dict(seed=int(run.split('/')[0][5:]), batch_size=4, learning_rate=1e-4, weight_decay=1e-4,
                max_steps=1540, checkpoint_interval=154, num_workers=0, maximum_gradient_norm=5.,
                verify_image_sha256=True, sampling_policy='uniform_without_replacement', domain_ratios=[]))
        write(item/'training/run.json', train)
        metrics = {k: v for k, v in scalar['val']['all_samples']['metrics'].items() if k != 'ade_selection_regret_m'}
        metrics.update(loss=.5, regression_loss=.4, candidate_score_loss=.1, selected_yaw_mae_rad=.1, selected_kinematic_speed_mae_mps=.1)
        for label in ('1p0', '3p0', '6p4'):
            metrics.update({f'ade_{label}s_m': metrics['selected_ade_m'], f'fde_{label}s_m': metrics['selected_fde_m'], f'speed_mae_{label}s_mps': metrics['selected_speed_mae_mps']})
        evaluation = dict(corpus_fingerprint_sha256='c'*64, dataset_fingerprint_sha256='b'*64,
            training_dataset_fingerprint_sha256='a'*64, model_config_sha256=config_sha, checkpoint_sha256=checkpoint,
            vehicle_control_approved=False, device='cuda:0', hardware=train['hardware'], evaluation_split='val',
            status='OPEN_LOOP_EVALUATION_COMPLETE', sample_count=337, model_parameter_count=1056362,
            metrics=metrics, metric_counts={key: 337 for key in metrics}, rendered_trajectory_png=12,
            training_episode_count=3, evaluation_episode_count=1)
        if selected:
            evaluation.update(loss_config=loss, auxiliary_loss_metrics={'candidate_regret_loss': .1}, auxiliary_loss_metric_counts={'candidate_regret_loss': 337})
        write(item/'evaluation/metrics.json', evaluation)
        for index, value in enumerate(raw['val'][:12]):
            name = f'{index:04d}_'+value['sample_id'].replace(':', '_')+'.png'
            path = item/'evaluation/trajectories'/name; path.parent.mkdir(exist_ok=True); path.write_bytes(b'synthetic native plot bytes')
        audit = dict(corpus_fingerprint_sha256='c'*64, dataset_fingerprint_sha256='b'*64,
            training_dataset_fingerprint_sha256='a'*64, model_config_sha256=config_sha, checkpoint_sha256=checkpoint,
            vehicle_control_approved=False, evaluation_split='val', status='RUNTIME_GEOMETRY_AUDIT_COMPLETE',
            geometry=scalar['val']['all_samples']['geometry'], gate=dict(source='portable_e2e.runtime_geometry_gate.v8',
                thresholds=asdict(m.RuntimeGateConfig(candidate_count=12, future_points=64)), threshold_overrides=False),
            implementation={key+'_sha256': sources['portable_e2e/'+key+'.py'] for key in ('audit_runtime', 'runtime_contract')})
        write(item/'gate_v8.json', audit)
        from types import FunctionType
        command = FunctionType(m.runner.command.__code__, dict(vars(m.runner), digest=lambda path: checkpoint))
        for stage, timeout in m.runner.TIMEOUTS.items():
            argv = command(plan, workspace, workspace/'autoware_e2e', remote_output, run, stage,
                workspace/'benchmarks/plan.json', workspace/'benchmarks/run_stopmix_regret_research.py')
            initial = dict(run=run, stage=stage, command=argv, timeout_seconds=timeout, started_at_utc=instant.isoformat(), status='RUNNING')
            instant += timedelta(seconds=1)
            name = {'train': 'training/run.json', 'evaluate': 'evaluation/metrics.json', 'audit': 'gate_v8.json', 'behavior': 'behavior/summary.json'}[stage]
            completed = dict(initial, status='COMPLETE', ended_at_utc=instant.isoformat(), path=run+'/'+name,
                sha256=m.digest(item/name), checkpoint_sha256=checkpoint)
            write(item/f'{stage}_started.json', initial); write(item/f'{stage}_completed.json', completed)
            (item/f'{stage}.log').write_text('synthetic completed stage\n')
            receipts.append(completed)
    summary = dict(started, status='COMPLETE_NOT_PROMOTED', runs={name: 'COMPLETE' for name in m.runner.RUNS},
        stages=receipts, paired_results=m.runner.paired_results(root), source_and_view_postcheck_pass=True,
        ended_at_utc=instant.isoformat(), final_gpu=idle)
    write(root/'summary.json', summary)
    copied = {p.relative_to(root).as_posix(): m.digest(p) for p in root.rglob('*') if p.is_file()}
    receipt = tmp_path/'transport.json'
    transport = dict(schema='portable_e2e.stopmix_regret_transport.v1', status='COLLECTED', plan_sha256=m.PLAN_SHA,
        ssh_exit_code=0, remote_file_sha256=dict(copied, **checkpoints), copied_file_sha256=copied,
        checkpoint_payloads_copied=False, remote_inventory_unchanged=True, collected_at_utc=instant.isoformat())
    write(receipt, transport)
    original = Path.open
    def guard(path, *args, **kwargs):
        assert path.suffix != '.pt', 'a real checkpoint payload was opened'
        return original(path, *args, **kwargs)
    monkeypatch.setattr(Path, 'open', guard)
    verification, result = m.summarize(root, plan_path, receipt, m.digest(receipt))
    assert verification['fitted_models'] == 6 and verification['optimizer_history_rows'] == 9240
    assert verification['behavior_rows'] == 8904
    assert verification['checkpoint_payloads_loaded'] is False
    assert verification['archived_plan_sha256'] != verification['plan_sha256']
    assert result['all_three_paired_comparisons_pass'] is False
    assert len(result['seeds']) == 3 and all(r['metric_delta_B_minus_A']['selected_ade_m'] == 6. for r in result['seeds'])
