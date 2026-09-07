"""HH_260906 - Reject corrupted, leaky, miscounted or stale scorer-only research evidence."""

from copy import deepcopy
from dataclasses import asdict
import json
from pathlib import Path

import pytest

from portable_e2e.contract import ContractError
from scripts.e2e import summarize_portable_frozen_selector as summary


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, allow_nan=False))
    return summary.digest(path.read_bytes())


def read_json(path):
    return json.loads(path.read_text())


def metric(split, after=False):
    n = summary.SPLIT_SIZES[split]
    values = {name: 1.0 for name in summary.METRICS}
    values.update(selected_ade_m=2.5 if after else 3.0, selected_fde_m=5.0 if after else 6.0,
        selected_speed_mae_mps=0.9 if after else 1.0, oracle_ade_m=1.0, regression_loss=1.0,
        ade_6p4s_m=2.5 if after else 3.0, fde_6p4s_m=5.0 if after else 6.0)
    return {'split': split, 'sample_count': n, 'metrics': values,
        'metric_counts': {name: n for name in summary.METRICS},
        'selected_histogram': [n, 0, 0, 0, 0, 0], 'composite_oracle_histogram': [n, 0, 0, 0, 0, 0],
        'selected_composite_oracle_agreement_count': n}


def history():
    rows, order, seen = [], [], 0
    for offset in range(1540):
        epoch, step_in_epoch = divmod(offset, 287)
        indices = list(range(step_in_epoch * 4, min(step_in_epoch * 4 + 4, 1147)))
        seen += len(indices)
        rows.append({'global_step': offset + 1, 'epoch': epoch, 'batch_samples': len(indices),
            'samples_seen': seen, 'sample_indices': indices, 'candidate_score_loss': 2.0,
            'optimization_loss': 0.2, 'gradient_norm': 1.0})
        order.append({'epoch': epoch, 'indices': indices})
    return rows, summary.digest(json.dumps(order, separators=(',', ':')).encode())


def geometry(route_dir, sample_ids):
    candidates = [{'candidate_index': index, 'geometry_pass': True, 'failure_codes': []} for index in range(6)]
    rows = [{'candidates': deepcopy(candidates), 'all_candidates_geometry_pass': True,
        'any_candidate_geometry_pass': True, 'selected_candidate_index': 0,
        'selected_geometry_pass': True, 'selected_failure_codes': []} for _ in range(337)]
    accumulator = summary._AuditAccumulator(summary.RuntimeGateConfig())
    for row in rows:
        accumulator.add(row)
    route_dir.mkdir(parents=True)
    rendered = []
    for index in (0, 67, 134, 201, 268, 336):
        name = f'val_phase_{index:03d}.png'
        (route_dir / name).write_bytes(b'opaque synthetic PNG fixture')
        rendered.append({'index': index, 'sample_id': sample_ids[index], 'path': name,
            'sha256': summary.digest((route_dir / name).read_bytes())})
    return {'gate': {'source': summary.RUNTIME_GATE_ID, **asdict(summary.RuntimeGateConfig())}, 'summary': accumulator.report(),
        'per_sample': rows, 'rendered': rendered, 'vehicle_control_approved': False}


@pytest.fixture
def campaign(tmp_path, monkeypatch):
    root, parent = tmp_path / 'campaign', tmp_path / 'parent'
    root.mkdir()
    parent.mkdir()
    plan_bytes = summary.runner.PLAN_PATH.read_bytes()
    plan = json.loads(plan_bytes)
    commit = 'b' * 40
    source_bytes = {'portable_e2e/frozen_selector.py': b'pinned synthetic core',
        'scripts/e2e/diagnose_portable_selector.py': b'pinned synthetic helper',
        'scripts/e2e/run_portable_frozen_selector.py': b'pinned synthetic runner',
        'config/portable_e2e_frozen_selector_20260908.json': plan_bytes}
    monkeypatch.setattr(summary, '_git_bytes', lambda revision, path: source_bytes[path])
    monkeypatch.setattr(summary, 'source_paths', lambda revision: sorted(list(source_bytes)[:2]))
    (root / 'provenance').mkdir()
    (root / 'provenance/active_runner.py').write_bytes(source_bytes['scripts/e2e/run_portable_frozen_selector.py'])
    (root / 'provenance/plan.json').write_bytes(plan_bytes)
    source = {'diagnostic_source_commit': commit,
        'source_sha256': {name: summary.digest(source_bytes[name]) for name in list(source_bytes)[:2]}}
    flags = {'vehicle_control_approved': False, 'automatic_promotion': False, 'test_evaluated': False,
        'test_used_for_training_or_selection': False, 'dataset_integrity_scope': summary.INTEGRITY_SCOPE}
    state = {'schema': 'portable_e2e.frozen_selector_campaign.v1',
        'status': 'FROZEN_SELECTOR_CAMPAIGN_COMPLETE_NOT_PROMOTED', 'source_commit': commit,
        'source': source, 'plan': plan, 'plan_sha256': summary.digest(plan_bytes),
        'runner_sha256': summary.digest(source_bytes['scripts/e2e/run_portable_frozen_selector.py']),
        'stages': [], 'completed_parent_caches': 3, 'completed_head_fits': 9, **flags}
    rows, order_sha = history()
    parent_manifest = []
    for seed in summary.runner.PARENT_SHA256:
        seed_root = root / f'seed_{seed}'
        seed_root.mkdir()
        parent_files = {}
        for name in ('training/run.json', 'evaluation/metrics.json', 'gate_v8.json'):
            relative = f'seed_{seed}/C_expanded_data/{name}'
            parent_files[name] = write_json(parent / relative, {'synthetic_seed': seed, 'file': name})
            parent_manifest.append({'path': relative, 'sha256': parent_files[name]})
        started = {'schema': summary.runner.SCHEMA, 'status': 'PREFLIGHT', 'seed': seed,
            'started_at_utc': '2026-09-08T00:01:00+00:00', 'script_sha256': state['runner_sha256'],
            'core_sha256': source['source_sha256']['portable_e2e/frozen_selector.py'], 'source': source,
            'plan': plan, 'plan_sha256': state['plan_sha256'], 'parent_checkpoint_sha256': summary.runner.PARENT_SHA256[seed],
            'parent_report_sha256': parent_files, 'manifest_sha256': summary.runner.MANIFEST_SHA256,
            'deadline_utc': plan['deadline_utc'], 'arms': [], **flags}
        write_json(seed_root / 'started.json', started)
        result = deepcopy(started)
        result.update(status='HEAD_TRAIN_EVAL_AUDIT_COMPLETE_NOT_PROMOTED',
            completed_at_utc='2026-09-08T00:02:00+00:00', loss_config=summary.TrajectoryLossConfig().to_dict(),
            gpu_uuid=summary.runner.GPU_UUID, generator_state_sha256='a' * 64, generator_state_after_sha256='a' * 64,
            model_config={'model_id': summary.runner.PHYSICAL_MODEL_ID},
            provenance={'checkpoint_sha256': summary.runner.PARENT_SHA256[seed],
                'training_dataset_fingerprint_sha256': summary.runner.SPLITS['train'][1],
                'training_episode_count': 3, 'evaluation_episode_count': 1, 'model_parameter_count': 954590}, cache_files={})
        for split, n in summary.SPLIT_SIZES.items():
            path = seed_root / f'{split}_cache.pt'
            path.write_bytes(f'opaque cache {seed} {split}'.encode())
            result['cache_files'][split] = {'sha256': summary.digest(path.read_bytes()),
                'cache_digest': summary.digest(f'cache tensors {seed} {split}'.encode()), 'sample_count': n}
        ids = {split: [f'{split}_episode:{index}' for index in range(n)] for split, n in summary.SPLIT_SIZES.items()}
        baseline = {split: metric(split) for split in summary.SPLIT_SIZES}
        write_json(seed_root / 'original_c_geometry.json', geometry(seed_root / 'original_c_routes', ids['val']))
        for arm in summary.runner.ARMS:
            arm_root = seed_root / arm
            arm_root.mkdir()
            head = {'artifact_id': summary.ARTIFACT_ID, 'status': 'RESEARCH_HEAD_TRAINING_COMPLETE_NOT_PROMOTED',
                'arm': arm, 'seed': seed, 'device': 'cuda:0', 'optimizer': 'AdamW', 'steps': 1540, 'batch_size': 4,
                'learning_rate': 0.0001, 'weight_decay': 0.0001, 'candidate_score_weight': 0.1,
                'maximum_gradient_norm': 5.0, 'loss_config': summary.TrajectoryLossConfig().to_dict(),
                'source_checkpoint_sha256': summary.runner.PARENT_SHA256[seed],
                'corpus_fingerprint_sha256': summary.runner.CORPUS_SHA256,
                'split_fingerprints': {split: summary.runner.SPLITS[split][1] for split in summary.SPLIT_SIZES},
                'head_parameter_count': 115201 if arm == 'candidate_reset' else 1542,
                'sampling_policy': 'Uniform without replacement via train._epoch_batches no-plan branch; CPU randperm seeded with seed+epoch. This is not the original domain-balanced order.',
                'sampling_order_sha256': order_sha, 'train_composite_targets_sha256': 'd' * 64,
                'cache_sha256_before': {split: item['cache_digest'] for split, item in result['cache_files'].items()},
                'cache_sha256_after': {split: item['cache_digest'] for split, item in result['cache_files'].items()},
                'original_head_sha256_before': 'e' * 64, 'original_head_sha256_after': 'e' * 64,
                'original_logit_parity': {split: {'cached_original_logits_supplied': True,
                    'max_abs_logit_difference': 0.0, 'selected_index_mismatch_count': 0} for split in summary.SPLIT_SIZES},
                'train_sample_ids': ids['train'], 'val_sample_ids': ids['val'],
                'baseline_metrics': deepcopy(baseline), 'pre_training_metrics': deepcopy(baseline),
                'post_training_metrics': {split: metric(split, True) for split in summary.SPLIT_SIZES},
                'history': rows, 'final_state': {'global_step': 1540, 'samples_seen': 6155, 'last_epoch_index': 5, 'last_epoch_steps': 105},
                'vehicle_control_approved': False, 'automatic_promotion': False}
            write_json(arm_root / 'report.json', head)
            (arm_root / 'metrics.jsonl').write_text(''.join(json.dumps(row) + '\n' for row in rows))
            for name in ('head_only.pt', 'final_logits.pt'):
                (arm_root / name).write_bytes(f'opaque {seed} {arm} {name}'.encode())
            write_json(arm_root / 'geometry.json', geometry(arm_root / 'routes', ids['val']))
            result['arms'].append({'arm': arm, 'status': 'COMPLETE_NOT_PROMOTED',
                'files': {name: summary.digest((arm_root / name).read_bytes()) for name in summary.ARM_FILES}})
        result_sha = write_json(seed_root / 'result.json', result)
        state['stages'].append({'seed': seed, 'status': 'COMPLETE_NOT_PROMOTED', 'returncode': 0,
            'completed_head_fits': 3, 'result_sha256': result_sha,
            'started_at_utc': '2026-09-08T00:00:59+00:00', 'finished_at_utc': '2026-09-08T00:02:01+00:00',
            'command': ['/private/venvs/py312/bin/python', '/private/repo/scripts/e2e/run_portable_frozen_selector.py',
                '--seed', str(seed), '--expected-source-commit', commit, '--output-dir',
                f'/private/{plan["campaign_id"]}/seed_{seed}', '--deadline-utc', plan['deadline_utc']]})
    write_json(root / 'status.json', state)
    monkeypatch.setattr(summary.parent_summary, 'summarize_campaign', lambda path:
        {'status': 'COMPLETE_NOT_PROMOTED', 'input_manifest': parent_manifest})
    return root, parent


def reseal(root, seed=20260903, arm='linear_continue'):
    result_path = root / f'seed_{seed}/result.json'
    result = read_json(result_path)
    record = next(item for item in result['arms'] if item['arm'] == arm)
    record['files'] = {name: summary.digest((root / f'seed_{seed}/{arm}/{name}').read_bytes()) for name in summary.ARM_FILES}
    result_sha = write_json(result_path, result)
    state = read_json(root / 'status.json')
    next(item for item in state['stages'] if item['seed'] == seed)['result_sha256'] = result_sha
    write_json(root / 'status.json', state)


def test_complete_nine_head_fits_not_full_models(campaign):
    result = summary.summarize_campaign(*campaign)
    assert result['status'] == 'COMPLETE_NOT_PROMOTED'
    assert result['completed_head_fits'] == 9 and result['scratch_full_model_training_runs'] == 0
    assert all(value == {'candidate_screen': 'PASS', 'absolute_quality': 'FAIL'} for value in result['arm_screens'].values())
    assert len([line for line in summary.render_markdown(result).splitlines() if line.startswith('| 2026')]) == 12
    assert len({row['path'] for row in result['input_manifest']}) == len(result['input_manifest'])
    assert result['test_evaluated'] is False and 'test_opened' not in result


def test_missing_campaign_is_incomplete(tmp_path):
    assert summary.summarize_campaign(tmp_path / 'missing', tmp_path / 'parent')['status'] == 'INCOMPLETE'


@pytest.mark.parametrize('field,value', [('vehicle_control_approved', True), ('test_evaluated', True),
    ('test_used_for_training_or_selection', True), ('dataset_integrity_scope', 'test untouched'),
    ('completed_head_fits', 8), ('completed_parent_caches', True), ('source_commit', 'unpinned')])
def test_campaign_scope_and_counts_fail_closed(campaign, field, value):
    root, parent = campaign
    state = read_json(root / 'status.json')
    state[field] = value
    write_json(root / 'status.json', state)
    with pytest.raises(ContractError):
        summary.summarize_campaign(root, parent)


@pytest.mark.parametrize('field,value', [('cache_sha256_after', {'train': '0' * 64, 'val': '0' * 64}),
    ('original_head_sha256_after', '0' * 64), ('train_composite_targets_sha256', '0' * 64),
    ('head_parameter_count', 954590), ('sampling_order_sha256', '0' * 64),
    ('artifact_id', 'normal-runtime-checkpoint'), ('sampling_policy', 'domain balanced')])
def test_head_freeze_identity_and_fairness_fail_closed(campaign, field, value):
    root, parent = campaign
    path = root / 'seed_20260903/linear_continue/report.json'
    head = read_json(path)
    head[field] = value
    write_json(path, head)
    reseal(root)
    with pytest.raises(ContractError):
        summary.summarize_campaign(root, parent)


def test_one_regressing_seed_prevents_all_seed_pass(campaign):
    root, parent = campaign
    path = root / 'seed_20260903/linear_continue/report.json'
    head = read_json(path)
    head['post_training_metrics']['val']['metrics']['selected_ade_m'] = 4.0
    write_json(path, head)
    reseal(root)
    result = summary.summarize_campaign(root, parent)
    assert result['arm_screens']['linear_continue']['candidate_screen'] == 'FAIL'
    assert result['arm_screens']['linear_reset']['candidate_screen'] == 'PASS'


@pytest.mark.parametrize('mutation', ['oracle', 'baseline', 'leakage', 'denominator', 'agreement', 'parity', 'geometry'])
def test_semantic_mutation_rejected_even_after_hashes_resealed(campaign, mutation):
    root, parent = campaign
    path = root / 'seed_20260903/linear_continue/report.json'
    head = read_json(path)
    if mutation == 'oracle':
        head['post_training_metrics']['val']['metrics']['oracle_ade_m'] += 0.1
    elif mutation == 'baseline':
        head['pre_training_metrics']['val']['metrics']['selected_ade_m'] += 0.1
    elif mutation == 'leakage':
        head['train_sample_ids'][0] = head['val_sample_ids'][0]
    elif mutation == 'denominator':
        head['post_training_metrics']['val']['metric_counts']['selected_ade_m'] = 336
    elif mutation == 'agreement':
        head['post_training_metrics']['val']['selected_composite_oracle_agreement_count'] = 338
    elif mutation == 'parity':
        head['original_logit_parity']['val']['selected_index_mismatch_count'] = 1
    else:
        geo_path = path.parent / 'geometry.json'
        geo = read_json(geo_path)
        geo['per_sample'][0]['candidates'][0]['extra_changed_geometry'] = 1
        write_json(geo_path, geo)
    write_json(path, head)
    reseal(root)
    with pytest.raises(ContractError):
        summary.summarize_campaign(root, parent)


@pytest.mark.parametrize('target', ['train_cache.pt', 'linear_continue/head_only.pt',
    'original_c_routes/val_phase_000.png', 'linear_continue/routes/val_phase_336.png'])
def test_opaque_artifact_and_route_bytes_are_bound(campaign, target):
    root, parent = campaign
    (root / f'seed_20260903/{target}').write_bytes(b'corruption')
    with pytest.raises(ContractError, match='SHA-256 mismatch'):
        summary.summarize_campaign(root, parent)


@pytest.mark.parametrize('mutation', ['step', 'batch', 'duplicate', 'score_weight', 'nonfinite', 'boolean'])
def test_history_full_coverage_and_numeric_safety(mutation):
    rows, order_sha = history()
    if mutation == 'step':
        rows[-1]['global_step'] = 1539
    elif mutation == 'batch':
        rows[286]['batch_samples'] = 4
    elif mutation == 'duplicate':
        rows[-1]['sample_indices'] = rows[-2]['sample_indices']
    elif mutation == 'score_weight':
        rows[0]['optimization_loss'] = 1.0
    elif mutation == 'nonfinite':
        rows[0]['gradient_norm'] = 'NaN'
    else:
        rows[0]['epoch'] = False
    report = {'history': rows, 'sampling_order_sha256': order_sha,
        'final_state': {'global_step': 1540, 'samples_seen': 6155, 'last_epoch_index': 5, 'last_epoch_steps': 105}}
    with pytest.raises(ContractError):
        summary.validate_history(('\n'.join(json.dumps(row) for row in rows)).encode(), report)


def test_cli_validates_before_creating_output_and_preserves_existing(campaign, tmp_path):
    root, parent = campaign
    output = tmp_path / 'summary'
    args = [str(root), '--parent-campaign', str(parent), '--output-dir', str(output)]
    assert summary.main(args) == 0
    old = (output / 'summary.json').read_bytes()
    with pytest.raises(FileExistsError):
        summary.main(args)
    assert (output / 'summary.json').read_bytes() == old
    (root / 'seed_20260903/train_cache.pt').write_bytes(b'bad')
    with pytest.raises(ContractError):
        summary.main(args[:-1] + [str(tmp_path / 'bad_output')])
    assert not (tmp_path / 'bad_output').exists()


def test_source_command_deadline_and_parent_must_match(campaign):
    root, parent = campaign
    state = read_json(root / 'status.json')
    state['stages'][0]['command'][3] = '20260904'
    write_json(root / 'status.json', state)
    with pytest.raises(ContractError, match='seed command'):
        summary.summarize_campaign(root, parent)
    with pytest.raises(ContractError, match='deadline'):
        summary.validate_stage({**state['stages'][1], 'finished_at_utc': '2026-09-08T02:00:00Z'},
            20260904, state['source_commit'], state['plan'])
    with pytest.raises(ContractError):
        summary.Inputs(root, parent).bytes('campaign', '../outside')


@pytest.mark.parametrize('field,value', [('source', 'portable_e2e.runtime_geometry_gate.v7'),
    ('source', None), ('maximum_speed_mps', 16.666666666666668)])
def test_recorded_gate_source_and_thresholds_remain_strict(campaign, field, value):
    root, parent = campaign
    path = root / 'seed_20260903/linear_continue/geometry.json'
    record = read_json(path)
    if value is None:
        record['gate'].pop(field)
    else:
        record['gate'][field] = value
    write_json(path, record)
    reseal(root)
    with pytest.raises(ContractError, match='geometry gate'):
        summary.summarize_campaign(root, parent)
