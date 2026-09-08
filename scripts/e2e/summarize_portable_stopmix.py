#!/usr/bin/env python3
"""HH_260906 - Verify fresh K6/K12 development results and separate behavior evidence without promoting models."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import re

from portable_e2e.contract import ContractError, _loads_json
from portable_e2e.runtime_contract import RuntimeGateConfig, RUNTIME_GATE_ID
from scripts.e2e import summarize_portable_candidate_regret as common
from scripts.e2e import summarize_portable_training_campaign as base
from scripts.e2e import summarize_portable_data_expansion as expansion

ROOT = Path(__file__).resolve().parents[2]
SCHEMA, PLAN_SCHEMA = 'portable_e2e.stopmix_summary.v1', 'portable_e2e.stopmix_development_campaign.v1'
CAMPAIGN_ID = 'hh260909-stopmix-development-ab-3seeds-v1'
ARMS = {'A_physical_drive': .0001, 'B_drive_stop_mix': .0001}
MODELS = {'A_physical_drive': common.MODEL,
    'B_drive_stop_mix': 'portable_e2e/config/perspective_trajectory_physical_stopmix_v1.model.json'}
MODEL_SHA256 = {'A_physical_drive': common.MODEL_SHA256,
    'B_drive_stop_mix': '29e537e8b216cff0216772624f670ece0f0849f0f2ec55c0b726f11c1d74b602'}
MODEL_IDS = {'A_physical_drive': 'portable_e2e.perspective_trajectory.physical.v1',
    'B_drive_stop_mix': 'portable_e2e.perspective_trajectory.physical_stopmix.v1'}
COUNTS, PARAMETERS = {'A_physical_drive': 6, 'B_drive_stop_mix': 12}, {'A_physical_drive': 954590, 'B_drive_stop_mix': 1056362}
PRIMITIVE_SHA256 = '4021eb2ba6d84260b6b658114fbe31a1e7ef826121e4d07690bfd4154919a92c'
SOURCE_PATHS = ('portable_e2e/model.py', 'portable_e2e/stop_primitive_research.py', 'portable_e2e/train.py',
    'portable_e2e/losses.py', 'portable_e2e/torch_dataset.py', 'portable_e2e/evaluate.py',
    'portable_e2e/audit_runtime.py', 'portable_e2e/runtime_contract.py', common.WORKER, *MODELS.values())
BEHAVIOR_SCRIPT = 'scripts/e2e/audit_portable_stopmix_behavior.py'
BEHAVIOR_SOURCES = (BEHAVIOR_SCRIPT, 'portable_e2e/model.py', 'portable_e2e/stop_primitive_research.py',
    'portable_e2e/contract.py', 'portable_e2e/dataset.py', 'portable_e2e/torch_dataset.py', 'portable_e2e/train.py',
    'portable_e2e/evaluate.py', 'portable_e2e/losses.py', 'portable_e2e/audit_runtime.py', 'portable_e2e/runtime.py',
    'portable_e2e/runtime_contract.py', 'portable_e2e/visualize.py', *MODELS.values())
RENDER_INDICES = (0, 31, 61, 92, 122, 153, 183, 214, 244, 275, 305, 336)
FAILURE_CODES = {'nonfinite_or_shape', 'spatial', 'speed', 'geometric_speed', 'speed_disagreement', 'distance_disagreement',
    'speed_rate', 'geometric_speed_rate', 'stationary_drift', 'first_distance', 'first_behind', 'backward_step',
    'step', 'heading', 'curvature', 'lateral_acceleration', 'extent', 'contract'}
require, read, sha_file, canonical, git_bytes = base._require, base._read, common.sha_file, common.canonical, common.git_bytes


def checked_hash(path):
    require(path.is_file() and all(not p.is_symlink() for p in (path, *path.parents)), 'nonregular or symlink input')
    return sha_file(path)


def archive_name(name):
    return 'provenance/' + ('active_runner.py' if name == common.WORKER else name)


def timestamp(value):
    # HH_260906 - Accept the behavior writer's UTC Z suffix on both Python 3.10 and the remote 3.12 interpreter.
    require(isinstance(value, str), 'missing evidence timestamp')
    try:
        return common.shared._time(value.removesuffix('Z') + '+00:00' if value.endswith('Z') else value)
    except ValueError as error:
        raise ContractError('invalid evidence timestamp') from error


def validate_plan(plan, state, payload, expected_source_commit, expected_plan_sha256):
    # HH_260906 - Explicit caller pins bind this reader to one externally declared campaign, not any self-consistent future plan.
    require(re.fullmatch('[0-9a-f]{40}', expected_source_commit or '') and re.fullmatch('[0-9a-f]{64}', expected_plan_sha256 or ''),
        'explicit source and plan pins are required')
    fixed = dict(schema=PLAN_SCHEMA, campaign_id=CAMPAIGN_ID, source_commit=expected_source_commit,
        gpu_uuid='GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5', dataset=common.DATASET,
        dataset_manifest_sha256=expansion.MANIFEST_SHA256, seeds=base.SEEDS, arms=ARMS,
        model_configs=MODELS, model_config_sha256=MODEL_SHA256, candidate_score_weight=.1,
        stop_primitive_sha256=PRIMITIVE_SHA256, steps=1540, batch_size=4, split='val',
        expected_train_samples=1147, expected_val_samples=337, finish_before_utc=common.DEADLINE,
        stage_timeout_seconds=common.STAGE_TIMEOUTS, finish_reserve_seconds=300,
        behavior_analysis_count=6, behavior_analysis_timeout_seconds=180)
    require(all(canonical(plan.get(k)) == canonical(v) for k, v in fixed.items()) and list(plan['arms']) == list(ARMS),
        'STOPMIX prospective plan differs from fixed scope')
    require(not any(k in plan for k in ('model_config', 'candidate_regret_weight', 'candidate_regret_weights',
        'resume', 'checkpoint', 'prerequisite_campaign_id', 'prerequisite_timeout_seconds', 'baseline_campaign_id', 'baseline_source_commit')),
        'STOPMIX must use six fresh original-loss fits')
    require(base._sha(payload) == expected_plan_sha256 == state.get('plan_sha256') and state.get('plan') == plan
        and state.get('source_commit') == expected_source_commit, 'external plan/source hash binding mismatch')
    require(plan.get('decision', {}).get('absolute_limits_m') == {k.removesuffix('_m'): v for k, v in base.ABSOLUTE_LIMITS.items()},
        'absolute criteria changed')
    require(state.get('reviewed_contract') == dict(train_samples=1147, val_samples=337, run_count=6, stage_count=18)
        and state.get('dataset_manifest_sha256') == expansion.MANIFEST_SHA256 and state.get('prerequisite') is None
        and state.get('vehicle_control_approved') is False, 'campaign scope/approval mismatch')
    require(state.get('behavior_analysis') == {'status': 'NOT_RUN_SEPARATE_WORKFLOW', 'expected_count': 6,
        'per_analysis_timeout_seconds': 180, 'included_in_completed_stage_count': False}, 'normal and behavior stages were conflated')
    sources = {name: git_bytes(expected_source_commit, name) for name in SOURCE_PATHS}
    behavior_plan = plan.get('behavior_analysis', {})
    behavior_fixed = {'script': BEHAVIOR_SCRIPT, 'source_sha256': base._sha(git_bytes(expected_source_commit, BEHAVIOR_SCRIPT)),
        'model_source_sha256': base._sha(sources['portable_e2e/model.py']), 'expected_pass_count': 6,
        'expected_samples_per_pass': 337, 'batch_size': 4, 'split': 'val', 'render_indices': list(RENDER_INDICES)}
    require(all(canonical(behavior_plan.get(k)) == canonical(v) for k, v in behavior_fixed.items()), 'prospective behavior source/shape/render plan differs')
    require(state.get('runner_sha256') == base._sha(sources[common.WORKER])
        and base._sha(sources['portable_e2e/stop_primitive_research.py']) == PRIMITIVE_SHA256, 'runner/primitive source pin mismatch')
    require(state.get('source_files') == {n: {'sha256': base._sha(v), 'archive_path': archive_name(n)} for n, v in sources.items()},
        'exact eleven pipeline source archive declarations required')
    configs = {arm: _loads_json(sources[name].decode(), name) for arm, name in MODELS.items()}
    expected_models = {arm: {'path': MODELS[arm], 'sha256': MODEL_SHA256[arm], 'model_config': config,
        'canonical_sha256': base._sha(canonical(config).encode())} for arm, config in configs.items()}
    require(state.get('model_configs') == expected_models and all(base._sha(sources[MODELS[a]]) == MODEL_SHA256[a]
        and c.get('model_id') == MODEL_IDS[a] and c.get('candidate_count') == COUNTS[a] for a, c in configs.items()), 'per-arm model proof mismatch')
    require(sha_file(ROOT / 'portable_e2e/runtime_contract.py') == base._sha(sources['portable_e2e/runtime_contract.py']),
        'loaded gate definition differs from the recorded source; use a matching reader environment')
    return configs, sources


def expected_command(record, checkpoint_sha):
    command = record.get('command')
    require(isinstance(command, list) and command and all(isinstance(v, str) for v in command), 'invalid command')
    python = Path(command[0]); workspace = python.parents[3] if len(python.parents) >= 4 else Path('.')
    require(python.is_absolute() and '..' not in python.parts and python.parts[-4:] == ('venvs', 'py312', 'bin', 'python')
        and workspace.parts[-3:] == ('personal', 'hwanhong', 'portable_e2e'), 'unreviewed personal interpreter')
    relative, stage = record['run'], record['stage']; seed, arm = relative.split('/')
    item = workspace / 'runs/campaigns' / CAMPAIGN_ID / relative
    prefix = [str(python), '-m', 'portable_e2e.' + ('audit_runtime' if stage == 'audit' else stage), str(workspace / common.DATASET)]
    shared = ['--device', 'cuda:0', '--batch-size', '4']
    if stage == 'train':
        expected = prefix + ['--run-dir', str(item / 'training'), '--model-config', str(workspace / 'autoware_e2e' / MODELS[arm]),
            '--split', 'train', *shared, '--seed', seed[5:], '--learning-rate', '0.0001', '--weight-decay', '0.0001',
            '--max-steps', '1540', '--checkpoint-interval', '154', '--num-workers', '0', '--maximum-gradient-norm', '5',
            '--sampling-policy', 'uniform_without_replacement']
    elif stage == 'evaluate':
        expected = prefix + ['--checkpoint', str(item / 'training/checkpoints/latest.pt'), '--output-dir', str(item / 'evaluation'),
            '--split', 'val', *shared, '--num-workers', '0', '--render-count', '12']
    else:
        require(re.fullmatch('[0-9a-f]{64}', checkpoint_sha or ''), 'checkpoint SHA required')
        expected = prefix + ['--checkpoint', str(item / 'training/checkpoints/latest.pt'), '--output-json', str(item / 'gate_v8.json'),
            '--split', 'val', *shared, '--checkpoint-sha256', checkpoint_sha]
    require(command == expected, 'exact normal-stage command mismatch')
    return [v.replace(str(workspace), '<PERSONAL_WORKSPACE>') for v in command]


def geometry(audit, candidates, count=337):
    # HH_260906 - K is an explicit per-arm shape, never an implicit six-candidate assumption or a relaxed numeric gate.
    gate = {'source': RUNTIME_GATE_ID, 'thresholds': asdict(RuntimeGateConfig(candidate_count=candidates)), 'threshold_overrides': False}
    require(audit.get('gate') == gate and audit.get('audit_id') == 'portable_e2e.runtime_geometry_audit.v8'
        and audit.get('status') == 'RUNTIME_GEOMETRY_AUDIT_COMPLETE', 'unchanged v8 gate required')
    g = audit.get('geometry', {}); require(g.get('sample_count') == count, 'geometry denominator mismatch')
    selected, all_rows = g.get('selected_result', {}), g.get('candidate_results', [])
    require(isinstance(all_rows, list) and [r.get('candidate_index') for r in all_rows] == list(range(candidates)), 'candidate geometry inventory mismatch')
    for row in [selected, *all_rows]:
        passed, rejected = row.get('geometry_pass_count'), row.get('geometry_reject_count')
        require(type(passed) is int and type(rejected) is int and passed >= 0 and rejected >= 0 and passed + rejected == count
            and row.get('sample_count') == count and row.get('geometry_pass_rate') == passed / count, 'inconsistent geometry counters')
        failures = row.get('failure_counts')
        require(isinstance(failures, dict) and all(k in FAILURE_CODES and type(v) is int and 0 < v <= rejected for k, v in failures.items())
            and bool(failures) == bool(rejected) and sum(failures.values()) >= rejected, 'invalid geometry failure counts')
    histogram = g.get('selector', {}).get('selection_counts', {})
    require(set(histogram) == {str(i) for i in range(candidates)} and all(type(v) is int and v >= 0 for v in histogram.values())
        and sum(histogram.values()) == count and g['selector'].get('invalid_selection_count') == 0, 'selector denominator mismatch')
    for kind in ('all', 'any'):
        n = g.get(f'{kind}_candidates_geometry_pass_count' if kind == 'all' else 'any_candidate_geometry_pass_count')
        rate = g.get(f'{kind}_candidates_geometry_pass_rate' if kind == 'all' else 'any_candidate_geometry_pass_rate')
        require(type(n) is int and 0 <= n <= count and rate == n / count, 'aggregate candidate geometry mismatch')
    require(g['all_candidates_geometry_pass_count'] <= min(r['geometry_pass_count'] for r in all_rows)
        <= max(r['geometry_pass_count'] for r in all_rows) <= g['any_candidate_geometry_pass_count']
        and selected['geometry_pass_count'] <= g['any_candidate_geometry_pass_count'], 'candidate pass-set bounds inconsistent')
    return {'sample_count': count, 'selected_pass_count': selected['geometry_pass_count'], 'selection_histogram': histogram,
        'selected_failure_counts': selected['failure_counts'], 'candidate_count': candidates, 'raw_geometry': g}


def validate_run(root, relative, plan, config, sources):
    seed, arm = relative.split('/'); seed = int(seed[5:]); item = root / relative
    train, evaluation, audit = (read(item / name) for name in expansion.STAGE_FILES.values())
    require(train.get('dataset_size') == 1147 and train.get('status') == 'TRAINING_TARGET_REACHED'
        and train.get('training_split') == 'train' and train.get('state', {}).get('global_step') == 1540
        and train.get('state', {}).get('domain_samples_seen') == {'carla': 6155}, 'training budget/split/exposure mismatch')
    episodes = train.get('training_episode_ids')
    require(isinstance(episodes, list) and all(isinstance(s, str) and s for s in episodes)
        and len(episodes) == len(set(episodes)) == 3, 'three unique training episodes required')
    expected = dict(seed=seed, learning_rate=.0001, batch_size=4, max_steps=1540, weight_decay=.0001,
        checkpoint_interval=154, num_workers=0, maximum_gradient_norm=5., verify_image_sha256=True,
        sampling_policy='uniform_without_replacement', domain_ratios=[])
    require(train.get('train_config') == expected and train.get('loss_config') == common.LOSS
        and train.get('model_config') == config and 'candidate_regret_loss' not in train.get('last_metrics', {}), 'training model/config/original loss mismatch')
    for report in (evaluation, audit):
        require(not any(k in report for k in ('loss_config', 'auxiliary_loss_metrics', 'auxiliary_loss_metric_counts')), 'unexpected auxiliary evaluation')
        require(report.get('training_episode_count') == 3 and report.get('evaluation_episode_count') == 1
            and report.get('evaluation_split') == 'val' and report.get('vehicle_control_approved') is False, 'evaluation split/approval mismatch')
    require(evaluation.get('sample_count') == 337 and evaluation.get('domain_sample_counts') == {'carla': 337}, 'full val337 required')
    require(evaluation.get('model_config_sha256') == base._sha(canonical(config).encode()), 'model config fingerprint mismatch')
    for key in ('checkpoint_sha256', 'model_config_sha256', 'corpus_fingerprint_sha256', 'dataset_fingerprint_sha256',
        'training_dataset_fingerprint_sha256', 'training_sampling_policy', 'training_sampling_plan_sha256',
        'training_domain_samples_seen', 'domain_sample_counts', 'runtime', 'device', 'batch_size', 'model_parameter_count'):
        require(evaluation.get(key) == audit.get(key), 'evaluation/audit provenance mismatch: ' + key)
    for tkey, ekey in (('corpus_fingerprint_sha256', 'corpus_fingerprint_sha256'),
        ('dataset_fingerprint_sha256', 'training_dataset_fingerprint_sha256'), ('sampling_plan_sha256', 'training_sampling_plan_sha256'),
        ('runtime', 'runtime'), ('hardware', 'hardware'), ('device', 'device'), ('model_parameter_count', 'model_parameter_count')):
        require(train.get(tkey) == evaluation.get(ekey), 'training/evaluation provenance mismatch: ' + tkey)
    require(evaluation.get('training_domain_samples_seen') == {'carla': 6155} and evaluation.get('model_parameter_count') == PARAMETERS[arm]
        and evaluation.get('device') == 'cuda:0' and evaluation.get('batch_size') == 4
        and evaluation.get('hardware', {}).get('device_uuid') == plan['gpu_uuid'].removeprefix('GPU-'), 'GPU, capacity or exposure mismatch')
    for name in ('audit_runtime', 'runtime_contract'):
        require(audit.get('implementation', {}).get(name + '_sha256') == base._sha(sources[f'portable_e2e/{name}.py']), 'audit source mismatch')
    checkpoint = item / 'training/checkpoints/latest.pt'
    if checkpoint.exists() or checkpoint.is_symlink():
        require(checked_hash(checkpoint) == evaluation['checkpoint_sha256'], 'checkpoint bytes mismatch')
    return {'run': relative, 'seed': seed, 'arm': arm, 'candidate_count': COUNTS[arm], 'model_parameter_count': PARAMETERS[arm],
        'training_dataset_size': 1147, 'training_episode_ids': sorted(episodes), 'training_sampling_plan_sha256': train['sampling_plan_sha256'],
        'validation_sample_count': 337, 'checkpoint_sha256': evaluation['checkpoint_sha256'],
        'checkpoint_bytes_locally_verified': checkpoint.exists(), 'geometry': geometry(audit, COUNTS[arm]),
        'checkpoint_validation_reference': {k: audit[k] for k in ('checkpoint_sha256', 'checkpoint_id', 'model_config_sha256',
            'training_dataset_fingerprint_sha256', 'training_episode_count', 'training_sampling_plan_sha256',
            'training_sampling_policy', 'training_domain_samples_seen', 'evaluation_episode_count', 'model_parameter_count')}}


def behavior_evidence(root, run, state, expected_commit, pins):
    # HH_260906 - Read only saved predictions; never load checkpoints, rerun inference or substitute this diagnostic for the normal gate.
    from scripts.e2e import audit_portable_stopmix_behavior as behavior
    prefix = run['run']; item = root / prefix
    initial = {p.relative_to(item).as_posix(): checked_hash(p) for p in item.rglob('*') if p.is_file() or p.is_symlink()}
    pins.update({prefix + '/' + n: v for n, v in initial.items()})
    if not (item / 'summary.json').exists():
        return {'run': prefix, 'status': 'INCOMPLETE' if item.exists() else 'NOT_RUN'}
    require('failed.json' not in initial, 'behavior contains both failed and completed outcomes')
    report = read(item / 'summary.json')
    started = read(item / 'started.json')
    require(started.get('status') == 'RUNNING' and all(report.get(k) == v for k, v in started.items() if k != 'status'),
        'behavior started/completed provenance mismatch')
    began, ended = timestamp(report.get('started_at_utc')), timestamp(report.get('ended_at_utc'))
    normal_end = max(common.shared._time(r['finished_at_utc']) for r in state['stages'])
    require(normal_end <= began <= ended and (ended - began).total_seconds() <= 180
        and ended <= common.shared._time(common.DEADLINE.replace('Z', '+00:00')), 'behavior chronology or reserved timeout/deadline differs')
    expected_sources = {n: base._sha(git_bytes(expected_commit, n)) for n in BEHAVIOR_SOURCES}
    require(report.get('schema') == 'portable_e2e.stopmix_behavior_audit.v1' and report.get('status') == 'COMPLETE_NOT_PROMOTED'
        and report.get('source_commit') == expected_commit and report.get('source_sha256') == expected_sources
        and report.get('source_checkpoint_and_corpus_postcheck_pass') is True, 'behavior source/completion mismatch')
    require(checked_hash(ROOT / BEHAVIOR_SCRIPT) == expected_sources[BEHAVIOR_SCRIPT], 'behavior aggregation helper differs from executed source')
    for key, value in dict(checkpoint_sha256=run['checkpoint_sha256'], model_id=MODEL_IDS[run['arm']], candidate_count=run['candidate_count'],
        dataset_manifest_sha256=expansion.MANIFEST_SHA256, corpus_fingerprint_sha256=expansion.CORPUS_SHA256,
        dataset_fingerprint_sha256=state['val_fingerprint_sha256'], evaluation_split='val', expected_sample_count=337,
        vehicle_control_approved=False, training_data_approved=False, test_inference_or_optimization_or_selection=False,
        batch_size=4, device='cuda:0').items():
        require(canonical(report.get(key)) == canonical(value), 'behavior checkpoint/model/split binding mismatch: ' + key)
    require(report.get('checkpoint_validation') == run['checkpoint_validation_reference'] and report.get('gate') ==
        {'source': RUNTIME_GATE_ID, 'thresholds': asdict(RuntimeGateConfig(candidate_count=run['candidate_count'])), 'threshold_overrides': False},
        'behavior capacity or gate differs')
    manifest = {}
    for line in (item / 'SHA256SUMS').read_text().splitlines():
        digest, name = line.split('  ')
        require(not Path(name).is_absolute() and '..' not in Path(name).parts and name not in manifest
            and re.fullmatch('[0-9a-f]{64}', digest), 'unsafe/duplicate behavior checksum entry')
        require(checked_hash(item / name) == digest, 'behavior payload checksum mismatch'); manifest[name] = digest
    expected_files = {'started.json', 'samples.jsonl', 'summary.json', *(f'trajectories/val_{i:03d}.png' for i in RENDER_INDICES)}
    require(set(manifest) == expected_files and report.get('samples_sha256') == manifest['samples.jsonl'], 'behavior inventory/rows hash mismatch')
    payload = (item / 'samples.jsonl').read_bytes(); require(payload.endswith(b'\n'), 'incomplete behavior row')
    rows = [_loads_json(line, 'behavior row') for line in payload.decode().splitlines()]
    require(len(rows) == 337 and [r.get('index') for r in rows] == list(range(337)), 'behavior row denominator/order mismatch')
    for row in rows:
        candidates = row.get('candidates', []); selected = row.get('selected_candidate_index')
        require([c.get('candidate_index') for c in candidates] == list(range(run['candidate_count']))
            and type(selected) is int and 0 <= selected < len(candidates), 'behavior candidate inventory/selection mismatch')
        require(row.get('runtime_geometry', {}).get('selected_candidate_index') == selected, 'behavior raw gate selection differs')
        for index, candidate in enumerate(candidates):
            require(candidate.get('family') == ('STOP' if run['candidate_count'] == 12 and index >= 6 else 'DRIVE'), 'candidate family mismatch')
            for name in ('ade_m', 'fde_m', 'speed_mae_mps'):
                value = candidate.get(name); require(type(value) in (int, float) and math.isfinite(value) and value >= 0, 'invalid behavior candidate metric')
        raw_gate = row['runtime_geometry']; gates = raw_gate.get('candidates', [])
        require([g.get('candidate_index') for g in gates] == list(range(run['candidate_count'])), 'behavior gate candidate inventory mismatch')
        for g in gates:
            codes = g.get('failure_codes')
            require(type(g.get('geometry_pass')) is bool and isinstance(codes, list) and len(codes) == len(set(codes))
                and all(c in FAILURE_CODES for c in codes) and bool(codes) != g['geometry_pass'], 'behavior raw candidate gate inconsistent')
        require(raw_gate.get('all_candidates_geometry_pass') is all(g['geometry_pass'] for g in gates)
            and raw_gate.get('any_candidate_geometry_pass') is any(g['geometry_pass'] for g in gates)
            and raw_gate.get('selected_geometry_pass') is gates[selected]['geometry_pass']
            and raw_gate.get('selected_failure_codes') == gates[selected]['failure_codes'], 'behavior raw selected/all/any gate inconsistent')
        require(row.get('selected_speed_behavior') == {k: candidates[selected].get(k) for k in
            ('terminal_speed_mps', 'terminal_exact_zero', 'terminal_at_or_below_0p1_mps', 'first_exact_zero_future_index',
             'reacceleration_after_exact_future_zero', 'nonincreasing_speed_exact')}, 'behavior selected speed record differs')
        for key, candidate_key in (('selected_ade_m', 'ade_m'), ('selected_fde_m', 'fde_m'), ('selected_speed_mae_mps', 'speed_mae_mps')):
            require(row.get(key) == candidates[selected][candidate_key], 'selected behavior metric differs')
        oracle = min(c['ade_m'] for c in candidates)
        require(row.get('oracle_ade_m') == oracle and row.get('ade_selection_regret_m') == row['selected_ade_m'] - oracle
            and row.get('selected_stop_candidate') is (selected >= 6 if run['candidate_count'] == 12 else None), 'behavior oracle/family aggregate mismatch')
    aggregates = behavior.summarize_rows(rows, run['candidate_count'])
    require(all(canonical(report.get(k)) == canonical(v) for k, v in aggregates.items()), 'behavior aggregate differs from all saved rows')
    renders = report.get('renders', [])
    require([r.get('index') for r in renders] == list(RENDER_INDICES) and report.get('fixed_render_indices') == list(RENDER_INDICES), 'behavior fixed render selection differs')
    for render in renders:
        row = rows[render['index']]; name = f"trajectories/val_{render['index']:03d}.png"
        require(render.get('file') == name and render.get('sha256') == manifest[name]
            and all(render.get(k) == row.get(k) for k in ('sample_id', 'anchor_timestamp_ns', 'camera_sha256')), 'behavior PNG/source-row binding differs')
    require({p.relative_to(item).as_posix(): checked_hash(p) for p in item.rglob('*') if p.is_file() or p.is_symlink()} == initial,
        'behavior inputs changed while recomputing evidence')
    identities = [{k: r.get(k) for k in ('sample_id', 'episode_id', 'sequence_index', 'anchor_timestamp_ns', 'camera_sha256',
        'source_manifest_sha256', 'raw_current_vx_mps', 'valid_future_points', 'target_motion_group')} for r in rows]
    return {'run': prefix, 'status': 'COMPLETE_NOT_PROMOTED', 'report_sha256': manifest['summary.json'],
        'sample_count': 337, 'sample_identity_sha256': base._sha(canonical(identities).encode()), **aggregates,
        'source_sha256': expected_sources, 'checkpoint_sha256': run['checkpoint_sha256']}


def summarize_campaign(root, *, expected_source_commit, expected_plan_sha256, behavior_root=None):
    root = Path(root).absolute(); require(all(not p.is_symlink() for p in (root, *root.parents)), 'symlink campaign root')
    reader_sha = checked_hash(Path(__file__).resolve())
    result = dict(schema=SCHEMA, status='INCOMPLETE', normal_stage_completion='INCOMPLETE', behavior_completion='INCOMPLETE',
        candidate_screen='NOT_EVALUATED', absolute_quality='NOT_EVALUATED', automatic_promotion=False,
        reader_source_sha256=reader_sha,
        vehicle_control_approved=False, training_data_approved_by_this_report=False, pairs=[], stages=[], behaviors=[],
        expected_stage_count=18, expected_behavior_count=6, test_evaluated=None, test_used_for_training_or_selection=None,
        limitations=['K6/954590 parameters versus K12/1056362: both candidate count and capacity change; this is not an isolated STOP-head causal effect.',
            'Original loss coefficients are equal but best-of-K and selector class count differ; total loss is not a capacity-controlled quality comparison.',
            'Legacy v3 data remains unchanged, including known warmup/target-quality limitations. No new data admission or learned closed-loop evidence.',
            'Full-corpus integrity checks may read held-out test bytes; no test predictions, optimization or selection are used.',
            'Behavior groups use future target motion for retrospective diagnosis, not causal stop intent; overlapping windows are not independent events.',
            'Saved behavior rows/aggregates are rechecked, but original target group labels and geometry are not replayed from absent prediction/raw-target arrays.',
            'Explicit caller source/plan pins bind the prospective experiment; preregistration chronology still requires the external launch receipt.'])
    result['missing_artifacts'] = [n for n in ('plan.json', 'status.json') if not (root / n).exists()]
    if result['missing_artifacts']: return result
    pins = {n: checked_hash(root / n) for n in ('plan.json', 'status.json')}
    plan, state = read(root / 'plan.json'), read(root / 'status.json')
    configs, sources = validate_plan(plan, state, (root / 'plan.json').read_bytes(), expected_source_commit, expected_plan_sha256)
    expected = [(f'seed_{s}/{a}', stage) for s in base.SEEDS for a in ARMS for stage in expansion.STAGE_FILES]
    required = [f'{r}/{expansion.STAGE_FILES[s]}' for r, s in expected] + [archive_name(n) for n in SOURCE_PATHS]
    for name in required + [f'seed_{s}/{a}/training/checkpoints/latest.pt' for s in base.SEEDS for a in ARMS]:
        path = root / name
        if path.exists() or path.is_symlink(): pins[name] = checked_hash(path)
    result['missing_artifacts'] = [n for n in required if n not in pins]
    for name, payload in sources.items():
        if archive_name(name) in pins: require(pins[archive_name(name)] == base._sha(payload), 'archived pipeline source mismatch')
    result.update(campaign_id=CAMPAIGN_ID, source_commit=expected_source_commit, plan_sha256=expected_plan_sha256,
        source_proof={n: base._sha(v) for n, v in sources.items()}, dataset_manifest_sha256=expansion.MANIFEST_SHA256,
        budget=dict(fresh_run_count=6, steps=1540, batch_size=4, train_samples=1147, val_samples=337,
            samples_seen_per_run=6155, full_epochs=5, partial_epoch_batches=105, partial_epoch_samples=420), runner_status=state.get('status'))
    stages = state.get('stages'); require(isinstance(stages, list) and all(isinstance(r, dict) for r in stages)
        and len(stages) <= 18 and [(r.get('run'), r.get('stage')) for r in stages] == expected[:len(stages)], 'normal-stage order/scope mismatch')
    previous, deadline = common.shared._time(state['created_at_utc']), common.shared._time(common.DEADLINE.replace('Z', '+00:00'))
    for index, record in enumerate(stages):
        status = record.get('status'); require(status in ('COMPLETE', 'RUNNING', 'FAILED') and
            (status == 'COMPLETE' or index == len(stages) - 1), 'unfinished normal stage followed by another stage')
        started = common.shared._time(record.get('started_at_utc'))
        require(started >= previous and (deadline - started).total_seconds() >= sum(common.STAGE_TIMEOUTS[s] for _, s in expected[index:]) + 1380,
            'normal stage chronology or reserved behavior budget differs')
        if status != 'RUNNING':
            previous = common.shared._time(record.get('finished_at_utc')); require(previous >= started, 'negative stage duration')
        if status == 'COMPLETE': require(type(record.get('returncode')) is int and record['returncode'] == 0 and previous <= deadline, 'completed stage exit/deadline differs')
        checksum = base._option(record.get('command', []), '--checkpoint-sha256') if record['stage'] == 'audit' else None
        command = expected_command(record, checksum)
        expansion._validate_stage(record, root, state, complete=status == 'COMPLETE')
        if status == 'COMPLETE' and record['stage'] == 'audit':
            require(read(root / record['run'] / 'gate_v8.json').get('checkpoint_sha256') == checksum, 'audit command checkpoint SHA differs')
        result['stages'].append({k: record.get(k) for k in ('run', 'stage', 'status', 'returncode', 'report', 'started_at_utc', 'finished_at_utc')} | {'command': command})
    result['completed_stage_count'] = sum(r['status'] == 'COMPLETE' for r in stages)
    result['not_started_stages'] = [{'run': r, 'stage': s, 'status': 'NOT_RUN'} for r, s in expected[len(stages):]]
    complete = not result['missing_artifacts'] and len(stages) == 18 and all(r['status'] == 'COMPLETE' for r in stages)
    behavior_pins = {}
    if complete:
        require(state.get('status') == 'TRAIN_EVAL_AUDIT_COMPLETE_NOT_PROMOTED', 'complete normal ledger/status mismatch')
        comparison = base.compare_reports([root / r / 'evaluation/metrics.json' for r, s in expected if s == 'evaluate'])
        runs = []
        for row, (relative, _) in zip(comparison['reports'], [(r, s) for r, s in expected if s == 'evaluate']):
            arm = relative.split('/')[1]; run = validate_run(root, relative, plan, configs[arm], sources)
            run['metrics'] = row['metrics']; run['absolute_checks'] = {k: k in row['metrics'] and row['metrics'][k] <= v for k, v in base.ABSOLUTE_LIMITS.items()}
            run['absolute_quality'] = 'PASS' if all(run['absolute_checks'].values()) else 'FAIL'; runs.append(run)
        require(len({tuple(r['training_episode_ids']) for r in runs}) == len({r['training_sampling_plan_sha256'] for r in runs}) == 1,
            'paired training episode or sampling plan mismatch')
        for index, seed in enumerate(base.SEEDS):
            a, b = runs[2 * index:2 * index + 2]
            checks = {'selected_ade_improves': b['metrics']['selected_ade_m'] < a['metrics']['selected_ade_m'],
                'selected_fde_improves': b['metrics']['selected_fde_m'] < a['metrics']['selected_fde_m'],
                'geometry_does_not_regress': b['geometry']['selected_pass_count'] >= a['geometry']['selected_pass_count'],
                'speed_mae_within_5_percent': b['metrics']['selected_speed_mae_mps'] <= 1.05 * a['metrics']['selected_speed_mae_mps']}
            result['pairs'].append(dict(seed=seed, baseline=a, candidate=b, relative_checks=checks, candidate_screen='PASS' if all(checks.values()) else 'FAIL',
                absolute_checks=b['absolute_checks'], absolute_quality=b['absolute_quality']))
        result.update(status='COMPLETE_NORMAL_STAGES', normal_stage_completion='COMPLETE', test_evaluated=False, test_used_for_training_or_selection=False,
            candidate_screen='PASS' if all(p['candidate_screen'] == 'PASS' for p in result['pairs']) else 'FAIL',
            absolute_quality='PASS' if all(p['absolute_quality'] == 'PASS' for p in result['pairs']) else 'FAIL', absolute_limits_m=base.ABSOLUTE_LIMITS)
        if behavior_root is not None:
            behavior_root = Path(behavior_root).absolute()
            require(all(not p.is_symlink() for p in (behavior_root, *behavior_root.parents)), 'symlink behavior root')
            result['behaviors'] = [behavior_evidence(behavior_root, run, state, expected_source_commit, behavior_pins) for run in runs]
            finished = [r for r in result['behaviors'] if r['status'] == 'COMPLETE_NOT_PROMOTED']
            if len(finished) == 6:
                require(len({r['sample_identity_sha256'] for r in finished}) == 1, 'behavior validation identity/group mismatch across six models')
                result.update(status='COMPLETE_NOT_PROMOTED', behavior_completion='COMPLETE')
    result['completed_behavior_count'] = sum(r['status'] == 'COMPLETE_NOT_PROMOTED' for r in result['behaviors'])
    require(checked_hash(Path(__file__).resolve()) == reader_sha, 'summary reader source changed during execution')
    require(all(checked_hash(root / n) == v for n, v in pins.items()), 'campaign inputs changed during summary')
    if behavior_root is not None: require(all(checked_hash(Path(behavior_root) / n) == v for n, v in behavior_pins.items()), 'behavior inputs changed during summary')
    result['input_manifest'] = [{'path': n, 'sha256': v} for n, v in sorted(pins.items())]
    result['behavior_input_manifest'] = [{'path': n, 'sha256': v} for n, v in sorted(behavior_pins.items())]
    return result


def render_markdown(report):
    lines = ['# 주행 6개 후보와 주행+정지 12개 후보 · 개발용 새 학습 비교', '',
        f"상태: `{report['status']}` · 기본18단계: `{report['normal_stage_completion']}` · 별도행동6회: `{report['behavior_completion']}`", '',
        '기존 v3 train1147 / val337, seed 3개 × 모델 2개를 각각 1540 step·6155회 샘플 노출로 새로 학습했습니다. 미완료 기록은 완료로 간주하지 않습니다.', '',
        '| Seed | 모델 | 후보/파라미터 | ADE m | FDE m | 속도 MAE m/s | Geometry | 상대/절대 |',
        '| --- | --- | ---: | ---: | ---: | ---: | ---: | --- |']
    for pair in report['pairs']:
        for label in ('baseline', 'candidate'):
            run = pair[label]; m = run['metrics']
            lines.append(f"| {pair['seed']} | {run['arm']} | {run['candidate_count']}/{run['model_parameter_count']} | {m['selected_ade_m']:.5f} | {m['selected_fde_m']:.5f} | {m['selected_speed_mae_mps']:.5f} | {run['geometry']['selected_pass_count']}/337 | {pair['candidate_screen']}/{run['absolute_quality']} |")
    lines += ['', '후보 수와 파라미터 용량이 동시에 바뀌므로 정지 기능 하나의 효과로 단정할 수 없습니다. 전체 후보의 gate 실패와 seed별 실패는 JSON에 유지합니다.',
        '행동 분석은 미래 정답으로 나눈 사후 그룹입니다. 신호·장애물 정지 의도를 학습했다는 증거, CARLA/Autoware 폐루프 주행, 실차 승인 또는 자동 모델 승격이 아닙니다.',
        '전체 corpus 무결성 검사에서 test 파일을 읽을 수 있지만 test 추론·학습·모델 선택은 하지 않습니다. 새 데이터 승인 없음.', '']
    return '\n'.join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('campaign_root', type=Path); parser.add_argument('--behavior-root', type=Path)
    parser.add_argument('--expected-source-commit', required=True); parser.add_argument('--expected-plan-sha256', required=True)
    parser.add_argument('--output-dir', type=Path, required=True); args = parser.parse_args(argv)
    try:
        output = args.output_dir.absolute()
        inputs = [args.campaign_root.resolve(), (ROOT / 'datasets').resolve()]
        if args.behavior_root: inputs.append(args.behavior_root.resolve())
        require(not output.exists() and all(not p.is_symlink() for p in (output, *output.parents))
            and not any(output.resolve().is_relative_to(p) for p in inputs), 'fresh output must be outside inputs and datasets')
        report = summarize_campaign(args.campaign_root, expected_source_commit=args.expected_source_commit,
            expected_plan_sha256=args.expected_plan_sha256, behavior_root=args.behavior_root)
        output.mkdir(parents=True, exist_ok=False)
        for name, text in (('summary.json', json.dumps(report, indent=2, allow_nan=False) + '\n'), ('README.md', render_markdown(report))):
            with (output / name).open('x') as stream: stream.write(text)
        with (output / 'SHA256SUMS').open('x') as stream:
            for name in ('summary.json', 'README.md'): stream.write(f'{sha_file(output / name)}  {name}\n')
    except (ContractError, OSError, ValueError, TypeError, KeyError, IndexError) as error:
        print(f'STOPMIX_SUMMARY_ERROR: {error}'); return 2
    print(json.dumps({k: report[k] for k in ('status', 'candidate_screen', 'absolute_quality')}))
    return 0 if report['normal_stage_completion'] == 'COMPLETE' else 1


if __name__ == '__main__':
    raise SystemExit(main())
