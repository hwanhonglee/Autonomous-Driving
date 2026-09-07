#!/usr/bin/env python3
"""HH_260906 - Verify nine head-only fits against their original same-cache C baselines."""

from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess
import sys

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from portable_e2e.audit_runtime import _AuditAccumulator
from portable_e2e.contract import ContractError, _loads_json
from portable_e2e.evaluate import CORE_METRIC_NAMES
from portable_e2e.frozen_selector import ARTIFACT_ID
from portable_e2e.losses import TrajectoryLossConfig
from portable_e2e.runtime_contract import RUNTIME_GATE_ID, RuntimeGateConfig
from scripts.e2e import run_portable_frozen_selector as runner
from scripts.e2e import summarize_portable_data_expansion as parent_summary
from scripts.e2e.summarize_portable_training_campaign import _git_bytes


SCHEMA = 'portable_e2e.frozen_selector_summary.v1'
INTEGRITY_SCOPE = 'The entire declared corpus, including test, is checked for integrity; no test model predictions or outcome analysis.'
SPLIT_SIZES = {'train': 1147, 'val': 337}
METRICS = tuple(CORE_METRIC_NAMES) + tuple(f'{metric}_{horizon}_{unit}'
    for horizon in ('1p0s', '3p0s', '6p4s')
    for metric, unit in (('ade', 'm'), ('fde', 'm'), ('speed_mae', 'mps')))
ARM_FILES = ('metrics.jsonl', 'head_only.pt', 'final_logits.pt', 'report.json', 'geometry.json')


def require(condition, message):
    if not condition:
        raise ContractError(message)


def digest(payload):
    return hashlib.sha256(payload).hexdigest()


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)


def sha(value, label):
    require(isinstance(value, str) and re.fullmatch('[0-9a-f]{64}', value) is not None, f'invalid {label} SHA-256')
    return value


def integer(value, label, expected=None):
    require(type(value) is int and value >= 0 and (expected is None or value == expected), f'invalid {label} integer/count')
    return value


def finite(value, label):
    require(not isinstance(value, bool) and isinstance(value, (int, float)), f'non-numeric {label}')
    try:
        result = float(value)
    except (ValueError, OverflowError) as error:
        raise ContractError(f'nonfinite {label}') from error
    require(math.isfinite(result) and result >= 0, f'nonfinite/negative {label}')
    return result


class Inputs:
    def __init__(self, campaign_root, parent_root):
        self.roots = {'campaign': campaign_root, 'parent': parent_root}
        self.manifest = {}

    def bytes(self, label, relative):
        require(isinstance(relative, str) and not Path(relative).is_absolute()
            and '..' not in Path(relative).parts, 'evidence paths must stay inside their declared root')
        path = self.roots[label] / relative
        require(path.resolve().is_relative_to(self.roots[label].resolve())
            and all(not parent.is_symlink() for parent in [path, *path.parents]
                if parent != self.roots[label].parent), f'symlink or escaping evidence: {label}/{relative}')
        require(path.is_file(), f'missing or nonregular evidence: {label}/{relative}')
        payload = path.read_bytes()
        self.manifest[f'{label}/{relative}'] = digest(payload)
        return payload

    def read(self, label, relative):
        return _loads_json(self.bytes(label, relative).decode('utf-8'), f'{label}/{relative}')

    def verify(self, label, relative, expected):
        require(digest(self.bytes(label, relative)) == sha(expected, relative), f'evidence SHA-256 mismatch: {label}/{relative}')


def source_paths(commit):
    output = subprocess.run(['git', '-C', str(REPO), 'ls-tree', '-r', '--name-only', commit, 'portable_e2e'],
        check=True, capture_output=True, text=True, timeout=15).stdout
    return sorted([path for path in output.splitlines() if path.endswith('.py')]
        + ['scripts/e2e/diagnose_portable_selector.py'])


def validate_source(state, inputs):
    commit = state.get('source_commit')
    require(isinstance(commit, str) and re.fullmatch('[0-9a-f]{40}', commit) is not None, 'source commit must be pinned')
    source = state.get('source', {})
    require(source.get('diagnostic_source_commit') == commit, 'campaign imported source mismatch')
    hashes = source.get('source_sha256', {})
    require(isinstance(hashes, dict) and sorted(hashes) == source_paths(commit), 'source proof omits or changes imported files')
    for path, expected in hashes.items():
        require(digest(_git_bytes(commit, path)) == sha(expected, path), f'pinned source bytes mismatch: {path}')
    runner_bytes = inputs.bytes('campaign', 'provenance/active_runner.py')
    require(digest(runner_bytes) == sha(state.get('runner_sha256'), 'runner')
        and runner_bytes == _git_bytes(commit, 'scripts/e2e/run_portable_frozen_selector.py'), 'archived runner differs from pinned source')
    plan_bytes = inputs.bytes('campaign', 'provenance/plan.json')
    require(digest(plan_bytes) == sha(state.get('plan_sha256'), 'plan')
        and plan_bytes == _git_bytes(commit, 'config/portable_e2e_frozen_selector_20260908.json'), 'archived plan differs from pinned source')
    plan = runner.load_plan(inputs.roots['campaign'] / 'provenance/plan.json')
    require(canonical(state.get('plan')) == canonical(plan), 'campaign frozen plan mismatch')
    return commit, plan


def validate_scope(record):
    require(record.get('vehicle_control_approved') is False and record.get('automatic_promotion') is False,
        'research evidence cannot approve control or automatic promotion')
    require(record.get('test_evaluated') is False and record.get('test_used_for_training_or_selection') is False,
        'test inference/training/selection scope is missing or invalid')
    require(record.get('dataset_integrity_scope') == INTEGRITY_SCOPE, 'whole-corpus integrity scope must be explicit')


def timestamp(value):
    try:
        result = datetime.fromisoformat(value.replace('Z', '+00:00'))
    except (AttributeError, ValueError) as error:
        raise ContractError('invalid evidence timestamp') from error
    require(result.tzinfo is not None and result.utcoffset() is not None, 'evidence timestamp must include a timezone')
    return result


def validate_stage(stage, seed, commit, plan):
    command = stage.get('command', [])
    require(isinstance(command, list) and len(command) == 10 and all(isinstance(item, str) for item in command),
        'seed command is missing or malformed')
    require(Path(command[0]).parts[-4:] == ('venvs', 'py312', 'bin', 'python')
        and Path(command[1]).name == 'run_portable_frozen_selector.py'
        and command[2:7] == ['--seed', str(seed), '--expected-source-commit', commit, '--output-dir']
        and Path(command[7]).parts[-2:] == (plan['campaign_id'], f'seed_{seed}')
        and command[8:] == ['--deadline-utc', plan['deadline_utc']], 'seed command changed its scoped interpreter/source/seed/output/deadline')
    require(timestamp(stage.get('started_at_utc')) <= timestamp(stage.get('finished_at_utc'))
        <= timestamp(plan['deadline_utc']), 'seed stage is out of chronological/deadline bounds')


def validate_history(payload, report):
    rows = [_loads_json(line, 'head step') for line in payload.decode('utf-8').splitlines()]
    require(len(rows) == 1540 and canonical(report.get('history')) == canonical(rows), 'head history must preserve all 1540 exact records')
    order, seen, epoch_indices = [], 0, {}
    for offset, row in enumerate(rows):
        step, epoch = offset + 1, offset // 287
        integer(row.get('global_step'), 'global_step', step)
        integer(row.get('epoch'), 'epoch', epoch)
        count = 3 if step % 287 == 0 else 4
        integer(row.get('batch_samples'), 'batch_samples', count)
        indices = row.get('sample_indices')
        require(isinstance(indices, list) and len(indices) == count
            and all(type(index) is int and 0 <= index < 1147 for index in indices), 'invalid training sample indices')
        epoch_indices.setdefault(epoch, []).extend(indices)
        seen += count
        integer(row.get('samples_seen'), 'samples_seen', seen)
        score, loss = finite(row.get('candidate_score_loss'), 'score loss'), finite(row.get('optimization_loss'), 'optimization loss')
        require(math.isclose(loss, 0.1 * score, rel_tol=1e-6, abs_tol=1e-9), 'optimization loss does not preserve score weight 0.1')
        finite(row.get('gradient_norm'), 'gradient norm')
        order.append({'epoch': epoch, 'indices': indices})
    require(seen == 6155 and all(sorted(epoch_indices[index]) == list(range(1147)) for index in range(5)),
        'complete training epochs must cover each sample without replacement')
    require(len(epoch_indices[5]) == 420 and len(set(epoch_indices[5])) == 420, 'partial sixth epoch repeats or omits exposure count')
    require(report.get('final_state') == {'global_step': 1540, 'samples_seen': 6155, 'last_epoch_index': 5, 'last_epoch_steps': 105},
        'final scorer state differs from the fixed budget')
    actual = digest(json.dumps(order, separators=(',', ':')).encode())
    require(report.get('sampling_order_sha256') == actual, 'sampling order SHA-256 mismatch')
    return actual


def validate_metrics(value, split):
    n = SPLIT_SIZES[split]
    require(value.get('split') == split, 'split metrics are mislabelled')
    integer(value.get('sample_count'), f'{split} sample count', n)
    metrics, counts = value.get('metrics', {}), value.get('metric_counts', {})
    require(set(metrics) == set(METRICS) and set(counts) == set(METRICS), 'full original CORE/horizon metric names are required')
    for name in METRICS:
        finite(metrics[name], name)
        count = integer(counts[name], name)
        require(0 < count <= n and (name not in CORE_METRIC_NAMES or count == n), 'metric denominator differs from frozen split')
    horizon_counts = [counts[f'ade_{horizon}_m'] for horizon in ('1p0s', '3p0s', '6p4s')]
    require(horizon_counts == sorted(horizon_counts, reverse=True), 'horizon denominator order is invalid')
    for horizon in ('1p0s', '3p0s', '6p4s'):
        require(len({counts[f'{name}_{horizon}_{unit}'] for name, unit in (('ade', 'm'), ('fde', 'm'), ('speed_mae', 'mps'))}) == 1,
            'horizon denominators disagree across metrics')
    for name in ('selected_histogram', 'composite_oracle_histogram'):
        histogram = value.get(name)
        require(isinstance(histogram, list) and len(histogram) == 6
            and all(type(item) is int and item >= 0 for item in histogram) and sum(histogram) == n, 'invalid selector/oracle histogram')
    agreement = integer(value.get('selected_composite_oracle_agreement_count'), 'oracle agreement')
    require(agreement <= sum(min(a, b) for a, b in zip(value['selected_histogram'], value['composite_oracle_histogram'])),
        'oracle agreement exceeds histogram bounds')
    require(metrics['selected_ade_m'] + 1e-6 >= metrics['oracle_ade_m'], 'selected ADE cannot improve on the ADE oracle')
    return {**metrics, 'selection_regret_ade_m': metrics['selected_ade_m'] - metrics['oracle_ade_m']}


def validate_geometry(value, baseline, sample_ids, inputs, route_dir):
    gate = RuntimeGateConfig()
    # HH_260906 - Bind both the v8 source identifier and every unchanged gate threshold recorded by the worker.
    require(value.get('gate') == {'source': RUNTIME_GATE_ID, **asdict(gate)}
        and value.get('vehicle_control_approved') is False, 'runtime geometry gate or approval changed')
    rows = value.get('per_sample', [])
    require(isinstance(rows, list) and len(rows) == 337, 'geometry must cover val337')
    accumulator = _AuditAccumulator(gate)
    for index, row in enumerate(rows):
        candidates = row.get('candidates', [])
        require(len(candidates) == 6, 'geometry candidate count changed')
        for candidate in candidates:
            require(type(candidate.get('geometry_pass')) is bool, 'geometry result must be Boolean')
            require(bool(candidate.get('failure_codes')) != candidate['geometry_pass'], 'geometry pass/failure codes disagree')
        require(row.get('all_candidates_geometry_pass') is all(item['geometry_pass'] for item in candidates)
            and row.get('any_candidate_geometry_pass') is any(item['geometry_pass'] for item in candidates), 'geometry candidate aggregate mismatch')
        selected = integer(row.get('selected_candidate_index'), 'selected candidate')
        require(selected < 6 and row.get('selected_geometry_pass') is candidates[selected]['geometry_pass']
            and row.get('selected_failure_codes') == candidates[selected]['failure_codes'], 'selected geometry differs from candidate result')
        if baseline is not None:
            require(candidates == baseline['per_sample'][index]['candidates'], 'frozen candidate geometry changed between scorer arms')
        accumulator.add(row)
    require(value.get('summary') == accumulator.report(), 'geometry summary differs from full sample audit')
    rendered = value.get('rendered', [])
    require([item.get('index') for item in rendered] == [0, 67, 134, 201, 268, 336], 'route render phases changed')
    for item in rendered:
        index = item['index']
        require(item.get('path') == f'val_phase_{index:03d}.png' and item.get('sample_id') == sample_ids[index], 'route render identity mismatch')
        inputs.verify('campaign', f'{route_dir}/{item["path"]}', item.get('sha256'))
    return {'selected_pass_count': value['summary']['selected_result']['geometry_pass_count'],
        'sample_count': 337, 'selection_histogram': [value['summary']['selector']['selection_counts'][str(index)] for index in range(6)]}


def validate_head(report, seed, arm, state, inputs, relative):
    fixed = {'artifact_id': ARTIFACT_ID, 'status': 'RESEARCH_HEAD_TRAINING_COMPLETE_NOT_PROMOTED',
        'arm': arm, 'seed': seed, 'device': 'cuda:0', 'optimizer': 'AdamW', 'steps': 1540,
        'batch_size': 4, 'learning_rate': 0.0001, 'weight_decay': 0.0001,
        'candidate_score_weight': 0.1, 'maximum_gradient_norm': 5.0,
        'loss_config': TrajectoryLossConfig().to_dict(), 'source_checkpoint_sha256': runner.PARENT_SHA256[seed],
        'corpus_fingerprint_sha256': runner.CORPUS_SHA256,
        'split_fingerprints': {split: runner.SPLITS[split][1] for split in SPLIT_SIZES}}
    require(all(canonical(report.get(key)) == canonical(value) for key, value in fixed.items()), 'head fixed optimization/source/split contract changed')
    require(report.get('vehicle_control_approved') is False and report.get('automatic_promotion') is False, 'head cannot authorize deployment')
    integer(report.get('head_parameter_count'), 'head parameter count', 115201 if arm == 'candidate_reset' else 1542)
    require(report.get('sampling_policy') == 'Uniform without replacement via train._epoch_batches no-plan branch; CPU randperm seeded with seed+epoch. This is not the original domain-balanced order.',
        'scorer sampling policy changed')
    for split in SPLIT_SIZES:
        expected = state['cache_files'][split]['cache_digest']
        require(report.get('cache_sha256_before', {}).get(split) == expected
            and report.get('cache_sha256_after', {}).get(split) == expected, 'head input cache freeze proof changed')
    original = sha(report.get('original_head_sha256_before'), 'original head')
    require(report.get('original_head_sha256_after') == original, 'original C head changed')
    sha(report.get('train_composite_targets_sha256'), 'training targets')
    ids = {}
    for split, n in SPLIT_SIZES.items():
        values = report.get(f'{split}_sample_ids')
        require(isinstance(values, list) and len(values) == n and all(isinstance(value, str) and value for value in values)
            and len(set(values)) == n, 'full unique train/val sample IDs are required')
        ids[split] = values
        parity = report.get('original_logit_parity', {}).get(split, {})
        require(parity.get('cached_original_logits_supplied') is True, 'same-cache original logits were not supplied')
        finite(parity.get('max_abs_logit_difference'), 'original logits parity')
        integer(parity.get('selected_index_mismatch_count'), 'original selection parity', 0)
    require(not set(ids['train']) & set(ids['val']), 'train/val sample leakage')
    require(not {value.rsplit(':', 1)[0] for value in ids['train']} & {value.rsplit(':', 1)[0] for value in ids['val']},
        'train/val episode-token leakage')
    metrics = {}
    for phase in ('baseline_metrics', 'pre_training_metrics', 'post_training_metrics'):
        require(set(report.get(phase, {})) == set(SPLIT_SIZES), 'metrics must keep full train and val separate')
        metrics[phase] = {split: validate_metrics(report[phase][split], split) for split in SPLIT_SIZES}
    for split in SPLIT_SIZES:
        baseline = report['baseline_metrics'][split]
        for phase in ('pre_training_metrics', 'post_training_metrics'):
            value = report[phase][split]
            require(value['metric_counts'] == baseline['metric_counts']
                and value['metrics']['oracle_ade_m'] == baseline['metrics']['oracle_ade_m']
                and value['metrics']['regression_loss'] == baseline['metrics']['regression_loss']
                and value['composite_oracle_histogram'] == baseline['composite_oracle_histogram'], 'fixed candidate/oracle metrics changed')
    if arm == 'linear_continue':
        require(report['pre_training_metrics'] == report['baseline_metrics'], 'continued linear head must begin at the original same-cache baseline')
    order_sha = validate_history(inputs.bytes('campaign', f'{relative}/metrics.jsonl'), report)
    return ids, metrics, order_sha


def summarize_campaign(campaign_root, parent_root):
    campaign_root, parent_root = Path(campaign_root), Path(parent_root)
    report = {'schema': SCHEMA, 'status': 'INCOMPLETE', 'seeds': [], 'arm_screens': {},
        'completed_head_fits': 0, 'scratch_full_model_training_runs': 0,
        'vehicle_control_approved': False, 'automatic_promotion': False,
        'test_evaluated': False, 'test_used_for_training_or_selection': False,
        'dataset_integrity_scope': INTEGRITY_SCOPE,
        'experiment': 'Three independent frozen C generators, each with three scorer-only fits; original same-cache C logits are the reference, not random initialized heads.',
        'verification_limits': ['Cache/head/logit artifacts are byte-hashed but not torch-loaded by this summary; semantic freeze and cache provenance are pinned worker proofs.',
            'History coverage/order hashes are verified without regenerating randperm across different local/remote Torch versions.',
            'Validation development and offline cached geometry do not establish runtime latency, learned driving, feature completion or deployment.']}
    if not (campaign_root / 'status.json').exists():
        report['missing_artifacts'] = ['campaign/status.json']
        return report
    require(campaign_root.resolve() != parent_root.resolve(), 'parent and head campaign roots must differ')
    inputs = Inputs(campaign_root, parent_root)
    state = inputs.read('campaign', 'status.json')
    validate_scope(state)
    require(state.get('schema') == 'portable_e2e.frozen_selector_campaign.v1', 'wrong head campaign schema')
    commit, plan = validate_source(state, inputs)
    report.update(campaign_id=plan['campaign_id'], source_commit=commit, plan_sha256=state['plan_sha256'])
    stages = state.get('stages', [])
    require(isinstance(stages, list) and [stage.get('seed') for stage in stages] == list(runner.PARENT_SHA256)[:len(stages)], 'parent seed stage order differs')
    if state.get('status') != 'FROZEN_SELECTOR_CAMPAIGN_COMPLETE_NOT_PROMOTED' or len(stages) != 3:
        report['runner_status'] = state.get('status')
        report['missing_artifacts'] = [f'campaign/seed_{seed}/result.json' for seed in runner.PARENT_SHA256
            if not (campaign_root / f'seed_{seed}/result.json').exists()]
        return report
    integer(state.get('completed_parent_caches'), 'parent cache count', 3)
    integer(state.get('completed_head_fits'), 'head fit count', 9)
    parent = parent_summary.summarize_campaign(parent_root)
    require(parent.get('status') == 'COMPLETE_NOT_PROMOTED', 'original C parent campaign is incomplete')
    for item in parent['input_manifest']:
        inputs.verify('parent', item['path'], item['sha256'])
    source, core_sha = state['source'], state['source']['source_sha256']['portable_e2e/frozen_selector.py']
    for seed, stage in zip(runner.PARENT_SHA256, stages):
        validate_stage(stage, seed, commit, plan)
        require(stage.get('status') == 'COMPLETE_NOT_PROMOTED', 'unfinished seed stage')
        integer(stage.get('returncode'), 'seed returncode', 0)
        integer(stage.get('completed_head_fits'), 'seed head fits', 3)
        relative = f'seed_{seed}'
        inputs.verify('campaign', f'{relative}/result.json', stage.get('result_sha256'))
        result = inputs.read('campaign', f'{relative}/result.json')
        started = inputs.read('campaign', f'{relative}/started.json')
        validate_scope(result)
        validate_scope(started)
        require(started.get('status') == 'PREFLIGHT' and started.get('arms') == [] and all(result.get(key) == value for key, value in started.items()
            if key not in ('status', 'arms')), 'seed startup proof differs from completion')
        require(result.get('schema') == runner.SCHEMA and result.get('status') == 'HEAD_TRAIN_EVAL_AUDIT_COMPLETE_NOT_PROMOTED'
            and result.get('seed') == seed and result.get('source') == source and result.get('script_sha256') == state['runner_sha256']
            and result.get('core_sha256') == core_sha and result.get('plan') == plan and result.get('plan_sha256') == state['plan_sha256']
            and result.get('deadline_utc') == plan['deadline_utc'], 'seed source/runner/plan scope mismatch')
        require(result.get('parent_checkpoint_sha256') == runner.PARENT_SHA256[seed]
            and result.get('manifest_sha256') == runner.MANIFEST_SHA256 and result.get('gpu_uuid') == runner.GPU_UUID,
            'seed parent/data/GPU pin mismatch')
        require(result.get('loss_config') == TrajectoryLossConfig().to_dict(), 'parent loss configuration changed')
        provenance = result.get('provenance', {})
        require(provenance.get('checkpoint_sha256') == runner.PARENT_SHA256[seed]
            and provenance.get('training_dataset_fingerprint_sha256') == runner.SPLITS['train'][1]
            and provenance.get('training_episode_count') == 3 and provenance.get('evaluation_episode_count') == 1
            and provenance.get('model_parameter_count') == 954590
            and result.get('model_config', {}).get('model_id') == runner.PHYSICAL_MODEL_ID,
            'loaded checkpoint provenance mismatch')
        require(timestamp(stage['started_at_utc']) <= timestamp(result.get('started_at_utc'))
            <= timestamp(result.get('completed_at_utc')) <= timestamp(stage['finished_at_utc']),
            'seed report timing falls outside its owned stage')
        generator = sha(result.get('generator_state_sha256'), 'generator')
        require(result.get('generator_state_after_sha256') == generator, 'original generator changed')
        parent_files = result.get('parent_report_sha256', {})
        require(set(parent_files) == {'training/run.json', 'evaluation/metrics.json', 'gate_v8.json'}, 'parent report proof missing')
        for name, expected in parent_files.items():
            inputs.verify('parent', f'seed_{seed}/C_expanded_data/{name}', expected)
        require(set(result.get('cache_files', {})) == set(SPLIT_SIZES), 'both frozen caches are required')
        for split, n in SPLIT_SIZES.items():
            cache = result['cache_files'][split]
            integer(cache.get('sample_count'), 'cache sample count', n)
            sha(cache.get('cache_digest'), 'cache content')
            inputs.verify('campaign', f'{relative}/{split}_cache.pt', cache.get('sha256'))
        require([arm.get('arm') for arm in result.get('arms', [])] == list(runner.ARMS), 'seed requires all three ordered scorer fits')
        original_geometry = inputs.read('campaign', f'{relative}/original_c_geometry.json')
        seed_report, first = {'seed': seed, 'parent_checkpoint_sha256': runner.PARENT_SHA256[seed], 'arms': []}, None
        for arm_record in result['arms']:
            arm = arm_record['arm']
            require(arm_record.get('status') == 'COMPLETE_NOT_PROMOTED' and set(arm_record.get('files', {})) == set(ARM_FILES), 'head artifact set is incomplete')
            for name, expected in arm_record['files'].items():
                inputs.verify('campaign', f'{relative}/{arm}/{name}', expected)
            head = inputs.read('campaign', f'{relative}/{arm}/report.json')
            ids, metrics, order_sha = validate_head(head, seed, arm, result, inputs, f'{relative}/{arm}')
            shared = (head['baseline_metrics'], ids, order_sha, head['train_composite_targets_sha256'], head['original_head_sha256_before'])
            if first is None:
                first = shared
                geometry = validate_geometry(original_geometry, None, ids['val'], inputs, f'{relative}/original_c_routes')
                require(geometry['selection_histogram'] == head['baseline_metrics']['val']['selected_histogram'], 'original cached logits and geometry selection disagree')
                seed_report['baseline'] = {'arm': 'original_c_same_cache', 'metrics': metrics['baseline_metrics']['val'],
                    'train_metrics': metrics['baseline_metrics']['train'], 'geometry': geometry}
            require(shared == first, 'arms do not share baseline, sample identities, targets, original head or batch order')
            geometry = validate_geometry(inputs.read('campaign', f'{relative}/{arm}/geometry.json'), original_geometry,
                ids['val'], inputs, f'{relative}/{arm}/routes')
            require(geometry['selection_histogram'] == head['post_training_metrics']['val']['selected_histogram'], 'head metrics and geometry selected histograms disagree')
            before, after = seed_report['baseline']['metrics'], metrics['post_training_metrics']['val']
            checks = {'selected_ade_improves': after['selected_ade_m'] < before['selected_ade_m'],
                'selected_fde_improves': after['selected_fde_m'] < before['selected_fde_m'],
                'geometry_does_not_regress': geometry['selected_pass_count'] >= seed_report['baseline']['geometry']['selected_pass_count'],
                'speed_mae_within_5_percent': after['selected_speed_mae_mps'] <= 1.05 * before['selected_speed_mae_mps']}
            absolute = {name: after[name] <= limit for name, limit in runner.ABSOLUTE_LIMITS.items()}
            seed_report['arms'].append({'arm': arm, 'metrics': after, 'train_metrics': metrics['post_training_metrics']['train'],
                'geometry': geometry, 'sampling_order_sha256': order_sha, 'relative_checks': checks, 'absolute_checks': absolute,
                'candidate_screen': 'PASS' if all(checks.values()) else 'FAIL',
                'absolute_quality': 'PASS' if all(absolute.values()) else 'FAIL'})
        report['seeds'].append(seed_report)
    report.update(status='COMPLETE_NOT_PROMOTED', completed_head_fits=9, completed_parent_caches=3,
        absolute_limits_m=runner.ABSOLUTE_LIMITS, input_layout='frozen_selector_campaign_parent_roots_v1',
        input_manifest=[{'path': path, 'sha256': value} for path, value in sorted(inputs.manifest.items())])
    report['arm_screens'] = {arm: {field: 'PASS' if all(next(item for item in seed['arms'] if item['arm'] == arm)[field] == 'PASS'
        for seed in report['seeds']) else 'FAIL' for field in ('candidate_screen', 'absolute_quality')} for arm in runner.ARMS}
    return report


def render_markdown(report):
    lines = ['# 고정 C 경로·context 위 점수 head 전용 학습 비교', '',
        '<!-- HH_260906 - Separate nine head-only fits from full-model training and compare each against original same-cache logits. -->', '',
        f'상태: `{report["status"]}`. 총 9회는 **점수 head만 추가 학습**한 실험이며 scratch 전체 모델 9회가 아닙니다.',
        'C 생성기 3개는 각각 고정했고 서로 다른 생성기의 후보 번호를 합쳐 학습하지 않았습니다.',
        '기준은 동일 캐시의 원래 C logits입니다. Train과 val은 분리하며 새 모델·실차 제어·10 Hz 운용 승인은 없습니다.',
        '전체 corpus 무결성 검사는 held-out test bytes도 확인할 수 있지만 test 예측·학습·선택·결과 분석은 하지 않았습니다.', '']
    if report['status'] != 'COMPLETE_NOT_PROMOTED':
        return '\n'.join(lines + ['실험 또는 증거가 미완료이므로 성능 판정을 하지 않습니다.', ''])
    lines += ['| Seed | 모델/head | val ADE m | val FDE m | 속도 MAE m/s | selected geometry | 상대 | 절대 |',
        '|---|---|---:|---:|---:|---:|---|---|']
    for seed in report['seeds']:
        for row in [seed['baseline'], *seed['arms']]:
            metrics = row['metrics']
            lines.append(f'| {seed["seed"]} | {row["arm"]} | {metrics["selected_ade_m"]:.6f} | {metrics["selected_fde_m"]:.6f} | '
                f'{metrics["selected_speed_mae_mps"]:.6f} | {row["geometry"]["selected_pass_count"]}/337 | '
                f'{row.get("candidate_screen", "기준")} | {row.get("absolute_quality", "기준")} |')
    lines += ['', '## 3-seed 판정', '']
    for arm, result in report['arm_screens'].items():
        lines.append(f'- `{arm}`: 상대 `{result["candidate_screen"]}`, 절대 `{result["absolute_quality"]}`. 자동 승격 없음.')
    lines += ['', '각 head는 1,540 steps·batch4·6,155 sample exposures이며 5개 full epoch와 420개 노출분의 partial epoch입니다.',
        '후보 geometry가 고정되어도 selector가 다른 후보를 선택할 수 있으므로 selected geometry를 별도로 검사했습니다.',
        '캐시·head 파일은 bytes SHA로 검증했습니다. 이 요약기는 tensor를 다시 load/추론하지 않으며 의미론적 freeze 확인은 고정 소스 worker의 기록입니다.', '']
    return '\n'.join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('campaign_root', type=Path)
    parser.add_argument('--parent-campaign', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args(argv)
    report = summarize_campaign(args.campaign_root, args.parent_campaign)
    args.output_dir.mkdir(parents=True, exist_ok=False)
    for name, value in [('summary.json', json.dumps(report, indent=2, allow_nan=False) + '\n'), ('README.md', render_markdown(report))]:
        with (args.output_dir / name).open('x', encoding='utf-8') as stream:
            stream.write(value)
    print(json.dumps({'status': report['status'], 'arm_screens': report['arm_screens']}))
    return 0 if report['status'] == 'COMPLETE_NOT_PROMOTED' else 1


if __name__ == '__main__':
    raise SystemExit(main())
