#!/usr/bin/env python3
"""HH_260906 - Authenticate saved frozen-generator scorer diagnostics without inference, promotion or new screening thresholds."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import struct
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
PLAN_SHA = 'a11c5c94e94077fbbd6eb3d270fe54e62ccec5590082c28b9f76c8552479cdfa'
REPORT_SHA = '22e26bcc2b1db98594b973127bdb8c4144eaab801ec8ff53384b3bc44a9fd7cf'
MANIFEST_SHA = '7d6552d0dc242915806e36e700a8190890c6d1a01fa1c96991798535641288b9'
TRANSPORT_SHA = '8464f790fda319021d9766e57503a20666c589726888efee86b84af0953c8cb0'
COMMIT = 'b478f02e42b94bf04bffec5c8170e05edc33b0f8'
EXTERNAL = {'external_worker.py': '469bf474d63dbaa0c7556d77af981e12eee42df8f63318fc9f522a0b38e4bb8a',
    'external_core.py': '4716c22d29b8e306b86f8835bf2a5ec2bf956bbb6859c6b6f001a261865b8de3',
    'external_oracle_helper.py': '35d3d0a464fb43c9789f6e24a35e453736c99fdceb36e3292d085c5ceb960d37',
    'external_owner_helper.py': 'a995722f1a60f192a059433afb44e6297937d97166877c5cc4de23220d15b436'}
SEEDS = ('20260903', '20260904', '20260905')
ARMS = ('linear_pair_reset', 'candidate_reset')
COUNTS = {'train': 1147, 'val': 337}
PARAMETERS = dict(linear_pair_reset=3084, candidate_reset=115201)
INDICES = (0, 31, 61, 92, 122, 153, 183, 214, 244, 275, 305, 336)
GROUPS = ('unavailable_masks', 'stationary_hold', 'moving_to_stop', 'continuing_motion', 'other_motion')
DENIALS = ('generator_training', 'candidate_regeneration_during_head_fit', 'runtime_checkpoint_export',
    'automatic_promotion', 'vehicle_control_approved', 'training_data_approved', 'source_or_label_changes',
    'test_neural_inference', 'test_used_for_selection')
IDENTITY = ('index', 'sample_id', 'episode_id', 'sequence_index', 'anchor_timestamp_ns', 'camera_sha256',
    'source_manifest_sha256', 'model_input_sha256', 'target_sha256', 'raw_current_vx_mps',
    'target_motion_group', 'valid_future_points')
COST_INVARIANT = ('candidate_composite_costs', 'candidate_ade_m', 'composite_oracle_index',
    'exact_minimum_indices', 'original_target_valid')
SPEED_FIELDS = ('terminal_speed_mps', 'terminal_exact_zero', 'terminal_at_or_below_0p1_mps',
    'first_exact_zero_future_index', 'reacceleration_after_exact_future_zero', 'nonincreasing_speed_exact')
METRICS = ('selected_ade_m', 'selected_fde_m', 'selected_speed_mae_mps', 'oracle_ade_m', 'ade_selection_regret_m')
CORE_METRICS = ('loss', 'regression_loss', 'candidate_score_loss', 'oracle_ade_m', 'selected_ade_m',
    'selected_fde_m', 'selected_speed_mae_mps', 'selected_yaw_mae_rad', 'selected_kinematic_speed_mae_mps',
    *(f'{kind}_{horizon}s_{unit}' for horizon in ('1p0', '3p0', '6p4') for kind, unit in (('ade', 'm'), ('fde', 'm'), ('speed_mae', 'mps'))))
FAILURE_CODES = {'nonfinite_or_shape', 'spatial', 'speed', 'geometric_speed', 'speed_disagreement', 'distance_disagreement',
    'speed_rate', 'geometric_speed_rate', 'stationary_drift', 'first_distance', 'first_behind', 'backward_step',
    'step', 'heading', 'curvature', 'lateral_acceleration', 'extent', 'contract'}


def require(value, message):
    if not value: raise ValueError(message)


def encoded(value): return (json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False) + '\n').encode()


def sha(raw): return hashlib.sha256(raw).hexdigest()


def finite(value): return type(value) in (int, float) and math.isfinite(value)


def hexsha(value): return type(value) is str and re.fullmatch('[0-9a-f]{64}', value) is not None


def same(a, b, message): require(encoded(a) == encoded(b), message)


def decode(raw):
    def pairs(items):
        result = {}
        for key, value in items:
            require(key not in result, 'duplicate JSON key'); result[key] = value
        return result
    value = json.loads(raw, object_pairs_hook=pairs, parse_constant=lambda _: require(False, 'nonfinite JSON'))
    def check(item):
        if isinstance(item, float): require(math.isfinite(item), 'nonfinite JSON number')
        elif isinstance(item, dict):
            for child in item.values(): check(child)
        elif isinstance(item, list):
            for child in item: check(child)
    check(value)
    return value


def safe_name(name):
    require(type(name) is str and name and not Path(name).is_absolute() and '..' not in Path(name).parts
        and Path(name).as_posix() == name and '\\' not in name and '\n' not in name, 'unsafe artifact path')
    return name


def regular(path):
    path = Path(path).absolute()
    require(path.is_file() and not any(p.is_symlink() for p in (path, *path.parents)), 'regular nonsymlink evidence required')
    return path


def digest(path): return sha(regular(path).read_bytes())


def timestamp(value):
    require(type(value) is str, 'UTC timestamp required')
    result = datetime.fromisoformat(value.replace('Z', '+00:00'))
    require(result.utcoffset() is not None and result.utcoffset().total_seconds() == 0, 'UTC timestamp required')
    return result


def json_rows(raw, count):
    require(raw.endswith(b'\n'), 'truncated JSONL final row')
    result = [decode(line) for line in raw.splitlines()]
    require(len(result) == count, 'all rows required')
    return result


def inventory(root):
    result = {}
    require(not any(p.is_symlink() for p in (root, *root.parents)), 'symlink input directory')
    for path in sorted(root.rglob('*')):
        require(not path.is_symlink(), 'symlink in evidence inventory')
        if path.is_file(): result[path.relative_to(root).as_posix()] = digest(path)
        else: require(path.is_dir(), 'nonregular evidence entry')
    return result


def manifest(raw):
    require(raw.endswith(b'\n'), 'complete original SHA256SUMS required')
    result = {}
    for line in raw.decode().splitlines():
        value, name = line.split('  '); safe_name(name)
        require(hexsha(value) and name not in result and name != 'SHA256SUMS', 'invalid original manifest entry')
        result[name] = value
    return result


def expected_names(sources):
    names = {'plan.json', 'report.json'} | {'source/' + safe_name(n) for n in sources}
    for seed in SEEDS:
        prefix = f'seed_{seed}/'
        names |= {prefix + split + suffix for split in COUNTS for suffix in ('_cache.pt', '_cache_progress.jsonl')}
        for view in ('original', *(a + '/analysis' for a in ARMS)):
            names |= {prefix + view + '/' + s + '/' + n for s in COUNTS for n in ('samples.jsonl', 'summary.json')}
            names |= {prefix + view + '/val/' + f'val_{i:03d}.png' for i in INDICES}
        names |= {prefix + arm + '/' + n for arm in ARMS for n in ('metrics.jsonl', 'report.json', 'head_only.pt', 'final_logits.pt')}
    return names


def verify_transport(root, before, original, missing):
    if not missing:
        return dict(tensor_bytes_locally_verified=True, remote_transport_rechecked=False)
    require('transport_verification.json' in before, 'omitted tensors require explicit verified transport receipt')
    require(before['transport_verification.json'] == TRANSPORT_SHA, 'frozen actual transport receipt differs')
    value = decode(regular(root / 'transport_verification.json').read_bytes())
    require(value['status'] == 'VERIFIED_SIX_HEAD_FITS_NOT_PROMOTED' and value['source_commit'] == COMMIT
        and value['plan_sha256'] == PLAN_SHA and value['report_sha256'] == REPORT_SHA
        and value['original_manifest_sha256'] == MANIFEST_SHA, 'transport execution pins differ')
    for key in ('remote_pre_post_bytes_unchanged', 'source_and_parent_bytes_unchanged',
        'cooperative_gpu0_lease_available_and_held', 'first_rejected_launcher_preserved', 'local_launch_evidence_unchanged'):
        require(value[key] is True, 'transport postcheck failed: ' + key)
    require(all(value[key] is False for key in ('collector_tensor_loading', 'tensor_artifacts_transferred', 'model_promotion', 'data_admission')),
        'transport must not load/transfer tensors or admit data')
    expected = dict(original, SHA256SUMS=MANIFEST_SHA); remote = value['regular_file_inventory']
    require(set(remote) == set(expected) and value['remote_file_count'] == 205 and value['collected_file_count'] == 187
        and set(value['omitted_files']) == missing, 'complete transport inventory differs')
    for name, pin in expected.items():
        entry = remote[name]
        require(entry['sha256'] == pin and type(entry['size_bytes']) is int and entry['size_bytes'] >= 0, 'remote byte inventory differs')
        if name in missing:
            omitted = value['omitted_files'][name]
            require(omitted['sha256'] == pin and omitted['size_bytes'] == entry['size_bytes'] and bool(omitted['reason']), 'omitted tensor witness differs')
        else:
            require(before[name] == pin and regular(root / name).stat().st_size == entry['size_bytes'], 'mirrored byte count/hash differs')
    require(timestamp(value['collected_at_utc']) >= timestamp(decode(regular(root / 'report.json').read_bytes())['completed_at_utc']),
        'collection predates completion')
    return dict(tensor_bytes_locally_verified=False, remote_transport_rechecked=True,
        receipt_sha256=before['transport_verification.json'], remote_tensor_byte_hashes_verified_by_collector=True,
        local_reader_loaded_tensors=False, omitted_file_count=len(missing))


def verify_sources(root, plan, report):
    sources = dict(plan['source_sha256'], **EXTERNAL)
    require(len(plan['source_sha256']) == 18 and report['source_sha256'] == sources, 'exact22 execution source identities required')
    require(plan['worker_source_sha256'] == EXTERNAL['external_worker.py']
        and plan['core_source_sha256'] == EXTERNAL['external_core.py'], 'reviewed external scorer sources required')
    environment = dict(os.environ, GIT_NO_LAZY_FETCH='1', GIT_ALLOW_PROTOCOL='', GIT_TERMINAL_PROMPT='0')
    for name, value in sources.items():
        raw = regular(root / 'source' / safe_name(name)).read_bytes()
        require(hexsha(value) and sha(raw) == value, 'executed source archive differs: ' + name)
        if name not in EXTERNAL:
            old = subprocess.check_output(['git', '-c', 'protocol.allow=never', 'show', COMMIT + ':' + name],
                cwd=ROOT, env=environment, timeout=15)
            require(old == raw, 'source does not match offline historical b478: ' + name)
    projection = plan['loss_projection']; same(report['loss_projection'], projection, 'loss projection differs')
    loss = (root / 'source/portable_e2e/losses.py').read_bytes()
    require(sha(loss) == projection['original_loss_sha256'] and sha(b''.join(loss.splitlines(keepends=True)
        [projection['prefix_first_line'] - 1:projection['prefix_last_line']])) == projection['prefix_source_sha256'], 'original cost prefix differs')
    return sources


def check_cost(cost):
    arrays = [cost[n] for n in ('candidate_composite_costs', 'candidate_logits', 'candidate_ade_m', 'candidate_probabilities')]
    require(all(type(a) is list and len(a) == 12 and all(finite(v) for v in a) for a in arrays), 'all12 finite candidate costs required')
    values, logits, ade, probabilities = arrays
    require(all(v >= 0 for v in values + ade) and all(0 <= v <= 1 for v in probabilities)
        and abs(sum(probabilities) - 1) < 1e-6, 'invalid composite cost/ADE/probability')
    minimum = min(values); best = values.index(minimum); selected = max(range(12), key=logits.__getitem__)
    ties = [i for i, value in enumerate(values) if value == minimum]
    expected = dict(composite_oracle_index=best, exact_minimum_indices=ties, exact_minimum_count=len(ties),
        minimum_composite_cost=minimum, second_minus_first_composite_cost=sorted(values)[1] - minimum,
        selected_candidate_index=selected, selected_in_exact_minimum=selected in ties,
        selected_composite_regret=values[selected] - minimum, ade_oracle_index=ade.index(min(ade)),
        selected_ade_m=ade[selected], composite_oracle_ade_m=ade[best], oracle_ade_m=min(ade))
    for key, value in expected.items(): same(cost[key], value, 'saved hard-oracle arithmetic differs: ' + key)
    require(cost['candidate_cost_dtype'] == 'torch.float32' and cost['native_oracle_and_regression_exact'] is True,
        'source-bound original native oracle comparison required')
    require(cost['original_target_valid'] == [True] * 64 and all(type(v) is bool for v in cost['original_target_valid']), 'full64 masks required')


def check_row(row, index):
    same(row['index'], index, 'ordered unique sample index required')
    require(row['candidate_count'] == 12 and type(row['candidate_count']) is int and row['valid_future_points'] == 64
        and type(row['valid_future_points']) is int and row['target_motion_group'] in GROUPS, 'full K12/future/group contract differs')
    require(all(type(row[n]) is str and row[n] for n in ('sample_id', 'episode_id'))
        and all(type(row[n]) is int and row[n] >= 0 for n in ('sequence_index', 'anchor_timestamp_ns')), 'sample identity type differs')
    require(all(hexsha(row[n]) for n in ('model_input_sha256', 'target_sha256', 'source_manifest_sha256'))
        and len(row['camera_sha256']) == 6 and all(hexsha(x) for x in row['camera_sha256'])
        and finite(row['raw_current_vx_mps']), 'input/camera/raw-velocity identity differs')
    cost = row['loss_alignment']; check_cost(cost); selected = cost['selected_candidate_index']
    same(row['selected_candidate_index'], selected, 'cost and behavior selection differ')
    for key in ('raw_current_vx_mps', 'target_motion_group', 'episode_id'): same(row[key], cost[key], 'cost/behavior identity differs')
    require(cost['capture_phase'] == 'not_reclassified_by_this_experiment', 'unrecorded native phase classification')
    candidates = row['candidates']; require(len(candidates) == 12, 'all12 behavior candidates required')
    for i, candidate in enumerate(candidates):
        same(candidate['candidate_index'], i, 'candidate order differs')
        require(candidate['family'] == ('DRIVE' if i < 6 else 'STOP'), 'candidate family differs')
        require(all(finite(candidate[n]) and candidate[n] >= 0 for n in ('ade_m', 'fde_m', 'speed_mae_mps', 'terminal_speed_mps')),
            'finite nonnegative behavior metric required')
        require(all(type(candidate[n]) is bool for n in SPEED_FIELDS if n not in ('terminal_speed_mps', 'first_exact_zero_future_index')),
            'speed behavior boolean required')
        terminal, first = candidate['terminal_speed_mps'], candidate['first_exact_zero_future_index']
        require((first is None or type(first) is int and 0 <= first < 64)
            and candidate['terminal_exact_zero'] is (terminal == 0) and candidate['terminal_at_or_below_0p1_mps'] is (terminal <= .1)
            and (terminal != 0 or first is not None) and (first != 63 or terminal == 0)
            and (not candidate['reacceleration_after_exact_future_zero'] or first is not None and first < 63)
            and (not candidate['nonincreasing_speed_exact'] or not candidate['reacceleration_after_exact_future_zero'])
            and (first is None or terminal == 0 or candidate['reacceleration_after_exact_future_zero']), 'inconsistent terminal/reacceleration record')
    same(row['selected_speed_behavior'], {k: candidates[selected][k] for k in SPEED_FIELDS}, 'selected speed behavior differs')
    for key, target in zip(METRICS[:3], ('ade_m', 'fde_m', 'speed_mae_mps')):
        same(row[key], candidates[selected][target], 'selected candidate metric differs')
    minimum = min(c['ade_m'] for c in candidates)
    same(row['oracle_ade_m'], minimum, 'ADE oracle differs')
    same(row['ade_selection_regret_m'], row['selected_ade_m'] - minimum, 'ADE regret differs')
    require(row['selected_stop_candidate'] is (selected >= 6), 'STOP selection differs')
    gate = row['runtime_geometry']; require(len(gate['candidates']) == 12, 'all12 saved candidate gates required')
    for i, item in enumerate(gate['candidates']):
        same(item['candidate_index'], i, 'gate candidate order differs'); codes = item['failure_codes']
        require(type(item['geometry_pass']) is bool and type(codes) is list and len(codes) == len(set(codes))
            and all(c in FAILURE_CODES for c in codes) and item['geometry_pass'] is (not codes), 'inconsistent saved candidate gate')
    same(gate['selected_candidate_index'], selected, 'raw gate selected index differs')
    require(gate['selected_geometry_pass'] is gate['candidates'][selected]['geometry_pass']
        and gate['selected_failure_codes'] == gate['candidates'][selected]['failure_codes']
        and gate['all_candidates_geometry_pass'] is all(c['geometry_pass'] for c in gate['candidates'])
        and gate['any_candidate_geometry_pass'] is any(c['geometry_pass'] for c in gate['candidates']), 'saved gate aggregate differs')


def cost_aggregate(rows):
    keys = ('selected_ade_m', 'composite_oracle_ade_m', 'oracle_ade_m', 'selected_composite_regret', 'second_minus_first_composite_cost')
    return dict(sample_count=len(rows), selected_in_exact_minimum_count=sum(r['selected_in_exact_minimum'] for r in rows),
        multiple_exact_minimum_count=sum(r['exact_minimum_count'] > 1 for r in rows), raw_vx_nonpositive_count=sum(r['raw_current_vx_mps'] <= 0 for r in rows),
        selected_composite_oracle_index_agreement_count=sum(r['selected_candidate_index'] == r['composite_oracle_index'] for r in rows),
        composite_ade_oracle_index_agreement_count=sum(r['composite_oracle_index'] == r['ade_oracle_index'] for r in rows),
        selected_histogram={str(i): sum(r['selected_candidate_index'] == i for r in rows) for i in range(12)},
        composite_oracle_histogram={str(i): sum(r['composite_oracle_index'] == i for r in rows) for i in range(12)},
        means={k: sum(r[k] for r in rows) / len(rows) if rows else None for k in keys})


def cost_groups(rows):
    result = {'all_samples': cost_aggregate(rows)}
    for key in ('episode_id', 'target_motion_group', 'capture_phase'):
        categories = GROUPS if key == 'target_motion_group' else sorted({r[key] for r in rows})
        result[key] = {v: cost_aggregate([r for r in rows if r[key] == v]) for v in categories}
    result['raw_nonpositive'] = cost_aggregate([r for r in rows if r['raw_current_vx_mps'] <= 0])
    return result


def compare_aggregates(recorded, reconstructed, path=()):
    # HH_260906 - Preserve the existing entropy-only 1e-15 platform roundoff rule; all other saved aggregates remain exact.
    if path and path[-1] == 'normalized_selection_entropy':
        require(finite(recorded) and finite(reconstructed) and 0 <= recorded <= 1 and 0 <= reconstructed <= 1
            and math.isclose(recorded, reconstructed, abs_tol=1e-15, rel_tol=1e-15), 'entropy reconstruction differs')
    elif isinstance(recorded, dict) and isinstance(reconstructed, dict):
        require(set(recorded) == set(reconstructed), 'aggregate keys differ')
        for key in recorded: compare_aggregates(recorded[key], reconstructed[key], (*path, key))
    else: same(recorded, reconstructed, 'aggregate reconstruction differs: ' + '/'.join(path))


def oracle_digest(indices):
    require(all(type(i) is int and 0 <= i < 12 for i in indices), 'integer oracle tensor values required')
    header = json.dumps(['composite_oracle', 'torch.int64', [len(indices)]], separators=(',', ':')).encode()
    return sha(header + b'\0' + struct.pack('<' + 'q' * len(indices), *indices))


def verify_history(rows, head, seed):
    same(rows, head['history'], 'persisted fit history differs')
    require(len(rows) == 1540, 'full1540 fit history required')
    from portable_e2e.train import _epoch_batches
    expected = [(epoch, list(indices)) for epoch in range(6)
        for indices in _epoch_batches(1147, batch_size=4, seed=int(seed), epoch=epoch)][:1540]
    seen, order = 0, []
    for step, (row, (epoch, indices)) in enumerate(zip(rows, expected), 1):
        seen += len(indices)
        for name, value in dict(global_step=step, epoch=epoch, batch_samples=len(indices), samples_seen=seen, sample_indices=indices).items():
            same(row[name], value, 'fixed seeded epoch sampling differs: ' + name)
        require(all(finite(row[n]) and row[n] >= 0 for n in ('candidate_score_loss', 'optimization_loss', 'gradient_norm')), 'nonfinite/negative fit metric')
        order.append(dict(epoch=epoch, indices=indices))
    require(seen == 6155, 'all6155 sample exposures required')
    value = sha(json.dumps(order, separators=(',', ':')).encode())
    require(value == head['sampling_order_sha256'], 'sampling order hash differs')
    return value


def verify_core_metrics(metric, split):
    require(metric['split'] == split and type(metric['sample_count']) is int and metric['sample_count'] == COUNTS[split]
        and set(metric['metrics']) == set(CORE_METRICS) and set(metric['metric_counts']) == set(CORE_METRICS)
        and all(finite(v) and v >= 0 for v in metric['metrics'].values())
        and all(type(v) is int and v == COUNTS[split] for v in metric['metric_counts'].values()), 'all core metrics/full-mask denominators required')
    for key in ('selected_histogram', 'composite_oracle_histogram'):
        values = metric[key]
        require(type(values) is list and len(values) == 12 and all(type(v) is int and v >= 0 for v in values)
            and sum(values) == COUNTS[split], 'core evaluation histogram denominator differs')
    count = metric['selected_composite_oracle_agreement_count']
    require(type(count) is int and 0 <= count <= COUNTS[split], 'core oracle agreement count differs')


def verify_analysis(root, prefix, split, cache_digest, read, sources):
    from scripts.e2e.audit_portable_stopmix_behavior import summarize_rows
    report = decode(read(prefix + '/summary.json')); raw = read(prefix + '/samples.jsonl'); rows = json_rows(raw, COUNTS[split])
    require(report['schema'] == 'portable_e2e.cached_stopmix_selection_diagnostic.v1' and report['split'] == split
        and report['sample_count'] == len(rows) and type(report['sample_count']) is int and report['samples_sha256'] == sha(raw)
        and report['cache_digest'] == cache_digest and all(report[n] is False for n in DENIALS), 'cached split summary binding differs')
    for index, row in enumerate(rows): check_row(row, index)
    require(len({r['sample_id'] for r in rows}) == len(rows)
        and len({r['episode_id'] for r in rows}) == (3 if split == 'train' else 1), 'complete unique split identity required')
    costs = [r['loss_alignment'] for r in rows]
    invariant = [{k: r[k] for k in COST_INVARIANT} for r in costs]
    require(report['oracle_and_cost_sha256'] == sha(encoded(invariant))
        and report['oracle_indices'] == [r['composite_oracle_index'] for r in costs], 'fixed cost/oracle binding differs')
    reconstructed = summarize_rows(rows, 12)
    compare_aggregates({k: report[k] for k in reconstructed}, reconstructed)
    same(report['composite_cost_alignment'], cost_groups(costs), 'composite cost groups differ')
    require([r['index'] for r in report['renders']] == (list(INDICES) if split == 'val' else []), 'fixed validation render indices differ')
    from PIL import Image
    for render in report['renders']:
        row = rows[render['index']]; name = f"val_{render['index']:03d}.png"
        require(render['file'] == name and render['sha256'] == sha(read(prefix + '/' + name))
            and render['sample_id'] == row['sample_id'] and render['camera_sha256'] == row['camera_sha256'], 'PNG/source row binding differs')
        with Image.open(regular(root / prefix / name)) as image:
            require(image.format == 'PNG' and image.size == (900, 700), 'fixed original PNG ABI differs'); image.load()
    return report, rows


def verify_head(head, rows, seed, arm, plan, state, analyses, baselines):
    expected = dict(artifact_id='portable_e2e.frozen_stopmix_selector_research_head.v1', status='RESEARCH_HEAD_TRAINING_COMPLETE_NOT_PROMOTED',
        arm=arm, seed=int(seed), device='cuda:0', optimizer='AdamW', steps=1540, batch_size=4, learning_rate=.0001,
        weight_decay=.0001, candidate_score_weight=.1, maximum_gradient_norm=5., loss_config=plan['loss_config'],
        source_checkpoint_sha256=plan['parent_checkpoint_sha256'][seed], corpus_fingerprint_sha256=plan['corpus_fingerprint_sha256'],
        split_fingerprints={k: v['fingerprint_sha256'] for k, v in plan['split_contract'].items()},
        head_parameter_count=PARAMETERS[arm], candidate_count=12,
        final_state=dict(global_step=1540, samples_seen=6155, last_epoch_index=5))
    for key, value in expected.items(): same(head[key], value, 'head configuration differs: ' + key)
    require(all(head[k] is False for k in ('generator_parameters_trained', 'training_data_approved', 'vehicle_control_approved',
        'automatic_promotion', 'runtime_bundle_supported')), 'head research denial changed')
    require(head['cache_sha256_before'] == head['cache_sha256_after'] == state['cache_digest']
        and head['original_head_sha256_before'] == head['original_head_sha256_after'], 'cache or original head changed')
    require(set(head['original_head_sha256_before']) == {'drive', 'stop'}
        and all(hexsha(v) for v in head['original_head_sha256_before'].values())
        and all(hexsha(head[n]) for n in ('initial_head_sha256', 'final_head_sha256')), 'original/trained head digests required')
    order = verify_history(rows, head, seed)
    for split in COUNTS:
        summary, samples = analyses[split]; baseline, original = baselines[split]
        same(head[split + '_sample_ids'], [r['sample_id'] for r in samples], 'core ordered sample IDs differ')
        require(head['composite_oracle_sha256'][split] == oracle_digest(summary['oracle_indices']), 'core oracle tensor hash differs')
        parity = head['original_logit_parity'][split]
        require(parity['cached_original_logits_required'] is True and parity['byte_equal'] is True
            and finite(parity['max_abs_logit_difference']) and parity['max_abs_logit_difference'] == 0
            and type(parity['selected_index_mismatch_count']) is int and parity['selected_index_mismatch_count'] == 0,
            'original two-head cached logits were not reproduced exactly')
        for label, measured in (('baseline_metrics', baseline), ('post_training_metrics', summary)):
            metric = head[label][split]; costs = measured['composite_cost_alignment']['all_samples']
            verify_core_metrics(metric, split)
            require(metric['selected_histogram'] == [costs['selected_histogram'][str(i)] for i in range(12)]
                and metric['composite_oracle_histogram'] == [costs['composite_oracle_histogram'][str(i)] for i in range(12)]
                and metric['selected_composite_oracle_agreement_count'] == costs['selected_composite_oracle_index_agreement_count'],
                'core evaluation selection/count differs')
        before = head['pre_training_metrics'][split]
        verify_core_metrics(before, split)
        same(before['composite_oracle_histogram'], head['baseline_metrics'][split]['composite_oracle_histogram'], 'pre-training oracle histogram changed')
    return order


def comparison(original, candidate):
    # HH_260906 - Report every group and signed difference; these are observations, not a new success/promotion screen.
    def delta(a, b):
        require(a['sample_count'] == b['sample_count'], 'comparison denominator changed')
        if not a['sample_count']: return dict(sample_count=0, metric_delta=None)
        return dict(sample_count=a['sample_count'], metric_delta={k: b['metrics'][k] - a['metrics'][k] for k in METRICS},
            selected_geometry_pass_count_delta=b['geometry']['selected_result']['geometry_pass_count'] - a['geometry']['selected_result']['geometry_pass_count'],
            stop_selection_count_delta=b['stop_selection_count'] - a['stop_selection_count'])
    return dict(all_samples=delta(original['all_samples'], candidate['all_samples']),
        target_motion_groups={g: delta(original['target_motion_groups'][g], candidate['target_motion_groups'][g]) for g in GROUPS})


def outcome_ledger(before, report=None):
    # HH_260906 - Keep the entire preregistered denominator visible even when a worker fails before its first parent or final report.
    result = []
    for seed in SEEDS:
        for arm in ARMS:
            prefix = f'seed_{seed}/{arm}/'
            states = [a for s in (report or {}).get('seeds', []) if s.get('seed') == seed
                for a in s.get('arms', []) if a.get('arm') == arm]
            require(len(states) <= 1, 'duplicate partial head state')
            recorded = states[0].get('status') if states else None
            result.append(dict(seed=seed, arm=arm, status='INCOMPLETE' if any(n.startswith(prefix) for n in before) or states else 'NOT_RUN',
                recorded_worker_status=recorded))
    return result


def summarize_campaign(input_root):
    root = Path(input_root).absolute(); before = inventory(root); own_sha = digest(Path(__file__))
    require('plan.json' in before and before['plan.json'] == PLAN_SHA, 'exact prospective plan required')
    read = lambda name: regular(root / safe_name(name)).read_bytes()
    plan = decode(read('plan.json'))
    result = dict(schema='portable_e2e.frozen_stopmix_selector_summary.v1', status='INCOMPLETE_NOT_PROMOTED',
        plan_sha256=PLAN_SHA, summary_source_sha256=own_sha, seeds=[], **{k: False for k in DENIALS})
    if 'report.json' not in before:
        result['performed_heads'] = outcome_ledger(before)
        result['input_sha256'] = before; require(inventory(root) == before, 'incomplete inputs changed'); return result
    report = decode(read('report.json'))
    result['performed_heads'] = outcome_ledger(before, report)
    require(report['schema'] == 'portable_e2e.external_frozen_stopmix_selector.v1' and report['plan_sha256'] == PLAN_SHA
        and plan['source_commit'] == COMMIT and all(report[n] is False and plan[n] is False for n in DENIALS), 'worker plan/scope differs')
    if report['status'] != 'SIX_HEAD_FITS_COMPLETE_NOT_PROMOTED':
        require(report['status'] == 'FAILED_OR_PARTIAL_NOT_PROMOTED', 'unknown worker completion status')
        result.update(worker_status=report['status'], worker_report_sha256=before['report.json'], retained_worker_report=report,
            input_sha256=before, interpretation='Failed or partial evidence retained; no complete numerical comparison claimed.')
        require(inventory(root) == before, 'partial inputs changed'); return result
    require(report['completed_head_fit_count'] == 6 and type(report['completed_head_fit_count']) is int and report['postcheck_errors'] == []
        and report['source_and_inputs_unchanged'] is True and report['deadline_met'] is True, 'complete source-bound six-head experiment required')
    require(before['report.json'] == REPORT_SHA and before.get('SHA256SUMS') == MANIFEST_SHA, 'frozen actual completion evidence differs')
    require(timestamp(plan['declared_at_utc']) <= timestamp(report['started_at_utc']) <= timestamp(report['completed_at_utc'])
        <= timestamp(plan['finish_before_utc']), 'prospective execution chronology differs')
    require((timestamp(report['completed_at_utc']) - timestamp(report['started_at_utc'])).total_seconds() <= 3600, 'external study time budget exceeded')
    sources = verify_sources(root, plan, report); original = manifest(read('SHA256SUMS'))
    require(set(original) == expected_names(sources), 'complete original204-payload inventory required')
    missing = set(original) - set(before)
    require(all(n.endswith('.pt') for n in missing) and len(missing) in (0, 18), 'only all18 remote research tensors may be omitted')
    for name in set(original) & set(before): require(before[name] == original[name], 'original manifest payload differs: ' + name)
    require(not any(n not in original and n not in ('SHA256SUMS', 'transport_verification.json') for n in before), 'unexpected local result payload')
    transport = verify_transport(root, before, original, missing)
    # HH_260906 - Omitted tensor hashes are original worker manifest witnesses, not claims that this reader verified remote or local tensor bytes.
    require(report['torch_device_uuid'] == plan['gpu_uuid'].removeprefix('GPU-'), 'recorded visible GPU UUID differs')
    same(report['gpu_preflight'], dict(physical_index=0, uuid=plan['gpu_uuid'], pre_cuda_compute_processes=0), 'recorded exclusive physical GPU0 preflight differs')
    names = ('contract', 'dataset', 'torch_dataset', 'losses', 'model', 'train', 'evaluate', 'audit_runtime', 'runtime',
        'runtime_contract', 'frozen_selector', 'visualize', 'stop_primitive_research')
    same(report['imported_native_modules'], dict({'portable_e2e.' + n: 'portable_e2e/' + n + '.py' for n in names},
        **{'scripts.e2e.audit_portable_stopmix_behavior': 'scripts/e2e/audit_portable_stopmix_behavior.py'}), 'native import provenance differs')
    require(len(report['original_input_sha256']) == 17 and all(hexsha(v) for v in report['original_input_sha256'].values()), 'all17 original lineage witnesses required')
    for name, pin in plan['original_receipt_sha256'].items(): require(report['original_input_sha256'].get(name) == pin, 'lineage receipt differs')
    for seed in SEEDS:
        name = f"runs/campaigns/{plan['parent_campaign_id']}/seed_{seed}/B_drive_stop_mix/training/checkpoints/latest.pt"
        require(report['original_input_sha256'].get(name) == plan['parent_checkpoint_sha256'][seed], 'parent checkpoint lineage differs')
    # HH_260906 - Only pure saved-row aggregation and CPU seeded sampling are reused; no checkpoint/NN/data-loader operation is called.
    current_sources = {n: digest(ROOT / n) for n in plan['source_sha256']}
    require(current_sources == plan['source_sha256'], 'local numerical helper dependency bytes differ from frozen source')
    require([s['seed'] for s in report['seeds']] == list(SEEDS), 'all3 ordered parents required')
    shared_ids = {}; all_head_reports = []; previous_end = timestamp(report['started_at_utc'])
    for state, seed in zip(report['seeds'], SEEDS):
        prefix = 'seed_' + seed
        require(state['directory'] == prefix and state['status'] == 'TWO_HEADS_COMPLETE_NOT_PROMOTED'
            and state['parent_checkpoint_sha256'] == plan['parent_checkpoint_sha256'][seed]
            and hexsha(state['generator_state_sha256']) and state['generator_state_sha256'] == state['generator_state_after_sha256'], 'completed frozen generator required')
        model_config = decode(read('source/' + plan['model_config']))
        provenance = state['checkpoint_validation']
        for name, value in dict(checkpoint_sha256=state['parent_checkpoint_sha256'], checkpoint_id='portable_e2e.pytorch_checkpoint.v1',
            model_config_sha256=sha(json.dumps(model_config, sort_keys=True, separators=(',', ':')).encode()),
            training_dataset_fingerprint_sha256=plan['split_contract']['train']['fingerprint_sha256'], training_episode_count=3,
            training_sampling_policy='uniform_without_replacement', training_domain_samples_seen={'carla': 11470}).items():
            same(provenance[name], value, 'continued parent checkpoint metadata differs: ' + name)
        require(hexsha(provenance['training_sampling_plan_sha256']), 'parent sampling plan hash required')
        same(state['completed_files'], {n[len(prefix) + 1:]: v for n, v in original.items() if n.startswith(prefix + '/')}, 'seed file sealing differs')
        baselines = {}; measured = {}; seed_result = dict(seed=seed, parent_checkpoint_sha256=state['parent_checkpoint_sha256'], variants={}, comparisons={})
        for split, count in COUNTS.items():
            require(hexsha(state['cache_digest'][split]), 'cache SHA required')
            baseline = verify_analysis(root, prefix + '/original/' + split, split, state['cache_digest'][split], read, sources)
            baselines[split] = baseline; ledger = json_rows(read(prefix + '/' + split + '_cache_progress.jsonl'), count)
            same(ledger, [{k: r[k] for k in ('index', 'sample_id', 'model_input_sha256', 'target_sha256')} for r in baseline[1]], 'cache journal binding differs')
            ids = [{k: row[k] for k in IDENTITY} for row in baseline[1]]
            if split in shared_ids: same(shared_ids[split], ids, 'parent seeds used different ordered inputs/targets')
            shared_ids[split] = ids
        seed_result['variants']['original'] = {s: x[0] for s, x in baselines.items()}
        require([a['arm'] for a in state['arms']] == list(ARMS), 'both ordered fresh head arms required')
        order = None; paired_baseline_metrics = None
        for arm_state, arm in zip(state['arms'], ARMS):
            item = prefix + '/' + arm; head = decode(read(item + '/report.json'))
            require(arm_state['status'] == 'HEAD_COMPLETE_NOT_PROMOTED', 'complete head required')
            same(arm_state['files'], {n[len(item) + 1:]: v for n, v in original.items() if n.startswith(item + '/')}, 'head file sealing differs')
            ended = timestamp(arm_state['completed_at_utc']); require(previous_end <= ended <= timestamp(state['completed_at_utc']), 'head chronology differs'); previous_end = ended
            analyses = {s: verify_analysis(root, item + '/analysis/' + s, s, state['cache_digest'][s], read, sources) for s in COUNTS}
            for split in COUNTS:
                for a, b in zip(baselines[split][1], analyses[split][1]):
                    same({k: a[k] for k in IDENTITY}, {k: b[k] for k in IDENTITY}, 'head changed cached input/target identity')
                    same(a['candidates'], b['candidates'], 'head changed frozen candidate behavior')
                    same(a['runtime_geometry']['candidates'], b['runtime_geometry']['candidates'], 'head changed raw candidate gates')
                    same({k: a['loss_alignment'][k] for k in COST_INVARIANT}, {k: b['loss_alignment'][k] for k in COST_INVARIANT}, 'head changed frozen costs/oracles/masks')
            current_order = verify_head(head, json_rows(read(item + '/metrics.jsonl'), 1540), seed, arm, plan, state, analyses, baselines)
            require(order is None or current_order == order, 'paired sampling order differs'); order = current_order
            require(paired_baseline_metrics is None or head['baseline_metrics'] == paired_baseline_metrics, 'paired original baseline metrics differ')
            paired_baseline_metrics = head['baseline_metrics']; all_head_reports.append(head)
            measured[arm] = {s: x[0] for s, x in analyses.items()}; seed_result['variants'][arm] = measured[arm]
            seed_result['comparisons'][arm + '_minus_original'] = {s: comparison(baselines[s][0], measured[arm][s]) for s in COUNTS}
        seed_result['comparisons']['candidate_minus_linear'] = {s: comparison(measured[ARMS[0]][s], measured[ARMS[1]][s]) for s in COUNTS}
        seed_result['sampling_order_sha256'] = order; result['seeds'].append(seed_result)
        require(timestamp(state['completed_at_utc']) <= timestamp(report['completed_at_utc']), 'parent completion after report')
        previous_end = timestamp(state['completed_at_utc'])
    require(not {r['sample_id'] for r in shared_ids['train']} & {r['sample_id'] for r in shared_ids['val']}
        and not {r['episode_id'] for r in shared_ids['train']} & {r['episode_id'] for r in shared_ids['val']}, 'TRAIN/VAL overlap')
    result.update(status='COMPLETE_DIAGNOSTIC_NOT_PROMOTED', worker_report_sha256=before['report.json'], source_commit=COMMIT,
        execution_source_sha256=sources, input_sha256=before, original_payload_sha256=original,
        transport_verification=transport,
        remote_only_tensor_manifest_witnesses={n: original[n] for n in sorted(missing)}, local_tensor_bytes_verified=not missing,
        completed_head_fit_count=6, full_model_fit_count=0, original_baseline_count=3, analysis_variant_count=9,
        train_rows_per_variant=1147, val_rows_per_variant=337, total_analysis_rows=13356, head_history_rows=9240,
        head_sample_exposures=36930, cache_journal_rows=4452, fixed_val_png_count=108,
        recorded_core_metrics=[{k: h[k] for k in ('seed', 'arm', 'baseline_metrics', 'pre_training_metrics', 'post_training_metrics')} for h in all_head_reports],
        interpretation='TRAIN is in-sample; VAL is one repeatedly used development episode, not an independent holdout. Compare all groups without post-hoc threshold selection. Scorer architecture/capacity (3084 versus 115201 parameters) are confounded. No winner, model promotion, admission, traffic-intent, vehicle-control or real-time claim.',
        numerical_scope='Saved hard-cost argmin/exact ties/regrets and per-candidate behavior/gate consistency independently recomputed. Full XY/speed arrays and target speeds are absent from these compact rows: no fresh trajectory-gate replay or independent future-motion-group reclassification. Native raw-current-vx gate and group computation are source-bound. GPU float32 core metric reductions retained separately from saved-row Python reductions.',
        entropy_roundoff_rule='Only normalized selection entropy permits existing absolute/relative 1e-15 roundoff; all other reconstructed saved aggregates exact.')
    for item in result['performed_heads']: item['status'] = 'COMPLETE_DIAGNOSTIC_NOT_PROMOTED'
    require(inventory(root) == before and digest(Path(__file__)) == own_sha
        and {n: digest(ROOT / n) for n in current_sources} == current_sources, 'input or summary dependency changed during analysis')
    for name in current_sources:
        module = sys.modules.get(name[:-3].replace('/', '.')) if name.endswith('.py') else None
        if module is not None: require(Path(module.__file__).resolve() == ROOT / name, 'loaded numerical helper escaped verified repository')
    return result


def run(input_root, output_dir):
    input_root, output = Path(input_root).absolute(), Path(output_dir).absolute()
    require(not output.exists() and not any(p.is_symlink() for p in (output, *output.parents)), 'create-only nonsymlink output required')
    require(not any(output.resolve() == p or p in output.resolve().parents for p in (input_root.resolve(), (ROOT / 'datasets').resolve())), 'output must be outside input/dataset trees')
    result = summarize_campaign(input_root)
    output.mkdir(parents=True, exist_ok=False)
    try:
        with (output / 'summary.json').open('xb') as stream: stream.write(encoded(result))
        require(inventory(input_root) == result['input_sha256'] and digest(Path(__file__)) == result['summary_source_sha256'], 'evidence changed during persistence')
        if 'execution_source_sha256' in result:
            require(all(digest(ROOT / name) == pin for name, pin in result['execution_source_sha256'].items() if name not in EXTERNAL),
                'numerical dependency changed during persistence')
        with (output / 'SHA256SUMS').open('x') as stream: stream.write(digest(output / 'summary.json') + '  summary.json\n')
    except BaseException as error:
        with (output / 'failed.json').open('xb') as stream: stream.write(encoded(dict(status='PUBLICATION_FAILED_NOT_PROMOTED', error_type=type(error).__name__, error=str(error))))
        raise
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('input_root', type=Path); parser.add_argument('output_dir', type=Path)
    args = parser.parse_args(argv); result = run(args.input_root, args.output_dir)
    print(json.dumps({'status': result['status'], 'summary_sha256': digest(args.output_dir / 'summary.json')}))
    return 0 if result['status'] == 'COMPLETE_DIAGNOSTIC_NOT_PROMOTED' else 2


if __name__ == '__main__': raise SystemExit(main())
