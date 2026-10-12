#!/usr/bin/env python3
"""HH_260906 - Report fixed-epoch learning curves without loading weights or TEST data."""
from __future__ import annotations

import argparse
from collections import defaultdict
import hashlib
import json
import math
from pathlib import Path

SEEDS = (20260903, 20260904, 20260905)
ARMS = ('A_original_loss', 'B_cost_aware')
RUNS = tuple(f'seed_{seed}/{arm}' for seed in SEEDS for arm in ARMS)
KEYS = ('ade_1p0s_m', 'ade_3p0s_m', 'ade_6p4s_m', 'fde_6p4s_m',
        'selected_speed_mae_mps', 'oracle_ade_m')
LIMITS = {'ade_1p0s_m': .5, 'ade_3p0s_m': 1., 'ade_6p4s_m': 2., 'fde_6p4s_m': 4.}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def decode(payload):
    def pairs(items):
        result = {}
        for key, value in items:
            require(key not in result, 'duplicate JSON field')
            result[key] = value
        return result
    return json.loads(payload, object_pairs_hook=pairs,
                      parse_constant=lambda _: require(False, 'nonfinite JSON'))


def read(path):
    path = Path(path)
    require(path.is_file() and not path.is_symlink(), f'missing regular report: {path}')
    return decode(path.read_text())


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def point(item, epoch, parent):
    train = read(item / 'training/run.json')
    evaluation = read(item / 'evaluation/metrics.json')
    audit = read(item / 'gate_v8.json')
    old = read(parent / 'training/run.json')
    require(train['status'] == 'TRAINING_TARGET_REACHED', 'unfinished training')
    require(evaluation['status'] == 'OPEN_LOOP_EVALUATION_COMPLETE', 'unfinished evaluation')
    require(audit['status'] == 'RUNTIME_GEOMETRY_AUDIT_COMPLETE', 'unfinished geometry audit')
    for report in (train, evaluation, audit):
        require(report['corpus_fingerprint_sha256'] == old['corpus_fingerprint_sha256'], 'corpus differs')
    for field in ('model_config', 'loss_config', 'dataset_fingerprint_sha256', 'training_episode_ids'):
        require(train[field] == old[field], f'training lineage differs: {field}')
    require(train['dataset_size'] == 1147 and train['model_parameter_count'] == 1056362,
            'training population or model differs')
    expected_steps = 1540 if epoch is None else 287 * epoch
    expected_samples = 6155 if epoch is None else 1147 * epoch
    expected_config = dict(old['train_config'], max_steps=expected_steps)
    require(train['train_config'] == expected_config, 'training configuration changed beyond duration')
    for field in ('runtime', 'hardware', 'sampling_plan', 'sampling_plan_sha256', 'device', 'created_at_utc'):
        require(train[field] == old[field], f'continued training provenance differs: {field}')
    if epoch is not None:
        require('resumed_at_utc' in train, 'continuation must record exact resume')
    require(train['state']['global_step'] == expected_steps and train['state']['samples_seen'] == expected_samples,
            'checkpoint endpoint exposure differs')
    for report in (evaluation, audit):
        require(report['evaluation_split'] == 'val' and report['vehicle_control_approved'] is False,
                'must be unapproved VAL research')
        require(report['training_dataset_fingerprint_sha256'] == old['dataset_fingerprint_sha256'],
                'evaluated training split differs')
    original_eval = read(parent / 'evaluation/metrics.json')
    require(evaluation['dataset_fingerprint_sha256'] == original_eval['dataset_fingerprint_sha256']
            == audit['dataset_fingerprint_sha256'], 'VAL split differs')
    require(evaluation['sample_count'] == audit['geometry']['sample_count'] == 337, 'VAL denominator differs')
    require(evaluation['checkpoint_sha256'] == audit['checkpoint_sha256'], 'evaluation/audit checkpoints differ')
    for stage, name in (('train', 'training/run.json'), ('evaluate', 'evaluation/metrics.json'), ('audit', 'gate_v8.json')):
        receipt = read(item / f'{stage}_completed.json')
        require(receipt['status'] == 'COMPLETE' and receipt['checkpoint_sha256'] == evaluation['checkpoint_sha256']
                and receipt['sha256'] == sha(item / name), 'completed receipt does not bind report/checkpoint')
    require(audit['gate'] == read(parent / 'gate_v8.json')['gate'], 'geometry gate changed')
    metrics = {key: evaluation['metrics'][key] for key in KEYS}
    require(all(type(value) in (float, int) and math.isfinite(value) for value in metrics.values()), 'invalid metric')
    require(all(evaluation['metric_counts'][key] == 337 for key in KEYS), 'metric denominator differs')
    history = (item / 'training/metrics.jsonl').read_bytes()
    parent_history = (parent / 'training/metrics.jsonl').read_bytes()
    require(history.startswith(parent_history), 'inherited training history changed')
    rows = [decode(line) for line in history.splitlines()]
    require(len(rows) == expected_steps and [r['global_step'] for r in rows] == list(range(1, expected_steps + 1)),
            'training history gaps or duplicated steps')
    require(rows[-1] == train['last_metrics'], 'training last row differs')
    for step, row in enumerate(rows, 1):
        samples = 4 * step - step // 287
        require(row['epoch'] == (step - 1) // 287
                and row['batch_domain_sample_counts'] == {'carla': 3 if step % 287 == 0 else 4}
                and row['samples_seen'] == samples and row['domain_samples_seen'] == {'carla': samples},
                'history epoch/batch/exposure differs from fixed sampling schedule')
    geometry = audit['geometry']
    return dict(epoch=epoch, effective_epochs=expected_samples / 1147,
                global_step=expected_steps, samples_seen=expected_samples,
                new_steps=expected_steps - 1540, new_samples_seen=expected_samples - 6155,
                metrics=metrics, absolute_checks={key: metrics[key] <= limit for key, limit in LIMITS.items()},
                checkpoint_sha256=evaluation['checkpoint_sha256'], geometry=geometry,
                report_sha256={name: sha(item / name) for name in
                               ('training/run.json', 'training/metrics.jsonl', 'evaluation/metrics.json', 'gate_v8.json')}), rows


def batch_curve(rows):
    # HH_260906 - Batch losses reflect changing weights, not fixed-checkpoint TRAIN evaluation.
    groups = defaultdict(list)
    for row in rows:
        count = sum(row['batch_domain_sample_counts'].values())
        require(type(count) is int and 0 < count <= 4, 'invalid batch denominator')
        require(type(row['loss']) in (float, int) and math.isfinite(row['loss']), 'invalid batch loss')
        groups[row['epoch']].append((count, row['loss']))
    return [dict(epoch=index + 1, sample_count=sum(n for n, _ in values),
                 complete_epoch=sum(n for n, _ in values) == 1147,
                 sample_weighted_batch_loss=math.fsum(n * value for n, value in values) / sum(n for n, _ in values))
            for index, values in sorted(groups.items())]


def summarize(root, parent, allow_partial=False):
    root, parent = Path(root), Path(parent)
    require(root.resolve() != parent.resolve(), 'continuation must not overwrite parent')
    result = dict(schema='portable_e2e.duration_learning_curves.v1',
                  status='PARTIAL_RESEARCH_NOT_PROMOTED', runs={}, paired_epochs=[],
                  vehicle_control_approved=False, automatic_promotion=False, test_read_or_use=False,
                  admitted_new_episodes=0, learned_closed_loop_passed=0,
                  campaign_completion_verified=False,
                  exposure_scope='Only fully trained, evaluated and geometry-audited endpoints; in-flight work is excluded.',
                  limitations=['TRAIN is three episodes (~134 seconds); VAL is one repeatedly evaluated right turn.',
                               'VAL contains zero stationary-hold samples.',
                               'Training curves are changing-model batch averages, not fixed-checkpoint TRAIN losses.',
                               'A and B use different total-loss objectives; compare task metrics, not raw total losses.',
                               'Figures are offline predictions and metrics, not learned CARLA driving.'])
    for run in RUNS:
        baseline, history = point(parent / run, None, parent / run)
        points = [baseline]
        for epoch in range(6, 21):
            item = root / 'epochs' / f'epoch_{epoch:03d}' / run
            complete = all((item / name).is_file() for name in
                           ('training/run.json', 'training/metrics.jsonl', 'evaluation/metrics.json', 'gate_v8.json',
                            'train_completed.json', 'evaluate_completed.json', 'audit_completed.json'))
            if not complete:
                require(allow_partial, f'missing complete epoch {epoch}: {run}')
                break
            record, history = point(item, epoch, parent / run)
            record['delta_from_parent'] = {key: record['metrics'][key] - baseline['metrics'][key] for key in KEYS}
            points.append(record)
        result['runs'][run] = dict(points=points, training_batch_curve=batch_curve(history),
                                   complete_new_epochs=len(points) - 1)
    for epoch in (None, *range(6, 21)):
        for seed in SEEDS:
            pair = [[p for p in result['runs'][f'seed_{seed}/{arm}']['points'] if p['epoch'] == epoch] for arm in ARMS]
            if all(pair):
                a, b = pair[0][0], pair[1][0]
                result['paired_epochs'].append(dict(epoch=epoch, seed=seed,
                    delta_B_minus_A={key: b['metrics'][key] - a['metrics'][key] for key in KEYS}))
    result['completed_epoch_runs'] = sum(run['complete_new_epochs'] for run in result['runs'].values())
    result['planned_epoch_runs'] = 90
    result['new_optimizer_steps'] = sum(run['points'][-1]['new_steps'] for run in result['runs'].values())
    result['new_sample_exposures'] = sum(run['points'][-1]['new_samples_seen'] for run in result['runs'].values())
    if result['completed_epoch_runs'] == 90:
        result['status'] = 'DURATION_CURVES_COMPLETE_NOT_PROMOTED'
    return result


def render(result, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    names = []
    for seed in SEEDS:
        fig, axes = plt.subplots(2, 3, figsize=(15, 8), constrained_layout=True)
        for axis, key in zip(axes.flat, KEYS):
            for arm in ARMS:
                points = result['runs'][f'seed_{seed}/{arm}']['points']
                axis.plot([p['effective_epochs'] for p in points], [p['metrics'][key] for p in points],
                          marker='o', markersize=3, label=arm)
            if key in LIMITS:
                axis.axhline(LIMITS[key], linestyle=':', color='red', label='project target')
            axis.set_title(key); axis.set_xlabel('TRAIN sample exposures / 1147')
            axis.grid(alpha=.3); axis.legend(fontsize=7)
        fig.suptitle(f'Seed {seed}: fixed VAL 337 | OFFLINE ONLY | no model adoption')
        name = f'val_learning_curves_{seed}.png'; fig.savefig(output / name, dpi=140); plt.close(fig); names.append(name)
    fig, axes = plt.subplots(1, 2, figsize=(13, 5), constrained_layout=True)
    for axis, arm in zip(axes, ARMS):
        for seed in SEEDS:
            curve = result['runs'][f'seed_{seed}/{arm}']['training_batch_curve']
            full = [p for p in curve if p['complete_epoch']]
            axis.plot([p['epoch'] for p in full], [p['sample_weighted_batch_loss'] for p in full], label=str(seed))
        axis.set_title(arm); axis.set_xlabel('completed epoch'); axis.set_ylabel('sample-weighted batch loss')
        axis.grid(alpha=.3); axis.legend()
    fig.suptitle('Changing-model TRAIN losses | A/B objectives differ: do not compare total-loss levels')
    name = 'train_batch_learning_curves.png'; fig.savefig(output / name, dpi=140); plt.close(fig); names.append(name)
    return names


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--results', type=Path, required=True)
    parser.add_argument('--parent', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--allow-partial', action='store_true')
    args = parser.parse_args(argv)
    result = summarize(args.results, args.parent, args.allow_partial)
    require(not args.output.exists() and not args.output.is_symlink(), 'new report output required')
    require(not args.output.resolve().is_relative_to(args.results.resolve())
            and not args.output.resolve().is_relative_to(args.parent.resolve()), 'report must be outside source evidence')
    args.output.mkdir(parents=True)
    images = render(result, args.output)
    result['figures'] = {name: sha(args.output / name) for name in images}
    (args.output / 'learning_curves.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(json.dumps({key: result[key] for key in ('status', 'completed_epoch_runs', 'new_optimizer_steps', 'new_sample_exposures')}))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
