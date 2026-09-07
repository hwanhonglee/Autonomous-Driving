#!/usr/bin/env python3
"""HH_260906 - Render measured C/E learning and final validation from strictly verified paired campaigns."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from portable_e2e.contract import ContractError, _loads_json
from scripts.e2e import summarize_portable_candidate_rank as campaign_summary


# HH_260906 - Preserve actual metric names and fixed exposure accounting instead of relabelling partial epochs as complete.
SEEDS = (20260903, 20260904, 20260905)
ARMS = {'baseline': 'C_expanded_data', 'candidate': 'E_candidate_rank'}
TRAIN_METRICS = ('selected_ade_m', 'selected_fde_m', 'regression_loss', 'candidate_score_loss')
VAL_METRICS = ('selected_ade_m', 'selected_fde_m', 'selected_speed_mae_mps', 'selection_regret_ade_m')
STEPS = 1540
BATCHES_PER_EPOCH = 287
FULL_EPOCH_SAMPLES = 1147
TOTAL_EXPOSURES = 6155


def read_bytes(path):
    if path.is_symlink() or not path.is_file():
        raise ContractError('plot input must be a regular non-symlink file')
    return path.read_bytes()


def sha256(payload):
    return hashlib.sha256(payload).hexdigest()


def finite_nonnegative(value, field):
    if isinstance(value, bool) or not isinstance(value, (float, int)):
        raise ContractError(f'{field} must be finite, numeric and nonnegative')
    try:
        number = float(value)
    except (OverflowError, ValueError) as error:
        raise ContractError(f'{field} cannot be represented as a finite plot value') from error
    if not math.isfinite(number) or number < 0:
        raise ContractError(f'{field} must be finite, numeric and nonnegative')
    return number


def training_history(payload):
    rows = [_loads_json(line, 'training history row') for line in payload.decode('utf-8').splitlines()]
    if len(rows) != STEPS:
        raise ContractError('training history must contain all 1540 measured steps')
    seen, groups = 0, []
    for offset, row in enumerate(rows):
        step, epoch = offset + 1, offset // BATCHES_PER_EPOCH
        if type(row.get('global_step')) is not int or row['global_step'] != step:
            raise ContractError('training global_step must be ordered integers 1..1540')
        if type(row.get('epoch')) is not int or row['epoch'] != epoch:
            raise ContractError('training epoch does not match the frozen batch order')
        count = 3 if step % BATCHES_PER_EPOCH == 0 else 4
        batch = row.get('batch_domain_sample_counts')
        if not isinstance(batch, dict) or set(batch) != {'carla'} or type(batch['carla']) is not int or batch['carla'] != count:
            raise ContractError('training batches must contain four CARLA samples except each 287th batch of three')
        seen += count
        cumulative = row.get('domain_samples_seen')
        if (type(row.get('samples_seen')) is not int or row['samples_seen'] != seen
                or not isinstance(cumulative, dict) or set(cumulative) != {'carla'}
                or type(cumulative['carla']) is not int or cumulative['carla'] != seen):
            raise ContractError('training cumulative exposure counters are inconsistent')
        for metric in TRAIN_METRICS:
            finite_nonnegative(row.get(metric), metric)
        if len(groups) <= epoch:
            groups.append([])
        groups[epoch].append(row)
    if seen != TOTAL_EXPOSURES or [len(group) for group in groups] != [287] * 5 + [105]:
        raise ContractError('expected five complete epochs and one 105-batch partial epoch')
    epochs = []
    for index, group in enumerate(groups):
        exposures = sum(row['batch_domain_sample_counts']['carla'] for row in group)
        expected = FULL_EPOCH_SAMPLES if index < 5 else 420
        if exposures != expected:
            raise ContractError('epoch sample exposure total differs from the frozen plan')
        epochs.append({'epoch': index + 1, 'complete': index < 5,
            'label': f'Epoch {index + 1}' if index < 5 else 'Epoch 6 PARTIAL (420/1147 exposures)',
            'first_step': group[0]['global_step'], 'last_step': group[-1]['global_step'],
            'batch_count': len(group), 'sample_exposures': exposures,
            **{metric: math.fsum(row[metric] * (row['batch_domain_sample_counts']['carla'] / exposures)
                for row in group) for metric in TRAIN_METRICS}})
    return rows, epochs


def validate_summary(report):
    if (report.get('schema') != campaign_summary.SCHEMA or report.get('status') != 'COMPLETE_NOT_PROMOTED'
            or report.get('automatic_promotion') is not False or report.get('vehicle_control_approved') is not False
            or report.get('test_opened_by_this_campaign') is not False):
        raise ContractError('rendering requires a completed, unpromoted strict candidate-rank summary')
    if [pair.get('seed') for pair in report.get('pairs', [])] != list(SEEDS):
        raise ContractError('rendering requires exactly three ordered paired seeds')
    for pair in report['pairs']:
        if type(pair['seed']) is not int:
            raise ContractError('paired seed must be an integer')
        for label, arm in ARMS.items():
            run = pair[label]
            if run.get('arm') != arm or run.get('training_dataset_size') != FULL_EPOCH_SAMPLES or run.get('validation_sample_count') != 337:
                raise ContractError('paired arm or train1147/val337 contract differs')
            for metric in VAL_METRICS:
                finite_nonnegative(run['metrics'].get(metric), metric)


def draw_plots(output, report, measurements):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    colors = ('#2468a2', '#bf721a', '#28875b')
    fig, axes = plt.subplots(2, 2, figsize=(16, 11))
    titles = ('Training selected ADE (m)', 'Training selected FDE (m)',
        'Training regression_loss', 'Training candidate_score_loss')
    for index, item in enumerate(measurements):
        color, style = colors[index // 2], '-' if item['input_root'] == 'baseline' else '--'
        label = f'{item["seed"]}: {"C baseline" if item["input_root"] == "baseline" else "E candidate-aware"}'
        for axis, metric, title in zip(axes.flat, TRAIN_METRICS, titles):
            axis.plot([row['global_step'] for row in item['rows']], [row[metric] for row in item['rows']],
                color=color, linestyle=style, linewidth=0.4, alpha=0.10)
            axis.plot([(row['first_step'] + row['last_step']) / 2 for row in item['epochs']],
                [row[metric] for row in item['epochs']], color=color, linestyle=style, marker='o',
                linewidth=2, markersize=4, label=label)
            axis.set(xlabel='Optimizer step (all 1,540 measured batches)', ylabel=title, xlim=(1, STEPS))
    for axis in axes.flat:
        axis.axvspan(1435.5, 1540.5, color='#666666', alpha=0.10)
        for boundary in range(287, 1436, 287):
            axis.axvline(boundary + 0.5, color='#777777', linestyle=':', linewidth=0.7, alpha=0.35)
        axis.grid(alpha=0.20)
    handles, labels = axes.flat[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper center', bbox_to_anchor=(0.5, 0.94), ncol=3, fontsize=10)
    fig.suptitle('C / E measured training | same v3 data and step budget, different scorer architecture', y=0.985, fontsize=15)
    fig.text(0.5, 0.055, 'Faint lines: every measured batch. Strong lines: sample-exposure-weighted epoch means. No intermediate validation.',
        ha='center', fontsize=11)
    fig.text(0.5, 0.025, 'Five complete epochs (1,147 each) + shaded sixth PARTIAL epoch (105 batches / 420 exposures) = 6,155 exposures; not 10 epochs.',
        ha='center', fontsize=10)
    fig.subplots_adjust(left=0.075, right=0.975, top=0.82, bottom=0.15, hspace=0.28, wspace=0.23)
    fig.savefig(output / '01_training_curves.png', dpi=120)
    plt.close(fig)

    fig, axes = plt.subplots(2, 2, figsize=(16, 10))
    for axis, metric, title, limit in zip(axes.flat, VAL_METRICS,
            ('Selected ADE (m)', 'Selected FDE (m)', 'Selected speed MAE (m/s)', 'ADE selection regret (m)'),
            (2.0, 4.0, None, None)):
        for index, pair in enumerate(report['pairs']):
            for offset, label, color, text in ((-0.18, 'baseline', '#2468a2', 'C: baseline scorer'),
                    (0.18, 'candidate', '#bf721a', 'E: candidate-aware scorer')):
                value = pair[label]['metrics'][metric]
                axis.bar(index + offset, value, 0.34, color=color, label=text if index == 0 else None)
                axis.text(index + offset, value, f'{value:.3f}', ha='center', va='bottom', fontsize=10)
        if limit is not None:
            axis.axhline(limit, color='#b6362e', linestyle=':', linewidth=1.5)
            axis.text(0.98, limit, f'Fixed target {limit:g} m', color='#b6362e',
                transform=axis.get_yaxis_transform(), ha='right', va='bottom', fontsize=9)
        peak = max(pair[label]['metrics'][metric] for pair in report['pairs'] for label in ARMS)
        axis.set_ylim(0, max(peak, limit or 0, 0.01) * 1.20)
        axis.set_xticks(range(3), [str(seed) for seed in SEEDS])
        axis.set(ylabel=title, xlabel='Paired seed')
        axis.grid(axis='y', alpha=0.2)
    handles, labels = axes.flat[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper center', bbox_to_anchor=(0.5, 0.94), ncol=2, fontsize=11)
    fig.suptitle('Final checkpoints only | Town03 val337 | three paired seeds', y=0.985, fontsize=16)
    fig.text(0.5, 0.035, f'Relative screen: {report["candidate_screen"]} | Absolute quality: {report["absolute_quality"]} | No test selection or automatic promotion',
        ha='center', fontsize=12)
    fig.subplots_adjust(left=0.075, right=0.975, top=0.85, bottom=0.13, hspace=0.28, wspace=0.23)
    fig.savefig(output / '02_paired_validation.png', dpi=120)
    plt.close(fig)


def render(baseline_root, candidate_root, summary_path, output_dir):
    if output_dir.exists() or output_dir.is_symlink():
        raise ContractError('plot output directory must be new')
    summary_bytes = read_bytes(summary_path)
    renderer_sha256 = sha256(read_bytes(Path(__file__)))
    report = _loads_json(summary_bytes.decode('utf-8'), 'supplied candidate-rank summary')
    fresh = campaign_summary.summarize_campaign(baseline_root, candidate_root)
    if json.dumps(report, sort_keys=True, allow_nan=False) != json.dumps(fresh, sort_keys=True, allow_nan=False):
        raise ContractError('supplied summary differs from freshly verified raw campaigns')
    validate_summary(report)
    roots, inputs, measurements = {'baseline': baseline_root, 'candidate': candidate_root}, [], []
    for pair in report['pairs']:
        for label, arm in ARMS.items():
            relative = f'seed_{pair["seed"]}/{arm}/training/metrics.jsonl'
            payload = read_bytes(roots[label] / relative)
            rows, epochs = training_history(payload)
            inputs.append({'path': f'{label}/{relative}', 'sha256': sha256(payload)})
            measurements.append({'seed': pair['seed'], 'arm': arm, 'input_root': label,
                'optimizer_steps': STEPS, 'sample_exposures': TOTAL_EXPOSURES,
                'complete_epochs': 5, 'partial_epoch_exposures': 420, 'epochs': epochs, 'rows': rows})
    output_dir.mkdir(parents=True, exist_ok=False)
    draw_plots(output_dir, report, measurements)
    # HH_260906 - A manifest is published only if all measured inputs and the strict summary remain unchanged after drawing.
    if (read_bytes(summary_path) != summary_bytes
            or campaign_summary.summarize_campaign(baseline_root, candidate_root) != report
            or sha256(read_bytes(Path(__file__))) != renderer_sha256):
        raise ContractError('summary or raw campaign evidence changed while rendering')
    for item in inputs:
        label, relative = item['path'].split('/', 1)
        if sha256(read_bytes(roots[label] / relative)) != item['sha256']:
            raise ContractError('training history changed while rendering')
    manifest = {'schema': 'portable_e2e.candidate_rank_learning_plots.v1',
        'comment': 'HH_260906 - Plot actual training batches and weighted epochs; final validation only, without fabricated intermediate evaluations.',
        'input_layout': 'paired_campaign_roots_v1', 'summary_sha256': sha256(summary_bytes),
        'renderer_script_sha256': renderer_sha256, 'inputs': inputs,
        'training_metric_names': list(TRAIN_METRICS), 'validation_metric_names': list(VAL_METRICS),
        'training_epoch_metrics': [{key: value for key, value in item.items() if key != 'rows'} for item in measurements],
        'plots': [{'path': name, 'width_px': 1920, 'sha256': sha256(read_bytes(output_dir / name))}
            for name in ('01_training_curves.png', '02_paired_validation.png')],
        'interpretation': 'Different architectures, same data/seed/step budget; training means do not establish validation or driving performance. Sixth epoch is partial.',
        'vehicle_control_approved': False}
    with (output_dir / 'plot_inputs.json').open('x', encoding='utf-8') as stream:
        stream.write(json.dumps(manifest, indent=2, allow_nan=False) + '\n')
    return manifest


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('baseline_root', type=Path)
    parser.add_argument('candidate_root', type=Path)
    parser.add_argument('--summary', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args(argv)
    render(args.baseline_root, args.candidate_root, args.summary, args.output_dir)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
