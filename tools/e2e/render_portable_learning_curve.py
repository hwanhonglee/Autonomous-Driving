#!/usr/bin/env python3
"""HH_260906 - Render measured learning curves and paired validation errors without invented intermediate evaluations."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path


def epoch_metrics(path):
    # HH_260906 - Weight shuffled training batches by their actual sample exposures.
    rows = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    if len(rows) != 1540 or [row['global_step'] for row in rows] != list(range(1, 1541)):
        raise ValueError('expected the complete ordered 1540-step training history')
    groups = {}
    for row in rows:
        epoch = row['epoch']
        count = sum(row['batch_domain_sample_counts'].values())
        if type(epoch) is not int or not 0 <= epoch < 10 or type(count) is not int or count not in (1, 4):
            raise ValueError('invalid epoch or batch exposure count')
        values = [row['selected_ade_m'], row['selected_fde_m']]
        if any(isinstance(value, bool) or not math.isfinite(value) or value < 0 for value in values):
            raise ValueError('training error must be finite and nonnegative')
        item = groups.setdefault(epoch, [0, 0.0, 0.0])
        item[0] += count
        item[1] += count * values[0]
        item[2] += count * values[1]
    if sorted(groups) != list(range(10)) or any(item[0] != 613 for item in groups.values()):
        raise ValueError('training history must cover all 613 samples in each of ten epochs')
    return [{'epoch': epoch + 1, 'sample_exposures': groups[epoch][0],
        'selected_ade_m': groups[epoch][1] / groups[epoch][0],
        'selected_fde_m': groups[epoch][2] / groups[epoch][0]} for epoch in range(10)]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('campaign', type=Path)
    parser.add_argument('--summary', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args(argv)
    report = json.loads(args.summary.read_text())
    if report.get('status') != 'COMPLETE_NOT_PROMOTED' or len(report.get('pairs', [])) != 3:
        raise ValueError('only render a completed, independently validated six-run summary')
    measurements, inputs = [], []
    for pair in report['pairs']:
        for name in ('baseline', 'candidate'):
            run = pair[name]
            path = args.campaign / f'seed_{pair["seed"]}' / run['arm'] / 'training/metrics.jsonl'
            curves = epoch_metrics(path)
            measurements.append({'seed': pair['seed'], 'arm': run['arm'], 'epochs': curves})
            inputs.append({'path': str(path.relative_to(args.campaign)),
                'sha256': hashlib.sha256(path.read_bytes()).hexdigest()})
    args.output_dir.mkdir(parents=True, exist_ok=False)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    colors = ['#2468a2', '#df852c', '#28875b']
    fig, axes = plt.subplots(1, 2, figsize=(16, 6))
    for index, row in enumerate(measurements):
        for axis, metric, title in zip(axes, ['selected_ade_m', 'selected_fde_m'],
                ['Training selected ADE (m)', 'Training selected FDE (m)']):
            axis.plot([item['epoch'] for item in row['epochs']],
                [item[metric] for item in row['epochs']], color=colors[index // 2],
                linestyle='-' if index % 2 == 0 else '--',
                label=f'{row["seed"]}: {"A 1e-4" if index % 2 == 0 else "B 3e-5"}')
            axis.set(xlabel='Completed epoch (613 training samples)', ylabel=title)
            axis.grid(alpha=0.25)
    axes[1].legend(fontsize=9)
    fig.suptitle('Measured training curves | not validation or driving performance')
    fig.tight_layout()
    fig.savefig(args.output_dir / '01_training_curves.png', dpi=120)
    plt.close(fig)

    fig, axes = plt.subplots(2, 2, figsize=(16, 10))
    for axis, metric, label, limit in zip(axes.flat,
            ['selected_ade_m', 'selected_fde_m', 'selected_speed_mae_mps', 'selection_regret_ade_m'],
            ['Selected ADE (m)', 'Selected FDE (m)', 'Speed MAE (m/s)', 'Selection regret (m)'],
            [2.0, 4.0, None, None]):
        for index, pair in enumerate(report['pairs']):
            a, b = pair['baseline']['metrics'][metric], pair['candidate']['metrics'][metric]
            axis.bar(index - .18, a, .34, color='#2468a2', label='A: 1e-4' if index == 0 else None)
            axis.bar(index + .18, b, .34, color='#df852c', label='B: 3e-5' if index == 0 else None)
            for position, value in ((index - .18, a), (index + .18, b)):
                axis.text(position, value, f'{value:.3f}', ha='center', va='bottom', fontsize=9)
        if limit is not None:
            axis.axhline(limit, color='#c23d32', linestyle=':', label=f'Absolute target {limit:g} m')
        axis.set_xticks(range(3), [str(pair['seed']) for pair in report['pairs']])
        axis.set_ylabel(label)
        # HH_260906 - Reserve legend headroom so measured value labels remain visible.
        peak = max(pair[name]['metrics'][metric] for pair in report['pairs']
            for name in ('baseline', 'candidate'))
        axis.set_ylim(bottom=0, top=max(peak, limit or 0.0, 0.01) * 1.25)
        axis.grid(axis='y', alpha=.2)
        axis.legend(fontsize=9)
    fig.suptitle('Town03 val337 | paired seeds | no test selection or vehicle-control approval')
    fig.tight_layout()
    fig.savefig(args.output_dir / '02_paired_validation.png', dpi=120)
    plt.close(fig)
    (args.output_dir / 'plot_inputs.json').write_text(json.dumps({
        'comment': 'HH_260906 - Curves contain training measurements; validation is measured only at final checkpoints.',
        'summary_sha256': hashlib.sha256(args.summary.read_bytes()).hexdigest(),
        'inputs': inputs, 'training_epoch_metrics': measurements}, indent=2) + '\n')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
