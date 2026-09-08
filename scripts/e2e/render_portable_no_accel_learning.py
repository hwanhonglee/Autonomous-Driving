#!/usr/bin/env python3
"""HH_260906 - Plot complete measured input-ablation learning and final val337 results without model loading or promotion."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from portable_e2e.contract import ContractError, _loads_json
from scripts.e2e import summarize_portable_no_accel_development as campaign

ROOT = Path(__file__).resolve().parents[2]
ARMS = ('A_physical_input', 'B_no_accel_input')
SEEDS = (20260903, 20260904, 20260905)
TRAIN_METRICS = ('loss', 'selected_ade_m')
VAL_METRICS = ('selected_ade_m', 'selected_fde_m', 'selected_speed_mae_mps')
require = campaign.base._require


def read(path):
    require(path.is_file() and all(not p.is_symlink() for p in (path, *path.parents)), 'nonregular/symlink plot input')
    return path.read_bytes()


def checked_sha(path):
    require(path.is_file() and all(not p.is_symlink() for p in (path, *path.parents)), 'nonregular/symlink hashed input')
    return campaign.sha_file(path)


def finite(value):
    require(type(value) in (int, float) and math.isfinite(value) and value >= 0., 'plot metric must be finite and nonnegative')
    return value


def history(payload):
    # HH_260906 - Count every measured batch, including short epoch boundaries and the final forty-step display bin.
    rows = [_loads_json(line, 'training history') for line in payload.decode().splitlines()]
    require(len(rows) == 1540, 'history requires all 1540 rows')
    seen = 0
    for step, row in enumerate(rows, 1):
        count = 3 if step % 287 == 0 else 4
        seen += count
        for name, expected in (('global_step', step), ('epoch', (step - 1) // 287), ('samples_seen', seen)):
            require(type(row.get(name)) is int and row[name] == expected, 'history order/epoch/exposure counter mismatch')
        require(row.get('batch_domain_sample_counts') == {'carla': count}
            and type(row['batch_domain_sample_counts']['carla']) is int
            and row.get('domain_samples_seen') == {'carla': seen}
            and type(row['domain_samples_seen']['carla']) is int, 'history CARLA batch counters differ')
        for metric in TRAIN_METRICS: finite(row.get(metric))
        require(all(not isinstance(value, float) or math.isfinite(value) for value in row.values()), 'nonfinite history field')
    require(seen == 6155, 'expected 6155 actual sample exposures')
    bins = []
    for start in range(0, 1540, 100):
        group = rows[start:start + 100]; count = sum(r['batch_domain_sample_counts']['carla'] for r in group)
        bins.append({'first_step': start + 1, 'last_step': start + len(group), 'batch_count': len(group),
            'sample_exposures': count, 'partial_display_bin': len(group) < 100,
            **{metric: math.fsum(r[metric] * r['batch_domain_sample_counts']['carla'] for r in group) / count
                for metric in TRAIN_METRICS}})
    return rows, bins


def validate_summary(report):
    require(report.get('schema') == campaign.SCHEMA and report.get('status') == 'COMPLETE_NOT_PROMOTED'
        and report.get('automatic_promotion') is False and report.get('vehicle_control_approved') is False
        and report.get('test_evaluated') is False and report.get('test_used_for_training_or_selection') is False,
        'completed unpromoted train/val-only summary required')
    require([p.get('seed') for p in report.get('pairs', [])] == list(SEEDS), 'three paired seeds required')
    require(len(report.get('stages', [])) == 18 and all(s.get('status') == 'COMPLETE' for s in report['stages']),
        'all eighteen stages must be complete')
    for pair in report['pairs']:
        for label, arm in zip(('baseline', 'candidate'), ARMS):
            run = pair[label]
            require(run['arm'] == arm and run['training_dataset_size'] == 1147 and run['validation_sample_count'] == 337,
                'paired model or split denominator differs')
            for metric in VAL_METRICS: finite(run['metrics'][metric])
            geometry = run['geometry']; count = geometry['selected_pass_count']
            require(type(count) is int and 0 <= count <= 337 and geometry['sample_count'] == 337, 'invalid geometry count')


def draw(output, report, measurements):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    colors = ('#2468a2', '#bf721a', '#28875b')
    fig, axes = plt.subplots(1, 2, figsize=(16, 7))
    for index, item in enumerate(measurements):
        bins = item['bins']; label = f"{item['seed']}: {'A supplied accel' if index % 2 == 0 else 'B no accel'}"
        for axis, metric in zip(axes, TRAIN_METRICS):
            axis.plot([(b['first_step'] + b['last_step']) / 2 for b in bins], [b[metric] for b in bins],
                color=colors[index // 2], linestyle='-' if index % 2 == 0 else '--', marker='.', label=label)
            axis.set(xlabel='Optimizer step', ylabel='Sample-weighted ' + metric, xlim=(1, 1540))
    for axis in axes:
        axis.axvspan(1500.5, 1540.5, color='gray', alpha=.12); axis.grid(alpha=.2)
    fig.legend(*axes[0].get_legend_handles_labels(), loc='upper center', bbox_to_anchor=(.5, .93), ncol=3, fontsize=9)
    fig.suptitle('Measured training | 3 fresh paired seeds | all 1,540 batches', fontsize=15)
    fig.text(.5, .065, '100-step sample-weighted bins; shaded last bin = 40 steps / 160 exposures. No intermediate validation.', ha='center')
    fig.text(.5, .03, 'Each fit: 5 full epochs + 105 batches of epoch 6 = 6,155 exposures, not 10 epochs. Development only.', ha='center')
    fig.subplots_adjust(top=.78, bottom=.20, left=.075, right=.975, wspace=.24)
    fig.savefig(output / '01_training_curves.png', dpi=120); plt.close(fig)
    fig, axes = plt.subplots(2, 2, figsize=(16, 10))
    for axis, metric in zip(axes.flat, (*VAL_METRICS, 'geometry.selected_pass_count')):
        peak = 0.
        for index, pair in enumerate(report['pairs']):
            for offset, label, color in ((-.18, 'baseline', '#2468a2'), (.18, 'candidate', '#bf721a')):
                run = pair[label]; value = run['geometry']['selected_pass_count'] if metric.startswith('geometry') else run['metrics'][metric]
                axis.bar(index + offset, value, .34, color=color, label='A supplied accel' if label == 'baseline' else 'B no accel')
                axis.text(index + offset, value, f'{value}/337' if metric.startswith('geometry') else f'{value:.3f}', ha='center', va='bottom', fontsize=9)
                peak = max(peak, value)
        axis.set_xticks(range(3), [str(s) for s in SEEDS]); axis.set(xlabel='Paired seed', ylabel=metric, ylim=(0, max(.01, peak) * 1.20)); axis.grid(axis='y', alpha=.2)
    handles, labels = axes.flat[0].get_legend_handles_labels()
    fig.legend(handles[:2], labels[:2], loc='upper center', bbox_to_anchor=(.5, .94), ncol=2)
    fig.suptitle('Final checkpoint validation only | Town03 val337 | not learned closed-loop driving', fontsize=15)
    fig.text(.5, .035, f"Relative screen: {report['candidate_screen']} | Absolute quality: {report['absolute_quality']} | No automatic promotion", ha='center')
    fig.subplots_adjust(top=.84, bottom=.13, left=.08, right=.975, hspace=.28, wspace=.23)
    fig.savefig(output / '02_paired_validation.png', dpi=120); plt.close(fig)


def render(root, summary_path, output):
    root, summary_path, output = Path(root).absolute(), Path(summary_path).absolute(), Path(output).absolute()
    require(not output.exists() and all(not p.is_symlink() for p in (output, *output.parents))
        and not any(output.resolve().is_relative_to(p) for p in (root.resolve(), summary_path.parent.resolve(), (ROOT / 'datasets').resolve())),
        'new output must be outside campaign, summary and datasets')
    summary_bytes = read(summary_path); report = _loads_json(summary_bytes.decode(), 'strict summary')
    require(campaign.canonical(report) == campaign.canonical(campaign.summarize_campaign(root)), 'summary differs from freshly verified campaign')
    validate_summary(report)
    pins = {}
    for entry in report['input_manifest']:
        name = entry['path']; relative = Path(name)
        require(not relative.is_absolute() and '..' not in relative.parts and name not in pins, 'unsafe/duplicate input manifest path')
        read_path = root / relative
        require(read_path.is_file() and all(not p.is_symlink() for p in (read_path, *read_path.parents)), 'nonregular manifest input')
        require(checked_sha(read_path) == entry['sha256'], 'summary input SHA mismatch')
        pins[name] = entry['sha256']
    sources = {p.relative_to(ROOT).as_posix(): campaign.sha_file(p) for p in
        (Path(__file__).resolve(), Path(campaign.__file__).resolve(), Path(campaign.base.__file__).resolve(), Path(campaign.expansion.__file__).resolve())}
    measurements = []
    for pair in report['pairs']:
        for label, arm in zip(('baseline', 'candidate'), ARMS):
            prefix = f"seed_{pair['seed']}/{arm}"; name = prefix + '/training/metrics.jsonl'
            payload = read(root / name); rows, bins = history(payload); pins[name] = campaign.base._sha(payload)
            training = _loads_json(read(root / prefix / 'training/run.json').decode(), 'training report')
            evaluation = _loads_json(read(root / prefix / 'evaluation/metrics.json').decode(), 'evaluation report')
            require(campaign.canonical(rows[-1]) == campaign.canonical(training.get('last_metrics')), 'history final row differs from run.json')
            # HH_260906 - Compare plotted raw metrics only; strict comparison summaries also contain derived statistics.
            require(all(evaluation.get('metrics', {}).get(metric) == pair[label]['metrics'][metric]
                for metric in VAL_METRICS), 'summary validation metrics differ from raw evaluation')
            measurements.append({'seed': pair['seed'], 'arm': arm, 'bins': bins, 'optimizer_steps': 1540,
                'sample_exposures': 6155, 'complete_epochs': 5, 'partial_epoch_batches': 105, 'partial_epoch_exposures': 420})
    output.mkdir(parents=True, exist_ok=False); draw(output, report, measurements)
    require(read(summary_path) == summary_bytes and all(checked_sha(root / name) == value for name, value in pins.items()), 'plot inputs changed during rendering')
    require(all(campaign.sha_file(ROOT / name) == value for name, value in sources.items()), 'plotting source changed during rendering')
    proof = {'schema': 'portable_e2e.no_accel_learning_plots.v1', 'summary_sha256': campaign.base._sha(summary_bytes),
        'input_layout': 'single_campaign_root_v1', 'inputs': [{'path': name, 'sha256': value} for name, value in sorted(pins.items())],
        'source_sha256': sources, 'training_bins': measurements, 'training_metric_names': list(TRAIN_METRICS),
        'training_data_approved': False, 'model_loaded': False, 'model_training': False, 'test_evaluated': False,
        'note': 'HH_260906 - Plot measured batches and final val only; all failures and the last forty-step bin are retained.'}
    with (output / 'plot_inputs.json').open('x') as stream: stream.write(json.dumps(proof, indent=2, allow_nan=False) + '\n')
    with (output / 'SHA256SUMS').open('x') as stream:
        for path in sorted(output.iterdir()):
            if path.name != 'SHA256SUMS': stream.write(f'{campaign.sha_file(path)}  {path.name}\n')
    return proof


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('campaign_root', type=Path); parser.add_argument('--summary', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True); args = parser.parse_args(argv)
    render(args.campaign_root, args.summary, args.output_dir)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
