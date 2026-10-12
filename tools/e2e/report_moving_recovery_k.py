#!/usr/bin/env python3
"""HH_260906 - Verify completed K outputs and publish a non-promoting comparison report."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from scripts.e2e import run_moving_multiroute_recovery_k as runner


def digest(path):
    """HH_260906 - Hash checkpoint bytes without deserializing model payloads."""
    result = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            result.update(block)
    return result.hexdigest()


def read(path):
    """HH_260906 - Read only explicit completed-run metadata."""
    return json.loads(path.read_text())


def require(condition, message):
    """HH_260906 - Refuse publication if evidence is incomplete or inconsistent."""
    if not condition:
        raise ValueError(message)


def verify(root):
    """HH_260906 - Independently recompute strict and relative gates after both fits finish."""
    root = root.resolve()
    require(not (root / 'failed.json').exists(), 'failed run requires separate diagnosis')
    summary, plan, decision = [read(root / name) for name in ('summary.json', 'plan.json', 'decision.json')]
    require(summary['status'] == 'COMPLETE_NOT_PROMOTED', 'run is not complete')
    require(digest(root / 'plan.json') == summary['plan_sha256'], 'plan digest differs')
    require(digest(root / 'decision.json') == summary['decision_sha256'], 'decision digest differs')
    for name, expected in plan['source_sha256'].items():
        require(digest(root / 'source' / name) == expected, 'archived source mismatch: ' + name)
        require(digest(ROOT / name) == expected, 'local verification source differs: ' + name)
    evaluations, gates, histories = {}, {}, {}
    for arm in runner.ARMS:
        train = summary['training'][arm]
        require(train['global_step'] == 15000 and train['samples_seen'] == 60000, 'training budget differs')
        require(train['dev_evaluation_count_during_fit'] == 0, 'intermediate DEV observed')
        require(digest(root / train['checkpoint']) == train['checkpoint_sha256'], 'checkpoint mismatch')
        require(train == read(root / arm / 'training/run.json'), 'training report differs')
        require(train['initial_model_state_sha256'] == plan['model']['seeded_initial_model_state_sha256'],
                'paired initialization differs')
        ref = summary['evaluations'][arm]
        require(digest(root / ref['path']) == ref['sha256'], 'evaluation digest differs')
        evaluation = read(root / ref['path'])
        require(evaluation['checkpoint_sha256'] == train['checkpoint_sha256'], 'evaluated checkpoint differs')
        require(evaluation['sample_count'] == 665 and evaluation['evaluation_pass_index'] == 1, 'evaluation scope differs')
        evaluations[arm] = evaluation
        gates[arm] = runner.strict_gate(evaluation)
        require(gates[arm] == decision['strict_gates'][arm], 'strict decision mismatch')
        history = [json.loads(line) for line in (root / arm / 'training/epoch_history.jsonl').read_text().splitlines()]
        require(len(history) == 20, 'epoch history incomplete')
        for index, row in enumerate(history):
            require(row['completed_steps'] == (index + 1) * 750 and row['dev_observed'] is False,
                    'epoch accounting differs')
        histories[arm] = history
    relative = runner.paired_nonregression(*(evaluations[arm] for arm in runner.ARMS))
    require(relative == decision['K1_vs_K0_nonregression'], 'relative decision mismatch')
    passed = gates[runner.ARMS[1]]['pass'] and gates[runner.ARMS[0]]['geometry_pass_count'] == 3 and relative['pass']
    require(decision['pass'] == passed and decision['promotion_eligible'] is False, 'promotion/decision differs')
    return dict(status='VERIFIED_COMPLETED_RESEARCH_RUN_NOT_PROMOTED', decision=decision,
                training=summary['training'], evaluations=evaluations, histories=histories,
                summary_sha256=digest(root / 'summary.json'))


def publish(root, output):
    """HH_260906 - Create a human-readable result without changing training evidence."""
    report = verify(root)
    require(not output.exists(), 'create-only report directory required')
    output.mkdir(parents=True)
    (output / 'verification.json').write_text(json.dumps(report, indent=2, sort_keys=True) + '\n')
    lines = ['# K0/K1 전체 모델 학습·최종 평가', '',
             '<!-- HH_260906 - Keep training completion distinct from driving approval. -->', '',
             '상태: `' + report['decision']['status'] + '`', '',
             '두 모델 각각 15,000 step 전체 학습 후 같은 DEV 665개로 평가했다. TEST·실차·폐루프 제어는 수행하지 않았다.', '',
             '| 범위 | 지표 | K0 | K1 | K1−K0 |', '| --- | --- | ---: | ---: | ---: |']
    for row in report['decision']['K1_vs_K0_nonregression']['checks']:
        lines.append(f"| {row['group']} | {row['metric']} | {row['K0']:.6f} | {row['K1']:.6f} | {row['delta_K1_minus_K0']:+.6f} |")
    lines += ['', '낮을수록 좋다. ADE/FDE는 m, 속도 MAE는 m/s다.', '',
              '| 모델 | 고정 지표 통과 | 형상 기준 통과 | 학습 시간(분) | 첫/마지막 epoch TRAIN loss |',
              '| --- | ---: | ---: | ---: | ---: |']
    for arm in runner.ARMS:
        gate = report['decision']['strict_gates'][arm]
        history = report['histories'][arm]
        lines.append(f"| {arm} | {gate['metric_pass_count']}/9 | {gate['geometry_pass_count']}/3 | {report['training'][arm]['wall_seconds']/60:.1f} | {history[0]['mean_training_loss']:.4f} / {history[-1]['mean_training_loss']:.4f} |")
    lines += ['', 'K1의 K0 대비 비퇴행: ' + str(report['decision']['K1_vs_K0_nonregression']['pass_count']) + '/9.', '',
              '이는 반복 관측한 DEV의 연구 결과다. 통과하더라도 새로운 경로·seed·정지 보존·폐루프 검증 전에는 배포하지 않는다.', '',
              '증거 검증: 소스·계획·체크포인트·평가 SHA, 두 모델 동일 초기 가중치, 20개 epoch/각 15,000 step, 독립 기준 재계산.', '']
    (output / 'README.md').write_text('\n'.join(lines))
    plot_results(report, output)
    return {'status': report['status'], 'decision': report['decision']['status'], 'output': str(output)}


def plot_results(report, output, labels=None, training_title=None):
    """HH_260906 - Plot measured losses and final DEV errors, never simulated driving imagery."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    figure, axis = plt.subplots(figsize=(10, 5))
    for arm, label in zip(runner.ARMS, labels or ('K0: current data', 'K1: expanded data')):
        history = report['histories'][arm]
        axis.plot([row['completed_steps'] for row in history],
                  [row['mean_training_loss'] for row in history], marker='o', label=label)
    axis.set(xlabel='Optimizer step', ylabel='Mean epoch TRAIN loss',
             title=training_title or 'Full-model training — different TRAIN distributions; not a driving score')
    axis.grid(alpha=0.2)
    axis.legend()
    figure.tight_layout()
    figure.savefig(output / 'training_loss.png', dpi=150)
    plt.close(figure)
    groups = ('town03_right_turn_dev', 'town03_straight_dev', 'combined_val')
    metrics = ('ade_6p4s_m', 'fde_6p4s_m', 'speed_mae_6p4s_mps')
    figure, axes = plt.subplots(1, 3, figsize=(14, 4.5))
    for axis, metric, title in zip(axes, metrics, ('ADE (m)', 'FDE (m)', 'Speed MAE (m/s)')):
        for offset, arm, label in zip((-0.18, 0.18), runner.ARMS, labels or ('K0', 'K1')):
            values = [report['evaluations'][arm]['subgroups'][group]['metrics'][metric] for group in groups]
            bars = axis.bar([index + offset for index in range(3)], values, width=0.36, label=label)
            axis.bar_label(bars, fmt='%.3f', fontsize=8, padding=3)
        axis.set_xticks(range(3), ['Right (337)', 'Straight (328)', 'Combined (665)'])
        axis.set_title(title)
        axis.margins(y=0.2)
        axis.legend()
    figure.suptitle('Same final DEV set — lower is better; open-loop research, not closed-loop approval')
    figure.tight_layout()
    figure.savefig(output / 'final_dev_comparison.png', dpi=150)
    plt.close(figure)
    with (output / 'README.md').open('a') as stream:
        stream.write('\n![Measured training losses](training_loss.png)\n\n![Final DEV metrics](final_dev_comparison.png)\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(publish(args.run_root, args.output)))
