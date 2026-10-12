#!/usr/bin/env python3
"""HH_260906 - Publish all six verified duration curves and fixed offline galleries without selecting a model."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

from scripts.e2e import summarize_stopmix_duration_learning as curves

VERIFIER_SHA = 'd7e4a6e9a7d2988e462f2df8bc90947be09080cdb07644e6a4a1ebb59bf701e6'
GROUPS = ('unavailable_masks', 'stationary_hold', 'moving_to_stop', 'continuing_motion', 'other_motion')
INDICES = tuple(round(i * 336 / 11) for i in range(12))


def value(number):
    # HH_260906 - Missing subgroup measurements are not zero-error evidence.
    return '미관측' if number is None else f'{number:.4f}'


def decision_count(group, key):
    # HH_260906 - Zero observed cases are not proof of zero stopping mistakes.
    total, count = group['sample_count'], group.get(key)
    curves.require(type(total) is int and total >= 0, 'behavior decision count exceeds its observed denominator')
    if total == 0:
        curves.require(count is None or (type(count) is int and count == 0),
                       'behavior decision count exceeds its observed denominator')
        return '미관측'
    curves.require(type(count) is int and 0 <= count <= total,
                   'behavior decision count exceeds its observed denominator')
    return f'{count}/{total}'


def decision_row(label, split):
    # HH_260906 - Keep candidate-family selection distinct from actual terminal-zero predictions.
    continuing = split['target_motion_groups']['continuing_motion']
    hold = split['target_motion_groups']['stationary_hold']
    cells = [decision_count(continuing, key) for key in
             ('selected_terminal_exact_zero_count', 'selected_terminal_at_or_below_0p1_count', 'stop_selection_count')]
    cells += [decision_count(split['all_samples'], 'selected_reacceleration_after_exact_future_zero_count'),
              decision_count(hold, 'selected_terminal_exact_zero_count')]
    return f'| {label} | ' + ' | '.join(cells) + ' |'


def metric_table(result):
    lines = ['| seed / 손실 | 부모 ADE 6.4s | epoch 10 | epoch 20 | 부모 대비 변화 | 최종 ADE 1s / 3s | 최종 FDE 6.4s |',
             '| --- | ---: | ---: | ---: | ---: | ---: | ---: |']
    for run in curves.RUNS:
        points = result['runs'][run]['points']
        before = points[0]['metrics']
        middle = next(p['metrics'] for p in points if p['epoch'] == 10)
        after = next(p['metrics'] for p in points if p['epoch'] == 20)
        lines.append(f"| {run} | {value(before['ade_6p4s_m'])} | {value(middle['ade_6p4s_m'])} | "
                     f"{value(after['ade_6p4s_m'])} | {after['ade_6p4s_m']-before['ade_6p4s_m']:+.4f} | "
                     f"{value(after['ade_1p0s_m'])} / {value(after['ade_3p0s_m'])} | {value(after['fde_6p4s_m'])} |")
    return '\n'.join(lines)


def inventory_pin(source, results, inventory):
    # HH_260906 - Freeze publication to the independently verified remote receipt, not newly observed local bytes.
    key = source.relative_to(results).as_posix()
    pin = inventory['remote_file_sha256'].get(key)
    curves.require(pin is not None and curves.sha(source) == pin, f'artifact differs from verified remote inventory: {key}')
    return pin


def publish(results, analysis, verification, output, remote_inventory=None):
    results, analysis, verification, output = map(Path, (results, analysis, verification, output))
    require = curves.require
    require(not output.exists() and not output.is_symlink(), 'fresh publication directory required')
    for source in (results, analysis, verification):
        require(not output.resolve().is_relative_to(source.resolve())
                and not source.resolve().is_relative_to(output.resolve()), 'publication must be disjoint')
    verified = curves.read(verification)
    require(verified['status'] == 'PASS_NOT_PROMOTED' and verified['campaign_completion_verified'] is True
            and verified['verifier_sha256'] == VERIFIER_SHA, 'final independent verification required')
    require(remote_inventory is not None, 'verified remote inventory required')
    remote_inventory = Path(remote_inventory)
    require(curves.sha(remote_inventory) == verified['remote_inventory_sha256'], 'verified remote inventory changed')
    inventory = curves.read(remote_inventory)
    result = curves.read(analysis / 'learning_curves.json')
    require(result['status'] == 'DURATION_CURVES_COMPLETE_NOT_PROMOTED'
            and result['completed_epoch_runs'] == 90 and result['new_optimizer_steps'] == 25200
            and result['new_sample_exposures'] == 100710, 'complete duration analysis required')
    require(set(result['runs']) == set(curves.RUNS)
            and all([p['epoch'] for p in result['runs'][run]['points']] == [None, *range(6, 21)] for run in curves.RUNS),
            'exact six lineages and ordered epoch endpoints required')
    pending = {}
    source_pins = {verification: curves.sha(verification), remote_inventory: curves.sha(remote_inventory),
                   analysis / 'learning_curves.json': curves.sha(analysis / 'learning_curves.json')}

    def include(source, destination, expected=None):
        require(source.is_file() and not source.is_symlink(), 'regular source artifact required')
        data = source.read_bytes()
        digest = hashlib.sha256(data).hexdigest()
        require(expected is None or digest == expected, f'publication source changed: {source}')
        require(destination not in pending, 'duplicate destination')
        source_pins[source] = digest
        pending[destination] = data

    include(verification, 'verification.json')
    include(analysis / 'learning_curves.json', 'learning_curves.json')
    for run in curves.RUNS:
        point = result['runs'][run]['points'][0]
        source = results / 'parents' / run / 'evaluation/metrics.json'
        source_pins[source] = inventory_pin(source, results, inventory)
        require(point['metrics'] == {key: curves.read(source)['metrics'][key] for key in curves.KEYS},
                'parent curve metrics differ from verified archive')
    require(set(result['figures']) == {f'val_learning_curves_{s}.png' for s in curves.SEEDS} | {'train_batch_learning_curves.png'},
            'four complete curve figures required')
    for name, pin in result['figures'].items():
        include(analysis / name, name, pin)
    groups = ['# 행동별 오프라인 지표 — 모든 seed / A·B / epoch 10·20', '',
              '<!-- HH_260906 - Retain empty groups and regressions; TRAIN is fitted evidence, not independent validation. -->', '',
              '단위: 경로 m, 속도 m/s. 미관측은 성공이나 오차 0이 아니다. TRAIN은 학습에 사용한 자료다.', '',
              '| split / epoch / seed / 손실 | 정답 행동 | 표본 수 | 선택 ADE | 선택 FDE | 속도 MAE | 최선 후보 ADE | 선택 손실(ADE regret) |',
              '| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |']
    decisions = ['# 정지·재가속 선택 진단 — 실제 차량 동작이 아닌 오프라인 예측', '',
                 '<!-- HH_260906 - Expose stopping regressions with exact denominators rather than mean error alone. -->', '',
                 '계속 주행 정답 그룹과 정지 유지 정답 그룹을 구별한다. N=0은 미관측이며 통과가 아니다.', '',
                 '| split / epoch / seed / 손실 | 계속 주행 정답: 종점 속도 정확히 0 | 계속 주행 정답: 종점 ≤0.1m/s | 계속 주행 정답: STOP 후보 선택 | 전체: 미래 속도 0 뒤 재가속 | 정지 유지 정답: 종점 속도 정확히 0 |',
                 '| --- | ---: | ---: | ---: | ---: | ---: |']
    behavior = {}
    for epoch in range(6, 21):
        for run in curves.RUNS:
            item = results / 'epochs' / f'epoch_{epoch:03d}' / run
            point = next(p for p in result['runs'][run]['points'] if p['epoch'] == epoch)
            require(point['checkpoint_sha256'] == verified['endpoint_checkpoint_sha256'][f'{epoch}/{run}'],
                    'curve checkpoint differs from verified lineage')
            require(set(point['report_sha256']) == {'training/run.json', 'training/metrics.jsonl', 'evaluation/metrics.json', 'gate_v8.json'},
                    'all four verified endpoint report bindings required')
            for name, pin in point['report_sha256'].items():
                path = item / name
                require(inventory_pin(path, results, inventory) == pin, 'curve report no longer matches evidence')
                source_pins[path] = pin
            evaluation = curves.read(item / 'evaluation/metrics.json')
            require(point['metrics'] == {key: evaluation['metrics'][key] for key in curves.KEYS},
                    'published curve metrics differ from bound evaluation')
            require(point['absolute_checks'] == {key: point['metrics'][key] <= limit for key, limit in curves.LIMITS.items()},
                    'published absolute verdict differs from unchanged limits')
            if epoch not in (10, 20):
                continue
            path = item / 'behavior/summary.json'
            data = curves.read(path)
            require(data['checkpoint_sha256'] == point['checkpoint_sha256'], 'behavior checkpoint differs')
            require([r['index'] for r in data['renders']] == list(INDICES), 'fixed gallery indices differ')
            include(path, f'behavior/epoch_{epoch}/{run}/summary.json', inventory_pin(path, results, inventory))
            behavior[(epoch, run)] = data
            for split in ('train', 'val'):
                decisions.append(decision_row(f'{split} / {epoch} / {run}', data['splits'][split]))
                for group in GROUPS:
                    group_data = data['splits'][split]['target_motion_groups'][group]
                    metrics = group_data['metrics'] or {}
                    numbers = [metrics.get(k) for k in ('selected_ade_m', 'selected_fde_m', 'selected_speed_mae_mps', 'oracle_ade_m', 'ade_selection_regret_m')]
                    groups.append(f"| {split} / {epoch} / {run} | {group} | {group_data['sample_count']} | " + ' | '.join(map(value, numbers)) + ' |')
            for render in data['renders']:
                require(render['path'] == f"trajectories/val_{render['index']:03d}.png", 'unsafe render path')
                image = item / 'behavior' / render['path']
                require(inventory_pin(image, results, inventory) == render['sha256'], 'render inventory binding differs')
                include(image, f"behavior/epoch_{epoch}/{run}/{render['path']}", render['sha256'])
    pending['behavior_metrics.md'] = ('\n'.join(groups) + '\n').encode()
    decisions += ['', '각 셀은 해당 조건의 예측 수 / 관측 정답 표본 수다. STOP은 후보 계열 이름이며, DRIVE 후보도 종점 속도가 0일 수 있다.',
                  '미래 속도 0 뒤 재가속은 선택한 예측의 특성이다. 실제 차량 정지·충돌·재출발 사건을 측정한 것이 아니다.',
                  'TRAIN 정지 유지 성공을 독립 VAL 정지 성능으로 대체하지 않는다. 기존 전체 물리 검사와 오차 표도 함께 확인한다.']
    pending['behavior_decisions.md'] = ('\n'.join(decisions) + '\n').encode()
    for seed in curves.SEEDS:
        lines = [f'# Seed {seed} — 같은 VAL 장면의 A/B와 학습량 비교', '',
                 '<!-- HH_260906 - Publish every preregistered index; these PNGs are offline predictions, not vehicle-control footage. -->', '',
                 '12개 고정 VAL 위치 전체. 실제 CARLA/Autoware 제어 화면이 아닌 저장된 정답·후보·선택 경로의 오프라인 그림이다.', '']
        for index in INDICES:
            lines += [f'## VAL 표본 {index}', '', '| 학습량 | A: 기존 손실 | B: 비용 인지 손실 |', '| --- | --- | --- |']
            for epoch in (10, 20):
                images = [f'![epoch {epoch}, {arm}, VAL {index}](../behavior/epoch_{epoch}/seed_{seed}/{arm}/trajectories/val_{index:03d}.png)' for arm in curves.ARMS]
                lines.append(f'| epoch {epoch} | ' + ' | '.join(images) + ' |')
            lines.append('')
        pending[f'galleries/seed_{seed}.md'] = ('\n'.join(lines) + '\n').encode()
    final = [result['runs'][run]['points'][-1] for run in curves.RUNS]
    absolute_count = sum(all(point['absolute_checks'].values()) for point in final)
    lines = ['# 20 epoch 실제 이어학습 결과 — 6개 모델 전체 비교', '',
             '<!-- HH_260906 - Separate completed optimization from model quality, function acceptance, and deployment. -->', '',
             '실행·증거 검산 완료. **모델 채택·차량 제어 승인 아님.**', '',
             '- 새 optimizer 갱신 25,200회, 새 표본 노출 100,710회. 모델마다 누적 20 epoch / 5,740 step.',
             '- 90개 학습 종점·282개 단계 완료. TRAIN 3 episode / 1,147표본, 개발 VAL 1 episode / 337표본 그대로.',
             '- 새로 편입한 데이터 0 episode. 반복 노출을 독립 장면 수로 계산하지 않는다.',
             f'- 최종 네 가지 경로 오차 목표를 모두 만족한 모델: {absolute_count}/6. 기능 통과나 안전성 보장이 아니다.',
             '- 모델 직접 주행 통과 0/30, TEST 미사용, 실차 미승인. 아래 그림은 오프라인 예측/분석이다.', '',
             '## 부모 → 10 → 20 epoch', '', '경로 오차 단위 m. 변화가 음수면 개선, 양수면 악화. 모든 seed를 유지한다.', '',
             metric_table(result), '',
             '목표: ADE 1s ≤0.5m, 3s ≤1m, 6.4s ≤2m; FDE 6.4s ≤4m. 반복 확인한 VAL은 개발용이다.', '',
             '## 전체 학습 곡선', '',
             *[f'![Seed {seed} fixed VAL curves](val_learning_curves_{seed}.png)\n' for seed in curves.SEEDS],
             '![Changing-model TRAIN batch losses](train_batch_learning_curves.png)', '',
             'TRAIN 곡선은 가중치가 변하는 batch 손실이다. A/B 목적함수가 달라 총 손실 높이를 직접 비교하지 않는다.', '',
             '## 행동별 분석과 경로 그림', '',
             '[모든 행동별 오차](behavior_metrics.md) · [정지·재가속 선택 진단](behavior_decisions.md). VAL 정지 유지 표본은 0개이며 정지 성능을 검증하지 못한다.', '',
             *[f'- [Seed {seed}: 고정 12장면 × A/B × epoch 10/20](galleries/seed_{seed}.md)' for seed in curves.SEEDS], '',
             '[전체 곡선 원본](learning_curves.json) · [독립 실행 검산](verification.json).', '',
             '이 결과만으로 모든 맵 일반화·VAD/METEOR 동급 성능·실제 차량 사용 가능성을 주장하지 않는다.']
    pending['README.md'] = ('\n'.join(lines) + '\n').encode()
    require(all(curves.sha(path) == pin for path, pin in source_pins.items()), 'source changed during publication')
    output.mkdir(parents=True)
    for name, data in pending.items():
        path = output / name
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open('xb') as stream:
            stream.write(data)
    manifest = {name: hashlib.sha256(data).hexdigest() for name, data in sorted(pending.items())}
    with (output / 'SHA256SUMS').open('x') as stream:
        stream.writelines(f'{pin}  {name}\n' for name, pin in manifest.items())
    return dict(status='PUBLISHED_NOT_PROMOTED', file_count=len(pending), behavior_png_count=144,
                final_all_four_error_targets_met=absolute_count)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    for name in ('results', 'analysis', 'verification', 'output', 'remote-inventory'):
        parser.add_argument('--' + name, type=Path, required=True)
    print(json.dumps(publish(**vars(parser.parse_args(argv)))))


if __name__ == '__main__':
    main()
