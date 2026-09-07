#!/usr/bin/env python3
"""HH_260906 - Publish strictly verified frozen-scorer evidence with measured curves and portable metadata views."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from portable_e2e.contract import ContractError, _loads_json
from scripts.e2e import summarize_portable_frozen_selector as strict
from scripts.e2e.curate_portable_learning_campaign import (
    copy_file, publish_file, read_json, redact_metadata, sha,
)


ARMS = tuple(strict.runner.ARMS)
TRAIN_METRICS = ('candidate_score_loss', 'gradient_norm')
VAL_METRICS = ('selected_ade_m', 'selected_fde_m', 'selected_speed_mae_mps', 'selection_regret_ade_m')
PLOTS = ('01_measured_scorer_training.png', '02_same_cache_validation.png')
OMITTED_HEAD_FIELDS = ('history', 'train_sample_ids', 'val_sample_ids')


def write_new_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x', encoding='utf-8') as stream:
        stream.write(json.dumps(value, indent=2, allow_nan=False) + '\n')


def measurements(campaign, report):
    result = []
    for seed in report['seeds']:
        for arm in ARMS:
            relative = f'seed_{seed["seed"]}/{arm}'
            head = read_json(campaign / relative / 'report.json')
            payload = (campaign / relative / 'metrics.jsonl').read_bytes()
            strict.validate_history(payload, head)
            rows = [_loads_json(line, 'scorer training row') for line in payload.decode().splitlines()]
            epochs = []
            for index in range(6):
                group = [row for row in rows if row['epoch'] == index]
                n = sum(row['batch_samples'] for row in group)
                epochs.append({'epoch': index + 1, 'complete': index < 5, 'sample_exposures': n,
                    'first_step': group[0]['global_step'], 'last_step': group[-1]['global_step'],
                    **{name: math.fsum(row[name] * row['batch_samples'] / n for row in group) for name in TRAIN_METRICS}})
            result.append({'seed': seed['seed'], 'arm': arm, 'path': f'campaign/{relative}/metrics.jsonl',
                'sha256': strict.digest(payload), 'epochs': epochs, 'rows': rows})
    return result


def draw_plots(output, report, measured):
    # HH_260906 - Render every recorded batch and weighted epoch; no intermediate validation points were collected.
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    colors = {'linear_continue': '#2468a2', 'linear_reset': '#c2781b', 'candidate_reset': '#25815b'}
    fig, axes = plt.subplots(3, 2, figsize=(16, 12))
    for row_index, seed in enumerate(report['seeds']):
        for item in [item for item in measured if item['seed'] == seed['seed']]:
            for axis, metric, title in zip(axes[row_index], TRAIN_METRICS,
                    ('candidate_score_loss (cross entropy)', 'gradient_norm (before clipping)')):
                axis.plot([row['global_step'] for row in item['rows']], [row[metric] for row in item['rows']],
                    color=colors[item['arm']], alpha=0.13, linewidth=0.5)
                axis.plot([(row['first_step'] + row['last_step']) / 2 for row in item['epochs']],
                    [row[metric] for row in item['epochs']], color=colors[item['arm']],
                    linewidth=1.8, marker='o', markersize=4, label=item['arm'])
                axis.set(title=f'Seed {seed["seed"]} | {title}', xlabel='Optimizer step', xlim=(1, 1540))
        for axis in axes[row_index]:
            axis.axvspan(1435.5, 1540.5, color='#777777', alpha=0.10)
            for boundary in range(287, 1436, 287):
                axis.axvline(boundary + 0.5, color='#777777', alpha=0.3, linewidth=0.6, linestyle=':')
            axis.grid(alpha=0.18)
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper center', bbox_to_anchor=(0.5, 0.947), ncol=3, fontsize=11)
    fig.suptitle('Frozen C generators | 9 scorer-only fits | actual training measurements', fontsize=17, y=0.988)
    fig.text(0.5, 0.05, 'Faint: all 1,540 measured batches. Strong: sample-exposure-weighted epoch means. Optimization loss = 0.1 x score loss.',
        ha='center', fontsize=10)
    fig.text(0.5, 0.027, 'Five full epochs (1,147 each) + shaded PARTIAL epoch (420 exposures) = 6,155 exposures. No intermediate validation.',
        ha='center', fontsize=10)
    fig.subplots_adjust(left=0.075, right=0.975, top=0.87, bottom=0.12, hspace=0.43, wspace=0.25)
    fig.savefig(output / PLOTS[0], dpi=120)
    plt.close(fig)

    fig, axes = plt.subplots(2, 2, figsize=(16, 10))
    palette = {'original_c_same_cache': '#555f70', **colors}
    for axis, metric, title in zip(axes.flat, VAL_METRICS,
            ('Selected ADE (m)', 'Selected FDE (m)', 'Selected speed MAE (m/s)', 'ADE selection regret (m)')):
        peak = 0.0
        for group_index, seed in enumerate(report['seeds']):
            for arm_index, item in enumerate([seed['baseline'], *seed['arms']]):
                value = item['metrics'][metric]
                peak = max(peak, value)
                x = group_index + (arm_index - 1.5) * 0.2
                axis.bar(x, value, width=0.19, color=palette[item['arm']],
                    label=item['arm'] if group_index == 0 else None)
                axis.text(x, value, f'{value:.2f}', ha='center', va='bottom', fontsize=9)
        axis.set_ylim(0, max(peak, 0.01) * 1.20)
        axis.set_xticks(range(3), [str(seed['seed']) for seed in report['seeds']])
        axis.set(ylabel=title, xlabel='Paired C parent seed')
        axis.grid(axis='y', alpha=0.2)
    handles, labels = axes.flat[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper center', bbox_to_anchor=(0.5, 0.947), ncol=4, fontsize=10)
    fig.suptitle('Town03 val337 | original same-cache C logits versus final scorer heads', fontsize=17, y=0.988)
    screens = ' | '.join(f'{arm}: {value["candidate_screen"]}' for arm, value in report['arm_screens'].items())
    fig.text(0.5, 0.060, 'All-three-seed relative screens: ' + screens, ha='center', fontsize=10)
    fig.text(0.5, 0.031, 'Unchanged candidate XY/speed; original oracle unchanged. No automatic promotion, held-out test inference or learned closed-loop driving.',
        ha='center', fontsize=10)
    fig.subplots_adjust(left=0.075, right=0.975, top=0.84, bottom=0.15, hspace=0.30, wspace=0.23)
    fig.savefig(output / PLOTS[1], dpi=120)
    plt.close(fig)


def render_readme(report, state):
    finished = datetime.fromisoformat(state['completed_at_utc']).astimezone(timezone.utc).isoformat()
    screens = ' / '.join(f'`{arm}` 상대 **{value["candidate_screen"]}**, 절대 **{value["absolute_quality"]}**'
        for arm, value in report['arm_screens'].items())
    lines = ['# 생성기를 고정한 선택기 9회 학습 — 실제 결과', '',
        '<!-- HH_260906 - Preserve failed outcomes, unchanged frozen candidates and original evidence hashes in public views. -->', '',
        '> 공개 JSON은 계정별 경로를 치환한 metadata view입니다. 원본 SHA를 보존했지만 독립 재실행에는 private 원본이 필요합니다.', '',
        f'실제 완료: `{finished}`. 원격 기존 개인 py312 venv의 **GPU0만** 사용했습니다. 로컬에서 검증·그래프·이 자료를 생성했습니다.',
        '전체 모델 재학습이 아니라 **고정 C 생성기 3개 × 선택기 3종 = head 전용 학습 9회**입니다.',
        '3-seed 종합 판정: ' + screens + '. 운용 checkpoint는 교체하지 않았습니다.',
        'learned closed-loop 주행, 실시간 10 Hz 성능, 장애물 회피·차선변경 또는 실차 안전을 증명하지 않습니다.', '',
        '## 먼저 볼 그림', '',
        '![실제 선택기 학습 기록](visuals/01_measured_scorer_training.png)', '',
        '![원래 C와 최종 선택기의 동일 캐시 비교](visuals/02_same_cache_validation.png)', '',
        '학습 그래프의 옅은 선은 1,540개 실제 batch, 진한 선은 노출 표본 수로 가중한 epoch 평균입니다.',
        'batch4 중 매 287번째는 3개입니다. 5개 full epoch + 420개 노출의 partial epoch = **6,155 exposures**이며 6개 full epoch가 아닙니다.',
        '분류 loss와 clipping 전 gradient norm만 그렸습니다. 최적화 loss는 분류 loss의 0.1배이며 중간 val 측정점은 만들지 않았습니다.', '',
        '## 모든 seed 결과', '']
    source_table = strict.render_markdown(report).splitlines()
    lines += [line for line in source_table if line.startswith('|')]
    lines += ['', '각 방식은 모든 seed에서 ADE/FDE 개선, selected geometry 비악화, 속도 MAE 증가 5% 이하를 동시에 만족해야 합니다.',
        '별도 절대 기준은 1/3/6.4초 ADE ≤ 0.5/1/2 m, 6.4초 FDE ≤ 4 m입니다. 좋은 seed만 골라 성공으로 표시하지 않았습니다.',
        '깊이·용량·입력·초기화가 함께 달라졌으므로 후보 XY 입력만의 인과 효과로 단정할 수 없습니다.', '',
        '## 원본 경로 그림 72장', '',
        '아래 링크마다 val337의 고정 index **0, 67, 134, 201, 268, 336** PNG 6장이 있습니다.',
        '좌표는 ego 중심이며 모든 후보와 정답 궤적을 보여주는 **오프라인 예측**입니다. CARLA 주행 녹화나 Autoware 실시간 화면이 아닙니다.', '',
        '| Seed | 원래 C | Linear 이어 학습 | Linear 새 초기화 | 후보 인지 MLP |', '|---|---|---|---|---|']
    for seed in report['seeds']:
        root = f'seed_{seed["seed"]}'
        links = [f'[{name}](routes/{root}/{name}/README.md)' for name in ('original_c_same_cache', *ARMS)]
        lines.append(f'| {seed["seed"]} | ' + ' | '.join(links) + ' |')
    lines += ['', '## 데이터·검증·재현 범위', '',
        '- Train 1,147 / 개발용 val 337을 분리했습니다. Test는 전체 corpus 무결성 검사 대상일 수 있지만 모델 예측·학습·선택·성능 분석에 사용하지 않았습니다.',
        '- 원본 cache·생성기·선택 head의 전후 hash와 후보/oracle 불변성을 검증했습니다. 요약기는 tensor를 재로드하지 않고 bytes hash와 고정 소스 worker의 증명을 확인합니다.',
        f'- [summary.json](summary.json): 엄격한 {len(report["input_manifest"])}개 원본 입력의 SHA와 모든 판정. [plot_inputs.json](visuals/plot_inputs.json): 실제 학습 기록 9개·요약·그래프·현재 발행 스크립트의 SHA.',
        '- [execution_record.json](provenance/execution_record.json): 원격 실행 commit/runner SHA와 별도의 로컬 발행 코드 SHA. 발행 시 미커밋 코드의 SHA를 실행 commit으로 가장하지 않습니다.',
        '- [publication_manifest.json](publication_manifest.json): 원본 SHA / 공개 파일 SHA와 변환 종류. [SHA256SUMS](SHA256SUMS): 이 폴더의 모든 다른 파일 검증.',
        '- 원격 raw runner·checkpoint·캐시·logit tensor는 공개하지 않았습니다. PNG 72장과 숫자 metrics.jsonl은 bytes 그대로이며 head report의 중복 history/sample IDs는 원본 SHA를 남기고 view에서 제외했습니다.', '',
        '로컬 공개 파일 무결성 확인:', '', '```bash', 'cd docs/assets/validation/2026-09-08/portable_e2e_learning_cycle_v1/02_frozen_selector',
        'sha256sum -c SHA256SUMS', '```', '']
    return '\n'.join(lines)


def curate(campaign, parent, summary_path, output):
    campaign, parent, summary_path, output = map(Path, (campaign, parent, summary_path, output))
    if output.exists() or output.is_symlink():
        raise ContractError('publication output must be a new directory')
    if any(output.resolve().is_relative_to(path.resolve()) for path in (campaign, parent, summary_path.parent)):
        raise ContractError('publication cannot write inside its read-only input roots')
    started = datetime.now(timezone.utc).isoformat()
    publication_code = {str(path.relative_to(REPO)): sha(path) for path in
        (Path(__file__), Path(strict.__file__), REPO / 'scripts/e2e/curate_portable_learning_campaign.py',
            REPO / 'scripts/e2e/curate_independent_common10_capture_20260907.py')}
    report = read_json(summary_path)
    fresh = strict.summarize_campaign(campaign, parent)
    if report != fresh or report.get('status') != 'COMPLETE_NOT_PROMOTED':
        raise ContractError('publication requires a freshly verified complete raw summary')
    summary_sha = sha(summary_path)
    state = read_json(campaign / 'status.json')
    measured = measurements(campaign, report)
    sources = [(summary_path, Path('summary.json'), 'metadata'),
        (campaign / 'status.json', Path('provenance/status.json'), 'metadata'),
        (campaign / 'provenance/plan.json', Path('provenance/plan.json'), 'metadata')]
    route_indexes = []
    for seed in report['seeds']:
        relative = f'seed_{seed["seed"]}'
        sources.append((campaign / relative / 'result.json', Path('runs') / relative / 'result.json', 'metadata'))
        for arm in ('original_c_same_cache', *ARMS):
            origin = campaign / relative if arm == 'original_c_same_cache' else campaign / relative / arm
            geo = origin / ('original_c_geometry.json' if arm == 'original_c_same_cache' else 'geometry.json')
            route = origin / ('original_c_routes' if arm == 'original_c_same_cache' else 'routes')
            target = Path('routes') / relative / arm
            sources.append((geo, target / 'geometry.json', 'original'))
            for index in (0, 67, 134, 201, 268, 336):
                name = f'val_phase_{index:03d}.png'
                sources.append((route / name, target / name, 'original'))
            route_indexes.append((target / 'README.md', seed['seed'], arm))
            if arm != 'original_c_same_cache':
                sources.extend([(origin / 'report.json', Path('runs') / relative / arm / 'report.json', 'head_view'),
                    (origin / 'metrics.jsonl', Path('runs') / relative / arm / 'metrics.jsonl', 'original')])
    snapshots = {str(source): sha(source) for source, _, _ in sources}
    for source, _, representation in sources:
        if source.is_symlink() or not source.is_file():
            raise ContractError('publication source must be a regular file')
        if representation == 'original' and source.suffix != '.png' and redact_metadata(source.read_text()) != source.read_text():
            raise ContractError('original-byte numeric/geometry evidence unexpectedly contains private paths')
    output.mkdir(parents=True, exist_ok=False)
    visuals = output / 'visuals'
    visuals.mkdir()
    draw_plots(visuals, report, measured)
    manifest = []
    for source, target, representation in sources:
        destination = output / target
        if representation == 'head_view':
            record = read_json(source)
            for name in OMITTED_HEAD_FIELDS:
                record.pop(name)
            record.update(publication_notice='Redacted metadata view with duplicate history and sample IDs omitted; private original identified by raw_source_sha256.',
                raw_source_sha256=snapshots[str(source)], omitted_fields=list(OMITTED_HEAD_FIELDS))
            write_new_json(destination, redact_metadata(record))
        elif representation == 'original':
            copy_file(source, destination)
        else:
            representation = publish_file(source, destination)
        if source == summary_path:
            source_id = 'private_summary/summary.json'
        else:
            source_id = 'campaign/' + source.relative_to(campaign).as_posix()
        manifest.append({'published_path': target.as_posix(), 'source_path': source_id,
            'raw_source_sha256': snapshots[str(source)], 'sha256': sha(destination), 'representation': representation})
    for target, seed, arm in route_indexes:
        lines = [f'# Seed {seed} — {arm}', '',
            '<!-- HH_260906 - Show the six predeclared original offline validation phases without rerendering them. -->', '',
            '오프라인 val 후보 분석입니다. 실제 자율주행 제어 영상이 아닙니다. [전체 결과](../../../README.md)', '']
        for index in (0, 67, 134, 201, 268, 336):
            lines += [f'## Val index {index}', '', f'![val {index}](val_phase_{index:03d}.png)', '']
        with (output / target).open('x', encoding='utf-8') as stream:
            stream.write('\n'.join(lines))
    with (output / 'README.md').open('x', encoding='utf-8') as stream:
        stream.write(render_readme(report, state))
    plot_record = {'schema': 'portable_e2e.frozen_selector_public_plots.v1',
        'comment': 'HH_260906 - Actual batch records and weighted epoch means; final validation only.',
        'summary_raw_sha256': summary_sha, 'input_layout': 'frozen_selector_campaign_parent_roots_v1',
        'publication_code_sha256': publication_code,
        'inputs': [{key: value for key, value in item.items() if key != 'rows'} for item in measured],
        'plots': [{'path': name, 'width_px': 1920, 'sha256': sha(visuals / name)} for name in PLOTS],
        'vehicle_control_approved': False, 'automatic_promotion': False}
    write_new_json(visuals / 'plot_inputs.json', plot_record)
    # HH_260906 - Do not issue a successful publication manifest if any raw evidence or executing code changed during rendering.
    if sha(summary_path) != summary_sha or strict.summarize_campaign(campaign, parent) != report:
        raise ContractError('raw summary/evidence changed during publication')
    if any(sha(Path(path)) != expected for path, expected in snapshots.items()):
        raise ContractError('published source bytes changed during publication')
    if any(sha(REPO / path) != expected for path, expected in publication_code.items()):
        raise ContractError('publication code changed during execution')
    write_new_json(output / 'provenance/execution_record.json', {
        'schema': 'portable_e2e.frozen_selector_publication_provenance.v1',
        'training_source_commit': state['source_commit'], 'private_runner_sha256': state['runner_sha256'],
        'private_runner_relative_path': 'campaign/provenance/active_runner.py',
        'publication_started_at_utc': started, 'publication_completed_at_utc': datetime.now(timezone.utc).isoformat(),
        'publication_code_sha256': publication_code,
        'publication_source_policy': 'Executing working-tree files identified by SHA-256; these hashes are not a claim of a committed publication source.',
        'summary_raw_sha256': summary_sha, 'original_route_png_count': 72,
        'vehicle_control_approved': False, 'automatic_promotion': False})
    generated = [path for path in output.rglob('*') if path.is_file()
        and path.relative_to(output).as_posix() not in {item['published_path'] for item in manifest}]
    manifest += [{'published_path': path.relative_to(output).as_posix(), 'sha256': sha(path),
        'representation': 'generated_from_verified_evidence', 'summary_raw_sha256': summary_sha} for path in sorted(generated)]
    write_new_json(output / 'publication_manifest.json', {'schema': 'portable_e2e.frozen_selector_publication.v1',
        'comment': 'HH_260906 - Preserve original and derivative hashes; research tensors and the executable runner remain private.',
        'files': sorted(manifest, key=lambda item: item['published_path']),
        'vehicle_control_approved': False, 'automatic_promotion': False})
    with (output / 'SHA256SUMS').open('x') as stream:
        stream.write(''.join(f'{sha(path)}  {path.relative_to(output).as_posix()}\n'
            for path in sorted(output.rglob('*')) if path.is_file() and path.name != 'SHA256SUMS'))
    return {'status': 'PUBLISHED_NOT_PROMOTED', 'original_route_png_count': 72,
        'files': len(manifest) + 2, 'arm_screens': report['arm_screens']}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('campaign', type=Path)
    parser.add_argument('--parent-campaign', type=Path, required=True)
    parser.add_argument('--summary', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    print(json.dumps(curate(args.campaign, args.parent_campaign, args.summary, args.output)))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
