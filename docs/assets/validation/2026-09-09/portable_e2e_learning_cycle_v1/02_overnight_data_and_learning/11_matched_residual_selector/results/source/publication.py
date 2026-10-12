#!/usr/bin/env python3
"""HH_260906 - Publish all matched-residual saved outcomes without inference, tensor loading or admission."""
import argparse
import hashlib
import importlib.util
from io import BytesIO
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from scripts.e2e import summarize_frozen_stopmix_residual as audit

PRIVATE = ROOT / 'artifacts/training/2026-09-09'
INPUT = PRIVATE / 'frozen_stopmix_residual_v1'
PARENT = PRIVATE / 'frozen_stopmix_selector_v1'
SUMMARY = PRIVATE / 'frozen_stopmix_residual_summary_v1'
OUTPUT = ROOT / 'docs/assets/validation/2026-09-09/portable_e2e_learning_cycle_v1/02_overnight_data_and_learning/11_matched_residual_selector/results'
HELPER = PRIVATE / 'publish_frozen_stopmix_selector_v2.py'
HELPER_SHA = 'fe41b7ac5e40226edeb2c6e28a53dae76fe5619ea696745377911a975226ed8d'
AUDITOR_SHA = 'd9a877aa71bbb5529864f02af95dc76077268924f3757d9907f4609080ed1a01'
SUMMARY_SHA = '00f5c2429e23dc1aeec515f4e7f7986e728fcfc61b9370ff4d3ca17c5ef9b07a'
SUMMARY_MANIFEST_SHA = '0699790f06af210b9cb5bfe01144e0dc67756e4cce86378ff882f71910c5f5e0'
VARIANTS = ('original', *audit.ARMS)
EXECUTION = ('frozen_stopmix_residual_v1_console.log', 'frozen_stopmix_residual_v1_launch.json',
    'frozen_stopmix_residual_v1_ssh_completion.json', 'launch_frozen_stopmix_residual_v1.py')
require, digest, sha, encoded = audit.require, audit.digest, audit.sha, audit.encoded
base = audit.base

# HH_260906 - Reuse only reviewed byte-copy/privacy/checksum utilities, never the earlier experiment's authentication or outcome logic.
require(HELPER.is_file() and not HELPER.is_symlink() and hashlib.sha256(HELPER.read_bytes()).hexdigest() == HELPER_SHA,
    'reviewed publication utility bytes changed')
spec = importlib.util.spec_from_file_location('_residual_publication_utilities', HELPER)
utility = importlib.util.module_from_spec(spec); spec.loader.exec_module(utility)


def read(path): return audit.decode(base.regular(path).read_bytes())


def sample_path(seed, arm, split):
    return f'seed_{seed}/' + ('original' if arm == 'original' else arm + '/analysis') + f'/{split}/samples.jsonl'


def compare_selections(original, other):
    require(len(original) == len(other) and len(original) > 0, 'all paired selection rows required')
    changed = []
    for index, (left, right) in enumerate(zip(original, other)):
        require(type(left['index']) is int and left['index'] == right['index'] == index, 'selection index order differs')
        for field in base.IDENTITY: base.same(left[field], right[field], 'selection pair identity differs')
        for row in (left, right):
            require(type(row['selected_candidate_index']) is int and 0 <= row['selected_candidate_index'] < 12,
                'integer candidate index required')
        if left['selected_candidate_index'] != right['selected_candidate_index']:
            changed.append(dict(index=index, sample_id=left['sample_id'], original_index=left['selected_candidate_index'],
                residual_index=right['selected_candidate_index'], target_motion_group=left['target_motion_group']))
    return dict(sample_count=len(original), changed_selection_count=len(changed), unchanged_selection_count=len(original) - len(changed),
        changed_selections=changed)


def selection_counts(raw):
    result = dict(schema='portable_e2e.matched_residual_saved_selection_changes.v1', comparisons=[],
        scope='Exact saved candidate indices, not equality inferred from aggregate metrics; no new model or gate execution.')
    for seed in base.SEEDS:
        for split, count in base.COUNTS.items():
            original_name = sample_path(seed, 'original', split)
            original_bytes = base.regular(INPUT / original_name).read_bytes()
            require(sha(original_bytes) == raw[original_name], 'original selection source changed')
            original = [audit.decode(line) for line in original_bytes.splitlines()]
            require(len(original) == count, 'full split selection denominator required')
            for arm in audit.ARMS:
                name = sample_path(seed, arm, split); payload = base.regular(INPUT / name).read_bytes()
                require(sha(payload) == raw[name], 'residual selection source changed')
                paired = compare_selections(original, [audit.decode(line) for line in payload.splitlines()])
                result['comparisons'].append(dict(seed=seed, split=split, arm=arm,
                    original_samples_sha256=raw[original_name], residual_samples_sha256=raw[name], **paired))
    return result


def authenticate():
    require(digest(HELPER) == HELPER_SHA and digest(Path(audit.__file__)) == AUDITOR_SHA,
        'reviewed publication/independent summary source changed')
    require(digest(SUMMARY / 'summary.json') == SUMMARY_SHA and digest(SUMMARY / 'SHA256SUMS') == SUMMARY_MANIFEST_SHA,
        'frozen actual residual summary changed')
    expected = read(SUMMARY / 'summary.json')
    base.same(expected, audit.summarize_campaign(INPUT, PARENT), 'full residual diagnostic no longer reproduces')
    require(expected['status'] == 'COMPLETE_DIAGNOSTIC_NOT_PROMOTED' and expected['summary_source_sha256'] == AUDITOR_SHA,
        'complete independently reviewed residual summary required')
    for key, value in dict(total_analysis_rows=13356, head_history_rows=9240, fixed_val_png_count=108,
            full_model_fit_count=0, completed_head_fit_count=6, head_sample_exposures=36930,
            matched_allocated_head_parameters=115457, unchanged_original_cache_run_file_count=205).items():
        base.same(expected[key], value, 'complete residual denominator differs: ' + key)
    for key, value in audit.DENIALS.items(): base.same(expected[key], value, 'research denial differs')
    require(base.manifest((SUMMARY / 'SHA256SUMS').read_bytes()) == {'summary.json': SUMMARY_SHA}, 'summary manifest differs')
    raw, parent_raw = base.inventory(INPUT), base.inventory(PARENT)
    require(len(raw) == 184 and raw == expected['input_sha256'] and parent_raw == expected['parent_input_sha256'],
        'new184 and original188 collected inventory required')
    transport = read(INPUT / 'transport_verification.json'); launches = transport['local_launch_receipt_sha256']
    require(len(launches) == 11 and all(name in launches for name in EXECUTION), 'all new/parent launch evidence required')
    for name, pin in launches.items(): require(digest(PRIVATE / base.safe_name(name)) == pin, 'launch evidence changed')
    completion = read(PRIVATE / 'frozen_stopmix_residual_v1_ssh_completion.json')
    require(completion['exit_code'] == 0 and completion['error_type'] is None and completion['local_source_bytes_unchanged'] is True
        and completion['console_sha256'] == launches['frozen_stopmix_residual_v1_console.log'], 'successful residual launch proof differs')
    pins = {str((INPUT / n).relative_to(ROOT)): v for n, v in raw.items()}
    pins.update({str((PARENT / n).relative_to(ROOT)): v for n, v in parent_raw.items()})
    pins.update({str((PRIVATE / n).relative_to(ROOT)): v for n, v in launches.items()})
    for path in (SUMMARY / 'summary.json', SUMMARY / 'SHA256SUMS', HELPER, Path(audit.__file__), Path(base.__file__)):
        pins[str(path.relative_to(ROOT))] = digest(path)
    for path in [*(INPUT / n for n in raw), *(PRIVATE / n for n in EXECUTION), SUMMARY / 'summary.json',
            SUMMARY / 'SHA256SUMS', HELPER, Path(audit.__file__), Path(base.__file__)]:
        if path.suffix != '.png': utility.public_text(utility.view_bytes(base.regular(path).read_bytes(), path.suffix)[0].decode())
    counts = selection_counts(raw)
    return expected, counts, raw, {n: launches[n] for n in EXECUTION}, pins


def recheck(pins, raw, parent_raw):
    for name, pin in pins.items(): require(digest(ROOT / name) == pin, 'original input/source changed: ' + name)
    require(base.inventory(INPUT) == raw and base.inventory(PARENT) == parent_raw, 'new/parent inventory changed')


def fixed_chart(report):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(11, 5), dpi=150, layout='constrained')
    for i, (arm, color) in enumerate(zip(VARIANTS, ('#5d6a7c', '#247eaf', '#ce6d2c'))):
        for ax, key in zip(axes, ('selected_ade_m', 'selected_fde_m')):
            values = [s['variants'][arm]['val']['all_samples']['metrics'][key] for s in report['seeds']]
            ax.bar([x + (i - 1) * .24 for x in range(3)], values, .23, label=arm, color=color)
    for ax, title in zip(axes, ('Development VAL337: ADE (m)', 'Development VAL337: FDE (m)')):
        ax.set_title(title); ax.set_xticks(range(3), [s['seed'][-2:] for s in report['seeds']]); ax.set_xlabel('Seed suffix')
        ax.grid(axis='y', alpha=.2); ax.set_axisbelow(True)
    axes[0].legend(fontsize=8)
    fig.suptitle('Matched allocated parameters: geometry residual has mixed validation outcomes', fontsize=12)
    stream = BytesIO(); fig.savefig(stream, format='png'); plt.close(fig)
    return stream.getvalue(), dict(matplotlib_version=matplotlib.__version__, matplotlib_init_sha256=digest(Path(matplotlib.__file__)),
        selection='All original/A/B endpoints for all three fixed seeds; no smoothing, selection or new observation.',
        original_images_reprocessed=False, model_inference=False)


def readme(report, counts):
    lines = ['# 동일 크기 잔차 선택기 — 전체 실제 결과', '',
        '<!-- HH_260906 - Preserve all outcomes, including regressions; this is a frozen-generator information ablation, not driving or admission. -->', '',
        '기존 3개 생성기·12후보·입력 cache·composite teacher를 그대로 두고 **head 전용 6회**를 학습했습니다. 전체 모델을 다시 학습한 것이 아닙니다. 각각 1,540 step·6,155회 노출, 합계 9,240 step·36,930회 노출입니다. 3 baseline+6 head의 TRAIN1,147/VAL337 저장 행 13,356개와 원본 PNG108장을 검증했습니다.', '',
        '| seed | 선택기 | VAL ADE m | VAL FDE m | TRAIN hold ADE m | hold STOP |',
        '| --- | --- | ---: | ---: | ---: | ---: |']
    for seed in report['seeds']:
        for arm in VARIANTS:
            view = seed['variants'][arm]; val = view['val']['all_samples']['metrics']; hold = view['train']['target_motion_groups']['stationary_hold']
            lines.append(f"| {seed['seed']} | {arm} | {val['selected_ade_m']:.6f} | {val['selected_fde_m']:.6f} | {hold['metrics']['selected_ade_m']:.8f} | {hold['stop_selection_count']}/{hold['sample_count']} |")
    lines += ['', '![All validation outcomes](all_validation_endpoints.png)', '',
        'A(prior_only_residual)와 B(candidate_residual)는 같은 449→256→1 구조, 할당 파라미터115,457개, 같은 초기 head 상태를 사용합니다. original score에 마지막 층이 0으로 초기화된 잔차를 더합니다. A의 geometry192개 입력만 0이며 B는 실제 고정 후보 geometry를 받습니다. 후보·순서·원래 CE×0.1·학습률·optimizer는 같지만 A의 비활성 geometry 가중치와 유효 정보/gradient는 다릅니다. 따라서 같은 파라미터 수가 같은 유효 용량을 뜻하지 않습니다.', '',
        'B는 TRAIN 정지 유지118개에서 세 seed 모두 STOP118/118을 유지했지만, VAL seed20260904의 ADE/FDE는 원래 선택기보다 악화됐습니다. seed03·05의 두 지표는 감소했습니다. **VAL stationary_hold는 0개**이며 평균 VAL 개선으로 정지 성능을 검증할 수 없습니다. 승자·승격을 정하지 않았습니다.', '',
        '아래 선택 변화는 평균 metric 일치에서 추정하지 않고 동일한 saved sample의 candidate index를 전수 비교했습니다. A의 변경은 전체 TRAIN/VAL에서 0개입니다. 초기 logit 바이트 일치는 독립 요약의 별도 실행 근거이고, 최종 score 자체가 같다는 뜻은 아닙니다.', '',
        '| seed | split | arm | 변경 / 전체 선택 |', '| --- | --- | --- | ---: |']
    for item in counts['comparisons']:
        lines.append(f"| {item['seed']} | {item['split']} | {item['arm']} | {item['changed_selection_count']} / {item['sample_count']} |")
    lines += ['', '[전수 독립 요약·이전 선택기와 비교](independent_summary.json) · [선택 변화 전체 witness](selection_changes.json) · [실제 report](raw/report.json) · [실행 전 계획](raw/plan.json) · [전달 검증](raw/transport_verification.json)', '',
        'stationary_hold/moving_to_stop/continuing_motion/other_motion/unavailable_masks 전체 그룹 및 episode별 결과를 보존합니다. 이 그룹은 미래 정답을 이용한 사후 진단이며 NN 입력이나 실제 신호등 상태가 아닙니다. TRAIN은 in-sample, VAL은 반복 사용한 단일 개발 episode로 독립 holdout이 아닙니다. 기존 test 파일은 full-corpus 무결성 검사에서 읽힐 수 있지만 test NN·loss·metric·선택은 없습니다.', '',
        '## 원본 9개 variant × 고정 12개 VAL 그림', '']
    for seed in base.SEEDS:
        for arm in VARIANTS: lines.append(f'- [{seed} / {arm}: 모든 TRAIN/VAL행·PNG12장](galleries/{seed}_{arm}.md)')
    lines += ['', '## 전체 6개 학습 history', '']
    for seed in base.SEEDS:
        for arm in audit.ARMS: lines.append(f'- [{seed} / {arm}: 1,540행](raw/seed_{seed}/{arm}/metrics.jsonl) · [head report](raw/seed_{seed}/{arm}/report.json)')
    lines += ['', '## 원본·전송·해석 범위', '',
        '원격195개 중183개 원본과 transport receipt1개를 로컬에 보존했습니다. 새 head/logits PT12개는 원격에만 있고 SHA·크기로 연결합니다. 원래 cache 실행205개와 그 PT18개 및 continued parent 입력17개의 불변 검증도 유지합니다. 이 게시자는 어떤 PT도 읽거나 새 NN을 실행하지 않습니다. 실제 worker의 원래 cache6개 CPU weights-only 로드는 새 생성기 forward나 parent 모델 재학습이 아닙니다.', '',
        '원본 PNG108장·samples JSONL18개·전체 metrics JSONL6개는 바이트 그대로 복사합니다. 개인 경로/GPU UUID를 가린 metadata·source·log는 원본/공개 SHA를 각각 기록합니다. 일부 공개 source의 placeholder는 실행 가능한 배포본이 아니라 감사용 참조입니다. numeric chart만 기존 저장 숫자로 렌더링하며 원본 그림을 수정하지 않습니다.', '',
        '[실행 로그](execution/frozen_stopmix_residual_v1_console.log) · [종료 receipt](execution/frozen_stopmix_residual_v1_ssh_completion.json) · [이전 fresh-head 결과와 최초 launcher 거절 보존](../../10_frozen_stopmix_selector/results/README.md)', '',
        'compact 행으로 비용 argmin·정확한 동률·regret·모든 집계를 재검산했으나 전체 XY/speed/target 배열이 없어 새 trajectory gate 실행이나 그룹 재분류를 하지 않았습니다. raw current vx를 사용하는 원래 gate 결과는 source와 저장 행으로 연결합니다. GPU float32 core metric과 Python 저장 행 집계를 별도 보존합니다. legacy 개발 데이터·물리 한계·runtime allowlist·정답·새 원본 데이터 승인 상태는 바꾸지 않았습니다.', '',
        '[원본/공개 SHA provenance](publication_manifest.json) · [공개 체크섬](SHA256SUMS) · [원본 원격 체크섬 참조](raw/original_remote_SHA256SUMS.txt)', '',
        '원본 원격 체크섬에는 생략 PT와 비식별화 전 파일이 있으므로 공개 tree 검증에는 이 폴더의 SHA256SUMS만 사용합니다. 실행 native source b478, 외부 residual worker c1572/core d4bf, 새 요약 d9a877을 구분합니다. 실제 차량 주행·실시간 추론·배포 성능이나 데이터 승인 결과가 아닙니다.', '']
    return '\n'.join(lines).encode()


def publish():
    require(not OUTPUT.exists() and not any(p.is_symlink() for p in (OUTPUT, *OUTPUT.parents)), 'fresh results folder required')
    report, counts, raw, launches, pins = authenticate(); own = base.regular(Path(__file__)).read_bytes(); own_sha = sha(own)
    parent_raw = report['parent_input_sha256']; entries = {}
    for name, pin in raw.items():
        public = 'raw/' + ('original_remote_SHA256SUMS.txt' if name == 'SHA256SUMS' else name)
        entries[public] = utility.copy_view(INPUT / name, OUTPUT / public, pin)
    for name, pin in launches.items(): entries['execution/' + name] = utility.copy_view(PRIVATE / name, OUTPUT / 'execution' / name, pin)
    for path, name, pin in ((SUMMARY / 'summary.json', 'independent_summary.json', SUMMARY_SHA),
            (SUMMARY / 'SHA256SUMS', 'original_summary_SHA256SUMS.txt', SUMMARY_MANIFEST_SHA),
            (Path(audit.__file__), 'source/independent_summary.py', AUDITOR_SHA),
            (Path(base.__file__), 'source/parent_summary_dependency.py', audit.BASE_SHA),
            (HELPER, 'source/publication_utilities.py', HELPER_SHA)):
        entries[name] = utility.copy_view(path, OUTPUT / name, pin)
    for seed in base.SEEDS:
        for arm in VARIANTS: utility.new_bytes(OUTPUT / 'galleries' / f'{seed}_{arm}.md', utility.gallery(seed, arm))
    chart, renderer = fixed_chart(report); utility.new_bytes(OUTPUT / 'all_validation_endpoints.png', chart)
    utility.new_bytes(OUTPUT / 'selection_changes.json', encoded(counts)); utility.new_bytes(OUTPUT / 'README.md', readme(report, counts))
    utility.new_bytes(OUTPUT / 'source/publication.py', own)
    for name, value in base.inventory(OUTPUT).items():
        if name not in entries: entries[name] = dict(public_sha256=value, publication_kind='generated_publication_evidence')
    recheck(pins, raw, parent_raw); require(digest(Path(__file__)) == own_sha, 'publisher source changed')
    provenance = dict(schema='portable_e2e.matched_residual_publication.v1', publisher_sha256=own_sha,
        independent_summary_sha256=SUMMARY_SHA, independent_summary_source_sha256=AUDITOR_SHA,
        publication_utility_source_sha256=HELPER_SHA, original_input_sha256=pins, public_files=entries, renderer=renderer,
        all_six_head_histories_retained=True, all_nine_train_val_variants_retained=True, fixed_original_png_count=108,
        original_png_and_jsonl_bytes_unchanged=True, matched_allocated_head_parameters=115457,
        tensors_transferred_or_loaded_by_publisher=False, omitted_new_remote_tensor_count=12,
        unchanged_original_remote_tensor_count=18, original_cache_run_file_count=205, **audit.DENIALS)
    utility.new_bytes(OUTPUT / 'publication_manifest.json', encoded(provenance))
    final = base.inventory(OUTPUT)
    utility.new_bytes(OUTPUT / 'SHA256SUMS', ''.join(v + '  ' + n + '\n' for n, v in sorted(final.items())).encode())
    proof = utility.verify_public(OUTPUT); require(proof['png_count'] == 109, 'all108 original PNG plus one actual numeric plot required')
    recheck(pins, raw, parent_raw); require(digest(Path(__file__)) == own_sha, 'publisher final source postcheck failed')
    return proof


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False); parser.add_argument('--verify-only', action='store_true')
    args = parser.parse_args(argv)
    if args.verify_only:
        report, counts, raw, launches, pins = authenticate(); recheck(pins, raw, report['parent_input_sha256'])
        print(json.dumps(dict(status='VERIFIED_NO_WRITES', collected_files=len(raw), execution_receipts=len(launches),
            saved_rows=report['total_analysis_rows'], selection_comparisons=len(counts['comparisons']), fixed_png_count=report['fixed_val_png_count']))); return
    print(json.dumps(publish(), sort_keys=True))


if __name__ == '__main__': main()
