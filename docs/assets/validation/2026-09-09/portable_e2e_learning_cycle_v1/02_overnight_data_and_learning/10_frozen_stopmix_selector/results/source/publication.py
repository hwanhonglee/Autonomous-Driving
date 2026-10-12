#!/usr/bin/env python3
"""HH_260906 - Publish all frozen-selector outcomes and failed launcher provenance without rerunning networks or admitting data."""
import argparse
from io import BytesIO
import ipaddress
import json
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from scripts.e2e import summarize_frozen_stopmix_selector as audit

PRIVATE = ROOT / 'artifacts/training/2026-09-09'
INPUT = PRIVATE / 'frozen_stopmix_selector_v1'
SUMMARY = PRIVATE / 'frozen_stopmix_selector_summary_v1'
OUTPUT = ROOT / 'docs/assets/validation/2026-09-09/portable_e2e_learning_cycle_v1/02_overnight_data_and_learning/10_frozen_stopmix_selector/results'
AUDITOR_SHA = 'b6d32f8fa3aa446de0211123a032961fbbd2d22497b39d14b4a9dbd4f4b9427b'
SUMMARY_SHA = '0e6085856d436f8e296da412875af5fc07e2886db3020cd8340baa72f45a9ec2'
SUMMARY_MANIFEST_SHA = '9551363f0d336a6151eec7807e58a647d2f05ac2f217310902eaee955ef2200a'
ORIGINAL_PUBLISHER_SHA = '975b927cd957962538a6ee469b83f3bee2dd18cb5c208491909a40e77719f808'
FAILED_RECEIPT = PRIVATE / 'frozen_stopmix_selector_publication_attempt_001_receipt.json'
FAILED_RECEIPT_SHA = '6085e0221f79ce6bc515171ce85c28bfb513696c563d773ddf7b0d40e90affa8'
FAILED_ROOT = PRIVATE / 'frozen_stopmix_selector_publication_attempt_001_incomplete'
NOTICE = 'Public metadata view: only machine-specific roots/GPU UUIDs are redacted. raw_source_sha256 binds unchanged private bytes. Original manifests describe private/remote originals, not this public tree. No model or data admission.'
VARIANTS = ('original', 'linear_pair_reset', 'candidate_reset')
require, digest, sha, encoded = audit.require, audit.digest, audit.sha, audit.encoded


def read(path): return audit.decode(audit.regular(path).read_bytes())


def redact(value):
    if isinstance(value, str):
        value = re.sub(r'/(?:home)/[^/\s\"\']+', lambda _: '${USER_HOME}', value)
        value = re.sub(r'/(?:tmp|root)(?=/|$|[\s\"\'])', lambda _: '${PRIVATE_ROOT}', value)
        return re.sub(r'(?<![0-9A-Za-z])(?:GPU-)?[0-9a-fA-F]{8}(?:-[0-9a-fA-F]{4}){3}-[0-9a-fA-F]{12}(?![0-9A-Za-z])', '${GPU0_UUID}', value)
    if isinstance(value, list): return [redact(v) for v in value]
    if isinstance(value, dict): return {redact(k): redact(v) for k, v in value.items()}
    return value


def new_bytes(path, raw):
    require(not path.exists() and not any(p.is_symlink() for p in (path, *path.parents)), 'create-only nonsymlink publication required')
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('xb') as stream: stream.write(raw)


def view_bytes(raw, suffix):
    if suffix == '.json':
        value = audit.decode(raw)
        require(type(value) is dict and not {'raw_source_sha256', 'publication_notice'} & set(value), 'reserved metadata wrapper keys')
        value = redact(value); value.update(raw_source_sha256=sha(raw), publication_notice=NOTICE)
        return encoded(value), 'redacted_metadata_view'
    if suffix in ('.png', '.jsonl'): return raw, 'original_bytes'
    value = redact(raw.decode()).encode()
    return value, 'original_bytes' if value == raw else 'redacted_source_or_text_reference_not_executable'


def public_text(text):
    # HH_260906 - Match an actual complete PEM private-key header, not this source's own detection expression or explanatory text.
    require(not re.search(r'/(?:home/[^/\s]+|tmp|root)/', text)
        and not re.search(r'-----BEGIN (?:[A-Z0-9]+ )*PRIVATE KEY-----', text), 'private path/credential in publication')
    for value in re.findall(r'(?<![\d.])(?:\d{1,3}\.){3}\d{1,3}(?![\d.])', text):
        try: ipaddress.IPv4Address(value)
        except ipaddress.AddressValueError: continue
        raise ValueError('literal IP address in publication')


def copy_view(path, target, expected):
    raw = audit.regular(path).read_bytes(); require(sha(raw) == expected, 'source bytes differ before publication')
    public, kind = view_bytes(raw, path.suffix)
    if path.suffix != '.png': public_text(public.decode())
    if path.suffix == '.png':
        from PIL import Image
        with Image.open(BytesIO(raw)) as image:
            require(image.format == 'PNG' and image.size == (900, 700), 'fixed original behavior figure ABI differs'); image.load()
    new_bytes(target, public); require(digest(path) == expected and digest(target) == sha(public), 'source/public changed during copy')
    return dict(raw_private_path=path.relative_to(ROOT).as_posix(), raw_source_sha256=expected,
        public_sha256=sha(public), publication_kind=kind, original_bytes_preserved=public == raw)


def authenticate():
    require(digest(Path(audit.__file__)) == AUDITOR_SHA, 'reviewed independent summary source changed')
    require(digest(SUMMARY / 'summary.json') == SUMMARY_SHA and digest(SUMMARY / 'SHA256SUMS') == SUMMARY_MANIFEST_SHA,
        'frozen actual independent summary changed')
    expected = read(SUMMARY / 'summary.json'); observed = audit.summarize_campaign(INPUT)
    audit.same(expected, observed, 'actual full diagnostic no longer reproduces')
    require(expected['status'] == 'COMPLETE_DIAGNOSTIC_NOT_PROMOTED' and expected['summary_source_sha256'] == AUDITOR_SHA
        and expected['total_analysis_rows'] == 13356 and expected['head_history_rows'] == 9240 and expected['fixed_val_png_count'] == 108
        and expected['full_model_fit_count'] == 0 and expected['completed_head_fit_count'] == 6
        and all(expected[k] is False for k in audit.DENIALS), 'all6 head/9 diagnostic outcomes and denials required')
    require(audit.manifest((SUMMARY / 'SHA256SUMS').read_bytes()) == {'summary.json': SUMMARY_SHA}, 'summary manifest differs')
    raw = audit.inventory(INPUT); require(len(raw) == 188 and raw == expected['input_sha256'], 'all188 collected files required')
    receipt = read(INPUT / 'transport_verification.json'); launches = receipt['local_launch_receipt_sha256']
    require(len(launches) == 6 and set(launches) == {f'frozen_stopmix_selector_v{v}_{name}' for v in (1, 2)
        for name in ('launch.json', 'console.log', 'ssh_completion.json')}, 'both launcher attempts must be retained')
    for name, pin in launches.items(): require(digest(PRIVATE / audit.safe_name(name)) == pin, 'original launcher receipt changed')
    first, second = (read(PRIVATE / f'frozen_stopmix_selector_v{v}_ssh_completion.json') for v in (1, 2))
    require(first['exit_code'] == 1 and second['exit_code'] == 0 and first['error_type'] is None and second['error_type'] is None
        and first['local_source_bytes_unchanged'] is True and second['local_source_bytes_unchanged'] is True,
        'rejected first launcher and successful second execution required')
    require(first['console_sha256'] == launches['frozen_stopmix_selector_v1_console.log']
        and second['console_sha256'] == launches['frozen_stopmix_selector_v2_console.log'], 'launcher console binding differs')
    require(receipt['first_attempt']['head_fits'] == receipt['first_attempt']['neural_network_forwards'] == 0
        and receipt['first_attempt']['status'] == 'REJECTED_BEFORE_GPU_LEASE_AND_MODEL_IMPORT'
        and b'fresh output must be inside personal runs/diagnostics' in (PRIVATE / 'frozen_stopmix_selector_v1_console.log').read_bytes(),
        'pre-GPU rejection must not be relabeled as a failed fit')
    pins = {str((INPUT / n).relative_to(ROOT)): v for n, v in raw.items()}
    pins.update({str((PRIVATE / n).relative_to(ROOT)): v for n, v in launches.items()})
    pins.update({str((SUMMARY / n).relative_to(ROOT)): digest(SUMMARY / n) for n in ('summary.json', 'SHA256SUMS')})
    pins[str(Path(audit.__file__).relative_to(ROOT))] = AUDITOR_SHA
    failure_pins = verify_preserved_failure()
    for name in pins:
        path = ROOT / name
        if path.suffix != '.png': public_text(view_bytes(audit.regular(path).read_bytes(), path.suffix)[0].decode())
    pins.update(failure_pins)
    return expected, raw, launches, pins


def verify_preserved_failure():
    # HH_260906 - The prior publication failed its final privacy check; preserve all original output bytes without relabeling that attempt as successful.
    require(digest(FAILED_RECEIPT) == FAILED_RECEIPT_SHA, 'original failed-publication receipt changed')
    receipt = read(FAILED_RECEIPT); files = audit.inventory(FAILED_ROOT)
    require(receipt['status'] == 'PUBLICATION_FINAL_VERIFICATION_FAILED_ORIGINALS_PRESERVED'
        and receipt['publisher_sha256'] == ORIGINAL_PUBLISHER_SHA and receipt['exit_code'] == 1
        and receipt['completed_created_file_count'] == len(files) == 211
        and receipt['completed_payload_checksum_count'] == 210
        and receipt['all_created_payload_bytes_verified_before_relocation'] is True
        and all(receipt[k] is False for k in ('published_to_git', 'training_rerun', 'model_or_data_changed', 'safety_gate_changed')),
        'first failed publication must remain explicitly failed with all originals retained')
    require(files['source/publication.py'] == ORIGINAL_PUBLISHER_SHA
        and files['SHA256SUMS'] == receipt['created_manifest_sha256']
        and files['publication_manifest.json'] == receipt['created_publication_manifest_sha256'], 'preserved failed source/manifest changed')
    manifest = audit.manifest((FAILED_ROOT / 'SHA256SUMS').read_bytes())
    require(manifest == {n: v for n, v in files.items() if n != 'SHA256SUMS'}, 'preserved first-attempt payload hash differs')
    for name in files:
        if Path(name).suffix != '.png': public_text((FAILED_ROOT / name).read_text())
    pins = {str((FAILED_ROOT / n).relative_to(ROOT)): v for n, v in files.items()}
    pins[str(FAILED_RECEIPT.relative_to(ROOT))] = FAILED_RECEIPT_SHA
    return pins


def recheck(pins):
    for name, pin in pins.items(): require(digest(ROOT / name) == pin, 'original input changed during publication: ' + name)


def fixed_chart(report):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 3, figsize=(15, 5.2), dpi=150, layout='constrained')
    labels = [s['seed'][-2:] for s in report['seeds']]; colors = ('#5d6a7c', '#247eaf', '#ce6d2c')
    for i, (arm, color) in enumerate(zip(VARIANTS, colors)):
        points = [s['variants'][arm] for s in report['seeds']]; xs = [x + (i - 1) * .24 for x in range(3)]
        values = ([p['val']['all_samples']['metrics']['selected_ade_m'] for p in points],
            [p['train']['target_motion_groups']['stationary_hold']['metrics']['selected_ade_m'] for p in points],
            [p['train']['target_motion_groups']['stationary_hold']['stop_selection_count'] for p in points])
        for ax, ys in zip(axes, values): ax.bar(xs, ys, .23, color=color, label=arm)
    for ax, label in zip(axes, ('Development VAL337: selected ADE (m)', 'In-sample TRAIN hold118: selected ADE (m)', 'In-sample TRAIN hold118: STOP selections')):
        ax.set_title(label, fontsize=10); ax.set_xticks(range(3), labels); ax.set_xlabel('Seed suffix'); ax.grid(axis='y', alpha=.2); ax.set_axisbelow(True)
    axes[2].set_ylim(0, 128); axes[0].legend(fontsize=8)
    fig.suptitle('Frozen candidates: VAL mean improvements do not preserve TRAIN stationary-hold behavior', fontsize=13)
    stream = BytesIO(); fig.savefig(stream, format='png'); plt.close(fig)
    return stream.getvalue(), dict(matplotlib_version=matplotlib.__version__, matplotlib_init_sha256=digest(Path(matplotlib.__file__)),
        selection='All three fixed seeds and all original/A/B endpoints; no subset, smoothing or visual outcome selection.',
        native_input_images_reprocessed=False, model_inference=False)


def gallery(seed, arm):
    prefix = f'raw/seed_{seed}/' + ('original' if arm == 'original' else arm + '/analysis')
    lines = [f'# {seed} / {arm} — 고정 VAL 12시점', '', '<!-- HH_260906 - Preserve every preregistered validation plot; these are saved model trajectory plots, not vehicle driving. -->', '',
        '원래 337개 개발 VAL 중 실행 전에 고정한 12개 index입니다. 모두 900×700 원본 PNG 바이트를 보존했습니다. 실제 차량 주행·실시간 추론 영상이 아닙니다.', '',
        f'[전체 TRAIN1,147행](../{prefix}/train/samples.jsonl) · [전체 VAL337행](../{prefix}/val/samples.jsonl) · [VAL 집계](../{prefix}/val/summary.json)', '']
    for index in audit.INDICES: lines += [f'## VAL index {index}', '', f'![Fixed VAL {index}](../{prefix}/val/val_{index:03d}.png)', '']
    return '\n'.join(lines).encode()


def readme(report):
    lines = ['# 생성기 고정 선택기 — 실제 6개 head 학습의 전체 결과', '',
        '<!-- HH_260906 - Report fixed saved outcomes and group regressions without promotion or causal interpretation. -->', '',
        '원래 3개 STOPMIX 생성기·12후보를 고정하고 A/B 선택기만 각각 새로 학습했습니다. **전체 모델 6회가 아니라 head 전용 6회**이며, 각 1,540 step·6,155회 표본 노출입니다. 합계 9,240 step·36,930회 노출, 3 baseline+6 head의 TRAIN/VAL 총 13,356개 저장 행을 검증했습니다.', '',
        '| seed | 선택기 | VAL337 ADE m | VAL FDE m | TRAIN hold118 ADE m | hold STOP 선택 |',
        '| --- | --- | ---: | ---: | ---: | ---: |']
    for seed in report['seeds']:
        for arm in VARIANTS:
            item = seed['variants'][arm]; val = item['val']['all_samples']['metrics']; hold = item['train']['target_motion_groups']['stationary_hold']
            lines.append(f"| {seed['seed']} | {arm} | {val['selected_ade_m']:.6f} | {val['selected_fde_m']:.6f} | {hold['metrics']['selected_ade_m']:.8f} | {hold['stop_selection_count']}/{hold['sample_count']} |")
    lines += ['', '![All endpoints and retained hold regression](all_endpoints_and_hold_regression.png)', '',
        'B(candidate_reset)의 VAL ADE는 세 seed 모두 원래 선택기보다 줄었지만, TRAIN 정지 유지118개에서는 세 seed 모두 STOP 선택이 0/118로 퇴행했습니다. 원래 선택기와 A는 118/118을 유지합니다. **VAL에는 stationary_hold 표본이 0개**이므로 VAL 평균만으로 정지 성능 개선을 주장할 수 없습니다. B는 A보다 seed05의 VAL ADE가 나쁘고, seed04·05의 VAL FDE도 나쁩니다. 승자·승격은 정하지 않았습니다.', '',
        'A는 두 Linear(256,6), 총3,084 parameters이고 B는 공유448→256→1 MLP, 총115,201 parameters입니다. 후보 정보와 구조·용량 효과가 섞여 있습니다. hard-argmin composite teacher·CE×0.1·학습 순서·후보·정답·mask는 바꾸지 않았습니다. 정확한 동률만 사용하며, 동률이나 loss floor만으로 퇴행 원인·해결책을 단정하지 않습니다.', '',
        '미래 정답으로 분류한 stationary_hold/moving_to_stop/continuing_motion/other_motion/unavailable_masks 전체 그룹을 별도 보존합니다. TRAIN은 in-sample, VAL은 반복 사용한 단일 개발 episode로 독립 holdout이 아닙니다. 그룹은 입력이 아니라 사후 진단이며 실제 신호등 상태를 알려주지 않습니다. 기존 full-corpus 무결성 검사는 test 파일을 읽을 수 있지만 test NN forward·학습·metric·선택은 없습니다.', '',
        '[독립 전수 요약](independent_summary.json) · [실제 report](raw/report.json) · [실행 전 계획](raw/plan.json) · [원격/로컬 전달 검증](raw/transport_verification.json)', '',
        '## 모든 저장 행·고정 그림', '']
    for seed in audit.SEEDS:
        for arm in VARIANTS: lines.append(f'- [{seed} / {arm}: 고정 PNG12장·TRAIN/VAL 전체행](galleries/{seed}_{arm}.md)')
    lines += ['', '## 6개 전체 학습 기록', '']
    for seed in audit.SEEDS:
        for arm in audit.ARMS: lines.append(f'- [{seed} / {arm}: 전체1,540 step](raw/seed_{seed}/{arm}/metrics.jsonl) · [head report](raw/seed_{seed}/{arm}/report.json)')
    lines += ['', '## 첫 launcher 거절과 원본 보존', '',
        '첫 launcher는 잘못된 output namespace 때문에 GPU lease·모델 import·데이터 로딩·optimizer 생성 전에 거절됐습니다. head fit0/NN forward0이며 실패한 학습으로 더하지 않습니다. 둘째는 output 경로만 수정해 같은 worker/core/plan으로 실행했습니다. 최초 실패의 로그와 receipt도 보존했습니다.', '',
        '[첫 거절 로그](execution/frozen_stopmix_selector_v1_console.log) · [첫 종료 receipt](execution/frozen_stopmix_selector_v1_ssh_completion.json) · [둘째 로그](execution/frozen_stopmix_selector_v2_console.log) · [둘째 종료 receipt](execution/frozen_stopmix_selector_v2_ssh_completion.json)', '',
        '원격205개 파일 중187개 원본을 로컬에 받았고 transport receipt1개를 더했습니다. **18개 cache/head/logits PT는 원격에만 있으며 SHA·크기만 연결합니다.** 이 게시자는 PT를 읽거나 모델을 실행하지 않습니다. 원본 108 PNG, 18개 samples JSONL, 6개 metrics JSONL 및 cache journal은 그대로 복사합니다. JSON metadata/source/log에서 개인 경로·GPU UUID만 가린 경우 원본 SHA와 공개 SHA를 함께 기록합니다. 공개 source는 감사용 참조이며 일부 placeholder 때문에 실행 가능한 배포본이 아닙니다.', '',
        '저장된 candidate 비용 argmin·정확한 동률·regret·전체 행 집계는 재검산했습니다. compact JSONL에는 전체 XY/speed·target speed 배열이 없으므로 이 단계가 새로운 full-trajectory gate 실행이나 그룹 재분류를 했다고 주장하지 않습니다. 원래 raw current vx를 사용한 gate는 실행 source와 저장된 후보별 gate 결과로 연결됩니다. GPU float32 core 집계는 Python 저장 행 집계와 별도로 보존합니다.', '',
        '[원본/공개 provenance](publication_manifest.json) · [공개 체크섬](SHA256SUMS) · [원본 원격 체크섬 참조](raw/original_remote_SHA256SUMS.txt)', '',
        '원본 원격 체크섬은 생략 PT와 비식별화 전 파일을 포함하므로 공개 tree의 검증 명령으로 쓰지 않습니다. 이 폴더의 SHA256SUMS만 공개 파일에 해당합니다. source b478과 외부 worker469b/core4716을 구분하며 기존 데이터·runtime gate·모델 allowlist·정답을 변경하거나 새 데이터를 승인하지 않았습니다.', '']
    lines += ['## 게시 도구 v1의 실패 보존', '',
        '첫 게시 도구는 211개 파일을 만든 뒤, 소스 안의 비밀키 검사 문자열 자체를 실제 키로 오인하여 마지막 개인정보 검증에 실패했습니다. 실제 비밀키 노출은 발견되지 않았으며 211개 원본과 210개 payload SHA를 모두 private 경로에 그대로 보존했습니다. 이 v2는 완전한 PEM private-key header를 검출하도록 수정하고 generic·RSA·OpenSSH·encrypted 헤더 거절 및 자체 소스 검증 회귀 테스트를 추가했습니다. 최초 게시를 성공으로 바꾸거나 학습·그림을 다시 생성한 실험으로 세지 않습니다.', '',
        '[최초 게시 실패 보존 receipt](publication_attempt_001_receipt.json). 이 receipt는 root가 반환 traceback을 기록한 설명 자료이며 raw console stream이 아닙니다. 최초 publisher SHA975b927c…와 보존 tree 전체 SHA는 provenance에 연결합니다.', '']
    return '\n'.join(lines).encode()


def verify_public(root):
    from PIL import Image
    hashes = audit.manifest((root / 'SHA256SUMS').read_bytes()); actual = audit.inventory(root)
    require(set(hashes) == set(actual) - {'SHA256SUMS'} and all(actual[n] == v for n, v in hashes.items()), 'public checksum inventory differs')
    links = pngs = 0
    for name in hashes:
        path = root / name
        if path.suffix == '.png':
            with Image.open(path) as image: require(image.format == 'PNG', 'PNG format changed'); image.load()
            pngs += 1; continue
        text = path.read_text()
        public_text(text)
        if path.suffix == '.md':
            for target in re.findall(r'\]\(([^)]+)\)', text):
                require(not target.startswith(('http:', 'https:', 'file:')) and (path.parent / target).resolve().is_file(), 'broken local publication link'); links += 1
    return dict(file_count=len(actual), payload_count=len(hashes), png_count=pngs, markdown_links=links,
        total_bytes=sum((root / n).stat().st_size for n in actual))


def publish():
    require(not OUTPUT.exists() and not any(p.is_symlink() for p in (OUTPUT, *OUTPUT.parents)), 'fresh results folder required')
    report, raw, launches, pins = authenticate(); own = audit.regular(Path(__file__)).read_bytes(); own_sha = sha(own)
    entries = {}
    for name, pin in raw.items():
        public = 'raw/' + ('original_remote_SHA256SUMS.txt' if name == 'SHA256SUMS' else name)
        entries[public] = copy_view(INPUT / name, OUTPUT / public, pin)
    for name, pin in launches.items(): entries['execution/' + name] = copy_view(PRIVATE / name, OUTPUT / 'execution' / name, pin)
    entries['independent_summary.json'] = copy_view(SUMMARY / 'summary.json', OUTPUT / 'independent_summary.json', SUMMARY_SHA)
    entries['original_summary_SHA256SUMS.txt'] = copy_view(SUMMARY / 'SHA256SUMS', OUTPUT / 'original_summary_SHA256SUMS.txt', SUMMARY_MANIFEST_SHA)
    entries['source/independent_summary.py'] = copy_view(Path(audit.__file__), OUTPUT / 'source/independent_summary.py', AUDITOR_SHA)
    entries['publication_attempt_001_receipt.json'] = copy_view(FAILED_RECEIPT, OUTPUT / 'publication_attempt_001_receipt.json', FAILED_RECEIPT_SHA)
    for seed in audit.SEEDS:
        for arm in VARIANTS: new_bytes(OUTPUT / 'galleries' / f'{seed}_{arm}.md', gallery(seed, arm))
    image, renderer = fixed_chart(report); new_bytes(OUTPUT / 'all_endpoints_and_hold_regression.png', image)
    new_bytes(OUTPUT / 'README.md', readme(report)); new_bytes(OUTPUT / 'source/publication.py', own)
    for name, value in audit.inventory(OUTPUT).items():
        if name not in entries: entries[name] = dict(public_sha256=value, publication_kind='generated_publication_evidence')
    recheck(pins); require(audit.inventory(INPUT) == raw and digest(Path(__file__)) == own_sha, 'source/input inventory changed during publication')
    provenance = dict(schema='portable_e2e.frozen_stopmix_selector_publication.v1', publisher_sha256=own_sha,
        independent_summary_sha256=SUMMARY_SHA, independent_summary_source_sha256=AUDITOR_SHA,
        original_input_sha256=pins, public_files=entries, renderer=renderer,
        all_six_head_histories_retained=True, all_nine_train_val_variants_retained=True, fixed_original_png_count=108,
        original_png_and_jsonl_bytes_unchanged=True, preserved_first_launcher_fit_count=0,
        tensors_transferred_or_loaded_by_publisher=False, omitted_remote_tensor_count=18,
        prior_publication_status='FINAL_VERIFICATION_FAILED_ORIGINALS_PRESERVED', prior_publication_receipt_sha256=FAILED_RECEIPT_SHA,
        prior_publication_source_sha256=ORIGINAL_PUBLISHER_SHA, prior_created_file_count=211,
        **{k: False for k in audit.DENIALS})
    new_bytes(OUTPUT / 'publication_manifest.json', encoded(provenance))
    final = audit.inventory(OUTPUT)
    new_bytes(OUTPUT / 'SHA256SUMS', ''.join(value + '  ' + name + '\n' for name, value in sorted(final.items())).encode())
    proof = verify_public(OUTPUT); require(proof['png_count'] == 109, 'all108 original PNG plus one numeric plot required')
    recheck(pins); require(audit.inventory(INPUT) == raw and digest(Path(__file__)) == own_sha, 'final source/input postcheck failed')
    return proof


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False); parser.add_argument('--verify-only', action='store_true')
    args = parser.parse_args(argv)
    if args.verify_only:
        report, raw, launches, pins = authenticate(); recheck(pins)
        print(json.dumps(dict(status='VERIFIED_NO_WRITES', collected_files=len(raw), execution_receipts=len(launches),
            saved_rows=report['total_analysis_rows'], fixed_png_count=report['fixed_val_png_count']))); return
    print(json.dumps(publish(), sort_keys=True))


if __name__ == '__main__': main()
