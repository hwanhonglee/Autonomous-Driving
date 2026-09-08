#!/usr/bin/env python3
"""HH_260906 - Publish all fixed ten-epoch continuation outcomes from immutable local evidence without model execution."""

import argparse
from datetime import datetime, timezone
import hashlib
import ipaddress
import json
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from PIL import Image
from scripts.e2e import summarize_portable_stopmix_duration as audit
from scripts.e2e.curate_independent_common10_capture_20260907 import sanitize

PRIVATE = ROOT / 'artifacts/training/2026-09-09'
CAMPAIGN = PRIVATE / 'stopmix_duration_v1'
SUMMARY = PRIVATE / 'stopmix_duration_summary_v1'
PUBLIC = ROOT / 'docs/assets/validation/2026-09-09/portable_e2e_learning_cycle_v1/02_overnight_data_and_learning/08_stopmix_ten_epoch_continuation'
OUTPUT = PUBLIC / 'results'
SOURCE = 'b478f02e42b94bf04bffec5c8170e05edc33b0f8'
WORKER_SHA = 'e51d2c4e02ba15a24a376944c08a387d616b8d749585562732ee483b913958e3'
PLAN_SHA = 'bcdf31f09838750cd24fcabf063a9baeac8fb317ed045e6a41d734fff2e6233b'
AUDITOR_SHA = 'eaec7b75a7f3a438b80439b28e035b302e7696322e4a554f2167502611ea6e42'
PINS = {
    'stopmix_duration_summary_v1/summary.json': '85493ccf7c3d8365025c2d55c26e8911b34cf2ad3e35e6018c6603e1be8e3545',
    'stopmix_duration_collection_receipt_v1.json': '1761b9282fde94a3b2a81b960c606444221a0d735ef9697c204bfe7e7b90182a',
    'stopmix_duration_completion_before_v1.json': 'ed350c1843030f57ca7ed162f0750c713c8802ae3dbc7b4deb70513e2745358f',
    'stopmix_duration_completion_after_v1.json': '1c7e27ccf1218caf11b319b1e719932033205ed8590b1160e286f282872f6f9f',
    'stopmix_duration_v1/WORKFLOW_SHA256SUMS': 'a16779b84b44621398eb4865559d0bffcbbb4117d2a3aea550dd9556c7b9fa5d',
    'stopmix_duration_publication_attempt_001_failure.json': 'e5b2b900b6c1b0dea4cb0c69aa2bd482560a17e895e3fe748e1f2ed028c50374',
    'stopmix_duration_publication_attempt_001_incomplete/source/publish_stopmix_duration.py': '03a8eb527eae6da3a0b34a6482a263119f77771eb480201e70ac7222d34a8f4f',
}
RUNS = tuple(f'seed_{seed}/{arm}' for seed in (20260903, 20260904, 20260905) for arm in ('A_physical_drive', 'B_drive_stop_mix'))
INDICES = (0, 31, 61, 92, 122, 153, 183, 214, 244, 275, 305, 336)
NOTICE = 'Public metadata view with account paths/GPU UUIDs redacted. raw_source_sha256 binds the unchanged private original; raw inventories do not describe this derivative public tree.'


def require(value, message):
    if not value: raise ValueError(message)


def digest(path):
    path = Path(path).absolute()
    require(path.is_file() and not any(p.is_symlink() for p in (path, *path.parents)), 'regular nonsymlink input required')
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    return audit.read(path)


def redact(value):
    if isinstance(value, str):
        value = sanitize(value)
        value = re.sub(r'/(?:tmp|root)(?=/|$|[\s\"\'])', lambda _: '${PRIVATE_ROOT}', value)
        return re.sub(r'(?<![0-9A-Za-z])(?:GPU-)?[0-9a-fA-F]{8}(?:-[0-9a-fA-F]{4}){3}-[0-9a-fA-F]{12}(?![0-9A-Za-z])', '${GPU0_UUID}', value)
    if isinstance(value, list): return [redact(v) for v in value]
    if isinstance(value, dict): return {redact(k): redact(v) for k, v in value.items()}
    return value


def json_bytes(value):
    return (json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + '\n').encode()


def new_bytes(path, value):
    require(not path.exists() and not any(p.is_symlink() for p in (path, *path.parents)), 'refuse existing/unsafe publication path')
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('xb') as stream: stream.write(value)


def image_proof(path):
    with Image.open(path) as image:
        require(image.format == 'PNG', 'only actual PNG evidence accepted')
        image.load()
        return dict(width=image.width, height=image.height, mode=image.mode)


def copy_view(source, destination):
    before = digest(source); raw = source.read_bytes(); image = None; kind = 'original_bytes'
    require(hashlib.sha256(raw).hexdigest() == before, 'source read bytes changed before copy')
    if source.suffix == '.png':
        image = image_proof(source); payload = raw
    elif source.suffix == '.json':
        value = audit.old._loads_json(raw.decode('utf-8'), 'verified publication JSON bytes')
        require(isinstance(value, dict) and not {'raw_source_sha256', 'publication_notice'} & set(value), 'reserved metadata keys')
        value = redact(value); value.update(raw_source_sha256=before, publication_notice=NOTICE)
        payload = json_bytes(value); kind = 'redacted_metadata_view'
    else:
        text = raw.decode('utf-8'); payload = redact(text).encode()
        if payload != raw: kind = 'redacted_source_reference_not_executable' if source.suffix == '.py' else 'redacted_text_view'
        if source.name in ('SHA256SUMS', 'WORKFLOW_SHA256SUMS'): kind = 'original_private_checksum_reference_not_public_manifest'
    new_bytes(destination, payload)
    require(digest(source) == before, 'source changed during publication')
    if image: require(image_proof(destination) == image, 'PNG copy decode differs')
    return dict(raw_private_source=source.relative_to(ROOT).as_posix(), raw_source_sha256=before,
        public_sha256=digest(destination), publication_kind=kind, image=image)


def relative_path(name):
    require(isinstance(name, str) and name and not Path(name).is_absolute() and '..' not in Path(name).parts
        and Path(name).as_posix() == name, 'unsafe relative inventory path')
    return name


def tree_pins(root):
    return {path.relative_to(root).as_posix(): digest(path) for path in sorted(root.rglob('*')) if path.is_file() or path.is_symlink()}


def verify_public_manifest(root):
    rows = {}
    for line in (root / 'SHA256SUMS').read_text().splitlines():
        value, name = line.split('  '); relative_path(name)
        require(name not in rows and name != 'SHA256SUMS' and re.fullmatch('[0-9a-f]{64}', value), 'invalid public manifest')
        require(digest(root / name) == value, 'public payload changed'); rows[name] = value
    require(set(rows) == set(tree_pins(root)) - {'SHA256SUMS'}, 'public inventory incomplete')
    return rows


def validate_collection(before, after, receipt, actual):
    require(before['regular_file_inventory'] == after['regular_file_inventory'], 'remote before/after differ')
    inventory = before['regular_file_inventory']; checkpoints = before['checkpoint_names']
    expected = [run + '/training/checkpoints/latest.pt' for run in RUNS]
    require(checkpoints == expected and len(inventory) == 232 and len(actual) == 226, '232/226/6 inventory differs')
    require(set(actual) == set(inventory) - set(checkpoints), 'only six remote checkpoints may be absent locally')
    for name, value in actual.items():
        relative_path(name); require(value == inventory[name]['sha256'], 'local mirror SHA differs')
    require(receipt['remote_before_after_inventory_exact'] is True and receipt['local_payloads_exact'] is True
        and receipt['local_checkpoint_tensors_copied'] is False and receipt['checkpoint_tensors_loaded'] is False
        and receipt['copied_regular_files'] == 226 and receipt['omitted_checkpoint_count'] == 6
        and receipt['plan_sha256'] == PLAN_SHA and receipt['source_commit'] == SOURCE, 'collection scope differs')
    require(receipt['remote_checkpoint_sha256'] == {name: inventory[name]['sha256'] for name in checkpoints}, 'checkpoint remote SHA receipt differs')
    return inventory


def check_rows(root, report):
    require([r['run'] for r in report['lineages']] == list(RUNS), 'all six fixed lineages required')
    counts = dict(training_rows=0, inherited_training_rows=0, new_training_rows=0, behavior_rows=0)
    for item in report['lineages']:
        run = item['run']; folder = root / run
        raw = (folder / 'training/metrics.jsonl').read_bytes()
        require(raw.endswith(b'\n'), 'truncated training history')
        lines = raw.splitlines(keepends=True)
        require(len(lines) == 2870 and hashlib.sha256(raw).hexdigest() == item['continued']['history']['sha256'], 'full2870 history binding differs')
        require(hashlib.sha256(b''.join(lines[:1540])).hexdigest() == item['continued']['history']['parent_prefix_sha256'], 'inherited1540 prefix differs')
        history = [json.loads(line) for line in lines]
        require([r['global_step'] for r in history] == list(range(1,2871)) and history[1539]['samples_seen'] == 6155
            and history[-1]['samples_seen'] == 11470, 'training history budget differs')
        samples_raw = (folder / 'behavior/samples.jsonl').read_bytes()
        require(samples_raw.endswith(b'\n'), 'truncated behavior rows')
        samples = [json.loads(line) for line in samples_raw.splitlines()]
        require(len(samples) == 337 and item['continued']['behavior']['sample_count'] == 337, 'all337 behavior rows required')
        expected = {f'val_{index:03d}.png' for index in INDICES}
        require({p.name for p in (folder/'behavior/trajectories').iterdir()} == expected, 'twelve fixed frames differ')
        counts['training_rows'] += 2870; counts['inherited_training_rows'] += 1540
        counts['new_training_rows'] += 1330; counts['behavior_rows'] += len(samples)
    return counts


def bind_inputs():
    for name, value in PINS.items(): require(digest(PRIVATE / name) == value, 'frozen publication input differs: ' + name)
    require(digest(Path(audit.__file__)) == AUDITOR_SHA and digest(PUBLIC / 'plan.json') == PLAN_SHA, 'reader/public plan changed')
    before = read(PRIVATE/'stopmix_duration_completion_before_v1.json'); after = read(PRIVATE/'stopmix_duration_completion_after_v1.json')
    receipt = read(PRIVATE/'stopmix_duration_collection_receipt_v1.json'); actual = tree_pins(CAMPAIGN)
    inventory = validate_collection(before, after, receipt, actual)
    require(sum((CAMPAIGN/name).stat().st_size for name in actual) == receipt['copied_bytes'] == 21723125, 'local bytes differ')
    for name in actual: require((CAMPAIGN/name).stat().st_size == inventory[name]['size_bytes'], 'individual mirror size differs')
    report = read(SUMMARY/'summary.json')
    fresh = audit.summarize_campaign(CAMPAIGN, PRIVATE/'stopmix_ab_v1', PRIVATE/'stopmix_behavior_v1')
    expected = dict(report); rendering = expected.pop('rendering')
    require(audit.canonical(fresh) == audit.canonical(expected), 'strict fixed-lineage summary no longer reproduces')
    require(report['status'] == 'COMPLETE_NOT_PROMOTED' and report['completed_stage_count'] == 24
        and report['completed_lineage_count'] == 6 and report['checkpoint_tensors_loaded'] is False
        and report['source_commit'] == SOURCE and report['worker_sha256'] == WORKER_SHA, 'summary completion/source scope differs')
    require(rendering['learning_history_points'] == 17220 and rendering['files'] == ['01_continued_learning_curves.png','02_fixed_endpoint_validation.png'], 'actual plot denominator differs')
    audit.manifest(SUMMARY, 'SHA256SUMS')
    counts = check_rows(CAMPAIGN, report)
    pins = {str((CAMPAIGN/name).relative_to(ROOT)): value for name, value in actual.items()}
    pins.update({str((PRIVATE/name).relative_to(ROOT)): value for name,value in PINS.items()})
    pins.update({str(p.relative_to(ROOT)): digest(p) for p in SUMMARY.iterdir()})
    for category, root in (('parent',PRIVATE/'stopmix_ab_v1'), ('parent_behavior',PRIVATE/'stopmix_behavior_v1')):
        pins.update({str((root/r['path']).relative_to(ROOT)): r['sha256'] for r in report['input_manifest'][category]})
    for name, value in report['reader_source_sha256'].items(): pins[name] = value
    pins['scripts/e2e/curate_independent_common10_capture_20260907.py'] = digest(ROOT/'scripts/e2e/curate_independent_common10_capture_20260907.py')
    require(all(digest(ROOT/name) == value for name,value in pins.items()), 'bound input or dependency differs')
    return report, pins, counts


def selected_files():
    selected = []; omitted = []
    for source in sorted(CAMPAIGN.rglob('*')):
        if not source.is_file(): continue
        relative = source.relative_to(CAMPAIGN)
        if source.suffix == '.png' and 'evaluation/trajectories/' in relative.as_posix():
            omitted.append(relative.as_posix()); continue
        target = Path('campaign')/relative
        if source.name in ('SHA256SUMS','WORKFLOW_SHA256SUMS'): target = target.with_name(source.name+'.private_reference.txt')
        if source.suffix == '.py': target = target.with_name(source.name+'.redacted_reference.txt')
        selected.append((source,target))
    require(len(omitted) == 72 and len(selected) == 154, 'only72 default evaluation PNGs may be omitted from226 originals')
    for source in SUMMARY.iterdir():
        target = Path('plots')/source.name if source.suffix == '.png' else Path('audit')/source.name
        if source.name == 'SHA256SUMS': target = target.with_name('SHA256SUMS.private_reference.txt')
        selected.append((source,target))
    for name, target in (('before','completion_before.json'),('after','completion_after.json')):
        selected.append((PRIVATE/f'stopmix_duration_completion_{name}_v1.json',Path('receipts')/target))
    selected.append((PRIVATE/'stopmix_duration_collection_receipt_v1.json',Path('receipts/collection.json')))
    selected.append((PRIVATE/'stopmix_duration_publication_attempt_001_failure.json',Path('receipts/publication_attempt_001_failure.json')))
    selected.append((Path(__file__).resolve(),Path('source/publish_stopmix_duration.py')))
    return selected, omitted


def gallery(item):
    folder = CAMPAIGN/item['run']/'behavior'; rows = [json.loads(line) for line in (folder/'samples.jsonl').read_text().splitlines()]
    text = [f"# {item['run']} · 사전에 고정한 12개 validation 예측", '',
        '<!-- HH_260906 - Retain every fixed validation index and all337 records; do not select favorable frames. -->', '',
        '실제 차량·CARLA 폐루프·Autoware 화면이 아닙니다. 저장된 337개 validation 중 사전 고정 인덱스의 오프라인 모델 예측입니다.', '',
        '[전체337개 기록](samples.jsonl) · [행동 요약](summary.json)', '']
    for index in INDICES:
        row = rows[index]
        text += [f"## Val {index:03d} · {row['target_motion_group']}", '',
            f"선택 후보 {row['selected_candidate_index']} · ADE {row['selected_ade_m']:.4f} m · 기존 기하 검사 {'PASS' if row['runtime_geometry']['selected_geometry_pass'] else 'FAIL'}", '',
            f"![저장된 예측 Val {index:03d}](trajectories/val_{index:03d}.png)", '']
    return '\n'.join(text)+'\n'


def readme(report, counts):
    relative = sum(all(r['relative_checks'].values()) for r in report['lineages'])
    absolute = sum(all(r['absolute_checks'].values()) for r in report['lineages'])
    worse = {key:sum(r['continued']['metrics'][key] > r['parent']['metrics'][key] for r in report['lineages']) for key in ('ade_1p0s_m','ade_3p0s_m')}
    lines = ['# 실제 결과 — 6개 기존 모델의 고정 10 epoch 이어학습', '',
        '<!-- HH_260906 - Report all fixed endpoints, inherited updates and short-horizon regressions without promotion. -->', '',
        f"24/24 단계와 6/6 계보가 완료되었습니다. 상대조건은 {relative}/6, 절대조건은 {absolute}/6 통과입니다. 자동 승격·새 데이터 승인·차량 제어 승인은 없습니다.", '',
        '새 모델6개를 처음부터 학습한 것이 아닙니다. 각 부모1540 step/6155회 노출에서1330 step/5315회를 추가해 총2870 step/11470회, 정확히10 epoch까지 이어갔습니다. 데이터는 기존 train1147개·validation337개 그대로입니다.', '',
        f"공개 이력은 총{counts['training_rows']:,}행입니다. 이 중{counts['inherited_training_rows']:,}행은 복사한 부모 이력이고, {counts['new_training_rows']:,}행만 이번 새 update입니다. 행동 기록은6×337={counts['behavior_rows']:,}행 전부입니다.", '',
        f"중요: 1초 ADE는 {worse['ade_1p0s_m']}/6, 3초 ADE는 {worse['ade_3p0s_m']}/6 계보에서 오히려 증가했습니다. 일부 전체6.4초 ADE가 개선되어도 짧은 구간의 성능 향상을 의미하지 않습니다.", '',
        '| 계보 | 6.4초 ADE 이전→현재 m | 1초 ADE 이전→현재 m | 3초 ADE 이전→현재 m | 기하 PASS /337 | 상대조건 |',
        '| --- | ---: | ---: | ---: | ---: | --- |']
    for row in report['lineages']:
        a,b=row['parent'],row['continued']; am,bm=a['metrics'],b['metrics']
        lines.append(f"| [{row['run']}](campaign/{row['run']}/behavior/README.md) | {am['selected_ade_m']:.4f}→{bm['selected_ade_m']:.4f} | {am['ade_1p0s_m']:.4f}→{bm['ade_1p0s_m']:.4f} | {am['ade_3p0s_m']:.4f}→{bm['ade_3p0s_m']:.4f} | {a['geometry']['selected_pass_count']}→{b['geometry']['selected_pass_count']} | {all(row['relative_checks'].values())} |")
    lines += ['', '## 실제 저장 수치와 그림', '',
        '![전체 학습 이력](plots/01_continued_learning_curves.png)', '',
        '회색은 부모1–1540 update, 이후가 이번 추가 학습입니다. 전체 batch 점을 보존했고50-update 평균은 샘플 수로 가중했습니다. 1501–1550 bin은 부모40개와 새10개가 섞이며 마지막 bin은20개입니다.', '',
        '![고정 종점 validation 비교](plots/02_fixed_endpoint_validation.png)', '',
        '[엄격한24단계 감사](audit/summary.json) · [상태와 모든 명령](campaign/status.json) · [전송 검증](receipts/collection.json) · [공개파일/원본SHA 대응](publication_manifest.json) · [공개SHA256SUMS](SHA256SUMS)', '',
        '[원래1540-step 결과](../../04_stopmix_learning/README.md) · [이번 사전 계획](../plan.json)', '',
        '## 보존·비공개·안전 경계', '',
        '- 6개 계보마다 사전 고정12장, 총72개의 실제 저장 행동 PNG를 모두 표시합니다. 이것은 오프라인 예측이며 자율주행 촬영이나 실차 검증이 아닙니다.',
        '- 별도 수치 그래프2장도 실제 저장 지표에서 생성한 원본 PNG입니다. 초기12개만 고른 일반 evaluation PNG72장은 private 원본에 그대로 보존하고 공개 갤러리에서는 중복을 제외했습니다.',
        '- 원격232개 파일은 전후 해시가 같았고, 그중checkpoint6개를 제외한226개·21,723,125byte를 로컬에서 확인했습니다. checkpoint는 원격SHA 영수증만 제공하며 로컬 복사·tensor load를 하지 않았습니다.',
        '- 모든 stage log/metric, 원격 실행 worker, 원본 체크섬 참조를 포함합니다. `*.private_reference.txt`는 private 원본용 체크섬이며 공개파일 검증에는 이 폴더 최상위SHA256SUMS를 사용하세요. 0-byte run-lock6개도 감사용 원본 inventory로 보존했습니다.',
        '- JSON은 계정경로·GPU UUID를 가린 metadata view이며 원본 SHA를 별도 기록합니다. worker의 redacted reference는 실행 가능한 원본과 동일한 코드가 아닙니다. 원본 실행 source b478, worker e51의 SHA와 공개 source를 혼동하지 마세요.',
        '- 기존 모델·loss·학습률·sampling·gate는 그대로입니다. 후보 수/모델 용량이 다른 A6와B12의 전체 loss를 같은 난이도의 절대 점수로 해석하지 않습니다. 더 오래 학습한 결과만 비교하며 최적 epoch/seed를 고르지 않았습니다.',
        '- 기존 v3의 label/저속 문제는 해결되거나 승인된 것이 아닙니다. STOP 후보 학습은 실제 정지 의도·차선변경·장애물 회피 능력 검증이 아닙니다. 새 C-track 원본8회도 학습 데이터로 승인하지 않았습니다.',
        '- test 신경망 추론·최적화·모델 선택은 하지 않았습니다. 변함없는 전체 corpus 무결성 검사에서 test metadata/JPEG를 읽을 수는 있습니다.', '']
    lines += ['첫 공개 생성은 최종 SHA 파일을 만들기 전 README 링크를 검사하는 순서 문제로 실패했습니다. [실패 기록](receipts/publication_attempt_001_failure.json)과 원래 생성물171개는 private에 보존했고, 검증 순서만 수정했습니다. 재학습·재추론·이미지 재생성은 하지 않았습니다.', '']
    return '\n'.join(lines)


def privacy_and_links(root):
    for path in root.rglob('*'):
        if not path.is_file() or path.suffix == '.png': continue
        text = path.read_text()
        require(not re.search(r'/(?:home/[^/\s]+|tmp|root)/', text), 'account/private absolute path in publication')
        require(not re.search(r'GPU-[0-9a-fA-F]{8}-', text), 'raw GPU UUID in publication')
        for candidate in re.findall(r'(?<![\d.])(?:\d{1,3}\.){3}\d{1,3}(?![\d.])',text):
            try: ipaddress.IPv4Address(candidate)
            except ipaddress.AddressValueError: continue
            raise ValueError('private IPv4 in publication')
        if path.suffix == '.md':
            for target in re.findall(r'\]\(([^)]+)\)',text):
                require(not target.startswith(('http:','https:','file:')) and (path.parent/target).resolve().is_file(), 'broken local link')


def finish_publication(output):
    # HH_260906 - Create the real manifest before checking README links to it; no future-file exception is permitted.
    files=tree_pins(output)
    new_bytes(output/'SHA256SUMS',''.join(value+'  '+name+'\n' for name,value in sorted(files.items())).encode())
    privacy_and_links(output); verify_public_manifest(output)
    return len(files)+1


def publish():
    require(not OUTPUT.exists() and not any(p.is_symlink() for p in (OUTPUT,*OUTPUT.parents)), 'create-only results already exist')
    report,pins,counts=bind_inputs(); publisher_sha=digest(Path(__file__).resolve()); selected,omitted=selected_files()
    require(len({str(target) for _,target in selected})==len(selected),'duplicate publication destination')
    OUTPUT.mkdir(parents=True,exist_ok=False); entries={}
    for source,target in selected: entries[target.as_posix()]=copy_view(source,OUTPUT/target)
    for row in report['lineages']:
        target=Path('campaign')/row['run']/'behavior/README.md'; new_bytes(OUTPUT/target,gallery(row).encode())
        source=CAMPAIGN/row['run']/'behavior/samples.jsonl'
        entries[target.as_posix()]=dict(public_sha256=digest(OUTPUT/target),publication_kind='all_fixed_indices_gallery',raw_private_source=source.relative_to(ROOT).as_posix(),raw_source_sha256=digest(source))
    new_bytes(OUTPUT/'README.md',readme(report,counts).encode())
    entries['README.md']=dict(public_sha256=digest(OUTPUT/'README.md'),publication_kind='generated_complete_outcomes_summary')
    require(sum(name.endswith('.png') for name in entries)==74,'all72 prediction and2 plot PNGs required')
    require(all(digest(ROOT/name)==value for name,value in pins.items()) and digest(Path(__file__).resolve())==publisher_sha,'original source/input changed')
    require(digest(PUBLIC/'plan.json')==PLAN_SHA,'parent plan changed')
    provenance=dict(schema='portable_e2e.stopmix_duration_publication.v1',published_at_utc=datetime.now(timezone.utc).isoformat(),
        raw_training_source_commit=SOURCE,raw_worker_sha256=WORKER_SHA,raw_plan_sha256=PLAN_SHA,
        strict_summary_sha256=PINS['stopmix_duration_summary_v1/summary.json'],posthoc_auditor_sha256=AUDITOR_SHA,
        publication_worker_sha256=publisher_sha,publication_worker_source_commit=None,
        input_sha256=pins,public_files=entries,counts=counts,remote_regular_files=232,local_mirrored_regular_files=226,
        remote_only_checkpoint_count=6,checkpoint_tensors_copied_or_loaded=False,
        omitted_default_evaluation_pngs_preserved_private=omitted,published_fixed_behavior_pngs=72,published_numeric_plots=2,
        metadata_notice=NOTICE,automatic_promotion=False,training_data_approved=False,vehicle_control_approved=False)
    new_bytes(OUTPUT/'publication_manifest.json',json_bytes(redact(provenance)))
    count=finish_publication(OUTPUT)
    print(json.dumps(dict(status='PUBLISHED_NOT_PROMOTED',files=count,images=74,counts=counts,
        public_manifest_sha256=digest(OUTPUT/'publication_manifest.json'))))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False);parser.add_argument('--verify-only',action='store_true');args=parser.parse_args()
    if args.verify_only:
        report,pins,counts=bind_inputs();print(json.dumps(dict(status='BOUND_NO_PUBLIC_WRITES',input_files=len(pins),counts=counts)))
    else: publish()
