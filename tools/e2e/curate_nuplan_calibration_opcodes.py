#!/usr/bin/env python3
"""HH_260906 - Publish a verified static opcode inventory without exporting private calibration BLOBs."""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import json
from pathlib import Path
import re

from scripts.e2e import audit_nuplan_calibration_opcodes as audit
from scripts.e2e.curate_independent_common10_capture_20260907 import sanitize

SOURCE_SHA = 'f7b5640703914ec7042d68817f95dd62737205f9ac6f1e006f3e08188cb13b58'
PLAN_SHA = 'e76494acccb84a64934b34be91e190c7703b490e39c7a5791c4f977d58014b92'
REPO = Path(__file__).resolve().parents[2]
SCHEMA = 'nuplan.calibration_opcode_publication.v1'


def encoded(value):
    return (json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + '\n').encode()


def checked(path, pins, expected=None):
    raw = audit.read_small(path, expected)
    digest = audit.sha(raw)
    audit.require(path not in pins or pins[path] == digest, 'input changed during publication')
    pins[path] = digest
    return raw


def tree_files(root):
    result = set()
    for path in root.rglob('*'):
        audit.require(not path.is_symlink(), 'symlink in input inventory')
        if path.is_dir():
            continue
        audit.regular(path)
        result.add(path.relative_to(root).as_posix())
    return result


def manifest_entries(raw):
    entries = {}
    for line in raw.decode().splitlines():
        match = re.fullmatch(r'([0-9a-f]{64})  ([A-Za-z0-9_./-]+)', line)
        audit.require(match is not None, 'invalid checksum entry')
        digest, name = match.groups()
        audit.require(not Path(name).is_absolute() and all(p not in ('', '.', '..') for p in name.split('/'))
            and name not in entries and name != 'SHA256SUMS', 'unsafe or duplicate checksum entry')
        entries[name] = digest
    return entries


def bind_inputs(root, prior_path, report_sha, checksums_sha):
    # HH_260906 - The externally observed report and manifest hashes bind all copied private inputs before any public write.
    pins = {}
    for value in (report_sha, checksums_sha):
        audit.require(re.fullmatch('[0-9a-f]{64}', value or ''), 'external expected SHA256 required')
    source_path = Path(audit.__file__).absolute()
    checked(source_path, pins, SOURCE_SHA)
    checked(Path(__file__).absolute(), pins)
    checked(Path(__import__(sanitize.__module__, fromlist=['__file__']).__file__).absolute(), pins)
    prior = audit.json_load(checked(prior_path, pins, audit.PRIOR_SHA))
    raw_report = checked(root/'report.json', pins, report_sha)
    report = audit.json_load(raw_report)
    plan_raw = checked(root/'followup_plan.json', pins, PLAN_SHA)
    plan = audit.json_load(plan_raw)
    expected_plan = audit.build_plan(prior, declared_at_utc=plan.get('declared_at_utc'), source_sha256=SOURCE_SHA)
    audit.require(encoded(plan) == encoded(expected_plan), 'follow-up plan scope mismatch')
    checked(root/'source_executed.py', pins, SOURCE_SHA)
    raw_manifest = checked(root/'SHA256SUMS', pins, checksums_sha)
    entries = manifest_entries(raw_manifest)
    blob_names = {'opaque_blobs/'+c+'/'+f+'.bin' for c in audit.CHANNELS for f in audit.FIELDS}
    expected_names = blob_names | {'report.json', 'followup_plan.json', 'source_executed.py'}
    audit.require(set(entries) == expected_names and tree_files(root) == expected_names | {'SHA256SUMS'}, 'private inventory must contain exactly 36 original files')
    for name, digest in entries.items():
        checked(root/name, pins, digest)
    audit.require(report.get('schema') == audit.SCHEMA and report.get('status') in (
        'OPCODE_INVENTORY_COMPLETE_NOT_DECODED', 'OPCODE_INVENTORY_COMPLETE_WITH_PARSE_REJECTIONS'), 'incomplete or failed worker evidence retained privately')
    fixed = dict(source_sha256=SOURCE_SHA, followup_plan_sha256=PLAN_SHA, previous_report_sha256=audit.PRIOR_SHA,
        original_selection_plan_sha256=prior['selection_plan_sha256'], member_sha256=audit.MEMBER_SHA,
        database_payloads_opened=1, opaque_calibration_blob_copies_written=32, camera_payloads_read=0,
        lidar_payloads_read=0, map_payloads_read=0, database_written_to_disk=False,
        numeric_calibration_extracted=False, pickle_vm_or_constructors_executed=False, primitive_interpreter_implemented=False,
        archive_whole_content_rehashed_this_run=False, database_decompressed_in_memory=True, member_crc_verified_to_eof=True,
        finish_deadline_met=True, missing_calibration_fields=[], postcheck_errors=[], **audit.DENIALS)
    for key, value in fixed.items():
        audit.require(encoded(report.get(key)) == encoded(value), 'worker field mismatch: '+key)
    audit.require(report.get('archive_pre_identity') == plan['archive_identity']
        and report.get('previous_archive_content_sha256') == plan['previous_archive_content_sha256'], 'archive identity proof mismatch')
    declared, started, finished, deadline = [datetime.fromisoformat(value.replace('Z', '+00:00')) for value in (
        plan['declared_at_utc'], report['started_at_utc'], report['completed_at_utc'], plan['finish_before_utc'])]
    audit.require(all(t.tzinfo is not None and t.utcoffset().total_seconds() == 0 for t in (declared, started, finished, deadline))
        and declared <= started <= finished < deadline, 'execution chronology mismatch')
    rows = report.get('blob_reports', [])
    audit.require(len(rows) == 32, 'all 32 inventory rows required')
    seen = set()
    for row in rows:
        key = row['channel']+'/'+row['field']
        audit.require(key in plan['calibration_blob_pins'] and key not in seen, 'duplicate or unknown calibration field')
        seen.add(key)
        name = 'opaque_blobs/'+key+'.bin'
        blob = checked(root/name, pins, plan['calibration_blob_pins'][key]['sha256'])
        audit.require(len(blob) == plan['calibration_blob_pins'][key]['size_bytes'], 'original BLOB size mismatch')
        expected = dict(channel=row['channel'], field=row['field'], retained_private_blob=name, **audit.inspect_pickle_opcodes(blob))
        audit.require(encoded(row) == encoded(expected), 'static opcode inventory differs from retained private bytes')
    rejected = sum(row['parse_status'] == 'REJECTED' for row in rows)
    audit.require(type(report.get('parse_rejection_count')) is int and report['parse_rejection_count'] == rejected
        and report['status'] == ('OPCODE_INVENTORY_COMPLETE_WITH_PARSE_REJECTIONS' if rejected else 'OPCODE_INVENTORY_COMPLETE_NOT_DECODED'), 'parse rejection denominator mismatch')
    return report, plan, pins, raw_manifest


def aggregate(report):
    rows = report['blob_reports']; totals = Counter()
    for row in rows:
        totals.update(row['opcode_counts'])
    fields = []
    for field in audit.FIELDS:
        group = [row for row in rows if row['field'] == field]
        declarations = sorted({(entry['global_declaration'].get('module'), entry['global_declaration'].get('symbol'))
            for row in group for entry in row['opcodes'] if entry.get('global_declaration', {}).get('module')})
        fields.append(dict(field=field, blob_count=len(group), total_bytes=sum(row['size_bytes'] for row in group),
            parsed_encoding_count=sum(row['parse_status'] == 'PARSED_ENCODING_ONLY' for row in group),
            dynamic_opcode_count=sum(row['dynamic_opcode_count'] for row in group),
            lexical_global_declarations_not_resolved=[dict(module=m, symbol=s) for m,s in declarations]))
    return dict(schema='nuplan.calibration_opcode_aggregate.v1', blob_count=len(rows), camera_count=8,
        total_private_blob_bytes=sum(row['size_bytes'] for row in rows), parse_rejection_count=report['parse_rejection_count'],
        numeric_calibration_extracted=False, pickle_vm_or_constructors_executed=False,
        trusted_intrinsics_or_extrinsics=False, training_data_approved=False,
        opcode_counts=dict(sorted(totals.items())), fields=fields)


def privacy(raw):
    text = raw.decode()
    audit.require(not re.search(r'/home/[^/\s]+|/root/|GPU-[0-9a-f-]{16,}|(?<![\w.])(?:\d{1,3}\.){3}\d{1,3}(?![\w.])', text), 'private account path, GPU UUID or network address in publication')


def readme(report, stats):
    lines = ['# nuPlan 카메라 calibration: 정적 opcode 목록', '',
        '<!-- HH_260906 - Static metadata inventory is not trusted calibration, deserialization, dataset admission or model training. -->', '',
        '기존 계획과 별도로 사전 고정한 후속 검사입니다. 같은 DB 멤버 1개를 메모리에서 압축 해제하고 전체 SHA를 확인했습니다. DB 파일을 디스크에 풀지 않았으며 이미지·LiDAR·지도 payload는 읽지 않았습니다.', '',
        f"카메라 8개 × 필드 4개 = **32개 BLOB**({stats['total_private_blob_bytes']:,} bytes)을 모두 검사했습니다. 인코딩 파싱 거부는 {stats['parse_rejection_count']}개입니다. `GLOBAL`·`REDUCE` 등은 이름/위치만 기록했으며 함수나 생성자를 실행하지 않았습니다.", '',
        '| 필드 | BLOB 수 | bytes 합계 | 인코딩 파싱 | 실행 의미 미해결 opcode 수 |',
        '|---|---:|---:|---:|---:|']
    for row in stats['fields']:
        lines.append(f"| {row['field']} | {row['blob_count']} | {row['total_bytes']} | {row['parsed_encoding_count']}/8 | {row['dynamic_opcode_count']} |")
    lines += ['', '**이 결과는 안전하게 해석된 숫자 calibration이 아닙니다.** opcode의 숫자·문자 인자는 인코딩 관찰일 뿐 K/TF/회전·왜곡 값으로 검증되지 않았습니다. 스택·참조·순환 구조도 미해결이며 모든 필드는 `UNRESOLVED_INVENTORY_ONLY`입니다. 모델 학습·변환·데이터 이용약관 승인·차량 적용은 하지 않았습니다.', '',
        '32개 원본 `.bin`은 비공개 진단 폴더에만 보존하며 여기에는 복사하지 않습니다. 공개 JSON은 기계별 경로를 치환할 수 있는 metadata view이고 원본 및 공개 SHA를 각각 보존합니다. 원본 전체 ZIP은 이번에 다시 해시하지 않았습니다: ZIP stat 일치와 과거 ZIP 해시를 기록하고, 이번 DB 멤버 전체 및 32 BLOB 해시는 직접 확인했습니다.', '',
        '- [전체 opcode 관찰](report.json)', '- [사전 고정 후속 계획](followup_plan.json)',
        '- [필드별 집계](opcode_summary.json)', '- [실행된 원본 검사 코드](execution_source.py)',
        '- [비공개 원본 파일의 해시 목록](original_SHA256SUMS.txt)', '- [원본→공개 SHA 및 재검증 기록](publication_manifest.json)', '',
        '수집·공개 helper는 로컬 비공개 BLOB을 같은 정적 parser로 다시 확인했습니다(DB 재열기 없음). 재실행에는 비공개 원본 36개 파일, 이전 report, 고정된 검사 소스와 별도의 새 출력 폴더가 필요합니다. `git clone`만으로 원본 데이터나 이용 권한이 생기지는 않습니다.', '',
        f"원본 실행: `{report['started_at_utc']}` → `{report['completed_at_utc']}`; 상태 `{report['status']}`.", '']
    return '\n'.join(lines).encode()


def publish(root, prior_path, output, *, report_sha, checksums_sha):
    root, prior_path, output = (Path(p).absolute() for p in (root, prior_path, output))
    audit.require(not output.exists() and not output.is_symlink() and not any(p.is_symlink() for p in output.parents)
        and not output.is_relative_to(root) and not root.is_relative_to(output)
        and not output.is_relative_to(prior_path.parent) and not prior_path.parent.is_relative_to(output), 'fresh separate publication directory required')
    datasets = REPO/'datasets'
    audit.require(not output.resolve().is_relative_to(datasets.resolve()), 'no dataset output permitted')
    report, plan, pins, raw_manifest = bind_inputs(root, prior_path, report_sha, checksums_sha)
    stats = aggregate(report)
    payloads = {'README.md': readme(report, stats), 'opcode_summary.json': encoded(stats),
        'original_SHA256SUMS.txt': raw_manifest, 'execution_source.py': checked(root/'source_executed.py', pins, SOURCE_SHA)}
    mappings = []
    for name, record in (('report.json', report), ('followup_plan.json', plan)):
        raw = checked(root/name, pins)
        payloads[name] = encoded(dict(publication_notice='Metadata view; machine-specific paths may be redacted. Original SHA retained; not a training dataset.',
            raw_source_sha256=audit.sha(raw), record=sanitize(record)))
        mappings.append(dict(source_private_relative_path=name, raw_source_sha256=audit.sha(raw),
            public_path=name, public_sha256=audit.sha(payloads[name]), representation='metadata_view'))
    for name in ('source_executed.py', 'SHA256SUMS'):
        public_name = 'execution_source.py' if name == 'source_executed.py' else 'original_SHA256SUMS.txt'
        mappings.append(dict(source_private_relative_path=name, raw_source_sha256=pins[root/name], public_path=public_name,
            public_sha256=audit.sha(payloads[public_name]), representation='original_bytes'))
    for raw in payloads.values(): privacy(raw)
    output.mkdir(parents=True, exist_ok=False)
    for name, raw in payloads.items(): audit.write_new(output/name, raw)
    # HH_260906 - Recheck the complete original tree and all code/data inputs after writing; partial publication never receives a success manifest.
    for path, digest in list(pins.items()): checked(path, pins, digest)
    audit.require(tree_files(root) == set(manifest_entries(raw_manifest)) | {'SHA256SUMS'}, 'input inventory changed during publication')
    manifest = dict(schema=SCHEMA, status='PUBLISHED_STATIC_INVENTORY_NOT_CALIBRATION', published_at_utc=audit.utc(),
        comment='HH_260906 - Preserve original evidence and every unresolved field; no BLOB, calibration extraction, training or admission is published.',
        worker_source_sha256=SOURCE_SHA, prospective_plan_sha256=PLAN_SHA, previous_report_sha256=audit.PRIOR_SHA,
        raw_report_sha256=report_sha, raw_checksums_sha256=checksums_sha, publication_source_sha256=pins[Path(__file__).absolute()],
        publication_source_path='scripts/e2e/curate_nuplan_calibration_opcodes.py', original_file_count=36,
        private_blob_count=32, public_blob_count=0, all_32_static_inventories_recomputed=True, inputs_unchanged_before_after=True,
        database_reopened_by_publisher=False, training_data_approved=False, source_views=mappings,
        files=[dict(path=name, sha256=audit.sha(raw), size_bytes=len(raw)) for name, raw in sorted(payloads.items())])
    privacy(encoded(manifest)); audit.write_new(output/'publication_manifest.json', encoded(manifest))
    names = sorted([*payloads, 'publication_manifest.json'])
    audit.write_new(output/'SHA256SUMS', ''.join(audit.sha((output/name).read_bytes())+'  '+name+'\n' for name in names).encode())
    return manifest


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    for name in ('input-root', 'prior-report', 'output-dir', 'expected-report-sha256', 'expected-checksums-sha256'):
        parser.add_argument('--'+name, required=True)
    args = parser.parse_args(argv)
    result = publish(args.input_root, args.prior_report, args.output_dir,
        report_sha=args.expected_report_sha256, checksums_sha=args.expected_checksums_sha256)
    print(json.dumps({'status': result['status'], 'private_blobs_excluded': result['private_blob_count']}))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
