#!/usr/bin/env python3
"""HH_260906 - Publish completed learning evidence without copying executable model checkpoints into Git."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import sys

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from portable_e2e.contract import _loads_json
from scripts.e2e import summarize_portable_training_campaign as training_summary
from scripts.e2e import summarize_portable_data_expansion as expansion_summary
from scripts.e2e import summarize_portable_selector_weight as selector_summary
from scripts.e2e.curate_independent_common10_capture_20260907 import sanitize


def sha(path):
    result = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            result.update(block)
    return result.hexdigest()


def copy_file(source, destination):
    # HH_260906 - Fail on an existing destination and preserve original report bytes.
    if source.is_symlink() or not source.is_file():
        raise ValueError(f'not a regular source file: {source}')
    destination.parent.mkdir(parents=True, exist_ok=True)
    with source.open('rb') as incoming, destination.open('xb') as outgoing:
        shutil.copyfileobj(incoming, outgoing)
    if sha(source) != sha(destination):
        raise ValueError('evidence copy changed bytes')


def read_json(path):
    if not path.is_file() or path.is_symlink():
        raise ValueError(f'not a regular JSON source: {path}')
    result = _loads_json(path.read_text(), str(path))
    if not isinstance(result, dict):
        raise ValueError(f'JSON source must be an object: {path}')
    return result


def redact_string(value):
    # HH_260906 - Replace machine-specific roots while keeping portable artifact suffixes and source hashes intact.
    return re.sub(r'/tmp(?=/|$|[\s\"\'])', lambda _match: '${TMP_ROOT}', sanitize(value))


def redact_metadata(value):
    if isinstance(value, str):
        return redact_string(value)
    if isinstance(value, list):
        return [redact_metadata(item) for item in value]
    if isinstance(value, dict):
        return {redact_string(key): redact_metadata(item) for key, item in value.items()}
    return value


def publish_file(source, destination):
    # HH_260906 - Keep executable provenance private and label every public metadata derivative explicitly.
    raw_sha = sha(source)
    destination.parent.mkdir(parents=True, exist_ok=True)
    if source.suffix == '.json':
        payload = redact_metadata(read_json(source))
        if 'publication_notice' in payload or 'raw_source_sha256' in payload:
            raise ValueError('source already uses reserved publication metadata fields')
        payload.update(publication_notice='Redacted metadata view; replay requires the private original identified by raw_source_sha256.',
            raw_source_sha256=raw_sha)
        with destination.open('x') as stream:
            stream.write(json.dumps(payload, indent=2, allow_nan=False) + '\n')
        return 'redacted_metadata_view'
    if source.suffix == '.md':
        notice = ('> 공개 metadata의 계정별 경로를 치환했습니다. 독립 재실행에는 원본 SHA로 식별한 '
            'private 원본 자료가 필요합니다.\n\n')
        with destination.open('x') as stream:
            stream.write(notice + redact_string(source.read_text()))
        return 'document_with_redaction_notice'
    if source.suffix == '.jsonl':
        original = source.read_text()
        if redact_string(original) != original:
            lines = [redact_metadata(_loads_json(line, str(source)))
                for line in original.splitlines() if line.strip()]
            with destination.open('x') as stream:
                for item in lines:
                    stream.write(json.dumps(item, allow_nan=False) + '\n')
            return 'redacted_jsonl_view'
    copy_file(source, destination)
    return 'original_bytes'


def recompute_summary(campaign, plan, baseline_campaign=None):
    # HH_260906 - Select the complete evidence validator by the frozen plan schema, never a supplied result label.
    schema = plan.get('schema')
    if schema == 'portable_e2e.selector_weight_campaign.v1':
        # HH_260906 - Revalidate both same-corpus campaigns instead of trusting a detached paired report.
        if baseline_campaign is None:
            raise ValueError('selector publication requires its baseline campaign')
        report = selector_summary.summarize_campaign(baseline_campaign, campaign)
        return report, selector_summary.render_markdown(report)
    if baseline_campaign is not None:
        raise ValueError('a baseline campaign is supported only for selector-weight publication')
    if schema == 'portable_e2e.lr_ab_campaign.v1':
        summarizer = training_summary
    elif schema == 'portable_e2e.data_expansion_campaign.v1':
        summarizer = expansion_summary
    else:
        raise ValueError('unsupported frozen campaign schema')
    report = summarizer.summarize_campaign(campaign)
    return report, summarizer.render_markdown(report)


def verify_plots(plots_dir, campaign, summary_path, state):
    # HH_260906 - Bind every curve input and the paired validation summary before publishing any plot.
    provenance = read_json(plots_dir / 'plot_inputs.json')
    if provenance.get('summary_sha256') != sha(summary_path):
        raise ValueError('plot summary SHA-256 does not match the supplied summary')
    expected = {f'{stage["run"]}/training/metrics.jsonl' for stage in state['stages']}
    inputs = provenance.get('inputs')
    if not isinstance(inputs, list) or len(inputs) != len(expected):
        raise ValueError('plot input list does not cover every campaign run')
    observed = set()
    for item in inputs:
        if not isinstance(item, dict) or not isinstance(item.get('path'), str):
            raise ValueError('plot input must name a relative campaign file')
        name = item['path']
        relative = Path(name)
        if relative.is_absolute() or '..' in relative.parts or name not in expected or name in observed:
            raise ValueError('plot input path is unsafe, duplicated, or outside the exact campaign runs')
        path = campaign / relative
        if not path.is_file() or path.is_symlink() or item.get('sha256') != sha(path):
            raise ValueError('plot input SHA-256 does not match the campaign training history')
        observed.add(name)
    if observed != expected:
        raise ValueError('plot input set differs from the exact campaign runs')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('campaign', type=Path)
    parser.add_argument('--summary-dir', type=Path, required=True)
    parser.add_argument('--plots-dir', type=Path)
    parser.add_argument('--baseline-campaign', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    summary_path = args.summary_dir / 'summary.json'
    summary = read_json(summary_path)
    state = read_json(args.campaign / 'status.json')
    plan = read_json(args.campaign / 'plan.json')
    if summary.get('status') != 'COMPLETE_NOT_PROMOTED' or state.get('status') != 'TRAIN_EVAL_AUDIT_COMPLETE_NOT_PROMOTED':
        raise ValueError('only publish independently summarized completed campaigns')
    if summary.get('vehicle_control_approved') is not False:
        raise ValueError('publication cannot approve actuation')
    fresh, markdown = recompute_summary(args.campaign, plan, args.baseline_campaign)
    if summary != fresh:
        raise ValueError('supplied summary differs from freshly verified campaign evidence')
    if (args.summary_dir / 'README.md').read_text() != markdown:
        raise ValueError('supplied README differs from the verified summary rendering')
    if args.plots_dir:
        verify_plots(args.plots_dir, args.campaign, summary_path, state)
    sources = [(args.summary_dir / name, Path(name)) for name in ('summary.json', 'README.md')]
    sources += [(args.campaign / name, Path('provenance') / name) for name in ('plan.json', 'status.json')]
    frozen = args.campaign / 'provenance/active_runner.py'
    if not frozen.is_file() or frozen.is_symlink() or sha(frozen) != state['runner_sha256']:
        raise ValueError('exact active runner must be retained and hash-verified')
    for run in sorted({stage['run'] for stage in state['stages']}):
        relative = Path(run)
        if relative.is_absolute() or '..' in relative.parts:
            raise ValueError('unsafe recorded run path')
        for name in ('training/run.json', 'training/metrics.jsonl', 'evaluation/metrics.json', 'gate_v8.json'):
            sources.append((args.campaign / relative / name, Path('runs') / relative / name))
    if args.plots_dir:
        sources += [(path, Path('visuals') / path.name) for path in sorted(args.plots_dir.iterdir())
            if path.suffix in ('.png', '.json')]
    for source, _ in sources:
        if not source.is_file() or source.is_symlink():
            raise ValueError(f'missing regular evidence: {source}')
    args.output.mkdir(parents=True, exist_ok=False)
    manifest = []
    for source, target in sources:
        destination = args.output / target
        representation = publish_file(source, destination)
        manifest.append({'published_path': target.as_posix(), 'source_path': redact_string(str(source)),
            'raw_source_sha256': sha(source), 'raw_source_size_bytes': source.stat().st_size,
            'sha256': sha(destination), 'size_bytes': destination.stat().st_size,
            'representation': representation})
    record_path = args.output / 'provenance/execution_record.json'
    execution_record = {'schema': 'portable_e2e.private_execution_provenance.v1',
        'publication_notice': 'The exact executable runner remains private; its original SHA-256 was verified before publication.',
        'runner_sha256': state['runner_sha256'], 'source_commit': state.get('source_commit'),
        'private_artifact_relative_path': 'provenance/active_runner.py',
        'raw_source_sha256': sha(frozen), 'vehicle_control_approved': False}
    with record_path.open('x') as stream:
        stream.write(json.dumps(execution_record, indent=2) + '\n')
    manifest.append({'published_path': 'provenance/execution_record.json',
        'source_path': redact_string(str(frozen)), 'raw_source_sha256': sha(frozen),
        'raw_source_size_bytes': frozen.stat().st_size, 'sha256': sha(record_path),
        'size_bytes': record_path.stat().st_size, 'representation': 'private_runner_hash_record'})
    (args.output / 'publication_manifest.json').write_text(json.dumps({
        'comment': 'HH_260906 - Metadata uses redacted public views with original and published hashes; executable runners and checkpoints remain private.',
        'vehicle_control_approved': False, 'files': manifest}, indent=2) + '\n')
    paths = sorted(path for path in args.output.rglob('*') if path.is_file())
    (args.output / 'SHA256SUMS').write_text(''.join(
        f'{sha(path)}  {path.relative_to(args.output).as_posix()}\n' for path in paths))
    print(json.dumps({'published_files': len(paths) + 1, 'output': str(args.output)}))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
