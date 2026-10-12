#!/usr/bin/env python3
"""HH_260906 - Read live duration progress without importing Torch or changing run artifacts."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess


def last_row(path):
    if not path.is_file():
        return None
    # HH_260906 - Ignore only a currently incomplete trailing write; never edit the underlying history.
    with path.open('rb') as stream:
        stream.seek(0, 2)
        size = stream.tell()
        stream.seek(max(0, size - 16384))
        payload = stream.read()
    complete = payload.split(b'\n')[:-1]
    if not complete:
        return None
    try:
        row = json.loads(complete[-1])
    except (ValueError, UnicodeDecodeError):
        return None
    return {key: row.get(key) for key in ('epoch', 'global_step', 'samples_seen', 'loss')}


def inspect(root):
    progress = root / 'progress.json'
    result = {'status': 'NOT_STARTED', 'verified_campaign_complete': False}
    if progress.is_file():
        result.update(json.loads(progress.read_text()))
    if result.get('run') and isinstance(result.get('epoch'), int):
        item = root / 'epochs' / f'epoch_{result["epoch"]:03d}' / result['run']
        result['current_training_tail'] = last_row(item / 'training/metrics.jsonl')
        evaluation = item / 'evaluation/metrics.json'
        if evaluation.is_file():
            value = json.loads(evaluation.read_text())
            result['current_val_metrics'] = {key: value['metrics'][key] for key in
                ('ade_1p0s_m', 'ade_3p0s_m', 'ade_6p4s_m', 'fde_6p4s_m')}
    summary = root / 'summary.json'
    if summary.is_file():
        value = json.loads(summary.read_text())
        result['final_receipt'] = {key: value.get(key) for key in
            ('status', 'ended_at_utc', 'total_new_optimizer_steps', 'total_new_sample_exposures',
             'source_view_parent_postcheck_pass')}
        result['receipt_stage_count'] = len(value.get('stages', []))
    # HH_260906 - This quick status does not rehash final receipts or independently certify completion.
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-root', type=Path, required=True)
    parser.add_argument('--gpu0', action='store_true')
    args = parser.parse_args(argv)
    result = inspect(args.run_root)
    if args.gpu0:
        result['gpu0_observation'] = subprocess.check_output([
            'nvidia-smi', '-i', '0', '--query-gpu=timestamp,index,memory.used,memory.total,utilization.gpu',
            '--format=csv,noheader'], text=True, timeout=10).strip()
    print(json.dumps(result, ensure_ascii=False, allow_nan=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
