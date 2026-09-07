#!/usr/bin/env python3
"""HH_260906 - Diagnose composite-loss and ADE oracle alignment without changing training."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import time

import torch
from torch.utils.data import DataLoader

import portable_e2e
from portable_e2e.audit_runtime import (
    _atomic_new_json, _read_checkpoint_for_audit, _validate_checkpoint_and_model,
)
from portable_e2e.contract import ContractError
from portable_e2e.dataset import load_training_examples
from portable_e2e.losses import TrajectoryLossConfig, trajectory_loss
from portable_e2e.torch_dataset import Common10TorchDataset
from scripts.e2e.diagnose_portable_selector import summarize_predictions


# HH_260906 - Freeze this analysis to the already collected v3 corpus without opening held-out test examples.
SCHEMA = 'portable_e2e.objective_alignment_diagnostic.v1'
MANIFEST_SHA256 = '18262e5aa4abbb3e03e35e379b5da1e5ce7fd339a9a8942e02b58ca737f7242c'
CORPUS_SHA256 = '56d9ff663612a090cd61698b53cf0cf39b0a7ae0de6b42d096957a94b6c92047'
SPLITS = {
    'val': (337, '631be7323f502cafc0dd66766104203bd5e7fee7494436c479bff1e0dddc0285'),
    'train': (1147, 'd957e72c1eea755fed5b4ac682e775983861c16104fe083b7cfb6cbb4fed1928'),
}


def file_sha256(path):
    path = Path(path)
    if path.is_symlink() or not path.is_file():
        raise ContractError('diagnostic input must be a regular non-symlink file')
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def require_hash(path, expected):
    if not isinstance(expected, str) or not re.fullmatch('[0-9a-f]{64}', expected):
        raise ContractError('expected SHA-256 must be 64 lowercase hexadecimal characters')
    actual = file_sha256(path)
    if actual != expected:
        raise ContractError('diagnostic input SHA-256 mismatch')
    return actual


def source_provenance(expected_commit):
    # HH_260906 - The temporary diagnostic script is pinned separately from the unchanged clean imported repository.
    if not isinstance(expected_commit, str) or not re.fullmatch('[0-9a-f]{40}', expected_commit):
        raise ContractError('expected source commit must be 40 lowercase hexadecimal characters')
    root = Path(portable_e2e.__file__).resolve().parents[1]

    def git(*args):
        result = subprocess.run(['git', '-C', str(root), *args], check=True,
            capture_output=True, text=True, timeout=15)
        return result.stdout.strip()

    if Path(git('rev-parse', '--show-toplevel')).resolve() != root:
        raise ContractError('imported package is outside the expected repository root')
    if git('rev-parse', 'HEAD') != expected_commit:
        raise ContractError('diagnostic imported source commit mismatch')
    if git('status', '--porcelain', '--untracked-files=all'):
        raise ContractError('diagnostic imported repository must be clean')
    sources = sorted((root / 'portable_e2e').rglob('*.py'))
    sources.append(root / 'scripts/e2e/diagnose_portable_selector.py')
    return {'diagnostic_source_commit': expected_commit,
        'source_sha256': {str(path.relative_to(root)): file_sha256(path) for path in sources}}


def checkpoint_loss_config(payload):
    value = payload.get('loss_config')
    if not isinstance(value, dict) or set(value) != set(TrajectoryLossConfig().to_dict()):
        raise ContractError('checkpoint must contain the complete original loss_config')
    config = TrajectoryLossConfig(**value)
    config.validate()
    return config


def summarize_alignment(xy, speed, logits, target, target_speed, valid, target_yaw, loss_config):
    # HH_260906 - Use the actual training loss oracle, including original masks and yaw, rather than recreating its formula.
    if target_yaw is None:
        raise ContractError('objective alignment requires the original target yaw tensor')
    losses = trajectory_loss(xy, speed, logits, target, target_speed, valid,
        loss_config, target_yaw=target_yaw)
    geometry = summarize_predictions(xy, logits, target, valid)
    count, candidates = geometry['sample_count'], geometry['candidate_count']
    indices = {
        'selected': losses['selected_candidate_index'].detach().cpu(),
        'composite_oracle': losses['oracle_candidate_index'].detach().cpu(),
        'ade_oracle': torch.tensor([row['ade_oracle_index'] for row in geometry['per_sample']]),
    }
    summary = {'sample_count': count, 'candidate_count': candidates,
        'predicted_points': geometry['predicted_points'], 'per_sample': []}
    for name, index in indices.items():
        summary[f'{name}_histogram'] = torch.bincount(index, minlength=candidates).tolist()
    for left, right in (('selected', 'composite_oracle'), ('selected', 'ade_oracle'),
            ('composite_oracle', 'ade_oracle')):
        key = f'{left}_{right}'
        agreement = int((indices[left] == indices[right]).sum())
        summary[f'{key}_agreement_count'] = agreement
        summary[f'{key}_agreement_rate'] = agreement / count
        summary[f'{key}_confusion_rows_left_columns_right'] = torch.bincount(
            indices[left] * candidates + indices[right], minlength=candidates ** 2
        ).reshape(candidates, candidates).tolist()
    for index, row in enumerate(geometry['per_sample']):
        composite = int(indices['composite_oracle'][index])
        summary['per_sample'].append({
            'index': index, 'selected_index': row['selected_index'],
            'composite_oracle_index': composite, 'ade_oracle_index': row['ade_oracle_index'],
            'target_valid': valid[index].detach().cpu().tolist(),
            'valid_point_count': int(valid[index].sum()),
            'candidate_ade_m': row['candidate_ade_m'], 'selected_ade_m': row['selected_ade_m'],
            'composite_oracle_ade_m': row['candidate_ade_m'][composite],
            'ade_oracle_ade_m': row['oracle_ade_m'],
            'selected_minus_ade_oracle_m': row['selection_regret_m'],
            'composite_oracle_minus_ade_oracle_m': row['candidate_ade_m'][composite] - row['oracle_ade_m'],
            'selected_minus_composite_oracle_ade_m': row['selected_ade_m'] - row['candidate_ade_m'][composite],
        })
    for field in ('selected_ade_m', 'composite_oracle_ade_m', 'ade_oracle_ade_m',
            'selected_minus_ade_oracle_m', 'composite_oracle_minus_ade_oracle_m',
            'selected_minus_composite_oracle_ade_m'):
        summary[f'mean_{field}'] = sum(row[field] for row in summary['per_sample']) / count
    return summary


def validate_split(dataset, split, payload, training_ids):
    if split not in SPLITS or dataset.split != split:
        raise ContractError('objective alignment supports only separately labelled val or train')
    expected_count, expected_fingerprint = SPLITS[split]
    if len(dataset) != expected_count or dataset.fingerprint_sha256 != expected_fingerprint:
        raise ContractError('objective alignment requires the entire frozen v3 split')
    episodes = sorted({example.episode_id for example in dataset.examples})
    if split == 'train':
        if dataset.fingerprint_sha256 != payload.get('dataset_fingerprint_sha256'):
            raise ContractError('train diagnostic fingerprint does not match checkpoint training data')
        if episodes != list(training_ids):
            raise ContractError('train diagnostic episode IDs do not match checkpoint training data')
    elif set(episodes) & set(training_ids):
        raise ContractError('validation diagnostic has training episode leakage')


def analyze_split(model, dataset, loss_config):
    arrays = {name: [] for name in ('xy', 'speed', 'logits', 'target', 'target_speed', 'valid', 'target_yaw')}
    sample_ids = []
    with torch.no_grad():
        for batch in DataLoader(dataset, batch_size=4, shuffle=False, num_workers=0):
            xy, speed, logits = model(batch['images'], batch['calibration'], batch['ego_history'],
                batch['ego_history_mask'], batch['route_xy'], batch['route_mask'])
            values = (xy, speed, logits, batch['target_xy'], batch['target_speed_mps'],
                batch['target_valid'], batch['target_yaw_rad'])
            for name, value in zip(arrays, values):
                arrays[name].append(value.detach().cpu())
            sample_ids.extend(batch['sample_id'])
    if len(sample_ids) != len(dataset) or len(set(sample_ids)) != len(sample_ids):
        raise ContractError('objective diagnostic sample coverage is incomplete or duplicated')
    summary = summarize_alignment(**{name: torch.cat(parts) for name, parts in arrays.items()},
        loss_config=loss_config)
    for row, sample_id in zip(summary['per_sample'], sample_ids):
        row['sample_id'] = sample_id
    return {'split': dataset.split, 'dataset_fingerprint_sha256': dataset.fingerprint_sha256,
        'interpretation': 'Development validation, not independent test.' if dataset.split == 'val'
            else 'In-sample training diagnosis only; not validation or generalization evidence.',
        'summary': summary}


def diagnose(dataset_root, checkpoint, checkpoint_sha256, output_dir, script_sha256,
        expected_source_commit, include_train=False):
    started, tick = datetime.now(timezone.utc).isoformat(), time.monotonic()
    if os.environ.get('CUDA_VISIBLE_DEVICES') != '':
        raise ContractError('CPU diagnosis requires CUDA_VISIBLE_DEVICES to be explicitly empty')
    for name in ('OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
        if os.environ.get(name) != '4':
            raise ContractError('CPU diagnosis requires explicit OMP/MKL thread limits of four')
    if output_dir.exists() or output_dir.is_symlink():
        raise ContractError('diagnostic output directory must be new')
    require_hash(Path(__file__), script_sha256)
    source = source_provenance(expected_source_commit)
    manifest = dataset_root / 'dataset.json'
    require_hash(manifest, MANIFEST_SHA256)
    require_hash(checkpoint, checkpoint_sha256)
    torch.set_num_threads(4)
    torch.set_num_interop_threads(4)
    loaded = load_training_examples(dataset_root, split='val', mode='planning', check_image_hashes=True)
    if loaded.validation_report['dataset_fingerprint_sha256'] != CORPUS_SHA256:
        raise ContractError('objective alignment corpus fingerprint is not frozen v3')
    payload, config, training_ids, checkpoint_provenance = _read_checkpoint_for_audit(
        checkpoint_path=checkpoint, expected_checkpoint_sha256=checkpoint_sha256,
        corpus_fingerprint_sha256=CORPUS_SHA256)
    loss_config = checkpoint_loss_config(payload)
    val = Common10TorchDataset(loaded.examples, config, verify_image_sha256=True, split='val')
    validate_split(val, 'val', payload, training_ids)
    model, provenance = _validate_checkpoint_and_model(val, payload=payload, model_config=config,
        training_episode_ids=training_ids, checkpoint_provenance=checkpoint_provenance,
        device=torch.device('cpu'))
    datasets = [val]
    if include_train:
        loaded_train = load_training_examples(dataset_root, split='train', mode='planning', check_image_hashes=True)
        if loaded_train.validation_report['dataset_fingerprint_sha256'] != CORPUS_SHA256:
            raise ContractError('training diagnostic corpus changed')
        train = Common10TorchDataset(loaded_train.examples, config, verify_image_sha256=True, split='train')
        validate_split(train, 'train', payload, training_ids)
        datasets.append(train)
    splits = {dataset.split: analyze_split(model, dataset, loss_config) for dataset in datasets}
    # HH_260906 - Refuse publication if immutable inputs or imported source changed during the read-only analysis.
    require_hash(Path(__file__), script_sha256)
    require_hash(checkpoint, checkpoint_sha256)
    require_hash(manifest, MANIFEST_SHA256)
    if source_provenance(expected_source_commit) != source:
        raise ContractError('diagnostic source changed during analysis')
    report = {'schema': SCHEMA, 'status': 'DIAGNOSIS_COMPLETE',
        'started_at_utc': started, 'completed_at_utc': datetime.now(timezone.utc).isoformat(),
        'diagnostic_wall_seconds': time.monotonic() - tick,
        'diagnostic_script_sha256': script_sha256, **source, **provenance,
        'corpus_fingerprint_sha256': CORPUS_SHA256, 'manifest_sha256': MANIFEST_SHA256,
        'loss_config': loss_config.to_dict(), 'device': 'cpu', 'torch_version': str(torch.__version__),
        'torch_num_threads': torch.get_num_threads(),
        'torch_num_interop_threads': torch.get_num_interop_threads(),
        'composite_oracle_definition': 'Unmodified trajectory_loss oracle_candidate_index with checkpoint loss_config and original XY/speed/yaw/valid tensors. Candidate score weight scales classification loss, not the composite regression oracle.',
        'ade_oracle_definition': 'Minimum mean Euclidean XY error over original valid expert points. First index wins exact ties for both oracles and selector.',
        'interpretation': 'Read-only development diagnosis, not model promotion, held-out test evaluation, safety approval, or runtime latency evidence. Train and val are never pooled.',
        'vehicle_control_approved': False, 'splits': splits}
    output_dir.mkdir(parents=True, exist_ok=False)
    _atomic_new_json(output_dir / 'objective_alignment.json', report)
    return report


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('dataset', type=Path)
    parser.add_argument('--checkpoint', type=Path, required=True)
    parser.add_argument('--checkpoint-sha256', required=True)
    parser.add_argument('--script-sha256', required=True)
    parser.add_argument('--expected-source-commit', required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--include-train', action='store_true')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    report = diagnose(args.dataset, args.checkpoint, args.checkpoint_sha256, args.output_dir,
        args.script_sha256, args.expected_source_commit, args.include_train)
    print(json.dumps({split: {key: value for key, value in result['summary'].items()
        if key != 'per_sample'} for split, result in report['splits'].items()}, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
