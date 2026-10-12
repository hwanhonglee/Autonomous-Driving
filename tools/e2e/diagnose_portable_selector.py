#!/usr/bin/env python3
"""HH_260906 - Diagnose fixed selector indices separately from candidate geometry on val337."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import time

import torch
from torch.utils.data import DataLoader

from portable_e2e.audit_runtime import _atomic_new_json, _read_checkpoint_for_audit, _validate_checkpoint_and_model
from portable_e2e.contract import ContractError
from portable_e2e.dataset import load_training_examples
from portable_e2e.torch_dataset import Common10TorchDataset
from portable_e2e.visualize import render_trajectory_png


def phase_indices(count):
    # HH_260906 - Freeze six uniformly spaced validation positions before inspecting predictions.
    if count != 337:
        raise ContractError('this diagnostic requires the frozen full val337 split')
    return [phase * (count - 1) // 5 for phase in range(6)]


def summarize_predictions(xy, logits, target, valid):
    # HH_260906 - ADE uses valid expert points; geometric separation uses every predicted horizon point.
    if xy.ndim != 4 or xy.shape[-1] != 2 or xy.shape[1] != 6:
        raise ContractError('diagnosis requires six candidate XY trajectories')
    n, k, horizon, _ = xy.shape
    if not n or not horizon or logits.shape != (n, k) or target.shape != (n, horizon, 2):
        raise ContractError('prediction and target shapes disagree')
    if valid.shape != (n, horizon) or valid.dtype != torch.bool or not bool(valid.any(dim=1).all()):
        raise ContractError('each sample requires a nonempty Boolean target mask')
    if not all(bool(torch.isfinite(value).all()) for value in (xy, logits, target)):
        raise ContractError('nonfinite selector diagnostic input')
    displacement = torch.norm(xy - target[:, None], dim=-1)
    ade = (displacement * valid[:, None]).sum(dim=-1) / valid.sum(dim=-1)[:, None]
    selected = logits.argmax(dim=1)
    oracle = ade.argmin(dim=1)
    rows = torch.arange(n)
    selected_ade, oracle_ade = ade[rows, selected], ade[rows, oracle]
    pairs = [(left, right) for left in range(k) for right in range(left + 1, k)]
    pairwise = torch.stack([torch.norm(xy[:, left] - xy[:, right], dim=-1)
        for left, right in pairs], dim=1)
    path_separation = pairwise.mean(dim=(1, 2))
    endpoint_separation = pairwise[:, :, -1].mean(dim=1)
    per_sample = [{'index': index, 'selected_index': int(selected[index]),
        'ade_oracle_index': int(oracle[index]), 'candidate_ade_m': ade[index].tolist(),
        'selected_ade_m': float(selected_ade[index]), 'oracle_ade_m': float(oracle_ade[index]),
        'selection_regret_m': float(selected_ade[index] - oracle_ade[index]),
        'mean_pairwise_path_distance_m': float(path_separation[index]),
        'mean_pairwise_endpoint_distance_m': float(endpoint_separation[index])}
        for index in range(n)]
    return {'sample_count': n, 'candidate_count': k, 'predicted_points': horizon,
        'selected_histogram': torch.bincount(selected, minlength=k).tolist(),
        'ade_oracle_histogram': torch.bincount(oracle, minlength=k).tolist(),
        'selector_ade_oracle_agreement_count': int((selected == oracle).sum()),
        'selector_ade_oracle_agreement_rate': float((selected == oracle).float().mean()),
        'mean_selected_ade_m': float(selected_ade.mean()), 'mean_oracle_ade_m': float(oracle_ade.mean()),
        'mean_selection_regret_m': float((selected_ade - oracle_ade).mean()),
        'mean_candidate_ade_m': ade.mean(dim=0).tolist(),
        'mean_pairwise_path_distance_m': float(path_separation.mean()),
        'min_sample_pairwise_path_distance_m': float(path_separation.min()),
        'max_sample_pairwise_path_distance_m': float(path_separation.max()),
        'mean_pairwise_endpoint_distance_m': float(endpoint_separation.mean()),
        'per_sample': per_sample}


def diagnose(dataset_root, checkpoint, expected_sha256, output_dir):
    started = datetime.now(timezone.utc).isoformat()
    wall_started = time.monotonic()
    device = torch.device('cpu')
    torch.set_num_threads(4)
    torch.set_num_interop_threads(4)
    loaded = load_training_examples(dataset_root, split='val', mode='planning', check_image_hashes=True)
    corpus = loaded.validation_report['dataset_fingerprint_sha256']
    payload, config, training_ids, checkpoint_provenance = _read_checkpoint_for_audit(
        checkpoint_path=checkpoint, expected_checkpoint_sha256=expected_sha256,
        corpus_fingerprint_sha256=corpus)
    dataset = Common10TorchDataset(loaded.examples, config, verify_image_sha256=True, split='val')
    render_indices = phase_indices(len(dataset))
    model, provenance = _validate_checkpoint_and_model(dataset, payload=payload, model_config=config,
        training_episode_ids=training_ids, checkpoint_provenance=checkpoint_provenance, device=device)
    output_dir.mkdir(parents=True, exist_ok=False)
    arrays = {name: [] for name in ('xy', 'logits', 'target', 'valid')}
    sample_ids, rendered = [], []
    with torch.no_grad():
        for batch in DataLoader(dataset, batch_size=4, shuffle=False, num_workers=0):
            xy, _, logits = model(batch['images'], batch['calibration'], batch['ego_history'],
                batch['ego_history_mask'], batch['route_xy'], batch['route_mask'])
            for name, value in zip(arrays, (xy, logits, batch['target_xy'], batch['target_valid'])):
                arrays[name].append(value.detach().cpu())
            for position, sample_id in enumerate(batch['sample_id']):
                index = len(sample_ids)
                sample_ids.append(sample_id)
                if index not in render_indices:
                    continue
                name = f'val_phase_{index:03d}.png'
                render_trajectory_png(output_dir / name,
                    route_xy=batch['route_xy'][position][batch['route_mask'][position]].tolist(),
                    target_xy=batch['target_xy'][position].tolist(),
                    target_valid=batch['target_valid'][position].tolist(),
                    candidate_xy=xy[position].tolist(), candidate_logits=logits[position].tolist(),
                    title=f'val337 index {index}; CPU diagnostic; {expected_sha256[:12]}')
                rendered.append({'index': index, 'sample_id': sample_id, 'path': name,
                    'sha256': hashlib.sha256((output_dir / name).read_bytes()).hexdigest()})
    summary = summarize_predictions(**{name: torch.cat(parts) for name, parts in arrays.items()})
    for row, sample_id in zip(summary['per_sample'], sample_ids):
        row['sample_id'] = sample_id
    report = {'schema': 'portable_e2e.selector_diagnostic.v1', 'status': 'DIAGNOSIS_COMPLETE',
        'started_at_utc': started, 'completed_at_utc': datetime.now(timezone.utc).isoformat(),
        'diagnostic_wall_seconds': time.monotonic() - wall_started,
        'diagnostic_script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'device': 'cpu', 'torch_num_threads': torch.get_num_threads(),
        'torch_num_interop_threads': torch.get_num_interop_threads(), 'evaluation_split': 'val',
        'dataset_fingerprint_sha256': dataset.fingerprint_sha256, 'corpus_fingerprint_sha256': corpus,
        **provenance, 'render_indices': render_indices, 'render_policy': 'floor(phase * 336 / 5), phase=0..5',
        'oracle_definition': 'Minimum ADE over valid expert XY points; first index wins exact ties. This is not the training loss oracle.',
        'geometry_definition': 'Mean Euclidean separation across all 15 unordered candidate pairs and all predicted time points; endpoint uses the final predicted point.',
        'interpretation': 'A fixed selected index does not prove identical candidate geometries. CPU diagnostic timing is not GPU or runtime latency. No model promotion.',
        'vehicle_control_approved': False, 'summary': summary, 'rendered': rendered}
    _atomic_new_json(output_dir / 'selector_diagnostic.json', report)
    return report


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('dataset', type=Path)
    parser.add_argument('--checkpoint', type=Path, required=True)
    parser.add_argument('--checkpoint-sha256', required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    report = diagnose(args.dataset, args.checkpoint, args.checkpoint_sha256, args.output_dir)
    print(json.dumps({key: value for key, value in report['summary'].items() if key != 'per_sample'}, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
