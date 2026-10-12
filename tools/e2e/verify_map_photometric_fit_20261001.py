#!/usr/bin/env python3
"""HH_261001 - Recount final known-split predictions and audit matched training histories."""

import argparse
import hashlib
import json
from pathlib import Path
import numpy as np


ROOT = Path(__file__).resolve().parents[2]
FIT = ROOT / 'storage/experiments/training/2026-10-01/map_photometric_matched_20261001_v1/fit'
DATA = ROOT / 'storage/experiments/training/2026-09-29/map_town02_p00_route_holdout_data_v1'
SUMMARY_SHA = None


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify():
    if digest(FIT / 'summary.json') != SUMMARY_SHA:
        raise ValueError('final imported summary changed')
    summary = json.loads((FIT / 'summary.json').read_text())
    if summary['status'] != 'PHOTOMETRIC_PAIR_COMPLETE_DIAGNOSTIC_NOT_PROMOTED':
        raise ValueError('incomplete fit')
    pair = summary['arms']
    for key in ('initial_state_sha256', 'sample_sequence_sha256', 'crop_sequence_sha256'):
        if pair['control'][key] != pair['photometric'][key]:
            raise ValueError('matched input or initialization differs: ' + key)
    # HH_261001 - Reproduce the preceding no-augmentation appearance model, not its old OFF control.
    predecessor = ROOT / 'storage/experiments/training/2026-10-01/map_appearance_matched_20261001_v1/fit'
    if digest(predecessor/'summary.json') != '463741ec65e9fce501bcb06dbaecc6c02d5b46bd566d6510ca0a7050b54e5253':
        raise ValueError('preceding appearance fit changed')
    with np.load(predecessor/'appearance/weights.npz',allow_pickle=False) as previous, np.load(FIT/'control/weights.npz',allow_pickle=False) as fresh:
        if set(previous.files) != set(fresh.files) or any(not np.array_equal(previous[key],fresh[key]) for key in previous.files):
            raise ValueError('fresh control does not reproduce all prior appearance tensors')
    manifest = json.loads((FIT.parent/'manifest.json').read_text())
    if (pair['photometric']['photo_trace'] != manifest['expected_photo_trace']
            or pair['control']['photo_trace']['applied_calls'] != 0
            or pair['control']['photo_trace']['eligible_calls'] != 0
            or not pair['control']['predecessor_reference_state_reproduced']):
        raise ValueError('actual photometric treatment or control identity differs')
    histories = {}
    for arm, receipt in pair.items():
        for filename, key in (('weights.npz', 'weights_sha256'), ('checkpoint.pt', 'checkpoint_sha256')):
            if digest(FIT / arm / filename) != receipt[key]:
                raise ValueError('endpoint artifact hash differs')
        rows = [json.loads(line) for line in (FIT / arm / 'history.jsonl').read_text().splitlines()]
        if ([row['step'] for row in rows] != list(range(100, 10001, 100))
                or any(row['arm'] != arm or not np.isfinite(row['mean_loss_100']) for row in rows)
                or any(a['wall_seconds'] >= b['wall_seconds'] for a, b in zip(rows, rows[1:]))
                or receipt['steps'] != 10000):
            raise ValueError('complete finite ten-thousand-step history required')
        histories[arm] = dict(steps=10000, log_intervals=len(rows),
                              training_wall_seconds=receipt['training_wall_seconds'],
                              checkpoint_sha256=receipt['checkpoint_sha256'])
    checks = {}
    for split, arms in summary['known_evaluation'].items():
        path = DATA / (split + '.npz')
        expected = {value['source_sha256'] for value in arms.values()}
        if expected != {digest(path)}:
            raise ValueError('local known-split bytes do not match remote evaluation')
        with np.load(path, allow_pickle=False) as archive:
            target = archive['target'].copy()
        for arm, reported in arms.items():
            pred_path = FIT / arm / (split + '_predictions.npz')
            with np.load(pred_path, allow_pickle=False) as archive:
                prediction = archive['prediction'].copy()
            if (prediction.shape != target.shape or prediction.dtype != np.uint8
                    or prediction.max() >= 7 or len(target) != reported['count']):
                raise ValueError('prediction ABI/count differs')
            counted = np.bincount((7 * target.astype(np.int16) + prediction).ravel(),
                                  minlength=49).reshape(7, 7)
            if counted.tolist() != reported['confusion']:
                raise ValueError('native all-pixel confusion differs')
            checks[split + '/' + arm] = dict(frames=len(target), pixels=int(counted.sum()),
                prediction_sha256=digest(pred_path), all_pixel_confusion_exact=True)
    if digest(FIT / 'summary.json') != SUMMARY_SHA:
        raise ValueError('summary changed during verification')
    return dict(schema='portable_e2e.map_photometric_fit_recount.v1', status='PASS',
        summary_sha256=SUMMARY_SHA, histories=histories, checks=checks,
        total_recounted_pixels=sum(row['pixels'] for row in checks.values()),
        new_training_steps=0, model_promotion=False)


def main():
    global SUMMARY_SHA
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--expected-summary-sha256',required=True)
    args = parser.parse_args()
    SUMMARY_SHA = args.expected_summary_sha256
    result = verify()
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2, sort_keys=True)
        stream.write('\n')
    print(json.dumps(dict(status=result['status'], pixels=result['total_recounted_pixels'])))


if __name__ == '__main__':
    main()
