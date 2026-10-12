#!/usr/bin/env python3
"""HH_261001 - Independently recount all paired map predictions against native semantic bytes."""

import argparse
import hashlib
import json
from pathlib import Path
import numpy as np


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify(folder):
    summary_path = folder / 'summary.json'
    before = digest(summary_path)
    summary = json.loads(summary_path.read_text())
    source = Path(summary['source'])
    records = json.loads((source / 'records.json').read_text())
    if len(records) != 240:
        raise ValueError('all 240 frames required')
    # HH_261001 - Reconstruct seven-class IDs independently of the evaluator implementation.
    table = np.zeros(256, dtype=np.uint8)
    for tag, class_id in ((1, 1), (2, 2), (24, 3), (7, 4), (8, 5), (6, 6)):
        table[tag] = class_id
    target = []
    for row in records:
        path = source / (row['stem'] + '.npz')
        if digest(path) != row['artifact_sha256']['.npz']:
            raise ValueError('raw source hash changed')
        with np.load(path, allow_pickle=False) as data:
            target.append(table[data['semantic'][..., 2]])
    target = np.stack(target)
    checks = {}
    for filename, expected_sha in summary['prediction_artifacts'].items():
        path = folder / filename
        if digest(path) != expected_sha:
            raise ValueError('prediction hash changed')
        with np.load(path, allow_pickle=False) as data:
            prediction = data['prediction'].copy()
        if prediction.shape != target.shape or prediction.dtype != np.uint8 or prediction.max() >= 7:
            raise ValueError('prediction shape/class contract differs')
        confusion = np.bincount((7 * target.astype(np.int16) + prediction).ravel(), minlength=49).reshape(7, 7)
        key = filename.removesuffix('_all240_predictions.npz')
        if confusion.tolist() != summary['model_results'][key]['confusion']:
            raise ValueError('full confusion differs: ' + key)
        checks[key] = dict(pixels=int(confusion.sum()), frames=240, confusion_exact=True,
                           prediction_sha256=expected_sha)
    if before != digest(summary_path) or len(checks) != 4:
        raise ValueError('summary changed or incomplete paired results')
    return dict(summary_sha256=before, source=str(source), checks=checks,
                pixel_predictions_recounted=sum(row['pixels'] for row in checks.values()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--folder', type=Path, action='append', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = dict(schema='portable_e2e.paired_map_pixel_recount.v1', status='PASS',
                  results={str(folder): verify(folder) for folder in args.folder},
                  fitting_steps=0, promotion=False)
    with args.output.open('x') as stream:
        json.dump(result, stream, sort_keys=True, indent=2)
        stream.write('\n')
    print(json.dumps({'status': 'PASS', 'pixel_predictions_recounted': sum(
        row['pixel_predictions_recounted'] for row in result['results'].values())}))


if __name__ == '__main__':
    main()
