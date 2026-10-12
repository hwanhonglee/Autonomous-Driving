#!/usr/bin/env python3
"""HH_261001 - Replay both trained map models from data-only weights on local CPU."""

from __future__ import annotations
import json
from pathlib import Path
import time
import numpy as np
import torch
from portable_e2e.map_perception import MapPerception, LABELS
from portable_e2e.map_context_recovery import MapContextRecovery
from scripts.e2e import audit_map_context_recovery_20261001 as audit
from scripts.e2e import train_map_town02_p00_route_holdout as contract


def main():
    torch.set_num_threads(2)
    require = audit.visual.require
    run = audit.RUN
    output = audit.OUTPUT.parent / 'cpu_verification.json'
    require(not output.exists(), 'fresh CPU verification output required')
    summary = audit.visual.read_json(run / 'summary.json')
    manifest = audit.visual.read_json(audit.visual.DATA / 'manifest.json')
    require(audit.visual.digest(audit.visual.DATA / 'manifest.json') == audit.visual.DATA_MANIFEST_SHA256,
            'evaluation source manifest changed')
    matrices, details = {}, {}
    for arm, cls in (('A', MapPerception), ('B', MapContextRecovery)):
        weights = run / arm / 'weights.npz'
        require(audit.visual.digest(weights) == summary[arm]['weights_sha256'], 'weights hash mismatch')
        model = cls()
        expected = model.state_dict()
        with np.load(weights, allow_pickle=False) as archive:
            require(set(archive.files) == set(expected), 'state keys differ')
            state = {}
            for name in archive.files:
                array = archive[name]
                require(array.dtype == np.float32 and tuple(array.shape) == tuple(expected[name].shape)
                        and bool(np.isfinite(array).all()), 'state tensor ABI mismatch')
                state[name] = torch.from_numpy(array.copy())
        model.load_state_dict(state, strict=True)
        model.eval()
        for split, spec in audit.visual.SPLITS.items():
            ref = manifest['files'][spec['dataset']]
            path = audit.visual.DATA / ref['file']
            require(audit.visual.digest(path) == ref['sha256'], 'evaluation bytes differ')
            with np.load(path, allow_pickle=False) as archive:
                rgb, target = archive['rgb'], archive['target']
            with np.load(run / arm / (split + '_predictions.npz'), allow_pickle=False) as archive:
                saved = archive['prediction']
            matrix = np.zeros((7, 7), dtype=np.int64)
            identical, times = 0, []
            with torch.no_grad():
                for index, pixels in enumerate(rgb):
                    tensor = torch.from_numpy(pixels.transpose(2, 0, 1).copy()).float().unsqueeze(0) / 255
                    start = time.perf_counter()
                    prediction = model(tensor).argmax(1)[0].numpy().astype(np.uint8)
                    times.append(time.perf_counter() - start)
                    identical += int((prediction == saved[index]).sum())
                    code = target[index].astype(np.int64) * 7 + prediction
                    matrix += np.bincount(code.ravel(), minlength=49).reshape(7, 7)
                    if (index + 1) % 50 == 0:
                        print(json.dumps({'arm': arm, 'split': split, 'replayed': index + 1}), flush=True)
            metrics = contract.metrics_from_confusion(matrix, LABELS, np)
            delta = {name: None if metrics['iou'][name] is None or summary[arm][split]['iou'][name] is None
                     else abs(metrics['iou'][name] - summary[arm][split]['iou'][name]) for name in LABELS}
            agreement = identical / target.size
            passed = agreement >= 0.999 and all(v is not None and v <= 0.005 for v in delta.values())
            key = arm + '_' + split
            matrices[key] = metrics
            details[key] = {'frames': len(rgb), 'pixel_agreement': agreement,
                            'absolute_iou_difference': delta, 'passed': passed,
                            'inference_p50_ms': float(np.quantile(times, 0.5) * 1000),
                            'inference_p95_ms': float(np.quantile(times, 0.95) * 1000)}
    result = {'schema': 'portable_e2e.map_context_cpu_verification.v1',
              'status': 'PASS' if all(r['passed'] for r in details.values()) else 'FAIL',
              'torch': torch.__version__, 'threads': 2, 'device': 'cpu', 'frames': 700,
              'source_summary_sha256': audit.visual.digest(run / 'summary.json'),
              'details': details, 'recomputed_metrics': matrices,
              'model_promotion': False, 'full_pipeline_10hz_verified': False}
    with output.open('x') as stream:
        json.dump(result, stream, indent=2, sort_keys=True)
        stream.write('\n')
    print(json.dumps({'status': result['status'], 'frames': result['frames']}), flush=True)


if __name__ == '__main__':
    main()
