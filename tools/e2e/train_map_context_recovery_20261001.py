#!/usr/bin/env python3
"""HH_261001 - Compare fixed-budget map continuation and spatial context on GPU0."""

from __future__ import annotations
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import time

WORKSPACE = Path('/home/pro6000/personal/hwanhong/portable_e2e')
NAME = 'map_context_recovery_20261001_v1'
SOURCE = WORKSPACE / 'source_worktrees' / NAME
OUTPUT = WORKSPACE / 'runs/diagnostics' / NAME
DATA = WORKSPACE.parent / 'dataset/map_town02_p00_route_holdout_20260929_v1'
PARENT = WORKSPACE / 'runs/diagnostics/map_town02_p00_holdout_20260929_v1'
PARENT_SHA = '04614038b3d77a38f79b5a043c995b78ef6ee1d64537575fb2903c261ea5a746'
SUMMARY_SHA = 'bd0d9a100ca8e8420cbaea856c4566c3985683e7040782bcd7138f83d1d488f6'
DATA_SHA = 'f4764b9aa9d015f3d136a145f6194dd60bda8900c27dd2423de6145b9f472834'
STEPS, BATCH, SEED = 10000, 8, 20261001
LR, WEIGHT_DECAY = 3e-5, 1e-4
GPU_UUID = 'GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5'


def require(ok, message):
    if not ok:
        raise ValueError(message)


def digest(path):
    require(path.is_file() and not path.is_symlink(), f'regular file required: {path}')
    sha = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            sha.update(chunk)
    return sha.hexdigest()


def validate_environment():
    # HH_261001 - Resolve every ancestor, never install packages or choose another GPU.
    require(os.uname().nodename == 'pro6000', 'training host must be pro6000')
    require(Path(sys.prefix) == WORKSPACE / 'venvs/py312', 'personal venv required')
    require(Path(__file__).resolve().parent == SOURCE and Path.cwd().resolve() == SOURCE,
            'frozen source directory required')
    for path in (WORKSPACE, SOURCE, DATA, PARENT, OUTPUT.parent):
        require(path.is_dir() and path.resolve() == path, f'unsafe personal directory: {path}')
    require(not OUTPUT.exists() and not OUTPUT.is_symlink(), 'fresh output required')
    expected = {'CUDA_VISIBLE_DEVICES': GPU_UUID, 'CUBLAS_WORKSPACE_CONFIG': ':4096:8',
                'PYTHONNOUSERSITE': '1', 'OMP_NUM_THREADS': '16', 'MKL_NUM_THREADS': '16'}
    require(all(os.environ.get(key) == value for key, value in expected.items()),
            'fixed GPU0 environment differs')
    require(not any(os.environ.get(key) for key in
                    ('PYTHONPATH', 'PYTHONHOME', 'PYTHONSTARTUP', 'PYTHONUSERBASE')),
            'inherited Python overrides forbidden')


def load_array(np, manifest, split):
    ref = manifest['files'][split]
    require(ref['file'] == split + '.npz', 'unexpected split path')
    path = DATA / ref['file']
    require(digest(path) == ref['sha256'], f'split bytes differ: {split}')
    with np.load(path, allow_pickle=False) as archive:
        require(set(archive.files) == {'rgb', 'target'}, 'array keys differ')
        rgb, target = archive['rgb'].copy(), archive['target'].copy()
    count = ref['count']
    require(rgb.shape == (count, 360, 640, 3) and target.shape == (count, 360, 640)
            and rgb.dtype == target.dtype == np.uint8 and int(target.max()) < 7,
            'native seven-class data contract differs')
    require(np.bincount(target.ravel(), minlength=7).tolist() == ref['class_pixels'],
            'source class histogram differs')
    return rgb, target


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--validate-only', action='store_true')
    args = parser.parse_args()
    validate_environment()
    bundle = json.loads((SOURCE / 'bundle.json').read_text())
    require(set(bundle) == {'schema', 'files'} and bundle['schema'] == 'map_context_source.v1',
            'source bundle schema differs')
    required = {Path(__file__).name, 'map_context_recovery.py', 'map_perception.py',
                'map_learning_balance.py', 'dual_recovery_objectives.py',
                'train_map_photometric_pilot.py', 'train_map_town02_p00_route_holdout.py',
                'plan.md'}
    require(set(bundle['files']) == required, 'source bundle members differ')
    for name, expected in bundle['files'].items():
        require(digest(SOURCE / name) == expected, 'source changed: ' + name)
    require(digest(DATA / 'manifest.json') == DATA_SHA, 'dataset manifest changed')
    require(digest(PARENT / 'B/checkpoint.pt') == PARENT_SHA, 'parent checkpoint changed')
    require(digest(PARENT / 'summary.json') == SUMMARY_SHA, 'parent summary changed')

    import numpy as np
    import torch
    import train_map_photometric_pilot as guard
    import train_map_town02_p00_route_holdout as contract
    from map_perception import MapPerception, LABELS, semantic_loss
    from map_context_recovery import MapContextRecovery
    from dual_recovery_objectives import paired_crop

    for name, expected in contract.SUPPORT_SHA256.items():
        require(digest(SOURCE / name) == expected, 'frozen baseline support differs')
    for name in ('map_perception', 'map_context_recovery', 'dual_recovery_objectives',
                 'train_map_photometric_pilot', 'train_map_town02_p00_route_holdout'):
        require(Path(sys.modules[name].__file__).resolve() == SOURCE / (name + '.py'),
                'import escaped frozen bundle: ' + name)
    manifest = json.loads((DATA / 'manifest.json').read_text())
    require(manifest['schema'] == contract.SCHEMA and
            {key: ref['count'] for key, ref in manifest['files'].items()} == contract.COUNTS,
            'frozen data counts differ')
    rgb, target = load_array(np, manifest, 'train_b')
    parent_summary = json.loads((PARENT / 'summary.json').read_text())
    parent_started = json.loads((PARENT / 'started.json').read_text())
    weights_np = np.array(parent_started['class_weights'], dtype=np.float32)
    require(weights_np.shape == (7,) and bool(np.isfinite(weights_np).all())
            and bool((weights_np > 0).all()), 'invalid fixed loss weights')
    receipt = {'source_bundle_sha256': digest(SOURCE / 'bundle.json'),
               'parent_checkpoint_sha256': PARENT_SHA, 'data_manifest_sha256': DATA_SHA,
               'train_frames': len(rgb), 'additional_steps_per_arm': STEPS,
               'batch_size': BATCH, 'learning_rate': LR, 'seed': SEED,
               'dev_arrays_opened': False, 'test_used': False}
    if args.validate_only:
        print(json.dumps({'status': 'CPU_PREFLIGHT_PASS', **receipt}), flush=True)
        return

    with guard.gpu0_lease():
        gpu = guard.preflight_gpu()
        device = torch.device('cuda:0')
        OUTPUT.mkdir()
        for name in (*required, 'bundle.json'):
            guard.copy_new(SOURCE / name, OUTPUT / 'source' / name)
        guard.copy_new(DATA / 'manifest.json', OUTPUT / 'data_manifest.json')
        guard.write_new(OUTPUT / 'started.json', {
            **receipt, 'gpu': gpu, 'weights': weights_np.tolist(),
            'single_factor': 'identity-initialized dilated residual context',
            'optimizer': 'fresh AdamW, both arms', 'weight_decay': WEIGHT_DECAY,
            'independent_dev_sites': 0, 'model_promotion': False})
        # HH_261001 - Checkpoint identity was verified before the restricted tensor loader.
        parent = torch.load(PARENT / 'B/checkpoint.pt', map_location='cpu', weights_only=True)
        weights = torch.tensor(weights_np, device=device)
        arm_info = {}
        sample_hashes = []
        crop_states = []
        for arm, cls in (('A', MapPerception), ('B', MapContextRecovery)):
            guard.seed_everything(SEED)
            model = cls().to(device)
            if arm == 'A':
                model.load_state_dict(parent['model_state_dict'], strict=True)
            else:
                model.load_baseline(parent['model_state_dict'])
            reference = MapPerception().to(device)
            reference.load_state_dict(parent['model_state_dict'], strict=True)
            sample = torch.tensor(rgb[:1].transpose(0, 3, 1, 2).copy(),
                                  dtype=torch.float32, device=device) / 255
            with torch.no_grad():
                parity = float((reference(sample) - model(sample)).abs().max())
            require(parity == 0.0, 'initial baseline output differs')
            del reference, sample
            optimizer = torch.optim.AdamW(model.parameters(), lr=LR, weight_decay=WEIGHT_DECAY)
            sampler = np.random.default_rng(SEED)
            crop_rng = np.random.default_rng(SEED + 1)
            sampling_sha = hashlib.sha256()
            arm_dir = OUTPUT / arm
            arm_dir.mkdir()
            model.train()
            torch.cuda.synchronize()
            start = time.monotonic()
            loss_sum = 0.0
            with (arm_dir / 'history.jsonl').open('x') as log:
                for step in range(1, STEPS + 1):
                    ids = sampler.integers(0, len(rgb), size=BATCH, dtype=np.int64)
                    sampling_sha.update(ids.tobytes())
                    pairs = [paired_crop(rgb[i], target[i], crop_rng) for i in ids]
                    x = torch.tensor(np.stack([p[0] for p in pairs]).transpose(0, 3, 1, 2).copy(),
                                     device=device, dtype=torch.float32) / 255
                    y = torch.tensor(np.stack([p[1] for p in pairs]).astype(np.int64), device=device)
                    optimizer.zero_grad(set_to_none=True)
                    loss = semantic_loss(model(x), y, weights)
                    require(bool(torch.isfinite(loss)), 'nonfinite loss')
                    loss.backward()
                    norm = torch.nn.utils.clip_grad_norm_(model.parameters(), 5.0)
                    require(bool(torch.isfinite(norm)), 'nonfinite gradient')
                    optimizer.step()
                    loss_sum += float(loss.detach())
                    if step % 100 == 0:
                        row = {'step': step, 'mean_loss_100': loss_sum / 100,
                               'wall_seconds': time.monotonic() - start}
                        log.write(json.dumps(row) + '\n')
                        log.flush()
                        print(json.dumps({'arm': arm, **row}), flush=True)
                        loss_sum = 0.0
            torch.cuda.synchronize()
            elapsed = time.monotonic() - start
            checkpoint = {'model_state_dict': model.state_dict(), 'labels': LABELS,
                          'optimizer_state_dict': optimizer.state_dict(), 'steps': STEPS,
                          'parent_steps': 2000, 'parent_checkpoint_sha256': PARENT_SHA,
                          'model_id': 'map_context_recovery_' + arm, 'deployable': False}
            with (arm_dir / 'checkpoint.pt').open('xb') as stream:
                torch.save(checkpoint, stream)
            # HH_261001 - Safe arrays permit local old-PyTorch verification without pickle loading.
            np.savez_compressed(arm_dir / 'weights.npz',
                                **{key: val.detach().cpu().numpy() for key, val in model.state_dict().items()})
            sample_hashes.append(sampling_sha.hexdigest())
            crop_states.append(crop_rng.bit_generator.state)
            arm_info[arm] = {'training_wall_seconds': elapsed,
                             'parameter_count': sum(p.numel() for p in model.parameters()),
                             'initial_max_absolute_error': parity,
                             'sample_sequence_sha256': sample_hashes[-1],
                             'final_crop_rng_state': crop_states[-1],
                             'checkpoint_sha256': digest(arm_dir / 'checkpoint.pt'),
                             'weights_sha256': digest(arm_dir / 'weights.npz')}
            guard.write_new(arm_dir / 'training.json', arm_info[arm])
            del model, optimizer, checkpoint
        require(sample_hashes[0] == sample_hashes[1] and crop_states[0] == crop_states[1],
                'A/B input or augmentation sequence differed')

        # HH_261001 - Open evaluation arrays only after BOTH final checkpoints are immutable.
        evaluation = {key: load_array(np, manifest, split) for key, split in
                      (('town01', 'dev_town01'), ('p02', 'holdout_town02_p02'))}
        results = {}
        for arm, cls in (('A', MapPerception), ('B', MapContextRecovery)):
            arm_dir = OUTPUT / arm
            model = cls().to(device)
            checkpoint = torch.load(arm_dir / 'checkpoint.pt', map_location=device, weights_only=True)
            model.load_state_dict(checkpoint['model_state_dict'], strict=True)
            model.eval()
            metrics = dict(arm_info[arm])
            for name, (images, labels) in evaluation.items():
                predictions = []
                confusion = np.zeros((7, 7), dtype=np.int64)
                with torch.no_grad():
                    for begin in range(0, len(images), BATCH):
                        x = torch.tensor(images[begin:begin + BATCH].transpose(0, 3, 1, 2).copy(),
                                         device=device, dtype=torch.float32) / 255
                        pred = model(x).argmax(1).cpu().numpy().astype(np.uint8)
                        predictions.append(pred)
                        encoded = labels[begin:begin + BATCH].astype(np.int64) * 7 + pred
                        confusion += np.bincount(encoded.ravel(), minlength=49).reshape(7, 7)
                metrics[name] = contract.metrics_from_confusion(confusion, LABELS, np)
                np.savez_compressed(arm_dir / (name + '_predictions.npz'),
                                    prediction=np.concatenate(predictions))
            metrics['original_fixed_gates'] = contract.probe_gates(parent_summary['A'], metrics)
            guard.write_new(arm_dir / 'metrics.json', metrics)
            results[arm] = metrics
            del model, checkpoint
        guard.write_new(OUTPUT / 'summary.json', {
            'status': 'FIXED_CONTEXT_AB_COMPLETE_RESEARCH_ONLY', **results,
            'identical_sample_and_crop_sequences': True, 'model_promotion': False,
            'independent_dev_sites': 0, 'test_used': False,
            'dev_used_for_optimization': False,
            'context_minus_continuation_iou': {
                split: {name: None if results['A'][split]['iou'][name] is None
                        or results['B'][split]['iou'][name] is None else
                        results['B'][split]['iou'][name] - results['A'][split]['iou'][name]
                        for name in LABELS} for split in evaluation}})
        print('FIXED_CONTEXT_AB_COMPLETE_RESEARCH_ONLY', flush=True)


if __name__ == '__main__':
    main()
