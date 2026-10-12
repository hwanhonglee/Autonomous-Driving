#!/usr/bin/env python3
"""HH_261001 - Independently recount map A/B pixels and render full-FOV evidence."""

from __future__ import annotations
import json
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw
from scripts.e2e import render_map_town02_p00_holdout_visuals as visual
from scripts.e2e import train_map_town02_p00_route_holdout as contract

ROOT = Path(__file__).resolve().parents[2]
RUN = ROOT / 'storage/experiments/training/2026-10-01/map_context_recovery_20261001_v1'
OUTPUT = ROOT / 'docs/assets/validation/2026-10-01/map_context_recovery/visuals'


def render(split, index, rgb, target, parent, a, b):
    # HH_261001 - Never crop or stretch source pixels; keep all six native views aligned.
    canvas = Image.new('RGB', (1920, 872), (18, 27, 40))
    draw = ImageDraw.Draw(canvas)
    draw.text((12, 8), f'{split} | fixed frame {index} | research comparison, NOT Autoware driving',
              fill='white', font=visual.font(22))
    error = np.zeros((*target.shape, 3), dtype=np.uint8)
    error[:] = (190, 55, 55)
    error[b == target] = (35, 145, 70)
    panels = (('RGB input', rgb), ('CARLA ground truth', visual.COLORS[target]),
              ('Original p00 B | 2,000 steps', visual.COLORS[parent]),
              ('A | same structure +10,000 steps', visual.COLORS[a]),
              ('B | broader context +10,000 steps', visual.COLORS[b]),
              ('B error | green correct / red wrong', error))
    for n, (title, pixels) in enumerate(panels):
        left, top = n % 3 * 640, 42 + n // 3 * 400
        draw.text((left + 8, top), title, fill='white', font=visual.font(18))
        canvas.paste(Image.fromarray(pixels), (left, top + 28))
    draw.text((12, 841), 'Known research routes only. No independent DEV, HD-map approval, or model promotion.',
              fill='white', font=visual.font(18))
    return canvas


def main():
    require = visual.require
    require(not OUTPUT.exists(), 'fresh visualization directory required')
    manifest, old, _ = visual.load_contract(visual.DATA, visual.RUN)
    summary = visual.read_json(RUN / 'summary.json')
    started = visual.read_json(RUN / 'started.json')
    require(summary['status'] == 'FIXED_CONTEXT_AB_COMPLETE_RESEARCH_ONLY'
            and summary['model_promotion'] is False and summary['test_used'] is False
            and summary['independent_dev_sites'] == 0
            and started['data_manifest_sha256'] == visual.DATA_MANIFEST_SHA256
            and started['additional_steps_per_arm'] == 10000,
            'new run scope differs')
    bundle = visual.read_json(RUN / 'source/bundle.json')
    require(visual.digest(RUN / 'source/bundle.json') == started['source_bundle_sha256'],
            'archived source manifest differs')
    for name, sha in bundle['files'].items():
        require(visual.digest(RUN / 'source' / name) == sha, 'archived source differs: ' + name)
    checks, images, source_hashes = {}, {}, {}
    for arm in ('A', 'B'):
        for filename, field in (('checkpoint.pt', 'checkpoint_sha256'), ('weights.npz', 'weights_sha256')):
            require(visual.digest(RUN / arm / filename) == summary[arm][field], 'trained weights changed')
        history = [json.loads(row) for row in (RUN / arm / 'history.jsonl').read_text().splitlines()]
        require([row['step'] for row in history] == list(range(100, 10001, 100))
                and all(np.isfinite(row['mean_loss_100']) for row in history), 'incomplete training')
        require(summary[arm]['initial_max_absolute_error'] == 0.0, 'initial baseline parity failed')
        require(contract.probe_gates(old['A'], summary[arm]) == summary[arm]['original_fixed_gates'],
                'fixed gate calculation differs')
    require(summary['A']['sample_sequence_sha256'] == summary['B']['sample_sequence_sha256']
            and summary['A']['final_crop_rng_state'] == summary['B']['final_crop_rng_state'],
            'training sample/crop sequences differ')
    for name, spec in visual.SPLITS.items():
        ref = manifest['files'][spec['dataset']]
        path = visual.DATA / ref['file']
        require(visual.digest(path) == ref['sha256'], 'evaluation source changed')
        with np.load(path, allow_pickle=False) as data:
            rgb, target = data['rgb'], data['target']
        with np.load(visual.RUN / 'B' / (name + '_predictions.npz'), allow_pickle=False) as archive:
            parent = archive['prediction']
        predictions = []
        for arm in ('A', 'B'):
            path = RUN / arm / (name + '_predictions.npz')
            with np.load(path, allow_pickle=False) as archive:
                require(set(archive.files) == {'prediction'}, 'prediction keys differ')
                pred = archive['prediction']
            require(pred.dtype == np.uint8 and pred.shape == target.shape and int(pred.max()) < 7,
                    'native prediction ABI differs')
            matrix = visual.confusion(target, pred)
            require(matrix == summary[arm][name]['confusion'], 'pixel matrix recount differs')
            metrics = contract.metrics_from_confusion(np.array(matrix), visual.LABELS, np)
            require(metrics == summary[arm][name], 'derived seven-class metrics differ')
            source_hashes[f'{arm}/{name}_predictions.npz'] = visual.digest(path)
            checks[f'{arm}_{name}'] = {'frames_recounted': len(target), 'all_pixels_match': True}
            predictions.append(pred)
        images[name] = [(index, render(name, index, rgb[index], target[index], parent[index],
                                      predictions[0][index], predictions[1][index]))
                        for index in spec['indices']]
    OUTPUT.mkdir()
    files = {}
    for split, samples in images.items():
        for index, frame in samples:
            path = OUTPUT / f'{split}_{index:03d}_full_fov.png'
            frame.save(path)
            files[path.name] = visual.digest(path)
        path = OUTPUT / f'{split}_three_sampled_frames.gif'
        frames = [frame for _, frame in samples]
        frames[0].save(path, save_all=True, append_images=frames[1:], duration=1400, loop=0)
        files[path.name] = visual.digest(path)
    result = {'schema': 'portable_e2e.map_context_result_audit.v1', 'status': 'PASS',
              'checks': checks, 'prediction_frames_recounted': 700,
              'source_summary_sha256': visual.digest(RUN / 'summary.json'),
              'source_predictions_sha256': source_hashes, 'visuals_sha256': files,
              'gif_is_continuous_driving': False, 'independent_dev_sites': 0,
              'model_promotion': False,
              'fixed_gates': {arm: summary[arm]['original_fixed_gates'] for arm in ('A', 'B')}}
    with (OUTPUT.parent / 'audit.json').open('x') as stream:
        json.dump(result, stream, indent=2, sort_keys=True)
        stream.write('\n')
    print(json.dumps({'status': result['status'], 'frames': 700, 'files': len(files)}))


if __name__ == '__main__':
    main()
