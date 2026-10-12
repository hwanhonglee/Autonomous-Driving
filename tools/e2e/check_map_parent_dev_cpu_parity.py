"""HH_260906 - Quantify local CPU versus archived GPU DEV predictions without retraining."""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from scripts.e2e.evaluate_map_town06_cross_town import (
    COLORS, LABELS, PARENT, PARENT_RESULT, PARENT_SHA256, ROOT, class_metrics, digest, load_model, require,
)

DATA = ROOT / 'storage/datasets/prepared/map_expanded_20260926_ai_v2'
OUTPUT = ROOT / 'docs/assets/validation/2026-09-27/map_town06_cross_town_eval'
DATA_MANIFEST_SHA256 = 'e9805574a252e72f8e81fcd434156da7782b25e8e965b8078f976cf4f234892c'


def confusion_matrix(target, prediction):
    require(target.shape == prediction.shape and target.dtype == prediction.dtype == np.uint8,
            'aligned uint8 target/prediction required')
    n = len(LABELS)
    return np.bincount((target.astype(np.int64) * n + prediction).ravel(),
                       minlength=n * n).reshape(n, n)


def render_difference(path, index, rgb, target, archived, replay):
    font_path = Path('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf')
    font = ImageFont.truetype(str(font_path), 17) if font_path.is_file() else ImageFont.load_default()
    canvas = Image.new('RGB', (1280, 812), (16, 23, 33)); draw = ImageDraw.Draw(canvas)
    draw.text((10, 5), f'AI v2 DEV frame {index:04d} | CPU versus archived GPU argmax | red pixels differ | NO model selection',
              fill='white', font=font)
    difference = np.zeros_like(rgb); difference[archived != replay] = (255, 60, 60)
    panels = (('Original RGB', rgb), ('CARLA target', COLORS[target]),
              ('Archived GPU prediction', COLORS[archived]), ('CPU replay differences in red', difference))
    for j, (title, image) in enumerate(panels):
        left = (j % 2) * 640; top = 42 + 385 * (j // 2)
        draw.text((left + 8, top), title, fill='white', font=font)
        canvas.paste(Image.fromarray(image), (left, top + 25))
    canvas.save(path)


def main():
    require(os.environ.get('CUDA_VISIBLE_DEVICES') == '', 'explicit CPU-only CUDA mask required')
    require(OUTPUT.is_dir() and not (OUTPUT / 'dev_cpu_parity.json').exists(), 'fresh parity receipt required')
    manifest = json.loads((DATA / 'manifest.json').read_text())
    started = json.loads((PARENT_RESULT / 'started.json').read_text())
    summary = json.loads((PARENT_RESULT / 'summary.json').read_text())
    require(digest(DATA / 'manifest.json') == DATA_MANIFEST_SHA256 == started['data_manifest_sha256'],
            'parent data manifest differs')
    require(digest(DATA / 'dev.npz') == manifest['splits']['dev']['sha256']
            and digest(PARENT) == PARENT_SHA256 == summary['checkpoint_sha256'],
            'DEV or model bytes differ')
    with np.load(DATA / 'dev.npz', allow_pickle=False) as data:
        rgb, target = data['rgb'].copy(), data['target'].copy()
    with np.load(PARENT_RESULT / 'dev_predictions.npz', allow_pickle=False) as data:
        archived = data['prediction'].copy()
    require(rgb.shape == (118, 360, 640, 3) and target.shape == archived.shape == (118, 360, 640)
            and rgb.dtype == target.dtype == archived.dtype == np.uint8,
            'archived native DEV shape differs')
    archived_confusion = confusion_matrix(target, archived)
    require(archived_confusion.tolist() == summary['confusion'],
            'archived GPU prediction does not reproduce its summary')
    import torch
    torch.set_num_threads(2); torch.set_num_interop_threads(1)
    require(not torch.cuda.is_available(), 'CPU-only replay required')
    model = load_model(PARENT, PARENT_SHA256)
    replay = np.empty_like(archived)
    with torch.no_grad():
        for start in range(0, len(rgb), 8):
            # HH_260906 - This is the parent trainer's exact RGB uint8 -> float32 / 255 preprocessing.
            batch = torch.tensor(rgb[start:start + 8].transpose(0, 3, 1, 2).copy(),
                                 dtype=torch.float32) / 255
            replay[start:start + len(batch)] = model(batch).argmax(1).numpy().astype(np.uint8)
    replay_confusion = confusion_matrix(target, replay)
    per_frame = np.count_nonzero(replay != archived, axis=(1, 2))
    worst = int(np.argmax(per_frame))
    archived_stats = class_metrics(archived_confusion)
    replay_stats = class_metrics(replay_confusion)
    receipt = dict(schema='portable_e2e.map_parent_dev_cpu_parity.v1',
                   exact_prediction_match=bool(np.array_equal(replay, archived)),
                   exact_confusion_match=bool(np.array_equal(replay_confusion, archived_confusion)),
                   pixels_total=int(target.size), pixels_different=int(per_frame.sum()),
                   different_fraction=float(per_frame.sum() / target.size),
                   confusion_l1_difference=int(np.abs(replay_confusion - archived_confusion).sum()),
                   archived_confusion=archived_confusion.tolist(), cpu_confusion=replay_confusion.tolist(),
                   archived_metrics=archived_stats, cpu_metrics=replay_stats,
                   maximum_pixel_difference_frame=worst,
                   maximum_pixel_difference_in_frame=int(per_frame[worst]),
                   checkpoint_sha256=PARENT_SHA256, manifest_sha256=DATA_MANIFEST_SHA256,
                   dev_npz_sha256=digest(DATA / 'dev.npz'),
                   archived_prediction_sha256=digest(PARENT_RESULT / 'dev_predictions.npz'),
                   source_sha256=digest(Path(__file__)), torch_version=torch.__version__,
                   cpu_threads=2, batch_size=8, rgb_preprocessing='uint8 RGB CHW float32 / 255',
                   gpu_used=False, training_executed=False,
                   conclusion='Nonzero local CPU versus archived GPU argmax delta; report exact discrepancy, never claim bit parity.')
    render_difference(OUTPUT / 'dev_cpu_gpu_difference.png', worst, rgb[worst], target[worst],
                      archived[worst], replay[worst])
    with (OUTPUT / 'dev_cpu_parity.json').open('x') as stream:
        json.dump(receipt, stream, indent=2)
    print('PARENT_DEV_CPU_GPU_PARITY_MEASURED',receipt['pixels_different'],
          receipt['pixels_total'],receipt['confusion_l1_difference'],flush=True)


if __name__ == '__main__':
    main()
