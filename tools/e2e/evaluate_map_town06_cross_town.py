"""HH_260906 - CPU-only exploratory Town06 evaluation of two frozen RGB map models."""
from __future__ import annotations

import argparse
import hashlib
import inspect
import json
import math
import os
from pathlib import Path
import time

import numpy as np
from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[2]
CAPTURE = ROOT / 'storage/experiments/training/2026-09-27/map_sign_town06_p06_scout_v1'
SCOUT_AUDIT = ROOT / 'docs/assets/validation/2026-09-27/map_sign_town06_p06/summary.json'
PARENT = ROOT / 'storage/experiments/training/2026-09-26/dual_data_ai_v1/mapping_ai_v2_checkpoint/checkpoint.pt'
PARENT_RESULT = ROOT / 'docs/assets/validation/2026-09-26/dual_data_ai/results/mapping_ai_v2'
CANDIDATE_RESULT = ROOT / 'docs/assets/validation/2026-09-27/map_sign_scale_ab_results'
DEFAULT_OUTPUT = ROOT / 'docs/assets/validation/2026-09-27/map_town06_cross_town_eval'
ROUTE_SHA256 = 'e27e95cb60d40657638183f7f78ce2e1f4f070067707ac4dd3f2f89dd1c52283'
PARENT_SHA256 = 'ae489cd99e48f6f68da3241e8296c705847398c23b8ba69b71ec024cef4ef380'
CANDIDATE_SHA256 = '30c3c56b8cb6f34f602c1e4396a2802441f9a909d68275480a54b4885dd87b3b'
MODEL_SOURCE_SHA256 = '1b0076c8f9d34192ab5aedf6cdf96ce2c14458e2bb49986685a6811376b0750b'
LABELS = ('other', 'road', 'sidewalk', 'lane_marking', 'traffic_light_fixture', 'traffic_sign', 'pole')
TAG_TABLE = {'1': 'road', '24': 'lane_marking', '2': 'sidewalk', '7': 'traffic_light_fixture',
             '8': 'traffic_sign', '6': 'pole'}
COLORS = np.array([[30, 30, 40], [67, 92, 123], [141, 88, 147], [246, 222, 90],
                   [246, 100, 70], [60, 220, 180], [179, 190, 199]], dtype=np.uint8)


def require(value, message):
    if not value:
        raise ValueError(message)


def digest(path):
    path = Path(path)
    require(path.is_file() and not path.is_symlink(), f'regular source required: {path}')
    result = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b''):
            result.update(block)
    return result.hexdigest()


def read_json(path):
    return json.loads(Path(path).read_text())


def retained_indices(records, minimum_distance_m=0.25):
    """HH_260906 - Match the scout's first-frame/last-retained XY spacing rule exactly."""
    require(math.isfinite(minimum_distance_m) and minimum_distance_m > 0, 'positive spacing required')
    chosen = []
    for index, row in enumerate(records):
        x, y = float(row['ego_x']), float(row['ego_y'])
        require(math.isfinite(x) and math.isfinite(y), 'finite ego XY required')
        if not chosen or math.hypot(x - float(records[chosen[-1]]['ego_x']),
                                    y - float(records[chosen[-1]]['ego_y'])) >= minimum_distance_m:
            chosen.append(index)
    return np.asarray(chosen, dtype=np.int64)


def semantic_target(semantic):
    """HH_260906 - Apply the unchanged CARLA tag-to-label table used by map data preparation."""
    require(semantic.shape == (360, 640, 4) and semantic.dtype == np.uint8,
            'native uint8 CARLA semantic frame required')
    lut = np.zeros(256, dtype=np.uint8)
    for tag, name in TAG_TABLE.items():
        lut[int(tag)] = LABELS.index(name)
    return lut[semantic[..., 2]]


def class_metrics(confusion):
    require(confusion.shape == (len(LABELS), len(LABELS)) and (confusion >= 0).all(),
            'nonnegative seven-class confusion required')
    classes = {}
    for i, name in enumerate(LABELS):
        tp = int(confusion[i, i]); target = int(confusion[i].sum()); predicted = int(confusion[:, i].sum())
        union = target + predicted - tp
        classes[name] = dict(target_pixels=target, predicted_pixels=predicted, true_positive_pixels=tp,
                             precision=tp / predicted if predicted else None,
                             recall=tp / target if target else None,
                             iou=tp / union if union else None)
    present = [row['iou'] for row in classes.values() if row['iou'] is not None]
    require(bool(present) and int(confusion.sum()) > 0, 'nonempty evaluation required')
    return dict(classes=classes, confusion=confusion.tolist(),
                mean_iou_present=float(np.mean(present)),
                pixel_accuracy=float(np.trace(confusion) / confusion.sum()))


def verify_sources():
    """HH_260906 - Reject wrong owner, route, semantic schema or frozen model before inference."""
    root = CAPTURE; mapping = root / 'mapping'; audit = read_json(SCOUT_AUDIT)
    owner_plan = read_json(root / 'owner_plan.json')
    owner_result = read_json(root / 'owner_result.json')
    owner_started = read_json(root / 'owner_started.json')
    capture = read_json(mapping / 'summary.json')
    records = read_json(mapping / 'records.json')
    proof = audit['owner_provenance']
    for key, path in (('owner_plan_sha256', root / 'owner_plan.json'),
                      ('owner_result_sha256', root / 'owner_result.json'),
                      ('owner_started_sha256', root / 'owner_started.json'),
                      ('mapping_summary_sha256', mapping / 'summary.json')):
        require(digest(path) == proof[key], f'scout owner audit hash differs: {key}')
    require(audit['schema'] == 'portable_e2e.map_sign_town06_p06_audit.v1'
            and audit['source'] == str(mapping.relative_to(ROOT))
            and audit['native_frame_wh'] == [640, 360]
            and audit['retention_rule'] == 'first frame, then >=0.25m from last retained ego XY',
            'Town06 scout audit contract differs')
    route = Path(owner_plan['route_path'])
    require(route.resolve().is_relative_to(ROOT.resolve()) and digest(route) == ROUTE_SHA256,
            'Town06 route bytes changed')
    require(owner_plan['route_sha256'] == owner_started['route_sha256'] == capture['route_sha256']
            == proof['route_sha256'] == ROUTE_SHA256, 'route receipt differs')
    require(owner_plan['map'] == owner_started['map'] == capture['town'] == proof['town'] == 'Town06',
            'wrong captured Town')
    require(owner_plan['capture_mode'] == owner_result['capture_mode'] == 'mapping-probe'
            and owner_result['exit_code'] == 0 and owner_result['source_bytes_unchanged_and_archived'] is True
            and all(owner_result['source_checks'].values()), 'capture ownership/source failed')
    source_hashes = owner_plan['source_sha256']
    require(len(source_hashes) == proof['archived_source_count'] == 13
            and set(source_hashes) == set(owner_result['source_checks']), 'source inventory differs')
    for relative, expected in source_hashes.items():
        name = Path(relative)
        require(not name.is_absolute() and '..' not in name.parts and digest(root / 'provenance' / name) == expected,
                f'archived collector source changed: {relative}')
    require(capture['status'] == 'COMPLETE_ORACLE_PLUMBING_ONLY'
            and capture['label_table'] == TAG_TABLE and capture['frame_count'] == len(records) == 240
            and capture['physics_driving'] is True and capture['learned_model_control'] is False
            and capture['oracle_semantics'] is True and capture['oracle_pose'] is True
            and capture['collision_events'] == 0 and capture['live_replay_exact_match'] is True
            and capture['hd_map_approved'] is False and not capture['cleanup_errors'],
            'Town06 raw capture contract differs')
    for index, row in enumerate(records):
        require(row['stem'] == f'frames/{index:04d}' and isinstance(row['frame'], int)
                and (index == 0 or (row['frame'] > records[index - 1]['frame']
                    and row['timestamp_s'] > records[index - 1]['timestamp_s'])),
                'frame order differs')
    retained = retained_indices(records)
    require(len(retained) == audit['town06']['retained_frames_0p25m'] == 232
            and len(records) == audit['town06']['raw_frames'], 'scout spacing/count differs')
    require(digest(PARENT) == PARENT_SHA256 and digest(CANDIDATE_RESULT / 'checkpoint.pt') == CANDIDATE_SHA256,
            'frozen checkpoint hash differs')
    parent_result = read_json(PARENT_RESULT / 'summary.json')
    candidate_result = read_json(CANDIDATE_RESULT / 'summary.json')
    parent_started = read_json(PARENT_RESULT / 'started.json')
    candidate_started = read_json(CANDIDATE_RESULT / 'started.json')
    require(parent_result['checkpoint_sha256'] == PARENT_SHA256
            and candidate_result['checkpoint_sha256'] == CANDIDATE_SHA256
            and candidate_started['parent_checkpoint_sha256'] == PARENT_SHA256
            and parent_started['data_manifest_sha256'] == candidate_started['data_manifest_sha256']
            and parent_result['deployment_approved'] is False
            and candidate_result['deployment_approved'] is False, 'frozen model receipt differs')
    model_source = ROOT / 'learning/portable_e2e/map_perception.py'
    require(digest(model_source) == digest(CANDIDATE_RESULT / 'source/map_perception.py') == MODEL_SOURCE_SHA256
            and digest(CANDIDATE_RESULT / 'source/train_map_native_ah.py') == candidate_started['source_sha256'],
            'frozen architecture/training source differs')
    return records, retained, dict(owner_plan_sha256=digest(root / 'owner_plan.json'),
                                   owner_result_sha256=digest(root / 'owner_result.json'),
                                   owner_started_sha256=digest(root / 'owner_started.json'),
                                   mapping_summary_sha256=digest(mapping / 'summary.json'),
                                   scout_audit_sha256=digest(SCOUT_AUDIT),
                                   route_sha256=ROUTE_SHA256, model_source_sha256=MODEL_SOURCE_SHA256,
                                   parent_checkpoint_sha256=PARENT_SHA256,
                                   candidate_checkpoint_sha256=CANDIDATE_SHA256,
                                   parent_training_source_sha256_recorded=parent_started['source_sha256'],
                                   candidate_training_source_sha256=candidate_started['source_sha256'])


def load_model(path, expected_sha256):
    """HH_260906 - Torch 1.8 lacks weights_only; load only a checked, owned checkpoint on CPU."""
    import torch
    from portable_e2e.map_perception import MapPerception

    require(digest(path) == expected_sha256, 'checkpoint changed before CPU load')
    args = dict(map_location='cpu')
    if 'weights_only' in inspect.signature(torch.load).parameters:
        args['weights_only'] = True
    checkpoint = torch.load(path, **args)
    require(tuple(checkpoint['labels']) == LABELS and checkpoint['input_shape'] == [3, 360, 640]
            and checkpoint['steps'] == 2000 and checkpoint['deployable'] is False,
            'checkpoint schema differs')
    model = MapPerception().cpu().eval()
    model.load_state_dict(checkpoint['model_state_dict'], strict=True)
    require(sum(p.numel() for p in model.parameters()) == 130503, 'architecture differs')
    return model


def render_panel(path, frame_index, rgb, target, parent, candidate):
    """HH_260906 - Show each native full field of view; no crop or synthetic improvement."""
    require(all(arr.shape[:2] == (360, 640) for arr in (rgb, target, parent, candidate)),
            'native images required')
    font_path = Path('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf')
    font = ImageFont.truetype(str(font_path), 17) if font_path.is_file() else ImageFont.load_default()
    canvas = Image.new('RGB', (1280, 812), (16, 23, 33)); draw = ImageDraw.Draw(canvas)
    draw.text((10, 5), f'Town06 frame {frame_index:04d} | selected cross-Town scout | CPU inference | NOT blind TEST / HD-map approval',
              fill='white', font=font)
    panels = (('RGB input', rgb), ('CARLA oracle label', COLORS[target]),
              ('AI v2 frozen prediction', COLORS[parent]), ('Rejected A/B frozen prediction', COLORS[candidate]))
    for index, (title, data) in enumerate(panels):
        left = 640 * (index % 2); top = 42 + 385 * (index // 2)
        draw.text((left + 8, top), title, fill='white', font=font)
        canvas.paste(Image.fromarray(data), (left, top + 25))
    canvas.save(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    require(os.environ.get('CUDA_VISIBLE_DEVICES') == '', 'explicit CUDA_VISIBLE_DEVICES empty string required')
    output = args.output.absolute()
    require(output.parent.resolve() == DEFAULT_OUTPUT.parent.resolve() and not output.exists(),
            'fresh dated validation output under the scoped parent required')
    records, retained, sources = verify_sources()
    import torch
    torch.set_num_threads(2)
    torch.set_num_interop_threads(1)
    require(not torch.cuda.is_available(), 'CPU-only inference required')
    models = {'ai_v2': load_model(PARENT, PARENT_SHA256),
              'sign_scale_ab_rejected': load_model(CANDIDATE_RESULT / 'checkpoint.pt', CANDIDATE_SHA256)}
    prediction = {name: np.empty((len(records), 360, 640), dtype=np.uint8) for name in models}
    frame_hashes = {}; sign_pixel_counts = []; selected = {}
    started = time.monotonic()
    with torch.no_grad():
        for index, row in enumerate(records):
            stem = row['stem']; rgb_path = CAPTURE / 'mapping' / (stem + '.png')
            semantic_path = CAPTURE / 'mapping' / (stem + '.npz')
            frame_hashes[stem] = dict(rgb_sha256=digest(rgb_path), semantic_npz_sha256=digest(semantic_path))
            with Image.open(rgb_path) as image:
                require(image.mode == 'RGB' and image.size == (640, 360), 'native RGB capture required')
                rgb = np.asarray(image).copy()
            with np.load(semantic_path, allow_pickle=False) as raw:
                target = semantic_target(raw['semantic'])
            sign_count = int(np.count_nonzero(target == LABELS.index('traffic_sign')))
            sign_pixel_counts.append(sign_count)
            if index in (17, 208):
                selected[index] = dict(rgb=rgb, target=target)
            image_tensor = torch.from_numpy(rgb.transpose(2, 0, 1).copy()).unsqueeze(0).float().div_(255)
            for name, model in models.items():
                predicted = model(image_tensor).argmax(1).squeeze(0).numpy().astype(np.uint8)
                prediction[name][index] = predicted
                if index in selected:
                    selected[index][name] = predicted.copy()
            if (index + 1) % 40 == 0:
                print(f'CPU_TOWN06_FRAMES {index + 1}/{len(records)}', flush=True)
    audit = read_json(SCOUT_AUDIT)
    examples = {row['stem']: row['total_sign_pixels'] for row in audit['example_frames']}
    require(all(examples.get(row['stem']) == count for row, count in zip(records, sign_pixel_counts)),
            'captured sign pixels differ from the scout audit')
    require(sum(sign_pixel_counts) == audit['town06']['raw_tag8_pixel_sum']
            and sum(sign_pixel_counts[i] for i in retained) == audit['town06']['retained_tag8_pixel_sum'],
            'scout sign total differs')
    summaries = {}
    retained_set = set(retained.tolist())
    for name, values in prediction.items():
        raw_confusion = np.zeros((len(LABELS), len(LABELS)), dtype=np.int64)
        kept_confusion = np.zeros_like(raw_confusion)
        for index, row in enumerate(records):
            semantic_path = CAPTURE / 'mapping' / (row['stem'] + '.npz')
            with np.load(semantic_path, allow_pickle=False) as raw:
                target = semantic_target(raw['semantic'])
            confusion = np.bincount((target.astype(np.int64) * len(LABELS) + values[index]).ravel(),
                                    minlength=len(LABELS) ** 2).reshape(len(LABELS), len(LABELS))
            raw_confusion += confusion
            if index in retained_set:
                kept_confusion += confusion
        summaries[name] = dict(raw_240=class_metrics(raw_confusion), retained_232=class_metrics(kept_confusion))
    parent = summaries['ai_v2']['retained_232']; candidate = summaries['sign_scale_ab_rejected']['retained_232']
    gate = dict(parent_dev_rejected=True,
                town06_sign_iou_better=(candidate['classes']['traffic_sign']['iou'] or 0)
                > (parent['classes']['traffic_sign']['iou'] or 0),
                town06_sign_precision_better=(candidate['classes']['traffic_sign']['precision'] or 0)
                > (parent['classes']['traffic_sign']['precision'] or 0),
                town06_lane_nonregression=(candidate['classes']['lane_marking']['iou'] or 0)
                >= (parent['classes']['lane_marking']['iou'] or 0),
                automatic_promotion=False, deployment_approved=False)
    output.mkdir(parents=False, exist_ok=False)
    with (output / 'predictions.npz').open('xb') as stream:
        np.savez_compressed(stream, ai_v2=prediction['ai_v2'],
                            sign_scale_ab_rejected=prediction['sign_scale_ab_rejected'],
                            retained_indices=retained)
    for index, frame in selected.items():
        render_panel(output / f'native_comparison_{index:04d}.png', index, frame['rgb'], frame['target'],
                     frame['ai_v2'], frame['sign_scale_ab_rejected'])
    summary = dict(schema='portable_e2e.town06_cross_town_map_eval.v1', status='EXPLORATORY_EVALUATION_ONLY',
                   town='Town06', route_sha256=ROUTE_SHA256, labels=LABELS,
                   input='original 640x360 RGB; CARLA oracle semantic labels only for evaluation',
                   raw_frames=240, retained_frames=232, retention_rule=audit['retention_rule'],
                   models=summaries, gate=gate, source_sha256={**sources, 'evaluator_sha256': digest(Path(__file__))},
                   input_manifest_sha256=None, predictions_sha256=digest(output / 'predictions.npz'),
                   wall_seconds=time.monotonic() - started, cpu_threads=2, gpu_used=False,
                   training_executed=False, scout_training_data_approved=False,
                   blind_independent_test=False,
                   limitations='Selected because of sign-size gap; one Town06 route/weather, correlated frames, CARLA oracle labels, no HD-map or driving approval')
    with (output / 'input_frames.json').open('x') as stream:
        json.dump(frame_hashes, stream, indent=2)
    summary['input_manifest_sha256'] = digest(output / 'input_frames.json')
    with (output / 'summary.json').open('x') as stream:
        json.dump(summary, stream, indent=2)
    print('TOWN06_CROSS_TOWN_EXPLORATORY_COMPLETE_NO_PROMOTION', flush=True)


if __name__ == '__main__':
    main()
