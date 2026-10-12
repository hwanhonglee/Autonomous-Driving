#!/usr/bin/env python3
"""HH_261001 - Attribute frozen full-240 lane misses at matching sampled oracle pixels.

This is a read-only diagnostic. It does not call a model, project a new map,
change a threshold, or write an output file. Run with ``python3 -B``.
"""

from collections import Counter
from io import BytesIO
import hashlib
import json
import math
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
RUN = ROOT / 'docs/assets/validation/2026-10-01/map_geometry_full_projection/run_v1'
PREDICTIONS = ROOT / 'storage/experiments/validation/2026-10-01/map_geometry_endpoints_20261001_v1'
SUMMARY_SHA256 = 'b848700f12f740d1afb424187e4e5ef8fe3eb8af6819629b625da03da9397fd1'
PROJECTION_SHA256 = 'c42ab7d20454475f544b8073d6a23a104a0a2bb1cc2cbb7be092fa3e871cbbee'
FUSION_SHA256 = '264f9dda759f6986bde9c614e5cf3ae21680a22ae58d855db11564ceb3b3b867'
ROUTES = {'ctrack_left': 1375, 'ctrack_straight': 1172}
ARMS = ('control', 'geometry')
PREDICTED_CLASSES = ('other', 'road', 'sidewalk', 'lane_marking',
                     'traffic_light_fixture', 'traffic_sign', 'pole')
GROUPS = ('a_no_learned_observation', 'b_no_accepted_lane_evidence',
          'c_lane_evidence_loses_fusion', 'd_lane_dominant')


def require(condition, message):
    if not condition:
        raise ValueError(message)


def bound_bytes(path, expected):
    require(path.is_file() and not path.is_symlink(), f'ordinary input required: {path}')
    payload = path.read_bytes()
    require(hashlib.sha256(payload).hexdigest() == expected, f'frozen SHA differs: {path}')
    return payload


def frozen_grid(path, expected):
    with np.load(BytesIO(bound_bytes(path, expected)), allow_pickle=False) as archive:
        require(set(archive.files) == {'cell_xy', 'label_order', 'observed_frames', 'confidence_sum'},
                f'grid ABI differs: {path}')
        xy, order, frames, sums = (archive[key].copy() for key in
                                   ('cell_xy', 'label_order', 'observed_frames', 'confidence_sum'))
    require(xy.ndim == 2 and xy.shape[1] == 2 and order.shape == frames.shape == sums.shape == (len(xy), 6)
            and np.isfinite(sums).all(), f'grid dimensions differ: {path}')
    result = {}
    for index, key in enumerate(xy):
        insertion_order = [int(class_id) for class_id in order[index] if class_id >= 0]
        require(insertion_order and len(set(insertion_order)) == len(insertion_order), 'invalid label insertion order')
        # HH_261001 - Preserve the frozen map's first-insertion exact-tie behavior.
        dominant = max(insertion_order, key=lambda class_id: sums[index, class_id])
        result[tuple(map(int, key))] = (dominant, int(frames[index, 2]))
    require(len(result) == len(xy), 'duplicate grid coordinate')
    return result


def projected_cells(z, u, v, pose):
    # HH_261001 - Match frozen f, camera axes and 0.2 m floor quantization exactly.
    f = 640 / (2 * math.tan(math.radians(90) / 2))
    xyz = np.stack([z, (u - 320) * z / f, -(v - 180) * z / f, np.ones_like(z)])
    points = (pose @ xyz).T
    return [(math.floor(float(p[0]) / .2), math.floor(float(p[1]) / .2)) for p in points]


def group_for(key, learned):
    value = learned.get(key)
    if value is None:
        return GROUPS[0]
    if value[1] == 0:
        return GROUPS[1]
    return GROUPS[3] if value[0] == 2 else GROUPS[2]


def route_analysis(route, summary):
    binding = summary['bindings']['scenarios'][route]
    source = Path(binding['source'])
    require(source.is_dir() and not source.is_symlink(), 'frozen raw capture missing')
    grids = {}
    for arm in ('oracle',) + ARMS:
        name = f'{route}_{arm}_grid.npz'
        grids[arm] = frozen_grid(RUN / name, summary['artifact_sha256'][name])
    lane_cells = {key for key, value in grids['oracle'].items() if value[0] == 2}
    require(len(lane_cells) == ROUTES[route], 'original oracle-dominant lane denominator differs')
    predictions, b_cells = {}, {}
    for arm in ARMS:
        name = f'{arm}_true_all240_predictions.npz'
        with np.load(BytesIO(bound_bytes(PREDICTIONS / route / name,
                                        binding['predictions'][name])), allow_pickle=False) as archive:
            require(archive.files == ['prediction'], 'native prediction ABI differs')
            predictions[arm] = archive['prediction'].copy()
        require(predictions[arm].shape == (240, 360, 640)
                and predictions[arm].dtype == np.uint8 and int(predictions[arm].max()) < 7,
                'native full-240 prediction shape/class differs')
        b_cells[arm] = {key for key in lane_cells if group_for(key, grids[arm]) == GROUPS[1]}

    flags = {arm: {} for arm in ARMS}
    all_pixel_classes = {arm: Counter() for arm in ARMS}
    b_pixel_classes = {arm: Counter() for arm in ARMS}
    b_winning_confidence = {arm: [] for arm in ARMS}
    valid_raw_lane_pixels = sampled_lane_pixels = shifted_cells = 0
    require(len(binding['frames']) == 240, 'frozen frame denominator differs')
    for index, record in enumerate(binding['frames']):
        require(record['index'] == index, 'chronological source index differs')
        raw_name = record['stem'] + '.npz'
        with np.load(BytesIO(bound_bytes(source / raw_name,
                                        record['artifact_sha256']['.npz'])), allow_pickle=False) as archive:
            depth, semantic, pose = (archive[key].copy() for key in
                                     ('depth', 'semantic', 'world_from_camera'))
        require(depth.shape == semantic.shape == (360, 640, 4) and depth.dtype == semantic.dtype == np.uint8
                and pose.shape == (4, 4), 'native raw geometry ABI differs')
        # HH_261001 - Full-pixel count is a separate visibility statistic, not the BEV denominator.
        raw_lane_depth = depth[semantic[..., 2] == 24].astype(np.float64)
        raw_z = (raw_lane_depth[:, 2] + raw_lane_depth[:, 1] * 256
                 + raw_lane_depth[:, 0] * 65536) * (1000 / 16777215)
        valid_raw_lane_pixels += int(np.count_nonzero((raw_z > .1) & (raw_z < 40)))

        sampled_depth = depth[::3, ::3].astype(np.float64)
        z64 = (sampled_depth[..., 2] + sampled_depth[..., 1] * 256
               + sampled_depth[..., 0] * 65536) * (1000 / 16777215)
        valid = (semantic[::3, ::3, 2] == 24) & (z64 > .1) & (z64 < 40)
        v_index, u_index = np.nonzero(valid)
        z = z64[valid]
        sampled_lane_pixels += len(z)
        u, v = u_index * 3, v_index * 3
        oracle_cells = projected_cells(z, u, v, pose)
        # HH_261001 - Learned projection used the same raw depth encoded as float32 metres.
        learned_cells = projected_cells(z.astype(np.float32).astype(np.float64), u, v, pose)
        shifted_cells += sum(a != b for a, b in zip(oracle_cells, learned_cells))

        for arm in ARMS:
            name = f'{route}_{index:03d}_{arm}_confidence.npz'
            with np.load(BytesIO(bound_bytes(RUN / name,
                                            summary['artifact_sha256'][name])), allow_pickle=False) as archive:
                require(archive.files == ['confidence_stride3'], 'confidence ABI differs')
                confidence = archive['confidence_stride3'].copy()
            require(confidence.shape == (120, 214) and confidence.dtype == np.float32
                    and np.isfinite(confidence).all(), 'saved sampled confidence differs')
            classes = predictions[arm][index, ::3, ::3][valid]
            scores = confidence[valid]
            for oracle_cell, learned_cell, class_id, score in zip(oracle_cells, learned_cells, classes, scores):
                class_id, score = int(class_id), float(score)
                all_pixel_classes[arm][PREDICTED_CLASSES[class_id]] += 1
                if oracle_cell in b_cells[arm]:
                    b_pixel_classes[arm][PREDICTED_CLASSES[class_id]] += 1
                    b_winning_confidence[arm].append(score)
                if oracle_cell not in lane_cells:
                    continue
                if class_id == 3 and score < .5:
                    bit = 1  # lane argmax blocked by the frozen confidence gate
                elif class_id == 3 and oracle_cell == learned_cell:
                    bit = 2  # direct accepted lane at the same oracle cell
                elif class_id == 3:
                    bit = 4  # accepted lane, but in a different 0.2 m cell
                else:
                    bit = 8  # another winning class; its own confidence is not lane probability
                flags[arm][oracle_cell] = flags[arm].get(oracle_cell, 0) | bit

    result = {'oracle_dominant_lane_cells': len(lane_cells),
              'raw_valid_lane_pixels_all_image_positions': valid_raw_lane_pixels,
              'raw_valid_lane_pixels_on_stride3_lattice': sampled_lane_pixels,
              'sampled_pixel_cell_changed_by_float32_depth': shifted_cells, 'arms': {}}
    for arm in ARMS:
        groups = {name: Counter() for name in GROUPS}
        for key in lane_cells:
            bits = flags[arm].get(key, 0)
            require(bits, f'final oracle lane cell has no direct sampled raw lane pixel: {route} {key}')
            group = group_for(key, grids[arm])
            groups[group]['cells'] += 1
            for bit, name in ((1, 'has_lane_argmax_below_0p5'), (2, 'has_accepted_lane_same_cell'),
                              (4, 'has_accepted_lane_shifted_cell'), (8, 'has_nonlane_argmax')):
                if bits & bit:
                    groups[group][name] += 1
        # HH_261001 - An accepted same-cell lane must survive into the frozen learned grid.
        require(groups[GROUPS[0]]['has_accepted_lane_same_cell'] == 0
                and groups[GROUPS[1]]['has_accepted_lane_same_cell'] == 0,
                'same-cell accepted lane unexpectedly absent from frozen grid')
        quantiles = np.quantile(b_winning_confidence[arm], [.1, .5, .9]).tolist()
        result['arms'][arm] = {
            'oracle_lane_cell_groups': {name: dict(groups[name]) for name in GROUPS},
            'sampled_oracle_lane_pixel_predicted_classes': dict(all_pixel_classes[arm]),
            'sampled_oracle_lane_pixels_in_group_b_predicted_classes': dict(b_pixel_classes[arm]),
            'group_b_pixel_winning_class_confidence_q10_q50_q90': quantiles,
            'group_b_pixel_count': len(b_winning_confidence[arm]),
        }
        require(sum(group['cells'] for group in groups.values()) == len(lane_cells)
                and groups[GROUPS[1]]['cells'] == len(b_cells[arm])
                and sum(all_pixel_classes[arm].values()) == sampled_lane_pixels,
                'pixel or final-cell denominator differs')
    return result


def main():
    summary = json.loads(bound_bytes(RUN / 'summary.json', SUMMARY_SHA256))
    require(summary['status'] == 'FULL_RECORDING_ORACLE_ASSISTED_ACCUMULATION_DIAGNOSTIC'
            and summary['completed_frame_count'] == 480 and summary['policy']['stride'] == 3
            and summary['policy']['minimum_confidence'] == .5
            and summary['policy']['resolution_m'] == .2
            and summary['policy']['depth_strict_range_m'] == [.1, 40], 'frozen study policy differs')
    sources = {
        str(ROOT / 'learning/portable_e2e/mapping_projection.py'): PROJECTION_SHA256,
        str(ROOT / 'learning/portable_e2e/mapping.py'): FUSION_SHA256,
    }
    for name, expected in sources.items():
        bound_bytes(Path(name), expected)
        require(summary['sources'][name] == expected, 'frozen source binding differs')
    result = {'schema': 'portable_e2e.map_lane_raw_pixel_attribution.v1',
              'status': 'READ_ONLY_POST_HOC_FROZEN_EVIDENCE',
              'summary_sha256': SUMMARY_SHA256, 'projection_and_fusion_sha256': sources,
              'checked_input_counts': {'raw_npz': 480, 'prediction_npz': 4,
                                       'confidence_npz': 960, 'final_grid_npz': 6},
              'limitation': 'Only winning-class argmax/max-softmax was saved; nonwinning lane probability is unavailable.',
              'results': {route: route_analysis(route, summary) for route in ROUTES}}
    print(json.dumps(result, sort_keys=True, separators=(',', ':'), allow_nan=False))


if __name__ == '__main__':
    main()
