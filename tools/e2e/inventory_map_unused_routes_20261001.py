#!/usr/bin/env python3
"""HH_261001 - Hash all unused Town01 sensor frames without granting TRAIN admission."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / 'storage/experiments/training/2026-09-27'
OUT = ROOT / 'docs/assets/validation/2026-10-01/map_unused_route_inventory'
PARENT = ROOT / 'storage/datasets/prepared/map_expanded_20260926_ai_v2/manifest.json'
ORIGINAL = ROOT / 'storage/experiments/training/2026-09-29/map_town02_p00_route_holdout_data_v1/manifest.json'
PAIRED = ROOT / 'storage/experiments/training/2026-10-01/map_appearance_matched_20261001_v1'
AUDIT = ROOT / 'docs/assets/validation/2026-09-27/map_sign_probe/summary.json'
LABELS = ['other', 'road', 'sidewalk', 'lane_marking', 'traffic_light_fixture', 'traffic_sign', 'pole']
TAGS = {1: 1, 2: 2, 24: 3, 7: 4, 8: 5, 6: 6}
COLORS = np.asarray([(31,41,55),(122,129,138),(159,133,173),(248,211,70),
                     (237,93,79),(229,65,185),(81,190,205)], dtype=np.uint8)
PINS = {
    PARENT: 'e9805574a252e72f8e81fcd434156da7782b25e8e965b8078f976cf4f234892c',
    ORIGINAL: 'f4764b9aa9d015f3d136a145f6194dd60bda8900c27dd2423de6145b9f472834',
    PAIRED/'manifest.json': '32ed912505e0b215bfc54e563fcdba7553aa25412ba22d90b45bfa00339e93c8',
    AUDIT: 'a6de99f1ac6e09187cd01fba1a33df9f727240b6bd856e1c64d0237f667f3c43',
}


def require(ok, message):
    if not ok:
        raise ValueError(message)


def digest(path):
    require(path.is_file() and not path.is_symlink(), 'Missing/symlinked input: '+str(path))
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            h.update(chunk)
    return h.hexdigest()


def read(path):
    return json.loads(path.read_text())


def xy(rows):
    result = np.asarray([[r['ego_x'], r['ego_y']] for r in rows], dtype=np.float64)
    require(result.ndim == 2 and result.shape[1] == 2 and np.isfinite(result).all(), 'Invalid ego positions')
    return result


def nearest(a, b):
    return np.sqrt(((a[:, None] - b[None])**2).sum(-1)).min(1)


def historical_retention(positions):
    # HH_261001 - Reproduce the old spacing statistic, never a new quality filter.
    kept = [0]
    for i in range(1, len(positions)):
        if np.linalg.norm(positions[i] - positions[kept[-1]]) >= .25:
            kept.append(i)
    return kept


def capture_gate(plan, result, summary):
    require(plan.get('map') == summary.get('town') == 'Town01', 'Town01 only')
    require(plan.get('quality') == 'Epic' and plan.get('capture_mode') == 'mapping-probe', 'Capture contract')
    require(result.get('exit_code') == 0 and result.get('source_bytes_unchanged_and_archived') is True,
            'Failed owner is never admitted')
    require(summary.get('status') == 'COMPLETE_ORACLE_PLUMBING_ONLY'
            and summary.get('physics_driving') is True and summary.get('collision_events') == 0
            and summary.get('live_replay_exact_match') is True and summary.get('cleanup_errors') == []
            and summary.get('frame_count') == 240, 'Failed/incomplete capture')


def collect(name, old, parent, comparison_hashes):
    root = BASE/('map_sign_native_'+name+'_probe')
    source = root/'mapping'
    plan, started, result, summary = [read(p) for p in (root/'owner_plan.json', root/'owner_started.json',
                                                        root/'owner_result.json', source/'summary.json')]
    capture_gate(plan, result, summary)
    refs = {}
    for relative, key in [('owner_plan.json','owner_plan_sha256'), ('owner_result.json','owner_result_sha256'),
                           ('mapping/summary.json','mapping_summary_sha256')]:
        require(digest(root/relative) == old[key], 'Historical receipt changed')
    route = Path(plan['route_path'])
    require(digest(route) == plan['route_sha256'] == summary['route_sha256'] == old['route_sha256'], 'Route changed')
    refs[str(route.relative_to(ROOT))] = digest(route)
    for relative in ('owner_plan.json','owner_started.json','owner_result.json','mapping/summary.json','mapping/records.json'):
        refs[str((root/relative).relative_to(ROOT))] = digest(root/relative)
    require(set(result['source_checks']) == set(plan['source_sha256']) and all(result['source_checks'].values()), 'Source check failed')
    for relative, expected in plan['source_sha256'].items():
        p = root/'provenance'/relative
        require(digest(p) == expected, 'Archived source changed')
        refs[str(p.relative_to(ROOT))] = expected
    for stage in ('ready','after_capture','stopped'):
        p = root/'lifecycle'/(stage+'.json')
        life = read(p)
        require(life['status'] == 'PASS' and life['expected_map'] == 'Town01'
                and life['owner_pid'] == started['server_pid']
                and (stage != 'stopped' or life['port_released'] is True), 'Lifecycle failed')
        refs[str(p.relative_to(ROOT))] = digest(p)
    worker = root/'provenance/scripts/e2e/collect_carla_mapping_probe.py'
    text = worker.read_text()
    require('world.set_weather(carla.WeatherParameters.ClearNoon)' in text
            and 'bp.set_attribute("enable_postprocess_effects", "false")' in text, 'Archived appearance contract changed')
    rows = read(source/'records.json')
    require(len(rows) == 240, 'Full capture required')
    positions = xy(rows)
    kept = historical_retention(positions)
    require(len(kept) == old['retained_frame_count'], 'Historical spacing selection differs')
    skipped = sorted(set(range(240))-set(kept))
    frames, panels, counts, depths, pixel_hashes = [], [], [], [], []
    previous = None
    for i, row in enumerate(rows):
        require(row['stem'] == f'frames/{i:04d}', 'Frame sequence changed')
        if previous is not None:
            require(row['frame'] == previous['frame']+1 and abs(row['timestamp_s']-previous['timestamp_s']-.1) < 1e-5,
                    'Frame/time gap')
        previous = row
        rgb_path, sensor_path = source/(row['stem']+'.png'), source/(row['stem']+'.npz')
        rgb_sha, sensor_sha = digest(rgb_path), digest(sensor_path)
        with Image.open(rgb_path) as image:
            require(image.size == (640,360) and image.mode == 'RGB', 'RGB format')
            rgb = np.asarray(image).copy()
        pixel_hashes.append(hashlib.sha256(rgb.tobytes()).hexdigest())
        with np.load(sensor_path, allow_pickle=False) as data:
            require(set(data.files) == {'semantic','depth','world_from_camera'}, 'Raw sensor ABI')
            semantic, depth, pose = data['semantic'], data['depth'], data['world_from_camera']
            require(semantic.shape == depth.shape == (360,640,4) and semantic.dtype == depth.dtype == np.uint8,
                    'Depth/semantic ABI')
            require(pose.shape == (4,4) and np.isfinite(pose).all() and np.allclose(pose[3], [0,0,0,1]), 'Camera pose')
            encoded = depth[...,2].astype(np.uint32)+256*depth[...,1].astype(np.uint32)+65536*depth[...,0].astype(np.uint32)
            depths.append([int(encoded.min()), int(encoded.max()), int(np.count_nonzero(encoded == 0))])
            target = np.zeros((360,640), dtype=np.uint8)
            for tag, label in TAGS.items():
                target[semantic[...,2] == tag] = label
        pixels = np.bincount(target.ravel(), minlength=7).tolist()
        counts.append(pixels)
        frames.append({'sample_id':name+':'+row['stem'], 'capture_index':i, 'frame':row['frame'],
                       'timestamp_s':row['timestamp_s'], 'rgb_path':str(rgb_path.relative_to(ROOT)), 'rgb_sha256':rgb_sha,
                       'sensor_path':str(sensor_path.relative_to(ROOT)), 'sensor_sha256':sensor_sha,
                       'ego_xy':positions[i].tolist(), 'class_pixels':pixels,
                       'historically_retained':i in kept, 'proposed_include':True})
        if i in (0,119,239):
            panels.append(np.concatenate([rgb, COLORS[target]], axis=0))
    counts = np.asarray(counts, dtype=np.int64)
    gaps = {split:nearest(positions, xy(parent['splits'][split]['source_records'])) for split in ('train','dev')}
    paired_rows = read(ROOT/'storage/experiments/validation/2026-10-01/map_paired_town01_p01_20261001_v2/mapping/records.json')
    gaps['paired_town01'] = nearest(positions, xy(paired_rows))
    require(float(gaps['dev'].min()) >= 80, 'DEV corridor overlap: cannot propose this route')
    # HH_261001 - Paired registry hashes decoded uint8 pixels, not compressed PNG files.
    duplicate_counts = {key:sum(h in values for h in
        (pixel_hashes if key == 'paired_false_true480_decoded_pixels' else [f['rgb_sha256'] for f in frames]))
        for key,values in comparison_hashes.items()}
    require(not any(duplicate_counts.values()), 'Exact reference RGB duplication')
    speeds = np.asarray([r['speed_mps'] for r in rows])
    summary_out = {'capture_integrity':'PASS', 'train_eligible':False, 'capture_root':str(root.relative_to(ROOT)),
        'raw_frames':240, 'proposed_frames':240, 'invalid_frames':[], 'dropped_frames':[],
        'class_pixels_all240':dict(zip(LABELS, counts.sum(0).tolist())),
        'class_positive_frames_all240':dict(zip(LABELS, (counts>0).sum(0).tolist())),
        'historical_retained_count':len(kept), 'historical_skipped_indices':skipped,
        'historical_skipped_speed_min_max_mps':[float(speeds[skipped].min()),float(speeds[skipped].max())],
        'historical_retained_class_pixels':dict(zip(LABELS,counts[kept].sum(0).tolist())),
        'historical_skip_reason':'<0.25m from last retained ego XY; not corrupt or invalid frames',
        'weather_configured':'ClearNoon', 'rgb_postprocess_configured':False,
        'configuration_evidence':'hashed archived collector; no original per-sensor attribute receipt',
        'depth_all240_present':True, 'finite_pose_all240':True, 'oracle_depth_pose_model_inputs':False,
        'depth_encoded_min_max':[min(x[0] for x in depths),max(x[1] for x in depths)],
        'depth_zero_pixels':sum(x[2] for x in depths), 'travel_m':summary['travel_m'],
        'minimum_ego_distance_m':{k:float(v.min()) for k,v in gaps.items()},
        'rgb_duplicates_against_references':duplicate_counts,
        'duplicate_hash_abi':'PNG file bytes for historical sources; decoded HWC uint8 for paired registry',
        'town02_holdout_geometry':'Different town coordinate systems; Euclidean distances not meaningful',
        'source_refs':refs, 'frames':frames}
    return summary_out, positions, np.concatenate(panels,axis=1)


def build(output):
    require(not output.exists(), 'Fresh output required')
    for p, sha in PINS.items():
        require(digest(p) == sha, 'Frozen reference changed: '+str(p))
    parent, original, old, pair_manifest = read(PARENT), read(ORIGINAL), read(AUDIT), read(PAIRED/'manifest.json')
    require(digest(PAIRED/'paired_samples.json') == pair_manifest['paired_samples_sha256'], 'Paired sample identity changed')
    hashes = {split:set(v for p,v in parent['splits'][split]['source_hashes'].items() if p.endswith('.png'))
              for split in ('train','dev')}
    hashes['train'].update(r['rgb_sha256'] for r in original['provenance']['p00_retained_raw_pairs'])
    hashes['town02_p02_holdout'] = set(r['rgb_sha256'] for r in original['provenance']['p02_holdout_raw_pairs'])
    hashes['paired_false_true480_decoded_pixels'] = {r[k] for r in read(PAIRED/'paired_samples.json')
                                    for k in ('rgb_false_sha256','rgb_true_sha256')}
    reports, positions, panels = {}, {}, {}
    for name, historical in [('p02','p02_right'),('p04','p04_left')]:
        reports[name],positions[name],panels[name] = collect(name,old['captures'][historical],parent,hashes)
    overlap = {name:nearest(positions[name],positions[other]) for name,other in [('p02','p04'),('p04','p02')]}
    inventory = {'schema':'portable_e2e.map_unused_route_candidate_inventory.v1',
        'status':'CANDIDATE_ONLY_PENDING_NARROW_STUDY_AUTHORIZATION', 'train_eligible':False,
        'dataset_admitted':False, 'fit_authorized':False, 'new_site_count':0, 'new_weather_count':0,
        'candidate_raw_frames':480, 'proposed_frames':480, 'label_order':LABELS,
        'historical_small_sign_rejection_preserved':True, 'ctrack_train_eligible':False,
        'protected_raw_opened':False, 'reference_hashes':{str(p.relative_to(ROOT)):h for p,h in PINS.items()},
        'builder_sha256':digest(Path(__file__)), 'captures':reports,
        'inter_candidate_minimum_distance_m':float(overlap['p02'].min()),
        'inter_candidate_positions_within_1m':{k:int((v<1).sum()) for k,v in overlap.items()},
        'overlap_caveat':'Same physical Town, partly shared road, not independent routes/sites. All 480 retained.',
        'fixed_review_indices':[0,119,239], 'model_inference_performed':False}
    output.mkdir(parents=True)
    for name,panel in panels.items():
        Image.fromarray(panel).save(output/(name+'_fixed_rgb_oracle.png'))
    inventory['fixed_panels_sha256'] = {p.name:digest(p) for p in output.glob('*.png')}
    with (output/'candidate_manifest.json').open('x') as stream:
        json.dump(inventory,stream,indent=2,sort_keys=True)
        stream.write('\n')
    # HH_261001 - Reread every pinned raw input after computation; never alter sources.
    for report in reports.values():
        for relative, sha in report['source_refs'].items():
            require(digest(ROOT/relative) == sha, 'Source changed during audit')
        for frame in report['frames']:
            for kind in ('rgb','sensor'):
                require(digest(ROOT/frame[kind+'_path']) == frame[kind+'_sha256'], 'Raw changed during audit')
    require(sum(p.stat().st_size for p in output.iterdir()) < 2*1024*1024, 'Bounded artifact budget exceeded')
    print(json.dumps({'status':inventory['status'],'manifest_sha256':digest(output/'candidate_manifest.json'),
                      'bytes':sum(p.stat().st_size for p in output.iterdir()),'all480_source_fingerprints_unchanged':True}))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=OUT/'audit_v1')
    build(parser.parse_args().output)
