#!/usr/bin/env python3
"""HH_261001 - Freeze references locally; materialize sensor-derived arrays only on the personal remote host."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import numpy as np

NAME = 'map_route_geometry_matched_20261001_v1'
CANDIDATE_SHA = '3c89ab9516e032122728d01bd72b1f9e5c31b5e287253e292143aa67e06382a6'
BASE_SHA = '32ed912505e0b215bfc54e563fcdba7553aa25412ba22d90b45bfa00339e93c8'
PARENT_SHA = 'ff5d7d03a43b859d21a0a3f6b01d815054d89d78e850730121f326e5b79c023e'
POLICY_SHA = '4de567323d3964ec877da831ec2acb08e445fe77c9da659f052edcf52ca53484'
PERSONAL = Path('/home/pro6000/personal/hwanhong')
PACKAGE = PERSONAL/'dataset'/NAME
REFERENCE = PERSONAL/'dataset/map_appearance_matched_20261001_v1'
LOCAL_MANIFEST = Path(__file__).resolve().parents[2]/'docs/assets/validation/2026-10-01/map_unused_route_inventory/audit_v1/candidate_manifest.json'


def require(ok, message):
    if not ok:
        raise ValueError(message)


def sha(path):
    require(path.is_file() and not path.is_symlink(), 'Missing/symlinked input')
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):
            h.update(block)
    return h.hexdigest()


def read(path):
    return json.loads(path.read_text())


def eligible_for_photo(index):
    # HH_261001 - New FALSE slots must not accidentally satisfy the old unbounded >=729 condition.
    return 729 <= int(index) < 1209


def control_replay():
    # HH_261001 - Match Town and postprocess mode, without looking at class labels or model errors.
    return np.random.default_rng(20261002).permutation(497).astype(np.int64)[:480]


def schedule():
    sampler = np.random.default_rng(20261001)
    ids = np.concatenate([sampler.permutation(1689).astype(np.int64) for _ in range(48)])[:80000].reshape(10000,8)
    crop_rng = np.random.default_rng(20261002)
    crops = np.empty((80000,4),np.int16)
    for i in range(80000):
        if crop_rng.random() >= .5:
            crops[i] = (0,0,360,640)
        else:
            scale = float(crop_rng.uniform(.65,1.))
            h,w = max(1,int(360*scale)),max(1,int(640*scale))
            crops[i] = (int(crop_rng.integers(360-h+1)),int(crop_rng.integers(640-w+1)),h,w)
    return ids,crops.reshape(10000,8,4)


def plan():
    ids,crops = schedule()
    replay = control_replay()
    rng = np.random.default_rng(20261003)
    trace = hashlib.sha256()
    applied = eligible = 0
    for index in ids.ravel():
        if eligible_for_photo(index):
            active = rng.random() < .5
            if active:
                rng.uniform(0.,1.)
                rng.uniform(.35,1.)
            trace.update(np.asarray([index,int(active)],np.int64).tobytes())
            eligible += 1
            applied += int(active)
    return {'schema':'portable_e2e.map_geometry_reference_plan.v1','study_id':NAME,
        'candidate_manifest_sha256':CANDIDATE_SHA,'reference_manifest_sha256':BASE_SHA,
        'parent_weights_sha256':PARENT_SHA,'fit_authorized':False,'local_array_materialization':False,
        'control_replay_rule':'PCG64 seed20261002 permutation of original Town01 indices0..496, first480; no replacement',
        'control_original_indices':replay.tolist(),'control_replay_int64_sha256':hashlib.sha256(replay.tobytes()).hexdigest(),
        'candidate_order':['p02:frames/0000..0239','p04:frames/0000..0239'],
        'pool_counts':{'original_false':729,'paired_true':480,'extra_false_slots':480,'total_slots':1689},
        'schedule':{'seed':20261001,'steps':10000,'batch':8,'full_epochs':47,'last_partial_epoch':617,
            'indices_int64_sha256':hashlib.sha256(ids.tobytes()).hexdigest(),
            'crops_int16_sha256':hashlib.sha256(crops.tobytes()).hexdigest(),
            'original_false_exposures':int((ids<729).sum()),
            'paired_true_exposures':int(((ids>=729)&(ids<1209)).sum()),
            'extra_false_exposures':int((ids>=1209).sum()),
            'extra_candidate_p02_exposures':int(((ids>=1209)&(ids<1449)).sum()),
            'extra_candidate_p04_exposures':int((ids>=1449).sum())},
        'photometric':{'archived_policy_sha256':POLICY_SHA,'eligible_slot_bounds':[729,1209],
            'seed':20261003,'eligible_calls':eligible,'applied_calls':applied,
            'sample_gate_trace_sha256':trace.hexdigest(),'final_rng_state':rng.bit_generator.state,
            'identical_both_arms':True,'new_false480_transformed':False},
        'class_weights':'unchanged original Town01 reference weights','model_inputs':['RGB'],
        'ctrack_train_eligible':False,'broad_registry_modified':False,'depth_pose_model_input':False}


def transfer_files(manifest):
    require(sha(manifest) == CANDIDATE_SHA, 'Candidate references changed')
    value = read(manifest)
    paths = set(value['reference_hashes'])
    for capture in value['captures'].values():
        paths.update(capture['source_refs'])
        for frame in capture['frames']:
            paths.update([frame['rgb_path'],frame['sensor_path']])
    for item in paths:
        p = Path(item)
        require(not p.is_absolute() and '..' not in p.parts, 'Unsafe transfer reference')
    return sorted(paths)


def validate_authorization(value, frozen_plan_sha):
    require(value.get('schema') == 'portable_e2e.map_geometry_package_authorization.v1'
        and value.get('study_id') == NAME and value.get('candidate_manifest_sha256') == CANDIDATE_SHA
        and value.get('reference_plan_sha256') == frozen_plan_sha
        and value.get('study_train_authorized') is True and value.get('package_authorized') is True
        and value.get('fit_authorized') is False and value.get('full480_source_qa_reviewed') is True
        and value.get('protected_dev_distance_reviewed') is True
        and value.get('raw_guardrail_ontology_caveat_reviewed') is True
        and value.get('ctrack_train_eligible') is False and value.get('broad_registry_modified') is False,
        'Exact parent-reviewed package-only study authorization required')


def materialize(args):
    # HH_261001 - No option can override the host, venv, source, destination, or fresh-output boundary.
    require(os.uname().nodename == 'pro6000' and Path(sys.prefix) == PERSONAL/'portable_e2e/venvs/py312',
            'Only the existing personal remote host/venv may materialize arrays')
    require(Path(__file__).resolve().parent == PACKAGE/'source' and PACKAGE.resolve() == PACKAGE,
            'Exact fresh personal study source required')
    require(sha(Path(__file__)) == args.expected_builder_sha256, 'Frozen remote packaging source changed')
    out = PACKAGE/'prepared'
    require(not out.exists() and not out.is_symlink(), 'Fresh remote prepared directory required')
    require(sha(PACKAGE/'candidate_manifest.json') == CANDIDATE_SHA, 'Candidate changed')
    frozen = PACKAGE/'reference_plan.json'
    require(sha(frozen) == args.expected_plan_sha256 and read(frozen) == plan(), 'Frozen replay/schedule changed')
    authority = PACKAGE/'package_authorization.json'
    require(sha(authority) == args.expected_approval_sha256, 'Approval bytes changed')
    validate_authorization(read(authority),args.expected_plan_sha256)
    require(sha(REFERENCE/'manifest.json') == BASE_SHA, 'Reference package changed')
    reference = read(REFERENCE/'manifest.json')
    for name,ref in reference['files'].items():
        require(sha(REFERENCE/name) == ref['sha256'], 'Original immutable reference changed')
    raw = PACKAGE/'raw'
    candidates = read(PACKAGE/'candidate_manifest.json')
    source_hashes = dict(candidates['reference_hashes'])
    for capture in candidates['captures'].values():
        source_hashes.update(capture['source_refs'])
    for relative,expected in source_hashes.items():
        require(sha(raw/relative) == expected, 'Transferred source receipt mismatch')
    rgb = np.empty((480,360,640,3),np.uint8)
    targets = np.empty((480,360,640),np.uint8)
    source_ids = []
    from PIL import Image
    for offset,name in [(0,'p02'),(240,'p04')]:
        capture = candidates['captures'][name]
        require(capture['raw_frames'] == capture['proposed_frames'] == 240 and not capture['dropped_frames'], 'Whole capture required')
        for index,frame in enumerate(capture['frames']):
            require(index == frame['capture_index'] and frame['proposed_include'] is True, 'Selected/reordered frames')
            for kind in ('rgb','sensor'):
                require(sha(raw/frame[kind+'_path']) == frame[kind+'_sha256'], 'Raw hash mismatch')
            with Image.open(raw/frame['rgb_path']) as image:
                require(image.size == (640,360) and image.mode == 'RGB','Native RGB ABI')
                rgb[offset+index] = np.asarray(image)
            with np.load(raw/frame['sensor_path'],allow_pickle=False) as data:
                tags = data['semantic'][...,2]
            target = np.zeros((360,640),np.uint8)
            for tag,label in {1:1,2:2,24:3,7:4,8:5,6:6}.items():
                target[tags == tag] = label
            require(np.bincount(target.ravel(),minlength=7).tolist() == frame['class_pixels'], 'Target recount changed')
            targets[offset+index] = target
            source_ids.append(frame['sample_id'])
    out.mkdir()
    with (out/'new_false480.npz').open('xb') as stream:
        np.savez_compressed(stream,rgb=rgb,target=targets)
    ids,crops = schedule()
    with (out/'training_schedule.npz').open('xb') as stream:
        np.savez_compressed(stream,indices=ids,crops=crops,control_original_indices=control_replay())
    # HH_261001 - Recheck immutable raw/source hashes after materialization, before a success receipt.
    for relative,expected in source_hashes.items():
        require(sha(raw/relative) == expected, 'Transferred source changed during packaging')
    for capture in candidates['captures'].values():
        for frame in capture['frames']:
            for kind in ('rgb','sensor'):
                require(sha(raw/frame[kind+'_path']) == frame[kind+'_sha256'], 'Raw changed during packaging')
    result = {'schema':'portable_e2e.map_geometry_remote_package.v1','study_id':NAME,
        'candidate_manifest_sha256':CANDIDATE_SHA,'reference_plan_sha256':args.expected_plan_sha256,
        'approval_sha256':args.expected_approval_sha256,'source_ids':source_ids,
        'reference_root':str(REFERENCE),'reference_manifest_sha256':BASE_SHA,
        'reference_files':reference['files'],'common_class_weights':reference['common_class_weights'],
        'parent_weights_sha256':PARENT_SHA,'source_receipts_preserved':True,'fit_authorized':False,
        'files':{p.name:sha(p) for p in out.glob('*.npz')},'builder_sha256':sha(Path(__file__))}
    with (out/'manifest.json').open('x') as stream:
        json.dump(result,stream,indent=2,sort_keys=True)
        stream.write('\n')
    print(json.dumps({'status':'REMOTE_PACKAGE_BUILT_NO_FIT','manifest_sha256':sha(out/'manifest.json')}))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--print-plan',action='store_true')
    mode.add_argument('--print-files',action='store_true')
    mode.add_argument('--materialize-remote',action='store_true')
    parser.add_argument('--candidate-manifest',type=Path,default=LOCAL_MANIFEST)
    parser.add_argument('--expected-plan-sha256')
    parser.add_argument('--expected-approval-sha256')
    parser.add_argument('--expected-builder-sha256')
    args = parser.parse_args()
    if args.print_plan:
        print(json.dumps(plan(),indent=2,sort_keys=True))
    elif args.print_files:
        print('\n'.join(transfer_files(args.candidate_manifest)))
    else:
        materialize(args)
