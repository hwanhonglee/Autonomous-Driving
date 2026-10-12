#!/usr/bin/env python3
"""HH_261001 - Independently recount the full candidate frames without TRAIN admission."""

import hashlib
import json
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT/'docs/assets/validation/2026-10-01/map_unused_route_inventory/audit_v1'
MANIFEST_SHA = '3c89ab9516e032122728d01bd72b1f9e5c31b5e287253e292143aa67e06382a6'


def sha(path):
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            value.update(chunk)
    return value.hexdigest()


def main():
    manifest = OUTPUT/'candidate_manifest.json'
    if sha(manifest) != MANIFEST_SHA:
        raise ValueError('Pinned candidate manifest changed')
    report = json.loads(manifest.read_text())
    if any(report[k] is not False for k in ('train_eligible','dataset_admitted','fit_authorized','ctrack_train_eligible')):
        raise ValueError('Unexpected admission')
    labels = {'road':1,'sidewalk':2,'lane_marking':24,'traffic_light_fixture':7,'traffic_sign':8,'pole':6}
    pixels = 0
    sources = 0
    for capture in report['captures'].values():
        if len(capture['frames']) != 240 or capture['invalid_frames'] or capture['dropped_frames']:
            raise ValueError('Full capture not retained')
        aggregate = {label:0 for label in report['label_order']}
        for i, frame in enumerate(capture['frames']):
            if frame['capture_index'] != i or frame['proposed_include'] is not True:
                raise ValueError('Frame selection changed')
            for kind in ('rgb','sensor'):
                if sha(ROOT/frame[kind+'_path']) != frame[kind+'_sha256']:
                    raise ValueError('Raw bytes changed')
            with np.load(ROOT/frame['sensor_path'],allow_pickle=False) as raw:
                tags = raw['semantic'][...,2]
                # HH_261001 - Count raw tags directly, independently of the builder's target lookup.
                count = {label:int(np.count_nonzero(tags == tag)) for label,tag in labels.items()}
                count['other'] = tags.size-sum(count.values())
                if [count[label] for label in report['label_order']] != frame['class_pixels']:
                    raise ValueError('Per-frame raw count mismatch')
                pixels += tags.size
                for label,n in count.items():
                    aggregate[label] += n
        if aggregate != capture['class_pixels_all240']:
            raise ValueError('Aggregate mismatch')
        for path,expected in capture['source_refs'].items():
            if sha(ROOT/path) != expected:
                raise ValueError('Source closure changed')
            sources += 1
    for relative,expected in {**report['reference_hashes'],**{
            str(OUTPUT.relative_to(ROOT)/name):value for name,value in report['fixed_panels_sha256'].items()}}.items():
        if sha(ROOT/relative) != expected:
            raise ValueError('Reference/panel changed')
    if pixels != 480*640*360:
        raise ValueError('Not all 480 frames counted')
    receipt = {'status':'INDEPENDENT_RAW_RECOUNT_PASS_CANDIDATE_ONLY','candidate_manifest_sha256':MANIFEST_SHA,
        'frames':480,'pixels':pixels,'source_receipts_verified':sources,
        'source_fingerprints_unchanged':True,'train_eligible':False,'fit_authorized':False,
        'verifier_sha256':sha(Path(__file__))}
    with (OUTPUT/'independent_recount.json').open('x') as stream:
        json.dump(receipt,stream,indent=2,sort_keys=True)
        stream.write('\n')
    print(json.dumps(receipt,sort_keys=True))


if __name__ == '__main__':
    main()
