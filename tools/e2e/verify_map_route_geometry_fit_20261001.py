#!/usr/bin/env python3
"""HH_261001 - Independently audit a completed geometry fit and recount saved known-split predictions."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
FIT = ROOT/'storage/experiments/training/2026-10-01/map_route_geometry_matched_20261001_v1/fit'
KNOWN = ROOT/'storage/experiments/training/2026-09-29/map_town02_p00_route_holdout_data_v1'
PARENT = ROOT/'storage/experiments/training/2026-10-01/map_appearance_matched_20261001_v1/parent_weights.npz'
DOC = ROOT/'docs/assets/validation/2026-10-01/map_unused_route_inventory'
ARMS = ('control','geometry')
NAME = 'map_route_geometry_matched_20261001_v1'
LABELS = ('other','road','sidewalk','lane_marking','traffic_light_fixture','traffic_sign','pole')
PARENT_SHA = 'ff5d7d03a43b859d21a0a3f6b01d815054d89d78e850730121f326e5b79c023e'
INITIAL_SHA = 'edf12b02e3bf34526ce30de6fb450c2fa91085004e5dbd42c95c9e159d6a2b67'
SOURCE_SHA = '20697b557b8f14a645b6e0ef36833184de7188b77a3e64c48496da4b6437ef5c'
DATA_SHA = 'db54a65424b55bbd095373a3aecb43508037efa0f3d61b31354a14206371071d'
PLAN_SHA = '5731cbd35feff6e4a04b5e929279dbf553c327db6c70420f2a7126cc08487b68'
PACKAGE_AUTH_SHA = '844bbc685eb0e49c427687d53432d622d3d8123e519e870423c460375bdfe43e'
FIT_AUTH_SHA = '8e4e586d8d80eb0bb3c645ad01975954ba34dfb3f29cdaac07fc5d610da1fa38'
WEIGHTS = [0.13594788312911987,0.13577374815940857,0.2083463817834854,1.4355568885803223,
           1.9548143148422241,2.183657646179199,0.9459031224250793]
KNOWN_FILES = {'dev_town01':(118,'417c60f924abb044617e2aba5b1afd70b83f5da7f905e54513e9c220172ba4b3'),
    'holdout_town02_p02':(232,'858f92ccd874c154576d3cf6a85132508801a22671baf6d0c07abe3936eb16cb')}
KNOWN_MANIFEST_SHA = 'f4764b9aa9d015f3d136a145f6194dd60bda8900c27dd2423de6145b9f472834'
SOURCE_FILES = {
    'map_appearance_matched_contract_20261001.py':'9fb7b057a09d955a8952699f3a5868a8f6a6d04cdf78ea66ada7eea742c53b96',
    'map_context_recovery.py':'c8e2a8f0026852cc2b68ff653f78ed4666a1d2d2bad81997d1f5c16798fbc50d',
    'map_perception.py':'1b0076c8f9d34192ab5aedf6cdf96ce2c14458e2bb49986685a6811376b0750b',
    'map_route_geometry_contract_20261001.py':'f73b4ec07f054102f2ffa316e5af001454b415729caf52f860591cdecae2d681',
    'prepare_map_route_geometry_remote_20261001.py':'8c656bb1324049e27c9ec0c6fc2a533f465f529beb84ab9ae733ec9443d4d5e0',
    'train_map_photometric_pilot.py':'4de567323d3964ec877da831ec2acb08e445fe77c9da659f052edcf52ca53484',
    'train_map_route_geometry_matched_20261001.py':'4de75aedb780e9eb36e39d9b0f2cd239425f2b4d72af45bc3610f7f109d6a34e',
    'training_plan.md':'a803f683f8677e0b04c484445f4b5ee23645b1c60733b1cfb49dd067e14912c0',
    'weights_correction.md':'ae6e56e3fd186e467abde0a9d0d89a5943b9b36541dc46bf49c7a50ac4decc09'}
INPUT_FILES = {'prepared_manifest.json':DATA_SHA,'reference_plan.json':PLAN_SHA,
    'package_authorization.json':PACKAGE_AUTH_SHA,'fit_authorization.json':FIT_AUTH_SHA,
    'candidate_manifest.json':'3c89ab9516e032122728d01bd72b1f9e5c31b5e287253e292143aa67e06382a6',
    'reference_manifest.json':'32ed912505e0b215bfc54e563fcdba7553aa25412ba22d90b45bfa00339e93c8'}


def require(ok,message):
    if not ok:
        raise ValueError(message)


def digest(path):
    require(path.is_file() and not path.is_symlink(),'Regular immutable file required: '+str(path))
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda:stream.read(1024*1024),b''):
            value.update(chunk)
    return value.hexdigest()


def read_json(path):
    # HH_261001 - Reject nonstandard NaN/Infinity receipts instead of accepting nonfinite JSON.
    def bad_constant(value):
        raise ValueError('Nonfinite JSON token: '+value)
    return json.loads(path.read_text(),parse_constant=bad_constant)


def checked(path,expected,hashes):
    require(digest(path) == expected,'Artifact SHA mismatch: '+str(path))
    hashes[path] = expected
    return read_json(path)


def independent_schedule():
    """HH_261001 - Reimplement the recorded algorithm without importing training helpers."""
    rng = np.random.Generator(np.random.PCG64(20261001))
    flat = np.concatenate([rng.permutation(1689).astype(np.int64) for _ in range(48)])[:80000]
    replay = np.random.Generator(np.random.PCG64(20261002)).permutation(497).astype(np.int64)[:480]
    crop_rng = np.random.Generator(np.random.PCG64(20261002))
    crop_hash = hashlib.sha256()
    for _ in range(80000):
        if crop_rng.random() < .5:
            scale = float(crop_rng.uniform(.65,1.))
            height,width = int(360*scale),int(640*scale)
            row = (int(crop_rng.integers(361-height)),int(crop_rng.integers(641-width)),height,width)
        else:
            row = (0,0,360,640)
        crop_hash.update(np.asarray(row,np.int16).tobytes())
    photo_rng = np.random.Generator(np.random.PCG64(20261003))
    trace = hashlib.sha256()
    eligible = applied = 0
    for slot in flat:
        if 729 <= slot < 1209:
            active = photo_rng.random() < .5
            if active:
                photo_rng.uniform(0.,1.)
                photo_rng.uniform(.35,1.)
            trace.update(np.asarray((slot,int(active)),np.int64).tobytes())
            eligible += 1
            applied += int(active)
    return {'flat':flat,'replay':replay,'sample_sha256':hashlib.sha256(flat.tobytes()).hexdigest(),
        'crop_sha256':crop_hash.hexdigest(),'photo_trace':{'eligible_calls':eligible,'applied_calls':applied,
        'sample_gate_trace_sha256':trace.hexdigest(),'final_rng_state':photo_rng.bit_generator.state}}


def independent_exposure(flat,replay,arm):
    require(arm in ARMS,'Unexpected arm')
    counts = np.bincount(flat,minlength=1689)
    original = counts[:729].copy()
    paired = counts[729:1209].copy()
    geometry = np.zeros(480,np.int64)
    if arm == 'control':
        np.add.at(original,replay,counts[1209:])
    else:
        geometry[:] = counts[1209:]
    require(np.all(original > 0) and np.all(paired > 0),'An original/paired example disappeared')
    if arm == 'geometry':
        require(np.all(geometry > 0),'A full-capture geometry example disappeared')
    values = {'original':original,'paired':paired,'geometry':geometry}
    return {'slot_exposures':counts.tolist(),'physical_source_exposures':{k:v.tolist() for k,v in values.items()},
        'physical_source_totals':{k:int(v.sum()) for k,v in values.items()}}


def tensor_archive(path,template=None):
    with np.load(path,allow_pickle=False) as data:
        require(len(data.files) == len(set(data.files)),'Duplicate NPZ tensor key')
        if template is not None:
            require(set(data.files) == set(template),'Exact learned-tensor ABI keys differ')
        tensors = {}
        values = {}
        state = hashlib.sha256()
        for name in sorted(data.files):
            value = data[name]
            require(value.dtype == np.float32 and np.isfinite(value).all(),'Nonfinite/non-float32 tensor: '+name)
            if template is not None:
                require(list(value.shape) == template[name]['shape'],'Learned-tensor shape differs: '+name)
            raw = np.ascontiguousarray(value).tobytes()
            tensors[name] = {'shape':list(value.shape),'dtype':'float32','sha256':hashlib.sha256(raw).hexdigest()}
            header = json.dumps([name,'torch.float32',list(value.shape)]).encode('utf-8')
            state.update(len(header).to_bytes(8,'big'));state.update(header);state.update(raw)
            values[name] = int(value.size)
    return tensors,state.hexdigest(),sum(values.values())


def check_history(rows,arm,receipt):
    require(len(rows) == 100 and [r.get('step') for r in rows] == list(range(100,10001,100)),
        'Exactly 100 recorded intervals through step10000 required')
    require(all(r.get('arm') == arm and set(r) == {'arm','step','mean_loss_100','wall_seconds'}
        and isinstance(r['mean_loss_100'],(float,int)) and math.isfinite(r['mean_loss_100'])
        and r['mean_loss_100'] >= 0 and isinstance(r['wall_seconds'],(float,int))
        and math.isfinite(r['wall_seconds']) and r['wall_seconds'] > 0 for r in rows),'Invalid history row')
    require(all(a['wall_seconds'] < b['wall_seconds'] for a,b in zip(rows,rows[1:])),'Nonmonotonic history')
    require(receipt['steps'] == 10000 and receipt['complete'] is True
        and rows[-1]['wall_seconds'] <= receipt['training_wall_seconds'] < 2100,'Completed bounded endpoint required')
    return {'recorded_intervals':100,'final_recorded_step':10000,'first_loss_100':rows[0]['mean_loss_100'],
        'last_loss_100':rows[-1]['mean_loss_100'],'training_wall_seconds':receipt['training_wall_seconds']}


def recount(target,prediction):
    require(target.shape == prediction.shape and target.ndim == 3 and target.dtype == prediction.dtype == np.uint8,
        'Native target/prediction shape/dtype differs')
    require(target.size > 0 and target.max() < 7 and prediction.max() < 7,'Class ID outside seven-class ABI')
    confusion = np.zeros((7,7),np.int64)
    for truth,pred in zip(target,prediction):
        confusion += np.bincount((truth.astype(np.int16)*7+pred).ravel(),minlength=49).reshape(7,7)
    return confusion


def independent_metrics(confusion):
    cells = confusion.tolist()
    support = [sum(row) for row in cells]
    predicted = [sum(row[i] for row in cells) for i in range(7)]
    diagonal = [cells[i][i] for i in range(7)]
    return {'confusion':cells,'target_pixels':dict(zip(LABELS,support)),
        'predicted_pixels':dict(zip(LABELS,predicted)),
        'false_positive_pixels':dict(zip(LABELS,[p-t for p,t in zip(predicted,diagonal)])),
        'iou':{label:diagonal[i]/(support[i]+predicted[i]-diagonal[i]) if support[i] else None
               for i,label in enumerate(LABELS)},
        'pixel_accuracy':sum(diagonal)/sum(support),'absent_target_class_iou':'null; false positives retained'}


def verify(expected_summary):
    """HH_261001 - Call only after parent completion/import; summary-first guard refuses active/failed runs."""
    require(FIT.is_dir() and FIT.resolve() == FIT,'Canonical completed local import required')
    require(not any((FIT/name).exists() for name in ('failure.json',)),'Failed run is not a complete pair')
    hashes = {}
    summary = checked(FIT/'summary.json',expected_summary,hashes)
    require(summary.get('status') == 'GEOMETRY_SOURCE_PAIR_COMPLETE_DIAGNOSTIC_NOT_PROMOTED'
        and summary.get('study_id') == NAME and summary.get('source_and_data_unchanged') is True
        and summary.get('dev_used_during_optimization') is False and set(summary.get('arms',{})) == set(ARMS),
        'Completed diagnostic pair required before endpoint reads')
    bindings = {'source_bundle_sha256':SOURCE_SHA,'prepared_manifest_sha256':DATA_SHA,
        'reference_plan_sha256':PLAN_SHA,'package_authorization_sha256':PACKAGE_AUTH_SHA,
        'fit_authorization_sha256':FIT_AUTH_SHA,'parent_weights_sha256':PARENT_SHA,'common_class_weights':WEIGHTS,
        'fresh_fits':2,'steps_per_arm':10000,'batch':8,'learning_rate':3e-5,'weight_decay':1e-4,
        'gradient_clip_norm':5.,'fit_plus_known_eval_budget_seconds_per_arm':2400,'same_architecture':True,
        'ctrack_train_eligible':False,'broad_registry_modified':False,'model_promotion':False}
    require(all(summary.get(k) == v for k,v in bindings.items()),'Summary provenance/optimization differs')
    for name in ('started.json','fit_complete.json','known_evaluation.json'):
        hashes[FIT/name] = digest(FIT/name)
    started,complete,evaluation = [read_json(FIT/name) for name in ('started.json','fit_complete.json','known_evaluation.json')]
    require(all(summary.get(k) == v for k,v in started.items() if k not in ('gpu','created_at_utc')),
        'Started-to-summary receipt chain differs')
    require(all(started.get(k) == v for k,v in bindings.items())
        and started.get('schedule') == summary['schedule'] and started.get('photometric') == summary['photometric'],
        'Mandatory started provenance/exposure receipt missing')
    require(started.get('gpu',{}).get('uuid') == 'GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5'
        and started['gpu'].get('physical_index') == 0
        and started['gpu'].get('memory_used_mib_before_torch') == 0
        and started['gpu'].get('utilization_percent_before_torch') == 0,'Unexpected/busy physical GPU identity')
    require(complete.get('status') == 'BOTH_FIXED_ENDPOINTS_SAVED_BEFORE_DEV'
        and complete.get('arms') == summary['arms'],'Both-final completion chain differs')
    require(evaluation == summary['known_evaluation'] and set(evaluation) == set(KNOWN_FILES),
        'Known evaluation/summary chain differs')
    bundle = checked(FIT/'source/bundle.json',SOURCE_SHA,hashes)
    require(bundle.get('files') == SOURCE_FILES,'Exact nine-file source closure differs')
    for name,sha in SOURCE_FILES.items():
        require(digest(FIT/'source'/name) == sha,'Archived training source changed: '+name)
        hashes[FIT/'source'/name] = sha
    inputs = {name:checked(FIT/'inputs'/name,sha,hashes) for name,sha in INPUT_FILES.items()}
    require(inputs['fit_authorization.json'].get('fit_authorized') is True
        and inputs['package_authorization.json'].get('fit_authorized') is False
        and inputs['fit_authorization.json'].get('common_class_weights') == WEIGHTS,
        'Package-only and separate fit authority boundary differs')
    require(inputs['reference_manifest.json']['labels'] == list(LABELS)
        and inputs['reference_manifest.json']['common_class_weights'] == WEIGHTS
        and inputs['prepared_manifest.json']['common_class_weights'] == WEIGHTS,'Labels or fixed weight vector differ')
    algorithm = independent_schedule()
    plan = inputs['reference_plan.json']
    require(plan['control_original_indices'] == algorithm['replay'].tolist()
        and plan['schedule']['indices_int64_sha256'] == algorithm['sample_sha256']
        and plan['schedule']['crops_int16_sha256'] == algorithm['crop_sha256'],'Independent schedule/replay/crop differs')
    require(summary['schedule'] == plan['schedule'] and summary['photometric'] == plan['photometric'],
        'Recorded exposure/photo policy differs')
    require(algorithm['photo_trace'] == {k:plan['photometric'][k] for k in algorithm['photo_trace']},
        'Independent joint photometric RNG trace differs')
    require(digest(PARENT) == PARENT_SHA,'Existing original parent weight bytes changed')
    hashes[PARENT] = PARENT_SHA
    parent_template,parent_state,parent_count = tensor_archive(PARENT)
    require(parent_state == INITIAL_SHA and parent_count == 245831,'Exact original parent tensor state differs')
    histories,tensors,exposures = {},{},{}
    for arm in ARMS:
        root,record = FIT/arm,summary['arms'][arm]
        require(not any((root/name).exists() for name in ('failure.json','aborted_state.npz')),
            'Partial/failed arm cannot qualify as final')
        training = read_json(root/'training.json')
        hashes[root/'training.json'] = digest(root/'training.json')
        require(training == record,'Arm training/fit-complete/summary chain differs')
        require(record['initial_state_sha256'] == INITIAL_SHA and record['parameter_count'] == 245831
            and record['optimizer_initial_state_entries'] == 0 and record['common_class_weights'] == WEIGHTS,
            'Parent, parameter count, fresh optimizer or fixed weights differ')
        for name,key in (('weights.npz','weights_sha256'),('checkpoint.pt','checkpoint_sha256')):
            require(digest(root/name) == record[key],'Final endpoint SHA changed')
            hashes[root/name] = record[key]
        actual,state,count = tensor_archive(root/'weights.npz',parent_template)
        require(actual == record['final_tensors'] and state == record['final_state_sha256'] and count == 245831,
            'Independent native float32 tensor/state recount differs')
        tensors[arm] = {'tensor_count':len(actual),'parameters':count,'final_state_sha256':state,
                       'all_per_tensor_abi_and_hashes_exact':True}
        rows = [json.loads(line) for line in (root/'history.jsonl').read_text().splitlines()]
        hashes[root/'history.jsonl'] = digest(root/'history.jsonl')
        histories[arm] = check_history(rows,arm,record)
        expected_exposure = independent_exposure(algorithm['flat'],algorithm['replay'],arm)
        require(record['exposure'] == expected_exposure and record['sample_sequence_sha256'] == algorithm['sample_sha256']
            and record['crop_sequence_sha256'] == algorithm['crop_sha256']
            and record['photo_trace'] == algorithm['photo_trace'],'Actual arm sampling/crop/photo receipt differs')
        exposures[arm] = expected_exposure['physical_source_totals']
    checked(KNOWN/'manifest.json',KNOWN_MANIFEST_SHA,hashes)
    results = {}
    for split,(count,sha) in KNOWN_FILES.items():
        path = KNOWN/(split+'.npz')
        require(digest(path) == sha,'Existing known target archive changed')
        hashes[path] = sha
        with np.load(path,allow_pickle=False) as data:
            require(set(data.files) == {'rgb','target'},'Existing known split archive ABI')
            target = data['target']  # HH_261001 - Never decode/import the existing RGB arrays.
        require(target.shape == (count,360,640) and target.dtype == np.uint8,'Native known target ABI')
        require(set(evaluation[split]) == set(ARMS),'Both arms/all known splits required')
        for arm in ARMS:
            pred_path = FIT/arm/(split+'_predictions.npz')
            hashes[pred_path] = digest(pred_path)
            with np.load(pred_path,allow_pickle=False) as data:
                require(set(data.files) == {'prediction'},'Prediction NPZ ABI')
                pred = data['prediction']
            matrix = recount(target,pred)
            actual = {'count':count,'source_sha256':sha,**independent_metrics(matrix)}
            require(actual == evaluation[split][arm],'Independent native all-pixel metrics differ: '+split+'/'+arm)
            results[split+'/'+arm] = {'frames':count,'pixels':int(matrix.sum()),'prediction_sha256':hashes[pred_path],
                'target_archive_sha256':sha,'all_pixel_confusion_and_metrics_exact':True,**independent_metrics(matrix)}
    require(sum(value['pixels'] for value in results.values()) == 161280000,'Known350 both-arm full-pixel denominator differs')
    for path,sha in hashes.items():
        require(digest(path) == sha,'Artifact changed during independent audit: '+str(path))
    return {'schema':'portable_e2e.map_geometry_fit_independent_recount.v1','status':'PASS_DIAGNOSTIC_NOT_PROMOTED',
        'study_id':NAME,'summary_sha256':expected_summary,'source_bundle_sha256':SOURCE_SHA,
        'fit_authorization_sha256':FIT_AUTH_SHA,'histories':histories,'tensor_checks':tensors,
        'actual_physical_source_totals':exposures,'known_checks':results,'total_recounted_pixels':161280000,
        'unique_known_frames':350,'prediction_frames_both_arms':700,'all_original_and_true_samples_retained':True,
        'new_geometry_all480_retained':True,'source9_verified':True,'artifacts_unchanged_during_audit':True,
        'artifact_sha256':{str(path.relative_to(ROOT)):sha for path,sha in hashes.items()},
        'checkpoint_pickle_opened':False,'new_model_forwards':0,'new_training_steps':0,'gpu_used':False,
        'new_geometry_arrays_reopened_locally':False,'training_data_assurance':'pinned remote package/preflight and frozen run hash-check receipts',
        'prediction_hash_binding':'first independent imported-file hashes; trainer binds their exact reported confusion, not separate prediction-file SHA',
        'model_promotion':False,'independent_certification':False,'verifier_sha256':digest(Path(__file__))}


def main():
    parser = argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument('--expected-summary-sha256',required=True)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    require(not args.output.exists() and args.output.parent.is_dir(),'Fresh small report in existing directory required')
    result = verify(args.expected_summary_sha256)
    encoded = json.dumps(result,indent=2,sort_keys=True,allow_nan=False)+'\n'
    require(len(encoded.encode()) < 128*1024,'Small report budget exceeded')
    with args.output.open('x') as stream:
        stream.write(encoded)
    print(json.dumps({'status':result['status'],'pixels':result['total_recounted_pixels'],
                      'report_sha256':digest(args.output)}))


if __name__ == '__main__':
    main()
