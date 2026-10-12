#!/usr/bin/env python3
"""HH_261001 - Frozen all-frame fresh TRUE-control versus photometric endpoint diagnostics."""

from datetime import datetime, timezone
import argparse
import importlib
import json
import os
from pathlib import Path
import sys
import time
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from scripts.e2e import evaluate_map_appearance_endpoints_20261001 as previous
from scripts.e2e import map_photometric_matched_contract_20261001 as photo
from scripts.e2e import train_map_photometric_matched_20261001 as training

sys.dont_write_bytecode = True
ROOT, contract, frozen = previous.ROOT, previous.contract, previous.frozen
PACKAGE = ROOT/'storage/experiments/training/2026-10-01'/photo.NAME
FIT = PACKAGE/'fit'
DOCS = ROOT/'docs/assets/validation/2026-10-01/map_photometric_endpoints'
PREREG = DOCS/'preregistration.json'
DATA_SHA = 'b7458a62bac606d0d33c0d565a11fd7f6b4eeda8ba8300d82d9eafaa9f9c3b40'
REGISTRY_SHA = '4e5b220400315d082bde1ae80471ee0e9f81dd3840234082eaabce4c6a0663a6'
SOURCE_SHA = 'bacf1892edc10d03c35c4a4b867638f5423594d602b6f9f53c82373e8512961d'
PREVIOUS_PREREG_SHA = 'dbd037ee055103e53b40d43e8aec57247386929bf990beac20cca6f414438616'
ARMS, INDICES, SCENARIOS = photo.ARMS, previous.INDICES, previous.SCENARIOS
locations, raw_frame = previous.locations, previous.raw_frame
capture_binding, verify_raw_hashes = previous.capture_binding, previous.verify_raw_hashes


def output_path(scenario):
    contract.require(scenario in SCENARIOS, 'unknown fixed scenario')
    return frozen.OUTPUT_ROOT/f'map_photometric_endpoints_{scenario}_20261001_v1'


def policy():
    value = previous.policy()
    value.update(study_id=photo.NAME, arms=list(ARMS), data_manifest_sha256=DATA_SHA,
        source_bundle_sha256=SOURCE_SHA, study_registry_sha256=REGISTRY_SHA, fit_path=str(FIT),
        order='frame, condition false/true where available, fresh TRUE-control/photometric',
        control='fresh original729+TRUE480 without photometric augmentation',
        candidate='same inputs; frozen archived grayscale/brightness augmentation on TRUE480 only during fitting',
        openblas_threads=2, inference_augmentation=False,
        primary_comparison='photometric minus this study fresh TRUE-control; never previous FALSE-control',
        preceding_comparison='separate prior appearance endpoint: all-pixel prediction equality and per-class IoU deltas',
        prior_appearance_fit_summary_sha256=photo.PREDECESSOR_FIT_SHA,
        prior_appearance_preregistration_sha256=PREVIOUS_PREREG_SHA,
        require_both_fresh_fits=True, photometric_policy=photo.policy())
    value['limitations'].append('Native CPU backend settings may differ from historical diagnostics; preceding prediction equality is measured, not assumed.')
    return value


def package_binding():
    training.source_contract(PACKAGE/'source', SOURCE_SHA, DATA_SHA, REGISTRY_SHA)
    return photo.study_binding(PACKAGE, DATA_SHA, REGISTRY_SHA, previous.PACKAGE)


def preceding_receipts():
    contract.require(contract.digest(previous.PREREG) == PREVIOUS_PREREG_SHA, 'prior frozen evaluation changed')
    result = {}
    for scenario in SCENARIOS:
        root = previous.output_path(scenario)
        summary, audit = (contract.read_json(root/name) for name in ('summary.json','prediction_audit.json'))
        contract.require(summary['status'] == 'DIAGNOSTIC_COMPLETE_NOT_PROMOTED' and audit['status'] == 'PASS'
            and summary['fit_artifact_sha256']['summary.json'] == photo.PREDECESSOR_FIT_SHA
            and audit['summary_sha256'] == contract.digest(root/'summary.json'), 'prior completed appearance diagnostic differs')
        conditions = locations(scenario)[2]
        result[scenario] = {'root':str(root), 'summary_sha256':contract.digest(root/'summary.json'),
            'audit_sha256':contract.digest(root/'prediction_audit.json'),
            'appearance_predictions':{condition:summary['prediction_artifacts'][f'appearance_{condition}_all240_predictions.npz']
                                      for condition in conditions}}
    return result


def sources():
    result = previous.sources()
    for path in (Path(__file__).resolve(), ROOT/'tools/e2e/verify_map_photometric_endpoints_20261001.py',
                 ROOT/'learning/tests/test_map_photometric_endpoints_20261001.py', DOCS/'PLAN.md',
                 Path(photo.__file__).resolve(), Path(training.__file__).resolve()):
        result[str(path)] = contract.digest(path)
    return result


def preregister():
    contract.require(not FIT.exists() and not any(output_path(s).exists() for s in SCENARIOS),
                     'register before local final import or predictions')
    package_binding()
    record = {'schema':'portable_e2e.map_photometric_endpoint_preregistration.v1',
        'created_at_utc':datetime.now(timezone.utc).isoformat(), 'policy':policy(), 'sources':sources(),
        'raw_receipts':previous.raw_receipts(), 'preceding_receipts':preceding_receipts(),
        'predictions_made':False, 'new_final_weights_locally_available':False,
        'remote_training_may_be_running':True, 'remote_candidate_results_inspected':False}
    frozen.write_new(PREREG, record)
    return {'status':'PREREGISTERED_BEFORE_NEW_ENDPOINT_IMPORT', 'sha256':contract.digest(PREREG)}


def validate_registration():
    record = contract.read_json(PREREG)
    contract.require(record['schema'] == 'portable_e2e.map_photometric_endpoint_preregistration.v1'
        and record['policy'] == policy() and record['sources'] == sources()
        and record['raw_receipts'] == previous.raw_receipts() and record['preceding_receipts'] == preceding_receipts()
        and record['predictions_made'] is False and record['new_final_weights_locally_available'] is False
        and record['remote_candidate_results_inspected'] is False, 'frozen photometric evaluation changed')
    package_binding()
    return record


def validate_endpoint_metadata(summary, started, complete, study, manifest):
    contract.require(summary.get('status') == 'PHOTOMETRIC_PAIR_COMPLETE_DIAGNOSTIC_NOT_PROMOTED'
        and complete.get('status') == 'BOTH_FIXED_ENDPOINTS_SAVED_BEFORE_DEV'
        and summary.get('arms') == complete.get('arms') and set(complete.get('arms',{})) == set(ARMS),
        'both fixed photometric endpoints required')
    for item in (summary, started):
        contract.require(item.get('study_id') == photo.NAME and item.get('data_manifest_sha256') == DATA_SHA
            and item.get('source_bundle_sha256') == SOURCE_SHA and item.get('study_registry_sha256') == REGISTRY_SHA
            and item.get('parent_weights_sha256') == contract.PARENT_SHA and item.get('steps_per_arm') == 10000
            and item.get('batch') == 8 and item.get('common_class_weights') == manifest['common_class_weights']
            and item.get('schedule_contract') == manifest['schedule_contract']
            and item.get('reference_data_manifest_sha256') == photo.REFERENCE_MANIFEST_SHA
            and item.get('photometric_policy') == photo.policy() and item.get('expected_photo_trace') == study['expected_photo_trace']
            and item.get('fresh_fits') == 2 and item.get('ctrack_train_eligible') is False
            and item.get('model_promotion') is False, 'fit study/appearance exposure differs')
    contract.require(summary.get('dev_used_during_optimization') is False and summary.get('source_and_data_unchanged') is True,
                     'fit selection/source boundary failed')
    for info in complete['arms'].values():
        contract.require(info.get('steps') == 10000 and info.get('parameter_count') == 245831
            and info.get('initial_state_sha256') == previous.INITIAL_STATE_SHA and info.get('optimizer_initial_state_entries') == 0
            and info.get('sample_sequence_sha256') == manifest['schedule_contract']['indices_sha256']
            and info.get('crop_sequence_sha256') == manifest['schedule_contract']['crops_sha256'], 'matched endpoint schedule/init differs')
    control, candidate = (complete['arms'][arm] for arm in ARMS)
    empty_trace = photo.Trace().receipt(np.random.default_rng(photo.PHOTO_SEED))
    contract.require(control.get('photo_trace') == empty_trace
        and control.get('predecessor_reference_state_reproduced') is True
        and control.get('final_state_sha256') == study['predecessor_appearance_state_sha256'], 'fresh no-photo reference differs')
    contract.require(candidate.get('photo_trace') == study['expected_photo_trace']
        and candidate.get('predecessor_reference_state_reproduced') is None, 'candidate frozen photo trace differs')


def fit_binding(expected_summary_sha):
    contract.require(contract.digest(FIT/'summary.json') == expected_summary_sha, 'explicit completed fit SHA differs')
    study, registry, manifest = package_binding()
    summary, started, complete = (contract.read_json(FIT/name) for name in ('summary.json','started.json','fit_complete.json'))
    validate_endpoint_metadata(summary, started, complete, study, manifest)
    training.source_contract(FIT/'source', SOURCE_SHA, DATA_SHA, REGISTRY_SHA)
    expected_inputs = {'inputs/manifest.json':DATA_SHA, 'inputs/study_train_registry.json':REGISTRY_SHA,
        'inputs/preview_review.json':study['preview_review_sha256'],
        'reference_inputs/manifest.json':photo.REFERENCE_MANIFEST_SHA,
        'reference_inputs/study_train_registry.json':photo.REFERENCE_REGISTRY_SHA,
        'reference_inputs/paired_samples.json':manifest['paired_samples_sha256']}
    for name, sha in expected_inputs.items():
        contract.require(contract.digest(FIT/name) == sha, 'archived study/reference input changed')
    hashes = {name:contract.digest(FIT/name) for name in ('summary.json','started.json','fit_complete.json')}
    for arm, info in complete['arms'].items():
        contract.require(contract.read_json(FIT/arm/'training.json') == info, 'fixed arm receipt differs')
        for name, key in (('weights.npz','weights_sha256'),('checkpoint.pt','checkpoint_sha256')):
            contract.require(contract.digest(FIT/arm/name) == info[key], 'fixed arm artifact changed')
            hashes[arm+'/'+name] = info[key]
        hashes[arm+'/training.json'] = contract.digest(FIT/arm/'training.json')
    return hashes


def load_models(bindings):
    contract.require(os.environ.get('CUDA_VISIBLE_DEVICES') == '' and os.environ.get('OMP_NUM_THREADS') == '2'
        and os.environ.get('OPENBLAS_NUM_THREADS') == '2', 'CPU2 launch environment required before imports')
    import torch
    from threadpoolctl import threadpool_info
    torch.set_num_threads(2); torch.set_num_interop_threads(2)
    pools = threadpool_info()
    contract.require(all(pool['num_threads'] == 2 for pool in pools if pool['user_api'] == 'blas'), 'native BLAS not CPU2')
    sys.path.insert(0, str(FIT/'source'))
    core, context = (importlib.import_module(name) for name in ('map_perception','map_context_recovery'))
    for module in (core, context):
        contract.require(Path(module.__file__).resolve().parent == FIT/'source', 'model import escaped frozen source')
    contract.require(tuple(core.LABELS) == frozen.LABELS, 'class order differs')
    return torch, {arm:frozen.safe_npz_state(context.MapContextRecovery(), FIT/arm/'weights.npz', bindings[arm+'/weights.npz'], torch)
                   for arm in ARMS}, pools


def panel_layout(rgb, target, predictions, conditions):
    if len(conditions) == 1:
        return [(0,57,rgb['true']), (640,57,frozen.COLORS[target]),
            (0,442,frozen.COLORS[predictions['control_true']]), (640,442,frozen.COLORS[predictions['photometric_true']])]
    return [(column*640,57+row*385,pixels) for column, condition in enumerate(conditions)
        for row,pixels in enumerate((rgb[condition],frozen.COLORS[target],
            frozen.COLORS[predictions['control_'+condition]],frozen.COLORS[predictions['photometric_'+condition]]))]


def render(scenario,index,rgb,target,predictions,conditions):
    paired = len(conditions) == 2
    image = Image.new('RGB',(1280,1640 if paired else 860),(18,27,40))
    draw = ImageDraw.Draw(image)
    font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',16)
    draw.text((8,7),f'{scenario} | fixed frame {index:03d}/239 | fresh TRUE-control vs photometric | 10000 steps',font=font,fill='white')
    captions = ([f'RGB {condition}', 'Shared GT', 'Fresh TRUE control: no augmentation', 'Photometric: same TRUE480, archived policy']
                for condition in conditions)
    captions = sum(captions,[]) if paired else ['Original RGB true','Semantic GT','Fresh TRUE control: no augmentation','Photometric: same TRUE480, archived policy']
    for (x,y,pixels),caption in zip(panel_layout(rgb,target,predictions,conditions),captions):
        draw.text((x+8,y-22),caption,font=font,fill='white'); image.paste(Image.fromarray(pixels),(x,y))
    draw.text((8,image.height-44),'TRAIN resubstitution only; not validation.' if paired else 'Exposed CTrack diagnostic; no CTrack optimization or independent certification.',font=font,fill='white')
    draw.text((8,image.height-21),'LEFT original FAIL and unavailable live equality remain. No promotion.',font=font,fill='white')
    return image


def preceding_comparison(scenario, registration, predictions, results):
    receipt = registration['preceding_receipts'][scenario]
    root = Path(receipt['root'])
    contract.require(contract.digest(root/'summary.json') == receipt['summary_sha256'], 'prior summary changed')
    prior = contract.read_json(root/'summary.json')
    result = {}
    for condition, sha in receipt['appearance_predictions'].items():
        path = root/f'appearance_{condition}_all240_predictions.npz'
        contract.require(contract.digest(path) == sha, 'prior appearance predictions changed')
        with np.load(path,allow_pickle=False) as data:
            contract.require(data.files == ['prediction'], 'prior prediction keys differ')
            old = data['prediction']
        contract.require(old.shape == (240,360,640) and old.dtype == np.uint8 and int(old.max()) < 7, 'prior prediction ABI differs')
        result[condition] = {arm:{'equal_predicted_pixels':int(np.count_nonzero(predictions[arm+'_'+condition] == old)),
            'total_predicted_pixels':int(old.size), 'iou_delta_vs_preceding_appearance':{
                label:None if prior['model_results']['appearance_'+condition]['class_metrics'][label]['iou'] is None else
                results[arm+'_'+condition]['class_metrics'][label]['iou'] - prior['model_results']['appearance_'+condition]['class_metrics'][label]['iou']
                for label in frozen.LABELS}} for arm in ARMS}
    return result


def execute(scenario, expected_summary_sha):
    registration = validate_registration(); bindings = fit_binding(expected_summary_sha)
    output = output_path(scenario)
    contract.require(not output.exists() and not output.is_symlink(), 'fresh diagnostic output required')
    source, records, conditions, qa, distinction = capture_binding(scenario, registration)
    torch, models, pools = load_models(bindings)
    output.mkdir()
    frozen.write_new(output/'started.json', {'created_at_utc':datetime.now(timezone.utc).isoformat(), 'scenario':scenario,
        'policy':registration['policy'], 'fit_artifact_sha256':bindings, 'preregistration_sha256':contract.digest(PREREG),
        'raw_receipt':registration['raw_receipts'][scenario], 'preceding_receipt':registration['preceding_receipts'][scenario],
        'native_threadpools':pools, **distinction})
    frozen.write_new(output/'source_qa_replayed.json',qa)
    keys = [arm+'_'+condition for condition in conditions for arm in ARMS]
    predictions = {key:np.empty((240,360,640),np.uint8) for key in keys}
    matrices = {key:np.zeros((7,7),np.int64) for key in keys}
    timings, rows, visuals = {key:[] for key in keys}, [], {}
    with torch.no_grad():
        for index,row in enumerate(records):
            contract.require(contract.digest(source/(row['stem']+'.npz')) == row['artifact_sha256']['.npz'], 'raw frame changed')
            rgb,target = raw_frame(source,row,conditions)
            report = {'index':index,'sensor_frame':row['frame'],'timestamp_s':row['timestamp_s']}
            for condition in conditions:
                tensor = torch.from_numpy(rgb[condition].transpose(2,0,1).copy()).float().unsqueeze(0)/255
                for arm,model in models.items():
                    key = arm+'_'+condition; start = time.perf_counter()
                    prediction = model(tensor).argmax(1)[0].numpy().astype(np.uint8)
                    timings[key].append((time.perf_counter()-start)*1000)
                    predictions[key][index] = prediction
                    matrix = frozen.confusion(target,prediction); matrices[key] += matrix
                    report[key] = {'confusion':matrix.tolist(),'inference_ms':timings[key][-1]}
            rows.append(report)
            if index in INDICES:
                path = output/f'{scenario}_{index:03d}_full_fov.png'
                render(scenario,index,rgb,target,{key:value[index] for key,value in predictions.items()},conditions).save(path)
                visuals[path.name] = contract.digest(path)
            if (index+1)%40 == 0:
                print(json.dumps({'scenario':scenario,'frames_per_model_condition':index+1}),flush=True)
    artifacts = {}
    for key,value in predictions.items():
        path = output/(key+'_all240_predictions.npz')
        with path.open('xb') as stream:
            np.savez_compressed(stream,prediction=value)
        artifacts[path.name] = contract.digest(path)
    results = {key:{**frozen.metrics(matrices[key]),'frames':240,
        'inference_p50_ms':float(np.quantile(timings[key],.5)),'inference_p95_ms':float(np.quantile(timings[key],.95)),
        'first_inference_ms':timings[key][0]} for key in keys}
    delta = {condition:{label:None if results['control_'+condition]['class_metrics'][label]['iou'] is None else
        results['photometric_'+condition]['class_metrics'][label]['iou']-results['control_'+condition]['class_metrics'][label]['iou']
        for label in frozen.LABELS} for condition in conditions}
    preceding = preceding_comparison(scenario,registration,predictions,results)
    verify_raw_hashes(scenario,registration)
    contract.require(fit_binding(expected_summary_sha) == bindings, 'endpoint changed during inference')
    validate_registration()
    frozen.write_new(output/'per_frame.json',rows)
    result = {'schema':'portable_e2e.map_photometric_endpoint_diagnostic.v1','status':'DIAGNOSTIC_COMPLETE_NOT_PROMOTED',
        'scenario':scenario,'source':str(source),'model_results':results,'iou_delta_photometric_minus_control':delta,
        'preceding_appearance_comparison':preceding,'prediction_artifacts':artifacts,'full_fov_visuals':visuals,
        'per_frame_sha256':contract.digest(output/'per_frame.json'),'fit_artifact_sha256':bindings,
        'preregistration_sha256':contract.digest(PREREG),'source_fingerprints_unchanged':True,
        'frames_per_model_condition':240,'device':'cpu','threads':2,'torch_version':torch.__version__,
        'native_threadpools':pools,'fit_steps_during_evaluation':0,'ctrack_train_eligible':False,'model_promotion':False,
        'independent_validation':False,'performance_gates':None,'limitations':registration['policy']['limitations'],**distinction}
    frozen.write_new(output/'summary.json',result)
    return {'status':result['status'],'scenario':scenario,'output':str(output),'iou_delta_photometric_minus_control':delta}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument('--preregister',action='store_true'); modes.add_argument('--execute',action='store_true')
    parser.add_argument('--scenario',choices=SCENARIOS)
    parser.add_argument('--expected-fit-summary-sha256')
    args = parser.parse_args()
    if args.preregister:
        result = preregister()
    elif args.execute:
        contract.require(args.scenario and args.expected_fit_summary_sha256, 'scenario and completed fit SHA required')
        result = execute(args.scenario,args.expected_fit_summary_sha256)
    else:
        validate_registration(); result = {'status':'FROZEN_PREREGISTRATION_PASS','predictions_made':False}
    print(json.dumps(result,sort_keys=True),flush=True)
