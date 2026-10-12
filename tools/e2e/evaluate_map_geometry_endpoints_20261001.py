#!/usr/bin/env python3
"""HH_261001 - Frozen geometry-source endpoints; fixed all-frame CPU diagnostics only."""

import argparse
from datetime import datetime, timezone
import hashlib
import importlib
import io
import json
import os
from pathlib import Path
import shutil
import sys
import time
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from scripts.e2e import evaluate_map_photometric_endpoints_20261001 as historical
from scripts.e2e import map_route_geometry_contract_20261001 as geometry

ROOT, contract, frozen = historical.ROOT, historical.contract, historical.frozen
ARMS, SCENARIOS, INDICES = geometry.ARMS, historical.SCENARIOS, historical.INDICES
DOCS = ROOT/'docs/assets/validation/2026-10-01/map_geometry_endpoints'
PREREG = DOCS/'preregistration.json'
INVENTORY = ROOT/'docs/assets/validation/2026-10-01/map_unused_route_inventory'
FIT = ROOT/'storage/experiments/training/2026-10-01'/geometry.NAME/'fit'
OUTPUT = frozen.OUTPUT_ROOT/'map_geometry_endpoints_20261001_v1'
SOURCE_SHA = '20697b557b8f14a645b6e0ef36833184de7188b77a3e64c48496da4b6437ef5c'
FIT_APPROVAL_SHA = '8e4e586d8d80eb0bb3c645ad01975954ba34dfb3f29cdaac07fc5d610da1fa38'
PHOTO_PREREG_SHA = '7019e192782a05eb4cfc2066a43c01ef02c1e8c32a5573341355a090e91e2711'
PHOTO_FIT_SHA = '27f2951c6a62da6c4ee76ccd1dae24350afaf85cc1b41824ae50dbeecada6c6d'
MAX_OUTPUT_BYTES, FAILURE_RESERVE_BYTES = 30*1024**2, 2*1024**2
FREE_FLOOR_BYTES = 5*1024**3
locations, raw_frame = historical.locations, historical.raw_frame
capture_binding, verify_raw_hashes = historical.capture_binding, historical.verify_raw_hashes


def output_path(scenario):
    contract.require(scenario in SCENARIOS, 'unknown fixed scenario')
    return OUTPUT/scenario


def policy():
    value = historical.previous.policy()
    value.update(study_id=geometry.NAME, arms=list(ARMS), data_manifest_sha256=geometry.DATA_SHA,
        source_bundle_sha256=SOURCE_SHA, study_registry_sha256=geometry.PACKAGE_APPROVAL_SHA,
        fit_authorization_sha256=FIT_APPROVAL_SHA,
        reference_plan_sha256=geometry.PLAN_SHA, fit_path=str(FIT), output_root=str(OUTPUT),
        order='frame, false/true where paired, control/geometry', openblas_threads=2,
        primary_comparison='geometry minus fresh matched geometry-study control',
        historical_comparison='prior photometric endpoint only; different exposure schedule, never the matched control',
        historical_fit_sha256=PHOTO_FIT_SHA, historical_preregistration_sha256=PHOTO_PREREG_SHA,
        input_augmentation=False, maximum_output_bytes=MAX_OUTPUT_BYTES, failure_reserve_bytes=FAILURE_RESERVE_BYTES,
        free_disk_floor_bytes=FREE_FLOOR_BYTES, budget_rule='precompress before every write; abort, retain partial evidence; no frame/panel dropping',
        known350='separate existing trainer receipt only; no known-split inputs or new known-split forwards here',
        scenario_deadline_seconds=600, require_both_fresh_fits=True)
    return value


def package_metadata():
    checks = {'remote_package_manifest.json':geometry.DATA_SHA, 'reference_plan.json':geometry.PLAN_SHA,
        'package_authorization.json':geometry.PACKAGE_APPROVAL_SHA, 'fit_authorization.json':FIT_APPROVAL_SHA}
    for name, sha in checks.items():
        contract.require(contract.digest(INVENTORY/name) == sha, 'prepared metadata changed: '+name)
    manifest = contract.read_json(INVENTORY/'remote_package_manifest.json')
    reference = contract.read_json(historical.previous.PACKAGE/'manifest.json')
    contract.require(contract.digest(historical.previous.PACKAGE/'manifest.json') == geometry.package.BASE_SHA,
                     'reference metadata changed')
    geometry.validate_prepared_manifest(manifest, reference)
    plan = geometry.validate_plan(contract.read_json(INVENTORY/'reference_plan.json'))
    geometry.validate_fit_authorization(contract.read_json(INVENTORY/'fit_authorization.json'), geometry.DATA_SHA, SOURCE_SHA)
    geometry.source_binding(INVENTORY/'training_source_v1', SOURCE_SHA)
    return manifest, plan


def historical_receipts():
    contract.require(contract.digest(historical.PREREG) == PHOTO_PREREG_SHA, 'historical registration changed')
    result = {}
    for scenario in SCENARIOS:
        root = historical.output_path(scenario)
        summary, audit = (contract.read_json(root/name) for name in ('summary.json', 'prediction_audit.json'))
        contract.require(summary['status'] == 'DIAGNOSTIC_COMPLETE_NOT_PROMOTED' and audit['status'] == 'PASS'
            and summary['fit_artifact_sha256']['summary.json'] == PHOTO_FIT_SHA
            and audit['summary_sha256'] == contract.digest(root/'summary.json'), 'historical completed photo receipt differs')
        predictions = {condition:summary['prediction_artifacts'][f'photometric_{condition}_all240_predictions.npz']
                       for condition in locations(scenario)[2]}
        result[scenario] = dict(root=str(root), summary_sha256=contract.digest(root/'summary.json'),
            audit_sha256=contract.digest(root/'prediction_audit.json'), predictions=predictions,
            historical_directory_bytes=sum(p.stat().st_size for p in root.iterdir() if p.is_file()))
    return result


def sources():
    value = historical.sources()
    for path in (Path(__file__).resolve(), ROOT/'tools/e2e/verify_map_geometry_endpoints_20261001.py',
                 ROOT/'learning/tests/test_map_geometry_endpoints_20261001.py', DOCS/'PLAN.md',
                 Path(geometry.__file__).resolve(), Path(geometry.package.__file__).resolve()):
        value[str(path)] = contract.digest(path)
    return value


def preregister():
    contract.require(not FIT.exists() and not OUTPUT.exists(), 'register before local endpoint import or outputs')
    package_metadata()
    receipt = dict(schema='portable_e2e.map_geometry_endpoint_preregistration.v1',
        created_at_utc=datetime.now(timezone.utc).isoformat(), policy=policy(), sources=sources(),
        raw_receipts=historical.previous.raw_receipts(), historical_receipts=historical_receipts(),
        predictions_made=False, endpoints_locally_available=False, new_endpoint_results_inspected=False)
    frozen.write_new(PREREG, receipt)
    return dict(status='PREREGISTERED_BEFORE_ENDPOINT_IMPORT', sha256=contract.digest(PREREG))


def validate_registration():
    value = contract.read_json(PREREG)
    contract.require(value.get('schema') == 'portable_e2e.map_geometry_endpoint_preregistration.v1'
        and value['policy'] == policy() and value['sources'] == sources()
        and value['raw_receipts'] == historical.previous.raw_receipts()
        and value['historical_receipts'] == historical_receipts()
        and value['predictions_made'] is False and value['endpoints_locally_available'] is False
        and value['new_endpoint_results_inspected'] is False, 'frozen geometry evaluation changed')
    package_metadata()
    return value


def validate_endpoint_metadata(summary, started, complete, plan):
    contract.require(summary.get('status') == 'GEOMETRY_SOURCE_PAIR_COMPLETE_DIAGNOSTIC_NOT_PROMOTED'
        and complete.get('status') == 'BOTH_FIXED_ENDPOINTS_SAVED_BEFORE_DEV'
        and summary.get('arms') == complete.get('arms') and set(complete.get('arms', {})) == set(ARMS),
        'both completed fixed geometry endpoints required')
    for value in (summary, started):
        expected = dict(study_id=geometry.NAME, source_bundle_sha256=SOURCE_SHA,
            prepared_manifest_sha256=geometry.DATA_SHA, reference_plan_sha256=geometry.PLAN_SHA,
            package_authorization_sha256=geometry.PACKAGE_APPROVAL_SHA, parent_weights_sha256=contract.PARENT_SHA,
            fit_authorization_sha256=FIT_APPROVAL_SHA,
            common_class_weights=geometry.CLASS_WEIGHTS, schedule=plan['schedule'], photometric=plan['photometric'],
            fresh_fits=2, steps_per_arm=10000, batch=8, learning_rate=contract.LR,
            weight_decay=contract.WEIGHT_DECAY, gradient_clip_norm=5., same_architecture=True,
            ctrack_train_eligible=False, broad_registry_modified=False, model_promotion=False)
        contract.require(all(value.get(key) == item for key, item in expected.items()), 'fit data/source/optimizer contract differs')
    contract.require(summary.get('dev_used_during_optimization') is False
        and summary.get('source_and_data_unchanged') is True, 'fit selection/source boundary failed')
    indices, _ = geometry.package.schedule()
    replay = geometry.package.control_replay()
    trace_keys = ('eligible_calls', 'applied_calls', 'sample_gate_trace_sha256', 'final_rng_state')
    for arm, info in complete['arms'].items():
        contract.require(info.get('complete') is True and info.get('steps') == 10000
            and info.get('parameter_count') == 245831 and info.get('optimizer_initial_state_entries') == 0
            and info.get('initial_state_sha256') == historical.previous.INITIAL_STATE_SHA
            and info.get('sample_sequence_sha256') == plan['schedule']['indices_int64_sha256']
            and info.get('crop_sequence_sha256') == plan['schedule']['crops_int16_sha256']
            and info.get('common_class_weights') == geometry.CLASS_WEIGHTS
            and info.get('photo_trace') == {key:plan['photometric'][key] for key in trace_keys}
            and info.get('exposure') == geometry.exposure_receipt(indices, replay, arm), 'endpoint exposure/trace differs')


def verify_numeric_npz(path, info):
    # HH_261001 - Recompute the training tensor digest without loading a pickle checkpoint.
    sha = hashlib.sha256()
    with np.load(path, allow_pickle=False) as data:
        contract.require(len(data.files) == len(set(data.files)) and set(data.files) == set(info['final_tensors']), 'tensor keys differ')
        for name in sorted(data.files):
            value = data[name]
            contract.require(value.dtype == np.float32 and np.isfinite(value).all(), 'unsafe/nonfinite weight dtype')
            item = dict(shape=list(value.shape), dtype=str(value.dtype), sha256=hashlib.sha256(value.tobytes()).hexdigest())
            contract.require(item == info['final_tensors'][name], 'individual tensor receipt differs')
            header = json.dumps([name, 'torch.float32', list(value.shape)]).encode()
            sha.update(len(header).to_bytes(8, 'big')); sha.update(header); sha.update(value.tobytes())
    contract.require(sha.hexdigest() == info['final_state_sha256'], 'numeric state digest differs')


def validate_known_receipt(value):
    contract.require(set(value) == {'dev_town01', 'holdout_town02_p02'}, 'known split keys differ')
    for split, count in (('dev_town01',118), ('holdout_town02_p02',232)):
        contract.require(set(value[split]) == set(ARMS), 'known split arms differ')
        for info in value[split].values():
            matrix = np.asarray(info['confusion'])
            contract.require(info['count'] == count and matrix.shape == (7,7)
                and np.issubdtype(matrix.dtype, np.integer) and (matrix >= 0).all()
                and int(matrix.sum()) == count*360*640, 'known split count differs')


def fit_binding(expected_summary_sha):
    contract.require(FIT.is_dir() and FIT.resolve() == FIT and not any(path.is_symlink() for path in FIT.rglob('*')),
                     'canonical non-symlink local fit required')
    contract.require(contract.digest(FIT/'summary.json') == expected_summary_sha, 'explicit final fit SHA differs')
    _, plan = package_metadata()
    summary, started, complete = (contract.read_json(FIT/name) for name in ('summary.json','started.json','fit_complete.json'))
    validate_endpoint_metadata(summary, started, complete, plan)
    geometry.source_binding(FIT/'source', SOURCE_SHA)
    expected = {'prepared_manifest.json':geometry.DATA_SHA, 'reference_plan.json':geometry.PLAN_SHA,
        'candidate_manifest.json':geometry.package.CANDIDATE_SHA, 'package_authorization.json':geometry.PACKAGE_APPROVAL_SHA,
        'reference_manifest.json':geometry.package.BASE_SHA, 'fit_authorization.json':summary['fit_authorization_sha256']}
    for name, sha in expected.items():
        contract.require(contract.digest(FIT/'inputs'/name) == sha, 'archived fit input changed')
    geometry.validate_fit_authorization(contract.read_json(FIT/'inputs/fit_authorization.json'), geometry.DATA_SHA, SOURCE_SHA)
    known = contract.read_json(FIT/'known_evaluation.json')
    contract.require(known == summary['known_evaluation'], 'known350 original receipt differs')
    validate_known_receipt(known)
    hashes = {name:contract.digest(FIT/name) for name in ('summary.json','started.json','fit_complete.json','known_evaluation.json')}
    for arm, info in complete['arms'].items():
        contract.require(contract.read_json(FIT/arm/'training.json') == info, 'arm receipt differs')
        for name, key in (('weights.npz','weights_sha256'), ('checkpoint.pt','checkpoint_sha256')):
            contract.require(contract.digest(FIT/arm/name) == info[key], 'endpoint artifact changed')
            hashes[arm+'/'+name] = info[key]
        verify_numeric_npz(FIT/arm/'weights.npz', info)
        hashes[arm+'/training.json'] = contract.digest(FIT/arm/'training.json')
    return hashes


def load_models(bindings):
    contract.require(os.environ.get('CUDA_VISIBLE_DEVICES') == '' and os.environ.get('OMP_NUM_THREADS') == '2'
        and os.environ.get('OPENBLAS_NUM_THREADS') == '2', 'CPU2 launch environment required before imports')
    import torch
    from threadpoolctl import threadpool_info
    torch.set_num_threads(2); torch.set_num_interop_threads(2)
    pools = threadpool_info()
    contract.require(all(p['num_threads'] == 2 for p in pools if p['user_api'] == 'blas'), 'native BLAS not CPU2')
    sys.path.insert(0, str(FIT/'source'))
    core, context = (importlib.import_module(name) for name in ('map_perception','map_context_recovery'))
    for module in (core, context):
        contract.require(Path(module.__file__).resolve().parent == FIT/'source', 'model escaped frozen source')
    contract.require(tuple(core.LABELS) == frozen.LABELS, 'class order differs')
    return torch, {arm:frozen.safe_npz_state(context.MapContextRecovery(), FIT/arm/'weights.npz', bindings[arm+'/weights.npz'], torch)
                   for arm in ARMS}, pools


def panel_layout(rgb, target, predictions, conditions):
    if len(conditions) == 1:
        return [(0,57,rgb['true']), (640,57,frozen.COLORS[target]),
            (0,442,frozen.COLORS[predictions['control_true']]), (640,442,frozen.COLORS[predictions['geometry_true']])]
    return [(column*640,57+row*385,pixels) for column,condition in enumerate(conditions)
        for row,pixels in enumerate((rgb[condition], frozen.COLORS[target],
            frozen.COLORS[predictions['control_'+condition]], frozen.COLORS[predictions['geometry_'+condition]]))]


def render(scenario, index, rgb, target, predictions, conditions):
    paired = len(conditions) == 2
    canvas = Image.new('RGB', (1280,1640 if paired else 860), (18,27,40))
    draw = ImageDraw.Draw(canvas)
    font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 16)
    draw.text((8,7), f'{scenario} | fixed frame {index:03d}/239 | fresh control vs geometry | 10000 steps', font=font, fill='white')
    captions = ([text for condition in conditions for text in (f'RGB {condition}','Shared GT','Control: old-source replay','Geometry: new p02/p04 source')]
                if paired else ['Original RGB true','Semantic GT','Control: old-source replay','Geometry: new p02/p04 source'])
    for (x,y,pixels), caption in zip(panel_layout(rgb,target,predictions,conditions), captions):
        draw.text((x+8,y-22), caption, font=font, fill='white'); canvas.paste(Image.fromarray(pixels),(x,y))
    draw.text((8,canvas.height-44), 'TRAIN resubstitution only.' if paired else 'Exposed CTrack EVAL only; not independent validation.', font=font, fill='white')
    draw.text((8,canvas.height-21), 'LEFT original FAIL remains. Historical photo is not this paired control. No promotion.', font=font, fill='white')
    return canvas


def used_bytes():
    if not OUTPUT.exists():
        return 0
    paths = list(OUTPUT.rglob('*'))
    contract.require(not OUTPUT.is_symlink() and not any(p.is_symlink() for p in paths), 'output symlink forbidden')
    return sum(p.stat().st_size for p in paths if p.is_file())


def write_artifact(path, payload, failure=False):
    contract.require(path.parent.parent == OUTPUT and path.name == Path(path.name).name, 'fixed scenario output only')
    maximum = MAX_OUTPUT_BYTES if failure else MAX_OUTPUT_BYTES-FAILURE_RESERVE_BYTES
    contract.require(used_bytes()+len(payload) < maximum, 'OUTPUT_BUDGET_EXCEEDED_KEEP_REMOTE_REQUIRED')
    contract.require(shutil.disk_usage(OUTPUT.parent).free-len(payload) >= FREE_FLOOR_BYTES, 'five-GiB free floor')
    with path.open('xb') as stream:
        stream.write(payload)
    return contract.digest(path)


def json_bytes(value):
    return (json.dumps(value, indent=2, sort_keys=True, allow_nan=False)+'\n').encode()


def compare_historical(scenario, registration, predictions, results):
    receipt = registration['historical_receipts'][scenario]
    root = Path(receipt['root'])
    contract.require(contract.digest(root/'summary.json') == receipt['summary_sha256'], 'historical summary changed')
    prior = contract.read_json(root/'summary.json')
    result = {}
    for condition, sha in receipt['predictions'].items():
        path = root/f'photometric_{condition}_all240_predictions.npz'
        contract.require(contract.digest(path) == sha, 'historical prediction changed')
        with np.load(path, allow_pickle=False) as data:
            contract.require(data.files == ['prediction'], 'historical prediction keys differ')
            old = data['prediction']
        contract.require(old.shape == (240,360,640) and old.dtype == np.uint8 and int(old.max()) < 7, 'historical mask ABI')
        result[condition] = {arm:dict(equal_predicted_pixels=int(np.count_nonzero(predictions[arm+'_'+condition] == old)),
            total_predicted_pixels=int(old.size), iou_delta_vs_historical_photo={label:
                None if prior['model_results']['photometric_'+condition]['class_metrics'][label]['iou'] is None else
                results[arm+'_'+condition]['class_metrics'][label]['iou']-prior['model_results']['photometric_'+condition]['class_metrics'][label]['iou']
                for label in frozen.LABELS}) for arm in ARMS}
    return result


def execute(scenario, expected_summary_sha):
    registration = validate_registration(); bindings = fit_binding(expected_summary_sha)
    output = output_path(scenario)
    contract.require(not output.exists() and not output.is_symlink(), 'fresh scenario required; no resume')
    source, records, conditions, qa, distinction = capture_binding(scenario, registration)
    torch, models, pools = load_models(bindings)
    contract.require(shutil.disk_usage(OUTPUT.parent).free >= FREE_FLOOR_BYTES+MAX_OUTPUT_BYTES-used_bytes(), 'output disk reserve unavailable')
    output.mkdir(parents=True)
    rows, visuals, predictions = [], {}, {}
    start_time = time.monotonic()
    try:
        write_artifact(output/'started.json', json_bytes(dict(scenario=scenario, policy=policy(),
            fit_artifact_sha256=bindings, preregistration_sha256=contract.digest(PREREG),
            raw_receipt=registration['raw_receipts'][scenario], historical_receipt=registration['historical_receipts'][scenario],
            native_threadpools=pools, **distinction)))
        write_artifact(output/'source_qa_replayed.json', json_bytes(qa))
        keys = [arm+'_'+condition for condition in conditions for arm in ARMS]
        predictions = {key:np.empty((240,360,640), np.uint8) for key in keys}
        matrices, timings = {key:np.zeros((7,7), np.int64) for key in keys}, {key:[] for key in keys}
        with torch.no_grad():
            for index, row in enumerate(records):
                contract.require(time.monotonic()-start_time < 600, 'fixed scenario wall budget exceeded')
                contract.require(contract.digest(source/(row['stem']+'.npz')) == row['artifact_sha256']['.npz'], 'raw frame changed')
                rgb, target = raw_frame(source,row,conditions)
                report = dict(index=index, sensor_frame=row['frame'], timestamp_s=row['timestamp_s'])
                for condition in conditions:
                    tensor = torch.from_numpy(rgb[condition].transpose(2,0,1).copy()).float().unsqueeze(0)/255
                    for arm, model in models.items():
                        key = arm+'_'+condition; started = time.perf_counter()
                        logits = model(tensor)
                        prediction = logits.argmax(1)[0].numpy().astype(np.uint8)
                        timings[key].append((time.perf_counter()-started)*1000)
                        contract.require(bool(torch.isfinite(logits).all()), 'nonfinite model output')
                        predictions[key][index] = prediction
                        matrix = frozen.confusion(target,prediction); matrices[key] += matrix
                        report[key] = dict(confusion=matrix.tolist(), inference_ms=timings[key][-1])
                rows.append(report)
                if index in INDICES:
                    buffer = io.BytesIO()
                    render(scenario,index,rgb,target,{key:value[index] for key,value in predictions.items()},conditions).save(buffer,format='PNG')
                    name = f'{scenario}_{index:03d}_full_fov.png'
                    visuals[name] = write_artifact(output/name, buffer.getvalue())
                if (index+1)%40 == 0:
                    print(json.dumps(dict(scenario=scenario, frames_per_model_condition=index+1)), flush=True)
        artifacts = {}
        for key, value in predictions.items():
            buffer = io.BytesIO(); np.savez_compressed(buffer,prediction=value)
            name = key+'_all240_predictions.npz'
            artifacts[name] = write_artifact(output/name,buffer.getvalue())
        results = {key:{**frozen.metrics(matrices[key]), 'frames':240,
            'inference_p50_ms':float(np.quantile(timings[key],.5)), 'inference_p95_ms':float(np.quantile(timings[key],.95)),
            'first_inference_ms':timings[key][0]} for key in keys}
        delta = {condition:{label:None if results['control_'+condition]['class_metrics'][label]['iou'] is None else
            results['geometry_'+condition]['class_metrics'][label]['iou']-results['control_'+condition]['class_metrics'][label]['iou']
            for label in frozen.LABELS} for condition in conditions}
        prior = compare_historical(scenario,registration,predictions,results)
        verify_raw_hashes(scenario,registration)
        contract.require(fit_binding(expected_summary_sha) == bindings, 'endpoint changed during inference')
        validate_registration()
        row_sha = write_artifact(output/'per_frame.json',json_bytes(rows))
        result = dict(schema='portable_e2e.map_geometry_endpoint_diagnostic.v1', status='DIAGNOSTIC_COMPLETE_NOT_PROMOTED',
            scenario=scenario, source=str(source), model_results=results, iou_delta_geometry_minus_control=delta,
            historical_photo_comparison=prior, prediction_artifacts=artifacts, full_fov_visuals=visuals,
            per_frame_sha256=row_sha, fit_artifact_sha256=bindings, preregistration_sha256=contract.digest(PREREG),
            source_fingerprints_unchanged=True, frames_per_model_condition=240, device='cpu', threads=2,
            torch_version=torch.__version__, native_threadpools=pools, fit_steps_during_evaluation=0,
            ctrack_train_eligible=False, model_promotion=False, independent_validation=False, performance_gates=None,
            known350_role='separate original trainer receipt; not recounted or merged here',
            limitations=policy()['limitations'], **distinction)
        write_artifact(output/'summary.json',json_bytes(result))
        return dict(status=result['status'], scenario=scenario, output=str(output), aggregate_bytes=used_bytes())
    except BaseException as error:
        failure = dict(status='FAILED_PRESERVED_NO_PARTIAL_PASS', error=repr(error), scenario=scenario,
            completed_frame_count=len(rows), fit_summary_sha256=expected_summary_sha,
            completed_per_frame=rows,
            completed_prediction_numeric_sha256={key:hashlib.sha256(value[:len(rows)].tobytes()).hexdigest()
                for key,value in predictions.items()},
            all_prediction_masks_persisted=False, saved_partial_artifacts_retained=True,
            full_frame_coverage_not_reduced=True, remote_storage_stage_requires_separate_parent_review=True)
        try:
            write_artifact(output/'failure.json',json_bytes(failure),failure=True)
        except BaseException:
            print(json.dumps(failure), flush=True)
        raise


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument('--preregister',action='store_true'); modes.add_argument('--execute',action='store_true')
    parser.add_argument('--scenario',choices=SCENARIOS)
    parser.add_argument('--expected-fit-summary-sha256')
    args = parser.parse_args()
    if args.preregister:
        result = preregister()
    elif args.execute:
        contract.require(args.scenario and args.expected_fit_summary_sha256, 'fixed scenario and final fit SHA required')
        result = execute(args.scenario,args.expected_fit_summary_sha256)
    else:
        validate_registration(); result = dict(status='FROZEN_PREREGISTRATION_PASS', predictions_made=False)
    print(json.dumps(result, sort_keys=True), flush=True)
