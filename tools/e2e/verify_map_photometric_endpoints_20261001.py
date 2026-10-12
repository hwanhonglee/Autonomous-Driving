#!/usr/bin/env python3
"""HH_261001 - Independently recount all new photometric predictions and native panels."""

from datetime import datetime, timezone
import argparse
import json
import numpy as np
from PIL import Image
from scripts.e2e import evaluate_map_photometric_endpoints_20261001 as evaluate
from scripts.e2e.verify_map_appearance_endpoints_20261001 import independent_target, independent_confusion, check_metrics


def verify(scenario, expected_summary_sha):
    c = evaluate.contract
    registration = evaluate.validate_registration(); bindings = evaluate.fit_binding(expected_summary_sha)
    source,records,conditions,qa,distinction = evaluate.capture_binding(scenario,registration)
    output = evaluate.output_path(scenario)
    summary,started = (c.read_json(output/name) for name in ('summary.json','started.json'))
    c.require(summary['schema'] == 'portable_e2e.map_photometric_endpoint_diagnostic.v1'
        and summary['status'] == 'DIAGNOSTIC_COMPLETE_NOT_PROMOTED' and summary['scenario'] == scenario
        and summary['fit_steps_during_evaluation'] == 0 and summary['frames_per_model_condition'] == 240
        and summary['ctrack_train_eligible'] is False and summary['model_promotion'] is False
        and summary['independent_validation'] is False and summary['performance_gates'] is None
        and summary['source_fingerprints_unchanged'] is True, 'diagnostic completion/role differs')
    for item in (summary,started):
        c.require(item['fit_artifact_sha256'] == bindings and item['preregistration_sha256'] == c.digest(evaluate.PREREG)
            and all(item[key] == value for key,value in distinction.items()), 'endpoint/forensic binding differs')
    c.require(started['policy'] == registration['policy'] and started['raw_receipt'] == registration['raw_receipts'][scenario]
        and started['preceding_receipt'] == registration['preceding_receipts'][scenario], 'started registration differs')
    c.require(c.digest(output/'per_frame.json') == summary['per_frame_sha256'], 'per-frame receipt changed')
    rows = c.read_json(output/'per_frame.json'); c.require(len(rows) == 240, 'per-frame count differs')
    keys = [arm+'_'+condition for condition in conditions for arm in evaluate.ARMS]
    predictions,matrices = {},{}
    c.require(set(summary['prediction_artifacts']) == {key+'_all240_predictions.npz' for key in keys}, 'prediction members differ')
    for key in keys:
        path = output/(key+'_all240_predictions.npz')
        c.require(c.digest(path) == summary['prediction_artifacts'][path.name], 'prediction bytes changed')
        with np.load(path,allow_pickle=False) as data:
            c.require(data.files == ['prediction'], 'prediction keys differ'); value = data['prediction'].copy()
        c.require(value.shape == (240,360,640) and value.dtype == np.uint8 and int(value.max()) < 7, 'all240 prediction ABI differs')
        predictions[key],matrices[key] = value,np.zeros((7,7),np.int64)
    checked_panels = 0
    for index,row in enumerate(records):
        c.require(rows[index]['index'] == index and rows[index]['sensor_frame'] == row['frame']
            and rows[index]['timestamp_s'] == row['timestamp_s'], 'per-frame identity changed')
        raw = source/(row['stem']+'.npz')
        c.require(c.digest(raw) == row['artifact_sha256']['.npz'], 'raw target changed')
        with np.load(raw,allow_pickle=False) as data:
            target = independent_target(data['semantic'])
            rgb = {condition:data['rgb' if len(conditions)==1 else 'rgb_'+condition][...,[2,1,0]] for condition in conditions}
        for key in keys:
            matrix = independent_confusion(target,predictions[key][index])
            c.require(matrix.tolist() == rows[index][key]['confusion'], 'independent frame confusion differs')
            matrices[key] += matrix
        if index in evaluate.INDICES:
            path = output/f'{scenario}_{index:03d}_full_fov.png'
            c.require(c.digest(path) == summary['full_fov_visuals'][path.name], 'panel artifact changed')
            canvas = np.asarray(Image.open(path))
            c.require(canvas.shape == (1640 if len(conditions)==2 else 860,1280,3), 'native canvas ABI differs')
            # HH_261001 - Independent coordinates keep every native RGB/GT/new-arm pixel visible.
            if len(conditions) == 1:
                panels = [(0,57,rgb['true']),(640,57,evaluate.frozen.COLORS[target]),
                    (0,442,evaluate.frozen.COLORS[predictions['control_true'][index]]),
                    (640,442,evaluate.frozen.COLORS[predictions['photometric_true'][index]])]
            else:
                panels = [(column*640,y,pixels) for column,condition in enumerate(('false','true'))
                    for y,pixels in ((57,rgb[condition]),(442,evaluate.frozen.COLORS[target]),
                        (827,evaluate.frozen.COLORS[predictions['control_'+condition][index]]),
                        (1212,evaluate.frozen.COLORS[predictions['photometric_'+condition][index]]))]
            for x,y,pixels in panels:
                c.require(np.array_equal(canvas[y:y+360,x:x+640],pixels), 'full native panel pixels differ')
                checked_panels += 1
    for key,matrix in matrices.items():
        c.require(int(matrix.sum()) == 240*360*640, 'full pixel count differs')
        check_metrics(matrix,summary['model_results'][key])
    for condition in conditions:
        for label in evaluate.frozen.LABELS:
            left = summary['model_results']['control_'+condition]['class_metrics'][label]['iou']
            right = summary['model_results']['photometric_'+condition]['class_metrics'][label]['iou']
            c.require(summary['iou_delta_photometric_minus_control'][condition][label] == (None if left is None else right-left), 'new-arm delta differs')
    prior_receipt = registration['preceding_receipts'][scenario]
    prior_root = evaluate.Path(prior_receipt['root'])
    prior = c.read_json(prior_root/'summary.json')
    for condition,sha in prior_receipt['appearance_predictions'].items():
        path = prior_root/f'appearance_{condition}_all240_predictions.npz'
        c.require(c.digest(path) == sha, 'preceding appearance prediction changed')
        with np.load(path,allow_pickle=False) as data:
            old = data['prediction'].copy()
        for arm in evaluate.ARMS:
            actual = summary['preceding_appearance_comparison'][condition][arm]
            c.require(actual['total_predicted_pixels'] == old.size
                and actual['equal_predicted_pixels'] == old.size-int(np.count_nonzero(old != predictions[arm+'_'+condition])), 'preceding all-pixel comparison differs')
            for label in evaluate.frozen.LABELS:
                left = prior['model_results']['appearance_'+condition]['class_metrics'][label]['iou']
                right = summary['model_results'][arm+'_'+condition]['class_metrics'][label]['iou']
                c.require(actual['iou_delta_vs_preceding_appearance'][label] == (None if left is None else right-left), 'preceding appearance metric delta differs')
    evaluate.verify_raw_hashes(scenario,registration)
    c.require(evaluate.fit_binding(expected_summary_sha) == bindings, 'fixed endpoint changed during recount')
    evaluate.validate_registration()
    result = {'schema':'portable_e2e.map_photometric_endpoint_recount.v1','status':'PASS',
        'created_at_utc':datetime.now(timezone.utc).isoformat(),'scenario':scenario,
        'frames_per_model_condition':240,'model_condition_count':len(keys),'predicted_pixels_recounted':len(keys)*240*360*640,
        'native_full_fov_panels_verified':checked_panels,'all_per_frame_and_aggregate_counts_match':True,
        'preceding_appearance_all_pixel_comparison_verified':True,'summary_sha256':c.digest(output/'summary.json'),
        'fit_summary_sha256':expected_summary_sha,'inference_repeated':False,'gpu_used':False,
        'model_promotion':False,'ctrack_train_eligible':False,**distinction}
    evaluate.frozen.write_new(output/'prediction_audit.json',result)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument('--scenario',choices=evaluate.SCENARIOS,required=True)
    parser.add_argument('--expected-fit-summary-sha256',required=True)
    args = parser.parse_args()
    print(json.dumps(verify(args.scenario,args.expected_fit_summary_sha256),sort_keys=True),flush=True)
