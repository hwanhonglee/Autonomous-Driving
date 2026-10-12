#!/usr/bin/env python3
"""HH_261001 - Aggregate all four fixed diagnostics only after complete independent recounts."""

from datetime import datetime, timezone
import json
from pathlib import Path
from scripts.e2e import evaluate_map_photometric_endpoints_20261001 as evaluate

FIT_SHA = '27f2951c6a62da6c4ee76ccd1dae24350afaf85cc1b41824ae50dbeecada6c6d'
PREREG_SHA = '7019e192782a05eb4cfc2066a43c01ef02c1e8c32a5573341355a090e91e2711'


def percentage(value):
    return 'null' if value is None else f'{100*value:.4f}'


def run():
    c = evaluate.contract; registration = evaluate.validate_registration(); binding = evaluate.fit_binding(FIT_SHA)
    c.require(c.digest(evaluate.PREREG) == PREREG_SHA, 'preregistration changed')
    scenarios,total_pixels,total_panels = {},0,0
    for scenario in evaluate.SCENARIOS:
        output = evaluate.output_path(scenario)
        summary,audit = (c.read_json(output/name) for name in ('summary.json','prediction_audit.json'))
        c.require(audit['status'] == 'PASS' and audit['summary_sha256'] == c.digest(output/'summary.json')
            and audit['fit_summary_sha256'] == FIT_SHA and summary['fit_artifact_sha256'] == binding
            and summary['preregistration_sha256'] == PREREG_SHA
            and audit['preceding_appearance_all_pixel_comparison_verified'] is True, 'complete independent scenario audit required')
        scenarios[scenario] = {'output':str(output),'summary_sha256':c.digest(output/'summary.json'),
            'recount_sha256':c.digest(output/'prediction_audit.json'),'results':summary['model_results'],
            'photometric_minus_control_iou':summary['iou_delta_photometric_minus_control'],
            'preceding_appearance_comparison':summary['preceding_appearance_comparison'],
            'native_panels':summary['full_fov_visuals'],'role':summary['evaluation_role'],
            'source_qa_status':summary['source_qa_status'],'original_owner_status':summary['original_owner_status'],
            'original_live_replay_exact_match':summary['original_live_replay_exact_match']}
        total_pixels += audit['predicted_pixels_recounted']; total_panels += audit['native_full_fov_panels_verified']
    c.require(total_pixels == 663552000 and total_panels == 72, 'all four scenarios and native panels required')
    known_path = evaluate.DOCS/'known_split_independent_recount.json'
    known = c.read_json(known_path)
    c.require(known['status'] == 'PASS' and known['summary_sha256'] == FIT_SHA
        and known['total_recounted_pixels'] == 161280000, 'known-split independent recount missing')
    for key,check in known['checks'].items():
        split,arm = key.split('/')
        c.require(check['all_pixel_confusion_exact'] is True
            and c.digest(evaluate.FIT/arm/(split+'_predictions.npz')) == check['prediction_sha256'], 'known prediction receipt differs')
    fit = c.read_json(evaluate.FIT/'summary.json')
    value = {'schema':'portable_e2e.map_photometric_all_endpoint_results.v1','status':'ALL_FOUR_DIAGNOSTICS_RECOUNTED',
        'created_at_utc':datetime.now(timezone.utc).isoformat(),'fit_summary_sha256':FIT_SHA,'preregistration_sha256':PREREG_SHA,
        'predicted_pixels_recounted':total_pixels,'native_panels_verified':total_panels,'scenarios':scenarios,
        'fit_artifacts':binding,'known_final_only_results':fit['known_evaluation'],
        'known_split_recount_sha256':c.digest(known_path),'known_split_pixels_recounted_separately':161280000,
        'model_promotion':False,'ctrack_train_eligible':False,'independent_validation':False,
        'limitations':registration['policy']['limitations'],'report_source_sha256':c.digest(Path(__file__).resolve())}
    lines = ['# Pure photometric matched endpoints: all fixed results','',
        '<!-- HH_261001 - Report all preregistered outputs without selection, retuning or promotion. -->','',
        'All four scenarios independently recounted: 2,880 predictions, 663,552,000 pixels and 72 native full-FOV panels. No frames filtered.', '',
        'Both fresh models use original729 + pairedTRUE480, the original B parent, architecture, 10,000 steps and identical sample/crop exposure. Only the candidate applies the archived photometric policy to TRUE480 during fitting. The reference is NOT the preceding FALSE-RGB control.', '',
        'CTrack remains exposed EVAL-only diagnostic reuse. Paired TRAIN is resubstitution, not held-out validation. LEFT original owner FAIL and unavailable live-map equality remain unchanged.', '',
        f'Preregistration: `{PREREG_SHA}`. Imported final fit: `{FIT_SHA}`.', '',
        'IoU values are percentages; deltas are percentage points. Absent-GT classes have null IoU, with false positives retained.','']
    for scenario,result in scenarios.items():
        lines.extend(['## '+scenario,'',f"Role: `{result['role']}`. QA `{result['source_qa_status']}`; original live equality `{result['original_live_replay_exact_match']}`.",''])
        for condition in evaluate.locations(scenario)[2]:
            lines.extend(['### RGB postprocess='+condition,'',
                '| Class | Fresh TRUE control % | Photometric % | Delta pp | GT pixels | Control FP | Photo FP |',
                '| --- | ---: | ---: | ---: | ---: | ---: | ---: |'])
            control,candidate = (result['results'][arm+'_'+condition] for arm in evaluate.ARMS)
            for label in evaluate.frozen.LABELS:
                a,b = control['class_metrics'][label],candidate['class_metrics'][label]
                delta = None if a['iou'] is None else b['iou']-a['iou']
                lines.append(f"| {label} | {percentage(a['iou'])} | {percentage(b['iou'])} | {percentage(delta)} | {a['ground_truth_pixels']:,} | {a['false_positive_pixels']:,} | {b['false_positive_pixels']:,} |")
            lines.extend(['',f"CPU2 forward+argmax p95: control {control['inference_p95_ms']:.3f} ms; photometric {candidate['inference_p95_ms']:.3f} ms. Cold first included; not unloaded or end-to-end 10Hz proof.",''])
            prior = result['preceding_appearance_comparison'][condition]['control']
            lines.extend([f"Separate preceding appearance reference: fresh control agrees on {prior['equal_predicted_pixels']:,}/{prior['total_predicted_pixels']:,} predicted labels. Exact per-class deltas for both new arms are retained in RESULTS.json.",''])
        lines.extend(['Fixed full-FOV panels (0/119/239):',''])
        lines.extend(f"- [{name}]({result['output']}/{name})" for name in sorted(result['native_panels']))
        lines.append('')
    lines.extend(['## External road-to-sidewalk errors','',
        '| Scenario | Model | GT road | Road predicted sidewalk | Fraction % |','| --- | --- | ---: | ---: | ---: |'])
    for scenario in ('ctrack_left','ctrack_straight'):
        for arm in evaluate.ARMS:
            row = scenarios[scenario]['results'][arm+'_true']['confusion'][1]
            lines.append(f'| {scenario} | {arm} | {sum(row):,} | {row[2]:,} | {100*row[2]/sum(row):.4f} |')
    lines.extend(['','## Known old-lighting retention','',
        'Final-only known DEV/holdout, independently recounted by the parent: 161,280,000 additional pixels. No intermediate selection; known/exposed splits are not independent certification.','',
        '| Split | Class | Fresh TRUE control % | Photometric % | Delta pp |','| --- | --- | ---: | ---: | ---: |'])
    for split,result in fit['known_evaluation'].items():
        for label in evaluate.frozen.LABELS:
            a,b = (result[arm]['iou'][label] for arm in evaluate.ARMS)
            lines.append(f'| {split} | {label} | {percentage(a)} | {percentage(b)} | {percentage(None if a is None else b-a)} |')
    lines.extend(['','No promotion, CTrack training admission, HD-map completion or closed-loop driving claim follows.',''])
    evaluate.frozen.write_new(evaluate.DOCS/'RESULTS.json',value)
    with (evaluate.DOCS/'RESULTS.md').open('x') as stream:
        stream.write('\n'.join(lines))
    return {'status':value['status'],'predicted_pixels_recounted':total_pixels,'native_panels_verified':total_panels,
        'json_sha256':c.digest(evaluate.DOCS/'RESULTS.json'),'report':str(evaluate.DOCS/'RESULTS.md')}


if __name__ == '__main__':
    print(json.dumps(run(),sort_keys=True),flush=True)
