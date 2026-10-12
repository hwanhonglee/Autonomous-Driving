#!/usr/bin/env python3
"""HH_261001 - Publish every completed fixed geometry diagnostic; no inference or selection."""

import argparse
import json
from pathlib import Path
from scripts.e2e import evaluate_map_geometry_endpoints_20261001 as evaluate

FIT_SHA = 'caa1a60bcff8179b80de0a5861aa2ab0497d838638fbf80a3f4790e8074e11ef'
KNOWN_SHA = '760861c1f4327f98c8a2c3a66b0a22f666a85100ff47bd2e026de69429a66558'


def percentage(value):
    return 'null' if value is None else f'{100*value:.4f}'


def collect():
    c = evaluate.contract
    evaluate.validate_registration()
    bindings = evaluate.fit_binding(FIT_SHA)
    cases = {}
    total_pixels = total_panels = 0
    for scenario in evaluate.SCENARIOS:
        root = evaluate.output_path(scenario)
        summary, audit = (c.read_json(root/name) for name in ('summary.json','prediction_audit.json'))
        c.require(summary['status'] == 'DIAGNOSTIC_COMPLETE_NOT_PROMOTED' and audit['status'] == 'PASS'
            and audit['summary_sha256'] == c.digest(root/'summary.json') and audit['fit_summary_sha256'] == FIT_SHA
            and summary['fit_artifact_sha256'] == bindings and not (root/'failure.json').exists(),
            'all four complete independently recounted scenarios required')
        for field in ('prediction_artifacts','full_fov_visuals'):
            for name,sha in summary[field].items():
                c.require(c.digest(root/name) == sha, 'recounted output bytes changed')
        total_pixels += audit['predicted_pixels_recounted']
        total_panels += audit['native_full_fov_panels_verified']
        cases[scenario] = dict(summary=summary, summary_sha256=c.digest(root/'summary.json'),
            independent_recount=audit, independent_recount_sha256=c.digest(root/'prediction_audit.json'))
    c.require(total_pixels == 663552000 and total_panels == 72, 'fixed complete denominators differ')
    known_path = evaluate.INVENTORY/'known_split_independent_recount.json'
    c.require(c.digest(known_path) == KNOWN_SHA, 'separate known350 recount changed')
    known = c.read_json(known_path)
    c.require(known['status'] == 'PASS_DIAGNOSTIC_NOT_PROMOTED' and known['summary_sha256'] == FIT_SHA
        and known['total_recounted_pixels'] == 161280000 and known['unique_known_frames'] == 350,
        'known350 integrity receipt differs')
    fitted = c.read_json(evaluate.FIT/'summary.json')
    training = {arm:{key:info[key] for key in ('steps','training_wall_seconds','initial_state_sha256',
        'final_state_sha256','weights_sha256','checkpoint_sha256','sample_sequence_sha256','crop_sequence_sha256',
        'common_class_weights','photo_trace','exposure')} for arm,info in fitted['arms'].items()}
    return dict(schema='portable_e2e.map_geometry_complete_results.v1', status='ALL_FIXED_DIAGNOSTICS_RECOUNTED_NOT_PROMOTED',
        fit_summary_sha256=FIT_SHA, preregistration_sha256=c.digest(evaluate.PREREG),
        completed_training=training,
        all_four_fixed_scenarios=cases, predictions=2880, predicted_pixels=total_pixels,
        native_full_fov_panels=total_panels, output_bytes=evaluate.used_bytes(),
        known350_separate=dict(receipt_sha256=KNOWN_SHA, receipt=known, trainer_original=c.read_json(evaluate.FIT/'known_evaluation.json')),
        no_new_inference=True, no_fit=True, model_promotion=False, ctrack_train_eligible=False,
        independent_validation=False, summarizer_sha256=c.digest(Path(__file__)))


def english(result):
    lines = ['# Fixed geometry-source endpoints: all results', '',
        '<!-- HH_261001 - Preserve all scenarios, regressions and historical distinctions. -->', '',
        'Both fresh control/geometry endpoints completed10,000 steps. The only paired intervention is old Town01 FALSE replay versus new p02/p04 FALSE route sources in the matched extra480 slots. Common TRUE augmentation, parent, model, class weights, crop/sample budget and losses remain fixed.', '',
        '**All four fixed diagnostics independently recounted:**2,880 predictions,663,552,000 pixels,72 native full-FOV subpanels. No frame filtering. CTrack is exposed EVAL only; paired TRAIN is resubstitution. LEFT original owner FAIL and unavailable original live equality remain. This is not independent certification, mapping completion or model promotion.', '',
        'IoU values below are percentages; deltas are percentage points. Null denotes an absent GT class; false positives remain counted. Historical photo comparisons are retained separately in `RESULTS.json` and are not the new paired control.', '']
    for scenario, case in result['all_four_fixed_scenarios'].items():
        summary = case['summary']; conditions = evaluate.locations(scenario)[2]
        lines += ['## '+scenario, '', 'Role: `'+summary['evaluation_role']+'`; source QA `'+summary['source_qa_status']+'`.', '']
        for condition in conditions:
            lines += ['### RGB postprocess='+condition, '',
                '| Class | Fresh control % | Geometry % | Delta pp | GT pixels | Control FP | Geometry FP |',
                '|---|---:|---:|---:|---:|---:|---:|']
            for label in evaluate.frozen.LABELS:
                a,b = (summary['model_results'][arm+'_'+condition]['class_metrics'][label] for arm in evaluate.ARMS)
                delta = summary['iou_delta_geometry_minus_control'][condition][label]
                lines += [f"| {label} | {percentage(a['iou'])} | {percentage(b['iou'])} | {percentage(delta)} | {a['ground_truth_pixels']:,} | {a['false_positive_pixels']:,} | {b['false_positive_pixels']:,} |"]
            a,b = (summary['model_results'][arm+'_'+condition] for arm in evaluate.ARMS)
            lines += ['',f"CPU2 forward+argmax p95: control{a['inference_p95_ms']:.3f}ms; geometry{b['inference_p95_ms']:.3f}ms. Cold first included; not end-to-end10Hz.",'']
        lines += ['Fixed native PNGs: '+ ' · '.join(f'[{i:03d}]({evaluate.output_path(scenario)}/{scenario}_{i:03d}_full_fov.png)' for i in evaluate.INDICES), '']
    lines += ['## External road predicted as sidewalk', '',
        '| Scenario | Control % of GT road | Geometry % of GT road |', '|---|---:|---:|']
    for scenario in ('ctrack_left','ctrack_straight'):
        summary = result['all_four_fixed_scenarios'][scenario]['summary']
        matrices = [summary['model_results'][arm+'_true']['confusion'] for arm in evaluate.ARMS]
        values = [100*matrix[1][2]/sum(matrix[1]) for matrix in matrices]
        lines += [f'| {scenario} | {values[0]:.4f} | {values[1]:.4f} |']
    lines += ['', '## Separate known350 retention', '',
        'Original trainer predictions on known Town01 DEV118 and Town02 p02 holdout232 were independently recounted without new forwards:161,280,000 pixels across both models. These are separate denominators, not additional external or unseen validation.', '',
        '| Split | Class | Control % | Geometry % | Delta pp |','|---|---|---:|---:|---:|']
    for split, arms in result['known350_separate']['trainer_original'].items():
        for label in evaluate.frozen.LABELS:
            a,b = (arms[arm]['iou'][label] for arm in evaluate.ARMS)
            lines += [f'| {split} | {label} | {percentage(a)} | {percentage(b)} | {percentage(None if a is None else b-a)} |']
    lines += ['', '## Evidence and limits', '',
        f"Fresh four-scenario output bytes: **{result['output_bytes']:,}**, strictly below30MiB. [Frozen plan](PLAN.md), [preregistration](preregistration.json), [complete original metrics, masks/panel hashes and all recounts](RESULTS.json).", '',
        'No new projection experiment, geometry/depth learning, HD-map topology, full-route coverage, driving validation or real-sensor validation is implied. The raw guardrail ontology caveat and limited Town01 FALSE source diversity remain. No thresholds, protected splits or failed records were changed.', '']
    return '\n'.join(lines)


def main():
    result = collect()
    paths = [evaluate.DOCS/name for name in ('RESULTS.json','RESULTS.md')]
    evaluate.contract.require(not any(p.exists() for p in paths), 'fresh report paths required')
    payload = evaluate.json_bytes(result)
    markdown = english(result).encode()
    evaluate.contract.require(evaluate.used_bytes()+len(payload)+len(markdown) < evaluate.MAX_OUTPUT_BYTES,
                              'reports plus prediction outputs exceed30MiB')
    evaluate.frozen.write_new(paths[0],result)
    with paths[1].open('xb') as stream:
        stream.write(markdown)
    print(json.dumps(dict(status=result['status'], output_bytes=result['output_bytes'],
        json_sha256=evaluate.contract.digest(paths[0]), markdown_sha256=evaluate.contract.digest(paths[1]))))


if __name__ == '__main__':
    argparse.ArgumentParser(description=__doc__,allow_abbrev=False).parse_args()
    main()
