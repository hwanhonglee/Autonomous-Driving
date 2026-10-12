"""HH_261001 - Independently recount released fixed A/B receipts, numeric exports, DEV and TRAIN probes."""

import argparse
from collections import Counter
import json
import math
from pathlib import Path

from scripts.e2e import report_warmup_duration_w2_comparison_20261001 as original

ROOT = Path(__file__).resolve().parents[2]
ARMS = ('A_native_W1_reproduction','B_directional_lower_speed_gradient')
LABELS = ('A','B')
SCHEMA = 'portable_e2e.directional_lower_gradient_ab_20261001.v1'
CLOSURE_SHA = 'cec35999a624292318bb8b2640ea778730b16ca28ff3f99be7a291a105c0745c'
TRAIN_PLAN = '9fffd0ef2cef46a667b51affeb4e2f08eccaa3100e34fd19dd9ea7c4eff23908'
PLAN = ROOT/'docs/assets/validation/2026-10-01/selector_phase_ab/DIRECTIONAL_GRADIENT_REPORT_PROTOCOL.md'
PLAN_SHA = 'f8f6dc74b4de5fa5bd2996dce4da090d31520aecde6cded1fe6440baf38cce62'
TRACE_STEPS = (1,1000,2000,3000)
FIT = dict(global_step=2999,scheduled_steps=3000,samples_seen=11996,scheduled_samples=12000,
    active_supervision_exposures=11549,skipped_schedule_steps_one_based=[1693],
    optimizer_state_included=True,optimizer_resumed=False)
EXPORT_PINS = {
    'tools/e2e/export_directional_gradient_cpu_20261001.py':'bd418c869f2a16e419944cbfae14db924550869a5be0835308ab6461c37682d5',
    'learning/tests/test_directional_gradient_cpu_export_20261001.py':'3c7c1066c7587c1700fe9e2a98e82e3be4448991eed07a4cb2cb342cb769ac3d',
}
HELPER_PINS = {
    ROOT/'tools/e2e/report_warmup_duration_w2_comparison_20261001.py':'c1f40a766ae3ef40fcb02df77e947161ddbe30b39ce3ef8e555ae1d9d7d7bcdd',
    ROOT/'tools/e2e/report_warmup_supervision_train_probes_20261001.py':'810835aa81e5414036355db481149056b5695c96ff79f33102945b09631f6a7a',
    PLAN:PLAN_SHA,
}
require, sha, read = original.require, original.sha, original.read


def relabel(value):
    """HH_261001 - Rename helper result keys only; never alter observations, labels, tokens or metric values."""
    if isinstance(value,dict):
        return {(key.replace('W1','A').replace('W2','B') if isinstance(key,str) else key):relabel(item)
                for key,item in value.items()}
    if isinstance(value,list):
        return [relabel(item) for item in value]
    return value


def validate_history(rows):
    expected = sorted(list(range(100,3001,100))+[1693])
    require([r['scheduled_step'] for r in rows] == expected, 'fixed periodic history/skip coverage differs')
    for row in rows:
        step = row['scheduled_step']
        require(row['optimizer_updates'] == step-int(step>=1693), 'history actual update count differs')
        if step == 1693:
            require(row.get('skipped') is True and row.get('no_forward_or_optimizer_call') is True
                and row.get('indices') == [617,619,322,317], 'exact all-warmup skip evidence differs')
        else:
            require(not row.get('skipped') and all(math.isfinite(row[k]) for k in
                ('total_loss','base_loss','near_loss','gradient_norm','elapsed_s')), 'nonfinite periodic history')
    return dict(periodic_rows=len(rows),skipped_schedule_steps=[1693],actual_final_updates=2999)


def validate_trace(row, step, ids):
    require(row.get('scheduled_step') == step and step in TRACE_STEPS
        and row.get('optimizer_updates') == step-int(step>=1693)
        and row.get('indices') == list(ids) and row.get('backward_calls') == 64,
        'fixed B trace location/indices/backward coverage differs')
    names=('lower_saturated','lower_inward','lower_outward','lower_zero')
    require(all(type(row.get(k)) is int and 0 <= row[k] <= 4*6*64 for k in names), 'invalid detached trace counts')
    require(row['lower_saturated'] == sum(row[k] for k in names[1:])
        and isinstance(row.get('minimum_preclamp_speed'),(int,float))
        and math.isfinite(row['minimum_preclamp_speed']), 'trace partition/minimum differs')
    return dict(row,inward_fraction_of_lower=None if row['lower_saturated']==0 else row['lower_inward']/row['lower_saturated'])


def validate_export_binding(binding, complete):
    require(binding.get('schema') == 'portable_e2e.directional_gradient_export.v1'
        and binding.get('status') == 'BOTH_FIXED_ENDPOINTS_EXPORTED_NOT_PROMOTED'
        and binding.get('training_schema') == SCHEMA and binding.get('training_source_closure_sha256') == CLOSURE_SHA
        and binding.get('training_plan_sha256') == TRAIN_PLAN and binding.get('fixed_fit_each_arm') == FIT
        and binding.get('parent_checkpoint_sha256') == original.PARENT_SHA
        and binding.get('current_and_archived_training_sources_verified_before_and_after') is True
        and binding.get('original_inference_model') is True and binding.get('trained_surrogate_required_at_runtime') is False
        and binding.get('endpoint_selection_used_metric_values') is False
        and binding.get('vehicle_control_approved') is False and binding.get('model_promoted') is False
        and binding.get('simulator_launched') is False and binding.get('optimizer_updates') == 0
        and binding.get('export_source_sha256') == EXPORT_PINS
        and set(binding.get('endpoints',{})) == set(ARMS), 'restricted dual-endpoint export contract differs')
    for arm in ARMS:
        endpoint = binding['endpoints'][arm]
        require(endpoint['checkpoint'] == arm+'/training/final_checkpoint.pt'
            and endpoint['checkpoint_sha256'] == complete[arm]['checkpoint_sha256']
            and endpoint['model_state_sha256'] == complete[arm]['final_model_state_sha256']
            and endpoint['weights'] == 'exports/'+arm+'.npz'
            and endpoint['metadata'] == 'exports/'+arm+'.json', 'endpoint path/identity differs')
    require(binding['endpoints'][ARMS[0]]['model_state_sha256'] == original.W1_STATE,
            'native A does not reproduce W1 actual tensors')


def evidence_relative(root,name):
    relative=Path(name)
    require(not relative.is_absolute() and '..' not in relative.parts, 'unsafe relative evidence reference')
    path=root/relative
    require(path.resolve().is_relative_to(root.resolve()), 'evidence escaped released training root')
    return path


def report(args):
    ledger={}
    for path,expected in HELPER_PINS.items():
        require(sha(path)==expected, 'fixed independent reporter helper/protocol differs')
        ledger[str(path)]=expected
    # HH_261001 - The caller releases a completed pair; this reader has no monitoring or partial-fit mode.
    summary=read(args.training/'summary.json',ledger)
    complete=read(args.training/'both_final_checkpoints_complete.json',ledger)
    declaration=read(args.training/'declaration.json',ledger)
    require(summary.get('schema')==SCHEMA and summary.get('status')=='COMPLETE_NOT_PROMOTED'
        and summary.get('source_hashes_match') is True and summary.get('training')==complete
        and set(complete)==set(ARMS) and summary.get('model_promoted') is False
        and summary.get('training_data_approved') is False and summary.get('vehicle_control_approved') is False,
        'released complete fixed pair required, not partial or promoted results')
    sources=declaration['source_sha256']
    require(len(sources)==34 and original.json_sha(sources)==CLOSURE_SHA
        and declaration.get('schema')==SCHEMA and declaration.get('training_plan_sha256')==TRAIN_PLAN
        and declaration.get('fixed_fit')==FIT and declaration.get('parent_checkpoint_sha256')==original.PARENT_SHA
        and declaration.get('warmup_mask_sha256')==original.MASK
        and declaration.get('exposure_order_sha256')==original.ORDER
        and declaration.get('training_count')==2787, 'frozen matched pair declaration differs')
    for name,expected in sources.items():
        path=evidence_relative(args.training/'source',name)
        require(sha(path)==expected,'archived training source differs')
        ledger[str(path)]=expected
    w1_decl=read(args.w1_reference/'declaration.json',ledger)
    w1_receipt=read(args.w1_reference/original.W1/'training_receipt.json',ledger)
    require(ledger[str(args.w1_reference/'declaration.json')]==original.W1_DECLARATION
        and ledger[str(args.w1_reference/original.W1/'training_receipt.json')]==original.W1_RECEIPT
        and w1_receipt['final_model_state_sha256']==original.W1_STATE
        and declaration['fixed_probe_indices']==w1_decl['fixed_probe_indices'], 'original fixed W1 reference differs')
    proof=read(args.training/'provenance_receipt.json',ledger)
    original_proof=read(args.w1_reference/'provenance_receipt.json',ledger)
    require(proof==original_proof and original.json_sha(proof)==declaration['provenance_receipt_sha256'],
            'same fixed TRAIN labels/masks/order/provenance required')
    accounting,evaluations,probe_rows,traces={},{},{},[]
    for label,arm in zip(LABELS,ARMS):
        require(not (args.training/arm/'failure.json').exists(),'retained failed arm cannot be treated as completed')
        receipt=read(args.training/arm/'training_receipt.json',ledger)
        require(receipt==complete[arm] and receipt.get('fit')==FIT
            and receipt.get('exposure_order_sha256')==original.ORDER and receipt.get('warmup_mask_sha256')==original.MASK,
            'fixed actual fit counts/order/mask differ')
        history_path=args.training/arm/'training/history.jsonl'
        ledger[str(history_path)]=sha(history_path)
        history=[json.loads(line) for line in history_path.read_text().splitlines()]
        accounting[label]=dict(fit=FIT,elapsed_training_s=receipt['elapsed_training_s'],history=validate_history(history))
        expected_steps=[] if label=='A' else list(TRACE_STEPS)
        require([r['scheduled_step'] for r in receipt['gradient_trace']]==expected_steps,'trace schedule in fit receipt differs')
        actual_files=sorted(p.name for p in (args.training/arm/'training').glob('gradient_trace_*.json'))
        require(actual_files==[f'gradient_trace_{step:04d}.json' for step in expected_steps],'unexpected/missing gradient trace files')
        for row in receipt['gradient_trace']:
            step=row['scheduled_step']
            actual=read(args.training/arm/'training'/f'gradient_trace_{step:04d}.json',ledger)
            require(actual==row,'archived detached trace differs')
            traces.append(validate_trace(row,step,proof['schedule'][step-1]))
        evaluation=read(args.training/arm/'final_dev_evaluation.json',ledger)
        require(evaluation==summary['evaluations'][arm] and evaluation.get('schema')==SCHEMA
            and evaluation.get('checkpoint_sha256')==receipt['checkpoint_sha256'],'final DEV/checkpoint chain differs')
        evaluations[label]=evaluation
        probe_rows[label]=read(args.training/arm/'train_probe_after.json',ledger)
    native=read(args.training/ARMS[0]/'native_identity.json',ledger)
    require(native.get('status')=='PASS_EXACT_W1_STATE'
        and native.get('actual_model_state_sha256')==native.get('expected_model_state_sha256')==original.W1_STATE
        and native.get('actual_fit')==FIT and native.get('checkpoint_sha256')==complete[ARMS[0]]['checkpoint_sha256']
        and native.get('checkpoint_used_for_model_selection') is False,'native reproduction barrier differs')
    gates={label:summary['strict_gates'][arm] for label,arm in zip(LABELS,ARMS)}
    metric_rows=relabel(original.compare_metrics(evaluations['A'],evaluations['B'],gates['A'],gates['B']))
    historical_probe=read(args.w1_reference/original.W1/'train_probe_after.json',ledger)
    original.compare_probes(historical_probe,probe_rows['A'],declaration,proof)
    model_probes,changes=original.compare_probes(probe_rows['A'],probe_rows['B'],declaration,proof)
    change_counts=original.change_counts(changes)
    model_probes,changes=relabel(model_probes),relabel(changes)
    require(sha(args.binding)==args.binding_sha256,'caller-pinned fixed dual export binding differs')
    binding=read(args.binding,ledger)
    validate_export_binding(binding,complete)
    require(len(binding['evidence_file_sha256'])==42,'complete42-file dual-endpoint proof required')
    for name,expected in binding['evidence_file_sha256'].items():
        path=evidence_relative(args.training,name)
        require(sha(path)==expected,'export evidence changed after restricted checkpoint proof')
        ledger[str(path)]=expected
    exports={}
    reference_weights=args.w1_reference/'exports'/(original.W1+'.npz')
    reference_metadata=read(reference_weights.with_suffix('.json'),ledger)
    require(original.npz_state_digest(reference_weights,reference_metadata)==original.W1_STATE,'historical W1 numeric identity differs')
    ledger[str(reference_weights)]=sha(reference_weights)
    for label,arm in zip(LABELS,ARMS):
        endpoint=binding['endpoints'][arm]
        weights=evidence_relative(args.training,endpoint['weights'])
        metadata_path=evidence_relative(args.training,endpoint['metadata'])
        metadata=read(metadata_path,ledger)
        require(ledger[str(metadata_path)]==endpoint['metadata_sha256']
            and metadata['source_checkpoint_sha256']==endpoint['checkpoint_sha256']
            and metadata['source_checkpoint_schema']==SCHEMA and metadata['weights_sha256']==endpoint['weights_sha256']
            and metadata['model_config']==reference_metadata['model_config']
            and metadata['tensors']==reference_metadata['tensors'],'native inference export ABI/sidecar differs')
        state=original.npz_state_digest(weights,metadata)
        require(state==endpoint['model_state_sha256'],'actual numeric export state differs')
        ledger[str(weights)]=sha(weights)
        exports[label]=dict(checkpoint_sha256=endpoint['checkpoint_sha256'],model_state_sha256=state,
            NPZ_numeric_digest_independently_recomputed=True,local_pickle_deserialization=False)
    result=dict(schema='portable_e2e.directional_gradient_independent_comparison.v1',
        status='FIXED_RESULTS_RECOUNTED_NOT_PROMOTED',input_sha256=ledger,
        accounting=accounting,native_identity=native,archived_sources_verified=34,
        export_binding_sha256=args.binding_sha256,export_identity=exports,
        original_final_dev=evaluations,original_strict_gates=gates,all_dev_metric_deltas=metric_rows,
        fixed_train_probes=model_probes,per_probe_changes=changes,fixed_probe_change_counts=change_counts,
        A_historical_W1_probe_payloads_exact=historical_probe==probe_rows['A'],
        B_detached_gradient_trace=traces,gradient_trace_scope='Only four fixed batches; all rows/candidates including zero-weight/unselected paths, not full-fit rates.',
        new_neural_forwards=0,new_optimizer_steps=0,test_read_or_evaluated=False,
        simulator_launched=False,training_data_approved=False,model_promoted=False,vehicle_control_approved=False,
        limitations=['Known TRAIN/DEV, not independent validation or a TEST evaluation.',
            'Dynamic full-composite oracles differ with model candidates; oracle agreement is not fixed-class accuracy.',
            'Detached surrogate inward counts are not motion labels, global training rates, guaranteed descent, or live improvement.',
            'Protected STOP/hold labels include acquisition/tail provenance; do not infer genuine traffic/goal-stop competence.',
            'No new control ACK, physical state, lane-body sweep or counterfactual rollout is produced.'])
    require(all(sha(Path(path))==expected for path,expected in ledger.items()),'released report input changed during audit')
    return result


def markdown(result):
    lines=['# HH_261001 — Fixed directional-gradient A/B accounting','',
        'Both fixed endpoints recounted without a new neural forward, training update, TEST read or live run. Neither model is promoted.','',
        'A and B each executed 2,999 updates. A reproduces the exact W1 model tensor state. All 34 archived source files and both numeric export states are verified.','',
        '| Fixed TRAIN group | n | A oracle match | B oracle match | A first5 speed MAE | B first5 speed MAE |',
        '| --- | ---: | ---: | ---: | ---: | ---: |']
    for name in ('original_immediate_start18','protected_stop_hold200','moving_speed_1p5_to_2p5','right_fixed_probes'):
        a,b=(result['fixed_train_probes'][label]['groups'][name] for label in LABELS)
        lines.append(f"| {name} | {a['count']} | {a['oracle_agreement']} | {b['oracle_agreement']} | {a['mean_first5_speed_mae_mps']:.6f} | {b['mean_first5_speed_mae_mps']:.6f} |")
    lines+=['','| Known DEV metric | A | B | B − A |','| --- | ---: | ---: | ---: |']
    for row in result['all_dev_metric_deltas']:
        if row['metric'] in ('ade_6p4s_m','fde_6p4s_m','speed_mae_6p4s_mps'):
            lines.append(f"| {row['group']} / {row['metric']} | {row['A']:.6f} | {row['B']:.6f} | {row['B_minus_A']:+.6f} |")
    lines+=['','| B scheduled batch | Lower saturated | Inward | Outward blocked | Zero | Minimum z |',
            '| ---: | ---: | ---: | ---: | ---: | ---: |']
    for row in result['B_detached_gradient_trace']:
        lines.append(f"| {row['scheduled_step']} | {row['lower_saturated']} | {row['lower_inward']} | {row['lower_outward']} | {row['lower_zero']} | {row['minimum_preclamp_speed']:.6f} |")
    lines+=['','These four trace samples include unselected and zero-weight paths. They are not global training rates or evidence of safer motion.',
        '',f"Original absolute gates: A {result['original_strict_gates']['A']['metric_pass_count']}/9; B {result['original_strict_gates']['B']['metric_pass_count']}/9.",
        '', 'All denominators, failures, individual changes, geometry gates and STOP/tail provenance remain in comparison.json. Live driving and future physical body sweeps were not evaluated.','']
    return '\n'.join(lines)


def main():
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    for name in ('training','w1-reference','binding','output'):
        parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--binding-sha256',required=True)
    args=parser.parse_args()
    require(not args.output.exists() and not args.output.is_symlink(),'fresh independent report directory required')
    result=report(args)
    result['report_source_sha256']={str(Path(__file__)):sha(Path(__file__)),
        str(ROOT/'learning/tests/test_report_directional_lower_gradient_ab_20261001.py'):
        sha(ROOT/'learning/tests/test_report_directional_lower_gradient_ab_20261001.py')}
    args.output.mkdir(parents=True)
    with (args.output/'comparison.json').open('x') as stream:
        json.dump(result,stream,sort_keys=True,indent=2,allow_nan=False);stream.write('\n')
    with (args.output/'README.md').open('x') as stream:
        stream.write(markdown(result))
    print(json.dumps(dict(status=result['status'],output=str(args.output),new_neural_forwards=0,
        A_updates=2999,B_updates=2999)),flush=True)
    return 0


if __name__=='__main__':
    raise SystemExit(main())
