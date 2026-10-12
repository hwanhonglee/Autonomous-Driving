"""HH_261001 - Independent fixed W1/W2 receipt, export and existing metric comparison; no new evaluation."""

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from scripts.e2e import report_warmup_supervision_train_probes_20261001 as probes

W1 = 'W1_warmup_anchor_off'
W2 = 'W2_warmup_anchor_off_4blocks'
W2_SCHEMA = 'portable_e2e.warmup_duration_w2_20261001.v1'
PARENT_SHA = '4841d4076fb49145643ab2895d87befcae9021ad02f08b8307e75f77be137d91'
W1_STATE = 'ffcb58cd319bb6d1db412da71f8f9ef8697613482cdd2d760c64b7abbf6f110f'
W1_CHECKPOINT = 'b9ee946ada6824e7d79f51bc1693463dc506db3faaa9038ddaa05816cf7ecf76'
W1_DECLARATION = '8de2bb58d6b7deadab5523c68fc12f45d854cf9a217c18e99606d9045d334f31'
W1_RECEIPT = '7115a4b49e6d9842c6a05d86b77286ec7e89c304222b548072f76733598e455a'
TRAIN_SOURCE = 'c218dc27e09a94406e7bc2a23d5c7320a1b73eeb63cd09093e578980f5ef29c6'
TRAIN_PLAN = '70229908028ac30cb9fd51a64e66d144485099280ebfd7128422e937dc5010b3'
ORDER = 'f0d3d522b3ec4787b84286f82e61017a6aee4619270a99451d3cea08e35110d9'
MASK = '2c46c730096452291e10eed040c228e31ca2dc6f2ef424a02820e8d04efc3b1e'
FIRST_FIT = dict(scheduled_steps=3000, global_step=2999, samples_seen=11996, active_supervision_exposures=11549)
FINAL_FIT = dict(global_step=11996, scheduled_steps=12000, samples_seen=47984, scheduled_samples=48000,
    active_supervision_exposures=46196, skipped_schedule_steps_one_based=[1693, 4693, 7693, 10693],
    optimizer_state_included=True, optimizer_resumed=False, optimizer_reset_between_blocks=False)


def require(condition, reason):
    if not condition:
        raise ValueError(reason)


def sha(path):
    require(path.is_file() and not path.is_symlink(), 'regular non-symlink evidence required: ' + str(path))
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def compact(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()


def json_sha(value):
    return hashlib.sha256(compact(value)).hexdigest()


def read(path, ledger):
    ledger[str(path)] = sha(path)
    return json.loads(path.read_text())


def npz_state_digest(path, metadata):
    """HH_261001 - Independently reproduce the tensor digest without loading any pickle/checkpoint."""
    require(sha(path) == metadata['weights_sha256'], 'NPZ bytes differ from sidecar')
    digest = hashlib.sha256()
    with np.load(path, allow_pickle=False) as tensors:
        require(set(tensors.files) == set(metadata['tensors']), 'NPZ tensor keys differ')
        for name in sorted(tensors.files):
            value, declared = tensors[name], metadata['tensors'][name]
            require(value.dtype == np.dtype('float32') and not value.dtype.hasobject
                    and list(value.shape) == declared['shape'] and str(value.dtype) == declared['dtype']
                    and bool(np.isfinite(value).all()), 'nonfinite/unsafe/different tensor ABI: ' + name)
            header = compact([name, 'torch.float32', list(value.shape)])
            digest.update(len(header).to_bytes(8, 'big'))
            digest.update(header)
            digest.update(np.ascontiguousarray(value).tobytes())
    return digest.hexdigest()


def verify_first(first):
    require(first['status'] == 'PASS_EXACT_W1_STATE'
        and first['expected_model_state_sha256'] == first['actual_model_state_sha256'] == W1_STATE
        and first['fit'] == FIRST_FIT and first['optimizer_same_object'] is True
        and first['optimizer_reloaded'] is False and first['checkpoint_used_for_model_selection'] is False
        and first['checkpoint'] == 'first_block_identity_checkpoint.pt', 'first block is not exact continued W1')


def compare_metrics(before, after, gates_before, gates_after):
    require(before['sample_count'] == after['sample_count'] == 665
        and before['evaluation_pass_index'] == after['evaluation_pass_index'] == 1
        and before['test_evaluated'] is after['test_evaluated'] is False, 'fixed known DEV population/pass differs')
    require(set(before['subgroups']) == set(after['subgroups']), 'DEV subgroup coverage differs')
    rows = []
    for group, baseline in before['subgroups'].items():
        candidate = after['subgroups'][group]
        require(baseline['metric_counts'] == candidate['metric_counts']
                and set(baseline['metrics']) == set(candidate['metrics']), 'DEV metric denominator differs')
        for metric, value in baseline['metrics'].items():
            current = candidate['metrics'][metric]
            require(math.isfinite(value) and math.isfinite(current), 'nonfinite preserved DEV metric')
            rows.append(dict(group=group, metric=metric, count=baseline['metric_counts'][metric], W1=value, W2=current,
                W2_minus_W1=current-value, numerical_direction='lower' if current < value else 'higher' if current > value else 'equal'))
    require(len(gates_before['metric_checks']) == len(gates_after['metric_checks']) == 9, 'original nine gate coverage differs')
    for old, new in zip(gates_before['metric_checks'], gates_after['metric_checks']):
        require((old['group'], old['metric'], old['maximum']) == (new['group'], new['metric'], new['maximum']), 'absolute gate changed')
        for report, gate in ((before, old), (after, new)):
            require(gate['observed'] == report['subgroups'][gate['group']]['metrics'][gate['metric']]
                    and gate['pass'] == (gate['observed'] <= gate['maximum']), 'published metric gate does not recompute')
    for gate in (gates_before, gates_after):
        require(gate['metric_pass_count'] == sum(row['pass'] for row in gate['metric_checks'])
                and gate['geometry_pass_count'] == sum(row['pass'] for row in gate['geometry_checks']), 'gate totals differ')
    for report, gates in ((before, gates_before), (after, gates_after)):
        observed = dict(selected_geometry_pass_count=report['geometry']['selected_result']['geometry_pass_count'],
            any_candidate_geometry_pass_count=report['geometry']['any_candidate_geometry_pass_count'],
            all_candidates_geometry_pass_count=report['geometry']['all_candidates_geometry_pass_count'])
        require(len(gates['geometry_checks']) == 3, 'geometry gate coverage differs')
        for gate in gates['geometry_checks']:
            require(gate['minimum'] == 664 and gate['observed'] == observed[gate['metric']]
                    and gate['pass'] == (gate['observed'] >= gate['minimum']), 'geometry gate does not recompute')
    return rows


def compare_probes(before, after, declaration, proof):
    expected = declaration['fixed_probe_indices']
    require(len(expected) == 279 and [row['index'] for row in before] == [row['index'] for row in after] == expected,
            'fixed 279 probes changed or reordered')
    models = {}
    for label, rows in (('W1', before), ('W2', after)):
        for baseline, row in zip(before, rows):
            require(all(row[key] == baseline[key] for key in ('token', 'current_speed_mps', 'target_speed', 'target_xy', 'target_valid')),
                    'fixed TRAIN probe input/target identity changed')
        records = [probes.compact(row, proof['rows'][row['index']], proof['phase_ids'][row['index']]) for row in rows]
        grouped = probes.groups(records)
        require(grouped['original_immediate_start18']['count'] == 18 and grouped['protected_stop_hold200']['count'] == 200,
                'start/STOP denominator changed')
        models[label] = dict(groups=grouped, records=records,
            stop_hold_acquisition_phase_counts=dict(Counter(row['acquisition_phase'] for row in records if row['original_phase_id'] == 0)),
            stop_hold_future_crosses_imposed_tail_count=sum(proof['rows'][row['index']]['future_crosses_imposed_tail']
                for row in records if row['original_phase_id'] == 0),
            stop_hold_masked_warmup_count=sum(proof['rows'][row['index']]['warmup_off']
                for row in records if row['original_phase_id'] == 0),
            consecutive_5_to_1=probes.consecutive_switches(records))
    changes = []
    for before_row, after_row in zip(models['W1']['records'], models['W2']['records']):
        changes.append(dict(index=before_row['index'], token=before_row['token'], phase_id=before_row['original_phase_id'],
            route_group=before_row['route_group'], current_speed_mps=before_row['current_speed_mps'],
            W1_selected=before_row['selected'], W2_selected=after_row['selected'],
            W1_oracle=before_row['oracle'], W2_oracle=after_row['oracle'],
            oracle_agreement_change=int(after_row['oracle_agreement'])-int(before_row['oracle_agreement']),
            near_speed_mae_delta_mps=after_row['first5_selected_speed_mae_mps']-before_row['first5_selected_speed_mae_mps'],
            gate_pass_change=int(after_row['selected_runtime_gate_pass'])-int(before_row['selected_runtime_gate_pass'])))
    return models, changes


def change_counts(changes):
    cohorts = dict(all_fixed279=changes, immediate_start18=[row for row in changes if row['phase_id'] == 1],
        stop_hold200=[row for row in changes if row['phase_id'] == 0],
        moving_1p5_to_2p5=[row for row in changes if 1.5 <= row['current_speed_mps'] <= 2.5],
        right_fixed=[row for row in changes if row['route_group'] == 'right'])
    return {name: dict(count=len(rows), oracle_match_gained=sum(row['oracle_agreement_change'] > 0 for row in rows),
        oracle_match_lost=sum(row['oracle_agreement_change'] < 0 for row in rows),
        early_speed_mae_lower=sum(row['near_speed_mae_delta_mps'] < 0 for row in rows),
        early_speed_mae_higher=sum(row['near_speed_mae_delta_mps'] > 0 for row in rows),
        early_speed_mae_equal=sum(row['near_speed_mae_delta_mps'] == 0 for row in rows),
        selected_candidate_changed=sum(row['W1_selected'] != row['W2_selected'] for row in rows),
        dynamic_oracle_changed=sum(row['W1_oracle'] != row['W2_oracle'] for row in rows),
        selected_geometry_pass_lost=sum(row['gate_pass_change'] < 0 for row in rows)) for name, rows in cohorts.items()}


def report(args):
    ledger = {}
    w1_decl = read(args.w1 / 'declaration.json', ledger)
    w1_receipt = read(args.w1 / W1 / 'training_receipt.json', ledger)
    require(ledger[str(args.w1 / 'declaration.json')] == W1_DECLARATION
            and ledger[str(args.w1 / W1 / 'training_receipt.json')] == W1_RECEIPT, 'fixed W1 reference changed')
    declaration = read(args.w2 / 'declaration.json', ledger)
    sources = declaration['source_sha256']
    require(len(sources) == 27 and json_sha(sources) == TRAIN_SOURCE, 'exact W2 source27 closure differs')
    for name, digest in sources.items():
        relative = Path(name)
        require(not relative.is_absolute() and '..' not in relative.parts, 'unsafe archived source path')
        path = args.w2 / 'source' / relative
        require(sha(path) == digest, 'archived W2 source differs: ' + name)
        ledger[str(path)] = digest
    require(declaration['schema'] == W2_SCHEMA and declaration['training_plan_sha256'] == TRAIN_PLAN
        and declaration['fixed_probe_indices'] == w1_decl['fixed_probe_indices']
        and declaration['training_count'] == 2787 and declaration['warmup_mask_sha256'] == MASK
        and declaration['block_schedule_sha256'] == ORDER, 'W2 fixed data/schedule/probe declaration differs')
    receipt = read(args.w2 / W2 / 'training_receipt.json', ledger)
    complete = read(args.w2 / 'fixed_final_checkpoint_complete.json', ledger)
    summary = read(args.w2 / 'summary.json', ledger)
    require(receipt == complete == summary['training'] and receipt['fit'] == FINAL_FIT
        and summary['status'] == 'COMPLETE_NOT_PROMOTED' and summary['source_hashes_match'] is True,
        'actual fixed endpoint is missing or accounting differs')
    require(not (args.w2 / W2 / 'failure.json').exists(), 'retained failed W2 fit cannot be treated as complete')
    first = read(args.w2 / W2 / 'first_block_identity.json', ledger)
    verify_first(first)
    require(receipt['first_block_identity'] == first and w1_receipt['final_model_state_sha256'] == W1_STATE,
            'first-block receipt chain differs')
    for index, block in enumerate(receipt['block_receipts'], 1):
        expected = {key: value * index for key, value in FIRST_FIT.items()}
        require(index <= 4 and block['block'] == index and block['fit'] == expected and block['optimizer_same_object'] is True
                and block['block_exposure_order_sha256'] == ORDER, 'continuous optimizer/block accounting differs')
        archived = read(args.w2 / W2 / 'training' / f'block_{index:02d}_accounting.json', ledger)
        require(archived == block, 'archived block receipt differs')
    require(len(receipt['block_receipts']) == 4, 'missing fixed training block')
    log_path = args.w2 / W2 / 'training/history.jsonl'
    ledger[str(log_path)] = sha(log_path)
    history = [json.loads(line) for line in log_path.read_text().splitlines()]
    require([row['scheduled_step'] for row in history if row.get('skipped')] == FINAL_FIT['skipped_schedule_steps_one_based']
            and history[-1]['scheduled_step'] == 12000 and history[-1]['optimizer_updates'] == 11996,
            'periodic training log endpoint/skips differ')
    w1_summary = read(args.w1 / 'summary.json', ledger)
    before = read(args.w1 / W1 / 'final_dev_evaluation.json', ledger)
    after = read(args.w2 / W2 / 'final_dev_evaluation.json', ledger)
    require(before == w1_summary['evaluations'][W1] and after == summary['evaluation']
            and after['checkpoint_sha256'] == receipt['checkpoint_sha256'] and after['schema'] == W2_SCHEMA
            and summary['model_promoted'] is summary['training_data_approved'] is summary['vehicle_control_approved'] is False,
            'final evaluation/checkpoint/scope chain differs')
    metrics = compare_metrics(before, after, w1_summary['strict_gates'][W1], summary['strict_gates'])
    proof = read(args.w1 / 'provenance_receipt.json', ledger)
    w2_proof = read(args.w2 / 'provenance_receipt.json', ledger)
    require(proof == w2_proof and json_sha(proof) == declaration['provenance_receipt_sha256'], 'fixed TRAIN provenance changed')
    before_probe = read(args.w1 / W1 / 'train_probe_after.json', ledger)
    after_probe = read(args.w2 / W2 / 'train_probe_after.json', ledger)
    model_probes, changes = compare_probes(before_probe, after_probe, declaration, proof)
    require(sha(args.binding) == args.binding_sha256, 'caller-pinned export binding changed')
    binding = read(args.binding, ledger)
    require(binding['schema'] == 'portable_e2e.w2_replay_endpoint_binding.v1'
        and binding['checkpoint_sha256'] == receipt['checkpoint_sha256']
        and binding['model_state_sha256'] == receipt['final_model_state_sha256']
        and binding['fit'] == FINAL_FIT and binding['first_block_identity'] == first
        and binding['training_source_sha256'] == TRAIN_SOURCE and binding['training_plan_sha256'] == TRAIN_PLAN
        and binding['parent_checkpoint_sha256'] == PARENT_SHA, 'secure export/checkpoint identity chain differs')
    metadata = read(args.weights.with_suffix('.json'), ledger)
    require(ledger[str(args.weights.with_suffix('.json'))] == binding['metadata_sha256']
        and metadata['source_checkpoint_sha256'] == receipt['checkpoint_sha256']
        and metadata['source_checkpoint_schema'] == W2_SCHEMA
        and metadata['weights_sha256'] == binding['weights_sha256'], 'W2 export sidecar differs')
    state_hash = npz_state_digest(args.weights, metadata)
    require(state_hash == receipt['final_model_state_sha256'], 'independent NPZ numeric digest differs from actual endpoint')
    ledger[str(args.weights)] = sha(args.weights)
    w1_weights = args.w1 / 'exports' / (W1 + '.npz')
    w1_metadata = read(w1_weights.with_suffix('.json'), ledger)
    require(metadata['model_config'] == w1_metadata['model_config'] and metadata['tensors'] == w1_metadata['tensors']
            and w1_metadata['source_checkpoint_sha256'] == W1_CHECKPOINT
            and npz_state_digest(w1_weights, w1_metadata) == W1_STATE, 'W1/W2 architecture or W1 tensor identity differs')
    ledger[str(w1_weights)] = sha(w1_weights)
    local_checkpoints = {}
    for name, expected in ((W2 + '/training/final_checkpoint.pt', receipt['checkpoint_sha256']),
                           (W2 + '/training/first_block_identity_checkpoint.pt', first['checkpoint_sha256'])):
        matches = [digest for path, digest in binding['evidence_file_sha256'].items() if path.endswith('/' + name)]
        require(matches == [expected], 'restricted remote export checkpoint evidence differs')
        path = args.w2 / name
        local_checkpoints[name] = dict(present=path.is_file(), bytes_sha256_verified=False)
        if path.is_file():
            require(sha(path) == expected, 'local checkpoint bytes differ')
            ledger[str(path)] = expected
            local_checkpoints[name]['bytes_sha256_verified'] = True
    result = dict(schema='portable_e2e.warmup_duration_independent_comparison.v1', status='FIXED_RESULTS_RECOUNTED_NOT_PROMOTED',
        input_sha256=ledger, W1_checkpoint_sha256=W1_CHECKPOINT, W2_checkpoint_sha256=receipt['checkpoint_sha256'],
        accounting=dict(W1=w1_receipt['fit'], W2=receipt['fit'], first_block=first, archived_W2_sources_verified=27,
            elapsed_W2_training_s=receipt['elapsed_training_s'], periodic_history_rows=len(history)),
        export_identity=dict(W1_npz_model_state_sha256=W1_STATE, W2_npz_model_state_sha256=state_hash,
            independently_recomputed_npz_numeric_identity=True, local_checkpoint_bytes=local_checkpoints,
            local_pickle_deserialization_performed=False, remote_restricted_checkpoint_export_binding_sha256=args.binding_sha256),
        original_final_dev={'W1': before, 'W2': after}, original_strict_gates={'W1': w1_summary['strict_gates'][W1], 'W2': summary['strict_gates']},
        all_dev_metric_deltas=metrics, fixed_train_probes=model_probes, per_probe_changes=changes, fixed_probe_change_counts=change_counts(changes),
        new_neural_forwards_by_report=0, new_optimizer_steps_by_report=0, test_read_or_evaluated=False,
        training_data_approved=False, model_promoted=False, vehicle_control_approved=False,
        limitations=['Known DEV and TRAIN diagnostics, not independent validation or a new TEST evaluation.',
            'The dynamic oracle is recomputed over each model candidates; oracle agreement is not fixed-class accuracy.',
            'STOP/hold acquisition phase, warmup masking and future crossings into imposed tail are reported separately; a label is not proof of a genuine traffic/goal stop.',
            'Checkpoint tensor deserialization was performed by the recorded restricted remote export; this report independently checks NPZ numeric digest and local checkpoint bytes if present.',
            'No live driving outcome or cure can be inferred from these metrics alone.'])
    require(all(sha(Path(path)) == digest for path, digest in ledger.items()), 'comparison input changed during audit')
    return result


def markdown(result):
    lines = ['# HH_261001 — Fixed W1 versus W2 duration comparison', '',
        'Fixed results recounted; neither model is promoted. No new training, neural evaluation or TEST read was performed.', '',
        f"W1: 2,999 optimizer updates. W2: 11,996 updates across four unchanged schedules; exact first-block W1 state `{W1_STATE}` verified by receipt chain.", '',
        f"Archived training closure: 27/27 files verified. W2 actual fit time: {result['accounting']['elapsed_W2_training_s']:.1f} s. Export NPZ numeric digest independently matches the final trained-state receipt.", '',
        '| Fixed TRAIN probe group | n | W1 oracle match | W2 oracle match | W1 early speed MAE | W2 early speed MAE |',
        '| --- | ---: | ---: | ---: | ---: | ---: |']
    groups = ('original_immediate_start18', 'protected_stop_hold200', 'moving_speed_1p5_to_2p5', 'right_fixed_probes')
    for name in groups:
        a, b = (result['fixed_train_probes'][arm]['groups'][name] for arm in ('W1', 'W2'))
        lines.append(f"| {name} | {a['count']} | {a['oracle_agreement']} | {b['oracle_agreement']} | {a['mean_first5_speed_mae_mps']:.6f} | {b['mean_first5_speed_mae_mps']:.6f} |")
    stop = result['fixed_train_probes']['W1']
    lines += ['', 'Oracle means each model’s existing dynamic best-of-12 objective; not an invented moving-candidate label.', '',
        f"STOP/hold200 acquisition phases: {stop['stop_hold_acquisition_phase_counts']}; masked warmup anchors: {stop['stop_hold_masked_warmup_count']}; futures crossing imposed tail: {stop['stop_hold_future_crosses_imposed_tail_count']}. No genuine traffic/goal-stop claim is inferred.", '',
        '| Fixed group | Oracle matches gained / lost | Early speed MAE improved / worsened / equal |',
        '| --- | ---: | ---: |']
    for name in ('immediate_start18', 'stop_hold200', 'moving_1p5_to_2p5', 'right_fixed'):
        row = result['fixed_probe_change_counts'][name]
        lines.append(f"| {name} | {row['oracle_match_gained']} / {row['oracle_match_lost']} | {row['early_speed_mae_lower']} / {row['early_speed_mae_higher']} / {row['early_speed_mae_equal']} |")
    lines += ['',
        '| Known DEV metric | W1 | W2 | W2 − W1 |', '| --- | ---: | ---: | ---: |']
    for row in result['all_dev_metric_deltas']:
        if row['metric'] in ('ade_6p4s_m', 'fde_6p4s_m', 'speed_mae_6p4s_mps'):
            lines.append(f"| {row['group']} / {row['metric']} | {row['W1']:.6f} | {row['W2']:.6f} | {row['W2_minus_W1']:+.6f} |")
    gates = result['original_strict_gates']
    lines += ['', f"Original absolute gates: W1 {gates['W1']['metric_pass_count']}/9, W2 {gates['W2']['metric_pass_count']}/9. All original metrics, failures, denominators, geometry gates and individual probe changes are retained in `comparison.json`.", '',
        'This is a bounded duration test on the same data, not a new data-coverage remedy, model approval or proof of a live cure.', '']
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('w1', 'w2', 'binding', 'weights', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--binding-sha256', required=True)
    args = parser.parse_args()
    require(not args.output.exists(), 'fresh independent report directory required')
    result = report(args)
    result['report_source_sha256'] = {str(Path(__file__).resolve()): sha(Path(__file__)), str(Path(probes.__file__).resolve()): sha(Path(probes.__file__))}
    args.output.mkdir(parents=True)
    (args.output / 'comparison.json').write_text(json.dumps(result, indent=2, sort_keys=True, allow_nan=False) + '\n')
    (args.output / 'README.md').write_text(markdown(result))
    print(json.dumps(dict(status=result['status'], output=str(args.output), actual_W2_optimizer_updates=result['accounting']['W2']['global_step'])))


if __name__ == '__main__':
    main()
