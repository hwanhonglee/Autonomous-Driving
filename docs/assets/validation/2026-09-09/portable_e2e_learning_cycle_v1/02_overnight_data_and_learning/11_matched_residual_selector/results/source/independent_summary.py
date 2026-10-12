#!/usr/bin/env python3
"""HH_260906 - Verify saved matched-residual information ablations without tensor loading, new gates or promotion."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import struct

from scripts.e2e import summarize_frozen_stopmix_selector as base

ROOT = Path(__file__).resolve().parents[2]
BASE_SHA = 'b6d32f8fa3aa446de0211123a032961fbbd2d22497b39d14b4a9dbd4f4b9427b'
PLAN_SHA = '5bdb3e49d39db3788c6db4ed1b717510d7f53abd0532f3ca66fb01ee006d98b9'
REPORT_SHA = '6259f0408fdba35ccfda0714603f5e5b6b4566e351a23d657756dfeb34edb2fe'
MANIFEST_SHA = 'c2245a99e60945117c1e7602f921cedde9fd23399dcb90aa64aaf853bd6d3ae3'
TRANSPORT_SHA = '930b4c45f79160a806115d72c163c2752fd8bc9a1b99ad78252a4323fbfa4f04'
NEW_SOURCES = {'external_residual_worker.py': 'c1572d06d8718ce9c22f9899ef0c2a2e02d35652d16d95164959fd41bc66e9e6',
    'external_residual_core.py': 'd4bf42fc9f4db930f31ef7b8ccd79900249687642f47bf525dedb240207550b7'}
ARMS = ('prior_only_residual', 'candidate_residual')
DENIALS = dict(generator_constructed=False, generator_neural_forwards=0, generator_training=False,
    new_candidate_decoding=False, cached_candidates_regenerated=False, original_parent_checkpoints_loaded=False,
    runtime_checkpoint_export=False, automatic_promotion=False, vehicle_control_approved=False,
    training_data_approved=False, source_or_label_changes=False, test_neural_inference=False, test_used_for_selection=False)
require, same, sha, digest, decode, encoded = base.require, base.same, base.sha, base.digest, base.decode, base.encoded


def expected_names(sources):
    names = {'plan.json', 'report.json'} | {'source/' + base.safe_name(n) for n in sources}
    for seed in base.SEEDS:
        prefix = f'seed_{seed}/'
        for view in ('original', *(a + '/analysis' for a in ARMS)):
            names |= {prefix + view + '/' + s + '/' + n for s in base.COUNTS for n in ('samples.jsonl', 'summary.json')}
            names |= {prefix + view + '/val/' + f'val_{i:03d}.png' for i in base.INDICES}
        names |= {prefix + arm + '/' + n for arm in ARMS for n in ('metrics.jsonl', 'report.json', 'head_only.pt', 'final_logits.pt')}
    return names


def outcome_ledger(before, report):
    rows = []
    for seed in base.SEEDS:
        for arm in ARMS:
            states = [a for s in report.get('seeds', []) if s.get('seed') == seed for a in s.get('arms', []) if a.get('arm') == arm]
            require(len(states) <= 1, 'duplicate residual head state')
            rows.append(dict(seed=seed, arm=arm, recorded_worker_status=states[0].get('status') if states else None,
                status='INCOMPLETE' if states or any(n.startswith(f'seed_{seed}/{arm}/') for n in before) else 'NOT_RUN'))
    return rows


def logits_digest(rows):
    # HH_260906 - Reconstruct only the byte digest of saved float32 logits, never deserialize a tensor or run a scorer.
    values = [v for row in rows for v in row['loss_alignment']['candidate_logits']]
    require(len(values) == len(rows) * 12 and all(base.finite(v) for v in values), 'all finite saved logits required')
    header = json.dumps(['logits', 'torch.float32', [len(rows), 12]], separators=(',', ':')).encode()
    return sha(header + b'\0' + struct.pack('<' + 'f' * len(values), *values))


def verify_cache_proof(proof, split, seed, parent_state, parent_files):
    expected = dict(path=f'seed_{seed}/{split}_cache.pt', cache_digest=parent_state['cache_digest'][split],
        sample_count=base.COUNTS[split], weights_only=True, map_location='cpu', targets_and_routes_reconstructed_exactly=True,
        fused_candidates_independently_regenerated=False, model_input_sha_recomputed=False)
    expected['sha256'] = parent_files[expected['path']]
    same(proof, expected, 'secure original cache identity/reconstruction scope differs')


def verify_fixed_rows(original, candidate, *, original_logits=False):
    require(len(original) == len(candidate), 'all paired samples required')
    for a, b in zip(original, candidate):
        same({k: a[k] for k in base.IDENTITY}, {k: b[k] for k in base.IDENTITY}, 'original cached input/target changed')
        same(a['candidates'], b['candidates'], 'frozen candidate behavior changed')
        same(a['runtime_geometry']['candidates'], b['runtime_geometry']['candidates'], 'raw-current candidate gates changed')
        same({k: a['loss_alignment'][k] for k in base.COST_INVARIANT},
            {k: b['loss_alignment'][k] for k in base.COST_INVARIANT}, 'fixed candidate costs/oracles/masks changed')
        if original_logits: same(a, b, 'cached original baseline replay changed')


def verify_head(head, history, seed, arm, plan, state, baselines, analyses):
    expected = dict(artifact_id='portable_e2e.frozen_stopmix_residual_research_head.v1', status='RESEARCH_HEAD_TRAINING_COMPLETE_NOT_PROMOTED',
        arm=arm, seed=int(seed), device='cuda:0', optimizer='AdamW', steps=1540, batch_size=4, learning_rate=.0001,
        weight_decay=.0001, candidate_score_weight=.1, maximum_gradient_norm=5., loss_config=plan['loss_config'],
        source_checkpoint_sha256=plan['parent_checkpoint_sha256'][seed], corpus_fingerprint_sha256=plan['corpus_fingerprint_sha256'],
        split_fingerprints={s: v['fingerprint_sha256'] for s, v in plan['split_contract'].items()}, head_parameter_count=115457,
        candidate_count=12, final_state=dict(global_step=1540, samples_seen=6155, last_epoch_index=5),
        residual_inputs=dict(frozen_context_features=256, original_candidate_score_features=1, candidate_geometry_features=192,
            geometry_provided=arm == 'candidate_residual', xy_divisor_m=120., speed_divisor_mps=30 / 3.6))
    for key, value in expected.items(): same(head[key], value, 'residual head contract differs: ' + key)
    require(all(head[k] is False for k in ('generator_parameters_trained', 'original_heads_recomputed_or_trained',
        'training_data_approved', 'vehicle_control_approved', 'automatic_promotion', 'runtime_bundle_supported')), 'research head denial changed')
    require(head['cache_sha256_before'] == head['cache_sha256_after'] == state['cache_digest']
        and all(base.hexsha(head[n]) for n in ('initial_head_sha256', 'final_head_sha256')), 'frozen cache or head digest invalid')
    order = base.verify_history(history, head, seed)
    for split in base.COUNTS:
        original, original_rows = baselines[split]; measured, rows = analyses[split]
        same(head[split + '_sample_ids'], [r['sample_id'] for r in rows], 'head sample IDs differ')
        require(head['composite_oracle_sha256'][split] == base.oracle_digest(measured['oracle_indices']), 'residual oracle tensor hash differs')
        parity = head['original_logit_parity'][split]; expected_logits = logits_digest(original_rows)
        require(parity['cached_original_logits_required'] is True and parity['byte_equal'] is True
            and parity['original_logits_sha256'] == parity['initial_logits_sha256'] == expected_logits
            and base.finite(parity['max_abs_logit_difference']) and parity['max_abs_logit_difference'] == 0
            and type(parity['selected_index_mismatch_count']) is int and parity['selected_index_mismatch_count'] == 0,
            'zero residual did not preserve original saved logit bytes')
        for name, view in (('baseline_metrics', original), ('post_training_metrics', measured)):
            metric = head[name][split]; base.verify_core_metrics(metric, split); costs = view['composite_cost_alignment']['all_samples']
            same(metric['selected_histogram'], [costs['selected_histogram'][str(i)] for i in range(12)], 'residual selected histogram differs')
            same(metric['composite_oracle_histogram'], [costs['composite_oracle_histogram'][str(i)] for i in range(12)], 'residual oracle histogram differs')
            same(metric['selected_composite_oracle_agreement_count'], costs['selected_composite_oracle_index_agreement_count'], 'residual oracle agreement differs')
        same(head['pre_training_metrics'][split], head['baseline_metrics'][split], 'zero residual initial metrics changed')
    return order


def verify_transport(root, before, original, missing, parent_transport, parent_report):
    require(before.get('transport_verification.json') == TRANSPORT_SHA, 'frozen residual transport receipt required')
    value = decode(base.regular(root / 'transport_verification.json').read_bytes())
    # HH_260906 - This fixed schema is completed with the independently reviewed collector before actual result admission to this summary.
    require(value['status'] == 'VERIFIED_SIX_RESIDUAL_HEAD_FITS_NOT_PROMOTED' and value['source_commit'] == base.COMMIT
        and value['plan_sha256'] == PLAN_SHA and value['report_sha256'] == REPORT_SHA
        and value['original_manifest_sha256'] == MANIFEST_SHA, 'residual transport execution binding differs')
    remote = value['regular_file_inventory']; expected = dict(original, SHA256SUMS=MANIFEST_SHA)
    require(set(remote) == set(expected) and set(value['omitted_files']) == missing
        and value['remote_file_count'] == 195 and value['collected_file_count'] == 183, 'residual transport denominator differs')
    for key in ('remote_pre_post_bytes_unchanged', 'source_and_parent_bytes_unchanged', 'cooperative_gpu0_lease_available_and_held',
        'local_launch_evidence_unchanged', 'first_parent_rejected_launcher_preserved'):
        require(value[key] is True, 'residual transport postcheck failed')
    require(all(value[k] is False for k in ('collector_tensor_loading', 'tensor_artifacts_transferred', 'model_promotion', 'data_admission')),
        'metadata-only residual collection required')
    same(value['original_input_verification']['original_cache_run_inventory'], parent_transport['regular_file_inventory'], 'original205 cache-run byte/size witnesses changed')
    same(value['original_input_verification']['continued_parent_input_sha256'], parent_report['original_input_sha256'], 'transport original17 parent witnesses changed')
    for key, expected_count in dict(analysis_sample_rows=13356, head_history_rows=9240, head_only_fit_count=6,
        full_model_fit_count=0, generator_neural_forwards=0, fixed_val_png_count=108, original_cache_files_loaded_by_worker=6).items():
        same(value[key], expected_count, 'residual transport count differs: ' + key)
    for name, pin in expected.items():
        proof = remote[name]; require(proof['sha256'] == pin and type(proof['size_bytes']) is int and proof['size_bytes'] >= 0, 'remote original inventory differs')
        if name in missing:
            item = value['omitted_files'][name]
            require(item['sha256'] == pin and item['size_bytes'] == proof['size_bytes'] and item['reason'], 'omitted residual PT witness differs')
        else: require(before[name] == pin and base.regular(root / name).stat().st_size == proof['size_bytes'], 'mirrored residual bytes differ')
    require(base.timestamp(value['collected_at_utc']) >= base.timestamp(decode(base.regular(root / 'report.json').read_bytes())['completed_at_utc']), 'residual collection predates completion')
    return dict(receipt_sha256=TRANSPORT_SHA, remote_tensor_bytes_verified_by_collector=True,
        local_tensor_bytes_loaded=False, omitted_remote_tensor_count=12)


def summarize_campaign(input_root, parent_root=None):
    require(digest(Path(base.__file__)) == BASE_SHA, 'reviewed original summary helper changed')
    root = Path(input_root).absolute(); parent_root = Path(parent_root).absolute() if parent_root else root.parent / 'frozen_stopmix_selector_v1'
    before = base.inventory(root); own = digest(Path(__file__))
    require(before.get('plan.json') == PLAN_SHA, 'exact residual prospective plan required')
    read = lambda name: base.regular(root / base.safe_name(name)).read_bytes()
    plan = decode(read('plan.json')); report = decode(read('report.json')) if 'report.json' in before else {}
    result = dict(schema='portable_e2e.frozen_stopmix_residual_summary.v1', status='INCOMPLETE_NOT_PROMOTED',
        plan_sha256=PLAN_SHA, summary_source_sha256=own, base_summary_source_sha256=BASE_SHA,
        performed_heads=outcome_ledger(before, report), input_sha256=before, seeds=[], **DENIALS)
    if not report or report.get('status') == 'FAILED_OR_PARTIAL_NOT_PROMOTED':
        result['retained_worker_report'] = report; require(base.inventory(root) == before, 'partial evidence changed'); return result
    require(report['schema'] == 'portable_e2e.external_frozen_stopmix_residual.v1'
        and report['status'] == 'SIX_RESIDUAL_HEAD_FITS_COMPLETE_NOT_PROMOTED'
        and before['report.json'] == REPORT_SHA and before.get('SHA256SUMS') == MANIFEST_SHA
        and report['plan_sha256'] == PLAN_SHA and plan['source_commit'] == base.COMMIT, 'frozen residual completion evidence required')
    for key, value in DENIALS.items(): same(report[key], value, 'recorded residual scope differs'); same(plan[key], value, 'residual plan scope differs')
    require(type(report['completed_head_fit_count']) is int and report['completed_head_fit_count'] == 6
        and report['postcheck_errors'] == [] and report['source_and_inputs_unchanged'] is True and report['deadline_met'] is True,
        'all six source-bound residual heads required')
    require(base.timestamp(plan['declared_at_utc']) <= base.timestamp(report['started_at_utc']) <= base.timestamp(report['completed_at_utc'])
        <= base.timestamp(plan['finish_before_utc']) and (base.timestamp(report['completed_at_utc']) - base.timestamp(report['started_at_utc'])).total_seconds() <= 2100,
        'residual prospective timing differs')
    parent = base.summarize_campaign(parent_root); parent_before = base.inventory(parent_root)
    require(parent['status'] == 'COMPLETE_DIAGNOSTIC_NOT_PROMOTED', 'complete independently authenticated parent cache study required')
    parent_report = decode(base.regular(parent_root / 'report.json').read_bytes())
    parent_plan = decode(base.regular(parent_root / 'plan.json').read_bytes())
    require(base.timestamp(parent_report['completed_at_utc']) <= base.timestamp(plan['declared_at_utc']), 'residual plan predates parent completion')
    for key, value in dict(parent_cache_plan_sha256=base.PLAN_SHA, parent_cache_report_sha256=base.REPORT_SHA,
        parent_cache_manifest_sha256=base.MANIFEST_SHA, parent_cache_source_sha256=base.EXTERNAL['external_worker.py']).items():
        require(plan[key] == value, 'parent cache plan/report/source binding differs')
    require(report['parent_cache_report_sha256'] == base.REPORT_SHA and report['parent_cache_manifest_sha256'] == base.MANIFEST_SHA,
        'worker parent report/manifest binding differs')
    parent_files = dict(parent['original_payload_sha256'], SHA256SUMS=base.MANIFEST_SHA)
    same(report['original_cache_run_file_sha256'], parent_files, 'all205 original cache-run file witnesses changed')
    same(report['original_parent_input_sha256'], parent_report['original_input_sha256'], 'original17 parent lineage inputs changed')
    sources = dict(parent_report['source_sha256'], **NEW_SOURCES)
    require(report['source_sha256'] == sources and plan['source_sha256'] == parent_plan['source_sha256']
        and plan['worker_source_sha256'] == NEW_SOURCES['external_residual_worker.py']
        and plan['residual_core_source_sha256'] == NEW_SOURCES['external_residual_core.py'], 'exact24 residual sources required')
    for name, value in sources.items(): require(digest(root / 'source' / base.safe_name(name)) == value, 'residual source archive changed')
    same(report['loss_projection'], plan['loss_projection'], 'residual native cost projection differs')
    same(plan['loss_projection'], parent_plan['loss_projection'], 'original loss definition changed')
    original = base.manifest(read('SHA256SUMS')); require(set(original) == expected_names(sources), 'exact194 original payloads required')
    missing = set(original) - set(before)
    require(missing == {n for n in original if n.endswith('.pt')} and len(missing) == 12
        and set(before) == set(original) - missing | {'SHA256SUMS', 'transport_verification.json'}, 'only12 residual PT may be omitted')
    for name in set(original) & set(before): require(before[name] == original[name], 'residual original SHA mismatch')
    transport = verify_transport(root, before, original, missing, decode(base.regular(parent_root / 'transport_verification.json').read_bytes()), parent_report)
    same(report['gpu_preflight'], parent_report['gpu_preflight'], 'exclusive GPU0 preflight changed')
    same(report['torch_device_uuid'], parent_report['torch_device_uuid'], 'GPU0 UUID changed')
    same(report['imported_native_modules'], parent_report['imported_native_modules'], 'native module path provenance changed')
    require([s['seed'] for s in report['seeds']] == list(base.SEEDS), 'three ordered residual parents required')
    previous_end = base.timestamp(report['started_at_utc']); core_metrics = []
    for state, seed in zip(report['seeds'], base.SEEDS):
        prefix = 'seed_' + seed; parent_state = next(s for s in parent_report['seeds'] if s['seed'] == seed)
        require(state['directory'] == prefix and state['status'] == 'TWO_RESIDUAL_HEADS_COMPLETE_NOT_PROMOTED'
            and state['parent_checkpoint_sha256'] == parent_state['parent_checkpoint_sha256']
            and state['cache_digest'] == parent_state['cache_digest'], 'original residual generator/cache binding differs')
        same(state['completed_files'], {n[len(prefix) + 1:]: v for n, v in original.items() if n.startswith(prefix + '/')}, 'completed residual seed file sealing differs')
        baselines = {}; variants = {}; seed_result = dict(seed=seed, parent_checkpoint_sha256=state['parent_checkpoint_sha256'], variants=variants, comparisons={})
        for split in base.COUNTS:
            verify_cache_proof(state['cache_proofs'][split], split, seed, parent_state, parent_files)
            measured = base.verify_analysis(root, prefix + '/original/' + split, split, state['cache_digest'][split], read, sources)
            rows = base.json_rows(base.regular(parent_root / prefix / 'original' / split / 'samples.jsonl').read_bytes(), base.COUNTS[split])
            verify_fixed_rows(rows, measured[1], original_logits=True); baselines[split] = measured
        variants['original'] = {s: value[0] for s, value in baselines.items()}
        require([a['arm'] for a in state['arms']] == list(ARMS), 'both ordered residual arms required')
        initial = order = prior_metrics = None
        for arm_state, arm in zip(state['arms'], ARMS):
            item = prefix + '/' + arm; head = decode(read(item + '/report.json'))
            require(arm_state['status'] == 'HEAD_COMPLETE_NOT_PROMOTED', 'complete residual head required')
            same(arm_state['files'], {n[len(item) + 1:]: v for n, v in original.items() if n.startswith(item + '/')}, 'residual head file sealing differs')
            ended = base.timestamp(arm_state['completed_at_utc']); require(previous_end <= ended <= base.timestamp(state['completed_at_utc']), 'residual head chronology differs'); previous_end = ended
            analyses = {s: base.verify_analysis(root, item + '/analysis/' + s, s, state['cache_digest'][s], read, sources) for s in base.COUNTS}
            for split in base.COUNTS: verify_fixed_rows(baselines[split][1], analyses[split][1])
            current_order = verify_head(head, base.json_rows(read(item + '/metrics.jsonl'), 1540), seed, arm, plan, state, baselines, analyses)
            require(initial is None or initial == head['initial_head_sha256'], 'matched residual initial states differ')
            require(order is None or order == current_order, 'matched residual sampling differs')
            require(prior_metrics is None or prior_metrics == head['baseline_metrics'], 'paired residual baseline metrics differ')
            initial, order, prior_metrics = head['initial_head_sha256'], current_order, head['baseline_metrics']
            variants[arm] = {s: value[0] for s, value in analyses.items()}
            seed_result['comparisons'][arm + '_minus_original'] = {s: base.comparison(variants['original'][s], variants[arm][s]) for s in base.COUNTS}
            core_metrics.append({k: head[k] for k in ('seed', 'arm', 'baseline_metrics', 'pre_training_metrics', 'post_training_metrics')})
        seed_result['comparisons']['candidate_minus_prior_only'] = {s: base.comparison(variants[ARMS[0]][s], variants[ARMS[1]][s]) for s in base.COUNTS}
        earlier = next(s for s in parent['seeds'] if s['seed'] == seed)
        seed_result['earlier_fresh_head_comparisons'] = {new + '_minus_' + old: {
            s: base.comparison(earlier['variants'][old][s], variants[new][s]) for s in base.COUNTS}
            for new in ARMS for old in base.ARMS}
        seed_result.update(initial_head_sha256=initial, sampling_order_sha256=order); result['seeds'].append(seed_result)
        previous_end = base.timestamp(state['completed_at_utc']); require(previous_end <= base.timestamp(report['completed_at_utc']), 'residual seed completes after worker')
    result.update(status='COMPLETE_DIAGNOSTIC_NOT_PROMOTED', worker_report_sha256=REPORT_SHA, source_commit=base.COMMIT,
        execution_source_sha256=sources, original_payload_sha256=original, transport_verification=transport,
        parent_input_sha256=parent_before, parent_report_sha256=base.REPORT_SHA, parent_manifest_sha256=base.MANIFEST_SHA,
        completed_head_fit_count=6, full_model_fit_count=0, generator_forward_count=0, original_baseline_count=3,
        total_analysis_rows=13356, head_history_rows=9240, head_sample_exposures=36930, fixed_val_png_count=108,
        secure_original_cache_load_count=6, unchanged_original_cache_run_file_count=205, original_parent_checkpoint_load_count=0,
        matched_allocated_head_parameters=115457, local_tensor_bytes_loaded=False, recorded_core_metrics=core_metrics,
        interpretation='All3 seeds and all TRAIN/VAL groups retained. Same allocated115457 structure, initialization, prior and sampling; geometry information differs and inactive zero-geometry connections mean effective functional capacity is not equal. Initial original behavior preservation is not a post-training guarantee. TRAIN is in-sample and VAL is a repeatedly used development episode, not an independent holdout. No winner, promotion, traffic-intent, driving or admission claim.',
        numerical_scope='Saved cost argmin/exact ties/regrets, original logit byte digests, group aggregates and per-candidate gate consistency verified. Full XY/speed and raw target-speed arrays are absent: no new runtime trajectory-gate replay or motion-group reclassification. Native raw-vx gates and future-derived groups are source-bound; GPU core metric reductions retained separately.')
    for item in result['performed_heads']: item['status'] = 'COMPLETE_DIAGNOSTIC_NOT_PROMOTED'
    recheck(root, parent_root, result)
    return result


def recheck(root, parent_root, result):
    require(base.inventory(root) == result['input_sha256'] and base.inventory(parent_root) == result['parent_input_sha256']
        and digest(Path(__file__)) == result['summary_source_sha256'] and digest(Path(base.__file__)) == BASE_SHA,
        'residual/parent input or summary source changed')
    for name, pin in result['execution_source_sha256'].items():
        if name not in base.EXTERNAL and name not in NEW_SOURCES: require(digest(ROOT / name) == pin, 'frozen numeric dependency changed')


def run(input_root, output_dir, parent_root=None):
    root, output = Path(input_root).absolute(), Path(output_dir).absolute()
    parent = Path(parent_root).absolute() if parent_root else root.parent / 'frozen_stopmix_selector_v1'
    require(not output.exists() and not any(p.is_symlink() for p in (output, *output.parents)), 'fresh residual summary output required')
    require(not any(output.resolve() == p or p in output.resolve().parents for p in (root.resolve(), parent.resolve(), (ROOT / 'datasets').resolve())), 'summary output must be outside all input/dataset trees')
    result = summarize_campaign(root, parent); output.mkdir(parents=True, exist_ok=False)
    try:
        with (output / 'summary.json').open('xb') as stream: stream.write(encoded(result))
        if result['status'] == 'COMPLETE_DIAGNOSTIC_NOT_PROMOTED': recheck(root, parent, result)
        else: require(base.inventory(root) == result['input_sha256'], 'partial residual input changed')
        with (output / 'SHA256SUMS').open('x') as stream: stream.write(digest(output / 'summary.json') + '  summary.json\n')
    except BaseException as error:
        with (output / 'failed.json').open('xb') as stream: stream.write(encoded(dict(status='SUMMARY_FAILED_NOT_PROMOTED', error_type=type(error).__name__, error=str(error))))
        raise
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('input_root', type=Path); parser.add_argument('output_dir', type=Path); parser.add_argument('--parent-root', type=Path)
    args = parser.parse_args(argv); result = run(args.input_root, args.output_dir, args.parent_root)
    print(json.dumps(dict(status=result['status'], summary_sha256=digest(args.output_dir / 'summary.json'))))
    return 0 if result['status'] == 'COMPLETE_DIAGNOSTIC_NOT_PROMOTED' else 2


if __name__ == '__main__': raise SystemExit(main())
