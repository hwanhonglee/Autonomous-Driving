#!/usr/bin/env python3
"""HH_260906 - Verify fixed-parent DRIVE-ranking saved outcomes without tensor loading, intermediate-logit replay or promotion."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import struct

from scripts.e2e import summarize_frozen_stopmix_residual as residual

base = residual.base
ROOT = Path(__file__).resolve().parents[2]
RESIDUAL_SHA = 'd9a877aa71bbb5529864f02af95dc76077268924f3757d9907f4609080ed1a01'
PLAN_SHA = 'f5c1eda4c1f39e869b07aba853f0c813890663fd11cc456a0a754f5d35c19ad7'
# HH_260906 - Resume on 2026-09-14 using root-observed hashes of the completed 2026-09-09 run.
REPORT_SHA = '569979f1a28ba964f4b28c3c5cd6cf1b614d940831369806f8fdd5ded8f9d15f'
MANIFEST_SHA = '80e57a2503e461861fc2af2b4ecf16f97b0196370b270989e1aaef6e4748f531'
TRANSPORT_SHA = '8a30af417d79bb77f706b4aa208f2763ba38fe46ceb57b0eb513fa0d330c2fc2'
NEW_SOURCES = {'external_rank_worker.py': '34a7f870526e43d35d209fcddc73ea9f2e1545cd5f3f3cd3d1f7e6798249ebc1',
    'external_rank_core.py': '206313ff9a0b50c00408e615946f0cf69cca93f3e2b939eddd78b7956ab96a5d'}
SUMMARY_REFERENCES = {'frozen_stopmix_residual_summary_v1/summary.json': '00f5c2429e23dc1aeec515f4e7f7986e728fcfc61b9370ff4d3ca17c5ef9b07a',
    'frozen_stopmix_selector_summary_v1/summary.json': '0e6085856d436f8e296da412875af5fc07e2886db3020cd8340baa72f45a9ec2'}
ARMS = ('ce_continue', 'drive_pair_continue')
BASELINE = 'parent_candidate_residual'
DENIALS = dict(residual.DENIALS)
require, same, sha, digest, decode, encoded = base.require, base.same, base.sha, base.digest, base.decode, base.encoded


def f32(value):
    require(base.finite(value), 'finite nonboolean number required')
    try: result = struct.unpack('<f', struct.pack('<f', value))[0]
    except (OverflowError, struct.error) as error: raise ValueError('float32 value overflow') from error
    require(math.isfinite(result), 'finite float32 required')
    return result


def verify_history(rows, head, seed, arm):
    order = base.verify_history(rows, head, seed); weight = ARMS.index(arm)
    fields = {'global_step', 'epoch', 'batch_samples', 'samples_seen', 'sample_indices', 'candidate_score_loss',
        'drive_pair_rank_loss', 'drive_pair_weight', 'optimization_loss', 'gradient_norm'}
    for row in rows:
        require(set(row) == fields, 'exact rank history scalar fields required')
        same(row['drive_pair_weight'], weight, 'predeclared ranking weight changed')
        for key in ('candidate_score_loss', 'drive_pair_rank_loss', 'optimization_loss', 'gradient_norm'):
            require(base.finite(row[key]) and row[key] >= 0 and f32(row[key]) == row[key], 'native finite nonnegative float32 history required')
        objective = f32(f32(.1) * row['candidate_score_loss'])
        if weight: objective = f32(objective + f32(f32(.1) * row['drive_pair_rank_loss']))
        same(row['optimization_loss'], objective, 'recorded scalar objective violates original float32 operation order')
    return order


def matrix_bytes(name, rows, field):
    values = [v for row in rows for v in row['loss_alignment'][field]]
    require(len(values) == len(rows) * 12 and all(base.finite(v) and f32(v) == v for v in values), 'all native float32 matrix values required')
    return json.dumps([name, 'torch.float32', [len(rows), 12]], separators=(',', ':')).encode() + b'\0' + struct.pack('<' + 'f' * len(values), *values)


def paired_row_measurement(row):
    # HH_260906 - These binary64 diagnostics use final saved logits only; they are not a replay of training-batch logits or CUDA reductions.
    costs, logits = row['loss_alignment']['candidate_composite_costs'], row['loss_alignment']['candidate_logits']
    require(len(costs) == len(logits) == 12 and all(base.finite(v) for v in costs + logits), 'all finite saved K12 costs/logits required')
    selected = row['selected_candidate_index']
    require(type(selected) is int and 0 <= selected < 12, 'selected index invalid')
    oracle = min(range(12), key=costs.__getitem__); minimum = min(costs); best_drive = min(costs[:6])
    best_scored_drive = max(range(6), key=logits.__getitem__)
    pairs = [(costs[i] - costs[j], logits[i] - logits[j]) for i in range(6) for j in range(i + 1, 6)]
    gaps = [abs(dc) for dc, _ in pairs]; total = math.fsum(gaps)
    require(math.isfinite(total) and all(math.isfinite(dc) and math.isfinite(dl) for dc, dl in pairs), 'pair arithmetic overflow')
    loss = 0.
    if total:
        for (dc, dl), gap in zip(pairs, gaps):
            signed = (1. if dc > 0 else -1. if dc < 0 else 0.) * dl
            loss += (gap / total) * (max(signed, 0.) + math.log1p(math.exp(-abs(signed))))
    return dict(selected_drive=selected < 6, hard_oracle_drive=oracle < 6,
        family_outside_exact_minimum_set=not any((i < 6) == (selected < 6) for i, c in enumerate(costs) if c == minimum),
        cross_family_minimum_tie=any(c == minimum for c in costs[:6]) and any(c == minimum for c in costs[6:]),
        selected_drive_is_best=selected < 6 and costs[selected] == best_drive,
        selected_drive_regret=costs[selected] - best_drive if selected < 6 else None,
        best_scored_drive_regret=costs[best_scored_drive] - best_drive,
        global_selected_regret=costs[selected] - minimum,
        cost_tied_pair_count=sum(dc == 0 for dc, _ in pairs),
        strict_pair_count=sum(dc != 0 for dc, _ in pairs),
        strict_logit_tied_pair_count=sum(dc != 0 and dl == 0 for dc, dl in pairs),
        discordant_strict_pair_count=sum(dc != 0 and (1 if dc > 0 else -1) * dl > 0 for dc, dl in pairs),
        normalized_pair_loss_float64=loss)


def drive_diagnostics(rows):
    measured = [paired_row_measurement(row) for row in rows]
    def group(indices):
        values = [measured[i] for i in indices]; n = len(values)
        chosen = [v for v in values if v['selected_drive']]
        drive_oracle = [v for v in values if v['hard_oracle_drive']]
        both = [v for v in drive_oracle if v['selected_drive']]
        confusion = {a: {b: 0 for b in ('DRIVE', 'STOP')} for a in ('DRIVE', 'STOP')}
        for v in values: confusion['DRIVE' if v['hard_oracle_drive'] else 'STOP']['DRIVE' if v['selected_drive'] else 'STOP'] += 1
        return dict(sample_count=n, selected_drive_count=len(chosen), selected_stop_count=n-len(chosen),
            hard_oracle_drive_count=len(drive_oracle), hard_oracle_stop_count=n-len(drive_oracle),
            hard_oracle_family_to_selected_family=confusion,
            selected_family_outside_exact_minimum_set_count=sum(v['family_outside_exact_minimum_set'] for v in values),
            cross_family_exact_minimum_tie_count=sum(v['cross_family_minimum_tie'] for v in values),
            selected_drive_best_cost_agreement_count=sum(v['selected_drive_is_best'] for v in chosen),
            selected_drive_error_count=sum(not v['selected_drive_is_best'] for v in chosen),
            hard_oracle_drive_and_selected_drive_count=len(both),
            within_drive_error_when_hard_oracle_drive_and_selected_drive_count=sum(not v['selected_drive_is_best'] for v in both),
            selected_drive_regret_mean=sum(v['selected_drive_regret'] for v in chosen)/len(chosen) if chosen else None,
            best_scored_drive_regret_mean_all_rows=sum(v['best_scored_drive_regret'] for v in values)/n if n else None,
            global_selected_cost_regret_mean=sum(v['global_selected_regret'] for v in values)/n if n else None,
            all_drive_pair_count=15*n, cost_tied_pair_count=sum(v['cost_tied_pair_count'] for v in values),
            strict_pair_count=sum(v['strict_pair_count'] for v in values),
            strict_logit_tied_pair_count=sum(v['strict_logit_tied_pair_count'] for v in values),
            discordant_strict_pair_count=sum(v['discordant_strict_pair_count'] for v in values),
            final_saved_normalized_pair_loss_float64_mean=sum(v['normalized_pair_loss_float64'] for v in values)/n if n else None)
    result = dict(all_samples=group(range(len(rows))), target_motion_groups={g: group([i for i, r in enumerate(rows)
        if r['target_motion_group'] == g]) for g in base.GROUPS})
    for field in ('episode_id', 'target_motion_group'):
        result[field] = {str(k): group([i for i, row in enumerate(rows) if row[field] == k]) for k in sorted({r[field] for r in rows})}
    return result


def expected_names(sources):
    names = {'plan.json', 'report.json'} | {'source/' + base.safe_name(n) for n in sources}
    for seed in base.SEEDS:
        prefix = f'seed_{seed}/'
        for view in (BASELINE, *(a + '/analysis' for a in ARMS)):
            names |= {prefix + view + '/' + s + '/' + n for s in base.COUNTS for n in ('samples.jsonl', 'summary.json')}
            names |= {prefix + view + '/val/' + f'val_{i:03d}.png' for i in base.INDICES}
        names |= {prefix + arm + '/' + n for arm in ARMS for n in ('metrics.jsonl', 'report.json', 'head_only.pt', 'final_logits.pt')}
    return names


def outcome_ledger(before, report):
    result = []
    for seed in base.SEEDS:
        for arm in ARMS:
            states = [a for s in report.get('seeds', []) if s.get('seed') == seed for a in s.get('arms', []) if a.get('arm') == arm]
            require(len(states) <= 1, 'duplicate rank head state')
            result.append(dict(seed=seed, arm=arm, recorded_worker_status=states[0].get('status') if states else None,
                status='INCOMPLETE' if states or any(n.startswith(f'seed_{seed}/{arm}/') for n in before) else 'NOT_RUN'))
    return result


def verify_parent_proof(proof, parent_head, parent_files, seed, baselines):
    prefix = f'seed_{seed}/candidate_residual/'
    expected = dict(files={n: parent_files[prefix+n] for n in ('head_only.pt', 'final_logits.pt')},
        head_state_sha256=parent_head['final_head_sha256'], logit_sha256={s: residual.logits_digest(v[1]) for s, v in baselines.items()},
        weights_only=True, map_location='cpu', optimizer_loaded=False)
    same(proof, expected, 'secure parent head/logit proof differs')


def verify_head(head, history, seed, arm, plan, state, parent_head, baselines, analyses):
    weight = ARMS.index(arm)
    expected = dict(artifact_id='portable_e2e.frozen_stopmix_drive_rank_research_head.v1', status='RESEARCH_HEAD_TRAINING_COMPLETE_NOT_PROMOTED',
        arm=arm, seed=int(seed), device='cuda:0', optimizer='AdamW', optimizer_resumed=False, steps=1540, batch_size=4,
        learning_rate=.0001, weight_decay=.0001, candidate_score_weight=.1, maximum_gradient_norm=5., loss_config=plan['loss_config'],
        source_checkpoint_sha256=plan['parent_checkpoint_sha256'][seed], corpus_fingerprint_sha256=plan['corpus_fingerprint_sha256'],
        split_fingerprints={s:v['fingerprint_sha256'] for s,v in plan['split_contract'].items()}, head_parameter_count=115457,
        candidate_count=12, final_state=dict(global_step=1540, samples_seen=6155, last_epoch_index=5),
        initialization='Every arm copies the same fixed parent B residual head; fresh Adam state, not optimizer-state resume or fresh random head.')
    for key, value in expected.items(): same(head[key], value, 'rank head configuration differs: '+key)
    same(head['selector_objective'], dict(drive_pair_weight=weight, auxiliary_coefficient=.1*weight,
        drive_candidate_indices=list(range(6)), unordered_pairs_per_sample=15,
        formula='0.1*hard_composite_CE + 0.1*lambda*mean_rows(sum_pairs(abs(delta_cost)/sum15_abs_gaps * softplus(sign(delta_cost)*delta_logit)))',
        exact_ties='zero weight; all-tied DRIVE row contributes zero; no epsilon, temperature, group filter or sample deletion'), 'rank objective changed')
    require(all(head[k] is False for k in ('generator_parameters_trained', 'original_heads_recomputed_or_trained',
        'training_data_approved', 'vehicle_control_approved', 'automatic_promotion', 'runtime_bundle_supported')), 'rank research denial changed')
    same(head['projection_proof'], plan['loss_projection'], 'rank projection proof changed')
    require(head['projection_proof_sha256'] == sha(encoded(plan['loss_projection']).rstrip(b'\n')),
        'projection proof digest differs')
    require(head['cache_sha256_before'] == head['cache_sha256_after'] == state['cache_digest']
        and head['initial_head_sha256'] == head['parent_head_sha256_before'] == head['parent_head_sha256_after']
        == parent_head['final_head_sha256'] == state['parent_head_proof']['head_state_sha256']
        and base.hexsha(head['final_head_sha256']), 'fixed parent state/cache differs')
    all_logits = sha(b''.join(matrix_bytes(s, baselines[s][1], 'candidate_logits') for s in base.COUNTS))
    require(head['parent_logits_sha256_before'] == head['parent_logits_sha256_after'] == all_logits, 'parent whole logit bytes differ')
    order = verify_history(history, head, seed, arm)
    for split in base.COUNTS:
        original, original_rows = baselines[split]; measured, rows = analyses[split]
        same(head[split+'_sample_ids'], [r['sample_id'] for r in rows], 'rank full split sample order differs')
        require(head['composite_oracle_sha256'][split] == base.oracle_digest(measured['oracle_indices'])
            and head['projected_cost_sha256'][split] == sha(matrix_bytes('projected_costs', rows, 'candidate_composite_costs')),
            'all saved projected costs or oracle digest differ')
        parity = head['parent_logit_parity'][split]; pin = residual.logits_digest(original_rows)
        require(parity['byte_equal'] is True and parity['parent_logits_sha256'] == parity['initial_logits_sha256'] == pin
            and base.finite(parity['max_abs_logit_difference']) and parity['max_abs_logit_difference'] == 0
            and type(parity['selected_index_mismatch_count']) is int and parity['selected_index_mismatch_count'] == 0,
            'initial parent saved-logit byte parity failed')
        for name, view in (('baseline_metrics', original), ('post_training_metrics', measured)):
            metric = head[name][split]; base.verify_core_metrics(metric, split); costs = view['composite_cost_alignment']['all_samples']
            same(metric['selected_histogram'], [costs['selected_histogram'][str(i)] for i in range(12)], 'rank selection histogram differs')
            same(metric['composite_oracle_histogram'], [costs['composite_oracle_histogram'][str(i)] for i in range(12)], 'rank oracle histogram differs')
            same(metric['selected_composite_oracle_agreement_count'], costs['selected_composite_oracle_index_agreement_count'], 'rank oracle agreement differs')
        same(head['pre_training_metrics'][split], head['baseline_metrics'][split], 'rank initial metrics changed')
        same(head['baseline_metrics'][split], parent_head['post_training_metrics'][split], 'fixed parent float32 core metrics changed')
    return order


def verify_transport(root, before, original, missing, residual_transport, original_transport, residual_report):
    require(before.get('transport_verification.json') == TRANSPORT_SHA, 'exact actual rank transport required')
    value = decode(base.regular(root/'transport_verification.json').read_bytes())
    require(value['status'] == 'VERIFIED_SIX_DRIVE_RANK_HEAD_FITS_NOT_PROMOTED' and value['source_commit'] == base.COMMIT
        and value['plan_sha256'] == PLAN_SHA and value['report_sha256'] == REPORT_SHA and value['original_manifest_sha256'] == MANIFEST_SHA,
        'rank transport source/report binding differs')
    expected = dict(original, SHA256SUMS=MANIFEST_SHA); remote = value['regular_file_inventory']
    require(set(remote) == set(expected) and set(value['omitted_files']) == missing, 'rank transport complete file inventory differs')
    for key in ('remote_pre_post_bytes_unchanged', 'source_and_parent_bytes_unchanged', 'cooperative_gpu0_lease_available_and_held',
        'local_launch_evidence_unchanged', 'first_parent_rejected_launcher_preserved'): require(value[key] is True, 'rank transport postcheck failed')
    for key in ('collector_tensor_loading', 'tensor_artifacts_transferred', 'model_promotion', 'data_admission'):
        require(value[key] is False, 'rank collection scope differs')
    for key, count in dict(remote_file_count=197, collected_file_count=185, analysis_sample_rows=13356,
        head_history_rows=9240, head_only_fit_count=6, full_model_fit_count=0, generator_neural_forwards=0,
        fixed_val_png_count=108, original_cache_files_loaded_by_worker=6, parent_head_files_loaded_by_worker=3,
        parent_logit_files_loaded_by_worker=3).items(): same(value[key], count, 'rank transport count differs: '+key)
    prior = value['original_input_verification']
    same(prior['original_cache_run_inventory'], original_transport['regular_file_inventory'], 'all205 cache sources changed')
    same(prior['original_residual_run_inventory'], residual_transport['regular_file_inventory'], 'all195 residual parent sources changed')
    same(prior['continued_parent_input_sha256'], residual_report['original_parent_input_sha256'], '17 original lineage sources changed')
    for name, pin in expected.items():
        item = remote[name]; require(item['sha256'] == pin and type(item['size_bytes']) is int and item['size_bytes'] >= 0, 'rank remote byte witness differs')
        if name in missing:
            omitted = value['omitted_files'][name]
            require(omitted['sha256'] == pin and omitted['size_bytes'] == item['size_bytes'] and omitted['reason'], 'rank omitted tensor witness differs')
        else: require(before[name] == pin and base.regular(root/name).stat().st_size == item['size_bytes'], 'rank local mirrored bytes differ')
    require(base.timestamp(value['collected_at_utc']) >= base.timestamp(decode(base.regular(root/'report.json').read_bytes())['completed_at_utc']), 'rank collection predates completion')
    return dict(receipt_sha256=TRANSPORT_SHA, remote_tensor_bytes_verified_by_collector=True, local_tensor_bytes_loaded=False,
        omitted_remote_tensor_count=12, original_residual_tensor_count=12, original_cache_run_tensor_count=18)


def summarize_campaign(input_root, parent_root=None, original_root=None):
    require(digest(Path(residual.__file__)) == RESIDUAL_SHA and digest(Path(base.__file__)) == residual.BASE_SHA, 'reviewed parent summary dependency changed')
    root = Path(input_root).absolute(); parent = Path(parent_root).absolute() if parent_root else root.parent/'frozen_stopmix_residual_v1'
    original_root = Path(original_root).absolute() if original_root else root.parent/'frozen_stopmix_selector_v1'
    before = base.inventory(root); own = digest(Path(__file__)); read = lambda name: base.regular(root/base.safe_name(name)).read_bytes()
    require(before.get('plan.json') == PLAN_SHA, 'exact prospective rank plan required')
    plan = decode(read('plan.json')); report = decode(read('report.json')) if 'report.json' in before else {}
    result = dict(schema='portable_e2e.frozen_stopmix_drive_rank_summary.v1', status='INCOMPLETE_NOT_PROMOTED',
        plan_sha256=PLAN_SHA, summary_source_sha256=own, residual_summary_source_sha256=RESIDUAL_SHA,
        base_summary_source_sha256=residual.BASE_SHA, performed_heads=outcome_ledger(before, report), input_sha256=before, seeds=[], **DENIALS)
    if not report or report.get('status') == 'FAILED_OR_PARTIAL_NOT_PROMOTED':
        result['retained_worker_report'] = report; require(base.inventory(root) == before, 'partial rank evidence changed'); return result
    require(all(base.hexsha(v) for v in (REPORT_SHA, MANIFEST_SHA, TRANSPORT_SHA)), 'actual pins not released')
    require(report['schema'] == 'portable_e2e.external_frozen_stopmix_drive_rank.v1' and report['status'] == 'SIX_DRIVE_RANK_HEAD_FITS_COMPLETE_NOT_PROMOTED'
        and before['report.json'] == REPORT_SHA and before.get('SHA256SUMS') == MANIFEST_SHA and report['plan_sha256'] == PLAN_SHA
        and plan['source_commit'] == base.COMMIT and report['baseline_directory'] == plan['baseline_directory'] == BASELINE, 'rank completion binding differs')
    for key, value in DENIALS.items(): same(report[key], value, 'rank worker scope differs'); same(plan[key], value, 'rank plan scope differs')
    same(report['completed_head_fit_count'], 6, 'six complete rank heads required')
    require(report['postcheck_errors'] == [] and report['source_and_inputs_unchanged'] is True and report['deadline_met'] is True, 'rank completion checks failed')
    started, ended = base.timestamp(report['started_at_utc']), base.timestamp(report['completed_at_utc'])
    require(base.timestamp(plan['declared_at_utc']) <= started <= ended <= base.timestamp(plan['finish_before_utc'])
        and (ended-started).total_seconds() <= 2400, 'rank prospective timing differs')
    prior = residual.summarize_campaign(parent, original_root); parent_before, original_before = base.inventory(parent), base.inventory(original_root)
    require(prior['status'] == 'COMPLETE_DIAGNOSTIC_NOT_PROMOTED', 'independently complete residual parents required')
    references = {}
    for name, pin in SUMMARY_REFERENCES.items():
        path = root.parent/name; require(digest(path) == pin, 'frozen prior independent summary changed'); references[name] = pin
    same(decode(base.regular(root.parent/'frozen_stopmix_residual_summary_v1/summary.json').read_bytes()), prior, 'original residual independent summary no longer reproduces')
    parent_report = decode(base.regular(parent/'report.json').read_bytes()); parent_plan = decode(base.regular(parent/'plan.json').read_bytes())
    original_report = decode(base.regular(original_root/'report.json').read_bytes())
    require(base.timestamp(parent_report['completed_at_utc']) <= base.timestamp(plan['declared_at_utc']), 'rank plan predates completed parent')
    for key, value in dict(parent_residual_plan_sha256=residual.PLAN_SHA, parent_residual_report_sha256=residual.REPORT_SHA,
        parent_residual_manifest_sha256=residual.MANIFEST_SHA, parent_cache_plan_sha256=base.PLAN_SHA,
        parent_cache_report_sha256=base.REPORT_SHA, parent_cache_manifest_sha256=base.MANIFEST_SHA).items(): require(plan[key] == value, 'rank lineage plan binding differs')
    require(report['parent_residual_report_sha256'] == residual.REPORT_SHA and report['parent_residual_manifest_sha256'] == residual.MANIFEST_SHA, 'rank parent report binding differs')
    parent_files = dict(prior['original_payload_sha256'], SHA256SUMS=residual.MANIFEST_SHA)
    same(report['original_residual_run_file_sha256'], parent_files, 'all195 residual parent byte pins differ')
    same(report['original_cache_run_file_sha256'], parent_report['original_cache_run_file_sha256'], 'all205 original cache byte pins differ')
    same(report['original_parent_input_sha256'], parent_report['original_parent_input_sha256'], 'all17 continued inputs differ')
    sources = dict(parent_report['source_sha256'], **NEW_SOURCES)
    require(len(sources) == 26 and report['source_sha256'] == sources and plan['source_sha256'] == parent_plan['source_sha256']
        and plan['worker_source_sha256'] == NEW_SOURCES['external_rank_worker.py'] and plan['rank_core_source_sha256'] == NEW_SOURCES['external_rank_core.py'], 'all26 exact rank sources required')
    for name, pin in sources.items(): require(digest(root/'source'/base.safe_name(name)) == pin, 'rank archived source changed')
    same(report['loss_projection'], plan['loss_projection'], 'rank original cost projection changed')
    same(plan['loss_projection'], parent_plan['loss_projection'], 'parent native cost definition changed')
    payload = base.manifest(read('SHA256SUMS')); require(set(payload) == expected_names(sources), 'exact196 rank payloads required')
    missing = set(payload)-set(before)
    require(missing == {n for n in payload if n.endswith('.pt')} and len(missing) == 12
        and set(before) == set(payload)-missing | {'SHA256SUMS', 'transport_verification.json'}, 'only12 new rank tensors may be omitted')
    for name in set(payload)&set(before): require(before[name] == payload[name], 'rank original payload changed')
    transport = verify_transport(root, before, payload, missing, decode(base.regular(parent/'transport_verification.json').read_bytes()),
        decode(base.regular(original_root/'transport_verification.json').read_bytes()), parent_report)
    for key in ('gpu_preflight', 'torch_device_uuid', 'imported_native_modules'): same(report[key], parent_report[key], 'rank execution hardware/native import proof differs')
    require([s['seed'] for s in report['seeds']] == list(base.SEEDS), 'three ordered rank parents required')
    previous_end = started; core_metrics = []
    for state, seed in zip(report['seeds'], base.SEEDS):
        prefix = 'seed_'+seed; old_state = next(s for s in parent_report['seeds'] if s['seed'] == seed)
        original_state = next(s for s in original_report['seeds'] if s['seed'] == seed)
        require(state['directory'] == prefix and state['status'] == 'TWO_DRIVE_RANK_HEADS_COMPLETE_NOT_PROMOTED'
            and state['parent_checkpoint_sha256'] == old_state['parent_checkpoint_sha256'] and state['cache_digest'] == old_state['cache_digest'], 'rank parent/cache binding differs')
        same(state['completed_files'], {n[len(prefix)+1:]:v for n,v in payload.items() if n.startswith(prefix+'/')}, 'rank seed seal differs')
        parent_head = decode(base.regular(parent/prefix/'candidate_residual/report.json').read_bytes())
        baselines = {}; variants = {}; rank_views = {}
        for split in base.COUNTS:
            residual.verify_cache_proof(state['cache_proofs'][split], split, seed, original_state, report['original_cache_run_file_sha256'])
            measured = base.verify_analysis(root, prefix+'/'+BASELINE+'/'+split, split, state['cache_digest'][split], read, sources)
            old_rows = base.json_rows(base.regular(parent/prefix/'candidate_residual/analysis'/split/'samples.jsonl').read_bytes(), base.COUNTS[split])
            residual.verify_fixed_rows(old_rows, measured[1], original_logits=True); baselines[split] = measured
        verify_parent_proof(state['parent_head_proof'], parent_head, parent_files, seed, baselines)
        variants[BASELINE] = {s:v[0] for s,v in baselines.items()}; rank_views[BASELINE] = {s:drive_diagnostics(v[1]) for s,v in baselines.items()}
        require([a['arm'] for a in state['arms']] == list(ARMS), 'ordered rank arms required')
        order = None; metrics = None; seed_result = dict(seed=seed, parent_checkpoint_sha256=state['parent_checkpoint_sha256'],
            parent_residual_head_sha256=state['parent_head_proof']['head_state_sha256'], variants=variants, drive_ranking_diagnostics=rank_views, comparisons={})
        for arm_state, arm in zip(state['arms'], ARMS):
            item = prefix+'/'+arm; head = decode(read(item+'/report.json'))
            require(arm_state['status'] == 'HEAD_COMPLETE_NOT_PROMOTED', 'complete rank head required')
            same(arm_state['files'], {n[len(item)+1:]:v for n,v in payload.items() if n.startswith(item+'/')}, 'rank head seal differs')
            head_end = base.timestamp(arm_state['completed_at_utc']); require(previous_end <= head_end <= base.timestamp(state['completed_at_utc']), 'rank head chronology differs'); previous_end = head_end
            analyses = {s:base.verify_analysis(root, item+'/analysis/'+s, s, state['cache_digest'][s], read, sources) for s in base.COUNTS}
            for split in base.COUNTS: residual.verify_fixed_rows(baselines[split][1], analyses[split][1])
            current = verify_head(head, base.json_rows(read(item+'/metrics.jsonl'), 1540), seed, arm, plan, state, parent_head, baselines, analyses)
            require(order is None or order == current, 'rank paired full sampling differs')
            require(metrics is None or metrics == head['baseline_metrics'], 'rank paired parent metrics differ')
            order, metrics = current, head['baseline_metrics']; variants[arm] = {s:v[0] for s,v in analyses.items()}
            rank_views[arm] = {s:drive_diagnostics(v[1]) for s,v in analyses.items()}
            seed_result['comparisons'][arm+'_minus_parent'] = {s:base.comparison(variants[BASELINE][s], variants[arm][s]) for s in base.COUNTS}
            core_metrics.append({k:head[k] for k in ('seed', 'arm', 'baseline_metrics', 'pre_training_metrics', 'post_training_metrics')})
        seed_result['comparisons']['drive_pair_minus_ce'] = {s:base.comparison(variants[ARMS[0]][s], variants[ARMS[1]][s]) for s in base.COUNTS}
        earlier = next(s for s in prior['seeds'] if s['seed'] == seed)
        seed_result['earlier_comparisons'] = {new+'_minus_'+old:{s:base.comparison(earlier['variants'][old][s], variants[new][s]) for s in base.COUNTS}
            for new in ARMS for old in ('original', 'prior_only_residual')}
        seed_result['sampling_order_sha256'] = order; result['seeds'].append(seed_result)
        previous_end = base.timestamp(state['completed_at_utc']); require(previous_end <= ended, 'rank seed ended after worker')
    result.update(status='COMPLETE_DIAGNOSTIC_NOT_PROMOTED', source_commit=base.COMMIT, worker_report_sha256=REPORT_SHA,
        execution_source_sha256=sources, original_payload_sha256=payload, transport_verification=transport,
        parent_input_sha256=parent_before, original_cache_input_sha256=original_before, frozen_prior_summary_sha256=references,
        completed_head_fit_count=6, full_model_fit_count=0, generator_forward_count=0, baseline_count=3,
        total_analysis_rows=13356, head_history_rows=9240, head_sample_exposures=36930, fixed_val_png_count=108,
        secure_original_cache_load_count=6, secure_parent_head_load_count=3, secure_parent_logit_load_count=3,
        unchanged_original_cache_run_file_count=205, unchanged_original_residual_run_file_count=195,
        matched_allocated_head_parameters=115457, local_tensor_bytes_loaded=False, recorded_core_metrics=core_metrics,
        numerical_scope='All1540 scalar history rows per head checked for native float32 coefficient/multiply/add order. No batch logits were saved: this does not replay intermediate CE, rank loss or gradients. Final saved K12 costs/logits independently give all15 DRIVE pair diagnostics in binary64, not bitwise CUDA reduction parity. Full XY/speed absent: no new trajectory-gate replay or target-group reclassification. Raw-current gates remain source-bound.',
        interpretation='All outcomes retained. Both heads begin at the same completed geometry-residual parent parameters/logits with fresh optimizers; only pair-loss weight differs. Shared parameters may alter STOP scores although direct auxiliary logits are DRIVE-only. No stationary guarantee, winner or promotion. TRAIN in-sample and reused single-episode development VAL are not independent holdouts; no traffic-intent or admission claim.')
    for item in result['performed_heads']: item['status'] = 'COMPLETE_DIAGNOSTIC_NOT_PROMOTED'
    recheck(root, parent, original_root, result); return result


def recheck(root, parent, original, result):
    require(base.inventory(root) == result['input_sha256'] and base.inventory(parent) == result['parent_input_sha256']
        and base.inventory(original) == result['original_cache_input_sha256'] and digest(Path(__file__)) == result['summary_source_sha256']
        and digest(Path(residual.__file__)) == RESIDUAL_SHA and digest(Path(base.__file__)) == residual.BASE_SHA, 'rank inputs or summary dependencies changed')
    for name, pin in result['frozen_prior_summary_sha256'].items(): require(digest(root.parent/name) == pin, 'frozen prior summary changed')
    for name, pin in result['execution_source_sha256'].items():
        if name not in base.EXTERNAL and name not in residual.NEW_SOURCES and name not in NEW_SOURCES:
            require(digest(ROOT/name) == pin, 'unchanged native dependency changed')


def run(input_root, output_dir, parent_root=None, original_root=None):
    root, output = Path(input_root).absolute(), Path(output_dir).absolute()
    parent = Path(parent_root).absolute() if parent_root else root.parent/'frozen_stopmix_residual_v1'
    original = Path(original_root).absolute() if original_root else root.parent/'frozen_stopmix_selector_v1'
    require(not output.exists() and not any(p.is_symlink() for p in (output, *output.parents)), 'fresh rank summary output required')
    require(not any(output.resolve().is_relative_to(p.resolve()) for p in (root, parent, original, ROOT/'datasets')), 'output overlaps input/dataset')
    result = summarize_campaign(root, parent, original); output.mkdir(parents=True, exist_ok=False)
    try:
        with (output/'summary.json').open('xb') as stream: stream.write(encoded(result))
        if result['status'] == 'COMPLETE_DIAGNOSTIC_NOT_PROMOTED': recheck(root, parent, original, result)
        else: require(base.inventory(root) == result['input_sha256'], 'partial rank input changed')
        with (output/'SHA256SUMS').open('x') as stream: stream.write(digest(output/'summary.json')+'  summary.json\n')
    except BaseException as error:
        with (output/'failed.json').open('xb') as stream: stream.write(encoded(dict(status='SUMMARY_FAILED_NOT_PROMOTED', error_type=type(error).__name__, error=str(error))))
        raise
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('input_root', type=Path); parser.add_argument('output_dir', type=Path)
    parser.add_argument('--parent-root', type=Path); parser.add_argument('--original-root', type=Path)
    args = parser.parse_args(argv); result = run(args.input_root, args.output_dir, args.parent_root, args.original_root)
    print(json.dumps(dict(status=result['status'], summary_sha256=digest(args.output_dir/'summary.json'))))
    return 0 if result['status'] == 'COMPLETE_DIAGNOSTIC_NOT_PROMOTED' else 2


if __name__ == '__main__': raise SystemExit(main())
