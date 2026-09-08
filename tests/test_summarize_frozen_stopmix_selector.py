"""HH_260906 - Challenge saved-row scorer summaries with CPU-only synthetic evidence; never execute a fit or read datasets."""
from copy import deepcopy
import json
from pathlib import Path
import struct

import pytest

from scripts.e2e import summarize_frozen_stopmix_selector as summary


def cost(selected=6):
    values = [float(i + 1) for i in range(6)] + [0.] * 6
    logits = [0.] * 12; logits[selected] = 1.
    ade = [float(i + 1) for i in range(6)] + [0.] * 6
    return dict(candidate_composite_costs=values, candidate_logits=logits, candidate_ade_m=ade,
        candidate_probabilities=[1 / 12] * 12, composite_oracle_index=6, exact_minimum_indices=list(range(6, 12)),
        exact_minimum_count=6, minimum_composite_cost=0., second_minus_first_composite_cost=0.,
        selected_candidate_index=selected, selected_in_exact_minimum=selected >= 6, selected_composite_regret=values[selected],
        ade_oracle_index=6, selected_ade_m=ade[selected], composite_oracle_ade_m=0., oracle_ade_m=0.,
        candidate_cost_dtype='torch.float32', native_oracle_and_regression_exact=True,
        original_target_valid=[True] * 64, raw_current_vx_mps=0., target_motion_group='stationary_hold',
        episode_id='synthetic_val', capture_phase='not_reclassified_by_this_experiment')


def row(index=0, selected=6):
    candidates = [dict(candidate_index=i, family='DRIVE' if i < 6 else 'STOP', ade_m=float(i + 1) if i < 6 else 0.,
        fde_m=2. if i < 6 else 0., speed_mae_mps=1. if i < 6 else 0., terminal_speed_mps=1. if i < 6 else 0.,
        terminal_exact_zero=i >= 6, terminal_at_or_below_0p1_mps=i >= 6,
        first_exact_zero_future_index=0 if i >= 6 else None,
        reacceleration_after_exact_future_zero=False, nonincreasing_speed_exact=True) for i in range(12)]
    gates = [dict(candidate_index=i, geometry_pass=i >= 6, failure_codes=[] if i >= 6 else ['curvature']) for i in range(12)]
    chosen = candidates[selected]
    return dict(index=index, sample_id=f'synthetic_{index}', episode_id='synthetic_val', sequence_index=index,
        anchor_timestamp_ns=1000000000 + index * 100000000, camera_sha256=['a' * 64] * 6,
        source_manifest_sha256='b' * 64, model_input_sha256='c' * 64, target_sha256='d' * 64,
        raw_current_vx_mps=0., candidate_count=12, valid_future_points=64, target_motion_group='stationary_hold',
        selected_candidate_index=selected, selected_stop_candidate=selected >= 6, selected_ade_m=chosen['ade_m'],
        selected_fde_m=chosen['fde_m'], selected_speed_mae_mps=chosen['speed_mae_mps'], oracle_ade_m=0.,
        ade_selection_regret_m=chosen['ade_m'], selected_speed_behavior={k: chosen[k] for k in summary.SPEED_FIELDS},
        candidates=candidates, loss_alignment=cost(selected), runtime_geometry=dict(candidates=gates,
            selected_candidate_index=selected, selected_geometry_pass=selected >= 6,
            selected_failure_codes=[] if selected >= 6 else ['curvature'],
            all_candidates_geometry_pass=False, any_candidate_geometry_pass=True))


def test_all_tied_stop_candidates_and_a_real_hold_regression_are_retained():
    from scripts.e2e.audit_portable_stopmix_behavior import summarize_rows
    original, regressed = row(), row(selected=0)
    summary.check_row(original, 0); summary.check_row(regressed, 0)
    a, b = summarize_rows([original], 12), summarize_rows([regressed], 12)
    result = summary.comparison(a, b)
    assert result['target_motion_groups']['stationary_hold']['metric_delta']['selected_ade_m'] == 1.
    assert result['all_samples']['stop_selection_count_delta'] == -1
    assert result['all_samples']['selected_geometry_pass_count_delta'] == -1
    assert set(result['target_motion_groups']) == set(summary.GROUPS)
    assert result['target_motion_groups']['moving_to_stop'] == dict(sample_count=0, metric_delta=None)
    assert 'pass' not in result and 'winner' not in result


@pytest.mark.parametrize('field,value', [('composite_oracle_index', 7), ('exact_minimum_indices', [6]),
    ('exact_minimum_count', True), ('selected_in_exact_minimum', 1), ('selected_composite_regret', .01),
    ('native_oracle_and_regression_exact', False), ('candidate_cost_dtype', 'torch.float64'),
    ('original_target_valid', [True] * 63 + [False]), ('original_target_valid', [1] * 64),
    ('candidate_logits', [float('inf')] * 12), ('candidate_logits', [False] * 12),
    ('candidate_probabilities', [1.] * 12), ('candidate_composite_costs', [0.] * 11)])
def test_cost_mutations_fail_closed(field, value):
    data = cost(); data[field] = value
    with pytest.raises(ValueError): summary.check_cost(data)


@pytest.mark.parametrize('path,value', [
    (('index',), False), (('candidate_count',), True), (('valid_future_points',), 63),
    (('raw_current_vx_mps',), float('nan')), (('camera_sha256',), ['a' * 64] * 5),
    (('selected_stop_candidate',), False), (('selected_ade_m',), 1.),
    (('runtime_geometry', 'selected_geometry_pass'), False),
    (('runtime_geometry', 'candidates', 0, 'geometry_pass'), True),
    (('runtime_geometry', 'candidates', 0, 'failure_codes'), ['invented']),
    (('runtime_geometry', 'candidates', 0, 'failure_codes'), ['curvature', 'curvature']),
    (('candidates', 6, 'first_exact_zero_future_index'), None),
    (('candidates', 6, 'terminal_exact_zero'), 1),
    (('loss_alignment', 'capture_phase'), 'warmup'),
    (('loss_alignment', 'raw_current_vx_mps'), .1)])
def test_row_gate_identity_and_behavior_mutations_fail_closed(path, value):
    data = row(); current = data
    for key in path[:-1]: current = current[key]
    current[path[-1]] = value
    with pytest.raises(ValueError): summary.check_row(data, 0)


def test_exact_tie_no_epsilon_and_lowest_logit_index_are_explicit():
    data = cost(selected=6); data['candidate_composite_costs'][7] = 1e-30
    with pytest.raises(ValueError): summary.check_cost(data)
    data = cost(); data['candidate_logits'] = [1.] * 12
    with pytest.raises(ValueError): summary.check_cost(data)


def test_every_cost_group_and_empty_group_is_recomputed():
    a, b = cost(), cost(0); b['episode_id'] = 'second'; b['raw_current_vx_mps'] = .2
    groups = summary.cost_groups([a, b])
    assert groups['all_samples']['sample_count'] == 2
    assert groups['all_samples']['selected_in_exact_minimum_count'] == 1
    assert groups['all_samples']['multiple_exact_minimum_count'] == 2
    assert groups['all_samples']['means']['selected_composite_regret'] == .5
    assert groups['raw_nonpositive']['sample_count'] == 1
    assert groups['target_motion_group']['moving_to_stop']['means']['oracle_ade_m'] is None
    assert set(groups['episode_id']) == {'synthetic_val', 'second'}


@pytest.mark.parametrize('raw', [b'{"a":1,"a":2}', b'{"a":NaN}', b'{"a":1e999}', b'{"a":Infinity}'])
def test_json_duplicate_and_nonfinite_rejected(raw):
    with pytest.raises(ValueError): summary.decode(raw)


@pytest.mark.parametrize('name', ['/tmp/x', '../x', 'a/../x', 'a\\x', 'a//x', './a', 'a\nx', ''])
def test_manifest_paths_rejected(name):
    with pytest.raises(ValueError): summary.safe_name(name)


def test_manifest_requires_exact_nonduplicate_payloads_and_no_self_hash():
    good = ('a' * 64 + '  report.json\n').encode()
    assert summary.manifest(good) == {'report.json': 'a' * 64}
    for raw in (good + good, good.rstrip(), ('a' * 64 + '  SHA256SUMS\n').encode()):
        with pytest.raises(ValueError): summary.manifest(raw)
    with pytest.raises(ValueError): summary.json_rows(b'{"index":0}', 1)
    with pytest.raises(ValueError): summary.json_rows(b'{}\n', 2)


def test_inventory_refuses_symlinks_including_ancestor(tmp_path):
    target = tmp_path / 'original'; target.mkdir(); (target / 'row').write_bytes(b'original')
    link = tmp_path / 'alias'; link.symlink_to(target)
    with pytest.raises(ValueError): summary.inventory(link)
    (target / 'linked').symlink_to(target / 'row')
    with pytest.raises(ValueError): summary.inventory(target)


def test_exact_expected_inventory_keeps_18_remote_tensors_and_108_png():
    sources = {f'source_{i}.py': 'a' * 64 for i in range(22)}
    names = summary.expected_names(sources)
    assert len(names) == 204
    assert sum(n.endswith('.pt') for n in names) == 18
    assert sum(n.endswith('.png') for n in names) == 108
    assert sum(n.endswith('/metrics.jsonl') for n in names) == 6
    assert sum(n.endswith('/samples.jsonl') for n in names) == 18


def test_int64_oracle_digest_matches_pinned_tensor_serializer():
    import torch
    from portable_e2e.frozen_selector import _digest_tensors
    indices = [6, 0, 11, 1]
    assert summary.oracle_digest(indices) == _digest_tensors([('composite_oracle', torch.tensor(indices))])
    with pytest.raises(ValueError): summary.oracle_digest([True])


def history(seed='20260903'):
    from portable_e2e.train import _epoch_batches
    records = []; seen = 0; order = []
    for epoch in range(6):
        for batch in _epoch_batches(1147, batch_size=4, seed=int(seed), epoch=epoch):
            if len(records) == 1540: break
            seen += len(batch); indices = list(batch); order.append(dict(epoch=epoch, indices=indices))
            records.append(dict(global_step=len(records) + 1, epoch=epoch, batch_samples=len(batch), samples_seen=seen,
                sample_indices=indices, candidate_score_loss=1., optimization_loss=.1, gradient_norm=.5))
    head = dict(history=deepcopy(records), sampling_order_sha256=summary.sha(json.dumps(order, separators=(',', ':')).encode()))
    return records, head


def test_full_history_recomputes_seeded_1540_steps_and_all6155_exposures():
    rows, head = history(); assert summary.verify_history(rows, head, '20260903') == head['sampling_order_sha256']
    assert rows[-1]['samples_seen'] == 6155 and rows[-1]['epoch'] == 5
    assert sum(r['batch_samples'] == 3 for r in rows) == 5


@pytest.mark.parametrize('field,value', [('global_step', True), ('samples_seen', 5), ('gradient_norm', float('inf')),
    ('candidate_score_loss', -1.), ('optimization_loss', False), ('sample_indices', [0, 0, 1, 2])])
def test_history_mutation_fails_even_if_copied_into_report(field, value):
    rows, head = history(); rows[0][field] = value; head['history'] = deepcopy(rows)
    with pytest.raises(ValueError): summary.verify_history(rows, head, '20260903')


def test_sampling_source_seed_cannot_be_replaced_by_same_pair_wrong_order():
    rows, head = history('20260904')
    with pytest.raises(ValueError): summary.verify_history(rows, head, '20260903')


def test_only_existing_entropy_roundoff_is_allowed():
    summary.compare_aggregates({'selector': {'normalized_selection_entropy': .5}}, {'selector': {'normalized_selection_entropy': .5 + 1e-16}})
    for a, b in [({'metrics': {'ade': 1.}}, {'metrics': {'ade': 1. + 1e-15}}),
        ({'count': 1}, {'count': True}), ({'selector': {'normalized_selection_entropy': .5}}, {'selector': {'normalized_selection_entropy': .51}})]:
        with pytest.raises(ValueError): summary.compare_aggregates(a, b)


def small_analysis(tmp_path, monkeypatch):
    from PIL import Image
    from scripts.e2e.audit_portable_stopmix_behavior import summarize_rows
    monkeypatch.setattr(summary, 'COUNTS', {'train': 1147, 'val': 2}); monkeypatch.setattr(summary, 'INDICES', (0, 1))
    rows = [row(i) for i in range(2)]; prefix = 'original/val'; output = tmp_path / prefix; output.mkdir(parents=True)
    raw = b''.join(summary.encoded(r) for r in rows); (output / 'samples.jsonl').write_bytes(raw)
    renders = []
    for i in (0, 1):
        path = output / f'val_{i:03d}.png'; Image.new('RGB', (900, 700), 'white').save(path)
        renders.append(dict(index=i, sample_id=rows[i]['sample_id'], camera_sha256=rows[i]['camera_sha256'], file=path.name, sha256=summary.digest(path)))
    costs = [r['loss_alignment'] for r in rows]
    report = dict(schema='portable_e2e.cached_stopmix_selection_diagnostic.v1', split='val', sample_count=2,
        samples_sha256=summary.sha(raw), cache_digest='a' * 64,
        oracle_and_cost_sha256=summary.sha(summary.encoded([{k: r[k] for k in summary.COST_INVARIANT} for r in costs])),
        oracle_indices=[6, 6], **summarize_rows(rows, 12), composite_cost_alignment=summary.cost_groups(costs),
        renders=renders, **{k: False for k in summary.DENIALS})
    (output / 'summary.json').write_bytes(summary.encoded(report))
    return prefix, report, rows


def test_saved_analysis_full_decode_aggregates_and_source_image_binding(tmp_path, monkeypatch):
    prefix, report, rows = small_analysis(tmp_path, monkeypatch)
    actual, observed = summary.verify_analysis(tmp_path, prefix, 'val', 'a' * 64, lambda n: (tmp_path / n).read_bytes(), {})
    assert actual == report and observed == rows


@pytest.mark.parametrize('mutation', ['admit', 'metric', 'render', 'oracle', 'png'])
def test_analysis_payload_mutation_is_not_hidden_by_resealed_summary(tmp_path, monkeypatch, mutation):
    prefix, report, rows = small_analysis(tmp_path, monkeypatch)
    if mutation == 'admit': report['training_data_approved'] = True
    elif mutation == 'metric': report['all_samples']['metrics']['selected_ade_m'] = .01
    elif mutation == 'render': report['renders'][0]['sample_id'] = 'wrong'
    elif mutation == 'oracle': report['oracle_indices'][0] = 7
    else:
        path = tmp_path / prefix / 'val_000.png'; path.write_bytes(b'not an image'); report['renders'][0]['sha256'] = summary.digest(path)
    (tmp_path / prefix / 'summary.json').write_bytes(summary.encoded(report))
    with pytest.raises((ValueError, OSError)): summary.verify_analysis(tmp_path, prefix, 'val', 'a' * 64, lambda n: (tmp_path / n).read_bytes(), {})


def test_missing_worker_report_explicitly_preserves_all_six_head_denominator(tmp_path, monkeypatch):
    raw = b'{}\n'; (tmp_path / 'plan.json').write_bytes(raw); monkeypatch.setattr(summary, 'PLAN_SHA', summary.sha(raw))
    path = tmp_path / 'seed_20260903/linear_pair_reset'; path.mkdir(parents=True); (path / 'metrics.jsonl').write_bytes(b'{}\n')
    result = summary.summarize_campaign(tmp_path)
    assert result['status'] == 'INCOMPLETE_NOT_PROMOTED'
    assert len(result['performed_heads']) == 6
    assert [r['status'] for r in result['performed_heads']].count('INCOMPLETE') == 1
    assert result['automatic_promotion'] is False


def test_fresh_output_and_mutation_during_persistence_fail_closed(tmp_path, monkeypatch):
    inputs = tmp_path / 'input'; inputs.mkdir(); (inputs / 'plan.json').write_bytes(b'{}\n')
    monkeypatch.setattr(summary, 'PLAN_SHA', summary.sha(b'{}\n'))
    output = tmp_path / 'out'; result = summary.run(inputs, output)
    assert result['status'] == 'INCOMPLETE_NOT_PROMOTED'
    assert summary.manifest((output / 'SHA256SUMS').read_bytes()) == {'summary.json': summary.digest(output / 'summary.json')}
    with pytest.raises(ValueError): summary.run(inputs, output)
    with pytest.raises(ValueError): summary.run(inputs, inputs / 'nested')
    old_inventory = summary.inventory; calls = 0
    def mutated(path):
        nonlocal calls
        calls += 1
        if calls == 3: (inputs / 'changed').write_bytes(b'new')
        return old_inventory(path)
    monkeypatch.setattr(summary, 'inventory', mutated)
    with pytest.raises(ValueError, match='persistence'): summary.run(inputs, tmp_path / 'bad')
    assert (tmp_path / 'bad/failed.json').is_file() and not (tmp_path / 'bad/SHA256SUMS').exists()


def test_dataset_symlink_alias_cannot_receive_results(tmp_path, monkeypatch):
    repo = tmp_path / 'repo'; repo.mkdir(); data = tmp_path / 'physical_data'; data.mkdir()
    (repo / 'datasets').symlink_to(data); monkeypatch.setattr(summary, 'ROOT', repo)
    with pytest.raises(ValueError, match='dataset'): summary.run(tmp_path / 'inputs', data / 'result')


def test_timezone_must_be_explicit_utc():
    assert summary.timestamp('2026-09-09T00:00:00Z').utcoffset().total_seconds() == 0
    for value in ('2026-09-09T00:00:00', '2026-09-09T09:00:00+09:00', None):
        with pytest.raises(ValueError): summary.timestamp(value)


def metric(split):
    count = summary.COUNTS[split]; histogram = [0] * 12; histogram[6] = count
    return dict(split=split, sample_count=count, metrics={k: 0. for k in summary.CORE_METRICS},
        metric_counts={k: count for k in summary.CORE_METRICS}, selected_histogram=histogram,
        composite_oracle_histogram=list(histogram), selected_composite_oracle_agreement_count=count)


@pytest.mark.parametrize('mutation', ['count', 'bool', 'missing', 'negative', 'histogram', 'agreement'])
def test_core_metrics_require_all_horizons_and_exact_denominators(mutation):
    value = metric('val'); summary.verify_core_metrics(value, 'val')
    if mutation == 'count': value['metric_counts']['ade_1p0s_m'] = 336
    elif mutation == 'bool': value['metric_counts']['ade_1p0s_m'] = True
    elif mutation == 'missing': del value['metrics']['speed_mae_6p4s_mps']
    elif mutation == 'negative': value['metrics']['regression_loss'] = -1.
    elif mutation == 'histogram': value['selected_histogram'][6] = 336
    else: value['selected_composite_oracle_agreement_count'] = True
    with pytest.raises(ValueError): summary.verify_core_metrics(value, 'val')


def head_fixture():
    rows, head = history(); seed = '20260903'; cache = {s: 'b' * 64 for s in summary.COUNTS}
    plan = dict(loss_config={'candidate_score_weight': .1}, parent_checkpoint_sha256={seed: 'a' * 64},
        corpus_fingerprint_sha256='c' * 64, split_contract={s: {'fingerprint_sha256': 'd' * 64} for s in summary.COUNTS})
    head.update(artifact_id='portable_e2e.frozen_stopmix_selector_research_head.v1', status='RESEARCH_HEAD_TRAINING_COMPLETE_NOT_PROMOTED',
        arm='linear_pair_reset', seed=int(seed), device='cuda:0', optimizer='AdamW', steps=1540, batch_size=4, learning_rate=.0001,
        weight_decay=.0001, candidate_score_weight=.1, maximum_gradient_norm=5., loss_config=plan['loss_config'],
        source_checkpoint_sha256='a' * 64, corpus_fingerprint_sha256='c' * 64,
        split_fingerprints={s: 'd' * 64 for s in summary.COUNTS}, head_parameter_count=3084, candidate_count=12,
        final_state=dict(global_step=1540, samples_seen=6155, last_epoch_index=5),
        generator_parameters_trained=False, training_data_approved=False, vehicle_control_approved=False,
        automatic_promotion=False, runtime_bundle_supported=False, cache_sha256_before=cache, cache_sha256_after=dict(cache),
        original_head_sha256_before={'drive': 'e' * 64, 'stop': 'f' * 64}, original_head_sha256_after={'drive': 'e' * 64, 'stop': 'f' * 64},
        initial_head_sha256='a' * 64, final_head_sha256='b' * 64,
        original_logit_parity={s: dict(cached_original_logits_required=True, byte_equal=True, max_abs_logit_difference=0., selected_index_mismatch_count=0) for s in summary.COUNTS},
        baseline_metrics={s: metric(s) for s in summary.COUNTS}, pre_training_metrics={s: metric(s) for s in summary.COUNTS},
        post_training_metrics={s: metric(s) for s in summary.COUNTS}, composite_oracle_sha256={})
    analyses = {}
    for split, count in summary.COUNTS.items():
        samples = [dict(sample_id=f'{split}_{i}') for i in range(count)]; head[split + '_sample_ids'] = [r['sample_id'] for r in samples]
        oracles = [6] * count; head['composite_oracle_sha256'][split] = summary.oracle_digest(oracles)
        h = {str(i): count if i == 6 else 0 for i in range(12)}
        # HH_260906 - JSON writers sort dictionary keys lexically; index10 must never be treated as histogram slot2.
        h = json.loads(json.dumps(h, sort_keys=True))
        analyses[split] = (dict(oracle_indices=oracles, composite_cost_alignment={'all_samples': dict(selected_histogram=h,
            composite_oracle_histogram=h, selected_composite_oracle_index_agreement_count=count)}), samples)
    return head, rows, seed, plan, dict(cache_digest=cache), analyses


def test_complete_head_configuration_baseline_and_oracle_hash_binding():
    head, rows, seed, plan, state, analyses = head_fixture()
    result = summary.verify_head(head, rows, seed, 'linear_pair_reset', plan, state, analyses, analyses)
    assert result == head['sampling_order_sha256']


@pytest.mark.parametrize('mutation', ['cache', 'old_head', 'parity', 'oracle', 'ids', 'capacity', 'generator', 'fingerprint', 'loss'])
def test_head_cannot_relabel_changes_as_frozen_training(mutation):
    head, rows, seed, plan, state, analyses = head_fixture()
    if mutation == 'cache': head['cache_sha256_after']['train'] = 'f' * 64
    elif mutation == 'old_head': head['original_head_sha256_after']['drive'] = 'a' * 64
    elif mutation == 'parity': head['original_logit_parity']['val']['byte_equal'] = False
    elif mutation == 'oracle': head['composite_oracle_sha256']['val'] = 'a' * 64
    elif mutation == 'ids': head['train_sample_ids'][0] = 'foreign'
    elif mutation == 'capacity': head['head_parameter_count'] = 115201
    elif mutation == 'generator': head['generator_parameters_trained'] = True
    elif mutation == 'fingerprint': head['split_fingerprints']['val'] = 'f' * 64
    else: head['loss_config'] = {'candidate_score_weight': .2}
    with pytest.raises(ValueError): summary.verify_head(head, rows, seed, 'linear_pair_reset', plan, state, analyses, analyses)


def transport_fixture(tmp_path, monkeypatch):
    original = {}; remote = {}; missing = {f'tensor_{i}.pt' for i in range(18)}
    for name in ('report.json', *(f'meta_{i}.json' for i in range(185)), *sorted(missing)):
        raw = summary.encoded(dict(completed_at_utc='2026-09-08T20:40:00Z')) if name == 'report.json' else b'{}\n'
        original[name] = summary.sha(raw); remote[name] = dict(sha256=summary.sha(raw), size_bytes=len(raw))
        if name not in missing: (tmp_path / name).write_bytes(raw)
    raw = ''.join(v + '  ' + n + '\n' for n, v in original.items()).encode(); (tmp_path / 'SHA256SUMS').write_bytes(raw)
    remote['SHA256SUMS'] = dict(sha256=summary.sha(raw), size_bytes=len(raw))
    monkeypatch.setattr(summary, 'REPORT_SHA', original['report.json']); monkeypatch.setattr(summary, 'MANIFEST_SHA', summary.sha(raw))
    value = dict(status='VERIFIED_SIX_HEAD_FITS_NOT_PROMOTED', source_commit=summary.COMMIT, plan_sha256=summary.PLAN_SHA,
        report_sha256=summary.REPORT_SHA, original_manifest_sha256=summary.MANIFEST_SHA,
        remote_pre_post_bytes_unchanged=True, source_and_parent_bytes_unchanged=True, cooperative_gpu0_lease_available_and_held=True,
        first_rejected_launcher_preserved=True, local_launch_evidence_unchanged=True,
        collector_tensor_loading=False, tensor_artifacts_transferred=False, model_promotion=False, data_admission=False,
        regular_file_inventory=remote, remote_file_count=205, collected_file_count=187,
        omitted_files={n: dict(remote[n], reason='remote tensor, never transferred') for n in missing},
        collected_at_utc='2026-09-08T20:45:00Z')
    (tmp_path / 'transport_verification.json').write_bytes(summary.encoded(value))
    monkeypatch.setattr(summary, 'TRANSPORT_SHA', summary.digest(tmp_path / 'transport_verification.json'))
    return original, missing, value


def test_transport_authenticates_exact_mirrored_bytes_and_separates_remote_pt_witnesses(tmp_path, monkeypatch):
    original, missing, _ = transport_fixture(tmp_path, monkeypatch)
    proof = summary.verify_transport(tmp_path, summary.inventory(tmp_path), original, missing)
    assert proof['tensor_bytes_locally_verified'] is False and proof['omitted_file_count'] == 18
    assert proof['remote_tensor_byte_hashes_verified_by_collector'] is True and proof['local_reader_loaded_tensors'] is False


@pytest.mark.parametrize('mutation', ['size', 'sha', 'missing', 'extra', 'postcheck', 'admission', 'time', 'tensor', 'receipt'])
def test_transport_omission_and_failure_cannot_be_resealed(tmp_path, monkeypatch, mutation):
    original, missing, value = transport_fixture(tmp_path, monkeypatch)
    if mutation == 'size': value['regular_file_inventory']['meta_0.json']['size_bytes'] += 1
    elif mutation == 'sha': value['regular_file_inventory']['tensor_0.pt']['sha256'] = 'f' * 64
    elif mutation == 'missing': value['omitted_files'].pop('tensor_0.pt')
    elif mutation == 'extra': value['regular_file_inventory']['test.pt'] = dict(sha256='a' * 64, size_bytes=1)
    elif mutation == 'postcheck': value['source_and_parent_bytes_unchanged'] = False
    elif mutation == 'admission': value['data_admission'] = True
    elif mutation == 'time': value['collected_at_utc'] = '2026-09-08T20:39:00Z'
    elif mutation == 'tensor': value['tensor_artifacts_transferred'] = True
    else: value['first_rejected_launcher_preserved'] = False
    (tmp_path / 'transport_verification.json').write_bytes(summary.encoded(value))
    monkeypatch.setattr(summary, 'TRANSPORT_SHA', summary.digest(tmp_path / 'transport_verification.json'))
    with pytest.raises(ValueError): summary.verify_transport(tmp_path, summary.inventory(tmp_path), original, missing)


def test_omitted_tensors_never_accepted_without_transport(tmp_path):
    with pytest.raises(ValueError, match='receipt'): summary.verify_transport(tmp_path, {}, {'a.pt': 'a' * 64}, {'a.pt'})


def test_actual_transport_receipt_pin_cannot_be_replaced_by_other_valid_json(tmp_path, monkeypatch):
    original, missing, value = transport_fixture(tmp_path, monkeypatch)
    value['extra'] = 'not executed evidence'; (tmp_path / 'transport_verification.json').write_bytes(summary.encoded(value))
    with pytest.raises(ValueError, match='frozen actual transport'): summary.verify_transport(tmp_path, summary.inventory(tmp_path), original, missing)


def test_partial_recorded_complete_head_never_makes_full_study_complete():
    report = {'seeds': [{'seed': '20260903', 'arms': [{'arm': 'linear_pair_reset', 'status': 'HEAD_COMPLETE_NOT_PROMOTED'}]}]}
    result = summary.outcome_ledger({'seed_20260903/linear_pair_reset/report.json': 'a' * 64}, report)
    assert len(result) == 6 and result[0]['status'] == 'INCOMPLETE'
    assert result[0]['recorded_worker_status'] == 'HEAD_COMPLETE_NOT_PROMOTED'
    assert all(r['status'] == 'NOT_RUN' for r in result[1:])
