"""HH_260906 - Challenge residual-only provenance and exact cached-prior witnesses with synthetic CPU evidence."""
from copy import deepcopy
import importlib.util
import json
from pathlib import Path

import pytest

from scripts.e2e import summarize_frozen_stopmix_residual as summary

spec = importlib.util.spec_from_file_location('_frozen_summary_synthetic_fixtures', Path(__file__).with_name('test_summarize_frozen_stopmix_selector.py'))
fixtures = importlib.util.module_from_spec(spec); spec.loader.exec_module(fixtures)


def test_previous_summary_and_frozen_execution_contract_remain_separate():
    assert summary.digest(Path(summary.base.__file__)) == summary.BASE_SHA
    assert summary.ARMS == ('prior_only_residual', 'candidate_residual')
    assert summary.base.ARMS == ('linear_pair_reset', 'candidate_reset')
    names = summary.expected_names({f'source_{i}.py': 'a' * 64 for i in range(24)})
    assert len(names) == 194 and sum(n.endswith('.pt') for n in names) == 12
    assert sum(n.endswith('.png') for n in names) == 108
    assert sum(n.endswith('/samples.jsonl') for n in names) == 18
    assert not any('_cache.pt' in n or 'cache_progress' in n for n in names)


def test_saved_logit_bytes_match_original_tensor_digest_without_network():
    import torch
    from portable_e2e.frozen_selector import _digest_tensors
    rows = [fixtures.row(i) for i in range(3)]
    values = torch.tensor([r['loss_alignment']['candidate_logits'] for r in rows], dtype=torch.float32)
    assert summary.logits_digest(rows) == _digest_tensors([('logits', values)])
    rows[0]['loss_alignment']['candidate_logits'][0] = -0.
    assert summary.logits_digest(rows) != _digest_tensors([('logits', values)])


@pytest.mark.parametrize('values', [[1.] * 11, [True] * 12, [float('inf')] * 12])
def test_saved_logit_digest_rejects_bad_shape_or_values(values):
    row = fixtures.row(); row['loss_alignment']['candidate_logits'] = values
    with pytest.raises(ValueError): summary.logits_digest([row])


def cache_proof():
    state = dict(cache_digest={'train': 'a' * 64, 'val': 'b' * 64})
    files = {'seed_20260903/train_cache.pt': 'c' * 64}
    proof = dict(path='seed_20260903/train_cache.pt', sha256='c' * 64, cache_digest='a' * 64, sample_count=1147,
        weights_only=True, map_location='cpu', targets_and_routes_reconstructed_exactly=True,
        fused_candidates_independently_regenerated=False, model_input_sha_recomputed=False)
    return state, files, proof


def test_secure_cache_proof_reports_reuse_not_new_generator_input_replay():
    state, files, proof = cache_proof(); summary.verify_cache_proof(proof, 'train', '20260903', state, files)
    assert proof['model_input_sha_recomputed'] is False


@pytest.mark.parametrize('field,value', [('path', '../wrong.pt'), ('sha256', 'd' * 64), ('cache_digest', 'e' * 64),
    ('sample_count', 1146), ('sample_count', True), ('weights_only', False), ('map_location', 'cuda:0'),
    ('targets_and_routes_reconstructed_exactly', False), ('fused_candidates_independently_regenerated', True),
    ('model_input_sha_recomputed', True)])
def test_secure_cache_scope_cannot_be_expanded_or_reclassified(field, value):
    state, files, proof = cache_proof(); proof[field] = value
    with pytest.raises(ValueError): summary.verify_cache_proof(proof, 'train', '20260903', state, files)


def test_fixed_rows_allow_changed_selection_only_not_candidate_or_input_rewrites():
    a, b = [fixtures.row()], [fixtures.row(selected=0)]
    summary.verify_fixed_rows(a, b)
    with pytest.raises(ValueError, match='original baseline'): summary.verify_fixed_rows(a, b, original_logits=True)
    summary.verify_fixed_rows(a, deepcopy(a), original_logits=True)


@pytest.mark.parametrize('path,value', [(('model_input_sha256',), 'e' * 64), (('target_sha256',), 'f' * 64),
    (('raw_current_vx_mps',), .1), (('candidates', 0, 'ade_m'), .01),
    (('runtime_geometry', 'candidates', 0, 'failure_codes'), ['speed_rate']),
    (('loss_alignment', 'candidate_composite_costs', 0), .5),
    (('loss_alignment', 'original_target_valid', 63), False)])
def test_fixed_candidate_cache_is_not_repaired_or_filtered(path, value):
    a, b = fixtures.row(), fixtures.row()
    current = b
    for key in path[:-1]: current = current[key]
    current[path[-1]] = value
    with pytest.raises(ValueError): summary.verify_fixed_rows([a], [b])


def head_fixture(arm='prior_only_residual'):
    head, history, seed, plan, state, analyses = fixtures.head_fixture()
    head.update(artifact_id='portable_e2e.frozen_stopmix_residual_research_head.v1', arm=arm, head_parameter_count=115457,
        original_heads_recomputed_or_trained=False,
        residual_inputs=dict(frozen_context_features=256, original_candidate_score_features=1, candidate_geometry_features=192,
            geometry_provided=arm == 'candidate_residual', xy_divisor_m=120., speed_divisor_mps=30 / 3.6))
    for split, (report, rows) in analyses.items():
        for index, old in enumerate(rows):
            row = fixtures.row(index); row['sample_id'] = old['sample_id']; old.update(row)
        pin = summary.logits_digest(rows)
        head['original_logit_parity'][split].update(original_logits_sha256=pin, initial_logits_sha256=pin)
    return head, history, seed, plan, state, analyses


@pytest.mark.parametrize('arm', summary.ARMS)
def test_complete_residual_head_same_capacity_prior_byte_witness_and_fixed_sampling(arm):
    head, history, seed, plan, state, analyses = head_fixture(arm)
    assert summary.verify_head(head, history, seed, arm, plan, state, analyses, analyses) == head['sampling_order_sha256']
    assert head['head_parameter_count'] == 115457 and head['cache_sha256_before'] == head['cache_sha256_after']


@pytest.mark.parametrize('mutation', ['arm', 'capacity', 'geometry', 'prior_digest', 'initial_digest', 'parity',
    'before_metric', 'cache', 'oracle', 'training', 'original_head', 'learning_rate'])
def test_residual_core_reports_fail_closed_on_changed_contract(mutation):
    head, history, seed, plan, state, analyses = head_fixture()
    if mutation == 'arm': head['arm'] = 'candidate_reset'
    elif mutation == 'capacity': head['head_parameter_count'] = 115201
    elif mutation == 'geometry': head['residual_inputs']['geometry_provided'] = True
    elif mutation == 'prior_digest': head['original_logit_parity']['train']['original_logits_sha256'] = 'a' * 64
    elif mutation == 'initial_digest': head['original_logit_parity']['train']['initial_logits_sha256'] = 'a' * 64
    elif mutation == 'parity': head['original_logit_parity']['val']['byte_equal'] = False
    elif mutation == 'before_metric': head['pre_training_metrics']['val']['metrics']['loss'] = 1.
    elif mutation == 'cache': head['cache_sha256_after']['train'] = 'a' * 64
    elif mutation == 'oracle': head['composite_oracle_sha256']['val'] = 'a' * 64
    elif mutation == 'training': head['generator_parameters_trained'] = True
    elif mutation == 'original_head': head['original_heads_recomputed_or_trained'] = True
    else: head['learning_rate'] = .001
    with pytest.raises(ValueError): summary.verify_head(head, history, seed, 'prior_only_residual', plan, state, analyses, analyses)


def test_partial_study_always_retains_all_six_declared_heads():
    before = {'seed_20260903/prior_only_residual/metrics.jsonl': 'a' * 64}
    result = summary.outcome_ledger(before, {'seeds': []})
    assert len(result) == 6 and result[0]['status'] == 'INCOMPLETE'
    assert all(r['status'] == 'NOT_RUN' for r in result[1:])
    assert [r['arm'] for r in result] == list(summary.ARMS) * 3


def test_missing_report_is_retained_without_reading_parent_or_claiming_completion(tmp_path, monkeypatch):
    raw = b'{}\n'; (tmp_path / 'plan.json').write_bytes(raw); monkeypatch.setattr(summary, 'PLAN_SHA', summary.sha(raw))
    monkeypatch.setattr(summary.base, 'summarize_campaign', lambda *_: pytest.fail('incomplete run cannot invoke parent'))
    result = summary.summarize_campaign(tmp_path)
    assert result['status'] == 'INCOMPLETE_NOT_PROMOTED' and len(result['performed_heads']) == 6
    assert result['generator_neural_forwards'] == 0 and result['training_data_approved'] is False


def test_unknown_completion_and_old_study_pins_cannot_masquerade_as_residual(tmp_path, monkeypatch):
    plan = b'{}\n'; (tmp_path / 'plan.json').write_bytes(plan); monkeypatch.setattr(summary, 'PLAN_SHA', summary.sha(plan))
    (tmp_path / 'report.json').write_bytes(summary.encoded(dict(schema='old', status='SIX_HEAD_FITS_COMPLETE_NOT_PROMOTED')))
    with pytest.raises(ValueError): summary.summarize_campaign(tmp_path)


def test_residual_summary_refuses_existing_input_or_dataset_outputs(tmp_path, monkeypatch):
    root = tmp_path / 'input'; root.mkdir(); parent = tmp_path / 'parent'; parent.mkdir()
    for output in (root, root / 'nested', parent / 'nested'):
        with pytest.raises(ValueError): summary.run(root, output, parent)
    repo = tmp_path / 'repo'; repo.mkdir(); data = tmp_path / 'dataset'; data.mkdir(); (repo / 'datasets').symlink_to(data)
    monkeypatch.setattr(summary, 'ROOT', repo)
    with pytest.raises(ValueError): summary.run(root, data / 'new', parent)


def test_partial_persistence_keeps_self_checksum_outside_manifest(tmp_path, monkeypatch):
    root = tmp_path / 'input'; root.mkdir(); raw = b'{}\n'; (root / 'plan.json').write_bytes(raw)
    monkeypatch.setattr(summary, 'PLAN_SHA', summary.sha(raw)); out = tmp_path / 'out'
    result = summary.run(root, out)
    assert result['status'] == 'INCOMPLETE_NOT_PROMOTED'
    assert summary.base.manifest((out / 'SHA256SUMS').read_bytes()) == {'summary.json': summary.digest(out / 'summary.json')}
    with pytest.raises(ValueError): summary.run(root, out)


def transport_fixture(tmp_path, monkeypatch):
    original = {}; remote = {}; missing = {f'tensor_{i}.pt' for i in range(12)}
    for name in ('report.json', *(f'file_{i}.json' for i in range(181)), *sorted(missing)):
        raw = summary.encoded(dict(completed_at_utc='2026-09-08T21:14:00Z')) if name == 'report.json' else b'{}\n'
        original[name] = summary.sha(raw); remote[name] = dict(sha256=summary.sha(raw), size_bytes=len(raw))
        if name not in missing: (tmp_path / name).write_bytes(raw)
    raw = ''.join(v + '  ' + n + '\n' for n, v in original.items()).encode(); (tmp_path / 'SHA256SUMS').write_bytes(raw)
    remote['SHA256SUMS'] = dict(sha256=summary.sha(raw), size_bytes=len(raw))
    monkeypatch.setattr(summary, 'REPORT_SHA', original['report.json']); monkeypatch.setattr(summary, 'MANIFEST_SHA', summary.sha(raw))
    parent_transport = {'regular_file_inventory': {f'parent_{i}': dict(sha256='a' * 64, size_bytes=1) for i in range(205)}}
    parent_report = {'original_input_sha256': {f'lineage_{i}': 'b' * 64 for i in range(17)}}
    value = dict(status='VERIFIED_SIX_RESIDUAL_HEAD_FITS_NOT_PROMOTED', source_commit=summary.base.COMMIT,
        plan_sha256=summary.PLAN_SHA, report_sha256=summary.REPORT_SHA, original_manifest_sha256=summary.MANIFEST_SHA,
        regular_file_inventory=remote, remote_file_count=195, collected_file_count=183,
        omitted_files={n: dict(remote[n], reason='Remote tensor; hash only.') for n in missing},
        remote_pre_post_bytes_unchanged=True, source_and_parent_bytes_unchanged=True,
        cooperative_gpu0_lease_available_and_held=True, local_launch_evidence_unchanged=True,
        first_parent_rejected_launcher_preserved=True, collector_tensor_loading=False,
        tensor_artifacts_transferred=False, model_promotion=False, data_admission=False,
        original_input_verification=dict(original_cache_run_inventory=deepcopy(parent_transport['regular_file_inventory']),
            continued_parent_input_sha256=dict(parent_report['original_input_sha256'])),
        analysis_sample_rows=13356, head_history_rows=9240, head_only_fit_count=6, full_model_fit_count=0,
        generator_neural_forwards=0, fixed_val_png_count=108, original_cache_files_loaded_by_worker=6,
        collected_at_utc='2026-09-08T21:20:00Z')
    path = tmp_path / 'transport_verification.json'; path.write_bytes(summary.encoded(value))
    monkeypatch.setattr(summary, 'TRANSPORT_SHA', summary.digest(path))
    return original, missing, value, parent_transport, parent_report


def test_transport_keeps_new12_pt_and_all_original205_plus17_lineages_separate(tmp_path, monkeypatch):
    original, missing, _, parent_transport, parent_report = transport_fixture(tmp_path, monkeypatch)
    result = summary.verify_transport(tmp_path, summary.base.inventory(tmp_path), original, missing, parent_transport, parent_report)
    assert result['omitted_remote_tensor_count'] == 12 and result['local_tensor_bytes_loaded'] is False
    assert result['remote_tensor_bytes_verified_by_collector'] is True


@pytest.mark.parametrize('mutation', ['parent_cache', 'parent_lineage', 'new_pt', 'mirror_size', 'omitted', 'count',
    'bool_count', 'neural', 'tensor_load', 'postcheck', 'prior_failure', 'time'])
def test_residual_transport_cannot_hide_original_changes_or_tensor_execution(tmp_path, monkeypatch, mutation):
    original, missing, value, parent_transport, parent_report = transport_fixture(tmp_path, monkeypatch)
    if mutation == 'parent_cache': value['original_input_verification']['original_cache_run_inventory']['parent_0']['sha256'] = 'c' * 64
    elif mutation == 'parent_lineage': value['original_input_verification']['continued_parent_input_sha256']['lineage_0'] = 'c' * 64
    elif mutation == 'new_pt': value['regular_file_inventory']['tensor_0.pt']['sha256'] = 'c' * 64
    elif mutation == 'mirror_size': value['regular_file_inventory']['file_0.json']['size_bytes'] = 100
    elif mutation == 'omitted': value['omitted_files'].pop('tensor_0.pt')
    elif mutation == 'count': value['head_only_fit_count'] = 5
    elif mutation == 'bool_count': value['generator_neural_forwards'] = False
    elif mutation == 'neural': value['generator_neural_forwards'] = 1
    elif mutation == 'tensor_load': value['collector_tensor_loading'] = True
    elif mutation == 'postcheck': value['source_and_parent_bytes_unchanged'] = False
    elif mutation == 'prior_failure': value['first_parent_rejected_launcher_preserved'] = False
    else: value['collected_at_utc'] = '2026-09-08T21:13:00Z'
    path = tmp_path / 'transport_verification.json'; path.write_bytes(summary.encoded(value))
    monkeypatch.setattr(summary, 'TRANSPORT_SHA', summary.digest(path))
    with pytest.raises(ValueError): summary.verify_transport(tmp_path, summary.base.inventory(tmp_path), original, missing, parent_transport, parent_report)


def test_actual_transport_pin_required_even_if_structurally_valid(tmp_path, monkeypatch):
    original, missing, value, parent_transport, parent_report = transport_fixture(tmp_path, monkeypatch)
    value['extra'] = True; (tmp_path / 'transport_verification.json').write_bytes(summary.encoded(value))
    with pytest.raises(ValueError, match='receipt'):
        summary.verify_transport(tmp_path, summary.base.inventory(tmp_path), original, missing, parent_transport, parent_report)


def test_final_recheck_detects_parent_or_dependency_mutation(tmp_path, monkeypatch):
    root = tmp_path / 'run'; parent = tmp_path / 'parent'; root.mkdir(); parent.mkdir()
    (root / 'x').write_bytes(b'one'); (parent / 'y').write_bytes(b'two')
    result = dict(input_sha256=summary.base.inventory(root), parent_input_sha256=summary.base.inventory(parent),
        summary_source_sha256=summary.digest(Path(summary.__file__)), execution_source_sha256={})
    summary.recheck(root, parent, result)
    (parent / 'y').write_bytes(b'changed')
    with pytest.raises(ValueError): summary.recheck(root, parent, result)


def test_group_diagnostics_preserve_gate_failures_at_launch_and_full_horizon():
    from scripts.e2e.audit_portable_stopmix_behavior import summarize_rows
    row = fixtures.row(selected=0)
    row['runtime_geometry']['candidates'][0]['failure_codes'] = ['first_distance', 'speed_rate', 'curvature']
    row['runtime_geometry']['selected_failure_codes'] = ['first_distance', 'speed_rate', 'curvature']
    summary.base.check_row(row, 0)
    result = summarize_rows([row], 12)
    assert result['all_samples']['geometry']['selected_result']['failure_counts'] == {'speed_rate': 1, 'first_distance': 1, 'curvature': 1}
    assert result['target_motion_groups']['stationary_hold']['geometry']['selected_result']['geometry_reject_count'] == 1
