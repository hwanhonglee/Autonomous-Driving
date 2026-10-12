"""HH_260906 - CPU-only synthetic checks of rank provenance, final pair diagnostics and recorded scalar arithmetic."""
from copy import deepcopy
import importlib.util
import json
import math
from pathlib import Path

import pytest

from scripts.e2e import summarize_frozen_stopmix_drive_rank as s

spec = importlib.util.spec_from_file_location('_rank_summary_residual_fixtures', Path(__file__).with_name('test_summarize_frozen_stopmix_residual.py'))
rf = importlib.util.module_from_spec(spec); spec.loader.exec_module(rf)
fixtures = rf.fixtures


def test_research_revision_does_not_override_historical_source_pins():
    assert s.digest(Path(s.residual.__file__)) == s.RESIDUAL_SHA
    assert s.digest(Path(s.base.__file__)) == s.residual.BASE_SHA
    assert s.ARMS == ('ce_continue', 'drive_pair_continue') and s.residual.ARMS == ('prior_only_residual', 'candidate_residual')
    files = s.expected_names({f'source{i}.py':'a'*64 for i in range(26)})
    assert len(files) == 196 and sum(n.endswith('.pt') for n in files) == 12
    assert sum(n.endswith('.png') for n in files) == 108 and sum(n.endswith('/samples.jsonl') for n in files) == 18
    assert any('/parent_candidate_residual/' in n for n in files)


@pytest.mark.parametrize('value', [False, float('nan'), float('inf'), 1e100])
def test_float32_refuses_nonfinite_boolean_and_overflow(value):
    with pytest.raises(ValueError): s.f32(value)


def rank_history(arm='ce_continue'):
    rows, head = fixtures.history(); weight = s.ARMS.index(arm)
    for row in rows:
        ce, rank = s.f32(1.1234567), s.f32(.4321098)
        row.update(candidate_score_loss=ce, drive_pair_rank_loss=rank, drive_pair_weight=weight,
            optimization_loss=s.f32(s.f32(s.f32(.1)*ce) + s.f32(s.f32(.1)*rank)) if weight else s.f32(s.f32(.1)*ce))
    head['history'] = deepcopy(rows)
    return rows, head


@pytest.mark.parametrize('arm', s.ARMS)
def test_full1540_rows_scalar_float32_order_and_all6155_exposures(arm):
    rows, head = rank_history(arm)
    assert s.verify_history(rows, head, '20260903', arm) == head['sampling_order_sha256']
    assert len(rows) == 1540 and rows[-1]['samples_seen'] == 6155


@pytest.mark.parametrize('field,value', [('drive_pair_rank_loss', -.1), ('drive_pair_rank_loss', .1),
    ('drive_pair_weight', True), ('drive_pair_weight', 1), ('optimization_loss', 0.),
    ('candidate_score_loss', float('nan')), ('gradient_norm', True), ('global_step', True)])
def test_history_math_cannot_be_hidden_by_resealing_report(field, value):
    rows, head = rank_history(); rows[0][field] = value; head['history'] = deepcopy(rows)
    with pytest.raises(ValueError): s.verify_history(rows, head, '20260903', 'ce_continue')


def test_scalar_operation_order_matches_actual_cpu_float32_multiply_then_add():
    import torch
    for ce, rank in ((1.1234567,.4321098), (1e20,1e-20), (1e-30,2e-30), (90.,19.)):
        a, b = torch.tensor(ce), torch.tensor(rank)
        expected = float(.1*a + .1*b)
        actual = s.f32(s.f32(s.f32(.1)*float(a)) + s.f32(s.f32(.1)*float(b)))
        assert actual == expected


def test_native_matrix_digest_retains_signed_zero_and_entire_candidate_axis():
    import torch
    from portable_e2e.frozen_selector import _digest_tensors
    rows = [fixtures.row(i) for i in range(3)]
    actual = s.sha(s.matrix_bytes('projected_costs', rows, 'candidate_composite_costs'))
    tensor = torch.tensor([r['loss_alignment']['candidate_composite_costs'] for r in rows])
    assert actual == _digest_tensors([('projected_costs', tensor)])
    rows[0]['loss_alignment']['candidate_composite_costs'][6] = -0.
    assert s.sha(s.matrix_bytes('projected_costs', rows, 'candidate_composite_costs')) != actual


@pytest.mark.parametrize('bad', [[0.]*11, [False]*12, [float('nan')]*12, [.1]*12])
def test_matrix_byte_proof_rejects_non_native_or_incomplete_values(bad):
    row = fixtures.row(); row['loss_alignment']['candidate_logits'] = bad
    with pytest.raises(ValueError): s.matrix_bytes('logits', [row], 'candidate_logits')


def pair_row(costs, logits=None, selected=0, group='continuing_motion'):
    row = fixtures.row(selected=selected); row['target_motion_group'] = group
    row['loss_alignment']['candidate_composite_costs'] = costs
    row['loss_alignment']['candidate_logits'] = logits if logits is not None else [0.]*12
    return row


def test_all_exact_ties_zero_auxiliary_keep_family_tie_and_hard_first_index():
    value = s.drive_diagnostics([pair_row([0.]*12, selected=6)])['all_samples']
    assert value['all_drive_pair_count'] == value['cost_tied_pair_count'] == 15
    assert value['strict_pair_count'] == 0 and value['final_saved_normalized_pair_loss_float64_mean'] == 0.
    assert value['selected_drive_regret_mean'] is None and value['selected_stop_count'] == 1
    assert value['hard_oracle_family_to_selected_family']['DRIVE']['STOP'] == 1
    assert value['selected_family_outside_exact_minimum_set_count'] == 0 and value['cross_family_exact_minimum_tie_count'] == 1


def test_fifteen_pair_sum_weights_and_stable_softplus_match_independent_core_cpu_double():
    import torch
    from portable_e2e.frozen_stopmix_drive_rank import drive_pair_rank_loss
    costs = [4.,0.,3.,1.,5.,2.,10.,11.,12.,13.,14.,15.]
    logits = [1000.,-1000.,.3,-.2,9.,4.,9999.,-9999.,0.,0.,0.,0.]
    value = s.paired_row_measurement(pair_row(costs, logits))
    expected = float(drive_pair_rank_loss(torch.tensor([logits],dtype=torch.float64), torch.tensor([costs],dtype=torch.float64)))
    assert value['normalized_pair_loss_float64'] == pytest.approx(expected, abs=1e-12)
    assert value['strict_pair_count'] == 15 and math.isfinite(expected)


def test_pair_diagnostics_separate_family_errors_and_within_drive_denominators():
    rows = [pair_row([0.,1.,2.,3.,4.,5.]+[10.]*6, selected=1),
        pair_row([0.,1.,2.,3.,4.,5.]+[10.]*6, selected=6),
        pair_row([2.,3.,4.,5.,6.,7.]+[0.]*6, selected=0),
        pair_row([2.,3.,4.,5.,6.,7.]+[0.]*6, selected=6, group='stationary_hold')]
    r = s.drive_diagnostics(rows); a = r['all_samples']
    assert a['sample_count'] == 4 and a['selected_drive_count'] == a['selected_stop_count'] == 2
    assert a['hard_oracle_drive_and_selected_drive_count'] == 1
    assert a['within_drive_error_when_hard_oracle_drive_and_selected_drive_count'] == 1
    assert a['selected_drive_error_count'] == 1 and a['selected_drive_regret_mean'] == .5
    assert a['selected_family_outside_exact_minimum_set_count'] == 2
    assert a['hard_oracle_family_to_selected_family'] == {'DRIVE':{'DRIVE':1,'STOP':1},'STOP':{'DRIVE':1,'STOP':1}}
    assert r['target_motion_groups']['moving_to_stop']['sample_count'] == 0
    assert r['target_motion_groups']['stationary_hold']['sample_count'] == 1


def test_cost_near_ties_not_rounded_or_removed_and_stop_logits_no_direct_auxiliary():
    row = pair_row([0.,1e-30,0.,0.,0.,0.]+[9.]*6)
    a = s.paired_row_measurement(row); assert a['strict_pair_count'] == 5 and a['cost_tied_pair_count'] == 10
    row['loss_alignment']['candidate_logits'][6:] = [1e100]*6
    assert s.paired_row_measurement(row)['normalized_pair_loss_float64'] == a['normalized_pair_loss_float64']


@pytest.mark.parametrize('value', [float('nan'), True, float('inf')])
def test_bad_saved_pair_costs_rejected(value):
    row = fixtures.row(); row['loss_alignment']['candidate_composite_costs'][0] = value
    with pytest.raises(ValueError): s.paired_row_measurement(row)


def head_fixture(arm='ce_continue'):
    head, _, seed, plan, state, analyses = rf.head_fixture('candidate_residual')
    history, record = rank_history(arm); parent = deepcopy(head); parent['final_head_sha256'] = 'c'*64
    parent['post_training_metrics'] = deepcopy(head['baseline_metrics'])
    state['parent_head_proof'] = {'head_state_sha256':parent['final_head_sha256']}
    proof = dict(original_loss_sha256=s.base.EXTERNAL['external_core.py'])
    plan['loss_projection'] = proof
    head.update(artifact_id='portable_e2e.frozen_stopmix_drive_rank_research_head.v1', arm=arm, optimizer_resumed=False,
        initialization='Every arm copies the same fixed parent B residual head; fresh Adam state, not optimizer-state resume or fresh random head.',
        initial_head_sha256=parent['final_head_sha256'], parent_head_sha256_before=parent['final_head_sha256'], parent_head_sha256_after=parent['final_head_sha256'],
        parent_logits_sha256_before=s.sha(b''.join(s.matrix_bytes(split, analyses[split][1], 'candidate_logits') for split in s.base.COUNTS)),
        projection_proof=proof, projection_proof_sha256=s.sha(s.encoded(proof).rstrip(b'\n')),
        projected_cost_sha256={split:s.sha(s.matrix_bytes('projected_costs', v[1], 'candidate_composite_costs')) for split,v in analyses.items()},
        parent_logit_parity={}, history=deepcopy(history), sampling_order_sha256=record['sampling_order_sha256'])
    head['parent_logits_sha256_after'] = head['parent_logits_sha256_before']
    for split, value in analyses.items():
        pin = s.residual.logits_digest(value[1]); head['parent_logit_parity'][split] = dict(byte_equal=True,
            parent_logits_sha256=pin, initial_logits_sha256=pin, max_abs_logit_difference=0., selected_index_mismatch_count=0)
    weight = s.ARMS.index(arm)
    head['selector_objective'] = dict(drive_pair_weight=weight, auxiliary_coefficient=.1*weight,
        drive_candidate_indices=list(range(6)), unordered_pairs_per_sample=15,
        formula='0.1*hard_composite_CE + 0.1*lambda*mean_rows(sum_pairs(abs(delta_cost)/sum15_abs_gaps * softplus(sign(delta_cost)*delta_logit)))',
        exact_ties='zero weight; all-tied DRIVE row contributes zero; no epsilon, temperature, group filter or sample deletion')
    return head,history,seed,plan,state,parent,analyses


@pytest.mark.parametrize('arm', s.ARMS)
def test_all_parent_logits_state_sampling_and_core_head_proofs(arm):
    h,rows,seed,plan,state,parent,analyses = head_fixture(arm)
    assert s.verify_head(h,rows,seed,arm,plan,state,parent,analyses,analyses) == h['sampling_order_sha256']


@pytest.mark.parametrize('mutation', ['initial', 'optimizer', 'parent_logits', 'before', 'geometry_count', 'direct_stop_pair',
    'projection', 'cost_bytes', 'parent_baseline', 'training', 'core_oracle', 'capacity', 'paired_weight'])
def test_rank_head_cannot_masquerade_as_other_objective_or_parent(mutation):
    h,rows,seed,plan,state,parent,analyses = head_fixture()
    if mutation == 'initial': h['initial_head_sha256'] = 'd'*64
    elif mutation == 'optimizer': h['optimizer_resumed'] = True
    elif mutation == 'parent_logits': h['parent_logits_sha256_before'] = 'd'*64
    elif mutation == 'before': h['pre_training_metrics']['val']['metrics']['loss'] = 1.
    elif mutation == 'geometry_count': h['selector_objective']['unordered_pairs_per_sample'] = 66
    elif mutation == 'direct_stop_pair': h['selector_objective']['drive_candidate_indices'] = list(range(12))
    elif mutation == 'projection': h['projection_proof_sha256'] = 'd'*64
    elif mutation == 'cost_bytes': h['projected_cost_sha256']['train'] = 'd'*64
    elif mutation == 'parent_baseline': parent['post_training_metrics']['val']['metrics']['loss'] = 1.
    elif mutation == 'training': h['generator_parameters_trained'] = True
    elif mutation == 'core_oracle': h['composite_oracle_sha256']['val'] = 'd'*64
    elif mutation == 'capacity': h['head_parameter_count'] = 115201
    else: h['selector_objective']['drive_pair_weight'] = 1
    with pytest.raises(ValueError): s.verify_head(h,rows,seed,'ce_continue',plan,state,parent,analyses,analyses)


def test_parent_secure_cpu_artifact_proof_is_not_claimed_local_tensor_loading():
    h,_,seed,_,_,parent,analyses = head_fixture()
    files = {f'seed_{seed}/candidate_residual/{n}':'b'*64 for n in ('head_only.pt','final_logits.pt')}
    proof = dict(files={n:'b'*64 for n in ('head_only.pt','final_logits.pt')}, head_state_sha256=parent['final_head_sha256'],
        logit_sha256={split:s.residual.logits_digest(v[1]) for split,v in analyses.items()}, weights_only=True,map_location='cpu',optimizer_loaded=False)
    s.verify_parent_proof(proof,parent,files,seed,analyses)
    for key,value in (('weights_only',False),('map_location','cuda:0'),('optimizer_loaded',True),('head_state_sha256','d'*64)):
        wrong=deepcopy(proof);wrong[key]=value
        with pytest.raises(ValueError):s.verify_parent_proof(wrong,parent,files,seed,analyses)


def test_partial_missing_report_keeps_all_six_outcomes_without_parent_read(tmp_path, monkeypatch):
    raw=b'{}\n';(tmp_path/'plan.json').write_bytes(raw);monkeypatch.setattr(s,'PLAN_SHA',s.sha(raw))
    monkeypatch.setattr(s.residual,'summarize_campaign',lambda *_:pytest.fail('partial cannot read parents'))
    result=s.summarize_campaign(tmp_path)
    assert result['status']=='INCOMPLETE_NOT_PROMOTED' and len(result['performed_heads'])==6
    assert all(v['status']=='NOT_RUN' for v in result['performed_heads'])
    assert result['training_data_approved'] is False and result['generator_neural_forwards']==0


def test_partial_started_head_is_not_lost(tmp_path):
    result=s.outcome_ledger({'seed_20260903/ce_continue/metrics.jsonl':'a'*64},{})
    assert result[0]['status']=='INCOMPLETE' and all(v['status']=='NOT_RUN' for v in result[1:])


def test_fresh_outputs_refuse_all_three_inputs_and_dataset_alias(tmp_path,monkeypatch):
    roots=[tmp_path/n for n in ('run','residual','original')]
    for root in roots:root.mkdir()
    for root in roots:
        with pytest.raises(ValueError):s.run(roots[0],root/'inside',roots[1],roots[2])
    repo=tmp_path/'repo';repo.mkdir();data=tmp_path/'data';data.mkdir();(repo/'datasets').symlink_to(data)
    monkeypatch.setattr(s,'ROOT',repo)
    with pytest.raises(ValueError):s.run(roots[0],data/'inside',roots[1],roots[2])


def test_partial_persistence_has_no_checksum_self_entry_or_overwrite(tmp_path,monkeypatch):
    root=tmp_path/'in';root.mkdir();raw=b'{}\n';(root/'plan.json').write_bytes(raw);monkeypatch.setattr(s,'PLAN_SHA',s.sha(raw))
    out=tmp_path/'out';result=s.run(root,out)
    assert result['status']=='INCOMPLETE_NOT_PROMOTED'
    assert s.base.manifest((out/'SHA256SUMS').read_bytes())=={'summary.json':s.digest(out/'summary.json')}
    with pytest.raises(ValueError):s.run(root,out)
