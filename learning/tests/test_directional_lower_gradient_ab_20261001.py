"""HH_261001 - Pin one training-only intervention and preserve native-control failure evidence."""

import inspect
import json
from types import SimpleNamespace
import pytest
import torch
from scripts.e2e import run_ah_directional_lower_gradient_ab_20261001 as study


def test_exact_native_update_helper_and_counts():
    assert study.optimize_batch is study.reference.optimize_batch
    assert study.COUNTS['global_step']==2999 and study.COUNTS['samples_seen']==11996
    assert study.COUNTS['active_supervision_exposures']==11549
    assert study.COUNTS['skipped_schedule_steps_one_based']==[1693]
    assert study.supervision.ACTIVE_WEIGHT==11996/11549
    assert study.model_class(study.ARMS[0]) is study.base.PerspectiveTrajectoryModel
    assert study.model_class(study.ARMS[1]) is study.directional.DirectionalLowerSpeedGradientModel
    with pytest.raises(Exception):study.model_class('other')


def test_only_four_predeclared_B_trace_batches():
    assert study.TRACE_STEPS==(1,1000,2000,3000)
    assert all(study.trace_for(study.ARMS[0],step) is None for step in range(1,3001))
    actual=[step for step in range(1,3001) if study.trace_for(study.ARMS[1],step) is not None]
    assert actual==list(study.TRACE_STEPS)
    with pytest.raises(Exception):study.trace_for(study.ARMS[1],3001)


def test_same_constructor_rng_and_parameter_abi():
    torch.set_num_threads(2)
    from portable_e2e.model import ModelConfig,PHYSICAL_STOPMIX_MODEL_ID
    cfg=ModelConfig(model_id=PHYSICAL_STOPMIX_MODEL_ID,candidate_count=12,maximum_step_m=1.)
    torch.manual_seed(20260906)
    native=study.model_class(study.ARMS[0])(cfg)
    rng_a=torch.get_rng_state().clone()
    torch.manual_seed(20260906)
    candidate=study.model_class(study.ARMS[1])(cfg)
    assert torch.equal(rng_a,torch.get_rng_state())
    assert study.base.tensor_digest(native.state_dict())==study.base.tensor_digest(candidate.state_dict())


def test_exact_A_barrier_saves_receipt_before_failure(tmp_path):
    folder=tmp_path/study.ARMS[0];folder.mkdir()
    receipt=dict(final_model_state_sha256='0'*64,checkpoint_sha256='a'*64,fit=dict(study.COUNTS))
    with pytest.raises(Exception,match='native control differs'):
        study.native_identity_barrier(study.ARMS[0],receipt,tmp_path)
    evidence=json.loads((folder/'native_identity.json').read_text())
    assert evidence['status']=='FAIL_NATIVE_W1_REPRODUCTION'
    assert evidence['actual_model_state_sha256']=='0'*64
    assert evidence['checkpoint_sha256']=='a'*64


def test_A_barrier_exact_and_B_never_selected_by_model_hash(tmp_path):
    (tmp_path/study.ARMS[0]).mkdir()
    receipt=dict(final_model_state_sha256=study.reference.EXPECTED_W1_STATE,
        checkpoint_sha256='a'*64,fit=dict(study.COUNTS))
    assert study.native_identity_barrier(study.ARMS[0],receipt,tmp_path)['status']=='PASS_EXACT_W1_STATE'
    receipt['final_model_state_sha256']='0'*64
    assert study.native_identity_barrier(study.ARMS[1],receipt,tmp_path) is None


def test_budget_is_fixed_and_not_autoextended(monkeypatch):
    monkeypatch.setattr(study.time,'monotonic',lambda:1800.)
    study.check_budget(0.)
    monkeypatch.setattr(study.time,'monotonic',lambda:1800.001)
    with pytest.raises(Exception,match='budget exhausted'):study.check_budget(0.)


def test_skip_precedes_optimizer_helper_and_both_finals_precede_DEV():
    source=inspect.getsource(study.fit)
    assert source.index('if step in supervision.SKIP_ONE_BASED')<source.index('= optimize_batch(')
    assert source.index('base.save_new(checkpoint, payload)')<source.index('native_identity_barrier(')
    assert source.index('base._seed_everything(')<source.index('model = model_class(')<source.index('torch.optim.AdamW(')
    assert 'probe(' not in source and 'load_state_dict(' in source
    main=inspect.getsource(study.main)
    assert main.index("both_final_checkpoints_complete.json")<main.index("dev_view = plan['views']['Town03_DEV']")
    assert 'base.PerspectiveTrajectoryModel(cfg)' in main


def test_evaluation_schema_binding_restores_on_failure():
    original=study.previous.SCHEMA,study.previous.STEPS
    with pytest.raises(RuntimeError):
        with study.evaluation_scope():
            assert study.previous.SCHEMA==study.SCHEMA and study.previous.STEPS==2999
            raise RuntimeError('injected')
    assert (study.previous.SCHEMA,study.previous.STEPS)==original


def test_fixed_source34_closure_preserves_all_original_W1_sources():
    from pathlib import Path
    root=Path(__file__).resolve().parents[2]
    additions=study.verify_new_sources(root)
    reference=json.loads((root/'storage/experiments/training/2026-10-01/ah_warmup_supervision_ab_20261001_v1/declaration.json').read_text())
    old=reference['source_sha256']
    assert all(study.base.digest(root/name)==sha for name,sha in old.items())
    assert len(dict(old,**additions))==34


def test_contract_keeps_forward_runtime_and_labels_unchanged():
    value=study.contract()
    for field in ('forward_decoder_changed','runtime_decoder_changed','parameter_abi_changed',
                  'current_speed_clamp_changed','upper_clamp_gradient_changed','STOP_branch_changed',
                  'loss_changed','new_near_preview_loss','acceleration_auxiliary','labels_changed',
                  'training_mask_changed','trace_A','trace_other_batches','simulator_launched'):
        assert value[field] is False
    assert value['both_fixed_checkpoints_before_DEV'] is True
