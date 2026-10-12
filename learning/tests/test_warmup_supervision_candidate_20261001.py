"""HH_261001 - Reject mixed endpoint provenance and restore process-only bindings."""

from types import SimpleNamespace
import pytest
import torch
from portable_e2e import simulation_research as research
from scripts.e2e import research_warmup_supervision_candidate_20261001 as candidate


def payload(arm):
    return dict(arm=arm, checkpoint_schema=candidate.SCHEMA,
        parent_checkpoint_sha256=candidate.PARENT_SHA256, training_plan_sha256='a' * 64,
        fit=dict(global_step=2999, scheduled_steps=3000, samples_seen=11996,
                 scheduled_samples=12000, skipped_schedule_steps_one_based=[1693],
                 active_supervision_exposures=11996 if arm == candidate.ARMS[0] else 11549))


@pytest.mark.parametrize('arm', candidate.ARMS)
def test_exact_endpoint_metadata(arm):
    value = payload(arm)
    candidate.validate_payload(value, arm, 'a' * 64)
    value['fit']['global_step'] = 3000
    with pytest.raises(ValueError):
        candidate.validate_payload(value, arm, 'a' * 64)


def test_plan_and_one_based_skip_are_pinned():
    value = payload(candidate.ARMS[0])
    with pytest.raises(ValueError):
        candidate.validate_payload(value, candidate.ARMS[0], 'b' * 64)
    value['fit']['skipped_schedule_steps_one_based'] = [1692]
    with pytest.raises(ValueError):
        candidate.validate_payload(value, candidate.ARMS[0], 'a' * 64)


def test_identity_restored_on_exception():
    before = research.AH_CHECKPOINT_SHA256, research.CHECKPOINT_SCHEMA
    collector = SimpleNamespace(AH_CHECKPOINT_SHA256=before[0])
    with pytest.raises(RuntimeError):
        with candidate.identity('1' * 64, collector):
            assert collector.AH_CHECKPOINT_SHA256 == research.AH_CHECKPOINT_SHA256 == '1' * 64
            assert research.CHECKPOINT_SCHEMA == candidate.SCHEMA
            raise RuntimeError('expected')
    assert (research.AH_CHECKPOINT_SHA256, research.CHECKPOINT_SCHEMA) == before
    assert collector.AH_CHECKPOINT_SHA256 == before[0]


def test_actual_tensors_not_just_declared_hashes():
    from scripts.e2e.run_moving_multiroute_recovery_k import tensor_digest
    parent = dict(model_state_dict={'weight': torch.ones(2)})
    trained = dict(model_state_dict={'weight': torch.zeros(2)},
                   initial_model_state_sha256=tensor_digest(parent['model_state_dict']))
    trained['final_model_state_sha256'] = tensor_digest(trained['model_state_dict'])
    candidate.validate_tensor_provenance(trained, parent)
    trained['model_state_dict']['weight'][0] = 3.
    with pytest.raises(ValueError):
        candidate.validate_tensor_provenance(trained, parent)


@pytest.mark.parametrize('sha', ['', 'z' * 64, candidate.PARENT_SHA256])
def test_original_or_invalid_identity_rejected(sha):
    with pytest.raises(ValueError):
        with candidate.identity(sha):
            pass
