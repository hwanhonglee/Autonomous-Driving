"""HH_261001 - Test process-local selector provenance and fail-closed candidate identity."""

from types import SimpleNamespace
import pytest
import torch
from portable_e2e import simulation_research as research
from scripts.e2e import research_selector_phase_candidate_20261001 as candidate


def payload():
    return dict(arm=candidate.ARMS[0], checkpoint_schema=candidate.SCHEMA,
                parent_checkpoint_sha256=candidate.PARENT_SHA256, fit=dict(global_step=3000))


def test_exact_new_schema_and_arm_required():
    candidate.validate_payload(payload(), candidate.ARMS[0])
    with pytest.raises(ValueError):
        candidate.validate_payload(payload(), candidate.ARMS[1])
    wrong = payload()
    wrong['fit']['global_step'] = 2999
    with pytest.raises(ValueError):
        candidate.validate_payload(wrong, candidate.ARMS[0])


def test_restores_process_identity_even_after_failure():
    before = research.AH_CHECKPOINT_SHA256, research.CHECKPOINT_SCHEMA
    collector = SimpleNamespace(AH_CHECKPOINT_SHA256=before[0])
    with pytest.raises(RuntimeError):
        with candidate.identity('1' * 64, collector):
            assert collector.AH_CHECKPOINT_SHA256 == research.AH_CHECKPOINT_SHA256 == '1' * 64
            assert research.CHECKPOINT_SCHEMA == candidate.SCHEMA
            raise RuntimeError('test')
    assert (research.AH_CHECKPOINT_SHA256, research.CHECKPOINT_SCHEMA) == before
    assert collector.AH_CHECKPOINT_SHA256 == before[0]


@pytest.mark.parametrize('sha', ['', 'z' * 64, candidate.PARENT_SHA256])
def test_original_or_invalid_hash_cannot_masquerade_as_candidate(sha):
    with pytest.raises(ValueError):
        with candidate.identity(sha):
            pass


def test_export_checks_actual_frozen_tensor_bytes():
    from scripts.e2e.run_moving_multiroute_recovery_k import tensor_digest
    parent = dict(model_state_dict={'backbone.weight': torch.ones(2), 'candidate_head.weight': torch.zeros(2)})
    result = dict(model_state_dict={'backbone.weight': torch.ones(2), 'candidate_head.weight': torch.ones(2)},
                  fit=dict(samples_seen=12000))
    result['final_model_state_sha256'] = tensor_digest(result['model_state_dict'])
    result['frozen_generator_state_sha256'] = tensor_digest({'backbone.weight': torch.ones(2)})
    candidate.assert_frozen_generator(result, parent)
    result['model_state_dict']['backbone.weight'][0] = 2.
    result['final_model_state_sha256'] = tensor_digest(result['model_state_dict'])
    with pytest.raises(ValueError, match='frozen AH'):
        candidate.assert_frozen_generator(result, parent)
