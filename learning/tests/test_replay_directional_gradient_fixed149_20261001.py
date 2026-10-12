"""HH_261001 - Prepare released-endpoint replay without opening active fit artifacts."""

from copy import deepcopy
import pytest
from scripts.e2e import replay_directional_gradient_fixed149_20261001 as replay


def test_identity_scope_restores_actual_runtime_globals():
    original=replay.research.AH_CHECKPOINT_SHA256,replay.research.CHECKPOINT_SCHEMA
    with pytest.raises(RuntimeError):
        with replay.endpoint_identity('b'*64):
            assert replay.research.AH_CHECKPOINT_SHA256=='b'*64
            assert replay.research.CHECKPOINT_SCHEMA==replay.audit.SCHEMA
            raise RuntimeError('injected')
    assert (replay.research.AH_CHECKPOINT_SHA256,replay.research.CHECKPOINT_SCHEMA)==original


def test_exact_helpers_pinned_and_not_replaced():
    assert replay.prior.CONTROLLER['speed_cap_mps']==2.5
    assert replay.prior.CONTROLLER['speed_preview_s']==.3
    assert all(replay.audit.sha(path)==sha for path,sha in replay.PINS.items())


def test_descriptive_selection_change_does_not_claim_ACK_or_repaired_old_candidate():
    raw=dict(candidate_xy=[[[0.,0.]for _ in range(64)]for _ in range(12)],
        candidate_speed=[[0.]*64 for _ in range(12)],candidate_logits=[0.]*12)
    a,_=replay.prior.gate_and_control(raw,0.)
    b=deepcopy(a);b['raw_argmax']=5;b['raw_controller_preview_speed_mps']=.4
    b['all_candidate_preview_speeds_mps'][5]=.4;b['tracker_proposal']['throttle']=.28;b['tracker_proposal']['brake']=0.
    result=replay.summarize([dict(A=a,B=b,historical_W1_vx_mps=0.)])
    assert result['resolved_zero_previews']==dict(count=1,raw_argmax_changed=1,old_A_candidate_index_still_exact_zero_under_B=1)
    assert result['actual_B_control_ACK_count']==result['new_simulator_states']==0


def test_no_unpinned_endpoint_identity():
    with pytest.raises(ValueError):
        with replay.endpoint_identity('*'):pass
