"""HH_261001 - Fixed raw selection, prefix metrics and no fabricated W2 actuator receipt."""

from dataclasses import asdict
import pytest
from scripts.e2e import replay_w2_fixed149_counterfactual_20261001 as study


def raw():
    return dict(candidate_xy=[[[0.,0.] for _ in range(64)] for _ in range(12)],
        candidate_speed=[[0.]*64 for _ in range(12)],candidate_logits=[0.]*12)


def test_fixed3_20_64_geometry_prefixes():
    speed=[1.]*64;xy=[[(i+1)*.1,0.] for i in range(64)]
    for points in (3,20,64):
        result=study.prefix_metrics(xy,speed,points)
        assert result['speed_mean_mps']==1.
        assert result['path_length_m']==pytest.approx(points*.1)
        assert result['endpoint_xy_m']==xy[points-1]
    with pytest.raises(ValueError):study.prefix_metrics(xy,speed,5)


def test_raw_argmax_tie_and_no_acknowledged_counterfactual():
    result,selected=study.gate_and_control(raw(),0.)
    assert result['raw_argmax']==0 and selected.candidate_index==0
    assert result['selected_trajectory_gate_pass'] is True
    assert result['tracker_proposal']['brake']==.3
    assert result['counterfactual_control_acknowledged'] is False


def test_invalid_raw_winner_not_reranked_to_valid_candidate():
    value=raw();value['candidate_logits'][2]=10.;value['candidate_speed'][2]=[10.]*64
    result,selected=study.gate_and_control(value,0.)
    assert result['raw_argmax']==2 and selected is None
    assert result['candidate_gates'][0]['passed'] is True
    assert result['selected_trajectory_gate_pass'] is False
    assert result['tracker_proposal']['brake']==1.


def test_actual_W1_control_must_match_not_just_raw_hash():
    result,selected=study.gate_and_control(raw(),0.)
    decision=dict(candidate_gates=result['candidate_gates'],selected=asdict(selected),
        control_source='learned_trajectory_tracker',control=dict(result['tracker_proposal']))
    assert study.verify_w1_actual(result,selected,decision) is True
    decision['control']['brake']=.9
    with pytest.raises(ValueError):study.verify_w1_actual(result,selected,decision)


def test_same_controller_and_immutable_preregistration():
    assert study.CONTROLLER['speed_cap_mps']==2.5 and study.CONTROLLER['speed_preview_s']==.3
    assert study.CONTROLLER['throttle_gain']==study.CONTROLLER['brake_gain']==.7
    assert study.replay.digest(study.PLAN)==study.PLAN_SHA
    assert study.replay.digest(study.BINDING)==study.BINDING_SHA
    assert study.replay.digest(study.RECEIPT)==study.RECEIPT_SHA
