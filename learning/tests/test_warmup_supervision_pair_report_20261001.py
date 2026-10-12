"""HH_261001 - Candidate transitions use native inference states and retain every mode change."""

import pytest
from scripts.e2e.report_warmup_supervision_pair_20261001 import transitions


def command(frame, candidate):
    return dict(inference_frame=frame, raw_output={}, selected=dict(candidate_index=candidate,
        speed_mps=[0., 0., 0.]), control=dict(throttle=0., brake=.1),
        application=dict(learned_control_applied=True))


def test_deduplicates_held_commands_and_uses_inference_state():
    states = [dict(frame=10, timestamp=1., vx=.7), dict(frame=12, timestamp=1.1, vx=2.)]
    rows = [command(10, 5), command(10, 5), command(12, 1), command(12, 1)]
    result = transitions(states, rows)
    assert result['unique_inferences'] == 2
    change = result['candidate_transitions'][1]
    assert change['from_candidate'] == 5 and change['to_candidate'] == 1
    assert change['observed_vx_mps'] == 2.
    assert change['seconds_since_first_inference'] == pytest.approx(.1)


def test_missing_state_is_not_silently_interpolated():
    with pytest.raises(ValueError):
        transitions([], [command(10, 5)])


def test_empty_does_not_claim_transition_or_success():
    result = transitions([], [])
    assert result['unique_inferences'] == 0 and result['candidate_transitions'] == []
