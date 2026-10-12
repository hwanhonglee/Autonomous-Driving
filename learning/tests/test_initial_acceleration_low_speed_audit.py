"""HH_261001 - Keep velocity chatter and candidate availability distinct from driving success."""

import pytest
from scripts.e2e.audit_initial_acceleration_low_speed_20261001 import analyze


def fixture():
    states = [dict(frame=i, timestamp=i * .05, vx=v,
                   control_transport=dict(status='PASS', expected_receipt_sequence=i))
              for i, v in enumerate((.6, .04, -.06, .04))]
    raw = [[0.] * 64 for _ in range(12)]
    raw[5][2] = .8
    commands = [dict(ack_receipt_sequence=i, application=dict(learned_control_applied=True),
                     inference_frame=i, raw_output=dict(candidate_speed=raw),
                     candidate_gates=[dict(index=j, passed=True) for j in range(12)],
                     selected=dict(candidate_index=3, speed_mps=[0.] * 64)) for i in range(4)]
    return states, commands


def test_chatter_is_low_speed_even_when_it_interrupts_tight_zero_tail():
    result = analyze(*fixture())
    assert result['trailing_time_abs_vx_at_most_0p5_s'] == pytest.approx(.15)
    assert result['reverse_time_vx_below_minus_0p05_s'] == .05
    assert result['low_speed_frames_selecting_preview_at_most_0p05_despite_other_drive_above_0p5'] == 3
    assert result['controls_changed'] is False


def test_non_acknowledged_commands_are_not_attributed():
    states, commands = fixture()
    for row in commands:
        row['application']['learned_control_applied'] = False
    with pytest.raises(ValueError, match='no acknowledged'):
        analyze(states, commands)


def test_missing_native_timestep_rejected():
    states, commands = fixture()
    states[-1]['timestamp'] += .05
    with pytest.raises(ValueError, match='20 Hz'):
        analyze(states, commands)
