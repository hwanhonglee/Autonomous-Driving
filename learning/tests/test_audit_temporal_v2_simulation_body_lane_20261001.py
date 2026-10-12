"""HH_261001 - Test the explicit V2 failed-source adapter without simulator, GPU or approval bypass."""

from copy import deepcopy
import json
from pathlib import Path

import pytest

from scripts.e2e import audit_temporal_v2_simulation_body_lane_20261001 as audit


def source_state(frame, vx):
    return dict(frame=frame, timestamp=frame*.05, vx=vx, vy=.001,
                world_velocity_carla=[vx, .001, 0.])


def command(sequence, frame, rejected=False, held=False):
    return dict(ack_receipt_sequence=sequence, command_frame=frame, inference_frame=frame,
        control_source="emergency_brake" if rejected else "learned_trajectory_tracker",
        emergency_reason="actual current speed exceeds unchanged low-speed envelope; no clipping" if rejected else None,
        held_between_camera_frames=held,
        learned_control_proposed=not rejected,
        application=dict(learned_control_applied=not rejected),
        raw_output=dict(curvature_knots=[0.]) if not held else None,
        decoded_output=None if rejected else dict(candidate_xy=[[[.1, 0.]]]),
        selected=None if rejected else dict(candidate_index=0),
        total_seconds=.085, raw_nn_seconds=.059,
        control=dict(throttle=0. if rejected else .14, brake=1. if rejected else 0., steer=0.))


def fixture():
    return [command(1, 10), command(2, 11, True), command(3, 12, True, True)], \
           [source_state(10, 2.49), source_state(11, 2.506), source_state(12, .2)]


def test_actual_speed_first_failure_is_not_a_deadline_or_legacy_candidate_rewrite():
    commands, states = fixture()
    report = audit.first_failure_evidence(commands, states)
    assert report["first_rejection_ACK_sequence"] == 2
    assert report["actual_unclipped_base_planar_speed_mps"] > 2.5
    assert report["original_policy_seconds"] == .085
    assert report["original_rejection_was_policy_deadline"] is False
    assert report["original_rejected_raw_output"] == commands[1]["raw_output"]
    assert report["whole_latched_command_tail_count"] == 2
    assert report["failed_tail_cropped"] is False
    assert report["previous_accepted_fresh_command"] == commands[0]


@pytest.mark.parametrize("fault", ["resume", "throttle", "brake", "reason", "speed", "deadline", "raw", "selected"])
def test_wrong_failure_or_latch_is_never_hidden(fault):
    commands, states = deepcopy(fixture())
    if fault == "resume": commands[-1]["application"]["learned_control_applied"] = True
    elif fault == "throttle": commands[-1]["control"]["throttle"] = .2
    elif fault == "brake": commands[-1]["control"]["brake"] = .5
    elif fault == "reason": commands[1]["emergency_reason"] = "deadline"
    elif fault == "speed": states[1]["vx"] = 2.4
    elif fault == "deadline": commands[1]["total_seconds"] = .101
    elif fault == "raw": commands[1]["raw_output"] = None
    elif fault == "selected": commands[1]["selected"] = dict(candidate_index=0)
    with pytest.raises(ValueError): audit.first_failure_evidence(commands, states)


def test_output_cannot_escape_fixed_run_or_overwrite(monkeypatch, tmp_path):
    monkeypatch.setattr(audit, "RUN", tmp_path/"actual")
    monkeypatch.setattr(audit, "bind_archive", lambda: pytest.fail("invalid destination must not read source"))
    with pytest.raises(ValueError): audit.audit(tmp_path/"outside")
    inside=tmp_path/"actual/audit_v1/existing";inside.mkdir(parents=True)
    with pytest.raises(ValueError): audit.audit(inside)


def test_digest_rejects_symlink_even_when_it_targets_regular_file(tmp_path):
    actual=tmp_path/"regular";actual.write_bytes(b"actual archived source")
    link=tmp_path/"alias";link.symlink_to(actual)
    with pytest.raises(ValueError): audit.digest(link)


def test_frozen_body_and_ack_pins_are_exact():
    assert all(audit.digest(audit.ROOT/name) == value for name, value in audit.FROZEN.items())
    assert len(audit.FROZEN) == 3


def test_missing_annotation_is_unrecorded_and_strict_json_encodable():
    result=audit.state_annotation_counts([{}, dict(research_control_source=None),
                                        dict(research_control_source="emergency_brake")])
    assert result == dict(unrecorded=2, emergency_brake=1)
    assert json.loads(json.dumps(result,sort_keys=True,allow_nan=False)) == result
