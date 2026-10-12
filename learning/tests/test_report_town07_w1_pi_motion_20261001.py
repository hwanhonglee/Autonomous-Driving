"""HH_261001 - Reject fake actuator effects, duplicated integration and rewritten PI motion evidence."""

from copy import deepcopy
from dataclasses import asdict
from pathlib import Path

import pytest

from portable_e2e.longitudinal_pi_research_20261001 import LongitudinalPIConfig, LongitudinalPIResearch
from scripts.e2e import report_town07_w1_pi_motion_20261001 as report


def fixture(arm="B_pi_only", stop=False):
    """HH_261001 - Keep a genuine terminal ACK without inventing a subsequent snapshot."""
    states, receipts, commands = [], [], []
    pi = LongitudinalPIResearch()
    previous = None
    for i in range(4):
        frame, sequence = 100 + i, 2 + i
        state = dict(frame=frame, timestamp=1. + .05 * i, vx=1., vy=.1, ax=.1 * i, ay=0.,
                     world_velocity_carla=(1. + .05 * i, 0., 0.),
                     control_transport=dict(expected_receipt_sequence=sequence - 1,
                                            next_command_receipt_sequence=sequence))
        states.append(state)
        if i % 2 == 0:
            desired = 0. if stop else 1.2
            original = dict(throttle=0. if stop else .14, brake=1. if stop else 0., steer=.03,
                            desired_speed_mps=desired, reverse=False, hand_brake=False)
            proposed = dict(original)
            if arm == "B_pi_only":
                request, numerical = pi.update(desired, state["vx"])
                if request is not None: proposed.update(request)
            else:
                numerical = dict(integral_before=0., integral_after=0.)
            audit = dict(arm=arm, gains=asdict(LongitudinalPIConfig()), actuator_acknowledged=False,
                camera_policy_timestep_s=.1, held_control_native_timestep_s=.05,
                camera_policy_timestamp_ns=round(state["timestamp"] * 1e9),
                original_normal_tracker_control=original, proposed_normal_tracker_control=proposed,
                committed_integral_after=pi.integral_error, **numerical)
            for key in ("feedforward_added", "throttle_floor_added", "selected_speed_or_path_changed",
                        "steering_changed", "stop_branch_changed", "emergency_authority_changed", "control_cadence_changed"):
                audit[key] = False
            command = dict(control_source="learned_trajectory_tracker", raw_output={"truth": "unaltered"}, selected={"index": 0},
                control=proposed, longitudinal_pi_research=audit, total_seconds=.01,
                application=dict(learned_control_applied=True, acknowledged=True, vehicle_control_approved=False),
                emergency_reason=None, command_frame=frame, inference_frame=frame,
                held_between_camera_frames=False, ack_receipt_sequence=sequence)
            previous = command
        else:
            command = deepcopy(previous)
            command.update(command_frame=frame, held_between_camera_frames=True, ack_receipt_sequence=sequence)
        chosen = {key: command["control"][key] for key in ("throttle", "brake", "steer", "reverse", "hand_brake")}
        chosen.update(gear=0, manual_gear_shift=False)
        state["next_control"] = chosen
        if i:
            # HH_261001 - Automatic transmission gear is exempt under the unchanged native ACK contract.
            state["current_control"] = dict(receipts[-1]["requested_control"], gear=1)
        else:
            state["current_control"] = dict(chosen, throttle=0., brake=1., gear=1)
        receipts.append(dict(sequence=sequence, before_frame=frame, after_ack_frame=frame,
            server_accepted=True, status="ACKNOWLEDGED", do_tick=False,
            physical_actuation_proven=False, requested_control=chosen))
        commands.append(command)
    return states, receipts, commands


@pytest.mark.parametrize("arm", ["A_original", "B_pi_only"])
def test_real_ack_effect_is_next_tick_and_terminal_ack_is_retained(arm):
    values = fixture(arm)
    result = report.command_audit(*values, arm)
    assert result["summary"]["learned_acknowledged_commands"] == 4
    assert result["summary"]["ack_matched_observed_effect_states"] == 3
    assert result["summary"]["terminal_ack_without_following_snapshot_sequences"] == [5]
    assert result["summary"]["real_camera_inferences"] == 2
    assert result["summary"]["held_native_commands"] == 2
    assert result["effect_states"][0]["frame"] == 101


@pytest.mark.parametrize("kind", ["ack", "effect_sequence", "next_control", "measured_throttle",
    "held_control", "held_integrator", "integral_rewrite", "protected_steering", "fake_tick", "fake_approval"])
def test_command_or_integrator_evidence_rewrites_fail_closed(kind):
    states, receipts, commands = fixture()
    if kind == "ack": receipts[0]["server_accepted"] = False
    if kind == "effect_sequence": states[1]["control_transport"]["expected_receipt_sequence"] = 5
    if kind == "next_control": states[0]["next_control"] = dict(states[0]["next_control"], throttle=.8)
    if kind == "measured_throttle": states[1]["current_control"]["throttle"] = .4
    if kind == "held_control": commands[1]["control"]["desired_speed_mps"] = 1.3
    if kind == "held_integrator": commands[1]["longitudinal_pi_research"]["integral_after"] = .04
    if kind == "integral_rewrite": commands[0]["longitudinal_pi_research"]["integral_after"] = .9
    if kind == "protected_steering": commands[0]["longitudinal_pi_research"]["steering_changed"] = True
    if kind == "fake_tick": receipts[0]["do_tick"] = True
    if kind == "fake_approval": commands[0]["application"]["vehicle_control_approved"] = True
    with pytest.raises(ValueError): report.command_audit(states, receipts, commands, "B_pi_only")


def test_stop_replay_does_not_claim_unexercised_emergency():
    result = report.command_audit(*fixture(stop=True), "B_pi_only")["summary"]
    assert result["observed_model_stop_targets"] == 2
    assert result["original_model_stop_branch_exercised"] is True
    assert result["original_emergency_branch_exercised"] is False


def test_stationary_observation_cells_are_not_hidden_by_zero_timestamp_span():
    rows = [dict(frame=i, timestamp=i * .05, world_velocity_carla=(0., 0., 0.)) for i in range(20)]
    result = report.stationary_runs(rows, .05)
    assert result["runs"][0]["timestamp_span_s"] == pytest.approx(.95)
    assert result["maximum_observation_cell_upper_bound_s"] == 1.
    assert result["continuous_time_stationary_proven"] is False


def test_motion_reports_native_and_fd_reference_definitions_separately():
    result = report.motion_diagnostics(fixture()[0])
    assert result[0]["native_rotated_actor_ax_mps2"] == pytest.approx(.1)
    assert result[0]["finite_difference_base_vx_mps2"] == 0.
    assert result[0]["finite_difference_actor_planar_speed_mps2"] == pytest.approx(1.)
    assert result[0]["native_ax_sample_difference_jerk_mps3"] == pytest.approx(2.)


def test_motion_gap_does_not_get_interpolated_to_good_result():
    states = fixture()[0]
    states[2]["frame"] += 1
    with pytest.raises(ValueError, match="contiguous"): report.motion_diagnostics(states)


def test_percentile_is_explicit_linear_interpolation():
    assert report.percentile([0., 1., 2., 3.], .95) == pytest.approx(2.85)


def test_cli_is_create_only_before_reading_or_writing_inputs(tmp_path):
    output = tmp_path / "already_present"
    output.mkdir()
    with pytest.raises(ValueError, match="create-only"):
        report.main(["--runs", str(tmp_path / "absent"), "--body-audits", str(tmp_path / "absent"),
                     "--output", str(output)])
    assert list(output.iterdir()) == []
