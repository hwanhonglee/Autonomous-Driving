"""HH_261001 - Test honest full-capture publication and previous-ACK failure-tail visualization."""

import copy
import math
from pathlib import Path

import pytest

from scripts.e2e import publish_temporal_v2_actual_failure_20261001 as publisher


def receipt(sequence, frame, **kwargs):
    return dict(sequence=sequence, before_frame=frame, status="ACKNOWLEDGED", server_accepted=True, **kwargs)


def command(sequence, source, frame, *, learned=False, **kwargs):
    return dict(ack_receipt_sequence=sequence, command_frame=frame, inference_frame=frame,
        control_source=source, application=dict(acknowledged=True, applied_source="expert" if source == "expert_startup" else source,
        learned_control_applied=learned), selected=dict(xy_base_m=[[1., 0.]]*64), **kwargs)


def test_emergency_ack_clears_older_learned_plan_instead_of_searching_only_learned_commands():
    receipts = [receipt(1, 10), receipt(2, 12)]
    commands = [command(1, "learned_trajectory_tracker", 10, learned=True), command(2, "emergency_brake", 12)]
    state = dict(frame=13)
    ack, row, plan, label = publisher.observed_prior_ack(state, receipts, commands, {10: dict(x=0., y=0., yaw=0.)})
    assert ack["sequence"] == 2 and row["control_source"] == "emergency_brake"
    assert plan == [] and "EMERGENCY BRAKE" in label and "startup" not in label


def test_same_frame_emergency_proposal_not_shown_as_already_observed_actuator_effect():
    receipts = [receipt(1, 10), receipt(2, 12)]
    commands = [command(1, "learned_trajectory_tracker", 10, learned=True), command(2, "emergency_brake", 12)]
    ack, row, plan, label = publisher.observed_prior_ack(dict(frame=12), receipts, commands,
                                                       {10: dict(x=5., y=6., yaw=math.pi/2)})
    assert ack["sequence"] == 1 and len(plan) == 64
    assert plan[0] == pytest.approx((5., 7.)) and "learned" in label


def test_latest_prior_expert_ack_clears_learned_plan_and_labels_startup_explicitly():
    commands = [command(1, "learned_trajectory_tracker", 10, learned=True), command(2, "expert_startup", 12)]
    ack, row, plan, label = publisher.observed_prior_ack(dict(frame=13), [receipt(1, 10), receipt(2, 12)], commands, {})
    assert ack["sequence"] == 2 and plan == [] and label == "expert startup only / not learned driving"


def test_warmup_without_model_command_is_not_expert_or_learned_driving():
    ack, row, plan, label = publisher.observed_prior_ack(dict(frame=13), [receipt(1, 12, reason="next_stationary_warmup_control")], [], {})
    assert row is None and plan == [] and "warmup" in label


def test_held_learned_plan_uses_original_inference_pose_not_latest_vehicle_pose():
    row = command(1, "learned_trajectory_tracker", 12, learned=True, held_between_camera_frames=True)
    row["inference_frame"] = 10
    ack, original, plan, label = publisher.observed_prior_ack(dict(frame=13, x=999., y=999.), [receipt(1, 12)], [row],
        {10: dict(x=2., y=3., yaw=0.), 12: dict(x=777., y=777., yaw=0.)})
    assert plan[0] == (3., 3.) and "held" in label


@pytest.mark.parametrize("mutation", ["unacknowledged", "future_inference", "mismatched_frame", "unknown_source", "short_plan", "emergency_claims_learned"])
def test_uncausal_or_mislabelled_proposal_cannot_be_displayed_as_applied(mutation):
    row = command(1, "learned_trajectory_tracker", 10, learned=True)
    if mutation == "unacknowledged": row["application"]["acknowledged"] = False
    if mutation == "future_inference": row["inference_frame"] = 11
    if mutation == "mismatched_frame": row["command_frame"] = 9
    if mutation == "unknown_source": row["application"]["applied_source"] = "unclassified"
    if mutation == "short_plan": row["selected"]["xy_base_m"] = [[1., 0.]]
    if mutation == "emergency_claims_learned": row["application"]["applied_source"] = "emergency_brake"
    with pytest.raises(ValueError):
        publisher.observed_prior_ack(dict(frame=12), [receipt(1, 10)], [row], {10: dict(x=0., y=0., yaw=0.)})


def test_no_previous_ack_fails_closed_and_future_ack_is_not_used():
    with pytest.raises(ValueError, match="earlier accepted"):
        publisher.observed_prior_ack(dict(frame=10), [receipt(1, 10)], [], {})


def test_duplicate_command_ack_association_is_rejected():
    row = command(1, "emergency_brake", 10)
    with pytest.raises(ValueError, match="ambiguous"):
        publisher.observed_prior_ack(dict(frame=12), [receipt(1, 10)], [row, copy.deepcopy(row)], {})


def test_exact40_fixed_frames_cover_failed_tail_and_twenty_seconds_without_outcome_selection():
    indices = publisher.displayed_camera_indices()
    assert indices == list(range(0, 200, 5)) and len(indices) == 40
    assert indices[0] == 0 and indices[-1] == 195
    assert sum(index*2 > 82 for index in indices) == 31
    assert len(indices)*500 == 20_000
    assert publisher.PNG_INDICES["03_rejection_observation_prior_learned_ACK.png"] == 41
    assert publisher.PNG_INDICES["04_first_saved_camera_after_emergency_ACK.png"] == 42


@pytest.mark.parametrize("count", [199, 201, True, 40])
def test_short_or_subselected_camera_denominator_rejected(count):
    with pytest.raises(ValueError): publisher.displayed_camera_indices(count)


@pytest.mark.parametrize("name", ["../raw.jpg", "/tmp/raw.jpg", "images/../../raw.jpg", "other/frame.jpg"])
def test_original_camera_path_cannot_escape_episode(tmp_path, name):
    with pytest.raises(ValueError): publisher.safe_image(tmp_path, name)


def test_source_digest_rejects_symlink_and_does_not_write_inputs(tmp_path):
    original = tmp_path / "raw"; original.write_bytes(b"original failure")
    link = tmp_path / "alias"; link.symlink_to(original)
    with pytest.raises(ValueError, match="regular original"): publisher.digest(link)
    pins = {original: publisher.digest(original)}
    publisher.assert_pins(pins)
    assert original.read_bytes() == b"original failure"
    original.write_bytes(b"fixture corruption")
    with pytest.raises(ValueError, match="artifact changed"): publisher.assert_pins(pins)


def test_publication_is_new_path_and_old_center_renderer_has_fixed_pin():
    assert publisher.OUTPUT.name == "temporal_v2_actual_failure_results_v1"
    path = publisher.ROOT / "tools/e2e/publish_town07_w1_pi_gallery_20261001.py"
    assert publisher.digest(path) == publisher.FIXED_SHA256[path]
    assert not publisher.OUTPUT.is_relative_to(publisher.RUN)


def test_actual_frozen_summary_failure_flags_and_report_wording_remain_readable():
    # HH_261001 - Read tiny existing result receipts only; never execute rendering, raw data scans or a simulator.
    body = publisher.read_json(publisher.BODY / "summary.json")
    diagnostic = publisher.read_json(publisher.RUN / "diagnostic_report.json")
    owner = publisher.read_json(publisher.RUN / "owner_result.json")
    manifest = publisher.read_json(publisher.RUN / "episode.partial/manifest.json")
    publisher.verify_outcomes(body, diagnostic, owner, manifest)
    text = publisher.korean_readme(dict(body=body, diagnostic=diagnostic))
    assert "0/30" in text and "4.681 m" in text and "2.50556175 m/s" in text
    assert "101.065 ms" in text and "318개" in text
    assert "최초 비상제동 원인은 시간 초과가 아니라" in text
    assert "Autoware/RViz 실시간 화면을 촬영한 것이라고 주장하지 않습니다" in text


@pytest.mark.parametrize("mutation", ["driving_pass", "tail_cropped", "body_denial", "manifest_success", "deadline_relabel"])
def test_failed_capture_cannot_be_relabelled_success_or_hide_tail(mutation):
    body = publisher.read_json(publisher.BODY / "summary.json")
    diagnostic = publisher.read_json(publisher.RUN / "diagnostic_report.json")
    owner = publisher.read_json(publisher.RUN / "owner_result.json")
    manifest = publisher.read_json(publisher.RUN / "episode.partial/manifest.json")
    if mutation == "driving_pass": diagnostic["learned_driving_screen_passed"] = True
    if mutation == "tail_cropped": body["original_failed_tail_cropped"] = True
    if mutation == "body_denial": body["vehicle_control_approved"] = True
    if mutation == "manifest_success": manifest["status"] = "complete"
    if mutation == "deadline_relabel": body["first_latched_failure"]["original_rejection_was_policy_deadline"] = True
    with pytest.raises(ValueError): publisher.verify_outcomes(body, diagnostic, owner, manifest)
